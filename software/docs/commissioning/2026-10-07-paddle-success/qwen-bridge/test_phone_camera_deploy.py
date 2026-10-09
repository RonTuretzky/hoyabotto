"""Offline unit tests only: temporary trees, fake processes, no network or signals."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import phone_camera_deploy as deploy


class FakeRuntime:
    def __init__(self, process):
        self.rows = [process]
        self.events = []
        self.next_pid = 200
        self.ports = frozenset({("127.0.0.1", 8876), ("127.0.0.1", 8877)})
        self.preflight_error = None
        self.stop_error = None
        self.fail_starts = set()
        self.unready_pids = set()
        self.on_preflight = None
        self.on_stop = None

    def processes(self, server):
        return [p for p in self.rows if deploy._script_command(p.argv, server)]

    def preflight(self, process, sources):
        self.events.append(("preflight", process.pid))
        if self.preflight_error:
            raise self.preflight_error
        if self.on_preflight:
            self.on_preflight()

    def stop(self, process, timeout):
        self.events.append(("stop", process.pid))
        if self.stop_error:
            raise self.stop_error
        if process not in self.rows:
            raise deploy.DeployError("fake process identity changed")
        self.rows.remove(process)
        if self.on_stop:
            self.on_stop()

    def start(self, process):
        pid = self.next_pid
        self.next_pid += 1
        self.events.append(("start", pid))
        if pid in self.fail_starts:
            raise RuntimeError("synthetic startup failure; secret=DO_NOT_REPORT")
        result = replace(process, pid=pid, created=float(pid))
        self.rows.append(result)
        return result

    def listeners(self, process):
        if process not in self.rows:
            raise deploy.DeployError("fake process identity changed")
        return frozenset() if process.pid in self.unready_pids else self.ports


class DeployTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.root = self.base / "robot root with spaces"
        self.target = self.root / "work/phone_camera"
        self.target.mkdir(parents=True)
        self.source = self.base / "source with spaces"
        self.source.mkdir()
        self.sources = {"server.py": b"import frame_timing\nVALUE = 'new'\n",
                        "camera.html": b"<html>new camera</html>\n",
                        "frame_timing.py": b"import math\nVALUE = 1\n"}
        self.originals = {"server.py": b"VALUE = 'old'\n", "camera.html": b"<html>old camera</html>\n"}
        for name, data in self.sources.items():
            (self.source / name).write_bytes(data)
        for name, data in self.originals.items():
            (self.target / name).write_bytes(data)
            (self.target / name).chmod(0o640)
        self.protected = {"config.json": b'{"viewer_token":"FAKE_TEST_TOKEN"}',
                          "server.key": b"FAKE TLS key", "server.pem": b"FAKE cert",
                          "setup.py": b"raise RuntimeError('never execute setup')\n",
                          "setup.html": b"setup", "viewer.html": b"viewer",
                          "latest.jpg": b"frame", "server.pid": b"NOT_TRUSTED"}
        for name, data in self.protected.items():
            (self.target / name).write_bytes(data)
        self.log = self.root / "work/phone.log"
        self.log.write_bytes(b"fake existing log\n")
        self.process = deploy.Process(100, 100.0,
                                      ("/fake/venv with spaces/bin/python3.12", "-u", "-B", str(self.target / "server.py")),
                                      str(self.root), str(self.log), str(self.log))
        self.runtime = FakeRuntime(self.process)
        # Exercise optional-module support with a synthetic reviewed whitelist.
        # The actual archived implementation keeps timing inline in server.py.
        whitelist = patch.object(deploy, "ALLOWED_MODULES", frozenset({"frame_timing.py"}))
        whitelist.start()
        self.addCleanup(whitelist.stop)
        # Any accidental real operation is a hard test failure. The adapter's
        # subprocess commands are tested separately with explicit mocks.
        for owner, name in ((subprocess, "run"), (subprocess, "Popen"),
                            (deploy.os, "kill"), (deploy.os, "killpg"),
                            (socket, "socket"), (socket, "create_connection")):
            guard = patch.object(owner, name, side_effect=AssertionError("live operation forbidden"))
            guard.start()
            self.addCleanup(guard.stop)

    def run_deploy(self, **overrides):
        options = dict(source_dir=self.source, modules=("frame_timing.py",),
                       runtime=self.runtime, dry_run=False, timeout=0.001)
        options.update(overrides)
        return deploy.deploy_phone_camera(self.root, **options)

    def assert_protected(self):
        for name, data in self.protected.items():
            self.assertEqual((self.target / name).read_bytes(), data, name)
        self.assertEqual(self.log.read_bytes(), b"fake existing log\n")

    def assert_originals(self):
        for name, data in self.originals.items():
            self.assertEqual((self.target / name).read_bytes(), data)
            self.assertEqual((self.target / name).stat().st_mode & 0o777, 0o640)
        self.assertFalse((self.target / "frame_timing.py").exists())
        self.assert_protected()

    def assert_no_stop(self, result):
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["state"], "aborted", result)
        self.assertFalse(any(e[0] in ("stop", "start") for e in self.runtime.events))
        self.assert_originals()

    def test_deploy_only_whitelist_backups_hashes_same_launch_and_listeners(self):
        unrelated = replace(self.process, pid=80, argv=("/usr/bin/python3", str(self.root / "work/motor.py")))
        self.runtime.rows.append(unrelated)
        result = self.run_deploy()
        self.assertEqual((result["ok"], result["state"], result["pid"]), (True, "deployed", 200))
        self.assertEqual(self.runtime.events, [("preflight", 100), ("stop", 100), ("start", 200)])
        self.assertIn(unrelated, self.runtime.rows)
        for name, data in self.sources.items():
            self.assertEqual((self.target / name).read_bytes(), data)
            self.assertEqual(result["installed_sha256"][name], hashlib.sha256(data).hexdigest())
        backup = Path(result["backup_dir"])
        self.assertEqual((backup / "server.py").read_bytes(), self.originals["server.py"])
        self.assertIsNone(json.loads((backup / "manifest.json").read_text())["frame_timing.py"])
        self.assertEqual(self.runtime.rows[-1].argv, self.process.argv)
        self.assertEqual(self.runtime.rows[-1].cwd, self.process.cwd)
        self.assertEqual(self.runtime.rows[-1].stdout, self.process.stdout)
        self.assert_protected()

    def test_dry_run_no_mutation_no_signals(self):
        before = {p.relative_to(self.base): (p.read_bytes(), p.stat().st_mode) for p in self.base.rglob("*") if p.is_file()}
        with patch.object(deploy, "_atomic_write", side_effect=AssertionError("dry-run write")), \
                patch.object(deploy.tempfile, "mkdtemp", side_effect=AssertionError("dry-run mkdir")):
            result = self.run_deploy(dry_run=True)
        self.assertEqual(result["state"], "dry_run")
        self.assertTrue(result["ok"])
        self.assertEqual(self.runtime.events, [("preflight", 100)])
        after = {p.relative_to(self.base): (p.read_bytes(), p.stat().st_mode) for p in self.base.rglob("*") if p.is_file()}
        self.assertEqual(before, after)

    def test_unchanged_does_not_restart_or_backup(self):
        for name, data in self.sources.items():
            (self.target / name).write_bytes(data)
        result = self.run_deploy()
        self.assertEqual(result["state"], "unchanged")
        self.assertEqual(self.runtime.events, [("preflight", 100)])
        self.assertNotIn("backup_dir", result)

    def test_default_is_dry_run(self):
        result = deploy.deploy_phone_camera(self.root, source_dir=self.source,
                                           modules=("frame_timing.py",), runtime=self.runtime)
        self.assertEqual(result["state"], "dry_run")
        self.assertEqual(self.runtime.events, [("preflight", 100)])

    def test_bad_modules_fail_before_stop(self):
        for modules in (("../server.key",), (str(self.source / "frame_timing.py"),),
                        ("setup.py",), ("config.json",), ("x.py",),
                        ("frame_timing.py", "frame_timing.py"), "frame_timing.py"):
            with self.subTest(modules=modules):
                self.assert_no_stop(self.run_deploy(modules=modules))

    def test_missing_local_dependency_is_not_silently_omitted(self):
        self.assert_no_stop(self.run_deploy(modules=()))

    def test_missing_whitelisted_file(self):
        (self.source / "frame_timing.py").unlink()
        self.assert_no_stop(self.run_deploy())

    def test_unreviewed_dependency_fails_before_stop(self):
        (self.source / "frame_timing.py").write_text("import unreviewed_new_package\n")
        self.assert_no_stop(self.run_deploy())

    def test_invalid_python_fails_before_preflight_or_stop(self):
        (self.source / "frame_timing.py").write_text("if invalid syntax:\n")
        self.assert_no_stop(self.run_deploy())
        self.assertEqual(self.runtime.events, [])

    def test_sources_are_never_executed(self):
        (self.source / "server.py").write_text("raise RuntimeError('would read config or start listener')\n")
        self.assertTrue(self.run_deploy()["ok"])

    def test_protected_files_are_not_read(self):
        read = Path.read_bytes

        def guarded(path):
            if path.parent == self.target and path.name in self.protected:
                raise AssertionError("protected file read")
            return read(path)

        with patch.object(Path, "read_bytes", guarded):
            result = self.run_deploy()
        self.assertTrue(result["ok"], result)

    def test_dependency_preflight_failure_leaves_old_running_and_redacts(self):
        self.runtime.preflight_error = RuntimeError("FAKE_TEST_TOKEN DO_NOT_REPORT")
        result = self.run_deploy()
        self.assert_no_stop(result)
        self.assertNotIn("FAKE_TEST_TOKEN", json.dumps(result))
        self.assertNotIn("DO_NOT_REPORT", json.dumps(result))
        self.assertEqual(result["error"], "RuntimeError")

    def test_zero_or_multiple_processes_fail(self):
        for rows in ([], [self.process, replace(self.process, pid=101)]):
            self.runtime.rows = rows
            self.assert_no_stop(self.run_deploy())

    def test_pid_file_and_command_substrings_do_not_match(self):
        server = str(self.target / "server.py")
        decoys = [("/bin/sh", "-c", "/usr/bin/python3 " + server),
                  ("/usr/bin/python3", "-c", server),
                  ("/usr/bin/python3", "-m", "some_module", server),
                  ("/usr/bin/python3", server + ".old"),
                  ("/usr/bin/python3", "work/phone_camera/server.py")]
        self.runtime.rows = [replace(self.process, pid=300 + i, argv=argv) for i, argv in enumerate(decoys)]
        self.assert_no_stop(self.run_deploy())

    def test_other_python_script_argument_is_not_a_server_candidate(self):
        decoy = replace(self.process, pid=90,
                        argv=("/usr/bin/python3", "/fake/other_service.py", self.process.argv[-1]))
        self.runtime.rows.append(decoy)
        result = self.run_deploy()
        self.assertTrue(result["ok"], result)
        self.assertIn(decoy, self.runtime.rows)
        self.assertNotIn(("stop", 90), self.runtime.events)

    def test_unsafe_or_unknown_launch_flags_rejected(self):
        for flags in (("-X", "importtime"), ("--unknown",), ("-W", "ignore"), ("-I",), ("-S",)):
            self.runtime.rows = [replace(self.process, argv=(self.process.argv[0], *flags, self.process.argv[-1]))]
            self.assert_no_stop(self.run_deploy())

    def test_script_arguments_and_relative_interpreter_rejected(self):
        for argv in (self.process.argv + ("--token=DO_NOT_REPORT",), ("python3", self.process.argv[-1])):
            self.runtime.rows = [replace(self.process, argv=argv)]
            result = self.run_deploy()
            self.assert_no_stop(result)
            self.assertNotIn("DO_NOT_REPORT", json.dumps(result))

    def test_non_log_output_refused(self):
        for value in ("/dev/ttys001", str(self.target / "config.json"), str(self.base / "missing.log")):
            self.runtime.rows = [replace(self.process, stdout=value)]
            self.assert_no_stop(self.run_deploy())

    def test_symlink_and_hardlink_sources_refused(self):
        path = self.source / "camera.html"
        path.unlink()
        path.symlink_to(self.target / "config.json")
        self.assert_no_stop(self.run_deploy())
        path.unlink()
        deploy.os.link(self.target / "config.json", path)
        self.assert_no_stop(self.run_deploy())

    def test_symlink_target_refused(self):
        (self.target / "frame_timing.py").symlink_to(self.target / "config.json")
        result = self.run_deploy()
        self.assertFalse(result["ok"])
        self.assertEqual(self.runtime.events, [])
        self.assert_protected()

    def test_no_listener_refuses_stop(self):
        self.runtime.ports = frozenset()
        self.assert_no_stop(self.run_deploy())

    def test_preflight_health_failure_refuses_stop(self):
        self.assert_no_stop(self.run_deploy(healthcheck=lambda p: False))

    def test_identity_change_during_preflight_refuses_stop(self):
        self.runtime.on_preflight = lambda: setattr(self.runtime, "rows", [replace(self.process, created=101)])
        self.assert_no_stop(self.run_deploy())

    def test_backup_failure_refuses_stop(self):
        with patch.object(deploy, "_atomic_write", side_effect=OSError("DO_NOT_REPORT")):
            self.assert_no_stop(self.run_deploy())

    def test_stop_timeout_reports_original_running(self):
        self.runtime.stop_error = TimeoutError("DO_NOT_REPORT")
        result = self.run_deploy()
        self.assertFalse(result["ok"])
        self.assertEqual(result["state"], "old_running")
        self.assertEqual(self.runtime.rows, [self.process])
        self.assert_originals()

    def test_stop_error_after_exit_rolls_back(self):
        def fail_after_stop():
            raise RuntimeError("synthetic error after original exits")

        self.runtime.on_stop = fail_after_stop
        result = self.run_deploy()
        self.assertEqual(result["state"], "rolled_back")
        self.assert_originals()

    def test_keyboard_interrupt_after_stop_rolls_back(self):
        def interrupt_after_stop():
            raise KeyboardInterrupt()

        self.runtime.on_stop = interrupt_after_stop
        result = self.run_deploy()
        self.assertEqual(result["state"], "rolled_back")
        self.assertEqual(result["error"], "KeyboardInterrupt")
        self.assert_originals()

    def test_backup_exists_before_any_stop(self):
        def verify_backup():
            backups = list(self.target.glob(".phone-deploy-*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual((backups[0] / "server.py").read_bytes(), self.originals["server.py"])
            self.assertTrue((backups[0] / "manifest.json").is_file())

        self.runtime.on_stop = verify_backup
        self.assertTrue(self.run_deploy()["ok"])

    def test_start_failure_restores_files_modes_absence_and_restarts_old(self):
        self.runtime.fail_starts = {200}
        result = self.run_deploy()
        self.assertEqual((result["ok"], result["state"], result["pid"]), (False, "rolled_back", 201))
        self.assert_originals()
        self.assertNotIn("DO_NOT_REPORT", json.dumps(result))

    def test_listener_failure_stops_only_new_then_rolls_back(self):
        self.runtime.unready_pids = {200}
        result = self.run_deploy()
        self.assertEqual(result["state"], "rolled_back")
        self.assertEqual(self.runtime.events, [("preflight", 100), ("stop", 100), ("start", 200), ("stop", 200), ("start", 201)])
        self.assert_originals()

    def test_injected_healthcheck_failure_rolls_back(self):
        result = self.run_deploy(healthcheck=lambda p: p.pid != 200)
        self.assertEqual(result["state"], "rolled_back")
        self.assert_originals()

    def test_partial_install_failure_rolls_back(self):
        real_write = deploy._atomic_write
        failed = False

        def write(path, data, mode):
            nonlocal failed
            if path == self.target / "camera.html" and not failed:
                failed = True
                raise OSError("synthetic write error")
            return real_write(path, data, mode)

        with patch.object(deploy, "_atomic_write", side_effect=write):
            result = self.run_deploy()
        self.assertEqual(result["state"], "rolled_back")
        self.assert_originals()

    def test_hash_mismatch_after_start_rolls_back(self):
        def health(process):
            if process.pid == 200:
                (self.target / "camera.html").write_bytes(b"unexpected modification")
            return True

        result = self.run_deploy(healthcheck=health)
        self.assertEqual(result["state"], "rolled_back")
        self.assert_originals()

    def test_rollback_start_failure_is_truthful(self):
        self.runtime.fail_starts = {200, 201}
        result = self.run_deploy()
        self.assertEqual((result["ok"], result["state"]), (False, "rollback_failed"))
        self.assertEqual(self.runtime.rows, [])
        self.assert_originals()

    def test_rollback_health_failure_is_truthful(self):
        self.runtime.unready_pids = {200, 201}
        result = self.run_deploy()
        self.assertEqual((result["ok"], result["state"]), (False, "rollback_failed"))
        self.assert_originals()

    def test_unexpected_process_after_stop_is_never_killed(self):
        other = replace(self.process, pid=999, created=999)
        self.runtime.on_stop = lambda: self.runtime.rows.append(other)
        result = self.run_deploy()
        self.assertEqual(result["state"], "rollback_failed")
        self.assertEqual(self.runtime.rows, [other])
        self.assertEqual(self.runtime.events, [("preflight", 100), ("stop", 100)])
        self.assert_originals()

    def test_local_preflight_uses_same_interpreter_flags_cwd_no_server_import(self):
        local = object.__new__(deploy.LocalRuntime)
        with patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0)) as run:
            local.preflight(self.process, self.sources)
        args, kwargs = run.call_args
        self.assertEqual(args[0][:-2], [*self.process.argv[:-1], "-B"])
        self.assertEqual(args[0][-2], "-c")
        payload = json.loads(kwargs["input"])
        self.assertIn("aiohttp.web", payload["imports"])
        self.assertIn("PIL.Image", payload["imports"])
        self.assertIn("math", payload["imports"])
        self.assertNotIn("frame_timing", payload["imports"])
        self.assertNotIn("import server", args[0][-1])
        self.assertNotIn("config", args[0][-1])
        self.assertEqual(kwargs["cwd"], self.process.cwd)
        self.assertFalse(kwargs.get("shell", False))

    def test_local_stop_rechecks_identity_before_term(self):
        local = object.__new__(deploy.LocalRuntime)
        with patch.object(local, "_same", side_effect=deploy.DeployError("changed")) as same:
            with self.assertRaises(deploy.DeployError):
                local.stop(self.process, 0.001)
        same.assert_called_once_with(self.process)

    def test_local_stop_only_terminates_exact_verified_process(self):
        local = object.__new__(deploy.LocalRuntime)
        local.children = {}
        proc = Mock()
        with patch.object(local, "_same", return_value=proc):
            local.stop(self.process, 0.001)
        proc.terminate.assert_called_once_with()
        proc.wait.assert_called_once_with(timeout=0.001)
        proc.kill.assert_not_called()

    def test_local_start_reuses_interpreter_flags_cwd_and_existing_logs(self):
        local = object.__new__(deploy.LocalRuntime)
        local.psutil = Mock()
        local.children = {}
        child = Mock(pid=200)
        expected = replace(self.process, pid=200, created=200)
        with patch.object(subprocess, "Popen", return_value=child) as start, \
                patch.object(local, "_snapshot", return_value=expected):
            actual = local.start(self.process)
        self.assertEqual(actual, expected)
        args, kwargs = start.call_args
        self.assertEqual(args[0], self.process.argv)
        self.assertEqual(kwargs["cwd"], self.process.cwd)
        self.assertEqual(kwargs["stdout"].name, self.process.stdout)
        self.assertEqual(kwargs["stderr"].name, self.process.stderr)
        self.assertFalse(kwargs.get("shell", False))
        self.assert_protected()

    def test_lsof_output_is_only_used_for_exact_log_descriptors(self):
        local = object.__new__(deploy.LocalRuntime)
        output = "p100\nf1\nn" + str(self.log) + "\nf2\nn" + str(self.log) + "\n"
        with patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0, stdout=output)) as run:
            self.assertEqual(local._logs(100), (str(self.log), str(self.log)))
        self.assertEqual(run.call_args.args[0], ["lsof", "-a", "-p", "100", "-d", "1,2", "-Ffn"])

    def test_report_never_contains_credentials_argv_or_log_content(self):
        result = self.run_deploy()
        report = json.dumps(result)
        for secret in ("FAKE_TEST_TOKEN", "DO_NOT_REPORT", "fake existing log", "argv", "viewer_token"):
            self.assertNotIn(secret, report)

    def test_actual_archived_sources_need_no_extra_module(self):
        result = self.run_deploy(source_dir=deploy.SOURCE, modules=(), dry_run=True)
        self.assertEqual(result["state"], "dry_run", result)
        self.assertEqual(set(result["source_sha256"]), {"server.py", "camera.html"})


if __name__ == "__main__":
    unittest.main()
