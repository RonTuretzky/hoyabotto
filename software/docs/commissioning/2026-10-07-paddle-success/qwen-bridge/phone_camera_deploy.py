"""Narrow phone-only deployment; importing this module has no side effects.

Call deploy_phone_camera(ROOT, dry_run=False) from the phone-only dispatch before
any motor/API/camera setup code. The default backend requires psutil and lsof.
No config, environment of another process, token, certificate or PID file is read.
Only the explicit source whitelist is copied. See Runtime for the fake/backend
contract. An optional healthcheck receives the exact candidate Process and must
return True (never credential-bearing output); listener ownership is also checked.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile
import time
from typing import Callable, Mapping, Protocol, Sequence


SOURCE = Path(__file__).resolve().parents[3] / "session-archive-2026-10-05/phone_camera"
BASE_FILES = ("server.py", "camera.html")
# Confirmed against the timing worker's local files/report: timing is inline in
# server.py; no separate runtime module exists. Add names only after source review.
ALLOWED_MODULES: frozenset[str] = frozenset()
DEFAULT_MODULES: tuple[str, ...] = ()
SAFE_FLAGS = frozenset({"-u", "-B", "-E", "-s", "-O", "-OO"})
PYTHON_NAME = re.compile(r"python(?:\d+(?:\.\d+)*)?", re.IGNORECASE)


class DeployError(Exception):
    """Only fixed, credential-free messages belong in this exception."""


@dataclass(frozen=True)
class Process:
    pid: int
    created: float
    argv: tuple[str, ...]
    cwd: str
    stdout: str
    stderr: str


class Runtime(Protocol):
    """All methods must raise on incomplete inspection, never silently omit it.

    processes returns Python processes with the exact absolute script argument.
    stop must recheck PID/create-time/argv immediately before SIGTERM and wait;
    never signal a process group or escalate to SIGKILL. start reuses the supplied
    argv/cwd/logs, and must either return its child identity or clean up its own
    child before raising. preflight compiles bytes and checks aiohttp.web/PIL.Image
    in that same interpreter without importing server.py or writing bytecode.
    listeners returns only TCP LISTEN addresses owned by that exact process.
    """

    def processes(self, server: Path) -> Sequence[Process]: ...
    def preflight(self, process: Process, sources: Mapping[str, bytes]) -> None: ...
    def stop(self, process: Process, timeout: float) -> None: ...
    def start(self, process: Process) -> Process: ...
    def listeners(self, process: Process) -> frozenset[tuple[str, int]]: ...


def _script_command(argv: Sequence[str], server: Path) -> bool:
    if not argv or not PYTHON_NAME.fullmatch(Path(argv[0]).name):
        return False
    # Locate the script operand, never a substring or an argument of another
    # script. Unsupported flags still identify a candidate, then fail validation.
    index = 1
    while index < len(argv):
        value = argv[index]
        if value in ("-c", "-m") or value.startswith(("-c", "-m")):
            return False
        if value == "--":
            return index + 1 < len(argv) and argv[index + 1] == str(server)
        if value in ("-W", "-X", "--check-hash-based-pycs"):
            index += 2
        elif value.startswith("-"):
            index += 1
        else:
            return value == str(server)
    return False


def _validate_process(process: Process, server: Path, work: Path) -> None:
    argv = process.argv
    if not _script_command(argv, server) or not Path(argv[0]).is_absolute():
        raise DeployError("phone interpreter/script must use absolute paths")
    flags = argv[1:-1]
    if argv[-1] != str(server) or any(f not in SAFE_FLAGS for f in flags):
        raise DeployError("unsupported phone launch flags or script arguments")
    if process.pid <= 0 or process.created <= 0 or not Path(process.cwd).is_absolute():
        raise DeployError("incomplete phone process identity")
    if not Path(process.cwd).is_dir():
        raise DeployError("phone working directory is unavailable")
    for value in (process.stdout, process.stderr):
        path = Path(value)
        if (not path.is_absolute() or path.resolve() != path or not path.is_relative_to(work)
                or path.suffix != ".log" or not path.is_file() or path.stat().st_nlink != 1):
            raise DeployError("phone stdout/stderr must be existing regular work logs")


def _one(runtime: Runtime, server: Path) -> Process:
    rows = list(runtime.processes(server))
    if len(rows) != 1:
        raise DeployError("expected exactly one phone Python server process")
    return rows[0]


def _hash(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _dependencies(sources: Mapping[str, bytes]) -> list[str]:
    """Static imports only; never execute the service as a preflight check."""
    local = {Path(name).stem for name in sources if name.endswith(".py")}
    required = {"aiohttp.web", "PIL.Image"}
    for name, data in sources.items():
        if not name.endswith(".py"):
            continue
        for node in ast.walk(ast.parse(data, name)):
            if isinstance(node, ast.ImportFrom) and node.level:
                raise DeployError("relative imports are unsupported for the standalone phone service")
            names = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                     else [node.module] if isinstance(node, ast.ImportFrom) else [])
            for module in names:
                top = module.split(".")[0]
                if top in local:
                    continue
                if top not in sys.stdlib_module_names and top not in {"aiohttp", "PIL"}:
                    raise DeployError("phone source contains an unreviewed dependency")
                required.add(module)
    return sorted(required)


def _read_file(path: Path, *, optional: bool = False) -> tuple[bytes, int] | None:
    # lstat catches dangling symlinks too. No config or directory-wide copying.
    try:
        info = path.lstat()
    except FileNotFoundError:
        if optional:
            return None
        raise DeployError("required whitelisted source file is missing") from None
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise DeployError("whitelisted files must be regular, unlinked files")
    return path.read_bytes(), stat.S_IMODE(info.st_mode)


def _atomic_write(path: Path, data: bytes, mode: int) -> None:
    fd, temporary = tempfile.mkstemp(prefix=".phone-file-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
            os.fchmod(stream.fileno(), mode)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def _check_files(directory: Path, expected: Mapping[str, tuple[bytes, int] | None]) -> None:
    if any(_read_file(directory / name, optional=True) != value for name, value in expected.items()):
        raise DeployError("phone file content or mode changed unexpectedly")


def _ready(runtime: Runtime, process: Process, server: Path,
           listeners: frozenset[tuple[str, int]], timeout: float,
           healthcheck: Callable[[Process], bool] | None) -> None:
    deadline = time.monotonic() + timeout
    while True:
        if _one(runtime, server) != process:
            raise DeployError("phone process identity changed during readiness check")
        if listeners <= runtime.listeners(process) and (healthcheck is None or healthcheck(process) is True):
            # Healthcheck may have yielded; require the identity again afterwards.
            if _one(runtime, server) == process:
                return
            raise DeployError("phone process identity changed after healthcheck")
        if time.monotonic() >= deadline:
            raise DeployError("phone listener or healthcheck readiness timed out")
        time.sleep(min(0.1, timeout))


def deploy_phone_camera(root: Path | str, *, source_dir: Path | str = SOURCE,
                        modules: Sequence[str] = DEFAULT_MODULES, dry_run: bool = True,
                        runtime: Runtime | None = None,
                        healthcheck: Callable[[Process], bool] | None = None,
                        timeout: float = 10.0) -> dict:
    """Return a sanitized report; ok=False always means the deployment failed.

    States: dry_run, unchanged, deployed, aborted (no stop requested), old_running
    (original identity still present), rolled_back, rollback_failed. A successful
    rollback still returns ok=False. Errors expose a phase and fixed message or
    exception *type*, never argv, config, logs, subprocess output or str(exception).
    Backups persist at report['backup_dir']; only changed whitelisted bytes/modes
    and an absence/hash manifest are stored. Dry-run creates no backups or files,
    launches no server, and sends no signals. Preflight may launch a -B validator.
    """
    report = {"ok": False, "state": "aborted", "phase": "validate", "dry_run": dry_run}
    old = None
    backup = None
    stop_requested = False
    new = None
    touched: list[str] = []
    originals: dict[str, tuple[bytes, int] | None] = {}
    try:
        if (isinstance(modules, str) or any(not isinstance(n, str) or not re.fullmatch(r"[A-Za-z_]\w*\.py", n) for n in modules)
                or len(set(modules)) != len(modules) or not set(modules) <= ALLOWED_MODULES):
            raise DeployError("unsupported phone module; only explicit whitelist names are allowed")
        if not isinstance(dry_run, bool) or not 0 < timeout <= 120:
            raise DeployError("invalid deployment options")
        root, source = Path(root).absolute(), Path(source_dir).absolute()
        target = root / "work/phone_camera"
        work = root / "work"
        if target.resolve() != target or not target.is_dir() or source.resolve() != source:
            raise DeployError("phone source/target directories must exist without symlink traversal")
        names = BASE_FILES + tuple(modules)
        sources = {name: _read_file(source / name)[0] for name in names}
        for name, data in sources.items():
            if name.endswith(".py"):
                compile(data, name, "exec")
                tree = ast.parse(data, name)
                for node in ast.walk(tree):
                    imports = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                               else [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                    for module in imports:
                        dependency = module.split(".")[0] + ".py"
                        if (source / dependency).exists() and dependency not in sources:
                            raise DeployError("source imports a local module outside the selected whitelist")
        _dependencies(sources)
        originals = {name: _read_file(target / name, optional=True) for name in names}
        changed = [name for name in names if originals[name] is None or originals[name][0] != sources[name]]
        report.update(changed=changed, source_sha256={n: _hash(v) for n, v in sources.items()})
        runtime = runtime if runtime is not None else LocalRuntime()
        report["phase"] = "preflight"
        old = _one(runtime, target / "server.py")
        _validate_process(old, target / "server.py", work)
        report["original_pid"] = old.pid
        listeners = runtime.listeners(old)
        if not listeners:
            raise DeployError("original phone process has no owned TCP listener")
        runtime.preflight(old, sources)
        _ready(runtime, old, target / "server.py", listeners, timeout, healthcheck)
        if dry_run or not changed:
            _check_files(target, originals)
            report.update(ok=True, state="dry_run" if dry_run else "unchanged", phase="complete", pid=old.pid)
            return report

        report["phase"] = "backup"
        backup = Path(tempfile.mkdtemp(prefix=".phone-deploy-", dir=target))
        report["backup_dir"] = str(backup)
        manifest = {}
        for name in changed:
            value = originals[name]
            manifest[name] = None if value is None else {"sha256": _hash(value[0]), "mode": value[1]}
            if value is not None:
                _atomic_write(backup / name, value[0], value[1])
        _atomic_write(backup / "manifest.json", json.dumps(manifest, sort_keys=True).encode(), 0o600)
        _check_files(target, originals)
        if _one(runtime, target / "server.py") != old:
            raise DeployError("original phone process changed before stop")
        report["phase"] = "stop"
        stop_requested = True
        runtime.stop(old, timeout)
        if runtime.processes(target / "server.py"):
            raise DeployError("phone process remains after stop")
        _check_files(target, originals)
        report["phase"] = "install"
        for name in changed:
            touched.append(name)  # Include a write that fails after replacement.
            mode = originals[name][1] if originals[name] is not None else 0o644
            _atomic_write(target / name, sources[name], mode)
        installed = {n: (sources[n], originals[n][1] if originals[n] else 0o644) for n in names}
        _check_files(target, installed)
        report["phase"] = "start"
        new = runtime.start(old)
        _validate_process(new, target / "server.py", work)
        if (new.argv, new.cwd, new.stdout, new.stderr) != (old.argv, old.cwd, old.stdout, old.stderr):
            raise DeployError("replacement phone launch settings changed")
        if new == old:
            raise DeployError("replacement has the original process identity")
        report["phase"] = "readiness"
        _ready(runtime, new, target / "server.py", listeners, timeout, healthcheck)
        _check_files(target, installed)
        report.update(ok=True, state="deployed", phase="complete", pid=new.pid,
                      installed_sha256={n: _hash(v[0]) for n, v in installed.items()})
        return report
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = str(error) if isinstance(error, DeployError) else type(error).__name__
        if not stop_requested:
            return report

    # Recovery is deliberately narrow: never stop a process we did not start.
    try:
        rows = list(runtime.processes(target / "server.py"))
        if rows == [old] and not touched:
            _check_files(target, originals)
            report.update(state="old_running", pid=old.pid)
            return report
        if rows:
            if new is None or rows != [new]:
                raise DeployError("rollback blocked by an unexpected phone process")
            runtime.stop(new, timeout)
            if runtime.processes(target / "server.py"):
                raise DeployError("replacement phone process remains; rollback cannot overwrite files")
        for name in touched:
            value = originals[name]
            if value is None:
                (target / name).unlink(missing_ok=True)
            else:
                _atomic_write(target / name, value[0], value[1])
        _check_files(target, originals)
        restored = runtime.start(old)
        if (restored.argv, restored.cwd, restored.stdout, restored.stderr) != (old.argv, old.cwd, old.stdout, old.stderr):
            raise DeployError("rollback phone launch settings changed")
        _ready(runtime, restored, target / "server.py", listeners, timeout, healthcheck)
        _check_files(target, originals)
        report.update(state="rolled_back", pid=restored.pid,
                      restored_sha256={n: _hash(v[0]) if v else None for n, v in originals.items()})
    except (Exception, KeyboardInterrupt) as error:
        report.update(state="rollback_failed", rollback_error=str(error) if isinstance(error, DeployError) else type(error).__name__)
    return report


class LocalRuntime:
    """Opt-in local backend. Constructed only inside an explicit deployment call.

    psutil supplies exact argv/create-time/cwd and PID-owned listener information;
    lsof supplies stdout/stderr paths. Inspection errors abort. No process PID file
    is trusted. Server environment is inherited from the caller; custom environment
    launchers are outside this backend's contract. There is no shell invocation.
    """

    def __init__(self):
        import psutil
        self.psutil = psutil
        self.children = {}  # Keep Popen handles alive to reap our own children.

    def _logs(self, pid: int) -> tuple[str, str]:
        result = subprocess.run(["lsof", "-a", "-p", str(pid), "-d", "1,2", "-Ffn"],
                                capture_output=True, text=True, check=True, timeout=5)
        records = {}
        fd = None
        for line in result.stdout.splitlines():
            if line.startswith("f"):
                fd = line[1:]
            elif line.startswith("n") and fd in ("1", "2"):
                if fd in records:
                    raise DeployError("ambiguous phone log descriptor")
                records[fd] = line[1:]
        if set(records) != {"1", "2"}:
            raise DeployError("cannot establish existing phone stdout/stderr logs")
        return records["1"], records["2"]

    def _snapshot(self, proc) -> Process:
        stdout, stderr = self._logs(proc.pid)
        return Process(proc.pid, proc.create_time(), tuple(proc.cmdline()), proc.cwd(), stdout, stderr)

    def processes(self, server: Path) -> list[Process]:
        result = []
        for proc in self.psutil.process_iter():
            try:
                if PYTHON_NAME.fullmatch(proc.name()):
                    argv = proc.cmdline()
                    if _script_command(argv, server):
                        result.append(self._snapshot(proc))
            except self.psutil.NoSuchProcess:
                continue
        return result

    def _same(self, process: Process):
        proc = self.psutil.Process(process.pid)
        if self._snapshot(proc) != process:
            raise DeployError("phone PID or launch identity changed")
        return proc

    def listeners(self, process: Process) -> frozenset[tuple[str, int]]:
        proc = self._same(process)
        rows = proc.net_connections(kind="tcp")
        return frozenset((row.laddr.ip, row.laddr.port) for row in rows if row.status == self.psutil.CONN_LISTEN)

    def preflight(self, process: Process, sources: Mapping[str, bytes]) -> None:
        # Source never executes. Imports are fixed; server/config is not imported.
        code = ("import importlib,json,sys; "
                "data=json.load(sys.stdin); "
                "sys.path[0]=data['script_directory']; "
                "[compile(bytes.fromhex(v),k,'exec') for k,v in data['sources'].items() if k.endswith('.py')]; "
                "[importlib.import_module(m) for m in data['imports']]")
        payload = {"sources": {n: b.hex() for n, b in sources.items()}, "imports": _dependencies(sources),
                   "script_directory": str(Path(process.argv[-1]).parent)}
        result = subprocess.run([*process.argv[:-1], "-B", "-c", code],
                                input=json.dumps(payload),
                                text=True, capture_output=True, cwd=process.cwd, timeout=30)
        if result.returncode:
            raise DeployError("phone interpreter compile/dependency preflight failed")

    def stop(self, process: Process, timeout: float) -> None:
        proc = self._same(process)
        proc.terminate()  # psutil also checks creation time before signaling.
        proc.wait(timeout=timeout)
        child = self.children.pop(process.pid, None)
        if child is not None:
            child.wait(timeout=timeout)

    def start(self, process: Process) -> Process:
        server = Path(process.argv[-1])
        _validate_process(process, server, server.parent.parent)
        with open(process.stdout, "ab") as out, open(process.stderr, "ab") as err:
            child = subprocess.Popen(process.argv, cwd=process.cwd, stdin=subprocess.DEVNULL,
                                     stdout=out, stderr=err, start_new_session=True)
        self.children[child.pid] = child
        try:
            # exec may not have occurred when Popen returns on every platform.
            deadline = time.monotonic() + 2
            while True:
                snapshot = self._snapshot(self.psutil.Process(child.pid))
                if snapshot.argv == process.argv:
                    return snapshot
                if child.poll() is not None or time.monotonic() >= deadline:
                    raise DeployError("cannot verify new phone process identity")
                time.sleep(0.01)
        except (Exception, KeyboardInterrupt):
            # A Popen handle refers only to our own child, never an arbitrary PID.
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=5)
            self.children.pop(child.pid, None)
            raise
