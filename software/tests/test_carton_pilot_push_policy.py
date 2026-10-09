"""Source-only installer regression and real arc executors on fake transports.

Fixtures are frozen source excerpts, never imports of the live pilot. Only
selected functions/constants are compiled into a fake namespace; no robot,
model, camera, STOP/release, enable or process management is reachable.
"""
import ast
import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

SOFTWARE = Path(__file__).resolve().parents[1]
FIXTURES = Path(__file__).parent / "fixtures/carton_pilot_push"
spec = importlib.util.spec_from_file_location("push_installer", SOFTWARE / "tools/install_carton_pilot_push_policy.py")
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


@pytest.fixture
def sources():
    return {name: (FIXTURES / name).read_text() for name in installer.FILES}


def get_prompt(source):
    return installer.assignment(ast.parse(source), "SUPERVISOR_SYSTEM").value.value


def staged(tmp_path, sources):
    for name, content in sources.items():
        (tmp_path / name).write_text(content)
        (tmp_path / name).chmod(0o640)
    return tmp_path


def test_policy_replaces_conflicts_and_preserves_executable_structure(sources):
    patched = installer.patch_sources(sources)
    prompt = get_prompt(patched["chat_server.py"])
    for old in ("never push a flap", "a fold counts only", "fold the rest again", "is the PAYLOAD",
                "resend the same lift", "after a verified pinch", "verify by lifting", "lift 8 cm",
                "closing again is safe", "raise 5 cm", "with the gripper closed"):
        # Pinch instructions may legitimately keep the gripper closed, but the
        # old folded/mandatory-lift result strings must all disappear.
        assert old not in patched["chat_server.py"]
    for required in ("deliberate pushing", "no grasp verdict or mandatory lift test", "owner-reported support",
                     "not measured friction", "or an immovable fixture", "translation, rotation and tipping",
                     "wrist camera and arm links are not pushing tools", "load/current", "watchdog", "STOP",
                     "Never automatically retry", "historical fold_by_pinch", "contact can be offset"):
        assert required in prompt
    for name in installer.FILES:
        assert installer.text_only_ast(sources[name]) == installer.text_only_ast(patched[name])
    assert installer.patch_sources(patched) == patched


def test_unrelated_prompt_and_code_are_preserved(sources):
    sources["chat_server.py"] = sources["chat_server.py"].replace(
        "Cameras and what their directions mean:", "Custom metric carton guidance: preserve this.\n\nCameras and what their directions mean:")
    sources["chat_server.py"] += '\nEXTERNAL_OWNER_LIMIT = 123\n'
    patched = installer.patch_sources(sources)
    assert "Custom metric carton guidance: preserve this." in get_prompt(patched["chat_server.py"])
    assert patched["chat_server.py"].endswith("EXTERNAL_OWNER_LIMIT = 123\n")


def test_schema_changes_descriptions_only(sources):
    def schema(source):
        env = {}
        exec(compile(source, "<frozen-offline-schema>", "exec"), env)
        def without_descriptions(value):
            if isinstance(value, dict):
                return {k: without_descriptions(v) for k, v in value.items() if k != "description"}
            if isinstance(value, list):
                return [without_descriptions(v) for v in value]
            return value
        return without_descriptions(env["DECISION_SCHEMA"])
    assert schema(sources["model_backend.py"]) == schema(installer.patch_sources(sources)["model_backend.py"])


@pytest.mark.parametrize("change", ["missing", "duplicate", "nonliteral", "fold_drift", "lift_drift", "schema_drift", "syntax"])
def test_unknown_layout_refuses_before_writing(tmp_path, sources, change):
    if change == "missing":
        sources["chat_server.py"] = sources["chat_server.py"].replace("SUPERVISOR_SYSTEM=", "OTHER_PROMPT=", 1)
    elif change == "duplicate":
        sources["chat_server.py"] += '\nSUPERVISOR_SYSTEM="duplicate"\n'
    elif change == "nonliteral":
        sources["chat_server.py"] += '\n'
        node = installer.assignment(ast.parse(sources["chat_server.py"]), "SUPERVISOR_SYSTEM")
        sources["chat_server.py"] = installer.replace_node(sources["chat_server.py"], node.value, "get_prompt()")
    elif change == "fold_drift":
        sources["chat_server.py"] = sources["chat_server.py"].replace("Owner rule (9 October):", "Changed owner rule:")
    elif change == "lift_drift":
        sources["chat_server.py"] = sources["chat_server.py"].replace("raise the HOLDING arm 8 cm", "raise the HOLDING arm 7 cm")
    elif change == "schema_drift":
        sources["model_backend.py"] = sources["model_backend.py"].replace("fold: turn a pinched flap", "fold: changed semantics")
    else:
        sources["model_backend.py"] += '\nnot valid Python!'
    pilot = staged(tmp_path, sources)
    with pytest.raises((ValueError, SyntaxError)):
        installer.install(pilot)
    assert {n: (pilot / n).read_text() for n in installer.FILES} == sources
    assert not (pilot / installer.BACKUPS).exists()


def test_dry_run_idempotent_install_exact_rollback_and_modes(tmp_path, sources):
    pilot = staged(tmp_path, sources)
    original = {n: (pilot / n).read_bytes() for n in installer.FILES}
    preview = installer.install(pilot, dry_run=True)
    assert preview["changed"] and preview["dry_run"]
    assert not (pilot / installer.BACKUPS).exists()
    assert {n: (pilot / n).read_bytes() for n in installer.FILES} == original
    receipt = installer.install(pilot)
    assert receipt["changed"] and not receipt["runtime_activated"] and receipt["motor_writes"] == 0
    for name, row in receipt["files"].items():
        assert (pilot / row["backup"]).read_bytes() == original[name]
        assert (pilot / row["backup"]).stat().st_mode & 0o777 == 0o600
        assert (pilot / name).stat().st_mode & 0o777 == 0o640
        assert installer.sha((pilot / name).read_bytes()) == preview["files"][name]["after_sha256"]
    mtimes = {n: (pilot / n).stat().st_mtime_ns for n in installer.FILES}
    assert not installer.install(pilot)["changed"]
    assert {n: (pilot / n).stat().st_mtime_ns for n in installer.FILES} == mtimes
    result = installer.rollback(pilot, Path(receipt["receipt_path"]))
    assert result["rolled_back"]
    assert {n: (pilot / n).read_bytes() for n in installer.FILES} == original
    assert not installer.rollback(pilot, Path(receipt["receipt_path"]))["rolled_back"]


@pytest.mark.parametrize("change", ["first_drift", "second_drift", "corrupt_backup", "wrong_pilot"])
def test_rollback_refuses_drift_without_partial_writes(tmp_path, sources, change):
    pilot = staged(tmp_path, sources)
    receipt = installer.install(pilot)
    if change.endswith("drift"):
        name = "chat_server.py" if change.startswith("first") else "model_backend.py"
        with (pilot / name).open("a") as stream:
            stream.write("\n# someone else's update\n")
    elif change == "corrupt_backup":
        (pilot / receipt["files"]["model_backend.py"]["backup"]).write_text("corrupt")
    before = {n: (pilot / n).read_bytes() for n in installer.FILES}
    with pytest.raises(ValueError):
        installer.rollback(pilot / "different" if change == "wrong_pilot" else pilot, Path(receipt["receipt_path"]))
    assert {n: (pilot / n).read_bytes() for n in installer.FILES} == before


def test_second_file_write_failure_restores_first(tmp_path, sources, monkeypatch):
    pilot = staged(tmp_path, sources)
    write = installer.atomic_write
    def fail_backend(path, data, mode):
        if path == pilot / "model_backend.py":
            raise OSError("fake disk failure")
        return write(path, data, mode)
    monkeypatch.setattr(installer, "atomic_write", fail_backend)
    with pytest.raises(OSError, match="fake disk failure"):
        installer.install(pilot)
    assert {n: (pilot / n).read_text() for n in installer.FILES} == sources


def test_rollback_recovers_interrupted_pair_install(tmp_path, sources):
    pilot = staged(tmp_path, sources)
    receipt = installer.install(pilot)
    # Simulate an interruption with the first file installed and second original.
    (pilot / "model_backend.py").write_text(sources["model_backend.py"])
    assert installer.rollback(pilot, Path(receipt["receipt_path"]))["rolled_back"]
    assert {n: (pilot / n).read_text() for n in installer.FILES} == sources


def fake_executor(source, *, detail=None, gripper=2500):
    """Execute only source excerpts, with every possible transport replaced by fakes."""
    names = {"FOLD_TOWARD", "DEFAULT_PITCH_DEG", "GRIPPER_EMPTY_CLOSED", "PINCH_MAX_ABOVE",
             "PINCH_ABOVE_EMPTY", "PINCH_LOAD", "AIR_ABOVE_EMPTY", "PINCH_SHORT_TICKS", "PINCH_AIR_TICKS", "PINCH_AIR_LOAD"}
    methods = {"execute_fold", "execute_fold_path", "pinch_contradiction", "pinch_verdict"}
    selected = []
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in node.targets):
            selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in {"parse_fold", "fold_arc", "fold_words", "fold_steps"}:
            selected.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == "Chat":
            node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in methods]
            selected.append(node)
    tip = {"forward_m": .3, "left_m": .2, "up_m": .9}
    env = {"math": math, "model_claws": lambda *a: {"left_arm": tip},
           "state_ticks": lambda state: (state["ticks"], {}), "point_words": lambda point: str(point),
           "solve_reach_fn": lambda arm, target, ticks, ranges, pitch, mapping: {
               "ok": True, "ticks": {"left_arm_shoulder_lift": ticks.get("left_arm_shoulder_lift", 1000) + 10}}}
    exec(compile(ast.Module(body=selected, type_ignores=[]), "<extracted-offline-fold>", "exec"), env)
    class Fake(env["Chat"]):
        def __init__(self):
            self.calls = []
            self.last_pitch = {}
            self.last_move_detail = {}
            self.cancel = SimpleNamespace(is_set=lambda: False)
            self.last_state = {"ticks": {"left_arm_gripper": gripper, "left_arm_shoulder_lift": 1000}}
            self.last_closure = ""
            self.robot = SimpleNamespace(call=self.read_state)
        def record(self, *args, **kwargs): pass
        def record_event(self, *args, **kwargs): pass
        def emit(self, *args, **kwargs): return kwargs
        def drive_wait(self, seconds): self.calls.append(("wait", seconds)); return False
        def arm_state(self, send):
            six = [f"left_arm_{j}" for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")]
            return {"enabled_motors": six}, self.last_state["ticks"].copy(), {}, {}
        def call_tool(self, name, args, send):
            assert name == "robot_get_state"
            return self.last_state
        def read_state(self, name, args):
            assert name == "robot_get_state"
            return {"motors": [{"name": "left_arm_gripper", "Present_Load": 0}], "ticks": self.last_state["ticks"]}
        def execute_reach(self, decision, send, known, refused):
            self.calls.append(copy.deepcopy(decision))
            self.last_move_detail = detail or {}
            return ["robot_move_joint_targets: ok"]
        def execute_moves(self, calls, send, known, refused):
            assert all(c["tool"] == "robot_move_path" for c in calls)  # No enable, release or gripper.
            self.calls.extend(copy.deepcopy(calls))
            self.last_move_detail = detail or {}
            return ["robot_move_path: ok"]
    return env, Fake()


@pytest.mark.parametrize("path", [False, True])
@pytest.mark.parametrize("hinge", [False, True])
def test_open_gripper_and_no_pinch_run_unchanged_arc(sources, path, hinge):
    patched = installer.patch_sources(sources)
    all_calls = []
    for source, opening in [(sources["chat_server.py"], 2500), (patched["chat_server.py"], 2500),
                            (patched["chat_server.py"], 1357)]:
        env, fake = fake_executor(source, gripper=opening)
        d = {"arm": "left", "toward": "right", "radius_cm": 8, "degrees": 90, "steps": 3, "hold_s": 1, "path": path}
        if hinge: d["hinge_cm"] = [20, 82]
        d = env["parse_fold"](d)
        lines = fake.execute_fold(d, lambda *a: None, {"robot_move_path"}, set())
        all_calls.append(fake.calls)
        assert fake.last_state["ticks"]["left_arm_gripper"] == opening
        if source == patched["chat_server.py"]:
            assert "fold arc finished" in lines[-1] and "closure unverified" in lines[-1]
            assert "gripper command unchanged" in " ".join(lines)
            assert "gripper closed" not in " ".join(lines)
    assert all_calls[0] == all_calls[1] == all_calls[2]


@pytest.mark.parametrize("path", [False, True])
@pytest.mark.parametrize("detail", [{"closure_outcome": "contact_halt"}, {"halted": True}, {"contact": {"joint": "left_arm_elbow_flex"}}])
def test_contact_stops_after_one_dispatch_without_hold_retry_release_or_retreat(sources, path, detail):
    patched = installer.patch_sources(sources)
    env, fake = fake_executor(patched["chat_server.py"], detail=detail)
    d = env["parse_fold"]({"arm": "left", "toward": "forward", "radius_cm": 8, "steps": 6, "hold_s": 5, "path": path})
    lines = fake.execute_fold(d, lambda *a: None, {"robot_move_path"}, set())
    assert len(fake.calls) == 1
    assert "contact halt" in lines[-1] and "no automatic retry" in lines[-1]
    assert "cause and flap angle unverified" in lines[-1]
    assert "crease or the box resists" not in " ".join(lines)


@pytest.mark.parametrize("field,value", [("radius_cm", 3), ("radius_cm", 16), ("degrees", 121),
                                        ("steps", 7), ("hold_s", 31), ("pitch_end_deg", -91)])
def test_fold_parameter_limits_remain_enforced(sources, field, value):
    env, _ = fake_executor(installer.patch_sources(sources)["chat_server.py"])
    d = {"arm": "left", "toward": "forward", "radius_cm": 8, field: value}
    with pytest.raises(ValueError): env["parse_fold"](d)


def test_grasp_notes_do_not_reintroduce_lift_or_contact_retry(sources):
    env, fake = fake_executor(installer.patch_sources(sources)["chat_server.py"])
    fake.last_closure = "left PINCH LIKELY"
    note = fake.pinch_contradiction("pads empty")
    assert "Push-folding needs no pinch or lift test" in note
    for reading in (1364, 1480):
        note = fake.pinch_verdict({"arm": "left", "position_ticks": 1350},
                                  {"closure_outcome": "contact_halt", "readbacks": {"left_arm_gripper": reading}})
        assert "automatically retry" in note
        assert "lift 8 cm" not in note and "closing again is safe" not in note
