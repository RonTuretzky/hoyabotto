"""Tape sequence contracts on fake hardware; no claims about grip/adhesion physics."""
import json
import time
from types import SimpleNamespace

import pytest

from carton.cli import seed_sim_keyframes
from carton.cycle import CartonCycle
from carton.geometry import Box
from carton.sim import SimCartonVision
from carton.tape import GRIPPER, LEFT, POSES, TapeMotion, _LockedRunner
from farm.adapters.sim import Faults, ScriptedHuman
from farm.config import load_profile
from farm.safety.rules import SafetyStop
from farm.status import Reading, Status
from farm.system import System


@pytest.fixture
def station(tmp_path, monkeypatch):
    p = load_profile("carton-sim")
    p.data_dir = str(tmp_path / "station")
    p.raw["data_dir"] = p.data_dir
    f = Faults()
    s = System(p, human=ScriptedHuman(), faults=f)
    s.backends.vision = SimCartonVision(f)
    s.connect()
    seed_sim_keyframes(s)
    # Fast fake motion, retaining real per-tick clamps and gripper contact.
    s.robot.rate = 100000
    import farm.skills.runner as runner
    monkeypatch.setattr(runner, "TICK_S", 0.001)
    yield s
    s.disconnect()


def events(s, kind):
    return [json.loads(r["payload_json"]) for r in s.store.query("select payload_json from events where kind=? order by t", (kind,))]


def test_direct_dispenser_pickup_preserves_grip_without_turnover(station, monkeypatch):
    s = station
    motion = TapeMotion(s)
    calls = []
    original = s.skills.move_joints
    def record(targets, **kwargs):
        calls.append((motion.stage, dict(targets)))
        return original(targets, **kwargs)
    monkeypatch.setattr(s.skills, "move_joints", record)
    # A stale pose says to open the hand: sequence grip still overrides it.
    for name in POSES:
        s.keyframes._d[name]["joints"][GRIPPER] = 60
    out = motion.run()
    assert out.ok, out.note
    names = [e["name"] for e in events(s, "tape_pose")]
    assert names == ["tape_dispenser_above", "tape_dispenser_grip", "tape_lift_clear",
                     "tape_over_seam", "tape_down", "tape_retract"]
    for stage, targets in calls:
        assert set(targets) <= LEFT
        if stage in POSES[2:5]:
            assert targets[GRIPPER] == 0
    checks = [e["stage"] for e in events(s, "tape_check")]
    assert checks == ["ready", "before_pinch", "pinched", "lifted", "over_seam", "supported", "released"]
    assert s.skills.held["left"] is None


@pytest.mark.parametrize("fault,value,stage,forbidden", [
    ("tape_not_clear", True, "lifted", "tape_over_seam"),
    ("tape_wrong_side", True, "ready", "tape_dispenser_above"),
    ("tape_uncut", True, "ready", "tape_dispenser_above"),
    ("tape_dropped", True, "pinched", "tape_lift_clear"),
    ("tape_unknown", "supported", "supported", "tape_retract"),
    ("obstruction", True, "ready", "tape_dispenser_above"),
])
def test_failed_checks_stop_without_releasing_or_next_pose(station, fault, value, stage, forbidden):
    s = station
    s.faults.set(fault, value)
    out = TapeMotion(s).run()
    assert not out.ok and out.stage == stage
    assert forbidden not in [e["name"] for e in events(s, "tape_pose")]
    assert s.skills.held["left"] in ("tape", "tape_unverified") if stage != "ready" else s.skills.held["left"] is None
    if stage != "ready":
        assert s.robot.goal[GRIPPER] < 60  # failure holds; it never issues a release


@pytest.mark.parametrize("change", ["missing", "mixed", "calibration", "extra_joint", "nan"])
def test_invalid_teaching_bundle_refuses_before_any_motion(station, change):
    s = station
    if change == "missing":
        s.keyframes.delete("tape_lift_clear")
    elif change == "mixed":
        s.keyframes._d["tape_lift_clear"]["learned_by"] += "other-session"
    elif change == "calibration":
        s.robot.calibration_id = "changed"
    elif change == "extra_joint":
        s.keyframes._d["tape_lift_clear"]["joints"]["head_motor_1"] = 0
    else:
        s.keyframes._d["tape_lift_clear"]["joints"]["left_arm_wrist_roll"] = float("nan")
    out = TapeMotion(s).run()
    assert not out.ok and not s.robot.sent


@pytest.mark.parametrize("bad", ["stale", "missing", "future"])
def test_both_live_camera_views_required(station, monkeypatch, bad):
    s = station
    if bad == "missing":
        del s.cameras["head"]
    else:
        frame = s.cameras["head"].frame()
        frame.t = time.time() + (10 if bad == "future" else -10)
        monkeypatch.setattr(s.cameras["head"], "frame", lambda: frame)
    out = TapeMotion(s).run()
    assert not out.ok and not s.robot.sent


@pytest.mark.parametrize("field,value", [("tape_held", "true"), ("tape_held", 1), ("obstruction", "false"),
                                        ("confidence", float("nan")), ("confidence", 0.4)])
def test_vision_reply_must_be_strict_and_confident(station, monkeypatch, field, value):
    s = station
    original = s.backends.vision.complete_json
    def reply(*args, **kwargs):
        d, meta = original(*args, **kwargs)
        if args[0].startswith("TAPE_CHECK stage=pinched"):
            d[field] = value
        return d, meta
    monkeypatch.setattr(s.backends.vision, "complete_json", reply)
    out = TapeMotion(s).run()
    assert not out.ok and out.stage == "pinched"
    assert "tape_lift_clear" not in [e["name"] for e in events(s, "tape_pose")]


def test_stop_during_vision_call_wins_over_good_reply(station, monkeypatch):
    s = station
    original = s.backends.vision.complete_json
    def reply(*args, **kwargs):
        d, meta = original(*args, **kwargs)
        if args[0].startswith("TAPE_CHECK stage=lifted"):
            s.skills.estop.set()
        return d, meta
    monkeypatch.setattr(s.backends.vision, "complete_json", reply)
    out = TapeMotion(s).run()
    assert not out.ok and "STOP" in out.note
    assert "tape_over_seam" not in [e["name"] for e in events(s, "tape_pose")]


def test_locked_teaching_runner_rejects_other_arm_and_failed_move(station, monkeypatch):
    motion = TapeMotion(station)
    proxy = _LockedRunner(motion, 0)
    with pytest.raises(SafetyStop):
        proxy.move_arm_pose("right", station.skills.models["right"].pose)
    captured = []
    def fail(targets, **kwargs):
        captured.append(targets)
        return SimpleNamespace(ok=False, note="timeout")
    monkeypatch.setattr(station.skills, "move_joints", fail)
    with pytest.raises(SafetyStop, match="did not settle"):
        proxy.move_arm_pose("left", station.skills.models["left"].pose)
    assert captured[0][GRIPPER] == 0 and set(captured[0]) == LEFT


def test_teaching_publishes_only_after_full_success(station, monkeypatch):
    s = station
    from farm.skills.llm_servo import LLMServo
    def done(self, arm, goal, save_as=None, allow_gripper=True):
        assert save_as is None and allow_gripper is False
        return SimpleNamespace(ok=True)
    monkeypatch.setattr(LLMServo, "run", done)
    before = {n: s.keyframes.meta(n)["learned_by"] for n in POSES}
    s.faults.set("tape_wrong_side")
    bad = TapeMotion(s).run(teach=True)
    assert not bad.ok
    assert {n: s.keyframes.meta(n)["learned_by"] for n in POSES} == before
    s.faults.clear("tape_wrong_side")
    good = TapeMotion(s).run(teach=True)
    assert good.ok, good.note
    tags = {s.keyframes.meta(n)["learned_by"] for n in POSES}
    assert len(tags) == 1 and tags != set(before.values())
    TapeMotion(s)._preflight_poses()


def test_cycle_does_not_retry_failed_tape(station):
    s = station
    s.faults.set("tape_wrong_side")
    out = CartonCycle(s, Box()).run()
    assert out.result == "STOPPED"
    assert "press" not in out.steps_done
    assert len(s.store.query("select * from actions where skill='tape'")) == 1


def test_plan_command_and_individual_tape_teach_never_connect(monkeypatch):
    from carton import cli
    def forbidden(_):
        raise AssertionError("must not connect")
    monkeypatch.setattr(cli, "_system", forbidden)
    cli.main(["tape-test", "--plan"])
    with pytest.raises(SystemExit, match="taught together"):
        cli.main(["teach", "--name", "tape_lift_clear"])


def test_live_tape_trial_refuses_unattended_stdin_before_connect(monkeypatch):
    from carton import cli
    monkeypatch.setattr(cli.sys.stdin, "isatty", lambda: False)
    monkeypatch.setattr(cli, "_system", lambda _: pytest.fail("must not connect"))
    with pytest.raises(SystemExit, match="attended interactive"):
        cli.main(["tape-test", "--teach", "-p", "carton-v0"])


def test_wrong_stop_viewer_is_rejected(station, monkeypatch):
    from carton.cli import _start_viewer
    import farm.viewer.app
    import urllib.request
    import io
    monkeypatch.setattr(farm.viewer.app, "serve_in_thread", lambda *args: SimpleNamespace(is_alive=lambda: True))
    monkeypatch.setattr(urllib.request, "urlopen", lambda *args, **kwargs: io.StringIO('{"carton_viewer_session":"different-robot"}'))
    with pytest.raises(RuntimeError, match="STOP viewer"):
        _start_viewer(station)


def test_single_teach_disconnects_on_viewer_failure(station, monkeypatch):
    from carton import cli
    monkeypatch.setattr(cli, "_system", lambda _: station)
    monkeypatch.setattr(cli, "_start_viewer", lambda _: (_ for _ in ()).throw(RuntimeError("viewer failed")))
    monkeypatch.setattr(cli, "_teach_one", lambda *args: pytest.fail("must not move"))
    with pytest.raises(RuntimeError, match="viewer failed"):
        cli.cmd_teach(SimpleNamespace(name="far_touch"))
    assert not station.robot._connected


def test_batch_teach_stops_on_first_failed_pose(station, monkeypatch):
    from carton import cli
    monkeypatch.setattr(cli, "_system", lambda _: station)
    monkeypatch.setattr(cli, "_start_viewer", lambda _: None)
    calls = []
    monkeypatch.setattr(cli, "_teach_one", lambda _, kf: calls.append(kf.name) or False)
    with pytest.raises(SystemExit) as exc:
        cli.cmd_teach_all(SimpleNamespace(force=True))
    assert exc.value.code == 1 and calls == ["look_box"]
    assert not station.robot._connected


def test_legacy_turnover_bundle_cannot_replay(station):
    for name in POSES:
        station.keyframes._d[name]["learned_by"] = "carton-tape-v1:old-station"
    out = TapeMotion(station).run()
    assert not out.ok and not station.robot.sent


def test_removed_turnover_pose_refuses_before_connect(monkeypatch):
    from carton import cli
    monkeypatch.setattr(cli, "_system", lambda _: pytest.fail("must not connect"))
    with pytest.raises(SystemExit, match="Unknown keyframe"):
        cli.main(["teach", "--name", "tape_turn_half"])


def test_lost_orientation_after_pickup_stops_before_carry(station, monkeypatch):
    original = station.backends.vision.complete_json
    def reply(*args, **kwargs):
        d, meta = original(*args, **kwargs)
        if args[0].startswith("TAPE_CHECK stage=lifted"):
            d["adhesive_down"] = False
        return d, meta
    monkeypatch.setattr(station.backends.vision, "complete_json", reply)
    out = TapeMotion(station).run()
    assert not out.ok and out.stage == "lifted"
    assert "tape_over_seam" not in [e["name"] for e in events(station, "tape_pose")]
    assert station.skills.held["left"] == "tape"
