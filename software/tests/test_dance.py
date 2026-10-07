import json

import pytest

from farm.adapters.joycon import JoyConHold
from farm.adapters.sim import FakeRobot, Faults
from farm.config import load_profile
from farm.safety.rules import SafetyStop
from farm.skills.dance import DANCE_JOINTS, DANCE_STEP, dance
from farm.skills.keyframes import KeyframeStore
from farm.skills.runner import SkillRunner


def make_runner(tmp_path):
    p = load_profile("sim")
    f = Faults(); f.set("empty_gripper")
    robot = FakeRobot(f, rate_per_s=600.0)
    robot.connect()
    robot.pos.update({"left_arm_shoulder_lift": 40.0, "right_arm_elbow_flex": -30.0, "left_arm_gripper": 10.0, "right_arm_gripper": 10.0})
    robot.goal = dict(robot.pos)
    return SkillRunner(robot, p.limits, p.arms, KeyframeStore(tmp_path / "kf.yaml")), robot


def test_one_loop_moves_only_dance_joints_and_returns_home(tmp_path):
    runner, robot = make_runner(tmp_path)
    start = dict(robot.pos)
    res = dance(runner, lambda: True, max_loops=1)
    assert res.ok and res.data["loops"] == 1
    assert robot.sent, "the dance sent commands"
    for cmd in robot.sent:
        assert set(cmd) <= set(DANCE_JOINTS), "no shoulder, elbow or wheel command"
    for j in ("left_arm_shoulder_lift", "right_arm_elbow_flex", "left_arm_shoulder_pan", "right_arm_shoulder_pan"):
        assert robot.pos[j] == pytest.approx(start[j]), f"{j} must not move"
    for j in DANCE_JOINTS:
        assert robot.pos[j] == pytest.approx(start[j], abs=2.5), f"{j} ends near where it started"


def test_every_command_uses_the_slow_dance_step(tmp_path):
    runner, robot = make_runner(tmp_path)
    steps = []
    orig = robot.move_to

    def spy(targets, max_step=None):
        steps.append(max_step)
        return orig(targets, max_step=max_step)
    robot.move_to = spy
    dance(runner, lambda: True, max_loops=1)
    assert steps and all(s is not None and s <= DANCE_STEP for s in steps)
    assert DANCE_STEP <= runner.limits.step_deg_max


def test_release_stops_the_dance_and_returns_to_start(tmp_path):
    runner, robot = make_runner(tmp_path)
    start = dict(robot.pos)
    calls = {"n": 0}

    def keep_going():
        calls["n"] += 1
        return calls["n"] < 6   # released a few ticks in
    res = dance(runner, keep_going)
    assert res.ok and res.data["loops"] == 0
    for j in DANCE_JOINTS:
        assert robot.pos[j] == pytest.approx(start[j], abs=2.5)


def test_stop_button_freezes_and_does_not_move_back(tmp_path):
    runner, robot = make_runner(tmp_path)
    calls = {"n": 0}

    def keep_going():
        calls["n"] += 1
        if calls["n"] == 5:
            runner.estop.set()
        return True
    with pytest.raises(SafetyStop):
        dance(runner, keep_going)
    sent_after_stop = len(robot.sent)
    assert runner._stopped
    assert len(robot.sent) == sent_after_stop


def test_refuses_with_a_tool_in_hand(tmp_path):
    runner, robot = make_runner(tmp_path)
    runner.held["right"] = "bottle"
    with pytest.raises(SafetyStop):
        dance(runner, lambda: True, max_loops=1)
    assert robot.sent == []


def test_stale_load_stops(tmp_path):
    runner, robot = make_runner(tmp_path)
    robot.load["head_motor_1"] = 10_000.0
    with pytest.raises(SafetyStop):
        dance(runner, lambda: True, max_loops=1)


# ---- Joy-Con hold trigger ---------------------------------------------------------

def frame(pressed, source="live", event="sample", connected=True, name="Right Trigger", physical=None):
    return json.dumps({"schema_version": 1, "input_only": True, "source": source, "event": event,
                       "controllers": [{"id": "r", "connected": connected, "role": "right",
                                        "buttons": {name: {"pressed": pressed, "value": 1.0 if pressed else 0.0,
                                                           "physical_names": physical or []}}}]})


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_hold_only_while_pressed_and_fresh():
    clock = Clock()
    h = JoyConHold("Right Trigger", stale_s=0.3, clock=clock)
    assert not h.is_held()
    h.feed(frame(True))
    assert h.is_held()
    clock.t += 0.5            # reader went quiet
    assert not h.is_held()
    h.feed(frame(True))
    assert h.is_held()
    h.feed(frame(False))
    assert not h.is_held()


def test_disconnect_shutdown_demo_and_garbage_count_as_released():
    clock = Clock()
    h = JoyConHold("right trigger", clock=clock)
    h.feed(frame(True)); assert h.is_held()
    h.feed(frame(True, event="disconnect")); assert not h.is_held()
    h.feed(frame(True, source="demo")); assert not h.is_held()
    h.feed("not json"); h.feed(json.dumps({"input_only": False})); assert not h.is_held()
    h.feed(frame(True, connected=False)); assert not h.is_held()
    h.feed(frame(True)); assert h.is_held()
    h.feed(frame(False, event="shutdown")); assert not h.is_held() and h.ended
    h.feed(frame(True)); assert not h.is_held(), "nothing after shutdown"


def test_matches_physical_name():
    clock = Clock()
    h = JoyConHold("ZR", clock=clock)
    h.feed(frame(True, name="Button Shoulder", physical=["ZR"]))
    assert h.is_held()
