import threading
import time

import pytest

from farm.control.jev import (JOINTS, WHEELS, Limits, MotionController, MotionRefused,
                              SimActuator, action_catalog)
from farm.llm.backends import Meta


class Backend:
    def __init__(self, choice, p=1, confidence=1, callback=None):
        self.choice, self.p, self.confidence, self.callback = choice, p, confidence, callback
        self.calls = 0

    def decide(self, state, questions):
        self.calls += 1
        if self.callback:
            self.callback()
        answers = {}
        for name, q in questions.items():
            choice = self.choice[name] if isinstance(self.choice, dict) else self.choice
            probs = {n: ((1-self.p)/(len(q["criteria"])-1)) for n in q["criteria"]}
            probs[choice] = self.p
            answers[name] = {"choice": choice, "probabilities": probs, "confidence": self.confidence}
        return answers, Meta("test", "test", 1)


def observation(*actions, seq=1):
    return {"sequence": seq, "observed_at": time.time(), "camera_ok": True, "stop_requested": False,
            "allowed_actions": list(actions), "base_clear": True, "arms_stowed": True}


@pytest.mark.parametrize("name", list(action_catalog(Limits())))
def test_all_36_motions_execute_and_stop(name):
    robot = SimActuator()
    c = MotionController(robot, Backend(name), "perform bounded test", Limits(pulse_s=0.02))
    before = robot.snapshot()
    out = c.step(observation(name))
    assert out["executed"] and robot.sent == [name]
    assert robot.velocities == dict.fromkeys(WHEELS, 0)
    for j in JOINTS:
        assert abs(robot.positions[j] - before["positions"][j]) <= 32


@pytest.mark.parametrize("mutation", [
    lambda s: s.update(observed_at=time.time()-10),
    lambda s: s.update(observed_at=time.time()+10),
    lambda s: s.update(observed_at=float("nan")),
    lambda s: s.update(camera_ok=False),
    lambda s: s.update(stop_requested=True),
    lambda s: s.update(allowed_actions=["shell:rm"]),
    lambda s: s.update(base_clear=False),
    lambda s: s.update(arms_stowed=False),
])
def test_invalid_observation_cannot_dispatch(mutation):
    robot, backend = SimActuator(), Backend("drive_forward")
    c = MotionController(robot, backend, "drive")
    s = observation("drive_forward")
    mutation(s)
    assert not c.step(s)["executed"]
    assert backend.calls == 0 and not robot.sent and robot.stop_count == 1


@pytest.mark.parametrize("mutation", [
    lambda s: s["health"].pop(WHEELS[0]),
    lambda s: s["health"][WHEELS[0]].update(temperature=60),
    lambda s: s["health"][JOINTS[0]].update(load=float("nan")),
    lambda s: s["health"][JOINTS[0]].update(status=1),
    lambda s: s["positions"].update(head_motor_1=5000),
    lambda s: s["wheel_velocities"].update(base_left_wheel=100),
    lambda s: s.update(observed_at=time.time()-1),
])
def test_bad_readback_refuses_before_inference(mutation):
    robot = SimActuator()
    original = robot.snapshot
    def bad():
        s = original(); mutation(s); return s
    robot.snapshot = bad
    backend = Backend("head_motor_1_increase")
    assert not MotionController(robot, backend, "test").step(observation("head_motor_1_increase"))["executed"]
    assert not backend.calls


def test_pose_drift_late_answer_low_confidence_replay_and_stop():
    robot = SimActuator()
    backend = Backend("head_motor_1_increase", callback=lambda: robot.positions.update(head_motor_2=2100))
    c = MotionController(robot, backend, "test")
    assert "moved" in c.step(observation("head_motor_1_increase"))["reason"]
    assert not c.step(observation("head_motor_1_increase"))["executed"]
    backend.callback = lambda: c.request_stop()
    assert not c.step(observation("head_motor_1_increase", seq=2))["executed"]
    assert not robot.sent
    c = MotionController(robot, Backend("head_motor_1_increase", p=0.8), "test")
    assert not c.step(observation("head_motor_1_increase"))["executed"]
    c = MotionController(robot, Backend("head_motor_1_increase", callback=lambda: time.sleep(0.12)), "test", Limits(observation_age_s=0.1))
    assert not c.step(observation("head_motor_1_increase"))["executed"]


def test_wheel_pulse_interruption_stops_both_wheels():
    robot = SimActuator()
    c = MotionController(robot, Backend("drive_forward"), "drive")
    timer = threading.Timer(0.04, c.request_stop)
    timer.start()
    out = c.step(observation("drive_forward"))
    timer.join()
    assert not out["executed"] and robot.velocities == dict.fromkeys(WHEELS, 0)


def test_stop_failure_propagates_and_limits_fail_closed():
    robot = SimActuator()
    robot.stop = lambda: (_ for _ in ()).throw(MotionRefused("stop failed"))
    with pytest.raises(MotionRefused, match="stop failed"):
        MotionController(robot, Backend("hold"), "test").step(observation("head_motor_1_increase"))
    for kwargs in ({"step_ticks": 1000}, {"pulse_s": 5}, {"wheel_speed": float("nan")}, {"min_probability": .5}):
        with pytest.raises(ValueError):
            Limits(**kwargs)
