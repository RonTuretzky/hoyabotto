import threading
from types import SimpleNamespace

import pytest

from farm.control.feetech import ConnectedFeetechActuator
from farm.control.jev import JOINTS, MOTORS, WHEELS, Limits, MotionRefused, action_catalog


class Bus:
    def __init__(self, motors):
        self.motors = dict.fromkeys(motors)
        self.values = {m: {"Homing_Offset": 0, "Min_Position_Limit": 100, "Max_Position_Limit": 3900,
                          "Present_Position": 2000, "Goal_Position": 2000, "Present_Velocity": 0,
                          "Goal_Velocity": 90, "Present_Temperature": 30, "Present_Load": 0,
                          "Status": 0, "Torque_Enable": 1, "Operating_Mode": int(m in WHEELS)} for m in motors}
        self.writes = []
        self.fail_wheel = None

    def read(self, field, motor, **kw):
        return self.values[motor][field]

    def write(self, field, motor, value, **kw):
        self.writes.append((motor, field, value))
        if motor == self.fail_wheel:
            raise IOError("bus failure")
        self.values[motor][field] = value
        if field == "Goal_Position":
            self.values[motor]["Present_Position"] = value


def robot():
    bus1 = Bus([m for m in JOINTS if m.startswith("left") or m.startswith("head")])
    bus2 = Bus([m for m in MOTORS if m not in bus1.motors])
    r = SimpleNamespace(bus1=bus1, bus2=bus2,
                        calibration={m: SimpleNamespace(id=i, homing_offset=0, range_min=100, range_max=3900)
                                     for i, m in enumerate(JOINTS)})
    return r


def test_constructor_checks_saved_calibration_without_writing():
    r = robot()
    a = ConnectedFeetechActuator(r, threading.RLock())
    assert set(a.snapshot()["health"]) == set(MOTORS)
    assert not r.bus1.writes and not r.bus2.writes
    r.bus1.values["head_motor_1"]["Homing_Offset"] = 10
    with pytest.raises(MotionRefused, match="calibration differ"):
        ConnectedFeetechActuator(r, threading.RLock())
    assert not r.bus1.writes and not r.bus2.writes


@pytest.mark.parametrize("name", ["left_arm_gripper_increase", "right_arm_elbow_flex_decrease", "head_motor_2_increase", "drive_forward"])
def test_execution_writes_real_registers_and_restores_speed(name):
    r = robot(); a = ConnectedFeetechActuator(r, threading.RLock())
    limits = Limits(pulse_s=.02)
    action = action_catalog(limits)[name]
    before = a.snapshot()
    a.execute(action, before, limits, lambda: None)
    for m, delta in action.joints.items():
        assert a.snapshot()["positions"][m] == before["positions"][m] + delta
        assert a.buses[m].values[m]["Goal_Velocity"] == 90
    for m in WHEELS:
        assert r.bus2.values[m]["Goal_Velocity"] == 0
    assert all(f in ("Goal_Position", "Goal_Velocity") for m, f, v in r.bus1.writes + r.bus2.writes)


def test_one_failed_wheel_does_not_prevent_stopping_other_wheel():
    r = robot(); a = ConnectedFeetechActuator(r, threading.RLock())
    r.bus2.fail_wheel = WHEELS[0]
    with pytest.raises(MotionRefused, match="STOP"):
        a.stop()
    assert (WHEELS[1], "Goal_Velocity", 0) in r.bus2.writes


def test_disabled_joint_is_never_enabled_and_wrong_wheel_mode_cannot_drive():
    r = robot(); a = ConnectedFeetechActuator(r, threading.RLock())
    r.bus1.values["head_motor_1"]["Torque_Enable"] = 0
    limits = Limits()
    with pytest.raises(MotionRefused, match="disabled"):
        a.execute(action_catalog(limits)["head_motor_1_increase"], a.snapshot(), limits, lambda: None)
    assert not any(f == "Torque_Enable" for m, f, v in r.bus1.writes + r.bus2.writes)
