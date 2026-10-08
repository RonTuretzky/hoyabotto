"""Auto-calibration wrapper: file merging, staging and refusal paths with a fake workflow. The vendored
limit-seeking code itself needs real servos and is NOT exercised here."""
import json
import copy
import pytest
from dataclasses import dataclass
from pathlib import Path

from farm.tools import auto_calibrate as ac

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


@dataclass
class Cfg:
    port1: str = "/dev/left"
    port2: str = "/dev/right"
    id: str = "t"
    calibration_dir: str = ""


class FakeWorkflow:
    def __init__(self, rc=0, joints=JOINTS):
        self.rc, self.joints, self.calls = rc, joints, []

    def run_full_calibration(self, port, *, save, velocity_limit, timeout_s, interactive, result_sink):
        self.calls.append(("full", port, velocity_limit))
        if self.rc == 0:
            for i, j in enumerate(self.joints):
                result_sink[j] = {"id": i + 1, "drive_mode": 0, "homing_offset": 10 * i, "range_min": 800 + i, "range_max": 3300 - i}
        return self.rc

    def calibrate_single_motor(self, port, motor, *, velocity_limit, timeout_s, interactive):
        self.calls.append(("motor", port, motor)); return 0

    def unfold_joints(self, port, angle, *, interactive):
        self.calls.append(("unfold", port, angle)); return 0


def test_merge_keeps_other_entries_and_adds_wheels(tmp_path):
    f = tmp_path / "c.json"
    f.write_text(json.dumps({"head_motor_1": {"id": 7, "drive_mode": 0, "homing_offset": 5, "range_min": 100, "range_max": 200}}))
    cal = ac.merge_arm(f, "left", {j: {"id": i + 1, "drive_mode": 0, "homing_offset": 1, "range_min": 900, "range_max": 3000} for i, j in enumerate(JOINTS)})
    assert cal["head_motor_1"]["homing_offset"] == 5 and cal["left_arm_wrist_roll"]["id"] == 5
    assert cal["base_left_wheel"] == {"id": 9, "drive_mode": 0, "homing_offset": 0, "range_min": 0, "range_max": 4095}
    assert json.loads(f.read_text()) == cal
    missing = ac.missing_for_connect(cal)
    assert "head_motor_2" in missing and "right_arm_gripper" in missing and "left_arm_gripper" not in missing


def test_full_run_saves_only_after_yes_and_uses_the_arms_port(tmp_path):
    f = tmp_path / "c.json"
    wf, lines = FakeWorkflow(), []
    assert ac.run_arm(Cfg(), "right", ask=lambda q: "no", out=lines.append, workflow=wf, cal_path=f) == 2
    assert wf.calls == [] and not f.exists()                                   # declined: nothing moved, nothing written
    assert ac.run_arm(Cfg(), "right", ask=lambda q: "yes", out=lines.append, workflow=wf, cal_path=f) == 0
    assert wf.calls == [("full", "/dev/right", 1000)]
    cal = json.loads(f.read_text())
    assert cal["right_arm_shoulder_pan"]["range_min"] == 800 and "left_arm_shoulder_pan" not in cal
    assert any("still missing" in l and "head_motor_1" in l for l in lines)
    assert any("battery switch" in l for l in lines)                           # the checklist was shown


def test_failure_or_partial_result_leaves_the_file_alone(tmp_path):
    f = tmp_path / "c.json"; f.write_text("{}")
    assert ac.run_arm(Cfg(), "left", ask=lambda q: "yes", out=lambda s: None, workflow=FakeWorkflow(rc=130), cal_path=f) == 130
    assert ac.run_arm(Cfg(), "left", ask=lambda q: "yes", out=lambda s: None, workflow=FakeWorkflow(joints=JOINTS[:4]), cal_path=f) == 1
    assert f.read_text() == "{}"
    assert ac.run_arm(Cfg(port1=""), "left", ask=lambda q: "yes", out=lambda s: None, workflow=FakeWorkflow(), cal_path=f) == 1
    assert ac.run_arm(Cfg(), "head", ask=lambda q: "yes", out=lambda s: None, workflow=FakeWorkflow(), cal_path=f) == 1


def test_staged_modes_do_not_write_the_file(tmp_path):
    f = tmp_path / "c.json"
    wf = FakeWorkflow()
    assert ac.run_arm(Cfg(), "left", mode="motor", motor="gripper", ask=lambda q: "yes", out=lambda s: None, workflow=wf, cal_path=f) == 0
    assert ac.run_arm(Cfg(), "left", mode="unfold", ask=lambda q: "yes", out=lambda s: None, workflow=wf, cal_path=f) == 0
    assert ac.run_arm(Cfg(), "left", mode="motor", motor="elbow", ask=lambda q: "yes", out=lambda s: None, workflow=wf, cal_path=f) == 1
    assert [c[0] for c in wf.calls] == ["motor", "unfold"] and wf.calls[0][1:] == ("/dev/left", "gripper") and not f.exists()


class FakeHeadBus:
    def __init__(self):
        self.log = []
        self.settings = {n: {"Torque_Enable": 0, "Homing_Offset": 20, "Min_Position_Limit": 1200, "Max_Position_Limit": 2800} for n in ac.HEAD}
        self.initial = copy.deepcopy(self.settings)

    def connect(self): self.log.append("connect")
    def disable_torque(self, **kw): self.log.append("limp")
    def read(self, reg, motor, **kw): return self.settings[motor][reg]
    def write(self, reg, motor, value, **kw): self.settings[motor][reg] = value
    def set_half_turn_homings(self, motors):
        self.log.append("home")
        for m in motors:
            self.settings[m].update(Homing_Offset=12, Min_Position_Limit=0, Max_Position_Limit=4095)
        return {m: 12 for m in motors}
    def record_ranges_of_motion(self, motors, **kw):
        self.log.append("sweep")
        return {"head_motor_1": 1000, "head_motor_2": 1600}, {"head_motor_1": 3000, "head_motor_2": 2500}
    def disconnect(self, **kw): self.log.append("disconnect")


def test_head_is_merged_and_bus_released(tmp_path):
    f = tmp_path / "c.json"; f.write_text(json.dumps({"left_arm_gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 1, "range_max": 2}}))
    bus = FakeHeadBus()
    assert ac.calibrate_head(Cfg(), ask=lambda q: "", out=lambda s: None, bus=bus, cal_path=f) == 0
    cal = json.loads(f.read_text())
    assert cal["head_motor_1"] == {"id": 7, "drive_mode": 0, "homing_offset": 12, "range_min": 1000, "range_max": 3000} and cal["head_motor_2"]["id"] == 8
    assert "left_arm_gripper" in cal and bus.log[0] == "connect" and bus.log[-1] == "disconnect"
    for n in ac.HEAD:
        assert bus.settings[n] == {"Torque_Enable": 0, "Homing_Offset": cal[n]["homing_offset"], "Min_Position_Limit": cal[n]["range_min"], "Max_Position_Limit": cal[n]["range_max"]}


@pytest.mark.parametrize("failure", ["interrupt", "invalid_range", "readback", "save"])
def test_head_failures_restore_hardware_and_keep_file(tmp_path, monkeypatch, failure):
    f = tmp_path / "c.json"
    original = '{"left_arm_gripper": {"id": 6}}'
    f.write_text(original)
    bus = FakeHeadBus()
    if failure == "interrupt":
        def interrupt(*args, **kw): raise KeyboardInterrupt()
        bus.record_ranges_of_motion = interrupt
    elif failure == "invalid_range":
        bus.record_ranges_of_motion = lambda *args, **kw: ({n: 0 for n in ac.HEAD}, {n: 4095 for n in ac.HEAD})
    elif failure == "readback":
        write = bus.write
        def ignore_new_min(reg, motor, value, **kw):
            if reg != "Min_Position_Limit" or value == 1200:
                write(reg, motor, value, **kw)
        bus.write = ignore_new_min
    else:
        def fail_replace(*args): raise OSError("disk error")
        monkeypatch.setattr(ac.os, "replace", fail_replace)
    with pytest.raises((KeyboardInterrupt, ValueError, RuntimeError, OSError)):
        ac.calibrate_head(Cfg(), ask=lambda q: "", out=lambda s: None, bus=bus, cal_path=f)
    assert bus.settings == bus.initial
    assert f.read_text() == original
    assert bus.log[-1] == "disconnect"
    assert not list(tmp_path.glob("*.tmp"))


def test_merged_file_is_what_the_robot_class_loads(tmp_path):
    """A file built from two auto runs plus the head step must load as the XLeRobot calibration (no serial port is opened)."""
    from farm.vendor.config_xlerobot_2wheels import XLerobot2WheelsConfig
    from farm.vendor.xlerobot_2wheels import XLerobot2Wheels
    f = tmp_path / "farm_t.json"
    six = {j: {"id": i + 1, "drive_mode": 0, "homing_offset": 3, "range_min": 900, "range_max": 3100} for i, j in enumerate(JOINTS)}
    ac.merge_arm(f, "left", six); ac.merge_arm(f, "right", six)
    ac.calibrate_head(Cfg(), ask=lambda q: "", out=lambda s: None, bus=FakeHeadBus(), cal_path=f)
    assert ac.missing_for_connect(json.loads(f.read_text())) == []
    rc = XLerobot2WheelsConfig(id="farm_t", port1="/dev/null1", port2="/dev/null2", cameras={})
    rc.calibration_dir = Path(tmp_path)
    r = XLerobot2Wheels(rc)
    assert set(r.calibration) == set(r.bus1.motors) | set(r.bus2.motors)
    assert r.calibration["right_arm_wrist_roll"].range_max == 3100 and r.calibration["head_motor_2"].id == 8


def test_vendored_code_imports_and_has_the_stock_bus_underneath():
    from lerobot.motors.feetech import FeetechMotorsBus as Stock
    from farm.vendor.autocal import calibration_defaults as d, workflow
    assert issubclass(workflow.FeetechMotorsBus, Stock) and hasattr(workflow.FeetechMotorsBus, "measure_ranges_of_motion_multi")
    assert d.MOTOR_NAMES == JOINTS and d.CALIBRATE_FIRST == ["shoulder_pan"]
