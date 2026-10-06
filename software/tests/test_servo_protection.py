"""servo_protection on a fake Feetech bus. No hardware."""
import pytest

from farm.tools import servo_protection as sp


class FakeBus:
    """Registers per motor; records every write in order. Lock must be 0 for EEPROM writes."""
    EEPROM = set(sp.REGISTERS)

    def __init__(self, motors, fail_write=None):
        self.motors = {m: object() for m in motors}
        self.regs = {m: {"Max_Temperature_Limit": 70, "Unloading_Condition": 44, "LED_Alarm_Condition": 47,
                         "Torque_Enable": 1, "Lock": 1} for m in motors}
        self.writes = []
        self.fail_write = fail_write

    def read(self, reg, motor, normalize=False, num_retry=0):
        return self.regs[motor][reg]

    def write(self, reg, motor, value, normalize=True, num_retry=0):
        self.writes.append((motor, reg, value))
        if reg == self.fail_write:
            raise RuntimeError("bus error")
        if reg in self.EEPROM and self.regs[motor]["Lock"] != 0:
            return                      # a locked servo silently ignores EEPROM writes
        self.regs[motor][reg] = value


def test_plan_raises_limit_and_clears_only_the_temperature_bit():
    assert sp.plan({"Max_Temperature_Limit": 70, "Unloading_Condition": 44, "LED_Alarm_Condition": 47}) == \
        {"Max_Temperature_Limit": 100, "Unloading_Condition": 40, "LED_Alarm_Condition": 43}
    already = {"Max_Temperature_Limit": 100, "Unloading_Condition": 40, "LED_Alarm_Condition": 43}
    assert sp.plan(already) == already and sp.done(already)
    assert not sp.done({"Max_Temperature_Limit": 100, "Unloading_Condition": 44, "LED_Alarm_Condition": 43})


def test_apply_unlocks_writes_relocks_and_reads_back():
    bus = FakeBus(["left_arm_gripper"])
    after = sp.apply(bus, "left_arm_gripper")
    assert after == {"Max_Temperature_Limit": 100, "Unloading_Condition": 40, "LED_Alarm_Condition": 43}
    regs = [w[1] for w in bus.writes]
    assert regs[:2] == ["Torque_Enable", "Lock"] and regs[-1] == "Lock"
    assert bus.writes[1][2] == 0 and bus.writes[-1][2] == 1
    assert bus.regs["left_arm_gripper"]["Lock"] == 1


def test_apply_relocks_and_fails_loudly_when_the_servo_ignores_the_write():
    bus = FakeBus(["m"], fail_write="Unloading_Condition")
    with pytest.raises(RuntimeError, match="bus error"):
        sp.apply(bus, "m")
    assert bus.writes[-1] == ("m", "Lock", 1)

    class Stubborn(FakeBus):
        def write(self, reg, motor, value, normalize=True, num_retry=0):
            if reg != "Max_Temperature_Limit":
                super().write(reg, motor, value, normalize, num_retry)
    with pytest.raises(RuntimeError, match="reads back"):
        sp.apply(Stubborn(["m"]), "m")


def test_run_dry_reads_only_and_write_changes_every_motor(capsys):
    b1, b2 = FakeBus(["head_motor_1", "left_arm_gripper"]), FakeBus(["right_arm_gripper", "base_left_wheel"])
    lines = []
    res = sp.run([b1, b2], write=False, out=lines.append)
    assert res["ok"] and not res["written"] and not b1.writes and not b2.writes
    assert set(res["motors"]) == {"head_motor_1", "left_arm_gripper", "right_arm_gripper", "base_left_wheel"}
    assert all(m["after"] is None for m in res["motors"].values())
    assert any("still have temperature protection" in l for l in lines)

    res = sp.run([b1, b2], write=True, only=["left_arm_gripper"], out=lines.append)
    assert res["ok"] and list(res["motors"]) == ["left_arm_gripper"]
    assert b1.regs["left_arm_gripper"]["Max_Temperature_Limit"] == 100 and b1.regs["head_motor_1"]["Max_Temperature_Limit"] == 70

    res = sp.run([b1, b2], write=True, out=lines.append)
    assert res["ok"] and all(sp.done(bus.regs[m]) for bus in (b1, b2) for m in bus.motors)
    assert res["motors"]["left_arm_gripper"]["after"] == res["motors"]["left_arm_gripper"]["before"]   # already done, untouched
    assert "ALL DONE" in lines[-1]


def test_run_reports_a_failing_servo_and_continues():
    bus = FakeBus(["a", "b"], fail_write="Lock")
    res = sp.run([bus], write=True, out=lambda s: None)
    assert not res["ok"] and "error" in res["motors"]["a"] and "error" in res["motors"]["b"]


def test_readonly_cli_does_not_disable_torque_and_closes_partial_connection(monkeypatch):
    from types import SimpleNamespace
    from farm import cli
    import farm.config
    import farm.adapters.robot_lerobot
    calls=[]
    class Bus:
        def __init__(self, fail=False): self.fail=fail
        def connect(self, **kwargs):
            calls.append(("connect",kwargs))
            if self.fail:raise RuntimeError("silent second bus")
        def disconnect(self, **kwargs):calls.append(("disconnect",kwargs))
    robot=SimpleNamespace(bus1=Bus(),bus2=Bus(True))
    monkeypatch.setattr(farm.config,"load_profile",lambda _:SimpleNamespace(simulated=False,robot=SimpleNamespace(kind="xlerobot")))
    monkeypatch.setattr(farm.adapters.robot_lerobot,"LeRobotXLeRobot",lambda _:SimpleNamespace(robot=robot))
    import pytest
    with pytest.raises(RuntimeError,match="silent second bus"):
        cli.cmd_servo_protection(SimpleNamespace(profile="unused",write=False,only=None))
    assert calls==[("connect",{"handshake":False}),("connect",{"handshake":False}),("disconnect",{"disable_torque":False})]
