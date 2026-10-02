"""bus_probe guess logic with a fake bus; light_monitor line formatting. No hardware."""
from farm.status import Reading, Status
from farm.tools import bus_probe, light_monitor


class FakeBus:
    def __init__(self, found, fail=None):
        self.found = found
        self.fail = fail
        self.disconnected = False

    def broadcast_ping(self, num_retry=0, raise_on_error=False):
        if self.fail:
            raise self.fail
        return self.found

    def disconnect(self, disable_torque=True):
        self.disconnected = True


def test_guess_bus():
    assert bus_probe.guess_bus(set(range(1, 9))) == "bus1 (left arm + head)"
    assert bus_probe.guess_bus({9, 10, 11, 12, 13, 14, 15, 16}) == "bus2 (right arm + wheels)"
    assert bus_probe.guess_bus({9, 10}) == "bus2 (right arm + wheels)"
    assert bus_probe.guess_bus(set(range(1, 8))) == "unknown"        # one head motor missing
    assert bus_probe.guess_bus(set()) == "unknown"


def test_probe_ports_with_fake_buses():
    buses = {
        "/dev/a": FakeBus({i: 777 for i in range(1, 9)}),
        "/dev/b": FakeBus({i: 777 for i in range(9, 17)}),
        "/dev/c": FakeBus(None),
        "/dev/d": FakeBus({}, fail=OSError("could not open port")),
    }

    def factory(port):
        if port == "/dev/e":
            raise FileNotFoundError(port)
        return buses[port]

    out = bus_probe.probe_ports(["/dev/a", "/dev/b", "/dev/c", "/dev/d", "/dev/e"], bus_factory=factory)
    by_port = {r["port"]: r for r in out}
    assert by_port["/dev/a"]["guess"] == "bus1 (left arm + head)" and by_port["/dev/a"]["ids"] == list(range(1, 9))
    assert by_port["/dev/a"]["models"][1] == "sts3215"
    assert by_port["/dev/b"]["guess"] == "bus2 (right arm + wheels)"
    assert "error" in by_port["/dev/c"] and "no response" in by_port["/dev/c"]["error"]
    assert by_port["/dev/d"]["error"].startswith("OSError")
    assert by_port["/dev/e"]["error"].startswith("FileNotFoundError")
    assert all(b.disconnected for p, b in buses.items() if p != "/dev/e")


def test_light_monitor_lines_carry_status():
    ok = Reading(412.5, Status.OK, seq=812, meta={"gap": False})
    gap = Reading(400.0, Status.STALE, seq=820, meta={"gap": True}, note="age 1.50s > 1.0s")
    bad = Reading(None, Status.INVALID, note="sensor error: i2c")
    assert "status=OK" in light_monitor.format_latest(ok) and "412.5" in light_monitor.format_latest(ok)
    assert "GAP" in light_monitor.format_latest(gap) and "STALE" in light_monitor.format_latest(gap)
    assert "---" in light_monitor.format_latest(bad) and "INVALID" in light_monitor.format_latest(bad)
    m = Reading({"median_lux": 400.0, "spread_lux": 3.0, "n": 10, "errors": 0, "saturated": False, "samples": []}, Status.OK)
    assert "median=400.0" in light_monitor.format_summary(m)
    assert "INVALID" in light_monitor.format_summary(Reading(None, Status.INVALID, note="only 2/10"))


# ---- robot_test: motors-only self-test on the fake robot ---------------------------------
def _tick(s):
    import time
    time.sleep(min(s, 0.01))


def _fake_robot(**faults):
    from farm.adapters.sim import FakeRobot, Faults
    f = Faults(); f.set("empty_gripper")
    for k, v in faults.items():
        f.set(k, v)
    r = FakeRobot(f, rate_per_s=5000.0)
    r.connect()
    return r


def test_robot_test_reads_every_joint_without_moving():
    from farm.tools import robot_test
    r = _fake_robot()
    lines = []
    res = robot_test.run(r, move=False, out=lines.append, sleep=_tick)
    assert res["ok"] and len(res["joints"]) == 14 and res["nudges"] == []
    assert r.sent == []                                   # nothing was commanded
    assert any("head_motor_1" in l for l in lines) and lines[-1].strip() == "ALL OK"


def test_robot_test_nudges_each_joint_and_returns():
    from farm.tools import robot_test
    r = _fake_robot()
    res = robot_test.run(r, move=True, delta=5.0, out=lambda s: None, sleep=_tick)
    assert res["ok"], res["problems"]
    assert [n["joint"] for n in res["nudges"]][:2] == ["head_motor_1", "head_motor_2"]
    assert all(n["status"] == "ok" and abs(n["moved"] - 5.0) < 0.5 for n in res["nudges"])
    assert all(abs(v) < 0.5 for v in r.joints().value.values())      # everything back where it started
    assert all(len(s) == 1 for s in r.sent)                          # one joint per command


def test_robot_test_only_group_and_upper_limit():
    from farm.tools import robot_test
    r = _fake_robot()
    r.pos["left_arm_elbow_flex"] = r.goal["left_arm_elbow_flex"] = 99.0
    res = robot_test.run(r, move=True, only="left", out=lambda s: None, sleep=_tick)
    assert res["ok"] and len(res["nudges"]) == 6
    elbow = next(n for n in res["nudges"] if n["joint"] == "left_arm_elbow_flex")
    assert elbow["target"] == 94.0                                   # near the top it nudges downward


def test_robot_test_reports_stuck_joint_wrong_part_and_heat():
    from farm.tools import robot_test
    r = _fake_robot()
    real_move = r.move_to
    r.move_to = lambda targets, max_step=None: real_move({k: v for k, v in targets.items() if k != "right_arm_wrist_roll"} or {"right_arm_wrist_roll": r.pos["right_arm_wrist_roll"]}, max_step)
    res = robot_test.run(r, move=True, only="right", out=lambda s: None, sleep=_tick)
    assert not res["ok"] and "right_arm_wrist_roll: did not move" in res["problems"]

    r2 = _fake_robot()
    res2 = robot_test.run(r2, move=True, only="head", ask=lambda q: "n", out=lambda s: None, sleep=_tick)
    assert not res2["ok"] and all("different part moved" in p for p in res2["problems"]) and len(res2["problems"]) == 2

    r3 = _fake_robot(servo_overtemp=True)
    res3 = robot_test.run(r3, move=False, only="head", out=lambda s: None, sleep=_tick)
    assert not res3["ok"] and "above 55 C" in res3["problems"][0]


def test_robot_test_describe():
    from farm.tools import robot_test
    assert robot_test.describe("head_motor_1").startswith("HEAD: head turns")
    assert robot_test.describe("right_arm_gripper") == "RIGHT arm: gripper jaw opens/closes"


# ---- calibration_report: sanity of a saved calibration -----------------------------------
def _cal(**over):
    """A plausible full calibration: every joint centred on 2047 with its nominal travel."""
    from farm.tools import calibration_report as cr
    cal = {}
    names = [f"{a}_arm_{j}" for a in ("left", "right") for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")] + ["head_motor_1", "head_motor_2"]
    for i, n in enumerate(names):
        nominal = cr.NOMINAL[cr._kind(n)][0]
        half = int(nominal / 2 / cr.DEG)
        cal[n] = {"id": i % 6 + 1, "drive_mode": 0, "homing_offset": 0, "range_min": 2047 - half, "range_max": 2047 + half}
    cal["base_left_wheel"] = {"id": 9, "drive_mode": 0, "homing_offset": 0, "range_min": 0, "range_max": 4095}
    cal["base_right_wheel"] = {"id": 10, "drive_mode": 0, "homing_offset": 0, "range_min": 0, "range_max": 4095}
    for k, v in over.items():
        cal[k].update(v)
    return cal


def test_calibration_report_accepts_a_full_sweep_and_skips_wheels():
    from farm.tools import calibration_report as cr
    res = cr.analyse(_cal())
    assert res["problems"] == [] and res["notes"] == []
    assert len(res["rows"]) == 14 and all(not r["joint"].startswith("base_") for r in res["rows"])


def test_calibration_report_flags_wrap_short_wide_mismatch_and_missing():
    from farm.tools import calibration_report as cr
    wrapped = cr.analyse(_cal(left_arm_wrist_roll={"range_min": 0, "range_max": 4095}))
    assert any("left_arm_wrist_roll" in p and "wrapped" in p for p in wrapped["problems"])
    short = cr.analyse(_cal(right_arm_elbow_flex={"range_min": 1800, "range_max": 2300}))
    assert any("right_arm_elbow_flex: swept only" in p for p in short["problems"])
    assert any("left_arm_elbow_flex and right_arm_elbow_flex" in p for p in short["problems"])      # and the arms disagree
    wide = cr.analyse(_cal(head_motor_2={"range_min": 600, "range_max": 3500}))
    assert any("head_motor_2" in p and "more than the joint can travel" in p for p in wide["problems"])
    cal = _cal(); del cal["head_motor_1"]
    assert any("missing" in p and "head_motor_1" in p for p in cr.analyse(cal)["problems"])


def test_calibration_report_notes_off_centre_and_prints(tmp_path):
    import json
    from farm.tools import calibration_report as cr
    cal = _cal(left_arm_shoulder_pan={"range_min": 2047 - 400, "range_max": 2047 + 2000})
    res = cr.analyse(cal)
    assert res["problems"] == [] and any("left_arm_shoulder_pan" in n for n in res["notes"])
    f = tmp_path / "c.json"; f.write_text(json.dumps(_cal()))
    lines = []
    assert cr.report(f, out=lines.append)["ok"] and lines[-1].strip() == "LOOKS COMPLETE"
    assert not cr.report(tmp_path / "missing.json", out=lines.append)["ok"]


def test_calibration_path_needs_no_ports():
    from farm.config import load_profile
    from farm.tools import calibration_report as cr
    p = cr.calibration_path(load_profile("paper-tray-v0").robot)
    assert p.name == "farm_xlerobot.json" and "xlerobot_2wheels" in str(p)
