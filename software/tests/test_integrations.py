"""Soak test, teach-by-hand and the MCP tool bodies, all on the simulator. No hardware, no network."""
import csv

from farm.adapters.base import ARM_JOINTS, arm_joint
from farm.status import Reading, Status


# ---- soak -----------------------------------------------------------------------------------
class HeatingRobot:
    """health() warms one servo by `rate` C per call; everything else stays at 30 C."""

    def __init__(self, rate=1.0, start=30.0, fail_after=None):
        self.t, self.rate, self.n, self.fail_after = start, rate, 0, fail_after

    def health(self):
        self.n += 1
        if self.fail_after is not None and self.n > self.fail_after:
            return Reading(None, Status.INVALID, note="bus silent")
        self.t += self.rate
        return Reading({"right_arm_shoulder_lift": {"temperature": self.t, "load": 300.0}, "right_arm_elbow_flex": {"temperature": 30.0, "load": 100.0}}, Status.OK)


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_soak_stops_at_the_ceiling_and_writes_the_log(tmp_path):
    from farm.tools import soak
    c = Clock()
    res = soak.run(HeatingRobot(rate=1.0), minutes=60, csv_path=tmp_path / "s.csv", interval_s=60, temp_max_c=55, out=lambda s: None, sleep=c.sleep, now=c.now)
    assert "reached 55 C" in res["reason"] and res["peak_joint"] == "right_arm_shoulder_lift" and res["peak_c"] == 55.0
    assert res["samples"] == 25 and abs(res["slope_c_per_min"] - 1.0) < 0.01
    rows = list(csv.DictReader((tmp_path / "s.csv").open()))
    assert len(rows) == 50 and rows[0]["joint"] == "right_arm_shoulder_lift" and rows[-1]["t_s"] == "1440.0"
    assert "Peak 55 C on right_arm_shoulder_lift" in soak.summary(res)


def test_soak_time_up_reports_slope_and_projection(tmp_path):
    from farm.tools import soak
    c = Clock()
    res = soak.run(HeatingRobot(rate=0.5), minutes=10, csv_path=tmp_path / "s.csv", interval_s=60, temp_max_c=55, out=lambda s: None, sleep=c.sleep, now=c.now)
    assert res["reason"] == "time up" and res["peak_c"] == 35.5                 # 11 samples: 30.5 .. 35.5
    assert abs(res["slope_c_per_min"] - 0.5) < 0.01 and abs(res["minutes_to_ceiling"] - 39.0) < 0.5
    assert "ceiling is about 39 min away" in soak.summary(res)
    flat = soak.run(HeatingRobot(rate=0.0), minutes=5, csv_path=tmp_path / "f.csv", interval_s=60, out=lambda s: None, sleep=c.sleep, now=c.now)
    assert flat["minutes_to_ceiling"] is None and "Level at the end" in soak.summary(flat)


def test_soak_gives_up_on_a_silent_bus_and_honours_stop(tmp_path):
    from farm.tools import soak
    c = Clock()
    res = soak.run(HeatingRobot(fail_after=3), minutes=60, csv_path=tmp_path / "s.csv", interval_s=1, out=lambda s: None, sleep=c.sleep, now=c.now)
    assert "health read failed 5 times" in res["reason"] and res["samples"] == 3
    stopped = soak.run(HeatingRobot(), minutes=60, csv_path=tmp_path / "t.csv", interval_s=1, out=lambda s: None, sleep=c.sleep, now=c.now, should_stop=lambda: True)
    assert stopped["reason"] == "stopped" and stopped["samples"] == 1


# ---- teach by hand ----------------------------------------------------------------------------
def test_hand_teach_saves_pose_marks_who_and_restores_torque(sim):
    from farm.tools import hand_teach
    s = sim()
    joints = [arm_joint("right", j) for j in ARM_JOINTS]
    torque_during = {}

    def ask(prompt):
        if "Place it" in prompt:                      # the person moves the limp arm, then presses ENTER
            torque_during.update({j: s.robot.torque[j] for j in joints})
            for j in joints:
                s.robot.pos[j] = s.robot.goal[j] = 12.0
        return ""
    res = hand_teach.teach(s.robot, s.keyframes, "right", "pour_B_hand", "ron", ask=ask, out=lambda m: None, store=s.store)
    assert res["ok"] and set(res["joints"]) == set(joints) and all(abs(v - 12.0) < 1e-6 for v in res["joints"].values())
    assert not any(torque_during.values())                        # limp while being placed
    assert all(s.robot.torque[j] for j in joints)                  # holding afterwards
    meta = s.keyframes.meta("pour_B_hand")
    assert meta["learned_by"] == "hand:ron" and meta["arm"] == "right"
    ev = s.store.query("select payload_json from events where kind = 'keyframe_taught'")
    assert len(ev) == 1 and '"who": "ron"' in ev[0]["payload_json"]


def test_hand_teach_needs_a_name_and_is_off_in_the_default_profile(sim):
    from farm.config import load_profile
    from farm.tools import hand_teach
    s = sim()
    res = hand_teach.teach(s.robot, s.keyframes, "left", "x", "  ", ask=lambda p: "", out=lambda m: None)
    assert not res["ok"] and "name is required" in res["reason"] and "x" not in s.keyframes.names()
    assert load_profile("paper-tray-v0").teaching.by_hand is False
    assert hand_teach.joints_of("head") == ["head_motor_1", "head_motor_2"]


# ---- MCP tool bodies ----------------------------------------------------------------------------
def test_mcp_look_tools(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    t = FarmTools(s)
    st = t.get_state()
    assert st["joints_status"] == "OK" and len(st["joints"]) == 14 and st["stop_pressed"] is False
    assert st["hottest"]["temperature_c"] == 35.0 and "rest_both" in st["keyframes"] or st["keyframes"]
    data, meta = t.get_camera_image("head")
    assert data[:2] == b"\xff\xd8" and meta["shape"] == [480, 640, 3]
    import pytest
    with pytest.raises(ValueError, match="unknown camera"):
        t.get_camera_image("ceiling")
    kfs = t.list_keyframes()
    assert kfs and {"name", "arm", "learned_by", "note", "joints"} <= set(kfs[0])
    assert set(t.recent_evidence(500)) == {"cycles", "open_actions"}


def test_mcp_actions_go_through_skills_and_respect_stop(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    t = FarmTools(s)
    name = s.keyframes.names()[0]
    assert t.go_keyframe(name).startswith(("at keyframe", "did not settle"))
    assert t.go_keyframe("no_such_pose").startswith("refused: no keyframe")
    s.keyframes.save("bad_wheels", {"base_left_wheel": 10.0}, arm="")
    assert "wheel" in t.go_keyframe("bad_wheels")
    assert t.go_rest() in ("at rest",) or t.go_rest().startswith("did not settle")
    assert t.stop().startswith("stopped") and s.skills.estop.is_set()
    sent = len(s.robot.sent)
    assert t.go_rest().startswith("refused: STOP is set") and t.go_keyframe(name).startswith("refused: STOP is set")
    assert len(s.robot.sent) == sent                                # nothing commanded after STOP
    calls = s.store.query("select payload_json from events where kind = 'mcp_call'")
    assert len(calls) >= 6                                          # every action call was written down


def test_mcp_refuses_motion_while_an_action_is_unknown(sim):
    from farm.mcp_server import FarmTools
    s = sim()
    s.state["needs_person"] = True
    assert FarmTools(s).go_rest().startswith("refused: an action is UNKNOWN")


def test_mcp_server_exposes_exactly_the_seven_tools(sim):
    import asyncio
    from farm.mcp_server import build_server
    server = build_server(sim())
    names = sorted(t.name for t in asyncio.run(server.list_tools()))
    assert names == ["get_camera_image", "get_state", "go_keyframe", "go_rest", "list_keyframes", "recent_evidence", "stop"]
