"""tools/tag_joint_check.py: read-only candidate degrees and predicted +40 tick motion; no hardware."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("lerobot")
from farm.kinematics.assets import verified_model
from farm.kinematics.tag_registration import candidate_degrees
from tools import tag_joint_check as T

NAMES = [f"right_arm_{n}" for n in T.JOINTS]
RANGES = {n: {"min_ticks": 1000, "max_ticks": 3000} for n in NAMES}
TICKS = dict(zip(NAMES, (2000, 1800, 2200, 2100, 2000)))


class LinearArm:
    """Gripper position is linear in the joint degrees: pan -> -y, lift -> +z, elbow -> +x, wrist_flex -> -z, roll -> none."""
    names = list(T.JOINTS)
    axes = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0], [0, 0, -1], [0, 0, 0]], float) * .005

    def __init__(self, folder=None):
        self.folder = folder

    def forward(self, q):
        q = np.asarray(q, float)
        if np.any(np.abs(q) > 100):
            raise ValueError("outside upstream URDF joint limits")
        t = np.eye(4)
        t[:3, 3] = q @ self.axes
        c, s = np.cos(np.radians(q[4])), np.sin(np.radians(q[4]))
        t[:2, :2] = [[c, -s], [s, c]]
        return t


def test_candidate_degrees_and_directions_follow_the_model():
    result = T.predict(TICKS, RANGES, LinearArm())
    assert [j["candidate_degrees"] for j in result["joints"]] == pytest.approx(candidate_degrees(TICKS, RANGES, NAMES))
    rows = {j["joint"]: j for j in result["joints"]}
    for row in rows.values():
        assert row["step_degrees"] == pytest.approx(40*360/4095, abs=.05)  # LeRobot DEGREES: about 3.5 deg per 40 ticks
    assert "right" in rows["shoulder_pan"]["predicted"] and rows["shoulder_pan"]["gripper_delta_m"][1] < 0
    assert rows["shoulder_lift"]["predicted"].startswith("shoulder_lift +40 ticks (+3.5 deg) -> gripper moves 1.8 cm (up 1.8 cm)")
    assert "forward" in rows["elbow_flex"]["predicted"] and "down" in rows["wrist_flex"]["predicted"]
    assert "under 0.5 mm" in rows["wrist_roll"]["predicted"] and rows["wrist_roll"]["gripper_rotation_deg"] == pytest.approx(3.5, abs=.1)
    assert result["motor_writes"] == 0


def test_step_leaving_the_saved_range_is_reported_not_extrapolated():
    ticks = dict(TICKS, right_arm_elbow_flex=2990)
    rows = {j["joint"]: j for j in T.predict(ticks, RANGES, LinearArm())["joints"]}
    assert rows["elbow_flex"]["predicted"] is None and "outside its recorded calibration range" in rows["elbow_flex"]["note"]
    assert rows["shoulder_lift"]["predicted"]


def test_cli_reads_a_saved_robot_get_state_reply(tmp_path, capsys):
    motors = [{"name": n, "Present_Position": q} for n, q in TICKS.items()] + [{"name": "left_arm_shoulder_pan", "Present_Position": 5}]
    state = tmp_path / "state.json"
    state.write_text(json.dumps({"ok": True, "result": {"motors": motors, "raw_calibration_ranges": RANGES}}))
    assert T.main(["--model-directory", "unused", "--state", str(state), "--json"], solver_factory=LinearArm) == 0
    out = json.loads(capsys.readouterr().out)
    assert [j["ticks"] for j in out["joints"]] == list(TICKS.values()) and out["arm"] == "right"
    # --ticks override the state; ranges from a LeRobot calibration file.
    cal = tmp_path / "cal.json"
    cal.write_text(json.dumps({n: {"range_min": 1000, "range_max": 3000, "homing_offset": 0} for n in NAMES}))
    argv = ["--model-directory", "unused", "--calibration", str(cal), "--ticks"] + [f"{n.removeprefix('right_arm_')}={q}" for n, q in TICKS.items()]
    assert T.main(argv, solver_factory=LinearArm) == 0
    text = capsys.readouterr().out
    assert "shoulder_lift +40 ticks" in text and "nothing was moved" in text


def test_cli_refuses_incomplete_input(tmp_path, capsys):
    cal = tmp_path / "cal.json"
    cal.write_text(json.dumps({n: {"range_min": 1000, "range_max": 3000} for n in NAMES}))
    assert T.main(["--model-directory", "x", "--calibration", str(cal), "--ticks", "shoulder_pan=2000"], solver_factory=LinearArm) == 1
    assert "Need ticks" in capsys.readouterr().err
    assert T.main(["--model-directory", "x", "--ticks", "shoulder_pan=2000"], solver_factory=LinearArm) == 1
    assert "raw_calibration_ranges" in capsys.readouterr().err
    with pytest.raises(ValueError):
        T.parse_ticks(["gripper=2000"], "right")


def test_real_so101_model_predictions():
    folder = Path(os.environ.get("CARTON_MODEL_DIR", "data-carton/models/so101")).resolve()
    if not folder.exists():
        pytest.skip("Set CARTON_MODEL_DIR to a verified SO-101 model directory")
    pytest.importorskip("placo")
    verified_model(folder)
    from farm.kinematics.lerobot import LeRobotSO101
    rows = {j["joint"]: j for j in T.predict(dict(zip(NAMES, [2000]*5)), RANGES, LeRobotSO101(folder))["joints"]}
    # Model at all-zero candidate degrees: arm stretched forward along +x.
    assert rows["shoulder_pan"]["gripper_delta_m"][1] < -.01   # +pan swings the gripper to the arm's right (-y)
    assert rows["wrist_flex"]["gripper_delta_m"][2] < -.003    # +wrist_flex pitches the gripper down
    assert np.linalg.norm(rows["wrist_roll"]["gripper_delta_m"]) < .001
    assert all(r["predicted"] for r in rows.values())
