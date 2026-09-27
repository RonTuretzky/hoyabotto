"""AprilTag identity, policy skill under the safety envelope, backup restore test, photo-replay camera."""
import sys
from pathlib import Path

import numpy as np

from farm.adapters.sim import Faults, FakeCamera
from farm.status import Reading, Status

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "parts"))


def test_apriltag_render_and_detect_confirms_tray():
    from make_parts import TAG36H11
    from farm.perception.tags import confirm_tray, detect_tags, render_tag
    grid = [row.replace("#", "1").replace(".", "0") for row in TAG36H11[1]]
    img = render_tag(grid, cell_px=30)
    canvas = np.full((480, 640, 3), 200, np.uint8); canvas[100:100 + img.shape[0], 200:200 + img.shape[1]] = img
    frame = Reading(canvas, Status.OK)
    tags = detect_tags(frame)
    assert tags.ok and 1 in tags.value, tags
    assert confirm_tray(frame, 1).value is True
    wrong = confirm_tray(frame, 2)
    assert wrong.value is False and "expected 2" in wrong.note
    assert confirm_tray(Reading(np.full((480, 640, 3), 200, np.uint8), Status.OK), 1).status is Status.UNKNOWN
    assert confirm_tray(frame, None).status is Status.NOT_APPLICABLE


def test_policy_skill_runs_inside_safety_envelope(sim):
    from farm.adapters.base import ARM_JOINTS, arm_joint
    from farm.skills.policy import PolicySkill, ScriptedPolicy
    s = sim()
    joints = [arm_joint("right", j) for j in ARM_JOINTS]
    # the policy asks for a 60-unit jump on wrist_flex: every tick must be clamped to step_deg_max (6)
    pol = ScriptedPolicy([[0, 0, 0, 60, 0, 40]] * 50)
    reached = {"n": 0}

    def done(cur):
        return cur[arm_joint("right", "wrist_flex")] > 30
    skill = PolicySkill(s.skills, s.cameras, pol, joints, {"observation.images.wrist": "right_wrist", "observation.images.head": "head"}, hz=30, max_steps=60, done_check=done)
    out = skill.run("reach")
    assert out.ok, out.reason
    assert out.steps >= 5                                  # 30/6 clamped steps at least
    assert pol.calls[0][1] == ["observation.images.head", "observation.images.wrist"]
    assert all(abs(a - b) <= 6.0 + 1e-6 for a, b in zip(s.robot.sent[-1].values(), s.robot.sent[-1].values()))


def test_policy_skill_stops_on_stall_and_bad_output(sim):
    from farm.adapters.base import ARM_JOINTS, arm_joint
    from farm.skills.policy import PolicySkill, ScriptedPolicy
    s = sim()
    joints = [arm_joint("right", j) for j in ARM_JOINTS]
    still = PolicySkill(s.skills, s.cameras, ScriptedPolicy([[0] * 6]), joints, {"observation.images.wrist": "right_wrist"}, hz=60, max_steps=100, stall_ticks=8)
    out = still.run()
    assert not out.ok and "stalled" in out.reason
    bad = PolicySkill(s.skills, s.cameras, ScriptedPolicy([[float("nan")] * 6]), joints, {"observation.images.wrist": "right_wrist"}, hz=60, max_steps=5)
    assert "non-finite" in bad.run().reason


def test_backup_verify(tmp_path):
    from farm.evidence.store import EvidenceStore
    st = EvidenceStore(tmp_path / "d")
    cid = st.start_cycle("B", "sim", "h", "cal", True); st.event(cid, "x", {})
    p = st.backup(tmp_path / "bk")
    r = st.verify_backup(p)
    assert r["ok"] and r["cycles"] == 1 and r["events"] >= 1


def test_fake_camera_replays_photos(tmp_path):
    import cv2
    f = Faults()
    for i, col in enumerate(((255, 0, 0), (0, 255, 0))):
        cv2.imwrite(str(tmp_path / f"p{i}.jpg"), np.full((100, 100, 3), col[::-1], np.uint8))
    f.set("replay_dir", str(tmp_path))
    cam = FakeCamera("head", f); cam.connect()
    a = cam.frame().value; b = cam.frame().value
    assert a.shape == (480, 640, 3) and a[0, 0].tolist() != b[0, 0].tolist()
