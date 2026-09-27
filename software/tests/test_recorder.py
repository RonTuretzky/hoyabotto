"""EpisodeRecorder writes a real LeRobotDataset (image mode, offline) that reopens."""
import numpy as np
import pytest

pytest.importorskip("lerobot.datasets.lerobot_dataset")

from farm.learning.recorder import EpisodeRecorder  # noqa: E402

JOINTS = ["right_arm_shoulder_pan", "right_arm_wrist_flex", "right_arm_gripper"]
CAMS = ["head", "wrist"]
HW = (48, 64)


def _frames(rng, cams=CAMS):
    return {c: rng.integers(0, 255, (HW[0], HW[1], 3), dtype=np.uint8) for c in cams}


def _joints(rng):
    return {j: float(rng.uniform(-100, 100)) for j in JOINTS}


def test_records_two_episodes_and_reopens(tmp_path):
    rng = np.random.default_rng(0)
    root = tmp_path / "ds"
    rec = EpisodeRecorder(root, "farm/test_pours", fps=10, camera_names=CAMS, frame_hw=HW, joint_names=JOINTS)
    lengths = [5, 7]
    for n in lengths:
        rec.start_episode("pour 30 ml into tray B")
        for _ in range(n):
            assert rec.tick(_joints(rng), _joints(rng), _frames(rng))
        assert rec.end_episode(save=True) == n
    # a missing camera frame is skipped, never fabricated
    rec.start_episode("discarded")
    assert rec.tick(_joints(rng), _joints(rng), _frames(rng, ["head"])) is False
    assert rec.skipped == 1
    assert rec.tick(_joints(rng), _joints(rng), _frames(rng))
    assert rec.end_episode(save=False) == 0
    rec.close()
    assert rec.episodes_saved == 2

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("farm/test_pours", root=root)
    assert ds.num_episodes == 2
    assert ds.num_frames == sum(lengths)
    assert ds.meta.features["observation.state"]["names"] == JOINTS
    assert ds.meta.features["action"]["shape"] == (len(JOINTS),)
    for c in CAMS:
        assert ds.meta.features[f"observation.images.{c}"]["dtype"] == "image"
        assert tuple(ds.meta.features[f"observation.images.{c}"]["shape"]) == (HW[0], HW[1], 3)
    item = ds[0]
    assert item["observation.state"].shape == (len(JOINTS),)
    assert item["task"] == "pour 30 ml into tray B"


def test_resume_appends_to_existing_dataset(tmp_path):
    rng = np.random.default_rng(1)
    root = tmp_path / "ds"
    for _ in range(2):
        rec = EpisodeRecorder(root, "farm/test_resume", fps=10, camera_names=CAMS, frame_hw=HW, joint_names=JOINTS)
        rec.start_episode("t")
        for _ in range(3):
            rec.tick(_joints(rng), _joints(rng), _frames(rng))
        rec.end_episode(save=True)
        rec.close()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("farm/test_resume", root=root)
    assert ds.num_episodes == 2 and ds.num_frames == 6
