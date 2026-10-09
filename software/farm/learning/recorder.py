"""Record the robot's own runs as a Hugging Face LeRobotDataset (lerobot 0.6.1).

Why: every supervised pour is a demonstration. Writing them in the native
LeRobot layout means the same files feed `lerobot-train` later with no
conversion step, and the joint order is pinned by `joint_names` so a policy
trained on them cannot silently swap arms.

Layout choices, verified against the installed 0.6.1 API:
  * `LeRobotDataset.create(repo_id, fps, features, root=..., robot_type=...,
    use_videos=False, image_writer_threads=0)` stores frames as images inside
    the parquet shards, so no ffmpeg / video encoder is needed and creation
    works fully offline (no Hub lookup for a fresh local root).
  * If `root` already holds a dataset, `LeRobotDataset.resume(repo_id, root=...)`
    appends new episodes to it.
  * `add_frame(dict)` needs every declared feature plus a `task` string;
    `save_episode()` writes the buffer; `clear_episode_buffer()` discards it;
    `finalize()` flushes metadata (idempotent).

A tick with a missing camera frame is skipped, never padded: a fabricated
image would teach the policy that blank frames are normal.
"""
from __future__ import annotations

import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")   # local datasets never need the Hub

import logging
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)

OBS_STATE = "observation.state"
ACTION = "action"
IMAGE_PREFIX = "observation.images."


def dataset_features(joint_names: list[str], camera_names: list[str], frame_hw: tuple[int, int],
                     extra: dict[str, dict[str, Any]] | None = None) -> dict[str, dict[str, Any]]:
    """LeRobot feature spec: float32 state/action vectors in joint order, one image feature per camera,
    plus optional extra float32 vector features (e.g. per-tick timing)."""
    h, w = frame_hw
    feats: dict[str, dict[str, Any]] = {
        OBS_STATE: {"dtype": "float32", "shape": (len(joint_names),), "names": list(joint_names)},
        ACTION: {"dtype": "float32", "shape": (len(joint_names),), "names": list(joint_names)},
    }
    for cam in camera_names:
        feats[f"{IMAGE_PREFIX}{cam}"] = {"dtype": "image", "shape": (h, w, 3), "names": ["height", "width", "channels"]}
    for key, spec in (extra or {}).items():
        feats[key] = dict(spec)
    return feats


class EpisodeRecorder:
    def __init__(self, root: Path, repo_id: str, fps: int, camera_names: list[str], frame_hw: tuple[int, int] = (480, 640),
                 joint_names: list[str] | None = None, robot_type: str = "xlerobot_2wheels",
                 extra_features: dict[str, dict[str, Any]] | None = None,
                 image_writer_threads: int = 0):
        if not joint_names:
            raise ValueError("joint_names must list the joints in the order they are recorded")
        self.root = Path(root)
        self.repo_id = repo_id
        self.fps = fps
        self.camera_names = list(camera_names)
        self.frame_hw = (int(frame_hw[0]), int(frame_hw[1]))
        self.joint_names = list(joint_names)
        self.extra_features = dict(extra_features or {})
        self.features = dataset_features(self.joint_names, self.camera_names, self.frame_hw, self.extra_features)
        self._task: str | None = None
        self._frames_in_episode = 0
        self.skipped = 0            # ticks dropped because a frame was missing or malformed
        self.errors = 0             # add_frame failures (logged, never raised into the control loop)
        self.episodes_saved = 0
        self._closed = False

        from lerobot.datasets.lerobot_dataset import LeRobotDataset  # slow import; keep it off module load

        if (self.root / "meta" / "info.json").exists():
            self.ds = LeRobotDataset.resume(repo_id, root=self.root, image_writer_threads=image_writer_threads)
            log.info("resuming dataset %s at %s (%d episodes)", repo_id, self.root, self.ds.meta.total_episodes)
        else:
            self.ds = LeRobotDataset.create(repo_id, fps=fps, features=self.features, root=self.root, robot_type=robot_type,
                                            use_videos=False, image_writer_threads=image_writer_threads)
            log.info("created dataset %s at %s", repo_id, self.root)

    # ---- episode lifecycle --------------------------------------------------------
    @property
    def recording(self) -> bool:
        return self._task is not None

    def start_episode(self, task: str) -> None:
        if self._task is not None:
            log.warning("start_episode while one is open; discarding %d buffered frames", self._frames_in_episode)
            self.ds.clear_episode_buffer()
        self._task = task
        self._frames_in_episode = 0

    def tick(self, joints: dict[str, float], action: dict[str, float], frames: dict[str, np.ndarray],
             extra: dict[str, np.ndarray] | None = None) -> bool:
        """Append one frame. Returns False (and records nothing) if any input is missing."""
        if self._task is None:
            return False
        extra = extra or {}
        if any(k not in extra for k in self.extra_features):
            self.skipped += 1
            log.debug("tick skipped: extra features missing %s", [k for k in self.extra_features if k not in extra])
            return False
        missing = [j for j in self.joint_names if j not in joints or j not in action]
        if missing:
            self.skipped += 1
            log.debug("tick skipped: joints missing %s", missing)
            return False
        frame: dict[str, Any] = {
            OBS_STATE: np.asarray([joints[j] for j in self.joint_names], dtype=np.float32),
            ACTION: np.asarray([action[j] for j in self.joint_names], dtype=np.float32),
            "task": self._task,
        }
        for cam in self.camera_names:
            img = frames.get(cam)
            if img is None:
                self.skipped += 1
                log.debug("tick skipped: no frame for %s", cam)
                return False
            img = self._conform(cam, img)
            if img is None:
                self.skipped += 1
                return False
            frame[f"{IMAGE_PREFIX}{cam}"] = img
        for key in self.extra_features:
            frame[key] = np.asarray(extra[key], dtype=np.float32)
        try:
            self.ds.add_frame(frame)
        except Exception as e:  # noqa: BLE001
            self.errors += 1
            log.warning("add_frame failed: %s", e)
            return False
        self._frames_in_episode += 1
        return True

    def end_episode(self, save: bool) -> int:
        """Close the open episode. Returns the number of frames saved (0 when discarded/empty)."""
        n = self._frames_in_episode
        self._task = None
        self._frames_in_episode = 0
        if not save or n == 0:
            self.ds.clear_episode_buffer()
            return 0
        try:
            self.ds.save_episode()
        except Exception as e:  # noqa: BLE001
            self.errors += 1
            log.error("save_episode failed: %s", e)
            self.ds.clear_episode_buffer()
            return 0
        self.episodes_saved += 1
        return n

    def close(self) -> None:
        if self._closed:
            return
        if self._task is not None:
            self.end_episode(save=False)
        try:
            self.ds.finalize()
        finally:
            self._closed = True

    # ---- helpers ------------------------------------------------------------------
    def _conform(self, cam: str, img: np.ndarray) -> np.ndarray | None:
        h, w = self.frame_hw
        if img.ndim != 3 or img.shape[2] != 3:
            log.warning("frame for %s has shape %s, expected (H, W, 3); skipping tick", cam, img.shape)
            return None
        if img.shape[:2] != (h, w):
            try:
                import cv2
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            except Exception as e:  # noqa: BLE001
                log.warning("frame for %s is %s not %s and resize failed (%s); skipping tick", cam, img.shape[:2], (h, w), e)
                return None
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)
        return np.ascontiguousarray(img)
