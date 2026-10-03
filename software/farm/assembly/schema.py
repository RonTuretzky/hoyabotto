"""Dataset schema for R2a demonstrations, and the fail-closed tick validator.

The system recorder (`System.enable_recording`) records all 14 joints and fills a missing
commanded action with the current state. That is fine for the watering runs it was written
for and wrong for a training set: a padded value looks like a demonstration. For R2a:

  * exactly ONE controlled arm; state and action are the same ordered six joints (".pos" names)
  * every camera the policy will see is a dataset feature; a missing or stale frame rejects the tick
  * the action logged is the command actually sent after the safety clamp, never the current state
  * every tick carries observation.timing = [t_obs, t_action, dt_prev, age_<cam>...] so uneven
    rates and dropped frames are visible in the data, not assumed away
  * a dataset on disk whose features differ is never resumed into
  * a checkpoint whose input features differ is never run against this schema
"""
from __future__ import annotations

import collections
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..learning.recorder import ACTION, IMAGE_PREFIX, OBS_STATE, EpisodeRecorder, dataset_features
from ..status import Reading, Status

SCHEMA_VERSION = "r2a-dataset-1"
TIMING = "observation.timing"


@dataclass
class DatasetSchema:
    controlled_arm: str
    joints: list[str]                       # bare runtime names, e.g. right_arm_shoulder_pan
    cameras: dict[str, str]                 # dataset image key suffix -> farm camera name
    fps: int = 10
    frame_hw: tuple[int, int] = (480, 640)
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.controlled_arm not in ("left", "right"):
            raise ValueError("controlled_arm must be left or right")
        bad = [j for j in self.joints if not j.startswith(f"{self.controlled_arm}_arm_")]
        if bad or not self.joints:
            raise ValueError(f"joints must all belong to the {self.controlled_arm} arm: {bad}")
        if len(set(self.joints)) != len(self.joints):
            raise ValueError("duplicate joint")
        if not self.cameras:
            raise ValueError("at least one camera")

    @property
    def state_names(self) -> list[str]:
        return [j + ".pos" for j in self.joints]

    @property
    def camera_keys(self) -> list[str]:
        return list(self.cameras)

    @property
    def timing_names(self) -> list[str]:
        return ["t_obs", "t_action", "dt_prev"] + [f"age_{k}" for k in self.camera_keys]

    def extra_features(self) -> dict[str, dict[str, Any]]:
        return {TIMING: {"dtype": "float32", "shape": (len(self.timing_names),), "names": self.timing_names}}

    def features(self) -> dict[str, dict[str, Any]]:
        return dataset_features(self.state_names, self.camera_keys, self.frame_hw, self.extra_features())

    def to_dict(self) -> dict[str, Any]:
        return {"schema_version": self.schema_version, "controlled_arm": self.controlled_arm, "joints": list(self.joints),
                "state_names": self.state_names, "cameras": dict(self.cameras), "fps": self.fps, "frame_hw": list(self.frame_hw)}

    # ---- compatibility checks ---------------------------------------------------------
    def problems_with_info(self, info: dict[str, Any]) -> list[str]:
        """Compare against a LeRobotDataset meta/info.json. Any difference means: do not append."""
        out: list[str] = []
        feats = info.get("features") or {}
        want = self.features()
        for key, spec in want.items():
            have = feats.get(key)
            if have is None:
                out.append(f"dataset lacks feature {key}")
                continue
            if tuple(have.get("shape") or ()) != tuple(spec["shape"]):
                out.append(f"{key}: shape {tuple(have.get('shape') or ())} != {tuple(spec['shape'])}")
            if spec.get("names") and list(have.get("names") or []) != list(spec["names"]):
                out.append(f"{key}: names differ")
            if have.get("dtype") != spec["dtype"]:
                out.append(f"{key}: dtype {have.get('dtype')} != {spec['dtype']}")
        extra = [k for k in feats if k not in want and k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")]
        if extra:
            out.append(f"dataset has features this schema does not: {extra}")
        if "fps" in info and int(info["fps"]) != int(self.fps):
            out.append(f"fps {info['fps']} != {self.fps}")
        return out

    def problems_with_checkpoint(self, config: dict[str, Any]) -> list[str]:
        """Compare against a policy config.json (input_features / output_features). Keys are not interchangeable."""
        out: list[str] = []
        inp = config.get("input_features") or {}
        outp = config.get("output_features") or {}
        st = inp.get(OBS_STATE)
        if st is None:
            out.append("checkpoint has no observation.state input")
        elif tuple(st.get("shape") or ()) != (len(self.joints),):
            out.append(f"checkpoint state dim {tuple(st.get('shape') or ())} != ({len(self.joints)},)")
        ck_cams = {k[len(IMAGE_PREFIX):] for k in inp if k.startswith(IMAGE_PREFIX)}
        ours = set(self.camera_keys)
        if ck_cams != ours:
            out.append(f"checkpoint cameras {sorted(ck_cams)} != schema cameras {sorted(ours)} (front/head/overhead are not interchangeable)")
        act = outp.get(ACTION)
        if act is None:
            out.append("checkpoint has no action output")
        elif tuple(act.get("shape") or ()) != (len(self.joints),):
            out.append(f"checkpoint action dim {tuple(act.get('shape') or ())} != ({len(self.joints)},)")
        return out


# ---- fail-closed tick ----------------------------------------------------------------
@dataclass
class TickVerdict:
    ok: bool
    reason: str = ""
    frame: dict[str, Any] | None = None


@dataclass
class StrictTick:
    """Turn raw readings into one dataset frame, or refuse. Nothing is ever padded."""
    schema: DatasetSchema
    step_max: float                  # the farm's per-tick clamp (LimitsCfg.step_deg_max); an action further away was not the clamped command
    watchdog_s: float                # joints older than this are stale (LimitsCfg.watchdog_s)
    frame_max_age_s: float = 1.0
    step_tolerance: float = 0.5
    prev_t: float | None = None
    accepted: int = 0
    rejected: collections.Counter = field(default_factory=collections.Counter)
    gripper_bounds: tuple[float, float] = (0.0, 100.0)
    joint_bounds: tuple[float, float] = (-100.0, 100.0)

    def _reject(self, why: str) -> TickVerdict:
        key = why.split(":")[0]
        self.rejected[key] += 1
        return TickVerdict(False, why)

    def build(self, joints: Reading[dict[str, float]], action_sent: dict[str, float] | None, frames: dict[str, Reading[np.ndarray]],
              now: float, t_action: float | None = None) -> TickVerdict:
        if joints.status is not Status.OK or not isinstance(joints.value, dict):
            return self._reject(f"joints not OK: {joints.status.value} {joints.note}".strip())
        age = now - joints.t
        if age > self.watchdog_s:
            return self._reject(f"joints stale: {age:.2f}s > watchdog {self.watchdog_s}s")
        state = []
        for j in self.schema.joints:
            v = joints.value.get(j)
            if v is None:
                return self._reject(f"joint missing: {j} (no fallback to zero or to another arm)")
            if not math.isfinite(float(v)):
                return self._reject(f"joint non-finite: {j}={v}")
            state.append(float(v))
        if not action_sent:
            return self._reject("action missing: no command was sent this tick (current state is not an action)")
        act = []
        for j, cur in zip(self.schema.joints, state):
            if j not in action_sent:
                return self._reject(f"action missing: {j}")
            a = action_sent[j]
            try:
                a = float(a)
            except (TypeError, ValueError):
                return self._reject(f"action malformed: {j}={a!r}")
            if not math.isfinite(a):
                return self._reject(f"action non-finite: {j}={a}")
            lo, hi = self.gripper_bounds if j.endswith("gripper") else self.joint_bounds
            if a < lo or a > hi:
                return self._reject(f"action out of bounds: {j}={a} not in [{lo}, {hi}]")
            if abs(a - cur) > self.step_max + self.step_tolerance:
                return self._reject(f"action exceeds step clamp: {j} |{a:.1f}-{cur:.1f}| > {self.step_max}; log the clamped command, not the request")
            act.append(a)
        ages = []
        imgs: dict[str, np.ndarray] = {}
        for key, cam in self.schema.cameras.items():
            r = frames.get(cam)
            if r is None:
                return self._reject(f"frame missing: camera {cam} (key {key})")
            if r.status is not Status.OK or r.value is None:
                return self._reject(f"frame not OK: {cam} {r.status.value} {r.note}".strip())
            fa = now - r.t
            if fa > self.frame_max_age_s:
                return self._reject(f"frame stale: {cam} {fa:.2f}s > {self.frame_max_age_s}s")
            arr = np.asarray(r.value)
            if arr.ndim != 3 or arr.shape[2] != 3:
                return self._reject(f"frame malformed: {cam} shape {arr.shape}")
            imgs[key] = arr
            ages.append(fa)
        dt_prev = (now - self.prev_t) if self.prev_t is not None else float("nan")
        self.prev_t = now
        self.accepted += 1
        timing = np.asarray([now, t_action if t_action is not None else now, dt_prev, *ages], dtype=np.float32)
        return TickVerdict(True, "", {
            "joints": {j + ".pos": v for j, v in zip(self.schema.joints, state)},
            "action": {j + ".pos": v for j, v in zip(self.schema.joints, act)},
            "frames": imgs,
            "extra": {TIMING: timing},
        })

    def summary(self) -> dict[str, Any]:
        return {"accepted": self.accepted, "rejected": dict(self.rejected), "rejected_total": sum(self.rejected.values())}


class IncompatibleDataset(RuntimeError):
    pass


class R2aRecorder:
    """EpisodeRecorder bound to a DatasetSchema; refuses to append to a dataset that does not match."""

    def __init__(self, schema: DatasetSchema, root: Path, repo_id: str, robot_type: str = "xlerobot_2wheels"):
        self.schema = schema
        self.root = Path(root)
        info = self.root / "meta" / "info.json"
        if info.exists():
            problems = schema.problems_with_info(json.loads(info.read_text()))
            if problems:
                raise IncompatibleDataset(f"{self.root} does not match {schema.schema_version}: " + "; ".join(problems))
        self.rec = EpisodeRecorder(self.root, repo_id, schema.fps, schema.camera_keys, schema.frame_hw, schema.state_names,
                                   robot_type=robot_type, extra_features=schema.extra_features())
        self._write_schema()

    def _write_schema(self) -> None:
        p = self.root / "meta" / "r2a_schema.json"
        p.parent.mkdir(parents=True, exist_ok=True)
        if not p.exists():
            p.write_text(json.dumps(self.schema.to_dict(), indent=2))

    def start_episode(self, task: str) -> None:
        self.rec.start_episode(task)

    def tick(self, verdict: TickVerdict) -> bool:
        if not verdict.ok or verdict.frame is None:
            return False
        f = verdict.frame
        return self.rec.tick(f["joints"], f["action"], f["frames"], extra=f["extra"])

    def end_episode(self, save: bool) -> int:
        return self.rec.end_episode(save)

    def close(self) -> None:
        self.rec.close()
