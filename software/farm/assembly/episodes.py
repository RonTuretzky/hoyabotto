"""Episode metadata and the held-out split for R2a demonstrations.

An episode without its metadata is not a demonstration; a split decided after looking at
results is not held out. Both are fixed here, before any training.
"""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

# Section 9C of the handoff. Every one of these must be present (not None / empty) for an episode to enter a dataset.
REQUIRED_META = (
    "episode_id", "session_id", "revision", "mesh_sha256", "printed_instance", "material_profile", "support_cleanup", "manual_modifications",
    "variant", "stage_results", "label", "human_wick_preload", "top_medium_type", "top_sheet_cut_mm", "top_sheet_count", "top_sheet_moisture",
    "trough_instance", "trough_orientation", "robot_id", "calibration_id", "tool_jaw_geometry", "joint_order", "joint_units",
    "station_transform_file", "camera_identities", "images", "schema_version", "code_commit", "split",
)


@dataclass
class EpisodeMeta:
    episode_id: str = ""
    session_id: str = ""                   # one physical setup session: same fixture, same calibration, same day
    revision: str = "R2a"
    mesh_sha256: dict[str, str] = field(default_factory=dict)
    printed_instance: str = ""             # which physical print (e.g. "carrier#1 2026-10-03 plate A")
    material_profile: str = ""
    support_cleanup: str = ""
    manual_modifications: str = ""         # "none" is a valid, explicit answer
    variant: str = ""
    stage_results: dict[str, str] = field(default_factory=dict)
    label: str = ""                        # SUCCESS | INTERVENED | FAILED | RIGID_PRACTICE | INCOMPLETE
    interventions: list[dict[str, Any]] = field(default_factory=list)
    retry_counts: dict[str, int] = field(default_factory=dict)
    failure_reason: str = ""
    human_wick_preload: str = ""           # e.g. "4 folded paper wicks loaded by <who>"
    top_medium_type: str = ""
    top_sheet_cut_mm: list[float] = field(default_factory=list)
    top_sheet_count: int | None = None
    top_sheet_thickness_mm: float | None = None
    top_sheet_moisture: str = ""
    trough_instance: str = ""
    trough_orientation: str = ""
    robot_id: str = ""
    calibration_id: str = ""
    tool_jaw_geometry: str = ""
    joint_order: list[str] = field(default_factory=list)
    joint_units: str = "lerobot_normalized_-100_100_gripper_0_100"
    station_transform_file: str = ""
    camera_identities: dict[str, str] = field(default_factory=dict)
    images: dict[str, str] = field(default_factory=dict)   # pre_grasp, post_lift, seated, released, final -> image hash
    safety_events: list[dict[str, Any]] = field(default_factory=list)
    schema_version: str = ""
    code_commit: str = ""
    split: str = ""                        # train | val | test, assigned BEFORE training
    relabeled: list[dict[str, Any]] = field(default_factory=list)
    started: float = 0.0
    ended: float = 0.0
    notes: str = ""

    def missing(self) -> list[str]:
        out = []
        for k in REQUIRED_META:
            v = getattr(self, k)
            if v is None or v == "" or v == {} or v == []:
                out.append(k)
        return out

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def save(self, dir_: Path) -> Path:
        miss = self.missing()
        if miss:
            raise ValueError("episode metadata incomplete: " + ", ".join(miss))
        dir_ = Path(dir_)
        dir_.mkdir(parents=True, exist_ok=True)
        p = dir_ / f"{self.episode_id}.json"
        p.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True))
        return p

    @classmethod
    def load(cls, path: Path) -> "EpisodeMeta":
        d = json.loads(Path(path).read_text())
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class SplitPlan:
    """Deterministic split by SESSION, decided before training.

    Sessions listed in `holdout_sessions` are test, always. Other sessions go to val with
    `val_percent` probability by a hash of the session id, else train. Splitting by session
    (not by episode) keeps the same fixture/calibration/day out of both sides.
    """
    holdout_sessions: list[str] = field(default_factory=list)
    val_percent: int = 15
    salt: str = "r2a"

    def assign(self, session_id: str) -> str:
        if not session_id:
            raise ValueError("session_id required to assign a split")
        if session_id in self.holdout_sessions:
            return "test"
        h = int(hashlib.sha256(f"{self.salt}:{session_id}".encode()).hexdigest(), 16) % 100
        return "val" if h < self.val_percent else "train"


def write_manifest(dir_: Path, episodes: list[EpisodeMeta], plan: SplitPlan) -> Path:
    """One file listing every episode's split, written before training; a later change is a relabel, not a split."""
    dir_ = Path(dir_)
    dir_.mkdir(parents=True, exist_ok=True)
    rows = []
    for e in episodes:
        s = plan.assign(e.session_id)
        if e.split and e.split != s:
            raise ValueError(f"{e.episode_id}: recorded split {e.split} disagrees with plan {s}")
        rows.append({"episode_id": e.episode_id, "session_id": e.session_id, "split": s, "label": e.label, "variant": e.variant})
    m = {"written": time.strftime("%Y-%m-%dT%H:%M:%S"), "plan": asdict(plan), "episodes": rows,
         "counts": {k: sum(1 for r in rows if r["split"] == k) for k in ("train", "val", "test")}}
    p = dir_ / "split_manifest.json"
    p.write_text(json.dumps(m, indent=2))
    return p


def is_held_out(manifest: dict[str, Any], eval_episode_ids: list[str]) -> tuple[bool, str]:
    """True only if every evaluated episode is in test and no test session appears in train."""
    by_id = {r["episode_id"]: r for r in manifest.get("episodes", [])}
    train_sessions = {r["session_id"] for r in manifest.get("episodes", []) if r["split"] == "train"}
    for eid in eval_episode_ids:
        r = by_id.get(eid)
        if r is None:
            return False, f"{eid} not in manifest"
        if r["split"] != "test":
            return False, f"{eid} is in {r['split']}, not test"
        if r["session_id"] in train_sessions:
            return False, f"session {r['session_id']} also has train episodes"
    return bool(eval_episode_ids), "" if eval_episode_ids else "no episodes"
