"""Small, explicit contracts shared by the controller and the no-hardware tests."""
from __future__ import annotations

import hashlib
import json
import math
import os
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


class Refused(RuntimeError):
    """An experiment cannot continue with the available evidence."""


def finite(value, name="value"):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise Refused(f"{name}: expected a finite number")
    return float(value)


def vector(value, size=None):
    a = np.asarray(value, dtype=float)
    if a.ndim != 1 or not np.isfinite(a).all() or (size is not None and len(a) != size):
        raise Refused("Invalid numeric vector")
    return a


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def read_json(path):
    try:
        return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    except (OSError, ValueError) as exc:
        raise Refused(f"Cannot read JSON {path}: {exc}") from exc


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with tmp.open("x") as f:
            json.dump(value, f, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)


@dataclass(frozen=True)
class Limits:
    # Ticks, NOT degrees or LeRobot's normalized positions. 4096 ticks/revolution.
    probe_ticks: int = 32
    step_ticks: int = 16
    trust_ticks: int = 96
    settle_ticks: int = 5
    max_steps: int = 80
    max_seconds: float = 180.0
    max_path_ticks: int = 1500
    frame_age_s: float = 1.0
    frame_skew_s: float = 0.3
    status_age_s: float = 0.75
    command_timeout_s: float = 5.0
    temperature_c: float = 55.0
    load_raw: int = 500
    tolerance_px: float = 3.0
    model_error_px: float = 4.0
    condition_max: float = 40.0
    min_response_px: float = 3.0
    return_error_px: float = 3.0

    def __post_init__(self):
        for k, v in asdict(self).items():
            if finite(v, k) <= 0:
                raise Refused(f"{k} must be positive")
        for k in ("probe_ticks", "step_ticks", "trust_ticks", "settle_ticks", "max_steps", "max_path_ticks", "load_raw"):
            if type(getattr(self, k)) is not int:
                raise Refused(f"{k} must be an integer")
        if not self.settle_ticks < self.probe_ticks <= 68 or not self.step_ticks <= 68:
            raise Refused("Probe/step exceeds the existing six-degree command limit")
        if self.trust_ticks > 136 or self.probe_ticks > self.trust_ticks:
            raise Refused("Local experiment envelope exceeds 136 encoder ticks")
        if self.temperature_c > 55 or self.load_raw > 500:
            raise Refused("Cannot relax the connected Mac's existing health limits")
        if self.max_seconds > 300 or self.max_steps > 200 or self.frame_age_s > 2 or self.status_age_s > 2:
            raise Refused("Experiment duration or freshness limit is too permissive")
        if (self.frame_skew_s > .3 or self.settle_ticks > 5 or self.max_path_ticks > 2000
                or self.command_timeout_s > 5 or self.tolerance_px > 4 or self.model_error_px > 6
                or self.return_error_px > 4 or self.condition_max > 50):
            raise Refused("Local control or evidence limit is too permissive")


def validate_config(c):
    if c.get("schema") != 1 or c.get("units") != "encoder_ticks":
        raise Refused("Expected schema 1 and explicit encoder_ticks units")
    arm = c.get("arm")
    if arm not in ("left", "right"):
        raise Refused("Exactly one arm must be selected")
    joints = c.get("joints", [])
    allowed = {f"{arm}_arm_{j}" for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")}
    if not joints or len(joints) != len(set(joints)) or not set(joints) <= allowed:
        raise Refused("Only unique positioning joints of the selected arm are allowed")
    all_joints = allowed | {f"{arm}_arm_gripper"}
    if not all_joints <= set(c.get("ranges", {})):
        raise Refused("Saved ranges for all six selected-arm motors are required")
    for j in all_joints:
        lo, hi = vector(c["ranges"][j], 2)
        if not 0 <= lo < hi <= 4095:
            raise Refused(f"Invalid encoder range for {j}")
    cameras = c.get("cameras", {})
    if set(cameras) != camera_names(c):
        raise Refused("Use the explicitly registered head/wrist or head/OAK camera pair")
    if len({v.get("camera_id") for v in cameras.values()}) != 2:
        raise Refused("Camera identities must be distinct")
    for name, cam in cameras.items():
        if not cam.get("camera_id") or not cam.get("reference") or not cam.get("manifest"):
            raise Refused(f"{name}: camera identity, manifest and seed image are required")
        if not cam.get("regions"):
            raise Refused(f"{name}: identify target/tool regions on its seed image first")
    if not any(r.get("anchor") for r in cameras["head"]["regions"].values()):
        raise Refused("Head view needs a stationary background anchor to detect camera movement")
    measurements = c.get("measurements", [])
    if len(measurements) < len(joints):
        raise Refused("Insufficient visual measurements for the selected joint dimensions")
    if len({m["name"] for m in measurements}) != len(measurements):
        raise Refused("Measurement names must be unique")
    for m in measurements:
        if m.get("axis") not in (0, 1) or type(m["axis"]) is not int:
            raise Refused("Measurement axis is 0 (x) or 1 (y)")
        regions = cameras[m["camera"]]["regions"]
        if m["a"] not in regions or (m.get("b") is not None and m["b"] not in regions):
            raise Refused("Measurement references an unknown tracked region")
    vector(c["target"], len(measurements))
    limits = Limits(**c.get("limits", {}))
    return limits


def camera_names(config):
    pair = config.get('camera_pair', 'head_wrist')
    if pair == 'head_wrist':
        return {'head', f"{config['arm']}_wrist"}
    if pair == 'head_oak':
        return {'head', 'oak'}
    raise Refused('Unknown registered camera pair')


def binding(c):
    """Bind calibration to seeds, camera identities, units, geometry and motor calibration."""
    value = {k: c[k] for k in ("schema", "units", "arm", "joints", "ranges", "measurements", "calibration_sha256")}
    value["cameras"] = {}
    if 'camera_pair' in c:
        value['camera_pair'] = c['camera_pair']
    for name, cam in c["cameras"].items():
        value["cameras"][name] = {
            "camera_id": cam["camera_id"], "regions": cam["regions"],
            "reference_sha256": hashlib.sha256(Path(cam["reference"]).read_bytes()).hexdigest(),
        }
    return digest(value)


@dataclass
class Observation:
    values: np.ndarray
    captured_at: float
    sequences: dict
    points: dict
    streams: dict
    captured_times: dict | None = None


class Trace:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / "trace.jsonl"
        self.file = self.path.open("x", buffering=1)

    def write(self, event, **data):
        self.file.write(json.dumps({"time": time.time(), "event": event, **data}, allow_nan=False) + "\n")

    def close(self):
        self.file.close()
