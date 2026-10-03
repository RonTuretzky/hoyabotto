"""Read-only checks on the robot laptop. Never opens serial ports or cameras."""
from __future__ import annotations

import json
import math

from ..config import load_profile
from ..skills.keyframes import KeyframeStore
from ..tools.calibration_report import analyse, calibration_path
from .rehearsal import required_keyframes


def laptop_report(assembly) -> dict:
    farm = load_profile(assembly.farm_profile)
    problems = []
    if farm.simulated or farm.robot.kind == "sim":
        problems.append("farm profile uses simulated devices")
    if not farm.robot.port1 or not farm.robot.port2 or farm.robot.port1 == farm.robot.port2:
        problems.append("two distinct motor-board ports have not been configured")
    if farm.robot.wheels:
        problems.append("base must remain parked (wheels: false)")
    cameras = {c.name: c for c in farm.cameras}
    for name in assembly.cameras.values():
        camera = cameras.get(name)
        if camera is None or (camera.index_or_path is None and not camera.match):
            problems.append(f"camera {name}: no device binding")
    cal_path = None
    cal_id = ""
    try:
        cal_path = calibration_path(farm.robot)
        if cal_path.is_file():
            problems.extend("calibration: " + p for p in analyse(json.loads(cal_path.read_text()))["problems"])
            cal_id = f"{cal_path.name}:{int(cal_path.stat().st_mtime)}"
        else:
            problems.append(f"calibration: no file at {cal_path}")
    except (ValueError, KeyError, TypeError, OSError) as exc:
        problems.append(f"calibration unreadable: {exc}")
    # Isolated file: never use bottle/watering poses for the planter.
    keyframes_path = farm.data_path / "r2a" / "keyframes.yaml"
    keyframes = KeyframeStore(keyframes_path)
    required = required_keyframes(assembly.variant)
    missing = []
    for name in required:
        joints = keyframes.get(name)
        if joints is None:
            missing.append(name)
            continue
        if set(joints) != set(assembly.state_joints):
            problems.append(f"keyframe {name}: must contain exactly the controlled arm's six joints")
        for joint, value in joints.items():
            lo = 0 if joint.endswith("gripper") else -100
            if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= 100:
                problems.append(f"keyframe {name}: invalid target for {joint}")
    if missing:
        problems.append(f"{len(missing)} R2a keyframes not taught")
    return {"scope": "files only; does not verify hardware", "farm_profile": farm.name,
            "calibration_file": str(cal_path) if cal_path else None, "calibration_id": cal_id,
            "keyframes_file": str(keyframes_path), "missing_keyframes": missing, "problems": problems}
