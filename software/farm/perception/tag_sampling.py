"""Read-only, stationary camera/encoder samples for geometric commissioning."""
from __future__ import annotations

import math

import numpy as np

from farm.perception.tag_geometry import fingerprint

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
HEAD_JOINTS = ("head_motor_1", "head_motor_2")


def stationary_sample(before, after, observation, arm):
    if arm not in ("left", "right"):
        raise ValueError("Choose one explicit arm")
    names = [f"{arm}_arm_{n}" for n in ARM_JOINTS] + list(HEAD_JOINTS)
    samples = []
    for payload in (before, after):
        if payload.get("ok") is not True or payload.get("result", {}).get("cached") is not False:
            raise ValueError("Need fresh successful owner encoder reads bracketing the camera")
        rows = {r["name"]: r for r in payload["result"]["motors"]}
        if any(n not in rows for n in names):
            raise ValueError("Missing arm/head encoders")
        for name in names:
            row = rows[name]
            if (row.get("Status") != 0 or row.get("Moving") != 0 or abs(row.get("Present_Velocity", math.inf)) > 1
                    or type(row.get("Present_Position")) is not int
                    or not math.isfinite(row.get("captured_at", math.nan))):
                raise ValueError(f"Joint is moving, faulty, or missing coherent telemetry: {name}")
        samples.append(rows)
    first, last = samples
    span = {n: abs(last[n]["Present_Position"]-first[n]["Present_Position"]) for n in names}
    if max(span.values()) > 3:
        raise ValueError("Arm/head moved by more than three ticks during the capture bracket")
    lo, hi = max(first[n]["captured_at"] for n in names), min(last[n]["captured_at"] for n in names)
    stamp = observation.get("frame", {}).get("captured_at")
    if (type(stamp) not in (float, int) or not math.isfinite(stamp)
            or not lo <= stamp <= hi or not 0 < hi-lo <= 3):
        raise ValueError("RGB capture is not between coherent owner samples within three seconds")
    geometry = observation.get("pose_3d") or {}
    tags = {t["tag_id"]: t for t in geometry.get("tags", []) if t.get("center_camera_mm") is not None}
    if geometry.get("status") != "CAMERA_RELATIVE_ESTIMATE" or 1 not in tags or 2 not in tags:
        raise ValueError("Need metric table and gripper tags in the same frame")
    mount = tags[2].get("mount") or {}
    if (mount.get("arm") != arm or mount.get("body") != "fixed_gripper_housing"
            or not mount.get("source")):
        raise ValueError("Tag 2 must be explicitly confirmed on the selected arm's fixed gripper housing")
    if tags[2].get("orientation_ambiguous") or tags[2].get("camera_from_tag") is None:
        raise ValueError("Gripper tag orientation is ambiguous; choose a more informative view")
    ticks = {n: (first[n]["Present_Position"]+last[n]["Present_Position"])/2 for n in names}
    anchor_pixels = next((t.get("corners_px") for t in observation.get("tags", [])
                          if t.get("tag_id") == 1 and t.get("status") == "DETECTED"), None)
    corners = np.asarray(anchor_pixels, dtype=float)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError("Need decoded table-anchor corners to detect camera rotation or mount movement")
    frame = observation["frame"]
    return {"sample_id": fingerprint(frame)[:24], "arm": arm, "frame": frame,
            "camera_calibration_sha256": geometry["calibration_sha256"],
            "tag_geometry_sha256": geometry["geometry_config_sha256"],
            "camera_from_tag": tags[2]["camera_from_tag"], "gripper_tag_id": 2,
            "gripper_tag_mount": dict(mount),
            "anchor_center_camera_mm": tags[1]["center_camera_mm"],
            "anchor_corners_px": corners.tolist(),
            "head_ticks": [ticks[n] for n in HEAD_JOINTS], "joint_ticks": ticks,
            "stationary_bracket_verified": True, "capture_bracket_s": [lo, hi],
            "encoder_span_ticks": span, "orientation_ambiguous": False,
            "reprojection_rms_px": tags[2]["reprojection_rms_px"],
            "base_from_gripper": None, "split": None,
            "motor_writes": 0, "physical_mapping_validated": False}
