"""Camera-relative AprilTag geometry from matched RGB calibration and tag sizes.

This module has no robot I/O. A metric tag centre is not a jaw contact point or
a robot-frame target. Square-marker pose ambiguity is reported explicitly.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


# apriltag-geometry.json sections consumed elsewhere (farm/perception/paddle_target.py).
NON_MEASUREMENT_SECTIONS = ("paddle_grasp",)


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def square_points(size_m):
    """IPPE square order, paired with the detector's decoded corner order."""
    s = size_m / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)


def camera_calibration(meta, image, shape):
    """Use only calibration carried by the exact, hash-checked RGB frame."""
    for key in ("camera_id", "seq", "stream_id", "sha256"):
        if meta.get(key) is None or meta.get(key) != image.get(key):
            raise ValueError(f"Metric calibration is not bound to this RGB frame: {key}")
    h, w = shape[:2]
    if meta.get("width") != w or meta.get("height") != h:
        raise ValueError("Metric image dimensions do not match calibration metadata")
    k = np.asarray(meta.get("intrinsics"), dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0
            or not 0 <= k[0, 2] < w or not 0 <= k[1, 2] < h):
        raise ValueError("Missing or invalid matching camera intrinsics")
    projection = meta.get("projection")
    if image.get("projection") != projection:
        raise ValueError("RGB and calibration projection differ")
    if projection == "rectified_pinhole":
        # The provider retains the original factory D for provenance. Applying
        # it to its already-undistorted pixels would distort the pose twice.
        distortion = np.zeros(5)
    elif projection == "camera_pinhole_with_factory_distortion":
        distortion = np.asarray(meta.get("distortion_coefficients"), dtype=float)
        if distortion.shape not in ((4,), (5,), (8,), (12,), (14,)) or not np.isfinite(distortion).all():
            raise ValueError("Raw RGB requires its matching OpenCV distortion coefficients")
    else:
        raise ValueError("Unknown camera projection; cannot interpret metric pose")
    coordinate_frame = meta.get("coordinate_frame")
    if not isinstance(coordinate_frame, str) or not coordinate_frame:
        raise ValueError("Missing camera optical coordinate frame")
    binding = {"camera_id": meta["camera_id"], "image_size_px": [w, h], "intrinsics": k.tolist(),
               "projection": projection, "effective_distortion": distortion.tolist(),
               "coordinate_frame": coordinate_frame, "lens_position": meta.get("calibrated_lens_position")}
    return k, distortion, binding


def estimate_square(corners, size_mm, k, distortion):
    points = np.asarray(corners, dtype=float)
    if points.shape != (4, 2) or not np.isfinite(points).all() or not cv2.isContourConvex(points.astype(np.float32)):
        raise ValueError("Expected four finite convex tag corners in decoded order")
    obj = square_points(size_mm / 1000)
    count, rotations, translations, _ = cv2.solvePnPGeneric(
        obj, points, k, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rotation, translation in zip(rotations, translations):
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            continue
        r, _ = cv2.Rodrigues(rotation)
        if np.min((obj @ r.T + translation.reshape(3))[:, 2]) <= .02:
            continue
        projected, jacobian = cv2.projectPoints(obj, rotation, translation, k, distortion)
        error = float(np.sqrt(np.mean(np.sum((projected[:, 0] - points)**2, axis=1))))
        # Local sensitivity under an explicit 0.5px corner-noise assumption.
        # This does not include print-size, intrinsics or mounting errors.
        j = jacobian[:, :6]
        covariance = np.linalg.pinv(j.T @ j) * .5**2
        std_mm = np.sqrt(np.maximum(0, np.diag(covariance)[3:6])) * 1000
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = r, translation.reshape(3)
        candidates.append({"pose": pose, "error": error, "std_mm": std_mm})
    if not count or not candidates:
        raise ValueError("No finite square pose in front of the camera")
    candidates.sort(key=lambda c: c["error"])
    best = candidates[0]
    if best["error"] > 1.5:
        raise ValueError(f"Square reprojection error {best['error']:.3f}px exceeds 1.5px")
    ambiguous, spread = False, 0.0
    for candidate in candidates[1:]:
        if candidate["error"] <= best["error"] + .5:
            cosine = (np.trace(best["pose"][:3, :3].T @ candidate["pose"][:3, :3]) - 1) / 2
            angle = math.degrees(math.acos(float(np.clip(cosine, -1, 1))))
            ambiguous |= angle > 5
            spread = max(spread, float(np.linalg.norm(best["pose"][:3, 3] - candidate["pose"][:3, 3])) * 1000)
    if spread > 5:
        raise ValueError("Ambiguous square solutions disagree in position by more than 5mm")
    return {"status": "POSITION_ONLY_AMBIGUOUS_ORIENTATION" if ambiguous else "POSE_ESTIMATED",
            "center_camera_mm": (best["pose"][:3, 3] * 1000).tolist(),
            "camera_from_tag": None if ambiguous else best["pose"].tolist(),
            "transform_translation_units": "metres", "orientation_ambiguous": ambiguous,
            "reprojection_rms_px": best["error"],
            "alternative_position_spread_mm": spread,
            "position_std_mm_at_assumed_half_pixel_noise": best["std_mm"].tolist(),
            "coarse_position_only": bool(np.max(best["std_mm"]) > 2),
            "precision_note": "Local sensitivity estimate, not a measured accuracy guarantee or grasp clearance",
            "uncertainty_excludes": ["print_size", "camera_calibration", "paper_warp", "mounting"],
            "candidate_reprojection_rms_px": [c["error"] for c in candidates]}


class TagGeometry:
    def __init__(self, config):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text())
        if config.get("schema") != 1 or config.get("family") != "tag36h11":
            raise ValueError("Expected schema 1 tag36h11 geometry configuration")
        self.config = json.loads(json.dumps(config, allow_nan=False))
        self.sizes = self.config.get("tags", {})
        if not self.sizes:
            raise ValueError("Explicit tag sizes and their measurement sources are required")
        for key, spec in self.sizes.items():
            size = spec.get("black_square_mm")
            if (not key.isdigit() or not 0 <= int(key) <= 586 or type(size) not in (float, int)
                    or not math.isfinite(size) or not 5 <= size <= 300
                    or not isinstance(spec.get("source"), str) or not spec["source"].strip()):
                raise ValueError("Each tag needs an ID, 5–300mm black-square width and explicit source")
            if spec.get("mount") is not None:
                mount = spec["mount"]
                if (not isinstance(mount, dict) or mount.get("arm") not in ("left", "right")
                        or mount.get("body") not in ("fixed_gripper_housing", "moving_jaw")
                        or not isinstance(mount.get("source"), str) or not mount["source"].strip()):
                    raise ValueError("A gripper mount needs an explicit arm, body and confirmation source")
        if not isinstance(config.get("camera_ids"), list) or not config["camera_ids"] or any(
                not isinstance(c, str) or not c for c in config["camera_ids"]):
            raise ValueError("Bind metric geometry to explicit camera identities")
        # Owner-measured grasp offsets do not change any tag measurement; keep
        # them out of the fingerprint so measuring them does not invalidate a
        # registration bound to tag_geometry_sha256.
        self.fingerprint = fingerprint({k: v for k, v in self.config.items() if k not in NON_MEASUREMENT_SECTIONS})

    def measure(self, tags, meta, image, shape):
        out = {"status": "UNAVAILABLE", "coordinate_frame": None, "units": "mm",
               "method": "opencv_ippe_square", "depth_used": False,
               "robot_frame_calibrated": False, "contact_offset_calibrated": False,
               "physical_accuracy_validated": False, "geometry_config_sha256": self.fingerprint,
               "tags": [], "gripper_to_paddle_mm": None}
        try:
            if image.get("camera_id") not in self.config["camera_ids"]:
                raise ValueError("No metric geometry configured for this camera identity")
            k, distortion, binding = camera_calibration(meta, image, shape)
            if meta.get("rgb_captured_at", image.get("captured_at")) is None:
                raise ValueError("Receipt-only image is unsuitable for metric calibration sampling")
            out.update(coordinate_frame=binding["coordinate_frame"], camera_calibration=binding,
                       calibration_sha256=fingerprint(binding), axes="+x camera right, +y camera down, +z forward")
            accepted = {}
            for tag in tags:
                spec = self.sizes.get(str(tag["tag_id"]))
                if tag["status"] != "DETECTED" or spec is None:
                    continue
                entry = {"tag_id": tag["tag_id"], "black_square_mm": spec["black_square_mm"],
                         "size_source": spec["source"], "mount": spec.get("mount")}
                try:
                    entry.update(estimate_square(tag["corners_px"], spec["black_square_mm"], k, distortion))
                    accepted[tag["tag_id"]] = entry
                except (ValueError, cv2.error) as exc:
                    entry.update(status="REJECTED", reason=str(exc))
                out["tags"].append(entry)
            out["status"] = "CAMERA_RELATIVE_ESTIMATE" if accepted else "NO_VALID_METRIC_TAGS"
            if 2 in accepted and 3 in accepted:
                delta = np.array(accepted[3]["center_camera_mm"]) - accepted[2]["center_camera_mm"]
                out["gripper_to_paddle_mm"] = {"from_tag": 2, "to_tag": 3,
                    "delta_camera_mm": delta.tolist(), "tag_center_distance_mm": float(np.linalg.norm(delta)),
                    "contact_offset_calibrated": False, "robot_motion_target": False}
        except (ValueError, TypeError, KeyError, cv2.error) as exc:
            out["reason"] = str(exc)
        return out
