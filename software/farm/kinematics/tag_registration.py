"""Fit fixed-camera / gripper-tag registration using OpenCV hand-eye calibration.

For each synchronized stationary observation: B_G @ G_T = B_C @ C_T.
The arm model supplies B_G; camera tag geometry supplies C_T. Neither a single
pose nor rotations about only one axis identify both unknown transforms.
Fitting never installs configuration or enables motion.
"""
from __future__ import annotations

import cv2
import numpy as np
from types import SimpleNamespace

from .lerobot import transform, pose_error, LeRobotSO101
from .assets import verified_model
from farm.perception.tag_geometry import fingerprint


def _average(poses):
    out = np.eye(4)
    u, _, vt = np.linalg.svd(sum(p[:3, :3] for p in poses))
    sign = np.eye(3)
    sign[2, 2] = np.linalg.det(u @ vt)
    out[:3, :3] = u @ sign @ vt
    out[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return transform(out)


def _excitation(poses):
    vectors = [cv2.Rodrigues(poses[i][:3, :3].T @ poses[j][:3, :3])[0].ravel()
               for i in range(len(poses)) for j in range(i)]
    singular = np.linalg.svd(np.asarray(vectors), compute_uv=False)
    if len(singular) < 2 or singular[1] < np.deg2rad(15):
        raise ValueError("Need substantial gripper rotations about at least two independent axes")
    extent = np.ptp([p[:3, 3] for p in poses], axis=0)
    if np.linalg.norm(extent) < .025:
        raise ValueError("Gripper poses span less than 25mm; collect a spatially diverse dataset")
    return {"rotation_excitation_singular_values_rad": singular.tolist(), "position_extent_m": extent.tolist()}


def candidate_degrees(ticks, ranges, names):
    """Reuse the installed LeRobot DEGREES conversion, without opening a bus.

    Its midpoint convention is a candidate to validate, not a measured URDF
    zero. No motion target or replacement motor calibration is produced here.
    """
    from lerobot.motors.motors_bus import MotorNormMode, MotorsBus
    if any(type(ticks[n]) not in (int, float) or not np.isfinite(ticks[n])
           or not ranges[n]["min_ticks"] <= ticks[n] <= ranges[n]["max_ticks"] for n in names):
        raise ValueError("Sample encoder lies outside its recorded calibration range")
    fixture = SimpleNamespace(
        calibration={n: SimpleNamespace(range_min=ranges[n]["min_ticks"], range_max=ranges[n]["max_ticks"], drive_mode=0) for n in names},
        motors={n: SimpleNamespace(norm_mode=MotorNormMode.DEGREES) for n in names},
        model_resolution_table={"sts3215": 4096}, apply_drive_mode=False,
        _id_to_name=lambda i: names[i], _id_to_model=lambda i: "sts3215")
    values = MotorsBus._normalize(fixture, {i: ticks[n] for i, n in enumerate(names)})
    return [values[i] for i in range(len(names))]


def assemble_dataset(captures, model_directory):
    """Attach read-only candidate FK to captured data, retaining provenance."""
    solver = LeRobotSO101(model_directory)
    urdf, manifest = verified_model(model_directory)
    import hashlib
    result = {"schema": 1, "binding": None, "samples": [], "physical_mapping_validated": False}
    reference_ranges = None
    for capture in captures:
        sample = dict(capture["sample"])
        mount = sample.get("gripper_tag_mount") or {}
        if (mount.get("arm") != sample.get("arm") or mount.get("body") != "fixed_gripper_housing"
                or not mount.get("source")):
            raise ValueError("Capture lacks the confirmed matching arm and fixed tag mount")
        status = capture["arm_geometry_status"].get("result", {})
        cfg = status.get("configuration", {}).get("config") or {}
        if cfg.get("mapping") != "feetech_degrees_v1" or cfg.get("arm") != sample["arm"]:
            raise ValueError("Capture must identify the installed feetech_degrees_v1 candidate for this arm")
        assets = status.get("configuration", {}).get("model_assets", {})
        if assets.get("verified") is not True or assets.get("revision") != manifest["revision"]:
            raise ValueError("Captured robot model revision differs from the verified local model")
        ranges = capture["before"]["result"].get("raw_calibration_ranges", {})
        if ranges != capture["after"]["result"].get("raw_calibration_ranges"):
            raise ValueError("Motor calibration ranges changed within the capture")
        names = [f'{sample["arm"]}_arm_{n}' for n in solver.names]
        selected = {n: ranges[n] for n in names}
        if reference_ranges is not None and reference_ranges != selected:
            raise ValueError("Motor calibration ranges changed across captures")
        reference_ranges = selected
        binding = {"arm": sample["arm"], "camera_id": sample["frame"]["camera_id"],
                   "stream_id": sample["frame"]["stream_id"],
                   "camera_calibration_sha256": sample["camera_calibration_sha256"],
                   "tag_geometry_sha256": sample["tag_geometry_sha256"],
                   "robot_model_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                   "motor_calibration_sha256": cfg.get("calibration_sha256"),
                   "encoder_mapping_source": "installed LeRobot MotorsBus._normalize DEGREES; candidate pending physical fit",
                   "gripper_tag_id": sample["gripper_tag_id"], "gripper_tag_mount": mount}
        if result["binding"] is not None and result["binding"] != binding:
            raise ValueError("Capture bindings differ; do not combine calibration sessions")
        result["binding"] = binding
        q = candidate_degrees(sample["joint_ticks"], selected, names)
        sample.update(base_from_gripper=solver.forward(q).tolist(), candidate_model_degrees=q)
        result["samples"].append(sample)
    if not captures:
        raise ValueError("No captures supplied")
    return result


def fit_registration(dataset):
    if not hasattr(cv2, "calibrateHandEye"):
        raise ValueError("This OpenCV build lacks calibrateHandEye; install requirements-tag-geometry.txt")
    if dataset.get("schema") != 1:
        raise ValueError("Expected registration dataset schema 1")
    binding = dataset.get("binding", {})
    required = ("arm", "camera_id", "stream_id", "camera_calibration_sha256", "tag_geometry_sha256",
                "robot_model_sha256", "motor_calibration_sha256", "encoder_mapping_source", "gripper_tag_id")
    if any(not binding.get(k) for k in required) or binding["arm"] not in ("left", "right"):
        raise ValueError("Bind the dataset to one arm, camera, tag, model, mapping and calibration")
    mount = binding.get("gripper_tag_mount") or {}
    if mount.get("arm") != binding["arm"] or mount.get("body") != "fixed_gripper_housing" or not mount.get("source"):
        raise ValueError("Registration requires confirmed arm and fixed gripper-tag mounting")
    samples = dataset.get("samples", [])
    if len(samples) < 11:
        raise ValueError("Need at least eight fitting poses and three held-out validation poses")
    seen, train, validation = set(), [], []
    head_reference, anchor_reference, corners_reference = None, None, None
    for sample in samples:
        if (sample.get("arm") != binding["arm"] or sample.get("gripper_tag_id") != binding["gripper_tag_id"]
                or sample.get("gripper_tag_mount") != mount):
            raise ValueError("Sample arm, tag or mounting differs from the dataset binding")
        identity = sample.get("frame", {})
        key = (identity.get("stream_id"), identity.get("seq"), identity.get("sha256"))
        if (identity.get("camera_id") != binding["camera_id"] or key[0] != binding["stream_id"]
                or type(key[1]) is not int or not isinstance(key[2], str) or len(key[2]) != 64 or key in seen):
            raise ValueError("Each pose needs a distinct source frame from the bound camera stream")
        if sample.get("camera_calibration_sha256") != binding["camera_calibration_sha256"]:
            raise ValueError("Camera calibration changed within registration dataset")
        if sample.get("tag_geometry_sha256") != binding["tag_geometry_sha256"]:
            raise ValueError("Tag geometry changed within registration dataset")
        seen.add(key)
        head = np.asarray(sample.get("head_ticks"), dtype=float)
        anchor = np.asarray(sample.get("anchor_center_camera_mm"), dtype=float)
        corners = np.asarray(sample.get("anchor_corners_px"), dtype=float)
        if head.shape != (2,) or anchor.shape != (3,) or not np.isfinite(np.r_[head, anchor]).all():
            raise ValueError("Need head encoders and a stationary table-anchor observation")
        if head_reference is None:
            head_reference, anchor_reference = head, anchor
            corners_reference = corners
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise ValueError("Need finite decoded table-anchor corners in every registration frame")
        if np.max(np.linalg.norm(corners-corners_reference, axis=1)) > 2:
            raise ValueError("Table-anchor corners moved; camera rotation or mount movement invalidates registration")
        if np.max(np.abs(head-head_reference)) > 3 or np.linalg.norm(anchor-anchor_reference) > 5:
            raise ValueError("Head/camera/table anchor moved; fixed-camera registration invalid")
        if (sample.get("stationary_bracket_verified") is not True
                or sample.get("orientation_ambiguous") is not False):
            raise ValueError("Every pose needs stationary bracket evidence and unambiguous tag orientation")
        b_g, c_t = transform(sample["base_from_gripper"]), transform(sample["camera_from_tag"])
        if sample.get("split") == "train":
            train.append((b_g, c_t))
        elif sample.get("split") == "validation":
            validation.append((b_g, c_t))
        else:
            raise ValueError("Assign each pose to train or validation before fitting")
    if len(train) < 8 or len(validation) < 3:
        raise ValueError("Need at least eight training poses and three held-out validation poses")
    excitation = _excitation([p[0] for p in train])
    for b_g, _ in validation:
        if any(pose_error(b_g, trained[0])[0] < .01 and pose_error(b_g, trained[0])[1] < 5 for trained in train):
            raise ValueError("Held-out poses must differ from fitting poses by at least 10mm or 5 degrees")
    # Eye-to-hand arrangement: invert B_G before calling the standard hand-eye
    # solver. The returned transform is B_C, not its inverse.
    inverse = [np.linalg.inv(p[0]) for p in train]
    candidates = []
    for method in (cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_DANIILIDIS):
        try:
            rotation, translation = cv2.calibrateHandEye(
                [p[:3, :3] for p in inverse], [p[:3, 3] for p in inverse],
                [p[1][:3, :3] for p in train], [p[1][:3, 3] for p in train], method=method)
            b_c = np.eye(4)
            b_c[:3, :3], b_c[:3, 3] = rotation, translation.ravel()
            b_c = transform(b_c)
            g_t = _average([np.linalg.inv(b_g) @ b_c @ c_t for b_g, c_t in train])
            errors = [pose_error(b_g @ g_t, b_c @ c_t) for b_g, c_t in train]
            score = sum(e[0] + np.deg2rad(e[1])*.1 for e in errors)
            candidates.append((score, method, b_c, g_t))
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            continue
    if not candidates:
        raise ValueError("Hand-eye solve failed; dataset may be degenerate or inconsistent")
    _, method, b_c, g_t = min(candidates, key=lambda c: c[0])
    errors = {}
    for name, group in (("train", train), ("validation", validation)):
        values = [pose_error(b_g @ g_t, b_c @ c_t) for b_g, c_t in group]
        errors[name] = {"count": len(group), "position_rms_mm": float(np.sqrt(np.mean([e[0]**2 for e in values])))*1000,
                        "position_max_mm": max(e[0] for e in values)*1000,
                        "orientation_max_degrees": max(e[1] for e in values)}
    passed = all(e["position_rms_mm"] <= 2 and e["position_max_mm"] <= 4
                 and e["orientation_max_degrees"] <= 2 for e in errors.values())
    return {"schema": 1, "status": "REGISTRATION_VALIDATED" if passed else "REGISTRATION_REJECTED",
            "binding": binding, "dataset_sha256": fingerprint(dataset), "method": f"opencv_hand_eye_{method}",
            "base_from_camera": b_c.tolist() if passed else None,
            "gripper_from_tag": g_t.tolist() if passed else None,
            "transform_translation_units": "metres", "residuals": errors, "excitation": excitation,
            "head_ticks_reference": head_reference.tolist(), "anchor_center_camera_mm_reference": anchor_reference.tolist(),
            "anchor_corners_px_reference": corners_reference.tolist(),
            "motion_ready": False, "motor_writes": 0, "physical_grasp_validated": False,
            "remaining": ["Independently verify the jaw contact offset and workspace/path clearance",
                          "Revalidate after camera/head/tag mounting or motor calibration changes",
                          "A good fit validates this mapping over sampled poses, not all joint configurations"]}
