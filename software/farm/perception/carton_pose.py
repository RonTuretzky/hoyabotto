"""Carton pose in the RIGHT arm's base frame and the pilot's model frame, from carton AprilTags.

Read-only. Inputs are one fresh ``robot_get_tags`` observation of the OAK (metric ``pose_3d`` for the
carton IDs, which needs their sizes in apriltag-geometry.json ``carton_tags``) and the installed,
validated camera-to-arm registration (``tag-registration.json``: ``base_from_camera`` for the right
arm, valid only at ``head_ticks_reference``). Nothing here moves or enables motors.

Frames
- camera: ``CAM_A_optical`` (+x right, +y down, +z forward), mm from the tag solver.
- arm base: SO-101 URDF ``base_link`` of the RIGHT arm (+x forward, +y the arm's left, +z up), mm.
- model: the pilot's reach frame (``farm/sim/xlerobot_twin.py`` FRAME): origin on the floor below the
  midpoint between the two shoulder-pan axes, +forward, +left, +up, reported in cm. The right arm's
  base_link origin sits at (forward -38.8, left -136.5, up 777.4) mm in it: the URDF pan axis is at
  base x 0.0388 and the shoulder-lift axis 0.1166 m above base z=0, while the twin puts that pan axis
  at forward 0 / left -0.1365 and the lift axis 0.894 m above the floor (``farm/kinematics/xlerobot_geometry``).
  Assumed: base +x is the robot's forward, i.e. the pan midpoint tick points the arm straight ahead
  (the same ``feetech_degrees_v1`` candidate the reach solver and the registration use).

Carton (HACHIYO 379 x 283 x 108 mm, 140 mm flaps), print plan 2026-10-09, printed at 100 %: wall and
floor tags 45 mm, flap tags 35 mm black squares; wall tags at half wall height (54 mm below the rim);
26/27 at +-120 mm from the near-wall centre tag 10; flap tags 90 mm above the hinge and 70 mm toward
the far side along it. The near wall (10/26/27) faces the robot. ``mirrored=True`` (this carton): the
print plan's "left" panels (21/28, flap 11) are on the robot's RIGHT, 22 and flap 12 on its left.

Fit: the near wall is taken as a VERTICAL plane through the horizontal line fitted to the near-wall tag
centres (the three tags are collinear, so their centres alone cannot give the plane's tilt). The rim is
the tag height plus 54 mm; the right wall's top line (hinge of the right short flap) starts 379/2 mm to
the robot's right of the near-wall centre at rim height and runs 283 mm away from the robot.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from farm.kinematics.lerobot import transform
from farm.kinematics.xlerobot_geometry import RIGHT_BASE_IN_MODEL_M as _RIGHT_BASE_IN_MODEL_M
from farm.perception.carton_tags import CARTON_ROLES

TOOL_NAME = "robot_get_carton_pose"
CARTON_MM = {"long": 379.0, "short": 283.0, "height": 108.0, "flap": 140.0}
HALF_WALL_MM = CARTON_MM["height"] / 2
WALL_TAG_OFFSET_MM = 120.0       # 26/27 from the near-wall centre (tag 10) along the wall
FLAP_TAG_UP_MM = 90.0            # flap tag centre above the hinge when the flap stands vertical
FLAP_TAG_ALONG_MM = 70.0         # flap tag centre toward the far side from the flap centre, along the hinge
HEAD_TICK_LIMIT = 3
FRAME_MAX_AGE_S = 1.0
NEAR_WALL_IDS = (10, 26, 27)
CARTON_IDS = tuple(CARTON_ROLES)
PRINT_SIZES_MM = {10: 45, 26: 45, 27: 45, 21: 45, 28: 45, 22: 45, 24: 45, 25: 45, 11: 35, 12: 35, 13: 35, 14: 35}
SIZE_SOURCE = ("HACHIYO 12-tag print plan 2026-10-09 (software/docs/carton-bimanual-rgbd-simulation.md, "
               "carton-real-station-measurements.md); owner printed the sheet at 100 %, no ruler measurement")
# Right arm base_link origin in the model frame, metres (see the module docstring).
RIGHT_BASE_IN_MODEL_M = np.array(_RIGHT_BASE_IN_MODEL_M)
MODEL_UP, MODEL_LEFT, MODEL_FORWARD = np.array([0., 0., 1.]), np.array([0., 1., 0.]), np.array([1., 0., 0.])
ASSUMPTIONS = (
    "carton tag black squares are the print plan's 45 mm (walls/floor) and 35 mm (flaps): sheet printed at 100 %, not ruler-measured",
    "wall tags 10/26/27 sit at half wall height (54 mm below the rim) and 26/27 are 120 mm either side of 10",
    "the near wall is vertical; its plane is the vertical plane through the line of the near-wall tag centres",
    "right arm base +x is the robot's forward (pan midpoint = straight ahead), base origin at forward -3.9 / left -13.7 / up 77.7 cm (273 mm arm spacing)",
    "the registration is only valid with the head at its reference ticks; moving the head or the OAK invalidates every number",
    "carton outer dimensions 379 x 283 x 108 mm, flaps 140 mm (cardboard thickness ignored)",
)


def model_from_base_mm(point_base_mm):
    """Right-arm base (mm) -> model frame (metres, forward/left/up)."""
    p = np.asarray(point_base_mm, dtype=float).reshape(3) / 1000.0
    return p + RIGHT_BASE_IN_MODEL_M


def base_mm_from_model(point_model_m):
    p = np.asarray(point_model_m, dtype=float).reshape(3)
    return (p - RIGHT_BASE_IN_MODEL_M) * 1000.0


def _point(point_base_mm):
    m = model_from_base_mm(point_base_mm)
    return {"arm_base_mm": [round(float(v), 1) for v in np.asarray(point_base_mm, float)],
            "model_cm": {"forward_cm": round(float(m[0]) * 100, 1), "left_cm": round(float(m[1]) * 100, 1),
                         "up_cm": round(float(m[2]) * 100, 1)}}


def carton_tag_sizes_section():
    """The ``carton_tags`` section for apriltag-geometry.json (sizes only; no mounts)."""
    return {str(i): {"black_square_mm": PRINT_SIZES_MM[i], "role": CARTON_ROLES[i], "source": SIZE_SOURCE}
            for i in CARTON_IDS}


def install_carton_tag_sizes(geometry_path, *, backup_dir=None, clock=time.time):
    """Add the carton sizes to the geometry file as a separate section (fingerprint-neutral).

    Returns {'changed', 'backup', 'fingerprint_before', 'fingerprint_after'}. Existing entries are kept as
    they are; registration tags 1/2/3 are never touched. Backs up the file before writing.
    """
    from farm.perception.tag_geometry import TagGeometry
    path = Path(geometry_path)
    original = path.read_text()
    config = json.loads(original)
    before = TagGeometry(config).fingerprint
    section = dict(config.get("carton_tags") or {})
    changed = False
    for key, spec in carton_tag_sizes_section().items():
        if key in (config.get("tags") or {}):
            continue  # already a registration/measurement tag; leave the fingerprinted section alone
        if key not in section:
            section[key] = spec
            changed = True
    if not changed:
        return {"changed": False, "backup": None, "fingerprint_before": before, "fingerprint_after": before}
    updated = dict(config)
    updated["carton_tags"] = section
    after = TagGeometry(updated).fingerprint
    if after != before:
        raise ValueError("carton sizes must not change the registration fingerprint")
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(clock()))
    backup = (Path(backup_dir) if backup_dir else path.parent) / f"apriltag-geometry.before-carton-{stamp}.json"
    backup.parent.mkdir(parents=True, exist_ok=True)
    backup.write_text(original)
    temp = path.with_suffix(".carton.tmp")
    temp.write_text(json.dumps(updated, indent=2) + "\n")
    temp.replace(path)
    return {"changed": True, "backup": str(backup), "fingerprint_before": before, "fingerprint_after": after}


def _head_ticks(state_payload):
    rows = {r.get("name"): r for r in ((state_payload or {}).get("result") or {}).get("motors") or []}
    try:
        return [float(rows[n]["Present_Position"]) for n in ("head_motor_1", "head_motor_2")]
    except (KeyError, TypeError, ValueError):
        return None


def _check_head(head, registration):
    if head is None:
        raise ValueError("Head ticks were not read; the registration is only valid at its reference head pose")
    current = np.asarray(head, float)
    reference = np.asarray(registration.get("head_ticks_reference"), float)
    if current.shape != (2,) or reference.shape != (2,) or not np.isfinite([current, reference]).all():
        raise ValueError("Need two finite head ticks and registration reference ticks")
    shift = float(np.max(np.abs(current - reference)))
    if shift > HEAD_TICK_LIMIT:
        raise ValueError(f"Head moved since registration: {head} vs {reference.tolist()} "
                         f"({shift:.0f} > {HEAD_TICK_LIMIT} ticks); re-establish the registered head pose or re-register")


def _head_sample(payload, registration, now):
    if payload.get("ok") is not True:
        raise ValueError("Head state read failed")
    head = _head_ticks(payload)
    _check_head(head, registration)
    rows = {r.get("name"): r for r in payload.get("result", {}).get("motors", [])}
    times = []
    for name in ("head_motor_1", "head_motor_2"):
        row = rows[name]
        stamp = row.get("captured_at")
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or not 0 <= now-stamp <= FRAME_MAX_AGE_S:
            raise ValueError("Head telemetry is stale, future-dated or missing a capture time")
        if row.get("Moving") != 0 or row.get("Present_Velocity") != 0:
            raise ValueError("Head is moving or lacks stationary telemetry")
        times.append(stamp)
    return head, times


def _unit(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("Degenerate direction")
    return np.asarray(v, float) / n


def _fit_horizontal_line(points):
    """Horizontal unit direction (base frame) along the near wall and the points' mean."""
    pts = np.asarray(points, float)
    mean = pts.mean(axis=0)
    if len(pts) == 1:
        raise ValueError("Need at least two near-wall tags (10/26/27) to orient the carton")
    horizontal = (pts - mean)[:, :2]
    _, s, vt = np.linalg.svd(horizontal, full_matrices=False)
    if len(pts) == 2 and np.linalg.norm(horizontal[0] - horizontal[1]) < 100:
        raise ValueError("Near-wall tags are too close together to orient the carton")
    direction = np.array([vt[0][0], vt[0][1], 0.0])
    if len(pts) >= 3 and s[1] > 15:  # tags should be collinear within ~15 mm
        raise ValueError(f"Near-wall tag centres are not on one line (spread {s[1]:.1f} mm); check the mounting or the sizes")
    return _unit(direction), mean


def carton_pose_from_observation(row, registration, *, head_ticks=None, mirrored=True, rim_reference_m=None):
    """One OAK ``robot_get_tags`` observation row + registration -> carton tag centres and the carton pose.

    Raises ValueError with the reason when the registration cannot be applied (status, camera, head).
    """
    if registration.get("status") != "REGISTRATION_VALIDATED":
        raise ValueError("No validated camera-to-arm registration is installed")
    binding = registration.get("binding") or {}
    if binding.get("arm") != "right":
        raise ValueError("The installed registration is not for the right arm")
    if row.get("status") in (None, "UNKNOWN"):
        raise ValueError("OAK observation unavailable: " + str(row.get("reason", "no frame"))[:200])
    frame = row.get("frame") or {}
    if frame.get("camera_id") != binding.get("camera_id"):
        raise ValueError(f"Frame is from {frame.get('camera_id')!r}, registration is for {binding.get('camera_id')!r}")
    if frame.get("timestamp_basis") != "capture":
        raise ValueError("Need a capture-stamped OAK frame")
    age = frame.get("age_s_on_observer_clock")
    if not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= FRAME_MAX_AGE_S:
        raise ValueError("OAK frame is stale, future-dated or has unknown age")
    pose = row.get("pose_3d") or {}
    if pose.get("status") != "CAMERA_RELATIVE_ESTIMATE":
        raise ValueError("No metric tag poses in this frame: " + str(pose.get("reason", pose.get("status")))[:200])
    if pose.get("calibration_sha256") != binding.get("camera_calibration_sha256"):
        raise ValueError("OAK camera calibration changed since registration (resolution/intrinsics/distortion); re-register")
    if pose.get("geometry_config_sha256") != binding.get("tag_geometry_sha256"):
        raise ValueError("Tag registration geometry changed since registration; re-register")
    reference = registration.get("head_ticks_reference")
    _check_head(head_ticks, registration)
    base_from_camera = transform(registration["base_from_camera"])
    rotation = base_from_camera[:3, :3]

    side = {}
    for tag_id, role in CARTON_ROLES.items():
        robot_side = role
        if mirrored:
            robot_side = role.replace("left", "TMP").replace("right", "left").replace("TMP", "right")
        side[tag_id] = robot_side

    tags, warnings = [], []
    centres = {}
    for tag in pose.get("tags", []):
        tag_id = tag.get("tag_id")
        if tag_id not in CARTON_ROLES or tag.get("center_camera_mm") is None:
            continue
        if tag.get("black_square_mm") != PRINT_SIZES_MM[tag_id]:
            raise ValueError(f"Carton tag {tag_id} size differs from the print plan")
        if np.shape(tag["center_camera_mm"]) != (3,) or not np.isfinite(tag["center_camera_mm"]).all():
            raise ValueError("Carton tag centre must be a finite 3-vector")
        centre = base_from_camera @ np.r_[np.asarray(tag["center_camera_mm"], float) / 1000.0, 1.0]
        centre_mm = centre[:3] * 1000.0
        entry = {"tag_id": tag_id, "print_role": CARTON_ROLES[tag_id], "robot_side_role": side[tag_id],
                 "black_square_mm": tag.get("black_square_mm"), "orientation_ambiguous": tag.get("orientation_ambiguous", True),
                 "reprojection_rms_px": tag.get("reprojection_rms_px"), "coarse_position_only": tag.get("coarse_position_only")}
        entry.update(_point(centre_mm))
        std = tag.get("position_std_mm_at_assumed_half_pixel_noise")
        if std is not None and np.shape(std) == (3,):
            entry["position_std_mm_arm_base_axes"] = np.sqrt(np.diag(rotation @ np.diag(np.square(std)) @ rotation.T)).round(2).tolist()
        if tag.get("camera_from_tag") is not None and tag.get("orientation_ambiguous") is False:
            normal = rotation @ np.asarray(tag["camera_from_tag"], float)[:3, 2]
            entry["tag_normal_tilt_from_horizontal_deg"] = round(math.degrees(math.asin(float(np.clip(abs(normal[2]), 0, 1)))), 1)
        tags.append(entry)
        centres[tag_id] = centre_mm
    if not tags:
        raise ValueError("No carton tags with metric poses in this frame")

    near = {i: centres[i] for i in NEAR_WALL_IDS if i in centres}
    carton = None
    if len(near) >= 2:
        direction, mean = _fit_horizontal_line(list(near.values()))
        w = direction if direction[1] < 0 else -direction            # along the near wall, toward the robot's RIGHT (base -y)
        into = np.cross(MODEL_UP, w)                                  # from the near face away from the robot
        if into[0] < 0:
            into = -into
        # Near-wall centre: tag 10 directly, 26/27 shifted by their known offset (side from the observed position).
        estimates = []
        for tag_id, c in near.items():
            along = float(np.dot(c - mean, w))
            if tag_id == 10:
                estimates.append(c)
            else:
                sign = 1.0 if along > 0 else -1.0
                estimates.append(c - sign * WALL_TAG_OFFSET_MM * w)
        centre_tag_height = np.mean(estimates, axis=0)
        residual = float(np.max(np.linalg.norm(np.asarray(estimates) - centre_tag_height, axis=1))) if len(estimates) > 1 else 0.0
        rim_centre = centre_tag_height + HALF_WALL_MM * MODEL_UP
        rim_up_m = float(model_from_base_mm(rim_centre)[2])
        half = CARTON_MM["long"] / 2
        corner_near_right = rim_centre + half * w
        corner_near_left = rim_centre - half * w
        depth = CARTON_MM["short"]
        right_hinge = {"start": corner_near_right, "end": corner_near_right + depth * into}
        left_hinge = {"start": corner_near_left, "end": corner_near_left + depth * into}
        scale = None
        pairs = [(a, b) for a, b in ((26, 27), (10, 26), (10, 27)) if a in near and b in near]
        if pairs:
            scale = []
            for a, b in pairs:
                expected = 2 * WALL_TAG_OFFSET_MM if 10 not in (a, b) else WALL_TAG_OFFSET_MM
                d = float(np.linalg.norm(near[a] - near[b]))
                scale.append({"pair": [a, b], "measured_mm": round(d, 1), "expected_mm": expected, "error_mm": round(d - expected, 1)})
                if abs(d - expected) > 15:
                    warnings.append(f"tag {a}-{b} spacing {d:.0f} mm vs {expected:.0f} mm planned: print scale or mounting differs; distances are suspect")
        if residual > 15:
            warnings.append(f"near-wall tags disagree on the wall centre by {residual:.0f} mm")
        yaw = math.degrees(math.atan2(float(into[1]), float(into[0])))  # 0 = near wall square to the robot
        carton = {
            "status": "CARTON_POSE_ESTIMATED",
            "near_face": {"forward_cm_at_centre": round(float(model_from_base_mm(rim_centre)[0]) * 100, 1),
                          "centre_at_rim": _point(rim_centre), "yaw_deg_from_square": round(yaw, 1),
                          "wall_direction_toward_robot_right_base": [round(float(v), 4) for v in w],
                          "into_box_direction_base": [round(float(v), 4) for v in into],
                          "tags_used": sorted(near), "centre_residual_mm": round(residual, 1)},
            "rim_up_cm": round(rim_up_m * 100, 1),
            "right_wall_top": {"left_cm": _point(corner_near_right)["model_cm"]["left_cm"],
                               "up_cm": round(rim_up_m * 100, 1),
                               "near_corner": _point(corner_near_right), "far_corner": _point(right_hinge["end"]),
                               "midpoint": _point((right_hinge["start"] + right_hinge["end"]) / 2)},
            "right_short_flap_hinge": {"start": _point(right_hinge["start"]), "end": _point(right_hinge["end"]),
                                       "midpoint": _point((right_hinge["start"] + right_hinge["end"]) / 2),
                                       "direction_base": [round(float(v), 4) for v in into], "length_mm": depth,
                                       "flap_tag_id": 11 if mirrored else 12},
            "left_wall_top": {"near_corner": _point(corner_near_left), "far_corner": _point(left_hinge["end"]),
                              "midpoint": _point((left_hinge["start"] + left_hinge["end"]) / 2)},
            "far_face": {"forward_cm_at_centre": round(float(model_from_base_mm(rim_centre + depth * into)[0]) * 100, 1)},
            "scale_check": scale,
        }
        if rim_reference_m is not None:
            error_cm = (rim_up_m - rim_reference_m) * 100
            carton["rim_check"] = {"reference_up_cm": round(rim_reference_m * 100, 1), "error_cm": round(error_cm, 1),
                                   "note": "Diagnostic difference only. Keep model-frame up_cm unchanged for reach; "
                                           "verify tag mounting and the model floor height separately."}
            if abs(error_cm) > 1.5:
                warnings.append(f"tag-derived rim {rim_up_m * 100:.1f} cm differs from the workspace rim {rim_reference_m * 100:.1f} cm by "
                                f"{error_cm:+.1f} cm: verify tag mounting/model floor height; do not offset reach coordinates")
        # Right short flap: lean from its tag, if visible (standing vertical: 90 mm above the hinge, 70 mm toward the far side).
        flap_tag = 11 if mirrored else 12
        if flap_tag in centres:
            c = centres[flap_tag]
            hinge_mid = (right_hinge["start"] + right_hinge["end"]) / 2
            rel = c - hinge_mid
            up_part, out_part, along_part = float(rel[2]), float(np.dot(rel, w)), float(np.dot(rel, into))
            if abs(math.hypot(up_part, out_part) - FLAP_TAG_UP_MM) > 20 or abs(along_part - FLAP_TAG_ALONG_MM) > 20:
                raise ValueError("Right flap tag mounting disagrees with the assumed hinge/radius; no top-edge estimate")
            lean = math.degrees(math.atan2(out_part, up_part))
            tip = hinge_mid + CARTON_MM["flap"] * (math.cos(math.radians(lean)) * MODEL_UP + math.sin(math.radians(lean)) * w)
            carton["right_short_flap"] = {"tag_id": flap_tag, "lean_outward_deg": round(lean, 1),
                                          "tag_above_hinge_mm": round(up_part, 1), "tag_outward_mm": round(out_part, 1),
                                          "tag_along_hinge_from_centre_mm": round(along_part, 1),
                                          "expected_along_mm_if_standing": FLAP_TAG_ALONG_MM,
                                          "top_edge_midpoint_if_flat": _point(tip)}
        if 14 in centres:
            # Near long flap tag: 90 mm from the hinge along the flap. Above the rim = standing, below = hanging.
            c = centres[14]
            outside, above = float(np.dot(c - rim_centre, -into)), float(c[2] - rim_centre[2])
            standing = above > 0
            lean = math.degrees(math.atan2(outside, above)) if standing else math.degrees(math.atan2(outside, -above))
            carton["near_long_flap"] = {"tag_id": 14, "state": "standing" if standing else "hanging_down",
                                        "tag_outside_near_face_mm": round(outside, 1), "tag_above_rim_mm": round(above, 1),
                                        "lean_outward_deg": round(lean, 1),
                                        "top_edge_midpoint_if_flat": _point(rim_centre + CARTON_MM["flap"] * (
                                            math.cos(math.radians(lean)) * MODEL_UP - math.sin(math.radians(lean)) * into)) if standing else None}
    else:
        warnings.append("fewer than two near-wall tags (10/26/27) with metric poses: carton pose not fitted; tag centres only")

    return {"status": carton["status"] if carton else "TAG_CENTRES_ONLY", "arm": "right",
            "frame": {k: frame.get(k) for k in ("camera_id", "seq", "stream_id", "sha256", "captured_at", "age_s_on_observer_clock")},
            "head_ticks": list(head_ticks), "head_ticks_reference": reference,
            "tag_geometry_sha256": binding.get("tag_geometry_sha256"), "mirrored_placement": mirrored,
            "tags": tags, "carton": carton, "warnings": warnings, "assumptions": list(ASSUMPTIONS),
            "frames": {"arm_base_mm": "right arm SO-101 base_link: +x forward, +y arm's left, +z up, millimetres",
                       "model_cm": "pilot reach frame: forward/left/up from the floor below the shoulders, centimetres"},
            "motor_writes": 0, "robot_motion_target": False, "physical_grasp_validated": False,
            "validation_scope": "camera/geometry hashes and head capture checked; current motor/model/range bindings and gripper consistency NOT revalidated",
            "note": "Tag-derived geometry under the listed assumptions; a hinge line is not a pinch point or clearance proof."}


def tool_schema():
    return {"type": "function", "function": {
        "name": TOOL_NAME,
        "description": (
            "Read-only: the carton's position in the RIGHT arm's base frame (mm) and the pilot's model frame "
            "(forward/left/up cm) from fresh OAK AprilTag poses through the validated camera-to-arm registration. "
            "Returns each detected carton tag centre, the near face, rim height, the right wall's top line (the "
            "hinge of the right short flap) and, when its tag is visible, the right flap's lean. Refuses when the "
            "head is not at the registration pose. Not a pinch point or clearance proof; no motor commands."),
        "parameters": {"type": "object", "properties": {"include_images": {"type": "boolean", "default": False}},
                       "additionalProperties": False}}}


class CartonPoseRobot:
    """Decorates the pilot's robot client; adds robot_get_carton_pose on top of robot_get_tags."""

    tool_name = TOOL_NAME

    def __init__(self, robot, *, registration_path=None, clock=time.time, sleep=time.sleep, mirrored=True):
        self.robot = robot
        raw = robot
        while registration_path is None and raw is not None:
            config = getattr(raw, "config", None)
            if isinstance(config, (str, Path)):
                registration_path = Path(config).with_name("tag-registration.json")
                break
            raw = getattr(raw, "robot", None)
        self.registration_path = Path(registration_path) if registration_path else None
        self.workspace_path = self.registration_path.with_name("workspace.json") if self.registration_path else None
        self.clock = clock
        self.sleep = sleep
        self.mirrored = mirrored
        self.last_catalog = None

    def __getattr__(self, name):
        return getattr(self.robot, name)

    def get(self, path):
        return self.robot.get(path)

    def catalog(self):
        catalog = copy.deepcopy(self.robot.catalog())
        names = {t["function"]["name"] for t in catalog["tools"]}
        if self.tool_name not in names and "robot_get_tags" in names and "robot_get_state" in names:
            catalog["tools"].append(tool_schema())
            catalog.setdefault("metadata", {})[self.tool_name] = {
                "execution": "local_geometry_on_robot_get_tags_and_installed_registration", "motor_access": False}
        self.last_catalog = catalog
        return catalog

    def _forward(self, name, args, request_id):
        if request_id is None:
            return self.robot.call(name, args)
        return self.robot.call(name, args, request_id=request_id)

    def call(self, name, args, request_id=None):
        if name != self.tool_name:
            return self._forward(name, args, request_id)
        if not isinstance(args, dict) or set(args) - {"include_images"}:
            raise ValueError("robot_get_carton_pose takes only include_images")
        include = args.get("include_images", False)
        if type(include) is not bool:
            raise ValueError("include_images must be boolean")
        try:
            if self.registration_path is None or not self.registration_path.is_file():
                raise ValueError("No tag-registration.json beside the robot config")
            registration_text = self.registration_path.read_text()
            registration = json.loads(registration_text)
            def sub_id(suffix):
                return None if request_id is None else f"{request_id}:carton:{suffix}"
            state = self._forward("robot_get_state", {"fresh": True}, sub_id("before"))
            head, before_times = _head_sample(state, registration, self.clock())
            for attempt in range(4):
                payload = self._forward("robot_get_tags", {"cameras": ["oak"], "tag_ids": list(CARTON_IDS), "include_images": include}, sub_id(f"tags-{attempt}"))
                if payload.get("ok") is not True:
                    raise ValueError("robot_get_tags failed: " + str(payload.get("error", ""))[:200])
                row = (payload.get("result") or {}).get("observations", {}).get("oak") or {}
                after = self._forward("robot_get_state", {"fresh": True}, sub_id(f"after-{attempt}"))
                after_head, after_times = _head_sample(after, registration, self.clock())
                if np.max(np.abs(np.asarray(after_head)-head)) > 1:
                    raise ValueError("Head moved during carton observation")
                frame = row.get("frame") or {}
                captured = frame.get("captured_at")
                if not isinstance(captured, (int, float)) or not math.isfinite(captured):
                    raise ValueError("OAK frame has no finite capture time")
                if captured > min(after_times):
                    raise ValueError("OAK capture is after the head readback bracket")
                if captured >= max(before_times):
                    if not 0 <= self.clock()-captured <= FRAME_MAX_AGE_S:
                        raise ValueError("OAK capture is stale or future-dated at readback")
                    break
                if attempt == 3:
                    raise ValueError("No OAK capture inside the stationary head bracket; retry sensing")
                self.sleep(.12)
            if self.registration_path.read_text() != registration_text:
                raise ValueError("Registration changed during carton observation")
            rim = None
            try:
                rim = json.loads(self.workspace_path.read_text()).get("object_top_m") if self.workspace_path else None
            except (OSError, ValueError, AttributeError):
                rim = None
            estimate = carton_pose_from_observation(row, registration, head_ticks=head, mirrored=self.mirrored,
                                                    rim_reference_m=rim if isinstance(rim, (int, float)) else None)
            estimate["registration_file_sha256"] = hashlib.sha256(registration_text.encode()).hexdigest()
            return {"ok": True, "result": estimate, "images": payload.get("images", []) if include else [], "motor_writes": 0}
        except (ValueError, TypeError, KeyError, OSError) as exc:
            return {"ok": False, "error": f"{type(exc).__name__}: {str(exc)[:300]}", "motor_writes": 0}


def _cm(p):
    m = p["model_cm"]
    return f"{m['forward_cm']:.1f} fwd, {m['left_cm']:.1f} left, {m['up_cm']:.1f} up"


def carton_pose_lines(payload):
    """Compact supervisor text for sense what ["carton"]."""
    lines = ["Carton pose (OAK tags through the right-arm registration; model frame cm = forward/left/up from the floor "
             "below the shoulders; valid only while the head stays at its registration pose):"]
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        return lines + ["carton pose unavailable: " + str((payload or {}).get("error", "no result"))[:200]]
    r = payload["result"]
    f = r.get("frame") or {}
    lines.append(f"frame seq={f.get('seq')} age={f.get('age_s_on_observer_clock') or 0:.2f}s; head ticks {r.get('head_ticks')}; "
                 f"placement: print-plan left panels on the robot's {'RIGHT' if r.get('mirrored_placement') else 'left'}")
    for t in r.get("tags", []):
        amb = " (orientation ambiguous)" if t.get("orientation_ambiguous") else ""
        lines.append(f"  tag {t['tag_id']} {t['robot_side_role']}: {_cm(t)} cm{amb}")
    lines.append("  Coarse geometry only: current motor/model/range bindings and gripper consistency are not revalidated; not a motion target.")
    c = r.get("carton")
    if not c:
        return lines + ["  carton not fitted: " + "; ".join(r.get("warnings") or ["no near-wall tags"])]
    nf, rw, h = c["near_face"], c["right_wall_top"], c["right_short_flap_hinge"]
    lines.append(f"  near face: {nf['forward_cm_at_centre']:.1f} cm forward at the centre, yaw {nf['yaw_deg_from_square']:+.1f} deg "
                 f"(0 = square to the robot); far face {c['far_face']['forward_cm_at_centre']:.1f} cm forward")
    lines.append(f"  rim: {c['rim_up_cm']:.1f} cm in the reach model frame" + (f" (workspace says {c['rim_check']['reference_up_cm']:.1f}, "
                 f"difference {c['rim_check']['error_cm']:+.1f} cm; diagnostic only, keep reach up_cm unchanged)" if c.get("rim_check") else ""))
    lines.append(f"  right wall top / right short flap hinge: left {rw['left_cm']:.1f} cm (i.e. {abs(rw['left_cm']):.1f} cm to the right), "
                 f"up {rw['up_cm']:.1f} cm, from {_cm(h['start'])} to {_cm(h['end'])}; midpoint {_cm(h['midpoint'])}")
    lw = c["left_wall_top"]
    lines.append(f"  left wall top: {_cm(lw['near_corner'])} to {_cm(lw['far_corner'])}")
    if c.get("right_short_flap"):
        fl = c["right_short_flap"]
        lines.append(f"  right short flap (tag {fl['tag_id']}): leans {fl['lean_outward_deg']:+.0f} deg outward from vertical; "
                     f"top edge midpoint if flat: {_cm(fl['top_edge_midpoint_if_flat'])}")
    else:
        lines.append(f"  right short flap tag {h['flap_tag_id']} not seen: lean and top edge unknown; inspect visually before planning contact")
    if c.get("near_long_flap"):
        nl = c["near_long_flap"]
        lines.append(f"  near long flap (tag 14): {nl['state'].replace('_', ' ')}, leaning {nl['lean_outward_deg']:+.0f} deg outward"
                     + (f"; top edge midpoint if flat: {_cm(nl['top_edge_midpoint_if_flat'])}" if nl.get("top_edge_midpoint_if_flat") else ""))
    if c.get("scale_check"):
        lines.append("  scale check: " + "; ".join(f"tags {s['pair'][0]}-{s['pair'][1]} {s['measured_mm']:.0f} mm apart (planned {s['expected_mm']:.0f}, "
                                                   f"error {s['error_mm']:+.0f} mm)" for s in c["scale_check"]))
    for w in r.get("warnings") or []:
        lines.append("  WARNING: " + w)
    lines.append("  These are tag-derived under stated assumptions (vertical near wall, print sizes at 100 %); verify with a look before contact.")
    return lines
