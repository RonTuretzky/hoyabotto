"""Head (OAK) camera pose in the arm_base frame from one frame of the robot's own gripper tags. No robot I/O.

Replaces the manual head set-up of the fold policy (inclinometer on the head, floor tags 40-43, a hand pose solve):
the two gripper tags (left ID 4, right ID 2, 40 mm, on each gripper_link) are located in 3D from the arm encoder
ticks (owner-accepted joint maps, profiles/fold-joint-maps/, and the SO-101 kinematics of the fold training scene),
detected in the OAK frame, and every corner goes into one solvePnP. Optionally the carton's rigid wall/floor tags
(10, 21, 22, 24-28, 45 mm) are added when the carton sits in its nominal training spot (10 mm from the table edge,
centred, square to the robot): `use_box_tags=True`.

Frames
- `arm_base` (carton/folding_station_measured.py): origin midway between the two SO-101 base_link origins on their
  mounting plane, +x robot right, +y toward the table, +z up; metres.
- Camera rotations are OpenCV camera-to-arm_base (`rotation_cv` columns: image right, image down, optical axis).
- Head angles are reported in the XLeRobot model's convention (carton/xlerobot_cameras.py): tilt = camera pitch
  below horizontal (positive looks down; training 58 deg), pan = yaw of the optical axis from +y, positive to the
  robot's LEFT (head_pan_joint positive; training 0). Both come from the measured camera rotation, not from head
  ticks: the twin's head tick<->angle mapping was found off by ~15 deg tilt / ~17 deg pan on 8 Oct.

The SO-101 chain below is copied from the fold training scene (`scene-assets/arm-import.xml`, the two arms placed by
carton/folding_sim.build_scene at x = -/+ spacing/2 with euler z = 90 deg); tests check it against the recorded
scene with MuJoCo when that file is present. Gripper tags: carton/folding_sim.py marker(grip, ..., .040,
[.045, 0, .008], [0 1 0 0 0 1]).

Accuracy (tests/test_auto_head_pose.py): on MuJoCo renders of the training scene with the arms in
MEASUREMENT_ARM_POSE_DEG the pose comes back within 0.3 deg and 1.1 mm. Two 40 mm tags are a small target: with
0.5 px corner noise the 1-sigma is about 0.7 deg tilt / 1.1 deg pan (`sensitivity_1sigma`), so average frames. The
pose is relative to the arms as the joint maps place them: a zero error shared by both arms (e.g. shoulder_lift +2 deg)
tilts the answer by about as much with a perfect fit; a wrong joint on one arm usually shows as reprojection error.
The box tags place the camera relative to the table instead; with both sets the separate solutions are compared
(`subset_solutions`).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from carton.xlerobot_cameras import head_camera_model, model_dir_to_arm_base, model_to_arm_base

SOFTWARE = Path(__file__).resolve().parents[1]
STATION_PROFILE = SOFTWARE / 'profiles/fold-station-xlerobot-220.json'
JOINT_MAP_DIR = SOFTWARE / 'profiles/fold-joint-maps'
TRAINING_TILT_DEG, TRAINING_PAN_DEG = 58.0, 0.0
TRAINING_FOVY_DEG, TRAINING_ASPECT = 54.0, 4 / 3
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')

# SO-101 body chain (scene-assets/arm-import.xml): body -> (parent offset pos, quat w x y z); each body's joint is
# about its local z at its origin (the MJCF gives no axis/pos).
SO101_CHAIN = (
    ('shoulder_link', (0.0388353, -8.97657e-09, 0.0624), (1.76036e-12, 1.32679e-06, -1., -1.32679e-06), 'shoulder_pan'),
    ('upper_arm_link', (-0.0303992, -0.0182778, -0.0542), (0.499998, -0.5, -0.5, -0.500002), 'shoulder_lift'),
    ('lower_arm_link', (-0.11257, -0.028, 0.), (0.707105, 0., 0., 0.707108), 'elbow_flex'),
    ('wrist_link', (-0.1349, 0.0052, 0.), (0.707105, 0., 0., -0.707108), 'wrist_flex'),
    ('gripper_link', (0., -0.0611, 0.0181), (0.0172101, -0.0172081, 0.706899, 0.706896), 'wrist_roll'),
)
BASE_YAW_RAD = math.pi / 2          # folding_sim.build_scene: each base_link euler="0 0 pi/2"
GRIPPER_TAG_SIZE_M = .040
GRIPPER_TAG_IN_LINK = ((.045, 0., .008), (0., 1., 0., 0., 0., 1.))   # pos, xyaxes (printed right, printed top)
GRIPPER_TAGS = {4: 'left', 2: 'right'}
PRINT_PROUD_M = .0002               # folding_sim.marker: black cells drawn 0.2 mm above the tag body plane
# Rigid carton tags (not flaps) in the carton frame (x robot right, y away, z up; origin bottom centre), as in the
# training scene (carton/folding_markers.py BOX_MARKERS + tools/diagnose_short_flap_brace.py extras;
# tools/make_fold_box_tags.py prints them). 45 mm; the wall tags sit 1.8 mm proud of the cardboard.
BOX_TAG_SIZE_M = .045
BOX_TAGS = {
    10: ((0., -.1433, .054), (1, 0, 0, 0, 0, 1)),
    26: ((-.12, -.1433, .054), (1, 0, 0, 0, 0, 1)),
    27: ((.12, -.1433, .054), (1, 0, 0, 0, 0, 1)),
    21: ((-.1913, .04, .054), (0, -1, 0, 0, 0, 1)),
    28: ((-.1913, -.08, .054), (0, -1, 0, 0, 0, 1)),
    22: ((.1913, .04, .054), (0, 1, 0, 0, 0, 1)),
    25: ((0., 0., .0038), (1, 0, 0, 0, 1, 0)),
    24: ((.08, 0., .0038), (1, 0, 0, 0, 1, 0)),
}
CARTON_HALF_WIDTH_M = .1415         # carton/geometry.Box width 283 mm
CARTON_TO_EDGE_M = .01
CARTON_BOTTOM_ABOVE_TABLE_M = .001  # folding_sim.build_scene carton body z
MIN_TAGS = 2
MAX_RMS_PX = 1.0                    # corners of 60-70 px tags; a wrong joint on ONE arm shows here (common errors do not)
# Arms for the measurement (URDF degrees, shoulder_pan .. wrist_roll; jaw unchanged): grippers raised in front of the
# robot, tag centres at about (-/+0.10, 0.28, 0.20) m in arm_base, facing the head. Chosen in the training scene so both
# tags stay in a 640x360 (39 deg VFOV) frame for head tilts from about 33 to 60 deg and a 640x480 (54 deg) one at
# 46-64 deg (cradle lens offset included), at 60-70 px per tag side. Clear of the table; take the carton away first.
MEASUREMENT_ARM_POSE_DEG = {'left': (-1., 34., -4., -76., -14.), 'right': (1., 34., -4., -76., 20.)}


class HeadPoseRefused(ValueError):
    """The frame cannot give a trustworthy head pose (too few tags, poor fit, bad inputs)."""


# ----------------------------------------------------------------------------------------- geometry helpers
def _quat(q):
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def detector_frame(xyaxes):
    """Rotation of a printed tag's detector frame (pupil_apriltags corner order with
    farm.perception.tag_geometry.square_points) in its body frame: x = printed left, y = printed top, z = into
    the tag (carton/folding_markers.box_marker_poses, tools/camera_pose_from_tag.tag_frame)."""
    u, v = np.asarray(xyaxes[:3], float), np.asarray(xyaxes[3:], float)
    return np.column_stack((-u, v, -np.cross(u, v)))


def square_corners(size_m):
    s = size_m / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)


def tag_mount(body_rotation, body_position, pos, xyaxes, size_m, proud_m=PRINT_PROUD_M):
    """(centre, printed-right axis, printed-top axis, size) of a tag mounted at `pos`/`xyaxes` on a body, in the
    body's parent frame; the centre sits `proud_m` out of the mounting plane (the printed surface)."""
    r = np.asarray(body_rotation, float)
    u, v = r @ np.asarray(xyaxes[:3], float), r @ np.asarray(xyaxes[3:], float)
    centre = r @ np.asarray(pos, float) + np.asarray(body_position, float) + proud_m * np.cross(u, v)
    return centre, u, v, float(size_m)


def mount_corners(mount):
    """The four detector-order corners (4x3) of a tag mount."""
    centre, u, v, size = mount
    return centre + square_corners(size) @ detector_frame(np.r_[u, v]).T


def gripper_link_pose(q_rad):
    """gripper_link (R, p) in its arm's base_link frame for the five positioning joints (radians, URDF convention)."""
    r, p = np.eye(3), np.zeros(3)
    for (_, pos, quat, _), q in zip(SO101_CHAIN, q_rad):
        p = p + r @ np.asarray(pos, float)
        r = r @ _quat(quat) @ _rz(float(q))
    return r, p


def arm_base_from_base_link(side, base_spacing_m):
    x = -base_spacing_m / 2 if side == 'left' else base_spacing_m / 2
    return _rz(BASE_YAW_RAD), np.array([x, 0., 0.])


def gripper_tag_mounts(joint_rad, base_spacing_m):
    """{tag id: mount} of the gripper tags in arm_base; joint_rad: {'left': [5 rad], 'right': [5 rad]}."""
    out = {}
    pos, axes = GRIPPER_TAG_IN_LINK
    for tag_id, side in GRIPPER_TAGS.items():
        if side not in joint_rad:
            continue
        r_g, p_g = gripper_link_pose(joint_rad[side])
        r_b, p_b = arm_base_from_base_link(side, base_spacing_m)
        out[tag_id] = tag_mount(r_b @ r_g, p_b + r_b @ p_g, pos, axes, GRIPPER_TAG_SIZE_M)
    return out


def gripper_tag_points(joint_rad, base_spacing_m):
    """{tag id: 4x3 arm_base corners} of the gripper tags."""
    return {i: mount_corners(m) for i, m in gripper_tag_mounts(joint_rad, base_spacing_m).items()}


def nominal_carton_pose(station):
    """Carton frame (rotation, origin) in arm_base for the training placement: centred on the robot, square, its near
    wall 10 mm from the table edge (the recorded demonstrations' nominal spot, yaw 0, offset 0)."""
    y = float(station['base_line_to_table_edge_m']) + CARTON_TO_EDGE_M + CARTON_HALF_WIDTH_M
    z = CARTON_BOTTOM_ABOVE_TABLE_M - float(station['base_height_above_table_m'])
    return np.eye(3), np.array([0., y, z])


def box_tag_mounts(station):
    r, p = nominal_carton_pose(station)
    return {tag_id: tag_mount(r, p, pos, axes, BOX_TAG_SIZE_M, proud_m=.0003) for tag_id, (pos, axes) in BOX_TAGS.items()}


def box_tag_points(station):
    return {i: mount_corners(m) for i, m in box_tag_mounts(station).items()}


# ------------------------------------------------------------------------------------------------ inputs
def load_station(path=STATION_PROFILE):
    data = json.loads(Path(path).read_text())
    return data, data['station']


def training_front_camera(path=STATION_PROFILE):
    return json.loads(Path(path).read_text())['cameras']['front']


def arm_joint_radians(arm_maps, ticks):
    """{'left': [5 rad], 'right': [5 rad]} from owner encoder ticks through the measured joint maps (refuses ticks
    outside the saved calibration, carton.fold_policy_runner.ArmMap.ticks_to_rad)."""
    return {side: arm_maps[side].ticks_to_rad(ticks)[:5] for side in ('left', 'right')}


def measurement_pose_ticks(arm_maps):
    """Owner encoder targets of MEASUREMENT_ARM_POSE_DEG through the joint maps (for the pilot's own arm tools)."""
    out = {}
    for side, degrees in MEASUREMENT_ARM_POSE_DEG.items():
        for suffix, value in zip(ARM_JOINTS, degrees):
            u = arm_maps[side].units[suffix]
            out[f'{side}_arm_{suffix}'] = int(round(u.model_degrees_to_ticks(value)))
    return out


def intrinsics_matrix(intrinsics):
    """(K, distortion, width, height) from {fx, fy, cx, cy, width, height[, distortion]} or {'K': 3x3, ...}."""
    if 'K' in intrinsics:
        k = np.asarray(intrinsics['K'], float)
    else:
        k = np.array([[intrinsics['fx'], 0, intrinsics['cx']], [0, intrinsics['fy'], intrinsics['cy']], [0, 0, 1]], float)
    dist = np.asarray(intrinsics.get('distortion') or [], float)
    return k, (dist if dist.size else np.zeros(5)), int(intrinsics['width']), int(intrinsics['height'])


def aspect_report(width, height, fy):
    """How the real stream compares with the training camera (4:3, VFOV 54 deg)."""
    vfov = math.degrees(2 * math.atan(height / 2 / fy))
    aspect = width / height
    out = {'width': int(width), 'height': int(height), 'aspect': round(aspect, 4), 'vertical_fov_deg': round(vfov, 2),
           'training_aspect': round(TRAINING_ASPECT, 4), 'training_vertical_fov_deg': TRAINING_FOVY_DEG,
           'aspect_matches_training': abs(aspect - TRAINING_ASPECT) < .01,
           'vertical_fov_matches_training': abs(vfov - TRAINING_FOVY_DEG) < 1.5}
    warnings = []
    if not out['aspect_matches_training']:
        warnings.append(f'OAK stream is {width}x{height} (aspect {aspect:.3f}), not 4:3 like the training camera; '
                        f'a 4:3 crop of it has a {vfov:.1f} deg vertical FOV vs {TRAINING_FOVY_DEG:.0f} deg in training')
    elif not out['vertical_fov_matches_training']:
        warnings.append(f'OAK vertical FOV {vfov:.1f} deg differs from the training camera ({TRAINING_FOVY_DEG:.0f} deg)')
    out['warnings'] = warnings
    return out


# ---------------------------------------------------------------------------------------------- the solver
def head_angles(rotation_cv):
    """(tilt_deg, pan_deg, roll_deg) of an OpenCV camera-to-arm_base rotation in the head model's convention.

    tilt: optical axis below horizontal (positive down); pan: optical axis yaw from +y, positive toward the robot's
    left (-x); roll: rotation of the image about the optical axis relative to the model head at that tilt/pan
    (right-handed about the optical axis; 0 for a camera square in its mount)."""
    r = np.asarray(rotation_cv, float)
    forward, right = r[:, 2], r[:, 0]
    tilt = math.degrees(math.asin(max(-1., min(1., -forward[2]))))
    pan = math.degrees(math.atan2(-forward[0], forward[1]))
    _, rm = head_camera_model(math.radians(min(max(tilt, -43.), 83.)), math.radians(pan))
    f_m, up_m = model_dir_to_arm_base(rm[:, 0]), model_dir_to_arm_base(rm[:, 2])
    model_right = np.cross(f_m, up_m)
    roll = math.degrees(math.atan2(float(np.cross(model_right, right) @ forward), float(model_right @ right)))
    return tilt, pan, roll


def model_lens_position(tilt_deg, pan_deg, optical_offset=None):
    """Model head-camera optical centre in arm_base at the given head angles (+ the cradle's lens offset)."""
    if optical_offset is None:
        from farm.sim.xlerobot_twin import HEAD_OPTICAL_OFFSET_M as optical_offset
    lo, hi = -.76, 1.45
    position, r = head_camera_model(min(max(math.radians(tilt_deg), lo), hi), math.radians(pan_deg))
    return model_to_arm_base(position + r @ np.asarray(optical_offset, float))


def _pose_from_rt(rvec, tvec):
    r_cb, _ = cv2.Rodrigues(rvec)
    return -r_cb.T @ np.asarray(tvec, float).reshape(3), r_cb.T


def _rms(obj, img, rvec, tvec, k, dist):
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    return float(np.sqrt(np.mean(err ** 2))), err


def solve_camera(objects, detections, k, dist):
    """Joint solvePnP over every corner of the given tags. objects/detections: {id: 4x3}/{id: 4x2}.

    Seeds: each tag's two planar (IPPE square) solutions and SQPnP over all corners; each refined with
    Levenberg-Marquardt on all corners; the lowest-RMS seed with every corner in front of the camera wins."""
    ids = sorted(objects)
    obj = np.concatenate([objects[i] for i in ids]).astype(float)
    img = np.concatenate([np.asarray(detections[i], float) for i in ids])
    seeds = []
    for i in ids:
        c = objects[i]
        centre = c.mean(axis=0)
        ex, ey = c[1] - c[0], c[0] - c[3]
        size = float(np.linalg.norm(ex))
        r_bt = np.column_stack((ex / np.linalg.norm(ex), ey / np.linalg.norm(ey),
                                np.cross(ex, ey) / np.linalg.norm(np.cross(ex, ey))))
        try:
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(square_corners(size), np.asarray(detections[i], float), k, dist,
                                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            continue
        for rv, tv in zip(rvecs, tvecs):
            r_ct, _ = cv2.Rodrigues(rv)
            r_cb = r_ct @ r_bt.T
            seeds.append((cv2.Rodrigues(r_cb)[0], (tv.reshape(3) - r_cb @ centre).reshape(3, 1)))
    try:
        ok, rv, tv = cv2.solvePnP(obj, img, k, dist, flags=cv2.SOLVEPNP_SQPNP)
        if ok:
            seeds.append((rv, tv))
    except cv2.error:
        pass
    best = None
    for rv, tv in seeds:
        rv, tv = cv2.solvePnPRefineLM(obj, img, k, dist, rv.copy(), tv.copy(),
                                      (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-12))
        r, _ = cv2.Rodrigues(rv)
        if np.min((obj @ r.T + tv.reshape(3))[:, 2]) <= .02:
            continue
        rms, err = _rms(obj, img, rv, tv, k, dist)
        if best is None or rms < best[0]:
            best = (rms, rv, tv, err)
    if best is None:
        raise HeadPoseRefused('No camera pose puts every tag corner in front of the camera')
    rms, rv, tv, err = best
    per_tag = {int(i): float(np.sqrt(np.mean(err[4 * n:4 * n + 4] ** 2))) for n, i in enumerate(ids)}
    return rv, tv, rms, float(err.max()), per_tag, obj, img


def _sensitivity(obj, rv, tv, k, dist, sigma_px=.5):
    """1-sigma tilt/pan/roll (deg) and position (mm) for `sigma_px` independent corner noise (linearised).
    Excludes joint-map, kinematic, tag-print and intrinsics errors."""
    _, jac = cv2.projectPoints(obj, rv, tv, k, dist)
    j = jac[:, :6]
    cov = np.linalg.pinv(j.T @ j) * sigma_px ** 2
    x0 = np.r_[rv.reshape(3), tv.reshape(3)]

    def f(x):
        p, r = _pose_from_rt(x[:3].reshape(3, 1), x[3:].reshape(3, 1))
        return np.r_[head_angles(r), p * 1000]
    base, g = f(x0), np.zeros((6, 6))
    for n in range(6):
        h = 1e-6
        g[:, n] = (f(x0 + np.eye(6)[n] * h) - base) / h
    std = np.sqrt(np.maximum(0, np.diag(g @ cov @ g.T)))
    return {'tilt_deg': float(std[0]), 'pan_deg': float(std[1]), 'roll_deg': float(std[2]),
            'position_mm': std[3:].round(2).tolist(), 'corner_noise_px': sigma_px}


@dataclass
class HeadPoseConfig:
    base_spacing_m: float = .22
    station: dict | None = None             # needed with use_box_tags
    use_box_tags: bool = False
    min_tags: int = MIN_TAGS
    max_rms_px: float = MAX_RMS_PX
    optical_offset: tuple | None = None     # lens offset from the camera link (default: the twin's cradle design)


def estimate_head_pose(detections, k, dist, joint_rad, config: HeadPoseConfig = HeadPoseConfig()):
    """Head camera pose from detected tags. detections: {id: {'corners': 4x2, ...}} (farm.perception.tags format).

    Refuses (HeadPoseRefused) with fewer than `min_tags` known tags or an RMS reprojection above `max_rms_px`."""
    known = gripper_tag_points(joint_rad, config.base_spacing_m)
    if config.use_box_tags:
        if config.station is None:
            raise HeadPoseRefused('Box tags need the station geometry (base height, base line to table edge)')
        known.update(box_tag_points(config.station))
    used = sorted(i for i in detections if i in known)
    gripper_used = [i for i in used if i in GRIPPER_TAGS]
    seen = sorted(int(i) for i in detections)
    if len(used) < config.min_tags:
        raise HeadPoseRefused(f'{len(used)} usable tag(s) {used} (seen {seen}); need at least {config.min_tags}: hold '
                              'both grippers still with their tags (left 4, right 2) facing the head'
                              + (', or place the carton in its nominal spot' if config.use_box_tags else ''))
    objects = {i: known[i] for i in used}
    corners = {i: np.asarray(detections[i]['corners'], float) for i in used}
    rv, tv, rms, max_err, per_tag, obj, img = solve_camera(objects, corners, k, dist)
    position, rotation = _pose_from_rt(rv, tv)
    tilt, pan, roll = head_angles(rotation)
    result = {'position_m': position.round(5).tolist(), 'rotation_cv': rotation.round(7).tolist(),
              'tilt_deg': tilt, 'pan_deg': pan, 'roll_deg': roll,
              'reprojection_rms_px': rms, 'reprojection_max_px': max_err, 'per_tag_rms_px': per_tag,
              'tags_used': used, 'gripper_tags_used': gripper_used, 'box_tags_used': [i for i in used if i not in GRIPPER_TAGS],
              'tags_seen': seen, 'corners_used': int(len(obj)),
              'tag_side_px': {int(i): float(np.mean(np.linalg.norm(corners[i] - np.roll(corners[i], 1, axis=0), axis=1)))
                              for i in used},
              'sensitivity_1sigma': _sensitivity(obj, rv, tv, k, dist)}
    box_used = result['box_tags_used']
    if len(gripper_used) >= 2 and len(box_used) >= 2:
        # Independent cross-check: the box tags place the camera relative to the table and carton, the gripper tags
        # relative to the arms' joint maps. A common zero error in the maps shifts only the gripper solution.
        subsets = {}
        for name, ids in (('gripper', gripper_used), ('box', box_used)):
            rv_s, tv_s, rms_s, *_ = solve_camera({i: objects[i] for i in ids}, {i: corners[i] for i in ids}, k, dist)
            p_s, r_s = _pose_from_rt(rv_s, tv_s)
            t_s, pa_s, _ = head_angles(r_s)
            subsets[name] = {'tilt_deg': t_s, 'pan_deg': pa_s, 'position_m': p_s.round(5).tolist(), 'rms_px': rms_s}
        g, b = subsets['gripper'], subsets['box']
        subsets['gripper_minus_box'] = {
            'tilt_deg': g['tilt_deg'] - b['tilt_deg'], 'pan_deg': g['pan_deg'] - b['pan_deg'],
            'position_mm': ((np.asarray(g['position_m']) - np.asarray(b['position_m'])) * 1000).round(1).tolist()}
        result['subset_solutions'] = subsets
    lens = model_lens_position(tilt, pan, config.optical_offset)
    result['model_lens_position_at_measured_angles_m'] = lens.round(5).tolist()
    result['position_minus_model_lens_mm'] = ((position - lens) * 1000).round(1).tolist()
    if rms > config.max_rms_px:
        raise HeadPoseRefused(f'Reprojection RMS {rms:.2f} px exceeds {config.max_rms_px} px (tags {used}); a joint '
                              'map, tag mounting or intrinsics problem, or the arms moved during the frame')
    return result


def detect(image_rgb):
    from farm.perception.tags import detect_tags
    from farm.status import Reading, Status
    found = detect_tags(Reading(image_rgb, Status.OK))
    if found.status is not Status.OK:
        raise HeadPoseRefused(f'Tag detection not trustworthy: {found.note}')
    return found.value


def measure_head_pose(image_rgb, intrinsics, arm_maps, ticks, config: HeadPoseConfig = HeadPoseConfig(), detections=None):
    """One frame + intrinsics + current arm joint ticks -> head camera pose in arm_base (see estimate_head_pose).

    intrinsics: {fx, fy, cx, cy, width, height[, distortion]} or {'K', 'width', 'height'[, 'distortion']} for the
    exact image size."""
    k, dist, w, h = intrinsics_matrix(intrinsics)
    if image_rgb.shape[1] != w or image_rgb.shape[0] != h:
        raise HeadPoseRefused(f'Intrinsics are for {w}x{h}, image is {image_rgb.shape[1]}x{image_rgb.shape[0]}')
    joint_rad = arm_joint_radians(arm_maps, ticks)
    detections = detect(image_rgb) if detections is None else detections
    result = estimate_head_pose(detections, k, dist, joint_rad, config)
    result['image_size'] = [w, h]
    result['stream'] = aspect_report(w, h, float(k[1, 1]))
    result['arm_joint_deg'] = {side: [round(math.degrees(v), 3) for v in q] for side, q in joint_rad.items()}
    return result


def compare_with_training(result, training=None):
    """Measured pose vs the training 'front' camera (profiles/fold-station-xlerobot-220.json)."""
    training = training or training_front_camera()
    t_rot = np.asarray(training['rotation_cv'], float)
    t_tilt, t_pan, _ = head_angles(t_rot)
    rel = np.asarray(result['rotation_cv'], float).T @ t_rot
    angle = math.degrees(math.acos(max(-1., min(1., (np.trace(rel) - 1) / 2))))
    return {'training_tilt_deg': round(t_tilt, 3), 'training_pan_deg': round(t_pan, 3),
            'training_position_m': training['position_m'],
            'tilt_error_deg': result['tilt_deg'] - t_tilt, 'pan_error_deg': result['pan_deg'] - t_pan,
            'rotation_difference_deg': angle,
            'position_error_mm': ((np.asarray(result['position_m']) - np.asarray(training['position_m'])) * 1000).round(1).tolist(),
            'note': ('Training position is the model camera link at tilt 58 / pan 0. The OAK lens in the slot cradle sits '
                     'off that link (farm/sim/xlerobot_twin.py HEAD_OPTICAL_OFFSET_M), so a position difference remains '
                     'even at the training angles; restage the demonstrations with the measured entry '
                     '(tools/restage_fold_scenes.py) to match it.')}


def camera_entry(result, intrinsics, *, head_ticks=None, source=''):
    """`cameras.front` entry of a measurement file (carton/folding_station_measured.py load_measurement)."""
    k, dist, w, h = intrinsics_matrix(intrinsics)
    entry = {'position_m': result['position_m'], 'rotation_cv': result['rotation_cv'],
             'intrinsics': {'fx': float(k[0, 0]), 'fy': float(k[1, 1]), 'cx': float(k[0, 2]), 'cy': float(k[1, 2]),
                            'width': w, 'height': h},
             'head_tilt_deg': round(result['tilt_deg'], 3), 'head_pan_deg': round(result['pan_deg'], 3),
             'head_roll_deg': round(result['roll_deg'], 3),
             'sources': (f'carton/head_pose.py: gripper tags {result["gripper_tags_used"]}'
                         + (f', box tags {result["box_tags_used"]}' if result['box_tags_used'] else '')
                         + f' via arm FK; reprojection RMS {result["reprojection_rms_px"]:.2f} px'
                         + (f'; {source}' if source else ''))}
    if head_ticks is not None:
        entry['head_ticks'] = dict(head_ticks)
    return entry
