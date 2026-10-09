"""Automated fold-policy station check from one head-camera frame. Read-only: no robot, motor or camera contact.

The learned two-flap fold policy was trained at one station (carton/folding_station_measured.py,
profiles/fold-station-xlerobot-220.json, docs/carton-fold-policy-setup-slides.html steps 2-4). This module
replaces the tape-measure checks of those steps with one picture: it finds the printed box tags
(tools/make_fold_box_tags.py) on the carton, fits the carton's pose with solvePnP, moves it into the
`arm_base` frame through the head camera's pose, and says how to move the table and the carton.

`arm_base` frame: origin midway between the two SO-101 base origins on their mounting plane (the bottom of the
bases), +x to the robot's right, +y horizontally toward the table, +z up; metres.

Training station (the target):
- table top at z = -0.120 (the 120 mm gap below the arm bases), level;
- table edge at y = +0.150 (not visible to the check: it assumes step 3's cart-to-table setback);
- carton 379 x 283 x 108 mm (flaps 140 mm, all four standing up), centre at x = -0.010, square to the table
  (yaw 0), its nearest bottom corner 10 mm in from the table edge (y = +0.160).
Training ranges (batch-220-01): centre x -25..+5 mm, yaw -4..+4 deg; height and setback were not varied.

Carton frame (carton/geometry.Box): x to the robot's right, y away from the robot, z up, origin at the centre of
the carton's bottom face (the table top under the carton). So the table gap is -z of that origin.

Only the eight tags on rigid faces (near wall 26/10/27, left wall 21/28, right wall 22, inside floor 25/24) are
used for the pose. The four flap tags (11-14) move with the flaps; they are only compared with where an upright
flap would put them, to tell the owner to stand a flap up.

Head-camera pose input (`load_camera`), schema `xlerobot-head-camera-pose/1` (the `schema` key is optional):

    {"schema": "xlerobot-head-camera-pose/1", "frame": "arm_base",
     "position_m": [x, y, z],                 # lens centre in arm_base
     "rotation_cv": [[...], [...], [...]],    # columns: optical x (image right), y (image down), z (forward)
     "fx": 505.0, "fy": 505.0, "cx": 319.5, "cy": 179.5,   # pixels, for an image of width x height
     "width": 640, "height": 360,             # optional; intrinsics are scaled to a same-aspect image
     "dist": [k1, k2, p1, p2, k3]}            # optional OpenCV distortion; [] or omitted for rectified frames

Also accepted: the same keys with the intrinsics nested under "intrinsics" (tools/camera_pose_from_tag.py's
camera entry, "distortion" for "dist"), and a station-measurement file whose cameras.front is such an entry.
The model-derived `front` camera of profiles/fold-station-xlerobot-220.json is the fallback; it is flagged
`measured: false` because it is the XLeRobot model at head tilt 58 deg, not a measurement.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

from carton.geometry import Box

SCHEMA = 'xlerobot-station-check/1'
CAMERA_SCHEMA = 'xlerobot-head-camera-pose/1'
PROFILE = Path(__file__).resolve().parents[1] / 'profiles' / 'fold-station-xlerobot-220.json'

BOX = Box()
L, W, H = BOX.length, BOX.width, BOX.height
WALL_TAG_M, FLAP_TAG_M = .045, .035          # black square side (tools/make_fold_box_tags.py)
# Training scene: wall tags sit 2.1 mm outside the nominal wall plane (1.5 mm half-wall + 0.6 mm printed
# marker), floor tags 4.1 mm above the bottom (3 mm bottom + 1.1 mm marker).
TAG_STANDOFF_M = .0021
FLOOR_TAG_Z_M = .0041
SHORT_HINGE_Z_M, LONG_HINGE_Z_M, FLAP_TAG_UP_M = H, H + .0035, .090

TARGET = {'table_top_z_m': -.120, 'table_edge_y_m': .150, 'carton_x_m': -.010, 'near_wall_y_m': .160,
          'yaw_deg': 0.}
TOLERANCE = {'height_mm': 3., 'position_mm': 3., 'yaw_deg': 2., 'tilt_deg': 1.}
TRAINING_RANGE = {'carton_x_mm': (-25., 5.), 'yaw_deg': (-4., 4.)}
# Tag misfit (mm, in each tag's own plane) above which the tags do not sit where the training carton's geometry
# puts them: a different box, a misplaced tag, or a bent wall. RESIDUAL: the tag's corners against the joint fit
# of all tags; LEAVE_ONE_OUT: against the fit of the other tags. Rendered training-carton frames stay below
# 0.6 mm on both; a 15-20 mm misplaced tag or face gives 3-25 mm.
GEOMETRY_RESIDUAL_MM = 3.
GEOMETRY_LEAVE_ONE_OUT_MM = 6.
FLAP_NOT_UP_MM = 25.     # a flap tag this far from its upright place (about 15 deg of flap turn): not standing up
MIN_RIGID_TAGS = 2
LEAVE_ONE_OUT_MIN_TAGS = 3
# The tilt is only trusted from tags spread this far apart (the two floor tags, 80 mm apart, give up to 1 deg).
TILT_MIN_SPAN_M = .15
# The depth table plane (farm.perception.depth_scene, 10 mm RANSAC inliers) also takes in the carton floor and
# the OAK's millimetre depth steps: a coarse cross-check only.
DEPTH_DISAGREE_MM = 10.
MIN_EDGE_PX = 8.

# id: (face, centre in the carton frame, printed right, printed up). Every tag reads upright from outside its
# face (floor: seen from above, standing at the robot); the outward normal is right x up.
_S = TAG_STANDOFF_M
RIGID_TAGS = {
    26: ('near wall', (-.120, -W / 2 - _S, H / 2), (1, 0, 0), (0, 0, 1)),
    10: ('near wall', (0., -W / 2 - _S, H / 2), (1, 0, 0), (0, 0, 1)),
    27: ('near wall', (.120, -W / 2 - _S, H / 2), (1, 0, 0), (0, 0, 1)),
    21: ('left wall', (-L / 2 - _S, .040, H / 2), (0, -1, 0), (0, 0, 1)),
    28: ('left wall', (-L / 2 - _S, -.080, H / 2), (0, -1, 0), (0, 0, 1)),
    22: ('right wall', (L / 2 + _S, .040, H / 2), (0, 1, 0), (0, 0, 1)),
    25: ('floor', (0., 0., FLOOR_TAG_Z_M), (1, 0, 0), (0, 1, 0)),
    24: ('floor', (.080, 0., FLOOR_TAG_Z_M), (1, 0, 0), (0, 1, 0)),
}
# Flap tags with all four flaps standing straight up (the training start).
FLAP_TAGS = {
    11: ('left short flap', (-L / 2 - _S, .070, SHORT_HINGE_Z_M + FLAP_TAG_UP_M), (0, -1, 0), (0, 0, 1)),
    12: ('right short flap', (L / 2 + _S, .070, SHORT_HINGE_Z_M + FLAP_TAG_UP_M), (0, 1, 0), (0, 0, 1)),
    13: ('far long flap', (-.080, W / 2 + _S, LONG_HINGE_Z_M + FLAP_TAG_UP_M), (-1, 0, 0), (0, 0, 1)),
    14: ('near long flap', (.080, -W / 2 - _S, LONG_HINGE_Z_M + FLAP_TAG_UP_M), (1, 0, 0), (0, 0, 1)),
}


# ---------------------------------------------------------------- tag model

def tag_corners(tag_id):
    """The tag's four black-square corners in the carton frame, in the detector's corner order.

    Same convention as tools/camera_pose_from_tag.py: with farm.perception.tag_geometry.square_points and the
    pupil_apriltags corner order, the tag frame's x points to the printed tag's left, y to its top, z into it.
    """
    face, centre, right, up = {**RIGID_TAGS, **FLAP_TAGS}[tag_id]
    size = WALL_TAG_M if tag_id in RIGID_TAGS else FLAP_TAG_M
    right, up = np.asarray(right, float), np.asarray(up, float)
    rot = np.column_stack((-right, up, -np.cross(right, up)))
    s = size / 2
    square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    return square @ rot.T + np.asarray(centre, float)


def tag_size(tag_id):
    return WALL_TAG_M if tag_id in RIGID_TAGS else FLAP_TAG_M


# ---------------------------------------------------------------- camera input

def _intrinsics_from(spec):
    src = spec.get('intrinsics') if isinstance(spec.get('intrinsics'), dict) else spec
    if not all(src.get(k) is not None for k in ('fx', 'fy', 'cx', 'cy')):
        return None
    out = {k: float(src[k]) for k in ('fx', 'fy', 'cx', 'cy')}
    for k in ('width', 'height'):
        if src.get(k) is not None:
            out[k] = int(src[k])
    dist = src.get('dist', src.get('distortion', spec.get('dist', spec.get('distortion'))))
    out['dist'] = [float(v) for v in (dist or [])]
    return out


def _rotation(value):
    r = np.asarray(value, float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError('rotation_cv must be a finite 3x3 matrix')
    if not np.allclose(r.T @ r, np.eye(3), atol=2e-3) or np.linalg.det(r) < 0:
        raise ValueError('rotation_cv must be a proper rotation (orthonormal, det +1)')
    u, _, vt = np.linalg.svd(r)
    return u @ vt


def load_camera(source):
    """Head-camera pose (and intrinsics when given) from a head-pose JSON (path or dict); see the module doc."""
    spec = json.loads(Path(source).read_text()) if not isinstance(source, dict) else dict(source)
    origin = str(source) if not isinstance(source, dict) else 'dict'
    if 'position_m' not in spec and isinstance(spec.get('cameras'), dict) and 'front' in spec['cameras']:
        measured = not spec.get('model_derived', False) and spec.get('measured', True) is not False
        spec = dict(spec['cameras']['front'], measured=measured)
    if spec.get('schema') not in (None, CAMERA_SCHEMA):
        raise ValueError(f'Camera JSON schema must be {CAMERA_SCHEMA!r}')
    if spec.get('frame', 'arm_base') != 'arm_base':
        raise ValueError('The head camera pose must be given in the arm_base frame')
    position = np.asarray(spec.get('position_m'), float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('position_m must be three finite numbers (metres, arm_base frame)')
    return {'position_m': position, 'rotation_cv': _rotation(spec.get('rotation_cv')),
            'intrinsics': _intrinsics_from(spec), 'fovy_deg': spec.get('fovy_deg'),
            'measured': bool(spec.get('measured', True)), 'source': origin}


def model_camera(profile=PROFILE):
    """The model-derived head camera ('front') of the fold-station profile: a fallback, not a measurement."""
    cam = load_camera(json.loads(Path(profile).read_text())['cameras']['front'])
    cam.update(measured=False, source=f'{Path(profile).name} cameras.front (XLeRobot model, head tilt 58 deg; '
                                      'NOT measured)')
    return cam


def camera_matrix(intrinsics, shape, fovy_deg=None):
    """(K, dist, note) for an image of `shape`; intrinsics are scaled from their own size at the same aspect.

    Without intrinsics a 4:3 image may use `fovy_deg` (the model's OAK-D Lite 4:3 vertical field of view)."""
    h, w = shape[:2]
    if intrinsics is None:
        if fovy_deg is None or abs(w / h - 4 / 3) > .01:
            raise ValueError('No camera intrinsics: give fx, fy, cx, cy (the OAK manifest or --intrinsics); the '
                             'model field of view only describes the 4:3 full-sensor stream')
        f = h / 2 / math.tan(math.radians(float(fovy_deg)) / 2)
        return (np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.]]), np.zeros(5),
                f'model VFOV {float(fovy_deg):g} deg at {w}x{h}, centred principal point (assumed)')
    k = dict(intrinsics)
    note = 'given'
    if k.get('width') and k.get('height') and (k['width'], k['height']) != (w, h):
        if abs(k['width'] / k['height'] - w / h) > .01:
            raise ValueError(f'Intrinsics are for {k["width"]}x{k["height"]}, the image is {w}x{h} (different '
                             'aspect: a different crop of the sensor). Use intrinsics for this stream.')
        s = w / k['width']
        k.update(fx=k['fx'] * s, fy=k['fy'] * s, cx=(k['cx'] + .5) * s - .5, cy=(k['cy'] + .5) * s - .5)
        note = f'scaled from {intrinsics["width"]}x{intrinsics["height"]}'
    if not (0 < k['cx'] < w and 0 < k['cy'] < h and k['fx'] > 0 and k['fy'] > 0):
        raise ValueError(f'Intrinsics do not fit a {w}x{h} image')
    dist = np.asarray(k.get('dist') or [], float)
    return (np.array([[k['fx'], 0, k['cx']], [0, k['fy'], k['cy']], [0, 0, 1.]]),
            dist if dist.size else np.zeros(5), note)


# ---------------------------------------------------------------- pose

def _project(points, rvec, tvec, k, dist):
    return cv2.projectPoints(np.asarray(points, float), rvec, tvec, k, dist)[0].reshape(-1, 2)


def _fit(ids, detections, k, dist, seeds):
    obj = np.concatenate([tag_corners(t) for t in ids])
    img = np.concatenate([np.asarray(detections[t]['corners'], float) for t in ids])
    best = None
    for rvec, tvec in seeds:
        try:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, k, dist, np.asarray(rvec, float).reshape(3, 1).copy(),
                                              np.asarray(tvec, float).reshape(3, 1).copy())
        except cv2.error:
            continue
        r_cb = cv2.Rodrigues(rvec)[0]
        # The carton must be in front of the camera and seen from outside (above its floor).
        if (r_cb @ obj.T + tvec).T[:, 2].min() <= 0:
            continue
        err = np.linalg.norm(_project(obj, rvec, tvec, k, dist) - img, axis=1)
        rms = float(np.sqrt(np.mean(err ** 2)))
        if best is None or rms < best[2]:
            best = (rvec, tvec, rms)
    return best


def _seeds(ids, detections, k, dist, prior):
    seeds = [] if prior is None else [prior]
    obj = np.concatenate([tag_corners(t) for t in ids])
    img = np.concatenate([np.asarray(detections[t]['corners'], float) for t in ids])
    if len(ids) >= 2:
        try:
            ok, rvec, tvec = cv2.solvePnP(obj, img, k, dist, flags=cv2.SOLVEPNP_SQPNP)
            if ok:
                seeds.append((rvec, tvec))
        except cv2.error:
            pass
    for t in ids:   # both planar (IPPE) solutions of every tag, moved from the tag to the carton frame
        corners = tag_corners(t)
        centre = corners.mean(axis=0)
        x = corners[1] - corners[0]
        y = corners[0] - corners[3]
        r_bt = np.column_stack((x / np.linalg.norm(x), y / np.linalg.norm(y),
                                np.cross(x, y) / np.linalg.norm(np.cross(x, y))))
        local = (corners - centre) @ r_bt
        try:
            _, rvecs, tvecs, _ = cv2.solvePnPGeneric(local, np.asarray(detections[t]['corners'], float), k, dist,
                                                     flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            continue
        for rvec, tvec in zip(rvecs, tvecs):
            r_ct = cv2.Rodrigues(rvec)[0]
            r_cb = r_ct @ r_bt.T
            seeds.append((cv2.Rodrigues(r_cb)[0], tvec.reshape(3) - r_cb @ centre))
    return seeds


def carton_pose(detections, k, dist, prior=None):
    """Carton pose in the camera frame from the rigid box tags in `detections` ({id: {'corners': 4x2}})."""
    ids = sorted(t for t in RIGID_TAGS if t in detections)
    if not ids:
        return None
    best = _fit(ids, detections, k, dist, _seeds(ids, detections, k, dist, prior))
    if best is None:
        return None
    rvec, tvec, rms = best
    per_tag, loo = {}, {}
    for t in ids:
        corners = np.asarray(detections[t]['corners'], float)
        edge = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
        err = np.linalg.norm(_project(tag_corners(t), rvec, tvec, k, dist) - corners, axis=1)
        per_tag[t] = {'reprojection_px': round(float(np.sqrt(np.mean(err ** 2))), 2), 'edge_px': round(edge, 1),
                      'residual_mm': round(float(np.mean(err) * tag_size(t) * 1000 / edge), 2)}
        if len(ids) >= LEAVE_ONE_OUT_MIN_TAGS:
            rest = [u for u in ids if u != t]
            fit = _fit(rest, detections, k, dist, [(rvec, tvec)])
            if fit is not None:
                miss = np.linalg.norm(_project(tag_corners(t), fit[0], fit[1], k, dist) - corners, axis=1)
                # pixels -> mm in the tag's own plane, through its printed size
                loo[t] = float(np.mean(miss) * tag_size(t) * 1000 / edge)
                per_tag[t]['leave_one_out_mm'] = round(loo[t], 1)
    return {'rvec': rvec, 'tvec': tvec, 'rms_px': rms, 'tags': ids, 'per_tag': per_tag,
            'leave_one_out_mm': loo}


def to_arm_base(camera, rvec, tvec):
    r_ac, p_ac = camera['rotation_cv'], camera['position_m']
    r_cb = cv2.Rodrigues(rvec)[0]
    return r_ac @ r_cb, r_ac @ np.asarray(tvec, float).reshape(3) + p_ac


def nominal_prior(camera):
    """Training carton pose expressed in the camera frame: the solver's first seed."""
    r_ac, p_ac = camera['rotation_cv'], camera['position_m']
    p = np.array([TARGET['carton_x_m'], TARGET['near_wall_y_m'] + W / 2, TARGET['table_top_z_m']])
    return cv2.Rodrigues(r_ac.T)[0], r_ac.T @ (p - p_ac)


# ---------------------------------------------------------------- depth (second height estimate)

def depth_table_height(depth_mm, k, camera, dist=None):
    """Table-top height (arm_base z) and slope from the OAK depth image, by farm.perception.depth_scene's RANSAC
    table-plane fit. A second, carton-independent estimate; None when no table plane qualifies."""
    from farm.perception.depth_scene import fit_table_plane
    r_ac, p_ac = camera['rotation_cv'], camera['position_m']
    up_cam = r_ac.T @ np.array([0., 0., 1.])
    expected_d = float(p_ac[2] - TARGET['table_top_z_m'])
    plane = fit_table_plane(depth_mm, k, distortion=dist, expected_up_cam=up_cam, expected_d_m=expected_d)
    if not plane.get('ok'):
        return {'ok': False, 'reason': plane.get('reason')}
    n_arm = r_ac @ np.asarray(plane['normal_cam'], float)
    if n_arm[2] <= .5:
        return {'ok': False, 'reason': 'depth plane is not horizontal in the arm_base frame'}
    # Plane through the point d below the lens along -n; height under the lens: z = z_lens - d / n_z.
    z = float(p_ac[2] - plane['d_m'] / n_arm[2])
    return {'ok': True, 'table_top_z_m': z, 'gap_mm': -z * 1000,
            'tilt_deg': math.degrees(math.acos(min(1., n_arm[2]))),
            'inlier_fraction': plane.get('inlier_fraction'), 'rms_m': plane.get('rms_m')}


# ---------------------------------------------------------------- the check

def _num(v, digits=1):
    s = f'{abs(v):.{digits}f}'
    return s[:-2] if s.endswith('.0') else s


def _deg(v):
    return _num(round(abs(v) * 2) / 2)


def measure(r_ab, p_ab):
    """Station quantities (mm / deg) from the carton pose in the arm_base frame."""
    z_axis, x_axis = r_ab[:, 2], r_ab[:, 0]
    footprint = np.array([[sx * L / 2, sy * W / 2, 0.] for sx in (-1, 1) for sy in (-1, 1)]) @ r_ab.T + p_ab
    near_y = float(footprint[:, 1].min())
    return {
        'carton_centre_m': p_ab.round(5).tolist(),
        'table_gap_mm': -p_ab[2] * 1000,
        'yaw_deg': math.degrees(math.atan2(x_axis[1], x_axis[0])),
        'tilt_deg': math.degrees(math.acos(min(1., max(-1., z_axis[2])))),
        # Height change across the carton: right end minus left end (over L), far side minus near side (over W).
        'right_minus_left_mm': -z_axis[0] / z_axis[2] * L * 1000,
        'far_minus_near_mm': -z_axis[1] / z_axis[2] * W * 1000,
        'carton_x_mm': p_ab[0] * 1000,
        'near_wall_y_mm': near_y * 1000,
    }


def _check(name, value, target, tolerance, unit, instruction, training=None):
    error = value - target
    ok = bool(abs(error) <= tolerance)
    row = {'name': name, 'value': round(value, 2), 'target': target, 'error': round(error, 2),
           'tolerance': tolerance, 'unit': unit, 'ok': ok, 'instruction': 'OK' if ok else instruction(error)}
    if training is not None:
        row['training_range'] = list(training)
        row['within_training_range'] = bool(training[0] <= value <= training[1])
    return row


def checks(m, tilt_checked=True):
    t = TOLERANCE
    out = [
        _check('table height', m['table_gap_mm'], -TARGET['table_top_z_m'] * 1000, t['height_mm'], 'mm',
               lambda e: (f'Raise the table {_num(e, 0)} mm' if e > 0 else f'Lower the table {_num(e, 0)} mm')
               + f' (table top is {m["table_gap_mm"]:.0f} mm below the arm bases; want 120).'),
    ]
    lr, fn = m['right_minus_left_mm'], m['far_minus_near_mm']
    tilt_ok = bool(m['tilt_deg'] <= t['tilt_deg']) or not tilt_checked
    level = []
    if abs(lr) >= 2:
        level.append(f'its {"right" if lr < 0 else "left"} end (robot\'s view) is {_num(lr, 0)} mm lower than the '
                     f'other over the carton\'s 379 mm length: shim the {"right" if lr < 0 else "left"} side up')
    if abs(fn) >= 2:
        level.append(f'its {"far" if fn < 0 else "near"} side is {_num(fn, 0)} mm lower than the other over the '
                     f'carton\'s 283 mm depth: shim the {"far" if fn < 0 else "near"} side up')
    out.append({'name': 'table level', 'value': round(m['tilt_deg'], 2), 'target': 0., 'error': round(m['tilt_deg'], 2),
                'tolerance': t['tilt_deg'], 'unit': 'deg', 'ok': tilt_ok,
                'right_minus_left_mm': round(lr, 1), 'far_minus_near_mm': round(fn, 1), 'checked': tilt_checked,
                'instruction': ('OK' if tilt_checked else 'not checked: the visible tags are too close together')
                if tilt_ok else
                ('The carton is not level (' + '; '.join(level or ['tilted']) + '). Check the carton sits flat, '
                 'then level the table.')})
    out.append(_check('carton rotation', m['yaw_deg'], TARGET['yaw_deg'], t['yaw_deg'], 'deg',
                      lambda e: f'Turn the carton {_deg(e)} degrees {"clockwise" if e > 0 else "counter-clockwise"} '
                                '(seen from above), about its centre.', TRAINING_RANGE['yaw_deg']))
    out.append(_check('carton left/right', m['carton_x_mm'], TARGET['carton_x_m'] * 1000, t['position_mm'], 'mm',
                      lambda e: f'Move the carton {_num(e, 0)} mm to the robot\'s {"left" if e > 0 else "right"} '
                                '(its centre belongs 10 mm left of the midpoint between the arms).',
                      TRAINING_RANGE['carton_x_mm']))
    out.append(_check('carton distance', m['near_wall_y_mm'], TARGET['near_wall_y_m'] * 1000, t['position_mm'], 'mm',
                      lambda e: f'Move the carton {_num(e, 0)} mm {"toward" if e > 0 else "away from"} the robot '
                                '(its near wall belongs 160 mm in front of the arm bases\' line, 10 mm in from the '
                                'table edge).'))
    return out


def check_frame(bgr, camera, k, dist, detections=None, depth_mm=None):
    """Full check of one BGR frame. `camera`: load_camera()/model_camera(); k, dist: OpenCV intrinsics for the
    frame. Returns the report dict (see `format_report`)."""
    if detections is None:
        from carton.servo.features import tags_from_bgr
        detections = tags_from_bgr(bgr)
    report = {'schema': SCHEMA, 'ok': False, 'status': 'CANNOT_CHECK', 'image_size': [bgr.shape[1], bgr.shape[0]],
              'camera': {'source': camera['source'], 'measured': camera['measured'],
                         'position_m': np.round(camera['position_m'], 5).tolist()},
              'target': TARGET, 'tolerance': TOLERANCE, 'training_range': TRAINING_RANGE,
              'checks': [], 'instructions': [], 'warnings': []}
    if not camera['measured']:
        report['warnings'].append('Head-camera pose is the XLeRobot model\'s, not measured: a 1 degree tilt error '
                                  'moves the answers by about 8 mm. Measure it with the head-pose tool and pass '
                                  '--camera-json.')
    good = {t: d for t, d in detections.items()
            if d.get('hamming', 0) == 0 and d.get('margin', 100) >= 30
            and min(np.linalg.norm(np.asarray(d['corners']) - np.roll(np.asarray(d['corners']), 1, axis=0),
                                   axis=1)) >= MIN_EDGE_PX}
    rigid = sorted(t for t in good if t in RIGID_TAGS)
    report['tags'] = {'seen': sorted(detections), 'rigid_used': rigid,
                      'flap_seen': sorted(t for t in good if t in FLAP_TAGS),
                      'other': sorted(t for t in detections if t not in RIGID_TAGS and t not in FLAP_TAGS)}
    faces = {RIGID_TAGS[t][0] for t in rigid}
    if len(rigid) < MIN_RIGID_TAGS:
        report['instructions'].append(
            f'Cannot check: need at least {MIN_RIGID_TAGS} box tags (near wall 26/10/27, end walls 21/28 and 22, '
            f'inside floor 25/24); seen {rigid or "none"}. Put the tagged training carton in view with its flaps '
            'standing up, and the head at the policy pose.')
        return report
    pose = carton_pose(good, k, dist, nominal_prior(camera))
    if pose is None:
        report['instructions'].append('Cannot check: no consistent carton pose from the tags.')
        return report
    r_ab, p_ab = to_arm_base(camera, pose['rvec'], pose['tvec'])
    m = measure(r_ab, p_ab)
    report['pose'] = {'rms_reprojection_px': round(pose['rms_px'], 3), 'per_tag': pose['per_tag'],
                      'rotation_carton_to_arm_base': r_ab.round(6).tolist(),
                      'faces': sorted(faces)}
    report['measured'] = {k_: (round(v, 2) if isinstance(v, float) else v) for k_, v in m.items()}
    # Flap tags: compare with where an upright flap would put them.
    flaps = {}
    for t in report['tags']['flap_seen']:
        corners = np.asarray(good[t]['corners'], float)
        edge = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
        predicted = _project(FLAP_TAGS[t][1:2], pose['rvec'], pose['tvec'], k, dist)[0]
        off = float(np.linalg.norm(predicted - corners.mean(axis=0)) * FLAP_TAG_M * 1000 / edge)
        flaps[t] = {'flap': FLAP_TAGS[t][0], 'off_upright_mm': round(off, 1), 'upright': bool(off <= FLAP_NOT_UP_MM)}
    report['flaps'] = flaps
    misfit = {t: max(v['residual_mm'] / GEOMETRY_RESIDUAL_MM, v.get('leave_one_out_mm', 0.) / GEOMETRY_LEAVE_ONE_OUT_MM)
              for t, v in pose['per_tag'].items()}
    worst = max(misfit, key=misfit.get)
    worst_mm = max(pose['per_tag'][worst]['residual_mm'], pose['per_tag'][worst].get('leave_one_out_mm', 0.))
    implausible = m['tilt_deg'] > 15 or not 0 < m['table_gap_mm'] < 400
    report['geometry'] = {'worst_tag': worst, 'worst_misfit_mm': round(worst_mm, 1),
                          'residual_limit_mm': GEOMETRY_RESIDUAL_MM,
                          'leave_one_out_limit_mm': GEOMETRY_LEAVE_ONE_OUT_MM,
                          'consistent': bool(misfit[worst] <= 1 and not implausible), 'faces': sorted(faces)}
    if not report['geometry']['consistent']:
        report['status'] = 'MISMATCH'
        report['instructions'].append(
            'The visible box does not match the 379 x 283 x 108 mm training carton'
            + (f' (tag {worst} is {worst_mm:.0f} mm from where the training geometry and the other tags put it)'
               if misfit[worst] > 1 else '')
            + ('; the fitted carton is implausible' if implausible else '')
            + '. Use the training carton, flaps standing up, with the 12 tags placed as in setup steps 4c-4g, '
              'then re-run. The measurements are not trustworthy.')
    elif len(faces) < 2:
        report['warnings'].append(f'Only one face of the carton is tagged in view ({next(iter(faces))}): the tag '
                                  'geometry cannot be cross-checked, so a different box would go unnoticed.')
    centres = np.array([tag_corners(t).mean(axis=0) for t in rigid])
    span = float(max(np.linalg.norm(a - b) for a in centres for b in centres))
    report['checks'] = checks(m, tilt_checked=span >= TILT_MIN_SPAN_M)
    if depth_mm is not None:
        report['depth_table'] = depth_table_height(depth_mm, k, camera, dist)
        d = report['depth_table']
        if d.get('ok') and abs(d['gap_mm'] - m['table_gap_mm']) > DEPTH_DISAGREE_MM:
            report['warnings'].append(f'Depth plane puts the table {d["gap_mm"]:.0f} mm below the arm bases, the '
                                      f'carton tags {m["table_gap_mm"]:.0f} mm: check the carton sits flat on the '
                                      'table and the head-camera pose.')
    if report['status'] != 'MISMATCH':
        report['instructions'] += [c['instruction'] for c in report['checks'] if not c['ok']]
        not_up = [f for f in flaps.values() if not f['upright']]
        report['instructions'] += [f'Stand the {f["flap"]} straight up (its tag is {f["off_upright_mm"]:.0f} mm '
                                   'from the upright place).' for f in not_up]
        if len(faces) < 2:
            if 'floor' not in faces:
                report['instructions'].append('Make the inside floor tags 25/24 visible to the head camera (stand '
                                              'the flaps up, especially the near long flap) and re-run to confirm '
                                              'the carton.')
            else:
                report['instructions'].append('Bring the near-wall tags 26/10/27 into the head camera\'s view (aim '
                                              'the head a little lower, keep the near wall uncovered) and re-run to '
                                              'confirm the carton.')
        report['ok'] = not report['instructions']
        report['status'] = 'OK' if report['ok'] else 'ADJUST'
        if report['ok']:
            report['instructions'] = ['OK']
        outside = [c['name'] for c in report['checks'] if c.get('within_training_range') is False]
        if outside:
            report['warnings'].append('Outside the range the policy was trained on: ' + ', '.join(outside) + '.')
    return report


def format_report(report):
    lines = [f'Station check: {report["status"]}']
    cam = report['camera']
    lines.append(f'Head camera: {cam["source"]}' + ('' if cam['measured'] else '  [UNMEASURED FALLBACK]'))
    tags = report.get('tags', {})
    if tags:
        lines.append(f'Box tags used: {tags["rigid_used"] or "none"}'
                     + (f'  (reprojection {report["pose"]["rms_reprojection_px"]:.2f} px RMS)'
                        if report.get('pose') else '')
                     + (f'; flap tags seen: {tags["flap_seen"]}' if tags.get('flap_seen') else ''))
    if report['checks'] and report['status'] == 'MISMATCH':
        lines.append('Measurements (NOT trustworthy: the tags do not fit the training carton):')
    for c in report['checks']:
        unit = ' ' + c['unit'] if c['unit'] == 'mm' else ' deg'
        extra = (f', training range {c["training_range"][0]:g}..{c["training_range"][1]:g}'
                 if 'training_range' in c else '')
        lines.append(f'  {c["name"]:<18} {c["value"]:8.1f}{unit} (target {c["target"]:g} +/-{c["tolerance"]:g}'
                     f'{extra}): {"not checked" if c.get("checked") is False else "OK" if c["ok"] else "ADJUST"}')
    d = report.get('depth_table')
    if d:
        lines.append(f'  depth-plane gap    ' + (f'{d["gap_mm"]:8.1f} mm (second estimate, not used for the '
                                                  'instructions)' if d.get('ok') else f'unavailable: {d["reason"]}'))
    for w in report['warnings']:
        lines.append(f'Warning: {w}')
    lines.append('Do this, then re-run the check:' if report['status'] == 'ADJUST' else 'Result:')
    lines += [f'  {i + 1}. {s}' if report['status'] == 'ADJUST' else f'  {s}'
              for i, s in enumerate(report['instructions'])]
    return '\n'.join(lines)


__all__ = ['SCHEMA', 'CAMERA_SCHEMA', 'TARGET', 'TOLERANCE', 'TRAINING_RANGE', 'RIGID_TAGS', 'FLAP_TAGS',
           'tag_corners', 'load_camera', 'model_camera', 'camera_matrix', 'carton_pose', 'to_arm_base',
           'depth_table_height', 'measure', 'checks', 'check_frame', 'format_report']
