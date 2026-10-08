"""Numeric 3D positions from the OAK head depth image, in the robot frame. Pure functions, no I/O.

The depth image is the OAK's stereo depth aligned to the RGB camera (640x360, uint16 millimetres,
0 = invalid). With the RGB intrinsics every valid pixel back-projects to a point in the camera's
optical frame (+x image-right, +y image-down, +z out of the lens, metres); the camera pose from
``farm.sim.xlerobot_twin.camera_pose`` (position and a rotation whose columns are the optical axes in
the robot frame) carries it into the ROBOT frame: origin on the floor below the midpoint of the
shoulder-pan axes, +forward the robot's front, +left, +up, metres.

``scene_points`` summarises an image for an LLM supervisor that cannot read a PNG: a 5x3 grid of
region distances, the nearest coherent object (pixels within ``nearest_band_m`` of the 2nd-percentile
range, largest connected blob), points at caller-given pixels (5x5 median), and how much of the image
is invalid. "distance" is always the straight-line distance from the camera lens to the point (range),
not the axial depth; the per-axis forward/left/up values are the robot-frame coordinates.

Lens distortion: when distortion coefficients are given and OpenCV is importable, pixel coordinates
are undistorted (``cv2.undistortPoints``) before back-projection; otherwise the pinhole model is used
and the result says so (``undistorted`` False, ``undistortion`` reason). The OAK stream manifest says
``projection: rectified_pinhole`` when the RGB (and so the aligned depth) was already undistorted by
the factory mesh; pass no coefficients in that case.

Stereo depth is blind closer than about 0.2-0.3 m and on textureless or blown-out surfaces: those
pixels are 0. A mostly-invalid image centre usually means something is closer than that minimum.

Self-calibration on the table plane: the model camera pose (head tick mapping and optical site) is
unvalidated and on 8 October put a box top 21 cm too high. ``fit_table_plane`` finds the dominant plane
in the depth image by RANSAC (in the camera frame); ``calibrate_camera_pose`` then keeps the model's
heading and forward/left position but replaces the pitch and roll so that plane normal maps to robot
+up, and shifts the camera height so the plane lies at the owner's measured ``table_top_m``. Heights
and forward distances of scene points then no longer depend on the head-tilt model; only the heading
(pan) and the lens's forward offset still do.
"""
from __future__ import annotations

import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - exercised by monkeypatching cv2 to None in the tests
    cv2 = None

GRID_COLUMNS = ('left', 'centre-left', 'centre', 'centre-right', 'right')   # image left .. image right
GRID_ROWS = ('top', 'middle', 'bottom')
NEAREST_BAND_M = 0.08
NEAREST_PERCENTILE = 2.0
MAX_DEPTH_MM = 10000          # beyond 10 m the OAK stereo returns noise; treated as invalid
MIN_DEPTH_MM = 1
QUERY_PATCH = 5               # odd; the median over the valid pixels of this square around a query pixel
QUERY_MIN_VALID = 5           # of the 25 patch pixels
STEREO_BLIND_M = 0.25
CENTRE_BLIND_FRACTION = 0.5
DISTANCE_DEFINITION = 'straight-line metres from the camera lens to the point (range), not axial depth'
FRAME = ('Robot frame: origin on the floor directly below the midpoint between the two shoulder-pan axes; '
         '+forward_m the robot\'s front, +left_m the robot\'s left, +up_m height above the floor; metres. '
         'Image left is the robot\'s left when the head is at zero pan.')

PLANE_MIN_INLIER_FRACTION = 0.15   # of the valid, in-range points
PLANE_MIN_IMAGE_FRACTION = 0.10    # the plane must also cover this much of the whole image
PLANE_MAX_ROLL_DEG = 10.0          # the head has pan and tilt, no roll axis: a larger roll means a wrong plane
PLANE_MIN_POINTS = 300             # fewer valid in-range points than this: no fit
PLANE_MAX_POINTS = 4000            # RANSAC subsample
PLANE_MAX_ANGLE_DEG = 35.0         # largest tilt/roll correction the calibration accepts
PLANE_GATE_DEG = PLANE_MAX_ANGLE_DEG  # candidate normals farther than this from the expected up are not a table
PLANE_MAX_HEIGHT_M = 0.25          # largest camera height correction it accepts
UP = np.array([0.0, 0.0, 1.0])     # robot +up

_NORMALIZED_CACHE = {}  # (shape, intrinsics, distortion) -> (xn, yn); at most two entries


# ---------------------------------------------------------------- validation

def check_intrinsics(intrinsics):
    """3x3 pinhole matrix with positive focal lengths -> (fx, fy, cx, cy); ValueError otherwise."""
    k = np.asarray(intrinsics, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('intrinsics must be a finite 3x3 matrix with positive fx and fy')
    if not np.allclose(k[2], [0, 0, 1]) or k[1, 0] != 0:
        raise ValueError('intrinsics must be a pinhole matrix [[fx,0,cx],[0,fy,cy],[0,0,1]]')
    return float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])


def check_depth(depth_mm):
    """2-D integer or float array of millimetres -> float32 metres with NaN where invalid (0, negative, > 10 m)."""
    depth = np.asarray(depth_mm)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError('depth_mm must be a non-empty 2-D array (rows x columns) of millimetres')
    if depth.dtype.kind == 'f' and not np.isfinite(depth).all():
        depth = np.where(np.isfinite(depth), depth, 0)
    valid = (depth >= MIN_DEPTH_MM) & (depth <= MAX_DEPTH_MM)
    metres = np.where(valid, depth.astype(np.float64) / 1000.0, np.nan)
    return metres, valid


def check_pose(cam_pose):
    """{'position_m': [f,l,u], 'rotation': 3x3 with columns = optical x,y,z in the robot frame} -> (pos, R)."""
    if not isinstance(cam_pose, dict):
        raise ValueError('cam_pose must be a dict with position_m and rotation')
    pos = np.asarray(cam_pose.get('position_m'), dtype=float)
    rot = np.asarray(cam_pose.get('rotation'), dtype=float)
    if pos.shape != (3,) or not np.isfinite(pos).all():
        raise ValueError('cam_pose.position_m must be three finite metres [forward, left, up]')
    if rot.shape != (3, 3) or not np.isfinite(rot).all():
        raise ValueError('cam_pose.rotation must be a 3x3 matrix')
    if not np.allclose(rot @ rot.T, np.eye(3), atol=1e-4) or np.linalg.det(rot) < 0.99:
        raise ValueError('cam_pose.rotation must be a proper rotation (orthonormal, determinant +1)')
    return pos, rot


def _distortion(distortion):
    if distortion is None:
        return None
    d = np.asarray(distortion, dtype=float).ravel()
    if d.size not in (4, 5, 8, 12, 14) or not np.isfinite(d).all():
        raise ValueError('distortion must be 4, 5, 8, 12 or 14 finite OpenCV coefficients')
    return d


# ---------------------------------------------------------------- rays and back-projection

def normalized_rays(shape, intrinsics, distortion=None):
    """Per-pixel normalised image coordinates (xn, yn) at pixel centres: the ray through each pixel is
    (xn, yn, 1). Returns (xn, yn, undistorted: bool, reason: str). Cached for the last two layouts."""
    fx, fy, cx, cy = check_intrinsics(intrinsics)
    dist = _distortion(distortion)
    height, width = int(shape[0]), int(shape[1])
    key = ((height, width), (fx, fy, cx, cy), None if dist is None else tuple(dist.tolist()), cv2 is None)
    hit = _NORMALIZED_CACHE.get(key)
    if hit is not None:
        return hit
    us, vs = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    if dist is None or not dist.any():
        xn, yn = (us - cx) / fx, (vs - cy) / fy
        undistorted, reason = False, ('not needed: distortion coefficients absent or all zero, pixels treated as a '
                                      'rectified pinhole image')
    elif cv2 is None:
        xn, yn = (us - cx) / fx, (vs - cy) / fy
        undistorted, reason = False, ('skipped: OpenCV (cv2) is not installed, so off-centre positions carry the '
                                      'lens distortion')
    else:
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        pts = np.stack([us.ravel(), vs.ravel()], axis=1).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, k, dist).reshape(height, width, 2)
        xn, yn = np.ascontiguousarray(norm[..., 0]), np.ascontiguousarray(norm[..., 1])
        undistorted, reason = True, 'pixel coordinates undistorted with cv2.undistortPoints before back-projection'
    result = (xn, yn, undistorted, reason)
    if len(_NORMALIZED_CACHE) >= 2:
        _NORMALIZED_CACHE.pop(next(iter(_NORMALIZED_CACHE)))
    _NORMALIZED_CACHE[key] = result
    return result


def backproject(depth_mm, intrinsics, pixels, distortion=None):
    """Camera-optical-frame points (N, 3) in metres for integer pixels (N, 2) [x, y], using the depth at
    each pixel: x = xn * z, y = yn * z (+x image-right, +y image-down, +z out of the lens). Rows are NaN
    where the depth is invalid or the pixel is outside the image."""
    depth_m, valid = check_depth(depth_mm)
    px = np.asarray(pixels, dtype=float)
    if px.ndim == 1 and px.size == 2:
        px = px.reshape(1, 2)
    if px.ndim != 2 or px.shape[1] != 2:
        raise ValueError('pixels must be an (N, 2) array of [x, y]')
    xn, yn, _, _ = normalized_rays(depth_m.shape, intrinsics, distortion)
    height, width = depth_m.shape
    out = np.full((len(px), 3), np.nan)
    if not np.isfinite(px).all():
        return out
    xi, yi = np.rint(px[:, 0]).astype(int), np.rint(px[:, 1]).astype(int)
    inside = (xi >= 0) & (xi < width) & (yi >= 0) & (yi < height)
    xi_c, yi_c = xi[inside], yi[inside]
    z = depth_m[yi_c, xi_c]
    out[inside, 0] = xn[yi_c, xi_c] * z
    out[inside, 1] = yn[yi_c, xi_c] * z
    out[inside, 2] = z
    return out


def camera_to_robot(points_cam, cam_pose):
    """Optical-frame points (..., 3) -> robot-frame points (..., 3) [forward, left, up]."""
    pos, rot = check_pose(cam_pose)
    return np.asarray(points_cam, dtype=float) @ rot.T + pos


# ---------------------------------------------------------------- table plane self-calibration

def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _valid_points_cam(depth_mm, intrinsics, distortion, min_range_m, max_range_m):
    """(points (N, 3) in the optical frame, number of valid pixels) for valid pixels within the range window."""
    depth_m, valid = check_depth(depth_mm)
    xn, yn, _, _ = normalized_rays(depth_m.shape, intrinsics, distortion)
    z = np.where(valid, depth_m, 0.0)
    pts = np.stack([xn * z, yn * z, z], axis=-1)[valid]
    rng = np.linalg.norm(pts, axis=1)
    return pts[(rng >= min_range_m) & (rng <= max_range_m)], int(valid.sum())


def _plane_fail(reason, **extra):
    out = {'ok': False, 'reason': reason, 'normal_cam': None, 'd_m': None, 'inlier_fraction': 0.0, 'inliers': 0}
    out.update(extra)
    return out


def fit_table_plane(depth_mm, intrinsics, *, distortion=None, min_range_m=0.3, max_range_m=2.0, ransac_iters=200,
                    inlier_m=0.01, seed=0, expected_up_cam=None, expected_d_m=None,
                    min_inlier_fraction=PLANE_MIN_INLIER_FRACTION, min_image_fraction=PLANE_MIN_IMAGE_FRACTION,
                    max_points=PLANE_MAX_POINTS):
    """The dominant plane of the depth image, in the camera optical frame, by RANSAC on a subsample of the valid
    back-projected points between ``min_range_m`` and ``max_range_m`` from the lens.

    Returns {'ok', 'reason', 'normal_cam': [3] (unit, pointing from the plane toward the camera, i.e. the table's
    up), 'd_m' (the lens's perpendicular distance above the plane: normal . p + d = 0 on the plane), 'inlier_fraction'
    (of the sampled points, within ``inlier_m`` of the plane), 'inliers', 'image_fraction' (of all pixels),
    'points' (sampled), 'valid_pixels', 'rms_m', 'inlier_median_cam' [3], 'inlier_median_range_m',
    'angle_from_expected_up_deg' (when ``expected_up_cam``, the robot's up axis in camera coordinates from the
    model pose, is given), 'candidates'}.
    The plane must hold at least ``min_inlier_fraction`` of the sampled points and cover ``min_image_fraction`` of
    the image (a sleeve or a hand right under the lens holds many of the few valid points but little of the
    image). With ``expected_up_cam`` the candidates whose normal is more than ``PLANE_GATE_DEG`` from it (walls, the
    floor seen edge-on) are skipped; with ``expected_d_m`` (the model's lens height above the table) candidates
    farther than ``PLANE_MAX_HEIGHT_M`` from that distance (the floor, a box top, the robot's own arm under the
    lens) are skipped too. The chosen plane must also hold at least half as many points as the largest plane in
    the image, whatever its orientation (otherwise something other than the table dominates the view). With the
    distance prior, the LOWEST remaining plane (largest ``d_m``) wins: the table lies below the boxes on it, and the
    floor is outside the prior; without it the largest remaining plane wins, near-ties (within 10 %) going to the
    lowest. ``ok`` False with a ``reason`` when nothing qualifies.
    """
    if not (0 < inlier_m < 0.2) or ransac_iters < 1 or not (0 <= min_range_m < max_range_m):
        raise ValueError('fit_table_plane: inlier_m in (0, 0.2), ransac_iters >= 1 and 0 <= min_range_m < max_range_m')
    pts, valid_pixels = _valid_points_cam(depth_mm, intrinsics, distortion, min_range_m, max_range_m)
    image_pixels = int(np.asarray(depth_mm).shape[0] * np.asarray(depth_mm).shape[1])
    in_range = len(pts)
    if in_range < PLANE_MIN_POINTS:
        return _plane_fail(f'too few points: {in_range} valid depth pixels between {min_range_m} and {max_range_m} m '
                           f'(need {PLANE_MIN_POINTS}); the stereo is blind closer than about {STEREO_BLIND_M} m',
                           points=in_range, valid_pixels=valid_pixels, image_fraction=0.0)
    rs = np.random.RandomState(seed)
    if in_range > max_points:
        pts = pts[rs.choice(in_range, int(max_points), replace=False)]
    count = len(pts)
    expected = _unit(expected_up_cam) if expected_up_cam is not None else None
    # Inputs are finite (check_depth); Apple's Accelerate BLAS still raises spurious divide/overflow flags on matmul.
    with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
        result = _fit_plane(pts, count, valid_pixels, rs, expected, expected_d_m, int(ransac_iters), float(inlier_m),
                            float(min_inlier_fraction))
    # inliers among the sampled points scale to the whole image by the in-range count
    result['image_fraction'] = float(result.get('inliers', 0)) / count * in_range / image_pixels
    if result['ok'] and result['image_fraction'] < min_image_fraction:
        result.update(ok=False, reason=(f'plane covers only {result["image_fraction"]:.0%} of the image (need '
                                        f'{min_image_fraction:.0%}): {result["inlier_fraction"]:.0%} of the valid '
                                        'points, but most of the image has no depth or is out of range (something '
                                        'close to the lens, the robot\'s own arm?)'))
    return result


def _fit_plane(pts, count, valid_pixels, rs, expected, expected_d, ransac_iters, inlier_m, min_inlier_fraction):
    """fit_table_plane's RANSAC and refinement on the sampled points (N, 3)."""
    # RANSAC: every triple at once
    idx = rs.randint(0, count, size=(int(ransac_iters), 3))
    p0, p1, p2 = pts[idx[:, 0]], pts[idx[:, 1]], pts[idx[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    lengths = np.linalg.norm(normals, axis=1)
    usable = lengths > 1e-9
    normals[usable] /= lengths[usable, None]
    offsets = -np.einsum('ij,ij->i', normals, p0)
    # orient toward the camera: the lens is on the positive side (d > 0)
    flip = offsets < 0
    normals[flip] *= -1
    offsets[flip] *= -1
    inlier_counts = (np.abs(pts @ normals.T + offsets) < inlier_m).sum(axis=0)
    inlier_counts[~usable] = 0
    if expected is not None:
        angles = np.degrees(np.arccos(np.clip(normals @ expected, -1.0, 1.0)))
        gate = usable & (angles <= PLANE_GATE_DEG)
    else:
        angles = np.full(len(normals), np.nan)
        gate = usable
    if expected_d is not None:
        gate &= np.abs(offsets - float(expected_d)) <= PLANE_MAX_HEIGHT_M
    need = int(math.ceil(min_inlier_fraction * count))
    best_any = int(inlier_counts.argmax())
    top_any = int(inlier_counts[best_any])
    table_like = gate & (inlier_counts >= need)
    # a table-like plane must also be a major plane of the image: at least half the size of the largest one
    qualified = table_like & (inlier_counts >= 0.5 * top_any)
    if not qualified.any():
        fail = dict(points=count, valid_pixels=valid_pixels, angle_from_expected_up_deg=float(angles[best_any]),
                    d_m=float(offsets[best_any]), inlier_fraction=float(top_any / count), inliers=top_any,
                    candidates=0)
        if top_any < need:
            return _plane_fail(f'no plane: the best candidate holds {top_any / count:.0%} of the {count} points '
                               f'(need {min_inlier_fraction:.0%})', **fail)
        why = []
        if expected is not None and angles[best_any] > PLANE_GATE_DEG:
            why.append(f'its normal is {angles[best_any]:.0f} deg from the expected up (a wall or the floor edge-on?)')
        if expected_d is not None and abs(offsets[best_any] - expected_d) > PLANE_MAX_HEIGHT_M:
            why.append(f'it is {offsets[best_any]:.2f} m from the lens where the table should be about '
                       f'{expected_d:.2f} m (a box top, the floor or the robot\'s own arm?)')
        if table_like.any():
            runner = int(np.flatnonzero(table_like)[np.argmax(inlier_counts[table_like])])
            why.append(f'the largest table-like plane holds only {inlier_counts[runner] / count:.0%} of the points '
                       f'(the dominant plane has {top_any / count:.0%})')
        return _plane_fail(f'dominant plane is not table-like ({top_any / count:.0%} of the points, '
                           f'{offsets[best_any]:.2f} m from the lens): ' + ' and '.join(why), **fail)
    # with the distance prior the lowest qualified plane is the table (boxes sit above it); without it, the largest
    share = 0.5 if expected_d is not None else 0.9
    strong = qualified & (inlier_counts >= share * inlier_counts[qualified].max())
    best = int(np.flatnonzero(strong)[np.argmax(offsets[strong])])

    # refine by least squares on the inliers (twice)
    normal, d = normals[best], float(offsets[best])
    for _ in range(2):
        inliers = np.abs(pts @ normal + d) < inlier_m
        if inliers.sum() < 3:
            break
        centroid = pts[inliers].mean(axis=0)
        _, _, vt = np.linalg.svd(pts[inliers] - centroid, full_matrices=False)
        normal = vt[-1]
        d = -float(normal @ centroid)
        if d < 0:
            normal, d = -normal, -d
    residual = pts @ normal + d
    inliers = np.abs(residual) < inlier_m
    n_in = int(inliers.sum())
    angle = float(np.degrees(np.arccos(np.clip(normal @ expected, -1.0, 1.0)))) if expected is not None else None
    if n_in < need or (angle is not None and angle > PLANE_GATE_DEG):
        return _plane_fail(f'plane lost in refinement: {n_in / count:.0%} inliers' +
                           (f', normal {angle:.0f} deg from the expected up' if angle is not None else ''),
                           points=count, valid_pixels=valid_pixels, inlier_fraction=n_in / count, inliers=n_in,
                           angle_from_expected_up_deg=angle, candidates=int(qualified.sum()))
    median_cam = np.median(pts[inliers], axis=0)
    return {
        'ok': True,
        'reason': f'plane with {n_in / count:.0%} of {count} sampled points within {inlier_m * 1000:.0f} mm',
        'normal_cam': [float(v) for v in normal], 'd_m': d,
        'inlier_fraction': n_in / count, 'inliers': n_in, 'points': count, 'valid_pixels': valid_pixels,
        'rms_m': float(np.sqrt(np.mean(residual[inliers] ** 2))),
        'inlier_median_cam': [float(v) for v in median_cam],
        'inlier_median_range_m': float(np.median(np.linalg.norm(pts[inliers], axis=1))),
        'angle_from_expected_up_deg': angle, 'candidates': int(qualified.sum()), 'inlier_m': float(inlier_m),
    }


def tilt_deg(rotation):
    """Pitch of the optical axis below horizontal, degrees (positive = looking down)."""
    rot = np.asarray(rotation, dtype=float)
    return float(np.degrees(np.arcsin(np.clip(-rot[2, 2], -1.0, 1.0))))


def roll_deg(rotation):
    """Roll of the image about the optical axis, degrees: the signed angle from the horizontal image-right
    direction to the actual image-right axis, measured about the viewing direction (positive = clockwise seen from
    behind the camera, image-right dipping). 0.0 when the camera looks straight up or down (roll undefined)."""
    rot = np.asarray(rotation, dtype=float)
    x, z = rot[:, 0], rot[:, 2]
    level_right = np.cross(z, UP)
    if np.linalg.norm(level_right) < 1e-6:
        return 0.0
    level_right /= np.linalg.norm(level_right)
    return float(np.degrees(np.arctan2(np.cross(level_right, x) @ z, level_right @ x)))


def _heading(rot):
    """Unit horizontal vector the camera points along (its optical axis projected onto the floor; image-up's
    projection when the camera looks almost straight down)."""
    for axis in (rot[:, 2], -rot[:, 1]):
        h = np.array([axis[0], axis[1], 0.0])
        if np.linalg.norm(h) > 0.2:
            return h / np.linalg.norm(h)
    return np.array([1.0, 0.0, 0.0])


def calibrate_camera_pose(plane, model_pose, table_top_m, *, max_angle_deg=PLANE_MAX_ANGLE_DEG,
                          max_height_m=PLANE_MAX_HEIGHT_M, max_roll_deg=PLANE_MAX_ROLL_DEG):
    """A camera pose corrected by the table plane: the model's heading and forward/left position are kept; pitch
    and roll are replaced so the plane normal maps to robot +up; the height is shifted so the plane lies at
    ``table_top_m`` (metres above the floor, measured by the owner).

    Returns the model pose's fields plus {'ok', 'method': 'table_plane' | 'model', 'reason', 'tilt_correction_deg',
    'roll_correction_deg', 'angle_correction_deg', 'height_correction_m', 'tilt_deg', 'roll_deg', 'model_tilt_deg',
    'model_roll_deg', 'camera_above_table_m', 'table_top_m', 'inlier_fraction', 'inliers', 'model_position_m',
    'model_rotation'}. When the plane fit failed or a correction exceeds ``max_angle_deg`` / ``max_height_m`` /
    ``max_roll_deg`` (the head has no roll axis, so a big roll means the plane is not the table) the pose is the
    MODEL pose unchanged, ``ok`` False and ``method`` 'model' with the reason (the proposed corrections are still
    reported).
    """
    pos, rot = check_pose(model_pose)
    table_top = float(table_top_m)
    if not (0.0 < table_top < 2.0):
        raise ValueError('table_top_m must be a height above the floor between 0 and 2 m')
    out = dict(model_pose)
    out.update(method='model', ok=False, table_top_m=table_top, model_tilt_deg=round(tilt_deg(rot), 2),
               model_roll_deg=round(roll_deg(rot), 2), model_position_m=[float(v) for v in pos],
               model_rotation=[[float(v) for v in row] for row in rot],
               inlier_fraction=float((plane or {}).get('inlier_fraction') or 0.0),
               inliers=int((plane or {}).get('inliers') or 0),
               tilt_correction_deg=None, roll_correction_deg=None, angle_correction_deg=None,
               height_correction_m=None, camera_above_table_m=None)
    if not isinstance(plane, dict) or not plane.get('ok') or plane.get('normal_cam') is None:
        out['reason'] = 'no table plane: ' + str((plane or {}).get('reason') or 'plane fit missing')
        return out
    normal = _unit(plane['normal_cam'])
    d = float(plane['d_m'])
    if not np.isfinite(normal).all() or abs(np.linalg.norm(normal) - 1) > 1e-6 or not (0 < d < 5):
        out['reason'] = 'table plane is malformed (normal not a unit vector or distance out of range)'
        return out

    # rotation: plane normal -> +up, heading kept
    forward_cam = np.array([0.0, 0.0, 1.0]) - normal[2] * normal   # optical axis made perpendicular to the normal
    if np.linalg.norm(forward_cam) < 0.2:                           # looking almost straight down the normal
        image_up = np.array([0.0, -1.0, 0.0])
        forward_cam = image_up - (image_up @ normal) * normal
    forward_cam = _unit(forward_cam)
    heading = _heading(rot)
    cam_basis = np.stack([forward_cam, np.cross(normal, forward_cam), normal], axis=1)
    robot_basis = np.stack([heading, np.cross(UP, heading), UP], axis=1)
    new_rot = robot_basis @ cam_basis.T
    angle = float(np.degrees(np.arccos(np.clip((rot @ normal) @ UP, -1.0, 1.0))))
    tilt_new, roll_new = tilt_deg(new_rot), roll_deg(new_rot)
    # height: under new_rot the plane sits at up = pos_up - d; move the lens so that is table_top
    new_up = table_top + d
    height_correction = new_up - float(pos[2])
    out.update(tilt_correction_deg=round(tilt_new - out['model_tilt_deg'], 2),
               roll_correction_deg=round(roll_new - out['model_roll_deg'], 2),
               angle_correction_deg=round(angle, 2), height_correction_m=round(height_correction, 4),
               tilt_deg=round(tilt_new, 2), roll_deg=round(roll_new, 2), camera_above_table_m=round(d, 4))
    if angle > max_angle_deg:
        out['reason'] = (f'table plane rejected: it would turn the camera by {angle:.1f} deg (limit {max_angle_deg:g}); '
                         'the dominant plane is probably not the table, or the head mapping is badly off')
        return out
    if abs(height_correction) > max_height_m:
        out['reason'] = (f'table plane rejected: it would move the camera {height_correction * 100:+.0f} cm in height '
                         f'(limit {max_height_m * 100:.0f} cm); the dominant plane is probably not the table top at '
                         f'{table_top:.2f} m, or table_top_m is wrong')
        return out
    if abs(roll_new - out['model_roll_deg']) > max_roll_deg:
        out['reason'] = (f'table plane rejected: it would roll the camera by {roll_new - out["model_roll_deg"]:+.1f} deg '
                         f'(limit {max_roll_deg:g}); the head has pan and tilt but no roll axis, so the dominant plane '
                         'is probably not the table')
        return out
    out.update(ok=True, method='table_plane',
               position_m=[float(pos[0]), float(pos[1]), float(new_up)],
               rotation=[[float(v) for v in row] for row in new_rot],
               reason=(f'tilt {out["model_tilt_deg"]:.1f} -> {tilt_new:.1f} deg, roll {out["model_roll_deg"]:.1f} -> '
                       f'{roll_new:.1f} deg, lens height {pos[2]:.3f} -> {new_up:.3f} m so the fitted plane '
                       f'({plane.get("inlier_fraction", 0):.0%} of points) lies at {table_top:.2f} m; heading and '
                       'forward/left offset from the model'))
    return out


def table_check(depth_mm, intrinsics, cam_pose, plane, *, distortion=None, min_range_m=0.3, max_range_m=2.0):
    """Where the fitted plane's pixels land in the robot frame under ``cam_pose``: {'median_up_m' (equals
    table_top_m by construction after calibration), 'pixels', 'fraction' (of the image), 'forward_range_m' [5th,
    95th percentile], 'left_range_m' [5th, 95th]}; None when the plane is not ok."""
    if not isinstance(plane, dict) or not plane.get('ok'):
        return None
    pts, _ = _valid_points_cam(depth_mm, intrinsics, distortion, min_range_m, max_range_m)
    normal, d = _unit(plane['normal_cam']), float(plane['d_m'])
    with np.errstate(divide='ignore', over='ignore', invalid='ignore'):   # spurious Accelerate matmul flags
        on_plane = np.abs(pts @ normal + d) < float(plane.get('inlier_m', 0.01))
        if not on_plane.any():
            return None
        robot = camera_to_robot(pts[on_plane], cam_pose)
    height, width = np.asarray(depth_mm).shape[:2]
    fwd_lo, fwd_hi = np.percentile(robot[:, 0], [5, 95])
    left_lo, left_hi = np.percentile(robot[:, 1], [5, 95])
    return {'median_up_m': round(float(np.median(robot[:, 2])), 3), 'pixels': int(on_plane.sum()),
            'fraction': round(float(on_plane.sum()) / (height * width), 3),
            'forward_range_m': [round(float(fwd_lo), 3), round(float(fwd_hi), 3)],
            'left_range_m': [round(float(left_lo), 3), round(float(left_hi), 3)]}


# ---------------------------------------------------------------- connected components

def label_components(mask):
    """4-connected components of a boolean mask -> (labels int32, count). 0 is background, 1..count the blobs.
    Uses cv2.connectedComponents when available, else a two-pass union-find over row runs (pure numpy/Python)."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('mask must be 2-D')
    if cv2 is not None:
        count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=4)
        return labels.astype(np.int32), int(count) - 1
    labels = np.zeros(mask.shape, dtype=np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    previous = []  # (start, stop, label) runs of the row above
    for row in range(mask.shape[0]):
        line = mask[row]
        if not line.any():
            previous = []
            continue
        padded = np.concatenate(([False], line, [False]))
        edges = np.flatnonzero(padded[1:] != padded[:-1])
        current = []
        for start, stop in zip(edges[::2], edges[1::2]):
            label = 0
            for p_start, p_stop, p_label in previous:
                if p_start < stop and start < p_stop:  # 4-connectivity: column overlap
                    root = find(p_label)
                    if label == 0:
                        label = root
                    elif root != label:
                        parent[root] = label
            if label == 0:
                parent.append(len(parent))
                label = len(parent) - 1
            labels[row, start:stop] = label
            current.append((int(start), int(stop), label))
        previous = current
    if len(parent) == 1:
        return labels, 0
    roots = np.array([find(i) for i in range(len(parent))])
    unique, compact = np.unique(roots[1:], return_inverse=True)
    lookup = np.zeros(len(parent), dtype=np.int32)
    lookup[1:] = compact + 1
    return lookup[labels], int(len(unique))


# ---------------------------------------------------------------- scene summary

def _f(value, digits):
    value = float(value)
    if not math.isfinite(value):
        return None
    return round(value, digits) if digits is not None else value


def _point(points, digits):
    """Component-wise median of (N, 3) robot-frame points -> [forward, left, up]."""
    med = np.median(points, axis=0)
    return [_f(v, digits) for v in med]


def _region_labels(count, names, prefix):
    if count == len(names):
        return list(names)
    return [f'{prefix}{i}' for i in range(count)]


def scene_points(depth_mm, intrinsics, cam_pose, *, pixels=None, distortion=None, grid=(5, 3),
                 nearest_band_m=NEAREST_BAND_M, nearest_percentile=NEAREST_PERCENTILE, round_m=3, table_top_m=None):
    """Summarise a depth image as robot-frame numbers. See the module docstring.

    Returns {'image': {'width','height'}, 'valid_fraction', 'invalid_fraction', 'centre_invalid_fraction',
    'min_valid_distance_m', 'max_valid_distance_m', 'grid': {'columns', 'rows', 'regions': [...]},
    'nearest': {...} or None, 'query': [...], 'undistorted', 'undistortion', 'distance_definition', 'frame',
    'notes': [...]}; metres rounded to ``round_m`` decimals (None: unrounded). With ``table_top_m`` (height of
    the table top above the floor; give it only with a pose calibrated on that table) the nearest object also
    carries 'top_m' (95th percentile of its points' height), 'height_above_table_m' (centre) and
    'top_above_table_m'.
    """
    depth_m, valid = check_depth(depth_mm)
    pos, rot = check_pose(cam_pose)
    if table_top_m is not None and not (0 < float(table_top_m) < 2):
        raise ValueError('table_top_m must be a height above the floor between 0 and 2 m')
    columns, rows = int(grid[0]), int(grid[1])
    if columns < 1 or rows < 1:
        raise ValueError('grid must be (columns, rows) with both >= 1')
    if not (0 < nearest_band_m < 5):
        raise ValueError('nearest_band_m must be a positive distance below 5 m')
    height, width = depth_m.shape
    xn, yn, undistorted, undistortion = normalized_rays(depth_m.shape, intrinsics, distortion)

    z = np.where(valid, depth_m, 0.0)
    cam = np.stack([xn * z, yn * z, z], axis=-1)                 # (H, W, 3) optical frame
    rng = np.where(valid, np.linalg.norm(cam, axis=-1), np.inf)  # straight-line distance from the lens
    robot = cam @ rot.T + pos                                     # (H, W, 3) forward, left, up

    notes = []
    n_valid = int(valid.sum())
    valid_fraction = n_valid / valid.size
    cy0, cy1, cx0, cx1 = height // 3, 2 * height // 3, width // 3, 2 * width // 3
    centre = valid[cy0:max(cy1, cy0 + 1), cx0:max(cx1, cx0 + 1)]
    centre_invalid = 1.0 - float(centre.mean()) if centre.size else 1.0
    if n_valid == 0:
        notes.append('no valid depth anywhere: the camera may be covered, too close to everything (stereo is '
                     f'blind under about {STEREO_BLIND_M} m) or facing a textureless or blown-out surface')
    elif centre_invalid > CENTRE_BLIND_FRACTION:
        notes.append(f'{centre_invalid:.0%} of the image centre has no depth: something is probably closer than the '
                     f'stereo minimum (about {STEREO_BLIND_M} m), or the surface there is textureless or blown out')
    if 0 < n_valid and valid_fraction < 0.5:
        notes.append(f'only {valid_fraction:.0%} of the pixels have depth; region medians rest on few points')

    # grid of regions
    x_edges = np.linspace(0, width, columns + 1).round().astype(int)
    y_edges = np.linspace(0, height, rows + 1).round().astype(int)
    col_names = _region_labels(columns, GRID_COLUMNS, 'column')
    row_names = _region_labels(rows, GRID_ROWS, 'row')
    regions = []
    for r in range(rows):
        for c in range(columns):
            y0, y1, x0, x1 = y_edges[r], y_edges[r + 1], x_edges[c], x_edges[c + 1]
            cell_valid = valid[y0:y1, x0:x1]
            n = int(cell_valid.sum())
            entry = {'column': col_names[c], 'row': row_names[r], 'pixel_box': [int(x0), int(y0), int(x1), int(y1)],
                     'valid_fraction': _f(n / max(cell_valid.size, 1), 3),
                     'median_distance_m': None, 'median_point_m': None}
            if n:
                entry['median_distance_m'] = _f(np.median(rng[y0:y1, x0:x1][cell_valid]), round_m)
                entry['median_point_m'] = _point(robot[y0:y1, x0:x1][cell_valid], round_m)
            regions.append(entry)

    # nearest coherent object
    nearest = None
    if n_valid:
        ranges = rng[valid]
        near_m = float(np.percentile(ranges, nearest_percentile))
        band = valid & (rng <= near_m + nearest_band_m)
        labels, count = label_components(band)
        if count:
            sizes = np.bincount(labels.ravel(), minlength=count + 1)
            sizes[0] = 0
            blob = labels == int(sizes.argmax())
            ys, xs = np.nonzero(blob)
            pts_cam, pts_robot = cam[blob], robot[blob]
            nearest = {
                'centre_m': _point(pts_robot, round_m),
                'median_distance_m': _f(np.median(rng[blob]), round_m),
                'min_distance_m': _f(rng[blob].min(), round_m),
                'extent_m': {'width': _f(pts_cam[:, 0].max() - pts_cam[:, 0].min(), round_m),
                             'height': _f(pts_cam[:, 1].max() - pts_cam[:, 1].min(), round_m),
                             'depth': _f(pts_cam[:, 2].max() - pts_cam[:, 2].min(), round_m)},
                'pixel_bbox': [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                'pixel_centre': [int(round(xs.mean())), int(round(ys.mean()))],
                'pixel_count': int(blob.sum()),
                'image_fraction': _f(blob.sum() / valid.size, 4),
                'band': {'percentile': float(nearest_percentile), 'percentile_distance_m': _f(near_m, round_m),
                         'width_m': float(nearest_band_m), 'blobs': int(count)},
            }
            if table_top_m is not None:
                top = float(np.percentile(pts_robot[:, 2], 95))
                centre_up = float(np.median(pts_robot[:, 2]))
                nearest['top_m'] = _f(top, round_m)
                nearest['height_above_table_m'] = _f(centre_up - float(table_top_m), round_m)
                nearest['top_above_table_m'] = _f(top - float(table_top_m), round_m)
    if nearest is None:
        notes.append('no nearest object: no valid depth to cluster')

    # caller-given pixels
    queries = []
    half = QUERY_PATCH // 2
    for pixel in (pixels or []):
        px = np.asarray(pixel, dtype=float).ravel()
        if px.size != 2 or not np.isfinite(px).all():
            raise ValueError('each query pixel must be [x, y]')
        x, y = int(round(px[0])), int(round(px[1]))
        entry = {'pixel': [x, y], 'point_m': None, 'distance_m': None, 'patch_valid_fraction': 0.0}
        if not (0 <= x < width and 0 <= y < height):
            entry['reason'] = f'outside the {width}x{height} image'
            queries.append(entry)
            continue
        patch_valid = valid[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        patch_depth = depth_m[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        n = int(patch_valid.sum())
        entry['patch_valid_fraction'] = _f(n / QUERY_PATCH ** 2, 3)
        if n < QUERY_MIN_VALID:
            entry['reason'] = (f'invalid depth: only {n} of the {QUERY_PATCH}x{QUERY_PATCH} pixels around it have '
                               f'depth (stereo is blind under about {STEREO_BLIND_M} m and on textureless areas)')
            queries.append(entry)
            continue
        zq = float(np.median(patch_depth[patch_valid]))
        cam_q = np.array([xn[y, x] * zq, yn[y, x] * zq, zq])
        robot_q = rot @ cam_q + pos
        entry['point_m'] = [_f(v, round_m) for v in robot_q]
        entry['distance_m'] = _f(np.linalg.norm(cam_q), round_m)
        queries.append(entry)

    return {
        'image': {'width': int(width), 'height': int(height)},
        'valid_fraction': _f(valid_fraction, 3),
        'invalid_fraction': _f(1.0 - valid_fraction, 3),
        'centre_invalid_fraction': _f(centre_invalid, 3),
        'min_valid_distance_m': _f(rng[valid].min(), round_m) if n_valid else None,
        'max_valid_distance_m': _f(rng[valid].max(), round_m) if n_valid else None,
        'grid': {'columns': col_names, 'rows': row_names, 'regions': regions},
        'nearest': nearest,
        'query': queries,
        'undistorted': bool(undistorted),
        'undistortion': undistortion,
        'distance_definition': DISTANCE_DEFINITION,
        'frame': FRAME,
        'notes': notes,
    }
