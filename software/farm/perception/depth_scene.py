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
                 nearest_band_m=NEAREST_BAND_M, nearest_percentile=NEAREST_PERCENTILE, round_m=3):
    """Summarise a depth image as robot-frame numbers. See the module docstring.

    Returns {'image': {'width','height'}, 'valid_fraction', 'invalid_fraction', 'centre_invalid_fraction',
    'min_valid_distance_m', 'max_valid_distance_m', 'grid': {'columns', 'rows', 'regions': [...]},
    'nearest': {...} or None, 'query': [...], 'undistorted', 'undistortion', 'distance_definition', 'frame',
    'notes': [...]}; metres rounded to ``round_m`` decimals (None: unrounded).
    """
    depth_m, valid = check_depth(depth_mm)
    pos, rot = check_pose(cam_pose)
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
