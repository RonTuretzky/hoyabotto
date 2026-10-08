"""depth_scene: back-projection, region grid, nearest object and pixel queries on a synthetic depth image."""
import json
import math

import numpy as np
import pytest

from farm.perception import depth_scene as ds

# 640x360 OAK RGB-aligned frame, intrinsics as the camera reports them (fx=fy=505, cx=315, cy=193).
W, H = 640, 360
FX, FY, CX, CY = 505.0, 505.0, 315.0, 193.0
K = [[FX, 0, CX], [0, FY, CY], [0, 0, 1]]
# Camera 1.181 m up, 0.037 m ahead of the base, looking exactly forward and level (the twin's mapped zero):
# optical x = robot right (-left), y = down, z = forward.
POSE = {'position_m': [0.037, -0.002, 1.181],
        'rotation': [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]}
BACKGROUND_MM = 1500
BOX_MM = 500
BOX = (100, 130, 200, 210)  # x0, y0, x1, y1 (exclusive) of the near box in pixels: 100x80 = 8000 px (3.5 %)
HOLE = (500, 40, 560, 100)  # a patch of invalid depth


def synthetic():
    depth = np.full((H, W), BACKGROUND_MM, np.uint16)
    depth[BOX[1]:BOX[3], BOX[0]:BOX[2]] = BOX_MM
    depth[HOLE[1]:HOLE[3], HOLE[0]:HOLE[2]] = 0
    return depth


def test_backproject_hand_computed_points():
    depth = synthetic()
    pts = ds.backproject(depth, K, [[CX, CY], [CX + 100, CY], [CX, CY + 100], [150, 175], [520, 60], [-1, 5], [5, 900]])
    # principal point at 1.5 m: straight down the optical axis
    assert pts[0] == pytest.approx([0.0, 0.0, 1.5])
    # 100 px right of centre: x = 100 * 1.5 / 505
    assert pts[1] == pytest.approx([100 * 1.5 / 505, 0.0, 1.5])
    assert pts[2] == pytest.approx([0.0, 100 * 1.5 / 505, 1.5])
    # inside the box at 0.5 m
    assert pts[3] == pytest.approx([(150 - CX) * 0.5 / FX, (175 - CY) * 0.5 / FY, 0.5])
    assert np.isnan(pts[4]).all()  # hole
    assert np.isnan(pts[5]).all() and np.isnan(pts[6]).all()  # outside the image
    assert ds.backproject(depth, K, [CX, CY]).shape == (1, 3)  # a single pixel


def test_camera_to_robot_uses_the_pose():
    cam = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.5], [0.0, 0.1, 0.5]])
    robot = ds.camera_to_robot(cam, POSE)
    assert robot[0] == pytest.approx([0.537, -0.002, 1.181])          # straight ahead of the lens
    assert robot[1] == pytest.approx([0.537, -0.102, 1.181])          # image right is the robot's right
    assert robot[2] == pytest.approx([0.537, -0.002, 1.081])          # image down is down


@pytest.mark.parametrize('bad', [np.zeros((3, 2)), [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]],
                                 [[0, 0, 0], [0, 1, 0], [0, 0, 1]], [[500, 0, 300], [0, 500, 200], [0, 0, 2]]])
def test_bad_intrinsics_are_refused(bad):
    with pytest.raises(ValueError):
        ds.backproject(synthetic(), bad, [[1, 1]])


@pytest.mark.parametrize('bad', [{'position_m': [0, 0], 'rotation': np.eye(3).tolist()},
                                 {'position_m': [0, 0, 0], 'rotation': [[1, 0, 0], [0, 1, 0], [0, 0, 2]]},
                                 {'position_m': [0, 0, 0], 'rotation': [[1, 0, 0], [0, 1, 0], [0, 0, -1]]},
                                 'not a pose'])
def test_bad_pose_is_refused(bad):
    with pytest.raises(ValueError):
        ds.scene_points(synthetic(), K, bad)


def test_scene_points_grid_nearest_and_queries():
    depth = synthetic()
    scene = ds.scene_points(depth, K, POSE, pixels=[[150, 175], [530, 70], [400, 100], [900, 5]])
    assert scene['image'] == {'width': W, 'height': H}
    hole_fraction = (HOLE[2] - HOLE[0]) * (HOLE[3] - HOLE[1]) / (W * H)
    assert scene['invalid_fraction'] == pytest.approx(hole_fraction, abs=1e-3)
    assert scene['valid_fraction'] == pytest.approx(1 - hole_fraction, abs=1e-3)
    assert scene['centre_invalid_fraction'] == 0.0
    # the box pixel nearest the principal point is 116 px to its left: range 0.5 * sqrt(1 + (116/505)^2)
    assert scene['min_valid_distance_m'] == pytest.approx(0.5 * math.sqrt(1 + (116 / 505) ** 2), abs=0.002)
    assert scene['max_valid_distance_m'] > 1.5                               # background corners are farther than 1.5 m axial
    assert scene['undistorted'] is False and 'not needed' in scene['undistortion']
    assert scene['notes'] == []

    grid = scene['grid']
    assert grid['columns'] == ['left', 'centre-left', 'centre', 'centre-right', 'right']
    assert grid['rows'] == ['top', 'middle', 'bottom']
    regions = grid['regions']
    assert len(regions) == 15
    assert [r['column'] for r in regions[:5]] == grid['columns'] and regions[0]['row'] == 'top'
    assert regions[0]['pixel_box'] == [0, 0, 128, 120] and regions[14]['pixel_box'] == [512, 240, 640, 360]
    centre = next(r for r in regions if r['column'] == 'centre' and r['row'] == 'middle')
    assert centre['valid_fraction'] == 1.0
    assert 1.5 <= centre['median_distance_m'] <= 1.51
    assert centre['median_point_m'][0] == pytest.approx(1.537, abs=0.002)   # 1.5 m ahead of the lens
    # region centre pixel (319.5, 179.5) is 4.5 px right of and 13.5 px above the principal point, at 1.5 m
    assert centre['median_point_m'][1] == pytest.approx(-0.002 - 4.5 * 1.5 / 505, abs=0.003)
    assert centre['median_point_m'][2] == pytest.approx(1.181 + 13.5 * 1.5 / 505, abs=0.003)
    top_right = next(r for r in regions if r['column'] == 'right' and r['row'] == 'top')
    assert top_right['valid_fraction'] == pytest.approx(1 - 48 * 60 / (128 * 120), abs=1e-3)  # the hole's part right of x=512
    assert top_right['median_point_m'][1] < -0.2 and top_right['median_point_m'][2] > 1.3  # robot right, above the lens
    for r in regions:
        assert r['median_distance_m'] is not None and len(r['median_point_m']) == 3

    near = scene['nearest']
    assert near['pixel_bbox'] == [BOX[0], BOX[1], BOX[2] - 1, BOX[3] - 1]
    assert near['pixel_count'] == (BOX[2] - BOX[0]) * (BOX[3] - BOX[1])
    assert near['pixel_centre'] == [round((BOX[0] + BOX[2] - 1) / 2), round((BOX[1] + BOX[3] - 1) / 2)]
    assert near['median_distance_m'] == pytest.approx(0.5 * math.sqrt(1 + (165.5 / 505) ** 2), abs=0.01)
    assert near['min_distance_m'] == scene['min_valid_distance_m']
    # box centre pixel (149.5, 169.5): x = (149.5-315)*0.5/505 = -0.1639 (robot left +0.164), y = -0.0233 (up +0.023)
    assert near['centre_m'] == pytest.approx([0.537, -0.002 + 0.1639, 1.181 + 0.0233], abs=0.002)
    assert near['extent_m']['width'] == pytest.approx(99 * 0.5 / 505, abs=0.001)
    assert near['extent_m']['height'] == pytest.approx(79 * 0.5 / 505, abs=0.001)
    assert near['extent_m']['depth'] == 0.0
    assert near['image_fraction'] == pytest.approx(8000 / (W * H), abs=1e-4)
    assert near['band']['blobs'] == 1 and 0.51 <= near['band']['percentile_distance_m'] <= 0.53

    q = scene['query']
    assert [e['pixel'] for e in q] == [[150, 175], [530, 70], [400, 100], [900, 5]]
    assert q[0]['point_m'] == pytest.approx([0.537, -0.002 - (150 - CX) * 0.5 / FX, 1.181 - (175 - CY) * 0.5 / FY], abs=0.001)
    assert q[0]['distance_m'] == pytest.approx(0.5 * math.sqrt(1 + ((150 - CX) / FX) ** 2 + ((175 - CY) / FY) ** 2), abs=0.001)
    assert q[0]['patch_valid_fraction'] == 1.0
    assert q[1]['point_m'] is None and q[1]['distance_m'] is None and 'invalid depth' in q[1]['reason']
    assert q[1]['patch_valid_fraction'] == 0.0
    assert q[2]['point_m'][0] == pytest.approx(1.537, abs=0.001)
    assert q[3]['point_m'] is None and 'outside' in q[3]['reason']
    assert scene['distance_definition'].startswith('straight-line')
    json.dumps(scene, allow_nan=False)


def test_query_patch_median_bridges_a_few_invalid_pixels_but_not_most():
    depth = synthetic()
    depth[173:178, 148:153] = 0            # the whole 5x5 patch around (150, 175)
    depth[175, 150] = BOX_MM
    depth[173, 148:153] = BOX_MM           # 6 valid of 25
    scene = ds.scene_points(depth, K, POSE, pixels=[[150, 175]])
    assert scene['query'][0]['point_m'] is not None and scene['query'][0]['patch_valid_fraction'] == pytest.approx(6 / 25)
    depth[173, 148:153] = 0                # 1 valid of 25: too few
    scene = ds.scene_points(depth, K, POSE, pixels=[[150, 175]])
    assert scene['query'][0]['point_m'] is None


def test_nearest_is_the_largest_blob_in_the_near_band_not_the_largest_object():
    depth = synthetic()
    depth[250:350, 100:400] = 900       # a big object, but farther than the box
    depth[20:30, 600:630] = 480         # a small near speck (range 0.58 m, within the band): separate blob
    scene = ds.scene_points(depth, K, POSE)
    near = scene['nearest']
    assert near['pixel_bbox'] == [BOX[0], BOX[1], BOX[2] - 1, BOX[3] - 1]
    assert near['band']['blobs'] == 2
    depth[20:120, 500:630] = 480        # now the speck is the bigger near blob
    near = ds.scene_points(depth, K, POSE)['nearest']
    assert near['pixel_bbox'] == [500, 20, 629, 119] and near['pixel_count'] == 100 * 130


def test_nearest_band_excludes_farther_surfaces():
    depth = synthetic()
    depth[BOX[1]:BOX[3], BOX[2]:BOX[2] + 40] = BOX_MM + 200   # adjoining step 0.2 m behind: outside the 8 cm band
    near = ds.scene_points(depth, K, POSE)['nearest']
    assert near['pixel_bbox'][2] == BOX[2] - 1
    near = ds.scene_points(depth, K, POSE, nearest_band_m=0.3)['nearest']
    assert near['pixel_bbox'][2] == BOX[2] + 39


def test_all_invalid_depth_says_so():
    scene = ds.scene_points(np.zeros((H, W), np.uint16), K, POSE, pixels=[[320, 180]])
    assert scene['invalid_fraction'] == 1.0 and scene['centre_invalid_fraction'] == 1.0
    assert scene['min_valid_distance_m'] is None and scene['nearest'] is None
    assert all(r['median_distance_m'] is None and r['median_point_m'] is None and r['valid_fraction'] == 0.0
               for r in scene['grid']['regions'])
    assert scene['query'][0]['point_m'] is None
    assert any('no valid depth anywhere' in n for n in scene['notes']) and any('no nearest object' in n for n in scene['notes'])
    json.dumps(scene, allow_nan=False)


def test_blind_centre_is_flagged():
    depth = synthetic()
    depth[H // 3:2 * H // 3, W // 3:2 * W // 3] = 0   # something closer than the stereo minimum
    scene = ds.scene_points(depth, K, POSE)
    assert scene['centre_invalid_fraction'] == 1.0
    assert any('image centre has no depth' in n and '0.25 m' in n for n in scene['notes'])
    assert scene['nearest'] is not None   # the box survives at the side


def test_out_of_range_depth_is_invalid():
    depth = synthetic()
    depth[0:10, 0:10] = 65535
    scene = ds.scene_points(depth, K, POSE)
    assert scene['max_valid_distance_m'] < 3.0
    assert np.isnan(ds.backproject(depth, K, [[5, 5]])).all()


def test_label_components_fallback_matches_cv2(monkeypatch):
    mask = np.zeros((20, 30), bool)
    mask[2:5, 2:10] = True
    mask[4:9, 9:12] = True          # touches the first blob by column overlap (4-connected)
    mask[12:15, 20:25] = True
    mask[15, 25] = True             # diagonal only: a separate blob under 4-connectivity
    mask[0, 29] = True
    pytest.importorskip('cv2')
    labels_cv, count_cv = ds.label_components(mask)
    monkeypatch.setattr(ds, 'cv2', None)
    labels_py, count_py = ds.label_components(mask)
    assert count_cv == count_py == 4
    sizes_cv = sorted(np.bincount(labels_cv.ravel())[1:])
    sizes_py = sorted(np.bincount(labels_py.ravel())[1:])
    assert sizes_cv == sizes_py == [1, 1, 15, 38]  # 24 + 15 - 1 shared pixel
    assert (labels_py > 0).tolist() == mask.tolist()
    assert ds.label_components(np.zeros((3, 3), bool)) [1] == 0
    # U-shape: the two arms join at the bottom, so the union-find must merge labels
    u = np.zeros((6, 7), bool)
    u[0:5, 0:2] = True
    u[0:5, 5:7] = True
    u[5, :] = True
    assert ds.label_components(u)[1] == 1


def test_without_cv2_results_are_unchanged_and_undistortion_is_reported(monkeypatch):
    depth = synthetic()
    with_cv2 = ds.scene_points(depth, K, POSE, pixels=[[150, 175]])
    monkeypatch.setattr(ds, 'cv2', None)
    without = ds.scene_points(depth, K, POSE, pixels=[[150, 175]])
    assert without['nearest'] == with_cv2['nearest'] and without['grid'] == with_cv2['grid']
    assert without['query'] == with_cv2['query']
    skipped = ds.scene_points(depth, K, POSE, distortion=[0.1, -0.2, 0, 0, 0])
    assert skipped['undistorted'] is False and 'cv2' in skipped['undistortion']
    assert skipped['nearest']['centre_m'] == with_cv2['nearest']['centre_m']


def test_undistortion_with_cv2_changes_off_centre_points_only():
    pytest.importorskip('cv2')
    depth = synthetic()
    plain = ds.scene_points(depth, K, POSE, pixels=[[int(CX), int(CY)], [600, 300]])
    zeros = ds.scene_points(depth, K, POSE, pixels=[[int(CX), int(CY)], [600, 300]], distortion=[0.0] * 14)
    assert zeros['undistorted'] is False and zeros['query'] == plain['query']
    barrel = ds.scene_points(depth, K, POSE, pixels=[[int(CX), int(CY)], [600, 300]],
                             distortion=[-0.3, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    assert barrel['undistorted'] is True and 'undistortPoints' in barrel['undistortion']
    assert barrel['query'][0]['point_m'] == plain['query'][0]['point_m']          # the principal point is fixed
    assert barrel['query'][1]['point_m'] != plain['query'][1]['point_m']
    assert abs(barrel['query'][1]['point_m'][1]) > abs(plain['query'][1]['point_m'][1])  # barrel: true ray is farther out
    with pytest.raises(ValueError):
        ds.scene_points(depth, K, POSE, distortion=[1, 2, 3])


def test_tilted_camera_pose_moves_points_down_and_closer():
    """Camera pitched 30 deg down: a point on the optical axis lands lower and nearer than when level."""
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    tilted = {'position_m': POSE['position_m'],
              'rotation': [[0.0, -s, c], [-1.0, 0.0, 0.0], [0.0, -c, -s]]}   # z = forward*c - up*s, y = -up*c - forward*s
    level = ds.scene_points(synthetic(), K, POSE)['nearest']['centre_m']
    down = ds.scene_points(synthetic(), K, tilted)['nearest']['centre_m']
    assert down[2] < level[2] - 0.2 and down[0] < level[0] - 0.05
    assert down[1] == pytest.approx(level[1], abs=1e-6)


def test_unrounded_and_custom_grid():
    scene = ds.scene_points(synthetic(), K, POSE, grid=(2, 2), round_m=None)
    assert scene['grid']['columns'] == ['column0', 'column1'] and scene['grid']['rows'] == ['row0', 'row1']
    assert len(scene['grid']['regions']) == 4
    assert isinstance(scene['nearest']['median_distance_m'], float)
    with pytest.raises(ValueError):
        ds.scene_points(synthetic(), K, POSE, grid=(0, 3))
    with pytest.raises(ValueError):
        ds.scene_points(np.zeros((0, 5), np.uint16), K, POSE)
    with pytest.raises(ValueError):
        ds.scene_points(synthetic(), K, POSE, pixels=[[1, 2, 3]])


def test_scene_points_is_fast_enough_for_a_chat_tool():
    import time
    depth = synthetic()
    ds.scene_points(depth, K, POSE, pixels=[[150, 175]], distortion=[0.01] * 14)  # warm the ray cache
    start = time.perf_counter()
    for _ in range(3):
        ds.scene_points(depth, K, POSE, pixels=[[150, 175]], distortion=[0.01] * 14)
    assert (time.perf_counter() - start) / 3 < 0.5


# ---------------------------------------------------------------- table-plane self-calibration

TABLE_UP = 0.70
TABLE_BOX = ((0.30, 0.90), (-0.45, 0.45))          # forward and left extent of the table top
BOX3D = {'forward': (0.32, 0.52), 'left': (0.12, 0.27), 'top': 0.81}   # a 20x15x11 cm box on the table
UP = np.array([0.0, 0.0, 1.0])


def rotate_about(rot, axis, deg):
    """rot pre-multiplied by a rotation of deg about a robot-frame axis (Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    th = math.radians(deg)
    return (np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k) @ rot


def pose_from_angles(tilt_deg, roll_deg=0.0, pan_deg=0.0, position=(0.04, -0.01, 1.16)):
    """A camera pose: the level-forward optical frame of POSE tilted DOWN by tilt_deg about its image-right axis,
    rolled by roll_deg about the optical axis, then panned LEFT by pan_deg about robot up."""
    rot = np.array(POSE['rotation'], dtype=float)
    rot = rotate_about(rot, rot[:, 0], -tilt_deg)
    rot = rotate_about(rot, rot[:, 2], roll_deg)
    rot = rotate_about(rot, UP, pan_deg)
    return {'position_m': [float(v) for v in position], 'rotation': rot.tolist()}


def render_planes(pose, intrinsics=K, *, table_up=TABLE_UP, table_box=TABLE_BOX, floor=True, box=None,
                  wall_forward=None, noise_mm=0.0, seed=0, blind=(0.25, 8.0)):
    """uint16 millimetre depth (pinhole, no distortion) of a scene of planes seen from ``pose``: the table top at
    ``table_up`` over ``table_box`` ((forward0, forward1), (left0, left1)), the floor at 0, optionally a box
    {'forward': (f0, f1), 'left': (l0, l1), 'top': h} (top face plus its near vertical face) and a vertical wall
    at forward = ``wall_forward``. Gaussian range noise in mm; 0 outside the stereo window ``blind``."""
    fx, fy, cx, cy = ds.check_intrinsics(intrinsics)
    pos, rot = ds.check_pose(pose)
    us, vs = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us)], axis=-1)
    dirs = rays @ rot.T                     # robot-frame direction per metre of axial depth

    def plane(axis, value, forward=None, left=None, up=None):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (value - pos[axis]) / dirs[..., axis]
            hit = np.isfinite(t) & (t > 0)
            p = pos + np.where(hit, t, 0.0)[..., None] * dirs
        for i, window in ((0, forward), (1, left), (2, up)):
            if window is not None:
                hit &= (p[..., i] >= window[0]) & (p[..., i] <= window[1])
        return np.where(hit, t, np.inf)

    z = plane(2, table_up, *table_box)
    if floor:
        z = np.minimum(z, plane(2, 0.0))
    if box:
        z = np.minimum(z, plane(2, box['top'], box['forward'], box['left']))
        z = np.minimum(z, plane(0, box['forward'][0], None, box['left'], (table_up, box['top'])))
    if wall_forward is not None:
        z = np.minimum(z, plane(0, wall_forward))
    if noise_mm:
        z = z + np.random.RandomState(seed).normal(0.0, noise_mm / 1000.0, z.shape)
    valid = np.isfinite(z) & (z >= blind[0]) & (z <= blind[1])
    return np.where(valid, np.rint(z * 1000.0), 0).astype(np.uint16)


def project(pose, point, intrinsics=K):
    """Pixel [x, y] of a robot-frame point under a pose."""
    pos, rot = ds.check_pose(pose)
    fx, fy, cx, cy = ds.check_intrinsics(intrinsics)
    cam = rot.T @ (np.asarray(point, dtype=float) - pos)
    return [int(round(fx * cam[0] / cam[2] + cx)), int(round(fy * cam[1] / cam[2] + cy))]


def rotation_error_deg(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    return math.degrees(math.acos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


def test_pose_from_angles_matches_the_tilt_and_roll_readers():
    pose = pose_from_angles(35.0, 2.0, 5.0)
    rot = np.array(pose['rotation'])
    assert ds.tilt_deg(rot) == pytest.approx(35.0, abs=1e-6)
    assert ds.roll_deg(rot) == pytest.approx(2.0, abs=1e-6)
    assert ds.roll_deg(pose_from_angles(35.0, -7.0)['rotation']) == pytest.approx(-7.0, abs=1e-6)
    assert ds.roll_deg(pose_from_angles(90.0)['rotation']) == 0.0    # straight down: undefined, reported 0
    assert math.degrees(math.atan2(rot[1, 2], rot[0, 2])) == pytest.approx(5.0, abs=1e-6)   # heading 5 deg left
    assert ds.tilt_deg(POSE['rotation']) == 0.0 and ds.roll_deg(POSE['rotation']) == 0.0
    # the synthetic scene is consistent with the back-projection: the box top projects to the box top
    depth = render_planes(pose, box=BOX3D)
    pixel = project(pose, [0.42, 0.195, BOX3D['top']])
    assert ds.camera_to_robot(ds.backproject(depth, K, [pixel]), pose)[0] == pytest.approx([0.42, 0.195, 0.81], abs=0.003)


def test_fit_table_plane_recovers_a_synthetic_table():
    true = pose_from_angles(35.0, 2.0, 5.0)
    depth = render_planes(true, box=BOX3D, noise_mm=2.0)
    rot = np.array(true['rotation'])
    plane = ds.fit_table_plane(depth, K, expected_up_cam=rot.T @ UP)
    assert plane['ok'], plane
    normal = np.array(plane['normal_cam'])
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert math.degrees(math.acos(np.clip(normal @ (rot.T @ UP), -1, 1))) < 0.3
    assert plane['angle_from_expected_up_deg'] < 0.3
    assert plane['d_m'] == pytest.approx(1.16 - TABLE_UP, abs=0.003)     # the lens's height above the table
    assert plane['inlier_fraction'] > 0.3 and plane['rms_m'] < 0.004 and plane['candidates'] > 10
    assert plane['points'] == ds.PLANE_MAX_POINTS and plane['valid_pixels'] > 100000
    assert 0.3 < plane['image_fraction'] < 1.0
    assert plane['image_fraction'] == pytest.approx(ds.table_check(depth, K, true, plane)['fraction'], abs=0.03)
    # deterministic for a seed
    again = ds.fit_table_plane(depth, K, expected_up_cam=rot.T @ UP)
    assert again['normal_cam'] == plane['normal_cam'] and again['d_m'] == plane['d_m']
    # without the expected up the same plane wins (it dominates the image)
    free = ds.fit_table_plane(depth, K)
    assert free['ok'] and free['angle_from_expected_up_deg'] is None
    assert np.allclose(free['normal_cam'], plane['normal_cam'], atol=1e-3) and free['d_m'] == pytest.approx(plane['d_m'], abs=0.002)


def test_calibrate_camera_pose_recovers_the_true_pose_from_a_wrong_model():
    true = pose_from_angles(35.0, 2.0, 5.0, position=(0.04, -0.01, 1.16))
    wrong = pose_from_angles(20.0, 0.0, 5.0, position=(0.04, -0.01, 1.21))   # 15 deg too level, 5 cm too high
    depth = render_planes(true, box=BOX3D, noise_mm=2.0)
    wrong_rot = np.array(wrong['rotation'])
    plane = ds.fit_table_plane(depth, K, expected_up_cam=wrong_rot.T @ UP, expected_d_m=1.21 - TABLE_UP)
    cal = ds.calibrate_camera_pose(plane, wrong, TABLE_UP)
    assert cal['ok'] is True and cal['method'] == 'table_plane', cal
    assert cal['tilt_deg'] == pytest.approx(35.0, abs=0.2)
    assert cal['roll_deg'] == pytest.approx(ds.roll_deg(true['rotation']), abs=0.2)
    assert cal['position_m'][:2] == wrong['position_m'][:2]                         # forward/left kept
    assert cal['position_m'][2] == pytest.approx(1.16, abs=0.003)
    assert cal['tilt_correction_deg'] == pytest.approx(15.0, abs=0.2)
    assert cal['height_correction_m'] == pytest.approx(-0.05, abs=0.003)
    assert cal['angle_correction_deg'] == pytest.approx(rotation_error_deg(wrong['rotation'], true['rotation']), abs=0.3)
    assert rotation_error_deg(cal['rotation'], true['rotation']) < 0.3
    assert cal['model_tilt_deg'] == 20.0 and cal['model_position_m'] == wrong['position_m']
    assert cal['camera_above_table_m'] == pytest.approx(0.46, abs=0.003) and cal['table_top_m'] == TABLE_UP
    assert 'tilt 20.0 -> 35.0 deg' in cal['reason'] and cal['inlier_fraction'] == plane['inlier_fraction']
    ds.check_pose(cal)   # a proper rotation
    # the box top now comes out at 0.81 m, where the wrong pose had it 7 cm off
    pixel = project(true, [0.42, 0.195, BOX3D['top']])
    scene = ds.scene_points(depth, K, cal, pixels=[pixel], table_top_m=TABLE_UP)
    assert scene['query'][0]['point_m'] == pytest.approx([0.42, 0.195, 0.81], abs=0.006)
    assert scene['nearest']['top_m'] == pytest.approx(0.81, abs=0.01)
    assert scene['nearest']['top_above_table_m'] == pytest.approx(0.11, abs=0.01)
    assert 0.0 <= scene['nearest']['height_above_table_m'] <= 0.11
    bad = ds.scene_points(depth, K, wrong, pixels=[pixel])
    assert abs(bad['query'][0]['point_m'][2] - 0.81) > 0.05 and 'top_m' not in bad['nearest']
    # the table's pixels sit at the table height under the calibrated pose, and span the table
    check = ds.table_check(depth, K, cal, plane)
    assert check['median_up_m'] == pytest.approx(TABLE_UP, abs=0.002) and check['pixels'] > 50000
    assert 0.30 <= check['forward_range_m'][0] < check['forward_range_m'][1] <= 0.90
    assert -0.45 <= check['left_range_m'][0] < check['left_range_m'][1] <= 0.45
    assert ds.table_check(depth, K, cal, {'ok': False}) is None


def test_calibrate_rejects_large_corrections_and_keeps_the_model_pose():
    true = pose_from_angles(35.0)
    depth = render_planes(true, box=BOX3D)
    plane = ds.fit_table_plane(depth, K, expected_up_cam=np.array(true['rotation']).T @ UP)
    # the right pose but a table height 30 cm off: the height correction exceeds 25 cm
    cal = ds.calibrate_camera_pose(plane, true, 0.40)
    assert cal['ok'] is False and cal['method'] == 'model'
    assert 'cm in height' in cal['reason'] and cal['height_correction_m'] == pytest.approx(-0.30, abs=0.003)
    assert cal['position_m'] == true['position_m'] and cal['rotation'] == true['rotation']
    # a model 40 deg off: beyond the 35 deg limit, so the model pose stays
    wrong = pose_from_angles(-5.0)
    cal = ds.calibrate_camera_pose(ds.fit_table_plane(depth, K), wrong, TABLE_UP)
    assert cal['ok'] is False and cal['method'] == 'model' and 'deg' in cal['reason']
    assert cal['angle_correction_deg'] == pytest.approx(40.0, abs=0.3)
    assert cal['rotation'] == wrong['rotation'] and cal['position_m'] == wrong['position_m']
    # a failed plane fit: the model pose, with the fit's reason
    cal = ds.calibrate_camera_pose({'ok': False, 'reason': 'too few points'}, wrong, TABLE_UP)
    assert cal['ok'] is False and cal['reason'] == 'no table plane: too few points'
    assert cal['tilt_correction_deg'] is None and cal['rotation'] == wrong['rotation']
    assert ds.calibrate_camera_pose(None, wrong, TABLE_UP)['ok'] is False
    # a roll beyond 10 deg cannot come from a pan/tilt head: the plane is not the table
    rolled = pose_from_angles(35.0, 15.0)
    cal = ds.calibrate_camera_pose(ds.fit_table_plane(depth, K), rolled, TABLE_UP)
    assert cal['ok'] is False and 'roll the camera by -15.0 deg' in cal['reason']
    assert cal['rotation'] == rolled['rotation'] and cal['roll_correction_deg'] == pytest.approx(-15.0, abs=0.1)
    with pytest.raises(ValueError):
        ds.calibrate_camera_pose(plane, wrong, 2.5)
    with pytest.raises(ValueError):
        ds.calibrate_camera_pose(plane, {'position_m': [0, 0, 1]}, TABLE_UP)


def test_fit_table_plane_degenerate_cases_fall_back_cleanly():
    nothing = ds.fit_table_plane(np.zeros((H, W), np.uint16), K)
    assert nothing['ok'] is False and 'too few points' in nothing['reason'] and nothing['normal_cam'] is None
    assert nothing['inlier_fraction'] == 0.0 and nothing['inliers'] == 0
    too_close = ds.fit_table_plane(np.full((H, W), 200, np.uint16), K)       # all under min_range_m
    assert too_close['ok'] is False and 'too few points' in too_close['reason']
    noise = np.random.RandomState(1).randint(400, 2000, (H, W)).astype(np.uint16)
    scatter = ds.fit_table_plane(noise, K)
    assert scatter['ok'] is False and 'no plane' in scatter['reason'] and scatter['inlier_fraction'] < 0.15
    # a wall 1 m ahead of a level camera fills the image: with the expected up it is not a table
    level = pose_from_angles(0.0)
    wall = render_planes(level, floor=False, wall_forward=1.0)
    gated = ds.fit_table_plane(wall, K, expected_up_cam=np.array(level['rotation']).T @ UP)
    assert gated['ok'] is False and 'not table-like' in gated['reason']
    assert gated['angle_from_expected_up_deg'] == pytest.approx(90.0, abs=0.5)
    # ungated, the wall is a fine plane, and the calibration refuses to turn the camera 90 deg onto it
    free = ds.fit_table_plane(wall, K)
    assert free['ok'] and free['d_m'] == pytest.approx(0.96, abs=0.003)
    cal = ds.calibrate_camera_pose(free, level, TABLE_UP)
    assert cal['ok'] is False and cal['angle_correction_deg'] == pytest.approx(90.0, abs=0.5)
    # a plane that holds most of the few valid points but little of the image (a sleeve under the lens) is refused
    tilted = pose_from_angles(35.0)
    table = render_planes(tilted, box=BOX3D)
    patch = np.zeros_like(table)
    patch[150:230, 240:400] = table[150:230, 240:400]                  # 5.6 % of the image, all on the table
    small = ds.fit_table_plane(patch, K, expected_up_cam=np.array(tilted['rotation']).T @ UP)
    assert small['ok'] is False and 'covers only 6% of the image' in small['reason']
    assert small['inlier_fraction'] > 0.9 and small['image_fraction'] == pytest.approx(0.056, abs=0.005)
    # a table-like plane at the wrong distance (the robot's own arm under the lens, say) is skipped too
    wrong_distance = ds.fit_table_plane(table, K, expected_up_cam=np.array(tilted['rotation']).T @ UP, expected_d_m=0.05)
    assert wrong_distance['ok'] is False and 'dominant plane is not table-like' in wrong_distance['reason']
    assert 'it is 0.46 m from the lens where the table should be about 0.05 m' in wrong_distance['reason']
    assert wrong_distance['d_m'] == pytest.approx(0.46, abs=0.01)
    # a tall box (30 cm, its top outside the height prior) whose near face fills most of the view: the table rim
    # around it is table-like but holds far fewer points than that face, so it is not trusted
    tall = render_planes(tilted, box={'forward': (0.45, 0.85), 'left': (-0.25, 0.25), 'top': 1.00})
    dominated = ds.fit_table_plane(tall, K, expected_up_cam=np.array(tilted['rotation']).T @ UP, expected_d_m=0.46)
    assert dominated['ok'] is False and 'dominant plane is not table-like' in dominated['reason']
    assert dominated['angle_from_expected_up_deg'] == pytest.approx(90.0, abs=3.0)
    assert 'the largest table-like plane holds only' in dominated['reason'] and dominated['inlier_fraction'] > 0.5
    assert ds.calibrate_camera_pose(dominated, tilted, TABLE_UP)['ok'] is False
    # known limit: a box top 11 cm above the table that hides nearly all of the table is inside the height prior
    # and wins; the lens would then be placed 11 cm too low. The table must be visible for the calibration.
    hidden = render_planes(tilted, box={'forward': (0.32, 0.88), 'left': (-0.40, 0.40), 'top': 0.81})
    wrong_table = ds.fit_table_plane(hidden, K, expected_up_cam=np.array(tilted['rotation']).T @ UP, expected_d_m=0.46)
    assert wrong_table['ok'] and wrong_table['d_m'] == pytest.approx(0.35, abs=0.01)
    # argument checks
    for kwargs in ({'inlier_m': 0}, {'ransac_iters': 0}, {'min_range_m': 3.0, 'max_range_m': 2.0}):
        with pytest.raises(ValueError):
            ds.fit_table_plane(table, K, **kwargs)
    with pytest.raises(ValueError):
        ds.scene_points(table, K, tilted, table_top_m=0)


def test_fit_table_plane_prefers_the_table_over_a_box_top_and_the_floor():
    # a big box top (40 x 40 cm, 11 cm above the table) and the floor beyond the table's far edge
    true = pose_from_angles(45.0, position=(0.04, -0.01, 1.16))
    big_box = {'forward': (0.32, 0.72), 'left': (-0.2, 0.2), 'top': 0.81}
    depth = render_planes(true, box=big_box, table_box=((0.30, 0.75), (-0.45, 0.45)))
    up_cam = np.array(true['rotation']).T @ UP
    # the box top holds more points than the table; with the height prior the lowest major plane, the table, wins
    plane = ds.fit_table_plane(depth, K, expected_up_cam=up_cam, expected_d_m=1.16 - TABLE_UP)
    assert plane['ok'] and plane['d_m'] == pytest.approx(0.46, abs=0.003), plane
    cal = ds.calibrate_camera_pose(plane, true, TABLE_UP)
    assert cal['ok'] and abs(cal['height_correction_m']) < 0.003 and abs(cal['tilt_correction_deg']) < 0.2
    # without the prior the largest plane wins: the box top here, which the calibration then rejects (11 cm off)
    largest = ds.fit_table_plane(depth, K, expected_up_cam=up_cam)
    assert largest['ok'] and largest['d_m'] == pytest.approx(0.35, abs=0.003)
    # the floor beyond the far edge never wins with the prior, even when the head looks steeply down
    steep = pose_from_angles(60.0, position=(0.04, -0.01, 1.16))
    depth = render_planes(steep, table_box=((0.30, 0.75), (-0.45, 0.45)))
    plane = ds.fit_table_plane(depth, K, expected_up_cam=np.array(steep['rotation']).T @ UP, expected_d_m=0.46)
    assert plane['ok'] and plane['d_m'] == pytest.approx(0.46, abs=0.003), plane


def test_fit_table_plane_is_fast_enough_for_a_chat_tool():
    import time
    depth = render_planes(pose_from_angles(35.0), box=BOX3D, noise_mm=2.0)
    ds.fit_table_plane(depth, K)
    started = time.perf_counter()
    for _ in range(3):
        ds.fit_table_plane(depth, K, expected_up_cam=[0, -0.8, -0.6])
    assert (time.perf_counter() - started) / 3 < 0.5
