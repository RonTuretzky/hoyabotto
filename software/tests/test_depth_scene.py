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
