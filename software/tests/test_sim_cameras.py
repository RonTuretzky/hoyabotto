"""Simulated cameras for the box-grab bench: scene, renders, depth, clip ring, and the pregrasp wrist view."""
import base64
import io
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if sys.platform == 'darwin':
    os.environ.setdefault('MUJOCO_GL', 'cgl')

mujoco = pytest.importorskip('mujoco')
PIL = pytest.importorskip('PIL')
from PIL import Image  # noqa: E402

from farm.kinematics import so101_reach as reach  # noqa: E402
from farm.perception import depth_scene  # noqa: E402
from farm.sim import box_scene, sim_cameras  # noqa: E402
from farm.sim import xlerobot_twin as twin  # noqa: E402

PREVIEW_DIR = Path('/tmp/sim_cameras_preview')
ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
MOTORS = [f'{side}_arm_{j}' for side in ('left', 'right') for j in ARM] + ['head_motor_1', 'head_motor_2']
# Saved ranges like the robot's calibration (as in tests/test_so101_reach.py).
RANGES = {m: (1000, 3000) for m in MOTORS}
for side in ('left', 'right'):
    RANGES.update({f'{side}_arm_shoulder_pan': (959, 3135), f'{side}_arm_shoulder_lift': (855, 3239),
                   f'{side}_arm_elbow_flex': (942, 3152), f'{side}_arm_wrist_flex': (898, 3196),
                   f'{side}_arm_wrist_roll': (115, 3979), f'{side}_arm_gripper': (1275, 2819)})
NEUTRAL = {m: (lo + hi) // 2 for m, (lo, hi) in RANGES.items()}
PREGRASP = {'forward_m': 0.36, 'left_m': 0.21, 'up_m': 0.86}
PREGRASP_PITCH = -45.0
HEAD_TILT_DEG = 40.0          # head_motor_2 above its midpoint tilts the OAK down (twin convention)


def pregrasp_ticks():
    """Left arm at the real pregrasp (36 cm forward, 21 left, 86 up, pitch -45), gripper open, head tilted down."""
    solved = reach.solve_reach('left', PREGRASP, NEUTRAL, RANGES, pitch_deg=PREGRASP_PITCH)
    assert solved['ok'], solved['reason']
    ticks = dict(NEUTRAL, **solved['ticks'])
    lo, hi = RANGES['left_arm_gripper']
    ticks['left_arm_gripper'] = lo + int(0.45 * (hi - lo))
    ticks['head_motor_2'] = NEUTRAL['head_motor_2'] + int(HEAD_TILT_DEG * twin.TICKS_PER_TURN / 360)
    return ticks


@pytest.fixture(scope='module')
def world():
    return sim_cameras.StaticWorld(seed=0, preset='near7')   # the 8 October scene these views were checked on


@pytest.fixture(scope='module')
def cams(world):
    cameras = sim_cameras.SimCameras(world)
    yield cameras
    cameras.close()


def decode_jpeg(record):
    assert record['mime_type'] == 'image/jpeg'
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(record['data_base64']))).convert('RGB'))


def decode_depth(record):
    assert record['mime_type'] == 'image/png'
    image = Image.open(io.BytesIO(base64.b64decode(record['data_base64'])))
    depth = np.asarray(image)
    assert depth.dtype == np.uint16 and depth.ndim == 2
    return depth


# ---------------------------------------------------------------- scene

def test_scene_builds_with_table_box_lamp_and_cameras(world):
    m = world.model
    for body in ('box', twin.HEAD_CAMERA_BODY, 'Fixed_Jaw', 'Fixed_Jaw_2'):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body) >= 0, body
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'box_free') >= 0
    for geom in ('table', 'box_body', 'flap', 'lamp_bulb'):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom) >= 0, geom
    for camera in box_scene.CAMERA_NAMES:
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, camera) >= 0, camera
    box = box_scene.robot_frame_of_box(m, world.data)
    assert abs(box['forward_m'] - 0.42) <= box_scene.BOX_JITTER_M + 1e-6
    assert abs(box['left_m'] - 0.21) <= box_scene.BOX_JITTER_M + 1e-6
    assert abs(box['yaw_deg']) <= box_scene.BOX_JITTER_DEG + 1e-6
    assert box['top_m'] == pytest.approx(0.81, abs=0.002)
    # the open flap: 7 cm on a hinge along the near top edge, leaning 8 deg outward
    assert box['flap_angle_deg'] == pytest.approx(box_scene.FLAP_OPEN_DEG, abs=0.01)
    assert box['flap_top_m'] == pytest.approx(0.81 + 0.07 * math.cos(math.radians(8.0)), abs=0.002)
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'flap_hinge') >= 0
    assert box['up_m'] == pytest.approx(0.755, abs=0.002)
    # the box mass and friction are what was asked for
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'box')
    assert m.body_mass[bid] == pytest.approx(0.25)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'box_body')
    assert m.geom_friction[gid][0] == pytest.approx(1.0)


def test_seed_jitters_box_and_none_does_not():
    exact, _ = box_scene.box_pose(0.42, 0.21, 0.70, (0.2, 0.15, 0.11), None)
    a, yaw_a = box_scene.box_pose(0.42, 0.21, 0.70, (0.2, 0.15, 0.11), 1)
    b, yaw_b = box_scene.box_pose(0.42, 0.21, 0.70, (0.2, 0.15, 0.11), 2)
    assert np.allclose(exact, box_scene.robot_to_model(0.42, 0.21, 0.755))
    assert not np.allclose(a, b) or yaw_a != yaw_b
    assert np.abs(a - exact).max() <= 0.02 + 1e-9 and abs(yaw_a) <= math.radians(5) + 1e-9


def test_box_settles_on_the_table(world):
    with world.lock:
        before = box_scene.robot_frame_of_box(world.model, world.data)
    world.step(int(0.5 / world.model.opt.timestep))
    with world.lock:
        after = box_scene.robot_frame_of_box(world.model, world.data)
    assert abs(after['up_m'] - before['up_m']) < 0.005
    assert abs(after['forward_m'] - before['forward_m']) < 0.005


# ---------------------------------------------------------------- robot_get_cameras

def test_cameras_render_all_four_with_real_shapes(cams):
    result, images = cams.cameras(['oak', 'left_wrist', 'right_wrist', 'phone'])
    assert result['camera_errors'] == {} and result['all_requested_cameras_fresh'] is True
    assert set(result['cameras']) == {'oak', 'left_wrist', 'right_wrist', 'phone'}
    assert [i['camera_name'] for i in images] == ['oak', 'left_wrist', 'right_wrist', 'phone']
    for record in images:
        name = record['camera_name']
        for key in ('camera_id', 'mime_type', 'data_base64', 'captured_at', 'seq', 'width', 'height', 'sha256',
                    'stream_id'):
            assert key in record, (name, key)
        assert abs(record['captured_at'] - __import__('time').time()) < 5
        pixels = decode_jpeg(record)
        assert pixels.shape[:2] == (record['height'], record['width'])
        assert (pixels.shape[1], pixels.shape[0]) == box_scene.CAMERA_SIZES[name]
        assert pixels.std() > 10, f'{name} render is blank'
        manifest = result['cameras'][name]
        assert manifest['seq'] == record['seq'] and manifest['sha256'] == record['sha256']
    oak = result['cameras']['oak']
    assert oak['projection'] == 'rectified_pinhole' and oak['depth_units'] == 'mm' and oak['invalid_depth'] == 0
    assert oak['width'] == 640 and oak['height'] == 360
    assert np.asarray(oak['intrinsics']).shape == (3, 3)
    # the wrist records carry the arm like the real server
    assert {i.get('arm') for i in images if i['camera_name'].endswith('_wrist')} == {'left', 'right'}


def test_cameras_reports_unknown_names_without_dropping_others(cams):
    result, images = cams.cameras(['phone', 'nope'])
    assert list(result['cameras']) == ['phone'] and len(images) == 1
    assert result['camera_errors'] == {'nope': {'error': 'Unsupported camera', 'fresh': False}}
    assert result['all_requested_cameras_fresh'] is False


# ---------------------------------------------------------------- robot_get_depth

def test_depth_manifest_and_blind_zone(world, cams):
    world.pose_ticks(pregrasp_ticks(), RANGES)
    result, images = cams.depth()
    manifest = result['manifest']
    for key in ('projection', 'intrinsics', 'width', 'height', 'captured_at', 'depth_captured_at', 'seq', 'stream_id',
                'depth_units', 'invalid_depth', 'rgb_depth_pixel_registration_verified', 'camera_id', 'depth_image',
                'depth_sha256'):
        assert key in manifest, key
    assert manifest['projection'] == 'rectified_pinhole'
    assert manifest['rgb_depth_pixel_registration_verified'] is True
    depth_record = next(i for i in images if i['camera_id'].endswith(':depth'))
    depth = decode_depth(depth_record)
    assert depth.shape == (manifest['height'], manifest['width']) == (360, 640)
    raw = cams.render_depth('oak')
    assert raw.shape == depth.shape
    valid = depth > 0
    assert 0.3 < valid.mean() <= 1.0
    # within range: millimetres of the rendered metres (noise under 2 %); blind zone and far background are 0
    inside = (raw >= sim_cameras.DEPTH_MIN_M) & (raw <= sim_cameras.DEPTH_MAX_M)
    both = inside & valid
    assert np.abs(depth[both] / 1000.0 - raw[both]).max() < 0.02 * raw[both].max() + 0.002
    assert (depth[~inside] == 0).all()
    # speckle: some in-range pixels dropped, not many
    dropped = 1.0 - valid[inside].mean()
    assert 0.005 < dropped < 0.05
    # the table/box region (lower middle of the tilted head view) has plausible depths: 0.3-1.5 m
    lower = depth[240:340, 160:480]
    assert (lower > 0).mean() > 0.8
    assert 300 < np.median(lower[lower > 0]) < 1500


def test_depth_scene_nearest_object_is_the_box_or_claw(world, cams):
    world.pose_ticks(pregrasp_ticks(), RANGES)
    result, images = cams.depth()
    depth = decode_depth(next(i for i in images if i['camera_id'].endswith(':depth')))
    pose = cams.camera_pose('oak')
    # the sim camera pose agrees with the twin's camera_pose for the same ticks
    twin_pose = twin.camera_pose(pregrasp_ticks(), RANGES)
    assert np.allclose(pose['position_m'], twin_pose['position_m'], atol=1e-6)
    assert np.allclose(pose['rotation'], twin_pose['rotation'], atol=1e-6)
    scene = depth_scene.scene_points(depth, result['manifest']['intrinsics'], twin_pose)
    assert scene['nearest'] is not None
    with world.lock:
        box = box_scene.robot_frame_of_box(world.model, world.data)
    # query the pixel where the box's top centre projects: it must back-project to the box top within 5 cm
    rot = np.asarray(twin_pose['rotation'])
    pos = np.asarray(twin_pose['position_m'])
    top = np.array([box['forward_m'], box['left_m'], box['top_m']])
    cam = rot.T @ (top - pos)
    k = np.asarray(result['manifest']['intrinsics'])
    u, v = k[0, 0] * cam[0] / cam[2] + k[0, 2], k[1, 1] * cam[1] / cam[2] + k[1, 2]
    assert 0 <= u < 640 and 0 <= v < 360, (u, v)
    query = depth_scene.scene_points(depth, result['manifest']['intrinsics'], twin_pose, pixels=[[int(u), int(v)]])
    point = query['query'][0]['point_m']
    assert point is not None, query['query'][0]
    assert abs(point[2] - box['top_m']) < 0.05 and abs(point[0] - box['forward_m']) < 0.05
    assert abs(point[1] - box['left_m']) < 0.05
    # the nearest blob (the box, flap or the claw above it) sits within the box's neighbourhood
    centre = scene['nearest']['centre_m']
    assert abs(centre[0] - box['forward_m']) < 0.25 and abs(centre[1] - box['left_m']) < 0.25
    assert box['top_m'] - 0.05 < centre[2] < box['top_m'] + 0.15


def test_depth_scene_nearest_is_the_box_when_the_arms_are_out_of_view(world, cams):
    ticks = dict(NEUTRAL, head_motor_2=NEUTRAL['head_motor_2'] + int(HEAD_TILT_DEG * twin.TICKS_PER_TURN / 360))
    world.pose_ticks(ticks, RANGES)
    world.set_joint('Rotation_L', -90)   # both arms swung behind the robot: no arm pixels in the OAK view
    world.set_joint('Rotation_R', 90)
    result, images = cams.depth()
    depth = decode_depth(next(i for i in images if i['camera_id'].endswith(':depth')))
    scene = depth_scene.scene_points(depth, result['manifest']['intrinsics'], twin.camera_pose(ticks, RANGES))
    with world.lock:
        box = box_scene.robot_frame_of_box(world.model, world.data)
    nearest = scene['nearest']
    assert nearest is not None
    centre = nearest['centre_m']
    # the nearest coherent object is the box (its near face and top edge): the blob's centre lies within 5 cm of
    # the box's extent on every axis, and its nearest point is about half a metre from the lens
    depth_m, width_m, height_m = box['size_m']
    assert box['near_face_forward_m'] - 0.05 < centre[0] < box['forward_m'] + 0.05, (centre, box)
    assert abs(centre[1] - box['left_m']) < width_m / 2 + 0.05, (centre, box)
    assert box['up_m'] - height_m / 2 - 0.05 < centre[2] < box['flap_top_m'] + 0.05, (centre, box)
    assert 0.35 < nearest['min_distance_m'] < 0.6


# ---------------------------------------------------------------- robot_get_clip

def test_clip_after_three_seconds_gives_eight_frames_oldest_first(world, cams):
    steps = int(round(3.0 / world.model.opt.timestep))
    world.step(steps)
    result, images = cams.clip({'camera': 'left_wrist', 'seconds': 2, 'fps': 4})
    assert result['count'] == 8 and len(images) == 8
    stamps = [f['sim_time_s'] for f in result['frames']]
    assert stamps == sorted(stamps)
    assert [i['frame_index'] for i in images] == list(range(8))
    assert all(0 <= f['age_s'] <= 2.0 for f in result['frames'])
    assert result['span_s'] == pytest.approx(1.875, abs=0.13)
    assert result['requested'] == {'seconds': 2, 'fps': 4, 'max_width': 480}
    assert 16 <= result['buffer']['frames'] <= 32 and result['buffer']['newest_age_s'] <= 0.126
    for image in images:
        assert image['width'] == 480 and image['height'] == 360
        assert image['camera_name'] == 'left_wrist' and image['mime_type'] == 'image/jpeg'
        assert decode_jpeg(image).shape == (360, 480, 3)


def test_clip_argument_errors(cams):
    with pytest.raises(ValueError):
        cams.clip({'camera': 'nope'})
    with pytest.raises(ValueError):
        cams.clip({'camera': 'oak', 'seconds': 9})
    with pytest.raises(ValueError):
        cams.clip({'camera': 'oak', 'max_width': 320.0})


def test_clip_without_frames_is_unavailable():
    world = sim_cameras.StaticWorld(seed=None, preset='near7')
    with sim_cameras.SimCameras(world) as cams:
        with pytest.raises(sim_cameras.ClipUnavailable):
            cams.clip({'camera': 'oak'})


# ---------------------------------------------------------------- the pregrasp wrist view

def brown_mask(pixels):
    r, g, b = (pixels[..., i].astype(int) for i in range(3))
    return (r > 120) & (r > g + 15) & (g > b + 15) & (b < 190)


def test_left_wrist_sees_the_box_edge_between_the_jaws_at_pregrasp(world, cams):
    world.pose_ticks(pregrasp_ticks(), RANGES)
    paths = cams.save_previews(PREVIEW_DIR)
    assert all(p.exists() for p in paths)
    pixels = cams.render('left_wrist')
    assert pixels.shape == (480, 640, 3)
    h, w = pixels.shape[:2]
    lower_centre = pixels[int(h * 0.5):int(h * 0.85), int(w * 0.3):int(w * 0.7)]
    brown = brown_mask(lower_centre).mean()
    assert brown > 0.08, f'box (brown) pixels missing from the lower-centre of the left_wrist view: {brown:.3f}'
    # the jaws (left arm colour, orange) occupy the bottom of the frame
    bottom = pixels[int(h * 0.85):, int(w * 0.2):int(w * 0.8)]
    r, g, b = (bottom[..., i].astype(int) for i in range(3))
    orange = ((r > 150) & (g > 60) & (g < 190) & (b < 110)).mean()
    assert orange > 0.05, f'jaws missing from the bottom of the left_wrist view: {orange:.3f}'
