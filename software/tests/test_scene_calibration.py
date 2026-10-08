"""Table-plane self-calibration of the head camera pose against the simulated OAK (whose true pose is known) and
through the robot_get_scene_points tool with the owner's workspace.json."""
import base64
import io
import json
import os
import sys
import time

import numpy as np
import pytest

if sys.platform == 'darwin':
    os.environ.setdefault('MUJOCO_GL', 'cgl')

mujoco = pytest.importorskip('mujoco')
PIL = pytest.importorskip('PIL')
from PIL import Image  # noqa: E402

from farm.perception import depth_scene as ds  # noqa: E402
from farm.perception import twin_robot  # noqa: E402
from farm.perception.twin_robot import SCENE_TOOL_NAME, TwinRobot  # noqa: E402
from farm.sim import box_scene, sim_cameras  # noqa: E402
from farm.sim import xlerobot_twin as twin  # noqa: E402
from test_depth_scene import UP, project, rotate_about, rotation_error_deg  # noqa: E402

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
MOTORS = [f'{side}_arm_{j}' for side in ('left', 'right') for j in ARM] + ['head_motor_1', 'head_motor_2']
RANGES = {m: (1000, 3000) for m in MOTORS}
NEUTRAL = {m: (lo + hi) // 2 for m, (lo, hi) in RANGES.items()}
TABLE_TOP_M = 0.70            # box_scene default; the real table
TILT_ERROR_DEG = 15.0         # the "model" is this much too level ...
HEIGHT_ERROR_M = 0.03         # ... and this much too high
ROLL_ERROR_DEG = 2.0


def head_ticks(tilt_deg):
    """Neutral arms, head tilted down by tilt_deg (head_motor_2 above its midpoint)."""
    return dict(NEUTRAL, head_motor_2=NEUTRAL['head_motor_2'] + int(round(tilt_deg * twin.TICKS_PER_TURN / 360)))


@pytest.fixture(scope='module')
def world():
    return sim_cameras.StaticWorld(seed=0)


@pytest.fixture(scope='module')
def cams(world):
    cameras = sim_cameras.SimCameras(world)
    yield cameras
    cameras.close()


def decode_depth(record):
    assert record['mime_type'] == 'image/png'
    depth = np.asarray(Image.open(io.BytesIO(base64.b64decode(record['data_base64']))))
    assert depth.dtype == np.uint16 and depth.ndim == 2
    return depth


def capture(world, cams, tilt_deg):
    """(depth mm, manifest, true oak pose, box) with the head tilted and both arms swung behind the robot."""
    world.pose_ticks(head_ticks(tilt_deg), RANGES)
    world.set_joint('Rotation_L', -90)
    world.set_joint('Rotation_R', 90)
    result, images = cams.depth()
    depth = decode_depth(next(i for i in images if i['camera_id'].endswith(':depth')))
    with world.lock:
        box = box_scene.robot_frame_of_box(world.model, world.data)
    return depth, result['manifest'], cams.camera_pose('oak'), box


def perturb(true, tilt_error_deg=TILT_ERROR_DEG, height_error_m=HEIGHT_ERROR_M, roll_error_deg=ROLL_ERROR_DEG):
    """A wrong 'model' pose: the true one tilted up by tilt_error_deg, rolled, and raised by height_error_m."""
    rot = np.array(true['rotation'], dtype=float)
    rot = rotate_about(rot, rot[:, 0], tilt_error_deg)      # about image-right: + lifts the optical axis
    rot = rotate_about(rot, rot[:, 2], roll_error_deg)
    position = list(true['position_m'])
    position[2] += height_error_m
    return {**true, 'position_m': position, 'rotation': rot.tolist()}


@pytest.mark.parametrize('tilt', [25.0, 40.0, 55.0])
def test_sim_table_plane_recovers_the_true_camera_pose_and_the_box_top(world, cams, tilt):
    depth, manifest, true, box = capture(world, cams, tilt)
    wrong = perturb(true)
    assert ds.tilt_deg(wrong['rotation']) == pytest.approx(ds.tilt_deg(true['rotation']) - TILT_ERROR_DEG, abs=0.01)
    intrinsics = manifest['intrinsics']
    wrong_rot = np.array(wrong['rotation'])
    plane = ds.fit_table_plane(depth, intrinsics, expected_up_cam=wrong_rot.T @ UP,
                               expected_d_m=wrong['position_m'][2] - TABLE_TOP_M)
    assert plane['ok'] and plane['inlier_fraction'] > 0.3, plane
    cal = ds.calibrate_camera_pose(plane, wrong, TABLE_TOP_M)
    assert cal['ok'] and cal['method'] == 'table_plane', cal
    # tilt within 1 deg, roll within 1 deg, height within 1 cm of the truth
    assert abs(cal['tilt_deg'] - ds.tilt_deg(true['rotation'])) < 1.0
    assert abs(cal['roll_deg'] - ds.roll_deg(true['rotation'])) < 1.0
    assert abs(cal['position_m'][2] - true['position_m'][2]) < 0.01
    assert rotation_error_deg(cal['rotation'], true['rotation']) < 1.0
    assert cal['tilt_correction_deg'] == pytest.approx(TILT_ERROR_DEG, abs=1.0)
    assert cal['height_correction_m'] == pytest.approx(-HEIGHT_ERROR_M, abs=0.01)
    # the box top (0.81 m) through the calibrated pose, at the pixel where it really is
    pixel = project(true, [box['forward_m'], box['left_m'], box['top_m']], intrinsics)
    scene = ds.scene_points(depth, intrinsics, cal, pixels=[pixel], table_top_m=TABLE_TOP_M)
    point = scene['query'][0]['point_m']
    assert point is not None and abs(point[2] - 0.81) < 0.02 and abs(point[2] - box['top_m']) < 0.02, (point, box)
    assert abs(point[0] - box['forward_m']) < 0.02 and abs(point[1] - box['left_m']) < 0.02
    # the blob's top is the open flap's edge, or the box top when the 3.5 mm flap is too thin to resolve at this tilt
    assert box['top_m'] - 0.03 < scene['nearest']['top_m'] < box['flap_top_m'] + 0.03
    assert scene['nearest']['top_above_table_m'] == pytest.approx(scene['nearest']['top_m'] - TABLE_TOP_M, abs=0.001)
    # the wrong pose had the top more than 5 cm off (as on 8 October)
    bad = ds.scene_points(depth, intrinsics, wrong, pixels=[pixel])['query'][0]['point_m']
    assert abs(bad[2] - 0.81) > 0.05
    # the table's pixels land at the table height
    check = ds.table_check(depth, intrinsics, cal, plane)
    assert check['median_up_m'] == pytest.approx(TABLE_TOP_M, abs=0.002)
    assert 0.29 <= check['forward_range_m'][0] and check['forward_range_m'][1] <= 0.91


# ---------------------------------------------------------------- through the scene tool

def tool(name, properties=None):
    return {"type": "function", "function": {"name": name, "description": name,
            "parameters": {"type": "object", "properties": properties or {}, "additionalProperties": False}}}


class SimRobot:
    """A chat robot client answering robot_get_state with ticks and robot_get_depth from the simulated OAK."""

    def __init__(self, cams, ticks, config):
        self.cams = cams
        self.ticks = dict(ticks)
        self.config = config
        self.tools = [tool("robot_get_state", {"fresh": {"type": "boolean"}}), tool("robot_get_depth"),
                      tool("robot_stop")]
        self.calls = []

    def catalog(self):
        return {"tools": [dict(t) for t in self.tools], "metadata": {}}

    def call(self, name, args, request_id=None):
        self.calls.append(name)
        now = time.time()
        if name == "robot_get_state":
            return {"ok": True, "result": {
                "source": "sim", "time": now, "cached": False,
                "motors": [{"name": n, "Present_Position": p, "captured_at": now, "Torque_Enable": 0}
                           for n, p in self.ticks.items()],
                "raw_calibration_ranges": {n: {"min_ticks": lo, "max_ticks": hi} for n, (lo, hi) in RANGES.items()},
                "commandable_ranges": {n: [lo + 40, hi - 40] for n, (lo, hi) in RANGES.items()}}}
        if name == "robot_get_depth":
            result, images = self.cams.depth()
            return {"ok": True, "result": result, "images": images}
        return {"ok": True, "result": {"forwarded": name}}


def no_claws(positions_ticks, ranges, *, joint_map=None):
    return {"left_arm": None, "right_arm": None, "mapping": "feetech_degrees_v1", "mapping_validated": False,
            "unmapped": [], "model": "sim"}


@pytest.fixture
def scene_tool(world, cams, tmp_path):
    """(TwinRobot over the sim with a WRONG model camera pose, true pose, box, pixel of the box top, tmp dir)."""
    depth, manifest, true, box = capture(world, cams, 40.0)
    wrong = perturb(true)
    wrong.update(head_angles_deg={'pan': 0.0, 'tilt': 40.0 - TILT_ERROR_DEG}, site='test', mapping='feetech_degrees_v1',
                 mapping_validated=False, unmapped=[], model='sim')

    def wrong_pose(positions_ticks, ranges, *, joint_map=None, camera='oak'):
        return dict(wrong)

    robot = SimRobot(cams, head_ticks(40.0), tmp_path / 'robot.json')
    twin_tool = TwinRobot(robot, camera_pose=wrong_pose, claws=no_claws)
    pixel = project(true, [box['forward_m'], box['left_m'], box['top_m']], manifest['intrinsics'])
    return twin_tool, true, box, pixel, tmp_path


def test_scene_tool_calibrates_on_the_table_plane_when_workspace_json_gives_the_table_height(scene_tool):
    twin_tool, true, box, pixel, folder = scene_tool
    (folder / twin_robot.WORKSPACE_NAME).write_text(json.dumps({"table_top_m": 0.7, "object_top_m": 0.81}))
    assert twin_tool.workspace_path == folder / 'workspace.json'
    answer = twin_tool.call(SCENE_TOOL_NAME, {"pixels": [pixel], "with_claws": False})
    assert answer["ok"] is True, answer
    result = answer["result"]
    assert result["camera_pose_source"] == "table_plane" and result["camera"]["source"] == "table_plane"
    assert result["table_top_m"] == 0.7 and result["table_top_source"].endswith("workspace.json")
    plane = result["table_plane"]
    assert plane["ok"] is True and plane["inlier_fraction"] > 0.3
    assert plane["tilt_correction_deg"] == pytest.approx(TILT_ERROR_DEG, abs=1.0)
    assert plane["height_correction_m"] == pytest.approx(-HEIGHT_ERROR_M, abs=0.01)
    assert result["camera"]["tilt_correction_deg"] == plane["tilt_correction_deg"]
    assert abs(result["camera"]["tilt_deg"] - ds.tilt_deg(true["rotation"])) < 1.0
    assert abs(result["camera"]["position_m"][2] - true["position_m"][2]) < 0.01
    assert result["camera"]["model_position_m"][2] == pytest.approx(true["position_m"][2] + HEIGHT_ERROR_M, abs=0.001)
    assert result["table_check"]["median_up_m"] == pytest.approx(0.7, abs=0.002)
    point = result["query"][0]["point_m"]
    assert abs(point[2] - 0.81) < 0.02, point
    nearest = result["nearest"]
    assert box["top_m"] - 0.03 < nearest["top_m"] < box["flap_top_m"] + 0.03   # box top or the open flap's edge
    assert nearest["top_above_table_m"] == pytest.approx(nearest["top_m"] - 0.7, abs=0.001)
    assert "height_above_table_m" in nearest
    assert result["note"].startswith("robot-frame positions use the head-camera pose self-calibrated on the table plane")
    assert "tilt corrected by +1" in result["note"] and "table top 0.70 m" in result["note"]
    assert "tilt" in result["camera_pose_reason"] and result["motor_writes"] == 0
    json.dumps(answer, allow_nan=False)


def test_scene_tool_keeps_the_model_pose_without_a_table_height(scene_tool):
    twin_tool, true, box, pixel, folder = scene_tool
    result = twin_tool.call(SCENE_TOOL_NAME, {"pixels": [pixel], "with_claws": False})["result"]
    assert result["camera_pose_source"] == "model" and "not found" in result["camera_pose_reason"]
    assert result["table_top_m"] is None and result["table_plane"] is None and result["table_check"] is None
    assert "top_m" not in result["nearest"] and "height_above_table_m" not in result["nearest"]
    assert abs(result["query"][0]["point_m"][2] - 0.81) > 0.05       # the wrong model pose, as before
    assert result["note"] == twin_robot.SCENE_NOTE


def test_scene_tool_takes_the_table_height_from_the_arguments(scene_tool):
    twin_tool, true, box, pixel, folder = scene_tool
    result = twin_tool.call(SCENE_TOOL_NAME, {"pixels": [pixel], "with_claws": False, "table_top_m": 0.7})["result"]
    assert result["camera_pose_source"] == "table_plane" and result["table_top_source"] == "table_top_m from the tool arguments"
    assert abs(result["query"][0]["point_m"][2] - 0.81) < 0.02


def test_scene_tool_falls_back_to_the_model_pose_when_the_table_height_is_wrong(scene_tool):
    twin_tool, true, box, pixel, folder = scene_tool
    result = twin_tool.call(SCENE_TOOL_NAME, {"with_claws": False, "table_top_m": 0.3})["result"]
    assert result["camera_pose_source"] == "model" and result["table_top_m"] == 0.3
    assert result["table_plane"] is not None and result["table_check"] is None
    assert "table-plane calibration NOT applied" in result["note"]
    assert "top_m" not in result["nearest"]
    json.dumps(result, allow_nan=False)


def test_scene_tool_refuses_a_bad_table_height(scene_tool):
    twin_tool, *_ = scene_tool
    for bad in ({"table_top_m": 0}, {"table_top_m": 2}, {"table_top_m": "0.7"}, {"table_top_m": True}):
        answer = twin_tool.call(SCENE_TOOL_NAME, bad)
        assert answer["ok"] is False and "table_top_m" in answer["result"]["error"]


def test_malformed_workspace_json_is_reported_not_raised(scene_tool):
    twin_tool, true, box, pixel, folder = scene_tool
    (folder / twin_robot.WORKSPACE_NAME).write_text("{not json")
    result = twin_tool.call(SCENE_TOOL_NAME, {"with_claws": False})["result"]
    assert result["camera_pose_source"] == "model" and "not valid JSON" in result["camera_pose_reason"]
    (folder / twin_robot.WORKSPACE_NAME).write_text(json.dumps({"table_top_m": 5}))
    result = twin_tool.call(SCENE_TOOL_NAME, {"with_claws": False})["result"]
    assert result["camera_pose_source"] == "model" and "no table_top_m between 0 and 2" in result["camera_pose_reason"]
