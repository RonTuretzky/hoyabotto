"""SimRobot head: enable/release head_motor_1/2, robot_move_head rules (the real owner's --head scope, qwen-bridge
head_joint_executor.py / gemma_robot_tools.move_head), and the OAK camera following the head."""
import base64
import io
import math

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')

from farm.sim import box_scene  # noqa: E402
from farm.sim import sim_robot  # noqa: E402
from farm.sim.sim_robot import SimRobot  # noqa: E402

HEAD = ['head_motor_1', 'head_motor_2']
LEFT = [f'left_arm_{j}' for j in sim_robot.ARM_JOINTS]


@pytest.fixture
def robot():
    # head tilted 15 deg down (inside head_motor_2's saved range, so +100 ticks of tilt is still commandable)
    r = SimRobot(real_time=False, fidelity={'head_tilt_deg': 15.0})
    yield r
    r.close()


def ok(res):
    assert res['ok'] is True, res
    return res['result']


def refused(res):
    assert res['ok'] is False and res['http_status'] == 400, res
    return res['result']['error']


def enable_head(r, names=HEAD):
    return ok(r.call('robot_set_motor_enable', {'names': list(names), 'enabled': True}))


def oak_view(r):
    """The oak camera's viewing direction (optical +z) and position in the robot frame (forward, left, up)."""
    with r.lock:
        mujoco.mj_forward(r.model, r.data)
        pose = box_scene.camera_pose_from_model(r.model, r.data, 'oak')
    rotation = np.asarray(pose['rotation'])
    return rotation[:, 2], np.asarray(pose['position_m'])


def oak_pixels(r):
    res = r.call('robot_get_cameras', {'cameras': ['oak']})
    assert res['ok'] is True, res
    from PIL import Image
    record = next(i for i in res['images'] if i['camera_name'] == 'oak')
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(record['data_base64']))).convert('RGB')).astype(float)


def test_head_enables_alone_or_together_and_releases(robot):
    res = enable_head(robot, ['head_motor_1'])
    assert res['completed'] is True and res['readbacks'] == {'head_motor_1': 1}
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == ['head_motor_1']
    enable_head(robot)
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == HEAD
    before = robot.positions()
    ok(robot.call('robot_set_motor_enable', {'names': ['head_motor_2'], 'enabled': False}))
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == ['head_motor_1']
    after = robot.positions()
    assert all(abs(after[n] - before[n]) <= 1 for n in HEAD)   # enable/release hold the head where it is
    # the wheels stay read-only
    error = refused(robot.call('robot_set_motor_enable', {'names': ['base_left_wheel'], 'enabled': True}))
    assert error.startswith('UNSUPPORTED_OWNER_SCOPE: these motors are read-only and are never powered: base_left_wheel')


def test_arm_moves_never_require_the_head_and_head_is_not_in_the_six(robot):
    ok(robot.call('robot_set_motor_enable', {'names': LEFT, 'enabled': True}))
    pos = robot.positions()
    res = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'shoulder_pan': pos['left_arm_shoulder_pan'] + 60}, 'duration_s': 3}))
    assert res['completed'] is True
    enable_head(robot)
    partial = robot.call('robot_set_motor_enable', {'names': LEFT[:5] + HEAD, 'enabled': True})
    assert partial['ok'] is True


def test_move_head_pan_100_and_back(robot):
    enable_head(robot)
    start = robot.positions()['head_motor_1']
    res = ok(robot.call('robot_move_head', {'positions': {'head_motor_1': start + 100}, 'duration_s': 1}))
    assert res['accepted'] is True and res['completed'] is True and res['endpoint_reached'] is True
    assert res['closure_outcome'] == 'endpoint_settled' and abs(res['readbacks']['head_motor_1'] - (start + 100)) <= 2
    assert abs(robot.positions()['head_motor_1'] - (start + 100)) <= 2
    back = ok(robot.call('robot_move_head', {'positions': {'head_motor_1': start}}))   # default: the minimum, at least 1 s
    assert back['completed'] is True and abs(back['readbacks']['head_motor_1'] - start) <= 2 and back['duration_s'] == 1.0
    assert back['head_targets'] == {'head_motor_1': start}
    # joints already within 2 ticks are dropped; nothing left is a no-op, not a refusal
    noop = ok(robot.call('robot_move_head', {'positions': {'head_motor_1': start + 2}, 'duration_s': 1}))
    assert noop['no_op'] is True and noop['motor_writes'] == 0
    assert robot.counts['head_moves'] == 2


def test_move_head_refusals(robot):
    start = robot.positions()
    released = refused(robot.call('robot_move_head', {'positions': {'head_motor_1': start['head_motor_1'] + 50}, 'duration_s': 1}))
    assert released == 'Requested motor is released; explicitly enable it first: head_motor_1'
    enable_head(robot)
    far = refused(robot.call('robot_move_head', {'positions': {'head_motor_1': start['head_motor_1'] + 201}, 'duration_s': 5}))
    assert far == 'Head move: head_motor_1 travels 201 ticks; at most 200 per move'
    quick = refused(robot.call('robot_move_head', {'positions': {'head_motor_1': start['head_motor_1'] + 150}, 'duration_s': 1}))
    assert quick == 'Head move duration_s must be at least 1 s per 100 ticks: 150 ticks needs 1.50 s, got 1'
    hi = robot.calibration['head_motor_2'][1]
    outside = refused(robot.call('robot_move_head', {'positions': {'head_motor_2': hi - 39}, 'duration_s': 5}))
    assert outside == f'Target out of bounds: head_motor_2={hi - 39}; commandable inclusive range [1972, {hi - 40}] ticks (40-tick margin)'
    arm = refused(robot.call('robot_move_head', {'positions': {'left_arm_shoulder_pan': 2000}, 'duration_s': 3}))
    assert arm == 'Head tool accepts head motors only: left_arm_shoulder_pan'
    ok(robot.call('robot_set_motor_enable', {'names': LEFT, 'enabled': True}))
    joint = refused(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'head_motor_1': start['head_motor_1'] + 20}, 'duration_s': 3}))
    assert joint == 'Wrong-arm joint: head_motor_1; requested arm: left'   # the real bridge's wording (arm is required)
    path = refused(robot.call('robot_move_path', {'arm': 'left', 'waypoints': [{'head_motor_2': start['head_motor_2'] + 20}], 'duration_s': 3}))
    assert path == 'Wrong-arm joint: head_motor_2; requested arm: left'
    assert robot.positions()['head_motor_1'] == start['head_motor_1']   # nothing moved


def test_capabilities_and_catalog_report_the_head(robot):
    caps = ok(robot.call('robot_get_capabilities', {}))
    assert caps['head_supported'] is True
    assert caps['head_commandable_ranges']['head_motor_1'] == {'min_ticks': 1059, 'max_ticks': 3111, 'margin_ticks': 40}
    assert caps['head_commandable_ranges']['head_motor_2'] == {'min_ticks': 1972, 'max_ticks': 2625, 'margin_ticks': 40}
    tools = {t['function']['name']: t['function'] for t in robot.catalog()['tools']}
    head = tools['robot_move_head']['parameters']
    assert head['required'] == ['positions'] and set(head['properties']) == {'positions', 'duration_s'}
    assert head['properties']['positions']['properties']['head_motor_2'] == {'type': 'integer', 'minimum': 1936, 'maximum': 2661}  # raw +-4, as the real bridge
    assert caps['head_move_limits']['max_ticks_per_move'] == 200 and caps['head_move_limits']['min_duration_s_per_100_ticks'] == 1
    assert 'read-only' not in tools['robot_set_motor_enable']['description'].split('Wheels')[0]
    assert 'robot_move_head' in tools['robot_set_motor_enable']['description']


def test_stop_releases_the_head(robot):
    enable_head(robot)
    ok(robot.call('robot_stop', {}))
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == []
    error = refused(robot.call('robot_move_head', {'positions': {'head_motor_1': robot.positions()['head_motor_1'] + 50}, 'duration_s': 1}))
    assert error.startswith('Requested motor is released')


def test_oak_follows_the_head(robot):
    enable_head(robot)
    view0, pos0 = oak_view(robot)
    pixels0 = oak_pixels(robot)
    with robot.lock:
        pan_q0 = float(robot.data.qpos[robot.motors['head_motor_1'].qadr])
        tilt_q0 = float(robot.data.qpos[robot.motors['head_motor_2'].qadr])
    start = robot.positions()
    # pan +100 ticks: the view turns to the robot's LEFT (about 8.8 deg)
    ok(robot.call('robot_move_head', {'positions': {'head_motor_1': start['head_motor_1'] + 100}, 'duration_s': 1}))
    view1, _ = oak_view(robot)
    with robot.lock:
        pan_q1 = float(robot.data.qpos[robot.motors['head_motor_1'].qadr])
    assert pan_q1 - pan_q0 == pytest.approx(math.radians(100 * 360 / 4096), abs=0.01)
    yaw = lambda v: math.degrees(math.atan2(v[1], v[0]))  # noqa: E731
    assert yaw(view1) - yaw(view0) == pytest.approx(100 * 360 / 4096, abs=1.5)
    pixels1 = oak_pixels(robot)
    assert np.abs(pixels1 - pixels0).mean() > 3, 'the rendered oak view did not change after a pan'
    # tilt +100 ticks: it looks further DOWN
    ok(robot.call('robot_move_head', {'positions': {'head_motor_2': start['head_motor_2'] + 100}, 'duration_s': 1}))
    view2, _ = oak_view(robot)
    with robot.lock:
        tilt_q2 = float(robot.data.qpos[robot.motors['head_motor_2'].qadr])
    assert tilt_q2 - tilt_q0 == pytest.approx(math.radians(100 * 360 / 4096), abs=0.01)
    pitch = lambda v: math.degrees(math.asin(v[2] / np.linalg.norm(v)))  # noqa: E731
    assert pitch(view2) < pitch(view1) - 5
    assert np.abs(oak_pixels(robot) - pixels1).mean() > 3


def test_default_fidelity_head_starts_45_down_and_can_come_back_inside_the_range():
    r = SimRobot(real_time=False)
    try:
        lo, hi = r.calibration['head_motor_2']
        start = r.positions()['head_motor_2']
        assert start == (lo + hi) // 2 + 512    # 45 deg down, above the saved range_max
        view, _ = oak_view(r)
        assert math.degrees(math.asin(view[2])) == pytest.approx(-45, abs=3)
        enable_head(r, ['head_motor_2'])
        assert refused(r.call('robot_move_head', {'positions': {'head_motor_2': hi - 39}, 'duration_s': 3})).startswith(
            'Target out of bounds: head_motor_2=')
        res = ok(r.call('robot_move_head', {'positions': {'head_motor_2': hi - 40}, 'duration_s': 2}))
        assert res['readbacks']['head_motor_2'] == hi - 40
    finally:
        r.close()
