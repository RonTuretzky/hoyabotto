"""SimRobot: the simulated XLeRobot behind the pilot chat server's tool API (MuJoCo; skipped without it)."""
import json
import time

import pytest

mujoco = pytest.importorskip('mujoco')

from farm.sim import sim_robot  # noqa: E402
from farm.sim.sim_robot import SimRobot  # noqa: E402

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
LEFT = [f'left_arm_{j}' for j in ARM]
BOX_SIZE = (0.20, 0.15, 0.11)   # the scene builder's default box
GRASP_PITCH_DEG = -60.0


@pytest.fixture
def robot():
    r = SimRobot(real_time=False)
    yield r
    r.close()


def ok(res):
    assert res['ok'] is True, res
    return res['result']


def enable_left(r):
    return ok(r.call('robot_set_motor_enable', {'names': LEFT, 'enabled': True}))


def reach(r, forward, left, up, pitch):
    from farm.kinematics.so101_reach import solve_reach
    sol = solve_reach('left', {'forward_m': forward, 'left_m': left, 'up_m': up}, r.positions(), r.ranges(), pitch_deg=pitch)
    assert sol['ok'], sol['reason']
    return sol['ticks']


def box_frame(r):
    """near face forward, left, top height of the box from its centre pose and the default size."""
    b = r.box_pose()
    return b['forward_m'] - BOX_SIZE[0] / 2, b['left_m'], b['up_m'] + BOX_SIZE[2] / 2


def grasp(r, frame=None):
    """Scripted grasp from the validated recipe: pregrasp above the near top edge, lower so the fixed jaw sits just in
    front of the flap and the moving jaw closes onto the box's near top edge, then close. Returns the close result."""
    near, left, top = frame or box_frame(r)
    enable_left(r)
    pre = reach(r, near - 0.005, left, top + 0.07, GRASP_PITCH_DEG)
    pre['left_arm_gripper'] = 2500
    assert ok(r.call('robot_move_joint_targets', {'arm': 'left', 'positions': pre, 'duration_s': 8}))['completed'] is True
    low = reach(r, near - 0.005, left, top, GRASP_PITCH_DEG)
    assert ok(r.call('robot_move_joint_targets', {'arm': 'left', 'positions': low, 'duration_s': 6}))['completed'] is True
    return ok(r.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 1400, 'duration_s': 5}))


# ---------------------------------------------------------------- catalog and state shapes

def test_catalog_has_the_real_schemas(robot):
    cat = robot.catalog()
    names = [t['function']['name'] for t in cat['tools']]
    assert set(names) == set(sim_robot.IMPLEMENTED_TOOLS)
    assert cat['metadata']['stop_tool'] == 'robot_stop'
    by_name = {t['function']['name']: t['function']['parameters'] for t in cat['tools']}
    move = by_name['robot_move_joint_targets']
    assert move['required'] == ['arm', 'positions', 'duration_s']
    assert set(move['properties']) == {'arm', 'positions', 'duration_s', 'wait', 'replace'}
    lo, hi = robot.calibration['left_arm_shoulder_lift']
    assert move['properties']['positions']['properties']['left_arm_shoulder_lift'] == {'type': 'integer', 'minimum': lo + 4, 'maximum': hi - 4}
    assert 'allOf' in move and 'allOf' in by_name['robot_set_gripper']
    assert by_name['robot_move_path']['properties']['waypoints']['maxItems'] == 12
    assert by_name['robot_move_base']['properties']['linear_m_s']['maximum'] == 0.02
    assert json.dumps(cat)  # serialisable


def test_state_matches_the_real_sample_shape(robot):
    sample = json.loads(sim_robot.STATE_SAMPLE.read_text())['result']
    state = ok(robot.call('robot_get_state', {'fresh': False}))
    assert set(state) == set(sample)
    assert [m['name'] for m in state['motors']] == [m['name'] for m in sample['motors']]
    for row in state['motors']:
        assert set(row) == set(sample['motors'][0])
        assert type(row['Present_Position']) is int and 0 <= row['Present_Position'] <= 4095
        assert row['Torque_Enable'] == 0 and row['Present_Load'] == 0
    assert state['all_16_released'] is True and state['enabled_motors'] == []
    assert state['commandable_ranges']['left_arm_gripper'] == sample['commandable_ranges']['left_arm_gripper']
    assert state['raw_calibration_ranges'] == sample['raw_calibration_ranges']
    caps = ok(robot.call('robot_get_capabilities', {}))
    assert caps['execution_profile'] == 'paddle-success-v1' and caps['base_drive_limits'] == {'max_wheel_m_s': 0.02, 'max_duration_s': 3.0}
    assert robot.get('/health')['ok'] is True


# ---------------------------------------------------------------- moves and refusals

def test_enable_then_move_reaches_target(robot):
    pos = robot.positions()
    target = {'left_arm_shoulder_lift': pos['left_arm_shoulder_lift'] + 200, 'left_arm_elbow_flex': pos['left_arm_elbow_flex'] - 150}
    refused = robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': target, 'duration_s': 4})
    assert refused['ok'] is False and refused['http_status'] == 400
    assert refused['result']['error'] == 'Requested motor is released; explicitly enable it first: left_arm_shoulder_lift'
    res = enable_left(robot)
    assert res['completed'] is True and res['readbacks'] == {n: 1 for n in LEFT}
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == sorted(LEFT)
    res = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': target, 'duration_s': 4}))
    assert res['completed'] is True and res['closure_outcome'] == 'endpoint_settled' and res['endpoint_reached'] is True
    for name, want in target.items():
        assert type(res['readbacks'][name]) is int
        assert abs(res['readbacks'][name] - want) <= 40
        assert abs(res['settle_residual_ticks'][name]) <= 40
    assert res['motor_writes'] == 'canonical owner only' and res['mode'] == 'direct_joint'
    # a second (lagging) joint pair, with aliases and the no-op path
    again = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'shoulder_lift': target['left_arm_shoulder_lift']}, 'duration_s': 3}))
    assert again.get('no_op') is True or again['completed'] is True


def test_target_outside_commandable_range_is_refused(robot):
    enable_left(robot)
    lo, hi = robot.calibration['left_arm_shoulder_lift']
    res = robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'left_arm_shoulder_lift': lo + 10}, 'duration_s': 3})
    assert res['ok'] is False and res['http_status'] == 400
    assert res['result']['error'] == (f'Target out of bounds: left_arm_shoulder_lift={lo + 10}; commandable inclusive range '
                                      f'[{lo + 40}, {hi - 40}] ticks (40-tick margin)')
    wrong = robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'right_arm_shoulder_lift': 2000}, 'duration_s': 3})
    assert wrong['result']['error'] == 'Wrong-arm joint: right_arm_shoulder_lift; requested arm: left'
    head = robot.call('robot_set_motor_enable', {'names': ['head_motor_1'], 'enabled': True})
    assert head['ok'] is False and head['result']['error'].startswith('UNSUPPORTED_OWNER_SCOPE: these motors are read-only')
    partial = SimRobot(real_time=False)
    try:
        ok(partial.call('robot_set_motor_enable', {'names': LEFT[:5], 'enabled': True}))
        res = partial.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'shoulder_lift': 1500}, 'duration_s': 3})
        assert res['result']['error'] == 'Pickup requires all six joints of the commanded arm explicitly enabled: ["left_arm_gripper"]'
    finally:
        partial.close()
    assert robot.score()['refusals'] == 3   # the partial robot counts its own


def test_driving_into_the_table_faults_and_releases_everything(robot):
    enable_left(robot)
    pre = reach(robot, 0.30, 0.0, 0.80, -60)
    assert ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': pre, 'duration_s': 8}))['completed'] is True
    hit = reach(robot, 0.30, 0.0, 0.66, -60)   # 4 cm below the table top
    res = robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': hit, 'duration_s': 8})
    assert res['ok'] is False and res['http_status'] == 409
    assert res['result']['error'].startswith('Owner stopped: Pickup following error exceeds96ticks: left_arm_')
    assert res['result']['motor_writes'] == 'not_observed_by_bridge'
    state = ok(robot.call('robot_get_state', {}))
    assert state['all_16_released'] is True and state['enabled_motors'] == []
    score = robot.score()
    assert score['faults'] == 1 and score['released_all'] is True
    assert ok(robot.call('robot_get_motion', {}))['last_stop']['reason'].startswith('Pickup following error exceeds96ticks')
    assert robot.claw_positions()['left_arm']['up_m'] > 0.69   # the claw stopped on the table, not through it


def test_wait_false_motion_is_observable_then_settles(robot):
    enable_left(robot)
    pos = robot.positions()
    target = {'left_arm_shoulder_pan': pos['left_arm_shoulder_pan'] + 60}
    res = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': target, 'duration_s': 3, 'wait': False}))
    assert res['accepted'] is True and res['phase'] == 'moving' and res['completed'] is False
    motion = ok(robot.call('robot_get_motion', {}))
    assert motion['phase'] == 'moving' and motion['moving'] is True and motion['running_command_id'] == res['command_id']
    assert set(motion['joints']['left_arm_shoulder_pan']) == {'current_ticks', 'goal_ticks', 'target_ticks', 'following_error_ticks'}
    busy = ok(robot.call('robot_move_base', {'linear_m_s': 0.01, 'angular_rad_s': 0.0, 'duration_s': 1}))
    assert busy['accepted'] is False and busy['reason'] == 'OWNER_BUSY: moving'
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        motion = ok(robot.call('robot_get_motion', {}))
        if not motion['moving']:
            break
        time.sleep(0.2)
    assert motion['moving'] is False and motion['phase'] == 'holding'
    assert motion['closure_outcome'] == 'endpoint_settled' and motion['last_completed_command_id'] == res['command_id']
    assert abs(robot.positions()['left_arm_shoulder_pan'] - target['left_arm_shoulder_pan']) <= 40
    halt = ok(robot.call('robot_halt_motion', {}))
    assert halt['halted'] is True and halt['halted_command_id'] is None


def test_stop_releases_and_cancels(robot):
    enable_left(robot)
    pos = robot.positions()
    ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {'shoulder_pan': pos['left_arm_shoulder_pan'] + 100}, 'duration_s': 5, 'wait': False}))
    res = ok(robot.call('robot_stop', {}))
    assert res['stop_requested'] is True and res['release_confirmed'] is True and res['stop_latched'] is False
    assert ok(robot.call('robot_get_state', {}))['all_16_released'] is True
    assert ok(robot.call('robot_get_motion', {}))['moving'] is False


# ---------------------------------------------------------------- gripper, base, score

def test_gripper_closing_on_nothing_reaches_target(robot):
    opened = ok(robot.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 2000, 'duration_s': 4}))
    # 580 ticks run as two <=300-tick parts; the result is the last part's (so auto-enable shows on the first part only)
    assert opened['completed'] is True and len(opened['closure_parts']) == 2 and opened['final_target'] == 2000
    assert opened['gripper'] == 'left_arm_gripper' and opened['sequence_phase'] == 'completed'
    assert abs(opened['readbacks']['left_arm_gripper'] - 2000) <= 30
    assert ok(robot.call('robot_get_state', {}))['enabled_motors'] == sorted(LEFT)   # the whole arm was auto-enabled
    closed = ok(robot.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 1700, 'duration_s': 4}))
    assert closed['completed'] is True and closed['closure_outcome'] == 'endpoint_settled' and closed['sequence_phase'] == 'completed'
    assert abs(closed['readbacks']['left_arm_gripper'] - 1700) <= 30
    assert robot.score()['gripper_closes'] == 1 and robot.score()['moves'] == 1 and robot.score()['gripper_closed_on_box'] is False
    bad = robot.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 1300})
    assert bad['ok'] is False and bad['result']['error'].startswith('Gripper target out of bounds: left_arm_gripper=1300; valid inclusive range [1313, 2781]')


def test_gripper_closing_on_the_box_reports_contact(robot):
    res = grasp(robot)
    assert res['completed'] is False and res['closure_outcome'] == 'contact_halt' and res['holding'] is True
    assert res['gripper_result']['holding'] is True and res['sequence_phase'] == 'stopped_short'
    assert res['readbacks']['left_arm_gripper'] > 1420   # stopped on the box, not at the closed stop
    score = robot.score()
    assert score['gripper_closed_on_box'] is True and score['faults'] == 0 and score['box_held_now'] is False


def test_scripted_grasp_and_lift_scores_held(robot):
    t0 = time.monotonic()
    near, left, top = box_frame(robot)
    res = grasp(robot, (near, left, top))
    assert res['closure_outcome'] == 'contact_halt'
    # Up 8 cm and 3 cm back toward the robot (keeps the wrist inside the arm's reach at this pitch); the box pivots on its
    # far edge while the near edge is held, so its centre rises about half the lift.
    up = reach(robot, near - 0.035, left, top + 0.08, GRASP_PITCH_DEG)
    lifted = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': up, 'duration_s': 8}))
    assert lifted['completed'] is True
    score = robot.score()
    assert score['box_lifted_m'] >= 0.03 and score['gripper_closed_on_box'] is True and score['box_held_now'] is True
    assert score['faults'] == 0 and score['moves'] == 3 and score['gripper_closes'] == 1 and score['calls'] >= 5
    assert score['sim_time_s'] > 20 and time.monotonic() - t0 < 30
    snap = robot.snapshot()
    assert set(snap['positions']) == set(robot.calibration) and snap['box']['up_m'] > 0.78


def test_move_base_shifts_the_box(robot):
    before = robot.box_pose()
    res = ok(robot.call('robot_move_base', {'linear_m_s': 0.02, 'angular_rad_s': 0.0, 'duration_s': 2}))
    assert res['completed'] is True and res['mode'] == 'base_pulse'
    base = res['base_result']
    assert set(base) == {'wheel_delta_ticks', 'estimated_wheel_travel_cm', 'pulse_s', 'stopped_early', 'released', 'settings_restored', 'odometry_note'}
    assert base['wheel_delta_ticks'] == {'base_left_wheel': -410, 'base_right_wheel': 410}
    assert base['estimated_wheel_travel_cm']['base_right_wheel'] == pytest.approx(4.0, abs=0.1)
    after = robot.box_pose()
    assert before['forward_m'] - after['forward_m'] == pytest.approx(0.04, abs=0.005)
    assert abs(after['left_m'] - before['left_m']) < 0.003 and abs(after['up_m'] - before['up_m']) < 0.003
    state = ok(robot.call('robot_get_state', {}))
    wheels = {m['name']: m['Present_Position'] for m in state['motors'] if m['name'].startswith('base_')}
    assert wheels['base_right_wheel'] == (737 + 410) % 4096
    too_fast = robot.call('robot_move_base', {'linear_m_s': 0.02, 'angular_rad_s': 0.1, 'duration_s': 1})
    assert too_fast['ok'] is False and too_fast['result']['error'] == 'Each wheel is limited to 0.02 m/s; lower linear_m_s or angular_rad_s'
    turn = ok(robot.call('robot_move_base', {'linear_m_s': 0.0, 'angular_rad_s': 0.08, 'duration_s': 1}))
    turned = robot.box_pose()
    assert turned['left_m'] < after['left_m'] - 0.02   # the robot turned left, so the box swings to the right
    assert robot.score()['base_pulses'] == 2


def test_reset_and_fallback_scene():
    r = SimRobot(real_time=False)
    try:
        enable_left(r)
        r.reset()
        assert ok(r.call('robot_get_state', {}))['all_16_released'] is True and r.score()['moves'] == 0
    finally:
        r.close()
    xml = sim_robot._fallback_scene_xml(seed=3)
    model = mujoco.MjModel.from_xml_string(sim_robot._scene_with_actuators(xml))
    for kind, name in ((mujoco.mjtObj.mjOBJ_BODY, 'table'), (mujoco.mjtObj.mjOBJ_BODY, 'box'), (mujoco.mjtObj.mjOBJ_JOINT, 'box_free'),
                       (mujoco.mjtObj.mjOBJ_GEOM, 'flap'), (mujoco.mjtObj.mjOBJ_CAMERA, 'oak'), (mujoco.mjtObj.mjOBJ_CAMERA, 'left_wrist'),
                       (mujoco.mjtObj.mjOBJ_CAMERA, 'right_wrist'), (mujoco.mjtObj.mjOBJ_CAMERA, 'phone')):
        assert mujoco.mj_name2id(model, kind, name) >= 0, name
    fallback = SimRobot(scene_xml=xml, real_time=False)
    try:
        assert fallback.scene_source == 'given'
        assert 0.38 < fallback.box_pose()['forward_m'] < 0.46
    finally:
        fallback.close()
