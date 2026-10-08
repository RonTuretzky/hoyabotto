"""SimRobot: the simulated XLeRobot behind the pilot chat server's tool API (MuJoCo; skipped without it)."""
import json
import math
import time

import pytest

mujoco = pytest.importorskip('mujoco')

from farm.sim import sim_robot  # noqa: E402
from farm.sim.sim_robot import SimRobot  # noqa: E402

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
LEFT = [f'left_arm_{j}' for j in ARM]
BOX_SIZE = (0.20, 0.15, 0.11)   # the scene builder's default box
GRASP_PITCH_DEG = -60.0
PINCH_DEPTH_M = 0.015           # the claw tip goes this far below the flap's free edge: the pads hold the top strip


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


def solve(r, forward, left, up, pitch, current=None):
    from farm.kinematics.so101_reach import solve_reach
    return solve_reach('left', {'forward_m': forward, 'left_m': left, 'up_m': up}, current or r.positions(), r.ranges(), pitch_deg=pitch)


def reach(r, forward, left, up, pitch, current=None):
    sol = solve(r, forward, left, up, pitch, current)
    assert sol['ok'], sol['reason']
    return sol['ticks']


def reach_at_most(r, forward, left, up, pitch):
    """Ticks for the highest reachable point at or below ``up`` (1 cm steps): high targets at steep pitch run out of wrist."""
    for k in range(10):
        sol = solve(r, forward, left, up - 0.01 * k, pitch)
        if sol['ok']:
            return sol['ticks']
    raise AssertionError(sol['reason'])


def box_frame(r):
    """(near face forward, left, top height, flap edge height) of the box from the scene."""
    from farm.sim import box_scene
    with r.lock:
        b = box_scene.robot_frame_of_box(r.model, r.data)
    return b['near_face_forward_m'], b['left_m'], b['top_m'], b['flap_top_m']


def grasp(r, frame=None, depth=PINCH_DEPTH_M):
    """Scripted flap pinch: open above the flap edge at pitch -60, lower so the edge sits between the pads (claw tip
    ``depth`` below the edge, 5 mm in front of the near face), then close to 1400. Returns the close result."""
    near, left, top, edge = frame or box_frame(r)
    enable_left(r)
    pre = reach_at_most(r, near - 0.015, left, edge + 0.04, GRASP_PITCH_DEG)
    pre['left_arm_gripper'] = 2500
    assert ok(r.call('robot_move_joint_targets', {'arm': 'left', 'positions': pre, 'duration_s': 8}))['completed'] is True
    low = reach(r, near - 0.005, left, edge - depth, GRASP_PITCH_DEG)
    assert ok(r.call('robot_move_joint_targets', {'arm': 'left', 'positions': low, 'duration_s': 6}))['completed'] is True
    return ok(r.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 1400, 'duration_s': 5}))


def fold_path(r, near, left, top, radius=0.06, start_deg=-10.0, end_deg=95.0, steps=8):
    """Waypoints that carry a pinched edge on a quarter circle about the hinge (the near top edge) from vertical to
    flat on the top, the pitch easing from -60 to -40 (the steepest this arm reaches out there)."""
    waypoints, current = [], r.positions()
    for i in range(1, steps + 1):
        a = i / steps
        angle = math.radians(start_deg + (end_deg - start_deg) * a)
        ticks = reach(r, near + radius * math.sin(angle), left, top + radius * math.cos(angle), GRASP_PITCH_DEG + 20 * a, current)
        current = {**current, **ticks}
        waypoints.append(ticks)
    return waypoints


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


def test_flap_stands_open_on_its_hinge(robot):
    near, left, top, edge = box_frame(robot)
    score = robot.score()
    assert score['flap_angle_deg'] == pytest.approx(-8.0, abs=0.2) and score['flap_folded'] is False
    assert score['flap_pinched_now'] is False and score['flap_pinched_ever'] is False
    assert edge == pytest.approx(top + 0.07 * math.cos(math.radians(8)), abs=0.002)
    time.sleep(1.0)   # idle physics at 1x: the flap stays where it is
    assert robot.score()['flap_angle_deg'] == pytest.approx(-8.0, abs=0.2)


def test_gripper_closing_on_the_flap_reports_a_pinch(robot):
    res = grasp(robot)
    # the 3.5 mm flap stops the jaws ~30 ticks short of their 1400 meeting point: settled_short or contact_halt
    # (holding) when they stop >= 40 ticks behind the goal; either way the pinch verdict's evidence is there
    assert res['closure_outcome'] in ('contact_halt', 'settled_short', 'endpoint_settled'), res
    assert 1420 <= res['readbacks']['left_arm_gripper'] <= 1480
    score = robot.score()
    assert score['flap_pinched_now'] is True and score['gripper_closed_on_box'] is True
    assert score['faults'] == 0 and score['box_held_now'] is False and score['flap_folded'] is False
    assert -30 < score['flap_angle_deg'] < 0   # the pads turned the flap a little toward themselves, not folded it
    load = ok(robot.call('robot_get_state', {}))
    row = next(m for m in load['motors'] if m['name'] == 'left_arm_gripper')
    assert abs(row['Present_Load']) >= 60   # sustained squeeze: what the pilot's PINCH LIKELY verdict reads


def test_scripted_pinch_and_slow_lift_keeps_the_box_held(robot):
    t0 = time.monotonic()
    near, left, top, edge = box_frame(robot)
    res = grasp(robot, (near, left, top, edge))
    assert robot.score()['flap_pinched_now'] is True, res
    # Up 10 cm and 3 cm back toward the robot, slowly, easing the pitch to -45 (pitch -60 tops out near 90 cm here).
    # The box hangs from its flap and pivots on its far bottom edge, so its centre rises about half the lift.
    up = reach_at_most(robot, near - 0.035, left, edge - PINCH_DEPTH_M + 0.10, -45)
    lifted = ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': up, 'duration_s': 12}))
    assert lifted['completed'] is True
    for _ in range(3):   # hold for a while: a pinched flap must not creep out of the pads
        time.sleep(0.5)
    score = robot.score()
    assert score['box_lifted_m'] >= 0.03 and score['gripper_closed_on_box'] is True and score['box_held_now'] is True
    assert score['flap_pinched_now'] is True and score['flap_angle_deg'] < -20   # the box hangs off the hinge
    assert score['faults'] == 0 and score['moves'] == 3 and score['gripper_closes'] == 1 and score['calls'] >= 5
    assert score['sim_time_s'] > 20 and time.monotonic() - t0 < 40
    snap = robot.snapshot()
    assert set(snap['positions']) == set(robot.calibration) and snap['box']['up_m'] > 0.78


def test_scripted_fold_lays_the_flap_on_the_top(robot):
    near, left, top, edge = box_frame(robot)
    grasp(robot, (near, left, top, edge))
    assert robot.score()['flap_pinched_now'] is True
    swept = ok(robot.call('robot_move_path', {'arm': 'left', 'waypoints': fold_path(robot, near, left, top), 'duration_s': 12}))
    assert swept['completed'] is True
    assert robot.score()['flap_angle_deg'] > 75 and robot.score()['flap_folded'] is False   # still in the jaws
    ok(robot.call('robot_set_gripper', {'arm': 'left', 'position_ticks': 2200, 'duration_s': 2}))
    tip = robot.claw_positions()['left_arm']
    away = reach_at_most(robot, tip['forward_m'] - 0.03, left, tip['up_m'] + 0.06, -30)
    ok(robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': away, 'duration_s': 6}))
    time.sleep(1.0)
    score = robot.score()
    assert score['flap_folded'] is True and score['flap_angle_deg'] >= 75 and score['flap_pinched_now'] is False
    assert score['flap_pinched_ever'] is True   # folded from a pinch, not knocked over
    assert abs(score['box_lifted_m']) < 0.01 and score['box_moved_m'] < 0.06 and score['faults'] == 0


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
