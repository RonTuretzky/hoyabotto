"""SO-101 reach IK: upstream port (pure math), consistency with the twin's FK, reach solves, plan_steps."""
import itertools
import math
import threading
import time

import pytest

from farm.kinematics import so101_reach as reach
from farm.sim import xlerobot_twin as twin

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
MOTORS = [f'{side}_arm_{j}' for side in ('left', 'right') for j in ARM] + ['head_motor_1', 'head_motor_2']
# Saved ranges like the robot's calibration (calibration/farm_xlerobot/farm_xlerobot.json), both arms alike.
RANGES = {m: (1000, 3000) for m in MOTORS}
for side in ('left', 'right'):
    RANGES.update({f'{side}_arm_shoulder_pan': (959, 3135), f'{side}_arm_shoulder_lift': (855, 3239),
                   f'{side}_arm_elbow_flex': (942, 3152), f'{side}_arm_wrist_flex': (898, 3196),
                   f'{side}_arm_wrist_roll': (115, 3979), f'{side}_arm_gripper': (1275, 2819)})
NEUTRAL = {m: (lo + hi) // 2 for m, (lo, hi) in RANGES.items()}
EXAMPLE = {'forward_m': 0.30, 'left_m': 0.12, 'up_m': 0.84}   # "LEFT claw at forward 30 cm, left 12 cm, up 84 cm"
LEFT_JOINTS = [f'left_arm_{j}' for j in reach.JOINTS]


def _model_available():
    try:
        import mujoco  # noqa: F401
        twin.find_model()
        return True
    except (ImportError, FileNotFoundError):
        return False


model = pytest.mark.skipif(not _model_available(), reason='mujoco or the XLeRobot twin model is not available')


def _ticks_to_deg(motor, tick):
    lo, hi = RANGES[motor]
    return (tick - (lo + hi) / 2) * 360 / 4096


# ---------------------------------------------------------------- upstream port (no model needed)

def test_upstream_forward_kinematics_hand_values():
    # (0, 0): theta1 = 90 deg - atan2(0.028, 0.11257) = 76.0315 deg, theta2 = 90 deg - 16.1761 deg = 73.8239 deg,
    # forearm at theta1 + theta2 - 180 = -30.1446 deg:
    # x = 0.1159 cos 76.0315 + 0.1350 cos(-30.1446) = 0.027973 + 0.116747 = 0.144720
    # y = 0.1159 sin 76.0315 + 0.1350 sin(-30.1446) = 0.112471 - 0.067790 = 0.044681
    x, y = reach.upstream_forward_kinematics(0.0, 0.0)
    assert (x, y) == pytest.approx((0.144720, 0.044681), abs=2e-6)
    assert reach.upstream_forward_kinematics(0.0, 0.0) == (0.14471996741542967, 0.0446805129657646)  # upstream file


def test_upstream_inverse_kinematics_hand_values():
    # The joycon teleop's zero hand target (0.1629, 0.1131): r = 0.198278, cos(theta2) = 0.245019 -> theta2 = 75.8172 deg,
    # beta = 34.7749 deg, gamma = atan2(0.130881, 0.149076) = 41.2869 deg, theta1 = 76.0618 deg;
    # joint2 = 90 - (76.0618 + 13.9685) = -0.0303 ... (-0.0379 with full precision), joint3 = 75.8172 + 16.1761 - 90 = 1.9933 (1.9871)
    j2, j3 = reach.upstream_inverse_kinematics(0.1629, 0.1131)
    assert (j2, j3) == pytest.approx((-0.0379, 1.9871), abs=2e-3)
    assert reach.upstream_inverse_kinematics(0.1629, 0.1131) == (-0.037941061431311596, 1.9871497542919627)
    # beyond reach: scaled onto the boundary (straight arm), as upstream does
    j2, j3 = reach.upstream_inverse_kinematics(1.0, 0.0)
    assert j3 == pytest.approx(math.degrees(reach.THETA2_OFFSET) - 90)  # theta2 = 0 -> joint3 = offset


def test_upstream_fk_is_not_the_inverse_of_its_ik_but_arm_plane_forward_is():
    """Documented upstream quirk: IK(FK(j2, j3)) = (j2, 2 * theta2_offset_deg - j3) = (j2, 32.35 - j3)."""
    mirror = 2 * math.degrees(reach.THETA2_OFFSET)
    assert mirror == pytest.approx(32.351, abs=1e-3)
    for j2, j3 in [(0, 0), (20, 30), (-30, 45), (45, -20), (60, 60)]:
        got = reach.upstream_inverse_kinematics(*reach.upstream_forward_kinematics(j2, j3))
        assert got == pytest.approx((j2, mirror - j3), abs=1e-6)
        got = reach.upstream_inverse_kinematics(*reach.arm_plane_forward(j2, j3))
        assert got == pytest.approx((j2, j3), abs=1e-6)
    # the two forward models agree only when the forearm folds symmetrically (j3 = 16.18): elsewhere they differ by cm
    assert math.dist(reach.upstream_forward_kinematics(0, 0), reach.arm_plane_forward(0, 0)) > 0.05


def test_arm_plane_forward_zero_pose_matches_teleop_zero_target():
    x, y = reach.arm_plane_forward(0.0, 0.0)
    assert (x, y) == pytest.approx((0.1629, 0.1178), abs=5e-4)  # joycon zero target is (0.1629, 0.1131): 5 mm lower
    # independently measured on the twin: lift 45, elbow 45 puts the wrist-flex axis at (0.1046, -0.0751)
    assert reach.arm_plane_forward(45.0, 45.0) == pytest.approx((0.1046, -0.0751), abs=5e-4)
    assert reach.upstream_inverse_kinematics(0.1046, -0.0751) == pytest.approx((45.0, 45.0), abs=0.5)


def test_so101_joint_limits_are_reported_not_applied():
    assert reach.LIFT_LIMITS_DEG == pytest.approx((-107.68, 95.73), abs=0.01)
    assert reach.ELBOW_LIMITS_DEG == pytest.approx((-101.46, 90.0), abs=0.01)
    # wrist point straight below the lift axis: the IK clamps the lift at 95.7 deg and we say so
    problems = reach.analytic_solve('left', {'forward_m': 0.1956, 'left_m': 0.1552, 'up_m': 0.702})['problems']
    assert any('shoulder_lift would be at the SO-101 joint limit' in p for p in problems)
    assert reach.analytic_solve('left', EXAMPLE)['problems'] == []


def test_pitch_rule_and_tip_offset():
    assert reach.hand_elevation_deg(28.2, 51.8, -80.0) == pytest.approx(0.0)
    assert reach.tip_offset(0.0) == pytest.approx((reach.TIP_ALONG_M, -reach.TIP_BELOW_M))
    up = reach.tip_offset(90.0)
    assert up == pytest.approx((reach.TIP_BELOW_M, reach.TIP_ALONG_M), abs=1e-12)  # tip up: offset points up


def test_analytic_solve_geometry_and_problems():
    a = reach.analytic_solve('left', EXAMPLE)
    assert a['problems'] == []
    assert a['pan_deg'] == pytest.approx(math.degrees(math.atan2(0.1552 - 0.12, 0.30)))  # target right of the shoulder: +pan
    assert a['reach_m'] == pytest.approx(math.sqrt(0.30 ** 2 + (0.12 - 0.1552) ** 2 + (0.84 - 0.894) ** 2))
    d = a['degrees']
    assert d['left_arm_wrist_flex'] == pytest.approx(-d['left_arm_shoulder_lift'] - d['left_arm_elbow_flex'])
    # the analytic model closes the loop: its own FK puts the tip back on the target
    tip = reach.analytic_tip_robot_frame('left', *(d[m] for m in LEFT_JOINTS))
    assert tip == pytest.approx(EXAMPLE, abs=1e-9)
    mirrored = reach.analytic_solve('right', dict(EXAMPLE, left_m=-0.12))
    assert mirrored['pan_deg'] == pytest.approx(-a['pan_deg'])
    assert reach.analytic_solve('left', dict(EXAMPLE, forward_m=0.6))['problems'][0].startswith('unreachable')
    assert 'behind the shoulder' in reach.analytic_solve('left', dict(EXAMPLE, forward_m=-0.2))['problems'][0]
    assert 'below the floor' in reach.analytic_solve('left', dict(EXAMPLE, up_m=-0.1))['problems'][0]
    assert 'inside the cart' in reach.analytic_solve('left', {'forward_m': 0.0, 'left_m': 0.1, 'up_m': 0.5})['problems'][0]
    with pytest.raises(ValueError):
        reach.analytic_solve('middle', EXAMPLE)
    with pytest.raises(ValueError):
        reach.analytic_solve('left', {'forward_m': 0.3, 'left_m': 0.1})
    with pytest.raises(ValueError):
        reach.analytic_solve('left', dict(EXAMPLE, up_m=float('nan')))


# ---------------------------------------------------------------- consistency with the twin

@model
def test_twin_geometry_constants():
    """Shoulder point and pan direction, read from the twin, are what the solver assumes."""
    claws = twin.claw_positions(NEUTRAL, RANGES)
    for arm in ('left', 'right'):
        assert claws[f'{arm}_arm']['shoulder_up_m'] == pytest.approx(reach.SHOULDER_UP_M, abs=1e-4)
        assert claws[f'{arm}_arm']['shoulder_left_m'] == pytest.approx(reach.SHOULDER_LEFT_M[arm], abs=1e-4)
        panned = twin.claw_positions(dict(NEUTRAL, **{f'{arm}_arm_shoulder_pan': NEUTRAL[f'{arm}_arm_shoulder_pan'] + 341}),
                                     RANGES)[f'{arm}_arm']
        assert panned['left_m'] < claws[f'{arm}_arm']['left_m'] - 0.1  # +pan swings either arm to the robot's right
        assert panned['up_m'] == pytest.approx(claws[f'{arm}_arm']['up_m'], abs=1e-6)


@model
def test_analytic_tip_matches_twin_fk_over_a_grid():
    """Analytic model (lift ahead of pan, 2-link arm, fixed tip offset) vs the twin's claw tip: within 1 cm.

    Measured residual on the vendored model: 0.42-0.44 mm everywhere (the tip site's 0.4 mm lateral offset and
    link-length rounding). Upstream's own forward_kinematics is 5-20 cm off, confirming the IK convention is
    the one that matches the twin."""
    worst, worst_upstream = 0.0, 0.0
    for arm in ('left', 'right'):
        for pan, lift, elbow, wrist in itertools.product((-40, 0, 40), (-60, -20, 20, 60), (-60, 0, 60), (-45, 0, 45)):
            ticks = dict(NEUTRAL)
            for j, deg in zip(reach.JOINTS, (pan, lift, elbow, wrist)):
                ticks[f'{arm}_arm_{j}'] = NEUTRAL[f'{arm}_arm_{j}'] + deg * 4096 / 360
            got = twin.claw_positions(ticks, RANGES)[f'{arm}_arm']
            want = reach.analytic_tip_robot_frame(arm, pan, lift, elbow, wrist)
            err = math.dist([got[k] for k in want], [want[k] for k in want])
            worst = max(worst, err)
            # wrist point by upstream's FK + the same tip offset, for the record
            ux, uy = reach.upstream_forward_kinematics(lift, elbow)
            ox, oy = reach.tip_offset(reach.hand_elevation_deg(lift, elbow, wrist))
            d = reach.LIFT_AHEAD_M + ux + ox
            a = math.radians(-pan)
            up = (d * math.cos(a), reach.SHOULDER_LEFT_M[arm] + d * math.sin(a), reach.SHOULDER_UP_M + uy + oy)
            worst_upstream = max(worst_upstream, math.dist([got[k] for k in want], up))
    assert worst < 0.01, f'analytic vs twin residual {worst * 1000:.1f} mm'
    assert worst < 0.001, f'measured 0.44 mm on the vendored model; now {worst * 1000:.2f} mm'
    assert worst_upstream > 0.05


# ---------------------------------------------------------------- reach solves

def _assert_shape(result, arm):
    assert set(result) == {'ok', 'arm', 'target', 'pitch_deg', 'ticks', 'degrees', 'predicted', 'error_m', 'reach_m',
                           'analytic_error_m', 'iterations', 'clamped', 'band', 'reason', 'method', 'mapping',
                           'mapping_validated', 'model'}
    assert result['method'] == 'analytic+twin-refine' and result['arm'] == arm
    assert set(result['ticks']) >= {f'{arm}_arm_{j}' for j in reach.JOINTS}
    assert all(type(v) is int for v in result['ticks'].values())
    assert set(result['predicted']) == {'forward_m', 'left_m', 'up_m'}
    for m, t in result['ticks'].items():
        lo, hi = RANGES[m]
        assert lo + 40 <= t <= hi - 40


@model
def test_example_target_solves_and_is_level():
    r = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES)
    _assert_shape(r, 'left')
    assert r['ok'] is True and r['reason'] is None
    assert r['error_m'] < 0.002
    assert r['predicted'] == pytest.approx(EXAMPLE, abs=0.002)
    assert r['reach_m'] == pytest.approx(reach.analytic_solve('left', EXAMPLE)['reach_m'], abs=0.002)
    d = r['degrees']
    assert reach.hand_elevation_deg(d['left_arm_shoulder_lift'], d['left_arm_elbow_flex'], d['left_arm_wrist_flex']) == pytest.approx(0, abs=0.1)
    assert r['mapping'] == 'feetech_degrees_v1' and r['mapping_validated'] is False
    assert set(r['ticks']) == set(LEFT_JOINTS)  # wrist roll untouched unless asked
    assert 'ok' in reach.describe(r) and 'LEFT claw' in reach.describe(r) and 'level' in reach.describe(r)


@model
def test_grid_of_targets():
    """Reachable targets solve within 1 cm; the rest say why. Grid: forward 0.10..0.40, left within +-0.15 of the
    shoulder, up 0.75..1.05 (measured: 180 of 343 points solve with ranges like the robot's; worst 0.9 mm)."""
    ok = total = 0
    for f, dl, u in itertools.product((0.10, 0.20, 0.30, 0.40), (-0.15, -0.05, 0.05, 0.15), (0.75, 0.85, 0.95, 1.05)):
        target = {'forward_m': f, 'left_m': 0.1552 + dl, 'up_m': u}
        r = reach.solve_reach('left', target, NEUTRAL, RANGES)
        _assert_shape(r, 'left')
        total += 1
        if r['ok']:
            ok += 1
            assert r['error_m'] < 0.01 and r['reason'] is None
            assert r['predicted'] == pytest.approx(target, abs=0.01)
            assert math.dist(list(r['predicted'].values()), list(target.values())) == pytest.approx(r['error_m'], abs=1e-9)
        else:
            assert r['reason'] and any(k in r['reason'] for k in
                                       ('unreachable', 'outside joint ranges', 'inside the cart', 'too close', 'behind'))
            assert r['error_m'] >= 0.005 or r['clamped']
    assert ok >= 20, f'{ok}/{total} grid targets solved'


@model
def test_unreachable_targets_say_why():
    far = reach.solve_reach('left', dict(EXAMPLE, forward_m=0.60), NEUTRAL, RANGES)
    assert far['ok'] is False and far['reason'].startswith('unreachable') and 'max reach' in far['reason']
    assert far['error_m'] > 0.05
    assert 'NOT ok' in reach.describe(far)
    behind = reach.solve_reach('left', {'forward_m': -0.2, 'left_m': 0.3, 'up_m': 0.9}, NEUTRAL, RANGES)
    assert behind['ok'] is False and 'behind the shoulder' in behind['reason'] and 'shoulder_pan' in behind['reason']
    floor = reach.solve_reach('left', {'forward_m': 0.2, 'left_m': 0.1, 'up_m': -0.1}, NEUTRAL, RANGES)
    assert floor['ok'] is False and 'below the floor' in floor['reason']
    cart = reach.solve_reach('right', {'forward_m': 0.0, 'left_m': -0.1, 'up_m': 0.5}, NEUTRAL, RANGES)
    assert cart['ok'] is False and 'inside the cart' in cart['reason']
    narrow_pan = dict(RANGES, left_arm_shoulder_pan=(1500, 2500))  # +-40 deg band
    side = reach.solve_reach('left', {'forward_m': 0.25, 'left_m': 0.5, 'up_m': 0.9}, NEUTRAL, narrow_pan)  # pan -54 deg
    assert side['ok'] is False and 'left_arm_shoulder_pan' in side['reason'] and 'band 1540..2460' in side['reason']
    assert side['ticks']['left_arm_shoulder_pan'] == 1540
    # the band is honoured even for the failed attempt
    for r in (far, behind, floor, cart, side):
        _assert_shape(r, r['arm'])


@model
def test_mirrored_targets_give_mirrored_pans():
    left = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES)
    right = reach.solve_reach('right', dict(EXAMPLE, left_m=-EXAMPLE['left_m']), NEUTRAL, RANGES)
    assert left['ok'] and right['ok']
    mid = NEUTRAL['left_arm_shoulder_pan']
    assert left['ticks']['left_arm_shoulder_pan'] - mid == -(right['ticks']['right_arm_shoulder_pan'] - mid)
    for j in ('shoulder_lift', 'elbow_flex', 'wrist_flex'):
        assert abs(left['ticks'][f'left_arm_{j}'] - right['ticks'][f'right_arm_{j}']) <= 1
    assert right['predicted']['left_m'] == pytest.approx(-left['predicted']['left_m'], abs=0.002)
    assert 'RIGHT claw' in reach.describe(right)


@model
def test_joint_map_is_honoured():
    joint_map = {'validated': True, 'joints': {'left_arm_elbow_flex': {'zero_tick': 2100, 'sign': -1},
                                                'left_arm_shoulder_pan': {'sign': -1}}}
    plain = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES)
    mapped = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, joint_map=joint_map)
    assert mapped['ok'] and mapped['mapping'] == 'feetech_degrees_v1+joint_map' and mapped['mapping_validated'] is True
    assert mapped['predicted'] == pytest.approx(plain['predicted'], abs=0.002)
    mid = NEUTRAL['left_arm_elbow_flex']
    assert mapped['ticks']['left_arm_elbow_flex'] == pytest.approx(2100 - (plain['ticks']['left_arm_elbow_flex'] - mid), abs=1)
    pan_mid = NEUTRAL['left_arm_shoulder_pan']
    assert mapped['ticks']['left_arm_shoulder_pan'] - pan_mid == pytest.approx(-(plain['ticks']['left_arm_shoulder_pan'] - pan_mid), abs=1)
    assert mapped['ticks']['left_arm_shoulder_lift'] == plain['ticks']['left_arm_shoulder_lift']
    # the twin agrees that the mapped ticks land on the target
    claw = twin.claw_positions(dict(NEUTRAL, **mapped['ticks']), RANGES, joint_map=joint_map)['left_arm']
    assert {k: claw[k] for k in EXAMPLE} == pytest.approx(EXAMPLE, abs=0.002)
    with pytest.raises(ValueError):
        reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, joint_map={'joints': {'nope': {}}})


@model
def test_pitch_tilts_the_hand_and_keeps_the_tip():
    level = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES)
    down = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, pitch_deg=-30)
    assert level['ok'] and down['ok']
    assert down['predicted'] == pytest.approx(EXAMPLE, abs=0.002)
    assert down['ticks']['left_arm_shoulder_pan'] == level['ticks']['left_arm_shoulder_pan']  # pan is the same plane
    for r, pitch in ((level, 0.0), (down, -30.0)):
        d = r['degrees']
        assert reach.hand_elevation_deg(d['left_arm_shoulder_lift'], d['left_arm_elbow_flex'], d['left_arm_wrist_flex']) == pytest.approx(pitch, abs=0.1)
    # pitch only enters through the wrist rule: same lift/elbow => wrist differs by exactly the pitch
    d0, d1 = level['degrees'], down['degrees']
    assert (d1['left_arm_wrist_flex'] + d1['left_arm_shoulder_lift'] + d1['left_arm_elbow_flex']) - \
           (d0['left_arm_wrist_flex'] + d0['left_arm_shoulder_lift'] + d0['left_arm_elbow_flex']) == pytest.approx(30, abs=0.1)
    # tip down: the twin's hand (wrist-roll axis) points below the tip height of the wrist
    assert 'tip down' in reach.describe(down)
    # a tilt the wrist cannot give at this target is reported on the wrist, not silently wrong
    up = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, pitch_deg=60)
    assert up['ok'] is False and 'left_arm_wrist_flex' in up['reason']


@model
def test_roll_ticks_are_passed_through_and_clamped():
    r = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, roll_ticks=2500)
    assert r['ok'] and r['ticks']['left_arm_wrist_roll'] == 2500 and set(r['ticks']) == set(LEFT_JOINTS) | {'left_arm_wrist_roll'}
    r = reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, roll_ticks=4000)
    assert r['ok'] is False and r['ticks']['left_arm_wrist_roll'] == 3979 - 40 and 'left_arm_wrist_roll' in r['reason']


@model
def test_refinement_converges_from_a_bad_start():
    """The analytic start is already within a millimetre; push it 150 ticks off and watch the twin pull it back."""
    import numpy as np
    bands = {m: reach._band(RANGES, m) for m in LEFT_JOINTS}
    pose = reach._Pose('left', dict(NEUTRAL), RANGES, None, 0.0, bands)
    analytic = reach.analytic_solve('left', EXAMPLE)['degrees']
    raw = twin.motor_ticks(analytic, RANGES)
    start = np.array([raw[m] for m in LEFT_JOINTS[:3]]) + np.array([150.0, -150.0, 150.0])
    goal = np.array([EXAMPLE[k] for k in ('forward_m', 'left_m', 'up_m')])
    before = np.linalg.norm(pose.evaluate([pose.ticks(start)])[0][0] - goal)
    assert before > 0.03
    ticks3, iterations = reach._refine(pose, start, goal)
    after = np.linalg.norm(pose.evaluate([pose.ticks(ticks3)])[0][0] - goal)
    assert after < reach.REFINE_TOL_M and 1 <= iterations <= reach.MAX_ITERATIONS
    assert np.abs(ticks3 - np.array([raw[m] for m in LEFT_JOINTS[:3]])).max() < 3  # back on the analytic solution


@model
def test_missing_range_is_an_error_not_a_guess():
    ranges = {k: v for k, v in RANGES.items() if k != 'left_arm_elbow_flex'}
    with pytest.raises(ValueError, match='left_arm_elbow_flex'):
        reach.solve_reach('left', EXAMPLE, NEUTRAL, ranges)
    with pytest.raises(ValueError):
        reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES, roll_ticks=float('inf'))


@model
def test_fast_and_thread_safe():
    reach.solve_reach('left', EXAMPLE, NEUTRAL, RANGES)  # model load
    times = []
    for i in range(20):
        start = time.perf_counter()
        reach.solve_reach('left', dict(EXAMPLE, forward_m=0.25 + i * 0.005), NEUTRAL, RANGES)
        times.append(time.perf_counter() - start)
    assert sorted(times)[len(times) // 2] < 0.05, f'median {sorted(times)[10] * 1000:.1f} ms'
    results, errors = [], []

    def call(arm):
        try:
            results.append((arm, reach.solve_reach(arm, dict(EXAMPLE, left_m=EXAMPLE['left_m'] * (1 if arm == 'left' else -1)),
                                                   NEUTRAL, RANGES)['ticks']))
        except Exception as exc:  # pragma: no cover - surfaced below
            errors.append(exc)
    threads = [threading.Thread(target=call, args=('left' if i % 2 else 'right',)) for i in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=60)
    assert not errors and len(results) == 6
    expect = {arm: reach.solve_reach(arm, dict(EXAMPLE, left_m=EXAMPLE['left_m'] * (1 if arm == 'left' else -1)),
                                     NEUTRAL, RANGES)['ticks'] for arm in ('left', 'right')}
    assert all(ticks == expect[arm] for arm, ticks in results)


# ---------------------------------------------------------------- plan_steps and describe (no model needed)

def test_plan_steps_splits_proportionally_with_the_same_joints():
    current = {'left_arm_shoulder_pan': 2000, 'left_arm_shoulder_lift': 1200, 'left_arm_elbow_flex': 2900,
               'left_arm_wrist_flex': 2050, 'left_arm_wrist_roll': 1500}
    target = {'left_arm_shoulder_pan': 2010, 'left_arm_shoulder_lift': 2200, 'left_arm_elbow_flex': 1700,
              'left_arm_wrist_flex': 2050}
    steps = reach.plan_steps(current, target)
    assert len(steps) == 4  # 1200 ticks of elbow travel / 300
    assert steps[-1] == target
    previous = current
    for step in steps:
        assert set(step) == set(target)
        assert all(type(v) is int for v in step.values())
        assert all(abs(step[m] - previous[m]) <= 300 for m in step)
        previous = step
    # proportional: the pan creeps 2-3 ticks per waypoint instead of jumping at the end
    assert [s['left_arm_shoulder_pan'] for s in steps] == [2002, 2005, 2008, 2010]
    assert [s['left_arm_shoulder_lift'] for s in steps] == [1450, 1700, 1950, 2200]
    assert reach.plan_steps(current, target, max_step=1200) == [target]
    assert reach.plan_steps(current, target, max_step=100)[-1] == target
    assert len(reach.plan_steps(current, target, max_step=100)) == 12
    assert reach.plan_steps(current, {'left_arm_wrist_flex': 2050}) == [{'left_arm_wrist_flex': 2050}]  # no motion: one waypoint


def test_plan_steps_rejects_unknown_current_or_bad_step():
    with pytest.raises(ValueError, match='left_arm_elbow_flex'):
        reach.plan_steps({'left_arm_shoulder_pan': 2000}, {'left_arm_elbow_flex': 2100})
    with pytest.raises(ValueError):
        reach.plan_steps({'left_arm_shoulder_pan': 2000}, {'left_arm_shoulder_pan': 2100}, max_step=0)


def test_describe_without_a_model():
    result = {'ok': True, 'arm': 'left', 'target': EXAMPLE, 'pitch_deg': 0.0, 'error_m': 0.0005, 'reach_m': 0.307,
              'mapping': 'feetech_degrees_v1', 'mapping_validated': False,
              'ticks': {'left_arm_shoulder_pan': 2076, 'left_arm_shoulder_lift': 2321, 'left_arm_elbow_flex': 2589,
                        'left_arm_wrist_flex': 1090}}
    line = reach.describe(result)
    assert line.startswith('LEFT claw -> forward 0.300 left 0.120 up 0.840 m, level: ok')
    assert '0.5 mm' in line and 'unvalidated' in line and 'shoulder_pan=2076' in line and '\n' not in line
    bad = dict(result, ok=False, reason='unreachable: too far', error_m=0.107, pitch_deg=-20.0)
    line = reach.describe(bad)
    assert 'NOT ok: unreachable: too far' in line and 'pitch -20 deg (tip down)' in line and '107 mm' in line
