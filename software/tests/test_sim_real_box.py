"""The 'real' scene preset: the carton and the right-flap fold as the robot met them on 9 October (software/STATUS.md).

Rim 77 cm, 16 cm flaps, an open (hollow) box; a crease that springs back unless it is carried past flat and held;
the fold accounting behind score()['fold_by_pinch'] (the owner's rule: no pushing with the claw body); the right
gripper's meeting point and mid-travel stalls; the arm model reading ~3 cm high; jaws opening left/right unrolled.
"""
import math
import time

import mujoco
import pytest

from farm.kinematics.so101_reach import solve_reach
from farm.sim import box_scene
from farm.sim.sim_robot import SimRobot

RIGHT = [f'right_arm_{j}' for j in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')]
NO_STICKS = {'right_grip_sticks': None, 'meet_jitter_ticks': None}
HINGE_LEFT, HINGE_UP, RADIUS = -18.0, 80.0, 14.0   # model cm: the right wall top reads ~3 cm high, like the claw


def ok(res):
    assert res['ok'] is True, res
    return res['result']


def make(**fidelity):
    return SimRobot(seed=None, real_time=False, fidelity=dict(NO_STICKS, **fidelity))


def reach(r, forward, left, up, pitch, duration=3):
    sol = solve_reach('right', {'forward_m': forward / 100, 'left_m': left / 100, 'up_m': up / 100}, r.positions(), r.ranges(), pitch_deg=pitch)
    assert sol['ok'], sol['reason']
    if not all(r.motors[n].enabled for n in RIGHT):
        ok(r.call('robot_set_motor_enable', {'names': RIGHT, 'enabled': True}))
    pos = r.positions()
    ticks = {k: int(v) for k, v in sol['ticks'].items() if abs(int(v) - pos[k]) > 2}
    return ok(r.call('robot_move_joint_targets', {'arm': 'right', 'positions': ticks, 'duration_s': duration, 'wait': True})) if ticks else None


def grip(r, ticks):
    ok(r.call('robot_set_gripper', {'arm': 'right', 'position_ticks': ticks, 'duration_s': 2}))
    state = ok(r.call('robot_get_state', {'fresh': True}))
    return next(m['Present_Position'] for m in state['motors'] if m['name'] == 'right_arm_gripper')


def pinch(r):
    """Open, come down over the flap edge (tip 1 cm left of the flap, ~2 cm below its edge in truth), close."""
    grip(r, 2000)
    reach(r, 34, -20, 97, -35)
    reach(r, 34, -20, 94, -35)
    return grip(r, 1309)


def arc(r, angles):
    for a, pitch in angles:
        reach(r, 34, HINGE_LEFT + RADIUS * math.sin(math.radians(a)), HINGE_UP + RADIUS * math.cos(math.radians(a)), pitch)


def release(r):
    grip(r, 2000)
    reach(r, 34, -5, 95, -35)
    time.sleep(1.0)
    return r.score()


def test_real_scene_is_the_9_october_carton():
    r = SimRobot(seed=None, real_time=False)
    try:
        with r.lock:
            box = box_scene.robot_frame_of_box(r.model, r.data)
        assert box['top_m'] == pytest.approx(0.77, abs=0.002)
        assert box['near_face_forward_m'] == pytest.approx(0.27, abs=0.005)
        assert box['flap_angle_deg'] == pytest.approx(-11.0, abs=0.5)
        assert box['flap_top_m'] == pytest.approx(0.77 + 0.16 * math.cos(math.radians(11)), abs=0.004)   # ~93 cm
        for kind, name in ((mujoco.mjtObj.mjOBJ_GEOM, 'box_floor'), (mujoco.mjtObj.mjOBJ_JOINT, 'farflap_hinge'),
                           (mujoco.mjtObj.mjOBJ_SITE, 'box_flap_top')):
            assert mujoco.mj_name2id(r.model, kind, name) >= 0, name
        assert r.model.geom_contype[mujoco.mj_name2id(r.model, mujoco.mjtObj.mjOBJ_GEOM, 'box_body')] == 0   # open box
        score = r.score()
        assert score['flap_rest_deg'] == pytest.approx(-11.0, abs=0.1) and score['fold_by_pinch'] is False
    finally:
        r.close()


def test_model_reads_about_3_cm_high_and_jaws_open_left_right():
    r = make()
    try:
        reach(r, 34, -14, 96, -35)
        from farm.sim.xlerobot_twin import claw_positions
        model = claw_positions(r.positions(), r.ranges())['right_arm']
        truth = r.claw_positions()['right_arm']
        assert 0.02 < model['up_m'] - truth['up_m'] < 0.04
        fixed, moving = r.jaw_bodies['right_arm']
        with r.lock:
            left_axis = r.axes @ r.data.xmat[fixed].reshape(3, 3)[:, 0]   # the fixed jaw frame's x: the pads open along -x
        assert left_axis[1] > 0.9   # +x is the robot's left, so the moving jaw opens to the robot's right
    finally:
        r.close()


def test_flap_folded_flat_and_released_springs_back():
    r = make()
    try:
        assert pinch(r) >= 1360 and r.score()['flap_pinched_now'] is True
        arc(r, [(15, -35), (35, -35), (55, -40), (75, -40), (90, -45)])
        assert r.score()['flap_angle_deg'] > 80
        time.sleep(3.0)   # held flat 3 s (the robot's C3-C7 holds)
        score = release(r)
        assert score['flap_folded'] is False and score['flap_angle_deg'] < 70 and score['flap_rest_deg'] < 50
        assert score['flap_deg_by_pinch'] > 80 and score['flap_deg_by_push'] < 10   # brushes on the approach and the release
    finally:
        r.close()


def test_flap_carried_past_flat_and_held_stays_folded_by_pinch():
    r = make()
    try:
        pinch(r)
        arc(r, [(15, -35), (35, -35), (55, -40), (75, -40), (90, -45), (100, -50), (110, -50)])
        assert r.score()['flap_angle_deg'] > 100
        time.sleep(5.0)
        score = release(r)
        assert score['flap_folded'] is True and score['fold_by_pinch'] is True and score['flap_rest_deg'] > 75
        assert score['held_over_95_s'] > 4 and score['faults'] == 0 and score['box_moved_m'] < 0.04
    finally:
        r.close()


def test_pushing_the_flap_with_the_claw_body_is_not_a_fold_by_pinch():
    r = make()
    try:
        grip(r, 1309)                       # closed claw, no pinch
        reach(r, 22, -14, 100, -30)         # in front of the box
        reach(r, 22, -28, 92, -35)          # still in front, right of the flap
        reach(r, 34, -27, 88, -40)          # forward along its outside, below its edge
        for left, up in ((-16, 88), (-10, 86), (-6, 83), (-4, 80)):
            reach(r, 34, left, up, -40)      # sweep it over with the closed claw
        score = r.score()
        assert score['flap_deg_by_push'] > 20 and score['illegal_contact_s'] > 0
        assert score['fold_by_pinch'] is False
    finally:
        r.close()


def test_right_gripper_meets_near_1348_and_sticks_mid_travel():
    r = make(right_grip_sticks={'p_close': 1.0, 'p_open': 0.0, 'p_reopen': 0.0, 'close_band': (1520, 1610), 'open_band': (1500, 1950)})
    try:
        grip(r, 2300)
        stalled = grip(r, 1309)
        assert 1515 <= stalled <= 1620          # stuck on air, like the 9 October closes from ~1850
        assert 1340 <= grip(r, 1309) <= 1356    # the second close finishes: the pads meet
    finally:
        r.close()


def test_resent_open_after_an_open_stall_trips_the_no_progress_guard():
    r = make(right_grip_sticks={'p_close': 0.0, 'p_open': 1.0, 'p_reopen': 1.0, 'close_band': (1520, 1610), 'open_band': (1500, 1950)})
    try:
        grip(r, 1309)
        first = grip(r, 2300)
        assert first < 2250                     # stopped short, holding
        assert r.score()['faults'] == 0
        r.call('robot_set_gripper', {'arm': 'right', 'position_ticks': 2300, 'duration_s': 2})
        score = r.score()
        assert score['faults'] == 1 and score['released_all'] is True
    finally:
        r.close()


def test_fidelity_overrides_lean_and_crease():
    r = SimRobot(seed=3, real_time=False, fidelity={'lean_jitter_deg': [20.0, 20.0], 'plastic': {'rate_per_s': 0.1}})
    try:
        assert r.score()['flap_rest_deg'] == pytest.approx(20.0, abs=0.1)
        assert r.plastic['rate_per_s'] == 0.1 and r.plastic['yield_deg'] == box_scene.PLASTIC['yield_deg']
    finally:
        r.close()
    jittered = [SimRobot(seed=s, real_time=False) for s in (0, 3)]
    try:
        leans = [x.score()['flap_rest_deg'] for x in jittered]
        assert all(-12.0 <= v <= 15.0 for v in leans) and abs(leans[0] - leans[1]) > 5   # drawn per seed
    finally:
        for x in jittered:
            x.close()
