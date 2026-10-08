"""Vendored SO-101 analytic kinematics against the arm's real geometry.

The round-trip tests elsewhere (tests/test_kinematics.py, tests/test_carton_servo.py) only
show that FK and IK agree with each other; they passed while both were wrong against the
arm. This file pins the absolute geometry: (a) a table of wrist-flex-axis positions read
off the MuJoCo twin and the elbow/lift angles the correct IK must return for them,
(b) FK/IK round trips over a grid of angles and (c), when MuJoCo and the vendored twin
model are present, the vendored FK against the twin's own Wrist_Pitch joint anchor.
No motor I/O; no robot API.
"""
import math

import pytest

from farm.vendor.so101_kinematics import SO101Kinematics

# (shoulder_lift, elbow_flex) in LeRobot degrees -> wrist-flex axis (forward, up) in metres from
# the shoulder-lift axis, read from the MuJoCo twin (JOINT_TABLE: Pitch = 90 - lift,
# Elbow = 90 + elbow), and the angles the correct upstream IK returns for that point.
TABLE = [
    ((0.0, 0.0), (0.1629, 0.1178), (0.01, -0.06)),
    ((20.0, 0.0), (0.1934, 0.0550), (20.01, -0.06)),
    ((24.1, 36.45), (0.1424, -0.0236), (24.08, 36.42)),
    ((45.0, 45.0), (0.1046, -0.0751), (44.97, 44.98)),
    ((-30.0, 60.0), (0.0874, 0.0485), (-30.05, 59.99)),
]

# Before the fix the vendored IK returned these for the same points (elbow 30-70 deg off).
WRONG_ELBOW = [32.41, 32.41, -4.07, -12.63, -27.64]


@pytest.mark.parametrize('angles, wrist, expected', TABLE, ids=[f'lift{a[0]}_elbow{a[1]}' for a, _, _ in TABLE])
def test_ik_returns_the_arm_angles_for_twin_wrist_positions(angles, wrist, expected):
    lift, elbow = SO101Kinematics().inverse_kinematics(*wrist)
    assert (lift, elbow) == pytest.approx(expected, abs=0.1)
    # The twin was posed at ``angles``; the table rounds the wrist position to 0.1 mm,
    # which moves the IK answer by up to ~0.1 deg.
    assert (lift, elbow) == pytest.approx(angles, abs=0.2)


@pytest.mark.parametrize('angles, wrist, expected', TABLE, ids=[f'lift{a[0]}_elbow{a[1]}' for a, _, _ in TABLE])
def test_fk_returns_the_twin_wrist_position(angles, wrist, expected):
    assert SO101Kinematics().forward_kinematics(*angles) == pytest.approx(wrist, abs=5e-4)


@pytest.mark.parametrize('index', range(len(TABLE)))
def test_pre_fix_elbow_values_are_rejected(index):
    _, wrist, _ = TABLE[index]
    _, elbow = SO101Kinematics().inverse_kinematics(*wrist)
    assert abs(elbow - WRONG_ELBOW[index]) > 20


@pytest.mark.parametrize('lift', [-60.0, -30.0, 0.0, 20.0, 45.0, 70.0])
@pytest.mark.parametrize('elbow', [-60.0, -30.0, 0.0, 30.0, 60.0, 85.0])
def test_fk_ik_roundtrip_grid(lift, elbow):
    kin = SO101Kinematics()
    x, y = kin.forward_kinematics(lift, elbow)
    assert kin.inverse_kinematics(x, y) == pytest.approx((lift, elbow), abs=1e-9)
    assert kin.forward_kinematics(*kin.inverse_kinematics(x, y)) == pytest.approx((x, y), abs=1e-12)


def test_zero_pose_reach_is_upper_arm_up_forearm_forward():
    # At all zeros the SO-101 upper arm points up (tilted 14 deg forward by the 28/112.57 mm
    # bracket) and the forearm forward: x ~ l2, y ~ l1, within the two small link offsets.
    kin = SO101Kinematics()
    x, y = kin.forward_kinematics(0.0, 0.0)
    assert x == pytest.approx(0.1629, abs=1e-3)
    assert y == pytest.approx(0.1178, abs=1e-3)
    assert math.hypot(x, y) < kin.l1 + kin.l2


# ---------------------------------------------------------------- against the MuJoCo twin

def _twin_available():
    try:
        import mujoco  # noqa: F401
        from farm.sim import xlerobot_twin
        xlerobot_twin.find_model()
        return True
    except (ImportError, FileNotFoundError):
        return False


@pytest.mark.skipif(not _twin_available(), reason='mujoco or the XLeRobot twin model is not available')
@pytest.mark.parametrize('arm', ['left', 'right'])
def test_fk_matches_twin_wrist_flex_axis(arm):
    from farm.sim import xlerobot_twin as twin
    mj_twin = twin._Twin(twin.find_model()[0])
    mj = mj_twin.mj
    side = 'L' if arm == 'left' else 'R'
    lift_jid = mj.mj_name2id(mj_twin.model, mj.mjtObj.mjOBJ_JOINT, f'Pitch_{side}')
    wrist_jid = mj.mj_name2id(mj_twin.model, mj.mjtObj.mjOBJ_JOINT, f'Wrist_Pitch_{side}')
    assert lift_jid >= 0 and wrist_jid >= 0
    kin = SO101Kinematics()
    poses = [a for a, _, _ in TABLE] + [(60.0, 90.0), (-10.0, 20.0), (80.0, 100.0)]
    for lift, elbow in poses:
        angles = {motor: 0.0 for motor in twin.JOINT_TABLE}
        angles[f'{arm}_arm_shoulder_lift'] = lift
        angles[f'{arm}_arm_elbow_flex'] = elbow
        model_deg = {joint: offset + sign * angles[motor]
                     for motor, (joint, offset, sign) in twin.JOINT_TABLE.items() if offset is not None}
        mj_twin.pose(model_deg)
        delta = mj_twin.data.xanchor[wrist_jid] - mj_twin.data.xanchor[lift_jid]
        forward, left, up = (mj_twin.axes @ delta).tolist()
        x, y = kin.forward_kinematics(lift, elbow)
        assert abs(left) < 1e-3, (lift, elbow)
        assert math.hypot(forward - x, up - y) < 0.01, (lift, elbow, (forward, up), (x, y))
