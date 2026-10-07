"""Geometry, angle-bridge and planning rules for closing majors over held shorts.

Component checks only: they establish arithmetic and refusal rules, not that
the executed physical sequence folds a carton.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from carton.folding_majors_over_shorts import (
    _ContactAngleBridge, _kinematic_angle, _major_point, _push_planner, close_majors_over_held_shorts,
)
from carton.folding_progress import ContactProgressGuard
from carton.folding_sim import W, H


def reading(seq, angle=None, pose=None, flap='long_far'):
    pose = np.eye(4) if pose is None else pose
    angles = {} if angle is None else {flap: dict(degrees=angle)}
    return dict(seq=seq, world_from_box=pose.tolist(), angles=angles)


@pytest.mark.parametrize('flap', ['long_near', 'long_far'])
def test_major_point_matches_the_hinged_panel_geometry(flap):
    box = np.eye(4)
    sign = 1. if flap == 'long_near' else -1.
    upright = _major_point(box, flap, 0., .05, .14, 0.)
    assert upright == pytest.approx([.05, -sign*W/2, H + .0035 + .14])
    flat = _major_point(box, flap, 90., 0., .115, 0.)
    assert flat == pytest.approx([0., -sign*W/2 + sign*.115, H + .0035])
    # A negative inward offset is the outer side of the panel.
    outer = _major_point(box, flap, 0., 0., .1, -.003)
    assert outer[1] == pytest.approx(-sign*(W/2 + .003))


@pytest.mark.parametrize('flap', ['long_near', 'long_far'])
@pytest.mark.parametrize('theta', [-12., 5., 33., 70., 88.])
def test_kinematic_angle_inverts_the_contact_point(flap, theta):
    box = np.eye(4)
    box[:3, 3] = [.01, -.02, .003]
    outer = -.002
    point = _major_point(box, flap, theta, -.1, .115, outer)
    ik = SimpleNamespace(point=lambda q: point)
    sim = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(10)), arm_indices={'left': list(range(5))})
    assert _kinematic_angle(ik, sim, 'left', box, flap, outer) == pytest.approx(theta, abs=1e-9)


def bridge(angle=10.):
    first = reading(1, angle)
    guard = ContactProgressGuard('long_far', first)
    return _ContactAngleBridge('long_far', guard, first), first


def test_bridge_uses_kinematics_only_after_agreement_and_only_while_vision_is_missing():
    b, first = bridge()
    b.start(first, 12.)  # 2 degrees: agrees
    assert b.update(reading(2, 11.), 11.5, 11.2, -5, 95) == 11.
    assert b.update(reading(3), 12.5, 12.4, -5, 95) == 12.4
    assert b.rows[-1]['source'] == 'contact_kinematics'
    assert b.update(reading(4, 13.), 13.5, 13.2, -5, 95) == 13.
    assert b.bridged == 0


def test_bridge_refuses_without_prior_agreement():
    b, first = bridge()
    b.start(first, 14.)  # 4 degrees apart
    with pytest.raises(ValueError, match='before contact kinematics agreed'):
        b.update(reading(2), 11., 11., -5, 95)


def test_bridge_limits_consecutive_commands_and_checks_reacquisition():
    b, first = bridge()
    b.start(first, 10.)
    for seq in range(2, 14):
        b.update(reading(seq), 10. + seq, 10. + seq, -5, 95)
    with pytest.raises(ValueError, match='not visible for 12'):
        b.update(reading(14), 25., 25., -5, 95)
    b, first = bridge()
    b.start(first, 10.)
    b.update(reading(2), 11., 11., -5, 95)
    with pytest.raises(ValueError, match='disagrees with contact kinematics'):
        b.update(reading(3, 18.), 12., 12., -5, 95)


def test_bridge_requires_fresh_registration_within_carton_drift_bounds():
    b, first = bridge()
    b.start(first, 10.)
    moved = np.eye(4)
    moved[0, 3] = .016
    with pytest.raises(ValueError, match='Carton moved'):
        b.update(reading(2, pose=moved), 11., 11., -5, 95)
    b, first = bridge()
    b.start(first, 10.)
    with pytest.raises(ValueError, match='Fresh carton registration'):
        b.update(reading(1), 11., 11., -5, 95)


def test_push_planner_ignores_only_the_pushed_panel_in_a_model_copy():
    import mujoco
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="arm"><joint name="left_shoulder_pan" type="hinge"/><joint name="left_shoulder_lift" type="hinge"/>
        <joint name="left_elbow_flex" type="hinge"/><joint name="left_wrist_flex" type="hinge"/>
        <joint name="left_wrist_roll" type="hinge"/><geom name="left_jaw" size=".01"/></body>
      <geom name="long_far_cardboard" type="box" size=".1 .0015 .07"/>
      <geom name="long_near_cardboard" type="box" size=".1 .0015 .07" pos="0 .3 0"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    sim = SimpleNamespace(model=model, data=data, arm_indices={'left': list(range(5))},
                          forbidden_contact=lambda a, b: False)
    planner = _push_planner(sim, 'left', 'long_far_cardboard')
    far, near = model.geom('long_far_cardboard').id, model.geom('long_near_cardboard').id
    assert planner.model.geom_contype[far] == 0 and planner.model.geom_conaffinity[far] == 0
    assert planner.model.geom_contype[near] == model.geom_contype[near] != 0
    assert model.geom_contype[far] != 0


@pytest.mark.parametrize('kwargs', [dict(far_pin_degrees=20.), dict(far_pin_degrees=50.),
                                    dict(near_target_degrees=95.), dict(far_target_degrees=float('nan'))])
def test_undeclared_targets_refuse_before_plant_access(kwargs):
    with pytest.raises(ValueError, match='Far pin'):
        close_majors_over_held_shorts(None, None, **kwargs)
