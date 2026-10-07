"""A partial major hold must not silently inherit a closure assertion."""
import pytest

from carton.folding_far_contact import _MeasuredFarStroke, _verify_target_angle, fold_far_from_edge


@pytest.mark.parametrize('kwargs',[
    {'expected_near_degrees':float('nan')},
    {'target_degrees':float('inf')},
    {'expected_near_degrees':100.},
    {'target_degrees':10.},
    {'expected_near_degrees':40.},
    {'release_after':True},
    {'contact_profile':'unknown'},
    {'contact_profile':'central'},
    {'central_normal_extra_m':.001},
    {'contact_profile':'central','target_degrees':40.,'central_normal_extra_m':.0021},
    {'central_normal_extra_m':float('nan')},
    {'gripper_opening':1.},
    {'central_startup_lift_m':.001},
    {'contact_profile':'central','target_degrees':40.,'central_startup_lift_m':.0021},
    {'central_startup_lift_m':float('nan')},
])
def test_unsupported_proposals_rejected_before_accessing_plant(kwargs):
    with pytest.raises(ValueError):
        fold_far_from_edge(None,None,**kwargs)


def test_partial_hold_uses_declared_angle_without_mutating_observation():
    reading={'angles':{'long_near':{'degrees':41.2,'method':'rendered_RGB_D'}}}
    assert _verify_target_angle(reading,'long_near',40.)==41.2
    assert reading['angles']['long_near']=={'degrees':41.2,'method':'rendered_RGB_D'}
    with pytest.raises(ValueError):
        _verify_target_angle(reading,'long_near',90.)


@pytest.mark.parametrize('observation',[None,{}, {'degrees':float('nan')},{'degrees':34.9},{'degrees':45.1}])
def test_missing_invalid_or_lost_hold_rejected(observation):
    with pytest.raises(ValueError):
        _verify_target_angle({'angles':{'long_far':observation}},'long_far',40.)


def test_measured_stroke_never_runs_ahead_of_latest_observed_angle():
    stroke=_MeasuredFarStroke(10.,40.)
    assert stroke.next_angle()==11.
    stroke.observe(10.4)
    assert stroke.next_angle()==11.4
    stroke.observe(9.8)
    assert stroke.next_angle()==10.8


def test_repeated_bounded_stall_cannot_evade_the_original_command_window():
    stroke=_MeasuredFarStroke(10.,40.)
    for _ in range(11):
        stroke.next_angle()
        stroke.observe(10.)
    stroke.next_angle()
    with pytest.raises(ValueError,match='stalled'):
        stroke.observe(10.)


def test_measured_stroke_accepts_observed_progress_and_finishes_near_target():
    stroke=_MeasuredFarStroke(30.,40.)
    for degrees in (31.,32.,33.,34.,35.,36.,37.,38.,39.):
        stroke.next_angle()
        stroke.observe(degrees)
    assert stroke.next_angle() is None


def test_bounded_stroke_has_a_finite_command_budget():
    stroke=_MeasuredFarStroke(0.,90.)
    for _ in range(240):
        stroke.next_angle()
    with pytest.raises(ValueError,match='240'):
        stroke.next_angle()
