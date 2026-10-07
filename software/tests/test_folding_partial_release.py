"""A partial passive component needs fresh evidence and cannot clear faults."""
from types import SimpleNamespace

import numpy as np
import pytest

from carton.folding_partial_release import _PartialMajorHold, release_near_after_passive_far


def reading(seq, near=40., far=35., x=0.):
    pose = np.eye(4)
    pose[0, 3] = x
    return dict(seq=seq, world_from_box=pose.tolist(), angles={
        'long_near': {'degrees': near}, 'long_far': {'degrees': far}})


def prior():
    return dict(stage='partial far angle passively retained for five seconds',
                passive_retention_only=True, right_hand_parked=True,
                near_still_actively_held=True, physical_far_target_verified=True,
                target_far_degrees=35., expected_near_degrees=40., full_task_complete=False,
                visual_progress_checks=[{'seq': 10, 'stop_reason': None}])


@pytest.mark.parametrize('change', [
    {'stage': 'physically release partial far hold'},
    {'right_hand_parked': False}, {'passive_retention_only': False},
    {'target_far_degrees': 40.}, {'expected_near_degrees': 90.},
    {'visual_progress_checks': [{'seq': 10, 'stop_reason': 'lost observation'}]},
])
def test_incomplete_or_wrong_prior_stage_rejected_before_plant_access(change):
    report = prior()
    report.update(change)
    with pytest.raises(ValueError):
        release_near_after_passive_far(None, SimpleNamespace(far_edge_attempt=report))


def test_missing_prior_stage_rejected_before_plant_access():
    with pytest.raises(ValueError, match='Verified passive'):
        release_near_after_passive_far(None, SimpleNamespace())


def test_entry_cannot_reuse_the_last_frame_of_prior_release():
    c = SimpleNamespace(far_edge_attempt=prior(), sense=lambda _: reading(10))
    with pytest.raises(ValueError, match='new visual'):
        release_near_after_passive_far(None, c)
    assert c.partial_major_release['fault']
    assert not c.partial_major_release['full_task_complete']
    with pytest.raises(ValueError, match='already attempted'):
        release_near_after_passive_far(None, c)


@pytest.mark.parametrize('bad', [
    reading(11), reading(12, near=45.1), reading(12, far=29.9),
    reading(12, far=float('nan')), reading(12, x=.016),
    {'seq': 12, 'world_from_box': np.eye(4).tolist(), 'angles': {}},
])
def test_bad_or_lost_observation_latches_both_hold_checks(bad):
    hold = _PartialMajorHold(reading(11), after_seq=10)
    with pytest.raises(ValueError):
        hold.check(bad)
    with pytest.raises(ValueError, match='latched'):
        hold.check(reading(13))


def test_both_partial_angles_can_remain_in_band_without_closure():
    hold = _PartialMajorHold(reading(11), after_seq=10)
    hold.check(reading(12, near=39.2, far=34.5, x=.002))
    assert hold.fault is None
    assert all(guard.checks[-1]['stop_reason'] is None for guard in hold.guards.values())


def test_prior_report_does_not_replace_live_right_park_verification():
    c = SimpleNamespace(far_edge_attempt=prior(), sense=lambda _: reading(11))
    sim = SimpleNamespace(actual_control_position=lambda _: np.zeros(3))
    with pytest.raises(ValueError, match='Right parked-hand'):
        release_near_after_passive_far(sim, c)
    assert not c.partial_major_release['both_hands_parked']
    assert not c.partial_major_release['both_majors_passively_retained']
