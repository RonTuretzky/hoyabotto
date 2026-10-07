import copy
import math

import numpy as np
import pytest

from carton.folding_progress import ContactProgressGuard


def reading(seq, angle, x=0., rotation=0.):
    c, s = math.cos(rotation), math.sin(rotation)
    pose = [[c, -s, 0, x], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    return dict(seq=seq, world_from_box=pose, angles={'long_near': {'degrees': angle}})


def test_progress_accepts_folding_without_modifying_observation():
    initial = reading(1, -5)
    before = copy.deepcopy(initial)
    guard = ContactProgressGuard('long_near', initial)
    for i, angle in enumerate([0, 5, 10, 15, 20], 2):
        assert guard.check(reading(i, angle, .001), angle)['stop_reason'] is None
    assert initial == before


def test_sliding_stops_even_if_fold_angle_increases():
    guard = ContactProgressGuard('long_near', reading(1, 0))
    with pytest.raises(ValueError, match='moved'):
        guard.check(reading(2, 20, .016), 20)
    assert guard.checks[-1]['translation_mm'] == 16


def test_stationary_but_jammed_fold_stops():
    guard = ContactProgressGuard('long_near', reading(1, 0))
    guard.check(reading(2, .3), 6)
    with pytest.raises(ValueError, match='stalled'):
        guard.check(reading(3, .5), 12)


def test_rotation_stops_without_translation():
    guard = ContactProgressGuard('long_near', reading(1, 0))
    with pytest.raises(ValueError, match='rotated'):
        guard.check(reading(2, 5, rotation=math.radians(9)), 5)


def test_stale_pose_cannot_authorize_more_contact_motion():
    guard = ContactProgressGuard('long_near', reading(5, 0))
    with pytest.raises(ValueError, match='new visual'):
        guard.check(reading(5, 15), 15)


@pytest.mark.parametrize('bad', [None, {}, reading(0, 0), reading(1, np.nan), reading(1, 0, np.inf)])
def test_missing_or_invalid_observation_is_rejected(bad):
    with pytest.raises(ValueError):
        ContactProgressGuard('long_near', bad)


def test_new_stroke_rebases_angle_progress_but_keeps_original_drift_bound():
    guard = ContactProgressGuard('long_near', reading(1, -15))
    contact = reading(2, 5, .010)
    guard.check(contact, -15)
    guard.begin_stroke(contact)
    assert guard.check(reading(3, 5, .010), 5)['stop_reason'] is None
    with pytest.raises(ValueError, match='moved'):
        guard.check(reading(4, 20, .016), 20)
    with pytest.raises(ValueError, match='successful'):
        guard.begin_stroke(reading(4, 20, .016))


def test_rebased_stroke_still_rejects_no_progress():
    guard = ContactProgressGuard('long_near', reading(1, -15))
    contact = reading(2, 5)
    guard.check(contact, -15)
    guard.begin_stroke(contact)
    with pytest.raises(ValueError, match='stalled'):
        guard.check(reading(3, 5.1), 17)


def test_stroke_transition_cannot_use_unchecked_angle():
    guard = ContactProgressGuard('long_near', reading(1, -15))
    guard.check(reading(2, 5), -15)
    with pytest.raises(ValueError, match='checked'):
        guard.begin_stroke(reading(2, 30))


@pytest.mark.parametrize('bad', [None, {}, reading(2, 5), reading(3, np.nan)])
def test_missing_invalid_or_stale_check_cannot_be_cleared_by_prior_good_frame(bad):
    guard = ContactProgressGuard('long_near', reading(1, 0))
    last_good = reading(2, 5)
    guard.check(last_good, 5)
    with pytest.raises(ValueError):
        guard.check(bad, 6)
    with pytest.raises(ValueError, match='successful'):
        guard.begin_stroke(last_good)
    with pytest.raises(ValueError, match='latched'):
        guard.check(reading(4, 10), 10)
