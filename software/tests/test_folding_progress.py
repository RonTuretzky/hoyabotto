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
