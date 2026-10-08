"""Outward preparation must preserve sensing evidence and reject failed pushes."""
import copy

import numpy as np
import pytest

from carton.folding_progress import ContactProgressGuard
from carton.folding_short_opening import _opening_reading, open_shorts_before_majors


def reading(seq, degrees, shift=0.):
    pose=np.eye(4);pose[0,3]=shift
    return dict(seq=seq,world_from_box=pose.tolist(),
                angles={'short_left':dict(degrees=degrees,method='rendered_RGB_D'),
                        'long_near':dict(degrees=-15.,method='rendered_RGB_D')})


@pytest.mark.parametrize('target',[-36.,-9.,float('nan'),float('inf')])
def test_invalid_target_rejected_before_accessing_plant(target):
    with pytest.raises(ValueError,match='target must'):
        open_shorts_before_majors(None,None,target_degrees=target)


def test_outward_progress_uses_original_visual_evidence_without_mutation():
    initial=reading(1,6.)
    original=copy.deepcopy(initial)
    converted=_opening_reading(initial,'short_left')
    assert initial==original
    assert converted['angles']['short_left']['degrees']==-6.
    assert converted['angles']['short_left']['method']=='rendered_RGB_D'
    assert converted['angles']['long_near']==initial['angles']['long_near']
    guard=ContactProgressGuard('short_left',converted)
    row=guard.check(_opening_reading(reading(2,-8.),'short_left'),8.)
    assert row['observed_progress_degrees']==14.
    assert row['stop_reason'] is None


def test_outward_stall_is_rejected():
    guard=ContactProgressGuard('short_left',_opening_reading(reading(1,6.),'short_left'))
    with pytest.raises(ValueError,match='stalled'):
        guard.check(_opening_reading(reading(2,5.),'short_left'),8.)


def test_outward_carton_drift_is_rejected():
    guard=ContactProgressGuard('short_left',_opening_reading(reading(1,6.),'short_left'))
    with pytest.raises(ValueError,match='moved more than 15 mm'):
        guard.check(_opening_reading(reading(2,-8.,.020),'short_left'),8.)
