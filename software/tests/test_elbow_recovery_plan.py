import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
import pytest
from elbow_recovery_plan import recovery_target


def test_observed_sag_has_in_range_bounded_target():
    target=recovery_target(3175,1002,3092)
    assert target==3044
    assert 1002<target<3092
    assert (3175-target)*360/4096<12


@pytest.mark.parametrize('position',[3092,3050,900,3205,4096,-1])
def test_unexpected_start_refused(position):
    with pytest.raises(ValueError):recovery_target(position,1002,3092)


def test_excess_total_travel_refused():
    with pytest.raises(ValueError):recovery_target(3205,1002,3092)


def test_further_settling_uses_closer_goal_without_larger_movement():
    target=recovery_target(3198,1002,3092)
    assert target<=3092-24
    assert (3198-target)*360/4096<=12


@pytest.mark.parametrize('limits',[(4000,100),(3080,3092),(-1,3092),(100,5000)])
def test_invalid_narrow_or_wrapping_range_refused(limits):
    with pytest.raises(ValueError):recovery_target(3175,*limits)
