import math
import pytest

from carton.folding_station import FoldingStation


def test_table_gap_and_carton_inset_are_separate_distances():
    station = FoldingStation(.06, .15, .05)
    assert station.table_edge_y - station.base_y == pytest.approx(.15)
    assert -.283 / 2 - station.table_edge_y == pytest.approx(.05)
    assert station.setback == pytest.approx(.20)
    assert not station.report()['physical_registration_verified']


def test_moving_table_edge_does_not_silently_move_carton_or_base():
    a, b = FoldingStation(.06, .15, .05), FoldingStation(.06, .10, .10)
    assert a.base_y == pytest.approx(b.base_y)
    assert a.table_edge_y != b.table_edge_y
    assert a.table_tag_position[1] >= a.table_edge_y
    assert b.table_tag_position[1] >= b.table_edge_y


def test_historical_reference_exposes_bases_over_the_table():
    station = FoldingStation.historical_reference()
    assert station.table_edge_y == pytest.approx(-.43)
    assert station.base_y == pytest.approx(-.1815)
    assert station.base_to_table_edge == pytest.approx(-.2485)
    assert station.setback == pytest.approx(.04)
    assert station.reference_layout
    assert not station.report()['physical_registration_verified']


@pytest.mark.parametrize('args',[(math.nan,.15,.05),(.06,math.inf,.05),(.06,.15,-.05),(.06,-.15,.05),(.06,.15,.05,0)])
def test_invalid_or_implicit_over_table_station_is_rejected(args):
    with pytest.raises(ValueError):
        FoldingStation(*args)
