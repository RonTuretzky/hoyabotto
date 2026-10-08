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


def test_explicit_redundant_markers_keep_their_full_white_border_on_table():
    station=FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),backup_table_marker_xy=(.45,.7))
    assert station.table_tag_position==[-.5,.55,.0013]
    assert station.report()['backup_table_marker_xy']==(.45,.7)
    for coords in ((.53,.5),(0.,-.14),(float('nan'),.5)):
        with pytest.raises(ValueError):FoldingStation(.06,.15,.01,backup_table_marker_xy=coords)


def test_rotating_box_requires_translation_to_keep_near_corner_supported():
    station=FoldingStation(.06,.15,.01)
    yaw=math.radians(30)
    assert not station.carton_footprint(yaw=yaw)['fully_on_table']
    dy=.379/2*math.sin(yaw)+.283/2*math.cos(yaw)-.283/2
    footprint=station.carton_footprint((0,dy),yaw)
    assert footprint['fully_on_table']
    assert footprint['minimum_table_edge_clearance_m']==pytest.approx(.01)
    assert not station.carton_footprint((0,dy-.011),yaw)['fully_on_table']
