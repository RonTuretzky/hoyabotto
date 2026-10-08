import pytest
from carton.folding_cart import cart_boxes,table_overlap,report
from carton.folding_station import FoldingStation


def test_cart_front_not_mount_line_defines_clearance():
    s=FoldingStation(.06,.15,.05)
    assert report(s)['front_edge_to_table_edge_m']==pytest.approx(.035)
    assert not table_overlap(s)
    assert all(p.name.startswith('cart_') for p in cart_boxes(s))


@pytest.mark.parametrize('height',[.06,.12,.26])
def test_apparent_short_reach_layout_intersects_realistic_cart_envelope(height):
    s=FoldingStation(height,.05,.05)
    collisions=table_overlap(s)
    assert collisions
    assert any('post_' in p['part'] for p in collisions)


def test_both_mounts_are_supported_inside_cart_width():
    parts=cart_boxes(FoldingStation(.06,.15,.05))
    mounts=[p for p in parts if p.name.endswith('mount_plate')]
    assert len(mounts)==2
    assert all(abs(p.center[0])+p.half_size[0]<=.25 for p in mounts)
