"""Open short planes and refusal; synthetic render fixtures are test-only."""
import numpy as np
import pytest

from carton.folding_short_hinge_vision import depth_open_short_flap_angles
from carton.geometry import Box
from test_folding_hinge_vision import _panel, _render, _major


def _short(degrees, name='short_left', *, offset=0., half_width=None, along_shift=0.):
    b = Box(); inward = 1 if name == 'short_left' else -1
    theta = np.radians(degrees)
    radial = np.array([inward*np.sin(theta), 0., np.cos(theta)])
    normal = np.array([inward*np.cos(theta), 0., -np.sin(theta)])
    hinge = np.array([-inward*b.length/2, along_shift, b.height])
    return _panel(hinge + radial*b.flap/2 + normal*offset,
                  [0, 1, 0], radial, b.width/2-.004 if half_width is None else half_width,
                  b.flap/2)


@pytest.mark.parametrize('name', ['short_left', 'short_right'])
@pytest.mark.parametrize('angle', [-30., -15., 0., 10., 25.])
def test_open_short_angles_have_wide_hinge_support(name, angle):
    side = -1 if name == 'short_left' else 1
    inputs = _render([_short(angle, name)], camera=(side*.65, -.35, .45))
    result = depth_open_short_flap_angles(*inputs)
    row = result[name]
    assert row['degrees'] == pytest.approx(angle, abs=.5)
    assert row['supported_patches'] >= 6
    assert row['hinge_axis_error_deg'] <= 4.
    assert abs(row['hinge_plane_offset_mm']) <= 6.
    assert row['valid_angle_bounds_degrees'] == [-40., 30.]


def test_visible_back_surfaces_need_no_printed_short_tag():
    inputs = _render([_short(-12., 'short_left'), _short(-15., 'short_right'),
                      _major(39., 'long_near'), _major(35., 'long_far')],
                     camera=(0., -.50, .65))
    result = depth_open_short_flap_angles(*inputs)
    assert result['short_left']['degrees'] == pytest.approx(-12., abs=.5)
    assert result['short_right']['degrees'] == pytest.approx(-15., abs=.5)


def test_nearly_edge_on_short_still_refuses_without_enough_support():
    inputs = _render([_short(-30.)], camera=(-.5, -.35, .65))
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': -30.})


@pytest.mark.parametrize('angle', [40., 65., 90., 100.])
def test_closed_and_out_of_scope_shorts_are_not_mislabelled_open(angle):
    inputs = _render([_short(angle), _major(39., 'long_near'), _major(35., 'long_far')])
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': 10.})


def test_partial_majors_alone_are_not_short_identity():
    inputs = _render([_major(39., 'long_near'), _major(35., 'long_far')])
    assert depth_open_short_flap_angles(*inputs, {'short_left': -12., 'short_right': -15.}) == {}


def test_wrong_parallel_surface_does_not_inherit_hinge_alignment():
    inputs = _render([_short(10., offset=.030)], camera=(-.5, -.35, .65))
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': 10.})


def test_wrong_hinge_axis_is_refused():
    b = Box(); along = np.array([.2, 1., 0.]); along /= np.linalg.norm(along)
    panel = _panel([-b.length/2, 0, b.height+.07], along, [0, 0, 1], .13, .07)
    assert 'short_left' not in depth_open_short_flap_angles(*_render([panel], camera=(-.5, -.35, .65)))


def test_two_credible_open_planes_cannot_be_selected_by_prior():
    inputs = _render([_short(-15., half_width=.060, along_shift=-.075),
                      _short(10., half_width=.060, along_shift=.075)], camera=(-.5, -.35, .65))
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': 0.})


def test_missing_pixels_and_small_tool_face_cannot_be_filled_from_prior():
    assert depth_open_short_flap_angles(*_render([]), {'short_left': -12.}) == {}
    inputs = _render([_short(-12., half_width=.025)], camera=(-.5, -.35, .65))
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': -12.})


def test_prior_rejects_discontinuity_without_supply_of_missing_angle():
    inputs = _render([_short(-15.)], camera=(-.5, -.35, .65))
    assert 'short_left' in depth_open_short_flap_angles(*inputs)
    assert 'short_left' not in depth_open_short_flap_angles(*inputs, {'short_left': 25.})


def test_world_frame_change_leaves_pixel_estimate_identical():
    inputs = list(_render([_short(10.)], camera=(-.5, -.35, .65)))
    before = depth_open_short_flap_angles(*inputs)
    world = np.array([[0., -1., 0., .3], [1., 0., 0., -.2], [0., 0., 1., .04], [0., 0., 0., 1.]])
    inputs[3] = world @ inputs[3]; inputs[4] = world
    after = depth_open_short_flap_angles(*inputs)
    assert after['short_left']['degrees'] == pytest.approx(before['short_left']['degrees'])


@pytest.mark.parametrize('field,value,match', [(0,np.zeros((8,8,3)), 'uint8'),
    (1,np.zeros((8,8)), 'aligned'), (2,np.zeros((3,3)), 'intrinsics'),
    (3,np.zeros((4,4)), 'rigid'), (4,np.zeros((4,4)), 'rigid')])
def test_invalid_sensor_inputs_refuse(field,value,match):
    inputs=list(_render([])); inputs[field]=value
    with pytest.raises(ValueError, match=match): depth_open_short_flap_angles(*inputs)


def test_nonfinite_prior_and_invalid_task_dimensions_refuse():
    inputs=_render([])
    with pytest.raises(ValueError, match='priors'):
        depth_open_short_flap_angles(*inputs, {'short_left': np.nan})
    with pytest.raises(ValueError, match='dimensions'):
        depth_open_short_flap_angles(*inputs, box=Box(flap=.25))
