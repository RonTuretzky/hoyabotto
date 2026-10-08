"""Pixel observations and conservative refusal; synthetic geometry is test-only."""
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from carton.folding_hinge_vision import depth_major_flap_angles
from carton.geometry import Box


def _panel(center, along, radial, half_width, half_height, color=(150, 100, 60)):
    return tuple(np.asarray(value, dtype=float) for value in (center, along, radial)) + (
        half_width, half_height, color)


def _major(degrees, name='long_near', *, offset=0., half_width=None):
    box = Box()
    inward = 1 if name == 'long_near' else -1
    angle = np.radians(degrees)
    radial = np.array([0., inward * np.sin(angle), np.cos(angle)])
    normal = np.array([0., inward * np.cos(angle), -np.sin(angle)])
    hinge = np.array([0., -inward * box.width / 2, box.height + .0035])
    return _panel(hinge + radial * box.flap / 2 + normal * offset,
                  [1, 0, 0], radial,
                  box.length / 2 - .004 if half_width is None else half_width, box.flap / 2)


def _shorts():
    box = Box()
    return [_panel([sign * (box.length - box.flap) / 2, 0, box.height],
                   [1, 0, 0], [0, 1, 0], box.flap / 2, box.width / 2 - .004)
            for sign in (-1, 1)]


def _render(panels, *, camera=(0, -.50, .65), target=(0, 0, .13), noise=.0008,
            dropout=.25, seed=7, mask_center=None):
    """Analytic RGB-D z-buffer, without segmentation or truth passed to observer."""
    height, width = 480, 640
    k = np.array([[600., 0, width / 2], [0, 600., height / 2], [0, 0, 1]])
    pose = np.eye(4)
    pose[:3, 3] = camera
    forward = np.asarray(target) - camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1.])
    right /= np.linalg.norm(right)
    pose[:3, :3] = np.column_stack((right, np.cross(forward, right), forward))
    yy, xx = np.indices((height, width))
    rays = np.stack(((xx - width / 2) / 600, (yy - height / 2) / 600, np.ones_like(xx)), axis=-1)
    rays = np.einsum('...j,ij->...i', rays, pose[:3, :3])
    depth = np.full((height, width), np.inf)
    rgb = np.full((height, width, 3), 255, np.uint8)
    for center, along, radial, half_width, half_height, color in panels:
        normal = np.cross(along, radial)
        numerator = float((center - camera) @ normal)
        denominator = np.einsum('...i,i->...', rays, normal)
        z = np.divide(numerator, denominator, out=np.full_like(depth, np.inf),
                      where=np.abs(denominator) > 1e-8)
        points = rays * z[:, :, None] + camera
        displacement = points - center
        inside = ((np.abs(np.einsum('...i,i->...', displacement, along)) <= half_width)
                  & (np.abs(np.einsum('...i,i->...', displacement, radial)) <= half_height)
                  & (z > .1) & (z < depth))
        if mask_center is not None:
            inside &= np.abs(points[:, :, 0]) > mask_center
        depth[inside] = z[inside]
        rgb[inside] = color
    rng = np.random.default_rng(seed)
    depth[~np.isfinite(depth)] = 0.
    valid = depth > 0
    depth[valid] += rng.normal(0, noise, np.count_nonzero(valid))
    depth[rng.random(depth.shape) < dropout] = 0.
    return rgb, depth, k, pose, np.eye(4)


@pytest.mark.parametrize('name', ['long_near', 'long_far'])
@pytest.mark.parametrize('degrees', [-25., -5., 25., 65., 89., 94.])
def test_measures_open_and_closed_major_planes_with_depth_noise(name, degrees):
    side = -1 if name == 'long_near' else 1
    observation = depth_major_flap_angles(*_render([_major(degrees, name)], camera=(0, side * .5, .65)))
    assert observation[name]['degrees'] == pytest.approx(degrees, abs=.5)
    assert observation[name]['plane_rms_mm'] < 1.2
    assert observation[name]['supported_patches'] >= 6


def test_side_patches_recover_upright_flap_when_entire_center_is_obscured():
    observation = depth_major_flap_angles(*_render([_major(-15)], mask_center=.070),
                                          {'long_near': -14.})
    row = observation['long_near']
    assert row['degrees'] == pytest.approx(-15., abs=.5)
    assert row['short_gap_pixel_support'] == 0
    assert row['along_span_mm'] > 250


def test_brown_arm_face_cannot_replace_a_hinge_plane():
    # The wrong plane has abundant similarly coloured pixels, but is 30 mm
    # away from the hinge. Full-plane offset, not colour or prior, rejects it.
    bad = _major(13., offset=.030)
    assert 'long_near' not in depth_major_flap_angles(*_render([bad]), {'long_near': 13.})
    observation = depth_major_flap_angles(*_render([_major(-15.), bad]), {'long_near': -15.})
    assert observation['long_near']['degrees'] == pytest.approx(-15., abs=.5)


def test_wrong_hinge_axis_is_rejected_even_when_plane_crosses_hinge():
    box = Box()
    along = np.array([1., .2, 0.]); along /= np.linalg.norm(along)
    bad = _panel([0, -box.width / 2, box.height + .0735], along,
                 [0, 0, 1], .15, .07)
    assert 'long_near' not in depth_major_flap_angles(*_render([bad]))


def test_two_credible_hinge_planes_are_ambiguous():
    first = list(_major(-15, half_width=.08)); first[0] = first[0] + [-.09, 0, 0]
    second = list(_major(25, half_width=.08)); second[0] = second[0] + [.09, 0, 0]
    assert 'long_near' not in depth_major_flap_angles(*_render([first, second]), {'long_near': 0.})


def test_closed_short_flaps_do_not_count_as_major_closure():
    priors = {'long_near': 90., 'long_far': 90.}
    observation = depth_major_flap_angles(*_render(_shorts()), priors)
    assert observation == {}
    observation = depth_major_flap_angles(*_render([*_shorts(), _major(90.)]), priors)
    assert observation['long_near']['degrees'] == pytest.approx(90., abs=.5)
    assert 'long_far' not in observation


def test_closed_major_without_identity_pixels_is_omitted():
    observation = depth_major_flap_angles(*_render([_major(90.)], mask_center=.065),
                                          {'long_near': 90.})
    assert 'long_near' not in observation


def test_no_pixels_or_small_tool_face_cannot_be_recovered_from_prior():
    assert depth_major_flap_angles(*_render([]), {'long_near': 90.}) == {}
    assert 'long_near' not in depth_major_flap_angles(
        *_render([_major(-15, half_width=.025)]), {'long_near': -15.})
    inputs = list(_render([_major(-15)])); inputs[1][:] = np.nan
    assert depth_major_flap_angles(*inputs, {'long_near': -15.}) == {}


def test_prior_only_rejects_discontinuity_and_never_supplies_measurement():
    inputs = _render([_major(-15)])
    assert 'long_near' not in depth_major_flap_angles(*inputs, {'long_near': 75.})
    assert 'long_near' in depth_major_flap_angles(*inputs, {})


def test_world_coordinate_change_does_not_change_pixel_observation():
    inputs = list(_render([_major(35)]))
    expected = depth_major_flap_angles(*inputs)['long_near']['degrees']
    world = np.array([[0., -1., 0., .25], [1., 0., 0., -.13], [0., 0., 1., .05], [0., 0., 0., 1.]])
    inputs[3] = world @ inputs[3]; inputs[4] = world
    assert depth_major_flap_angles(*inputs)['long_near']['degrees'] == pytest.approx(expected)


@pytest.mark.parametrize('field,value,match', [
    (0, np.zeros((10, 10, 3), float), 'uint8'),
    (1, np.zeros((10, 10)), 'aligned'),
    (2, np.full((3, 3), np.nan), 'intrinsics'),
    (3, np.zeros((4, 4)), 'rigid'),
    (4, np.full((4, 4), np.nan), 'rigid'),
])
def test_invalid_sensor_calibration_is_rejected(field, value, match):
    inputs = list(_render([])); inputs[field] = value
    with pytest.raises(ValueError, match=match):
        depth_major_flap_angles(*inputs)


def test_nonfinite_priors_and_invalid_dimensions_are_rejected():
    inputs = _render([])
    with pytest.raises(ValueError, match='priors'):
        depth_major_flap_angles(*inputs, {'long_near': np.nan})
    with pytest.raises(ValueError, match='dimensions'):
        depth_major_flap_angles(*inputs, box=Box(flap=.25))


@pytest.mark.parametrize('environment,indices,width,height,camera,extra_markers', [
    ('CARTON_HINGE_REPLAY', (0, 100, 200, 300, 450, 500, 510, 520, 526, 527, 529), 1280, 720, 'station', True),
    ('CARTON_HINGE_CLOSED_REPLAY', (0, 40, 80, 100, 120, 140, 160, 180, 200, 220, 240, 264), 960, 540, 'front', False),
])
def test_recorded_states_with_fresh_apriltag_registration(monkeypatch, environment, indices,
                                                        width, height, camera, extra_markers):
    """Optional actual-scene regressions configured by local recording paths.

    CARTON_HINGE_REPLAY: claws-retention/parallel-near-center-release-01/trial-000/run.
    CARTON_HINGE_CLOSED_REPLAY: paddle-comparison/matched-final/loaded-weak-claws.

    Replay qpos is solely a renderer input and independent diagnostic score.
    Observer transforms are recovered from rendered production AprilTags.
    """
    if not os.getenv(environment):
        pytest.skip('optional local replay assets required')
    mujoco = pytest.importorskip('mujoco')
    from carton.folding_markers import BOX_MARKERS, BOX_TAG_SIZE, carton_pose_from_tags
    from carton.folding_vision import RGBDTagObserver, SIZES, depth_flap_angles
    for tag, x in (((24, .08), (25, 0.)) if extra_markers else ()):
        monkeypatch.setitem(BOX_MARKERS, tag, (f'box_tag_floor_{tag}', [x, 0, .0038], [1, 0, 0, 0, 1, 0]))
        monkeypatch.setitem(SIZES, tag, BOX_TAG_SIZE)
    run = Path(os.environ[environment])
    frames = json.loads((run / 'folding-frames.json').read_text())
    model = mujoco.MjModel.from_xml_path(str(run / 'scene.xml'))
    data = mujoco.MjData(model)
    option = mujoco.MjvOption(); option.geomgroup[3] = 0
    anchor = np.eye(4); anchor[:3, :3] = np.diag([-1, 1, -1]); anchor[:3, 3] = [-.5, .55, .0013]
    backup = anchor.copy(); backup[:3, 3] = [.45, .70, .0013]
    registration = RGBDTagObserver(anchor, additional_anchors={20: backup}, stationary_camera=True)
    focal = height / (2 * math.tan(math.radians(float(model.camera(camera).fovy[0])) / 2))
    k = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])
    rng = np.random.default_rng(0)
    priors = {}
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for seq, index in enumerate(indices, 1):
            data.qpos[:] = frames[index]['qpos']; mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            renderer.disable_depth_rendering(); rgb = renderer.render().copy()
            renderer.enable_depth_rendering(); depth = renderer.render().copy()
            depth += rng.normal(0, .0008, depth.shape); depth[rng.random(depth.shape) < .25] = 0
            tags = registration.observe(rgb, depth, k, seq=seq, timestamp=seq, depth_timestamp=seq)
            box, _ = carton_pose_from_tags(tags, registration.history[-1]['quality'])
            inputs = (rgb, depth, k, registration.world_from_camera, box)
            # The recorded samples can be separated by large motions. Priors
            # are a per-control-step gate, so sparse replay estimates omit it.
            observed = depth_major_flap_angles(*inputs, priors if extra_markers else None)
            assert set(observed) == {'long_near', 'long_far'}
            for name, row in observed.items():
                truth = float(np.degrees(data.qpos[model.joint(name + '_hinge').qposadr[0]]))
                assert row['degrees'] == pytest.approx(truth, abs=.75), (index, name, row, truth)
            priors.update({name: row['degrees'] for name, row in observed.items()})
            if index == 527:
                legacy = depth_flap_angles(*inputs, {'long_near': -15., 'long_far': -5.})
                assert legacy['long_near']['degrees'] > 5.
                assert observed['long_near']['pixel_support'] > 1000
                assert 'long_near' in depth_major_flap_angles(*inputs, priors)
                # Retain only the arm-occluded centre stripe using observed
                # metric points, not simulator segmentation or object IDs.
                yy, xx = np.indices(depth.shape)
                z = depth
                camera_points = np.stack(((xx - k[0, 2]) * z / k[0, 0],
                                          (yy - k[1, 2]) * z / k[1, 1], z), axis=-1)
                transform = np.linalg.inv(box) @ registration.world_from_camera
                box_points = np.einsum('...j,ij->...i', camera_points, transform[:3, :3]) + transform[:3, 3]
                masked_depth = np.where(np.abs(box_points[:, :, 0]) < .028, depth, 0.)
                assert 'long_near' not in depth_major_flap_angles(
                    rgb, masked_depth, k, registration.world_from_camera, box, priors)
