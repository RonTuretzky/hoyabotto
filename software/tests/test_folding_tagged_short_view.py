"""Explicit tagged closure phase; strict open-plane limits never expand."""
import copy
from dataclasses import replace

import numpy as np
import pytest

from carton import folding_additional_view as view
from carton.folding_partial_short_probe import _contact_reading
from test_folding_additional_view import open_config, packet, rig, short_plane
from test_folding_primary_open_short_view import enable_primary


def tagged_config(**changes):
    return replace(open_config(), observe_tagged_shorts=True, **changes)


def quality():
    return dict(valid_depth_pixels=200, square_fit_rms_mm=.2, plane_rms_mm=.3)


def tagged_row(angle):
    return dict(degrees=angle, method='apriltag_aligned_depth_plane')


def add_primary_tags(primary, values):
    for name, angle in values.items():
        tag = 11 if name == 'short_left' else 12
        primary.reading['tags'].append(tag)
        primary.reading['angles'][name] = tagged_row(angle)
    primary.observer.history[-1].update(detected=sorted(primary.reading['tags']),
        rejected={}, quality={11: quality(), 12: quality()})


def tag_pose(name, angle, world_from_box=None):
    """Analytic decoded-pose fixture, not rendered/image accuracy evidence."""
    theta = np.radians(angle)
    outward = -1 if name == 'short_left' else 1
    z = -np.array([outward * np.cos(theta), 0., np.sin(theta)])
    x = np.array([0., 1., 0.])
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    return pose if world_from_box is None else world_from_box @ pose


def enable_tagged(rig, *, primary_values=None, secondary_values=None, enabled=True):
    _, primary, controls = rig
    add_primary_tags(primary, primary_values or {})
    controls.tag_values = dict(secondary_values or {})
    controls.tag_history_mutation = lambda h, p: None
    port = view.AdditionalViewPixelPort(primary, configuration=replace(open_config(),
        observe_tagged_shorts=enabled), seed=3)
    original = port.additional_observer.observe

    def observe(*args, **kwargs):
        poses = original(*args, **kwargs)
        history = port.additional_observer.history[-1]
        history.update(rejected={})
        for name, angle in controls.tag_values.items():
            tag = 11 if name == 'short_left' else 12
            poses[tag] = tag_pose(name, angle, controls.box)
            history['quality'][tag] = quality()
        history['detected'] = sorted(poses)
        controls.tag_history_mutation(history, poses)
        return poses

    port.additional_observer.observe = observe
    return port, primary, controls


@pytest.mark.parametrize('changes', [dict(observe_open_shorts=False), dict(observe_tagged_shorts=1)])
def test_explicit_tagged_phase_requires_boolean_and_short_observation(changes):
    with pytest.raises(ValueError, match='Tagged-short phase'):
        replace(tagged_config(), **changes)


def test_default_cannot_use_secondary_tags_as_short_estimates(rig):
    port, _, _ = enable_tagged(rig, secondary_values={'short_left': 60., 'short_right': 90.}, enabled=False)
    assert not port.configuration.observe_tagged_shorts
    with pytest.raises(ValueError, match='Fresh required short_left missing'):
        port.observe('no implicit later phase')
    assert 'additional_tagged_short_observation' not in port.additional_view_history[-1]


def test_default_still_refuses_primary_tag_beyond_open_bounds(rig):
    port, _, _ = enable_tagged(rig, primary_values={'short_left': 60., 'short_right': 90.}, enabled=False)
    with pytest.raises(ValueError, match='outside explicitly supported open-short'):
        port.observe('no implicit primary extension')


def test_tagged_only_option_uses_current_history_even_without_primary_plane_mode(rig):
    values = {'short_left': 60., 'short_right': 90.}
    port, primary, _ = enable_tagged(rig, primary_values=values, secondary_values=values)
    assert not port.configuration.observe_primary_open_shorts
    # Actual ordinary PixelPort-shaped reading: observer history lives on the
    # observer, and is attached by the wrapper only for this current capture.
    assert 'observer_history' not in primary.reading
    reading = port.observe('explicit later tagged phase')
    assert 'observer_history' not in primary.readings[-1]
    assert reading['additional_view']['primary']['observer_history']['seq'] == reading['seq']
    for name, angle in values.items():
        row = reading['angles'][name]
        assert row['degrees'] == angle and row['source_camera'] == 'station'
        assert row['method'] == 'apriltag_aligned_depth_plane'
    assert primary.render_calls == [('front', False), ('front', True)]
    assert port.declaration['tagged_short_angle_bounds_degrees'] == [-40., 103.]
    assert 'no extension of open-plane bounds or motion authorization' in port.declaration['tagged_short_phase']


def test_additional_current_tag_uses_its_pose_and_explicit_source_method(rig):
    port, _, controls = enable_tagged(rig, secondary_values={'short_left': 60., 'short_right': 90.})
    controls.box[0, 3] = .004
    reading = port.observe('additional current tags')
    row = reading['angles']['short_left']
    assert row['degrees'] == pytest.approx(60.)
    assert row['method'] == 'additional_view_apriltag_aligned_depth_plane'
    assert row['source_method'] == 'apriltag_aligned_depth_plane'
    assert row['source_camera'] == 'front' and row['observed_seq'] == reading['seq']
    coherent, source = _contact_reading(reading, 'left')
    assert coherent['world_from_box'] == reading['additional_view']['additional']['world_from_box']
    assert coherent['world_from_box'] != reading['additional_view']['primary']['world_from_box']
    assert source['camera'] == 'front'
    assert reading['additional_view']['additional_tagged_short_observation']['comparisons']['short_left']['tag_depth_quality'] == quality()


def test_tagged_phase_can_mix_primary_and_additional_short_sources(rig):
    port, _, _ = enable_tagged(rig, primary_values={'short_left': 88.},
                              secondary_values={'short_right': 89.})
    reading = port.observe('each fresh identity in its source view')
    assert reading['angles']['short_left']['source_camera'] == 'station'
    assert reading['angles']['short_right']['source_camera'] == 'front'


@pytest.mark.parametrize('angle', [-40., 103., 170.])
def test_tag_range_is_existing_pixelport_range_not_clipped(rig, angle):
    port, _, _ = enable_tagged(rig, secondary_values={'short_left': angle, 'short_right': 90.})
    with pytest.raises(ValueError, match='existing observation range'):
        port.observe('outside existing tagged range')


@pytest.mark.parametrize('angle', [30., 65., 90.])
def test_tagged_phase_never_expands_plane_bounds(rig, angle):
    port, _, controls = enable_tagged(rig)
    controls.short_angles = {'short_left': short_plane(angle), 'short_right': short_plane(10.)}
    with pytest.raises(ValueError, match='outside explicitly supported open-short'):
        port.observe('out-of-range untagged plane')


def test_current_short_tag_and_strict_plane_are_corroborated(rig):
    port, _, controls = enable_tagged(rig, secondary_values={'short_left': 25., 'short_right': 25.})
    controls.short_angles = {'short_left': short_plane(24.), 'short_right': short_plane(24.)}
    reading = port.observe('shared overlap interval')
    row = reading['angles']['short_left']
    assert row['degrees'] == pytest.approx(25.) and row['depth_check_degrees'] == 24.
    assert reading['additional_view']['additional_tagged_short_observation']['comparisons']['short_left']['difference_degrees'] == pytest.approx(1.)


def test_current_short_tag_cannot_override_contradictory_strict_plane(rig):
    port, primary, controls = enable_tagged(rig, secondary_values={'short_left': 25., 'short_right': 25.})
    controls.short_angles = {'short_left': short_plane(20.), 'short_right': short_plane(25.)}
    with pytest.raises(ValueError, match='additional short tag and hinge plane disagree'):
        port.observe('contradictory current identity')
    assert primary._observation_failed and not port.additional_priors


def test_cross_view_current_tag_contradictions_remain_fatal(rig):
    port, primary, _ = enable_tagged(rig, primary_values={'short_left': 60., 'short_right': 90.},
        secondary_values={'short_left': 64., 'short_right': 90.})
    with pytest.raises(ValueError, match='views disagree on short_left'):
        port.observe('current decoded views disagree')
    audit = port.additional_view_history[-1]
    assert audit['primary']['angles']['short_left']['degrees'] == 60.
    assert audit['additional']['angles']['short_left']['degrees'] == pytest.approx(64.)
    assert primary._observation_failed
    with pytest.raises(ValueError, match='before motion'):
        port.move_arms({}, 1., 'blocked', None)


@pytest.mark.parametrize('role', ['primary', 'additional'])
@pytest.mark.parametrize('change', [dict(valid_depth_pixels=29), dict(square_fit_rms_mm=4.01),
    dict(plane_rms_mm=float('nan')), dict(valid_depth_pixels=True)])
def test_both_tag_sources_require_current_unchanged_depth_quality(rig, role, change):
    values = {'short_left': 60., 'short_right': 90.}
    port, primary, controls = enable_tagged(rig, primary_values=values, secondary_values=values)
    if role == 'primary':
        primary.observer.history[-1]['quality'][11].update(change)
    else:
        controls.tag_history_mutation = lambda h, p: h['quality'][11].update(change)
    with pytest.raises(ValueError, match='aligned tag-depth gate failed'):
        port.observe('bad current quality')
    assert primary._observation_failed


@pytest.mark.parametrize('role', ['primary', 'additional'])
@pytest.mark.parametrize('key', [11, '11'])
def test_decoded_but_rejected_short_cannot_fall_back_to_plane_or_other_view(rig, role, key):
    values = {'short_left': 10., 'short_right': 10.}
    port, primary, controls = enable_tagged(rig, primary_values=values, secondary_values=values)
    controls.short_angles = {name: short_plane(angle) for name, angle in values.items()}
    if role == 'primary':
        primary.reading['tags'].remove(11)
        primary.reading['angles']['short_left'] = short_plane(10.)
        h = primary.observer.history[-1]
        h['detected'].remove(11); h['quality'].pop(11)
        h['rejected'][key] = 'Marker is edge-on'
    else:
        def reject(h, p):
            p.pop(11); h['detected'].remove(11); h['quality'].pop(11)
            h['rejected'][key] = 'Insufficient valid aligned depth inside tag'
        controls.tag_history_mutation = reject
    with pytest.raises(ValueError, match='current decoded tag and matching accepted depth'):
        port.observe('decoded identity rejected by depth')
    assert primary._observation_failed


@pytest.mark.parametrize('mutation', [
    lambda h: h['quality'].pop(11),
    lambda h: h.update(seq=0),
    lambda h: h['detected'].remove(11),
    lambda h: h['quality'].update({'11': quality()}),
])
def test_secondary_missing_stale_or_ambiguous_history_refuses(rig, mutation):
    port, _, controls = enable_tagged(rig, secondary_values={'short_left': 60., 'short_right': 90.})
    controls.tag_history_mutation = lambda h, p: mutation(h)
    with pytest.raises(ValueError):
        port.observe('invalid tag history')


def test_missing_current_tag_does_not_use_tagged_prior_or_old_primary_row(rig):
    port, primary, controls = enable_tagged(rig, secondary_values={'short_left': 60., 'short_right': 90.})
    port.observe('accepted tagged state')
    controls.tag_values.pop('short_left')
    with pytest.raises(ValueError, match='Fresh required short_left missing'):
        port.observe('tag disappeared')
    assert primary._observation_failed


def test_primary_decoded_tag_without_current_tag_angle_refuses(rig):
    port, primary, _ = enable_tagged(rig, primary_values={'short_left': 60., 'short_right': 90.},
        secondary_values={'short_left': 60., 'short_right': 90.})
    primary.reading['angles'].pop('short_left')
    with pytest.raises(ValueError, match='decoded short tag lacks its current in-range tag angle'):
        port.observe('no current computed tag angle')


def test_primary_open_plane_option_remains_compatible_with_later_current_tags(rig, monkeypatch):
    previous, primary, controls = enable_primary(rig, monkeypatch)
    controls.primary_short_angles = {}
    controls.short_angles = {}
    add_primary_tags(primary, {'short_left': 60., 'short_right': 90.})
    port = view.AdditionalViewPixelPort(previous, configuration=replace(previous.configuration,
        observe_tagged_shorts=True))
    reading = port.observe('tags beyond plane domain with empty current plane result')
    assert reading['angles']['short_left']['degrees'] == 60.
    assert reading['additional_view']['primary_open_short_observation']['plane_estimates'] == {}
    assert not port.primary_short_priors


def test_tag_conversion_does_not_advance_noise_rng_or_recapture(rig):
    port, primary, _ = enable_tagged(rig, secondary_values={'short_left': 60., 'short_right': 90.})
    expected = np.random.default_rng(3)
    expected.normal(0, port.noise, (16, 24)); expected.random((16, 24))
    port.observe('existing current poses only')
    assert port.rng.bit_generator.state == expected.bit_generator.state
    assert primary.render_calls == [('front', False), ('front', True)]
    assert primary.sim.data.time == 2.


def test_tag_angle_is_invariant_to_common_world_frame_change():
    original = packet('front')
    original['tags'] += [11, 12]
    original['observer_history'] = dict(seq=1, detected=original['tags'], rejected={},
                                      quality={11: quality(), 12: quality()})
    poses = {11: tag_pose('short_left', 60.), 12: tag_pose('short_right', 90.)}
    before, _ = view._with_additional_tagged_shorts(original, poses, tagged_config())
    world = np.array([[0., -1., 0., .3], [1., 0., 0., -.2], [0., 0., 1., .04], [0., 0., 0., 1.]])
    changed = copy.deepcopy(original); changed['world_from_box'] = world.tolist()
    after, _ = view._with_additional_tagged_shorts(changed,
        {tag: world @ pose for tag, pose in poses.items()}, tagged_config())
    for name in view.SHORTS:
        assert before['angles'][name]['degrees'] == after['angles'][name]['degrees']
