"""Explicit primary-cache short planes; no renderer, clock or gate bypass."""
import copy
from dataclasses import replace

import numpy as np
import pytest

from carton import folding_additional_view as view
from carton.folding_observation_recording import LastRGBDFrameCache
from carton.folding_partial_short_probe import _contact_reading
from test_folding_additional_view import (
    enable_absence, open_config, packet, rig, short_plane,
)


def primary_config(**changes):
    return replace(open_config(), observe_primary_open_shorts=True, **changes)


def current_packet():
    row = packet()
    row.update(rgb_timestamp_s=2., depth_timestamp_s=2.,
               world_from_camera=np.eye(4).tolist(), observer_history={'seq': 1})
    return row


def cache(*, camera='station', clock='offline:frozen-clock', seq=1, time=2.):
    value = LastRGBDFrameCache(camera, clock_id=clock)
    rgb = np.zeros((16, 24, 3), np.uint8)
    depth = np.ones((16, 24), np.float32)
    value.capture(rgb, depth, seq=seq, timestamp_s=time, intrinsics=np.eye(3))
    return value


def enable_primary(rig, monkeypatch, *, enabled=True):
    _, primary, controls = rig
    primary.last_rgbd_frame = cache()
    controls.primary_short_angles = {'short_left': short_plane(-12.),
                                     'short_right': short_plane(-15.)}
    controls.short_angles = copy.deepcopy(controls.primary_short_angles)
    controls.primary_inputs = []
    original_observe = primary.observe

    def observe(label):
        row = original_observe(label)
        primary.last_rgbd_frame.capture(np.zeros((16, 24, 3), np.uint8),
            np.ones((16, 24), np.float32), seq=row['seq'],
            timestamp_s=primary.sim.data.time, intrinsics=np.eye(3))
        return row

    def shorts(rgb, depth, k, camera, box, priors):
        if rgb[0, 0, 0] == 0:
            controls.primary_inputs.append(dict(rgb=rgb.copy(), depth=depth.copy(),
                k=k.copy(), camera=camera.copy(), box=box.copy(), priors=copy.deepcopy(priors)))
            return copy.deepcopy(controls.primary_short_angles)
        return copy.deepcopy(controls.short_angles)

    primary.observe = observe
    monkeypatch.setattr(view, 'depth_open_short_flap_angles', shorts)
    cfg = replace(open_config(), observe_primary_open_shorts=enabled)
    return view.AdditionalViewPixelPort(primary, configuration=cfg, seed=3), primary, controls


@pytest.mark.parametrize('changes', [dict(observe_open_shorts=False),
                                    dict(observe_primary_open_shorts=1)])
def test_primary_requires_explicit_boolean_and_short_mode(changes):
    with pytest.raises(ValueError, match='Primary open-short opt-in'):
        replace(primary_config(), **changes)


def test_primary_opt_in_requires_real_exposed_frame_cache(rig):
    _, primary, _ = rig
    with pytest.raises(ValueError, match='exact exposed-frame caching'):
        view.AdditionalViewPixelPort(primary, configuration=primary_config())


def test_default_open_short_mode_does_not_read_primary_cache(rig, monkeypatch):
    port, primary, controls = enable_primary(rig, monkeypatch, enabled=False)
    reading = port.observe('baseline')
    assert not controls.primary_inputs
    assert 'primary_open_short_observation' not in reading['additional_view']
    assert 'primary_open_short_source' not in port.declaration
    assert all(reading['angles'][name]['source_camera'] == 'front' for name in view.SHORTS)
    assert primary.render_calls == [('front', False), ('front', True)]


def test_primary_uses_exact_current_cache_and_own_pose_with_source_coherent_targets(rig, monkeypatch):
    port, primary, controls = enable_primary(rig, monkeypatch)
    primary.reading['world_from_box'][0][3] = .005
    primary.observer.world_from_camera[1, 3] = .004
    primary.reading['angles']['short_left'] = dict(degrees=19., method='aligned_depth_cardboard_plane')
    controls.short_angles.pop('short_left')  # Secondary ambiguity/absence is not filled there.
    original = copy.deepcopy(primary.reading)
    reading = port.observe('both registered')
    audit = reading['additional_view']
    evidence = audit['primary_open_short_observation']
    assert primary.render_calls == [('front', False), ('front', True)]
    assert len(controls.primary_inputs) == 1
    inputs = controls.primary_inputs[0]
    np.testing.assert_array_equal(inputs['box'], original['world_from_box'])
    np.testing.assert_array_equal(inputs['camera'], primary.observer.world_from_camera)
    assert inputs['depth'].dtype == np.float32
    assert evidence['frame_metadata'] == primary.last_rgbd_frame.metadata
    assert evidence['original_pixelport_short_angles'] == {'short_left': original['angles']['short_left']}
    assert evidence['comparisons']['short_left']['legacy_stripe_omitted'] is True
    assert primary.readings[-1]['angles'] == original['angles']
    assert 'short_left' not in audit['additional']['angles']
    assert reading['angles']['short_left']['source_camera'] == 'station'
    assert reading['angles']['short_left']['method'] == 'aligned_depth_open_short_hinge_consistent_plane'
    coherent, source = _contact_reading(reading, 'left')
    assert coherent['world_from_box'] == audit['primary']['world_from_box']
    assert coherent['world_from_box'] != audit['additional']['world_from_box']
    assert source['camera'] == 'station'


@pytest.mark.parametrize('mutation,match', [
    (lambda c: c.clear(), 'not_captured'),
    (lambda c: None, 'different_sequence'),
])
def test_empty_and_old_cache_cannot_supply_current_primary(mutation, match, monkeypatch):
    primary = current_packet(); value = cache()
    mutation(value)
    if match == 'different_sequence':
        primary['seq'] = primary['observer_history']['seq'] = 2
    monkeypatch.setattr(view, 'depth_open_short_flap_angles', lambda *a: pytest.fail('stale pixels used'))
    with pytest.raises(ValueError, match=match):
        view._with_primary_open_shorts(primary, value, primary_config(), timestamp=2.)


@pytest.mark.parametrize('value,match', [
    (dict(camera='front'), 'Exact current'),
    (dict(clock='offline:old-clock'), 'different_clock'),
    (dict(time=1.), 'different_timestamp'),
])
def test_wrong_cache_identity_refuses_before_estimation(value, match, monkeypatch):
    monkeypatch.setattr(view, 'depth_open_short_flap_angles', lambda *a: pytest.fail('wrong pixels used'))
    with pytest.raises(ValueError, match=match):
        view._with_primary_open_shorts(current_packet(), cache(**value), primary_config(), timestamp=2.)


@pytest.mark.parametrize('mutate', [
    lambda p: p.pop('world_from_box'),
    lambda p: p.pop('world_from_camera'),
    lambda p: p['observer_history'].update(seq=0),
    lambda p: p.update(rgb_timestamp_s=1.),
    lambda p: p.update(camera='front'),
    lambda p: p.update(source='privileged_mechanics_probe'),
])
def test_missing_or_mismatched_current_registration_never_uses_pixels(mutate, monkeypatch):
    primary = current_packet(); mutate(primary)
    monkeypatch.setattr(view, 'depth_open_short_flap_angles', lambda *a: pytest.fail('uncalibrated pixels used'))
    with pytest.raises(ValueError):
        view._with_primary_open_shorts(primary, cache(), primary_config(), timestamp=2.)


def test_absent_primary_box_skips_estimator_and_never_borrows_secondary_pose(rig, monkeypatch):
    previous, primary, controls = enable_absence(rig)
    primary.last_rgbd_frame = cache()
    port = view.AdditionalViewPixelPort(previous, configuration=replace(previous.configuration,
        observe_primary_open_shorts=True))
    # A missing cache is deliberately safe here: no primary carton pose exists.
    reading = port.observe('explicit current absence')
    evidence = reading['additional_view']['primary_open_short_observation']
    assert evidence == dict(status='not_observed', reason='current_primary_carton_identity_absent')
    assert 'world_from_box' not in reading['additional_view']['primary']
    assert [row[0] for row in controls.pixel_calls] == ['major', 'short']
    assert all(reading['angles'][name]['source_camera'] == 'front' for name in view.SHORTS)
    assert not port.primary_short_priors


def test_no_primary_plane_does_not_revive_legacy_or_prior(rig, monkeypatch):
    port, primary, controls = enable_primary(rig, monkeypatch)
    primary.reading['angles']['short_left'] = dict(degrees=-12., method='aligned_depth_cardboard_plane')
    port.primary_short_priors = {'short_left': -12.}
    controls.primary_short_angles.pop('short_left')
    controls.short_angles.pop('short_left')
    with pytest.raises(ValueError, match='Fresh required short_left missing'):
        port.observe('no trusted plane')
    assert 'short_left' not in port.additional_view_history[-1]['primary']['angles']
    assert primary._observation_failed is True
    assert port.primary_short_priors == {'short_left': -12.}


def test_current_decoded_tag_keeps_precedence_after_plane_corroboration(rig, monkeypatch):
    port, primary, _ = enable_primary(rig, monkeypatch)
    primary.reading['tags'].append(11)
    tag = dict(degrees=-11.5, method='apriltag_aligned_depth_plane')
    primary.reading['angles']['short_left'] = tag
    reading = port.observe('tag and wide plane')
    row = reading['angles']['short_left']
    assert row['degrees'] == -11.5 and row['method'] == tag['method']
    assert reading['additional_view']['primary_open_short_observation']['comparisons']['short_left']['difference_degrees'] == .5


@pytest.mark.parametrize('decoded,angle,match', [(True, -6., 'identified estimate and hinge plane disagree'),
                                               (False, -12., 'decoded flap identity absent')])
def test_bad_tag_cannot_be_overridden_by_good_primary_plane(rig, monkeypatch, decoded, angle, match):
    port, primary, _ = enable_primary(rig, monkeypatch)
    if decoded:
        primary.reading['tags'].append(11)
    primary.reading['angles']['short_left'] = dict(degrees=angle, method='apriltag_aligned_depth_plane')
    with pytest.raises(ValueError, match=match):
        port.observe('invalid tag')
    assert not primary.render_calls and primary._observation_failed is True
    assert not port.primary_short_priors


@pytest.mark.parametrize('change', [dict(pixel_support=59), dict(supported_patches=5),
    dict(along_span_mm=74.), dict(radial_span_mm=29.), dict(hinge_axis_error_deg=4.1),
    dict(hinge_plane_offset_mm=6.1), dict(competing_plane_support_ratio=.35),
    dict(degrees=30.), dict(valid_angle_bounds_degrees=[-40., 31.])])
def test_primary_cannot_weaken_any_shared_plane_gate(rig, monkeypatch, change):
    port, primary, controls = enable_primary(rig, monkeypatch)
    controls.primary_short_angles['short_left'].update(change)
    with pytest.raises(ValueError):
        port.observe('bad primary plane')
    assert primary._observation_failed and not port.primary_short_priors


def test_cross_view_primary_plane_contradiction_stays_fatal_and_preserves_raw_sources(rig, monkeypatch):
    port, primary, controls = enable_primary(rig, monkeypatch)
    controls.short_angles['short_left']['degrees'] = -8.
    with pytest.raises(ValueError, match='Fresh independent camera views disagree on short_left'):
        port.observe('contradiction')
    audit = port.additional_view_history[-1]
    assert audit['primary']['angles']['short_left']['degrees'] == -12.
    assert audit['additional']['angles']['short_left']['degrees'] == -8.
    assert audit['primary_open_short_observation']['plane_estimates']['short_left']['degrees'] == -12.
    assert not port.primary_short_priors and not port.additional_priors
    with pytest.raises(ValueError, match='before motion'):
        port.move_arms({}, 1., 'blocked', None)


def test_primary_extra_processing_does_not_advance_secondary_rng_or_add_render(rig, monkeypatch):
    port, primary, controls = enable_primary(rig, monkeypatch)
    expected = np.random.default_rng(3)
    expected.normal(0, port.noise, (16, 24)); expected.random((16, 24))
    port.observe('current primary pixels')
    assert port.rng.bit_generator.state == expected.bit_generator.state
    assert primary.render_calls == [('front', False), ('front', True)]
    assert primary.sim.data.time == 2.
    assert port.primary_short_priors == {'short_left': -12., 'short_right': -15.}
    port.observe('next current pixels')
    assert controls.primary_inputs[-1]['priors'] == port.primary_short_priors


def test_rewrap_preserves_prefix_and_primary_evidence_without_relabeling_cache(rig, monkeypatch):
    previous, primary, _ = enable_primary(rig, monkeypatch)
    reading = previous.observe('accepted first phase')
    port = view.AdditionalViewPixelPort(previous, configuration=replace(primary_config(),
        assumption_id='offline:second-short-phase'))
    assert port.readings == previous.readings and port.readings[0] == reading
    assert port.additional_view_history == previous.additional_view_history
    assert port.primary_short_priors == previous.primary_short_priors
    assert primary.last_rgbd_frame.metadata is None
    assert port.declaration['previous_phase']['assumptions_sha256'] == previous.assumptions_sha256
