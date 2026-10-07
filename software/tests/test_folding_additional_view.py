"""Opt-in camera composition, synchronization and refusal tests; no robot action."""
import copy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from carton import folding_additional_view as view


def config(**kwargs):
    return view.AdditionalViewConfiguration('offline:hypothetical-front-v1',
        'offline:frozen-clock', ('long_near', 'long_far'), **kwargs)


def plane(angle):
    return dict(degrees=angle, method='aligned_depth_hinge_consistent_plane',
                pixel_support=1000, supported_patches=12, along_span_mm=280.,
                radial_span_mm=90., hinge_axis_error_deg=.1, hinge_plane_offset_mm=.2)


def packet(camera='station'):
    return dict(seq=1, camera=camera, world_from_box=np.eye(4).tolist(), tags=[1, 10],
                angles={'long_near': plane(39.), 'long_far': plane(38.)},
                box_registration={'visible_ids': [10], 'selected_id': 10}, label='test')


class Primary:
    camera = 'station'

    def __init__(self):
        self.reading = packet()
        self.readings = []
        self.observer = SimpleNamespace(anchors={1: np.eye(4), 20: np.eye(4)},
            world_from_camera=np.eye(4), history=[{'seq': 1, 'quality': {}}])
        self.arm_tag_checks = ['primary housing checks preserved']
        self.calls, self.render_calls, self.motion_calls = 0, [], []
        self.fail_primary, self.advance_at, self.fovy = False, None, 48.
        self.sim = SimpleNamespace(data=SimpleNamespace(time=2.),
            model=SimpleNamespace(camera=lambda name: SimpleNamespace(fovy=[self.fovy])),
            render=self.render)

    def observe(self, label):
        self.calls += 1
        if self.fail_primary:
            raise ValueError('Both housing tags required for initial arm registration check')
        if self.advance_at == 'primary':
            self.sim.data.time += .001
        self.reading['seq'] = self.calls
        self.reading['label'] = label
        self.observer.history[-1]['seq'] = self.calls
        reading = copy.deepcopy(self.reading)
        self.readings.append(reading)
        return reading

    def render(self, camera, depth=False):
        self.render_calls.append((camera, depth))
        if self.advance_at == ('depth' if depth else 'rgb'):
            self.sim.data.time += .001
        return np.ones((16, 24), dtype=float) if depth else np.full((16, 24, 3), 100, np.uint8)

    def move_arms(self, *args, **kwargs):
        self.motion_calls.append(('arms', args, kwargs))
        return 'normal primary collision guards'

    def set_grippers(self, *args, **kwargs):
        self.motion_calls.append(('grippers', args, kwargs))
        return 'normal primary gripper guards'


@pytest.fixture
def rig(monkeypatch):
    primary = Primary()
    controls = SimpleNamespace(angles=packet('front')['angles'], box=np.eye(4),
                               fail_anchor=False, fail_carton=False, calls=[], priors=[],
                               short_angles={}, pixel_calls=[])

    class Observer:
        def __init__(self, anchor, *, additional_anchors, stationary_camera):
            assert stationary_camera is False
            self.anchors = {1: anchor, **additional_anchors}
            self.world_from_camera = np.eye(4)
            self.history = []

        def observe(self, rgb, depth, k, *, seq, timestamp, depth_timestamp):
            controls.calls.append((seq, timestamp, depth_timestamp))
            if controls.fail_anchor:
                raise ValueError('Fresh table tag and aligned depth registration not observed')
            self.history.append(dict(seq=seq, quality={}, anchor_ids=[1], detected=[1, 10],
                                     stationary_camera=False))
            return {1: np.eye(4), 10: np.eye(4)}

    def carton(tags, quality):
        if controls.fail_carton:
            raise ValueError('Fresh carton marker and aligned depth required')
        return controls.box.copy(), {'visible_ids': [10], 'selected_id': 10}

    def majors(rgb, depth, k, camera, box, priors):
        controls.priors.append(copy.deepcopy(priors))
        controls.pixel_calls.append(('major', id(rgb), id(depth), id(k), id(camera), id(box)))
        return copy.deepcopy(controls.angles)

    def shorts(rgb, depth, k, camera, box, priors):
        controls.pixel_calls.append(('short', id(rgb), id(depth), id(k), id(camera), id(box)))
        return copy.deepcopy(controls.short_angles)

    monkeypatch.setattr(view, 'RGBDTagObserver', Observer)
    monkeypatch.setattr(view, 'carton_pose_from_tags', carton)
    monkeypatch.setattr(view, 'depth_major_flap_angles', majors)
    monkeypatch.setattr(view, 'depth_open_short_flap_angles', shorts)
    port = view.AdditionalViewPixelPort(primary, configuration=config(), seed=3)
    return port, primary, controls


def test_explicit_opt_in_primary_startup_and_motion_delegation(rig):
    port, primary, _ = rig
    assert port.observer is primary.observer
    assert port.arm_tag_checks is primary.arm_tag_checks
    assert port.move_arms({'left': [0]}, 1, 'move', None) == 'normal primary collision guards'
    assert port.set_grippers({'left': .3}, 1, 'grip') == 'normal primary gripper guards'
    assert len(primary.motion_calls) == 2
    assert port.declaration['physical_camera_verified'] is False
    assert port.declaration['simulation_only'] is True
    assert len(port.assumptions_sha256) == 64


@pytest.mark.parametrize('changes', [dict(assumption_id='measured'), dict(clock_id='wall'),
    dict(camera='overhead'), dict(primary_camera='front'), dict(required_flaps=()),
    dict(required_flaps=('short_left',)), dict(required_flaps=('long_far', 'long_far')),
    dict(camera_fovy_degrees=60), dict(max_carton_translation_m=.013),
    dict(max_carton_rotation_degrees=9), dict(max_flap_disagreement_degrees=4),
    dict(max_carton_translation_m=float('nan'))])
def test_invalid_or_weakened_declarations_refused(changes):
    with pytest.raises(ValueError):
        replace(config(), **changes)


def test_front_cannot_replace_primary_startup_view():
    primary = Primary(); primary.camera = 'front'
    with pytest.raises(ValueError, match='Station PixelPort'):
        view.AdditionalViewPixelPort(primary, configuration=config())


def test_primary_housing_failure_cannot_be_rescued_or_bypassed(rig):
    port, primary, controls = rig
    primary.fail_primary = True
    with pytest.raises(ValueError, match='Both housing tags'):
        port.observe('startup')
    assert not controls.calls and not primary.render_calls
    primary.fail_primary = False
    with pytest.raises(ValueError, match='reconstruct'):
        port.observe('retry')
    with pytest.raises(ValueError, match='before motion'):
        port.move_arms({}, 1, '', None)
    with pytest.raises(ValueError, match='before motion'):
        port.set_grippers({}, 1, '')
    assert primary.calls == 1 and not primary.motion_calls


def test_missing_primary_angle_uses_fresh_secondary_with_explicit_provenance(rig):
    port, primary, controls = rig
    del primary.reading['angles']['long_far']
    reading = port.observe('edge-on primary')
    row = reading['angles']['long_far']
    assert row['degrees'] == 38.
    assert row['source_camera'] == 'front'
    assert row['method'] == 'additional_view_hinge_consistent_plane'
    assert row['source_method'] == 'aligned_depth_hinge_consistent_plane'
    assert reading['angles']['long_near']['source_camera'] == 'station'
    # The existing portable single-camera adapter rejects this source rather
    # than treating front angles as if observed by its station calibration.
    assert reading['source'] not in (None, 'calibrated_rgbd')
    audit = reading['additional_view']
    assert audit['status'] == 'accepted'
    assert audit['additional']['observer_history']['stationary_camera'] is False
    assert audit['primary']['observer_history']['seq'] == audit['additional']['seq'] == 1
    assert audit['primary']['rgb_timestamp_s'] == audit['additional']['depth_timestamp_s'] == 2.
    assert len(audit['additional']['rgb_sha256']) == len(audit['additional']['exposed_depth_sha256']) == 64
    assert controls.calls == [(1, 2., 2.)]
    assert port.readings[-1] is reading and len(port.additional_view_history) == 1


def test_agreement_preserves_primary_without_averaging(rig):
    port, _, controls = rig
    controls.angles['long_far']['degrees'] = 39.7
    result = port.observe('both')
    assert result['angles']['long_far']['degrees'] == 38.
    assert result['additional_view']['comparison']['angles']['long_far']['difference_degrees'] == pytest.approx(1.7)


@pytest.mark.parametrize('name', view.MAJORS)
def test_either_common_major_disagreement_refuses(rig, name):
    port, _, controls = rig
    controls.angles[name]['degrees'] += 3.01
    with pytest.raises(ValueError, match='disagree on '+name):
        port.observe('conflict')
    assert port.readings[-1]['angles'] == {}
    assert port.additional_view_history[-1]['additional']['angles'][name]['degrees'] == controls.angles[name]['degrees']


@pytest.mark.parametrize('kind', ['translation', 'rotation'])
def test_carton_cross_view_disagreement_refuses_before_angle_use(rig, kind):
    port, _, controls = rig
    if kind == 'translation': controls.box[0, 3] = .01201
    else:
        angle = np.radians(8.01)
        controls.box[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    with pytest.raises(ValueError, match='carton pose'):
        port.observe('wrong registration')


@pytest.mark.parametrize('kind', ['primary', 'rgb', 'depth'])
def test_moving_simulation_clock_refuses_mixed_state_images(rig, kind):
    port, primary, _ = rig
    primary.advance_at = kind
    with pytest.raises(ValueError, match='same frozen simulated state'):
        port.observe('mixed time')
    assert port.additional_view_history[-1]['status'] == 'refused'


def test_wrong_intrinsics_refused(rig):
    port, primary, _ = rig; primary.fovy = 60.
    with pytest.raises(ValueError, match='intrinsics'):
        port.observe('wrong camera')


@pytest.mark.parametrize('failure', ['fail_anchor', 'fail_carton'])
def test_additional_registration_must_be_fresh_even_if_primary_angles_complete(rig, failure):
    port, _, controls = rig
    port.observe('first')
    setattr(controls, failure, True)
    with pytest.raises(ValueError, match='Fresh'):
        port.observe('occluded now')
    assert port.readings[-1]['angles'] == {}
    assert len(port.additional_view_history) == 2


def test_prior_does_not_fill_missing_required_angle(rig):
    port, primary, controls = rig
    port.observe('visible')
    del primary.reading['angles']['long_far']
    del controls.angles['long_far']
    with pytest.raises(ValueError, match='missing from both'):
        port.observe('now occluded')
    assert controls.priors[-1]['long_far'] == 38.
    assert port.readings[-1]['angles'] == {}


@pytest.mark.parametrize('key,value', [('supported_patches', 5), ('pixel_support', 59),
    ('along_span_mm', 74.), ('radial_span_mm', 29.), ('hinge_axis_error_deg', 4.01),
    ('hinge_plane_offset_mm', -6.01), ('method', 'legacy_cardboard_histogram'),
    ('method', 'PRIVILEGED_SIMULATOR_ANGLE'), ('unambiguous', False), ('observed_seq', 0),
    ('degrees', float('nan'))])
def test_no_weakening_of_major_plane_identity_or_support(rig, key, value):
    port, _, controls = rig
    controls.angles['long_far'][key] = value
    with pytest.raises(ValueError): port.observe('invalid identity')


def test_primary_tag_angle_needs_current_decoded_identity(rig):
    port, primary, _ = rig
    primary.reading['angles']['long_far'] = dict(degrees=38., method='apriltag_aligned_depth_plane')
    with pytest.raises(ValueError, match='decoded flap identity absent'):
        port.observe('missing tag')


def test_optional_missing_major_is_omitted_but_never_restored_from_default():
    first, second = packet(), packet('front')
    del first['angles']['long_near']; del second['angles']['long_near']
    cfg = replace(config(), required_flaps=('long_far',))
    result, _ = view._combine(first, second, cfg)
    assert set(result) == {'long_far'}


def test_ambiguous_secondary_is_not_ignored_when_primary_valid(rig):
    port, _, controls = rig
    controls.angles['long_far']['unambiguous'] = False
    with pytest.raises(ValueError, match='ambiguous'):
        port.observe('ambiguous')


def test_hidden_object_truth_does_not_enter_fixed_pixel_packet(rig):
    port, primary, controls = rig
    primary.sim.hidden_carton_qpos = [100., -200., 3.]
    first = port.observe('fixed pixels')
    # Independent hidden state changes; identical exposed clock, images,
    # declared calibration and packet sequence must give identical results.
    primary.sim.hidden_carton_qpos = [-10., 2000., 90.]
    primary.calls = 0
    primary.readings = []
    again = view.AdditionalViewPixelPort(primary, configuration=config(), seed=3)
    second = again.observe('fixed pixels')
    assert first == second


def test_enabling_after_primary_prefix_preserves_existing_trace(rig):
    _, primary, _ = rig
    prefix = primary.observe('near fold completed')
    port = view.AdditionalViewPixelPort(primary, configuration=config())
    assert port.readings == [prefix]
    assert port.readings[0] is not primary.readings[0]
    assert 'additional_view' not in port.readings[0]
    assert port.declaration['activation_after_primary_sequence'] == 1
    port.observe('far begins')
    assert [r['seq'] for r in port.readings] == [1, 2]
    assert port.readings[-1]['additional_view']['status'] == 'accepted'


def test_old_primary_reading_cannot_be_reused_when_enabling_additional_view(rig):
    _, primary, _ = rig
    primary.observe('prefix')
    port = view.AdditionalViewPixelPort(primary, configuration=config())
    primary.calls = 0
    with pytest.raises(ValueError, match='Fresh primary observation sequence'):
        port.observe('cached reading')


def test_primary_history_must_be_same_sequence(rig):
    port, primary, _ = rig
    observe = primary.observe
    def stale_history(label):
        reading = observe(label)
        primary.observer.history[-1]['seq'] = 0
        return reading
    primary.observe = stale_history
    with pytest.raises(ValueError, match='Fresh primary observer history'):
        port.observe('stale registration')


@pytest.mark.parametrize('has_later_reading', [False, True])
def test_constructing_on_failed_primary_startup_is_refused(rig, has_later_reading):
    _, primary, _ = rig
    primary.seq = 2
    if has_later_reading:
        row = packet(); row['seq'] = 2; primary.readings = [row]
    with pytest.raises(ValueError, match='failed or missing primary startup'):
        view.AdditionalViewPixelPort(primary, configuration=config())


def test_privileged_primary_source_is_refused(rig):
    port, primary, _ = rig
    primary.reading['privileged_mechanics_probe'] = True
    with pytest.raises(ValueError, match='ordinary pixel'):
        port.observe('probe')


def open_config():
    return replace(config(), assumption_id='offline:open-short-stage',
                   required_flaps=view.MAJORS + view.SHORTS, observe_open_shorts=True)


def short_plane(angle):
    row = plane(angle)
    row.update(method='aligned_depth_open_short_hinge_consistent_plane',
               valid_angle_bounds_degrees=[-40., 30.], competing_plane_support_ratio=.05)
    return row


def enable_shorts(rig):
    _, primary, controls = rig
    controls.short_angles = {'short_left': short_plane(-12.), 'short_right': short_plane(-15.)}
    primary.reading['angles']['short_right'] = dict(degrees=-14., method='aligned_depth_cardboard_plane')
    return view.AdditionalViewPixelPort(primary, configuration=open_config()), primary, controls


def test_default_major_mode_never_calls_open_short_observer(rig):
    port, _, controls = rig
    port.observe('major phase')
    assert [row[0] for row in controls.pixel_calls] == ['major']
    assert 'open_short_angle_bounds_degrees' not in port.declaration


def test_open_short_opt_in_uses_same_registered_pixels_without_recapture(rig):
    port, primary, controls = enable_shorts(rig)
    reading = port.observe('paired short stage')
    assert primary.render_calls == [('front', False), ('front', True)]
    assert controls.pixel_calls[0][1:] == controls.pixel_calls[1][1:]
    assert set(reading['angles']) == set(view.MAJORS + view.SHORTS)
    for name in view.SHORTS:
        row = reading['angles'][name]
        assert row['method'] == 'additional_view_open_short_hinge_consistent_plane'
        assert row['valid_angle_bounds_degrees'] == [-40., 30.]
        assert row['source_camera'] == 'front'
    comparison = reading['additional_view']['comparison']['angles']['short_right']
    assert comparison['primary_degrees'] is None
    assert comparison['primary_omitted_method'] == 'aligned_depth_cardboard_plane'


def test_required_short_cannot_be_filled_by_legacy_primary_or_prior(rig):
    port, _, controls = enable_shorts(rig)
    port.observe('seen')
    del controls.short_angles['short_right']
    with pytest.raises(ValueError, match='required short_right missing'):
        port.observe('hidden')
    assert port.readings[-1]['angles'] == {}


@pytest.mark.parametrize('field,value', [('degrees', 30.), ('degrees', -40.),
    ('valid_angle_bounds_degrees', [-40., 103.]), ('competing_plane_support_ratio', .35),
    ('supported_patches', 5), ('method', 'aligned_depth_cardboard_plane')])
def test_invalid_open_short_identity_or_scope_refuses(rig, field, value):
    port, _, controls = enable_shorts(rig)
    controls.short_angles['short_left'][field] = value
    with pytest.raises(ValueError): port.observe('invalid short')


@pytest.mark.parametrize('name,tag', [('short_left', 11), ('short_right', 12)])
def test_disagreeing_current_identified_short_tags_refuse(rig, name, tag):
    port, primary, controls = enable_shorts(rig)
    primary.reading['tags'].append(tag)
    primary.reading['angles'][name] = dict(degrees=controls.short_angles[name]['degrees']+3.01,
                                         method='apriltag_aligned_depth_plane')
    with pytest.raises(ValueError, match='disagree on '+name):
        port.observe('tag conflict')


def test_current_tag_can_supply_missing_additional_short_within_open_range(rig):
    port, primary, controls = enable_shorts(rig)
    primary.reading['tags'].append(11)
    primary.reading['angles']['short_left'] = dict(degrees=-12., method='apriltag_aligned_depth_plane')
    del controls.short_angles['short_left']
    result = port.observe('current identified primary')
    assert result['angles']['short_left']['source_camera'] == 'station'


def test_open_short_phase_preserves_full_prior_multiview_provenance(rig):
    previous, primary, controls = rig
    first = previous.observe('major stage')
    controls.short_angles = {'short_left': short_plane(-12.), 'short_right': short_plane(-15.)}
    port = view.AdditionalViewPixelPort(previous, configuration=open_config())
    assert port.primary is primary
    assert port.readings == [first] and port.readings[0] is not first
    assert port.additional_view_history == previous.additional_view_history
    assert port.declaration['previous_phase']['assumptions_sha256'] == previous.assumptions_sha256
    assert port.declaration['previous_phase']['declaration'] == previous.declaration
    port.observe('short stage')
    assert len(port.readings) == len(port.additional_view_history) == 2
    assert port.readings[0] == first
    assert port.additional_view_history[-1]['assumption_id'] == 'offline:open-short-stage'


def test_rewrapping_failed_view_cannot_bypass_failure_latch(rig):
    previous, _, controls = rig
    controls.fail_anchor = True
    with pytest.raises(ValueError): previous.observe('failed phase')
    with pytest.raises(ValueError, match='new primary startup'):
        view.AdditionalViewPixelPort(previous, configuration=open_config())
