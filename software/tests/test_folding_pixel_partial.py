"""Explicit post-startup carton absence; no image or robot truth shortcuts."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from carton.folding_markers import box_marker_poses
from carton.folding_observation_status import PARTIAL_VIEW_SCHEMA, PARTIAL_VIEW_SOURCE
from tools import simulate_bimanual_folding as driver


@pytest.fixture
def pixel(monkeypatch):
    mounts = box_marker_poses()
    control = SimpleNamespace(ids={1, 2, 4, 10, 20}, rejected={}, anchor_error=None,
                              tag_poses={}, fk_error=0., motion_calls=[])

    class Observer:
        def __init__(self, anchor, *, additional_anchors, stationary_camera):
            self.anchors = {1: anchor, **additional_anchors}
            self.world_from_camera = np.eye(4)
            self.history = []

        def observe(self, rgb, depth, k, *, seq, timestamp, depth_timestamp):
            if control.anchor_error:
                raise ValueError(control.anchor_error)
            tags = {i: control.tag_poses.get(i, mounts.get(i, np.eye(4))).copy()
                    for i in control.ids}
            self.history.append(dict(seq=seq, detected=sorted(tags),
                rejected=copy.deepcopy(control.rejected),
                quality={i: {'valid_depth_pixels': 50} for i in tags},
                anchor_ids=[1, 20], anchor_fit_rms_mm=.3, stationary_camera=True))
            return tags

    def fk(side):
        pose = np.eye(4); pose[0, 3] = control.fk_error
        return pose

    sim = SimpleNamespace(station=SimpleNamespace(table_tag_position=[-.5, .55, .0013],
            backup_table_marker_xy=[.45, .7]), data=SimpleNamespace(time=2.),
        model=SimpleNamespace(camera=lambda name: SimpleNamespace(fovy=[48.])),
        render=lambda camera, depth=False: (np.ones((16, 24), float) if depth
                                           else np.full((16, 24, 3), 150, np.uint8)),
        arm_tag_fk=fk, move=lambda *a, **kw: control.motion_calls.append((a, kw)))
    monkeypatch.setattr(driver, 'RGBDTagObserver', Observer)
    monkeypatch.setattr(driver, 'depth_flap_angles', lambda *a: {})
    monkeypatch.setattr(driver, 'depth_major_flap_angles', lambda *a: {})
    monkeypatch.setattr(driver, 'paddle_pose_from_tags', lambda *a: None)
    return driver.PixelPort(sim, camera='station', record=False), control


def start(pixel):
    port, control = pixel
    initial = port.observe('normal startup')
    assert initial['seq'] == 1 and port.startup_registration_verified
    control.ids = {1, 2, 20}
    return port, control, initial


def test_partial_requires_explicit_api_and_no_carton_cache(pixel):
    port, _, initial = start(pixel)
    old_box = port.box.copy()
    port.angle_priors = {'long_near': 91.}
    reading = port.observe_with_carton_absence('missing current identity')
    assert reading['source'] == PARTIAL_VIEW_SOURCE
    assert reading['packet_schema'] == PARTIAL_VIEW_SCHEMA
    assert reading['carton_status'] == 'missing_carton_identity'
    assert not {'angles', 'world_from_box', 'box_registration'} & reading.keys()
    assert port.box is None
    assert reading['seq'] == reading['observer_history']['seq'] == 2
    assert reading['rgb_timestamp_s'] == reading['depth_timestamp_s'] == 2.
    assert reading['tags'] == [1, 2, 20]
    assert reading['housing_fk_checks'] == [{'seq': 2, 'arm': 'right', 'tag_id': 2,
                                           'encoder_fk_error_mm': 0.}]
    assert len(reading['calibration_sha256']) == len(reading['exposed_depth_sha256']) == 64
    assert np.array_equal(old_box, initial['world_from_box'])
    assert port.readings == [initial, reading]


def test_default_observe_still_refuses_and_cannot_be_retried_as_partial(pixel):
    port, _, _ = start(pixel)
    with pytest.raises(ValueError, match='Fresh carton marker'): port.observe('missing')
    with pytest.raises(ValueError, match='Pixel observation failed'):
        port.observe_with_carton_absence('try rescue later')
    with pytest.raises(ValueError, match='before motion'): port.move_arms({}, 1, '', None)
    with pytest.raises(ValueError, match='before motion'): port.set_grippers({}, 1, '')


@pytest.mark.parametrize('missing', [set(), {10}, {2}, {4}])
def test_missing_or_failed_startup_cannot_be_counted_as_verified(pixel, missing):
    port, control = pixel
    with pytest.raises(ValueError, match='Normal initial registration'):
        port.observe_with_carton_absence('skip startup')
    control.ids -= missing or {2, 4}
    with pytest.raises(ValueError): port.observe('bad startup')
    assert not port.startup_registration_verified
    control.ids = {1, 2, 4, 10, 20}
    with pytest.raises(ValueError, match='Normal initial registration'):
        port.observe_with_carton_absence('failed seq1')
    with pytest.raises(ValueError, match='Pixel observation failed'): port.observe('retry seq2')


def test_bad_visible_housing_fk_cannot_be_rescued(pixel):
    port, control, _ = start(pixel); control.fk_error = .0121
    with pytest.raises(ValueError, match='Gripper tag disagrees'):
        port.observe_with_carton_absence('missing box and bad FK')
    assert port._observation_failed and len(port.readings) == 1


@pytest.mark.parametrize('reason', ['Fresh table tag and aligned depth registration not observed',
    'Fresh anchors disagree with stationary camera or surveyed geometry by over 6 mm',
    'Stale RGB/depth sequence', 'Unsynchronized RGB/depth', 'duplicate tag ID 10'])
def test_anchor_or_identity_failures_do_not_become_partial(pixel, reason):
    port, control, _ = start(pixel); control.anchor_error = reason
    with pytest.raises(ValueError, match=reason): port.observe_with_carton_absence('invalid')
    assert port._observation_failed and len(port.readings) == 1


@pytest.mark.parametrize('tag', [10, 21, 22, 11, 12])
def test_decoded_but_bad_carton_depth_is_not_missing_identity(pixel, tag):
    port, control, _ = start(pixel)
    control.rejected = {tag: 'Insufficient valid aligned depth inside tag'}
    with pytest.raises(ValueError, match='Fresh carton marker'):
        port.observe_with_carton_absence('invalid depth')
    assert port._observation_failed


def test_disagreeing_current_carton_markers_remain_fatal(pixel):
    port, control, _ = start(pixel); control.ids |= {10, 21}
    wrong = box_marker_poses()[21].copy(); wrong[0, 3] += .013
    control.tag_poses[21] = wrong
    with pytest.raises(ValueError, match='Fresh carton markers disagree'):
        port.observe_with_carton_absence('ambiguous carton')


def test_fresh_reappearing_carton_uses_normal_observation(pixel):
    port, control, _ = start(pixel)
    port.observe_with_carton_absence('missing')
    control.ids |= {10}
    current = port.observe_with_carton_absence('fresh identity returns')
    assert current['seq'] == 3 and 'world_from_box' in current
    assert 'packet_schema' not in current and 'carton_status' not in current


def test_hidden_object_state_does_not_enter_partial_packet(pixel):
    port, _, _ = start(pixel)
    port.sim.hidden_carton_qpos = [999., -500., 100.]
    reading = port.observe_with_carton_absence('pixels only')
    assert 'world_from_box' not in reading and 'angles' not in reading
    assert reading['world_from_camera'] == np.eye(4).tolist()


@pytest.mark.parametrize('tag', [2, 4])
@pytest.mark.parametrize('partial', [False, True])
def test_rejected_decoded_housing_depth_is_fatal_in_both_observation_paths(pixel, tag, partial):
    port, control, _ = start(pixel)
    control.ids = {1, 20} if partial else {1, 10, 20}
    control.rejected = {tag: 'Insufficient valid aligned depth inside tag'}
    observe = port.observe_with_carton_absence if partial else port.observe
    with pytest.raises(ValueError, match='Decoded housing tag has invalid aligned depth'):
        observe('decoded housing is invalid, not absent')
    assert port._observation_failed and len(port.readings) == 1
    with pytest.raises(ValueError, match='before motion'):
        port.move_arms({}, 1., 'after invalid identity', None)
    with pytest.raises(ValueError, match='before motion'):
        port.set_grippers({}, 1., 'after invalid identity')
    assert control.motion_calls == []


@pytest.mark.parametrize('partial', [False, True])
def test_legitimately_absent_housing_tags_remain_allowed_after_startup(pixel, partial):
    port, control, _ = start(pixel)
    control.ids = {1, 20} if partial else {1, 10, 20}
    control.rejected = {}
    observe = port.observe_with_carton_absence if partial else port.observe
    reading = observe('housing tags outside current view')
    assert reading['seq'] == 2 and not port._observation_failed
    assert not [r for r in port.arm_tag_checks if r['seq'] == 2]


def test_secondary_failure_blocks_actual_pixelport_direct_motion(pixel, monkeypatch):
    from carton import folding_additional_view as view

    primary, control, _ = start(pixel)
    class RejectingAdditionalObserver:
        def __init__(self, anchor, *, additional_anchors, stationary_camera):
            self.anchors = {1: anchor, **additional_anchors}
        def observe(self, *args, **kwargs):
            raise ValueError('Rejected additional camera registration')

    monkeypatch.setattr(view, 'RGBDTagObserver', RejectingAdditionalObserver)
    monkeypatch.setattr(view, 'verify_additional_view_camera', lambda *args: {})
    configuration = view.AdditionalViewConfiguration('offline:secondary-test',
        'offline:fixed-clock', ('long_near', 'long_far'), allow_primary_carton_absence=True)
    wrapped = view.AdditionalViewPixelPort(primary, configuration=configuration)
    with pytest.raises(ValueError, match='Rejected additional camera registration'):
        wrapped.observe('secondary failure after real primary partial packet')
    assert primary.last_rgbd_frame.metadata['seq'] == 2
    assert wrapped.additional_rgbd_frame.metadata['seq'] == 2
    assert primary.last_rgbd_frame.clock_id == wrapped.additional_rgbd_frame.clock_id == configuration.clock_id
    assert primary._observation_failed
    with pytest.raises(ValueError, match='Pixel observation failed'):
        primary.observe('direct primary retry')
    with pytest.raises(ValueError, match='before motion'):
        primary.move_arms({}, 1., 'direct primary move', None)
    with pytest.raises(ValueError, match='before motion'):
        primary.set_grippers({}, 1., 'direct primary gripper')
    assert control.motion_calls == []


@pytest.mark.parametrize('kind', ['arms', 'grippers'])
@pytest.mark.parametrize('failure', ['exception', 'step_error', 'penetration'])
def test_command_refusal_latches_raw_pixelport_and_cannot_be_rewrapped(pixel, kind, failure):
    from carton import folding_additional_view as view

    primary, control, _ = start(pixel)
    def rejected_move(*args, **kwargs):
        control.motion_calls.append((args, kwargs))
        if failure == 'exception':
            raise ValueError('Injected original command failure')
        return dict(step_error='Injected step refusal' if failure == 'step_error' else None,
                    bad_penetration_mm=1.01 if failure == 'penetration' else 0.,
                    max_target_tracking_error_m=0.)
    primary.sim.move = rejected_move
    with pytest.raises(ValueError):
        if kind == 'arms': primary.move_arms({}, 1., 'original refused command', None)
        else: primary.set_grippers({}, 1., 'original refused command')
    assert primary._command_failed and primary._command_failure_reason
    assert not primary._observation_failed
    with pytest.raises(ValueError, match='before motion'):
        primary.move_arms({}, 1., 'retry arm', None)
    with pytest.raises(ValueError, match='before motion'):
        primary.set_grippers({}, 1., 'retry gripper')
    with pytest.raises(ValueError, match='Pixel command failed'):
        primary.observe('fresh observation cannot recover a command fault')
    configuration = view.AdditionalViewConfiguration('offline:command-test',
        'offline:fixed-clock', ('long_near', 'long_far'))
    with pytest.raises(ValueError, match='Failed primary command'):
        view.AdditionalViewPixelPort(primary, configuration=configuration)
    assert len(control.motion_calls) == 1 and primary.seq == 1


def test_primary_cache_copies_exact_exposed_frames_without_extra_render_or_relabel(pixel):
    primary, control = pixel
    received, renders = [], []
    observe, render = primary.observer.observe, primary.sim.render
    def spy_observe(rgb, depth, k, **kwargs):
        received.append((rgb.copy(), depth.copy(), k.copy(), kwargs.copy()))
        return observe(rgb, depth, k, **kwargs)
    def spy_render(*args, **kwargs):
        renders.append((args, kwargs))
        return render(*args, **kwargs)
    primary.observer.observe, primary.sim.render = spy_observe, spy_render
    primary.observe('startup')
    control.ids = {1, 20}
    primary.observe_with_carton_absence('current partial')
    assert len(renders) == 4 and len(received) == 2
    frame = primary.last_rgbd_frame.read_current(expected_seq=2, timestamp_s=2., clock_id='offline:simulation')
    for name, expected in zip(('rgb', 'exposed_depth', 'intrinsics'), received[-1][:3]):
        assert frame[name].dtype == expected.dtype and frame[name].tobytes() == expected.tobytes()
    with pytest.raises(ValueError, match='different_sequence'):
        primary.last_rgbd_frame.read_current(expected_seq=1, timestamp_s=2., clock_id='offline:simulation')
    control.anchor_error = 'Invalid current anchor'
    with pytest.raises(ValueError, match='Invalid current anchor'):
        primary.observe_with_carton_absence('current refusal retains current pixels')
    assert primary.last_rgbd_frame.metadata['seq'] == 3
