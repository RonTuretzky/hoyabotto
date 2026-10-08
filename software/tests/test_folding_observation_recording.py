"""Exact array round trips and current-view identity, no simulator required."""
import hashlib
import json

import numpy as np
import pytest

from carton import folding_observation_recording as recording


def frame(camera='station', seq=4, timestamp=3., clock='offline:simulation', dtype=np.float32):
    cache = recording.LastRGBDFrameCache(camera, clock_id=clock)
    rgb = np.arange(72, dtype=np.uint8).reshape(4, 6, 3)
    depth = np.arange(24, dtype=dtype).reshape(4, 6)/10
    depth[0, :3] = [np.nan, np.inf, -0.]
    k = np.array([[5., 0., 3.], [0., 5., 2.], [0., 0., 1.]], dtype=np.float64)
    cache.capture(rgb, depth, seq=seq, timestamp_s=timestamp, intrinsics=k)
    return cache, rgb, depth, k


def save(path, primary, additional=None, **kwargs):
    return recording.save_refusal_rgbd(path, caches={'primary': primary, 'additional': additional},
        expected_seq=4, timestamp_s=3., clock_id='offline:simulation',
        refusal='Fresh required flap missing', source_sha256={'producer.py': 'a'*64}, **kwargs)


@pytest.mark.parametrize('dtype', [np.float32, np.float64, np.dtype('>f4')])
def test_exact_current_two_view_roundtrip_preserves_dtype_bytes_and_hashes(tmp_path, dtype):
    primary, rgb, depth, k = frame(dtype=dtype)
    additional, _, _, _ = frame('front_left_back', dtype=dtype)
    manifest = save(tmp_path/'failure', primary, additional, assumptions_sha256='b'*64)
    assert manifest['run_status'] == 'refused'
    assert manifest['observation_status'] == 'not_inferred_from_run_refusal'
    assert manifest['all_requested_views_current']
    assert json.loads((tmp_path/'failure/manifest.json').read_text()) == manifest
    path = tmp_path/'failure/frames.npz'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest['archive']['sha256']
    with np.load(path, allow_pickle=False) as archive:
        for role in ('primary', 'additional'):
            for name, expected in [('rgb', rgb), ('exposed_depth', depth), ('intrinsics', k)]:
                actual = archive[role+'_'+name]
                assert actual.dtype == expected.dtype
                assert actual.tobytes() == expected.tobytes()  # Includes NaN, inf and signed zero.
                declared = manifest['views'][role]['metadata']['arrays'][name]
                assert declared['sha256'] == hashlib.sha256(actual.tobytes()).hexdigest()
                assert declared['shape'] == list(actual.shape)
                assert declared['dtype'] == actual.dtype.str


def test_cache_owns_one_frame_and_readers_cannot_modify_it():
    cache, rgb, depth, k = frame()
    originals = (rgb.copy(), depth.copy(), k.copy())
    rgb[:] = 0; depth[:] = 0; k[:] = 0
    first = cache.read_current(expected_seq=4, timestamp_s=3., clock_id='offline:simulation')
    for name, original in zip(('rgb', 'exposed_depth', 'intrinsics'), originals):
        assert first[name].tobytes() == original.tobytes()
        first[name][:] = 0
    first['metadata']['seq'] = 999
    assert cache.metadata['seq'] == 4
    again = cache.read_current(expected_seq=4, timestamp_s=3., clock_id='offline:simulation')
    assert again['rgb'].tobytes() == originals[0].tobytes()
    capacity = cache.nbytes
    for seq in range(5, 15):
        cache.clear()
        cache.capture(*originals[:2], seq=seq, timestamp_s=3., intrinsics=originals[2])
        assert cache.nbytes == capacity
    assert cache.metadata['seq'] == 14


def test_camera_and_clock_properties_cannot_be_relabelled():
    cache, *_ = frame()
    with pytest.raises(AttributeError): cache.camera = 'other'
    with pytest.raises(AttributeError): cache.clock_id = 'other'


@pytest.mark.parametrize('kind,reason', [('sequence', 'different_sequence'),
    ('timestamp', 'different_timestamp'), ('clock', 'different_clock'),
    ('cleared', 'not_captured_for_current_attempt'), ('missing', 'not_captured_for_current_attempt')])
def test_stale_or_missing_secondary_never_exports_old_arrays(tmp_path, kind, reason):
    primary, *_ = frame()
    additional, *_ = frame('front', seq=3 if kind == 'sequence' else 4,
        timestamp=2.999 if kind == 'timestamp' else 3.,
        clock='offline:other' if kind == 'clock' else 'offline:simulation')
    if kind == 'cleared': additional.clear()
    if kind == 'missing': additional = None
    manifest = save(tmp_path/kind, primary, additional)
    assert manifest['views']['additional']['status'] == 'unavailable'
    assert manifest['views']['additional']['reason'] == reason
    assert not manifest['all_requested_views_current']
    with np.load(tmp_path/kind/'frames.npz', allow_pickle=False) as archive:
        assert set(archive.files) == {'primary_rgb', 'primary_exposed_depth', 'primary_intrinsics'}


def test_clearing_at_attempt_start_prevents_same_time_old_frame_reuse(tmp_path):
    primary, *_ = frame(); additional, *_ = frame('front')
    primary.clear(); additional.clear()
    manifest = save(tmp_path/'early_failure', primary, additional)
    assert manifest['archive'] is None
    assert all(v['status'] == 'unavailable' for v in manifest['views'].values())
    assert list((tmp_path/'early_failure').iterdir()) == [tmp_path/'early_failure/manifest.json']


@pytest.mark.parametrize('seq,time', [(4, 3.), (3, 4.), (5, 2.99)])
def test_capture_refuses_stale_or_regressing_input_and_drops_previous_arrays(seq, time):
    cache, rgb, depth, k = frame(); cache.clear()
    with pytest.raises(ValueError, match='Stale capture'):
        cache.capture(rgb, depth, seq=seq, timestamp_s=time, intrinsics=k)
    assert cache.metadata is None and cache.nbytes == 0


@pytest.mark.parametrize('change', [dict(expected_seq=5), dict(timestamp_s=3.00001), dict(clock_id='offline:other')])
def test_current_reader_refuses_identity_mismatch(change):
    cache, *_ = frame()
    args = dict(expected_seq=4, timestamp_s=3., clock_id='offline:simulation'); args.update(change)
    with pytest.raises(ValueError, match='unavailable'): cache.read_current(**args)


def test_invalid_depth_alignment_is_preserved_for_diagnosis():
    cache, rgb, depth, k = frame()
    cache.capture(rgb, depth[:2], seq=5, timestamp_s=3., intrinsics=k)
    assert cache.metadata['aligned_shapes'] is False
    assert cache.read_current(expected_seq=5, timestamp_s=3., clock_id='offline:simulation')['exposed_depth'].shape == (2, 6)


def test_memory_bound_refuses_before_allocating_copy(monkeypatch):
    cache, rgb, depth, k = frame()
    monkeypatch.setattr(recording, 'MAX_FRAME_BYTES', 1)
    with pytest.raises(ValueError, match='capacity'):
        cache.capture(rgb, depth, seq=5, timestamp_s=3., intrinsics=k)
    assert cache.nbytes == 0


def test_existing_evidence_directory_is_never_overwritten(tmp_path):
    primary, *_ = frame(); path = tmp_path/'failure'; save(path, primary)
    before = {p.name: p.read_bytes() for p in path.iterdir()}
    with pytest.raises(FileExistsError): save(path, primary)
    assert {p.name: p.read_bytes() for p in path.iterdir()} == before


@pytest.mark.parametrize('changes', [dict(refusal=''), dict(expected_seq=True),
    dict(timestamp_s=float('nan')), dict(source_sha256={}),
    dict(source_sha256={'x': 'bad'}), dict(assumptions_sha256='bad'),
    dict(caches={'unknown': None}), dict(caches={'primary': object()})])
def test_invalid_refusal_manifest_is_rejected_without_creating_output(tmp_path, changes):
    path = tmp_path/'invalid'
    args = dict(caches={}, expected_seq=4, timestamp_s=3., clock_id='offline:simulation',
                refusal='refused', source_sha256={'producer.py': 'a'*64})
    args.update(changes)
    with pytest.raises(ValueError): recording.save_refusal_rgbd(path, **args)
    assert not path.exists()


def test_duplicate_camera_cannot_claim_two_independent_views(tmp_path):
    primary, *_ = frame(); additional, *_ = frame()
    with pytest.raises(ValueError, match='one camera identity'):
        save(tmp_path/'duplicate', primary, additional)
    assert not (tmp_path/'duplicate').exists()
