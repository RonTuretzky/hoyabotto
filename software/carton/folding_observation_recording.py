"""Bounded exact exposed-frame evidence, with no renderer or simulator access.

Call ``clear`` at the start of each observation attempt and ``capture`` after
the producer has applied noise/dropout, before interpretation. No calibration
transform is inferred or borrowed by this helper. Persistence records refusals
only, and never labels an older view as the current additional camera.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np


SCHEMA = 'offline_exact_exposed_rgbd_refusal/v1'
MAX_FRAME_BYTES = 64 * 1024 * 1024


def _identity(value, name):
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError(f'Explicit nonempty {name} required')
    return value


def _stamp(seq, timestamp_s):
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
        raise ValueError('Positive integer observation sequence required')
    if (isinstance(timestamp_s, bool) or not isinstance(timestamp_s, (float, int))
            or not math.isfinite(timestamp_s) or timestamp_s < 0):
        raise ValueError('Finite nonnegative observation time required')


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _array_description(array):
    return dict(dtype=array.dtype.str, shape=list(array.shape), nbytes=int(array.nbytes),
                sha256=hashlib.sha256(array.tobytes(order='C')).hexdigest())


class LastRGBDFrameCache:
    """One owned immutable array set per view; no growing image history.

    Public camera/clock properties are read-only. Clearing removes the arrays
    and retains only scalar monotonic counters; a new clock requires a fresh
    cache. Nonfinite depth and alignment failures are preserved as evidence,
    never repaired. Reading a current frame returns detached copies.
    """
    __slots__ = ('_camera', '_clock_id', '_frame', '_last_seq', '_last_time')

    def __init__(self, camera: str, *, clock_id: str):
        self._camera = _identity(camera, 'camera identity')
        self._clock_id = _identity(clock_id, 'clock identity')
        self._frame = None
        self._last_seq = 0
        self._last_time = -1.

    @property
    def camera(self):
        return self._camera

    @property
    def clock_id(self):
        return self._clock_id

    @property
    def metadata(self):
        return None if self._frame is None else copy.deepcopy(self._frame['metadata'])

    @property
    def nbytes(self):
        return 0 if self._frame is None else sum(self._frame[k].nbytes for k in ('rgb', 'exposed_depth', 'intrinsics'))

    def clear(self):
        self._frame = None

    def capture(self, rgb, exposed_depth, *, seq: int, timestamp_s: float, intrinsics):
        """Copy the exact already-exposed arrays, preserving dtype and values."""
        self.clear()
        _stamp(seq, timestamp_s)
        if seq <= self._last_seq or timestamp_s < self._last_time:
            raise ValueError('Stale capture sequence or regressing observation time')
        arrays = dict(rgb=np.asarray(rgb), exposed_depth=np.asarray(exposed_depth),
                      intrinsics=np.asarray(intrinsics))
        if arrays['rgb'].dtype != np.uint8 or arrays['rgb'].ndim != 3 or arrays['rgb'].shape[2] != 3:
            raise ValueError('Exact uint8 RGB image required for evidence')
        if arrays['exposed_depth'].ndim != 2 or arrays['exposed_depth'].dtype.kind not in 'fiu':
            raise ValueError('Real numeric two-dimensional exposed depth required')
        if arrays['intrinsics'].shape != (3, 3) or arrays['intrinsics'].dtype.kind not in 'fiu':
            raise ValueError('Numeric 3x3 declared intrinsics required')
        if any(not all(n > 0 for n in a.shape) for a in arrays.values()):
            raise ValueError('Nonempty exposed arrays required')
        if sum(a.nbytes for a in arrays.values()) > MAX_FRAME_BYTES:
            raise ValueError('Exposed frame exceeds bounded evidence cache capacity')
        arrays = {key: np.array(value, copy=True, order='C') for key, value in arrays.items()}
        for array in arrays.values():
            array.flags.writeable = False
        metadata = dict(camera=self.camera, clock_id=self.clock_id, seq=seq,
            timestamp_s=timestamp_s, rgb_timestamp_s=timestamp_s, depth_timestamp_s=timestamp_s,
            time_semantics='producer supplied common observation clock; no separate exposure timing inferred',
            calibration_status='no camera transform recorded or inferred by raw cache',
            depth_semantics='exact exposed array; units and alignment remain producer responsibilities',
            aligned_shapes=arrays['rgb'].shape[:2] == arrays['exposed_depth'].shape,
            arrays={key: _array_description(value) for key, value in arrays.items()})
        self._frame = dict(arrays, metadata=metadata)
        self._last_seq, self._last_time = seq, timestamp_s
        return copy.deepcopy(metadata)

    def _availability(self, expected_seq, timestamp_s, clock_id):
        _stamp(expected_seq, timestamp_s)
        _identity(clock_id, 'expected clock identity')
        if self._frame is None:
            return 'not_captured_for_current_attempt'
        meta = self._frame['metadata']
        if self.clock_id != clock_id:
            return 'different_clock'
        if meta['seq'] != expected_seq:
            return 'different_sequence'
        if meta['timestamp_s'] != timestamp_s:
            return 'different_timestamp'
        return None

    def read_current(self, *, expected_seq, timestamp_s, clock_id):
        """Return detached arrays only when all current-frame identities match."""
        reason = self._availability(expected_seq, timestamp_s, clock_id)
        if reason:
            raise ValueError('Current exposed frame unavailable: '+reason)
        return dict(metadata=self.metadata,
                    **{key: self._frame[key].copy() for key in ('rgb', 'exposed_depth', 'intrinsics')})


def save_refusal_rgbd(directory, *, caches: Mapping[str, LastRGBDFrameCache | None],
                      expected_seq: int, timestamp_s: float, clock_id: str,
                      refusal: str, source_sha256: Mapping[str, str],
                      assumptions_sha256: str | None = None):
    """Write one NPZ and manifest from current caches into a fresh directory.

    Missing/stale views have explicit unavailable entries and no exported
    arrays. No rendering, RNG, physics, registration or recovery occurs here.
    Existing destinations are refused to preserve earlier evidence.
    """
    _stamp(expected_seq, timestamp_s)
    _identity(clock_id, 'expected clock identity')
    _identity(refusal, 'refusal reason')
    if not isinstance(caches, Mapping) or set(caches) - {'primary', 'additional'}:
        raise ValueError('Only named primary/additional frame caches are supported')
    if (not isinstance(source_sha256, Mapping) or not source_sha256
            or any(not isinstance(k, str) or not k or not _digest(v) for k, v in source_sha256.items())):
        raise ValueError('Nonempty source hash manifest required')
    if assumptions_sha256 is not None and not _digest(assumptions_sha256):
        raise ValueError('Valid optional assumptions hash required')
    arrays, views = {}, {}
    for role in ('primary', 'additional'):
        cache = caches.get(role)
        if cache is not None and not isinstance(cache, LastRGBDFrameCache):
            raise ValueError('Explicit bounded frame cache or unavailable None required')
        reason = ('not_captured_for_current_attempt' if cache is None else
                  cache._availability(expected_seq, timestamp_s, clock_id))
        if reason:
            views[role] = dict(status='unavailable', reason=reason,
                               cached_metadata=None if cache is None else cache.metadata)
            continue
        frame = cache.read_current(expected_seq=expected_seq, timestamp_s=timestamp_s, clock_id=clock_id)
        keys = {name: role+'_'+name for name in ('rgb', 'exposed_depth', 'intrinsics')}
        views[role] = dict(status='captured_current', metadata=frame['metadata'], array_keys=keys)
        arrays.update({key: frame[name] for name, key in keys.items()})
    selected = [v['metadata']['camera'] for v in views.values() if v['status'] == 'captured_current']
    if len(selected) != len(set(selected)):
        raise ValueError('Independent primary/additional roles cannot reuse one camera identity')
    manifest = dict(schema=SCHEMA, simulation_only=True, hardware_commands=False,
        run_status='refused', observation_status='not_inferred_from_run_refusal',
        refusal=refusal, expected_seq=expected_seq,
        timestamp_s=timestamp_s, clock_id=clock_id, views=views,
        all_requested_views_current=all(v['status'] == 'captured_current' for v in views.values()),
        source_sha256=dict(source_sha256), assumptions_sha256=assumptions_sha256,
        archive=None, no_new_render=True, no_new_noise=True, no_physics_step=True)
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=False)
    if arrays:
        path = destination/'frames.npz'
        with path.open('xb') as stream:
            np.savez_compressed(stream, **arrays)
        manifest['archive'] = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                   nbytes=path.stat().st_size)
    with (destination/'manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    return manifest
