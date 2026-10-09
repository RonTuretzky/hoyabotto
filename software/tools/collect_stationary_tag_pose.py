"""Capture one already-settled pose; never position, enable, STOP or renew a lease.

Five independently bracketed OAK observations, one second apart by default,
measure repeatability at ONE pose. Phone/wrist views are not metric inputs.
This tool neither fits nor installs calibration. Imports and --help are offline.
Run the API/CLI in its own main-thread process for a bounded capture deadline.
"""
from __future__ import annotations

import argparse
import copy
from contextlib import contextmanager
from dataclasses import asdict
import importlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

from carton.servo.common import Limits, Refused, atomic_json, digest
from carton.servo.gemma import GemmaTagObserver, GemmaTransport, result
from farm.kinematics.lerobot import transform
from farm.perception.gemma_tags import TagRobot
from farm.perception.tag_sampling import ARM_JOINTS, HEAD_JOINTS, gripper_tag_for_arm

MAX_CALLS = 1024
MAX_RESPONSE_BYTES = 8 * 1024 * 1024
MAX_EVIDENCE_BYTES = 64 * 1024 * 1024
DWELL_POLL_S = 0.25


class CaptureDeadline(BaseException):
    """Not swallowed by a client's ordinary network-error fallback handler."""


@contextmanager
def _hard_deadline(seconds):
    # A future cannot cancel blocking I/O; do not leave a worker touching a robot
    # after reporting a timeout. POSIX main-thread execution is deliberate.
    if threading.current_thread() is not threading.main_thread() or not hasattr(signal, 'setitimer'):
        raise Refused('Run capture in a POSIX main-thread process for a hard deadline')
    if signal.getitimer(signal.ITIMER_REAL) != (0.0, 0.0):
        raise Refused('An existing process timer must not be replaced')
    previous = signal.getsignal(signal.SIGALRM)

    def expired(*_):
        raise CaptureDeadline('Stationary capture hard time budget exhausted')

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


class ReadOnlyRobot:
    """Positive allowlist at the raw Robot boundary, including exact arguments.

    No attribute forwarding, raw get(), config, restart, revive, enable, move,
    STOP, or lease call is exposed. All raw responses are journaled before any
    telemetry validation, so a refused observer leaves its original evidence.
    """

    def __init__(self, robot, *, arm='right', journal=None, check=lambda: None):
        self.__robot = robot
        self.arm = arm
        self.tag_id = gripper_tag_for_arm(arm)
        self.journal = journal
        self.check = check
        self.calls = self.raw_calls = self.evidence_bytes = 0

    def _request(self, name, args, fn):
        self.check()
        if self.calls >= MAX_CALLS:
            raise Refused('Read-only call budget exhausted')
        self.calls += 1
        if name != 'decoded_robot_get_tags':
            self.raw_calls += 1
        record = {'name': name, 'arguments': copy.deepcopy(args)}
        path = self.journal / f'{self.calls:04d}.json' if self.journal else None
        if path:
            atomic_json(path, dict(record, status='REQUESTED'))
        try:
            payload = fn()
        except (Exception, CaptureDeadline, KeyboardInterrupt) as exc:
            if path:
                atomic_json(path, dict(record, status='FAILED', error=str(exc)))
            raise
        # Snapshot before later clients can mutate/reuse their response objects.
        encoded = json.dumps(payload, allow_nan=True).encode()
        if len(encoded) > MAX_RESPONSE_BYTES or self.evidence_bytes + len(encoded) > MAX_EVIDENCE_BYTES:
            if path:
                atomic_json(path, dict(record, status='RESOURCE_REJECTED', response_bytes=len(encoded),
                                       reason='Response exceeds bounded evidence storage'))
            raise Refused('Response/evidence byte budget exhausted')
        payload = json.loads(encoded)
        self.evidence_bytes += len(encoded)
        if path:
            # Nonfinite input is retained verbatim in the journal, then rejected
            # by validation. Normal accepted evidence uses strict atomic_json.
            data = json.dumps(dict(record, status='RECEIVED', response=payload), allow_nan=True).encode()
            temporary = path.with_suffix('.pending')
            with temporary.open('xb') as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            temporary.replace(path)
        self.check()
        return payload

    def call(self, name, args, request_id=None):
        if not isinstance(args, dict):
            raise Refused('Read-only calls require explicit argument dictionaries')
        wanted = {
            'robot_get_capabilities': {},
            'robot_get_execution': {},
            'robot_get_state': {'fresh': True},
            'robot_get_arm_pose': {'arm': self.arm},
            'robot_get_tags': {'cameras': ['oak'], 'tag_ids': [1, self.tag_id]},
            'robot_get_cameras': {'cameras': ['oak']},
        }
        if name not in wanted:
            raise Refused(f'Read-only allowlist rejects {name}')
        expected = wanted[name]
        extras = {'include_images'} if name == 'robot_get_tags' else {'revive'} if name == 'robot_get_cameras' else set()
        def same(actual, wanted):
            return (type(actual) is type(wanted) and actual == wanted
                    and (not isinstance(wanted, list)
                         or all(same(a, b) for a, b in zip(actual, wanted))))

        if set(args) - set(expected) - extras or any(not same(args.get(k), v) for k, v in expected.items()):
            raise Refused(f'Read-only allowlist rejects arguments for {name}')
        forwarded = copy.deepcopy(args)
        for key in extras:
            if key in args and args[key] is not False:
                raise Refused(f'Read-only capture requires {key}=False')
            forwarded[key] = False
        return self._request(name, forwarded, lambda: self.__robot.call(name, forwarded) if request_id is None
                             else self.__robot.call(name, forwarded, request_id=request_id))

    def catalog(self):
        catalog = self._request('catalog', {}, self.__robot.catalog)
        # TagRobot only needs these two schemas to choose native tags or local
        # decoding. Never expose a motion schema to the decorator.
        return {'tools': [t for t in catalog.get('tools', [])
                          if t.get('function', {}).get('name') in ('robot_get_tags', 'robot_get_cameras')]}


class _SettledTransport(GemmaTransport):
    """Tighten the existing validation, without changing its freshness limits."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, execute=False, **kwargs)
        self.complete_pose = self.owner_markers = None
        self.pose_reads = 0
        self.owner_reads = 0

    def _rows(self, rows):
        q = super()._rows(rows)
        expected = {f'{arm}_arm_{joint}' for arm in ('left', 'right') for joint in ARM_JOINTS}
        expected.update(HEAD_JOINTS)
        # The owner names the remaining two base motors; pin that complete set
        # on the first read rather than inventing names for other owner builds.
        if not expected <= q.keys():
            raise Refused('Missing complete both-arm/head telemetry')
        if self.complete_pose is None:
            self.complete_pose = q.copy()
        elif q != self.complete_pose:
            raise Refused('Complete motor pose changed during stationary dwell')
        self.pose_reads += 1
        return q

    def _execution(self):
        s = super()._execution()
        for key in ('accepted', 'completed', 'motor_writes', 'stop_count'):
            if type(s.get(key)) is not int or s[key] < 0:
                raise Refused(f'Missing unambiguous owner marker: {key}')
        markers = {k: copy.deepcopy(s.get(k)) for k in (
            'started', 'accepted', 'completed', 'motor_writes', 'stop_count',
            'last_stop', 'generation', 'owner_generation')}
        if self.owner_markers is not None and markers != self.owner_markers:
            raise Refused('Owner generation/write/STOP markers changed during dwell')
        self.owner_markers = markers
        self.owner_reads += 1
        return s


class _ObservedRobot:
    """Also retain locally decoded tags if the raw endpoint supplies images."""

    def __init__(self, robot, journal):
        self.robot, self.journal = robot, journal

    def call(self, name, args):
        if name == 'robot_get_tags':
            args = dict(args, include_images=False)
            return self.journal._request('decoded_robot_get_tags', args,
                                         lambda: self.robot.call(name, args))
        return self.robot.call(name, args)


def _geometry(robot, arm):
    payload = robot.call('robot_get_arm_pose', {'arm': arm})
    status = result(payload, 'robot_get_arm_pose')
    configuration = status.get('configuration') or {}
    cfg, assets = configuration.get('config') or {}, configuration.get('model_assets') or {}
    if (cfg.get('arm') != arm or cfg.get('mapping') != 'feetech_degrees_v1'
            or not isinstance(cfg.get('calibration_sha256'), str) or not cfg['calibration_sha256']
            or assets.get('verified') is not True or not assets.get('revision')):
        raise Refused('Need verified arm geometry, model revision and motor calibration provenance')
    return payload, digest(configuration)


def _metric_sample(observer, seen, reference_anchor):
    if observer.capture['sample'] is None:
        raise Refused(observer.capture['sample_rejection'] or 'Metric stationary sample rejected')
    sample = observer.capture['sample']  # produced by the unchanged stationary_sample
    row = observer.payload['result']['observations']['oak']
    for tags in (row['tags'], row['pose_3d']['tags']):
        for tag_id in (1, observer.gripper_tag_id):
            if sum(t.get('tag_id') == tag_id for t in tags) != 1:
                raise Refused('Duplicate or missing table/hand tag identity')
    hand = next(t for t in row['pose_3d']['tags'] if t['tag_id'] == observer.gripper_tag_id)
    if hand.get('orientation_ambiguous') is not False:
        raise Refused('Need explicitly unambiguous hand pose')
    transform(sample['camera_from_tag'])
    if not math.isfinite(sample['reprojection_rms_px']) or sample['reprojection_rms_px'] < 0:
        raise Refused('Invalid hand reprojection error')
    frame = sample['frame']
    sha = frame.get('sha256')
    if not isinstance(sha, str) or len(sha) != 64 or any(c not in '0123456789abcdef' for c in sha.lower()):
        raise Refused('Frame lacks original SHA256 identity')
    if sha.lower() in seen:
        raise Refused('Duplicate frame content is not an independent observation')
    for key in ('camera_calibration_sha256', 'tag_geometry_sha256'):
        if not isinstance(sample.get(key), str) or not sample[key]:
            raise Refused(f'Missing {key}')
    anchor = np.asarray(sample['anchor_center_camera_mm'], dtype=float)
    if anchor.shape != (3,) or not np.isfinite(anchor).all():
        raise Refused('Invalid metric table anchor')
    if reference_anchor is not None and np.linalg.norm(anchor-reference_anchor) > 5:
        raise Refused('Metric table anchor moved during dwell')
    return sample, anchor


def _variation(samples):
    if not samples:
        return {}
    poses = [transform(s['camera_from_tag']) for s in samples]
    xyz = np.asarray([p[:3, 3] for p in poses]) * 1000
    angles = [float(np.degrees(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3])-1)/2, -1, 1))))
              for i, a in enumerate(poses) for b in poses[:i]]
    return dict(seq=[s['frame']['seq'] for s in samples],
                position_mean_mm=xyz.mean(axis=0).tolist(), position_std_mm=xyz.std(axis=0).tolist(),
                position_peak_to_peak_mm=np.ptp(xyz, axis=0).tolist(),
                pairwise_position_mm_max=max((float(np.linalg.norm(a-b)) for i, a in enumerate(xyz) for b in xyz[:i]), default=0.),
                pairwise_rotation_deg_max=max(angles, default=0.),
                pairwise_rotation_deg_median=float(np.median(angles)) if angles else 0.,
                reprojection_rms_px=[s['reprojection_rms_px'] for s in samples],
                variation_scope='Observed tag-estimation repeatability at one unchanged encoder pose; not accuracy')


def collect_stationary_pose(raw_robot, out, *, arm, samples=5, interval_s=1., max_seconds=30.,
                            clock=time.time, monotonic=time.monotonic, sleep=time.sleep,
                            tag_robot_factory=None, geometry=None):
    """Return report, preserving failed evidence; invalid options raise before I/O.

    The caller must already have a released, idle owner and manually settled
    pose. No asynchronous view is retimestamped. Each OAK bracket stays subject
    to the existing one-second freshness / three-second bracket validation.
    `out` must be a new directory. A failed bundle is never fit-ready, even if
    it contains earlier individually valid observations.
    """
    gripper_tag_for_arm(arm)
    if type(samples) is not int or not 1 <= samples <= 30:
        raise ValueError('samples must be an integer from 1 to 30')
    for label, value, lo, hi in (('interval_s', interval_s, 1, 30), ('max_seconds', max_seconds, .01, 300)):
        if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{label} must be finite and within {lo}..{hi}')
    if (samples-1)*interval_s >= max_seconds:
        raise ValueError('Dwell spacing exceeds the total time budget')
    limits = Limits(max_seconds=max_seconds)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    began = monotonic()
    last_mono = began

    def check():
        nonlocal last_mono
        now = monotonic()
        if not math.isfinite(now) or now < last_mono or now-began >= max_seconds:
            raise Refused('Stationary capture time budget exhausted or clock went backwards')
        last_mono = now

    raw = ReadOnlyRobot(raw_robot, arm=arm, journal=out/'calls', check=check)
    transport = None
    accepted, seen, anchor = [], set(), None
    report = dict(schema=1, status='REJECTED', arm=arm, metric_camera='oak', requested_samples=samples,
                  interval_s=interval_s, motor_writes=0, stop_sent=False, lease_renewed=False,
                  physical_mapping_validated=False, calibration_ready=False,
                  required_diverse_poses={'train': 8, 'validation': 3}, output=str(out))
    atomic_json(out/'invocation.json', dict(report, max_seconds=max_seconds, limits=asdict(limits),
                                          max_calls=MAX_CALLS, max_response_bytes=MAX_RESPONSE_BYTES,
                                          max_evidence_bytes=MAX_EVIDENCE_BYTES))
    folder = None
    try:
        with _hard_deadline(max_seconds):
            decorated = tag_robot_factory(raw) if tag_robot_factory else TagRobot(raw, geometry=geometry)
            robot = _ObservedRobot(decorated, raw)
            transport = _SettledTransport(robot, arm, ['wrist_roll'], limits=limits, clock=clock)
            transport.preflight()
            geometry_before, geometry_hash = _geometry(robot, arm)
            atomic_json(out/'arm-geometry-status.json', geometry_before)
            observer = GemmaTagObserver(robot, transport, camera='oak', clock=clock)
            next_sample = monotonic()
            for index in range(samples):
                # Check markers throughout a long dwell, with bounded read count.
                # Neither deadline nor observed_at is renewed by waiting.
                while monotonic() < next_sample:
                    check()
                    transport.status()
                    remaining = next_sample-monotonic()
                    if remaining > 0:
                        before_sleep = monotonic()
                        sleep(min(DWELL_POLL_S, remaining))
                        if monotonic() <= before_sleep:
                            raise Refused('Dwell clock did not advance')
                check()
                folder = out/f'sample-{index:02d}'
                folder.mkdir()
                owner_before, _ = transport.status()
                atomic_json(folder/'owner-before.json', owner_before)
                observer.observe(after=observer.last['captured_at'] if observer.last else 0.)
                # Save the metric rejection too: observer permits corners-only
                # control, but this collector requires a usable 3-D sample.
                for key in ('before', 'after'):
                    atomic_json(folder/f'{key}.json', observer.capture[key])
                atomic_json(folder/'observation.json', {k: v for k, v in observer.payload.items() if k != 'images'})
                atomic_json(folder/'candidate-sample.json', observer.capture['sample'])
                sample, candidate_anchor = _metric_sample(observer, seen, anchor)
                owner_after, _ = transport.status()
                atomic_json(folder/'owner-after.json', owner_after)
                geometry_after, after_hash = _geometry(robot, arm)
                atomic_json(folder/'arm-geometry-status.json', geometry_after)
                if after_hash != geometry_hash:
                    raise Refused('Arm geometry/model/motor calibration changed during dwell')
                transport.status()
                capture = dict(observer.capture, arm_geometry_status=geometry_after)
                capture['sample'] = dict(sample, settled_pose_bundle=out.name, split=None)
                atomic_json(folder/'capture.json', capture)
                atomic_json(folder/'sample.json', capture['sample'])
                accepted.append(capture['sample'])
                seen.add(sample['frame']['sha256'].lower())
                if anchor is None:
                    anchor = candidate_anchor
                # Wait a full interval AFTER this bracket completed. Independent
                # brackets are intentionally not one long freshness exception.
                next_sample = monotonic() + interval_s
                folder = None
            final_owner, _ = transport.status()
            atomic_json(out/'final-execution.json', final_owner)
            check()
            report['status'] = 'SETTLED_POSE_CAPTURED'
    except (Exception, CaptureDeadline, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
        failure = dict(reason=report['error'], exception=type(exc).__name__, automatic_retry=False,
                       motor_writes=0, stop_sent=False)
        atomic_json(out/'failure.json', failure)
        if folder:
            atomic_json(folder/'sample-rejected.json', failure)
    # Deliberately no transport.finish(): this tool can never own a release or STOP.
    report.update(accepted_samples=len(accepted), distinct_calibration_poses=int(report['status'] == 'SETTLED_POSE_CAPTURED'),
                  encoder_same=bool(transport and transport.pose_reads and report['status'] == 'SETTLED_POSE_CAPTURED'),
                  elapsed_s=monotonic()-began, raw_calls=raw.raw_calls, journaled_calls=raw.calls,
                  response_bytes=raw.evidence_bytes,
                  complete_motor_pose=transport.complete_pose if transport else None,
                  owner_markers=transport.owner_markers if transport else None,
                  owner_markers_unchanged=report['status'] == 'SETTLED_POSE_CAPTURED',
                  raw_calibration_ranges_sha256=digest(transport.raw_ranges) if transport and transport.raw_ranges else None,
                  commandable_ranges_sha256=digest(transport.ranges) if transport and transport.ranges else None,
                  complete_pose_reads=transport.pose_reads if transport else 0,
                  owner_marker_reads=transport.owner_reads if transport else 0,
                  stationary_scope='Exact equality at every complete telemetry read; owner markers pinned throughout dwell',
                  **_variation(accepted))
    atomic_json(out/'report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot-root', type=Path, required=True)
    parser.add_argument('--geometry', type=Path, required=True, help='Read existing tag geometry only; never installed')
    parser.add_argument('--arm', choices=('left', 'right'), required=True)
    parser.add_argument('--out', type=Path, required=True, help='Unique new evidence directory')
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--interval-s', type=float, default=1.)
    parser.add_argument('--max-seconds', type=float, default=30.)
    args = parser.parse_args(argv)
    # No client import, credential loading or discovery happens during import or --help.
    sys.path.insert(0, str(args.pilot_root.resolve()))
    raw = importlib.import_module('chat_server').Robot(args.pilot_root/'.private/robot.json')
    report = collect_stationary_pose(raw, args.out, arm=args.arm, samples=args.samples,
                                     interval_s=args.interval_s, max_seconds=args.max_seconds, geometry=args.geometry)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['status'] == 'SETTLED_POSE_CAPTURED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
