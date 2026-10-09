"""API-level fake owner tests; no real cameras, serial devices or motion."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton.servo.common import Limits, Refused, digest
from carton.servo.gemma import FRAME_READ_ATTEMPTS, GemmaTagObserver, GemmaTransport
from carton.servo.tag_calibration import motion_lock, readiness, registration_offsets, run_calibration
from farm.perception.gemma_calibration import CalibrationRobot
from farm.perception.tag_sampling import ARM_JOINTS
from tools.install_gemma_calibration import patch_source, IMPORT, ANCHOR, BEFORE, AFTER


class Owner:
    def __init__(self, arm='right'):
        self.arm = arm
        self.tag_id = 2 if arm == 'right' else 4
        self.now = 1000.
        self.started = 900.
        self.names = [f'{arm}_arm_{n}' for arm in ('left', 'right') for n in ARM_JOINTS]
        self.names += ['head_motor_1', 'head_motor_2', 'base_left_wheel', 'base_right_wheel']
        self.q = dict.fromkeys(self.names, 2000)
        self.enabled = set()
        self.ranges = {n: dict(min_ticks=1000, max_ticks=3000, margin_ticks=4) for n in self.names}
        self.command = self.writes = self.seq = 0
        self.calls = []
        self.stopped = self.no_motion = self.missing_tag = self.ambiguous = False
        self.after_move = None
        self.phase = 'idle'
        self.lease = 15
        self.mount = dict(arm=arm, body='fixed_gripper_housing', source='fixture confirmation')
        self.config = Path('/tmp/fake-pilot/.private/robot.json')

    def clock(self):
        return self.now

    def catalog(self):
        return {'tools': []}

    def pose(self):
        a, b = (self.q[f'{self.arm}_arm_{n}']-2000 for n in ('shoulder_pan', 'wrist_flex'))
        out = np.eye(4)
        out[:3, :3] = cv2.Rodrigues(np.array([0., b, 0.])*np.pi/2048)[0] @ cv2.Rodrigues(np.array([0., 0., a])*np.pi/2048)[0]
        out[:3, 3] = [.2+a*.0003, .02+b*.0003, .3]
        return out

    def ack(self, readbacks):
        return dict(accepted=True, completed=True, command_id=self.command,
                    owner_started=self.started, owner_status_time=self.now, readbacks=readbacks)

    def rows(self):
        return {n: dict(Present_Position=q, Present_Velocity=0, Present_Load=0, Moving=0, Status=0,
                        Torque_Enable=int(n in self.enabled), captured_at=self.now-.001) for n, q in self.q.items()}

    def call(self, name, args, request_id=None):
        self.now += .02
        self.calls.append((name, copy.deepcopy(args)))
        if name == 'robot_get_capabilities':
            r = dict(motion_units='encoder_ticks; positioning-joint degrees use4095 ticks/rev',
                     motion_ready=not self.stopped, joint_blockers={}, blockers=[])
        elif name == 'robot_get_execution':
            # As the qwen-bridge API serves it: the owner's status.json, including its own telemetry rows.
            r = dict(time=self.now, started=self.started, ok=True, control_mode='direct_joint', hardware_server=True,
                     phase='stopped' if self.stopped else ('holding' if self.enabled and self.phase == 'idle' else self.phase), operator_armed=not self.stopped,
                     stop_latched=self.stopped, enabled_motors=sorted(self.enabled), accepted=self.command,
                     completed=self.command, motor_writes=self.writes, lease_remaining=self.lease,
                     stop_count=int(self.stopped), rows=self.rows(), status_age_s=.001)
        elif name == 'robot_get_state':
            r = dict(cached=False, time=self.now, commandable_ranges=copy.deepcopy(self.ranges),
                     raw_calibration_ranges=copy.deepcopy(self.ranges),
                     motors=[dict(name=n, **row) for n, row in self.rows().items()])
        elif name == 'robot_get_arm_pose':
            r = {'status': 'CANDIDATE', 'configuration': {'config': {'arm': self.arm,
                 'mapping': 'feetech_degrees_v1', 'calibration_sha256': 'motors'}}}
        elif name == 'robot_get_tags':
            self.seq += 1
            a, b = (self.q[f'{self.arm}_arm_{n}']-2000 for n in ('shoulder_pan', 'wrist_flex'))
            corners = (np.array([[300, 220], [340, 220], [340, 260], [300, 260]], float)
                       + np.array([a*.3, b*.3]))
            table = [[100, 100], [140, 100], [140, 140], [100, 140]]
            tags = [dict(tag_id=1, status='DETECTED', corners_px=table),
                    dict(tag_id=self.tag_id, status='DETECTED', corners_px=corners.tolist())]
            if self.missing_tag:
                tags.pop()
            row = dict(frame=dict(camera_id='oak-test', stream_id='one', seq=self.seq,
                sha256=digest(self.seq), captured_at=self.now, timestamp_basis='capture'), image_size_px=[640, 480],
                tags=tags, pose_3d=dict(status='CAMERA_RELATIVE_ESTIMATE', calibration_sha256='K',
                geometry_config_sha256='geometry', tags=[dict(tag_id=1, center_camera_mm=[0, 0, 600]),
                dict(tag_id=self.tag_id, center_camera_mm=[0, 0, 300], mount=self.mount, camera_from_tag=self.pose().tolist(),
                     orientation_ambiguous=self.ambiguous, reprojection_rms_px=.1)]))
            r = {'observations': {'oak': row}}
        elif name == 'robot_set_motor_enable':
            self.command += 1
            self.writes += 1
            if args['enabled']:
                self.enabled.update(args['names'])
            else:
                self.enabled.difference_update(args['names'])
            r = self.ack({n: int(args['enabled']) for n in args['names']})
        elif name == 'robot_move_motor_targets':
            self.command += 1
            self.writes += 1
            assert not self.stopped
            assert set(args['positions']) <= self.enabled
            if not self.no_motion:
                self.q.update(args['positions'])
            r = self.ack({n: self.q[n] for n in args['positions']})
            if self.after_move:
                self.after_move(self)
        elif name == 'robot_stop':
            self.enabled.clear()
            self.stopped = True
            r = dict(release_confirmed=True)
        else:
            raise AssertionError(name)
        return {'ok': True, 'result': r}


@pytest.fixture
def rig(tmp_path):
    return Owner(), dict(schema=1, arm='right', joints=['shoulder_pan', 'wrist_flex'], camera='oak',
                         lock_file=str(tmp_path/'motion.lock'), model_directory='fake-model')


def test_readiness_reports_visibility_without_any_write(rig):
    owner, cfg = rig
    r = readiness(owner, cfg, clock=owner.clock)
    assert r['status'] == 'READY_FOR_LOCAL_PROBES'
    assert r['detected_tag_ids'] == [1, 2] and r['tag_2_border_clearance_px'] > 0
    assert not owner.writes
    assert all(n.startswith('robot_get_') for n, _ in owner.calls)
    assert r['gripper_tag_id'] == 2
    assert r['gripper_tag_mount'] == r['tag_2_mount']
    assert r['gripper_tag_border_clearance_px'] == r['tag_2_border_clearance_px']


def test_left_tag_readiness_and_local_probe_use_only_left_binding(rig, tmp_path):
    _, cfg = rig
    owner = Owner('left')
    cfg.update(arm='left', gripper_tag_id=4)
    report = readiness(owner, cfg, clock=owner.clock)
    assert report['status'] == 'READY_FOR_LOCAL_PROBES'
    assert report['gripper_tag_id'] == 4 and report['tag_4_mount']['arm'] == 'left'
    assert 'tag_2_mount' not in report
    outcome = run_calibration(owner, cfg, 'local_model', tmp_path/'left', clock=owner.clock)
    assert outcome['status'] == 'LOCAL_MODEL_VALIDATED' and outcome['gripper_tag_id'] == 4
    assert outcome['cleanup']['release_confirmed'] and not owner.enabled
    assert all(args['tag_ids'] == [1, 4] for name, args in owner.calls if name == 'robot_get_tags')
    enabled = [args for name, args in owner.calls if name == 'robot_set_motor_enable' and args['enabled']]
    # The pickup-profile owner moves an arm only with all six of its motors enabled; the jaw just holds.
    assert enabled and enabled[0]['names'] == [f'left_arm_{n}' for n in ARM_JOINTS]
    assert all(not n.endswith('gripper') for name, args in owner.calls if name == 'robot_move_motor_targets' for n in args['positions'])
    sample = json.loads((tmp_path/'left/baseline/sample.json').read_text())
    assert sample['arm'] == 'left' and sample['gripper_tag_id'] == 4


@pytest.mark.parametrize('arm,tag', [('right', 4), ('left', 2), ('right', True)])
def test_wrong_explicit_tag_refused_before_any_owner_call(rig, tmp_path, arm, tag):
    owner, cfg = rig
    cfg.update(arm=arm, gripper_tag_id=tag)
    with pytest.raises(ValueError, match='requires gripper tag'):
        run_calibration(owner, cfg, 'local_model', tmp_path/'wrong-tag', clock=owner.clock)
    assert owner.calls == []


def test_left_tag_with_right_mount_never_enables(rig, tmp_path):
    _, cfg = rig
    owner = Owner('left')
    owner.mount['arm'] = 'right'
    cfg.update(arm='left', gripper_tag_id=4)
    with pytest.raises(Refused, match='mounting'):
        run_calibration(owner, cfg, 'local_model', tmp_path/'wrong-mount', clock=owner.clock)
    assert owner.writes == 0 and not owner.stopped


def test_existing_experiment_runs_bidirectional_probes_and_independent_holdouts(rig, tmp_path):
    owner, cfg = rig
    r = run_calibration(owner, cfg, 'local_model', tmp_path/'result', clock=owner.clock)
    assert r['status'] == 'LOCAL_MODEL_VALIDATED' and len(r['holdout']) == 2
    assert r['calibration_commands_sent'] == 12
    assert r['cleanup']['release_confirmed'] and not owner.enabled and not owner.stopped
    assert set(owner.q.values()) == {2000}
    enable = [a for n, a in owner.calls if n == 'robot_set_motor_enable']
    assert len(enable) == 2 and enable[0]['names'] == enable[1]['names'] == [f'right_arm_{n}' for n in ARM_JOINTS]
    moves = [args['positions'] for name, args in owner.calls if name == 'robot_move_motor_targets']
    assert moves and all(len(p) == 1 and not next(iter(p)).endswith('gripper') for p in moves)


@pytest.mark.parametrize('failure', ['no_motion', 'stop', 'restart', 'tag_loss', 'foreign_write', 'lease'])
def test_faults_abort_and_do_not_retry_motion(rig, tmp_path, failure):
    owner, cfg = rig
    if failure == 'no_motion':
        owner.no_motion = True
    elif failure == 'lease':
        owner.lease = 1
    else:
        def after(o):
            if failure == 'stop':o.stopped = True
            if failure == 'restart':o.started += 1
            if failure == 'tag_loss':o.missing_tag = True
            if failure == 'foreign_write':o.command += 1; o.writes += 1
        owner.after_move = after
    with pytest.raises(Refused):
        run_calibration(owner, cfg, 'local_model', tmp_path/'failed', clock=owner.clock)
    commands = [n for n, _ in owner.calls if n == 'robot_move_motor_targets']
    assert len(commands) <= 1
    assert owner.stopped and not owner.enabled
    report = json.loads((tmp_path/'failed/failure.json').read_text())
    assert report['automatic_retry'] is False


@pytest.mark.parametrize('bad', ['arm', 'mount', 'range', 'busy', 'enabled', 'limit', 'jaw'])
def test_invalid_start_never_enables_or_stops_someone_elses_owner(rig, tmp_path, bad):
    owner, cfg = rig
    if bad == 'arm':cfg['arm'] = 'left'
    if bad == 'mount':owner.mount['body'] = 'moving_jaw'
    if bad == 'range':owner.q['right_arm_elbow_flex'] = 3155
    if bad == 'busy':owner.phase = 'moving'
    if bad == 'enabled':owner.enabled.add('left_arm_gripper')
    if bad == 'limit':cfg['limits'] = {'trust_ticks': 300}
    if bad == 'jaw':cfg['joints'] = ['gripper']
    with pytest.raises(Refused):
        run_calibration(owner, cfg, 'local_model', tmp_path/'blocked', clock=owner.clock)
    assert not owner.writes
    assert not any(n == 'robot_stop' for n, _ in owner.calls)


def test_ambiguous_orientation_allows_pixels_but_blocks_registration_before_enable(rig, tmp_path):
    owner, cfg = rig
    owner.ambiguous = True
    with pytest.raises(Refused, match='baseline registration'):
        run_calibration(owner, cfg, 'registration', tmp_path/'reg', clock=owner.clock)
    assert not owner.writes
    r = run_calibration(owner, cfg, 'local_model', tmp_path/'pixels', clock=owner.clock)
    assert r['status'] == 'LOCAL_MODEL_VALIDATED'


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_registration_automatically_collects_and_fits_held_out_poses(rig, tmp_path, monkeypatch, arm):
    _, cfg = rig
    owner = Owner(arm)
    cfg.update(arm=arm)
    # Substitute only encoder-to-FK mapping, retaining sampler, API movement
    # adapter and OpenCV fitter. This is not a physical/collision simulation.
    def assemble(captures, directory):
        samples = []
        binding = None
        for capture in captures:
            s = copy.deepcopy(capture['sample'])
            a, b = (s['joint_ticks'][f'{arm}_arm_{n}']-2000 for n in ('shoulder_pan', 'wrist_flex'))
            pose = np.eye(4)
            pose[:3, :3] = cv2.Rodrigues(np.array([0., b, 0.])*np.pi/2048)[0] @ cv2.Rodrigues(np.array([0., 0., a])*np.pi/2048)[0]
            pose[:3, 3] = [.2+a*.0003, .02+b*.0003, .3]
            s['base_from_gripper'] = pose.tolist()
            samples.append(s)
            binding = dict(arm=arm, gripper_tag_id=owner.tag_id, gripper_tag_mount=owner.mount,
                           camera_id='oak-test', stream_id='one', camera_calibration_sha256='K',
                           tag_geometry_sha256='geometry', robot_model_sha256='model',
                           motor_calibration_sha256='motors', encoder_mapping_source='synthetic known FK')
        return dict(schema=1, samples=samples, binding=binding)
    monkeypatch.setattr('carton.servo.tag_calibration.assemble_dataset', assemble)
    from farm.kinematics.tag_registration import fit_registration
    def fit_after_release(dataset):
        assert not owner.enabled, 'Motors must be released before the offline hand-eye solve'
        return fit_registration(dataset)
    monkeypatch.setattr('carton.servo.tag_calibration.fit_registration', fit_after_release)
    r = run_calibration(owner, cfg, 'registration', tmp_path/'registration', clock=owner.clock)
    assert r['status'] == 'REGISTRATION_VALIDATED'
    assert r['residuals']['train']['count'] == 8 and r['residuals']['validation']['count'] == 3
    assert not r['motion_ready'] and r['cleanup']['release_confirmed']
    assert len(list((tmp_path/'registration').glob('pose-*.json'))) == 11
    assert set(owner.q.values()) == {2000} and not owner.enabled
    assert r['commanded_path_ticks'] <= 1500
    assert 'motor_writes' not in r and r['fitter_motor_writes'] == 0
    assert r['binding']['gripper_tag_id'] == owner.tag_id and r['binding']['arm'] == arm


def test_plan_cannot_exceed_budget_or_include_wrong_axis_count():
    assert registration_offsets(['a', 'b'], Limits())['planned_path_ticks'] == 1248
    with pytest.raises(Refused):registration_offsets(['a'], Limits())
    with pytest.raises(Refused):registration_offsets(['a', 'b'], Limits(max_path_ticks=1000))


def test_wrapper_shared_lock_blocks_mutations_but_not_stop_or_reads(rig, tmp_path):
    owner, cfg = rig
    path = tmp_path/'config.json'
    path.write_text(json.dumps(cfg))
    wrapped = CalibrationRobot(owner, path)
    assert {t['function']['name'] for t in wrapped.catalog()['tools']} == {'robot_calibration_status', 'robot_calibrate_tags', 'robot_get_registered_tags', 'robot_get_paddle_target'}
    with motion_lock(path.with_name('tag-calibration.lock')):
        with pytest.raises(Refused):wrapped.call('robot_set_motor_enable', {'names': [], 'enabled': True})
        assert wrapped.call('robot_get_state', {'fresh': True})['ok']
        assert wrapped.call('robot_stop', {})['ok']
    assert not wrapped.call('robot_calibrate_tags', {'mode': 'local_model', 'arm': 'left'})['ok']


def test_installer_is_narrow_idempotent_and_refuses_layout_drift():
    original = ANCHOR + '\ndef main():\n    ' + BEFORE + '\n'
    patched = patch_source(original)
    assert IMPORT in patched and AFTER in patched
    assert patch_source(patched) == patched
    with pytest.raises(ValueError):patch_source(original.replace(BEFORE, 'other()'))


@pytest.mark.parametrize('mutation', ['stale', 'future', 'repeated', 'stream', 'anchor', 'head', 'ranges', 'receipt'])
def test_observation_changes_cannot_feed_calibration(rig, mutation):
    owner, cfg = rig
    transport = GemmaTransport(owner, 'right', cfg['joints'], clock=owner.clock)
    transport.preflight()
    observer = GemmaTagObserver(owner, transport, clock=owner.clock)
    observer.observe()
    previous_seq = owner.seq
    original = owner.call
    def call(name, args, request_id=None):
        if mutation == 'head':owner.q['head_motor_1'] += 10
        if mutation == 'ranges':owner.ranges['right_arm_shoulder_pan']['min_ticks'] += 1
        payload = original(name, args, request_id)
        if name == 'robot_get_tags':
            row = payload['result']['observations']['oak']
            if mutation == 'stale':row['frame']['captured_at'] -= 10
            if mutation == 'future':row['frame']['captured_at'] += 10
            if mutation == 'repeated':row['frame']['seq'] = previous_seq
            if mutation == 'stream':row['frame']['stream_id'] = 'other'
            if mutation == 'receipt':row['frame']['timestamp_basis'] = 'receipt_only_capture_delay_unknown'
            if mutation == 'anchor':row['tags'][0]['corners_px'][0][0] += 10
        return payload
    owner.call = call
    with pytest.raises(Refused):observer.observe()
    assert not owner.writes


@pytest.fixture
def frame_rig(rig):
    owner, cfg = rig
    transport = GemmaTransport(owner, 'right', cfg['joints'], execute=True, clock=owner.clock)
    transport.preflight()
    return owner, transport, GemmaTagObserver(owner, transport, clock=owner.clock)


def cached_camera(owner, cached_reads=1, *, cached_row=None, mutate=None, state_hook=None):
    """Serve an actual prior fake capture, followed by new captures; record unchanged evidence.

    Hooks inject faults into the fake API only. None of these tests opens a camera or motor port.
    """
    original = owner.call
    cached = cached_row if cached_row is not None else original('robot_get_tags', {})['result']['observations']['oak']
    reads = {'tags': [], 'states': []}

    def call(name, args, request_id=None):
        payload = original(name, args, request_id)
        if name == 'robot_get_tags':
            index = len(reads['tags'])
            if cached_reads is None or index < cached_reads:
                payload['result']['observations']['oak'] = copy.deepcopy(cached)
            row = payload['result']['observations']['oak']
            if mutate:
                mutate(row, index)
            reads['tags'].append(copy.deepcopy(row))
        elif name == 'robot_get_state':
            if state_hook:
                state_hook(payload, len(reads['states']))
            reads['states'].append(copy.deepcopy(payload))
        return payload

    owner.call = call
    return reads


@pytest.mark.parametrize('cached_reads', [1, 3])
@pytest.mark.parametrize('already_observed', [False, True])
def test_cached_frames_wait_for_new_capture_inside_original_bracket(frame_rig, cached_reads, already_observed):
    owner, transport, observer = frame_rig
    if already_observed:
        observer.observe()
    cached_row = observer.payload['result']['observations']['oak'] if already_observed else None
    reads = cached_camera(owner, cached_reads, cached_row=cached_row)
    after = owner.now
    observation = observer.observe(after=after)
    capture = observer.capture
    earliest = max(r['captured_at'] for r in reads['states'][0]['result']['motors'])
    assert all(row['frame']['captured_at'] < earliest for row in reads['tags'][:-1])
    assert capture['before'] == reads['states'][0] and capture['after'] == reads['states'][-1]
    assert capture['frame_reads'] == len(reads['tags']) == cached_reads + 1
    assert len(reads['states']) == cached_reads + 2
    assert earliest <= observation.captured_at <= min(r['captured_at'] for r in capture['after']['result']['motors'])
    assert observation.captured_at > after
    assert observer.last == reads['tags'][-1]['frame'] == capture['sample']['frame']
    assert observer.payload['result']['observations']['oak'] == reads['tags'][-1]
    assert transport.observed_at == observation.captured_at and transport.observed_q == owner.q
    assert capture['sample']['stationary_bracket_verified'] is True
    assert not owner.writes


def test_permanently_frozen_capture_exhausts_attempts_without_sliding_bracket(frame_rig):
    owner, transport, observer = frame_rig
    reads = cached_camera(owner, cached_reads=None)
    with pytest.raises(Refused, match='camera frame did not advance'):
        observer.observe()
    assert len(reads['tags']) == FRAME_READ_ATTEMPTS
    assert len({r['frame']['captured_at'] for r in reads['tags']}) == 1
    assert len(reads['states']) == FRAME_READ_ATTEMPTS + 1
    assert observer.count == 0 and observer.last is observer.capture is observer.identity is observer.anchor is None
    assert transport.observed_at is transport.observed_q is None and not owner.writes


def test_frozen_encoder_poll_refuses_without_retrying_camera(frame_rig):
    owner, _, observer = frame_rig
    def state_hook(payload, index):
        if index:
            for row in payload['result']['motors']:
                row['captured_at'] = reads['states'][0]['result']['motors'][0]['captured_at']
    reads = cached_camera(owner, cached_reads=0, state_hook=state_hook)
    with pytest.raises(Refused, match='stationary encoder bracket'):
        observer.observe()
    assert len(reads['tags']) == 1 and len(reads['states']) == 6
    assert observer.last is None and not owner.writes


@pytest.mark.parametrize('during_catchup', [False, True])
def test_movement_during_frame_wait_refuses_before_a_return_to_original_pose(frame_rig, during_catchup):
    owner, _, observer = frame_rig
    def state_hook(payload, index):
        rows = payload['result']['motors']
        if during_catchup and index == 1:
            for row in rows:
                row['captured_at'] -= .04  # a valid owner poll that still precedes the fresh frame
        if index == (2 if during_catchup else 1):
            next(r for r in rows if r['name'] == 'right_arm_shoulder_pan')['Present_Position'] += 4
        # Subsequent reads would return the original pose. They must never be reached.
    reads = cached_camera(owner, cached_reads=0 if during_catchup else 1, state_hook=state_hook)
    with pytest.raises(Refused, match='motor drift'):
        observer.observe()
    assert len(reads['tags']) == 1 and len(reads['states']) == (3 if during_catchup else 2)
    assert observer.last is None and not owner.writes


@pytest.mark.parametrize('change', ['stream', 'camera', 'geometry', 'intrinsics', 'resolution', 'mount', 'anchor'])
def test_identity_and_table_anchor_are_bound_to_first_cached_candidate(frame_rig, change):
    owner, _, observer = frame_rig
    def mutate(row, index):
        if index != 1:
            return
        if change == 'stream':row['frame']['stream_id'] = 'restarted'
        if change == 'camera':row['frame']['camera_id'] = 'other-camera'
        if change == 'geometry':row['pose_3d']['geometry_config_sha256'] = 'other-geometry'
        if change == 'intrinsics':row['pose_3d']['calibration_sha256'] = 'other-intrinsics'
        if change == 'resolution':row['image_size_px'] = [1280, 960]
        if change == 'mount':row['pose_3d']['tags'][1]['mount'] = dict(owner.mount, source='replacement fixture')
        if change == 'anchor':row['tags'][0]['corners_px'][0][0] += 3
    reads = cached_camera(owner, mutate=mutate)
    with pytest.raises(Refused, match='changed'):
        observer.observe()
    assert len(reads['tags']) == 2 and observer.last is None and not owner.writes


@pytest.mark.parametrize('bad', ['missing_tag', 'bad_mount', 'corners_shape', 'corners_nan', 'corners_text',
                                'time_nan', 'time_inf', 'time_bool', 'future', 'stale', 'sequence_bool',
                                'sequence_negative', 'sequence_missing', 'camera_missing', 'stream_missing', 'receipt'])
@pytest.mark.parametrize('candidate', [0, 1])
def test_invalid_cached_or_new_frames_are_never_retried(frame_rig, bad, candidate):
    owner, _, observer = frame_rig
    def mutate(row, index):
        if index != candidate:
            return
        frame = row['frame']
        if bad == 'missing_tag':row['tags'].pop()
        if bad == 'bad_mount':row['pose_3d']['tags'][1]['mount'] = dict(owner.mount, body='moving_jaw')
        if bad == 'corners_shape':row['tags'][1]['corners_px'] = [[1, 2]]
        if bad == 'corners_nan':row['tags'][1]['corners_px'][0][0] = float('nan')
        if bad == 'corners_text':row['tags'][1]['corners_px'][0][0] = 'invalid'
        if bad == 'time_nan':frame['captured_at'] = float('nan')
        if bad == 'time_inf':frame['captured_at'] = float('inf')
        if bad == 'time_bool':frame['captured_at'] = True
        if bad == 'future':frame['captured_at'] = owner.now + .001
        if bad == 'stale':frame['captured_at'] = owner.now - 10
        if bad == 'sequence_bool':frame['seq'] = True
        if bad == 'sequence_negative':frame['seq'] = -1
        if bad == 'sequence_missing':frame.pop('seq')
        if bad == 'camera_missing':frame.pop('camera_id')
        if bad == 'stream_missing':frame.pop('stream_id')
        if bad == 'receipt':frame['timestamp_basis'] = 'receipt'
    reads = cached_camera(owner, mutate=mutate)
    with pytest.raises(Refused):
        observer.observe()
    assert len(reads['tags']) == candidate + 1
    assert observer.last is None and not owner.writes


@pytest.mark.parametrize('bad', ['backwards_sequence', 'backwards_time', 'changed_time', 'changed_hash', 'frozen_time'])
def test_invalid_frame_progression_is_not_waited_out(frame_rig, bad):
    owner, _, observer = frame_rig
    def mutate(row, index):
        if index != 1:
            return
        frame = row['frame']
        if bad == 'backwards_sequence':frame['seq'] -= 1
        if bad == 'backwards_time':frame['captured_at'] -= .001
        if bad == 'changed_time':frame['captured_at'] += .001
        if bad == 'changed_hash':frame['sha256'] = 'changed-without-new-sequence'
        if bad == 'frozen_time':frame['seq'] += 1
    reads = cached_camera(owner, cached_reads=2, mutate=mutate)
    with pytest.raises(Refused, match='Camera frame'):
        observer.observe()
    assert len(reads['tags']) == 2 and observer.last is None and not owner.writes


def test_frame_inside_bracket_but_before_movement_cutoff_is_not_retried(frame_rig):
    owner, _, observer = frame_rig
    reads = cached_camera(owner, cached_reads=0)
    with pytest.raises(Refused, match='camera frame did not advance'):
        observer.observe(after=owner.now + .5)
    assert len(reads['tags']) == 1 and not owner.writes


@pytest.mark.parametrize('expiry', ['already_expired', 'tag_reply', 'encoder_reply', 'encoder_catchup'])
def test_frame_wait_deadline_prevents_further_api_reads(frame_rig, expiry):
    owner, transport, observer = frame_rig
    def state_hook(payload, index):
        if expiry == 'encoder_catchup' and index >= 1:
            for row in payload['result']['motors']:
                row['captured_at'] = reads['states'][0]['result']['motors'][0]['captured_at']
    reads = cached_camera(owner, cached_reads=0 if expiry == 'encoder_catchup' else 1,
                          state_hook=state_hook)
    # The observation must use the original deadline, not a fresh budget on each loop.
    budget = {'already_expired': -.001, 'tag_reply': .035, 'encoder_reply': .055, 'encoder_catchup': .075}[expiry]
    transport.deadline = owner.now + budget
    with pytest.raises(Refused, match='expired|budget'):
        observer.observe()
    assert len(reads['tags']) == (0 if expiry == 'already_expired' else 1)
    assert len(reads['states']) == {'already_expired': 0, 'tag_reply': 1, 'encoder_reply': 2, 'encoder_catchup': 3}[expiry]
    assert observer.last is None and not owner.writes


def test_frame_wait_also_has_a_local_deadline_with_calibration_budget_remaining(frame_rig):
    owner, transport, observer = frame_rig
    transport.limits = Limits(frame_age_s=.15)
    def state_hook(payload, index):
        if index == 1:
            owner.now += .2  # slow response; telemetry still satisfies status_age_s
    reads = cached_camera(owner, state_hook=state_hook)
    calls_before = len(owner.calls)
    with pytest.raises(Refused, match='frame wait expired'):
        observer.observe()
    assert owner.now < transport.deadline
    assert len(reads['tags']) == 1 and len(reads['states']) == 2
    assert not any(n == 'robot_get_execution' for n, _ in owner.calls[calls_before:])
    assert observer.last is None and not owner.writes


@pytest.mark.parametrize('failed_tool', ['robot_get_tags', 'robot_get_state', 'robot_get_execution'])
@pytest.mark.parametrize('timeout', [False, True])
def test_api_failure_during_cached_wait_propagates_without_retry(frame_rig, failed_tool, timeout):
    owner, _, observer = frame_rig
    reads = cached_camera(owner)
    original = owner.call
    def call(name, args, request_id=None):
        payload = original(name, args, request_id)
        if name == failed_tool and (name != 'robot_get_state' or len(reads['states']) == 2):
            if timeout:
                raise TimeoutError('fake owner timeout')
            return {'ok': False, 'result': {'error': 'fake hardware error'}}
        return payload
    owner.call = call
    with pytest.raises(TimeoutError if timeout else Refused):
        observer.observe()
    assert len(reads['tags']) == 1 and observer.last is None and not owner.writes


@pytest.mark.parametrize('fault', ['status', 'moving', 'velocity', 'load', 'enabled', 'ranges', 'raw_ranges'])
def test_encoder_fault_during_cached_frame_wait_is_not_retried(frame_rig, fault):
    owner, _, observer = frame_rig
    def state_hook(payload, index):
        if index != 1:
            return
        state = payload['result']
        row = next(r for r in state['motors'] if r['name'] == 'right_arm_shoulder_pan')
        if fault == 'status':row['Status'] = 8
        if fault == 'moving':row['Moving'] = 1
        if fault == 'velocity':row['Present_Velocity'] = 2
        if fault == 'load':row['Present_Load'] = 500
        if fault == 'enabled':row['Torque_Enable'] = 1
        if fault == 'ranges':state['commandable_ranges']['right_arm_shoulder_pan']['min_ticks'] += 1
        if fault == 'raw_ranges':state['raw_calibration_ranges']['right_arm_shoulder_pan']['min_ticks'] += 1
    reads = cached_camera(owner, state_hook=state_hook)
    with pytest.raises(Refused):
        observer.observe()
    assert len(reads['tags']) == 1 and len(reads['states']) == 2 and not owner.writes


@pytest.mark.parametrize('fault', ['restart', 'foreign_write', 'stop', 'owner_motion'])
def test_owner_change_during_cached_frame_wait_is_not_retried(frame_rig, fault):
    owner, _, observer = frame_rig
    def mutate(row, index):
        if index == 0:
            if fault == 'restart':owner.started += 1
            if fault == 'foreign_write':owner.command += 1; owner.writes += 1
            if fault == 'stop':owner.stopped = True
            if fault == 'owner_motion':owner.phase = 'moving'
    reads = cached_camera(owner, mutate=mutate)
    with pytest.raises(Refused):
        observer.observe()
    assert len(reads['tags']) == 1 and observer.last is None
    assert all(n.startswith('robot_get_') for n, _ in owner.calls)


def test_post_observation_pose_change_blocks_first_enable(rig):
    owner, cfg = rig
    transport = GemmaTransport(owner, 'right', cfg['joints'], execute=True, clock=owner.clock)
    transport.preflight()
    observer = GemmaTagObserver(owner, transport, clock=owner.clock)
    observer.observe()
    owner.q['right_arm_shoulder_pan'] += 20
    with pytest.raises(Refused, match='camera observation'):transport.enable()
    assert not owner.writes


def test_response_acceptance_without_completion_does_not_count_as_motion(rig, tmp_path):
    owner, cfg = rig
    original = owner.call
    def call(name, args, request_id=None):
        r = original(name, args, request_id)
        if name == 'robot_move_motor_targets':r['result']['completed'] = False
        return r
    owner.call = call
    with pytest.raises(Refused, match='completion'):
        run_calibration(owner, cfg, 'local_model', tmp_path/'rejected', clock=owner.clock)
    assert owner.stopped
    assert len([n for n, a in owner.calls if n == 'robot_move_motor_targets']) == 1


@pytest.mark.parametrize('outcome', ['settled_short', 'contact_halt', 'halted'])
def test_owner_closure_outcomes_without_completion_stop_and_name_the_outcome(rig, tmp_path, outcome):
    """Today's owner answers completed=false with a closure_outcome while still holding; never retried."""
    owner, cfg = rig
    original = owner.call
    def call(name, args, request_id=None):
        r = original(name, args, request_id)
        if name == 'robot_move_motor_targets':
            r['result'].update(completed=False, endpoint_reached=False, closure_outcome=outcome, holding=True)
        return r
    owner.call = call
    with pytest.raises(Refused, match=outcome):
        run_calibration(owner, cfg, 'local_model', tmp_path/outcome, clock=owner.clock)
    assert owner.stopped and len([n for n, a in owner.calls if n == 'robot_move_motor_targets']) == 1
    failure = json.loads((tmp_path/outcome/'failure.json').read_text())
    assert failure['cleanup']['release_confirmed'] is True and failure['automatic_retry'] is False


def test_steps_below_the_owner_minimum_segment_are_refused_before_dispatch(rig):
    owner, cfg = rig
    transport = GemmaTransport(owner, 'right', cfg['joints'], execute=True, clock=owner.clock)
    transport.preflight()
    observer = GemmaTagObserver(owner, transport, clock=owner.clock)
    transport.enable()
    observer.observe()
    for ticks in (1, -2):
        with pytest.raises(Refused, match='3..68'):
            transport.move('right_arm_shoulder_pan', ticks)
    assert not any(n == 'robot_move_motor_targets' for n, _ in owner.calls)
    transport.move('right_arm_shoulder_pan', 3)


def test_stop_without_immediate_confirmation_waits_for_fresh_torque_zero(rig):
    """Soft release: robot_stop can return before torque is off; fresh reads confirm it afterwards."""
    owner, cfg = rig
    transport = GemmaTransport(owner, 'right', cfg['joints'], execute=True, clock=owner.clock)
    transport.preflight()
    transport.enable()
    original, ramp = owner.call, []
    def call(name, args, request_id=None):
        if name == 'robot_stop':
            ramp.append(owner.now)
            return {'ok': True, 'result': {'stop_requested': True, 'release_confirmed': False,
                    'release_reason': 'Fresh same-session all16 torque-zero readback not observed'}}
        if ramp and owner.enabled and owner.now-ramp[0] >= .1:
            owner.enabled.clear()
        return original(name, args, request_id)
    owner.call = call
    transport.finish(failed=True)
    assert transport.cleanup['release_confirmed'] is True and transport.cleanup['stop']['release_confirmed'] is False
    assert not owner.enabled
