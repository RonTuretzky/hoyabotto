"""API-level fake owner tests; no real cameras, serial devices or motion."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton.servo.common import Limits, Refused, digest
from carton.servo.gemma import GemmaTagObserver, GemmaTransport
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

    def call(self, name, args, request_id=None):
        self.now += .02
        self.calls.append((name, copy.deepcopy(args)))
        if name == 'robot_get_capabilities':
            r = dict(motion_units='encoder_ticks; positioning-joint degrees use4095 ticks/rev',
                     motion_ready=not self.stopped, joint_blockers={}, blockers=[])
        elif name == 'robot_get_execution':
            r = dict(time=self.now, started=self.started, ok=True, control_mode='direct_joint',
                     phase='stopped' if self.stopped else ('holding' if self.enabled and self.phase == 'idle' else self.phase), operator_armed=not self.stopped,
                     stop_latched=self.stopped, enabled_motors=sorted(self.enabled), accepted=self.command,
                     completed=self.command, motor_writes=self.writes, lease_remaining=self.lease)
        elif name == 'robot_get_state':
            r = dict(cached=False, time=self.now, commandable_ranges=copy.deepcopy(self.ranges),
                     raw_calibration_ranges=copy.deepcopy(self.ranges), motors=[dict(name=n, Present_Position=q,
                     Present_Velocity=0, Present_Load=0, Moving=0, Status=0, Torque_Enable=int(n in self.enabled),
                     captured_at=self.now-.001) for n, q in self.q.items()])
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
    assert enabled and all(n.startswith('left_arm_') and not n.endswith('gripper') for n in enabled[0]['names'])
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
    assert len(enable) == 2 and len(enable[0]['names']) == 5
    assert all(n.startswith('right_arm_') and not n.endswith('gripper') for n in enable[0]['names'])


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
