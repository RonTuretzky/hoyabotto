"""Read-only Gemma integration with fake telemetry, registration and IK only."""
import copy
import hashlib
import json

import pytest

from carton.servo.bimanual_owner import ARMS, ARM_JOINTS, JOINTS, PROTOCOL, VERSION
from carton.servo.common import Refused, digest
from carton.servo.folding_readiness import FoldingReadiness, READS, _ReadOnlyRobot
from farm.perception.gemma_calibration import CalibrationRobot


class Owner:
    def __init__(self):
        self.now = 1000.
        self.calls = []
        self.stop = False
        self.stale = False
        self.q = dict.fromkeys(JOINTS, 2000)
        self.caps = {'motion_ready': True, 'calibrated': True, 'folding_ready': True,
                     'bimanual_trajectory_protocol': PROTOCOL, 'bimanual_trajectory_version': VERSION}

    def clock(self):
        return self.now

    def catalog(self):
        return {'tools': []}

    def call(self, name, args, request_id=None):
        assert name in READS, f'Attempted non-read action: {name}'
        self.calls.append((name, copy.deepcopy(args)))
        if name == 'robot_get_capabilities':
            result = copy.deepcopy(self.caps)
        elif name == 'robot_get_execution':
            result = {'time': self.now, 'started': 900., 'ok': True, 'stop_latched': self.stop,
                      'phase': 'idle', 'accepted': 0, 'completed': 0, 'motor_writes': 0}
        elif name == 'robot_get_state':
            result = {'cached': False, 'time': self.now, 'raw_calibration_ranges': {
                n: {'min_ticks': 1000, 'max_ticks': 3000} for n in JOINTS},
                'motors': [{'name': n, 'Present_Position': q, 'Status': 0, 'Moving': 0,
                            'Present_Velocity': 0, 'Present_Load': 0,
                            'captured_at': self.now - (2 if self.stale else .001)} for n, q in self.q.items()]}
        else:
            raise AssertionError(name)
        return {'ok': True, 'result': result}


class Arm:
    instances = []

    def __init__(self, config):
        self.config = config
        self.instances.append(self)

    def plan(self, current, request):
        assert request['frame'] == 'station' and request['orientation'] == 'constrained'
        assert request['units'] == 'metres'
        return {'status': 'KINEMATIC_PROPOSAL_ONLY', 'waypoints': [
            {'joint_targets_ticks': {n: current[n]+i+1 for n in ARM_JOINTS[self.config['arm']][:-1]}}
            for i, _ in enumerate(request['tool_poses'])], 'motor_writes': 0}


@pytest.fixture
def rig(tmp_path):
    owner = Owner()
    artifacts = {}

    def save(name, obj):
        path = tmp_path / (name+'.json')
        path.write_text(json.dumps(obj))
        artifacts[name] = obj
        return {'path': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    cfg = {'schema': 1, 'arms': {}}
    hashes = {k: {} for k in ('calibration_sha256', 'registration_sha256', 'kinematics_sha256')}
    for arm in ARMS:
        tag_id = 4 if arm == 'left' else 2
        cal = save(arm+'-cal', {n: {'range_min': 1000, 'range_max': 3000, 'homing_offset': 0}
                                   for n in ARM_JOINTS[arm]})
        kin = save(arm+'-kin', {'schema': 1, 'arm': arm, 'calibration_file': cal['path'],
                              'calibration_sha256': cal['sha256'], 'model_directory': 'fake-model'})
        reg = save(arm+'-reg', {'status': 'REGISTRATION_VALIDATED', 'binding': {
            'arm': arm, 'gripper_tag_id': tag_id, 'motor_calibration_sha256': cal['sha256']}})
        cfg['arms'][arm] = {'kinematics': kin, 'registration': reg, 'tag_config': save(arm+'-tag', {
            'arm': arm, 'gripper_tag_id': tag_id, 'camera': arm+'-camera'})}
        for key, ref in zip(hashes, (cal, reg, kin)):
            hashes[key][arm] = ref['sha256']
    bindings = {**hashes, 'station_sha256': digest('station'), 'commissioning_sha256': digest('claimed record'),
                'camera_ids': {'left-camera': 'left-device', 'right-camera': 'right-device'},
                'camera_streams': {'left-camera': 'left-stream', 'right-camera': 'right-stream'},
                'ranges': {n: [1000, 3000] for n in JOINTS}}
    profile = {'protocol': PROTOCOL, 'schema': VERSION, 'units': 'encoder_ticks', 'joints': list(JOINTS),
               'bindings': bindings, 'commissioning_evidence': {'status': 'COMMISSIONED',
                   'reference': 'A claim is not measurement proof', 'sha256': bindings['commissioning_sha256']},
               'corridor': {n: [1800, 2400] for n in JOINTS}, 'velocity': dict.fromkeys(JOINTS, 100),
               'acceleration': dict.fromkeys(JOINTS, 200),
               'load_raw': {n: 250 if n.endswith('gripper') else 500 for n in JOINTS},
               'following_ticks': 24, 'start_ticks': 5, 'settle_ticks': 5, 'vision_age_s': .2,
               'max_camera_skew_s': .02, 'max_tick_gap_s': .2, 'telemetry_age_s': .2,
               'max_telemetry_skew_s': .02, 'max_dispatch_skew_s': .02, 'settle_timeout_s': 1,
               'max_sample_ticks': 68}
    cfg['profile'], cfg['bindings'] = save('profile', profile), save('bindings', bindings)
    cfg['simulation'] = save('simulation', {'status': 'CLAIMED_PASS', 'success': True, 'motion_ready': True})
    owner.caps.update(profile_sha256=digest(profile), bindings_sha256=digest(bindings))
    config_path = tmp_path/'folding-readiness.json'
    config_path.write_text(json.dumps(cfg))
    registered_calls = []

    def registered(robot, tag, registration, *, clock):
        registered_calls.append(tag['arm'])
        assert isinstance(robot, _ReadOnlyRobot)
        arm = tag['arm']
        return {'ok': True, 'result': {'status': 'REGISTERED_TAG_ESTIMATES', 'arm': arm,
                'motor_writes': 0, 'registration_sha256': digest(registration), 'tags': [],
                'frame': {'camera_id': arm+'-device', 'stream_id': arm+'-stream',
                          'seq': 1, 'sha256': digest(arm), 'captured_at': clock()}}}

    service = FoldingReadiness(owner, config_path, clock=owner.clock, arm_factory=Arm,
                               registered_reader=registered)
    return owner, service, cfg, artifacts, save, registered_calls


def request():
    pose = [[1, 0, 0, .2], [0, 1, 0, 0], [0, 0, 1, .2], [0, 0, 0, 1]]
    return {'targets': {a: [pose] for a in ARMS}, 'segment_seconds': .5}


def test_constructor_and_catalog_do_not_read_owner_or_camera(rig):
    owner, service, *_ = rig
    assert owner.calls == []
    wrapped = CalibrationRobot(owner, service.path.with_name('tag-calibration.json'))
    names = [t['function']['name'] for t in wrapped.catalog()['tools']]
    assert names == ['robot_folding_status', 'robot_folding_proposal']
    assert owner.calls == []


def test_claims_and_prepared_files_never_promote_motion_readiness(rig):
    owner, service, _, _, _, registrations = rig
    result = service.status()
    assert result['motion_ready'] is False and result['execution_available'] is False
    assert result['motor_writes'] == 0 and result['proposal_available'] is True
    assert result['simulation']['artifact_hash_verified'] is True
    assert result['simulation']['outcome_independently_verified'] is False
    assert result['owner']['local_profile_schema_verified'] is True
    assert result['owner']['physical_commissioning_verified'] is False
    assert result['policy']['complete_physical_policy_verified'] is False
    assert registrations == ['left', 'right']
    assert len(result['blockers']) >= 5
    assert all(name in READS for name, _ in owner.calls)
    assert result['calibration']['left']['gripper_tag_id'] == 4
    assert result['calibration']['right']['gripper_tag_id'] == 2


def test_paired_proposal_keeps_jaws_and_cannot_be_an_owner_command(rig):
    owner, service, *_ = rig
    result = service.proposal(request())
    assert result['status'] == 'PAIRED_KINEMATIC_PROPOSAL_ONLY'
    assert result['motion_ready'] is False
    p = result['proposal']
    assert p['protocol'] == PROTOCOL and p['schema'] == VERSION and p['op'] == 'proposal_only'
    assert 'session_started' not in p and 'id' not in p and p['scene_sha256'] is None
    assert p['collision_checked'] is False and p['motor_writes'] == 0
    assert len(p['waypoints']) == 2
    for row in p['waypoints']:
        assert set(row['positions']) == set(JOINTS)
        assert row['positions']['left_arm_gripper'] == row['positions']['right_arm_gripper'] == 2000
    assert set(p['waypoints'][1]['positions'].values()) == {2000, 2001}
    assert result['proposal_sha256'] == digest(p)
    assert all(name in READS for name, _ in owner.calls)


@pytest.mark.parametrize('fault, phrase', [('stop', 'STOP'), ('stale', 'telemetry is stale')])
def test_stop_or_stale_telemetry_prevents_proposal_without_recovery(rig, fault, phrase):
    owner, service, *_, registrations = rig
    setattr(owner, fault, True)
    result = service.proposal(request())
    assert result['proposal'] is None and result['motion_ready'] is False
    assert any(phrase in b for b in result['blockers'])
    assert registrations == []


def test_wrong_arm_tag_blocks_before_that_arm_registered_read(rig):
    _, service, cfg, artifacts, save, registrations = rig
    artifacts['left-tag']['gripper_tag_id'] = 2
    cfg['arms']['left']['tag_config'] = save('left-tag', artifacts['left-tag'])
    service.path.write_text(json.dumps(cfg))
    result = service.proposal(request())
    assert result['proposal'] is None and registrations == ['right']
    assert any('requires gripper tag 4' in b for b in result['blockers'])


def test_artifact_mutation_without_new_pin_is_blocked(rig):
    _, service, *_ = rig
    service.path.with_name('left-kin.json').write_text('{}')
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('hash changed: left-kin.json' in b for b in result['blockers'])


def test_saved_calibration_hash_differs_from_registration(rig):
    _, service, cfg, artifacts, save, _ = rig
    artifacts['left-reg']['binding']['motor_calibration_sha256'] = digest('wrong calibration')
    cfg['arms']['left']['registration'] = save('left-reg', artifacts['left-reg'])
    service.path.write_text(json.dumps(cfg))
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('saved motor calibration hashes differ' in b for b in result['blockers'])


def test_owner_profile_mismatch_is_not_cured_by_ready_boolean(rig):
    owner, service, *_ = rig
    owner.caps['profile_sha256'] = digest('another profile')
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('Owner advertised profile/bindings differ' in b for b in result['blockers'])


def test_telemetry_change_during_registered_capture_blocks_entire_pair(rig):
    owner, service, *_ = rig
    original = service.registered_reader
    def changed(*args, **kwargs):
        result = original(*args, **kwargs)
        owner.q['right_arm_shoulder_pan'] += 1
        return result
    service.registered_reader = changed
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('changed during evidence collection' in b for b in result['blockers'])


def test_read_facade_refuses_accidental_enable(rig):
    owner, service, *_ = rig
    def faulty(robot, *args, **kwargs):
        robot.call('robot_set_motor_enable', {'enabled': True})
    service.registered_reader = faulty
    result = service.proposal(request())
    assert result['proposal'] is None
    assert all(name in READS for name, _ in owner.calls)


def test_asymmetric_invalid_ik_does_not_return_partial_pair(rig):
    _, service, *_ = rig
    class BadArm(Arm):
        def plan(self, current, request):
            if self.config['arm'] == 'right':
                raise ValueError('right target unreachable')
            return super().plan(current, request)
    service.arm_factory = BadArm
    result = service.proposal(request())
    assert result['proposal'] is None and 'kinematic_results' not in result
    assert any('right target unreachable' in b for b in result['blockers'])


def test_missing_config_and_unknown_arguments_never_forward(rig):
    owner, service, *_ = rig
    wrapped = CalibrationRobot(owner, service.path.with_name('tag-calibration.json'))
    assert not wrapped.call('robot_folding_status', {'run': True})['ok']
    assert not wrapped.call('robot_folding_proposal', {'targets': {}, 'enable': True})['ok']
    service.path.unlink()
    assert not wrapped.call('robot_folding_status', {})['ok']
    assert owner.calls == []
    assert wrapped.catalog()['tools'] == []


def test_missing_local_measurements_yield_explicit_blockers(rig):
    _, service, *_ = rig
    service.path.with_name('left-cal.json').unlink()
    result = service.status()
    assert result['motion_ready'] is False and result['proposal_available'] is False
    assert any('left calibration/kinematics:' in b for b in result['blockers'])


def test_wrapper_returns_read_only_status_with_incomplete_manifest(rig):
    owner, service, *_ = rig
    service.path.write_text(json.dumps({'schema': 1, 'arms': {'left': {}, 'right': {}}}))
    wrapped = CalibrationRobot(owner, service.path.with_name('tag-calibration.json'), clock=owner.clock)
    response = wrapped.call('robot_folding_status', {})
    assert response['ok'] and response['result']['motion_ready'] is False
    assert response['result']['proposal_available'] is False and response['motor_writes'] == 0


@pytest.mark.parametrize('field,value', [('captured_at', 990), ('camera_id', 'replacement-device'), ('stream_id', 'new-stream')])
def test_profile_camera_identity_and_freshness_are_required_for_proposal(rig, field, value):
    _, service, *_ = rig
    original = service.registered_reader
    def mismatched(*args, **kwargs):
        result = original(*args, **kwargs)
        result['result']['frame'][field] = value
        return result
    service.registered_reader = mismatched
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('frame identity/stream/age' in b for b in result['blockers'])


def test_right_arm_corridor_failure_cannot_return_left_only_proposal(rig):
    _, service, *_ = rig
    class Outside(Arm):
        def plan(self, current, request):
            result = super().plan(current, request)
            if self.config['arm'] == 'right':
                result['waypoints'][0]['joint_targets_ticks']['right_arm_elbow_flex'] = 2500
            return result
    service.arm_factory = Outside
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('commissioned encoder corridor' in b for b in result['blockers'])


def test_registered_frames_expiring_during_ik_refuse_proposal(rig):
    owner, service, *_ = rig
    class SlowArm(Arm):
        def plan(self, current, request):
            result = super().plan(current, request)
            owner.now += .15
            return result
    service.arm_factory = SlowArm
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('Registered frames exceed paired profile age/skew' in b for b in result['blockers'])


def test_bound_profile_cannot_silently_substitute_saved_ranges(rig):
    owner, service, cfg, artifacts, save, _ = rig
    bindings = artifacts['bindings']
    bindings['ranges']['left_arm_elbow_flex'] = [999, 3000]
    profile = artifacts['profile']
    profile['bindings'] = bindings
    cfg['bindings'] = save('bindings', bindings)
    cfg['profile'] = save('profile', profile)
    service.path.write_text(json.dumps(cfg))
    owner.caps.update(profile_sha256=digest(profile), bindings_sha256=digest(bindings))
    result = service.proposal(request())
    assert result['proposal'] is None
    assert any('paired profile range differs from bound calibration' in b for b in result['blockers'])


def test_wrapper_dispatches_the_read_only_proposal_without_forwarding(rig, monkeypatch):
    import carton.servo.folding_readiness as module
    owner, service, *_ = rig
    wrapped = CalibrationRobot(owner, service.path.with_name('tag-calibration.json'), clock=owner.clock)
    monkeypatch.setattr(module, 'FoldingReadiness', lambda *args, **kwargs: service)
    response = wrapped.call('robot_folding_proposal', request(), request_id='synthetic-request')
    assert response['ok'] and response['motor_writes'] == 0
    assert response['result']['proposal']['op'] == 'proposal_only'
    assert response['result']['motion_ready'] is False
    assert all(name in READS for name, _ in owner.calls)


def test_catalog_refuses_a_server_name_collision_without_reading_devices(rig):
    owner, service, *_ = rig
    owner.catalog = lambda: {'tools': [{'type': 'function', 'function': {'name': 'robot_folding_status'}}]}
    wrapped = CalibrationRobot(owner, service.path.with_name('tag-calibration.json'))
    with pytest.raises(Refused, match='already supplied'):
        wrapped.catalog()
    assert owner.calls == []


def test_matching_hashes_without_protocol_advertisement_remain_blocked(rig):
    owner, service, *_ = rig
    owner.caps.pop('bimanual_trajectory_version')
    result = service.proposal(request())
    assert result['proposal'] is None and result['motion_ready'] is False
    assert any('does not advertise' in b for b in result['blockers'])


def test_partial_simulation_report_preserves_failure_without_promoting_claims(rig):
    _, service, cfg, _, save, _ = rig
    cfg['simulation'] = save('simulation', {
        'stage': 'far_major_attempt', 'error': 'Fresh marker missing',
        'full_task_complete': False, 'angles': {'near': 91.8, 'far': 13.6}})
    service.path.write_text(json.dumps(cfg))
    result = service.status()
    sim = result['simulation']
    assert sim['reported_stage'] == 'far_major_attempt'
    assert sim['reported_error'] == 'Fresh marker missing'
    assert sim['reported_full_task_complete'] is False
    assert sim['reported_angles'] == {'near': 91.8, 'far': 13.6}
    assert sim['outcome_independently_verified'] is False
    assert result['motion_ready'] is False and result['motor_writes'] == 0
