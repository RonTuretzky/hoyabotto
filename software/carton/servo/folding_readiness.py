"""Opt-in, read-only evidence and paired IK proposals for Gemma.

There is deliberately no motion/enable endpoint or readiness promotion here.
Saved artifacts, fresh registered marker estimates and a rate-limited IK proposal
are separate evidence domains; none certifies a physical folding policy.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import time

from .bimanual_owner import ARMS, ARM_JOINTS, JOINTS, PROTOCOL, VERSION, _session_identity, validate_profile
from .common import Refused, digest, finite
from .trajectory import JointTrajectory
from farm.kinematics.lerobot import CalibratedArm
from farm.perception.registered_tags import read_registered_tags
from farm.perception.tag_sampling import gripper_tag_for_arm

STATUS = 'robot_folding_status'
PROPOSAL = 'robot_folding_proposal'
READS = {'robot_get_capabilities', 'robot_get_execution', 'robot_get_state',
         'robot_get_tags', 'robot_get_arm_pose'}
FIXED_BLOCKERS = [
    'No folding execution endpoint is installed by this wrapper.',
    'Paired-owner physical commissioning, independent watchdog and I/O timeout evidence are not verified here.',
    'Fresh physical homing-offset/limit-register readbacks matching saved calibration are not verified here.',
    'No independent observed-scene collision certificate bound to the exact proposed trajectory is supplied.',
    'No complete physically validated adaptive four-flap folding policy is installed by this wrapper.',
]


def tool_definitions():
    matrix = {'type': 'array', 'minItems': 4, 'maxItems': 4, 'items': {
        'type': 'array', 'minItems': 4, 'maxItems': 4, 'items': {'type': 'number'}}}
    poses = {'type': 'array', 'minItems': 1, 'maxItems': 20, 'items': matrix}
    return [
        {'type': 'function', 'function': {'name': STATUS, 'description':
         'Read separate local simulation evidence, paired owner schema/capability, calibrated IK and fresh '
         'registered-tag evidence with exact blockers. Always motion_ready:false; saved files and claimed '
         'booleans do not certify physical calibration or folding. No enable, motor command or automatic retry.',
         'parameters': {'type': 'object', 'properties': {}, 'additionalProperties': False}}},
        {'type': 'function', 'function': {'name': PROPOSAL, 'description':
         'Prepare only a paired station-frame IK proposal from fresh encoders and registered tags. '
         'Provide equal-length left/right sequences of 4x4 tool poses in metres with constrained orientation. '
         'Keeps each jaw at its measured position, checks configured corridors and rates, and reports all '
         'remaining blockers. Not a complete folding policy, collision certificate, motor command or permission '
         'to execute. Never enables, executes or retries. Local paths and limits cannot be supplied here.',
         'parameters': {'type': 'object', 'properties': {
             'targets': {'type': 'object', 'properties': {a: poses for a in ARMS},
                         'required': list(ARMS), 'additionalProperties': False},
             'segment_seconds': {'type': 'number', 'minimum': .1, 'maximum': 5}},
             'required': ['targets', 'segment_seconds'], 'additionalProperties': False}}},
    ]


class _ReadOnlyRobot:
    def __init__(self, robot):
        self.robot = robot

    def call(self, name, args):
        if name not in READS:
            raise Refused(f'Folding evidence cannot call {name}')
        return self.robot.call(name, args)


class FoldingReadiness:
    def __init__(self, robot, config_path, *, clock=time.time, arm_factory=CalibratedArm,
                 registered_reader=read_registered_tags):
        # Construction/catalog enumeration never opens an owner session or camera.
        self.robot = _ReadOnlyRobot(robot)
        self.path = Path(config_path)
        self.clock, self.arm_factory, self.registered_reader = clock, arm_factory, registered_reader

    def _json(self, path):
        data = path.read_bytes()
        if len(data) > 2_000_000:
            raise Refused('Folding evidence JSON exceeds two megabytes')
        obj = json.loads(data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        if not isinstance(obj, dict):
            raise Refused('Folding evidence must be a JSON object')
        return obj, hashlib.sha256(data).hexdigest()

    def _artifact(self, reference):
        if not isinstance(reference, dict) or set(reference) != {'path', 'sha256'}:
            raise Refused('Missing local evidence path and exact byte SHA256')
        path = Path(reference['path'])
        path = path if path.is_absolute() else self.path.parent / path
        obj, actual = self._json(path)
        if actual != reference['sha256']:
            raise Refused(f'Local evidence hash changed: {path.name}')
        return obj, actual, path

    def _read(self, name, args=None):
        response = self.robot.call(name, {} if args is None else args)
        if response.get('ok') is not True or not isinstance(response.get('result'), dict):
            raise Refused(f'{name} did not return a successful read')
        return response['result']

    def _snapshot(self):
        execution = self._read('robot_get_execution')
        state = self._read('robot_get_state', {'fresh': True})
        if (state.get('cached') is not False or execution.get('ok') is not True
                or execution.get('stop_latched') is not False
                or execution.get('phase') not in ('idle', 'holding')):
            raise Refused('Need a fresh stationary owner snapshot with STOP clear; never reset it here')
        for name, row in (('execution', execution), ('state', state)):
            if not 0 <= self.clock() - finite(row.get('time')) <= .75:
                raise Refused(f'{name} snapshot is stale or future-dated')
        _session_identity(execution.get('started'))
        for key in ('accepted', 'completed', 'motor_writes'):
            finite(execution.get(key), key)
        if execution['accepted'] != execution['completed']:
            raise Refused('Owner has an incomplete command')
        rows = state.get('motors', [])
        if not isinstance(rows, list) or any(not isinstance(r, dict) for r in rows):
            raise Refused('Invalid motor snapshot')
        names = [r.get('name') for r in rows]
        if len(names) != len(set(names)):
            raise Refused('Duplicate motor telemetry')
        motors = {r['name']: r for r in rows}
        if not set(JOINTS) <= set(motors):
            raise Refused('Fresh telemetry does not cover both arms')
        q, stamps = {}, []
        for name in JOINTS:
            row = motors[name]
            stamp = finite(row.get('captured_at'))
            if not 0 <= self.clock() - stamp <= .2:
                raise Refused(f'{name}: telemetry is stale or future-dated')
            if (type(row.get('Present_Position')) is not int or not 0 <= row['Present_Position'] <= 4095
                    or type(row.get('Status')) is not int or row['Status'] != 0
                    or row.get('Moving') != 0 or abs(finite(row.get('Present_Velocity'))) > 1
                    or abs(finite(row.get('Present_Load'))) >= (250 if name.endswith('gripper') else 500)):
                raise Refused(f'{name}: invalid or nonstationary motor telemetry')
            q[name] = row['Present_Position']
            stamps.append(stamp)
        if max(stamps) - min(stamps) > .1:
            raise Refused('Both-arm telemetry capture skew exceeds 0.1 seconds')
        return {'execution': execution, 'state': state, 'positions': q,
                'telemetry_oldest_at': min(stamps), 'telemetry_skew_s': max(stamps)-min(stamps)}

    @staticmethod
    def _unchanged(before, after):
        keys = ('started', 'accepted', 'completed', 'motor_writes')
        if (any(before['execution'][k] != after['execution'][k] for k in keys)
                or before['positions'] != after['positions']
                or before['state'].get('raw_calibration_ranges') != after['state'].get('raw_calibration_ranges')):
            raise Refused('Owner, encoders or saved ranges changed during evidence collection')

    def _profile_fresh(self, snapshot, registered, profile):
        if (self.clock() - snapshot['telemetry_oldest_at'] > profile['telemetry_age_s']
                or snapshot['telemetry_skew_s'] > profile['max_telemetry_skew_s']):
            raise Refused('Telemetry exceeds paired profile age/skew limits')
        stamps = [finite(registered[a]['frame'].get('captured_at')) for a in ARMS]
        if (any(not 0 <= self.clock() - stamp <= profile['vision_age_s'] for stamp in stamps)
                or max(stamps) - min(stamps) > profile['max_camera_skew_s']):
            raise Refused('Registered frames exceed paired profile age/skew limits')

    def _inspect(self):
        report = {'status': 'FOLDING_BLOCKED', 'motion_ready': False, 'motor_writes': 0,
                  'proposal_available': False,
                  'execution_available': False, 'automatic_retry': False, 'blockers': list(FIXED_BLOCKERS),
                  'simulation': {}, 'kinematics': {a: {'local_solver_loaded': False} for a in ARMS},
                  'calibration': {}, 'owner': {},
                  'policy': {'complete_physical_policy_verified': False}}
        internal = {'models': {}, 'configs': {}, 'hashes': {}, 'registered': {}}
        try:
            cfg, _ = self._json(self.path)
            if cfg.get('schema') != 1 or set(cfg.get('arms', {})) != set(ARMS):
                raise Refused('Folding readiness schema 1 needs explicit left and right configurations')
        except (OSError, ValueError, TypeError, Refused) as exc:
            report['blockers'].append(f'Local manifest: {exc}')
            return report, internal
        try:
            sim, sha, _ = self._artifact(cfg.get('simulation'))
            report['simulation'] = {'artifact_sha256': sha, 'artifact_hash_verified': True,
                'reported_status': sim.get('status'), 'reported_stage': sim.get('stage'),
                'reported_error': sim.get('error'),
                'reported_full_task_complete': sim.get('full_task_complete'),
                'reported_angles': sim.get('angles'), 'outcome_independently_verified': False,
                'note': 'Only local artifact identity is checked here; this is not a simulation or physical pass.'}
        except (OSError, ValueError, KeyError, TypeError, Refused) as exc:
            report['simulation'] = {'artifact_hash_verified': False, 'blocker': str(exc)}
            report['blockers'].append(f'Simulation evidence: {exc}')
        try:
            internal['before'] = self._snapshot()
            caps = self._read('robot_get_capabilities')
            report['owner'] = {'fresh_stationary_snapshot': True,
                'reported_protocol': caps.get('bimanual_trajectory_protocol'),
                'reported_version': caps.get('bimanual_trajectory_version'),
                'paired_protocol_advertised': caps.get('bimanual_trajectory_protocol') == PROTOCOL
                    and caps.get('bimanual_trajectory_version') == VERSION,
                'deployment_independently_verified': False}
            internal['capabilities'] = caps
            if not report['owner']['paired_protocol_advertised']:
                report['blockers'].append('Current owner does not advertise the paired trajectory v2 protocol.')
        except (OSError, ValueError, KeyError, TypeError, Refused) as exc:
            report['owner']['fresh_stationary_snapshot'] = False
            report['blockers'].append(f'Owner: {exc}')
        for arm in ARMS:
            try:
                arm_cfg = cfg['arms'][arm]
                tag, _, _ = self._artifact(arm_cfg.get('tag_config'))
                registration, registration_sha, _ = self._artifact(arm_cfg.get('registration'))
                kin, kin_sha, kin_path = self._artifact(arm_cfg.get('kinematics'))
                if tag.get('arm') != arm or kin.get('arm') != arm or registration.get('binding', {}).get('arm') != arm:
                    raise Refused('Arm identity differs across tag, registration and kinematics artifacts')
                selected = gripper_tag_for_arm(arm, tag.get('gripper_tag_id'))
                if gripper_tag_for_arm(arm, registration['binding'].get('gripper_tag_id')) != selected:
                    raise Refused('Registration tag differs from configured arm tag')
                # Kinematics paths are relative to their own artifact, not process cwd.
                kin = copy.deepcopy(kin)
                for key in ('calibration_file', 'model_directory'):
                    p = Path(kin[key])
                    kin[key] = str(p if p.is_absolute() else kin_path.parent / p)
                calibration, cal_sha = self._json(Path(kin['calibration_file']))
                if (kin.get('calibration_sha256') != cal_sha
                        or registration['binding'].get('motor_calibration_sha256') != cal_sha):
                    raise Refused('Kinematics, registration and saved motor calibration hashes differ')
                model = self.arm_factory(kin)
                internal['models'][arm], internal['configs'][arm] = model, kin
                internal['hashes'][arm] = {'calibration_sha256': cal_sha,
                    'registration_sha256': registration_sha, 'kinematics_sha256': kin_sha}
                report['kinematics'][arm] = {'configuration_hash_verified': True, 'local_solver_loaded': True,
                    'kinematics_sha256': kin_sha, 'physical_zero_sign_tool_measurements_verified': False}
                if 'before' not in internal:
                    raise Refused('Fresh stationary owner readback is unavailable')
                ranges = internal['before']['state'].get('raw_calibration_ranges', {})
                for name in ARM_JOINTS[arm]:
                    saved, actual = calibration[name], ranges.get(name, {})
                    if (actual.get('min_ticks') != saved['range_min'] or actual.get('max_ticks') != saved['range_max']):
                        raise Refused(f'{name}: owner saved ranges differ from calibration file')
                # Existing registered-tag API performs residual/FK, stream, mount and anchor checks.
                tag = copy.deepcopy(tag)
                tag['model_directory'] = kin['model_directory']
                observed = self.registered_reader(self.robot, tag, registration, clock=self.clock)
                estimate = observed.get('result', {})
                if (observed.get('ok') is not True or estimate.get('status') != 'REGISTERED_TAG_ESTIMATES'
                        or estimate.get('arm') != arm or estimate.get('motor_writes') != 0):
                    raise Refused('Registered-tag verifier did not return read-only matching arm estimates')
                internal['registered'][arm] = estimate
                report['calibration'][arm] = {'gripper_tag_id': selected, 'registered_tag_estimates': estimate,
                    'saved_calibration_sha256': cal_sha, 'owner_saved_ranges_match': True,
                    'physical_register_readbacks_verified': False, 'physical_tool_contact_verified': False}
            except (OSError, ValueError, KeyError, TypeError, ImportError, Refused) as exc:
                report['blockers'].append(f'{arm} calibration/kinematics: {exc}')
                report['calibration'].setdefault(arm, {})['blocker'] = str(exc)
        try:
            profile, _, _ = self._artifact(cfg.get('profile'))
            bindings, _, _ = self._artifact(cfg.get('bindings'))
            profile_sha = validate_profile(profile, bindings)
            if not report['owner'].get('paired_protocol_advertised'):
                raise Refused('Owner does not advertise the paired trajectory v2 protocol')
            if set(internal['hashes']) != set(ARMS) or set(internal['registered']) != set(ARMS):
                raise Refused('Both arms need bound local calibration/kinematics and fresh registered-tag estimates')
            for arm in ARMS:
                for key, sha in internal['hashes'][arm].items():
                    if bindings[key][arm] != sha:
                        raise Refused(f'{arm}: paired profile binding differs from {key}')
                for name in ARM_JOINTS[arm]:
                    saved = internal['before']['state']['raw_calibration_ranges'][name]
                    if bindings['ranges'][name] != [saved['min_ticks'], saved['max_ticks']]:
                        raise Refused(f'{name}: paired profile range differs from bound calibration')
                frame = internal['registered'][arm]['frame']
                camera_names = [n for n, identity in bindings['camera_ids'].items() if identity == frame.get('camera_id')]
                if (len(camera_names) != 1 or frame.get('stream_id') != bindings['camera_streams'][camera_names[0]]
                        or not 0 <= self.clock() - finite(frame.get('captured_at')) <= profile['vision_age_s']):
                    raise Refused(f'{arm}: registered frame identity/stream/age differs from paired profile')
            if internal['capabilities'].get('profile_sha256') != profile_sha or internal['capabilities'].get('bindings_sha256') != digest(bindings):
                raise Refused('Owner advertised profile/bindings differ from local paired profile')
            internal['profile'], internal['bindings'] = profile, bindings
            report['owner'].update({'local_profile_schema_verified': True, 'profile_sha256': profile_sha,
                                   'bindings_sha256': digest(bindings), 'physical_commissioning_verified': False})
        except (OSError, ValueError, KeyError, TypeError, Refused) as exc:
            report['owner']['local_profile_schema_verified'] = False
            report['blockers'].append(f'Paired profile: {exc}')
        try:
            if 'before' not in internal:
                raise Refused('No coherent initial owner snapshot is available')
            internal['after'] = self._snapshot()
            self._unchanged(internal['before'], internal['after'])
            if 'profile' in internal and set(internal['registered']) == set(ARMS):
                self._profile_fresh(internal['after'], internal['registered'], internal['profile'])
            report['owner']['evidence_snapshot_stable'] = True
        except (OSError, ValueError, KeyError, TypeError, Refused) as exc:
            internal.pop('after', None)
            report['blockers'].append(f'Evidence stability: {exc}')
        report['proposal_available'] = (set(internal['registered']) == set(ARMS)
            and 'profile' in internal and 'after' in internal)
        return report, internal

    def status(self):
        return self._inspect()[0]

    def proposal(self, args):
        if (not isinstance(args, dict) or set(args) != {'targets', 'segment_seconds'}
                or not isinstance(args['targets'], dict) or set(args['targets']) != set(ARMS)):
            raise Refused('Proposal requires exactly both-arm targets and segment_seconds')
        seconds = finite(args['segment_seconds'])
        if not .1 <= seconds <= 5:
            raise Refused('Proposal segment_seconds must be between 0.1 and 5')
        targets = args['targets']
        if any(not isinstance(targets[a], list) or not 1 <= len(targets[a]) <= 20 for a in ARMS) or len(targets['left']) != len(targets['right']):
            raise Refused('Proposal needs equal-length sequences of 1–20 tool poses for both arms')
        report, internal = self._inspect()
        if not report.get('proposal_available'):
            return {**report, 'status': 'FOLDING_PROPOSAL_BLOCKED', 'proposal': None}
        try:
            current = internal['after']['positions']
            plans = {a: internal['models'][a].plan(current, {'frame': 'station', 'units': 'metres',
                     'orientation': 'constrained', 'tool_poses': targets[a]}) for a in ARMS}
            waypoints = [{'time_s': 0., 'positions': dict(current)}]
            for i in range(len(targets['left'])):
                q = dict(current)
                for arm in ARMS:
                    proposed = plans[arm]['waypoints'][i]['joint_targets_ticks']
                    if set(proposed) != set(ARM_JOINTS[arm][:-1]) or any(type(v) is not int for v in proposed.values()):
                        raise Refused('IK proposal must contain exactly five integer positioning joints per arm')
                    q.update(proposed)
                waypoints.append({'time_s': (i+1)*seconds, 'positions': q})
            p = internal['profile']
            trajectory = JointTrajectory(waypoints, JOINTS, p['corridor'], p['velocity'], p['acceleration'])
            if trajectory.duration + p['settle_timeout_s'] > 30:
                raise Refused('Paired proposal plus settling exceeds the owner supervision horizon')
            for point, stamp in zip(waypoints, trajectory.times):
                point['time_s'] = float(stamp)
            latest = self._snapshot()
            self._unchanged(internal['after'], latest)
            self._profile_fresh(latest, internal['registered'], p)
            proposal = {'protocol': PROTOCOL, 'schema': VERSION, 'op': 'proposal_only',
                'units': 'encoder_ticks', 'profile_sha256': report['owner']['profile_sha256'],
                'bindings_sha256': report['owner']['bindings_sha256'], 'waypoints': waypoints,
                'collision_checked': False, 'scene_sha256': None, 'contact_validated': False,
                'physical_task_completed': False, 'motor_writes': 0,
                'jaw_policy': 'retain measured current jaw encoder positions'}
            return {**report, 'status': 'PAIRED_KINEMATIC_PROPOSAL_ONLY', 'proposal': proposal,
                    'proposal_sha256': digest(proposal), 'kinematic_results': plans,
                    'note': 'Intentionally invalid as an owner command: no execution op, session, id, scene or collision certificate.'}
        except (OSError, ValueError, KeyError, TypeError, IndexError, Refused) as exc:
            return {**report, 'status': 'FOLDING_PROPOSAL_BLOCKED', 'proposal': None,
                    'blockers': report['blockers'] + [f'Paired IK proposal: {exc}']}
