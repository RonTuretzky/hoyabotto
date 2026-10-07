"""Opt-in Gemma tools for automatic tag calibration through its current client."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import time

from carton.servo.common import Refused, atomic_json
from carton.servo.tag_calibration import motion_lock, readiness, run_calibration
from farm.perception.registered_tags import read_registered_tags
from farm.perception.tag_sampling import gripper_tag_for_arm

STATUS = 'robot_calibration_status'
RUN = 'robot_calibrate_tags'
REGISTERED = 'robot_get_registered_tags'


class CalibrationRobot:
    def __init__(self, robot, config=None, *, clock=time.time):
        self.robot = robot
        raw = getattr(robot, 'robot', robot)
        self.config_path = Path(config) if config else Path(raw.config).with_name('tag-calibration.json')
        self.last_catalog = None
        self.clock = clock
        self.registration_path = self.config_path.with_name('tag-registration.json')
        self.folding_path = self.config_path.with_name('folding-readiness.json')

    def settings(self):
        cfg = json.loads(self.config_path.read_text())
        if cfg.get('schema') != 1 or cfg.get('arm') not in ('left', 'right') or not cfg.get('joints'):
            raise Refused('Need a local calibration config with explicit arm and positioning joints')
        cfg['gripper_tag_id'] = gripper_tag_for_arm(cfg['arm'], cfg.get('gripper_tag_id'))
        cfg['lock_file'] = str(self.config_path.with_name('tag-calibration.lock'))
        cfg['output_root'] = str(self.config_path.parent.parent/'tag-calibration-runs')
        return cfg

    def get(self, path):
        return self.robot.get(path)

    def catalog(self):
        catalog = copy.deepcopy(self.robot.catalog())
        if self.config_path.exists():
            self.settings()
            existing = {t['function']['name'] for t in catalog['tools']}
            if {STATUS, RUN, REGISTERED} & existing:
                raise Refused('Calibration tool name is already supplied by the server')
            for name, description, parameters in [
                (STATUS, 'Read current tag-calibration readiness and tag visibility. No motor commands. '
                 'Reports exact blockers and the configured gripper tag border clearance; a small margin is not proof of clipping.',
                 {'type': 'object', 'properties': {}, 'additionalProperties': False}),
                (REGISTERED, 'Read fresh AprilTag poses in the configured arm base using a passing registration. '
                 'Rechecks camera stream, table anchor, fixed gripper mount, model, motor mapping and current '
                 'camera/encoder agreement. Refuses missing or changed calibration. Read-only marker estimates; '
                 'these are not jaw contact targets or permission to grasp. No motor commands.',
                 {'type': 'object', 'properties': {}, 'additionalProperties': False}),
                (RUN, 'Perform the CURRENT user-requested automatic calibration through the existing owner. '
                 'Enables only the configured arm positioning motors, makes small observed joint movements, '
                 'then releases. local_model reuses bidirectional probes and independent visual-model checks; '
                 'registration collects eight fit and three held-out poses then runs the existing hand-eye fitter. '
                 'Use only after fresh readiness and with the operator supervising the cleared workspace. '
                 'Stops on changed state or a refusal; never resets STOP, starts another owner or retries a failed run. '
                 'A passing registration is saved for read-only robot_get_registered_tags. It does not grasp '
                 'an object or enable Cartesian control. Paths, arm, joints and limits '
                 'are local configuration and cannot be changed by tool arguments.',
                 {'type': 'object', 'properties': {'mode': {'type': 'string', 'enum': ['local_model', 'registration']}},
                  'required': ['mode'], 'additionalProperties': False})]:
                catalog['tools'].append({'type': 'function', 'function': {'name': name, 'description': description, 'parameters': parameters}})
        if self.folding_path.exists():
            from carton.servo.folding_readiness import tool_definitions
            additions = tool_definitions()
            names = {t['function']['name'] for t in catalog['tools']}
            if any(t['function']['name'] in names for t in additions):
                raise Refused('Folding tool name is already supplied by the server')
            catalog['tools'].extend(additions)
        self.last_catalog = catalog
        return catalog

    def call(self, name, args, request_id=None):
        if name in ('robot_folding_status', 'robot_folding_proposal'):
            from carton.servo.folding_readiness import FoldingReadiness, STATUS as FOLDING_STATUS
            try:
                if not self.folding_path.exists():
                    raise Refused('Local folding-readiness.json is not configured')
                if name == FOLDING_STATUS and args != {}:
                    raise Refused('Unexpected folding status arguments')
                folding = FoldingReadiness(self.robot, self.folding_path, clock=self.clock)
                outcome = folding.status() if name == FOLDING_STATUS else folding.proposal(args)
                return {'ok': True, 'result': outcome, 'motor_writes': 0}
            except (Refused, ValueError, OSError, KeyError, TypeError) as exc:
                return {'ok': False, 'result': {'error': str(exc), 'motion_ready': False,
                        'execution_available': False, 'motor_writes': 0, 'automatic_retry': False}}
        if name in (STATUS, RUN, REGISTERED):
            try:
                cfg = self.settings()
                if not isinstance(args, dict) or set(args) != ({'mode'} if name == RUN else set()):
                    raise Refused('Unexpected calibration arguments')
                if name == STATUS:
                    return {'ok': True, 'result': readiness(self.robot, cfg, clock=self.clock), 'motor_writes': 0}
                if name == REGISTERED:
                    return read_registered_tags(self.robot, cfg, json.loads(self.registration_path.read_text()), clock=self.clock)
                if args['mode'] not in ('local_model', 'registration'):
                    raise Refused('Unknown calibration mode')
                output = Path(cfg['output_root'])/f'{time.time_ns()}-{args["mode"]}'
                outcome = run_calibration(self.robot, cfg, args['mode'], output, clock=self.clock)
                if outcome.get('status') == 'REGISTRATION_VALIDATED':
                    atomic_json(self.registration_path, outcome)
                return {'ok': True, 'result': outcome}
            except (Refused, ValueError, OSError, KeyError) as exc:
                return {'ok': False, 'result': {'error': str(exc), 'automatic_retry': False}}
        # Ordinary read tools and independent STOP remain responsive during a run.
        if name == 'robot_stop' or name.startswith(('robot_get_', 'robot_list_')):
            return self._forward(name, args, request_id)
        with motion_lock(self.config_path.with_name('tag-calibration.lock')):
            return self._forward(name, args, request_id)

    def _forward(self, name, args, request_id):
        if request_id is None:
            return self.robot.call(name, args)
        return self.robot.call(name, args, request_id=request_id)
