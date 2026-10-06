"""Opt-in Gemma tools for automatic tag calibration through its current client."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import time

from carton.servo.common import Refused
from carton.servo.tag_calibration import motion_lock, readiness, run_calibration

STATUS = 'robot_calibration_status'
RUN = 'robot_calibrate_tags'


class CalibrationRobot:
    def __init__(self, robot, config=None):
        self.robot = robot
        raw = getattr(robot, 'robot', robot)
        self.config_path = Path(config) if config else Path(raw.config).with_name('tag-calibration.json')
        self.last_catalog = None

    def settings(self):
        cfg = json.loads(self.config_path.read_text())
        if cfg.get('schema') != 1 or cfg.get('arm') not in ('left', 'right') or not cfg.get('joints'):
            raise Refused('Need a local calibration config with explicit arm and positioning joints')
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
            if STATUS in existing or RUN in existing:
                raise Refused('Calibration tool name is already supplied by the server')
            for name, description, parameters in [
                (STATUS, 'Read current tag-calibration readiness and tag visibility. No motor commands. '
                 'Reports exact blockers and tag-2 border clearance; a small margin is not proof of clipping.',
                 {'type': 'object', 'properties': {}, 'additionalProperties': False}),
                (RUN, 'Perform the CURRENT user-requested automatic calibration through the existing owner. '
                 'Enables only the configured arm positioning motors, makes small observed joint movements, '
                 'then releases. local_model reuses bidirectional probes and independent visual-model checks; '
                 'registration collects eight fit and three held-out poses then runs the existing hand-eye fitter. '
                 'Use only after fresh readiness and with the operator supervising the cleared workspace. '
                 'Stops on changed state or a refusal; never resets STOP, starts another owner or retries a failed run. '
                 'This does not grasp an object or install a Cartesian transform. Paths, arm, joints and limits '
                 'are local configuration and cannot be changed by tool arguments.',
                 {'type': 'object', 'properties': {'mode': {'type': 'string', 'enum': ['local_model', 'registration']}},
                  'required': ['mode'], 'additionalProperties': False})]:
                catalog['tools'].append({'type': 'function', 'function': {'name': name, 'description': description, 'parameters': parameters}})
        self.last_catalog = catalog
        return catalog

    def call(self, name, args, request_id=None):
        if name in (STATUS, RUN):
            try:
                cfg = self.settings()
                if not isinstance(args, dict) or set(args) != (set() if name == STATUS else {'mode'}):
                    raise Refused('Unexpected calibration arguments')
                if name == STATUS:
                    return {'ok': True, 'result': readiness(self.robot, cfg), 'motor_writes': 0}
                if args['mode'] not in ('local_model', 'registration'):
                    raise Refused('Unknown calibration mode')
                output = Path(cfg['output_root'])/f'{time.time_ns()}-{args["mode"]}'
                return {'ok': True, 'result': run_calibration(self.robot, cfg, args['mode'], output)}
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
