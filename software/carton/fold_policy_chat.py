"""Fold-policy tools for the chat Mac's pilot (chat_server.py), as a Robot wrapper like CalibrationRobot.

The pilot is not in this repo. tools/install_fold_policy_chat.py adds one line that loads this file by path and wraps
the pilot's robot chain: chat=Chat(FoldPolicyRobot(CalibrationRobot(...))). Standard library only: it runs inside the
chat server's process, whose `farm` package is an older checkout (farm-live), so it imports nothing from this repo.

Tools it adds to the catalog:

- robot_get_fold_policy_status (read-only): configuration, blockers, the running job and the last runs.
- robot_fold_policy_dry_run: runs carton.fold_policy_runner WITHOUT --execute: it reads the owner and the cameras
  (robot_get_execution, robot_get_cameras), runs the policy and logs what it would send. No motor command.
- robot_fold_policy_run: the same runner WITH --execute. Refused unless the owner's config file (not a tool argument)
  enables execution, the named operator is listed there, the step count is within its cap, and a clean dry run with
  the same checkpoint, joint maps and cameras finished recently.

Each run is a subprocess of the chat server (the repo's runner, the configured Python, PYTHONPATH = this repo's
software/), started with --parent-pid so it halts if the chat server dies. The tool call blocks until the run ends.
While it runs, the shared local motion lock (tag-calibration.lock) is held and every other motion tool through this
wrapper is refused; reads and STOP pass. robot_stop first interrupts the runner (SIGINT: it halts, holding, and
releases nothing), then goes to the robot as always (the owner releases every motor). The runner never calls
robot_stop itself.

Configuration: <pilot>/.private/fold-policy.json, written by the installer and edited by the owner only:

    {"schema": 1, "software_root": ".../software", "python": ".../.venv/bin/python", "pilot_root": ".../pilot",
     "checkpoint": ".../pretrained_model", "joint_maps": {"left": "...json", "right": "...json"},
     "cameras": {"front": "oak", "left_wrist": "left_wrist", "right_wrist": "right_wrist"},
     "runs_dir": ".../fold-policy-runs", "device": "cpu", "gripper_mode": "hold",
     "dry_run_max_steps": 300, "dry_run_valid_s": 900,
     "execute_enabled": false, "execute_max_steps": 10, "operators": [], "blockers": ["..."]}
"""
from __future__ import annotations

import copy
import fcntl
import hashlib
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

STATUS = 'robot_get_fold_policy_status'
DRY_RUN = 'robot_fold_policy_dry_run'
RUN = 'robot_fold_policy_run'
TOOLS = (STATUS, DRY_RUN, RUN)
READ_PREFIXES = ('robot_get_', 'robot_list_')
CONFIG_NAME = 'fold-policy.json'
LOCK_NAME = 'tag-calibration.lock'          # shared with the tag calibration wrapper (carton.servo.tag_calibration)
STARTUP_S = 180.0                           # policy load + warm-up before the first tick (CPU, cold)
HALT_GRACE_S = 15.0                         # after SIGINT: the runner halts and writes its summary
INFORMATIONAL = ('Dry-run with motors enabled',)   # preflight warnings that do not make a dry run unclean

STATUS_DESCRIPTION = (
    'Read the learned carton-fold policy setup: whether execution is enabled, the blockers the owner listed, the '
    'checkpoint, joint maps and camera mapping, the running fold job if any, and the last dry run and run with their '
    'summaries and warnings. No motor commands.')
DRY_RUN_DESCRIPTION = (
    'Dry-run the learned carton-fold policy (ACT, trained only in simulation) on the live robot: reads joints and the '
    'head and wrist cameras at 10 Hz, runs the policy and logs every target it WOULD send. Sends no motor command; '
    'the arms should be released and supported, or holding. Use when the user asks to check the fold policy. Reports '
    'preflight warnings (start pose outside training, camera aspect, degenerate joints), aborts, effective rate and '
    'clamp counts; a run is clean only with no warnings and no abort. Blocks until the dry run ends.')
RUN_DESCRIPTION = (
    'Execute the learned carton-fold policy on the robot: streams the policy\'s joint targets to both arms at 10 Hz '
    'for at most the owner\'s configured step cap. Use only when the user asks for it in this conversation, after a '
    'clean dry run, with all twelve arm motors enabled and holding at the training start pose, the named operator at '
    'STOP and nobody in the arms\' sweep. Before calling, tell the user which arms will move and for how long. The '
    'owner\'s config decides whether execution is enabled, who may operate and the step cap; tool arguments cannot '
    'change them. Stops on any abort and ends HOLDING (releases nothing); STOP releases. Never retry a refused or '
    'aborted run on your own.')
NO_ARGS = {'type': 'object', 'properties': {}, 'additionalProperties': False}
DRY_RUN_PARAMS = {'type': 'object', 'properties': {
    'max_steps': {'type': 'integer', 'minimum': 1, 'maximum': 3000,
                  'description': 'ticks at 10 Hz (default 50 = 5 s); capped by the owner config'}},
    'additionalProperties': False}
RUN_PARAMS = {'type': 'object', 'properties': {
    'operator': {'type': 'string', 'minLength': 1, 'maxLength': 60,
                 'description': 'name of the person holding STOP, as listed in the owner config'},
    'max_steps': {'type': 'integer', 'minimum': 1, 'maximum': 3000,
                  'description': 'ticks at 10 Hz; must not exceed the owner config cap'}},
    'required': ['operator', 'max_steps'], 'additionalProperties': False}


class Refused(Exception):
    pass


def _base_robot(robot):
    """The pilot's Robot at the bottom of the wrapper chain (it carries the robot.json path as .config)."""
    seen = set()
    while id(robot) not in seen:
        seen.add(id(robot))
        if isinstance(getattr(robot, 'config', None), (str, Path)):
            return robot
        robot = getattr(robot, 'robot', robot)
    raise Refused('No pilot Robot with a .config path below this wrapper')


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def _read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


class FoldPolicyRobot:
    def __init__(self, robot, config=None, *, clock=time.time, popen=subprocess.Popen):
        self.robot = robot
        self.config_path = Path(config) if config else Path(_base_robot(robot).config).with_name(CONFIG_NAME)
        self.lock_path = self.config_path.with_name(LOCK_NAME)
        self.state_path = self.config_path.with_name('fold-policy-state.json')
        self.clock, self.popen = clock, popen
        self.job = None                      # {'process', 'kind', 'run_dir', 'started'}
        self.guard = threading.Lock()

    # ------------------------------------------------------------------ pass-through
    def __getattr__(self, name):              # get(), settings(), ... of the wrapped chain
        return getattr(self.robot, name)

    def catalog(self):
        catalog = copy.deepcopy(self.robot.catalog())
        if not self.config_path.exists():
            return catalog
        existing = {t['function']['name'] for t in catalog['tools']}
        if existing & set(TOOLS):
            raise Refused('Fold-policy tool name is already supplied by the server')
        for name, description, parameters in ((STATUS, STATUS_DESCRIPTION, NO_ARGS),
                                              (DRY_RUN, DRY_RUN_DESCRIPTION, DRY_RUN_PARAMS),
                                              (RUN, RUN_DESCRIPTION, RUN_PARAMS)):
            catalog['tools'].append({'type': 'function', 'function': {
                'name': name, 'description': description, 'parameters': parameters}})
        return catalog

    def call(self, name, args=None, request_id=None):
        args = args or {}
        if name in TOOLS:
            try:
                if name == STATUS:
                    return {'ok': True, 'result': self.status()}
                cfg = self.settings_file()
                if name == DRY_RUN:
                    return self.start_and_wait(cfg, execute=False, max_steps=self._dry_steps(cfg, args))
                return self.start_and_wait(cfg, execute=True, **self._run_gate(cfg, args))
            except Refused as exc:
                return {'ok': False, 'result': {'error': str(exc), 'motor_writes': 0, 'automatic_retry': False}}
        if name == 'robot_stop':
            self.interrupt('robot_stop')
            return self._forward(name, args, request_id)
        if self.running() and not name.startswith(READ_PREFIXES):
            return {'ok': False, 'result': {'error': f'A fold-policy {self.job["kind"]} is running; {name} is refused '
                                                     'until it ends (STOP is always available)',
                                            'motor_writes': 0, 'automatic_retry': False}}
        return self._forward(name, args, request_id)

    def _forward(self, name, args, request_id):
        if request_id is None:
            return self.robot.call(name, args)
        return self.robot.call(name, args, request_id=request_id)

    # ------------------------------------------------------------------ configuration and gates
    def settings_file(self):
        cfg = _read_json(self.config_path)
        if not isinstance(cfg, dict) or cfg.get('schema') != 1:
            raise Refused(f'{self.config_path} is missing or not schema 1 (tools/install_fold_policy_chat.py writes it)')
        missing = [k for k in ('software_root', 'python', 'pilot_root', 'checkpoint', 'joint_maps', 'cameras', 'runs_dir')
                   if not cfg.get(k)]
        if missing:
            raise Refused(f'Fold-policy config lacks {missing}')
        return cfg

    def _dry_steps(self, cfg, args):
        steps = int(args.get('max_steps', 50))
        cap = int(cfg.get('dry_run_max_steps', 300))
        if not 1 <= steps <= cap:
            raise Refused(f'Dry run max_steps must be 1..{cap} (owner config)')
        return steps

    def identity(self, cfg):
        """What a dry run vouches for: the same checkpoint weights, joint maps and camera mapping."""
        maps = {arm: _sha256(path) for arm, path in sorted(cfg['joint_maps'].items())}
        return {'model_sha256': _sha256(Path(cfg['checkpoint']) / 'model.safetensors'), 'joint_maps': maps,
                'cameras': dict(sorted(cfg['cameras'].items()))}

    def _run_gate(self, cfg, args):
        problems = []
        if cfg.get('execute_enabled') is not True:
            problems.append('execution is disabled in the owner config (execute_enabled false)')
        for blocker in cfg.get('blockers') or []:
            problems.append(f'open blocker: {blocker}')
        operator = str(args.get('operator') or '').strip()
        if operator not in (cfg.get('operators') or []):
            problems.append(f'operator {operator!r} is not listed in the owner config')
        steps = args.get('max_steps')
        cap = int(cfg.get('execute_max_steps', 0))
        if type(steps) is not int or not 1 <= steps <= cap:
            problems.append(f'max_steps must be 1..{cap} (owner config)')
        last = (_read_json(self.state_path) or {}).get('last_dry_run') or {}
        age = self.clock() - float(last.get('ended') or 0)
        if not last:
            problems.append('no dry run on record')
        else:
            if last.get('clean') is not True:
                problems.append(f'last dry run was not clean: {last.get("aborted") or last.get("warnings")}')
            if age > float(cfg.get('dry_run_valid_s', 900)):
                problems.append(f'last dry run is {age:.0f} s old (limit {cfg.get("dry_run_valid_s", 900)} s)')
            try:
                if last.get('identity') != self.identity(cfg):
                    problems.append('checkpoint, joint maps or cameras changed since the last dry run')
            except OSError as exc:
                problems.append(f'cannot hash the configured files: {exc}')
        if problems:
            raise Refused('Fold policy run refused: ' + '; '.join(problems))
        return {'max_steps': steps, 'operator': operator}

    # ------------------------------------------------------------------ jobs
    def running(self):
        return self.job is not None and self.job['process'].poll() is None

    def command(self, cfg, run_dir, *, execute, max_steps, operator=None):
        cmd = [cfg['python'], '-m', cfg.get('runner_module', 'carton.fold_policy_runner'),
               '--checkpoint', str(cfg['checkpoint']), '--transport', cfg.get('transport', 'api'),
               '--pilot-root', str(cfg['pilot_root']), '--out', str(run_dir), '--max-steps', str(max_steps),
               '--gripper-mode', cfg.get('gripper_mode', 'hold'), '--device', cfg.get('device', 'cpu'),
               '--parent-pid', str(os.getpid())]
        for arm, path in sorted(cfg['joint_maps'].items()):
            cmd += ['--joint-map', f'{arm}={path}']
        for key, camera in sorted(cfg['cameras'].items()):
            cmd += ['--camera', f'{key}={camera}']
        if execute:
            cmd += ['--execute', '--operator', operator]
        return cmd

    def start_and_wait(self, cfg, *, execute, max_steps, operator=None):
        kind = 'run' if execute else 'dry run'
        identity = self.identity(cfg)
        stamp = time.time()
        run_dir = Path(cfg['runs_dir']) / (time.strftime('%Y%m%d-%H%M%S', time.localtime(stamp))
                                           + f'.{int(stamp * 1000) % 1000:03d}-{"run" if execute else "dry"}')
        run_dir.parent.mkdir(parents=True, exist_ok=True)
        cmd = self.command(cfg, run_dir, execute=execute, max_steps=max_steps, operator=operator)
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(
            [str(cfg['software_root'])] + [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]))
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.lock_path, 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise Refused('Another local motion client or calibration is active') from None
            try:
                with self.guard:
                    if self.running():
                        raise Refused(f'A fold-policy {self.job["kind"]} is already running')
                    log = open(run_dir.parent / f'{run_dir.name}.log', 'w')
                    process = self.popen(cmd, cwd=str(cfg['software_root']), env=env, stdout=log,
                                         stderr=subprocess.STDOUT)
                    log.close()
                    self.job = {'process': process, 'kind': kind, 'run_dir': str(run_dir), 'started': self.clock(),
                                'operator': operator, 'max_steps': max_steps}
                timeout = STARTUP_S + max_steps / 10.0 * 1.5
                try:
                    code = process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    self.interrupt(f'tool timeout {timeout:.0f} s')
                    code = process.wait()
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)
        return {'ok': code == 0, 'result': self.record(kind, run_dir, code, identity, operator)}

    def interrupt(self, reason):
        """SIGINT the running job: the runner halts (holds every joint, releases nothing) and writes its summary."""
        job = self.job
        if job is None or job['process'].poll() is not None:
            return False
        job['interrupted'] = reason
        job['process'].send_signal(signal.SIGINT)
        threading.Thread(target=self._reap, args=(job,), daemon=True).start()
        return True

    def _reap(self, job):
        try:
            job['process'].wait(timeout=HALT_GRACE_S)
        except subprocess.TimeoutExpired:
            job['process'].kill()

    def record(self, kind, run_dir, code, identity, operator):
        summary = _read_json(run_dir / 'summary.json') or {}
        preflight = _read_json(run_dir / 'preflight.json') or {}
        warnings = list(preflight.get('warnings') or [])
        interrupted = (self.job or {}).get('interrupted')
        aborted = summary.get('aborted') or (None if code == 0 else (
            f'interrupted ({interrupted}) before the runner started its loop' if interrupted and not summary
            else f'runner exited with code {code} (see the log)'))
        entry = {'kind': kind, 'run_dir': str(run_dir), 'log': str(run_dir.parent / f'{run_dir.name}.log'),
                 'exit_code': code, 'ended': self.clock(), 'operator': operator,
                 'interrupted': interrupted,
                 'aborted': aborted, 'warnings': warnings, 'steps': summary.get('steps'),
                 'effective_hz': summary.get('effective_hz'), 'commands_sent': summary.get('commands_sent'),
                 'clamp_counts': summary.get('clamp_counts'), 'end': summary.get('end'),
                 'start_outside_training': preflight.get('start_outside_training'),
                 'clean': code == 0 and not aborted and not [w for w in warnings if not w.startswith(INFORMATIONAL)],
                 'identity': identity}
        state = _read_json(self.state_path) or {}
        state['last_dry_run' if kind == 'dry run' else 'last_run'] = entry
        tmp = self.state_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(state, indent=1))
        tmp.replace(self.state_path)
        self.job = None
        if kind == 'run':
            entry['note'] = ('The arms are HOLDING where the run ended (nothing was released). Support them before '
                             'robot_stop releases them.')
        return {k: v for k, v in entry.items() if k != 'identity'}

    def status(self):
        cfg = _read_json(self.config_path)
        state = _read_json(self.state_path) or {}
        out = {'config_file': str(self.config_path), 'configured': isinstance(cfg, dict)}
        if isinstance(cfg, dict):
            out.update({k: cfg.get(k) for k in ('checkpoint', 'joint_maps', 'cameras', 'device', 'gripper_mode',
                                                'execute_enabled', 'execute_max_steps', 'dry_run_max_steps',
                                                'operators', 'blockers')})
        job = self.job
        out['running'] = None if not self.running() else {
            'kind': job['kind'], 'run_dir': job['run_dir'], 'elapsed_s': round(self.clock() - job['started'], 1),
            'operator': job.get('operator'), 'max_steps': job.get('max_steps')}
        for key in ('last_dry_run', 'last_run'):
            entry = state.get(key)
            out[key] = None if entry is None else {k: v for k, v in entry.items() if k != 'identity'}
        return out
