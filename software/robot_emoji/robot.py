"""Robot API client (the chat Mac's paired mTLS certificate) and a fake robot for demos and tests.

Only the existing tool envelope is used: POST /call {"name", "arguments", "request_id"}, a fresh request_id for
every call, and no automatic retry of a call whose outcome is unknown.
"""
import json
import os
import threading
import time
import urllib.error
import urllib.request
import uuid

DEFAULT_CONFIG = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json'


class RobotError(RuntimeError):
    """The API answered ok=false (refused or failed), or could not be reached."""


class RobotTransportError(RobotError):
    """Transport failed; a read-only preflight can wait, but a write must not be replayed."""


class RobotClient:
    """Adapter around the server thread's existing paired mTLS client."""

    def __init__(self, config_path=None, prefer_lan=True):
        import sys
        from pathlib import Path
        self.config_path = config_path or os.environ.get('XLEROBOT_ADMIN_CONFIG', DEFAULT_CONFIG)
        pilot = Path(self.config_path).resolve().parent.parent
        # The handoff client imports its installed farm modules. This checkout can
        # predate those modules, so use the client's canonical farm source first.
        farm_source = pilot.parent / 'farm-live/software'
        if farm_source.exists():
            if str(farm_source) in sys.path:
                sys.path.remove(str(farm_source))
            sys.path.insert(0, str(farm_source))
        if str(pilot) not in sys.path:
            sys.path.insert(0, str(pilot))
        from chat_server import Robot
        if prefer_lan:
            self.client = Robot(Path(self.config_path))
        else:
            class InternetRobot(Robot):
                def lan_reachable(self, lan_url):
                    return False
            self.client = InternetRobot(Path(self.config_path))

    def describe(self):
        return {'paired_client': True, 'link': self.client.link}

    def health(self):
        try:
            return self.client.get('/health')
        except (OSError, ValueError) as exc:
            raise RobotTransportError(f'Robot API unreachable: {type(exc).__name__}') from None

    def call(self, name, arguments=None, timeout=30):
        try:
            body = self.client.call(name, arguments or {}, request_id='emoji-' + uuid.uuid4().hex)
        except (OSError, ValueError) as exc:
            # Never retry a robot command after an uncertain transport outcome.
            raise RobotTransportError(f'{name}: robot API unreachable ({type(exc).__name__})') from None
        if not body.get('ok'):
            # The API wraps the owner's own reply, so its reason can sit one level down.
            reason, layer = None, body
            for _ in range(3):
                if not isinstance(layer, dict):
                    break
                reason = reason or layer.get('error') or layer.get('reason')
                layer = layer.get('result')
            raise RobotError(f"{name}: {reason or 'refused'}")
        return body['result']


    # Robot-Mac administration (the same /admin API as robot_admin.py, same pinned certificates).
    def admin(self, path, payload=None, timeout=20):
        base, context = self.client.settings()
        request = urllib.request.Request(base + path, data=None if payload is None else json.dumps(payload).encode(),
                                         headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, context=context, timeout=timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read() or b'{}')

    def restart_owner(self, log=print, wait_s=300):
        """Full restart; if the owner cannot start (e.g. a dead board also carries the wheels), restart without the
        wheels so the arm on the working board still performs."""
        if self._restart(log, wait_s, 'restart'):
            return True
        log('full restart failed; restarting without the wheels so a single working arm can still perform')
        return self._restart(log, wait_s, 'restart-no-wheels')

    def _restart(self, log, wait_s, mode):
        """Restart the robot Mac's hardware owner and API on the version it already runs (motors come up released;
        the deploy refuses while any motor is held). True once the owner reports motion_ready again."""
        try:
            record = self.admin('/admin/deploy')['deploy']
            head = record.get('head') or record['record']['head']
            started = self.admin('/admin/deploy', {'ref': head, 'mode': mode})
        except (OSError, ValueError, KeyError) as e:
            log(f'robot restart could not start: {type(e).__name__}: {e}')
            return False
        if not started.get('ok'):
            log(f"robot restart refused: {started.get('error') or started}")
            return False
        job_id = started['job']['id']
        log(f'robot {mode} job {job_id} started on {head[:7]}')
        deadline = time.time() + wait_s
        state = 'running'
        while time.time() < deadline and state == 'running':
            time.sleep(3)
            try:
                state = self.admin(f'/admin/job?id={job_id}')['job'].get('state', 'running')
            except (OSError, ValueError, KeyError):
                continue            # the API restarts during the job
        if state != 'succeeded':
            log(f'robot restart job ended {state}')
            return False
        while time.time() < deadline:
            try:
                binding = self.health().get('execution_binding', {})
                if binding.get('motion_ready') is True and not binding.get('blockers'):
                    log('robot restarted: owner motion_ready, motors released')
                    return True
            except RobotError:
                pass
            time.sleep(3)
        log('robot restarted but the owner did not report motion_ready in time')
        return False


class FakeRobot:
    """Enough of the owner for the show: one arm's state, enable, paths at the owner's pace, halt and STOP.
    time_scale 0 makes moves instant (tests); 1 runs them in real time (demo without the robot)."""

    def __init__(self, positions=None, ranges=None, time_scale=1.0, busy_motors=()):
        from .gestures import ARM_JOINTS
        names = [f'{a}_arm_{j}' for a in ('left', 'right') for j in ARM_JOINTS]
        default_positions = {n: 2047 for n in names}
        default_positions.update(head_motor_1=2085, head_motor_2=2600)
        self.positions = dict(positions or default_positions)
        default_ranges = {n: {'min_ticks': 900, 'max_ticks': 3200, 'margin_ticks': 40} for n in names}
        default_ranges.update(head_motor_1={'min_ticks':1059,'max_ticks':3111,'margin_ticks':40},
                              head_motor_2={'min_ticks':1972,'max_ticks':2625,'margin_ticks':40})
        self.ranges = ranges or default_ranges
        self.time_scale = time_scale
        self.enabled = set(busy_motors)
        self.phase = 'holding' if self.enabled else 'idle'
        self.calls = []
        self.fail = {}            # tool name -> error message for its next call
        self.fail_always = {}     # tool name -> error message for every call (until restart_owner)
        self.fail_when = []       # (tool name, predicate(args), message): fail matching calls
        self.status = {}          # motor name -> fault Status reported by robot_get_state
        self.stopped = threading.Event()
        self.lock = threading.Lock()

    def describe(self):
        return {'fake': True}

    def restart_owner(self, log=print, wait_s=0):
        with self.lock:
            self.calls.append(('restart_owner', {}))
            self.fail.clear()
            if getattr(self, 'restart_result', True):
                self.fail_always.clear(); self.fail_when.clear()
            self.enabled.clear(); self.phase = 'idle'
        return getattr(self, 'restart_result', True)

    def health(self):
        return {'ok': True, 'motion_ready': True, 'motor_owner_active': True}

    def call(self, name, arguments=None, timeout=30):
        args = arguments or {}
        with self.lock:
            self.calls.append((name, args))
            for tool, matches, message in self.fail_when:
                if tool == name and matches(args):
                    raise RobotError(f'{name}: {message}')
            if name in self.fail_always:
                raise RobotError(f'{name}: {self.fail_always[name]}')
            if name in self.fail:
                message = self.fail.pop(name)
                if name == 'robot_move_path' and message.startswith('Owner stopped'):
                    self.enabled.clear(); self.phase = 'idle'   # an owner fault releases everything
                raise RobotError(f'{name}: {message}')
        if name == 'robot_get_motion':
            return {'phase': self.phase, 'moving': self.phase == 'moving', 'running_command_id': None,
                    'enabled_motors': sorted(self.enabled), 'base_drive_phase': 'released'}
        if name == 'robot_get_state':
            return {'source': 'canonical_hardware_owner', 'enabled_motors': sorted(self.enabled),
                    'motors': [{'name': n, 'Present_Position': q, 'Torque_Enable': int(n in self.enabled), 'Status': self.status.get(n, 0)}
                               for n, q in self.positions.items()],
                    'commandable_ranges': self.ranges}
        if name == 'robot_set_motor_enable':
            if args['enabled']:
                self.speed_profile = args.get('speed_profile', 'normal')
                self.stopped.clear()
                self.enabled |= set(args['names'])
                self.phase = 'holding'
            else:
                self.enabled -= set(args['names'])
                self.phase = 'holding' if self.enabled else 'idle'
            return {'ok': True, 'enabled_motors': sorted(self.enabled)}
        if name == 'robot_move_path':
            arm = args['arm']
            if not {f'{arm}_arm_{j}' for j in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')} <= self.enabled:
                raise RobotError('robot_move_path: all six joints of the arm must be enabled')
            self.phase = 'moving'
            for w in args['waypoints']:
                for n, q in w.items():
                    lo, hi = self.ranges[n]['min_ticks'], self.ranges[n]['max_ticks']
                    if not lo <= q <= hi:
                        raise RobotError(f'robot_move_path: target out of bounds {n}={q}')
            if self.stopped.wait(args['duration_s'] * self.time_scale):
                raise RobotError('robot_move_path: Owner stopped: STOP requested')
            for w in args['waypoints']:
                self.positions.update(w)
            self.phase = 'holding'
            return {'accepted': True, 'completed': True, 'endpoint_reached': True, 'closure_outcome': 'endpoint_settled'}
        if name == 'robot_move_head':
            points = args['positions']
            if not set(points) <= self.enabled:
                raise RobotError('Head motors must be enabled')
            travel = max(abs(q - self.positions[n]) for n,q in points.items())
            if travel > 200 or args['duration_s'] < travel / 100:
                raise RobotError('Bounded head limits exceeded')
            self.phase = 'moving'
            if self.stopped.wait(args['duration_s'] * self.time_scale):
                raise RobotError('Owner stopped: STOP requested')
            self.positions.update(points)
            self.phase = 'holding'
            return {'completed': True, 'endpoint_reached': True, 'closure_outcome': 'endpoint_settled'}
        if name == 'robot_halt_motion':
            self.phase = 'holding' if self.enabled else 'idle'
            return {'halted': True}
        if name == 'robot_stop':
            self.stopped.set()
            self.enabled.clear()
            self.phase = 'idle'
            return {'release_confirmed': True}
        raise RobotError(f'{name}: not simulated')
