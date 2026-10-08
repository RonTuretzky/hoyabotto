"""Robot API client (the chat Mac's paired mTLS certificate) and a fake robot for demos and tests.

Only the existing tool envelope is used: POST /call {"name", "arguments", "request_id"}, a fresh request_id for
every call, and no automatic retry of a call whose outcome is unknown.
"""
import json
import os
import socket
import ssl
import threading
import urllib.error
import urllib.parse
import urllib.request
import uuid

DEFAULT_CONFIG = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json'


class RobotError(RuntimeError):
    """The API answered ok=false (refused or failed), or could not be reached."""


class RobotClient:
    def __init__(self, config_path=None):
        self.config_path = config_path or os.environ.get('XLEROBOT_ADMIN_CONFIG', DEFAULT_CONFIG)
        c = json.loads(open(self.config_path).read())
        self.config = c
        self.ctx = ssl.create_default_context(cafile=c['server_certificate'])
        self.ctx.load_cert_chain(c['client_certificate'], c['client_key'])
        self.lan_ctx = ssl.create_default_context(cafile=c['server_certificate'])
        self.lan_ctx.load_cert_chain(c['client_certificate'], c['client_key'])
        self.lan_ctx.check_hostname = False  # still pinned to the robot's certificate; its SAN lacks the LAN name

    def describe(self):
        return {'config': self.config_path, 'relay_url': self.config['url'], 'lan_url': self.config.get('lan_url')}

    def _base(self):
        """The direct LAN address when it answers (both Macs on one network), else the Cloudflare relay."""
        lan = self.config.get('lan_url')
        if lan:
            u = urllib.parse.urlsplit(lan)
            try:
                with socket.create_connection((u.hostname, u.port or 443), timeout=.8):
                    return lan.rstrip('/'), self.lan_ctx
            except OSError:
                pass
        return self.config['url'].rstrip('/'), self.ctx

    def request(self, path, payload=None, timeout=30):
        base, ctx = self._base()
        req = urllib.request.Request(base + path, data=None if payload is None else json.dumps(payload).encode(),
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, context=ctx, timeout=timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read() or b'{}')
            except ValueError:
                raise RobotError(f'HTTP {e.code} from robot API') from None
        except (OSError, ValueError) as e:
            raise RobotError(f'Robot API unreachable: {e}') from None

    def health(self):
        return self.request('/health', timeout=10)

    def call(self, name, arguments=None, timeout=30):
        body = self.request('/call', {'name': name, 'arguments': arguments or {},
                                      'request_id': 'emoji-' + uuid.uuid4().hex}, timeout=timeout)
        if not body.get('ok'):
            result = body.get('result') or {}
            raise RobotError(f"{name}: {result.get('error') or result.get('reason') or body.get('error') or 'refused'}")
        return body['result']


class FakeRobot:
    """Enough of the owner for the show: one arm's state, enable, paths at the owner's pace, halt and STOP.
    time_scale 0 makes moves instant (tests); 1 runs them in real time (demo without the robot)."""

    def __init__(self, positions=None, ranges=None, time_scale=1.0, busy_motors=()):
        from .gestures import ARM_JOINTS
        names = [f'{a}_arm_{j}' for a in ('left', 'right') for j in ARM_JOINTS]
        self.positions = dict(positions or {n: 2047 for n in names})
        self.ranges = ranges or {n: {'min_ticks': 900, 'max_ticks': 3200, 'margin_ticks': 40} for n in names}
        self.time_scale = time_scale
        self.enabled = set(busy_motors)
        self.phase = 'holding' if self.enabled else 'idle'
        self.calls = []
        self.fail = {}            # tool name -> error message for its next call
        self.stopped = threading.Event()
        self.lock = threading.Lock()

    def describe(self):
        return {'fake': True}

    def health(self):
        return {'ok': True, 'motion_ready': True, 'motor_owner_active': True}

    def call(self, name, arguments=None, timeout=30):
        args = arguments or {}
        with self.lock:
            self.calls.append((name, args))
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
                    'motors': [{'name': n, 'Present_Position': q, 'Torque_Enable': int(n in self.enabled)}
                               for n, q in self.positions.items()],
                    'commandable_ranges': self.ranges}
        if name == 'robot_set_motor_enable':
            if args['enabled']:
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
        if name == 'robot_halt_motion':
            self.phase = 'holding' if self.enabled else 'idle'
            return {'halted': True}
        if name == 'robot_stop':
            self.stopped.set()
            self.enabled.clear()
            self.phase = 'idle'
            return {'release_confirmed': True}
        raise RobotError(f'{name}: not simulated')

