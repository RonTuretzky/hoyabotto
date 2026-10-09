"""Robot API client (the chat Mac's paired mTLS certificate) and a fake robot for demos and tests.

Only the existing tool envelope is used: POST /call {"name", "arguments", "request_id"}, a fresh request_id for
every call, and no automatic retry of a call whose outcome is unknown.
"""
import json
import os
import threading
import uuid

DEFAULT_CONFIG = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json'


class RobotError(RuntimeError):
    """The API answered ok=false (refused or failed), or could not be reached."""


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
            raise RobotError(f'Robot API unreachable: {type(exc).__name__}') from None

    def call(self, name, arguments=None, timeout=30):
        try:
            body = self.client.call(name, arguments or {}, request_id='emoji-' + uuid.uuid4().hex)
        except (OSError, ValueError) as exc:
            # Never retry a robot command after an uncertain transport outcome.
            raise RobotError(f'{name}: robot API unreachable ({type(exc).__name__})') from None
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
