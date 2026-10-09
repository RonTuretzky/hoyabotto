"""One performance: a visitor's gestures on one arm, from rest back to rest.

  preflight   robot_get_motion + robot_get_state: nobody else may hold a motor or be moving (Busy otherwise;
              nothing was written), and every target must be inside the live commandable range.
  enable      robot_set_motor_enable for all six joints of the arm (the pickup profile holds them where they are),
              with the gestures' speed_profile (demo = 300 ticks/s) when it is not normal.
  paths       robot_move_path, wait=true: each gesture's raise, then its motion; finally the return to the
              resting pose (clamped into the commandable range).
  release     robot_set_motor_enable(enabled=false) for those six joints only, at the resting pose.

If anything fails while the arm is still held, the arm is driven home and released; if that fails too, robot_stop
(soft release of every motor). After an owner fault or a STOP the owner has already released everything, so
nothing more is sent. A STOP from the operator page goes straight to robot_stop from the web thread; the
performance sees `aborted` and sends nothing further.
"""
import threading
import time

from . import gestures as G
from .robot import RobotError, RobotTransportError


class Busy(Exception):
    """The robot is in use by someone else; nothing was sent. Try again later."""


class PerformError(Exception):
    """moved=False: refused before any motor was enabled, so the request can safely run again."""

    def __init__(self, message, moved=True):
        super().__init__(message)
        self.moved = moved


ARM_ORDER = ('right', 'left')   # gestures are written for the right arm; the left arm is the mirror-image fallback
AVOID_S = 600                   # after an arm fails, prefer the other one for this long


def arm_problem(state, arm):
    """Why an arm cannot perform now (None when every joint reports a position and no fault)."""
    rows = {r.get('name'): r for r in state.get('motors', [])}
    for n in G.arm_motors(arm):
        r = rows.get(n)
        if r is None or type(r.get('Present_Position')) is not int:
            return f'{n} not reporting'
        if r.get('Status'):
            return f'{n} fault status {r.get("Status")}'
    return None


def _positions(state):
    if 'motors' in state:
        return {m['name']: m.get('Present_Position') for m in state['motors']}
    return {n: r.get('Present_Position') for n, r in (state.get('live_rows') or {}).items()}


class Performer:
    def __init__(self, robot, catalog, log=print):
        self.robot = robot
        self.catalog = catalog
        self.log = log
        self.aborted = threading.Event()
        self.avoid = {}           # arm -> time until which it is skipped after a failure
        self.clock = time.time

    def preflight(self, keys, arm=None):
        unknown = [k for k in keys if k not in self.catalog]
        if unknown or not keys:
            raise PerformError(f'Unknown gestures: {unknown}')
        motion = self.robot.call('robot_get_motion')
        holding = motion.get('enabled_motors') or []
        base_phase = motion.get('base_drive_phase')
        idle = motion.get('phase') == 'idle' and motion.get('moving') is False and not motion.get('running_command_id') and not holding
        if holding or motion.get('moving') is True or motion.get('phase') == 'moving' or motion.get('running_command_id') or (base_phase not in (None, 'released') and not idle):
            raise Busy(f"robot in use (phase {motion.get('phase')}, holding {len(holding)} motors)")
        if base_phase not in (None, 'released'):
            # The paired API reports progress of the running OR LAST motion. A
            # wheel abort releases the wheels without clearing that historical
            # subphase. Confirm fresh physical release and stationary encoders
            # before accepting an otherwise explicitly idle owner.
            samples = []
            for i in range(2):
                if i:
                    time.sleep(.15)
                state = self.robot.call('robot_get_state', {'fresh': True})
                rows = {r['name']: r for r in state.get('motors', [])}
                wheels = [rows.get(n) for n in ('base_left_wheel', 'base_right_wheel')]
                if state.get('all_16_released') is not True or state.get('cached') is not False or state.get('enabled_motors') or any(
                    not r or r.get('Torque_Enable') != 0 or r.get('Status') != 0 or type(r.get('Present_Position')) is not int for r in wheels
                ):
                    raise Busy(f'base phase {base_phase}: fresh wheel release is not confirmed')
                samples.append([r['Present_Position'] for r in wheels])
            if any(abs((after - before + 2048) % 4096 - 2048) > 2 for before, after in zip(*samples)):
                raise Busy(f'base phase {base_phase}: wheel encoders are still changing')
            self.log(f'verified idle owner, all motors released and wheel encoders stationary (last base phase: {base_phase})')
        else:
            state = self.robot.call('robot_get_state')
        gestures = [self.catalog[k] for k in keys]
        if arm is None and gestures[0].arm != 'head':
            arm = self.choose_arm(state)
        self.last_arm = arm
        try:
            return G.plan(gestures, _positions(state), state['commandable_ranges'], arm=arm)
        except (G.GestureError, KeyError) as e:
            raise PerformError(f'Plan refused before any motion ({arm or gestures[0].arm} arm): {e}', moved=False) from None

    def choose_arm(self, state):
        """The first arm in ARM_ORDER that reports healthy and has not failed recently; else any healthy arm."""
        now = self.clock()
        healthy = [a for a in ARM_ORDER if arm_problem(state, a) is None]
        for arm in healthy:
            if self.avoid.get(arm, 0) <= now:
                return arm
        if healthy:
            return healthy[0]
        raise PerformError('No arm is available: ' + '; '.join(f'{a}: {arm_problem(state, a)}' for a in ARM_ORDER), moved=False)

    def perform_any(self, keys, on_phase=lambda phase, detail=None: None):
        """perform(), and if the chosen arm is refused before anything moved, the same gestures on the other arm."""
        try:
            return self.perform(keys, on_phase)
        except PerformError as e:
            failed = getattr(e, 'arm', None)
            if e.moved or failed is None or self.aborted.is_set():
                raise
            self.avoid[failed] = self.clock() + AVOID_S
            other = next(a for a in ARM_ORDER if a != failed)
            self.log(f'{failed} arm refused before moving ({e}); trying the {other} arm')
            return self.perform(keys, on_phase, arm=other)

    def perform(self, keys, on_phase=lambda phase, detail=None: None, arm=None):
        """Runs the whole performance; returns a summary. Raises Busy (nothing sent) or PerformError."""
        if self.aborted.is_set():
            raise PerformError('Stopped by the operator; re-arm the show before another performance')
        self.last_arm = arm
        try:
            plan = self.preflight(keys, arm)
        except PerformError as e:
            e.arm = self.last_arm
            raise
        try:
            return self._perform(plan, on_phase)
        except PerformError as e:
            e.arm = plan['arm']
            if not e.moved:
                self.avoid[plan['arm']] = self.clock() + AVOID_S
            raise

    def _perform(self, plan, on_phase):
        motors = plan['motors']
        if self.aborted.is_set():
            raise PerformError('Stopped by the operator')
        on_phase('enable')
        try:
            enable = {'names': motors, 'enabled': True}
            if plan.get('speed_profile', 'normal') != 'normal':
                enable['speed_profile'] = plan['speed_profile']   # chosen before torque-on, as the owner requires
            self.robot.call('robot_set_motor_enable', enable)
        except RobotTransportError as e:
            if not self.aborted.is_set():
                self._recover(plan, e)
            raise PerformError('Motor-enable outcome was unknown; recovery attempted and show paused') from None
        except RobotError as e:
            raise PerformError(f'Enable refused (nothing moved, {plan["arm"]} arm): {e}', moved=False) from None
        summary = {'arm': plan['arm'], 'paths': [], 'home': plan['home']}
        try:
            for path in plan['paths']:
                if self.aborted.is_set():
                    raise PerformError('Stopped by the operator')
                on_phase(path['part'], path['gesture'])
                result = self._move_path(plan, path)
                outcome = result.get('closure_outcome')
                summary['paths'].append({'part': path['part'], 'gesture': path['gesture'], 'completed': result.get('completed'),
                                         'outcome': outcome, 'residual': result.get('settle_residual_ticks')})
                if result.get('completed') is not True and outcome != 'settled_short':
                    raise PerformError(f"{path['part']} did not complete: {outcome or result}")
                if outcome == 'settled_short':
                    self.log(f"{path['part']}: settled short {result.get('settle_residual_ticks')}; motors holding, continuing")
            on_phase('release')
            self.robot.call('robot_set_motor_enable', {'names': motors, 'enabled': False})
            return summary
        except (RobotError, PerformError) as e:
            if self.aborted.is_set():
                raise PerformError('Stopped by the operator') from None
            self._recover(plan, e)
            raise PerformError(str(e)) from None

    def _move_path(self, plan, path, minimum_duration_s=0):
        if plan['arm'] != 'head':
            return self.robot.call('robot_move_path', {'arm': plan['arm'], 'waypoints': path['waypoints'],
                'duration_s': path['duration_s'], 'wait': True}, timeout=path['duration_s'] + 60)
        result = {'completed': True, 'closure_outcome': 'already_at_target'}
        for waypoint in path['waypoints']:
            if self.aborted.is_set():
                raise PerformError('Stopped by the operator')
            current = _positions(self.robot.call('robot_get_state', {'fresh': True}))
            if any(type(current.get(n)) is not int for n in waypoint):
                raise PerformError('Fresh head encoder readings are missing')
            positions = {n:q for n,q in waypoint.items() if abs(q - current[n]) >= 3}
            if not positions:
                continue
            travel = max(abs(q - current[n]) for n,q in positions.items())
            if travel > G.HEAD_MAX_LEG_TICKS:
                raise PerformError('Head moved since planning; bounded head step refused')
            duration = max(travel / 100, G.path_seconds([positions], current), minimum_duration_s)
            if self.aborted.is_set():
                raise PerformError('Stopped by the operator')
            result = self.robot.call('robot_move_head', {'positions': positions, 'duration_s': duration})
            if result.get('completed') is not True and result.get('closure_outcome') != 'settled_short':
                raise PerformError('Head step did not complete')
        return result

    def _recover(self, plan, error):
        """After a failure: if the owner still holds our arm, take it home and release it; else nothing to do."""
        self.log(f'performance failed: {error}')
        try:
            motion = self.robot.call('robot_get_motion')
        except RobotError as e:
            self.log(f'cannot read the owner after the failure ({e}); sending robot_stop')
            self._stop()
            return
        ours = set(plan['motors']) & set(motion.get('enabled_motors') or [])
        if not ours:
            self.log('owner already released the arm (fault or STOP); nothing more sent')
            return
        try:
            if motion.get('phase') == 'moving':
                self.robot.call('robot_halt_motion')
            home = plan['paths'][-1]
            self._move_path(plan, dict(home, duration_s=max(home['duration_s'], 20)), minimum_duration_s=20)
            self.robot.call('robot_set_motor_enable', {'names': plan['motors'], 'enabled': False})
            self.log('arm returned home and released after the failure')
        except (RobotError, PerformError) as e:
            self.log(f'return home failed ({e}); sending robot_stop')
            self._stop()

    def _stop(self):
        try:
            self.robot.call('robot_stop', timeout=15)
        except RobotError as e:
            self.log(f'robot_stop failed: {e}. Use the 12 V switch if the arm is still powered.')

    def stop(self):
        """Operator STOP: releases every motor through the owner (soft release) and ends the performance."""
        self.aborted.set()
        return self.robot.call('robot_stop', timeout=15)
