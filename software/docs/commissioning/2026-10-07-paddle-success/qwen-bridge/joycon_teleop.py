"""Manual velocity control, run only by HardwareOwner. Never opens a serial port.

Latest-value mailbox, owner/session binding, short input leases, and no automatic
re-arm. Position rates are encoder ticks/s; the base uses the commissioned wheel
signs and 2 cm/s limit. The normal robot API cannot move while this session owns it.
"""
import math
from wheel_pulse_executor import WheelPulseExecutor, WHEELS

INPUT_TTL = .45
MAX_RATE = 100.0
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
SCOPES = {a: [a+'_arm_'+j for j in JOINTS] for a in ('left', 'right')}
SCOPES['both'] = SCOPES['left'] + SCOPES['right']
SCOPES['head'] = ['head_motor_1', 'head_motor_2']
SCOPES['drive'] = []
SCOPES['wholebody'] = SCOPES['both'] + SCOPES['head']


def finite(v, bound):
    return type(v) in (int, float) and math.isfinite(v) and abs(v) <= bound


def validate_input(c, scope):
    if set(c) != {'token', 'sequence', 'session_started', 'received', 'expires', 'rates', 'deadman', 'linear', 'angular'}:
        raise ValueError('Invalid teleop input fields')
    if type(c['sequence']) is not int or c['sequence'] < 1:
        raise ValueError('Invalid sequence')
    rates, dead = c['rates'], c['deadman']
    if not isinstance(rates, dict) or not set(rates) <= set(SCOPES[scope]) or any(not finite(v, MAX_RATE) for v in rates.values()):
        raise ValueError('Invalid joint rates or scope')
    if not isinstance(dead, dict) or set(dead) != {'left', 'right'} or any(type(v) is not bool for v in dead.values()):
        raise ValueError('Two boolean deadman values required')
    if not finite(c['linear'], .02) or not finite(c['angular'], .16):
        raise ValueError('Invalid base rate')
    if scope not in ('drive', 'wholebody') and (c['linear'] or c['angular']):
        raise ValueError('Base rate outside driving mode')
    if c['linear'] or c['angular']:
        WheelPulseExecutor.check_request({'linear_m_s': c['linear'], 'angular_rad_s': c['angular'], 'duration_s': .2})
    for n, rate in rates.items():
        side = 'right' if n.startswith('right_') else 'left'
        if rate and not dead[side]:
            raise ValueError('Joint rate without its deadman')
    if (c['linear'] or c['angular']) and not all(dead.values()):
        raise ValueError('Driving requires both deadmen')


class ManualTeleop:
    def __init__(self, owner):
        self.o = owner
        self.active = False
        self.wheel = None
        self.sequence = 0
        self.last_reason = 'Not armed'
        self.snapshot()

    def snapshot(self):
        self.o.state['teleop'] = {
            'available': True, 'active': self.active,
            'scope': getattr(self, 'scope', None), 'sequence': self.sequence,
            'reason': self.last_reason, 'input_timeout_s': INPUT_TTL,
            'position_rate_limit_ticks_s': MAX_RATE, 'wheel_limit_m_s': .02,
            'head_motors': [n for n in SCOPES['head'] if n in self.o.ranges],
            'upstream_reference_id':getattr(getattr(self.o,'upstream_reference',None),'reference_id',None),
        }
        if self.active:
            self.o.state['teleop'].update(token=self.token, neutral_seen=self.neutral_seen)

    def claim(self, c):
        o = self.o
        scope, token = c.get('scope'), c.get('token')
        if scope not in SCOPES or not isinstance(token, str) or not 20 <= len(token) <= 128:
            raise ValueError('Known scope and unique session token required')
        mode=c.get('control_mode','joint')
        if mode not in ('joint','upstream'):raise ValueError('Unknown manual control mode')
        if mode=='upstream':
            reference=getattr(o,'upstream_reference',None)
            if scope!='wholebody' or reference is None or c.get('reference_id')!=reference.reference_id:
                raise ValueError('Installed measured upstream Joy-Con reference required')
            reference.validate_calibration(o.cal)
        if scope == 'wholebody' and mode!='upstream' and not getattr(o, 'simulation_wholebody', False):
            raise ValueError('Whole-body Cartesian control is currently simulation-only')
        if self.active or o.enabled or (o.engine and o.engine.active) or any(r.get('Torque_Enable') != 0 for r in o.rows.values()):
            raise ValueError('Robot must be fully released before manual control')
        if not o.state.get('ok') or o.read_only or not o.camera_fresh():
            raise ValueError('Healthy owner and fresh supervision camera required')
        names = SCOPES[scope]
        allowed = o.commandable_names | set(SCOPES['head'])
        if not set(names) <= allowed or not set(names) <= set(o.ranges):
            raise ValueError('Requested components unavailable')
        if set(names) & set(o.state.get('calibration_mismatches', {})):
            raise ValueError('Requested component calibration differs from hardware')
        if scope in ('drive', 'wholebody') and len(o.wheel_names) != 2:
            raise ValueError('Both wheels required')
        # An explicit arm action primes measured positions, never a remembered pose.
        try:
            if names: o.enable(names, True, manual=True)
        except Exception:
            o.release_all('Manual enable failed')
            raise
        self.active = True
        self.token, self.scope, self.names = token, scope, names
        self.control_mode=mode
        self.sequence = 0
        self.received = self.last_tick = o.clock()
        self.expires = self.received + INPUT_TTL
        self.neutral_seen = False
        self.input = None
        self.deadman = {'left': False, 'right': False}
        self.targets = {n: float(o.goals[n]) for n in names}
        self.idle_since = o.clock()
        self.wheel = None
        self.last_reason = 'Release both triggers and center both sticks'
        self.snapshot()

    def accept(self, c):
        if not self.active or c.get('token') != self.token or c.get('session_started') != self.o.started:
            return  # old mailboxes can never reactivate a released session
        if c.get('sequence', 0) <= self.sequence:
            return
        validate_input(c, self.scope)
        now = self.o.clock()
        if not finite(c['received'], 1e12) or not finite(c['expires'], 1e12) or not c['received'] <= now <= c['expires'] <= c['received'] + INPUT_TTL + .001:
            raise RuntimeError('Expired manual input')
        neutral = not any(c['deadman'].values()) and not any(c['rates'].values()) and not c['linear'] and not c['angular']
        if not self.neutral_seen:
            if not neutral: raise RuntimeError('Manual session requires neutral input before movement')
            self.neutral_seen = True
        self.sequence = c['sequence']
        self.received, self.expires, self.input = c['received'], c['expires'], c
        self.last_reason = 'Manual control active'
        self.snapshot()

    def tick(self):
        if not self.active: return
        o = self.o
        now = o.clock()
        dt = now - self.last_tick
        if not 0 <= dt <= .35 or now > self.expires:
            raise RuntimeError('Manual input/owner watchdog expired; re-arm required')
        self.last_tick = now
        if not o.camera_fresh(): raise RuntimeError('Manual supervision camera stale; re-arm required')
        o.lease = now + 2
        if self.input is None: return
        c = self.input
        if any(c['deadman'].values()): self.idle_since = now
        if now-self.idle_since > 10: raise RuntimeError('Manual idle timeout; re-arm required')
        for n in self.names:
            side = 'right' if n.startswith('right_') else 'left'
            q = o.rows[n]['Present_Position']
            if abs(q-self.targets[n]) > 80:
                raise RuntimeError(n+': manual following error')
            # On trigger release stop at the measured pose instead of pursuing a previous goal.
            if self.deadman[side] and not c['deadman'][side]: self.targets[n] = float(q)
            rate = c['rates'].get(n, 0) if c['deadman'][side] else 0
            lo, hi = o.ranges[n]
            self.targets[n] = max(lo+4, min(hi-4, self.targets[n] + rate*min(dt, .1)))
        if self.names: o.setpoints({n: round(v) for n, v in self.targets.items()})
        self.deadman = dict(c['deadman'])
        if self.scope in ('drive', 'wholebody'): self.drive(c)
        self.snapshot()

    def drive(self, c):
        o = self.o
        moving = all(c['deadman'].values()) and (c['linear'] or c['angular'])
        if self.wheel and self.wheel.active:
            e = self.wheel
            if e.phase == 'driving':
                if moving:
                    commands = e.check_request({'linear_m_s': c['linear'], 'angular_rad_s': c['angular'], 'duration_s': .2})
                    for n, v in commands.items():
                        if v != e.commands[n]: o.write(n, 'Goal_Velocity', v)
                    e.commands = commands
                    # Extend only while fresh manual input is held. Owner watchdog is independent.
                    e.duration = o.clock()-e.started+.2
                    e.deadline = e.duration+4
                else: e.stopped_early = 'deadman released or stick centered'
            e.tick({n:o.rows[n]['Present_Position'] for n in WHEELS}, min(o.rows[n]['captured_at'] for n in WHEELS), o.rows)
        elif moving:
            e = WheelPulseExecutor(o.read, o.write, o.camera_fresh, clock=o.clock, wall=o.wall)
            self.wheel = e  # attach before the first write, so partially failed startup is aborted
            e.start({'id': self.sequence, 'session_started': o.started, 'linear_m_s': c['linear'], 'angular_rad_s': c['angular'], 'duration_s': .2}, o.rows, o.started)

    def cancel(self, reason):
        self.active = False
        self.last_reason = reason
        errors = self.wheel.abort() if self.wheel is not None and (self.wheel.active or self.wheel.saved) else []
        if not errors:self.wheel = None  # keep failed-release state so a later STOP can retry
        self.snapshot()
        return errors
