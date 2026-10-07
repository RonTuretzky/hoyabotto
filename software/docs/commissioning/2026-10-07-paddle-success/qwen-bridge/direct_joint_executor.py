"""Guarded raw joint control; saved travel limits, no Cartesian commissioning.

Called only inside the existing sole owner's health/camera/STOP loop.
It never opens serial devices or enables torque.
"""
import math
import time


class DirectJointExecutor:
    def __init__(self, joints, ranges, write, clock=time.monotonic, wall=time.time):
        self.joints = list(joints)
        self.ranges = ranges
        self.write = write
        self.clock = clock
        self.wall = wall
        self.active = False
        self.previous = {}
        self.diagnostics = {}
        self.samples = []

    def start(self, command, current, *, session_started):
        if self.active:
            raise ValueError('Previous direct command is still moving')
        if type(command.get('id')) is not int or command['id'] <= 0:
            raise ValueError('Positive integer command ID required')
        if command.get('session_started') != session_started:
            raise ValueError('Direct command belongs to another owner session')
        target = command.get('positions')
        if not isinstance(target, dict) or not target or not set(target) <= set(self.joints):
            raise ValueError('Target names must belong to this selected owner')
        if any(type(value) is not int for value in target.values()):
            raise ValueError('Targets must be integer encoder ticks')
        duration = command.get('duration_s')
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 < duration <= 25:
            raise ValueError('Finite positive duration no longer than25seconds required')
        start = {n: current[n] for n in self.joints}
        end = {**start, **target}
        for n in self.joints:
            lo, hi = self.ranges[n]
            if not lo+4 <= start[n] <= hi-4 or not lo+4 <= end[n] <= hi-4:
                raise ValueError(n + ': current or target position leaves saved range margins')
        changed = [n for n in target if abs(end[n]-start[n]) > 2]
        if not changed:
            raise ValueError('No requested movement greater than2ticks')
        distance = max(abs(end[n]-start[n]) for n in self.joints)
        # Analytic quintic extrema: no overshoot. Stretch rather than clip rates.
        duration = max(float(duration), 1.875*distance/100,
                       math.sqrt(5.773503*distance/200))
        if duration > 25:
            raise ValueError('Target needs more than25seconds at existing safe rates')
        self.start_positions, self.target = start, end
        self.changed, self.duration = changed, duration
        self.command_id = command['id']
        self.started = self.last_tick = self.clock()
        self.previous = dict(start)
        self.stable = 0
        self.samples = []
        self.last_sample = None
        self.active = True
        return {'accepted': self.command_id, 'phase': 'moving',
                'direct_duration_s': duration, 'direct_requested_targets': target,
                'direct_start_positions': start, 'direct_changed_joints': changed,
                'direct_settle_tolerances_ticks': {n: max(1,min(5,abs(end[n]-start[n])//10)) for n in changed},
                'direct_deadline_s': duration+4, 'direct_settle_samples_required': 3,
                'direct_commanded_degrees': sum(abs(end[n]-start[n])*360/4095
                    for n in changed if not n.endswith('gripper'))}

    def tick(self, current, *, telemetry_at):
        if not self.active:
            raise RuntimeError('No active direct command')
        now = self.clock()
        if type(telemetry_at) not in (int, float) or not math.isfinite(telemetry_at) or not 0 <= self.wall()-telemetry_at <= .2:
            raise RuntimeError('Direct encoder telemetry is stale')
        if not 0 <= now-self.last_tick <= .2:
            raise RuntimeError('Direct motor-owner watchdog expired')
        elapsed = now-self.started
        self.diagnostics = self.settle_diagnostics(current, elapsed, telemetry_at)
        self.diagnostics['previous_commanded_goals_ticks'] = dict(self.previous)
        self.diagnostics['following_errors_ticks'] = {n:current[n]-self.previous[n] for n in self.joints}
        self.samples.append(self.diagnostics)
        self.samples = self.samples[-128:]
        if any(abs(current[n]-self.previous[n]) > 24 for n in self.joints):
            raise RuntimeError('Direct encoder following error exceeds24ticks')
        if elapsed > self.duration+4:
            raise RuntimeError('Direct target did not settle before deadline; blockers: '+', '.join(self.diagnostics['blocking_joints']))
        u = min(1., max(0., elapsed/self.duration))
        blend = u*u*u*(10+u*(-15+6*u))
        goals = {n: round(self.start_positions[n]+blend*(self.target[n]-self.start_positions[n]))
                 for n in self.joints}
        if any(abs(goals[n]-self.previous[n]) > 68 for n in self.joints):
            raise RuntimeError('Direct interpolation step exceeds68ticks')
        self.write(goals)
        self.previous, self.last_tick = goals, now
        qualified = elapsed >= self.duration
        for n in self.changed:
            distance = abs(self.target[n]-self.start_positions[n])
            tolerance = max(1, min(5, distance//10))
            sign = 1 if self.target[n] > self.start_positions[n] else -1
            qualified = qualified and abs(current[n]-self.target[n]) <= tolerance
            qualified = qualified and (current[n]-self.start_positions[n])*sign >= distance-tolerance
        if telemetry_at != self.last_sample:
            self.stable = self.stable+1 if qualified else 0
            self.last_sample = telemetry_at
        if self.stable >= 3:
            self.active = False
            return {'completed': self.command_id, 'phase': 'holding',
                    'direct_actual_positions': dict(current), 'direct_elapsed_s': elapsed}
        return {'phase': 'moving', 'direct_elapsed_s': elapsed, 'direct_settle_diagnostics': self.diagnostics}

    def settle_diagnostics(self, current, elapsed, telemetry_at):
        joints = {}
        for n in self.changed:
            distance = abs(self.target[n]-self.start_positions[n])
            tolerance = max(1,min(5,distance//10))
            sign = 1 if self.target[n]>self.start_positions[n] else -1
            travel = (current[n]-self.start_positions[n])*sign
            error = current[n]-self.target[n]
            joints[n] = {'start_ticks':self.start_positions[n], 'target_ticks':self.target[n],
                'current_ticks':current[n], 'endpoint_error_ticks':error, 'tolerance_ticks':tolerance,
                'directed_travel_ticks':travel, 'required_travel_ticks':distance-tolerance,
                'endpoint_ok':abs(error)<=tolerance, 'travel_ok':travel>=distance-tolerance}
        return {'elapsed_s':elapsed, 'deadline_s':self.duration+4, 'telemetry_at':telemetry_at,
            'duration_elapsed':elapsed>=self.duration, 'stable_samples_before_tick':self.stable,
            'joints':joints, 'blocking_joints':[n for n,d in joints.items() if not d['endpoint_ok'] or not d['travel_ok']]}
