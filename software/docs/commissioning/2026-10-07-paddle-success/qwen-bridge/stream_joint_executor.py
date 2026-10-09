"""Stream executor for a 10 Hz learned-policy client; runs inside the sole owner (stream mode: on with the pickup profile). Never opens a port.

One executor object lives as long as the owner. Each stream_targets command only updates its targets, so the contact
counters, jaw guard and jaw blocks persist across commands. A stream session ends (holding, nothing released) when no
command arrives for STREAM_TIMEOUT_S, on arm contact, on halt/hold_here, or when a normal motion replaces it.

Rules (see STREAM-MODE.md):
- Targets: the 12 arm motors only (grippers included). Each named arm needs all six joints enabled (owner check).
  A target outside the saved range shrunk by MARGIN, or more than ENVELOPE from the present position, rejects the
  whole command (nothing is clamped, nothing changes).
- A joint whose target is within SKIP_TICKS of its present position or its held goal is skipped: its goal stays where
  it is. A command whose joints are all skipped is accepted as a no-op.
- Each owner loop moves every goal at most STEP ticks toward its target. |present - goal| > ENVELOPE is a fault (the
  owner releases everything), as are the owner's load limits (800 arm / 500 jaw), watchdog and lease.
- Jaws open and close in the stream, at most JAW_STEP ticks of goal change per command. A closing jaw that reads
  |Present_Load| >= JAW_CONTACT_LOAD or lags its goal by >= JAW_CONTACT_LAG is frozen at its present position and
  ignores closing targets until an opening target arrives (jaw_contact reports it). No 'did not become stationary'.
- Arm contact: the paddle executor's contact_halt rule (load >= CONTACT_HALT_LOAD for 2 samples, stalled for 2
  samples, lagging >= CONTACT_PUSH_TICKS) counted across commands. Every streamed joint is held at its present
  position and the stream ends 'contact_halt'; new streams are refused until hold_here, halt, a normal motion or a
  release acknowledges it.
"""
import time
from paddle_joint_executor import ENVELOPE, MARGIN, EDGE, STEP, CONTACT_HALT_LOAD, CONTACT_PUSH_TICKS, tolerance

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
SKIP_TICKS = 2           # a target this close to the present position or held goal changes nothing
JAW_STEP = 10            # max jaw goal change per stream command, opening or closing
JAW_CONTACT_LOAD = 250   # closing jaw at this |Present_Load| is frozen where it is
JAW_CONTACT_LAG = 40     # or lagging its goal by this many ticks
STREAM_TIMEOUT_S = 0.5   # no stream_targets for this long: the stream ends, holding its goals
CONTACT_SAMPLES = 2      # samples loaded and stalled before an arm joint counts as blocked
LIMITS = {'step_ticks_per_loop': STEP, 'envelope_ticks': ENVELOPE, 'margin_ticks': MARGIN, 'skip_ticks': SKIP_TICKS,
          'jaw_step_ticks_per_command': JAW_STEP, 'jaw_contact_load': JAW_CONTACT_LOAD, 'jaw_contact_lag_ticks': JAW_CONTACT_LAG,
          'stream_timeout_s': STREAM_TIMEOUT_S, 'contact_load': CONTACT_HALT_LOAD, 'contact_push_ticks': CONTACT_PUSH_TICKS,
          'contact_samples': CONTACT_SAMPLES}


def is_arm_motor(n):
    return isinstance(n, str) and n.startswith(('left_arm_', 'right_arm_')) and n.split('_arm_', 1)[1] in ARM_JOINTS


class StreamJointExecutor:
    def __init__(self, ranges, write, clock=time.monotonic, wall=time.time):
        self.ranges = ranges; self.write = write; self.clock = clock; self.wall = wall
        self.active = False; self.joints = []; self.goal = {}; self.target = {}
        self.samples = []; self.diagnostics = {}; self.command_id = None
        self.closing = {}         # jaw -> last commanded direction was closing
        self.jaw_blocked = {}     # jaw -> frozen goal; closing targets ignored until an opening target
        self.jaw_contact = {}     # jaw -> {'load', 'position'} reported in status
        self.contact_latched = None  # contact_halt details until acknowledged (hold_here/halt/normal motion/release)

    def reset(self):
        """Motors released: forget jaw blocks and the contact latch (the next enable re-reads every encoder)."""
        self.active = False; self.jaw_blocked = {}; self.jaw_contact = {}; self.closing = {}; self.contact_latched = None

    def forget_jaws(self, names):
        """Another motion moved these jaws: their stream block no longer describes where they are."""
        for n in names:
            self.jaw_blocked.pop(n, None); self.jaw_contact.pop(n, None); self.closing.pop(n, None)

    def command(self, c, current, held_goals, joints):
        """Apply one stream_targets command. Validates everything before changing anything; ValueError rejects."""
        if self.contact_latched:
            raise ValueError('Stream ended by contact_halt on ' + ', '.join(sorted(self.contact_latched)) +
                             '; send hold_here (or halt) to acknowledge before streaming again')
        targets = c.get('targets')
        if not isinstance(targets, dict) or not targets:
            raise ValueError('stream_targets needs a nonempty targets object of arm motor name to integer ticks')
        for n, t in targets.items():
            if not is_arm_motor(n):
                raise ValueError('Stream targets name arm motors only (left_arm_*/right_arm_*): ' + str(n))
            if type(t) is not int:
                raise ValueError('Integer tick target required: ' + n)
            if n not in joints or n not in self.ranges:
                raise ValueError(n + ': not a streamed joint in the owner scope')
        fresh = not self.active
        goal = {} if fresh else dict(self.goal)
        target = {} if fresh else dict(self.target)
        for n in joints:
            if n in goal:
                continue
            q = current[n]; lo, hi = self.ranges[n]
            if not lo + EDGE <= q <= hi - EDGE:
                raise ValueError(n + ': current position outside saved range')
            g = held_goals.get(n, q)
            if type(g) is not int or abs(g - q) > ENVELOPE or not lo + EDGE <= g <= hi - EDGE:
                raise ValueError(n + ': held goal outside envelope')
            goal[n] = target[n] = g
        new = {}; skipped = []; limited = {}; ignored = []
        for n in sorted(targets):
            t = targets[n]; lo, hi = self.ranges[n]; q = current[n]; g = goal[n]
            if not lo + MARGIN <= t <= hi - MARGIN:
                raise ValueError(f'{n}: target {t} outside the commandable range [{lo + MARGIN}, {hi - MARGIN}] '
                                 f'(saved range minus {MARGIN}); rejected, not clamped')
            jaw = n.endswith('gripper')
            if jaw and n in self.jaw_blocked and t < g - SKIP_TICKS:
                ignored.append(n); skipped.append(n); new[n] = g; continue  # closing a blocked jaw: ignored
            if abs(t - q) <= SKIP_TICKS or abs(t - g) <= SKIP_TICKS:
                skipped.append(n); new[n] = g; continue
            effective = g + max(-JAW_STEP, min(JAW_STEP, t - g)) if jaw else t
            if effective != t:
                limited[n] = effective
            if abs(effective - q) > ENVELOPE:
                raise ValueError(f'{n}: target {effective} is {abs(effective - q)} ticks from the present position {q}; '
                                 f'a stream target must be within {ENVELOPE}; rejected')
            new[n] = effective
        no_op = len(skipped) == len(targets)
        result = {'command_id': c.get('id'), 'client_command_id': c.get('command_id'), 'accepted': True, 'no_op': no_op,
                  'skipped_joints': skipped, 'jaw_limited': limited, 'jaw_ignored_closing': ignored}
        if fresh and no_op:
            # Nothing to move and no stream running: do not start one.
            return dict(result, stream_phase='holding', jaw_contact=dict(self.jaw_contact), started=False)
        for n, t in new.items():
            if not n.endswith('gripper') or t == goal[n]:
                continue
            if t > goal[n]:
                self.jaw_blocked.pop(n, None); self.jaw_contact.pop(n, None); self.closing[n] = False
            else:
                self.closing[n] = True
        self.goal = goal; self.target = {**target, **new}; self.joints = sorted(goal)
        now = self.clock(); self.last_command = now; self.command_id = c.get('id')
        if fresh:
            self.active = True; self.started = self.last_tick = now; self.last_sample = None
            self.still = dict.fromkeys(self.joints, 0); self.loaded = dict.fromkeys(self.joints, 0)
            self.last_q = {n: current[n] for n in self.joints}
        else:
            for n in self.joints:
                self.still.setdefault(n, 0); self.loaded.setdefault(n, 0); self.last_q.setdefault(n, current[n])
        return dict(result, stream_phase='streaming', jaw_contact=dict(self.jaw_contact), started=fresh, stream_joints=list(self.joints))

    def started_update(self, current):
        """Owner state for a stream that has just started."""
        return {'accepted': self.command_id, 'phase': 'moving', 'stream_phase': 'streaming', 'execution_profile': 'paddle-success-v1',
                'direct_start_positions': {n: current[n] for n in self.joints}, 'direct_requested_targets': dict(self.target),
                'closure_outcome': None, 'endpoint_reached': None, 'settle_residual_ticks': None, 'contact': None,
                'grasp_verified': False, 'stream_contact_joints': [], 'stream_joints': list(self.joints)}

    def halt(self, current):
        """Stop advancing now and hold the last commanded goals (at most one step ahead of the arm)."""
        self.target = dict(self.goal)
        return self.finish(current, 'halted')

    def finish(self, current, outcome):
        self.active = False
        phase = {'contact_halt': 'contact_halt', 'halted': 'halted'}.get(outcome, 'holding')
        return {'completed': self.command_id, 'phase': 'holding', 'stream_phase': phase, 'closure_outcome': outcome,
                'endpoint_reached': all(abs(current[n] - self.target[n]) <= tolerance(n) for n in self.joints),
                'direct_actual_positions': dict(current), 'settle_residual_ticks': {n: current[n] - self.target[n] for n in self.joints},
                'grasp_verified': False, 'contact': None, 'stream_contact_joints': [], 'jaw_contact': dict(self.jaw_contact),
                'stream_goals': dict(self.goal), 'direct_settle_diagnostics': self.diagnostics}

    def edge(self, n, q):
        lo, hi = self.ranges[n]
        return max(lo + EDGE, min(hi - EDGE, q))

    def tick(self, current, telemetry_at, rows=None):
        now = self.clock(); rows = rows or {}
        if not 0 <= self.wall() - telemetry_at <= 1.0 or not 0 <= now - self.last_tick <= 1.0:
            raise RuntimeError('Stream telemetry/watchdog expired')
        self.last_tick = now
        for n in self.joints:
            if abs(current[n] - self.goal[n]) > ENVELOPE:
                raise RuntimeError('Stream following error exceeds 96 ticks: ' + n)
        if telemetry_at != self.last_sample:
            self.last_sample = telemetry_at
            for n in self.joints:
                q = current[n]; row = rows.get(n, {})
                still = abs(row.get('Present_Velocity', 999)) < 3 and abs(q - self.last_q[n]) <= 3
                self.last_q[n] = q; self.still[n] = self.still[n] + 1 if still else 0
                self.loaded[n] = self.loaded[n] + 1 if abs(row.get('Present_Load', 0)) >= CONTACT_HALT_LOAD else 0
        self.diagnostics = {'stream': True, 'elapsed_s': round(now - self.started, 2), 'since_last_command_s': round(now - self.last_command, 3),
                            'command_id': self.command_id, 'final_targets': dict(self.target),
                            'joints': {n: {'goal_ticks': self.goal[n], 'current_ticks': current[n], 'target_ticks': self.target[n],
                                           'following_error_ticks': current[n] - self.goal[n], 'still_samples': self.still[n],
                                           'loaded_samples': self.loaded[n], 'load': rows.get(n, {}).get('Present_Load')} for n in self.joints},
                            'jaw_blocked': dict(self.jaw_blocked)}
        self.samples.append(dict(self.diagnostics, t=now)); self.samples = self.samples[-128:]
        # Arm contact: loaded, stalled and lagging behind the command. Hold everything where it is; release nothing.
        hits = {n: rows.get(n, {}).get('Present_Load') for n in self.joints
                if not n.endswith('gripper') and self.loaded[n] >= CONTACT_SAMPLES and self.still[n] >= CONTACT_SAMPLES
                and abs(self.goal[n] - current[n]) >= CONTACT_PUSH_TICKS}
        if hits:
            hold = {n: self.edge(n, current[n]) for n in self.joints}
            writes = {n: q for n, q in hold.items() if q != self.goal[n]}
            if writes:
                self.write(writes)
            self.goal.update(hold); self.target.update(hold)
            contact = {n: {'load': hits[n], 'position_ticks': current[n]} for n in hits}
            self.contact_latched = contact
            out = self.finish(current, 'contact_halt')
            out.update(contact=contact, stream_contact_joints=sorted(hits),
                       contact_note=f'Load >= {CONTACT_HALT_LOAD} on a stalled joint lagging >= {CONTACT_PUSH_TICKS} ticks behind its command '
                                    f'(counted across stream commands): every streamed joint holds where it is; nothing was released. '
                                    f'Send hold_here or halt before streaming again.')
            return out
        # Jaw guard: a closing jaw that meets resistance stops where it is and ignores closing until it is opened.
        for n in self.joints:
            if not n.endswith('gripper') or not self.closing.get(n) or n in self.jaw_blocked:
                continue
            load = rows.get(n, {}).get('Present_Load', 0)
            if abs(load) >= JAW_CONTACT_LOAD or current[n] - self.goal[n] >= JAW_CONTACT_LAG:
                lo, hi = self.ranges[n]; q = max(lo + MARGIN, min(hi - MARGIN, current[n]))
                if q != self.goal[n]:
                    self.write({n: q})
                self.goal[n] = self.target[n] = q; self.jaw_blocked[n] = q
                self.jaw_contact[n] = {'load': load, 'position': current[n]}
        if now - self.last_command > STREAM_TIMEOUT_S:
            return self.finish(current, 'stream_timeout')
        writes = {}
        for n in self.joints:
            delta = self.target[n] - self.goal[n]
            if delta:
                self.goal[n] += max(-STEP, min(STEP, delta)); writes[n] = self.goal[n]
        if writes:
            self.write(writes)
        return {'phase': 'moving', 'stream_phase': 'streaming', 'jaw_contact': dict(self.jaw_contact), 'direct_settle_diagnostics': self.diagnostics}

    def pause(self, seconds):
        # Camera pause: nothing advances. The stream timeout is not extended, so a stream left idle by the pause ends
        # holding at the next tick unless new targets arrive (they are refused while the feed is paused).
        self.last_tick = self.clock(); self.last_sample = None
        self.still = dict.fromkeys(self.joints, 0); self.loaded = dict.fromkeys(self.joints, 0)
