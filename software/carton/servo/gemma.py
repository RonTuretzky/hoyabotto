"""Adapters for the existing Gemma HTTPS owner and same-frame AprilTags.

No device connection, owner startup, STOP reset or limit changes. The caller
must serialize this experiment with other motion clients (see calibration.py).

Contract with today's robot server (qwen-bridge, paddle-success-v1 owner; checked by
qwen-bridge/test_tag_registration_contract.py):
- every move needs all six motors of the arm enabled, so the jaw is enabled and held, never moved;
- each commanded joint must travel 3..341 ticks (the server answers a <=2-tick target with a no-op);
- robot_get_execution carries the owner's own 16 telemetry rows, so one call gives a consistent
  phase/marker/encoder snapshot (fewer relay round trips between a camera frame and the next command);
- there is no STOP latch: a STOP or owner fault returns the owner to idle with motors released and
  bumps stop_count; STOP and faults ease torque off over about 2 s before release is confirmed.
"""
from __future__ import annotations

import base64
import time

import numpy as np

from .common import Limits, Observation, Refused, atomic_json, finite
from farm.perception.tag_sampling import ARM_JOINTS, HEAD_JOINTS, gripper_tag_for_arm, stationary_sample

MIN_STEP_TICKS = 3        # owner: each joint in a move travels 3..341 ticks; <=2 is answered as a no-op
MOVE_DURATION_S = 0.4     # one <=16-tick step; the pickup owner ramps at most 40 ticks per >=0.4 s interval
RELEASE_CONFIRM_S = 8.0   # STOP/fault release eases torque off over ~2 s; keep reading this long for torque-zero


def result(payload, tool):
    if not isinstance(payload, dict) or payload.get('ok') is not True or not isinstance(payload.get('result'), dict):
        raise Refused(f'{tool} refused: {str(payload)[:700]}')
    return payload['result']


class GemmaTransport:
    """One-joint steps through robot_move_motor_targets, with measured readback."""
    min_step_ticks = MIN_STEP_TICKS

    def __init__(self, robot, arm, joints, limits=None, *, execute=False, clock=time.time):
        if arm not in ('left', 'right'):
            raise Refused('Choose an explicit arm')
        self.robot, self.arm, self.clock = robot, arm, clock
        self.limits, self.execute = limits or Limits(), execute
        self.arm_names = [f'{arm}_arm_{n}' for n in ARM_JOINTS]
        self.position_names = self.arm_names[:-1]
        # The pickup-profile owner refuses any move unless all six motors of the arm are enabled.
        # The jaw is therefore enabled and held where it is; it is never a calibration joint.
        self.hold_names = list(self.arm_names)
        self.joints = [n if n.startswith(f'{arm}_arm_') else f'{arm}_arm_{n}' for n in joints]
        if not self.joints or len(set(self.joints)) != len(self.joints) or any(n not in self.position_names for n in self.joints):
            raise Refused('Select distinct positioning joints of the confirmed arm; no jaws, head or wheels')
        self.started = self.origin = self.ranges = self.last_marker = None
        self.path_ticks = self.commands_sent = 0
        self.write_attempted = self.enabled = False
        self.deadline = clock() + self.limits.max_seconds
        self.last_state = self.last_execution = None
        self.raw_ranges = self.observed_q = self.observed_at = None
        self.cleanup = None

    def _execution(self):
        s = result(self.robot.call('robot_get_execution', {}), 'robot_get_execution')
        stamp, started = finite(s.get('time'), 'owner timestamp'), finite(s.get('started'), 'owner session')
        if not 0 <= self.clock()-stamp <= self.limits.status_age_s:
            raise Refused('Motor owner status is stale or host clocks differ')
        if self.started is not None and started != self.started:
            raise Refused('Motor owner restarted during calibration')
        if s.get('control_mode') != 'direct_joint' or s.get('ok') is not True:
            raise Refused('Need the existing healthy direct-joint owner')
        if s.get('phase') not in ('idle', 'holding') or s.get('stop_latched') is not False or s.get('operator_armed') is not True:
            raise Refused(f'Owner is not stationary and armed: {s.get("root_failure") or s.get("phase")}')
        # No STOP latch: a STOP or fault shows up only as a new stop record and a released, idle owner.
        marker = (s.get('accepted'), s.get('completed'), s.get('motor_writes'), s.get('stop_count'))
        if self.last_marker is not None and marker != self.last_marker:
            raise Refused(f'Another command, STOP or motor write occurred during calibration: {s.get("last_stop")}')
        self.started, self.last_marker, self.last_execution = started, marker, s
        return s

    def _rows(self, rows):
        """Validate one owner snapshot of all sixteen motors; return measured ticks."""
        if len(rows) != 16 or len({r.get('name') for r in rows}) != 16:
            raise Refused('Need distinct telemetry for all sixteen motors')
        q = {}
        for row in rows:
            name = row['name']
            if not 0 <= self.clock()-finite(row.get('captured_at'), 'encoder timestamp') <= self.limits.status_age_s:
                raise Refused(f'Motor telemetry is stale or host clocks differ: {name}')
            if (type(row.get('Present_Position')) is not int or row.get('Status') != 0
                    or row.get('Moving') != 0 or abs(finite(row.get('Present_Velocity'))) > 1
                    or abs(finite(row.get('Present_Load'))) >= self.limits.load_raw):
                raise Refused(f'Unsettled, stale or faulty motor telemetry: {name}')
            expected_enable = 1 if self.enabled and name in self.hold_names else 0
            if row.get('Torque_Enable') != expected_enable:
                raise Refused(f'Unexpected motor enable state: {name}')
            q[name] = row['Present_Position']
        if any(n not in q for n in self.arm_names + list(HEAD_JOINTS)):
            raise Refused('Missing selected arm/head encoders')
        if self.ranges is not None:
            for n in self.hold_names:
                if not self.ranges[n]['min_ticks'] <= q[n] <= self.ranges[n]['max_ticks']:
                    raise Refused(f'{n}: {q[n]} outside commandable range {self.ranges[n]["min_ticks"]}..{self.ranges[n]["max_ticks"]}')
        if self.origin is not None:
            for n in q:
                bound = self.limits.trust_ticks + self.limits.settle_ticks if n in self.joints else 3
                if abs(q[n]-self.origin[n]) > bound:
                    raise Refused(f'{n}: uncommanded drift or local envelope exceeded')
        if self.observed_q is not None and any(abs(q[n]-self.observed_q[n]) > 3 for n in q):
            raise Refused('Robot moved after the camera observation')
        return q

    def read_state(self):
        payload = self.robot.call('robot_get_state', {'fresh': True})
        state = result(payload, 'robot_get_state')
        if state.get('cached') is not False:
            raise Refused('Need uncached encoder telemetry')
        ranges = state.get('commandable_ranges')
        if not isinstance(ranges, dict) or any(n not in ranges for n in self.arm_names):
            raise Refused('Missing commandable ranges')
        if self.ranges is not None and ranges != self.ranges:
            raise Refused('Commandable ranges changed during calibration')
        raw_ranges = state.get('raw_calibration_ranges')
        if not isinstance(raw_ranges, dict) or (self.raw_ranges is not None and raw_ranges != self.raw_ranges):
            raise Refused('Saved calibration ranges missing or changed')
        self.ranges, self.raw_ranges = ranges, raw_ranges
        q = self._rows(state.get('motors', []))
        self.last_state = payload
        return payload, q

    def status(self):
        """One robot_get_execution call: owner phase, command marker and its own 16 telemetry rows together."""
        if self.clock() > self.deadline:
            raise Refused('Calibration time budget exhausted')
        if self.ranges is None:
            self.read_state()
        s = self._execution()
        rows = s.get('rows')
        if not isinstance(rows, dict) or s.get('hardware_server') is not True:
            raise Refused('Owner execution status lacks its own telemetry rows')
        q = self._rows([dict(row, name=name) for name, row in rows.items()])
        if s['phase'] != ('holding' if self.enabled else 'idle'):
            raise Refused('Owner phase disagrees with the verified motor enable state')
        if sorted(s.get('enabled_motors') or []) != (sorted(self.hold_names) if self.enabled else []):
            raise Refused(f'Owner enabled motors differ from this calibration: {s.get("enabled_motors")}')
        if self.origin is None:
            self.origin = q.copy()
        return s, q

    def positions(self):
        return self.status()[1]

    def preflight(self):
        caps = result(self.robot.call('robot_get_capabilities', {}), 'robot_get_capabilities')
        if caps.get('motion_units') not in ('raw_encoder_ticks', 'encoder_ticks; positioning-joint degrees use4095 ticks/rev'):
            raise Refused('Owner does not advertise raw encoder ticks')
        if caps.get('motion_ready') is not True:
            raise Refused(f'Owner is not motion-ready: {caps.get("blockers")}')
        blockers = {n: v for n, v in caps.get('joint_blockers', {}).items() if n in self.hold_names and v}
        if blockers:
            raise Refused(f'Selected arm blockers: {blockers}')
        if self.ranges is None:
            self.read_state()
        s, q = self.status()
        if s.get('enabled_motors'):
            raise Refused('Calibration must start with all motors released and other motion clients idle')
        return {'arm': self.arm, 'joints': self.joints,
                # Experiment.align expects raw ranges and applies its own 4 tick margin.
                'ranges': {n: [v['min_ticks']-4, v['max_ticks']+4] for n, v in self.ranges.items()},
                'origin': q, 'owner_started': self.started, 'commandable_ranges': self.ranges,
                'execution_profile': s.get('execution_profile')}

    def _acknowledge(self, payload, tool):
        ack = result(payload, tool)
        if ack.get('accepted') is not True or ack.get('completed') is not True:
            outcome = ack.get('closure_outcome') or ack.get('reason')
            raise Refused(f'{tool}: no measured completion from the bound owner ({outcome}); readbacks {ack.get("readbacks")}')
        if ack.get('owner_started') != self.started or type(ack.get('command_id')) is not int:
            raise Refused(f'{tool}: missing measured completion from the bound owner')
        if not 0 <= self.clock()-finite(ack.get('owner_status_time'), 'completion timestamp') <= self.limits.status_age_s:
            raise Refused('Command completion is stale')
        self.last_marker = None
        state, q = self.status()
        if state.get('completed') != ack['command_id']:
            raise Refused('Owner completion was replaced by another command')
        return ack, q

    def enable(self):
        if not self.execute:
            raise Refused('Read-only calibration cannot enable motors')
        _, q = self.status()
        if any(abs(q[n]-self.origin[n]) > 3 for n in q):
            raise Refused('Robot moved from the observed starting pose before enabling')
        self.write_attempted = True
        payload = self.robot.call('robot_set_motor_enable', {'names': self.hold_names, 'enabled': True})
        ack = result(payload, 'robot_set_motor_enable')
        if ack.get('readbacks') != {n: 1 for n in self.hold_names}:
            raise Refused(f'Enable did not confirm exactly the six motors of the selected arm: {str(ack)[:500]}')
        self.enabled = True
        self._acknowledge(payload, 'robot_set_motor_enable')

    def move(self, joint, ticks):
        if not self.execute or not self.enabled:
            raise Refused('Motor execution is disabled')
        if joint not in self.joints or type(ticks) is not int or not self.min_step_ticks <= abs(ticks) <= 68:
            raise Refused(f'Invalid single positioning-joint step (owner moves {self.min_step_ticks}..68 ticks per command)')
        s, before = self.status()
        goal = before[joint] + ticks
        bounds = self.ranges[joint]
        if (not bounds['min_ticks'] <= goal <= bounds['max_ticks']
                or abs(goal-self.origin[joint]) > self.limits.trust_ticks
                or self.path_ticks+abs(ticks) > self.limits.max_path_ticks):
            raise Refused('Movement exceeds commandable range or calibration travel budget')
        if finite(s.get('lease_remaining'), 'lease remaining') <= self.limits.command_timeout_s:
            raise Refused('Insufficient owner lease; calibration will not renew an expired lease')
        if self.observed_at is None or not 0 <= self.clock()-self.observed_at <= self.limits.frame_age_s:
            raise Refused('Camera observation expired before motor dispatch')
        start = self.clock()
        self.commands_sent += 1
        self.path_ticks += abs(ticks)
        self.observed_q = self.observed_at = None
        ack, after = self._acknowledge(self.robot.call('robot_move_motor_targets',
            {'positions': {joint: goal}, 'duration_s': MOVE_DURATION_S}), 'robot_move_motor_targets')
        if self.clock()-start > self.limits.command_timeout_s:
            raise Refused('Motor command exceeded calibration timeout')
        if abs(after[joint]-goal) > self.limits.settle_ticks:
            raise Refused(f'Command completed but measured endpoint misses calibration tolerance: {joint} {after[joint]} vs {goal}')
        if any(abs(after[n]-before[n]) > 3 for n in before if n != joint):
            raise Refused('Uncommanded motor drifted during the step')
        if joint not in ack.get('readbacks', {}) or abs(ack['readbacks'][joint]-after[joint]) > self.limits.settle_ticks:
            raise Refused('Completion and fresh encoder readback disagree')
        return after, self.clock()

    def _released(self, rows):
        return len(rows) == 16 and all(r.get('Torque_Enable') == 0 and 0 <= self.clock()-finite(r.get('captured_at')) <= self.limits.status_age_s
                                       for r in rows)

    def finish(self, failed=False):
        if not self.write_attempted:
            return
        if failed:
            # STOP releases everything; the owner eases torque off over ~2 s first. Confirm from fresh reads.
            self.enabled = False
            payload = self.robot.call('robot_stop', {})
            stop = payload.get('result') if isinstance(payload, dict) and isinstance(payload.get('result'), dict) else {}
            confirmed = payload.get('ok') is True and stop.get('release_confirmed') is True
            deadline = self.clock() + RELEASE_CONFIRM_S
            for _ in range(40):
                if confirmed or self.clock() > deadline:
                    break
                try:
                    state = result(self.robot.call('robot_get_state', {'fresh': True}), 'robot_get_state')
                    confirmed = state.get('cached') is False and not state.get('enabled_motors') and self._released(state.get('motors', []))
                except (Refused, ValueError, KeyError, TypeError):
                    pass
            self.cleanup = {'release_confirmed': confirmed, 'stop': {k: stop.get(k) for k in (
                'stop_requested', 'release_confirmed', 'release_reason', 'release', 'owner_phase', 'release_errors') if k in stop},
                'owner_started': self.started}
            return
        payload = self.robot.call('robot_set_motor_enable', {'names': self.hold_names, 'enabled': False})
        self.enabled = False
        self._acknowledge(payload, 'robot_set_motor_enable')
        # Releasing may allow gravity movement; require fresh torque-zero, not a pose hold.
        state = result(self.robot.call('robot_get_state', {'fresh': True}), 'robot_get_state')
        if state.get('cached') is not False or not self._released(state.get('motors', [])) or len({r.get('name') for r in state['motors']}) != 16:
            raise Refused('Fresh all-sixteen release confirmation missing')
        self.cleanup = {'release_confirmed': True, 'owner_started': self.started}


class GemmaTagObserver:
    """Expose decoded tag corners to Experiment; save same-frame encoder brackets."""
    def __init__(self, robot, transport, camera='oak', *, gripper_tag_id=None, clock=time.time):
        self.robot, self.transport, self.camera, self.clock = robot, transport, camera, clock
        self.gripper_tag_id = gripper_tag_for_arm(transport.arm, gripper_tag_id)
        self.last = self.anchor = self.identity = None
        self.capture = self.payload = None
        self.count = 0

    def observe(self, after=0.0):
        before, _ = self.transport.read_state()
        payload = self.robot.call('robot_get_tags', {'cameras': [self.camera], 'tag_ids': [1, self.gripper_tag_id]})
        row = result(payload, 'robot_get_tags').get('observations', {}).get(self.camera, {})
        following, _ = self.transport.read_state()
        frame = row.get('frame', {})
        stamp = finite(frame.get('captured_at'), 'camera capture time')
        # On a link faster than the owner's poll the second read can repeat the first snapshot; read again
        # (bounded) until the owner has sampled every motor after the frame. No limit is relaxed.
        for _ in range(4):
            if min(r['captured_at'] for r in following['result']['motors']) >= stamp:
                break
            following, _ = self.transport.read_state()
        if (frame.get('timestamp_basis') != 'capture' or not frame.get('stream_id')
                or not 0 <= self.clock()-stamp <= self.transport.limits.frame_age_s or stamp <= after):
            raise Refused('Need a fresh camera capture after the measured movement')
        if self.last is not None and (frame['seq'] <= self.last['seq'] or stamp <= self.last['captured_at']):
            raise Refused('Camera frame did not advance')
        tags = {t['tag_id']: t for t in row.get('tags', []) if t.get('status') == 'DETECTED'}
        if not {1, self.gripper_tag_id} <= set(tags):
            raise Refused(f'Need visible table tag 1 and gripper tag {self.gripper_tag_id}')
        geometry = row.get('pose_3d') or {}
        mount = next((t.get('mount') for t in geometry.get('tags', []) if t.get('tag_id') == self.gripper_tag_id), None)
        if not mount or mount.get('arm') != self.transport.arm or mount.get('body') != 'fixed_gripper_housing' or not mount.get('source'):
            raise Refused(f'Tag {self.gripper_tag_id} lacks matching confirmed fixed-housing mounting')
        identity = (frame['camera_id'], frame['stream_id'], geometry.get('geometry_config_sha256'), geometry.get('calibration_sha256'), row.get('image_size_px'), self.transport.arm, self.gripper_tag_id, dict(mount))
        if self.identity is not None and identity != self.identity:
            raise Refused('Camera stream, geometry or intrinsics changed')
        anchor = np.asarray(tags[1]['corners_px'], dtype=float)
        corners = np.asarray(tags[self.gripper_tag_id]['corners_px'], dtype=float)
        if anchor.shape != (4, 2) or corners.shape != (4, 2) or not np.isfinite([anchor, corners]).all():
            raise Refused('Invalid tag corners')
        if self.anchor is not None and np.max(np.linalg.norm(anchor-self.anchor, axis=1)) > 2:
            raise Refused('Table anchor moved; camera or scene changed')
        self.identity = identity
        if self.anchor is None:
            self.anchor = anchor.copy()
        # The pixel controller can work without an unambiguous 3D orientation.
        try:
            sample = stationary_sample(before, following, row, self.transport.arm, gripper_tag_id=self.gripper_tag_id)
            rejection = None
        except ValueError as exc:
            sample, rejection = None, str(exc)
        a = {r['name']: r for r in before['result']['motors']}
        b = {r['name']: r for r in following['result']['motors']}
        if (any(abs(a[n]['Present_Position']-b[n]['Present_Position']) > 3 for n in a)
                or not max(r['captured_at'] for r in a.values()) <= stamp <= min(r['captured_at'] for r in b.values())):
            raise Refused('Camera observation lacks a stationary encoder bracket')
        self.capture = {'sample': sample, 'sample_rejection': rejection, 'before': before, 'after': following}
        self.transport.observed_q = {n: r['Present_Position'] for n, r in b.items()}
        self.transport.observed_at = stamp
        self.payload, self.last = payload, frame
        self.count += 1
        return Observation((corners-anchor.mean(axis=0)).ravel(), stamp,
            {self.camera: frame['seq']}, {'table': anchor.tolist(), 'gripper': corners.tolist()},
            {self.camera: frame['stream_id']})

    def evidence(self, folder, observation):
        folder.mkdir(parents=True, exist_ok=False)
        atomic_json(folder/'observation.json', {k: v for k, v in self.payload.items() if k != 'images'})
        for k in ('before', 'after', 'sample'):
            atomic_json(folder/f'{k}.json', self.capture[k])
        if self.capture['sample_rejection']:
            atomic_json(folder/'sample-rejected.json', {'reason': self.capture['sample_rejection']})
        for i, picture in enumerate(self.payload.get('images', [])):
            (folder/f'annotated-{i}.jpg').write_bytes(base64.b64decode(picture['data_base64']))
