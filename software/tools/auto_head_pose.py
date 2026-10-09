"""Point the head OAK at the fold policy's training pose and measure its pose, through the robot API.

Replaces the manual head set-up (docs/auto-head-pose.md): no inclinometer, no floor tags. The two gripper tags (left 4,
right 2) are located from the arm encoder ticks and the joint maps, seen by the OAK, and solved for the head camera's
pose in the arm_base frame (carton/head_pose.py). The training camera is tilt 58 deg down, pan 0
(profiles/fold-station-xlerobot-220.json, cameras.front).

DRY-RUN (default): read-only. Reads the owner status (arm and head ticks), one OAK frame with its intrinsics, and
reports the measured pose against the training pose. Nothing moves; no motor is enabled.

    PYTHONPATH=. python tools/auto_head_pose.py --pilot-root <pilot> --out runs/head-pose-1

--execute --operator NAME: closed loop. Each iteration takes a fresh frame, measures, and if |tilt - 58| or |pan| is
above the tolerance (1 deg) sends ONE robot_move_head of at most --max-step-ticks per joint (default 60, about 5 deg)
toward the target, inside the head's saved range minus the owner's margin (40 ticks); then re-measures. It stops when
both are within tolerance, after --max-iterations moves, or when the head range is reached. It aborts on any refused or
failed call, an owner STOP or fault, arms moving during a frame, or an unusable frame. It never calls robot_stop and
never releases anything: STOP stays with the operator, and the head holds wherever it stopped.

    PYTHONPATH=. python tools/auto_head_pose.py --pilot-root <pilot> --out runs/head-pose-2 \
        --execute --operator Ron [--enable-head]

The head moves only through robot_move_head (owner --head scope); --enable-head first enables the two head motors
with robot_set_motor_enable (they hold where they are; no motion). Tick direction: head_motor_2 (tilt) and
head_motor_1 (pan) follow the twin's sign (+ ticks = down / left) until the first move shows otherwise; the gain is
then re-estimated from each move (the twin's tick<->angle offsets are known to be wrong, so ticks are never trusted
for the pose itself).

Arms: hold both still with the gripper tags facing the head, e.g. carton.head_pose.MEASUREMENT_ARM_POSE_DEG
(`--print-arm-pose` prints its encoder targets for the pilot's own arm tools; this tool never moves an arm).

Output (<out>/auto-head-pose.json): every measurement (pose, RMS, tags, head and arm ticks, frame file, sha256, seq,
capture time), every command and its answer, the intrinsics binding, stream warnings (the training camera is 4:3 with
a 54 deg vertical FOV; today's OAK stream is 640x360) and the final `camera_entry` in the measurement-file format of
carton/folding_station_measured.py (cameras.front). <out>/station-measurement.json is --station with cameras.front
replaced, for tools/restage_fold_scenes.py; --update PATH writes the entry into an existing measurement file.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carton import head_pose as hp  # noqa: E402
from carton.servo.common import Refused  # noqa: E402

PAN, TILT = 'head_motor_1', 'head_motor_2'
HEAD = (PAN, TILT)
DEG_PER_TICK = 360 / 4096
ARM_SUFFIXES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
ARM_NAMES = tuple(f'{a}_arm_{s}' for a in ('left', 'right') for s in ARM_SUFFIXES)
OWNER_HEAD_MARGIN = 40          # head_joint_executor: targets 40 ticks inside the saved range
OWNER_HEAD_MAX_TICKS = 200      # head_joint_executor: at most 200 ticks per joint per move
OWNER_TICKS_PER_S = 100         # head_joint_executor: duration_s >= longest travel / 100


class Aborted(RuntimeError):
    """Stop the loop now; nothing further is sent (the head holds where it is)."""


@dataclass
class LoopConfig:
    target_tilt_deg: float = hp.TRAINING_TILT_DEG
    target_pan_deg: float = hp.TRAINING_PAN_DEG
    tolerance_deg: float = 1.0
    max_iterations: int = 12            # robot_move_head commands at most
    max_step_ticks: int = 60            # per joint per command (owner allows 200)
    margin_ticks: int = OWNER_HEAD_MARGIN
    tilt_sign: int = 1                  # + ticks tilt down (twin convention, unvalidated)
    pan_sign: int = 1                   # + ticks pan left (twin convention, unvalidated)
    duration_factor: float = 1.5        # duration_s = factor * the owner's minimum (1 s per 100 ticks), >= 1 s
    settle_s: float = .6
    frame_retries: int = 8
    still_ticks: int = 2                # arm/head encoder change allowed across one frame
    min_gain_ratio: float = .4          # measured deg/tick vs 360/4096, else the measurement is not trusted
    max_gain_ratio: float = 2.5
    gain_min_ticks: int = 25            # re-estimate deg/tick only from moves at least this long (~2 deg)
    frames_per_measurement: int = 3     # consecutive frames per measurement; their tag corners are averaged
    use_box_tags: bool = False
    min_tags: int = hp.MIN_TAGS
    max_rms_px: float = hp.MAX_RMS_PX


def _ok(payload, tool):
    if not isinstance(payload, dict) or payload.get('ok') is not True or not isinstance(payload.get('result'), dict):
        raise Aborted(f'{tool} refused: {str(payload)[:600]}')
    return payload['result']


def _round(v, n=3):
    return None if v is None else round(float(v), n)


class AutoHeadPose:
    def __init__(self, robot, arm_maps, out_dir, *, config: LoopConfig | None = None, station_path=hp.STATION_PROFILE,
                 intrinsics=None, operator=None, clock=time.time, sleep=time.sleep):
        self.robot, self.maps, self.cfg = robot, arm_maps, config or LoopConfig()
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / 'frames').mkdir()
        self.station_path = Path(station_path)
        self.station_doc, self.station = hp.load_station(self.station_path)
        self.training = self.station_doc['cameras']['front']
        self.base_spacing = float(self.station['base_spacing_m'])
        self.intrinsics_override, self.operator = intrinsics, operator
        self.clock, self.sleep = clock, sleep
        self.calls, self.measurements, self.commands, self.warnings = [], [], [], []
        self.binding = None
        self.gain = {TILT: self.cfg.tilt_sign * DEG_PER_TICK, PAN: self.cfg.pan_sign * DEG_PER_TICK}
        self.gain_log = []
        self.moved_since_measurement = False

    # ------------------------------------------------------------------------------------------- robot reads
    def call(self, name, args):
        t0 = self.clock()
        payload = self.robot.call(name, args)
        self.calls.append({'tool': name, 't': t0, 'ok': isinstance(payload, dict) and payload.get('ok') is True})
        return payload

    def owner(self):
        s = _ok(self.call('robot_get_execution', {}), 'robot_get_execution')
        if s.get('hardware_server') is not True:
            raise Aborted('No live hardware owner (robot_get_execution has no hardware_server)')
        return s

    def ticks(self, status):
        rows = status.get('rows') or {}
        out, missing = {}, []
        for n in (*ARM_NAMES, *HEAD):
            q = (rows.get(n) or {}).get('Present_Position')
            if type(q) is int:
                out[n] = q
            else:
                missing.append(n)
        if missing:
            state = _ok(self.call('robot_get_state', {'fresh': True}), 'robot_get_state')
            listed = {r.get('name'): r for r in state.get('motors') or []}
            listed.update(state.get('live_rows') or {})
            for n in list(missing):
                q = (listed.get(n) or {}).get('Present_Position')
                if type(q) is int:
                    out[n] = q
                    missing.remove(n)
        if missing:
            raise Aborted(f'No encoder reading for {missing}')
        return out

    def frame(self, not_before=None):
        """(rgb, image entry, manifest) of a fresh OAK frame captured at or after `not_before` (robot clock)."""
        for _ in range(self.cfg.frame_retries):
            payload = self.call('robot_get_cameras', {'cameras': ['oak'], 'revive': False})
            result = _ok(payload, 'robot_get_cameras')
            errors = result.get('camera_errors') or {}
            images = payload.get('images') or []
            if errors or len(images) != 1:
                raise Aborted(f'OAK frame unavailable: {errors or "no image"}')
            image = images[0]
            if not_before is not None and float(image.get('captured_at') or 0) < not_before:
                self.sleep(.1)
                continue
            data = base64.b64decode(image['data_base64'])
            if hashlib.sha256(data).hexdigest() != image.get('sha256'):
                raise Aborted('OAK image hash mismatch')
            bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise Aborted('OAK image undecodable')
            meta = (result.get('cameras') or {}).get('oak') or {}
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), image, meta, data
        raise Aborted(f'No OAK frame captured after the head move (waited {self.cfg.frame_retries} tries)')

    def intrinsics_for(self, rgb, image, meta):
        h, w = rgb.shape[:2]
        if self.intrinsics_override is not None:
            k, dist, iw, ih = hp.intrinsics_matrix(self.intrinsics_override)
            if (iw, ih) != (w, h):
                raise Aborted(f'--intrinsics are for {iw}x{ih}, the OAK frame is {w}x{h}')
            binding = {'source': 'override (--intrinsics)', 'intrinsics': k.tolist(), 'effective_distortion': dist.tolist(),
                       'image_size_px': [w, h]}
        else:
            from farm.perception.tag_geometry import camera_calibration
            try:
                k, dist, binding = camera_calibration(meta, image, rgb.shape)
            except ValueError as exc:
                raise Aborted(f'OAK frame carries no usable intrinsics ({exc}); pass --intrinsics from '
                              'tools/oak_intrinsics.py for this exact resolution') from exc
            binding = {**binding, 'source': 'OAK stream manifest (robot_get_cameras)',
                       'rgb_pipeline': meta.get('rgb_pipeline')}
        return {'K': np.asarray(k, float).tolist(), 'distortion': np.asarray(dist, float).tolist(),
                'width': w, 'height': h}, binding

    # ------------------------------------------------------------------------------------------- measuring
    def measure(self, label, not_before=None):
        """Pose from `frames_per_measurement` consecutive frames (tag corners averaged), with the arms and head shown
        to hold still from an encoder reading before the first frame to one taken after the last."""
        s0 = self.owner()
        t0 = self.ticks(s0)
        frames, seqs = [], set()
        for _ in range(4 * self.cfg.frames_per_measurement + self.cfg.frame_retries):
            if len(frames) == self.cfg.frames_per_measurement:
                break
            got = self.frame(not_before)
            if got[1].get('seq') in seqs:          # the stream has not published a new frame yet
                self.sleep(.05)
                continue
            seqs.add(got[1].get('seq'))
            frames.append(got)
        else:
            raise Aborted('The OAK stream did not deliver new frames')
        last = frames[-1][1]
        # The second encoder reading must be one the owner took after the last frame (its status is periodic).
        for _ in range(40):
            s1 = self.owner()
            if float(s1.get('time') or 0) > float(last.get('captured_at') or 0):
                break
            self.sleep(.05)
        else:
            raise Aborted('Owner status did not refresh after the frame; cannot check the arms held still')
        t1 = self.ticks(s1)
        moved = {n: t1[n] - t0[n] for n in t0 if abs(t1[n] - t0[n]) > self.cfg.still_ticks}
        n = len(self.measurements)
        files = []
        for j, (_, image, _, data) in enumerate(frames):
            frame_file = self.out / 'frames' / f'{n:02d}-{label}-{j}.jpg'
            frame_file.write_bytes(data)
            files.append({'frame': str(frame_file.relative_to(self.out)), 'sha256': image.get('sha256'),
                          'seq': image.get('seq'), 'stream_id': image.get('stream_id'),
                          'captured_at': image.get('captured_at')})
        record = {'index': n, 'label': label, 'frame': files[-1]['frame'], 'sha256': files[-1]['sha256'], 'frames': files,
                  'owner_time': [s0.get('time'), s1.get('time')], 'head_ticks': {k: t1[k] for k in HEAD},
                  'arm_ticks': {k: t1[k] for k in ARM_NAMES}}
        self.measurements.append(record)
        if moved:
            record['refused'] = f'Joints moved during the frame: {moved}; hold the arms and head still'
            raise Aborted(record['refused'])
        rgb, image, meta, _ = frames[-1]
        intrinsics, binding = self.intrinsics_for(rgb, image, meta)
        if self.binding is None:
            self.binding = binding
            stream = hp.aspect_report(intrinsics['width'], intrinsics['height'], intrinsics['K'][1][1])
            self.stream = stream
            self.warnings.extend(stream['warnings'])
        config = hp.HeadPoseConfig(base_spacing_m=self.base_spacing, station=self.station,
                                   use_box_tags=self.cfg.use_box_tags, min_tags=self.cfg.min_tags,
                                   max_rms_px=self.cfg.max_rms_px)
        detections, singles = [], []
        try:
            for rgb_j, *_ in frames:
                found = hp.detect(rgb_j)
                detections.append(found)
                try:      # each frame alone: their spread is the measured noise
                    one = hp.measure_head_pose(rgb_j, intrinsics, self.maps, t1, config, detections=found)
                    singles.append([one['tilt_deg'], one['pan_deg']])
                except hp.HeadPoseRefused:
                    pass
            common = set.intersection(*(set(f) for f in detections))
            averaged = {i: {'corners': np.mean([f[i]['corners'] for f in detections], axis=0).tolist()} for i in common}
            pose = hp.measure_head_pose(rgb, intrinsics, self.maps, t1, config, detections=averaged)
        except (hp.HeadPoseRefused, Refused) as exc:
            record['refused'] = str(exc)
            raise Aborted(f'No head pose from frames {[f["frame"] for f in files]}: {exc}') from exc
        record['single_frame_tilt_pan_deg'] = singles
        if len(singles) > 1:
            record['single_frame_spread_deg'] = {'tilt': float(np.ptp([v[0] for v in singles])),
                                                 'pan': float(np.ptp([v[1] for v in singles]))}
        record['pose'] = {k: pose[k] for k in ('position_m', 'rotation_cv', 'tilt_deg', 'pan_deg', 'roll_deg',
                                               'reprojection_rms_px', 'reprojection_max_px', 'per_tag_rms_px',
                                               'tags_used', 'tags_seen', 'tag_side_px', 'sensitivity_1sigma',
                                               'model_lens_position_at_measured_angles_m',
                                               'position_minus_model_lens_mm')}
        if 'subset_solutions' in pose:
            record['pose']['subset_solutions'] = diff = pose['subset_solutions']
            d = diff['gripper_minus_box']
            if max(abs(d['tilt_deg']), abs(d['pan_deg'])) > 1.5 or max(map(abs, d['position_mm'])) > 15:
                self.warnings.append(f'Gripper-tag and box-tag poses disagree (tilt {d["tilt_deg"]:+.1f} deg, pan '
                                     f'{d["pan_deg"]:+.1f} deg, position {d["position_mm"]} mm): an arm joint map zero, '
                                     'the carton spot or the base spacing is off')
        record['vs_training'] = hp.compare_with_training(pose, self.training)
        record['intrinsics'] = intrinsics
        self.moved_since_measurement = False
        return record

    # ------------------------------------------------------------------------------------------- moving
    def preflight(self, enable_head):
        caps = _ok(self.call('robot_get_capabilities', {}), 'robot_get_capabilities')
        if caps.get('head_supported') is not True:
            raise Aborted('The owner does not support head moves (started without --head, or the head was demoted '
                          f'to read-only): {caps.get("head_policy") or caps.get("scope_reduced")}')
        limits = caps.get('head_move_limits') or {}
        status = self.owner()
        self.stop_count, self.started = status.get('stop_count'), status.get('started')
        ranges = status.get('ranges') or {}
        for n in HEAD:
            if n not in ranges:
                raise Aborted(f'Owner reports no range for {n}')
        for arm, m in self.maps.items():
            for name, rng in m.ranges().items():
                if name in ranges and tuple(ranges[name]) != tuple(rng):
                    raise Aborted(f'{name}: owner range {ranges[name]} differs from the joint map calibration {rng}; '
                                  'bind the joint maps to the live calibration first (tools/bind_fold_joint_maps.py)')
        margin = max(self.cfg.margin_ticks, int(limits.get('margin_ticks') or OWNER_HEAD_MARGIN))
        self.limits = {n: (int(ranges[n][0]) + margin, int(ranges[n][1]) - margin) for n in HEAD}
        self.max_step = min(self.cfg.max_step_ticks, int(limits.get('max_ticks_per_move') or OWNER_HEAD_MAX_TICKS))
        per100 = float(limits.get('min_duration_s_per_100_ticks') or 100 / OWNER_TICKS_PER_S)
        self.seconds_per_tick = per100 / 100
        enabled = set(status.get('enabled_motors') or [])
        if not set(HEAD) <= enabled:
            if not enable_head:
                raise Aborted(f'Head motors not enabled ({sorted(set(HEAD) - enabled)}); enable them with '
                              'robot_set_motor_enable (they hold where they are) or pass --enable-head')
            result = _ok(self.call('robot_set_motor_enable', {'names': list(HEAD), 'enabled': True}),
                         'robot_set_motor_enable')
            if result.get('accepted') is False or result.get('completed') is False:
                raise Aborted(f'Head enable not accepted: {str(result)[:400]}')
            self.commands.append({'tool': 'robot_set_motor_enable', 'names': list(HEAD), 'result': _brief(result)})
            status = self.owner()
            if not set(HEAD) <= set(status.get('enabled_motors') or []):
                raise Aborted('Head motors still not enabled after robot_set_motor_enable')
            self.stop_count = status.get('stop_count')
        return {'head_move_limits': limits, 'commandable': self.limits, 'max_step_ticks': self.max_step}

    def check_owner(self):
        s = self.owner()
        if s.get('stop_count') != self.stop_count or s.get('started') != self.started or s.get('ok') is not True:
            raise Aborted(f'Owner STOP or fault (stop_count {self.stop_count} -> {s.get("stop_count")}, ok '
                          f'{s.get("ok")}, last_stop {s.get("last_stop")})')
        return s

    def update_gain(self, before, after):
        """Re-estimate deg/tick for each joint from the last move (sign included); refuses implausible responses."""
        for joint, key in ((TILT, 'tilt_deg'), (PAN, 'pan_deg')):
            dq = after['head_ticks'][joint] - before['head_ticks'][joint]
            if abs(dq) < self.cfg.gain_min_ticks:
                continue        # too small to separate from measurement noise
            g = (after['pose'][key] - before['pose'][key]) / dq
            ratio = abs(g) / DEG_PER_TICK
            entry = {'joint': joint, 'ticks': dq, 'degrees': after['pose'][key] - before['pose'][key],
                     'deg_per_tick': g, 'ratio_to_nominal': ratio}
            self.gain_log.append(entry)
            if not self.cfg.min_gain_ratio <= ratio <= self.cfg.max_gain_ratio:
                raise Aborted(f'{joint}: {dq} ticks turned the measured {key[:-4]} by {entry["degrees"]:.2f} deg '
                              f'({ratio:.2f}x the nominal 360/4096 deg per tick); the measurement or the head is not '
                              'behaving, stopping')
            if np.sign(g) != np.sign(self.gain[joint]):
                self.warnings.append(f'{joint}: ticks turn the camera the opposite way to the twin convention '
                                     f'({key[:-4]} {g:+.4f} deg/tick); using the measured direction')
            self.gain[joint] = g

    def plan(self, record):
        cfg, cur = self.cfg, record['head_ticks']
        errors = {TILT: cfg.target_tilt_deg - record['pose']['tilt_deg'],
                  PAN: cfg.target_pan_deg - record['pose']['pan_deg']}
        targets, limited = {}, {}
        for joint, err in errors.items():
            if abs(err) <= cfg.tolerance_deg / 2:
                continue
            step = max(-self.max_step, min(self.max_step, err / self.gain[joint]))
            want = int(round(cur[joint] + step))
            lo, hi = self.limits[joint]
            target = min(hi, max(lo, want))
            if target != want:
                limited[joint] = {'wanted': want, 'commandable': [lo, hi],
                                  'ticks_to_target_estimate': int(round(cur[joint] + err / self.gain[joint]))}
            if abs(target - cur[joint]) >= 3:
                targets[joint] = target
        return errors, targets, limited

    def move(self, targets, current):
        travel = max(abs(t - current[n]) for n, t in targets.items())
        duration = max(1.0, math.ceil(travel * self.seconds_per_tick * self.cfg.duration_factor * 100) / 100)
        args = {'positions': {n: int(t) for n, t in targets.items()}, 'duration_s': duration}
        self.moved_since_measurement = True
        payload = self.call('robot_move_head', args)
        entry = {'tool': 'robot_move_head', 'args': args, 'from': {n: current[n] for n in targets}}
        self.commands.append(entry)
        result = _ok(payload, 'robot_move_head')
        entry['result'] = _brief(result)
        if result.get('accepted') is not True:
            raise Aborted(f'robot_move_head not accepted: {str(result)[:400]}')
        if result.get('completed') is False or result.get('closure_outcome') not in (None, 'endpoint_settled'):
            raise Aborted(f'Head move did not settle at its target: {str(result)[:400]}')
        status = self.check_owner()
        return status

    # ------------------------------------------------------------------------------------------- the run
    def run(self, execute=False, enable_head=False):
        summary = {'execute': execute, 'operator': self.operator, 'config': asdict(self.cfg),
                   'target': {'tilt_deg': self.cfg.target_tilt_deg, 'pan_deg': self.cfg.target_pan_deg},
                   'station_profile': str(self.station_path), 'joint_maps': {a: m.config_path for a, m in self.maps.items()},
                   'started_local': self.clock(), 'stop_reason': None, 'converged': False, 'aborted': None}
        try:
            if execute:
                summary['preflight'] = self.preflight(enable_head)
            record = self.measure('start')
            moves = 0
            while True:
                errors, targets, limited = self.plan(record) if execute else self._errors_only(record)
                record['error_deg'] = {'tilt': errors[TILT], 'pan': errors[PAN]}
                within = all(abs(e) <= self.cfg.tolerance_deg for e in errors.values())
                if within:
                    summary['converged'], summary['stop_reason'] = True, 'within_tolerance'
                    break
                if not execute:
                    summary['stop_reason'] = 'dry_run'
                    break
                if moves >= self.cfg.max_iterations:
                    summary['stop_reason'] = 'max_iterations'
                    break
                if not targets:
                    summary['stop_reason'] = 'head_range_limit' if limited else 'step_below_resolution'
                    summary['range_limit'] = limited
                    break
                record['limited'] = limited
                status = self.move(targets, record['head_ticks'])
                moves += 1
                self.sleep(self.cfg.settle_s)
                before = record
                record = self.measure(f'after-move-{moves}', not_before=float(status.get('time') or 0))
                self.update_gain(before, record)
        except Aborted as exc:
            summary['aborted'] = str(exc)
            summary['stop_reason'] = 'aborted'
        summary['moves'] = sum(1 for c in self.commands if c['tool'] == 'robot_move_head')
        summary['measurements'] = self.measurements
        summary['commands'] = self.commands
        summary['gain_estimates'] = self.gain_log
        summary['intrinsics_binding'] = self.binding
        summary['stream'] = getattr(self, 'stream', None)
        final = next((m for m in reversed(self.measurements) if 'pose' in m), None)
        if final is not None:
            fresh = final is self.measurements[-1]
            summary['final'] = {'measurement_index': final['index'], 'is_last_frame': fresh,
                                'tilt_deg': final['pose']['tilt_deg'], 'pan_deg': final['pose']['pan_deg'],
                                'roll_deg': final['pose']['roll_deg'], 'position_m': final['pose']['position_m'],
                                'rotation_cv': final['pose']['rotation_cv'],
                                'reprojection_rms_px': final['pose']['reprojection_rms_px'],
                                'head_ticks': final['head_ticks'], 'vs_training': final['vs_training'],
                                'intrinsics': final['intrinsics']}
        if final is not None and (final is not self.measurements[-1] or self.moved_since_measurement):
            self.warnings.append('The head was commanded after the last pose (or the last frames gave none): `final` may '
                                 'be out of date, so no camera entry is written; re-run the dry run')
        elif final is not None:
            used = final['pose']['tags_used']
            entry = hp.camera_entry({**final['pose'], 'gripper_tags_used': [t for t in used if t in hp.GRIPPER_TAGS],
                                     'box_tags_used': [t for t in used if t not in hp.GRIPPER_TAGS]},
                                    final['intrinsics'], head_ticks=final['head_ticks'],
                                    source=f'tools/auto_head_pose.py {final["frame"]} ({final["sha256"][:12]}), '
                                           f'{"converged" if summary["converged"] else summary["stop_reason"]}')
            summary['camera_entry'] = entry
            if abs(final['pose']['roll_deg']) > 2:
                self.warnings.append(f'Camera roll {final["pose"]["roll_deg"]:.1f} deg about its axis (the training camera '
                                     'has none): check the OAK sits square in its cradle')
        if any('NOT MEASURED' in str(m.evidence.get('joints', '')) for m in self.maps.values()):
            self.warnings.append('Arm joint maps are owner-accepted, not measured: gripper tag positions inherit their '
                                 'error; prefer --box-tags with the carton in its spot for a cross-check')
        summary['warnings'] = list(dict.fromkeys(self.warnings))
        summary['calls'] = [{k: c[k] for k in ('tool', 'ok', 't')} for c in self.calls]
        summary['robot_stop_called'] = any(c['tool'] == 'robot_stop' for c in self.calls)
        (self.out / 'auto-head-pose.json').write_text(json.dumps(_clean(summary), indent=1))
        if 'camera_entry' in summary:
            measurement = json.loads(json.dumps(self.station_doc))
            measurement['cameras']['front'] = summary['camera_entry']
            measurement['front_camera_measured'] = {'tool': 'tools/auto_head_pose.py', 'converged': summary['converged'],
                                                    'stop_reason': summary['stop_reason'],
                                                    'note': 'Only cameras.front is measured here; the station numbers '
                                                            'are still those of the source profile.'}
            from carton.folding_station_measured import load_measurement
            load_measurement(measurement)
            (self.out / 'station-measurement.json').write_text(json.dumps(_clean(measurement), indent=1))
        return summary

    def _errors_only(self, record):
        return ({TILT: self.cfg.target_tilt_deg - record['pose']['tilt_deg'],
                 PAN: self.cfg.target_pan_deg - record['pose']['pan_deg']}, {}, {})


def _brief(result):
    keys = ('accepted', 'completed', 'no_op', 'command_id', 'closure_outcome', 'endpoint_reached', 'head_targets',
            'duration_s', 'reason', 'phase', 'motor_writes')
    return {k: result[k] for k in keys if k in result}


def _clean(value):
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _pairs(values, what):
    out = {}
    for item in values or []:
        if '=' not in item:
            raise SystemExit(f'--{what} takes KEY=VALUE, got {item!r}')
        k, v = item.split('=', 1)
        out[k] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], epilog='See docs/auto-head-pose.md.')
    ap.add_argument('--pilot-root', type=Path, help='chat pilot checkout with chat_server.Robot and .private/robot.json')
    ap.add_argument('--out', type=Path, help='new output directory')
    ap.add_argument('--joint-map', action='append', metavar='ARM=PATH',
                    help='per-arm joint map (default: profiles/fold-joint-maps/{left,right}-joint-map.json)')
    ap.add_argument('--station', type=Path, default=hp.STATION_PROFILE, help='station profile (training front camera, '
                    'base spacing, box-tag placement)')
    ap.add_argument('--intrinsics', type=Path, help='override the stream intrinsics (tools/oak_intrinsics.py JSON)')
    ap.add_argument('--box-tags', action='store_true', help='also use the carton tags (carton in its nominal spot)')
    ap.add_argument('--execute', action='store_true', help='move the head (default: dry-run, read-only)')
    ap.add_argument('--operator', help='the person holding STOP (required with --execute)')
    ap.add_argument('--enable-head', action='store_true', help='enable the head motors first (they hold in place)')
    ap.add_argument('--target-tilt', type=float, default=hp.TRAINING_TILT_DEG)
    ap.add_argument('--target-pan', type=float, default=hp.TRAINING_PAN_DEG)
    ap.add_argument('--tolerance-deg', type=float, default=1.0)
    ap.add_argument('--max-iterations', type=int, default=12)
    ap.add_argument('--max-step-ticks', type=int, default=60)
    ap.add_argument('--tilt-sign', type=int, choices=(-1, 1), default=1)
    ap.add_argument('--pan-sign', type=int, choices=(-1, 1), default=1)
    ap.add_argument('--max-rms-px', type=float, default=hp.MAX_RMS_PX)
    ap.add_argument('--frames', type=int, default=3, help='frames per measurement (tag corners averaged)')
    ap.add_argument('--update', type=Path, help='measurement JSON whose cameras.front is replaced')
    ap.add_argument('--print-arm-pose', action='store_true',
                    help='print the measurement arm pose as encoder targets for the joint maps, then exit (no robot)')
    args = ap.parse_args(argv)
    from carton.fold_policy_runner import load_arm_maps
    paths = _pairs(args.joint_map, 'joint-map') or {a: hp.JOINT_MAP_DIR / f'{a}-joint-map.json' for a in ('left', 'right')}
    try:
        maps = load_arm_maps(paths)
    except Refused as exc:
        raise SystemExit(str(exc)) from exc
    if args.print_arm_pose:
        print(json.dumps({'degrees': hp.MEASUREMENT_ARM_POSE_DEG, 'ticks': hp.measurement_pose_ticks(maps)}, indent=1))
        return 0
    if args.pilot_root is None or args.out is None:
        raise SystemExit('--pilot-root and --out are required')
    if args.execute and not args.operator:
        raise SystemExit('--execute needs --operator (the person holding STOP)')
    if not 0 < args.max_step_ticks <= OWNER_HEAD_MAX_TICKS:
        raise SystemExit(f'--max-step-ticks must be 1..{OWNER_HEAD_MAX_TICKS}')
    config = LoopConfig(target_tilt_deg=args.target_tilt, target_pan_deg=args.target_pan, tolerance_deg=args.tolerance_deg,
                        max_iterations=args.max_iterations, max_step_ticks=args.max_step_ticks, tilt_sign=args.tilt_sign,
                        pan_sign=args.pan_sign, use_box_tags=args.box_tags, max_rms_px=args.max_rms_px,
                        frames_per_measurement=max(1, args.frames))
    sys.path.insert(0, str(args.pilot_root.resolve()))
    import importlib
    robot = importlib.import_module('chat_server').Robot(args.pilot_root / '.private/robot.json')
    intrinsics = json.loads(args.intrinsics.read_text()) if args.intrinsics else None
    tool = AutoHeadPose(robot, maps, args.out, config=config, station_path=args.station, intrinsics=intrinsics,
                        operator=args.operator)
    summary = tool.run(execute=args.execute, enable_head=args.enable_head)
    if args.update and summary.get('camera_entry'):
        m = json.loads(args.update.read_text())
        m.setdefault('cameras', {})['front'] = summary['camera_entry']
        args.update.write_text(json.dumps(_clean(m), indent=1))
    final = summary.get('final') or {}
    print(json.dumps(_clean({'execute': summary['execute'], 'stop_reason': summary['stop_reason'],
                             'converged': summary['converged'], 'aborted': summary['aborted'], 'moves': summary['moves'],
                             'tilt_deg': _round(final.get('tilt_deg')), 'pan_deg': _round(final.get('pan_deg')),
                             'roll_deg': _round(final.get('roll_deg')), 'position_m': final.get('position_m'),
                             'reprojection_rms_px': _round(final.get('reprojection_rms_px')),
                             'head_ticks': final.get('head_ticks'),
                             'vs_training': final.get('vs_training'), 'warnings': summary['warnings'],
                             'out': str(args.out)}), indent=1))
    return 0 if summary['aborted'] is None and (summary['converged'] or not summary['execute']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
