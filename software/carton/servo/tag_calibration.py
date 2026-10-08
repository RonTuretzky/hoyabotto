"""Automatic calibration using the existing controller, owner and tag fitter."""
from __future__ import annotations

from contextlib import contextmanager
import copy
import fcntl
from pathlib import Path
import time

from .common import Limits, Refused, Trace, atomic_json, digest
from .controller import Experiment
from .gemma import GemmaTagObserver, GemmaTransport, result
from farm.kinematics.tag_registration import assemble_dataset, fit_registration
from farm.perception.tag_sampling import gripper_tag_for_arm


@contextmanager
def motion_lock(path):
    """Shared by the CLI and installed Gemma wrapper; STOP bypasses this lock."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise Refused('Another local motion client or calibration is active') from exc
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def registration_offsets(joints, limits):
    """Eight fit poses round a two-axis grid, three independent half-grid poses.

    At the default 96 tick radius the planned travel is 1248 ticks including
    return, within Limits. Each actual transition is split into observed steps.
    Nonparallel rotations/spatial span are checked by the existing fitter.
    """
    if len(joints) != 2:
        raise Refused('Registration grid requires exactly two selected positioning axes')
    r = limits.trust_ticks
    points = [(-r, -r), (-r, 0), (-r, r), (0, r), (r, r), (r, 0), (r, -r), (0, -r)]
    points += [(-r//2, -r//2), (-r//2, r//2), (r//2, r//2)]
    rows = [{'split': 'train' if i < 8 else 'validation', 'offset_ticks': dict(zip(joints, point))}
            for i, point in enumerate(points)]
    previous, travel = (0, 0), 0
    for point in points + [(0, 0)]:
        travel += sum(abs(a-b) for a, b in zip(previous, point))
        previous = point
    if travel > limits.max_path_ticks:
        raise Refused('Registration grid exceeds total travel budget')
    return {'poses': rows, 'planned_path_ticks': travel, 'collision_checked': False,
            'step_ticks': limits.step_ticks, 'radius_ticks': r}


# Relay calls made by a registration run (qwen-bridge/test_tag_registration_contract.py checks this model):
# each observed step = owner read, camera, owner read (frame bracket), status, move, status.
CALLS_PER_STEP, CALLS_PER_POSE, CALLS_FIXED = 6, 4, 26


def registration_timing(round_trip_s, limits=None, *, clock_offset_s=0.0, camera_latency_s=0.05,
                        detect_s=0.05, owner_publish_age_s=0.1, move_s=0.55):
    """Projected registration duration and freshness margins for a measured chat<->robot link.

    clock_offset_s is the robot clock minus this Mac's clock. GemmaTransport accepts owner rows aged
    0..status_age_s and frames aged 0..frame_age_s on this Mac's clock; neither limit is adjusted here.
    """
    limits = limits or Limits()
    plan = registration_offsets(['a', 'b'], limits)
    points = [tuple(p['offset_ticks'].values()) for p in plan['poses']]
    steps = sum(-(-abs(b-a) // limits.step_ticks) for p, q in zip([(0, 0)]+points, points+[(0, 0)]) for a, b in zip(p, q))
    poses = len(points)
    calls = CALLS_PER_STEP*steps + CALLS_PER_POSE*poses + CALLS_FIXED
    seconds = calls*round_trip_s + steps*(move_s+detect_s) + (poses+2)*detect_s
    # Frame capture -> tag response (half trip) -> after-frame owner read -> status read -> command check.
    frame_age = camera_latency_s + round_trip_s/2 + detect_s + 2*round_trip_s - clock_offset_s
    status_age = (owner_publish_age_s + round_trip_s/2 - clock_offset_s, round_trip_s/2 - clock_offset_s)
    return {'round_trip_s': round_trip_s, 'clock_offset_s': clock_offset_s, 'steps': steps, 'relay_calls': calls,
            'projected_seconds': seconds, 'budget_seconds': limits.max_seconds,
            'fits_time_budget': seconds <= limits.max_seconds,
            'frame_age_at_command_s': frame_age, 'frame_age_limit_s': limits.frame_age_s,
            'fits_frame_age': 0 <= frame_age <= limits.frame_age_s,
            'owner_status_age_range_s': list(status_age), 'status_age_limit_s': limits.status_age_s,
            'fits_status_age': status_age[1] >= 0 and status_age[0] <= limits.status_age_s,
            'assumptions': {'camera_latency_s': camera_latency_s, 'tag_detection_s': detect_s,
                            'owner_publish_age_s': owner_publish_age_s, 'owner_move_s': move_s}}


def readiness(robot, config, *, clock=time.time):
    """Live read-only report; collect vision even when motor readiness fails."""
    tag_id = gripper_tag_for_arm(config['arm'], config.get('gripper_tag_id'))
    limits = Limits(**config.get('limits', {}))
    transport = GemmaTransport(robot, config['arm'], config['joints'], limits, clock=clock)
    report = {'arm': config['arm'], 'gripper_tag_id': tag_id, 'joints': transport.joints, 'motor_writes': 0,
              'blockers': [], 'physical_registration_validated': False}
    try:
        report['owner'] = transport.preflight()
    except (Refused, ValueError, OSError) as exc:
        report['blockers'].append(str(exc))
    tags = result(robot.call('robot_get_tags', {'cameras': [config.get('camera', 'oak')], 'tag_ids': [1, tag_id]}), 'robot_get_tags')
    row = tags.get('observations', {}).get(config.get('camera', 'oak'), {})
    accepted = {t['tag_id']: t for t in row.get('tags', []) if t.get('status') == 'DETECTED'}
    report['detected_tag_ids'] = sorted(accepted)
    report['frame'] = row.get('frame')
    if not {1, tag_id} <= set(accepted):
        report['blockers'].append(f'Need table tag 1 and gripper tag {tag_id} in the same view')
    else:
        width, height = row['image_size_px']
        corners = accepted[tag_id]['corners_px']
        report['gripper_tag_border_clearance_px'] = min(min(x, y, width-1-x, height-1-y) for x, y in corners)
        report[f'tag_{tag_id}_border_clearance_px'] = report['gripper_tag_border_clearance_px']
    metric = row.get('pose_3d') or {}
    mount = next((t.get('mount') for t in metric.get('tags', []) if t.get('tag_id') == tag_id), None)
    report['gripper_tag_mount'] = report[f'tag_{tag_id}_mount'] = mount
    if not mount or mount.get('arm') != config['arm'] or mount.get('body') != 'fixed_gripper_housing' or not mount.get('source'):
        report['blockers'].append('Need confirmed matching fixed gripper-tag mounting')
    report['arm_geometry'] = robot.call('robot_get_arm_pose', {'arm': config['arm']})
    report['status'] = 'BLOCKED' if report['blockers'] else 'READY_FOR_LOCAL_PROBES'
    report['registration_plan'] = registration_offsets(transport.joints, limits) if len(transport.joints) == 2 else None
    report['note'] = 'Readiness is a snapshot, not a collision-checked path or permission to ignore a later refusal.'
    return report


def _observe(experiment, after=0.0):
    """Experiment.observe, taking the encoders from the observation's own after-frame owner read
    instead of one more position read. Every relay round trip between the camera frame and the next
    command counts against frame_age_s; the limit itself is unchanged."""
    observation = experiment.observer.observe(after=after)
    q = dict(experiment.transport.observed_q)
    experiment.latest = (observation, q)
    experiment.counter += 1
    experiment.trace.write("observation", index=experiment.counter, features=observation.values.tolist(),
                           captured_at=observation.captured_at, sequences=observation.sequences,
                           streams=observation.streams, joints=q, points=observation.points)
    if experiment.capture_evidence:
        experiment.observer.evidence(experiment.trace.folder/f"frame-{experiment.counter:04d}", observation)
    return observation, q


def _step(experiment, joint, ticks):
    """Experiment.move without its extra position read (see _observe). The transport's own fresh
    status read still refuses any motion since the observation (3 ticks) before dispatching."""
    observation, _ = experiment.latest
    if not 0 <= experiment.clock()-observation.captured_at <= experiment.limits.frame_age_s:
        raise Refused('Observation became stale before command dispatch')
    experiment.trace.write('command', joint=joint, delta_ticks=ticks)
    started = experiment.clock()
    q, finished = experiment.transport.move(joint, ticks)
    experiment.trace.write('acknowledgement', joint=joint, positions=q, duration_s=experiment.clock()-started)
    return _observe(experiment, after=finished)


def _move_to(experiment, targets):
    """Observed steps of at most step_ticks; never below the owner's 3-tick minimum segment."""
    minimum = getattr(experiment.transport, 'min_step_ticks', 1)
    for joint, target in targets.items():
        for _ in range(32):
            q = experiment.latest[1]  # measured right after the latest observation
            delta = round(target-q[joint])
            if abs(delta) <= max(experiment.limits.settle_ticks, minimum-1):
                break
            step = max(-experiment.limits.step_ticks, min(experiment.limits.step_ticks, delta))
            before = q[joint]
            _, measured = _step(experiment, joint, step)
            if (measured[joint]-before) * (1 if step > 0 else -1) < max(2, abs(step)*.4):
                raise Refused('Registration step produced insufficient or wrong-way motion')
        else:
            raise Refused('Registration waypoint did not converge')


def run_calibration(robot, config, mode, output, *, clock=time.time):
    """Explicit execution entrypoint; there is no automatic startup/resume path."""
    if mode not in ('local_model', 'registration'):
        raise Refused('Unknown calibration mode')
    tag_id = gripper_tag_for_arm(config['arm'], config.get('gripper_tag_id'))
    output = Path(output)
    if output.exists():
        raise Refused('Choose a new evidence directory')
    limits = Limits(**config.get('limits', {}))
    transport = GemmaTransport(robot, config['arm'], config['joints'], limits, execute=True, clock=clock)
    with motion_lock(config['lock_file']):
        trace = Trace(output)
        outcome = None
        released = False
        try:
            settings = transport.preflight()
            observer = GemmaTagObserver(robot, transport, config.get('camera', 'oak'), gripper_tag_id=tag_id, clock=clock)
            baseline = observer.observe()
            observer.evidence(output/'baseline', baseline)
            plan, arm_status = None, None
            if mode == 'registration':
                if observer.capture['sample'] is None:
                    raise Refused(f'No valid baseline registration sample: {observer.capture["sample_rejection"]}')
                if not config.get('model_directory'):
                    raise Refused('Registration needs the existing verified SO101 model directory')
                arm_status = robot.call('robot_get_arm_pose', {'arm': config['arm']})
                capture = dict(observer.capture, arm_geometry_status=arm_status)
                # Checks installed arm mapping, model/calibration hashes and FK dependencies BEFORE enabling.
                assemble_dataset([capture], config['model_directory'])
                plan = registration_offsets(transport.joints, limits)
                for pose in plan['poses']:
                    for joint, offset in pose['offset_ticks'].items():
                        goal, bounds = transport.origin[joint]+offset, transport.ranges[joint]
                        if not bounds['min_ticks'] <= goal <= bounds['max_ticks']:
                            raise Refused(f'Registration grid leaves commandable range for {joint}')
                atomic_json(output/'plan.json', plan)
            fingerprint = digest({'settings': settings, 'tags': observer.identity, 'mode': mode})
            experiment = Experiment(settings, transport, observer, trace, fingerprint, clock)
            transport.enable()
            if mode == 'local_model':
                outcome = experiment.calibrate()
                transport.finish()
                released = True
            else:
                _observe(experiment)
                captures = []
                for i, pose in enumerate(plan['poses']):
                    targets = {j: transport.origin[j]+offset for j, offset in pose['offset_ticks'].items()}
                    _move_to(experiment, targets)
                    # Read-only FK status first: nothing may sit between the pose frame and the next step.
                    arm_status = robot.call('robot_get_arm_pose', {'arm': config['arm']})
                    _observe(experiment)
                    sample = copy.deepcopy(observer.capture)
                    if sample['sample'] is None:
                        raise Refused(f'Pose {i}: {sample["sample_rejection"]}')
                    sample['sample']['split'] = pose['split']
                    sample['arm_geometry_status'] = arm_status
                    captures.append(sample)
                    atomic_json(output/f'pose-{i:02d}.json', sample)
                _move_to(experiment, {j: transport.origin[j] for j in transport.joints})
                # Offline FK/fitting can be slow; do not hold motors during it.
                transport.finish()
                released = True
                dataset = assemble_dataset(captures, config['model_directory'])
                atomic_json(output/'dataset.json', dataset)
                outcome = fit_registration(dataset)
            if 'motor_writes' in outcome:
                outcome['fitter_motor_writes'] = outcome.pop('motor_writes')
            outcome.update(gripper_tag_id=tag_id, calibration_commands_sent=transport.commands_sent, commanded_path_ticks=transport.path_ticks,
                           cleanup=transport.cleanup, output=str(output))
            atomic_json(output/'result.json', outcome)
            return outcome
        except BaseException as exc:
            try:
                if not released:
                    transport.finish(failed=True)
            except Exception as cleanup_error:
                transport.cleanup = {'release_confirmed': False, 'error': str(cleanup_error)}
            atomic_json(output/'failure.json', {'error': str(exc), 'calibration_commands_sent': transport.commands_sent,
                'write_attempted': transport.write_attempted, 'cleanup': transport.cleanup,
                'automatic_retry': False, 'physical_registration_validated': False})
            raise
        finally:
            trace.close()
