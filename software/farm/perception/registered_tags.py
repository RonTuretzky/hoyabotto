"""Use an independently validated hand-eye fit for read-only arm-base estimates.

No robot movement or calibration-register writes. A registered tag is still a
marker pose, not a jaw contact target or a collision-checked path.

Binding: a registration is tied to the camera (camera_id) and its geometry
hash (resolution, projection, intrinsics, effective distortion, optical frame,
lens position), never to the OAK stream_id, which changes on every publisher
restart. The first read from a new stream must pass the gripper-consistency
check (gripper tag versus arm model, 4 mm / 2 degrees) before any coordinates
are returned; every read runs that check on the same frame anyway.

Re-anchoring: both camera and arm ride on the cart, so a cart move or a shifted
table tag 1 does not change camera-to-arm. When head ticks are unchanged
(3 ticks) and the gripper-consistency check passes, the new tag-1 pixels become
the reference and the event is recorded. Head moves are never re-anchored.

The registration file stays immutable (its hash is bound elsewhere); the
mutable anchor/stream state lives in a separate binding-state record.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import time

import numpy as np

from farm.kinematics.lerobot import pose_error, transform
from farm.kinematics.tag_registration import assemble_dataset
from farm.perception.tag_geometry import fingerprint
from farm.perception.tag_sampling import gripper_tag_for_arm, stationary_sample

HEAD_TICK_LIMIT = 3
ANCHOR_CORNER_LIMIT_PX = 2
ANCHOR_CENTRE_LIMIT_MM = 5
CONSISTENCY_LIMIT_MM = 4
CONSISTENCY_LIMIT_DEGREES = 2
MAX_EVENTS = 50
REREGISTER = 'Fix: re-run robot_calibrate_tags {"mode": "registration"} (step 10).'
_GEOMETRY_FIELDS = (('camera_id', 'camera identity'), ('image_size_px', 'resolution'),
                    ('projection', 'projection'), ('intrinsics', 'intrinsics'),
                    ('effective_distortion', 'distortion coefficients'),
                    ('coordinate_frame', 'optical frame'), ('lens_position', 'lens position'))


def source_stream_id(registration):
    """Stream the registration was collected on (legacy files kept it in binding)."""
    return registration.get('source_stream_id') or registration.get('binding', {}).get('stream_id')


def binding_state(registration):
    """Initial mutable read state derived from an immutable registration."""
    return {'schema': 1, 'registration_sha256': fingerprint(registration),
            'verified_stream_id': source_stream_id(registration),
            'anchor_corners_px_reference': copy.deepcopy(registration.get('anchor_corners_px_reference')),
            'anchor_center_camera_mm_reference': copy.deepcopy(registration.get('anchor_center_camera_mm_reference')),
            'anchor_source': 'registration', 'events': []}


def load_binding_state(path, registration):
    """Saved state for THIS registration, or None (missing, or from an older registration)."""
    try:
        state = json.loads(Path(path).read_text())
    except FileNotFoundError:
        return None
    if (not isinstance(state, dict) or state.get('schema') != 1
            or state.get('registration_sha256') != fingerprint(registration)):
        return None
    return state


def _geometry_changes(registered, current):
    if not isinstance(registered, dict) or not isinstance(current, dict):
        return []
    changes = []
    for key, label in _GEOMETRY_FIELDS:
        if registered.get(key) != current.get(key):
            changes.append(label if key in ('intrinsics', 'effective_distortion')
                           else f'{label} {registered.get(key)!r} -> {current.get(key)!r}')
    return changes


def _check_binding(binding, sample, frame, model_sha256, ranges_sha256):
    """Refuse with the exact cause and fix; stream_id is deliberately not compared."""
    if binding.get('camera_id') != frame.get('camera_id'):
        raise ValueError(f'Camera identity changed: registered {binding.get("camera_id")!r}, '
                         f'this frame is from {frame.get("camera_id")!r}. Fix: reconnect the registered camera, '
                         f'or {REREGISTER[5:]}')
    if binding.get('camera_calibration_sha256') != sample['camera_calibration_sha256']:
        registered = binding.get('camera_geometry') or {}
        changes = _geometry_changes(registered, sample.get('camera_geometry'))
        detail = ', '.join(changes) if changes else 'resolution, projection, intrinsics or distortion hash differs'
        if any(c.startswith('projection') for c in changes):
            fix = (f'Fix: restart the OAK publisher in the registered mode (projection {registered.get("projection")!r}; '
                   "--wide gives 'camera_pinhole_with_factory_distortion', without --wide 'rectified_pinhole'), "
                   'or re-run registration (step 10).')
        else:
            fix = 'Fix: restart the OAK with the registered resolution and calibration, or re-run registration (step 10).'
        raise ValueError(f'OAK camera geometry changed since registration ({detail}). {fix}')
    if binding.get('tag_geometry_sha256') != sample['tag_geometry_sha256']:
        raise ValueError('AprilTag geometry file (apriltag-geometry.json: tag sizes, mounts, camera ids) changed '
                         'since registration. Fix: restore the file used at registration, or ' + REREGISTER[5:])
    if binding.get('gripper_tag_id') != sample['gripper_tag_id'] or binding.get('gripper_tag_mount') != sample['gripper_tag_mount']:
        raise ValueError(f'Gripper tag {sample["gripper_tag_id"]} identity or mount record changed since registration '
                         f'(registered {binding.get("gripper_tag_mount")!r}, now {sample["gripper_tag_mount"]!r}). '
                         'Fix: restore the mount record if unchanged physically; if the tag was re-mounted, ' + REREGISTER[5:])
    if binding.get('robot_model_sha256') != model_sha256:
        raise ValueError('SO-101 robot model (URDF) changed since registration. '
                         'Fix: restore the registered model, or redo steps 5 and 10.')
    if binding.get('raw_arm_ranges_sha256') != ranges_sha256:
        raise ValueError(f'{binding.get("arm")} arm saved calibration ranges changed since registration '
                         '(arm recalibrated). Fix: restore the previous motor calibration, or redo steps 5, 6 and 10-12.')


def registered_observation(before, after, observation, registration, arm_status,
                           model_sha256, base_from_gripper, *, state=None):
    if registration.get('status') != 'REGISTRATION_VALIDATED':
        raise ValueError('No independently validated registration is installed. ' + REREGISTER)
    binding = registration.get('binding', {})
    arm = binding.get('arm')
    tag_id = gripper_tag_for_arm(arm, binding.get('gripper_tag_id'))
    if state is None:
        state = binding_state(registration)
    elif state.get('registration_sha256') != fingerprint(registration):
        raise ValueError('Binding state belongs to a different registration; discard it and read again')
    sample = stationary_sample(before, after, observation, arm, gripper_tag_id=tag_id)
    frame = sample['frame']
    ranges = before['result'].get('raw_calibration_ranges', {})
    if ranges != after['result'].get('raw_calibration_ranges'):
        raise ValueError('Motor ranges changed during the capture')
    names = [f'{arm}_arm_{n}' for n in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')]
    if not all(n in ranges for n in names):
        raise ValueError('Missing selected arm calibration ranges')
    _check_binding(binding, sample, frame, model_sha256, fingerprint({n: ranges[n] for n in names}))
    cfg = arm_status.get('result', {}).get('configuration', {}).get('config') or {}
    if arm_status.get('ok') is not True or cfg.get('arm') != arm or cfg.get('mapping') != 'feetech_degrees_v1':
        raise ValueError(f'Robot API does not report the feetech_degrees_v1 {arm}-arm mapping used by the registration. '
                         'Fix: reinstall the registered arm configuration, or ' + REREGISTER[5:])
    if cfg.get('calibration_sha256') != binding.get('motor_calibration_sha256'):
        raise ValueError(f'{arm} arm motor calibration hash changed since registration (arm recalibrated). '
                         'Fix: restore the previous motor calibration, or redo steps 5, 6 and 10-12.')
    for name in ('train', 'validation'):
        residual = registration.get('residuals', {}).get(name, {})
        if (residual.get('count', 0) < (8 if name == 'train' else 3)
                or not all(np.isfinite(residual.get(k, np.inf)) and residual[k] <= bound
                           for k, bound in [('position_rms_mm',2),('position_max_mm',4),('orientation_max_degrees',2)])):
            raise ValueError('Registration lacks passing fitting and independent-validation residuals. ' + REREGISTER)
    head, head_reference = np.asarray(sample['head_ticks'], float), np.asarray(registration['head_ticks_reference'], float)
    head_shift = float(np.max(np.abs(head-head_reference)))
    if head_shift > HEAD_TICK_LIMIT:
        raise ValueError(f'Head moved since registration: head ticks {head.round(1).tolist()} vs registered '
                         f'{head_reference.tolist()} ({head_shift:.0f} > {HEAD_TICK_LIMIT} ticks), so camera-to-arm changed. '
                         f'Fix: return the head to {head_reference.tolist()} (within {HEAD_TICK_LIMIT} ticks), or ' + REREGISTER[5:])

    # Gripper-consistency check runs before any re-anchor or new-stream acceptance.
    base_from_camera = transform(registration['base_from_camera'])
    expected_gripper_tag = transform(base_from_gripper) @ transform(registration['gripper_from_tag'])
    observed_gripper_tag = base_from_camera @ transform(sample['camera_from_tag'])
    residual = pose_error(expected_gripper_tag, observed_gripper_tag)
    consistency = {'position_mm': residual[0]*1000, 'orientation_degrees': residual[1],
                   'limits': {'position_mm': CONSISTENCY_LIMIT_MM, 'orientation_degrees': CONSISTENCY_LIMIT_DEGREES}}
    consistent = consistency['position_mm'] <= CONSISTENCY_LIMIT_MM and residual[1] <= CONSISTENCY_LIMIT_DEGREES

    corners, centre = np.asarray(sample['anchor_corners_px'], float), np.asarray(sample['anchor_center_camera_mm'], float)
    corner_shift = float(np.max(np.linalg.norm(corners-np.asarray(state['anchor_corners_px_reference'], float), axis=1)))
    centre_shift = float(np.linalg.norm(centre-np.asarray(state['anchor_center_camera_mm_reference'], float)))
    anchor_moved = corner_shift > ANCHOR_CORNER_LIMIT_PX or centre_shift > ANCHOR_CENTRE_LIMIT_MM
    new_stream = frame.get('stream_id') != state.get('verified_stream_id')

    if not consistent:
        cause = (f'Gripper tag {tag_id} disagrees with the arm model by {consistency["position_mm"]:.1f} mm / '
                 f'{residual[1]:.1f} deg (limits {CONSISTENCY_LIMIT_MM} mm / {CONSISTENCY_LIMIT_DEGREES} deg)')
        context = []
        if new_stream:
            context.append(f'OAK publisher restarted (stream {state.get("verified_stream_id")} -> {frame.get("stream_id")}) '
                           'and the required post-restart gripper-consistency check failed')
        if anchor_moved:
            context.append(f'table tag 1 moved ({corner_shift:.1f} px, {centre_shift:.1f} mm) and cannot be re-anchored '
                           'without a passing gripper-consistency check')
        prefix = ('; '.join(context) + ': ') if context else ''
        raise ValueError(prefix[:1].upper() + prefix[1:] + cause + '. Likely tag 2 bumped or re-mounted, the arm calibration '
                         'drifted, or the camera moved relative to the arm. Fix: check the tag mount and arm, keep the gripper '
                         'tag fully in view and still; if it still fails, ' + REREGISTER[5:])

    state = copy.deepcopy(state)
    events = []
    identity = {k: frame.get(k) for k in ('camera_id', 'stream_id', 'seq', 'sha256', 'captured_at')}
    if new_stream:
        events.append({'event': 'camera_stream_reverified', 'frame': identity,
                       'previous_stream_id': state.get('verified_stream_id'), 'stream_id': frame.get('stream_id'),
                       'gripper_consistency': consistency,
                       'reason': 'New OAK publisher session with the registered camera geometry; '
                                 'gripper-consistency check passed before the first registered read.'})
        state['verified_stream_id'] = frame.get('stream_id')
    if anchor_moved:
        events.append({'event': 'table_anchor_re_anchored', 'frame': identity,
                       'previous_anchor_corners_px': state['anchor_corners_px_reference'],
                       'previous_anchor_center_camera_mm': state['anchor_center_camera_mm_reference'],
                       'anchor_corners_px': corners.tolist(), 'anchor_center_camera_mm': centre.tolist(),
                       'anchor_corner_shift_px': corner_shift, 'anchor_centre_shift_mm': centre_shift,
                       'head_ticks': head.tolist(), 'head_ticks_reference': head_reference.tolist(),
                       'gripper_consistency': consistency,
                       'reason': 'Table tag 1 moved relative to the camera (cart move/base pulse or tag 1 shifted) with '
                                 'head ticks unchanged and a passing gripper-consistency check; camera-to-arm is '
                                 'unchanged, so the new tag-1 pixels are the reference.'})
        state.update(anchor_corners_px_reference=corners.tolist(), anchor_center_camera_mm_reference=centre.tolist(),
                     anchor_source='re_anchor')
    state['events'] = (state.get('events', []) + events)[-MAX_EVENTS:]

    tags = []
    for tag in observation['pose_3d']['tags']:
        if tag.get('center_camera_mm') is None:
            continue
        point = base_from_camera @ np.r_[np.asarray(tag['center_camera_mm'])/1000,1]
        pose = (None if tag.get('camera_from_tag') is None or tag.get('orientation_ambiguous') is not False
                else base_from_camera @ transform(tag['camera_from_tag']))
        tags.append({'tag_id':tag['tag_id'], 'center_arm_base_mm':(point[:3]*1000).tolist(),
                     'arm_base_from_tag':None if pose is None else pose.tolist(),
                     'orientation_ambiguous':tag.get('orientation_ambiguous',True),
                     'coarse_position_only':tag.get('coarse_position_only',True)})
    return {'status':'REGISTERED_TAG_ESTIMATES', 'arm':arm, 'frame':frame,
            'registration_sha256':fingerprint(registration), 'tags':tags,
            'transform_translation_units':'metres', 'point_units':'millimetres',
            'current_gripper_consistency':{'position_mm':consistency['position_mm'],
                                           'orientation_degrees':consistency['orientation_degrees']},
            'binding_events':events, 'binding_state':state,
            'camera_stream_reverified':new_stream, 're_anchored':anchor_moved,
            'motor_writes':0, 'robot_motion_target':False, 'physical_grasp_validated':False,
            'note':'Marker poses only. Contact offsets, collision clearance and physical accuracy require separate validation.'}


def read_registered_tags(robot, config, registration, *, clock=time.time, state=None):
    """Fresh rendered/live tags through the existing owner; never enables motors.

    ``state`` is the saved binding state (verified stream, current table-anchor
    reference); the returned ``result.binding_state`` is what to save next.
    """
    if registration.get('binding', {}).get('arm') != config['arm']:
        raise ValueError('Installed registration belongs to a different arm')
    tag_id = gripper_tag_for_arm(config['arm'], config.get('gripper_tag_id'))
    if gripper_tag_for_arm(config['arm'], registration['binding'].get('gripper_tag_id')) != tag_id:
        raise ValueError('Installed registration belongs to a different gripper tag')
    before = robot.call('robot_get_state', {'fresh': True})
    payload = robot.call('robot_get_tags', {'cameras': [config.get('camera', 'oak')], 'tag_ids': [1, tag_id, 3]})
    after = robot.call('robot_get_state', {'fresh': True})
    if payload.get('ok') is not True:
        raise ValueError('Fresh tag observation failed')
    row = payload['result']['observations'][config.get('camera', 'oak')]
    frame = row.get('frame', {})
    age = clock() - frame.get('captured_at', float('-inf'))
    if frame.get('timestamp_basis') != 'capture' or not 0 <= age <= 3:
        raise ValueError('Need a fresh capture timestamp for registered coordinates')
    sample = stationary_sample(before, after, row, config['arm'], gripper_tag_id=tag_id)
    status = robot.call('robot_get_arm_pose', {'arm': config['arm']})
    dataset = assemble_dataset([{'before':before, 'after':after, 'sample':sample,
                                 'arm_geometry_status':status}], config['model_directory'])
    estimate = registered_observation(before, after, row, registration, status,
                                     dataset['binding']['robot_model_sha256'],
                                     dataset['samples'][0]['base_from_gripper'], state=state)
    return {'ok':True, 'result':estimate, 'images':payload.get('images', []), 'motor_writes':0}
