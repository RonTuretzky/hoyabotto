"""Read-only paddle-handle guidance in the registered arm base.

Combines a passing camera-to-arm registration (the same checks as
robot_get_registered_tags), a fresh paddle tag 3 pose and owner-measured
offsets from apriltag-geometry.json. The result is information for the pilot:
this module sends nothing to the motor owner. A reach-planner proposal, when
present, is labelled as such and must not be replayed as one move.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np

from farm.kinematics.lerobot import pose_error, transform
from farm.perception.registered_tags import read_registered_tags
from farm.perception.tag_geometry import fingerprint

TOOL_NAME = 'robot_get_paddle_target'
SECTION = 'paddle_grasp'
PADDLE_TAG_ID = 3
PROPOSAL_LABEL = 'proposal, not executed, not collision checked'
# The registered read already refuses frames older than 3 s. The planner call
# adds latency, so the assembled answer gets its own explicit ceiling.
MAX_RESULT_AGE_S = 6.0
DEFAULT_STANDOFF_MM = 60.0
# Printed tag frame (tag upright as in the kit image; +x toward its right edge,
# +y toward its top edge, +z out of the printed face) expressed in the decoded
# IPPE frame that robot_get_tags returns. Checked against rendered detections
# in tests/test_paddle_target.py.
DECODED_FROM_PRINTED = np.diag([-1., 1., -1.])

UNMEASURED = {
    'arm': 'right',
    'tag_id': PADDLE_TAG_ID,
    'tag_to_handle': {'status': 'unmeasured', 'handle_center_mm': None, 'approach_direction': None,
                      'jaw_closing_axis': None, 'tolerance_mm': None, 'source': None},
    'jaw_contact': {'status': 'unmeasured', 'gripper_from_jaw_contact': None,
                    'tolerance_mm': None, 'source': None},
    'pregrasp_standoff_mm': DEFAULT_STANDOFF_MM,
}

REACH_PROCEDURE = [
    'This target is guidance only. Nothing here moves the arm, and the planner proposal is never sent as one move.',
    'Move in small segments with the existing motion tools (robot_move_joint_targets, or robot_move_path with '
    'wait=false and robot_get_motion), changing each joint by a small part of the remaining difference.',
    'After every segment: look at the cameras (robot_get_cameras oak + right_wrist), then, with the arm settled, '
    'call robot_get_paddle_target again to re-detect tag 3 (it refuses while joints move and needs tags 1 and 2 '
    'in view). Halt (robot_halt_motion) or STOP if anything looks wrong.',
    'Stop at the pre-grasp. Do the final alignment and the grasp with the camera-guided paddle-success-v1 steps.',
    'If any read refuses (registration, stale frame, tag 3 missing or ambiguous), stop moving toward the target and '
    'continue camera-guided only.',
]


def _vector(value, name, *, unit=False, limit=None):
    v = np.asarray(value, dtype=float) if isinstance(value, list) else None
    if v is None or v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError(f'{name} must be three finite numbers')
    if unit:
        norm = float(np.linalg.norm(v))
        if not .98 <= norm <= 1.02:
            raise ValueError(f'{name} must be a unit vector')
        v = v / norm
    if limit is not None and np.linalg.norm(v) > limit:
        raise ValueError(f'{name} exceeds {limit} mm')
    return v


def _tolerance(entry, name):
    tol = entry.get('tolerance_mm')
    if type(tol) not in (int, float) or not math.isfinite(tol) or not 0 < tol <= 20:
        raise ValueError(f'{name}.tolerance_mm must be the measurement tolerance, 0-20 mm')
    if not isinstance(entry.get('source'), str) or not entry['source'].strip():
        raise ValueError(f'{name}.source must say how and when it was measured')
    return float(tol)


def grasp_geometry(config, arm):
    """Parse the owner-measured paddle_grasp section; unmeasured fields stay None."""
    section = (config or {}).get(SECTION)
    out = {'handle': None, 'jaw': None, 'standoff_mm': DEFAULT_STANDOFF_MM, 'sha256': None, 'unmeasured': []}
    if section is None:
        out['unmeasured'] = [f'{SECTION} (section absent from apriltag-geometry.json)']
        return out
    if not isinstance(section, dict) or section.get('arm') != arm or section.get('tag_id') != PADDLE_TAG_ID:
        raise ValueError(f'{SECTION} must name arm "{arm}" and tag_id {PADDLE_TAG_ID}')
    out['sha256'] = fingerprint(section)
    standoff = section.get('pregrasp_standoff_mm', DEFAULT_STANDOFF_MM)
    if type(standoff) not in (int, float) or not 20 <= standoff <= 150:
        raise ValueError(f'{SECTION}.pregrasp_standoff_mm must be 20-150 mm')
    out['standoff_mm'] = float(standoff)
    handle = section.get('tag_to_handle')
    name = f'{SECTION}.tag_to_handle'
    if not isinstance(handle, dict) or handle.get('status') not in ('unmeasured', 'measured'):
        raise ValueError(f'{name}.status must be "unmeasured" or "measured"')
    if handle['status'] == 'unmeasured':
        out['unmeasured'].append(name)
    else:
        center = _vector(handle.get('handle_center_mm'), name + '.handle_center_mm', limit=300)
        approach = _vector(handle.get('approach_direction'), name + '.approach_direction', unit=True)
        closing = _vector(handle.get('jaw_closing_axis'), name + '.jaw_closing_axis', unit=True)
        if abs(float(approach @ closing)) > .05:
            raise ValueError(f'{name}: approach_direction and jaw_closing_axis must be perpendicular')
        closing = closing - (approach @ closing) * approach
        closing /= np.linalg.norm(closing)
        out['handle'] = {'center_mm_decoded': DECODED_FROM_PRINTED @ center,
                         'approach_decoded': DECODED_FROM_PRINTED @ approach,
                         'closing_decoded': DECODED_FROM_PRINTED @ closing,
                         'lever_mm': float(np.linalg.norm(center)),
                         'tolerance_mm': _tolerance(handle, name), 'source': handle['source']}
    jaw = section.get('jaw_contact')
    name = f'{SECTION}.jaw_contact'
    if not isinstance(jaw, dict) or jaw.get('status') not in ('unmeasured', 'measured'):
        raise ValueError(f'{name}.status must be "unmeasured" or "measured"')
    if jaw['status'] == 'unmeasured':
        out['unmeasured'].append(name)
    else:
        offset = transform(jaw.get('gripper_from_jaw_contact'))
        if np.linalg.norm(offset[:3, 3]) > .25:
            raise ValueError(f'{name}.gripper_from_jaw_contact translation exceeds 0.25 m')
        out['jaw'] = {'gripper_from_jaw_contact': offset, 'tolerance_mm': _tolerance(jaw, name),
                      'source': jaw['source']}
    return out


def _age(frame, clock, limit):
    captured = frame.get('captured_at')
    if frame.get('timestamp_basis') != 'capture' or type(captured) not in (int, float) or not math.isfinite(captured):
        raise ValueError('Stale or untimed frame: need a capture timestamp; re-request')
    age = clock() - captured
    if not 0 <= age <= limit:
        raise ValueError(f'Stale frame: tag 3 observation is {age:.2f} s old (limit {limit} s); re-request')
    return age


def _uncertainty(registered, registration, tag, lever_mm=0., tolerance_mm=0.):
    residuals = registration.get('residuals', {})
    groups = [residuals.get('train', {}), residuals.get('validation', {})]
    reg_rms = max(float(g['position_rms_mm']) for g in groups)
    reg_max = max(float(g['position_max_mm']) for g in groups)
    reg_deg = max(float(g['orientation_max_degrees']) for g in groups)
    live = registered.get('current_gripper_consistency', {})
    live_mm, live_deg = float(live.get('position_mm', 0.)), float(live.get('orientation_degrees', 0.))
    std = tag.get('position_std_mm_camera_axes')
    tag_rss = None if std is None else float(np.linalg.norm(std))
    # Tag orientation error is not reported per tag; the registration and live
    # gripper-tag orientation disagreement stand in for it on the offset lever.
    lever_term = lever_mm * math.radians(reg_deg + live_deg)
    terms = [reg_rms, tag_rss or 0., lever_term, tolerance_mm, live_mm]
    return {'registration_position_rms_mm': reg_rms, 'registration_position_max_mm': reg_max,
            'registration_orientation_max_degrees': reg_deg,
            'live_gripper_consistency_mm': live_mm, 'live_gripper_consistency_degrees': live_deg,
            'tag3_position_std_mm_camera_axes': std,
            'tag3_position_std_mm_arm_base_axes': tag.get('position_std_mm_arm_base_axes'),
            'tag3_depth_std_mm': None if std is None else float(std[2]),
            'tag3_reprojection_rms_px': tag.get('reprojection_rms_px'),
            'tag3_coarse_position_only': tag.get('coarse_position_only', True),
            'offset_lever_mm': lever_mm, 'orientation_lever_term_mm': lever_term,
            'declared_offset_tolerance_mm': tolerance_mm,
            'combined_rss_mm': float(math.sqrt(sum(t*t for t in terms))),
            'conservative_bound_mm': reg_max + 3*(tag_rss or 0.) + lever_term + tolerance_mm + live_mm,
            'tag_noise_available': std is not None,
            'basis': ('Registration residuals, the live gripper-tag check, tag-3 corner noise (0.5 px assumed; '
                      'the camera z axis is the monocular depth direction, OAK stereo depth is not fused), '
                      'orientation error on the offset lever and the declared offset tolerance.'),
            'excludes': ['print_size', 'camera_intrinsics', 'paper_warp', 'joint_backlash', 'unsampled_workspace']}


def paddle_target(registered, registration, grasp, *, clock=time.time, max_age_s=MAX_RESULT_AGE_S):
    """Pure computation from a registered read; no robot calls."""
    arm = registered['arm']
    frame = registered['frame']
    age = _age(frame, clock, max_age_s)
    tag = next((t for t in registered.get('tags', []) if t.get('tag_id') == PADDLE_TAG_ID), None)
    if tag is None:
        raise ValueError('Paddle tag 3 has no metric detection in this fresh frame; no target. '
                         'Check the cameras and re-request.')
    pose = None if tag.get('arm_base_from_tag') is None else transform(tag['arm_base_from_tag'])
    result = {
        'status': 'PADDLE_TAG_POSE_ONLY', 'arm': arm, 'frame_id': f'{arm}_arm_base',
        'units': {'points': 'millimetres', 'transforms': 'metres (4x4, translation in metres)'},
        'paddle_tag': {'tag_id': PADDLE_TAG_ID, 'center_arm_base_mm': tag['center_arm_base_mm'],
                       'arm_base_from_tag': None if pose is None else pose.tolist(),
                       'orientation_ambiguous': tag.get('orientation_ambiguous', True),
                       'tag_frame': 'decoded AprilTag frame as returned by robot_get_tags (printed frame rotated 180 deg about +y)'},
        'grasp': None, 'grasp_unavailable_reason': None, 'tool_pose_unavailable_reason': None,
        'unmeasured_fields': list(grasp['unmeasured']),
        'uncertainty': _uncertainty(registered, registration, tag),
        'freshness': {'camera_id': frame.get('camera_id'), 'stream_id': frame.get('stream_id'),
                      'seq': frame.get('seq'), 'frame_sha256': frame.get('sha256'),
                      'captured_at': frame.get('captured_at'), 'timestamp_basis': frame.get('timestamp_basis'),
                      'age_s_at_target': age, 'max_age_s': max_age_s,
                      'camera_frame': registered.get('camera_frame')},
        'registration_sha256': registered.get('registration_sha256'),
        'grasp_geometry_sha256': grasp['sha256'],
        'reach_proposal': None,
        'motor_writes': 0, 'executed': False, 'robot_motion_target': False, 'collision_checked': False,
        'physical_grasp_validated': False, 'reach_procedure': REACH_PROCEDURE,
    }
    if pose is None:
        result['grasp_unavailable_reason'] = ('Tag 3 orientation is ambiguous: only its centre is known, so the handle '
                                              'offset cannot be applied. Change the view and re-request.')
        return result
    if grasp['handle'] is None:
        result['grasp_unavailable_reason'] = ('Grasp point withheld: the tag-3 to handle offset is unmeasured. '
                                              'Tag pose only. Measure it and record it in apriltag-geometry.json.')
        return result
    handle = grasp['handle']
    rotation = pose[:3, :3]
    point_m = pose @ np.r_[handle['center_mm_decoded'] / 1000, 1]
    approach = rotation @ handle['approach_decoded']
    closing = rotation @ handle['closing_decoded']
    tolerance = handle['tolerance_mm']
    out = {'handle_point_arm_base_mm': (point_m[:3]*1000).tolist(),
           'approach_direction_arm_base': approach.tolist(),
           'jaw_closing_axis_arm_base': closing.tolist(),
           'pregrasp_standoff_mm': grasp['standoff_mm'],
           'pregrasp_point_arm_base_mm': ((point_m[:3] - approach*grasp['standoff_mm']/1000)*1000).tolist(),
           'pregrasp_height_above_handle_mm': float(-approach[2]*grasp['standoff_mm']),
           'grasp_tool_pose_arm_base': None, 'pregrasp_tool_pose_arm_base': None,
           'offset_sources': {'tag_to_handle': handle['source']}}
    if grasp['jaw'] is None:
        result['tool_pose_unavailable_reason'] = ('Jaw contact offset unmeasured: handle point and approach only; '
                                                  'no jaw tool poses and no reach proposal.')
    else:
        tool = np.eye(4)
        tool[:3, :3] = np.column_stack([np.cross(closing, approach), closing, approach])
        tool[:3, 3] = point_m[:3]
        pregrasp = tool.copy()
        pregrasp[:3, 3] -= approach * grasp['standoff_mm'] / 1000
        out.update(grasp_tool_pose_arm_base=tool.tolist(), pregrasp_tool_pose_arm_base=pregrasp.tolist(),
                   tool_frame=('jaw contact frame: +z = approach (out of the jaws toward the handle), '
                               '+y = jaw closing axis'))
        out['offset_sources']['jaw_contact'] = grasp['jaw']['source']
        tolerance = math.hypot(tolerance, grasp['jaw']['tolerance_mm'])
    result.update(status='PADDLE_TARGET_GUIDANCE_ONLY', grasp=out,
                  uncertainty=_uncertainty(registered, registration, tag, handle['lever_mm'], tolerance))
    return result


def reach_proposal(robot, arm, pregrasp_tool, grasp, registration):
    """Ask the existing read-only planner; keep its ticks only if it targets the same jaw and calibration."""
    request = {'arm': arm, 'frame': 'arm_base', 'units': 'metres', 'orientation': 'constrained',
               'tool_poses': [pregrasp_tool]}
    out = {'available': False, 'label': PROPOSAL_LABEL, 'pose': 'pregrasp', 'executed': False,
           'collision_checked': False, 'send_to_motor_owner': False, 'reason': None,
           'note': 'Never send these ticks as one move. Use them only to judge direction and size of the '
                   'small segments in reach_procedure.'}
    try:
        response = robot.call('robot_plan_reach', request)
    except Exception as exc:  # A remote failure leaves the read-only target intact.
        out['reason'] = f'robot_plan_reach failed: {exc}'
        return out
    result = response.get('result') if isinstance(response, dict) else None
    result = result if isinstance(result, dict) else {}
    if not isinstance(response, dict) or response.get('ok') is not True:
        out['reason'] = 'robot_plan_reach refused: ' + str(result.get('error', 'no reason given'))
        return out
    if result.get('status') != 'KINEMATIC_PROPOSAL_ONLY':
        missing = (result.get('configuration') or {}).get('missing') or []
        out['reason'] = f'Planner not configured ({result.get("status")}): missing ' + (', '.join(missing) or 'unknown')
        return out
    config = (result.get('configuration') or {}).get('config') or {}
    if config.get('arm') != arm or config.get('calibration_sha256') != registration['binding'].get('motor_calibration_sha256'):
        out['reason'] = 'Planner configuration differs from the registration binding (arm or motor calibration)'
        return out
    try:
        planner_tool = transform(config.get('gripper_from_tool'))
    except (ValueError, TypeError):
        out['reason'] = 'Planner configuration has no valid gripper_from_tool'
        return out
    error_m, error_deg = pose_error(planner_tool, grasp['jaw']['gripper_from_jaw_contact'])
    if error_m > .001 or error_deg > 1:
        out['reason'] = (f'Planner gripper_from_tool differs from apriltag-geometry jaw_contact by {error_m*1000:.1f} mm / '
                         f'{error_deg:.1f} deg; its ticks would target a different point')
        return out
    waypoints = result.get('waypoints') or []
    if len(waypoints) != 1 or not isinstance(waypoints[0].get('joint_targets_ticks'), dict):
        out['reason'] = 'Planner returned no single pre-grasp waypoint'
        return out
    ticks = waypoints[0]['joint_targets_ticks']
    start = result.get('starting_ticks') or {}
    changes = {n: ticks[n] - start[n] for n in ticks if type(start.get(n)) in (int, float)}
    out.update(available=True, joint_targets_ticks=ticks, starting_ticks=start, joint_change_ticks=changes,
               largest_joint_change_ticks=max((abs(v) for v in changes.values()), default=None),
               planned_tool_pose_arm_base=waypoints[0].get('tool_pose'),
               position_error_m=waypoints[0].get('position_error_m'),
               orientation_error_deg=waypoints[0].get('orientation_error_deg'),
               kinematics_fingerprint=result.get('kinematics_fingerprint'), planner_status=result['status'])
    return out


def read_paddle_target(robot, config, registration_path, geometry_path, *, include_proposal=True,
                       clock=time.time, max_age_s=MAX_RESULT_AGE_S, binding_state=None):
    """Fresh registered read -> paddle target. Read-only robot calls only.

    ``binding_state`` is the saved verified-stream / table-anchor state; it is used, never written."""
    path = Path(geometry_path)
    geometry = json.loads(path.read_text()) if path.is_file() else {}
    grasp = grasp_geometry(geometry, config['arm'])
    try:
        registration = json.loads(Path(registration_path).read_text())
    except (OSError, ValueError) as exc:
        raise ValueError(f'Registration unavailable: {exc}') from exc
    try:
        registered = read_registered_tags(robot, config, registration, clock=clock, state=binding_state)
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError(f'robot_get_registered_tags refuses: {exc}') from exc
    result = paddle_target(registered['result'], registration, grasp, clock=clock, max_age_s=max_age_s)
    pregrasp = (result['grasp'] or {}).get('pregrasp_tool_pose_arm_base')
    if include_proposal and pregrasp is not None:
        result['reach_proposal'] = reach_proposal(robot, config['arm'], pregrasp, grasp, registration)
    elif include_proposal:
        result['reach_proposal'] = {'available': False, 'label': PROPOSAL_LABEL,
                                    'reason': 'No pre-grasp tool pose (see grasp_unavailable_reason / '
                                              'tool_pose_unavailable_reason)'}
    # The planner adds latency: the answer as a whole must still be fresh.
    result['freshness']['age_s_at_return'] = _age(registered['result']['frame'], clock, max_age_s)
    return {'ok': True, 'result': result, 'images': registered.get('images', []), 'motor_writes': 0}
