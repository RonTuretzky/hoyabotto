"""Use an independently validated hand-eye fit for read-only arm-base estimates.

No robot movement or calibration-register writes. A registered tag is still a
marker pose, not a jaw contact target or a collision-checked path.
"""
from __future__ import annotations

import numpy as np
import time

from farm.kinematics.lerobot import pose_error, transform
from farm.kinematics.tag_registration import assemble_dataset
from farm.perception.tag_geometry import fingerprint
from farm.perception.tag_sampling import gripper_tag_for_arm, stationary_sample


def registered_observation(before, after, observation, registration, arm_status,
                           model_sha256, base_from_gripper):
    if registration.get('status') != 'REGISTRATION_VALIDATED':
        raise ValueError('No independently validated registration is installed')
    binding = registration.get('binding', {})
    arm = binding.get('arm')
    tag_id = gripper_tag_for_arm(arm, binding.get('gripper_tag_id'))
    sample = stationary_sample(before, after, observation, arm, gripper_tag_id=tag_id)
    frame = sample['frame']
    ranges = before['result'].get('raw_calibration_ranges', {})
    if ranges != after['result'].get('raw_calibration_ranges'):
        raise ValueError('Motor ranges changed during the capture')
    names = [f'{arm}_arm_{n}' for n in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')]
    if not all(n in ranges for n in names):
        raise ValueError('Missing selected arm calibration ranges')
    expected = {'camera_id':frame.get('camera_id'), 'stream_id':frame.get('stream_id'),
                'camera_calibration_sha256':sample['camera_calibration_sha256'],
                'tag_geometry_sha256':sample['tag_geometry_sha256'],
                'gripper_tag_id':sample['gripper_tag_id'], 'gripper_tag_mount':sample['gripper_tag_mount'],
                'robot_model_sha256':model_sha256,
                'raw_arm_ranges_sha256':fingerprint({n:ranges[n] for n in names})}
    if any(binding.get(k) != v for k,v in expected.items()):
        raise ValueError('Camera, tag mounting, model or intrinsics changed since registration')
    cfg = arm_status.get('result', {}).get('configuration', {}).get('config') or {}
    if (arm_status.get('ok') is not True or cfg.get('arm') != arm or cfg.get('mapping') != 'feetech_degrees_v1'
            or cfg.get('calibration_sha256') != binding.get('motor_calibration_sha256')):
        raise ValueError('Arm encoder mapping/calibration no longer matches the fitted data')
    for name in ('train', 'validation'):
        residual = registration.get('residuals', {}).get(name, {})
        if (residual.get('count', 0) < (8 if name == 'train' else 3)
                or not all(np.isfinite(residual.get(k, np.inf)) and residual[k] <= bound
                           for k, bound in [('position_rms_mm',2),('position_max_mm',4),('orientation_max_degrees',2)])):
            raise ValueError('Registration lacks passing fitting and independent-validation residuals')
    if np.max(np.abs(np.asarray(sample['head_ticks'])-registration['head_ticks_reference'])) > 3:
        raise ValueError('Head/camera position changed since registration')
    if (np.linalg.norm(np.asarray(sample['anchor_center_camera_mm'])-registration['anchor_center_camera_mm_reference']) > 5
            or np.max(np.linalg.norm(np.asarray(sample['anchor_corners_px'])-registration['anchor_corners_px_reference'],axis=1)) > 2):
        raise ValueError('Fixed table marker moved relative to the camera')
    base_from_camera = transform(registration['base_from_camera'])
    expected_gripper_tag = transform(base_from_gripper) @ transform(registration['gripper_from_tag'])
    observed_gripper_tag = base_from_camera @ transform(sample['camera_from_tag'])
    residual = pose_error(expected_gripper_tag, observed_gripper_tag)
    if residual[0] > .004 or residual[1] > 2:
        raise ValueError('Current gripper-tag observation disagrees with the fitted model; revalidate the mapping/mount')
    tags = []
    for tag in observation['pose_3d']['tags']:
        if tag.get('center_camera_mm') is None:
            continue
        point = base_from_camera @ np.r_[np.asarray(tag['center_camera_mm'])/1000,1]
        pose = (None if tag.get('camera_from_tag') is None or tag.get('orientation_ambiguous') is not False
                else base_from_camera @ transform(tag['camera_from_tag']))
        std = tag.get('position_std_mm_at_assumed_half_pixel_noise')
        std_base = None
        if std is not None and np.shape(std) == (3,) and np.isfinite(std).all():
            rotation = base_from_camera[:3, :3]
            std_base = np.sqrt(np.diag(rotation @ np.diag(np.square(std)) @ rotation.T)).tolist()
        else:
            std = None
        tags.append({'tag_id':tag['tag_id'], 'center_arm_base_mm':(point[:3]*1000).tolist(),
                     'arm_base_from_tag':None if pose is None else pose.tolist(),
                     'orientation_ambiguous':tag.get('orientation_ambiguous',True),
                     'coarse_position_only':tag.get('coarse_position_only',True),
                     'reprojection_rms_px':tag.get('reprojection_rms_px'),
                     'position_std_mm_camera_axes':None if std is None else list(std),
                     'position_std_mm_arm_base_axes':std_base})
    return {'status':'REGISTERED_TAG_ESTIMATES', 'arm':arm, 'frame':frame,
            'camera_frame':(observation.get('pose_3d') or {}).get('coordinate_frame'),
            'registration_sha256':fingerprint(registration), 'tags':tags,
            'transform_translation_units':'metres', 'point_units':'millimetres',
            'current_gripper_consistency':{'position_mm':residual[0]*1000,'orientation_degrees':residual[1]},
            'motor_writes':0, 'robot_motion_target':False, 'physical_grasp_validated':False,
            'note':'Marker poses only. Contact offsets, collision clearance and physical accuracy require separate validation.'}


def read_registered_tags(robot, config, registration, *, clock=time.time):
    """Fresh rendered/live tags through the existing owner; never enables motors."""
    if registration.get('binding', {}).get('arm') != config['arm']:
        raise ValueError('Installed registration belongs to a different arm')
    tag_id = gripper_tag_for_arm(config['arm'], config.get('gripper_tag_id'))
    if gripper_tag_for_arm(config['arm'], registration['binding'].get('gripper_tag_id')) != tag_id:
        raise ValueError('Installed registration belongs to a different gripper tag')
    before = robot.call('robot_get_state', {'fresh': True})
    payload = robot.call('robot_get_tags', {'cameras': [config.get('camera', 'oak')], 'tag_ids': [1, tag_id, 3]})
    after = robot.call('robot_get_state', {'fresh': True})
    if payload.get('ok') is not True:
        reason = ((payload.get('result') or {}).get('observations', {})
                  .get(config.get('camera', 'oak'), {}).get('reason'))
        raise ValueError('Fresh tag observation failed' + (f': {reason}' if reason else ''))
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
                                     dataset['samples'][0]['base_from_gripper'])
    return {'ok':True, 'result':estimate, 'images':payload.get('images', []), 'motor_writes':0}
