import copy
import hashlib

import cv2
import numpy as np
import pytest

from farm.kinematics.tag_registration import fit_registration, candidate_degrees


def pose(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = cv2.Rodrigues(np.asarray(rotation, float))[0]
    t[:3, 3] = translation
    return t


def dataset():
    rng = np.random.default_rng(17)
    camera = pose([.7, -.3, .2], [.45, -.15, .5])
    tag = pose([.1, -.2, .3], [.015, .006, -.04])
    binding = dict(arm='left', camera_id='test', stream_id='one', camera_calibration_sha256='K',
                   tag_geometry_sha256='sizes', robot_model_sha256='model', motor_calibration_sha256='motors',
                   encoder_mapping_source='synthetic known mapping', gripper_tag_id=4)
    binding['gripper_tag_mount'] = dict(arm='left', body='fixed_gripper_housing', source='test fixture')
    samples = []
    for i in range(15):
        gripper = pose(rng.uniform(-.7, .7, 3), rng.uniform([.15, -.1, .1], [.4, .15, .4]))
        observed = np.linalg.inv(camera) @ gripper @ tag
        samples.append(dict(arm='left', gripper_tag_id=4, frame=dict(camera_id='test', stream_id='one', seq=i,
            sha256=hashlib.sha256(str(i).encode()).hexdigest()),
            camera_calibration_sha256='K', tag_geometry_sha256='sizes', head_ticks=[2000, 2400],
            gripper_tag_mount=copy.deepcopy(binding['gripper_tag_mount']),
            anchor_center_camera_mm=[40, 20, 600], stationary_bracket_verified=True, orientation_ambiguous=False,
            anchor_corners_px=[[100,100],[150,100],[150,150],[100,150]],
            base_from_gripper=gripper.tolist(), camera_from_tag=observed.tolist(), split='train' if i<11 else 'validation'))
    return {'schema': 1, 'binding': binding, 'samples': samples}, camera, tag


def test_known_eye_to_hand_geometry_recovered_with_independent_validation():
    data, camera, tag = dataset()
    result = fit_registration(data)
    assert result['status'] == 'REGISTRATION_VALIDATED'
    assert np.array(result['base_from_camera']) == pytest.approx(camera, abs=1e-8)
    assert np.array(result['gripper_from_tag']) == pytest.approx(tag, abs=1e-8)
    assert result['residuals']['validation']['position_max_mm'] < 1e-6
    assert not result['motion_ready'] and result['motor_writes'] == 0
    # Bound to camera identity + geometry hash; the publisher session is provenance only.
    assert 'stream_id' not in result['binding'] and result['source_stream_id'] == 'one'
    assert result['binding']['camera_id'] == 'test' and result['binding']['camera_calibration_sha256'] == 'K'


def test_held_out_bad_pose_rejects_transform_instead_of_fitting_it_away():
    data, _, _ = dataset()
    data['samples'][-1]['camera_from_tag'][0][3] += .025
    result = fit_registration(data)
    assert result['status'] == 'REGISTRATION_REJECTED'
    assert result['base_from_camera'] is None and result['gripper_from_tag'] is None
    assert result['residuals']['validation']['position_max_mm'] > 20


def test_consistently_relabeling_right_tag_as_left_cannot_validate_a_fit():
    data, _, _ = dataset()
    data['binding']['gripper_tag_id'] = 2
    for sample in data['samples']:
        sample['gripper_tag_id'] = 2
    with pytest.raises(ValueError, match='Selected arm left requires gripper tag 4'):
        fit_registration(data)


@pytest.mark.parametrize('change', [
    lambda d: d['samples'][1].update(frame=d['samples'][0]['frame']),
    lambda d: d['samples'][1]['frame'].update(stream_id='restarted-mid-collection'),
    lambda d: d['samples'][2].update(head_ticks=[2020, 2400]),
    lambda d: d['samples'][3].update(anchor_center_camera_mm=[60, 20, 600]),
    lambda d: d['samples'][4].update(orientation_ambiguous=True),
    lambda d: d['samples'][5].update(stationary_bracket_verified=False),
    lambda d: d['samples'][6].update(camera_calibration_sha256='different'),
    lambda d: d['samples'][6].update(tag_geometry_sha256='different'),
    lambda d: d['samples'][6]['gripper_tag_mount'].update(arm='right'),
    lambda d: d['samples'][6].update(arm='right'),
    lambda d: d['samples'][6].update(gripper_tag_id=3),
    lambda d: d['binding'].pop('gripper_tag_mount'),
    lambda d: d['binding']['gripper_tag_mount'].update(body='moving_jaw'),
    lambda d: d['samples'][6].update(anchor_corners_px=[[110,100],[160,100],[160,150],[110,150]]),
    lambda d: d.update(samples=d['samples'][:8]),
])
def test_invalid_or_moving_observation_sets_are_refused(change):
    data, _, _ = dataset()
    change(data)
    with pytest.raises(ValueError):
        fit_registration(data)


def test_rotating_only_one_joint_axis_cannot_validate_registration():
    data, camera, tag = dataset()
    for i, sample in enumerate(data['samples']):
        g = pose([0, 0, i*.1], [.2+i*.01, 0, .2])
        sample['base_from_gripper'] = g.tolist()
        sample['camera_from_tag'] = (np.linalg.inv(camera) @ g @ tag).tolist()
    with pytest.raises(ValueError, match='two independent axes'):
        fit_registration(data)


def test_candidate_normalization_reuses_lerobot_degrees_and_rejects_out_of_range():
    names = ['left_arm_shoulder_pan', 'left_arm_elbow_flex']
    ranges = {n: {'min_ticks': 1000, 'max_ticks': 3000} for n in names}
    result = candidate_degrees(dict(zip(names, [2000, 2409.5])), ranges, names)
    assert result == pytest.approx([0, 36])
    with pytest.raises(ValueError, match='outside'):
        candidate_degrees(dict(zip(names, [999, 2000])), ranges, names)
