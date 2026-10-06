import copy

import numpy as np
import pytest

from farm.perception.tag_sampling import ARM_JOINTS, HEAD_JOINTS, stationary_sample


def bracket():
    names = ['left_arm_'+n for n in ARM_JOINTS] + list(HEAD_JOINTS)
    def state(stamp):
        return {'ok': True, 'result': {'cached': False, 'motors': [
            dict(name=n, Status=0, Moving=0, Present_Velocity=0, Present_Position=2000,
                 captured_at=stamp) for n in names]}}
    observation = {'frame': {'captured_at': 101, 'camera_id': 'oak', 'seq': 1,
                             'stream_id': 'one', 'sha256': '0'*64},
                   'tags': [dict(tag_id=1, status='DETECTED', corners_px=[[100,100],[150,100],[150,150],[100,150]])],
                   'pose_3d': {'status': 'CAMERA_RELATIVE_ESTIMATE',
                       'calibration_sha256': 'K', 'geometry_config_sha256': 'sizes',
                       'tags': [dict(tag_id=1, center_camera_mm=[0, 0, 600]),
                                dict(tag_id=2, center_camera_mm=[0, 0, 300],
                                     orientation_ambiguous=False, camera_from_tag=np.eye(4).tolist(),
                                     reprojection_rms_px=.1)]}}
    return state(100), state(102), observation


def test_stationary_bracket_has_provenance_but_no_invented_robot_pose():
    before, after, observation = bracket()
    sample = stationary_sample(before, after, observation, 'left')
    assert sample['stationary_bracket_verified']
    assert sample['capture_bracket_s'] == [100, 102]
    assert sample['base_from_gripper'] is None and sample['split'] is None
    assert sample['motor_writes'] == 0


@pytest.mark.parametrize('mutation', [
    lambda b,a,o: b.update(ok=False),
    lambda b,a,o: a['result'].update(cached=True),
    lambda b,a,o: a['result']['motors'][0].update(Status=4),
    lambda b,a,o: b['result']['motors'][1].update(Moving=1),
    lambda b,a,o: a['result']['motors'][2].update(Present_Velocity=10),
    lambda b,a,o: a['result']['motors'][3].update(Present_Position=2004),
    lambda b,a,o: a['result']['motors'][4].update(captured_at=float('nan')),
    lambda b,a,o: o['frame'].update(captured_at=99),
    lambda b,a,o: o['frame'].update(captured_at=103),
    lambda b,a,o: o['pose_3d']['tags'][1].update(orientation_ambiguous=True),
    lambda b,a,o: o['pose_3d']['tags'].pop(0),
])
def test_untrustworthy_brackets_cannot_become_registration_samples(mutation):
    before, after, observation = bracket()
    mutation(before, after, observation)
    with pytest.raises(ValueError):
        stationary_sample(before, after, observation, 'left')


def test_long_bracket_rejected_even_if_endpoints_look_stationary():
    before, after, observation = bracket()
    for row in after['result']['motors']:
        row['captured_at'] = 104
    with pytest.raises(ValueError, match='three seconds'):
        stationary_sample(before, after, observation, 'left')
