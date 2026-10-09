"""Real tag pixels, identity/freshness rejection, and camera-only dispatch."""
import copy

import numpy as np
import pytest

from carton.servo.tag_kit import marker_image
from farm.perception.carton_tags import CARTON_ROLES, CartonTagRobot, TOOL_NAME, carton_tag_lines
from test_gemma_tags import CameraRobot, envelope


def carton_image(ids=(10, 12, 22)):
    image = np.full((480, 640, 3), 255, np.uint8)
    for tag_id, x in zip(ids, (20, 220, 420)):
        image[180:280, x:x+100] = marker_image(tag_id, 10)
    return image


def fixture():
    raw = CameraRobot()
    raw.functions[0]['function']['parameters']['properties']['revive'] = {'type': 'boolean'}
    payload = envelope(carton_image(), phone=False)
    def call(name, args, request_id=None):
        assert name == 'robot_get_cameras'  # Any non-camera dispatch fails the test.
        raw.calls.append((name, args, request_id))
        return copy.deepcopy(payload)
    raw.call = call
    return raw, payload, CartonTagRobot(raw, clock=lambda: 1000.1)


def test_carton_roles_and_annotations_are_separate_from_paddle_kit():
    raw, payload, robot = fixture()
    result = robot.call(TOOL_NAME, {'cameras': ['oak']}, request_id='read-only')
    assert raw.calls == [('robot_get_cameras', {'cameras': ['oak'], 'revive': False}, 'read-only')]
    row = result['result']['observations']['oak']
    assert [(t['tag_id'], t['role']) for t in row['tags']] == [
        (10, 'near_wall_center'), (12, 'right_short_flap'), (22, 'right_wall')]
    assert row['pose_3d'] is None and row['gripper_to_paddle_px'] is None
    assert set(row['missing_ids']) == set(CARTON_ROLES) - {10, 12, 22}
    assert result['images'][0]['source_sha256'] == payload['images'][0]['sha256']
    lines = '\n'.join(carton_tag_lines(result))
    assert 'not pinch points or robot coordinates' in lines
    assert 'seq=1' in lines and 'right_short_flap' in lines
    assert 'head' not in ' '.join(call[0] for call in raw.calls)


@pytest.mark.parametrize('change', ['stale', 'hash', 'identity'])
def test_invalid_carton_frames_never_supply_points(change):
    raw, payload, robot = fixture()
    if change == 'stale': payload['result']['cameras']['oak']['rgb_captured_at'] = 900
    elif change == 'hash': payload['images'][0]['sha256'] = 'wrong'
    else: payload['images'][0]['camera_id'] = 'unknown-camera'
    result = robot.call(TOOL_NAME, {'cameras': ['oak']})
    assert not result['ok']
    assert all(row['tags'] == [] for row in result['result']['observations'].values())
    assert result['images'] == [] and result['motor_writes'] == 0


def test_no_tags_and_unrelated_paddle_tags_never_establish_carton_or_fold():
    raw, payload, robot = fixture()
    payload.clear()
    payload.update(envelope(np.full((480, 640, 3), 255, np.uint8), phone=False))
    result = robot.call(TOOL_NAME, {'cameras': ['oak']})
    assert result['result']['observations']['oak']['status'] == 'NO_VALID_TAGS'
    assert not result['result']['physical_task_completed']
    payload.clear()
    payload.update(envelope(carton_image((1, 2, 3)), phone=False, seq=2))
    row = robot.call(TOOL_NAME, {'cameras': ['oak']})['result']['observations']['oak']
    assert all(t['role'] == 'unassigned' for t in row['tags'])
    assert row['gripper_to_paddle_px'] is None
    assert set(row['missing_ids']) == set(CARTON_ROLES)


def test_invalid_model_arguments_do_not_reach_cameras():
    raw, _, robot = fixture()
    for args in ({'execute': True}, {'revive': True}, {'cameras': ['bad']}, {'tag_ids': [True]}):
        with pytest.raises(ValueError): robot.call(TOOL_NAME, args)
    assert raw.calls == []
