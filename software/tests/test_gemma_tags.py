"""Real detector over camera-tool envelopes, plus Gemma catalog/dispatch tests."""
import base64
import copy
import hashlib
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton.servo.tag_kit import marker_image
from farm.perception.gemma_tags import TagObserver, TagRobot, TOOL_NAME


def scene(shift=0, duplicate=False, blank=False):
    image = np.full((480, 640, 3), 255, np.uint8)
    if not blank:
        for tag_id, x, y in ((1, 25, 30), (2, 230 + shift, 150), (2 if duplicate else 3, 480, 320)):
            marker = marker_image(tag_id, 10)
            image[y:y+100, x:x+100] = marker
    return image


def envelope(image=None, *, phone=True, captured=1000.0, seq=1):
    image = scene() if image is None else image
    ok, png = cv2.imencode('.png', image)
    assert ok
    digest = hashlib.sha256(png).hexdigest()
    frame = {'camera_id': 'oak-test', 'captured_at': captured, 'received_at': None,
             'seq': seq, 'stream_id': 'stream1', 'mime_type': 'image/png', 'sha256': digest,
             'data_base64': base64.b64encode(png).decode()}
    meta = {'camera_id': 'oak-test', 'sha256': digest, 'seq': seq, 'stream_id': 'stream1',
            'width': 640, 'height': 480, 'rgb_captured_at': captured}
    images, cameras = [frame], {'oak': meta}
    if phone:
        images.append({**frame, 'camera_id': 'phone_overview', 'captured_at': None,
                       'received_at': captured, 'stream_id': None})
        cameras['phone'] = {'seq': seq, 'received_at': captured, 'live': True, 'width': 640, 'height': 480}
    return {'ok': True, 'result': {'cameras': cameras}, 'images': images}


def observer():
    return TagObserver(clock=lambda: 1000.1)


def test_real_tags_and_per_image_displacement_are_returned_with_annotations():
    result = observer().observe(envelope(), ['oak', 'phone'])
    assert result['ok'] and result['motor_writes'] == 0
    assert result['result']['metric_pose_available'] is False
    assert len(result['images']) == 2
    for name, row in result['result']['observations'].items():
        assert row['missing_ids'] == []
        assert [r['tag_id'] for r in row['tags']] == [1, 2, 3]
        assert row['gripper_to_paddle_px']['dx'] == pytest.approx(250, abs=.2)
        assert row['gripper_to_paddle_px']['dy'] == pytest.approx(170, abs=.2)
        assert row['pose_3d'] is None
        assert row['frame']['capture_age_within_limit'] is (name == 'oak')
        assert not row['frame']['clock_synchronization_verified']
    image = result['images'][0]
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(image['data_base64']), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (480, 640, 3)
    assert image['source_sha256'] == envelope()['images'][0]['sha256']


def test_missing_tags_is_an_observation_not_a_tool_failure():
    result = observer().observe(envelope(scene(blank=True)), ['oak'])
    row = result['result']['observations']['oak']
    assert result['ok'] and row['status'] == 'NO_VALID_TAGS'
    assert row['missing_ids'] == [1, 2, 3] and row['gripper_to_paddle_px'] is None


def test_does_not_reuse_old_points_or_impose_carton_seed_envelope():
    tool = observer()
    first = tool.observe(envelope(phone=False), ['oak'])
    second = tool.observe(envelope(scene(shift=150), phone=False, seq=2), ['oak'])
    missing = tool.observe(envelope(scene(blank=True), phone=False, seq=3), ['oak'])
    assert second['ok']
    assert second['result']['observations']['oak']['gripper_to_paddle_px']['dx'] < first['result']['observations']['oak']['gripper_to_paddle_px']['dx'] - 140
    assert missing['result']['observations']['oak']['tags'] == []


@pytest.mark.parametrize('mutation,reason', [
    (lambda p: p['images'][0].update(sha256='bad'), 'hash'),
    (lambda p: p['result']['cameras']['oak'].update(width=900), 'width'),
    (lambda p: p['result']['cameras']['oak'].update(seq=5), 'seq'),
    (lambda p: p['images'][0].update(data_base64='not base64!'), 'base64'),
    (lambda p: p['images'][0].update(mime_type='application/octet-stream'), 'RGB'),
    (lambda p: p['images'][0].update(fresh=False), 'stale'),
    (lambda p: p['result']['cameras']['oak'].update(rgb_captured_at=990), 'Stale'),
    (lambda p: p['result']['cameras']['oak'].update(rgb_captured_at=1005), 'future'),
])
def test_invalid_frames_fail_without_detections(mutation, reason):
    payload = envelope(phone=False)
    mutation(payload)
    result = observer().observe(payload, ['oak'])
    assert not result['ok'] and result['images'] == []
    row = result['result']['observations']['oak']
    assert row['tags'] == [] and reason.lower() in row['reason'].lower()


def test_duplicate_tag_identity_is_unknown():
    result = observer().observe(envelope(scene(duplicate=True), phone=False), ['oak'])
    assert not result['ok']
    assert 'duplicate tag ID' in result['result']['observations']['oak']['reason']


def test_duplicate_camera_invalidates_both_pixels_and_measurements():
    payload = envelope(phone=False)
    payload['images'].append(copy.deepcopy(payload['images'][0]))
    result = observer().observe(payload, ['oak'])
    assert not result['ok'] and result['images'] == []


def test_missing_camera_and_receipt_only_view_are_explicit():
    result = observer().observe(envelope(), ['phone', 'head'])
    rows = result['result']['observations']
    assert rows['head']['status'] == 'UNKNOWN'
    assert rows['phone']['frame']['captured_at'] is None
    assert rows['phone']['frame']['timestamp_basis'] == 'receipt_only_capture_delay_unknown'


def test_frame_regression_and_same_sequence_mutation_are_rejected():
    tool = observer()
    tool.observe(envelope(phone=False, seq=3), ['oak'])
    assert not tool.observe(envelope(phone=False, seq=2), ['oak'])['ok']
    assert not tool.observe(envelope(scene(shift=10), phone=False, seq=3), ['oak'])['ok']
    repeat = tool.observe(envelope(phone=False, seq=3), ['oak'])
    assert repeat['ok'] and not repeat['result']['observations']['oak']['frame']['new_since_last_call']
    restarted = envelope(phone=False, seq=0)
    restarted['images'][0]['stream_id'] = restarted['result']['cameras']['oak']['stream_id'] = 'stream2'
    assert tool.observe(restarted, ['oak'])['ok']


class CameraRobot:
    def __init__(self):
        self.calls = []
        self.functions = [{'type': 'function', 'function': {'name': 'robot_get_cameras', 'parameters': {
            'type': 'object', 'properties': {'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': ['oak', 'phone']}}}}}},
            {'type': 'function', 'function': {'name': 'robot_stop', 'parameters': {'type': 'object'}}}]

    def catalog(self):
        return {'tools': self.functions, 'metadata': {'stop_tool': 'robot_stop'}}

    def call(self, name, args, request_id=None):
        self.calls.append((name, args, request_id))
        return envelope() if name == 'robot_get_cameras' else {'ok': True, 'native': name}

    def get(self, path):
        return {'path': path}


def test_gemma_discovery_and_dispatch_use_existing_camera_only():
    raw = CameraRobot()
    robot = TagRobot(raw, observer=observer())
    catalog = robot.catalog()
    assert len(catalog['tools']) == 3 and len(raw.functions) == 2
    result = robot.call(TOOL_NAME, {'cameras': ['oak'], 'include_images': False}, request_id='trace1')
    assert result['ok'] and result['images'] == []
    assert raw.calls == [('robot_get_cameras', {'cameras': ['oak']}, 'trace1')]
    assert robot.get('/health') == {'path': '/health'}
    assert robot.call('robot_stop', {}) == {'ok': True, 'native': 'robot_stop'}


def test_native_tag_tool_is_not_shadowed_and_inventory_refreshes():
    raw = CameraRobot()
    robot = TagRobot(raw, observer=observer())
    robot.catalog()
    raw.functions.append({'type': 'function', 'function': {'name': TOOL_NAME, 'parameters': {}}})
    assert len(robot.catalog()['tools']) == 3
    assert robot.call(TOOL_NAME, {})['native'] == TOOL_NAME
    raw.functions = []
    assert robot.catalog()['tools'] == []


@pytest.mark.parametrize('args', [{'cameras': ['unknown']}, {'cameras': ['oak', 'oak']}, {'tag_ids': [True]},
                                  {'tag_ids': [2, 2]}, {'tag_ids': [587]}, {'include_images': 1}, {'execute': True}])
def test_bad_model_arguments_never_dispatch(args):
    raw = CameraRobot()
    robot = TagRobot(raw, observer=observer())
    with pytest.raises(ValueError):
        robot.call(TOOL_NAME, args)
    assert raw.calls == []


def test_camera_failure_does_not_trigger_fallback_or_motor_call():
    raw = CameraRobot()
    raw.call = lambda *a, **k: {'ok': False, 'error': 'offline'}
    result = TagRobot(raw, observer=observer()).call(TOOL_NAME, {})
    assert not result['ok'] and result['motor_writes'] == 0


def test_detector_latency_is_counted_before_returning_points():
    from farm.perception.tags import detect_tags
    now = [1000.1]
    def slow(frame):
        result = detect_tags(frame)
        now[0] += 3
        return result
    result = TagObserver(clock=lambda: now[0], detector=slow).observe(envelope(phone=False), ['oak'])
    assert not result['ok'] and result['images'] == []


@pytest.mark.parametrize('change', [{'margin': 29}, {'hamming': 1}, {'corners': [[1, 1], [1, 10], [10, 10], [10, 1]]}])
def test_weak_small_or_corrected_tags_never_produce_displacements(change):
    from farm.perception.tags import detect_tags
    def weak(frame):
        result = detect_tags(frame)
        result.value[2].update(change)
        return result
    result = TagObserver(clock=lambda: 1000.1, detector=weak).observe(envelope(phone=False), ['oak'])
    row = result['result']['observations']['oak']
    assert row['missing_ids'] == [2] and row['gripper_to_paddle_px'] is None
    assert next(t for t in row['tags'] if t['tag_id'] == 2)['status'] == 'REJECTED'


def test_installer_preserves_existing_code_and_refuses_drift():
    path = Path(__file__).resolve().parents[1] / 'tools/install_gemma_tags.py'
    spec = importlib.util.spec_from_file_location('installer', path)
    installer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(installer)
    source = installer.ANCHOR + '\ndef main():\n    ' + installer.BEFORE + ';other_code()\n'
    patched = installer.patch_source(source)
    assert 'other_code()' in patched and installer.AFTER in patched
    assert installer.patch_source(patched) == patched
    with pytest.raises(ValueError):
        installer.patch_source(source.replace('args.config', 'changed.config'))
