"""Carton pose from tag poses + registration: synthetic geometry, refusals, fingerprint neutrality, read-only dispatch."""
import copy
import json

import numpy as np
import pytest

from farm.perception import carton_pose as cp
from farm.perception.tag_geometry import TagGeometry

# A camera above and ahead of the right arm, looking down-forward (like the OAK), in the right-arm base frame.
BASE_FROM_CAMERA = np.array([[0.0, -0.5, 0.8660254, 0.05],
                             [-1.0, 0.0, 0.0, 0.15],
                             [0.0, -0.8660254, -0.5, 0.40],
                             [0.0, 0.0, 0.0, 1.0]])
CAMERA_ID = 'oak-test'
CALIB = 'calib-sha'
REGISTRATION = {'status': 'REGISTRATION_VALIDATED', 'head_ticks_reference': [2085, 2623],
                'base_from_camera': BASE_FROM_CAMERA.tolist(),
                'binding': {'arm': 'right', 'camera_id': CAMERA_ID, 'camera_calibration_sha256': CALIB, 'tag_geometry_sha256': 'g'}}
GEOMETRY = {'schema': 1, 'family': 'tag36h11', 'camera_ids': [CAMERA_ID],
            'tags': {'1': {'black_square_mm': 60, 'source': 'x'}, '2': {'black_square_mm': 40, 'source': 'x'}, '3': {'black_square_mm': 40, 'source': 'x'}}}


def synthetic_carton(near_forward_m=0.25, centre_left_m=0.01, rim_up_m=0.81, yaw_deg=0.0, flap_lean_deg=0.0):
    """Tag centres (model frame, metres) of a carton whose near wall faces the robot, with the robot-right flap (tag 11)."""
    yaw = np.radians(yaw_deg)
    into = np.array([np.cos(yaw), np.sin(yaw), 0.0])       # away from the robot
    right = np.array([np.sin(yaw), -np.cos(yaw), 0.0])     # along the near wall toward the robot's right
    up = np.array([0.0, 0.0, 1.0])
    rim_centre = np.array([near_forward_m, centre_left_m, rim_up_m])
    tag_h = rim_centre - cp.HALF_WALL_MM / 1000 * up
    tags = {10: tag_h, 27: tag_h + 0.12 * right, 26: tag_h - 0.12 * right}
    hinge_mid = rim_centre + 0.1895 * right + 0.1415 * into
    lean = np.radians(flap_lean_deg)
    tags[11] = hinge_mid + 0.09 * (np.cos(lean) * up + np.sin(lean) * right) + 0.07 * into
    return tags, {'rim_centre': rim_centre, 'hinge_mid': hinge_mid, 'into': into, 'right': right}


def observation(tags_model_m, ambiguous=()):
    camera_from_base = np.linalg.inv(BASE_FROM_CAMERA)
    rows = []
    for tag_id, p in tags_model_m.items():
        base_mm = cp.base_mm_from_model(p)
        cam = camera_from_base @ np.r_[base_mm / 1000, 1.0]
        pose = np.eye(4)
        pose[:3, 3] = cam[:3]
        rows.append({'tag_id': tag_id, 'black_square_mm': cp.PRINT_SIZES_MM[tag_id], 'status': 'POSE_ESTIMATED',
                     'center_camera_mm': (cam[:3] * 1000).tolist(), 'camera_from_tag': None if tag_id in ambiguous else pose.tolist(),
                     'orientation_ambiguous': tag_id in ambiguous, 'reprojection_rms_px': 0.2,
                     'position_std_mm_at_assumed_half_pixel_noise': [0.3, 0.3, 1.0], 'coarse_position_only': False})
    return {'status': 'OBSERVED', 'frame': {'camera_id': CAMERA_ID, 'seq': 7, 'stream_id': 's', 'sha256': 'h',
                                            'captured_at': 1000.0, 'timestamp_basis': 'capture', 'age_s_on_observer_clock': 0.4},
            'pose_3d': {'status': 'CAMERA_RELATIVE_ESTIMATE', 'calibration_sha256': CALIB, 'geometry_config_sha256': 'g', 'tags': rows}}


def cm(point):
    m = point['model_cm']
    return np.array([m['forward_cm'], m['left_cm'], m['up_cm']])


def test_base_model_round_trip_and_origin():
    p = np.array([385.0, 21.0, 33.0])
    assert np.allclose(cp.base_mm_from_model(cp.model_from_base_mm(p)), p)
    assert np.allclose(cp.model_from_base_mm([38.8353, 0, 116.6]), [0, -0.1365, 0.894], atol=1e-6)  # pan axis / lift height


def test_carton_arm_local_to_model_matches_twin_shoulder():
    from farm.sim import xlerobot_twin as twin
    from farm.kinematics import so101_reach as reach
    pytest.importorskip('mujoco')
    ranges = {m: (1000, 3000) for m in twin.JOINT_TABLE}
    claws = twin.claw_positions({m: 2000 for m in ranges}, ranges)
    assert claws['left_arm']['shoulder_left_m'] - claws['right_arm']['shoulder_left_m'] == pytest.approx(0.273, abs=1e-6)
    point = cp.model_from_base_mm([38.8353, 0, 116.6])
    assert point == pytest.approx([0, claws['right_arm']['shoulder_left_m'], claws['right_arm']['shoulder_up_m']], abs=1e-6)
    assert reach.SHOULDER_LEFT_M['right'] == pytest.approx(point[1])


@pytest.mark.parametrize('yaw', [0.0, 4.0, -6.0])
def test_synthetic_carton_is_recovered(yaw):
    tags, truth = synthetic_carton(near_forward_m=0.22, centre_left_m=-0.02, rim_up_m=0.808, yaw_deg=yaw, flap_lean_deg=12.0)
    out = cp.carton_pose_from_observation(observation(tags), REGISTRATION, head_ticks=[2086, 2622], rim_reference_m=0.81)
    assert out['status'] == 'CARTON_POSE_ESTIMATED' and out['warnings'] == []
    c = out['carton']
    assert abs(c['rim_up_cm'] - 80.8) < 0.11 and abs(c['rim_check']['error_cm'] + 0.2) < 0.11
    assert np.allclose(cm(c['near_face']['centre_at_rim']), truth['rim_centre'] * 100, atol=0.11)
    assert abs(c['near_face']['yaw_deg_from_square'] - yaw) < 0.2
    assert np.allclose(cm(c['right_short_flap_hinge']['midpoint']), truth['hinge_mid'] * 100, atol=0.11)
    start, end = cm(c['right_short_flap_hinge']['start']), cm(c['right_short_flap_hinge']['end'])
    assert abs(np.linalg.norm(end - start) - 28.3) < 0.15  # endpoints are rounded to 0.1 cm
    assert np.allclose(end - start, truth['into'] * 28.3, atol=0.1)
    assert c['right_wall_top']['up_cm'] == c['rim_up_cm']
    assert [s['error_mm'] for s in c['scale_check']] == [0, 0, 0]
    assert c['right_short_flap']['tag_id'] == 11 and abs(c['right_short_flap']['lean_outward_deg'] - 12) < 0.3
    assert abs(c['right_short_flap']['tag_along_hinge_from_centre_mm'] - 70) < 1
    tip = cm(c['right_short_flap']['top_edge_midpoint_if_flat'])
    expected_tip = truth['hinge_mid'] * 100 + 14 * (np.cos(np.radians(12)) * np.array([0, 0, 1]) + np.sin(np.radians(12)) * truth['right'])
    assert np.allclose(tip, expected_tip, atol=0.11)
    roles = {t['tag_id']: t['robot_side_role'] for t in out['tags']}
    assert roles[11] == 'right_short_flap' and roles[27] == 'near_wall_left'
    assert out['motor_writes'] == 0 and out['robot_motion_target'] is False


def test_two_wall_tags_suffice_and_one_does_not():
    tags, truth = synthetic_carton()
    two = {k: v for k, v in tags.items() if k in (10, 26)}
    out = cp.carton_pose_from_observation(observation(two), REGISTRATION, head_ticks=[2085, 2623])
    assert np.allclose(cm(out['carton']['near_face']['centre_at_rim']), truth['rim_centre'] * 100, atol=0.11)
    assert [s['pair'] for s in out['carton']['scale_check']] == [[10, 26]]
    one = cp.carton_pose_from_observation(observation({10: tags[10], 11: tags[11]}), REGISTRATION, head_ticks=[2085, 2623])
    assert one['status'] == 'TAG_CENTRES_ONLY' and one['carton'] is None and one['tags'][0]['tag_id'] == 10


def test_unmirrored_placement_uses_tag_12():
    tags, _ = synthetic_carton()
    tags[12] = tags.pop(11)
    out = cp.carton_pose_from_observation(observation(tags), REGISTRATION, head_ticks=[2085, 2623], mirrored=False)
    assert out['carton']['right_short_flap']['tag_id'] == 12
    assert {t['tag_id']: t['robot_side_role'] for t in out['tags']}[12] == 'right_short_flap'


@pytest.mark.parametrize('change, message', [
    ('head', 'Head moved'), ('camera', 'registration is for'), ('status', 'No validated'),
    ('calibration', 'calibration changed'), ('metric', 'No metric tag poses'), ('arm', 'not for the right arm'),
    ('no_head', 'Head ticks were not read')])
def test_refusals_name_the_cause(change, message):
    tags, _ = synthetic_carton()
    row, registration, head = observation(tags), copy.deepcopy(REGISTRATION), [2085, 2623]
    if change == 'head': head = [2085, 2630]
    if change == 'no_head': head = None
    if change == 'camera': row['frame']['camera_id'] = 'oak-other'
    if change == 'status': registration['status'] = 'FIT_ONLY'
    if change == 'arm': registration['binding']['arm'] = 'left'
    if change == 'calibration': row['pose_3d']['calibration_sha256'] = 'other'
    if change == 'metric': row['pose_3d'] = {'status': 'UNAVAILABLE', 'reason': 'no sizes'}
    with pytest.raises(ValueError, match=message):
        cp.carton_pose_from_observation(row, registration, head_ticks=head)


def test_carton_sizes_do_not_change_the_registration_fingerprint(tmp_path):
    path = tmp_path / 'apriltag-geometry.json'
    path.write_text(json.dumps(GEOMETRY))
    before = TagGeometry(GEOMETRY).fingerprint
    receipt = cp.install_carton_tag_sizes(path, clock=lambda: 0)
    assert receipt['changed'] and receipt['fingerprint_before'] == before == receipt['fingerprint_after']
    assert json.loads(open(receipt['backup']).read()) == GEOMETRY
    config = json.loads(path.read_text())
    assert config['tags'] == GEOMETRY['tags'] and set(config['carton_tags']) == {str(i) for i in cp.CARTON_IDS}
    geometry = TagGeometry(config)
    assert geometry.fingerprint == before and geometry.sizes['14']['black_square_mm'] == 35 and geometry.sizes['10']['black_square_mm'] == 45
    assert cp.install_carton_tag_sizes(path, clock=lambda: 0)['changed'] is False  # idempotent
    with pytest.raises(ValueError, match='only one size section'):
        TagGeometry(dict(config, carton_tags=dict(config['carton_tags'], **{'2': {'black_square_mm': 40, 'source': 'x'}})))


class FakeRobot:
    config = '/tmp/does-not-exist/robot.json'

    def __init__(self, row, head=(2085, 2623)):
        self.row, self.head, self.calls = row, head, []
        self.samples = 0
        self.functions = [{'type': 'function', 'function': {'name': n, 'parameters': {}}} for n in ('robot_get_tags', 'robot_get_state', 'robot_move_path')]

    def catalog(self):
        return {'tools': copy.deepcopy(self.functions)}

    def call(self, name, args, request_id=None):
        self.calls.append((name, args))
        if name == 'robot_get_state':
            self.samples += 1
            stamp = 999.9 if self.samples == 1 else 1000.1
            return {'ok': True, 'result': {'motors': [{'name': 'head_motor_1', 'Present_Position': self.head[0], 'captured_at': stamp, 'Moving': 0, 'Present_Velocity': 0},
                                                      {'name': 'head_motor_2', 'Present_Position': self.head[1], 'captured_at': stamp, 'Moving': 0, 'Present_Velocity': 0}]}}
        if name == 'robot_get_tags':
            return {'ok': True, 'result': {'observations': {'oak': copy.deepcopy(self.row)}}, 'images': [{'view': 'oak'}]}
        return {'ok': True, 'passed_through': name}


def test_wrapper_reads_only_and_reports_in_both_frames(tmp_path):
    (tmp_path / 'tag-registration.json').write_text(json.dumps(REGISTRATION))
    (tmp_path / 'workspace.json').write_text(json.dumps({'object_top_m': 0.81}))
    tags, truth = synthetic_carton()
    raw = FakeRobot(observation(tags))
    robot = cp.CartonPoseRobot(raw, registration_path=tmp_path / 'tag-registration.json', clock=lambda: 1000.2, sleep=lambda _: None)
    names = [t['function']['name'] for t in robot.catalog()['tools']]
    assert names[-1] == cp.TOOL_NAME and names.count(cp.TOOL_NAME) == 1
    out = robot.call(cp.TOOL_NAME, {})
    assert [c[0] for c in raw.calls] == ['robot_get_state', 'robot_get_tags', 'robot_get_state'] and out['images'] == []
    assert raw.calls[1][1] == {'cameras': ['oak'], 'tag_ids': list(cp.CARTON_IDS), 'include_images': False}
    assert out['ok'] and out['result']['carton']['rim_check']['reference_up_cm'] == 81.0
    hinge = out['result']['carton']['right_short_flap_hinge']['midpoint']
    assert np.allclose(hinge['arm_base_mm'], cp.base_mm_from_model(truth['hinge_mid']), atol=0.1)
    text = '\n'.join(cp.carton_pose_lines(out))
    assert 'right wall top / right short flap hinge' in text and 'rim: 81.0 cm' in text and 'WARNING' not in text
    # Head moved: refuses without any further call, and the text says so.
    raw.calls.clear()
    raw.head = (2085, 2700)
    refused = robot.call(cp.TOOL_NAME, {'include_images': True})
    assert [c[0] for c in raw.calls] == ['robot_get_state']
    assert refused['ok'] is False and 'Head moved' in refused['error'] and refused['motor_writes'] == 0
    assert 'carton pose unavailable: ValueError: Head moved' in '\n'.join(cp.carton_pose_lines(refused))
    with pytest.raises(ValueError):
        robot.call(cp.TOOL_NAME, {'cameras': ['oak']})
    assert robot.call('robot_move_path', {'x': 1}) == {'ok': True, 'passed_through': 'robot_move_path'}  # pass-through
    assert raw.calls[-1] == ('robot_move_path', {'x': 1})


@pytest.mark.parametrize('bad_age', [None, -0.01, 1.01, float('nan'), float('inf')])
def test_frame_age_must_be_known_finite_and_fresh(bad_age):
    row = observation(synthetic_carton()[0])
    row['frame']['age_s_on_observer_clock'] = bad_age
    with pytest.raises(ValueError, match='stale, future-dated or has unknown age'):
        cp.carton_pose_from_observation(row, REGISTRATION, head_ticks=[2085, 2623])


@pytest.mark.parametrize('head', [[float('nan'), 2623], [2085], [2085, float('inf')]])
def test_bad_head_never_yields_geometry(head):
    with pytest.raises(ValueError, match='finite head ticks'):
        cp.carton_pose_from_observation(observation(synthetic_carton()[0]), REGISTRATION, head_ticks=head)


def test_geometry_and_print_size_bindings():
    row = observation(synthetic_carton()[0])
    row['pose_3d']['geometry_config_sha256'] = 'changed'
    with pytest.raises(ValueError, match='geometry changed'):
        cp.carton_pose_from_observation(row, REGISTRATION, head_ticks=[2085, 2623])
    row['pose_3d']['geometry_config_sha256'] = 'g'
    row['pose_3d']['tags'][0]['black_square_mm'] = 60
    with pytest.raises(ValueError, match='size differs'):
        cp.carton_pose_from_observation(row, REGISTRATION, head_ticks=[2085, 2623])
    row['pose_3d']['tags'][0]['black_square_mm'] = 45
    row['pose_3d']['tags'][0]['center_camera_mm'][0] = float('nan')
    with pytest.raises(ValueError, match='finite 3-vector'):
        cp.carton_pose_from_observation(row, REGISTRATION, head_ticks=[2085, 2623])


def test_absent_or_misplaced_flap_never_assumes_vertical():
    tags, _ = synthetic_carton()
    missing = dict(tags)
    del missing[11]
    result = cp.carton_pose_from_observation(observation(missing), REGISTRATION, head_ticks=[2085, 2623])
    assert 'right_short_flap' not in result['carton']
    assert 'lean and top edge unknown' in '\n'.join(cp.carton_pose_lines({'ok': True, 'result': result}))
    tags[11] = tags[11] + [0, 0, 0.05]
    with pytest.raises(ValueError, match='mounting disagrees'):
        cp.carton_pose_from_observation(observation(tags), REGISTRATION, head_ticks=[2085, 2623])


@pytest.mark.parametrize('fault, expected', [
    ('moving', 'moving'), ('old_head', 'telemetry is stale'),
    ('head_shift', 'moved during'), ('future_capture', 'after the head readback'),
    ('old_capture', 'No OAK capture inside'), ('registration', 'Registration changed')])
def test_wrapper_refuses_bad_stationary_capture_brackets(tmp_path, fault, expected):
    path = tmp_path / 'tag-registration.json'
    path.write_text(json.dumps(REGISTRATION))
    class FaultRobot(FakeRobot):
        def call(self, name, args, request_id=None):
            out = super().call(name, args, request_id)
            if name == 'robot_get_state':
                motor = out['result']['motors'][0]
                if fault == 'moving': motor['Moving'] = 1
                if fault == 'old_head': motor['captured_at'] = 998
                if fault == 'head_shift' and self.samples > 1: motor['Present_Position'] += 2
            if name == 'robot_get_tags':
                frame = out['result']['observations']['oak']['frame']
                if fault == 'future_capture': frame['captured_at'] = 1000.15
                if fault == 'old_capture': frame['captured_at'] = 999.8
                if fault == 'registration': path.write_text('{}')
            return out
    raw = FaultRobot(observation(synthetic_carton()[0]))
    robot = cp.CartonPoseRobot(raw, registration_path=path, clock=lambda: 1000.2, sleep=lambda _: None)
    out = robot.call(cp.TOOL_NAME, {})
    assert not out['ok'] and expected in out['error'] and out['motor_writes'] == 0
    assert {name for name, _ in raw.calls} <= {'robot_get_state', 'robot_get_tags'}


def test_old_frame_retry_gets_distinct_requests_and_new_capture(tmp_path):
    path = tmp_path / 'tag-registration.json'
    path.write_text(json.dumps(REGISTRATION))
    class RetryingRobot(FakeRobot):
        ids = []
        def call(self, name, args, request_id=None):
            self.ids.append(request_id)
            out = super().call(name, args, request_id)
            if name == 'robot_get_tags' and self.samples == 1:
                out['result']['observations']['oak']['frame']['captured_at'] = 999.8
            return out
    raw = RetryingRobot(observation(synthetic_carton()[0]))
    robot = cp.CartonPoseRobot(raw, registration_path=path, clock=lambda: 1000.2, sleep=lambda _: None)
    assert robot.call(cp.TOOL_NAME, {}, request_id='parent')['ok']
    assert len(raw.ids) == len(set(raw.ids)) == 5
