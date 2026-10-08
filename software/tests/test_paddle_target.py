"""robot_get_paddle_target on rendered tag observations; read-only, no motion.

The camera image is rendered by tools/simulate_gemma_tags.MarkerScene and goes
through the production detector, IPPE pose, registered-tag checks and the
paddle-target code. Ground truth from the scene is used only to score results.
"""
import copy
import json

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from farm.kinematics.lerobot import pose_error  # noqa: E402
from farm.perception.gemma_calibration import CalibrationRobot  # noqa: E402
from farm.perception.gemma_tags import TagObserver, TagRobot  # noqa: E402
from farm.perception.paddle_target import (  # noqa: E402
    PROPOSAL_LABEL, UNMEASURED, grasp_geometry, paddle_target)
from farm.perception.tag_geometry import TagGeometry, fingerprint  # noqa: E402
from farm.perception.tag_sampling import ARM_JOINTS  # noqa: E402
from tools.simulate_gemma_tags import MarkerScene  # noqa: E402

READ_ONLY = {'robot_get_cameras', 'robot_get_state', 'robot_get_arm_pose', 'robot_plan_reach'}
GEOMETRY = {'schema': 1, 'family': 'tag36h11', 'camera_ids': ['sim'], 'tags': {
    '1': {'black_square_mm': 60, 'source': 'rendered scene'},
    '2': {'black_square_mm': 40, 'source': 'rendered scene',
          'mount': {'arm': 'right', 'body': 'fixed_gripper_housing', 'source': 'rigid scene body'}},
    '3': {'black_square_mm': 40, 'source': 'rendered scene'}}}
JAW = np.eye(4)
JAW[:3, 3] = [.0, -.01, -.095]


@pytest.fixture(scope='module')
def scene():
    s = MarkerScene()
    try:
        s.render()
    except Exception as exc:  # No offscreen OpenGL context on this machine.
        pytest.skip(f'MuJoCo rendering unavailable: {exc}')
    yield s
    s.close()


def _body(scene, name):
    out = np.eye(4)
    body = scene.data.body(name)
    out[:3, :3], out[:3, 3] = body.xmat.reshape(3, 3), body.xpos
    return out


def handle_truth(scene):
    """Handle offset in the printed tag frame (the marker body), plus base-frame truth."""
    printed = (np.linalg.inv(_body(scene, 'tag3')) @ scene.world_from('handle'))[:3, 3]
    base_from_printed = np.linalg.inv(scene.world_from('arm_base')) @ _body(scene, 'tag3')
    return printed*1000, scene.base_from('handle')[:3, 3]*1000, base_from_printed[:3, :3]


def measured(scene, *, jaw=True):
    printed, _, _ = handle_truth(scene)
    section = copy.deepcopy(UNMEASURED)
    section['tag_to_handle'] = {'status': 'measured', 'handle_center_mm': printed.round(1).tolist(),
                                'approach_direction': [0, 0, -1], 'jaw_closing_axis': [0, 1, 0],
                                'tolerance_mm': 2, 'source': 'scene CAD, test'}
    if jaw:
        section['jaw_contact'] = {'status': 'measured', 'gripper_from_jaw_contact': JAW.tolist(),
                                  'tolerance_mm': 2, 'source': 'test jaw'}
    return section


class SceneOwner:
    """Robot API stand-in: rendered camera, stationary encoders, scripted read-only planner."""

    def __init__(self, scene):
        self.scene, self.now, self.calls = scene, 1000., []
        self.frame_delay = self.plan_delay = 0.
        self.names = [f'{a}_arm_{n}' for a in ('left', 'right') for n in ARM_JOINTS]
        self.names += ['head_motor_1', 'head_motor_2', 'base_left_wheel', 'base_right_wheel']
        self.ranges = {n: dict(min_ticks=1000, max_ticks=3000, margin_ticks=4) for n in self.names}
        self.planner = self.plan

    def clock(self):
        return self.now

    def catalog(self):
        return {'tools': [{'type': 'function', 'function': {'name': 'robot_get_cameras', 'parameters': {
            'type': 'object', 'properties': {'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': ['sim']}}}}}}]}

    def plan(self, args, gripper_from_tool=JAW):
        ticks = {f'right_arm_{n}': 2000 + 40*i for i, n in enumerate(ARM_JOINTS[:5])}
        return {'ok': True, 'result': {
            'status': 'KINEMATIC_PROPOSAL_ONLY', 'units': 'encoder_ticks', 'motor_writes': 0, 'collision_checked': False,
            'starting_ticks': {n: 2000 for n in ticks}, 'kinematics_fingerprint': 'fake',
            'configuration': {'config': {'arm': 'right', 'calibration_sha256': 'motors',
                                         'gripper_from_tool': np.asarray(gripper_from_tool).tolist()}},
            'waypoints': [{'joint_targets_ticks': ticks, 'tool_pose': args['tool_poses'][0],
                           'position_error_m': .0004, 'orientation_error_deg': .3}]}}

    def call(self, name, args, request_id=None):
        self.now += .02
        self.calls.append((name, copy.deepcopy(args)))
        if name not in READ_ONLY:
            raise AssertionError(f'paddle target must not call {name}')
        if name == 'robot_get_cameras':
            return self.scene.payload(self.now - self.frame_delay)
        if name == 'robot_get_state':
            return {'ok': True, 'result': dict(cached=False, time=self.now, raw_calibration_ranges=copy.deepcopy(self.ranges),
                    commandable_ranges=copy.deepcopy(self.ranges), motors=[dict(
                        name=n, Present_Position=2000, Present_Velocity=0, Present_Load=0, Moving=0, Status=0,
                        Torque_Enable=0, captured_at=self.now-.001) for n in self.names])}
        if name == 'robot_get_arm_pose':
            return {'ok': True, 'result': {'configuration': {'config': {
                'arm': 'right', 'mapping': 'feetech_degrees_v1', 'calibration_sha256': 'motors'}}}}
        self.now += self.plan_delay
        return self.planner(args)


@pytest.fixture
def rig(scene, tmp_path, monkeypatch):
    owner = SceneOwner(scene)
    tagged = TagRobot(owner, observer=TagObserver(clock=owner.clock, geometry=GEOMETRY))
    tagged.catalog()
    monkeypatch.setattr('farm.perception.registered_tags.assemble_dataset', lambda captures, model: {
        'binding': {'robot_model_sha256': 'model'},
        'samples': [{'base_from_gripper': scene.base_from('gripper').tolist()}]})
    config = tmp_path/'tag-calibration.json'
    config.write_text(json.dumps(dict(schema=1, arm='right', joints=['shoulder_pan', 'wrist_flex'],
                                      camera='sim', model_directory='fake')))
    robot = CalibrationRobot(tagged, config, clock=owner.clock)

    def install(section=None, registration=True):
        geometry = dict(GEOMETRY) if section is None else {**GEOMETRY, 'paddle_grasp': section}
        robot.geometry_path.write_text(json.dumps(geometry))
        if registration:
            reg = fitted_registration(scene, owner, tagged)
            robot.registration_path.write_text(json.dumps(reg))
            owner.calls.clear()
    return owner, robot, install


def fitted_registration(scene, owner, tagged):
    """What a passing fit of this scene would install (true transforms, passing residuals)."""
    row = tagged.call('robot_get_tags', {'cameras': ['sim']})['result']['observations']['sim']
    tags = {t['tag_id']: t for t in row['pose_3d']['tags']}
    anchor = next(t for t in row['tags'] if t['tag_id'] == 1)
    selected = {n: owner.ranges[n] for n in owner.ranges if n.startswith('right_arm_') and not n.endswith('gripper')}
    return dict(status='REGISTRATION_VALIDATED', binding=dict(
        arm='right', camera_id='sim', stream_id=scene.stream_id,
        camera_calibration_sha256=row['pose_3d']['calibration_sha256'],
        tag_geometry_sha256=row['pose_3d']['geometry_config_sha256'], gripper_tag_id=2,
        gripper_tag_mount=tags[2]['mount'], robot_model_sha256='model', motor_calibration_sha256='motors',
        raw_arm_ranges_sha256=fingerprint(selected)),
        residuals={s: dict(count=n, position_rms_mm=1., position_max_mm=2., orientation_max_degrees=.8)
                   for s, n in [('train', 8), ('validation', 3)]},
        base_from_camera=scene.base_from('camera_optical').tolist(),
        gripper_from_tag=(np.linalg.inv(scene.world_from('gripper')) @ scene.world_from('tag2')).tolist(),
        head_ticks_reference=[2000, 2000], anchor_center_camera_mm_reference=tags[1]['center_camera_mm'],
        anchor_corners_px_reference=anchor['corners_px'])


def only_reads(owner):
    assert {name for name, _ in owner.calls} <= READ_ONLY


def test_catalog_describes_read_only_tool_and_procedure(rig):
    _, robot, install = rig
    install(registration=False)
    tool = next(t['function'] for t in robot.catalog()['tools'] if t['function']['name'] == 'robot_get_paddle_target')
    assert PROPOSAL_LABEL in tool['description'] and 'never send the proposal as one move' in tool['description']
    assert 'registration unavailable' in tool['description']
    assert tool['parameters']['additionalProperties'] is False
    assert not robot.call('robot_get_paddle_target', {'arm': 'left'})['ok']


def test_unregistered_refuses_with_the_registration_reason(rig):
    owner, robot, install = rig
    install(measured(owner.scene), registration=False)
    answer = robot.call('robot_get_paddle_target', {})
    assert not answer['ok'] and answer['result']['motor_writes'] == 0
    assert answer['result']['error'].startswith('Registration unavailable') and 'tag-registration.json' in answer['result']['error']
    assert owner.calls == []
    rejected = fitted_registration(owner.scene, owner, robot.robot)
    rejected['status'] = 'REGISTRATION_REJECTED'
    robot.registration_path.write_text(json.dumps(rejected))
    answer = robot.call('robot_get_paddle_target', {})
    assert not answer['ok'] and 'No independently validated registration is installed' in answer['result']['error']
    only_reads(owner)


@pytest.mark.parametrize('section', [None, UNMEASURED])
def test_unmeasured_offsets_return_the_tag_pose_only(rig, section):
    owner, robot, install = rig
    install(section)
    answer = robot.call('robot_get_paddle_target', {})
    assert answer['ok'] and answer['motor_writes'] == 0
    r = answer['result']
    assert r['status'] == 'PADDLE_TAG_POSE_ONLY' and r['grasp'] is None
    assert 'unmeasured' in r['grasp_unavailable_reason']
    assert r['unmeasured_fields'] and r['frame_id'] == 'right_arm_base'
    truth = owner.scene.base_from('tag3')
    assert np.linalg.norm(np.array(r['paddle_tag']['center_arm_base_mm']) - truth[:3, 3]*1000) < 4
    assert pose_error(np.array(r['paddle_tag']['arm_base_from_tag']), truth)[1] < 2
    assert r['uncertainty']['tag_noise_available'] and r['uncertainty']['tag3_depth_std_mm'] > 0
    assert r['reach_proposal']['available'] is False
    assert 'robot_plan_reach' not in {n for n, _ in owner.calls}
    only_reads(owner)


def test_measured_offsets_give_grasp_point_near_ground_truth_and_labelled_proposal(rig):
    owner, robot, install = rig
    install(measured(owner.scene))
    answer = robot.call('robot_get_paddle_target', {})
    assert answer['ok'], answer
    r = answer['result']
    _, handle, base_from_printed = handle_truth(owner.scene)
    grasp = r['grasp']
    error = np.linalg.norm(np.array(grasp['handle_point_arm_base_mm']) - handle)
    assert r['status'] == 'PADDLE_TARGET_GUIDANCE_ONLY' and error < 4
    assert error <= r['uncertainty']['conservative_bound_mm']
    approach = np.array(grasp['approach_direction_arm_base'])
    assert np.degrees(np.arccos(np.clip(approach @ (base_from_printed @ [0, 0, -1]), -1, 1))) < 2
    assert grasp['pregrasp_height_above_handle_mm'] == pytest.approx(60, abs=2)
    tool, pregrasp = np.array(grasp['grasp_tool_pose_arm_base']), np.array(grasp['pregrasp_tool_pose_arm_base'])
    assert tool[:3, 2] == pytest.approx(approach) and tool[:3, 1] == pytest.approx(grasp['jaw_closing_axis_arm_base'])
    assert pregrasp[:3, 3] == pytest.approx(tool[:3, 3] - approach*.06)
    proposal = r['reach_proposal']
    assert proposal['available'] and proposal['label'] == 'proposal, not executed, not collision checked'
    assert proposal['executed'] is False and proposal['collision_checked'] is False
    assert proposal['largest_joint_change_ticks'] == 160
    request = next(a for n, a in owner.calls if n == 'robot_plan_reach')
    assert request['frame'] == 'arm_base' and request['tool_poses'] == [pregrasp.tolist()]
    fresh = r['freshness']
    assert fresh['camera_id'] == 'sim' and fresh['stream_id'] == owner.scene.stream_id
    assert len(fresh['frame_sha256']) == 64 and 0 <= fresh['age_s_at_return'] <= fresh['max_age_s']
    assert r['registration_sha256'] and r['grasp_geometry_sha256'] and r['motor_writes'] == 0
    only_reads(owner)


def test_handle_without_jaw_offset_gives_point_but_no_tool_pose_or_planner_call(rig):
    owner, robot, install = rig
    install(measured(owner.scene, jaw=False))
    r = robot.call('robot_get_paddle_target', {})['result']
    assert r['grasp']['handle_point_arm_base_mm'] and r['grasp']['pregrasp_tool_pose_arm_base'] is None
    assert 'Jaw contact offset unmeasured' in r['tool_pose_unavailable_reason']
    assert r['unmeasured_fields'] == ['paddle_grasp.jaw_contact']
    assert 'robot_plan_reach' not in {n for n, _ in owner.calls}


def test_stale_frames_are_refused(rig):
    owner, robot, install = rig
    install(measured(owner.scene))
    owner.frame_delay = 10
    answer = robot.call('robot_get_paddle_target', {})
    assert not answer['ok'] and 'Stale' in answer['result']['error']
    owner.frame_delay, owner.plan_delay = 0, 7
    answer = robot.call('robot_get_paddle_target', {})
    assert not answer['ok'] and 'Stale frame' in answer['result']['error']
    only_reads(owner)


def test_planner_proposal_withheld_when_unconfigured_or_for_a_different_jaw(rig):
    owner, robot, install = rig
    install(measured(owner.scene))
    owner.planner = lambda args: {'ok': True, 'result': {'status': 'NEEDS_GEOMETRIC_CONFIGURATION',
                                  'configuration': {'missing': ['gripper_from_tool', 'workspace_bounds_m']}}}
    r = robot.call('robot_get_paddle_target', {})['result']
    assert r['grasp']['handle_point_arm_base_mm'] and not r['reach_proposal']['available']
    assert 'gripper_from_tool' in r['reach_proposal']['reason']
    other = JAW.copy()
    other[2, 3] += .005
    owner.planner = lambda args: owner.plan(args, other)
    r = robot.call('robot_get_paddle_target', {})['result']
    assert not r['reach_proposal']['available'] and 'differs' in r['reach_proposal']['reason']
    assert 'joint_targets_ticks' not in r['reach_proposal']
    owner.calls.clear()
    r = robot.call('robot_get_paddle_target', {'include_proposal': False})['result']
    assert r['reach_proposal'] is None and 'robot_plan_reach' not in {n for n, _ in owner.calls}


def test_ambiguous_tag3_keeps_centre_but_no_grasp():
    registered = {'arm': 'right', 'frame': {'captured_at': 99.5, 'timestamp_basis': 'capture'},
                  'current_gripper_consistency': {'position_mm': 1, 'orientation_degrees': .5},
                  'tags': [{'tag_id': 3, 'center_arm_base_mm': [250, -40, 10], 'arm_base_from_tag': None,
                            'orientation_ambiguous': True}]}
    registration = {'residuals': {s: dict(position_rms_mm=1, position_max_mm=2, orientation_max_degrees=1)
                                  for s in ('train', 'validation')}}
    section = copy.deepcopy(UNMEASURED)
    section['tag_to_handle'] = {'status': 'measured', 'handle_center_mm': [80, 0, -6], 'approach_direction': [0, 0, -1],
                                'jaw_closing_axis': [0, 1, 0], 'tolerance_mm': 2, 'source': 'ruler'}
    r = paddle_target(registered, registration, grasp_geometry({'paddle_grasp': section}, 'right'), clock=lambda: 100.)
    assert r['grasp'] is None and 'ambiguous' in r['grasp_unavailable_reason']
    assert r['paddle_tag']['center_arm_base_mm'] == [250, -40, 10]
    with pytest.raises(ValueError, match='tag 3'):
        paddle_target({**registered, 'tags': []}, registration, grasp_geometry({}, 'right'), clock=lambda: 100.)


@pytest.mark.parametrize('mutate', [
    lambda s: s['tag_to_handle'].update(tolerance_mm=None),
    lambda s: s['tag_to_handle'].update(source=''),
    lambda s: s['tag_to_handle'].update(jaw_closing_axis=[0, 0, 1]),
    lambda s: s['tag_to_handle'].update(approach_direction=[0, 0, -2]),
    lambda s: s['tag_to_handle'].update(status='guessed'),
    lambda s: s['jaw_contact'].update(gripper_from_jaw_contact=[[1, 0, 0, 0]]),
    lambda s: s.update(arm='left'),
    lambda s: s.update(pregrasp_standoff_mm=500),
])
def test_malformed_measurements_refuse_before_any_robot_call(rig, mutate):
    owner, robot, install = rig
    section = measured(owner.scene)
    mutate(section)
    install(section)
    answer = robot.call('robot_get_paddle_target', {})
    assert not answer['ok'] and owner.calls == []


def test_recording_offsets_does_not_change_the_registered_tag_geometry_hash():
    base = TagGeometry(GEOMETRY).fingerprint
    assert base == fingerprint(GEOMETRY)  # Existing registrations stay bound.
    section = copy.deepcopy(UNMEASURED)
    assert TagGeometry({**GEOMETRY, 'paddle_grasp': section}).fingerprint == base
    resized = copy.deepcopy(GEOMETRY)
    resized['tags']['3']['black_square_mm'] = 41
    assert TagGeometry(resized).fingerprint != base
