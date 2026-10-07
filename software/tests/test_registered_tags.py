"""Read-only registration consumption and stale-binding rejection."""
import copy
import json

import numpy as np
import pytest

from farm.perception.registered_tags import registered_observation, read_registered_tags
from farm.perception.gemma_calibration import CalibrationRobot
from farm.perception.tag_geometry import fingerprint
from test_gemma_calibration import Owner


def fixture(arm='right'):
    owner = Owner(arm)
    before = owner.call('robot_get_state', {'fresh':True})
    row = owner.call('robot_get_tags', {'cameras':['oak']})['result']['observations']['oak']
    after = owner.call('robot_get_state', {'fresh':True})
    camera = np.eye(4)
    camera[:3,3] = [.1,.2,.3]
    paddle = np.eye(4)
    paddle[:3,3] = [.3,.1,.5]
    row['pose_3d']['tags'].append(dict(tag_id=3, center_camera_mm=[300,100,500],
                                     camera_from_tag=paddle.tolist(),orientation_ambiguous=False))
    ranges = before['result']['raw_calibration_ranges']
    selected = {n:ranges[n] for n in ranges if n.startswith(f'{arm}_arm_') and not n.endswith('gripper')}
    reg = dict(status='REGISTRATION_VALIDATED',binding=dict(arm=arm,camera_id='oak-test',stream_id='one',
        camera_calibration_sha256='K',tag_geometry_sha256='geometry',gripper_tag_id=owner.tag_id,
        gripper_tag_mount=owner.mount,robot_model_sha256='model',motor_calibration_sha256='motors',
        raw_arm_ranges_sha256=fingerprint(selected)),
        residuals={s:dict(count=n,position_rms_mm=.1,position_max_mm=.2,orientation_max_degrees=.1)
                   for s,n in [('train',8),('validation',3)]},base_from_camera=camera.tolist(),
        gripper_from_tag=np.eye(4).tolist(),head_ticks_reference=[2000,2000],
        anchor_center_camera_mm_reference=[0,0,600],anchor_corners_px_reference=row['tags'][0]['corners_px'])
    status = dict(ok=True,result=dict(configuration=dict(config=dict(arm=arm,mapping='feetech_degrees_v1',calibration_sha256='motors'))))
    args=[before,after,row,reg,status,'model',camera@owner.pose()]
    return owner,args


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_transforms_decoded_pose_and_marker_center_without_motor_targets(arm):
    owner,args=fixture(arm)
    result=registered_observation(*args)
    paddle=next(t for t in result['tags'] if t['tag_id']==3)
    assert paddle['center_arm_base_mm']==pytest.approx([400,300,800])
    assert np.array(paddle['arm_base_from_tag'])[:3,3]==pytest.approx([.4,.3,.8])
    assert result['motor_writes']==0 and not result['robot_motion_target']
    assert not owner.writes
    assert result['arm'] == arm


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_registered_read_requests_the_bound_gripper_and_keeps_no_motion_semantics(arm, monkeypatch):
    owner, args = fixture(arm)
    reg, base_from_gripper = args[3], args[6]
    def assemble(captures, model):
        assert captures[0]['sample']['gripper_tag_id'] == owner.tag_id
        return {'binding': {'robot_model_sha256': 'model'},
                'samples': [{'base_from_gripper': base_from_gripper.tolist()}]}
    monkeypatch.setattr('farm.perception.registered_tags.assemble_dataset', assemble)
    result = read_registered_tags(owner, {'arm': arm, 'camera': 'oak', 'model_directory': 'fake'}, reg, clock=owner.clock)
    assert result['ok'] and result['motor_writes'] == 0 and not owner.writes
    assert result['result']['robot_motion_target'] is False
    assert owner.calls[-3][0] == 'robot_get_tags'
    assert owner.calls[-3][1]['tag_ids'] == [1, owner.tag_id, 3]


@pytest.mark.parametrize('arm,tag', [('right', 4), ('left', 2)])
def test_registered_read_rejects_crossed_registration_before_owner_access(arm, tag):
    owner = Owner(arm)
    with pytest.raises(ValueError, match='requires gripper tag'):
        read_registered_tags(owner, {'arm': arm}, {'binding': {'arm': arm, 'gripper_tag_id': tag}}, clock=owner.clock)
    assert owner.calls == []


@pytest.mark.parametrize('fault',['stream','head','anchor','mapping','model','geometry','raw_ranges',
                                  'validation','gripper_position','gripper_rotation','mount','moving','timestamp'])
def test_changed_state_refuses_registered_coordinates(fault):
    _,a=fixture()
    before,after,row,reg,status,_,g=a
    if fault=='stream':row['frame']['stream_id']='new-session'
    if fault=='head':reg['head_ticks_reference'][0]+=10
    if fault=='anchor':reg['anchor_corners_px_reference']=np.add(reg['anchor_corners_px_reference'],10).tolist()
    if fault=='mapping':status['result']['configuration']['config']['mapping']='other'
    if fault=='model':a[5]='other-model'
    if fault=='geometry':row['pose_3d']['geometry_config_sha256']='resized-print'
    if fault=='raw_ranges':
        before['result']['raw_calibration_ranges']['right_arm_shoulder_pan']['min_ticks']+=10
        after['result']['raw_calibration_ranges']=copy.deepcopy(before['result']['raw_calibration_ranges'])
    if fault=='validation':reg['residuals']['validation']['position_max_mm']=5
    if fault=='gripper_position':g[0,3]+=.01
    if fault=='gripper_rotation':g[:3,:3]=np.diag([-1,-1,1])
    if fault=='mount':reg['binding']['gripper_tag_mount']=dict(arm='right',body='moving_jaw',source='bad')
    if fault=='moving':before['result']['motors'][0]['Moving']=1;before['result']['motors'][6]['Moving']=1
    if fault=='timestamp':row['frame']['captured_at']=0
    with pytest.raises(ValueError):registered_observation(*a)


def test_ambiguous_paddle_can_only_return_a_center_not_an_offset_target():
    _,a=fixture()
    a[2]['pose_3d']['tags'][-1]['orientation_ambiguous']=True
    result=registered_observation(*a)
    assert result['tags'][-1]['arm_base_from_tag'] is None
    assert result['tags'][-1]['center_arm_base_mm'] is not None


def test_wrapper_refuses_missing_registration_without_motor_commands(tmp_path):
    owner=Owner()
    config=tmp_path/'config.json'
    config.write_text(json.dumps(dict(schema=1,arm='right',joints=['shoulder_pan','wrist_flex'])))
    robot=CalibrationRobot(owner,config,clock=owner.clock)
    answer=robot.call('robot_get_registered_tags',{})
    assert not answer['ok'] and not owner.calls
    assert not robot.call('robot_get_registered_tags',{'arm':'left'})['ok']


def test_wrapper_publishes_only_successful_registration(tmp_path,monkeypatch):
    owner=Owner()
    config=tmp_path/'config.json'
    config.write_text(json.dumps(dict(schema=1,arm='right',joints=['shoulder_pan','wrist_flex'])))
    robot=CalibrationRobot(owner,config,clock=owner.clock)
    _,a=fixture()
    reg=a[3]
    monkeypatch.setattr('farm.perception.gemma_calibration.run_calibration',lambda *a,**k:reg)
    assert robot.call('robot_calibrate_tags',{'mode':'registration'})['ok']
    assert json.loads(robot.registration_path.read_text())==reg
    rejected={'status':'REGISTRATION_REJECTED','base_from_camera':None}
    monkeypatch.setattr('farm.perception.gemma_calibration.run_calibration',lambda *a,**k:rejected)
    robot.call('robot_calibrate_tags',{'mode':'registration'})
    assert json.loads(robot.registration_path.read_text())==reg


def test_read_rejects_receipt_only_timestamps_before_using_fk():
    owner=Owner()
    original=owner.call
    def call(name,args,request_id=None):
        r=original(name,args,request_id)
        if name=='robot_get_tags':r['result']['observations']['oak']['frame']['timestamp_basis']='receipt_only_capture_delay_unknown'
        return r
    owner.call=call
    with pytest.raises(ValueError,match='capture timestamp'):
        read_registered_tags(owner,dict(arm='right',camera='oak'),{'binding':{'arm':'right'}},clock=owner.clock)
    assert not owner.writes
