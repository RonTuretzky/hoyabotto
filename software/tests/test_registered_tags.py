"""Read-only registration consumption and stale-binding rejection."""
import copy
import json

import numpy as np
import pytest

from farm.perception.registered_tags import binding_state, registered_observation, read_registered_tags
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


@pytest.mark.parametrize('fault,cause',[('head','Head moved'),('mapping','feetech_degrees_v1'),
    ('motor_calibration','motor calibration hash changed'),('model','robot model'),
    ('geometry','apriltag-geometry.json'),('raw_ranges','calibration ranges changed'),
    ('camera_id','Camera identity changed'),('camera_geometry','camera geometry changed'),
    ('validation','residuals'),('gripper_position','disagrees with the arm model'),
    ('gripper_rotation','disagrees with the arm model'),('mount','mount record changed'),
    ('moving','moving'),('timestamp','RGB capture')])
def test_changed_state_refuses_registered_coordinates_naming_cause(fault, cause):
    _,a=fixture()
    before,after,row,reg,status,_,g=a
    if fault=='head':reg['head_ticks_reference'][0]+=10
    if fault=='mapping':status['result']['configuration']['config']['mapping']='other'
    if fault=='motor_calibration':status['result']['configuration']['config']['calibration_sha256']='recalibrated'
    if fault=='model':a[5]='other-model'
    if fault=='geometry':row['pose_3d']['geometry_config_sha256']='resized-print'
    if fault=='raw_ranges':
        before['result']['raw_calibration_ranges']['right_arm_shoulder_pan']['min_ticks']+=10
        after['result']['raw_calibration_ranges']=copy.deepcopy(before['result']['raw_calibration_ranges'])
    if fault=='camera_id':row['frame']['camera_id']='oak-replacement'
    if fault=='camera_geometry':row['pose_3d']['calibration_sha256']='other-intrinsics'
    if fault=='validation':reg['residuals']['validation']['position_max_mm']=5
    if fault=='gripper_position':g[0,3]+=.01
    if fault=='gripper_rotation':g[:3,:3]=np.diag([-1,-1,1])
    if fault=='mount':reg['binding']['gripper_tag_mount']=dict(arm='right',body='moving_jaw',source='bad')
    if fault=='moving':before['result']['motors'][0]['Moving']=1;before['result']['motors'][6]['Moving']=1
    if fault=='timestamp':row['frame']['captured_at']=0
    with pytest.raises(ValueError, match=cause) as refused:registered_observation(*a)
    if fault not in ('moving','timestamp'):
        assert 'Fix:' in str(refused.value)


def geometry(projection='camera_pinhole_with_factory_distortion', fx=500.):
    return dict(camera_id='oak-test', image_size_px=[640,360], projection=projection,
                intrinsics=[[fx,0,320],[0,fx,180],[0,0,1]],
                effective_distortion=[-.3,.1,0,0,0] if projection!='rectified_pinhole' else [0]*5,
                coordinate_frame='CAM_A_optical', lens_position=120)


def with_geometry(a, registered, current):
    a[3]['binding'].update(camera_geometry=registered, camera_calibration_sha256=fingerprint(registered))
    a[2]['pose_3d'].update(camera_calibration=current, calibration_sha256=fingerprint(current))


def test_oak_restart_with_same_geometry_is_accepted_after_gripper_consistency():
    _,a=fixture()
    with_geometry(a, geometry(), geometry())
    a[2]['frame']['stream_id']='restarted-session'
    result=registered_observation(*a)
    assert result['camera_stream_reverified'] and not result['re_anchored']
    [event]=result['binding_events']
    assert event['event']=='camera_stream_reverified'
    assert (event['previous_stream_id'],event['stream_id'])==('one','restarted-session')
    assert event['gripper_consistency']['position_mm']<4
    state=result['binding_state']
    assert state['verified_stream_id']=='restarted-session'
    # Subsequent reads from the same new session are not re-verification events.
    again=registered_observation(*a,state=state)
    assert again['binding_events']==[] and not again['camera_stream_reverified']


def test_oak_restart_refuses_first_read_when_gripper_consistency_fails():
    _,a=fixture()
    a[2]['frame']['stream_id']='restarted-session'
    a[6][0,3]+=.01
    with pytest.raises(ValueError, match='OAK publisher restarted.*post-restart gripper-consistency check failed'):
        registered_observation(*a)


@pytest.mark.parametrize('current,cause', [
    (geometry(fx=505.), 'intrinsics'),
    (geometry(projection='rectified_pinhole'), "projection 'camera_pinhole_with_factory_distortion' -> 'rectified_pinhole'.*--wide"),
    ({**geometry(), 'image_size_px':[1280,720]}, 'resolution'),
])
def test_changed_intrinsics_projection_or_resolution_refused_even_on_same_stream(current, cause):
    _,a=fixture()
    with_geometry(a, geometry(), current)
    with pytest.raises(ValueError, match=f'camera geometry changed.*{cause}'):
        registered_observation(*a)


def test_cart_move_with_unchanged_head_re_anchors_table_tag():
    _,a=fixture()
    row,reg=a[2],a[3]
    original=copy.deepcopy(reg['anchor_corners_px_reference'])
    row['tags'][0]['corners_px']=np.add(row['tags'][0]['corners_px'],[25,-12]).tolist()
    row['pose_3d']['tags'][0]['center_camera_mm']=[40,-20,610]
    result=registered_observation(*a)
    assert result['re_anchored'] and not result['camera_stream_reverified']
    [event]=result['binding_events']
    assert event['event']=='table_anchor_re_anchored'
    assert event['previous_anchor_corners_px']==original
    assert event['anchor_corner_shift_px']==pytest.approx(np.hypot(25,12))
    state=result['binding_state']
    assert state['anchor_corners_px_reference']==row['tags'][0]['corners_px']
    assert state['anchor_center_camera_mm_reference']==[40,-20,610]
    assert state['anchor_source']=='re_anchor'
    assert reg['anchor_corners_px_reference']==original, 'the registration itself stays immutable'
    assert registered_observation(*a,state=state)['binding_events']==[]


def test_cart_move_is_not_re_anchored_when_gripper_consistency_fails():
    _,a=fixture()
    a[2]['tags'][0]['corners_px']=np.add(a[2]['tags'][0]['corners_px'],25).tolist()
    a[6][1,3]+=.008
    with pytest.raises(ValueError, match='Table tag 1 moved.*cannot be re-anchored'):
        registered_observation(*a)


def test_head_move_is_refused_even_if_it_looks_like_a_cart_move():
    _,a=fixture()
    a[2]['tags'][0]['corners_px']=np.add(a[2]['tags'][0]['corners_px'],25).tolist()
    a[3]['head_ticks_reference']=[2000,1990]
    with pytest.raises(ValueError, match=r'Head moved.*Fix: return the head to \[2000.0, 1990.0\]'):
        registered_observation(*a)


def test_binding_state_from_another_registration_is_refused():
    _,a=fixture()
    state=binding_state(a[3])
    a[3]['residuals']['train']['position_rms_mm']=.2
    with pytest.raises(ValueError, match='different registration'):
        registered_observation(*a,state=state)


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


def test_wrapper_persists_restart_and_re_anchor_events_and_new_registration_resets_them(tmp_path, monkeypatch):
    owner=Owner()
    config=tmp_path/'config.json'
    config.write_text(json.dumps(dict(schema=1,arm='right',joints=['shoulder_pan','wrist_flex'],model_directory='fake')))
    robot=CalibrationRobot(owner,config,clock=owner.clock)
    _,a=fixture()
    reg,base_from_gripper=a[3],a[6]
    robot.registration_path.write_text(json.dumps(reg))
    monkeypatch.setattr('farm.perception.registered_tags.assemble_dataset',lambda captures,model:
        {'binding':{'robot_model_sha256':'model'},'samples':[{'base_from_gripper':base_from_gripper.tolist()}]})
    first=robot.call('robot_get_registered_tags',{})
    assert first['ok'] and first['result']['binding_events']==[] and not robot.binding_state_path.exists()
    original=owner.call
    def restarted_and_cart_moved(name,args,request_id=None):
        r=original(name,args,request_id)
        if name=='robot_get_tags':
            row=r['result']['observations']['oak']
            row['frame']['stream_id']='restarted'
            row['tags'][0]['corners_px']=np.add(row['tags'][0]['corners_px'],30).tolist()
        return r
    owner.call=restarted_and_cart_moved
    moved=robot.call('robot_get_registered_tags',{})
    assert moved['ok'] and moved['motor_writes']==0 and not owner.writes
    assert [e['event'] for e in moved['result']['binding_events']]==['camera_stream_reverified','table_anchor_re_anchored']
    saved=json.loads(robot.binding_state_path.read_text())
    assert saved['verified_stream_id']=='restarted' and saved['anchor_source']=='re_anchor' and len(saved['events'])==2
    again=robot.call('robot_get_registered_tags',{})
    assert again['ok'] and again['result']['binding_events']==[]
    assert json.loads(robot.registration_path.read_text())==reg
    # A new registration supersedes the saved stream/anchor state.
    monkeypatch.setattr('farm.perception.gemma_calibration.run_calibration',lambda *a,**k:reg)
    assert robot.call('robot_calibrate_tags',{'mode':'registration'})['ok']
    assert not robot.binding_state_path.exists()


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


@pytest.mark.parametrize('stale_reads', [1, 3])
def test_read_waits_for_a_frame_captured_inside_the_encoder_bracket(monkeypatch, stale_reads):
    """2026-10-10 live: the OAK publishes ~0.45 s after capture, so the first tag read is from before the
    encoder sample and was refused as 'not between coherent owner samples'. The read now re-reads."""
    owner, args = fixture('right')
    reg, base_from_gripper = args[3], args[6]
    monkeypatch.setattr('farm.perception.registered_tags.assemble_dataset', lambda captures, model: {
        'binding': {'robot_model_sha256': 'model'}, 'samples': [{'base_from_gripper': base_from_gripper.tolist()}]})
    original = owner.call
    tag_reads = []
    def call(name, args, request_id=None):
        r = original(name, args, request_id)
        if name == 'robot_get_tags':
            if len(tag_reads) < stale_reads:
                r['result']['observations']['oak']['frame']['captured_at'] -= .5   # published late: capture precedes the bracket
            tag_reads.append(r['result']['observations']['oak']['frame']['captured_at'])
        return r
    owner.call = call
    result = read_registered_tags(owner, {'arm': 'right', 'camera': 'oak', 'model_directory': 'fake'}, reg, clock=owner.clock)
    assert result['ok'] and len(tag_reads) == stale_reads + 1 and not owner.writes
    assert [n for n, _ in owner.calls if n == 'robot_get_state'][-2:] == ['robot_get_state', 'robot_get_state']


def test_read_gives_up_waiting_after_eight_frames_without_motion(monkeypatch):
    owner, args = fixture('right')
    reg = args[3]
    original = owner.call
    def call(name, args, request_id=None):
        r = original(name, args, request_id)
        if name == 'robot_get_tags':
            r['result']['observations']['oak']['frame']['captured_at'] -= .5
        return r
    owner.call = call
    reads_before = len([n for n, _ in owner.calls if n == 'robot_get_tags'])   # fixture() itself read once
    with pytest.raises(ValueError, match='coherent owner samples'):
        read_registered_tags(owner, {'arm': 'right', 'camera': 'oak', 'model_directory': 'fake'}, reg, clock=owner.clock)
    assert len([n for n, _ in owner.calls if n == 'robot_get_tags']) - reads_before == 8 and not owner.writes
