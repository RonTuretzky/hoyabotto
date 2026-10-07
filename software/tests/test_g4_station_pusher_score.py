"""Adversarial measurement-contract tests; synthetic states are not dynamics evidence."""
import copy
import mujoco
import numpy as np
import pytest
from planter.g4_station_pusher_score import score_pusher_motion


@pytest.fixture
def sample():
    vertices=' '.join(str(v) for x in (-.03,.03) for y in (-.02,.02) for z in (-.004,.004) for v in (x,y,z))
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep=".002"/>
      <asset><mesh name="shape" vertex="{vertices}"/></asset><worldbody>
      <geom name="transferred_pusher_rest" type="box" size=".05 .05 .02" pos="0 0 .02"/>
      <body name="pusher"><freejoint name="pusher_free"/><geom name="pusher_visual" type="mesh" mesh="shape" mass=".02"/></body>
      <body name="right_gripper_link"><geom name="right_wrist_roll_follower" type="box" size=".01 .01 .01"/>
      <body name="right_moving_jaw_so101_v1_link"><geom name="right_moving_jaw" type="box" size=".01 .01 .01"/></body></body>
      <geom name="other_fixture" type="box" size=".05 .05 .01" pos="0 0 .11"/>
      </worldbody></mujoco>''')
    tool=model.geom('pusher_visual').id;rest=model.geom('transferred_pusher_rest').id
    fixed=model.geom('right_wrist_roll_follower').id;moving=model.geom('right_moving_jaw').id
    bid=model.body('pusher').id;grip=model.body('right_gripper_link').id
    def contact(other,normal):
        frame=np.array([[1,0,0],[0,1,0],[0,0,1]],float)
        if normal==(-1,0,0):frame=np.diag([-1,-1,1])
        if normal==(0,0,1):frame=np.array([[0,0,1],[1,0,0],[0,1,0]])
        return dict(geom1=other,geom2=tool,frame=frame.tolist(),wrench=[.1,0,0,0,0,0],position_m=[0,0,.04],distance_m=0.)
    jaw=[contact(fixed,(1,0,0)),contact(moving,(-1,0,0))];support=contact(rest,(0,0,1))
    phases=[('initial',1),('pusher_initial_settle',25)]+[('pusher_'+p,n) for p,n in
        [('approach_above',10),('lower_outside',10),('approach_handle',10),('close',100),('lift',100),
         ('hold',1050),('lower',100),('release',100),('disengage',10),('withdraw',100),('released_hold',550)]]
    states=[]
    for phase,count in phases:
        for j in range(count):
            xpos=np.zeros((model.nbody,3));xpos[bid]=[0,0,.124 if phase in ('pusher_lift','pusher_hold') else .044];xpos[grip]=xpos[bid]
            contacts=[]
            if phase in ('pusher_close','pusher_lift','pusher_hold','pusher_lower'):contacts+=copy.deepcopy(jaw)
            if phase not in ('pusher_lift','pusher_hold') and (phase!='pusher_lower' or j>=70):contacts+=[copy.deepcopy(support)]
            states.append(dict(time_s=len(states)*.002,phase=phase,xpos=xpos,xmat=np.tile(np.eye(3),(model.nbody,1,1)),
                qvel=np.zeros(model.nv),contacts=contacts,step_contacts=copy.deepcopy(contacts)))
    return model,states,jaw,support


def test_complete_measurement_contract_does_not_claim_full_task(sample):
    model,states,_,_=sample;result=score_pusher_motion(model,states)
    assert result['passed'],result
    assert result['requires_native_trace_audit'] and not result['full_assembly_success'] and not result['physical_success']


def test_late_capture_cannot_pass(sample):
    model,states,_,_=sample
    for s in states:
        if s['phase']=='pusher_close':s['contacts']=[]
    assert 'grip_not_established_before_lift' in score_pusher_motion(model,states)['failure_reasons']


def test_lateral_rest_contact_is_not_setdown(sample):
    model,states,_,support=sample;rest=support['geom1']
    for s in states:
        if s['phase']=='pusher_lower':
            for c in s['contacts']:
                if c['geom1']==rest:c['frame']=np.eye(3).tolist()
    assert 'supported_setdown_not_observed_before_release' in score_pusher_motion(model,states)['failure_reasons']


def test_unilateral_robot_support_is_not_free_release(sample):
    model,states,jaw,_=sample
    for s in states:
        if s['phase']=='pusher_released_hold':s['contacts'].append(copy.deepcopy(jaw[0]))
    assert 'release_not_free_and_supported' in score_pusher_motion(model,states)['failure_reasons']


def test_opposed_jaws_do_not_hide_support_from_another_fixture(sample):
    model,states,_,support=sample
    for s in states:
        if s['phase']=='pusher_hold':
            contact=copy.deepcopy(support);contact['geom1']=model.geom('other_fixture').id
            s['contacts'].append(contact)
    result=score_pusher_motion(model,states)
    assert 'nonjaw_support_during_carry' in result['failure_reasons']
    assert result['maximum_forbidden_carry_support_n']==pytest.approx(.1)
    assert result['forbidden_carry_support_duration_s']==pytest.approx(2.1)
    assert result['forbidden_carry_support_impulse_n_s']==pytest.approx(.21)


def test_rigid_rotation_without_centroid_motion_is_slip(sample):
    model,states,_,_=sample;bid=model.body('pusher').id
    for s in states:
        if s['phase']=='pusher_hold':s['xmat'][bid]=[[0,-1,0],[1,0,0],[0,0,1]]
    assert 'whole_pusher_grasp_drift' in score_pusher_motion(model,states)['failure_reasons']


def test_missing_release_and_nonfinite_pose_fail_closed(sample):
    model,states,_,_=sample
    assert not score_pusher_motion(model,[s for s in states if s['phase']!='pusher_released_hold'])['passed']
    states[10]['xpos'][model.body('pusher').id,0]=np.nan
    assert not score_pusher_motion(model,states)['passed']
