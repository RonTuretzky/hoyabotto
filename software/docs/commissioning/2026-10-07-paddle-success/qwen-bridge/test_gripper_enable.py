"""Pickup profile: robot_set_gripper on a released arm enables all six joints (they hold), then moves the gripper."""
import tempfile,time
from pathlib import Path
from gemma_direct_client import DirectJointClient,atomic_json
arm=['right_arm_'+j for j in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')]
g='right_arm_gripper'
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp)
    state={'hardware_server':True,'execution_profile':'paddle-success-v1','control_mode':'direct_joint','supportsselectedjoints':arm,
           'supported_motors':arm,'commandable_motors':arm,'ranges':{n:[826,3268] for n in arm},'operator_armed':True,'started':7,
           'phase':'idle','ok':True,'enabled_motors':[],'lease_remaining':0,
           'rows':{n:{'Torque_Enable':0,'Status':0,'Present_Position':2000,'Present_Temperature':30,'Present_Load':0,'Present_Voltage':121} for n in arm},
           'time':time.time()}
    atomic_json(folder/'status.json',state)
    c=DirectJointClient(folder,{n:{'range_min':826,'range_max':3268} for n in arm})
    enabled=[];moved=[]
    def set_motor_enable(names,on):
        enabled.append(sorted(names))
        for n in names:state['rows'][n]['Torque_Enable']=1
        state.update(enabled_motors=sorted(arm),phase='holding',lease_remaining=100,time=time.time());atomic_json(folder/'status.json',state)
        return {'completed':True}
    def command(request,wait=True):
        moved.append(request);return {'accepted':True,'completed':True,'readbacks':{g:2300}}
    c.set_motor_enable=set_motor_enable;c._command=command
    c.readiness=lambda:{'available_to_accept_authorized_command':True,'joint_blockers':{},'blockers':[]}
    result=c.set_gripper('right',2300,3)
    assert enabled==[sorted(arm)],enabled                      # the whole arm, not only the gripper
    assert moved[0]['positions']=={g:2300} and sorted(result['auto_enabled_motors'])==sorted(arm) and result['completed']
print('Gripper enable: pickup profile auto-enables all six joints of the arm, then moves only the gripper')

# A gripper that stops short while holding is a reported outcome, not an error.
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp);state.update(time=time.time(),phase='holding',enabled_motors=sorted(arm),lease_remaining=100)
    for n in arm:state['rows'][n]['Torque_Enable']=1
    atomic_json(folder/'status.json',state)
    c=DirectJointClient(folder,{n:{'range_min':826,'range_max':3268} for n in arm})
    c.readiness=lambda:{'available_to_accept_authorized_command':True,'joint_blockers':{},'blockers':[]}
    c._command=lambda request,wait=True:{'accepted':True,'completed':False,'holding':True,'closure_outcome':'settled_short','readbacks':{g:1960}}
    r=c.set_gripper('right',1900,3)
    assert r['sequence_phase']=='stopped_short' and r['holding'] and 'object' in r['note']
    c._command=lambda request,wait=True:{'accepted':True,'completed':False,'holding':False,'closure_outcome':'halted'}
    try:c.set_gripper('right',1900,3)
    except RuntimeError as e:assert 'not completed' in str(e)
    else:raise AssertionError('released failure reported as success')
print('Gripper: stopped-short-and-holding reported as an outcome; other failures still raise')
