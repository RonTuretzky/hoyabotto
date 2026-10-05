import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
import math
from types import SimpleNamespace as C
import pytest
from carton_session import camera_allowed, plan_delta, enable_primed_goal, elbow_trajectory

CFG={'start_max_age_s':10,'motion_max_age_s':15,'hold_max_age_s':30}

def test_gap_does_not_drop_held_arm_but_blocks_new_movement():
    assert camera_allowed(20,'holding',CFG)
    assert not camera_allowed(20,'holding',CFG,starting_move=True)
    assert camera_allowed(9,'holding',CFG,starting_move=True)

def test_short_step_can_finish_during_brief_gap():
    assert camera_allowed(12,'moving',CFG)
    assert not camera_allowed(16,'moving',CFG)
    assert not camera_allowed(31,'holding',CFG)
    assert not camera_allowed(11,'starting',CFG)

@pytest.mark.parametrize('age',[-1,math.inf,math.nan,None])
def test_invalid_or_unbounded_camera_age_rejected(age):
    assert not camera_allowed(age,'holding',CFG)

def test_physical_step_and_motor_scope_remain_bounded():
    cal={'j':C(range_min=100,range_max=1000)}
    assert plan_delta({'j':500},{'j':68},cal,['j'])=={'j':568}
    for delta in ({'j':69},{'j':float('nan')},{'j':True},{'wheel':1},{}, {'j':-68}):
        with pytest.raises(ValueError):plan_delta({'j':110},delta,cal,['j'])

def test_recovery_delivers_goal_if_enable_discards_internal_trajectory():
    registers={'Torque_Enable':0, 'Goal_Position':3900, 'Lock':0}
    active_goal=None
    seen=[]
    def write(name, field, value):
        nonlocal active_goal
        assert name=='elbow'
        if field=='Torque_Enable':
            # Even firmware that starts immediately sees only the checked goal.
            assert registers['Goal_Position']==3063
            active_goal=None
        registers[field]=value
        if field=='Goal_Position' and registers['Torque_Enable']:
            active_goal=value
        seen.append((field,value))
    enable_primed_goal(write,'elbow',3063,lambda n,t:write(n,'Goal_Position',t))
    assert active_goal==3063
    assert registers['Lock']==1
    assert all(value==3063 for field,value in seen if field=='Goal_Position')

def test_recovery_stops_before_enable_if_priming_is_not_verified():
    seen=[]
    def failed_write(name,field,value):
        seen.append(field)
        raise RuntimeError('Readback mismatch')
    with pytest.raises(RuntimeError):enable_primed_goal(failed_write,'elbow',3063,lambda n,t:failed_write(n,'Goal_Position',t))
    assert seen==['Goal_Position']

@pytest.mark.parametrize('failure',[None,'ack','speed'])
def test_elbow_packet_is_atomic_and_verifies_speed(failure):
    packets=[]
    def send(port,id,addr,size,data):
        packets.append((id,addr,size,data))
        return (1 if failure=='ack' else 0),0
    fields={'Acceleration':10,'Goal_Position':3063,'Goal_Time':0,'Goal_Velocity':0 if failure=='speed' else 100}
    bus=C(packet_handler=C(writeTxRx=send),port_handler=object(),motors={'right_arm_elbow_flex':C(id=3)},
          _is_comm_success=lambda x:x==0,_is_error=bool,read=lambda f,*a,**k:fields[f])
    if failure:
        with pytest.raises(RuntimeError):elbow_trajectory(bus,'right_arm_elbow_flex',3063)
    else:elbow_trajectory(bus,'right_arm_elbow_flex',3063)
    assert packets==[(3,41,7,[10,247,11,0,0,100,0])]
