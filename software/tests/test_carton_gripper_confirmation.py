import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
from types import SimpleNamespace
import pytest
from carton_session import confirm_gripper_spike


def row(temp=36, status=0, load=-128, torque=0, position=1465):
    return dict(Present_Temperature=temp,Status=status,Present_Load=load,
                Torque_Enable=torque,Present_Position=position)


def exercise(initial=None, phase='moving', goal=1500, attempts=0, samples=None, guard=None):
    calls=[];records=[]
    values=iter(samples or [row(),row(),row()])
    def read():
        calls.append('read');return next(values)
    def check():
        if guard:guard()
    def record(event,value):records.append((event,value))
    def run():
        return confirm_gripper_spike(initial or row(82),phase,goal,
            SimpleNamespace(range_min=1292,range_max=2802),attempts,
            lambda:calls.append('release'),read,
            lambda field,value:calls.append((field,value)),
            lambda seconds:calls.append(('sleep',seconds)),record,check)
    return run,calls,records


def test_spike_releases_then_three_coherent_cool_samples_and_same_bounded_goal():
    run,calls,records=exercise()
    result,count=run()
    assert calls[:7]==['release',('sleep',.1),'read',('sleep',.1),'read',('sleep',.1),'read']
    assert calls[7:]==[('Goal_Position',1465),('Torque_Enable',1),('Lock',1),('Goal_Position',1500)]
    assert result['Present_Temperature']==36 and count==1
    assert [event for event,_ in records]==['gripper_temperature_trigger']+['gripper_temperature_confirmation']*3+['gripper_temperature_resumed']
    assert records[0][1]['Present_Temperature']==82


@pytest.mark.parametrize('bad',[row(82),row(status=4),row(torque=1),row(load=251)])
def test_sustained_heat_fault_unreleased_or_loaded_followup_never_reenables(bad):
    run,calls,records=exercise(samples=[bad])
    with pytest.raises(RuntimeError):run()
    assert calls==['release',('sleep',.1),'read']
    assert records[-1][1]==bad


@pytest.mark.parametrize('kwargs',[{'phase':'elbow_recovery'}, {'initial':row(82,status=4)},
                                 {'initial':row(82,load=251)}, {'attempts':2}])
def test_ineligible_or_exhausted_confirmation_does_not_retry(kwargs):
    run,calls,_=exercise(**kwargs)
    with pytest.raises(RuntimeError):run()
    assert not calls


@pytest.mark.parametrize('goal,samples',[(1600,None),(1500,[row(position=1200)]*3)])
def test_release_drift_outside_range_or_original_step_cannot_create_new_motion(goal,samples):
    run,calls,_=exercise(goal=goal,samples=samples)
    with pytest.raises(RuntimeError):run()
    assert not any(isinstance(call,tuple) and call[0]=='Torque_Enable' for call in calls)


def test_original_deadline_or_stop_failure_is_not_silently_dropped():
    def guard():raise RuntimeError('Original deadline expired')
    run,calls,_=exercise(guard=guard)
    with pytest.raises(RuntimeError,match='deadline'):run()
    assert calls==['release']


def test_normal_reading_does_not_release():
    run,calls,_=exercise(initial=row())
    result,count=run()
    assert count==0 and result['Present_Temperature']==36 and not calls
