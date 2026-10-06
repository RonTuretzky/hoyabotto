from types import SimpleNamespace
import pytest
from farm.adapters.robot_lerobot import LeRobotXLeRobot
from farm.config import RobotCfg
from lerobot.motors import MotorCalibration

class Bus:
    def __init__(self, name, events, fail_enable=False):
        self.name=name; self.events=events; self.is_connected=False
        self.calibration={}; self.goal={}; self.fail_enable=fail_enable
        self.registers={}
    def connect(self): self.is_connected=True
    def disconnect(self,disable_torque=True):
        assert not disable_torque
        self.is_connected=False
    def disable_torque(self, motors, **kw): self.events.append(('off',tuple(motors)))
    def enable_torque(self,motors,**kw):
        assert all(m in self.goal for m in motors)
        self.events.append(('on',tuple(motors)))
        if self.fail_enable: raise OSError('injected enable failure')
    def read(self, field,m,**kw):
        if field=='Goal_Position':return self.goal[m]
        if (field,m) in self.registers:return self.registers[(field,m)]
        return {'Homing_Offset':0,'Min_Position_Limit':100,'Max_Position_Limit':3900}[field]
    def write(self,field,m,value,**kw):
        assert not m.startswith('base_')
        self.registers[(field,m)]=value
        self.events.append((field,m,value))
    def sync_read(self,field,motors,normalize=True,**kw):
        assert not any(m.startswith('base_') for m in motors)
        return dict.fromkeys(motors,0 if normalize else 2000)
    def sync_write(self,field,values,**kw):
        assert field=='Goal_Position'
        assert not any(m.startswith('base_') for m in values)
        self.goal.update(values); self.events.append(('goal',dict(values)))

def make(tmp_path, fail=False):
    events=[]; a=LeRobotXLeRobot.__new__(LeRobotXLeRobot)
    a.cfg=RobotCfg();a._connected=False
    path=tmp_path/'cal.json';path.write_text('{}')
    names=['left_arm_shoulder_pan','head_motor_1','right_arm_gripper']
    a.robot=SimpleNamespace(bus1=Bus('left',events),bus2=Bus('right',events,fail),
      left_arm_motors=names[:1],head_motors=names[1:2],right_arm_motors=names[2:],
      calibration_fpath=path,calibration={n:MotorCalibration(id=i+1,drive_mode=0,homing_offset=0,range_min=100,range_max=3900) for i,n in enumerate(names)})
    return a,events

def test_holds_before_torque_and_excludes_wheels(tmp_path):
    a,events=make(tmp_path);a.connect()
    first_on=next(i for i,e in enumerate(events) if e[0]=='on')
    goals={m for e in events[:first_on] if e[0]=='goal' for m in e[1]}
    assert goals==set(a.robot.calibration)
    r=a.move_to({'left_arm_shoulder_pan':50},max_step=3)
    assert r.ok and r.value=={'left_arm_shoulder_pan':3}
    assert not a.move_to({'base_left_wheel':1}).ok
    a.disconnect()
    assert not a.robot.bus1.is_connected and not a.robot.bus2.is_connected

def test_connection_failure_releases_both_buses(tmp_path):
    a,events=make(tmp_path,True)
    with pytest.raises(OSError):a.connect()
    assert not a._connected and not a.robot.bus1.is_connected and not a.robot.bus2.is_connected
    assert events[-2:]==[('off',('left_arm_shoulder_pan','head_motor_1')),('off',('right_arm_gripper',))]

def test_selected_position_settings_are_verified_before_enable(tmp_path):
    a,events=make(tmp_path)
    a.cfg.position_settings={'right_arm_gripper':{'P_Coefficient':32,'Torque_Limit':500,'Goal_Velocity':200}}
    a.connect()
    first_on=next(i for i,e in enumerate(events) if e[0]=='on')
    for field,value in a.cfg.position_settings['right_arm_gripper'].items():
        assert (field,'right_arm_gripper',value) in events[:first_on]
    assert a.robot.bus1.registers[('P_Coefficient','left_arm_shoulder_pan')]==16
    assert not any(e[0]=='Torque_Limit' and e[1]!='right_arm_gripper' for e in events)
    a.disconnect()

def test_position_setting_readback_failure_never_enables_motors(tmp_path):
    a,events=make(tmp_path)
    a.cfg.position_settings={'right_arm_gripper':{'P_Coefficient':32}}
    read=a.robot.bus2.read
    a.robot.bus2.read=lambda field,m,**kw: 16 if field=='P_Coefficient' else read(field,m,**kw)
    with pytest.raises(RuntimeError,match='Position setting readback mismatch'): a.connect()
    assert not any(e[0]=='on' for e in events)
    assert not a.robot.bus1.is_connected and not a.robot.bus2.is_connected

@pytest.mark.parametrize('settings', [
    {'base_left_wheel':{'P_Coefficient':32}},
    {'right_arm_gripper':{'Homing_Offset':100}},
    {'right_arm_gripper':{'P_Coefficient':33}},
    {'right_arm_gripper':{'Torque_Limit':1001}},
    {'right_arm_gripper':{'Goal_Velocity':0}},
])
def test_invalid_position_settings_rejected_before_bus_connect(tmp_path,settings):
    a,events=make(tmp_path);a.cfg.position_settings=settings
    with pytest.raises(ValueError): a.connect()
    assert events==[]
