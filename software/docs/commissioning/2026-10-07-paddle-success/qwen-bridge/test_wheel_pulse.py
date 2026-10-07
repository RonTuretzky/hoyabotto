import json,tempfile,threading,time
from pathlib import Path
from types import SimpleNamespace as C
from wheel_pulse_executor import WheelPulseExecutor,WHEELS,wheel_ticks_per_s
L,R=WHEELS
assert wheel_ticks_per_s(.02,0)=={L:-261,R:261} and wheel_ticks_per_s(-.02,0)=={L:261,R:-261} # matches validated drive-pulse.py
for bad in [{'linear_m_s':.05,'angular_rad_s':0,'duration_s':1},{'linear_m_s':.02,'angular_rad_s':.2,'duration_s':1},{'linear_m_s':.02,'angular_rad_s':0,'duration_s':4},{'linear_m_s':0,'angular_rad_s':0,'duration_s':1}]:
 try:WheelPulseExecutor.check_request(bad)
 except ValueError:pass
 else:raise AssertionError('Unsafe base request accepted: '+json.dumps(bad))

class Plant:
 """Fake bus registers plus wheel physics: velocity mode with torque on moves the encoder."""
 def __init__(self,names):
  self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=0,Max_Position_Limit=4095,Present_Position=2000,Goal_Position=2000,Goal_Velocity=0,Lock=1,Torque_Limit=1000,Acceleration=0,P_Coefficient=16,Goal_Time=0,Status=0,Present_Load=0,Present_Voltage=120,Present_Velocity=0) for n in names}
  self.motors=dict.fromkeys(names);self.roll_after_release=0
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
 def step(self,dt):
  for n in WHEELS:
   r=self.r[n];v=r['Goal_Velocity'] if r['Torque_Enable'] and r['Operating_Mode']==1 else self.roll_after_release
   r['Present_Velocity']=v;r['Present_Position']=(r['Present_Position']+round(v*dt))%4096

# Executor alone: full forward pulse, then settings restored.
t=[0.];p=Plant(WHEELS);cam={'received_at':0,'seq':1}
def rows():return {n:dict(p.r[n]) for n in WHEELS}
def fresh():return t[0]-cam['received_at']<10
def run(e,steps=200):
 r=None
 for i in range(steps):
  t[0]=round(t[0]+.05,6);p.step(.05);r=e.tick({n:p.r[n]['Present_Position'] for n in WHEELS},telemetry_at=t[0],rows=rows())
  if not e.active:return r
 raise AssertionError('Pulse never finished')
e=WheelPulseExecutor(lambda n,f:p.r[n][f],lambda n,f,v:p.r[n].__setitem__(f,v),fresh,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':1,'session_started':7,'linear_m_s':.02,'angular_rad_s':0,'duration_s':1},rows(),session_started=7)
assert p.r[L]['Operating_Mode']==1 and p.r[L]['Torque_Enable']==1 and p.r[L]['Goal_Velocity']==-261
r=run(e)
d=r['base_result']['wheel_delta_ticks'];assert d[L]<-200 and d[R]>200 and r['base_result']['released'] and r['base_result']['stopped_early'] is None
assert all(p.r[n]['Torque_Enable']==0 and p.r[n]['Goal_Velocity']==0 and p.r[n]['Operating_Mode']==0 and p.r[n]['Lock']==1 and p.r[n]['Acceleration']==0 for n in WHEELS)
# Stale phone feed brakes early instead of faulting.
t[0]=100;cam['received_at']=100;e=WheelPulseExecutor(lambda n,f:p.r[n][f],lambda n,f,v:p.r[n].__setitem__(f,v),fresh,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':2,'session_started':7,'linear_m_s':.02,'angular_rad_s':0,'duration_s':3},rows(),session_started=7);cam['received_at']=89
r=run(e);assert r['base_result']['stopped_early']=='phone camera stale' and r['base_result']['pulse_s']<.2
# Health fault raises; abort zeroes velocity, turns torque off and restores settings.
t[0]=200;cam['received_at']=200;e=WheelPulseExecutor(lambda n,f:p.r[n][f],lambda n,f,v:p.r[n].__setitem__(f,v),fresh,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':3,'session_started':7,'linear_m_s':0,'angular_rad_s':.16,'duration_s':1},rows(),session_started=7);p.r[R]['Present_Load']=600
try:run(e)
except RuntimeError as x:assert 'health' in str(x)
else:raise AssertionError('Overload ignored')
assert e.abort()==[] and all(p.r[n]['Torque_Enable']==0 and p.r[n]['Goal_Velocity']==0 and p.r[n]['Operating_Mode']==0 for n in WHEELS);p.r[R]['Present_Load']=0
# Rolling after release is a fault.
t[0]=300;cam['received_at']=300;e=WheelPulseExecutor(lambda n,f:p.r[n][f],lambda n,f,v:p.r[n].__setitem__(f,v),fresh,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':4,'session_started':7,'linear_m_s':.02,'angular_rad_s':0,'duration_s':.5},rows(),session_started=7)
orig=p.step
def rolling(dt):
 orig(dt)
 if e.phase=='released':p.roll_after_release=40
p.step=rolling
try:run(e)
except RuntimeError as x:assert 'rolling' in str(x)
else:raise AssertionError('Rolling after release accepted')
p.step=orig;p.roll_after_release=0

# Owner: base pulse runs beside a released arm, STOP mid-drive stops the wheels, scope flag is required.
from gemma_hardware_owner import HardwareOwner
arm=['right_arm_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]
bus=Plant(arm+list(WHEELS));t[0]=1000;[bus.r[n].update(Min_Position_Limit=100,Max_Position_Limit=4000) for n in arm];meta={'received_at':1000,'seq':1}
def telemetry(b,n):r=b.r[n];return {k:r[k] for k in ('Present_Position','Present_Load','Present_Voltage','Present_Velocity','Status')}|{'Moving':0}
def owner(**kw):
 o=HardwareOwner([bus],{n:C(range_min=100,range_max=4000,homing_offset=0) for n in arm},telemetry,clock=lambda:t[0],wall=lambda:t[0],position_scope=arm,paddle_profile=True,camera_metadata=lambda:meta,**kw);o.inspect();return o
o=owner()
try:o.command({'id':1,'session_started':o.started,'op':'base_pulse','linear_m_s':.02,'angular_rad_s':0,'duration_s':1})
except ValueError as x:assert '--wheels' in str(x)
else:raise AssertionError('Base drive without --wheels')
o=owner(wheels=True);assert o.state['base_drive_supported'] and L not in o.commandable_names
o.command({'id':2,'session_started':o.started,'op':'base_pulse','linear_m_s':.02,'angular_rad_s':0,'duration_s':1})
for i in range(200):
 t[0]=round(t[0]+.05,6);bus.step(.05);o.poll()
 if not o.engine.active:break
assert o.state['completed']==2 and o.state['base_result']['released'] and o.state['phase']=='idle' and not o.enabled
o.command({'id':3,'session_started':o.started,'op':'base_pulse','linear_m_s':-.02,'angular_rad_s':0,'duration_s':3})
for i in range(5):t[0]=round(t[0]+.05,6);bus.step(.05);o.poll()
assert bus.r[L]['Torque_Enable']==1;o.command({'id':4,'session_started':o.started,'op':'stop'})
assert all(bus.r[n]['Torque_Enable']==0 and bus.r[n]['Goal_Velocity']==0 and bus.r[n]['Operating_Mode']==0 for n in WHEELS) and o.state['last_stop']['command_id']==3 and not o.engine.active
# An owner fault during a drive (here a wheel status fault) also stops the wheels.
t[0]=round(t[0]+.05,6);o.poll() # the stale torque-on row from before STOP must be re-read first
o.command({'id':5,'session_started':o.started,'op':'base_pulse','linear_m_s':.02,'angular_rad_s':0,'duration_s':2})
t[0]=round(t[0]+.05,6);bus.step(.05);bus.r[R]['Status']=8
try:o.poll()
except RuntimeError as x:o.release_all(str(x))
assert all(bus.r[n]['Torque_Enable']==0 and bus.r[n]['Goal_Velocity']==0 for n in WHEELS);bus.r[R]['Status']=0

# Client: drive_base waits for the released result.
from gemma_direct_client import DirectJointClient,atomic_json
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);state={'hardware_server':True,'control_mode':'direct_joint','execution_profile':'paddle-success-v1','base_drive_supported':True,'supportsselectedjoints':arm,'supported_motors':arm+list(WHEELS),'commandable_motors':arm,'ranges':{},'operator_armed':True,'started':7,'phase':'idle','ok':True,'enabled_motors':[],'lease_remaining':0,'rows':{n:{'Torque_Enable':0,'Status':0,'Present_Position':2000,'Present_Load':0} for n in arm+list(WHEELS)}}
 def save():state['time']=time.time();atomic_json(folder/'status.json',state)
 save();c=DirectJointClient(folder,{n:{'range_min':100,'range_max':4000} for n in arm});stop=threading.Event()
 def fake_owner():
  while not stop.wait(.01):
   if (folder/'command.json').exists():
    cmd=json.loads((folder/'command.json').read_text())
    if cmd['op']=='base_pulse':state.update(completed=cmd['id'],base_result={'released':True,'wheel_delta_ticks':{L:-261,R:261}});save();return
 th=threading.Thread(target=fake_owner,daemon=True);th.start()
 try:res=c.drive_base(.02,0,1);assert res['completed'] and res['base_result']['wheel_delta_ticks'][R]==261
 finally:stop.set();th.join(1)
 state['base_drive_supported']=False;save()
 try:c.drive_base(.02,0,1)
 except (ValueError,RuntimeError) as x:assert '--wheels' in str(x)
 else:raise AssertionError('Client sent base pulse to an owner without wheels')
print('Wheel pulses: validated speed mapping and limits, full pulse with restore, early camera brake, health fault abort, rolling-after-release, owner STOP/fault stop wheels, --wheels scope, client completion passed; no hardware')
