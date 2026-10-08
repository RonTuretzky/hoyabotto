import json,tempfile,threading,time
from pathlib import Path
from types import SimpleNamespace as C
from paddle_joint_executor import PaddleJointExecutor
r='right_arm_';lift,elbow=r+'shoulder_lift',r+'elbow_flex';RANGES={lift:[826,3268],elbow:[826,3268]};t=[0.];writes=[]
row={'Moving':0,'Present_Velocity':0}
def run(e,plant=lambda j,g:g,steps=600,until=None):
 out=None
 for i in range(steps):
  t[0]=round(t[0]+.1,6);out=e.tick({j:plant(j,e.goal[j]) for j in e.joints},telemetry_at=t[0],rows={j:row for j in e.joints})
  if not e.active or (until and until()):return out
 raise AssertionError('motion never finished')

# A three-waypoint path is one continuous motion: no settle pause at intermediate waypoints.
e=PaddleJointExecutor([lift,elbow],RANGES,lambda w:writes.append((t[0],dict(w))),clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':1,'session_started':7,'duration_s':6,'waypoints':[{lift:2500,elbow:2000},{lift:2300,elbow:2200},{lift:2100,elbow:2200}]},{lift:2697,elbow:2000},session_started=7)
out=run(e)
assert out['closure_outcome']=='endpoint_settled' and e.goal=={lift:2100,elbow:2200}
gaps=[b[0]-a[0] for a,b in zip(writes,writes[1:])]
assert max(gaps)<=e.interval+.11,('paused between waypoints',max(gaps),e.interval)
assert any(w[1].get(elbow)==2200 for w in writes) and writes[-1][1].get(lift)==2100
# Each leg is bounded; a path cannot close the gripper.
try:PaddleJointExecutor([lift],RANGES,writes.append,clock=lambda:t[0],wall=lambda:t[0]).start({'id':2,'session_started':7,'duration_s':5,'waypoints':[{lift:2697-342}]},{lift:2697},session_started=7)
except ValueError as x:assert '341' in str(x)
else:raise AssertionError('oversized leg accepted')

# halt mid-motion holds the last commanded goals and reports 'halted', not success.
writes.clear();e=PaddleJointExecutor([lift],RANGES,writes.append,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':3,'session_started':7,'positions':{lift:2400},'duration_s':3},{lift:2697},session_started=7)
run(e,until=lambda:e.goal[lift]<=2600)
held=e.goal[lift];out=e.halt({lift:held})
assert out['closure_outcome']=='halted' and out['endpoint_reached'] is False and not e.active and e.goal[lift]==held

# Owner: start, halt, replace while moving; nothing released.
from gemma_hardware_owner import HardwareOwner
names=[r+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]
class Bus:
 def __init__(self):
  self.motors=dict.fromkeys(names);self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=826,Max_Position_Limit=3268,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in names}
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
b=Bus();t[0]=100.
tel=lambda bus,n:dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=0,Present_Voltage=124,Moving=0,Present_Velocity=0,Status=0)
o=HardwareOwner([b],{n:C(range_min=826,range_max=3268,homing_offset=0) for n in names},tel,clock=lambda:t[0],wall=lambda:t[0],position_scope=names,paddle_profile=True,camera_metadata=lambda:{'received_at':t[0],'seq':1});o.inspect();o.enable(names,True)
def poll(k=1):
 for _ in range(k):
  t[0]=round(t[0]+.1,6)
  for n in names:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
  o.poll()
o.command({'id':10,'session_started':o.started,'op':'direct_joint','waypoints':[{lift:2200,elbow:2000},{lift:2400,elbow:2200}],'duration_s':4});poll(8)
assert o.state['phase']=='moving' and o.engine.active
o.command({'id':11,'session_started':o.started,'op':'halt'});poll()
assert o.state['completed']==11 and o.state['halted_command_id']==10 and o.state['closure_outcome']=='halted' and o.state['phase']=='holding' and o.enabled==set(names)
o.command({'id':12,'session_started':o.started,'op':'direct_joint','positions':{lift:1800},'duration_s':3});poll(4)
try:o.command({'id':13,'session_started':o.started,'op':'direct_joint','positions':{lift:2300},'duration_s':3})
except ValueError as x:assert 'replace=true' in str(x)
else:raise AssertionError('second motion accepted without replace')
o.command({'id':14,'session_started':o.started,'op':'direct_joint','positions':{lift:2300},'duration_s':3,'replace':True})
assert o.state['replaced_command_id']==12 and o.engine.command_id==14
poll(80);assert o.state['completed']==14 and o.state['closure_outcome']=='endpoint_settled' and b.r[lift]['Present_Position']==2300 and o.enabled==set(names)

# Client: wait=false returns once the motion starts; halt returns once the owner holds.
from gemma_direct_client import DirectJointClient,atomic_json
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);state={'hardware_server':True,'control_mode':'direct_joint','execution_profile':'paddle-success-v1','supportsselectedjoints':[lift],'supported_motors':[lift],'commandable_motors':[lift],'pickup_required_enabled_motors':[lift],'ranges':{lift:[826,3268]},'operator_armed':True,'started':7,'phase':'holding','ok':True,'enabled_motors':[lift],'lease_remaining':100,'goals':{lift:2697},'rows':{lift:{'Torque_Enable':1,'Status':0,'Present_Position':2697,'Present_Temperature':30,'Present_Load':0}}}
 def save():state['time']=time.time();atomic_json(folder/'status.json',state)
 save();c=DirectJointClient(folder,{lift:{'range_min':826,'range_max':3268}});stop=threading.Event()
 def owner():
  handled=set()
  while not stop.wait(.01):
   cmd=json.loads((folder/'command.json').read_text()) if (folder/'command.json').exists() else {}
   if cmd.get('id') in handled or not cmd:continue
   handled.add(cmd['id'])
   if cmd['op']=='direct_joint':state.update(accepted=cmd['id'],phase='moving')
   elif cmd['op']=='halt':state.update(completed=cmd['id'],halted_command_id=state.get('accepted'),closure_outcome='halted',phase='holding')
   save()
 th=threading.Thread(target=owner,daemon=True);th.start()
 try:
  res=c.execute({lift:2450},3,wait=False);assert res['started'] and not res['completed'] and res['phase']=='moving'
  assert c.motion()['moving']
  res=c.halt();assert res['halted'] and res['phase']=='holding'
 finally:stop.set();th.join(1)
print('Continuous motion: pass-through waypoints, bounded legs, halt holds, owner halt/replace without release, client wait=false and halt passed; no hardware')
