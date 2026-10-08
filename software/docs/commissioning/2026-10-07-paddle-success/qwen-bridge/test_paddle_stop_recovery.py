"""No STOP latch: STOP and faults release all motors, the owner returns to idle, motors stay released until an explicit enable. Fake hardware only."""
import ast,json,tempfile,threading,time
from pathlib import Path
from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner,atomic
from gemma_direct_client import DirectJointClient
from paddle_segments import paddle_target_segments
names=['right_arm_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]
others=['left_arm_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]+['head_motor_1','head_motor_2','base_left_wheel','base_right_wheel']
class Bus:
 def __init__(self):
  self.motors=dict.fromkeys(names+others);self.stuck=set();self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=100,Max_Position_Limit=4000,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in self.motors}
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:
   if n not in self.stuck:self.r[n]['Torque_Enable']=0
def telemetry(bus,n):
 r=bus.r[n]
 if r['Torque_Enable']:r['Present_Position']=r['Goal_Position'] # perfect tracking
 return dict(Present_Position=r['Present_Position'],Present_Load=0,Present_Voltage=124,Moving=0,Present_Velocity=0,Status=r['Status'])
cal={n:C(range_min=100,range_max=4000,homing_offset=0) for n in names}
meta=lambda:{'received_at':time.time(),'seq':time.time_ns()}
def make():
 b=Bus();o=HardwareOwner([b],cal,telemetry,position_scope=names,paddle_profile=True,camera_metadata=meta);o.inspect();return o,b
checks=0

# 1. Owner alone: STOP mid-move releases all, returns to idle with no latch; explicit enable then a move completes.
o,b=make();o.enable(names,True)
o.command({'id':1,'session_started':o.started,'op':'direct_joint','positions':{names[1]:1800},'duration_s':2});assert o.engine.active
o.poll();o.command({'id':2,'session_started':o.started,'op':'stop'})
assert not o.enabled and not o.engine.active and all(b.r[n]['Torque_Enable']==0 for n in b.motors) and o.state['phase']=='idle' and o.state['operator_armed'] and o.state['ok'] and o.state['stop_latched'] is False
assert o.state['last_stop']['command_id']==1 and o.state['last_stop']['reason']=='Operator STOP' and o.state['stop_count']==1 and o.state['completed']==2;checks+=1
goal=b.r[names[1]]['Goal_Position'];o.poll();assert b.r[names[1]]['Goal_Position']==goal and not o.enabled # cancelled move is never resumed, nothing re-enables
o.enable(names,True);assert o.enabled==set(names)
o.command({'id':3,'session_started':o.started,'op':'direct_joint','positions':{names[1]:goal-200},'duration_s':2})
deadline=time.time()+10
while o.engine.active and time.time()<deadline:o.poll();time.sleep(.05)
assert o.state['completed']==3 and o.state['closure_outcome']=='endpoint_settled' and o.goals[names[1]]==goal-200 and o.state['stop_count']==1;checks+=1

# 2. A failed release keeps ok false; the owner refuses enable until a STOP confirms release.
o,b=make();o.enable(names,True);b.stuck.add(names[0])
o.command({'id':1,'session_started':o.started,'op':'stop'})
assert o.state['ok'] is False and o.state['release_errors'] and names[0] in o.enabled and o.state['last_stop']['released'] is False and o.state['phase']=='holding';checks+=1
try:o.enable(names[1:2],True)
except ValueError as exc:assert 'OWNER_NOT_HEALTHY' in str(exc)
else:raise AssertionError('Enable accepted after failed release')
with tempfile.TemporaryDirectory() as tmp:
 o.publish();atomic(Path(tmp)/'status.json',o.state);ready=DirectJointClient(tmp,{n:{'range_min':100,'range_max':4000} for n in names}).readiness()
 assert not ready['motion_ready'] and any(x.startswith('OWNER_NOT_HEALTHY') for x in ready['blockers']) and not any('LATCH' in x for x in ready['blockers']);checks+=1
b.stuck.clear();o.command({'id':2,'session_started':o.started,'op':'stop'});assert o.state['ok'] and not o.enabled and o.state['phase']=='idle' and o.state['stop_count']==2;checks+=1

# A camera-timeout fault does not block the next explicit enable once the phone feed is fresh again.
t=[0.];feed={'received_at':0,'seq':1}
b=Bus();o=HardwareOwner([b],cal,telemetry,clock=lambda:t[0],wall=lambda:t[0],position_scope=names,paddle_profile=True,camera_metadata=lambda:feed);o.inspect();o.enable(names,True)
t[0]=11;o.poll();assert o.state['camera_pause_active']
t[0]=31.5
try:o.poll()
except RuntimeError as exc:assert 'stale after monitored 20-second hold' in str(exc);o.release_all(str(exc))
else:raise AssertionError('Camera timeout ignored')
assert not o.enabled and o.state['phase']=='idle' and o.state['last_stop']['reason'].startswith('Pickup phone feed stale')
try:o.enable(names,True)
except ValueError as exc:assert 'stale' in str(exc)
else:raise AssertionError('Stale feed enable accepted')
feed.update(received_at=t[0],seq=2);o.enable(names,True);assert o.enabled==set(names);checks+=1

# 3. Owner loop (as in main) plus file client.
def serve(o,folder,halt):
 last=None
 while not halt.is_set():
  try:o.poll()
  except RuntimeError as e:o.state['failed_command_id']=o.current_command;o.release_all(str(e))
  p=folder/'command.json'
  if p.exists():
   c=json.loads(p.read_text())
   if c.get('id')!=last:
    last=c.get('id')
    try:o.command(c)
    except ValueError as e:o.state['last_rejected']={'id':last,'reason':str(e)}
  atomic(folder/'status.json',o.state);time.sleep(.02)
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);o,b=make();atomic(folder/'status.json',o.state);halt=threading.Event();thread=threading.Thread(target=serve,args=(o,folder,halt),daemon=True);thread.start()
 try:
  c=DirectJointClient(folder,{n:{'range_min':100,'range_max':4000} for n in names});time.sleep(.1)
  assert c.set_motor_enable(names,True)['completed']
  # Client stop() confirms the release without any 'stopped' phase.
  result=c.stop();assert result['release_confirmed'] and result['owner_phase']=='idle' and result['stop_latched'] is False and result['owner_restart_required'] is False;checks+=1
  assert c.readiness()['motion_ready'];assert c.set_motor_enable(names,True)['completed']
  result=c.execute({names[1]:1800},2);assert result['completed'] and result['readbacks'][names[1]]==1800;checks+=1
  # Fault mid-move: owner releases everything; client reports it promptly instead of waiting out the 90 s deadline.
  def fault():time.sleep(.5);b.r[names[0]]['Status']=8
  threading.Thread(target=fault,daemon=True).start();t0=time.time()
  try:c.execute({names[1]:1600},4)
  except RuntimeError as exc:assert str(exc).startswith('Owner stopped: right_arm_shoulder_pan: Status=8'),exc
  else:raise AssertionError('Mid-move fault not reported')
  assert time.time()-t0<5 and not o.enabled and all(b.r[n]['Torque_Enable']==0 for n in b.motors) and o.state['phase']=='idle' and 'Status=8' in o.state['root_failure'];checks+=1
  # Motors stay released until an explicit enable; once the fault clears the same owner session moves again.
  time.sleep(.2);assert not o.enabled;b.r[names[0]]['Status']=0;time.sleep(.1)
  started=o.started;assert c.set_motor_enable(names,True)['completed'] and c.execute({names[1]:1700},2)['completed'] and o.started==started;checks+=1
  # A STOP written but not yet read by the owner is never overwritten by a new command.
  halt.set();thread.join(1);atomic(folder/'status.json',dict(o.state,time=time.time()));(folder/'command.json').write_text(json.dumps({'id':time.time_ns(),'op':'stop','session_started':o.started}))
  try:c.execute({names[1]:1600},2)
  except RuntimeError as exc:assert 'STOP pending' in str(exc)
  else:raise AssertionError('Pending STOP overwritten')
  assert json.loads((folder/'command.json').read_text())['op']=='stop';checks+=1
 finally:halt.set();thread.join(1)

# 4. A long move runs as one continuous owner path (a STOP inside it is handled by the owner); a STOP after the
# arm part must still cancel the gripper closure that would follow (execute_targets run against a fake client).
source=Path(__file__).with_name('gemma_robot_tools.py').read_text()
function=next(node for node in ast.parse(source).body if isinstance(node,ast.FunctionDef) and node.name=='execute_targets')
grip=names[5]
class Client:
 cancel_generation=0;calls=[]
 def status(self):return {'execution_profile':'paddle-success-v1','rows':{names[1]:{'Present_Position':2697},grip:{'Present_Position':1592}}}
 def execute_path(self,path,duration,wait=True,replace=False):self.calls.append(('path',path));self.cancel_generation+=1;return {'completed':True,'closure_outcome':'endpoint_settled'}
 def execute(self,segment,duration,wait=True,replace=False):self.calls.append(('move',segment));return {'completed':True,'closure_outcome':'endpoint_settled','readbacks':segment}
space={'DIRECT_CLIENT':Client(),'paddle_target_segments':paddle_target_segments};exec(compile(ast.Module([function],[]),'gemma_robot_tools.py','exec'),space)
try:space['execute_targets']({names[1]:2000,grip:1400},6)
except RuntimeError as exc:assert 'STOP cancelled the remaining gripper closure' in str(exc)
else:raise AssertionError('Gripper closure continued after STOP')
calls=space['DIRECT_CLIENT'].calls;assert len(calls)==1 and calls[0][0]=='path' and len(calls[0][1])==3;checks+=1
print({'no_stop_latch_checks':checks,'hardware_access':False})
