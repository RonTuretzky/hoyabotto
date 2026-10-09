"""Stream mode (owner --stream): stream_targets / hold_here for a 10 Hz policy client. Fake bus, virtual clock; no hardware."""
import json,os,sys,tempfile,threading,time,types
from pathlib import Path
from types import SimpleNamespace as C
from unittest import mock
from gemma_hardware_owner import HardwareOwner,atomic
from paddle_joint_executor import PaddleJointExecutor
J_=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
R=['right_arm_'+j for j in J_];L=['left_arm_'+j for j in J_];ALL=R+L
PAN,LIFT,ELBOW,JAW=R[0],R[1],R[2],R[5]
load={};vel={}
class Bus:
 def __init__(self):
  self.motors=dict.fromkeys(ALL);self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=826,Max_Position_Limit=3268,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in ALL}
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
t=[0.]
tel=lambda bus,n:dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=load.get(n,0),Present_Voltage=124,Moving=0,Present_Velocity=vel.get(n,0),Status=bus.r[n]['Status'])
def owner(stream=True,clock=None,wall=None):
 load.clear();vel.clear();b=Bus();clock=clock or (lambda:t[0]);wall=wall or (lambda:t[0])
 o=HardwareOwner([b],{n:C(range_min=826,range_max=3268,homing_offset=0) for n in ALL},tel,clock=clock,wall=wall,position_scope=ALL,paddle_profile=True,camera_metadata=lambda:{'received_at':wall(),'seq':1},stream=stream)
 o.inspect();return o,b
ids=[0]
def send(o,op,**kw):
 ids[0]+=1;o.command({'id':ids[0],'session_started':o.started,'op':op,**kw});return ids[0]
def stream(o,targets,**kw):return send(o,'stream_targets',targets=targets,**kw)
def poll(o,b,plant=None,k=1,each=None):
 # One owner loop: the servos move (plant), the owner polls, then (as main() does) reads the next command.
 for _ in range(k):
  t[0]=round(t[0]+.1,6)
  for n in ALL:
   if b.r[n]['Torque_Enable']:b.r[n]['Present_Position']=plant(n,b.r[n]['Goal_Position'],b.r[n]['Present_Position']) if plant else b.r[n]['Goal_Position']
  o.poll()
  if each:each()
def goal(b,n):return b.r[n]['Goal_Position']
def rejected(fn,text):
 try:fn()
 except ValueError as x:assert text in str(x),str(x);return str(x)
 raise AssertionError('accepted: '+text)

# 1. Off by default: no 'stream' capability; both ops are rejected exactly like an unknown op; stream needs the profile.
o,b=owner(stream=False);o.enable(ALL,True)
assert 'stream' not in o.state['capabilities'] and 'stream_limits' not in o.state and o.stream_executor is None
unknown=rejected(lambda:send(o,'bogus'),'Unsupported hardware command')
assert rejected(lambda:stream(o,{LIFT:2050}),'Unsupported')==unknown and rejected(lambda:send(o,'hold_here'),'Unsupported')==unknown
assert all(goal(b,n)==2000 for n in ALL) and o.engine is None and o.state['phase']=='holding'
try:HardwareOwner([Bus()],{n:C(range_min=826,range_max=3268,homing_offset=0) for n in ALL},tel,clock=lambda:t[0],wall=lambda:t[0],position_scope=ALL,stream=True)
except ValueError as x:assert 'pickup profile' in str(x)
else:raise AssertionError('stream without the pickup profile')

# 2. Both arms in one stream; each goal moves at most 40 ticks per owner loop.
o,b=owner();o.enable(ALL,True);assert 'stream' in o.state['capabilities'] and o.state['stream_limits']['step_ticks_per_loop']==40
stream(o,{LIFT:2090,L[1]:1920,L[5]:2006})
assert o.state['phase']=='moving' and o.state['stream_phase']=='streaming' and set(o.engine.joints)==set(ALL)
history=[];poll(o,b,k=3,each=lambda:history.append((goal(b,LIFT),goal(b,L[1]),goal(b,L[5]))))
assert [h[0] for h in history]==[2040,2080,2090] and [h[1] for h in history]==[1960,1920,1920] and history[0][2]==2006,history
assert all(abs(a-c)<=40 for x,y in zip([(2000,2000,2000)]+history,history) for a,c in zip(x,y))

# 3. Out-of-range or out-of-envelope targets are rejected whole (not clamped); nothing changes.
before=dict(o.goals);ex=o.engine
rejected(lambda:stream(o,{LIFT:2100,PAN:865}),'rejected, not clamped')
rejected(lambda:stream(o,{L[2]:3229}),'commandable range')
rejected(lambda:stream(o,{LIFT:2090+97}),'within 96')
assert o.goals==before and o.engine is ex and ex.target[LIFT]==2090 and o.state['last_stop'] is None

# 4. A joint within 2 ticks of its present position or held goal is skipped; all-skipped is an accepted no-op.
stream(o,{LIFT:2091,ELBOW:2050});r=o.state['stream_result']
assert r['skipped_joints']==[LIFT] and not r['no_op'] and ex.target[LIFT]==2090 and ex.target[ELBOW]==2050
stream(o,{LIFT:2089,ELBOW:2001});r=o.state['stream_result'];assert r['no_op'] and r['skipped_joints']==[ELBOW,LIFT] and r['accepted']
o,b=owner();o.enable(ALL,True)
stream(o,{LIFT:2001,L[1]:2000});r=o.state['stream_result']
assert r['no_op'] and r['accepted'] and not r['started'] and o.engine is None and o.state['phase']=='holding' and o.state['stream_phase']=='holding' and o.state['stream_ack']==ids[0]

# 5. Jaw closure streams at 10 Hz in <=10-tick goal steps; no 'did not become stationary' fault.
o,b=owner();o.enable(ALL,True);jaw=[]
def close():stream(o,{JAW:1900});jaw.append(goal(b,JAW))
stream(o,{JAW:1900});assert o.state['stream_result']['jaw_limited']=={JAW:1990}
poll(o,b,k=14,each=close)
steps=[2000]+jaw;assert all(0<=a-c<=10 for a,c in zip(steps,steps[1:])) and jaw[-1]==1900 and o.engine.active and o.state['stop_count']==0,jaw

# 6. Jaw guard: a closing jaw at load 250 freezes at its present position, then ignores closing until opened.
o,b=owner();o.enable(ALL,True)
lagging=lambda n,g,q:g+5 if n==JAW and not o.engine.jaw_blocked else g  # the jaw trails its goal by 5 while closing
def close_loaded():
 if goal(b,JAW)<=1960:load[JAW]=250
 stream(o,{JAW:1900})
stream(o,{JAW:1900});poll(o,b,plant=lagging,k=6,each=close_loaded)
frozen=goal(b,JAW);assert frozen==b.r[JAW]['Present_Position'] and frozen>1900 and o.engine.jaw_blocked=={JAW:frozen}
assert o.state['jaw_contact']=={JAW:{'load':250,'position':frozen}},o.state['jaw_contact']
poll(o,b,k=3,each=lambda:stream(o,{JAW:1880}))
assert goal(b,JAW)==frozen and JAW in o.state['stream_result']['jaw_ignored_closing'] and JAW in o.state['stream_result']['skipped_joints']
load.clear();stream(o,{JAW:frozen+50});poll(o,b)
assert goal(b,JAW)==frozen+10 and o.state['jaw_contact']=={} and JAW not in o.engine.jaw_blocked
poll(o,b,each=lambda:stream(o,{JAW:frozen}));poll(o,b);assert goal(b,JAW)==frozen  # closing allowed again after the opening (once the jaw has left frozen)
# Lag variant: the jaws stop on an object at 1975 while the goal keeps closing; frozen once 40 ticks behind.
o,b=owner();o.enable(ALL,True);stream(o,{JAW:1900})
poll(o,b,plant=lambda n,g,q:max(g,1975) if n==JAW else g,k=8,each=lambda:stream(o,{JAW:1900}))
assert goal(b,JAW)==1975 and o.state['jaw_contact'][JAW]['position']==1975 and o.state['stop_count']==0

# 7. Arm contact counted across stream commands: holds every streamed joint where it is, releases nothing.
o,b=owner();o.enable(ALL,True)
def blocked(n,g,q):
 if n!=LIFT:return g
 q=max(g,1985);load[LIFT]=400 if q-g>=10 else 0;return q
stream(o,{LIFT:1920,ELBOW:2040});ex=o.engine
poll(o,b,plant=blocked,k=6,each=lambda:o.engine is ex and ex.active and stream(o,{LIFT:1920,ELBOW:2040}))
assert o.stream_executor is ex and o.state['closure_outcome']=='contact_halt' and o.state['stream_phase']=='contact_halt' and o.state['phase']=='holding',o.state.get('closure_outcome')
assert o.state['stream_contact_joints']==[LIFT] and o.state['contact'][LIFT]['load']==400
assert o.goals[LIFT]==1985 and o.goals[ELBOW]==b.r[ELBOW]['Present_Position'] and all(o.goals[n]==b.r[n]['Present_Position'] for n in ALL)
assert o.enabled==set(ALL) and all(b.r[n]['Torque_Enable']==1 for n in ALL) and o.state['stop_count']==0
load.clear();rejected(lambda:stream(o,{LIFT:1960}),'contact_halt')
send(o,'hold_here');stream(o,{LIFT:1960});assert o.engine.active and o.state['stream_phase']=='streaming'

# 8. Stream timeout: no targets for 0.5 s -> holding the current goals; nothing released.
o,b=owner();o.enable(ALL,True);stream(o,{LIFT:2060});poll(o,b,k=5)
assert o.engine.active and o.state['stream_phase']=='streaming'
poll(o,b);assert not o.engine.active and o.state['stream_phase']=='holding' and o.state['closure_outcome']=='stream_timeout' and o.state['phase']=='holding'
assert o.goals[LIFT]==2060 and o.enabled==set(ALL) and o.state['stop_count']==0 and o.lease>t[0]+100
poll(o,b,k=5);assert o.state['phase']=='holding' and o.state['stop_count']==0
stream(o,{LIFT:2100});assert o.engine.active  # a later command starts a new stream

# 9. hold_here: goals to present (clamped 4 inside the range) with no torque ramp and no release; ends the stream.
o,b=owner();o.enable(ALL,True);limits={n:b.r[n]['Torque_Limit'] for n in ALL}
stream(o,{LIFT:2080});poll(o,b,plant=lambda n,g,q:g-25 if n==LIFT else g);assert goal(b,LIFT)==2040  # the arm trails its goal
b.r[PAN]['Present_Position']=828;writes=o.state['motor_writes']
send(o,'hold_here')
assert not o.engine.active and o.state['closure_outcome']=='halted' and o.state['phase']=='holding' and o.state['stream_phase']=='holding'
assert o.goals[LIFT]==1975 and goal(b,LIFT)==1975 and o.goals[PAN]==830 and o.state['hold_here']['goals'][PAN]==830
assert all(o.goals[n]==b.r[n]['Present_Position'] for n in ALL if n!=PAN) and o.state['motor_writes']==writes+2
assert {n:b.r[n]['Torque_Limit'] for n in ALL}==limits and o.enabled==set(ALL) and o.state['stop_count']==0

# 10. Replace semantics: a stream cannot replace a running normal motion; a normal motion ends a running stream.
o,b=owner();o.enable(ALL,True)
send(o,'direct_joint',positions={LIFT:2200},duration_s=2);assert isinstance(o.engine,PaddleJointExecutor) and o.engine.active
rejected(lambda:stream(o,{ELBOW:2050}),'halt it')
send(o,'halt');sid=stream(o,{ELBOW:2050});poll(o,b)
mid=send(o,'direct_joint',positions={LIFT:1800},duration_s=2)
assert o.state['replaced_command_id']==sid and o.stream_executor.active is False and isinstance(o.engine,PaddleJointExecutor) and o.engine.command_id==mid
o,b=owner();o.enable(ALL,True);stream(o,{ELBOW:2050});poll(o,b);send(o,'halt')
assert o.state['closure_outcome']=='halted' and o.state['phase']=='holding' and o.goals[ELBOW]==2040

# 11. Faults keep their meaning in stream mode: 96-tick following error, load > 800 (arm) / > 500 (jaw) release everything.
for name,setup,text in [('envelope',lambda b:b.r[LIFT].update(Present_Position=goal(b,LIFT)+97),'exceeds 96'),
                        ('arm load',lambda b:load.update({ELBOW:801}),'Present_Load=801'),
                        ('jaw load',lambda b:load.update({JAW:501}),'Present_Load=501')]:
 o,b=owner();o.enable(ALL,True);stream(o,{LIFT:2040,JAW:1990});poll(o,b);setup(b);t[0]=round(t[0]+.1,6)
 try:o.poll()
 except RuntimeError as e:assert text in str(e),(name,str(e));o.release_all(str(e))
 else:raise AssertionError(name+' fault not raised')
 assert not o.enabled and all(b.r[n]['Torque_Enable']==0 for n in ALL) and o.state['stop_count']==1 and not o.stream_executor.active and o.state['stream_phase']=='released',name
 rejected(lambda:stream(o,{LIFT:2040}),'explicitly enabled')
# The holding drift fault stays at 96 ticks after a stream ends.
o,b=owner();o.enable(ALL,True);stream(o,{LIFT:2040});poll(o,b,k=7);assert not o.engine.active
b.r[LIFT]['Present_Position']=2040+97;t[0]+=.1
try:o.poll()
except RuntimeError as e:assert 'holding drift' in str(e)
else:raise AssertionError('holding drift not raised')

# 12. DirectJointClient against a real owner loop (main()-style, wall time): refusals return accepted False, never STOP.
from gemma_direct_client import DirectJointClient
def run_loop(o,b,folder,stop):
 last=None
 while not stop.is_set():
  for n in ALL:
   if b.r[n]['Torque_Enable']:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
  try:o.poll()
  except RuntimeError as e:o.release_all(str(e))
  p=folder/'command.json'
  if p.exists():
   c=json.loads(p.read_text())
   if c.get('id')!=last:
    last=c.get('id')
    try:o.command(c)
    except ValueError as e:o.state['last_rejected']={'id':last,'reason':str(e)}
  atomic(folder/'status.json',o.state);time.sleep(.02)
cal={n:{'range_min':826,'range_max':3268} for n in ALL}
for streaming in (True,False):
 with tempfile.TemporaryDirectory() as tmp:
  folder=Path(tmp);o,b=owner(stream=streaming,clock=time.monotonic,wall=time.time);o.enable(ALL,True);atomic(folder/'status.json',o.state)
  stop=threading.Event();th=threading.Thread(target=run_loop,args=(o,b,folder,stop),daemon=True);th.start()
  client=DirectJointClient(folder,cal)
  try:
   if not streaming:
    r=client.stream({LIFT:2040});assert r['accepted'] is False and 'STREAM_MODE_DISABLED' in r['reason'] and not (folder/'command.json').exists(),r
    r=client.hold_here();assert r['accepted'] is False and 'STREAM_MODE_DISABLED' in r['reason'] and not (folder/'command.json').exists(),r
    continue
   r=client.stream({LIFT:2040,L[5]:1950},command_id='step-1')
   assert r['accepted'] and r['phase']=='streaming' and r['jaw_limited']=={L[5]:1990} and r['client_command_id']=='step-1' and r['stop_count']==0,r
   r=client.stream({LIFT:2041,ELBOW:2020});assert r['accepted'] and r['skipped_joints']==[LIFT],r
   r=client.stream({LIFT:865});assert r['accepted'] is False and r['rejected'] and 'not clamped' in r['reason'],r
   assert json.loads((folder/'command.json').read_text())['op']=='stream_targets' and o.enabled==set(ALL) and o.state['stop_count']==0
   r=client.stream({'head_motor_1':2000});assert r['accepted'] is False and 'arm motors' in r['reason'],r
   r=client.stream({LIFT:'2040'});assert r['accepted'] is False,r
   r=client.hold_here();assert r['accepted'] and r['completed'] and r['phase']=='holding' and set(r['goals'])==set(ALL),r
   assert o.enabled==set(ALL) and o.state['stop_count']==0
   # An owner fault stays visible (stop_count); the client still sends no STOP of its own.
   r=client.stream({LIFT:2080});assert r['accepted']
   load[ELBOW]=801  # the next owner poll faults and releases everything
   r=client.stream({LIFT:2100});assert r['accepted'] is False and (r.get('owner_stopped') or 'explicitly enabled' in r['reason']),r
   time.sleep(.1);assert o.state['stop_count']==1 and not o.enabled and client.motion()['last_stop']['reason'].startswith(ELBOW+': Present_Load=801')
   assert json.loads((folder/'command.json').read_text())['op']=='stream_targets'
  finally:stop.set();th.join(2)
  # Stale status (owner loop gone): refused before anything is written, no STOP.
  before=(folder/'command.json').read_text();time.sleep(1.1)
  r=client.stream({LIFT:2040});assert r['accepted'] is False and r['reason']=='OWNER_STATUS_STALE' and (folder/'command.json').read_text()==before,r
  r=client.hold_here();assert r['accepted'] is False and (folder/'command.json').read_text()==before,r

# 13. API tools: both stream tools exist, are motion tools, are hidden from the pilot, and refuse without 'stream'.
BRIDGE=Path(__file__).resolve().parent
NAMES=ALL+['head_motor_1','head_motor_2','base_left_wheel','base_right_wheel']
with tempfile.TemporaryDirectory() as tmp:
 tmp=Path(tmp);cal_path=tmp/'farm_xlerobot.json';cal_path.write_text(json.dumps({n:{'id':i,'drive_mode':0,'homing_offset':0,'range_min':826,'range_max':3268} for i,n in enumerate(NAMES)}))
 stubs={'carton_preflight':dict(run=lambda:{'error':'no serial'}),'gemma_execution_binding':dict(TrustedExecutionBinding=lambda s:C(bind=None)),
        'gemma_reach_planner':dict(POSE_SCHEMA={'type':'array'},validate_poses=lambda p:None,inspect_or_plan=lambda *a,**k:{}),
        'carton':{},'carton.servo':{},'carton.servo.common':dict(atomic_json=lambda p,v:None)}
 saved={k:sys.modules.get(k) for k in stubs}
 for k,attrs in stubs.items():
  if k=='carton.servo.common' or k not in sys.modules:m=types.ModuleType(k);m.__dict__.update(attrs);sys.modules[k]=m
 original=Path.read_text
 def read_text(self,*a,**k):
  if self.name=='farm_xlerobot.json':return original(cal_path)
  if self.name=='gateway-server.pem':return original(BRIDGE/'gateway-server.pem')
  return original(self,*a,**k)
 environment,path=dict(os.environ),list(sys.path)
 try:
  with mock.patch.object(Path,'read_text',read_text):import gemma_robot_tools as tools
 finally:
  os.environ.clear();os.environ.update(environment);sys.path[:]=path
  for k,v in saved.items():
   if v is None:sys.modules.pop(k,None)
   else:sys.modules[k]=v
 names=[x['function']['name'] for x in tools.TOOLS]
 assert {'robot_stream_joint_targets','robot_hold_here'}<=set(names)<=set(tools.SCHEMAS) and {'robot_stream_joint_targets','robot_hold_here'}<=tools.PILOT_HIDDEN
 schema=tools.SCHEMAS['robot_stream_joint_targets']
 assert schema['required']==['positions'] and set(schema['properties']['positions']['properties'])==set(ALL) and schema['properties']['positions']['properties'][LIFT]=={'type':'integer','minimum':866,'maximum':3228}
 assert tools.SCHEMAS['robot_hold_here']['properties']=={}
 calls=[]
 class Fake:
  caps=['read','direct_joint']
  def status(self):return {'capabilities':self.caps,'execution_profile':'paddle-success-v1'}
  def stream(self,p):calls.append(('stream',p));return {'accepted':True,'skipped_joints':[],'jaw_contact':{},'phase':'streaming','command_id':1,'goals':{}}
  def hold_here(self):calls.append('hold');return {'accepted':True,'phase':'holding'}
 tools.DIRECT_CLIENT=Fake()
 r,_=tools.dispatch('robot_stream_joint_targets',{'positions':{LIFT:2040,L[5]:1500}});assert r['accepted'] is False and 'STREAM_MODE_DISABLED' in r['reason'] and not calls,r
 r,_=tools.dispatch('robot_hold_here',{});assert r['accepted'] is False and not calls
 tools.DIRECT_CLIENT.caps=['read','direct_joint','stream']
 r,_=tools.dispatch('robot_stream_joint_targets',{'positions':{LIFT:2040,L[5]:1500}});assert r=={'accepted':True,'skipped_joints':[],'jaw_contact':{},'phase':'streaming','command_id':1} and calls==[('stream',{LIFT:2040,L[5]:1500})],r
 r,_=tools.dispatch('robot_hold_here',{});assert r['accepted'] and calls[-1]=='hold'
 for bad in ({LIFT:865},{'head_motor_1':2000}):
  try:tools.dispatch('robot_stream_joint_targets',{'positions':bad})
  except ValueError:pass
  else:raise AssertionError('accepted '+str(bad))
 assert len(calls)==2
print('Stream mode: off by default, both-arm 40-tick ramp, range/envelope rejection, 2-tick skip and no-op, 10-tick jaw steps, jaw guard freeze and reopen, '
      'cross-command arm contact hold, 0.5 s timeout hold, hold_here at present, replace rules, 96/800/500 faults release, client refusals without STOP, API tools; no hardware')
