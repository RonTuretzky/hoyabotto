"""--head scope (2026-10-09, OAK on the head): head motors enable/release like joints, move only through head_move
(robot_move_head) with their own limits, never join an arm's six-joint rule; the arm guards still apply. No hardware."""
import json,tempfile,time
from pathlib import Path
from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner
from gemma_direct_client import DirectJointClient,atomic_json
from head_joint_executor import HeadJointExecutor,check_head_request
J_=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
R=['right_arm_'+j for j in J_];L=['left_arm_'+j for j in J_];H=['head_motor_1','head_motor_2'];W=['base_left_wheel','base_right_wheel']
RANGE={**{n:(826,3268) for n in R+L},'head_motor_1':(1019,3151),'head_motor_2':(1932,2665)}
START={'head_motor_1':2058,'head_motor_2':2139}
class Bus:
 def __init__(self,mismatch=(),names=R+L+H):
  self.motors=dict.fromkeys(names)
  self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=RANGE[n][0],Max_Position_Limit=RANGE[n][1],Present_Position=START.get(n,2000),Goal_Position=START.get(n,2000),Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0,Load=0) for n in names}
  for n in mismatch:self.r[n]['Homing_Offset']=677
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
t=[0.]
tel=lambda bus,n:dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=bus.r[n]['Load'],Present_Voltage=124,Moving=0,Present_Velocity=0,Status=0)
def owner(head=True,mismatch=(),names=R+L+H):
 t[0]=0.;b=Bus(mismatch,names);o=HardwareOwner([b],{n:C(range_min=RANGE[n][0],range_max=RANGE[n][1],homing_offset=0) for n in names},tel,clock=lambda:t[0],wall=lambda:t[0],position_scope=R+L,paddle_profile=True,camera_metadata=lambda:{'received_at':t[0],'seq':1},head=head);o.inspect();return o,b
def run(o,b,follow=True,limit=120):
 for _ in range(limit):
  t[0]=round(t[0]+.1,6)
  if follow:
   for n in b.motors:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
  o.poll()
  if not(o.engine and o.engine.active):return
 raise AssertionError('motion did not finish')
def refused(fn,text):
 try:fn()
 except (ValueError,RuntimeError) as x:assert text in str(x),(text,str(x));return str(x)
 raise AssertionError('accepted: expected refusal containing '+text)
cid=[0]
def cmd(o,**c):cid[0]+=1;o.command({'id':cid[0],'session_started':o.started,**c});return cid[0]

# Scope: head commandable, reported, not part of the arms' required set.
o,b=owner()
assert set(H)<=o.commandable_names and o.state['head_supported'] and o.state['head_motors']==H and 'head_move' in o.state['capabilities']
assert set(o.state['pickup_required_enabled_motors'])==set(R+L) and o.state['head_move_limits']['max_ticks_per_move']==200
assert o.state['ranges']['head_motor_1']==[1019,3151] and 'head_motor_1' not in o.state['read_only_motors']
# Enable the head alone (no arm): held where it is, head torque limit 500, 120 s idle lease like the arms.
o.enable(H,True);assert o.enabled==set(H) and o.goals=={'head_motor_1':2058,'head_motor_2':2139} and b.r['head_motor_1']['Torque_Limit']==500 and o.lease==120
# Head names never go through direct_joint (arm targets).
refused(lambda:cmd(o,op='direct_joint',positions={'head_motor_1':2100},duration_s=2),'only through robot_move_head')
# Pan +100 in 1 s and back; tilt +100 and back; no arm enabled, none required.
k=cmd(o,op='head_move',positions={'head_motor_1':2158},duration_s=1);run(o,b)
assert o.state['completed']==k and o.state['closure_outcome']=='endpoint_settled' and b.r['head_motor_1']['Present_Position']==2158 and o.lease>=t[0]+119
k=cmd(o,op='head_move',positions={'head_motor_1':2058},duration_s=1);run(o,b);assert b.r['head_motor_1']['Present_Position']==2058
k=cmd(o,op='head_move',positions={'head_motor_2':2239,'head_motor_1':2100},duration_s=1);run(o,b);assert b.r['head_motor_2']['Present_Position']==2239 and b.r['head_motor_1']['Present_Position']==2100
# Limits: 200 ticks, 1 s per 100 ticks, 40-tick margin, head names only, enabled first.
refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2301},duration_s=3),'at most 200 per move')
refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2250},duration_s=1.4),'at least 1 s per 100 ticks')
refused(lambda:HeadJointExecutor(['head_motor_2'],{'head_motor_2':[1932,2665]},lambda _:None).start({'id':1,'session_started':0,'positions':{'head_motor_2':2640},'duration_s':3},{'head_motor_2':2600},0),'40 ticks inside the saved range')
refused(lambda:cmd(o,op='head_move',positions={'right_arm_shoulder_lift':2100},duration_s=3),'head motors only')
refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2101},duration_s=3),'travel 3..341')
refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2150},waypoints=[{'head_motor_1':2150}],duration_s=3),'positions, not waypoints')
o.enable(['head_motor_2'],False);assert o.enabled=={'head_motor_1'}
refused(lambda:cmd(o,op='head_move',positions={'head_motor_2':2200},duration_s=3),'explicitly enabled: head_motor_2')
# An arm move needs that arm's six joints, never the head; the head stays enabled and held meanwhile.
o.enable(R,True);k=cmd(o,op='direct_joint',positions={R[1]:2200},duration_s=2);run(o,b);assert o.state['completed']==k and o.enabled==set(R)|{'head_motor_1'}
# No head move while an arm motion runs.
cmd(o,op='direct_joint',positions={R[1]:2000},duration_s=2);refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2000},duration_s=1),'Previous motion has not completed');run(o,b)
# Following-error guard: a head that does not follow its goal is released like an arm joint.
k=cmd(o,op='head_move',positions={'head_motor_1':2250},duration_s=2)
refused(lambda:run(o,b,follow=False),'following error exceeds96ticks: head_motor_1')
o.release_all('test fault');assert not o.enabled and b.r['head_motor_1']['Torque_Enable']==0 and b.r['head_motor_1']['Torque_Limit']==1000
# Contact guard: loaded >=350, stalled, lagging >=50 -> contact_halt, holding (nothing released).
o.enable(H,True);k=cmd(o,op='head_move',positions={'head_motor_2':2339},duration_s=2)
for i in range(30):
 t[0]=round(t[0]+.1,6);b.r['head_motor_2']['Present_Position']=2239;b.r['head_motor_2']['Load']=400
 if abs(b.r['head_motor_2']['Goal_Position']-2239)>=90:break
 o.poll()
 if not o.engine.active:break
while o.engine.active:t[0]=round(t[0]+.1,6);o.poll()
assert o.state['closure_outcome']=='contact_halt' and 'head_motor_2' in o.enabled,o.state.get('closure_outcome')
b.r['head_motor_2']['Load']=0;o.release_all('Operator STOP')
# Load fault level for the head is 500 (arm joints 800).
o.enable(H,True);b.r['head_motor_1']['Load']=600;t[0]+=.1
refused(o.poll,'Present_Load=600');o.release_all('test');b.r['head_motor_1']['Load']=0
# Without --head: read-only as before.
o,b=owner(head=False);assert not o.state['head_supported'] and 'head_motor_1' in o.state['read_only_motors']
refused(lambda:o.enable(['head_motor_1'],True),'UNSUPPORTED_OWNER_SCOPE')
refused(lambda:cmd(o,op='head_move',positions={'head_motor_1':2100},duration_s=1),'without --head')
# Head calibration mismatch: the head alone goes back to read-only; the arms still start and work.
o,b=owner(mismatch=['head_motor_2'])
assert not o.state['head_supported'] and o.state['scope_reduced']['head']['motors']==['head_motor_2'] and not set(H)&o.commandable_names and o.commandable_names==set(R+L)
o.enable(R,True)
# Head motors absent from the answering buses: head_supported false, owner still starts.
o,b=owner(names=R+L);assert not o.state['head_supported'] and o.state['head_motors']==[]
# --head without the pickup profile is refused.
refused(lambda:HardwareOwner([Bus()],{},tel,position_scope=None,head=True),'requires the pickup profile')
# Pure executor rule check.
assert check_head_request({'positions':{'head_motor_1':2200},'duration_s':2},{'head_motor_1':2000})=={'head_motor_1':200}

# Client: validation messages and dispatch of head_move; arm/head separation.
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);cal={n:{'range_min':RANGE[n][0],'range_max':RANGE[n][1]} for n in R+L+H}
 rows={n:{'Torque_Enable':1 if n in H else 0,'Status':0,'Present_Position':START.get(n,2000),'Present_Temperature':30,'Present_Load':0,'Operating_Mode':0} for n in R+L+H+W}
 state={'hardware_server':True,'execution_profile':'paddle-success-v1','control_mode':'direct_joint','supportsselectedjoints':R+L+H,'supported_motors':R+L+H+W,'commandable_motors':R+L+H,
        'ranges':{n:list(RANGE[n]) for n in R+L+H},'operator_armed':True,'started':7,'phase':'holding','ok':True,'enabled_motors':H,'goals':dict(START),'lease_remaining':100,'rows':rows,
        'head_supported':True,'head_motors':H,'head_move_limits':{'max_ticks_per_move':200}}
 def save():state['time']=time.time();atomic_json(folder/'status.json',state)
 save();c=DirectJointClient(folder,cal)
 assert c.readiness()['head_supported'] is True and 'head_motor_1' not in c.readiness()['joint_blockers']
 refused(lambda:c._validate({'op':'direct_joint','positions':{'head_motor_1':2100},'duration_s':2},state),'only through robot_move_head')
 refused(lambda:c._validate({'op':'head_move','positions':{'head_motor_1':2300},'duration_s':3},state),'at most 200 per move')
 refused(lambda:c._validate({'op':'head_move','positions':{'head_motor_2':2630},'duration_s':3},state),'40-tick margin')
 refused(lambda:c._validate({'op':'head_move','positions':{'head_motor_1':2158},'duration_s':.5},state),'at least 1 s per 100 ticks')
 refused(lambda:c._validate({'op':'head_move','positions':{'right_arm_gripper':2100},'duration_s':3},state),'head motors only')
 c._validate({'op':'head_move','positions':{'head_motor_1':2158},'duration_s':1},state)   # no arm enabled: fine
 c._validate({'op':'enable_motors','names':H,'enabled':True},state)
 msg=refused(lambda:c._validate({'op':'enable_motors','names':['base_left_wheel'],'enabled':True},state),'robot_move_base')
 assert 'head cannot be moved' not in msg
 import threading
 def owner_thread():
  for _ in range(300):
   p=folder/'command.json'
   if p.exists():
    command=json.loads(p.read_text())
    if command['op']=='head_move':
     rows['head_motor_1']['Present_Position']=2160;state.update(completed=command['id'],closure_outcome='endpoint_settled',endpoint_reached=True,phase='holding');save();return
   time.sleep(.01)
 th=threading.Thread(target=owner_thread,daemon=True);th.start()
 r=c.move_head({'head_motor_1':2158},1);th.join(2)
 assert r['completed'] and r['readbacks']=={'head_motor_1':2160} and json.loads((folder/'command.json').read_text())['op']=='head_move'
 state.update(head_supported=False,commandable_motors=R+L,supportsselectedjoints=R+L);save()
 msg=refused(lambda:c._validate({'op':'enable_motors','names':H,'enabled':True},state),'started without --head')
 refused(lambda:c._validate({'op':'head_move','positions':{'head_motor_1':2158},'duration_s':1},state),'without --head')
print('Head scope: enable/release, robot_move_head limits (200 ticks, 1 s/100 ticks, 40-tick margin), arm guards (following error, contact halt, load 500), no six-joint coupling, read-only without --head or on mismatch; client validation and dispatch passed; no hardware')
