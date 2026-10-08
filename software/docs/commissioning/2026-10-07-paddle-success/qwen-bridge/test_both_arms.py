import json,tempfile
from pathlib import Path
from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner
import calibration_job as J
J_=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
R=['right_arm_'+j for j in J_];L=['left_arm_'+j for j in J_]
class Bus:
 def __init__(self,mismatch=()):
  self.motors=dict.fromkeys(R+L);self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=826,Max_Position_Limit=3268,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in R+L}
  for n in mismatch:self.r[n]['Homing_Offset']=677
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
t=[0.]
tel=lambda bus,n:dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=0,Present_Voltage=124,Moving=0,Present_Velocity=0,Status=0)
def owner(mismatch=(),scope=R+L):
 b=Bus(mismatch);o=HardwareOwner([b],{n:C(range_min=826,range_max=3268,homing_offset=0) for n in R+L},tel,clock=lambda:t[0],wall=lambda:t[0],position_scope=scope,paddle_profile=True,camera_metadata=lambda:{'received_at':t[0],'seq':1});o.inspect();return o,b
# Both arms calibrated: both movable; a command needs that arm's six joints enabled, not the other arm's.
o,b=owner();assert set(o.commandable_names)==set(R+L)
o.enable(L,True);o.command({'id':1,'session_started':o.started,'op':'direct_joint','positions':{L[1]:2200},'duration_s':2})
for i in range(1,80):
 t[0]=round(t[0]+.1,6)
 for n in R+L:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
 o.poll()
 if not o.engine.active:break
assert o.state['completed']==1 and b.r[L[1]]['Present_Position']==2200
o.enable([R[1]],True) # one right joint only
try:o.command({'id':2,'session_started':o.started,'op':'direct_joint','positions':{R[1]:2200},'duration_s':2})
except ValueError as x:assert 'right-arm motors explicitly enabled' in str(x)
else:raise AssertionError('right arm moved without being enabled')
# Left calibration mismatch: left becomes read-only, right still works, the reduction is reported.
t[0]=0;o,b=owner(mismatch=[L[1],L[3]])
assert set(o.commandable_names)==set(R) and o.state['scope_reduced']['left']['motors']==[L[1],L[3]] and o.state['supportsselectedjoints']==R
try:o.enable(L,True)
except ValueError as x:assert 'read-only' in str(x)
else:raise AssertionError('mismatched arm enabled')
o.enable(R,True);assert o.enabled==set(R)
# Single-arm scope keeps the old rule: a mismatch there refuses startup.
try:owner(mismatch=[R[1]],scope=R)
except RuntimeError as x:assert 'saved calibration differs' in str(x)
else:raise AssertionError('mismatched single-arm scope started')
# Mismatch in every scoped arm also refuses.
try:owner(mismatch=[R[0],L[0]])
except RuntimeError:pass
else:raise AssertionError('all arms mismatched but started')
# Restore job: no camera needed, no runner, restore from the live file, then restart.
with tempfile.TemporaryDirectory() as tmp:
 tmp=Path(tmp);work=tmp/'work';(work/'gemma-hardware-session').mkdir(parents=True)
 (work/'gemma-hardware-session/status.json').write_text(json.dumps({'enabled_motors':[],'rows':{},'phase':'idle'}))
 job=tmp/'job.json';job.write_text(json.dumps({'id':'j','action':'restore','arm':'left','checkout':str(tmp/'c'),'work':str(work),'state':'running'}));calls=[]
 out=J.run(job,stop_owner=lambda w:calls.append('stop') or True,run_runner=lambda *a:calls.append('runner'),restore=lambda s,a,bf:calls.append(('restore',a,bf)) or {'restored':True},restart=lambda c:calls.append('restart') or 0)
 assert out['state']=='succeeded' and out['outcome']=='restored' and calls==['stop',('restore','left',None),'restart'],calls
print('Both arms: per-arm enable requirement, mismatched arm demoted to read-only, single-arm/all-arm mismatch still refuses, no-motion restore job passed; no hardware')
