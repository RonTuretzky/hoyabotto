from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner
names=['right_arm_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]
class Bus:
 def __init__(self):
  self.motors=dict.fromkeys(names);self.log=[];self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=826,Max_Position_Limit=3268,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in names}
  self.fail=False
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):
  if self.fail:raise RuntimeError('no status packet')
  self.r[n][f]=v;self.log.append((n,f,v))
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0;self.log.append((n,'Torque_Enable',0))
t=[0.];sleeps=[]
tel=lambda bus,n:dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=0,Present_Voltage=124,Moving=0,Present_Velocity=0,Status=0)
def owner():
 b=Bus();o=HardwareOwner([b],{n:C(range_min=826,range_max=3268,homing_offset=0) for n in names},tel,clock=lambda:t[0],wall=lambda:t[0],position_scope=names,paddle_profile=True,camera_metadata=lambda:{'received_at':t[0],'seq':1},soft_release_s=2.0,sleep=sleeps.append)
 o.inspect();o.enable(names,True);return o,b
lift=names[1]
# STOP: hold present position, torque limit stepped down to 0 over ~2 s, then torque off and settings restored.
o,b=owner();b.r[lift]['Present_Position']=2100;b.log.clear()
o.command({'id':1,'session_started':o.started,'op':'stop'})
limits=[v for n,f,v in b.log if n==lift and f=='Torque_Limit']
assert ('right_arm_shoulder_lift','Goal_Position',2100) in b.log and limits[:10]==[720,640,560,480,400,320,240,160,80,0],limits
assert abs(sum(sleeps)-2.0)<1e-9 and len(sleeps)==10
order=[f for n,f,v in b.log if n==lift];assert order.index('Torque_Enable')>order.index('Goal_Position') and limits[9]==0
assert all(b.r[n]['Torque_Enable']==0 for n in names) and b.r[lift]['Torque_Limit']==1000 and o.state['last_release_mode']=='soft' and not o.enabled
# The status file is written at every ramp step, so a client waiting on a move sees fresh status (not "telemetry stale")
# for the whole 2 s and then the real stop reason.
o,b=owner();writes=[];o.writer=lambda:writes.append((o.state['time'],o.state.get('releasing')))
def tick(d):t[0]+=d
o.sleep=lambda d:(sleeps.append(d),tick(d))
o.release_all('Pickup closure did not become stationary')
assert len(writes)==10 and all(r for _,r in writes) and writes[-1][0]>writes[0][0] and o.state['releasing'] is False
assert o.state['last_stop']['reason']=='Pickup closure did not become stationary'
# Communication failure: immediate release, no easing.
o,b=owner();sleeps.clear();o.release_all('Coherent servo read communication failure: -6');assert sleeps==[] and o.state['last_release_mode']=='immediate'
# If easing itself fails (writes rejected), it falls back to the immediate release path.
o,b=owner();sleeps.clear();b.fail=True;o.release_all('Operator STOP')
assert o.state['last_release_mode'].startswith('immediate (soft release failed') and all(b.r[n]['Torque_Enable']==0 for n in names)
print('Soft release: hold in place, 10-step torque ramp over 2 s, torque off + restore, comm faults immediate, fallback on write failure passed; no hardware')
