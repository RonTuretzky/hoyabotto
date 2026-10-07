from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner
names=['right_arm_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']]
class Bus:
 def __init__(self):
  self.motors=dict.fromkeys(names);self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=100,Max_Position_Limit=4000,Present_Position=2000,Goal_Position=2000,Lock=1,Torque_Limit=1000,Goal_Velocity=0,Goal_Time=0,Acceleration=0,P_Coefficient=16,Status=0) for n in names}
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v
 def disable_torque(self,ns,**kw):
  for n in ns:self.r[n]['Torque_Enable']=0
b=Bus();t=[0.];meta={'received_at':0,'seq':1}
def telemetry(bus,n):return dict(Present_Position=bus.r[n]['Present_Position'],Present_Load=0,Present_Voltage=124,Moving=0,Present_Velocity=0,Status=bus.r[n]['Status']) # No temperature needed under profile.
o=HardwareOwner([b],{n:C(range_min=100,range_max=4000,homing_offset=0) for n in names},telemetry,clock=lambda:t[0],wall=lambda:t[0],position_scope=names,paddle_profile=True,camera_metadata=lambda:meta);o.inspect();o.enable(names,True)
assert o.lease==120 and b.r[names[1]]['P_Coefficient']==32 and b.r[names[2]]['Torque_Limit']==400 and b.r[names[-1]]['Torque_Limit']==500 and b.r[names[-1]]['Goal_Velocity']==200
c={'id':1,'session_started':0,'op':'direct_joint','positions':{names[1]:1800},'duration_s':2};o.command(c)
for i in range(1,111):
 t[0]=i*.1
 for n in names:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
 o.poll()
assert o.lease>120 # completion grants 120s, then paused hold extends it.
assert o.state['camera_pause_active'];goals=dict(o.goals)
for i in range(111,121):t[0]=i*.1;o.poll()
assert o.goals==goals
meta.update(received_at=t[0],seq=2);t[0]+=.1;o.poll();assert not o.state['camera_pause_active']
# Simultaneous movement: one command moves two joints together through the owner.
c2={'id':3,'session_started':0,'op':'direct_joint','positions':{names[1]:1900,names[2]:2100},'duration_s':2};o.command(c2);assert o.engine.joints==[names[1],names[2]]
for i in range(1,80):
 t[0]+=.1
 for n in names:b.r[n]['Present_Position']=b.r[n]['Goal_Position']
 o.poll()
 if not o.engine.active:break
assert o.state['completed']==3 and o.state['closure_outcome']=='endpoint_settled' and o.goals[names[1]]==1900 and o.goals[names[2]]==2100 and o.state['stop_count']==0
o.motion_count=20
try:o.command(dict(c,id=2))
except ValueError as exc:assert 'budget' in str(exc)
else:raise AssertionError('Budget ignored')
# Health faults still take effect while the camera is paused.
t[0]+=.1;meta['received_at']=-20;b.r[names[0]]['Status']=8
try:o.poll()
except RuntimeError as exc:assert 'Status' in str(exc)
else:raise AssertionError('Paused camera masked servo fault')
o.release_all('test fault');assert not o.enabled and all(b.r[n]['Torque_Enable']==0 and b.r[n]['P_Coefficient']==16 and b.r[n]['Torque_Limit']==1000 for n in names)
print('Pickup owner settings, simultaneous two-joint move, 120s holds, pause health monitoring, 20-segment budget and release restoration passed; no hardware')
