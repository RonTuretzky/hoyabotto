from types import SimpleNamespace as C
from gemma_hardware_owner import HardwareOwner
class B:
 def __init__(self):
  self.motors={'left_arm_test':None,'right_arm_test':None,'base_left_wheel':None};self.writes=[]
  self.r={n:dict(Torque_Enable=0,Operating_Mode=0,Homing_Offset=0,Min_Position_Limit=100 if not n.startswith('base') else 0,Max_Position_Limit=1000 if not n.startswith('base') else 4095,Present_Position=500 if n!='right_arm_test' else 1100,Lock=1,Torque_Limit=1000,Goal_Velocity=200,Goal_Time=0,Acceleration=20,P_Coefficient=16,Goal_Position=500) for n in self.motors}
 def read(self,f,n,**kw):return self.r[n].get(f,0)
 def write(self,f,n,v,**kw):self.r[n][f]=v;self.writes.append((n,f,v))
 def disable_torque(self,ns,**kw):
  for n in ns:self.write('Torque_Enable',n,0)
def telemetry(b,n):return dict(Present_Position=b.r[n]['Present_Position'],Present_Temperature=30,Present_Load=0,Present_Velocity=0,Status=0)
def make():
 b=B();cal={n:C(range_min=100,range_max=1000,homing_offset=0) for n in b.motors if not n.startswith('base')};o=HardwareOwner([b],cal,telemetry);o.inspect();return o,b
count=0
o,b=make();assert not b.writes and o.state['phase']=='idle';count+=1
o.command({'id':1,'session_started':o.started,'op':'enable_motors','names':['left_arm_test'],'enabled':True});assert o.rows['left_arm_test']['Torque_Enable']==1 and o.rows['right_arm_test']['Torque_Enable']==0;count+=1
seq=[(f,v) for n,f,v in b.writes if n=='left_arm_test'];assert seq.index(('Goal_Position',500))<seq.index(('Torque_Enable',1));assert b.r['left_arm_test']['Torque_Limit']==400;count+=1
try:o.enable(['right_arm_test'],True)
except ValueError:count+=1
else:raise AssertionError('Out of range accepted')
o.enable(['base_left_wheel'],True);assert b.r['base_left_wheel']['Torque_Enable']==1 and b.r['base_left_wheel']['Operating_Mode']==0;count+=1
try:o.command({'id':2,'session_started':o.started,'op':'direct_joint','positions':{'base_left_wheel':600},'duration_s':6})
except ValueError:count+=1
else:raise AssertionError('Wheel motion accepted')
o.command({'id':3,'session_started':o.started,'op':'stop'});assert not o.enabled and o.latched and o.state['released'];count+=1
try:o.enable(['left_arm_test'],True)
except ValueError:count+=1
else:raise AssertionError('STOP reset accepted')
o.enable(['right_arm_test'],False);count+=1
for invalid in [(['left_arm_test','left_arm_test'],True),(['unknown'],True),(['left_arm_test'],1)]:
 try:o.enable(*invalid)
 except ValueError:count+=1
 else:raise AssertionError('Invalid enable accepted')
o,b=make()
try:o.command({'id':1,'session_started':o.started-1,'op':'enable_motors','names':['left_arm_test'],'enabled':True})
except ValueError:assert not b.writes;count+=1
else:raise AssertionError('Old session accepted')
o.enable(['left_arm_test'],True);o.lease=o.clock()-1
try:o.poll()
except RuntimeError:count+=1
else:raise AssertionError('Heartbeat ignored')
o.release_all('fake test');assert b.r['left_arm_test']['Torque_Limit']==1000 and b.r['left_arm_test']['Goal_Velocity']==200;count+=1
print({'hardware_owner_fake_checks_passed':count,'real_hardware_access':False})
o,b=make();o.enable(['left_arm_test'],True);before=list(b.writes)
for duration in [0,-1,float('nan'),26,'bad']:
 try:o.command({'id':8,'session_started':o.started,'op':'direct_joint','positions':{'left_arm_test':700},'duration_s':duration})
 except ValueError:assert b.writes==before and o.engine is None
 else:raise AssertionError('Malformed duration accepted')
o.release_all('Direct target did not settle before deadline');o.release_all('Hardware-owner exit')
assert o.state['error']=='Direct target did not settle before deadline' and o.state['root_failure']==o.state['error']
assert o.state['rows']['left_arm_test']['Torque_Enable']==0 and not o.enabled
print({'added_fault_and_invalid_request_checks':7,'real_hardware_access':False})
o,b=make();calls=[]
def corrupt_once(bus,name):
 calls.append(name)
 if len(calls)==1:raise RuntimeError('Coherent servo read communication failure: -7; transaction=fake')
 return telemetry(bus,name)
o.telemetry=corrupt_once;assert o.read_telemetry('left_arm_test')['Present_Position']==500
assert len(calls)==2 and o.state['idle_read_recovery']['recovered_count']==1
calls.clear();o.enabled.add('left_arm_test')
try:o.read_telemetry('left_arm_test')
except RuntimeError:assert len(calls)==1
else:raise AssertionError('Powered communication fault masked')
o.enabled.clear();calls.clear()
def always_bad(bus,name):calls.append(name);raise RuntimeError('Coherent servo read communication failure: -7')
o.telemetry=always_bad
try:o.read_telemetry('left_arm_test')
except RuntimeError:assert len(calls)==2
else:raise AssertionError('Persistent corruption masked')
calls.clear();o.rows['right_arm_test']['Torque_Enable']=1
try:o.read_telemetry('left_arm_test')
except RuntimeError:assert len(calls)==1
else:raise AssertionError('Powered row masked')
print({'idle_recovery_checks':4,'hardware_access':False})

o,b=make();o.telemetry=always_bad;calls.clear()
for row in o.rows.values():row['captured_at']=o.wall()-2
try:o.read_telemetry('left_arm_test')
except RuntimeError:assert len(calls)==1
else:raise AssertionError('Stale released proof allowed retry')
print({'stale_release_no_retry_check':True})
for reason in ['Coherent servo read packet fault: 1','Coherent servo read communication failure: -6']:
 o,b=make();calls.clear()
 def fault(bus,name):calls.append(name);raise RuntimeError(reason)
 o.telemetry=fault
 try:o.read_telemetry('left_arm_test')
 except RuntimeError:assert len(calls)==1
 else:raise AssertionError('Non-corrupt failure retried')
print({'no_servo_fault_or_timeout_retry_checks':2})
for unknown in ['missing_row','unknown_torque']:
 o,b=make();o.telemetry=always_bad;calls.clear()
 if unknown=='missing_row':o.rows.pop('right_arm_test')
 else:o.rows['right_arm_test']['Torque_Enable']=None
 try:o.read_telemetry('left_arm_test')
 except RuntimeError:assert len(calls)==1
 else:raise AssertionError('Unknown torque/rows retried')
print({'unknown_release_no_retry_checks':2})
o,b=make();o.enable(['left_arm_test'],True);d=o.state['enable_register_diagnostics']['left_arm_test'];assert d['pre_enable']['registers']['Torque_Limit']==1000 and d['applied']['registers']['Torque_Limit']==400
assert o.state['last_write_readbacks']['left_arm_test']['Goal_Position']['readback']==500
print({'pre_enable_applied_and_goal_diagnostics_checks':2})
o,b=make();o.enable(['left_arm_test'],True)
def hot(bus,name):return dict(telemetry(bus,name),Present_Temperature=90 if name=='left_arm_test' else 30)
o.telemetry=hot
try:o.poll()
except RuntimeError as exc:assert 'Present_Temperature=90; must be <= 70' in str(exc) and o.state['health_fault']['field']=='Present_Temperature'
else:raise AssertionError('Temperature guard missed')
print({'precise_health_diagnostic_check':True,'limits_unchanged':True})

for temperature in (70,71,90):
 o,b=make();o.enable(['left_arm_test'],True)
 def temp_sample(bus,name):return dict(telemetry(bus,name),Present_Temperature=temperature if name=='left_arm_test' else 30)
 o.telemetry=temp_sample
 if temperature==70:assert o.poll()['rows']['left_arm_test']['Present_Temperature']==70
 else:
  try:o.poll()
  except RuntimeError as exc:assert f'Present_Temperature={temperature}; must be <= 70' in str(exc)
  else:raise AssertionError('High temperature accepted')
print({'temperature_boundary_checks':3,'hardware_access':False})
