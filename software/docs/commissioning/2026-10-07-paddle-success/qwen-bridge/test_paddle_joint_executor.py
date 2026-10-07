from paddle_joint_executor import PaddleJointExecutor
n='right_arm_shoulder_lift';t=[0.];writes=[]
def make(name=n,start=2697,target=2450):
 e=PaddleJointExecutor([name],{name:[826,3268] if name==n else [1270,2824]},writes.append,clock=lambda:t[0],wall=lambda:t[0]);e.start({'id':1,'session_started':7,'positions':{name:target},'duration_s':2},{name:start},session_started=7);return e
row={'Moving':0,'Present_Velocity':0};e=make();e.tick({n:2697},telemetry_at=0,rows={n:row});assert writes[-1][n]==2657
t[0]=.1;e.tick({n:2682},telemetry_at=.1,rows={n:{'Moving':1,'Present_Velocity':-100}});assert e.active # 25-tick lag permitted
try:
 t[0]=.2;e.tick({n:2754},telemetry_at=.2,rows={n:row})
except RuntimeError as x:assert '96ticks' in str(x)
else:raise AssertionError('Excess lag accepted')
t[0]=0;e=make();q=2697
for i in range(1,120):
 t[0]=i*.1;q=e.goal;e.tick({n:q},telemetry_at=t[0],rows={n:row})
 if not e.active:break
assert not e.active and e.goal==2450
for target in [2300,830]:
 try:make(target=target)
 except ValueError:pass
 else:raise AssertionError('Segment/margin ignored')
g='right_arm_gripper';t[0]=0;e=make(g,1592,1310)
for i in range(1,300):
 t[0]=i*.1;q=max(1350,e.goal);r=e.tick({g:q},telemetry_at=t[0],rows={g:row})
 if not e.active:break
assert not e.active and r['closure_outcome']=='stationary_closure_unverified' and r['grasp_verified'] is False
# Quiet position is mandatory: tolerance alone cannot complete a moving joint.
t[0]=0;e=make();e.goal=e.target
for i in range(1,5):
 t[0]=i*.1;e.tick({n:e.target},telemetry_at=t[0],rows={n:{'Moving':1,'Present_Velocity':10}})
assert e.active
print('Pickup executor: lag, envelope, ramp, margin, segment, contact, moving-settle checks passed; no hardware access')
