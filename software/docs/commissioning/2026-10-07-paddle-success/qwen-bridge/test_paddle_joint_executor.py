from paddle_joint_executor import PaddleJointExecutor
n='right_arm_shoulder_lift';e2='right_arm_elbow_flex';g='right_arm_gripper';t=[0.];writes=[]
RANGES={n:[826,3268],e2:[826,3268],g:[1270,2824]}
def make(name=n,start=2697,target=2450,**kw):
 e=PaddleJointExecutor([name],{name:RANGES[name]},writes.append,clock=lambda:t[0],wall=lambda:t[0]);e.start({'id':1,'session_started':7,'positions':{name:target},'duration_s':2},{name:start},session_started=7,**kw);return e
def run(e,plant,steps=400,dt=.1):
 """Drive the executor with a plant mapping commanded goals to measured positions."""
 r=None
 for i in range(1,steps):
  t[0]=round(t[0]+dt,6);r=e.tick({j:plant(j,e.goal[j]) for j in e.joints},telemetry_at=t[0],rows={j:row for j in e.joints})
  if not e.active:return r
 raise AssertionError('Executor still active')
row={'Moving':0,'Present_Velocity':0};e=make();e.tick({n:2697},telemetry_at=0,rows={n:row});assert writes[-1][n]==2657
t[0]=.1;e.tick({n:2682},telemetry_at=.1,rows={n:{'Moving':1,'Present_Velocity':-100}});assert e.active # 25-tick lag permitted
try:
 t[0]=.2;e.tick({n:2754},telemetry_at=.2,rows={n:row})
except RuntimeError as x:assert '96ticks' in str(x)
else:raise AssertionError('Excess lag accepted')
t[0]=0;e=make();r=run(e,lambda j,goal:goal)
assert e.goal[n]==2450 and r['closure_outcome']=='endpoint_settled' and r['endpoint_reached'] and e.corrections[n]==0
# A joint resting near its limit (inside the 40-tick margin) can be commanded back inward, but not further out.
t[0]=0;e=make(start=3250,target=3100);r=run(e,lambda j,goal:goal);assert r['closure_outcome']=='endpoint_settled' and e.goal[n]==3100
try:make(start=3250,target=3260)
except ValueError:pass
else:raise AssertionError('Target inside the 40-tick margin accepted')
for target in [2300,830]:
 try:make(target=target)
 except ValueError:pass
 else:raise AssertionError('Segment/margin ignored')
t[0]=0;e=make(g,1592,1310);r=run(e,lambda j,goal:max(1350,goal))
assert r['closure_outcome']=='stationary_closure_unverified' and r['grasp_verified'] is False and r['endpoint_reached'] is False
# Quiet position is mandatory: tolerance alone cannot complete a moving joint.
t[0]=0;e=make();e.goal[n]=e.targets[n]
for i in range(1,5):
 t[0]=i*.1;e.tick({n:e.targets[n]},telemetry_at=t[0],rows={n:{'Moving':1,'Present_Velocity':10}})
assert e.active
# A joint that keeps moving still fails at the deadline; no correction can turn it into a success.
t[0]=0;e=make();e.goal[n]=e.targets[n];e.first_step=False;wobble=[0]
try:
 for i in range(1,400):
  t[0]=i*.1;wobble[0]=-wobble[0] or 6;e.tick({n:e.targets[n]-58+wobble[0]},telemetry_at=t[0],rows={n:{'Moving':1,'Present_Velocity':40}})
except RuntimeError as x:assert 'deadline' in str(x) and e.corrections[n]==0
else:raise AssertionError('Unsettled joint completed')
# Gravity sag: the joint comes to rest 58 ticks short (the failed move). Bounded overdrive brings it within tolerance.
t[0]=0;writes.clear();e=make();r=run(e,lambda j,goal:goal+58)
assert r['closure_outcome']=='endpoint_settled' and r['endpoint_reached'] and abs(r['settle_residual_ticks'][n])<=57
assert 0<abs(e.bias[n])<=57 and e.corrections[n]>=1 and all(abs(w[n]-(e.goal[n]-e.bias[n]))<=57 for w in writes[-e.corrections[n]:])
assert all(s['joints'][n]['following_error_ticks']<=96 for s in e.samples)
# Stuck Moving flag inside tolerance: corrected once, then completes on physical rest.
t[0]=0;e=make();rows_flag={n:{'Moving':1,'Present_Velocity':0}}
for i in range(1,200):
 t[0]=round(i*.1,6);r=e.tick({n:e.goal[n]+20 if e.goal[n]==e.targets[n] else e.goal[n]},telemetry_at=t[0],rows=rows_flag)
 if not e.active:break
assert not e.active and r['closure_outcome']=='endpoint_settled'
# Sag too large to correct inside the 96-tick envelope: hold and report settled_short; never claim the endpoint.
t[0]=0;e=make();r=run(e,lambda j,goal:goal+90)
assert r['closure_outcome']=='settled_short' and r['endpoint_reached'] is False and r['settle_residual_ticks'][n]==90 and r['phase']=='holding'
# Simultaneous movement: two joints ramp together and arrive on the same step.
t[0]=0;writes.clear();e=PaddleJointExecutor([n,e2],RANGES,writes.append,clock=lambda:t[0],wall=lambda:t[0])
e.start({'id':2,'session_started':7,'positions':{n:2450,e2:2100},'duration_s':3},{n:2697,e2:2000},session_started=7)
r=run(e,lambda j,goal:goal)
assert r['closure_outcome']=='endpoint_settled' and set(r['direct_actual_positions'])=={n,e2}
assert all(set(w)=={n,e2} for w in writes) and writes[-1]=={n:2450,e2:2100} and max(abs(w-v) for a,b in zip(writes,writes[1:]) for w,v in [(a[n],b[n])])<=40
assert e.deadline<=28+5
# A closing gripper must be commanded alone; an opening gripper may move with other joints.
try:PaddleJointExecutor([n,g],RANGES,writes.append,clock=lambda:t[0],wall=lambda:t[0]).start({'id':3,'session_started':7,'positions':{n:2450,g:1400},'duration_s':2},{n:2697,g:1592},session_started=7)
except ValueError as x:assert 'alone' in str(x)
else:raise AssertionError('Contact closure combined with other joints')
PaddleJointExecutor([n,g],RANGES,writes.append,clock=lambda:t[0],wall=lambda:t[0]).start({'id':3,'session_started':7,'positions':{n:2450,g:1800},'duration_s':2},{n:2697,g:1592},session_started=7)
print('Pickup executor: lag, envelope, ramp, margin, segment, contact, moving-settle, deadline, sag correction, settled-short and simultaneous checks passed; no hardware access')
