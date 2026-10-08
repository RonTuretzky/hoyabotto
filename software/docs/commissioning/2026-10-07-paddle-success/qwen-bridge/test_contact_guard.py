from paddle_joint_executor import PaddleJointExecutor
lift='right_arm_shoulder_lift';t=[0.];writes=[]
def make(target=2400):
 e=PaddleJointExecutor([lift],{lift:[826,3268]},writes.append,clock=lambda:t[0],wall=lambda:t[0]);e.start({'id':1,'session_started':7,'positions':{lift:target},'duration_s':3},{lift:2697},session_started=7);return e
def run(e,plant,load,steps=400):
 for i in range(steps):
  t[0]=round(t[0]+.1,6);q=plant(e.goal[lift]);out=e.tick({lift:q},telemetry_at=t[0],rows={lift:{'Moving':0,'Present_Velocity':0,'Present_Load':load(q,e.goal[lift])}})
  if not e.active:return out
 raise AssertionError('never finished')
# Blocked by something at 2550 while commanded to 2400: load builds once it presses. Halt, back off, hold.
writes.clear();e=make();out=run(e,lambda g:max(g,2550),lambda q,g:700 if g<q-10 else 150)
assert out['closure_outcome']=='contact_halt' and out['endpoint_reached'] is False and out['contact'][lift]['load']==700
assert writes[-1]=={lift:2550} and e.goal[lift]==2550 # no longer pushing
# Heavy load while keeping up with the ramp is not contact.
t[0]=0;e=make();out=run(e,lambda g:g,lambda q,g:650)
assert out['closure_outcome']=='endpoint_settled'
# Gravity sag at moderate load is still corrected.
t[0]=0;e=make();out=run(e,lambda g:g+58,lambda q,g:300)
assert out['closure_outcome']=='endpoint_settled' and e.corrections[lift]>=1
# A correction that does not move the joint is not repeated (no ratcheting into an obstacle); load stays below the halt level.
t[0]=0;e=make();out=run(e,lambda g:max(g+58,2470),lambda q,g:400)
assert out['closure_outcome']=='settled_short' and e.corrections[lift]==1 and out['possible_contact_joints']==[lift]
print('Contact guard: halt and back off on loaded lag, no false halt while keeping up, sag correction kept, no correction ratcheting passed; no hardware')
