from paddle_camera_gate import PaddleCameraGate
from paddle_joint_executor import PaddleJointExecutor
t=[0.];meta={'received_at':0,'seq':1};g=PaddleCameraGate(lambda:meta,clock=lambda:t[0],wall=lambda:t[0]);assert g.update()
t[0]=11
try:g.update()
except RuntimeError:pass
else:raise AssertionError('Stale preflight accepted')
assert g.update(holding=True) is False
t[0]=12;meta['received_at']=12;assert g.update(holding=True) is False # Same sequence cannot resume.
meta['seq']=2;assert g.update(holding=True)
t[0]=23;assert g.update(holding=True) is False
t[0]=43
try:g.update(holding=True)
except RuntimeError:assert g.events[-1]['timed_out']
else:raise AssertionError('Stale camera indefinitely held')
t[0]=0;writes=[];n='right_arm_shoulder_lift';e=PaddleJointExecutor([n],{n:[826,3268]},writes.append,clock=lambda:t[0],wall=lambda:t[0]);e.start({'id':1,'session_started':7,'positions':{n:2450},'duration_s':2},{n:2697},session_started=7,held_goals={n:2720});e.tick({n:2697},telemetry_at=0,rows={n:{'Moving':0,'Present_Velocity':0}});assert writes[-1][n]==2680 # Step from held goal, not encoder.
old=dict(writes[-1]);t[0]=15;e.pause(15);assert writes[-1]==old and e.started==15 and e.last_tick==15
t[0]=15.1;e.tick({n:2680},telemetry_at=t[0],rows={n:{'Moving':0,'Present_Velocity':0}});assert writes[-1]==old
print('Camera preflight, changed-sequence recovery, 20s timeout, held-waypoint ramp and nonblocking timer pause passed')
