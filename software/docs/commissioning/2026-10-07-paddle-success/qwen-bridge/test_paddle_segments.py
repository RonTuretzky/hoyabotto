from paddle_segments import paddle_target_segments
r='right_arm_';rows=lambda **q:{'rows':{r+n:{'Present_Position':v} for n,v in q.items()}}
s=paddle_target_segments({r+'shoulder_lift':2450,r+'elbow_flex':2100},rows(shoulder_lift=2697,elbow_flex=2000))
assert s==[{r+'shoulder_lift':2450,r+'elbow_flex':2100}] # one simultaneous segment
s=paddle_target_segments({r+'shoulder_lift':2000,r+'elbow_flex':2100,r+'wrist_roll':2001},rows(shoulder_lift=2697,elbow_flex=2000,wrist_roll=2000))
assert len(s)==3 and all(r+'shoulder_lift' in x for x in s) and [r+'elbow_flex' in x for x in s]==[False,False,True] and all(r+'wrist_roll' not in x for x in s)
assert paddle_target_segments({r+'shoulder_lift':2356},rows(shoulder_lift=2697))==[{r+'shoulder_lift':2356}] # pilot's 341-tick step stays one segment
assert s[-1]=={r+'shoulder_lift':2000,r+'elbow_flex':2100} and max(abs(a[r+'shoulder_lift']-b[r+'shoulder_lift']) for a,b in zip([{r+'shoulder_lift':2697}]+s,s))<=280
s=paddle_target_segments({r+'shoulder_lift':2600,r+'gripper':1400},rows(shoulder_lift=2697,gripper=1592))
assert s==[{r+'shoulder_lift':2600},{r+'gripper':1400}] # closing gripper last and alone
s=paddle_target_segments({r+'shoulder_lift':2600,r+'gripper':1800},rows(shoulder_lift=2697,gripper=1592))
assert s==[{r+'shoulder_lift':2600,r+'gripper':1800}] # opening gripper moves with the arm
assert paddle_target_segments({r+'shoulder_lift':2698},rows(shoulder_lift=2697))==[]
try:paddle_target_segments({r+'shoulder_lift':2600},{'rows':{}})
except ValueError:pass
else:raise AssertionError('Missing encoder accepted')
print('Pickup segments: simultaneous joints, <=341 single segment, 280-tick spans beyond, closing gripper last and alone, no-op residuals passed')
