from paddle_segments import paddle_target_segments
r='right_arm_';rows=lambda **q:{'rows':{r+n:{'Present_Position':v} for n,v in q.items()}}
s=paddle_target_segments({r+'shoulder_lift':2450,r+'elbow_flex':2100},rows(shoulder_lift=2697,elbow_flex=2000))
assert s==[{r+'shoulder_lift':2450,r+'elbow_flex':2100}] # one simultaneous segment
s=paddle_target_segments({r+'shoulder_lift':2000,r+'elbow_flex':2100,r+'wrist_roll':2001},rows(shoulder_lift=2697,elbow_flex=2000,wrist_roll=2000))
assert len(s)==3 and all(set(x)=={r+'shoulder_lift',r+'elbow_flex'} for x in s) and all(r+'wrist_roll' not in x for x in s)  # same joints in every segment
assert [x[r+'elbow_flex'] for x in s]==[2033,2067,2100]  # moves with the arm, not only at the end
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

# The real owner client refuses a path whose waypoints name different joints (2026-10-08 12:2x refusals):
# shoulder_lift 1002->1350 (348 ticks, 2 segments) with three short joints.
s=paddle_target_segments({r+'shoulder_lift':1350,r+'shoulder_pan':2350,r+'elbow_flex':2650,r+'wrist_flex':1750},
                         rows(shoulder_lift=1002,shoulder_pan=2051,elbow_flex=3131,wrist_flex=2818))
assert len({frozenset(x) for x in s})==1, s
prev={r+'shoulder_lift':1002,r+'shoulder_pan':2051,r+'elbow_flex':3131,r+'wrist_flex':2818}
for x in s:
    assert all(abs(x[n]-prev[n])<=341 for n in x);prev=x
print('Paddle segments: same joints in every segment, proportional motion, legs within limits')

# A joint resting just outside the commandable band (16:1x today: lift at 3210, band ends at 3207) must not produce
# intermediate waypoints outside the band when a long move of another joint forces several segments.
st={'rows':{r+'shoulder_lift':{'Present_Position':3210},r+'elbow_flex':{'Present_Position':2097}},'ranges':{r+'shoulder_lift':[847,3247],r+'elbow_flex':[944,3150]}}
s=paddle_target_segments({r+'shoulder_lift':3200,r+'elbow_flex':1000},st)
assert len(s)==4 and all(x[r+'shoulder_lift']<=3207 for x in s) and s[-1]=={r+'shoulder_lift':3200,r+'elbow_flex':1000},s
print('Paddle segments: intermediate waypoints clamped into the 40-tick band')
