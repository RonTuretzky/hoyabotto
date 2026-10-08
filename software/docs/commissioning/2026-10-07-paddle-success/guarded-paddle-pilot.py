import os
if os.environ.get('XLEROBOT_RUN_ARCHIVED_PROTOTYPE') != '1':
 raise SystemExit('Archived commissioning prototype: read README; never run beside the sole hardware owner.')
import sys,json,time,select
from pathlib import Path
sys.path.insert(0,'/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/scripts')
from carton_robot.servo_ownership import ServoOwnership
from carton_robot.guarded_pr3282_calibration import PORTS,install_calibration_reply_guard
from carton_robot.coherent_servo_telemetry import read_servo_telemetry
from farm.vendor.autocal.workflow import FeetechMotorsBus,SO_FOLLOWER_MOTORS
cal=json.loads(Path('/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json').read_text());held=list(SO_FOLLOWER_MOTORS)
lock=ServoOwnership(PORTS).acquire();b=FeetechMotorsBus(port=PORTS[1],motors=SO_FOLLOWER_MOTORS.copy());backup={};report={'segments':[],'samples':[]};goals={};rc=1
camera=Path('/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera/latest.json')
def fresh():
 d=json.loads(camera.read_text());stamp=d.get('received_at')
 if stamp is not None and time.time()-stamp<10:return
 if not b.is_connected or len(goals)!=len(held):raise RuntimeError('Phone feed stale before hold established')
 # Keep the last issued waypoint; never advance a target while camera data is stale.
 started=time.monotonic();pause={'start':time.time(),'initial_sequence':d.get('seq')};report.setdefault('camera_pauses',[]).append(pause)
 print('CAMERA PAUSE: holding current waypoint; motor-health checks continue',flush=True)
 while time.monotonic()-started<20:
  sample(check_camera=False)
  d=json.loads(camera.read_text());stamp=d.get('received_at')
  if stamp is not None and time.time()-stamp<2 and d.get('seq')!=pause['initial_sequence']:
   pause['resumed_after_s']=time.monotonic()-started;print('CAMERA RESUMED',flush=True);return
  time.sleep(.1)
 pause['timed_out']=True;raise RuntimeError('Phone feed stale after monitored 20s hold')
try:
 fresh();b.connect(handshake=False);install_calibration_reply_guard(b)
 for n in b.motors:
  assert b.read('Torque_Enable',n,normalize=False)==0 and b.read('Status',n,normalize=False)==0
  c=cal['right_arm_'+n]
  for reg,key in [('Homing_Offset','homing_offset'),('Min_Position_Limit','range_min'),('Max_Position_Limit','range_max')]:assert b.read(reg,n,normalize=False)==c[key],n
  assert b.read('Operating_Mode',n,normalize=False)==0
 for n in held:
  pos=b.read('Present_Position',n,normalize=False);goals[n]=pos
  backup[n]={r:b.read(r,n,normalize=False) for r in ('Torque_Limit','Goal_Velocity','Acceleration','P_Coefficient')}
  for reg,v in [('Torque_Limit',400 if n=='elbow_flex' else (min(500,backup[n]['Torque_Limit']) if n=='gripper' else min(800,backup[n]['Torque_Limit']))),('Goal_Velocity',200 if n=='gripper' else 100),('Acceleration',5),('P_Coefficient',32 if n in ('elbow_flex','shoulder_lift') else backup[n]['P_Coefficient']),('Goal_Position',pos)]:
   b.write(reg,n,v,normalize=False);assert b.read(reg,n,normalize=False)==v
 report['settings_before']=backup
 for n in held:b.write('Torque_Enable',n,1,normalize=False);assert b.read('Torque_Enable',n,normalize=False)==1
 def sample(check_camera=True):
  if check_camera:fresh()
  rows={n:read_servo_telemetry(b,n) for n in held};report['samples'].append(rows)
  for n,row in rows.items():
   c=cal['right_arm_'+n];assert row['Status']==0 and abs(row['Present_Load'])<=(500 if n=='gripper' else 800) and 100<=row['Present_Voltage']<=140,(n,row)
   assert c['range_min']<=row['Present_Position']<=c['range_max'] and abs(row['Present_Position']-goals[n])<=96,(n,row,goals[n])
  return rows
 def move(n,delta):
  current=b.read('Present_Position',n,normalize=False);target=current+delta;c=cal['right_arm_'+n];assert abs(delta)<=341 and c['range_min']+40<=target<=c['range_max']-40
  progress_pos=current;progress_time=time.monotonic()
  while goals[n]!=target:
   difference=target-goals[n];goals[n]+=max(-40,min(40,difference));b.write('Goal_Position',n,goals[n],normalize=False)
   assert b.read('Goal_Position',n,normalize=False)==goals[n]
   deadline=time.monotonic()+.4
   while time.monotonic()<deadline:
    rows=sample()
    if n=='gripper':
     observed=rows[n]['Present_Position']
     if abs(observed-progress_pos)>=8:progress_pos=observed;progress_time=time.monotonic()
     assert abs(observed-goals[n])<=30 or time.monotonic()-progress_time<=1,'Gripper no-progress guard'
    time.sleep(.05)
  deadline=time.monotonic()+3;stable=0;last=None
  while time.monotonic()<deadline:
   rows=sample();row=rows[n];pos=row['Present_Position'];ok=abs(pos-target)<=(30 if n=='gripper' else 57) and last is not None and abs(pos-last)<=3 and row['Moving']==0 and abs(row['Present_Velocity'])<3
   stable=stable+1 if ok else 0;last=pos
   if stable>=3:break
   time.sleep(.05)
  assert stable>=3,'Motion did not settle'
  r={'joint':n,'start':current,'target':target,'position':pos,'actual_delta_deg':(pos-current)*360/4096,'all_goals':goals.copy()};report['segments'].append(r);Path('work/right-reach-current.json').write_text(json.dumps(r));print(json.dumps(r),flush=True)
 def close_contact():
  n='gripper';start=b.read('Present_Position',n,normalize=False);target=cal['right_arm_'+n]['range_min']+40
  assert 0<=start-target<=341,'Closure exceeds bounded segment'
  outcome='closed_limit_without_contact'
  while goals[n]>target:
   goals[n]=max(target,goals[n]-10);b.write('Goal_Position',n,goals[n],normalize=False)
   assert b.read('Goal_Position',n,normalize=False)==goals[n]
   deadline=time.monotonic()+1.5;last=None;stationary_since=None;stopped=False
   while time.monotonic()<deadline:
    row=sample()[n];pos=row['Present_Position'];quiet=row['Moving']==0 and abs(row['Present_Velocity'])<3 and last is not None and abs(pos-last)<=3
    stationary_since=(stationary_since if stationary_since is not None else time.monotonic()) if quiet else None
    if quiet and stationary_since is not None and time.monotonic()-stationary_since>=.3:
     if pos-goals[n]>=40:
      outcome='stationary_closure_unverified';stopped=True
     break
    last=pos;time.sleep(.05)
   else:raise RuntimeError('Closure did not become stationary')
   if stopped:break
  rows=sample();pos=rows[n]['Present_Position']
  r={'joint':n,'start':start,'target':goals[n],'position':pos,'closure_outcome':outcome,'load':rows[n]['Present_Load'],'grasp_verified':False,'all_goals':goals.copy()}
  report['segments'].append(r);Path('work/right-reach-current.json').write_text(json.dumps(r));print(json.dumps(r),flush=True)
 lift=b.read('Present_Position','shoulder_lift',normalize=False)
 if lift<1124:move('shoulder_lift',min(341,1181-lift))
 for count in range(20):
  print('Command: JOINT DELTA_TICKS, CLOSE, or STOP (120s monitored hold)',flush=True);deadline=time.monotonic()+120
  while time.monotonic()<deadline:
   sample()
   if select.select([sys.stdin],[],[],.05)[0]:answer=sys.stdin.readline().strip();break
  else:raise RuntimeError('Hold deadline expired; release')
  if answer.lower()=='stop':rc=0;break
  if answer.upper()=='CLOSE':close_contact();continue
  n,d=answer.split();assert n in held;move(n,int(d))
except BaseException as e:report['error']=str(e);print('ABORT',str(e),flush=True)
finally:
 if b.is_connected:
  report['release']={}
  for n in held:
   try:
    b.write('Torque_Enable',n,0,normalize=False,num_retry=3);report['release'][n]=b.read('Torque_Enable',n,normalize=False)==0
   except BaseException as e:report['release'][n]={'error':str(e)};rc=1
  if all(v is True for v in report['release'].values()):
   try:
    for n,settings in backup.items():
     for reg,v in settings.items():b.write(reg,n,v,normalize=False)
     b.write('Torque_Enable',n,0,normalize=False);assert b.read('Torque_Enable',n,normalize=False)==0
    report['settings_restored']=True
   except BaseException as e:report['restore_error']=str(e);rc=1
  try:report['all_six_torque_off']=all(b.read('Torque_Enable',n,normalize=False)==0 for n in b.motors)
  except BaseException as e:report['release_check_error']=str(e);rc=1
  b.disconnect(disable_torque=False)
 lock.close();Path('outputs/Right-Paddle-Contact-Approach-Result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','segments')}),flush=True)
sys.exit(rc)
