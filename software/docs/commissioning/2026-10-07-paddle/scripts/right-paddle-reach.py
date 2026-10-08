"""Archived commissioning prototype; see ../README.md. Not a production controller."""
import os as _archive_os
if _archive_os.environ.get("XLEROBOT_RUN_ARCHIVED_PROTOTYPE") != "1":
    raise SystemExit("Archived prototype: inspect source and README before explicit opt-in")
import sys,json,time,select
from pathlib import Path
sys.path.insert(0,'/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/scripts')
from carton_robot.servo_ownership import ServoOwnership
from carton_robot.guarded_pr3282_calibration import PORTS,install_calibration_reply_guard
from carton_robot.coherent_servo_telemetry import read_servo_telemetry
from farm.vendor.autocal.workflow import FeetechMotorsBus,SO_FOLLOWER_MOTORS
cal=json.loads(Path('/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json').read_text());held=list(SO_FOLLOWER_MOTORS)[:5]
lock=ServoOwnership(PORTS).acquire();b=FeetechMotorsBus(port=PORTS[1],motors=SO_FOLLOWER_MOTORS.copy());backup={};report={'segments':[],'samples':[]};goals={};rc=1
camera=Path('/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera/latest.json')
def fresh():
 d=json.loads(camera.read_text());assert d.get('received_at') is not None and time.time()-d['received_at']<10,'Phone feed stale'
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
  for reg,v in [('Torque_Limit',400 if n=='elbow_flex' else min(800,backup[n]['Torque_Limit'])),('Goal_Velocity',100),('Acceleration',5),('P_Coefficient',32 if n=='elbow_flex' else backup[n]['P_Coefficient']),('Goal_Position',pos)]:
   b.write(reg,n,v,normalize=False);assert b.read(reg,n,normalize=False)==v
 for n in held:b.write('Torque_Enable',n,1,normalize=False);assert b.read('Torque_Enable',n,normalize=False)==1
 def sample():
  fresh();rows={n:read_servo_telemetry(b,n) for n in held};report['samples'].append(rows)
  for n,row in rows.items():
   c=cal['right_arm_'+n];assert row['Status']==0 and abs(row['Present_Load'])<=800 and 100<=row['Present_Voltage']<=140,(n,row)
   assert c['range_min']<=row['Present_Position']<=c['range_max'] and abs(row['Present_Position']-goals[n])<=96,(n,row,goals[n])
  return rows
 def move(n,delta):
  current=b.read('Present_Position',n,normalize=False);target=current+delta;c=cal['right_arm_'+n];assert abs(delta)<=341 and c['range_min']+40<=target<=c['range_max']-40
  while goals[n]!=target:
   difference=target-goals[n];goals[n]+=max(-40,min(40,difference));b.write('Goal_Position',n,goals[n],normalize=False)
   assert b.read('Goal_Position',n,normalize=False)==goals[n]
   deadline=time.monotonic()+.4
   while time.monotonic()<deadline:sample();time.sleep(.05)
  deadline=time.monotonic()+3;stable=0;last=None
  while time.monotonic()<deadline:
   rows=sample();row=rows[n];pos=row['Present_Position'];ok=abs(pos-target)<=57 and last is not None and abs(pos-last)<=3 and row['Moving']==0 and abs(row['Present_Velocity'])<3
   stable=stable+1 if ok else 0;last=pos
   if stable>=3:break
   time.sleep(.05)
  assert stable>=3,'Motion did not settle'
  r={'joint':n,'start':current,'target':target,'position':pos,'actual_delta_deg':(pos-current)*360/4096,'all_goals':goals.copy()};report['segments'].append(r);Path('work/right-reach-current.json').write_text(json.dumps(r));print(json.dumps(r),flush=True)
 lift=b.read('Present_Position','shoulder_lift',normalize=False)
 if lift<1124:move('shoulder_lift',min(341,1181-lift))
 for count in range(12):
  print('Command: JOINT DELTA_TICKS, or STOP (120s monitored hold)',flush=True);deadline=time.monotonic()+120
  while time.monotonic()<deadline:
   sample()
   if select.select([sys.stdin],[],[],.05)[0]:answer=sys.stdin.readline().strip();break
  else:raise RuntimeError('Hold deadline expired; release')
  if answer.lower()=='stop':rc=0;break
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
 lock.close();Path('outputs/Right-Paddle-Reach-Result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','segments')}),flush=True)
sys.exit(rc)
