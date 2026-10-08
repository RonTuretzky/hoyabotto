import os
if os.environ.get('XLEROBOT_RUN_ARCHIVED_PROTOTYPE') != '1':
 raise SystemExit('Archived commissioning prototype: read README; never run beside the sole hardware owner.')
import sys,json,time,math
from pathlib import Path
sys.path[:0]=['/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/scripts','/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/work']
from carton_robot.servo_ownership import ServoOwnership
from carton_robot.guarded_pr3282_calibration import PORTS,install_calibration_reply_guard
from carton_robot.coherent_servo_telemetry import read_servo_telemetry
from farm.vendor.autocal.workflow import FeetechMotorsBus,SO_FOLLOWER_MOTORS
from lerobot.motors import Motor,MotorNormMode
from wheel_stop_check import stationary_released,settled_before_release
names=('base_left_wheel','base_right_wheel');duration=float(sys.argv[1]);out=Path(sys.argv[2]);assert .2<=duration<=1.0
direction=sys.argv[3] if len(sys.argv)>3 else 'forward';assert direction in ('forward','backward');sign=1 if direction=='forward' else -1
camera=Path('/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera/latest.json')
def fresh():
 d=json.loads(camera.read_text());assert d.get('received_at') is not None and time.time()-d['received_at']<10,'Camera stale'
lock=ServoOwnership(PORTS).acquire();b=None;before={};report={'samples':[],'success':False,'duration_s':duration,'speed_m_s':.02,'direction':direction};rc=1
try:
 fresh()
 for side,port,extra in [('left',PORTS[0],{'head_pan':7,'head_tilt':8}),('right',PORTS[1],{'wheel_left':9,'wheel_right':10})]:
  motors=SO_FOLLOWER_MOTORS.copy();motors.update({n:Motor(i,'sts3215',MotorNormMode.RANGE_M100_100) for n,i in extra.items()});check=FeetechMotorsBus(port=port,motors=motors)
  try:
   check.connect(handshake=False);install_calibration_reply_guard(check)
   for n in motors:assert check.read('Torque_Enable',n,normalize=False)==0 and check.read('Status',n,normalize=False)==0,n
  finally:
   if check.is_connected:check.disconnect(disable_torque=False)
 b=FeetechMotorsBus(port=PORTS[1],motors={n:Motor(i,'sts3215',MotorNormMode.RANGE_M100_100) for n,i in zip(names,(9,10))});b.connect(handshake=False);install_calibration_reply_guard(b)
 def rd(reg,n):return b.read(reg,n,normalize=False,num_retry=0)
 def wr(reg,n,v):b.write(reg,n,v,normalize=False,num_retry=0)
 def sample():
  rows={n:read_servo_telemetry(b,n) for n in names}
  for n,r in rows.items():assert r['Status']==0 and abs(r['Present_Load'])<=500 and abs(r['Present_Velocity'])<=400 and 100<=r['Present_Voltage']<=140,(n,r)
  report['samples'].append(rows);return rows
 for n in names:
  before[n]={r:rd(r,n) for r in ('Operating_Mode','Acceleration','Torque_Limit','Lock','Homing_Offset')}
  wr('Goal_Velocity',n,0);wr('Lock',n,0);wr('Operating_Mode',n,1);wr('Acceleration',n,10)
  assert rd('Operating_Mode',n)==1 and rd('Goal_Velocity',n)==0
 initial=sample();report['before']={n:initial[n]['Present_Position'] for n in names};report['settings_before']=before
 commands=dict(zip(names,(-261*sign,261*sign)));report['commands']=commands
 for n in names:wr('Torque_Enable',n,1);assert rd('Torque_Enable',n)==1 and rd('Goal_Velocity',n)==0
 started=time.monotonic();deadline=started+duration
 for n,v in commands.items():wr('Goal_Velocity',n,v);assert rd('Goal_Velocity',n)==v;assert time.monotonic()-started<.15
 while time.monotonic()<deadline:
  fresh()
  if deadline-time.monotonic()>.25:sample()
  time.sleep(min(.02,max(0,deadline-time.monotonic())))
 for n in names:wr('Goal_Velocity',n,0)
 stopped=time.monotonic();report['pulse_elapsed_s']=stopped-started
 for n in names:assert rd('Goal_Velocity',n)==0
 window=[]
 while time.monotonic()-stopped<.9:
  rows=sample();window.append({'t':time.monotonic()-stopped,'position':{n:rows[n]['Present_Position'] for n in names},'velocity':{n:rows[n]['Present_Velocity'] for n in names}})
  if settled_before_release(window):break
  time.sleep(.045)
 else:raise RuntimeError('Wheels did not settle within .9s')
 for n in names:wr('Torque_Enable',n,0);assert rd('Torque_Enable',n)==0
 released=[]
 for _ in range(5):
  rows=sample();released.append({'position':{n:rows[n]['Present_Position'] for n in names},'velocity':{n:rows[n]['Present_Velocity'] for n in names}});time.sleep(.05)
 assert stationary_released(released),'Rolling after release'
 final=released[-1]['position'];delta={n:((final[n]-report['before'][n]+2048)%4096)-2048 for n in names}
 assert delta[names[0]]*sign<-5 and delta[names[1]]*sign>5 and all(abs(v)<=450 for v in delta.values()),delta
 assert abs(abs(delta[names[0]])-abs(delta[names[1]]))<=35,delta
 report.update(after=final,wheel_delta_ticks=delta,estimated_cm={n:abs(v)*2*math.pi*.05/4096*100 for n,v in delta.items()},stop_samples=window,released_stop_samples=released,success=True);rc=0
except BaseException as e:report['error']=str(e)
finally:
 if b is not None and b.is_connected:
  report['release']={}
  for n in names:
   try:
    b.write('Goal_Velocity',n,0,normalize=False,num_retry=3);b.write('Torque_Enable',n,0,normalize=False,num_retry=3)
    report['release'][n]=b.read('Torque_Enable',n,normalize=False)==0 and b.read('Goal_Velocity',n,normalize=False)==0
   except BaseException as e:report['release'][n]={'error':str(e)};rc=1
  if all(v is True for v in report['release'].values()):
   try:
    for n in names:
     for reg in ('Operating_Mode','Acceleration','Torque_Limit','Lock'):
      b.write(reg,n,before[n][reg],normalize=False);assert b.read(reg,n,normalize=False)==before[n][reg]
    for n in names:
     b.write('Torque_Enable',n,0,normalize=False,num_retry=3)
     assert b.read('Torque_Enable',n,normalize=False)==0 and b.read('Goal_Velocity',n,normalize=False)==0
    report['settings_restored']=True
    report['post_restore_release_verified']=True
   except BaseException as e:report['restore_error']=str(e);rc=1
  b.disconnect(disable_torque=False)
 lock.close();out.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','stop_samples','released_stop_samples')}))
sys.exit(rc)
