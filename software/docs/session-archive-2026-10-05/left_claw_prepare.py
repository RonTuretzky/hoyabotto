"""One isolated left-claw opening, raw +68 ticks; no other motor writes."""
import json,sys,time,signal,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'xlerobot-farm/software'))
from farm.config import load_profile
from farm.adapters.robot_lerobot import LeRobotXLeRobot
from strict_servo_replies import guard_replies
from coherent_servo_telemetry import read_servo_telemetry
sys.path.insert(0,str(ROOT/'work/carton-visual-controller/software'))
from carton.servo.vision import ManifestCamera
result={'motor':'left_arm_gripper','requested_delta_ticks':68,'samples':[],'ok':False,'started':time.time()}
name='left_arm_gripper';r=LeRobotXLeRobot(load_profile('paper-tray-v0').robot).robot;b=r.bus1;c=r.calibration[name];old={};powered=False
cameras={n:ManifestCamera(ROOT/'work/robot-camera-stream'/f'{n}.json',ident,5) for n,ident in [('head','0x12400005a39230'),('oak','oak-1944301091DA1C2E00')]}
def evidence(label):
 for n,cam in cameras.items():
  f=cam.read();(ROOT/'work'/f'left-claw-{label}-{n}.jpg').write_bytes(f.raw_bytes)
def guard():
 for cam in cameras.values():cam.read()
def read(field):return int(b.read(field,name,normalize=False,num_retry=2))
def checked(field,value):
 b.write(field,name,value,normalize=False,num_retry=2)
 if read(field)!=value:raise RuntimeError(field+' readback mismatch')
def sample():
 guard();row=read_servo_telemetry(b,name);result['samples'].append({'time':time.time(),**row})
 if row['Status'] or row['Present_Temperature']>55 or abs(row['Present_Load'])>250 or not 105<=row['Present_Voltage']<=140:raise RuntimeError('Claw health limit: '+json.dumps(row))
 return row
signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt('STOP')))
try:
 if '--execute' not in sys.argv:raise RuntimeError('Explicit --execute required')
 for port in ('/dev/cu.usbmodem5B790186401','/dev/cu.usbmodem5B790182091'):
  q=subprocess.run(['lsof','-t',port],capture_output=True,text=True)
  if q.returncode!=1 or q.stdout.strip():raise RuntimeError('Serial port already owned')
 evidence('before');guard_replies(b);b.connect(handshake=False)
 for n in b.motors:
  if int(b.read('Torque_Enable',n,normalize=False,num_retry=2))!=0:raise RuntimeError('Other motor holding: '+n)
 for f,v in [('Homing_Offset',c.homing_offset),('Min_Position_Limit',c.range_min),('Max_Position_Limit',c.range_max)]:
  if read(f)!=v:raise RuntimeError('Calibration mismatch')
 if read('Operating_Mode')!=0:raise RuntimeError('Claw not in position mode')
 start=sample()['Present_Position'];target=start+68
 if not c.range_min+4<=start<target<=c.range_max-4:raise RuntimeError('Opening outside saved working range')
 result.update(start=start,target=target)
 old={f:read(f) for f in ('Goal_Velocity','Acceleration','Torque_Limit','Lock')}
 checked('Lock',0);checked('Goal_Velocity',100);checked('Acceleration',10);checked('Torque_Limit',min(250,old['Torque_Limit']))
 checked('Goal_Position',start);guard();checked('Torque_Enable',1);powered=True;checked('Lock',1);checked('Goal_Position',target)
 deadline=time.monotonic()+3;stable=0
 while time.monotonic()<deadline:
  row=sample();p=row['Present_Position']
  if not start-4<=p<=target+4:raise RuntimeError('Opening travel envelope exceeded')
  stable=stable+1 if abs(p-target)<=11 else 0
  if stable>=3 and p-start>=12:result.update(ok=True,position=p,moved_ticks=p-start);break
  time.sleep(.06)
 if not result['ok']:raise RuntimeError('Claw did not reach the bounded opening')
 time.sleep(.25);evidence('opened')
except BaseException as e:result['error']=str(e)
finally:
 if b.is_connected:
  try:
   b.disable_torque([name],num_retry=3);result['released']=read('Torque_Enable')==0
   if not result['released']:raise RuntimeError('Release not verified')
   checked('Lock',0)
   for f in ('Goal_Velocity','Acceleration','Torque_Limit'):
    if f in old:checked(f,old[f])
   if 'Lock' in old:checked('Lock',old['Lock'])
   result['all_left_released']=all(int(b.read('Torque_Enable',n,normalize=False,num_retry=2))==0 for n in b.motors)
  except BaseException as e:result.update(ok=False,release_error=str(e))
  finally:b.disconnect(disable_torque=False)
 (ROOT/'work/left-claw-prepare-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
raise SystemExit(0 if result['ok'] and result.get('released') else 1)
