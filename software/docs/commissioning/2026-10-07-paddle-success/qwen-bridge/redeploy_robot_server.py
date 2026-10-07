"""Replace the running Qwen robot server with this folder's version, on the robot Mac.

Stops the API and the sole hardware owner, installs the changed qwen-bridge files into the
work folder (old copies are backed up), then starts a fresh owner with
--right-arm-only --paddle-profile and a fresh API. A fresh owner starts with every motor
released and stop_latched false (the owner has no STOP latch); nothing moves. The cloudflared relay is left running.
It also starts the native wrist-camera publisher for any wrist that has no fresh stream
(camera problems are reported, never fatal).

Refuses to stop an owner that is holding motors (torque off would drop the arm) unless
--release-holding is given. Run with --dry-run first to see what would be replaced.
"""
import argparse,difflib,json,os,shutil,signal,socket,subprocess,sys,time
from pathlib import Path

BRIDGE=Path(__file__).resolve().parent
ROOT=Path(os.environ.get('XLEROBOT_WORK_ROOT','/Users/teachera/Documents/Codex/2026-10-05/m'))
PYTHON=os.environ.get('XLEROBOT_PYTHON','/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python')
OAK_RAW_DIR=os.environ.get('XLEROBOT_OAK_RAW_DIR','/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/work/oak-live-stream')
WORK=ROOT/'work';SESSION=WORK/'gemma-hardware-session';STATUS=SESSION/'status.json'
OWNER_RECORD=WORK/'gemma-hardware-owner-process.json';API_RECORD=WORK/'gemma-robot-tools-process.json'
OWNER_LOG=WORK/'gemma-hardware-owner.log';API_LOG=WORK/'qwen-server-recovery/api.log'
INSTALL=['wheel_pulse_executor.py','paddle_joint_executor.py','paddle_segments.py','paddle_camera_gate.py','gemma_hardware_owner.py','gemma_direct_client.py','gemma_robot_tools.py','wrist_cameras.py','paddle-procedure.json','restart_gemma_owner_released.py']
TESTS=['test_continuous_motion.py','test_wheel_pulse.py','test_paddle_joint_executor.py','test_paddle_segments.py','test_paddle_camera_gate.py','test_paddle_owner.py','test_paddle_client.py','test_paddle_stop_recovery.py','test_gemma_hardware_owner.py','test_wrist_cameras.py']
OWNER_ARGS=['--right-arm-only','--paddle-profile','--wheels'];API_PORT=1241
WRIST_STREAM=WORK/'wrist-camera-stream';CAPTURE=WORK/'capture-single'
CAPTURE_SOURCE=BRIDGE.parents[1]/'session-archive-2026-10-05/capture-single.swift'
sys.path.insert(0,str(BRIDGE))
from wrist_cameras import WRIST_CAMERA_IDS,select_wrist_manifest,wrist_dirs

def say(message):print('[redeploy] '+message,flush=True)
def fail(message):say('ABORT: '+message);sys.exit(1)

def processes(script):
 """PIDs whose command line runs work/<script> (exact path inside this work folder)."""
 out=subprocess.run(['ps','-axo','pid=,command='],capture_output=True,text=True).stdout
 return [int(line.split(None,1)[0]) for line in out.splitlines() if str(WORK/script) in line or (' work/'+script) in line]

def alive(pid):
 state=subprocess.run(['ps','-o','stat=','-p',str(pid)],capture_output=True,text=True).stdout.strip()
 return bool(state) and not state.startswith('Z')

def read_status():
 try:s=json.loads(STATUS.read_text());s['age_s']=time.time()-s['time'];return s
 except (OSError,ValueError,KeyError):return None

def stop(pids,label,timeout):
 for pid in pids:
  say(f'stopping {label} pid {pid} (SIGTERM)');os.kill(pid,signal.SIGTERM)
 deadline=time.time()+timeout
 while time.time()<deadline and any(alive(p) for p in pids):time.sleep(.1)
 return [p for p in pids if alive(p)]

def run_tests():
 for test in TESTS:
  result=subprocess.run([sys.executable,test],cwd=BRIDGE,capture_output=True,text=True,timeout=120)
  if result.returncode:fail(f'{test} failed; nothing was stopped.\n'+result.stdout+result.stderr)
 say(f'{len(TESTS)} fake-hardware test files passed')

def show_changes():
 changed=[]
 for name in INSTALL:
  new=(BRIDGE/name).read_text();target=WORK/name
  old=target.read_text() if target.exists() else ''
  if old!=new:
   diff=list(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm='',n=0))
   changed.append(name);say(f'{name}: {"new file" if not old else str(sum(1 for d in diff if d[:1] in "+-" and d[:3] not in ("+++","---")))+" changed lines"}')
 if not changed:say('installed files already match this folder')
 return changed

def wrist_fresh(name):
 try:select_wrist_manifest(name,wrist_dirs(ROOT));return True
 except (RuntimeError,ValueError):return False

def ensure_wrist_publishers(dry_run):
 """Start capture-single for each wrist without a fresh stream; needs Terminal's camera permission."""
 out=subprocess.run(['ps','-axo','pid=,command='],capture_output=True,text=True).stdout
 for name,camera_id in WRIST_CAMERA_IDS.items():
  if wrist_fresh(name):say(f'{name}: already publishing fresh frames');continue
  running=[line.split(None,1)[0] for line in out.splitlines() if camera_id in line]
  if running:say(f'WARNING {name}: publisher pid {",".join(running)} is running but its frames are stale; not opening the camera twice');continue
  if dry_run:say(f'{name}: no fresh stream; would start {CAPTURE.name} {name}={camera_id}');continue
  if not CAPTURE.exists():
   say(f'building {CAPTURE} from {CAPTURE_SOURCE}')
   built=subprocess.run(['swiftc',str(CAPTURE_SOURCE),'-o',str(CAPTURE)],capture_output=True,text=True)
   if built.returncode:say('WARNING wrist cameras unavailable: swiftc failed: '+built.stderr.strip()[-400:]);return
  WRIST_STREAM.mkdir(parents=True,exist_ok=True)
  with (WRIST_STREAM/(name+'.log')).open('ab') as log:
   proc=subprocess.Popen([str(CAPTURE),str(WRIST_STREAM),f'{name}={camera_id}'],cwd=ROOT,stdout=log,stderr=log,start_new_session=True)
  deadline=time.time()+8
  while time.time()<deadline and proc.poll() is None and not wrist_fresh(name):time.sleep(.2)
  if wrist_fresh(name):say(f'{name}: publisher pid {proc.pid} streaming to {WRIST_STREAM}')
  else:
   log=(WRIST_STREAM/(name+'.log'));tail=log.read_text(errors='replace').strip().splitlines()[-3:] if log.exists() else []
   say(f'WARNING {name}: no fresh frames after 8 s (exit code {proc.poll()}). Publisher said: {" | ".join(tail) or "nothing"}. '
       'Camera access needs Terminal: open Terminal.app yourself and run ./restart-robot-server.sh --cameras-only')

def main():
 parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
 parser.add_argument('--dry-run',action='store_true',help='run tests and show what would change; stop nothing')
 parser.add_argument('--cameras-only',action='store_true',help='only start missing wrist-camera publishers (run from Terminal); the server is not touched')
 parser.add_argument('--no-wheels',action='store_true',help='start the owner without base drive (robot_move_base refused)')
 parser.add_argument('--no-wrist-cams',action='store_true',help='do not start wrist-camera publishers')
 parser.add_argument('--release-holding',action='store_true',help='allow stopping an owner that is holding motors (the arm will lose torque; support it first)')
 args=parser.parse_args()
 if not WORK.is_dir():fail(f'{WORK} not found; set XLEROBOT_WORK_ROOT')
 if args.cameras_only:
  ensure_wrist_publishers(False);print(json.dumps({'wrist_cameras_fresh':{n:wrist_fresh(n) for n in WRIST_CAMERA_IDS}},indent=2));return
 if not Path(PYTHON).exists():fail(f'{PYTHON} not found; set XLEROBOT_PYTHON')
 run_tests();changed=show_changes()
 owners=processes('gemma_hardware_owner.py');apis=processes('gemma_robot_tools.py');status=read_status()
 say(f'running owner pids {owners or "none"}, API pids {apis or "none"}')
 if status:
  enabled=status.get('enabled_motors') or [n for n,r in status.get('rows',{}).items() if r.get('Torque_Enable')==1]
  say(f"owner status: phase={status.get('phase')} stop_latched={status.get('stop_latched')} enabled={enabled} age={status['age_s']:.1f}s")
  if owners and enabled and not args.release_holding:
   fail('motors are holding '+', '.join(enabled)+'. Stopping the owner turns their torque off and the arm will drop. '
        'Support the arm, then call robot_stop (or rerun with --release-holding).')
 if args.no_wheels:OWNER_ARGS.remove('--wheels')
 if args.dry_run:
  if not args.no_wrist_cams:ensure_wrist_publishers(True)
  say('dry run: nothing stopped or installed');return
 # API first, so no new command reaches the owner while it shuts down.
 if stop(apis,'API',10):fail('API did not exit; owner left running')
 if owners:
  left=stop(owners,'hardware owner',20)
  if left:fail(f'hardware owner {left} did not exit within 20 s; not forcing it (it releases motors on exit). Inspect {OWNER_LOG}')
  final=read_status()
  if final and any(r.get('Torque_Enable')!=0 for r in final.get('rows',{}).values()):fail('old owner exited without confirming all motors released; inspect before restarting')
  say('old owner exited with all motors released')
 if changed:
  backup=WORK/'backups'/time.strftime('qwen-bridge-%Y%m%d-%H%M%S');backup.mkdir(parents=True)
  for name in INSTALL:
   if (WORK/name).exists():shutil.copy2(WORK/name,backup/name)
  for name in INSTALL:shutil.copy2(BRIDGE/name,WORK/name)
  say(f'installed {", ".join(changed)}; previous copies in {backup}')
 previous=read_status();previous_started=(previous or {}).get('started',0)
 if previous:(ROOT/'outputs').mkdir(exist_ok=True);(ROOT/'outputs/Gemma-Previous-Owner-Fault-Status.json').write_text(json.dumps(previous))
 with OWNER_LOG.open('ab') as log:
  owner=subprocess.Popen([PYTHON,str(WORK/'gemma_hardware_owner.py'),*OWNER_ARGS],cwd=ROOT,stdout=log,stderr=log,start_new_session=True)
 OWNER_RECORD.write_text(json.dumps({'pid':owner.pid,'started':time.time(),'args':OWNER_ARGS},indent=2))
 say(f'started hardware owner pid {owner.pid} {" ".join(OWNER_ARGS)}')
 deadline=time.time()+30
 while True:
  if owner.poll() is not None:fail(f'owner exited during startup; see {OWNER_LOG}')
  s=read_status()
  if s and s.get('started',0)>previous_started and s['age_s']<1 and s.get('phase')=='idle':break
  if time.time()>deadline:fail(f'no fresh owner status within 30 s; see {OWNER_LOG}')
  time.sleep(.1)
 rows=s.get('rows',{})
 if len(rows)!=16 or any(r.get('Torque_Enable')!=0 for r in rows.values()) or s.get('motor_writes')!=0 or s.get('stop_latched'):fail('fresh owner is not all-16 released with zero writes and STOP clear: '+json.dumps({k:s.get(k) for k in ('phase','motor_writes','stop_latched')}))
 if s.get('execution_profile')!='paddle-success-v1':fail('fresh owner is not running the paddle-success-v1 profile')
 if s.get('base_drive_supported') is not ('--wheels' in OWNER_ARGS):fail('fresh owner base_drive_supported does not match the requested --wheels setting')
 API_LOG.parent.mkdir(parents=True,exist_ok=True)
 env=dict(os.environ,XLEROBOT_PASSIVE_RECOVERY='1',XLEROBOT_OAK_RAW_DIR=OAK_RAW_DIR)
 with API_LOG.open('ab') as log:
  api=subprocess.Popen([PYTHON,str(WORK/'gemma_robot_tools.py')],cwd=ROOT,stdout=log,stderr=log,start_new_session=True,env=env)
 API_RECORD.write_text(json.dumps({'pid':api.pid,'started_at':time.time(),'log':str(API_LOG),'detached':True,'armed':False},indent=2))
 deadline=time.time()+20
 while True:
  if api.poll() is not None:fail(f'API exited during startup; see {API_LOG}')
  try:
   with socket.create_connection(('127.0.0.1',API_PORT),timeout=.5):break
  except OSError:pass
  if time.time()>deadline:fail(f'API not listening on 127.0.0.1:{API_PORT} within 20 s; see {API_LOG}')
  time.sleep(.2)
 if not args.no_wrist_cams:ensure_wrist_publishers(False)
 print(json.dumps({'owner_pid':owner.pid,'owner_session_started':s['started'],'execution_profile':s['execution_profile'],'phase':s['phase'],
                   'all16_released':True,'base_drive_supported':s.get('base_drive_supported'),'motor_writes':0,'stop_latched':False,'api_pid':api.pid,'api':f'https://127.0.0.1:{API_PORT}',
                   'relay':'unchanged','installed':changed,'wrist_cameras_fresh':{n:wrist_fresh(n) for n in WRIST_CAMERA_IDS}},indent=2))
 say('done. Motors are released; enable all six right-arm joints explicitly before any pickup move.')

if __name__=='__main__':main()
