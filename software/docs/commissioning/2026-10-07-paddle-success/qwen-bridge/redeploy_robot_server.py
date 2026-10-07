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
INSTALL=['calibration_job.py','remote_admin.py','wheel_pulse_executor.py','paddle_joint_executor.py','paddle_segments.py','paddle_camera_gate.py','gemma_hardware_owner.py','gemma_direct_client.py','gemma_robot_tools.py','wrist_cameras.py','paddle-procedure.json','restart_gemma_owner_released.py']
TESTS=['test_both_arms.py','test_calibration_job.py','test_soft_release.py','test_remote_admin.py','test_contact_guard.py','test_continuous_motion.py','test_wheel_pulse.py','test_paddle_joint_executor.py','test_paddle_segments.py','test_paddle_camera_gate.py','test_paddle_owner.py','test_paddle_client.py','test_paddle_stop_recovery.py','test_gemma_hardware_owner.py','test_wrist_cameras.py']
OWNER_ARGS=['--both-arms','--paddle-profile','--wheels'];  # an arm whose calibration mismatches stays read-only
API_PORT=1241
WRIST_STREAM=WORK/'wrist-camera-stream';CAPTURE=WORK/'capture-single'
CAPTURE_SOURCE=BRIDGE.parents[2]/'session-archive-2026-10-05/capture-single.swift'  # software/docs/session-archive-…
sys.path.insert(0,str(BRIDGE))
from wrist_cameras import WRIST_CAMERA_IDS,IDENTITY_VERIFIED,CONFIG as WRIST_CONFIG,select_wrist_manifest,wrist_dirs,resolve_ids,configure as configure_wrist_ids
configure_wrist_ids(ROOT)

CAMERA_REPORT=None  # list collecting camera-setup messages while setup_wrist_cameras runs
def say(message):
 print('[redeploy] '+message,flush=True)
 try:
  with (WORK/'redeploy.log').open('a') as f:f.write(time.strftime('%Y-%m-%d %H:%M:%S ')+message+'\n')
 except OSError:pass
 if CAMERA_REPORT is not None:CAMERA_REPORT.append(message)
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

def camera_processes(camera_id):
 """Native capture publishers (executable named capture*) that were started with this camera ID."""
 out=subprocess.run(['ps','-axo','pid=,args='],capture_output=True,text=True).stdout;found=[]
 for line in out.splitlines():
  parts=line.split()
  if len(parts)>2 and Path(parts[1]).name.startswith('capture') and any(a.endswith('='+camera_id) for a in parts[2:]) and int(parts[0])!=os.getpid():found.append(int(parts[0]))
 return found

def setup_wrist_cameras(dry_run):
 global CAMERA_REPORT
 CAMERA_REPORT=[]
 try:return _setup_wrist_cameras(dry_run)
 except Exception as e:say(f'WARNING wrist camera setup error: {type(e).__name__}: {e}');return False
 finally:
  report={'time':time.time(),'dry_run':dry_run,'messages':CAMERA_REPORT,'ids':dict(WRIST_CAMERA_IDS),'identity_verified':dict(IDENTITY_VERIFIED),'fresh':{n:wrist_fresh(n) for n in WRIST_CAMERA_IDS}}
  CAMERA_REPORT=None
  if not dry_run:
   try:(WORK/'wrist-camera-setup.json').write_text(json.dumps(report,indent=2))
   except OSError:pass

def _setup_wrist_cameras(dry_run):
 """Detect the wrist cameras, save their IDs for the API, and make sure each one is streaming.
 Returns True when the saved IDs changed (a running API must restart to pin them). Never fatal."""
 if not CAPTURE.exists():
  if dry_run:say(f'would build {CAPTURE} from {CAPTURE_SOURCE}');return False
  say(f'building {CAPTURE} from {CAPTURE_SOURCE}')
  built=subprocess.run(['swiftc',str(CAPTURE_SOURCE),'-o',str(CAPTURE)],capture_output=True,text=True)
  if built.returncode:say('WARNING wrist cameras unavailable: swiftc failed: '+built.stderr.strip()[-400:]);return False
 try:listed=json.loads(subprocess.run([str(CAPTURE),'--list'],capture_output=True,text=True,timeout=15).stdout or '[]')
 except Exception as e:say(f'WARNING wrist cameras unavailable: could not list cameras: {e}');return False
 say('cameras this Mac sees: '+(', '.join(f"{d.get('name')} [{d.get('camera_id')}]" for d in listed) or 'none'))
 ids,verified,missing=resolve_ids(listed)
 changed=any(n in ids and (ids[n]!=WRIST_CAMERA_IDS[n] or verified[n]!=IDENTITY_VERIFIED[n]) for n in WRIST_CAMERA_IDS)
 for n in WRIST_CAMERA_IDS:
  if n in missing:say(f'WARNING {n}: no camera available for it (expected {WRIST_CAMERA_IDS[n]})')
  elif ids[n]!=WRIST_CAMERA_IDS[n]:say(f'{n}: ID changed {WRIST_CAMERA_IDS[n]} -> {ids[n]}; left/right auto-assigned and marked unverified')
 if changed and not dry_run:
  WRIST_CAMERA_IDS.update(ids);IDENTITY_VERIFIED.update(verified)
  (WORK/WRIST_CONFIG).write_text(json.dumps({**{n:{'camera_id':WRIST_CAMERA_IDS[n],'identity_verified':IDENTITY_VERIFIED[n]} for n in WRIST_CAMERA_IDS},'detected_at':time.time(),'available':listed},indent=2))
  say(f'saved wrist camera IDs to {WORK/WRIST_CONFIG}')
 for name in WRIST_CAMERA_IDS:
  if name in missing:continue
  camera_id=ids[name]
  if wrist_fresh(name):say(f'{name}: already streaming');continue
  if dry_run:say(f'{name}: would start {CAPTURE.name} {name}={camera_id}');continue
  for pid in camera_processes(camera_id):  # a stale camera-only process holding this device
   say(f'{name}: stopping stale camera process {pid}');os.kill(pid,signal.SIGTERM);time.sleep(1)
  WRIST_STREAM.mkdir(parents=True,exist_ok=True)
  with (WRIST_STREAM/(name+'.log')).open('ab') as log:
   proc=subprocess.Popen([str(CAPTURE),str(WRIST_STREAM),f'{name}={camera_id}'],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  deadline=time.time()+8
  while time.time()<deadline and proc.poll() is None and not wrist_fresh(name):time.sleep(.2)
  if wrist_fresh(name):say(f'{name}: streaming (pid {proc.pid}){"" if IDENTITY_VERIFIED[name] else ", left/right unverified"}')
  else:
   log=WRIST_STREAM/(name+'.log');tail=log.read_text(errors='replace').strip().splitlines()[-3:] if log.exists() else []
   if proc.poll() is None:proc.terminate()
   say(f'WARNING {name}: no frames after 8 s (exit code {proc.poll()}): {" | ".join(tail) or "no output"}')
 return changed and not dry_run

def start_api():
 API_LOG.parent.mkdir(parents=True,exist_ok=True)
 env=dict(os.environ,XLEROBOT_PASSIVE_RECOVERY='1',XLEROBOT_OAK_RAW_DIR=OAK_RAW_DIR)
 with API_LOG.open('ab') as log:
  api=subprocess.Popen([PYTHON,str(WORK/'gemma_robot_tools.py')],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,env=env)
 API_RECORD.write_text(json.dumps({'pid':api.pid,'started_at':time.time(),'log':str(API_LOG),'detached':True,'armed':False},indent=2))
 deadline=time.time()+20
 while True:
  if api.poll() is not None:fail(f'API exited during startup; see {API_LOG}')
  try:
   with socket.create_connection(('127.0.0.1',API_PORT),timeout=.5):return api
  except OSError:pass
  if time.time()>deadline:fail(f'API not listening on 127.0.0.1:{API_PORT} within 20 s; see {API_LOG}')
  time.sleep(.2)

def main():
 parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
 parser.add_argument('--dry-run',action='store_true',help='run tests and show what would change; stop nothing')
 parser.add_argument('--cameras-only',action='store_true',help='only start missing wrist-camera publishers (run from Terminal); the server is not touched')
 parser.add_argument('--right-arm-only',action='store_true',help='only the right arm is movable (the left stays read-only)')
 parser.add_argument('--no-wheels',action='store_true',help='start the owner without base drive (robot_move_base refused)')
 parser.add_argument('--no-wrist-cams',action='store_true',help='do not start wrist-camera publishers')
 parser.add_argument('--release-holding',action='store_true',help='allow stopping an owner that is holding motors (the arm will lose torque; support it first)')
 args=parser.parse_args()
 if not WORK.is_dir():fail(f'{WORK} not found; set XLEROBOT_WORK_ROOT')
 if args.cameras_only:
  if setup_wrist_cameras(False):
   apis=processes('gemma_robot_tools.py')
   if apis:
    say('restarting the API so it uses the new wrist camera IDs (no motors involved)')
    if stop(apis,'API',10):fail('API did not exit')
    start_api()
  record_deploy('cameras-only')
  print(json.dumps({'wrist_cameras_fresh':{n:wrist_fresh(n) for n in WRIST_CAMERA_IDS},'wrist_camera_ids':WRIST_CAMERA_IDS,'identity_verified':IDENTITY_VERIFIED},indent=2));return
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
 if args.right_arm_only:OWNER_ARGS[OWNER_ARGS.index('--both-arms')]='--right-arm-only'
 if args.dry_run:
  if not args.no_wrist_cams:setup_wrist_cameras(True)
  say('dry run: nothing stopped or installed');return
 # API first, so no new command reaches the owner while it shuts down.
 if stop(apis,'API',10):fail('API did not exit; owner left running')
 if owners:
  left=stop(owners,'hardware owner',20)
  if left:fail(f'hardware owner {left} did not exit within 20 s; not forcing it (it releases motors on exit). Inspect {OWNER_LOG}')
  final=read_status()
  if final and any(r.get('Torque_Enable')!=0 for r in final.get('rows',{}).values()):fail('old owner exited without confirming all motors released; inspect before restarting')
  say('old owner exited with all motors released')
 backup=None
 if changed:
  backup=WORK/'backups'/time.strftime('qwen-bridge-%Y%m%d-%H%M%S');backup.mkdir(parents=True)
  for name in INSTALL:
   if (WORK/name).exists():shutil.copy2(WORK/name,backup/name)
  for name in INSTALL:shutil.copy2(BRIDGE/name,WORK/name)
  say(f'installed {", ".join(changed)}; previous copies in {backup}')
 try:s,owner,api=bring_up(args)
 except SystemExit:
  if not changed:raise
  say('ROLLBACK: the new version did not come up; restoring the previous files and restarting them')
  for pids,label in ((processes('gemma_robot_tools.py'),'API'),(processes('gemma_hardware_owner.py'),'hardware owner')):
   if stop(pids,label,20):fail(f'{label} did not exit during rollback; inspect before restarting')
  for name in INSTALL:
   if (backup/name).exists():shutil.copy2(backup/name,WORK/name)
  s,owner,api=bring_up(args)
  record_deploy('rolled-back')  # keep the checkout known so a fix can be deployed remotely
  say(f'ROLLBACK complete: the previous version is running again; the failed attempt is in {WORK/"redeploy.log"}');sys.exit(1)
 record_deploy('restart')
 print(json.dumps({'owner_pid':owner.pid,'owner_session_started':s['started'],'execution_profile':s['execution_profile'],'phase':s['phase'],
                   'all16_released':True,'base_drive_supported':s.get('base_drive_supported'),'motor_writes':0,'stop_latched':False,'api_pid':api.pid,'api':f'https://127.0.0.1:{API_PORT}',
                   'relay':'unchanged','installed':changed,'wrist_cameras_fresh':{n:wrist_fresh(n) for n in WRIST_CAMERA_IDS},'wrist_identity_verified':IDENTITY_VERIFIED},indent=2))
 say('done. Motors are released; enable all six right-arm joints explicitly before any pickup move.')

def bring_up(args):
 """Start the owner from the installed files, check it, set up the wrist cameras, start the API."""
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
 if not args.no_wrist_cams:setup_wrist_cameras(False)  # before the API, which loads the detected IDs at startup
 api=start_api()

 return s,owner,api

def record_deploy(mode):
 """Remember which checkout/commit is deployed, so /admin/deploy can update and restart it remotely."""
 top=subprocess.run(['git','-C',str(BRIDGE),'rev-parse','--show-toplevel'],capture_output=True,text=True).stdout.strip()
 head=subprocess.run(['git','-C',str(BRIDGE),'rev-parse','HEAD'],capture_output=True,text=True).stdout.strip()
 if top:(WORK/'deploy.json').write_text(json.dumps({'checkout':top,'head':head,'mode':mode,'time':time.time()},indent=2))

if __name__=='__main__':main()
