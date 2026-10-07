"""Automatic arm calibration (LeRobot PR #3282) as a robot API job.

Started by the robot_auto_calibrate tool through remote_admin; runs detached because the hardware owner,
which holds both servo ports, has to stop for the duration. Steps:
  1. precheck: owner idle with every motor released, phone camera fresh (someone must be able to watch)
  2. stop the hardware owner (it releases on exit) so the calibration can open the ports
  3. run the pinned runner scripts/carton_robot/upstream_pr3282_calibration.py --execute --install:
     the unchanged upstream sweep, then validation; the result is installed only if it validates
  4. if it was not installed, write the previous offsets/limits back into the six servos (torque off,
     read back) so file and hardware agree again; if torque release could not be verified, stop there
     and ask for 12 V off
  5. restart the robot server (owner + API) so the new or restored calibration is in use
robot_stop interrupts a running sweep (SIGINT to the runner: the upstream routine makes the motors limp).
Usage: python calibration_job.py <job.json>
"""
import json,os,subprocess,sys,time
from pathlib import Path

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
REGISTERS={'homing_offset':'Homing_Offset','range_min':'Min_Position_Limit','range_max':'Max_Position_Limit'}
VELOCITIES=(200,300)


def precheck(status,camera,now=None,need_camera=True):
    """Blockers that must be clear before a powered sweep (or, without the camera check, a register restore)."""
    now=time.time() if now is None else now;blockers=[]
    if status:
        if status.get('enabled_motors') or any(r.get('Torque_Enable')==1 for r in (status.get('rows') or {}).values()):
            blockers.append('motors are powered: release the arm first (support it, then STOP)')
        if status.get('phase')=='moving':blockers.append('a motion is running')
    stamp=(camera or {}).get('received_at')
    if need_camera and (type(stamp) not in (int,float) or not 0<=now-stamp<10):
        blockers.append('phone camera is not fresh: nobody can watch the sweep')
    return blockers


def restore_registers(bus,arm,before):
    """Write the pre-run offsets/limits (and position mode) back into the arm's six servos with torque off; verify."""
    report={'restored':False,'joints':{}}
    entries={j:before.get(f'{arm}_arm_{j}') for j in JOINTS}
    missing=[j for j,e in entries.items() if not e]
    if missing:report['error']='previous calibration has no entry for '+', '.join(missing);return report
    bus.disable_torque(num_retry=3)
    for j in JOINTS:
        if bus.read('Torque_Enable',j,normalize=False)!=0:report['error']=f'{j}: torque still on; not writing';return report
    for j in JOINTS:
        bus.write('Operating_Mode',j,0,normalize=False,num_retry=3)
        for key,reg in REGISTERS.items():bus.write(reg,j,int(entries[j][key]),normalize=False,num_retry=3)
    ok=True
    for j in JOINTS:
        got={key:bus.read(reg,j,normalize=False) for key,reg in REGISTERS.items()};got['operating_mode']=bus.read('Operating_Mode',j,normalize=False)
        want={key:int(entries[j][key]) for key in REGISTERS};want['operating_mode']=0
        report['joints'][j]={'expected':want,'readback':got,'match':got==want};ok&=got==want
    report['restored']=ok
    return report


def read_json(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return None


def run(job_path,*,stop_owner=None,run_runner=None,restore=None,restart=None,now=time.time):
    job_path=Path(job_path);job=json.loads(job_path.read_text())
    work=Path(job['work']);checkout=Path(job['checkout']);software=checkout/'software'
    def save(**kw):
        job.update(kw);job_path.write_text(json.dumps(job,indent=2))
    def say(msg):print(time.strftime('%H:%M:%S ')+msg,flush=True)
    def finish(state,**kw):
        save(state=state,finished=now(),**kw);say(f'job {state}');return job
    status=read_json(work/'gemma-hardware-session/status.json');camera=read_json(work/'phone_camera/latest.json')
    if job.get('action')=='restore':
        # No motion: write the saved file's offsets/limits/position mode into the arm's servos, then restart.
        blockers=precheck(status,camera,now(),need_camera=False)
        if blockers:say('refused: '+'; '.join(blockers));return finish('refused',blockers=blockers)
        say(f"stopping the hardware owner to restore the {job['arm']} arm's saved calibration into its servos (no motion)")
        if not (stop_owner or _stop_owner)(work):return finish('failed',error='hardware owner did not stop; nothing changed')
        save(phase='restoring')
        report=(restore or _restore)(software,job['arm'],None)
        say('saved calibration written and verified' if report.get('restored') else 'RESTORE FAILED: '+str(report.get('error') or report))
        say('restarting the robot server (owner + API)')
        restart_code=(restart or _restart)(checkout)
        return finish('succeeded' if report.get('restored') and restart_code==0 else 'failed',outcome='restored' if report.get('restored') else 'restore_failed',summary={'restore':report},restart_exit=restart_code)
    blockers=precheck(status,camera,now())
    if blockers:say('refused: '+'; '.join(blockers));return finish('refused',blockers=blockers)
    say(f"stopping the hardware owner so the {job['arm']} arm calibration can open the servo ports")
    if not (stop_owner or _stop_owner)(work):return finish('failed',error='hardware owner did not stop; nothing moved')
    evidence=work/'calibration-runs'/f"{job['arm']}-{time.strftime('%Y%m%d-%H%M%S')}";save(evidence=str(evidence),phase='sweeping')
    say(f"running PR #3282 calibration on the {job['arm']} arm at velocity {job['velocity']} (STOP interrupts it)")
    code=(run_runner or _run_runner)(software,job,evidence,save)
    result=read_json(evidence/'result.json') or {}
    save(phase='checking',runner_exit=code,result=result)
    summary={'routine_completed':result.get('routine_completed'),'full_range_validated':result.get('full_range_validated'),
             'installed':result.get('installed'),'problems':result.get('problems'),'error':result.get('error'),'evidence':str(evidence)}
    if result.get('installed'):
        say('calibration validated and installed')
        outcome='installed'
    elif result.get('release_verified') is False and result:
        say(f"SWITCH THE {job['arm'].upper()} ARM 12 V OFF: torque release could not be verified. The server is NOT restarted.")
        return finish('failed',outcome='release_unverified',summary=summary)
    elif (evidence/'calibration-before.json').exists() and 'upstream_return' in result:
        say('result not installed; restoring the previous offsets/limits into the servos (torque off, no motion)')
        report=(restore or _restore)(software,job['arm'],read_json(evidence/'calibration-before.json'))
        summary['restore']=report
        outcome='not_installed_restored' if report.get('restored') else 'not_installed_restore_failed'
        say('previous calibration restored and verified' if report.get('restored') else 'RESTORE FAILED: file and servos may disagree; '+str(report.get('error')))
    else:
        outcome='not_started'
        say('the sweep did not start (preflight or setup failed); servo settings were not changed')
    say('restarting the robot server (owner + API) with the current calibration')
    restart_code=(restart or _restart)(checkout)
    return finish('succeeded' if outcome=='installed' and restart_code==0 else 'failed',outcome=outcome,summary=summary,restart_exit=restart_code)


def _stop_owner(work):
    """SIGTERM the hardware owner (it releases every motor on exit) and confirm it is gone and released."""
    script=str(work/'gemma_hardware_owner.py')
    out=subprocess.run(['ps','-axo','pid=,args='],capture_output=True,text=True).stdout
    pids=[int(l.split(None,1)[0]) for l in out.splitlines() if script in l or ' work/gemma_hardware_owner.py' in l]
    for pid in pids:os.kill(pid,15)
    deadline=time.time()+20
    def alive(pid):
        st=subprocess.run(['ps','-o','stat=','-p',str(pid)],capture_output=True,text=True).stdout.strip()
        return bool(st) and not st.startswith('Z')
    while time.time()<deadline and any(alive(p) for p in pids):time.sleep(.2)
    if any(alive(p) for p in pids):return False
    status=read_json(work/'gemma-hardware-session/status.json') or {}
    return all(r.get('Torque_Enable')==0 for r in (status.get('rows') or {}).values())


def _run_runner(software,job,evidence,save):
    py=software/'.venv/bin/python'
    cmd=[str(py if py.exists() else sys.executable),str(software/'scripts/carton_robot/upstream_pr3282_calibration.py'),'--arm',job['arm'],
         '--velocity',str(job['velocity']),'--execute','--clearance-confirmed','--install','--backup-dir',str(evidence)]
    proc=subprocess.Popen(cmd,cwd=str(software),stdin=subprocess.PIPE,text=True,start_new_session=True)
    save(runner_pid=proc.pid)
    proc.communicate('yes\n')
    return proc.returncode


def _restore(software,arm,before):
    sys.path.insert(0,str(software));sys.path.insert(0,str(software/'scripts'))
    from carton_robot.guarded_pr3282_calibration import PORTS,CAL,install_calibration_reply_guard
    if before is None:before=read_json(CAL)  # restore action: the live saved calibration file
    from carton_robot.servo_ownership import ServoOwnership
    from farm.vendor.autocal.workflow import FeetechMotorsBus,SO_FOLLOWER_MOTORS
    port=PORTS[0 if arm=='left' else 1];lock=ServoOwnership(PORTS).acquire()
    bus=FeetechMotorsBus(port=port,motors=SO_FOLLOWER_MOTORS.copy())
    try:
        bus.connect(handshake=False);install_calibration_reply_guard(bus)
        return restore_registers(bus,arm,before or {})
    except Exception as e:return {'restored':False,'error':f'{type(e).__name__}: {e}'}
    finally:
        if bus.is_connected:bus.disconnect(disable_torque=False)
        lock.close()


def _restart(checkout):
    return subprocess.run(['bash',str(checkout/'restart-robot-server.sh')],cwd=str(checkout),stdin=subprocess.DEVNULL).returncode


if __name__=='__main__':run(sys.argv[1])
