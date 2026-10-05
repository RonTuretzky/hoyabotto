"""Supervised, parked, single-arm joint-space teaching session.

Explicit commands are <=6 physical degrees from the measured pose, with speed,
torque, health, travel, deadline, phone freshness and operator STOP checks.
No IK, learned-policy actions, wheels, calibration writes, or unattended loops.
"""
import argparse
import json
import secrets
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'xlerobot-farm/software'))
from farm.config import load_profile
from farm.adapters.robot_lerobot import LeRobotXLeRobot
from strict_servo_replies import guard_replies
from elbow_recovery_plan import recovery_target
from temperature_confirmation import confirm_temperature

def confirm_gripper_spike(row, phase, goal, calibration, attempts,
                          release, read_sample, write, sleep, record, guard):
    """Release/confirm a suspect gripper reading, then restore the same goal."""
    if row['Present_Temperature'] <= 55:
        return row, attempts
    if phase not in ('holding','moving') or row['Status'] or abs(row['Present_Load'])>250:
        raise RuntimeError('Right gripper temperature confirmation is not eligible')
    if attempts >= 2:
        raise RuntimeError('Right gripper temperature confirmation budget exhausted')
    record('gripper_temperature_trigger',row.copy())
    samples=[]
    def followup():
        guard()
        fresh=read_sample()
        record('gripper_temperature_confirmation',fresh.copy())
        samples.append(fresh)
        if abs(fresh['Present_Load'])>250:
            raise RuntimeError('Right gripper confirmation load limit')
        return {'temperature':fresh['Present_Temperature'], 'status':fresh['Status'],
                'torque':fresh['Torque_Enable']}
    def wait(seconds):
        guard();sleep(seconds);guard()
    confirm_temperature(row['Present_Temperature'],55,release,followup,wait)
    current=samples[-1]['Present_Position']
    if not calibration.range_min <= current <= calibration.range_max:
        raise RuntimeError('Released gripper outside saved range')
    if type(goal) is not int or not calibration.range_min <= goal <= calibration.range_max or abs(goal-current)>68:
        raise RuntimeError('Original gripper goal no longer within its bounded step')
    guard()
    write('Goal_Position',current)
    write('Torque_Enable',1)
    write('Lock',1)
    write('Goal_Position',goal)
    record('gripper_temperature_resumed',{'current':current,'original_goal':goal})
    resumed=samples[-1].copy()
    resumed.pop('Torque_Enable',None)  # Released-state readback remains in confirmation log.
    return resumed, attempts+1
from coherent_servo_telemetry import read_servo_telemetry

def plan_delta(current, delta, calibration, selected):
    if not isinstance(delta, dict) or not delta or not set(delta) <= set(selected):
        raise ValueError('Command must name only selected arm joints')
    target = {}
    for n, d in delta.items():
        if type(d) is not int or abs(d) > 68:
            raise ValueError('A joint step cannot exceed six physical degrees')
        c = calibration[n]
        t = current[n] + d
        if not c.range_min + 4 <= t <= c.range_max - 4:
            raise ValueError(f'{n}: target outside saved working range')
        target[n] = t
    return target

def save(path, obj):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(obj, indent=2))
    tmp.replace(path)

def camera_allowed(age, phase, config, starting_move=False):
    if not isinstance(age, (int,float)) or not 0 <= age:
        return False
    key = 'start_max_age_s' if starting_move or phase == 'starting' else 'motion_max_age_s' if phase in ('moving','elbow_recovery') else 'hold_max_age_s'
    return age <= config[key]

def enable_primed_goal(write, name, target, commit):
    """Prime before enable, then deliver that same goal to the active controller.

    Match LeRobot's Feetech enable sequence, including EEPROM locking. Some
    starts acknowledge the pre-enable register without starting the trajectory;
    the one post-enable write never changes or extends the checked target.
    Every write goes through the caller's readback verification.
    """
    commit(name, target)
    write(name, 'Torque_Enable', 1)
    write(name, 'Lock', 1)
    commit(name, target)

def elbow_trajectory(bus, name, target):
    """One canonical STS WritePosEx packet: acceleration/goal/time/speed.

    Source: FeetechServo/SMS_STS.cpp WritePosEx (registers 41..47).
    The caller must have checked the physical envelope before this function.
    """
    if name != 'right_arm_elbow_flex' or type(target) is not int or not 0<=target<=4095:
        raise ValueError('Invalid isolated elbow trajectory')
    expected={'Acceleration':10,'Goal_Position':target,'Goal_Time':0,'Goal_Velocity':100}
    payload=[10,target & 255,target >> 8,0,0,100,0]
    comm,error=bus.packet_handler.writeTxRx(bus.port_handler,bus.motors[name].id,41,7,payload)
    if not bus._is_comm_success(comm) or bus._is_error(error):
        raise RuntimeError('Elbow trajectory packet was not acknowledged')
    for field,value in expected.items():
        if int(bus.read(field,name,normalize=False,num_retry=2)) != value:
            raise RuntimeError(f'Elbow trajectory {field} was not verified')

def run(arm, recover_right_elbow=False):
    folder = ROOT / 'work/carton-session'
    folder.mkdir(exist_ok=True)
    config = json.loads((folder/'config.json').read_text())['camera']
    if not 0 < config['start_max_age_s'] <= config['motion_max_age_s'] <= config['hold_max_age_s'] <= 30:
        raise ValueError('Invalid camera grace configuration')
    token = secrets.token_urlsafe(24)
    stop = threading.Event()
    state = {'arm': arm, 'phase': 'starting', 'ok': False, 'started': time.time()}
    last_command = None
    lease = time.monotonic() + 180
    r = LeRobotXLeRobot(load_profile('paper-tray-v0').robot).robot
    bus = r.bus2 if arm == 'right' else r.bus1
    selected = list(r.right_arm_motors if arm == 'right' else r.left_arm_motors)
    old = {}; enabled = set(); goals = {}; pending = None
    temperature_confirmations = 0
    recovery_cameras = {}
    if recover_right_elbow:
        if arm != 'right': raise ValueError('Recovery is restricted to the right elbow')
        sys.path.insert(0,str(ROOT/'work/carton-visual-controller/software'))
        from carton.servo.vision import ManifestCamera
        for name, identity in (('head','0x12400005a39230'),('right_wrist','0x12200005a39230')):
            recovery_cameras[name] = ManifestCamera(ROOT/'work/robot-camera-stream'/f'{name}.json', identity, 1.0)
    history = open(folder / f'trace-{time.time_ns()}.jsonl', 'a', buffering=1)
    def read(n,f): return int(bus.read(f,n,normalize=False,num_retry=2))
    def write(n,f,v):
        bus.write(f,n,v,normalize=False,num_retry=2)
        if read(n,f) != v: raise RuntimeError(f'{n}: {f} write not verified')
    def camera_age():
        if config.get('source')=='robot':
            ages=[]
            for name in ('head',arm+'_wrist'):
                stamp=json.loads((ROOT/'work/robot-camera-stream'/f'{name}.json').read_text())['received_at']
                ages.append(time.time()-stamp)
            return max(ages) if all(a>=0 for a in ages) else -1
        stamp=json.loads((ROOT/'work/phone_camera/latest.json').read_text())['received_at']
        return time.time()-stamp
    def guard():
        if stop.is_set(): raise RuntimeError('Operator STOP')
        if time.monotonic() > lease: raise RuntimeError('Supervision lease expired')
        age=camera_age()
        state['camera_age_s']=round(age,2)
        state['camera_source']=config.get('source','phone')
        if not camera_allowed(age,state['phase'],config): raise RuntimeError('Camera grace interval exhausted')
        if state['phase']=='elbow_recovery':
            for camera in recovery_cameras.values(): camera.read()
    def sample(recovery=None):
        nonlocal temperature_confirmations
        guard()
        rows = {}
        for n in selected:
            row=read_servo_telemetry(bus,n)
            state['last_sample']={'motor':n,'time':time.time(),'row':row.copy()}
            history.write(json.dumps({'event':'motor_sample',**state['last_sample']})+'\n')
            if state['phase']=='elbow_recovery' and n=='right_arm_elbow_flex':
                for f in ('Torque_Enable','Torque_Limit','Goal_Position','Goal_Position_2','Goal_Velocity','Acceleration','Goal_Time','Lock'):
                    row[f]=read(n,f)
                history.write(json.dumps({'event':'elbow_recovery_sample','time':time.time(),'row':row})+'\n')
                if row['Torque_Enable'] != 1:
                    raise RuntimeError('Elbow torque enable cleared during recovery')
                if not 1 <= row['Torque_Limit'] <= 400:
                    raise RuntimeError('Elbow live torque limit invalid during recovery')
                if row['Goal_Position'] != goals[n]:
                    raise RuntimeError('Elbow live goal changed during recovery')
            c=r.calibration[n]
            if n=='right_arm_gripper' and state['phase'] in ('holding','moving') and row['Present_Temperature']>55:
                def confirmation_guard():
                    guard()
                    if pending and time.monotonic()>pending['deadline']:
                        raise RuntimeError('Original joint deadline expired during temperature confirmation')
                def release_gripper():
                    bus.disable_torque([n],num_retry=3)
                    if read(n,'Torque_Enable')!=0:
                        raise RuntimeError('Right gripper temperature-confirmation release not verified')
                def released_sample():
                    fresh=read_servo_telemetry(bus,n)
                    fresh['Torque_Enable']=read(n,'Torque_Enable')
                    return fresh
                def resume_write(field,value):
                    if field=='Torque_Enable':
                        write(n,'Torque_Limit',min(250,old[n]['Torque_Limit']))
                    write(n,field,value)
                def record_confirmation(event,value):
                    history.write(json.dumps({'event':event,'time':time.time(),'motor':n,'row':value})+'\n')
                row,temperature_confirmations=confirm_gripper_spike(row,state['phase'],goals[n],c,
                    temperature_confirmations,release_gripper,released_sample,resume_write,time.sleep,
                    record_confirmation,confirmation_guard)
            rows[n]=row
            if row['Status'] or row['Present_Temperature'] > 55 or abs(row['Present_Load']) > 500:
                raise RuntimeError(f'{n}: health limit or fault')
            if recovery and n==recovery[0]:
                if abs(row['Present_Load'])>400 or abs(row['Present_Velocity'])>200:
                    raise RuntimeError('Elbow recovery load or speed limit')
            lo,hi=(recovery[1],recovery[2]) if recovery and n==recovery[0] else (c.range_min,c.range_max)
            if not lo <= row['Present_Position'] <= hi:
                raise RuntimeError(f'{n}: outside verified travel envelope')
        state.update(time=time.time(), rows=rows, goals=goals, lease_remaining=round(lease-time.monotonic(),1))
        save(folder/'status.json',state);history.write(json.dumps(state)+'\n')
        return {n:rows[n]['Present_Position'] for n in selected}
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            if self.path != '/?token='+token: self.send_error(403);return
            body=("<!doctype html><meta name='viewport' content='width=device-width'><title>Carton test STOP</title>"
                "<style>body{font:24px system-ui;padding:40px}button{background:#b00;color:white;font-size:40px;padding:35px}</style>"
                "<h1>Carton arm test</h1><p>Wheels stay off. STOP releases the active arm.</p>"
                "<button onclick=\"fetch('/stop?token="+token+"',{method:'POST'}).then(()=>this.textContent='STOP REQUESTED')\">STOP — RELEASE ARM</button>").encode()
            self.send_response(200);self.send_header('Content-Type','text/html');self.end_headers();self.wfile.write(body)
        def do_POST(self):
            if self.path != '/stop?token='+token: self.send_error(403);return
            stop.set();self.send_response(200);self.end_headers();self.wfile.write(b'Stopping')
    server = ThreadingHTTPServer(('127.0.0.1',8768),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    save(folder/'access.json',{'url':'http://127.0.0.1:8768/?token='+token})
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    try:
        guard()
        for b in (r.bus1,r.bus2):
            guard_replies(b);b.connect(handshake=False)
            for n in b.motors:
                if b.read('Torque_Enable',n,normalize=False,num_retry=3)!=0:
                    raise RuntimeError(f'{n}: already enabled before this session')
        other = r.bus1 if arm=='right' else r.bus2
        other.disconnect(disable_torque=False)
        current = {}
        for n in selected:
            c=r.calibration[n]
            for f,v in [('Homing_Offset',c.homing_offset),('Min_Position_Limit',c.range_min),('Max_Position_Limit',c.range_max)]:
                if read(n,f)!=v: raise RuntimeError(f'{n}: calibration mismatch')
            if read(n,'Operating_Mode')!=0: raise RuntimeError(f'{n}: not position mode')
            if read(n,'Status') or read(n,'Present_Temperature')>55: raise RuntimeError(f'{n}: preflight health')
            current[n]=read(n,'Present_Position')
            old[n]={f:read(n,f) for f in ('Lock','Goal_Velocity','Goal_Time','Acceleration','Torque_Limit','P_Coefficient')}
        outside=[n for n in selected if not r.calibration[n].range_min<=current[n]<=r.calibration[n].range_max]
        recovery=None
        if outside:
            if not recover_right_elbow or outside != ['right_arm_elbow_flex']:
                raise RuntimeError('Out-of-range starting joint; physical repositioning required: '+', '.join(outside))
            n=outside[0];c=r.calibration[n]
            recovery=(n,recovery_target(current[n],c.range_min,c.range_max),current[n])
        for n in selected:
            write(n,'Lock',0);write(n,'Goal_Velocity',100);write(n,'Acceleration',10)
            write(n,'Torque_Limit',min(400 if not n.endswith('gripper') else 250,old[n]['Torque_Limit']))
            if n in ('right_arm_shoulder_lift','right_arm_elbow_flex','right_arm_gripper'): write(n,'P_Coefficient',32)
            if not recovery or n != recovery[0]: write(n,'Goal_Position',current[n])
        if recovery:
            n,target,start=recovery
            state['phase']='elbow_recovery';goals[n]=target
            enabled.add(n);enable_primed_goal(write,n,target,lambda n,t:elbow_trajectory(bus,n,t))
            end=time.monotonic()+3; stable=[]
            while time.monotonic()<end:
                cur=sample((n,start-int(12*4096/360),start+3))
                if abs(cur[n]-target)<=40 and cur[n]<=r.calibration[n].range_max: stable.append(cur[n])
                else: stable=[]
                if len(stable)>=3 and max(stable[-3:])-min(stable[-3:])<=2: break
                time.sleep(.06)
            else: raise RuntimeError('Elbow recovery did not settle')
            # Remain powered across the established small inward step and handoff.
            inward=plan_delta(cur,{n:-68},r.calibration,selected)[n]
            elbow_trajectory(bus,n,inward);goals[n]=inward
            end=time.monotonic()+3;stable=[]
            while time.monotonic()<end:
                cur=sample()
                if cur[n]<=r.calibration[n].range_max-32: stable.append(cur[n])
                else: stable=[]
                if len(stable)>=3 and max(stable[-3:])-min(stable[-3:])<=2: break
                time.sleep(.06)
            else: raise RuntimeError('Elbow could not establish a stable interior holding pose')
        for n in selected:
            if n in enabled: continue
            pos=read(n,'Present_Position');c=r.calibration[n]
            if not c.range_min<=pos<=c.range_max: raise RuntimeError(f'{n}: drifted before enable')
            write(n,'Goal_Position',pos);goals[n]=pos
            enabled.add(n);write(n,'Torque_Enable',1)
        state.update(phase='holding',ok=True)
        print('ARM READY: selected arm holds its measured pose; wheels, other arm, and head remain off.',flush=True)
        # Ignore leftover commands from an earlier session.
        command_path=folder/'command.json'
        if command_path.exists():last_command=json.loads(command_path.read_text()).get('id')
        while True:
            current=sample()
            if pending:
                if max(abs(current[n]-goals[n]) for n in pending['joints']) <= 24:
                    pending['stable']+=1
                    if pending['stable']>=3:
                        state.update(phase='holding',completed=pending['id']);pending=None
                else: pending['stable']=0
                if pending and time.monotonic()>pending['deadline']:raise RuntimeError('Requested joint step did not settle')
            else:
                if any(abs(current[n]-goals[n])>68 for n in selected):raise RuntimeError('Holding pose drifted more than six degrees')
            if command_path.exists():
                cmd=json.loads(command_path.read_text())
                if cmd.get('id') != last_command:
                    last_command=cmd.get('id');op=cmd.get('op')
                    if op=='stop':stop.set();guard()
                    if op=='hold': lease=time.monotonic()+180
                    elif op=='move':
                        if pending:raise RuntimeError('New move before previous step settled')
                        if not camera_allowed(camera_age(),state['phase'],config,starting_move=True):
                            state['last_rejected']={'id':last_command,'reason':'Waiting for a recent image before a new movement'}
                            continue
                        target=plan_delta(current,cmd.get('delta_ticks'),r.calibration,selected)
                        # One joint per visual experiment prevents coupled unobserved movement.
                        if len(target)!=1:raise RuntimeError('Only one joint may change per command')
                        guard()
                        for n,t in target.items():write(n,'Goal_Position',t);goals[n]=t
                        pending={'id':last_command,'joints':list(target),'stable':0,'deadline':time.monotonic()+4}
                        lease=time.monotonic()+180;state.update(phase='moving',accepted=last_command)
                    elif op!='hold':raise RuntimeError('Unknown command')
            time.sleep(.08)
    except BaseException as exc:
        state.update(phase='stopped',error=str(exc),ok=False)
        print('Session stopped: '+str(exc),flush=True)
    finally:
        errors=[]
        if bus.is_connected:
            for n in selected:
                try:
                    bus.disable_torque([n],num_retry=3)
                    if read(n,'Torque_Enable')!=0:raise RuntimeError('release not confirmed')
                    for f in ('Goal_Velocity','Goal_Time','Acceleration','Torque_Limit','P_Coefficient','Lock'):
                        if n in old:write(n,f,old[n][f])
                except Exception as exc: errors.append(f'{n}: {exc}')
        for b in (r.bus1,r.bus2):
            if b.is_connected:b.disconnect(disable_torque=False)
        state.update(released=not errors,release_errors=errors,finished=time.time())
        save(folder/'status.json',state);history.close();server.shutdown();server.server_close()
        print(json.dumps({k:v for k,v in state.items() if k!='rows'}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arm',choices=['left','right'],required=True)
    p.add_argument('--recover-right-elbow',action='store_true');a=p.parse_args();run(a.arm,a.recover_right_elbow)
