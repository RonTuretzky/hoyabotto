"""Single exclusive hardware owner, initially torque-off; no geometry planner."""
import json,time,math,threading,signal,os,sys
from pathlib import Path
from direct_joint_executor import DirectJointExecutor
from gemma_control_limits import SOFTWARE_TEMPERATURE_LIMIT_C
from wheel_pulse_executor import WheelPulseExecutor,WHEELS
PHONE_CAMERA=Path('/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera/latest.json')

DIAGNOSTIC_REGISTERS=['Torque_Enable','Operating_Mode','Goal_Position','Goal_Time','Goal_Velocity','Acceleration','Torque_Limit','Max_Torque_Limit','Max_Temperature_Limit','P_Coefficient','I_Coefficient','D_Coefficient','CW_Dead_Zone','CCW_Dead_Zone','Minimum_Startup_Force','Protection_Current','Protective_Torque','Protection_Time','Overload_Torque','Over_Current_Protection_Time','Unloading_Condition','Lock']

class HardwareOwner:
 def __init__(self,buses,calibration,read_telemetry,clock=time.monotonic,wall=time.time,read_only=False,position_scope=None,paddle_profile=False,camera_metadata=None,wheels=False):
  self.read_only=read_only
  self.paddle_profile=paddle_profile
  self.motion_count=0
  if camera_metadata is None:camera_metadata=lambda:json.loads(PHONE_CAMERA.read_text())
  self.camera_metadata=camera_metadata
  if paddle_profile:
   from paddle_camera_gate import PaddleCameraGate
   self.camera_gate=PaddleCameraGate(camera_metadata,clock=clock,wall=wall)
  if paddle_profile and (read_only or not position_scope or any(not n.startswith("right_arm_") for n in position_scope)):raise ValueError("Pickup profile requires explicit right-arm scope")
  self.buses=buses;self.cal=calibration;self.telemetry=read_telemetry;self.clock=clock;self.wall=wall
  self.names=[n for b in buses for n in b.motors];self.by_name={n:b for b in buses for n in b.motors}
  all_position_names=[n for n in self.names if not n.startswith('base_')]
  if position_scope is not None and (not position_scope or not set(position_scope)<=set(all_position_names)):raise ValueError('Invalid explicit position scope')
  self.position_names=all_position_names if position_scope is None else list(position_scope)
  self.commandable_names=set() if read_only else set(self.names if position_scope is None else self.position_names)
  self.ranges={n:[self.cal[n].range_min,self.cal[n].range_max] for n in self.position_names}
  # Base drive: guarded velocity pulses only (wheel_pulse_executor); wheels never join the enabled/hold set.
  self.wheel_names=[n for n in WHEELS if n in self.names] if wheels and not read_only else []
  if wheels and not read_only and len(self.wheel_names)!=2:raise ValueError('Base drive requires both wheel motors on the owner buses')
  self.enabled=set();self.old={};self.goals={};self.rows={};self.limits={};self.last_tick=clock();self.lease=clock()+30
  self.started=wall();self.engine=None;self.current_command=None
  self.state={'started':self.started,'control_mode':'direct_joint','hardware_server':True,'phase':'idle','ok':True,'operator_armed':not read_only,'read_only':read_only,'camera_supervision_ok':True,'camera_supervision_required':paddle_profile,'supportsselectedjoints':self.position_names,'supported_motors':self.names,'commandable_motors':sorted(self.commandable_names),'read_only_motors':[n for n in self.names if n not in self.commandable_names],'ranges':self.ranges,'capabilities':['read','enable_motors','direct_joint','stop','release'],'pickup_required_enabled_motors':self.position_names if paddle_profile else [],'pickup_motion_segment_budget':None,'pickup_idle_hold_seconds':120 if paddle_profile else 30,'execution_profile':'paddle-success-v1' if paddle_profile else 'legacy-direct','base_drive_supported':bool(self.wheel_names),'base_drive_limits':{'max_wheel_m_s':.02,'max_duration_s':3.0} if self.wheel_names else None,'motor_writes':0,'stop_latched':False,'stop_count':0,'last_stop':None,'software_temperature_limit_c':SOFTWARE_TEMPERATURE_LIMIT_C}
 def read(self,n,f):return int(self.by_name[n].read(f,n,normalize=False,num_retry=2))
 def write(self,n,f,v):
  self.by_name[n].write(f,n,v,normalize=False,num_retry=2);self.state['motor_writes']+=1
  actual=self.read(n,f)
  self.state.setdefault('last_write_readbacks',{}).setdefault(n,{})[f]={'requested':v,'readback':actual,'time':self.wall()}
  if actual!=v:raise RuntimeError(n+': '+f+' readback mismatch')
 def inspect(self):
  for n in self.names:
   if self.read(n,'Torque_Enable')!=0:raise RuntimeError(n+': already powered before hardware-owner startup')
   self.limits[n]=[self.read(n,'Min_Position_Limit'),self.read(n,'Max_Position_Limit')]
   if n in self.cal:
    c=self.cal[n]
    expected={'Homing_Offset':c.homing_offset,'Min_Position_Limit':c.range_min,'Max_Position_Limit':c.range_max}
    actual={f:self.read(n,f) for f in expected}
    if actual!=expected:
     self.state.setdefault('calibration_mismatches',{})[n]={'expected':expected,'actual':actual}
     if n in self.commandable_names:raise RuntimeError(n+': saved calibration differs from hardware')
  self.state['released_register_diagnostics']={n:self.register_diagnostics(n) for n in self.names if n.endswith('gripper')}
  self.poll()
  n='right_arm_gripper'
  if n in self.by_name and not self.paddle_profile:
   coherent=dict(self.rows[n])
   independent=self.read(n,'Present_Temperature')
   self.state['independent_temperature_check']={'motor':n,'time':self.wall(),'register':63,'length':1,'coherent_temperature':coherent['Present_Temperature'],'independent_temperature':independent,'coherent_evidence':coherent.get('coherent_read_evidence'),'independent_reply':getattr(self.by_name[n],'last_reply_evidence',None),'matches':independent==coherent['Present_Temperature'],'motor_writes':0}
 def register_diagnostics(self,n):
  return {'captured_at':self.wall(),'registers':{f:self.read(n,f) for f in DIAGNOSTIC_REGISTERS},'read_only':True}
 def read_telemetry(self,n):
  # Retry one transport corruption/timeout only in a fully observed released idle scope.
  idle_released=(not self.enabled and not (self.engine and self.engine.active)
                 and len(self.rows)==len(self.names)
                 and all(row.get('Torque_Enable')==0 and type(row.get('captured_at')) in (int,float)
                         and 0<=self.wall()-row['captured_at']<=1 for row in self.rows.values()))
  for attempt in range(2 if idle_released else 1):
   try:
    row=self.telemetry(self.by_name[n],n)
    evidence=getattr(self.by_name[n],'last_telemetry_sample',None)
    if evidence is not None:row['coherent_read_evidence']=evidence
    if attempt:
     self.state['idle_read_recovery']['recovered_count']+=1
     self.state['idle_read_recovery']['events'][-1]['outcome']='recovered_on_retry'
     self.state['idle_read_recovery']['events'][-1]['recovered_at']=self.wall()
    return row
   except RuntimeError as exc:
    transport_failure=str(exc).startswith('Coherent servo read communication failure: -7')
    if not idle_released or not transport_failure:raise
    report=self.state.setdefault('idle_read_recovery',{'failure_count':0,'recovered_count':0,'events':[],'max_attempts':2,'powered_retries':False})
    report['failure_count']+=1
    report['events'].append({'time':self.wall(),'motor':n,'attempt':attempt+1,'outcome':'retry_pending' if attempt==0 else 'retry_failed','diagnostic':str(exc),'transaction':getattr(self.by_name[n],'last_telemetry_failure',None)})
    report['events']=report['events'][-16:]
    if attempt:raise
    time.sleep(.01)

 def poll(self):
  now=self.clock()
  if self.engine and self.engine.active and not 0<=now-self.last_tick<=(1.0 if self.paddle_profile else .2):raise RuntimeError('Motor-owner watchdog expired')
  poll_elapsed=now-self.last_tick
  self.last_tick=now
  for n in self.names:
   row=self.read_telemetry(n);row['Torque_Enable']=self.read(n,'Torque_Enable');row['Operating_Mode']=self.read(n,'Operating_Mode');row['captured_at']=self.wall();row['firmware_position_limits']=self.limits.get(n);self.rows[n]=row
   if n in self.enabled:
    failures=[('Torque_Enable',row['Torque_Enable'],'must equal',1),('Status',row['Status'],'must equal',0),*([('Present_Temperature',row['Present_Temperature'],'must be <=',SOFTWARE_TEMPERATURE_LIMIT_C)] if not self.paddle_profile else []),('Present_Load',row['Present_Load'],'absolute must be <=',500 if n.endswith('gripper') or not self.paddle_profile else 800)]
    for field,value,rule,limit in failures:
     failed=(value!=limit if field in ('Torque_Enable','Status') else value>limit if field=='Present_Temperature' else abs(value)>limit)
     if failed:
      self.state['health_fault']={'motor':n,'field':field,'value':value,'rule':rule,'limit':limit,'captured_at':row['captured_at']}
      raise RuntimeError(f'{n}: {field}={value}; {rule} {limit}')
    if self.paddle_profile and not 100<=row['Present_Voltage']<=140:raise RuntimeError(n+': pickup supply voltage outside 10..14V')
    if n in self.ranges and not self.ranges[n][0]<=row['Present_Position']<=self.ranges[n][1]:raise RuntimeError(n+': outside saved travel range')
    if not(self.engine and self.engine.active and n in self.engine.joints) and abs(row['Present_Position']-self.goals[n])>(96 if self.paddle_profile else 68):raise RuntimeError(n+': uncommanded holding drift')
  if self.enabled and self.clock()>self.lease:raise RuntimeError('Command heartbeat expired')
  camera_ready=True
  if self.paddle_profile:
   camera_ready=self.camera_gate.update(holding=bool(self.enabled)) if self.enabled else True
   self.state.update(camera_supervision_ok=camera_ready,camera_pause_active=not camera_ready,camera_pauses=self.camera_gate.events)
   if not camera_ready:
    self.lease+=max(0,poll_elapsed)
    if self.engine and self.engine.active and not self.driving():self.engine.pause(max(0,poll_elapsed))
  if self.engine and self.engine.active and (camera_ready or self.driving()):
   wheel=self.driving()
   current={n:self.rows[n]['Present_Position'] for n in self.engine.joints}
   update=self.engine.tick(current,telemetry_at=min(self.rows[n]['captured_at'] for n in self.engine.joints),**({'rows':self.rows} if self.paddle_profile or wheel else {}));self.state.update(update)
   if self.paddle_profile and not wheel and not self.engine.active:self.lease=self.clock()+120
   if self.state.get('local_gripper_probe') and not self.engine.active:
    self.release_all('Local probe complete',record=False)
  self.publish();return self.state
 def driving(self):return isinstance(self.engine,WheelPulseExecutor) and self.engine.active
 def camera_fresh(self):
  try:d=self.camera_metadata()
  except (OSError,ValueError,TypeError):return False
  stamp=d.get('received_at')
  return type(stamp) in (int,float) and math.isfinite(stamp) and 0<=self.wall()-stamp<10 and d.get('seq') is not None
 def publish(self):
  if not(self.engine and self.engine.active):self.state['phase']='holding' if self.enabled else 'idle'
  self.state.update(time=self.wall(),rows=self.rows,enabled_motors=sorted(self.enabled),goals=self.goals,lease_remaining=max(0,self.lease-self.clock()),stop_latched=False)
 def enable(self,names,enabled):
  if not isinstance(names,list) or not names or len(set(names))!=len(names) or not set(names)<=set(self.names) or type(enabled)is not bool:raise ValueError('Select known distinct motor names and boolean enabled')
  if not enabled:
   if self.engine and self.engine.active:self.release_all('Release requested during movement')
   else:
    for n in names:self.release(n)
   self.publish();return
  if self.paddle_profile:
   try:camera_ready=self.camera_gate.update(holding=False)
   except RuntimeError as exc:raise ValueError(str(exc)) from exc
   if not camera_ready:raise ValueError('Pickup phone feed paused; motor activation refused')
  if self.read_only:raise ValueError('READ_ONLY_OWNER: motor activation disabled; calibration mismatch must be resolved deliberately')
  if not set(names)<=self.commandable_names:raise ValueError('UNSUPPORTED_OWNER_SCOPE: requested motors are read-only')
  # No STOP latch: after a STOP or fault this explicit request is the only way motors re-enable. A failed torque-off is a hardware problem.
  if self.state.get('ok') is not True:raise ValueError('OWNER_NOT_HEALTHY: last release failed '+json.dumps(self.state.get('release_errors'))+'; STOP must confirm release first')
  if self.engine and self.engine.active:raise ValueError('Movement is in progress')
  # Validate the entire request before enabling any motor.
  for n in names:
   row=self.rows[n];q=row['Present_Position'];lo,hi=self.limits[n]
   if row['Status'] or (not self.paddle_profile and row['Present_Temperature']>SOFTWARE_TEMPERATURE_LIMIT_C) or abs(row['Present_Load'])>(500 if n.endswith('gripper') or not self.paddle_profile else 800):raise ValueError(n+': fault or health limit')
   if self.paddle_profile and not 100<=row['Present_Voltage']<=140:raise ValueError(n+': pickup supply voltage outside 10..14V')
   if row['Operating_Mode']!=0:raise ValueError(n+': current mode is not supported position-hold mode')
   # Enable anywhere inside the saved range so a joint resting near a limit can drive itself back; pickup targets stay 40 ticks inside.
   if n in self.ranges and not self.ranges[n][0]+4<=q<=self.ranges[n][1]-4:raise ValueError(n+': current position outside saved travel margin')
   if not 0<=q<=4095 or (lo<hi and not lo<=q<=hi):raise ValueError(n+': current position outside firmware position limits')
  for n in names:
   if n in self.enabled:continue
   self.state.setdefault('enable_register_diagnostics',{})[n]={'pre_enable':self.register_diagnostics(n)}
   self.old[n]={f:self.read(n,f) for f in ['Lock','Torque_Limit','Goal_Velocity','Goal_Time','Acceleration','P_Coefficient']}
   self.write(n,'Lock',0)
   torque=(500 if n.endswith('gripper') else 400 if n.endswith('elbow_flex') else 800) if self.paddle_profile else (250 if n.endswith('gripper') else 400)
   self.write(n,'Torque_Limit',min(self.old[n]['Torque_Limit'],torque));self.write(n,'Goal_Velocity',200 if self.paddle_profile and n.endswith('gripper') else 100);self.write(n,'Goal_Time',0);self.write(n,'Acceleration',5 if self.paddle_profile else 10)
   if self.paddle_profile and n.endswith(('shoulder_lift','elbow_flex')):self.write(n,'P_Coefficient',32)
   q=self.read(n,'Present_Position');lo,hi=self.limits[n]
   if n in self.ranges and not self.ranges[n][0]+4<=q<=self.ranges[n][1]-4:raise RuntimeError(n+': drifted before enable')
   if not 0<=q<=4095 or(lo<hi and not lo<=q<=hi):raise RuntimeError(n+': drifted outside firmware limits')
   self.write(n,'Goal_Position',q);self.enabled.add(n)
   self.write(n,'Torque_Enable',1);self.write(n,'Lock',1);self.write(n,'Goal_Position',q);self.goals[n]=q
   self.state['enable_register_diagnostics'][n]['applied']=self.register_diagnostics(n)
  self.lease=self.clock()+(120 if self.paddle_profile else 30);self.state['released']=False;self.poll()
 def release(self,n):
  self.by_name[n].disable_torque([n],num_retry=3)
  if self.read(n,'Torque_Enable')!=0:raise RuntimeError(n+': release not confirmed')
  self.rows.setdefault(n,{})['Torque_Enable']=0
  self.rows[n]['released_readback_at']=self.wall()
  self.enabled.discard(n)
  if n in self.old:
   for f in ['Torque_Limit','Goal_Velocity','Goal_Time','Acceleration','P_Coefficient','Lock']:self.write(n,f,self.old[n][f])
   self.old.pop(n)
 def release_all(self,reason,record=True):
  # Releases every enabled motor and cancels any move; nothing re-enables or resumes until an explicit enable_motors. No latch.
  errors=[]
  if isinstance(self.engine,WheelPulseExecutor) and (self.engine.active or self.engine.powered):errors+=self.engine.abort() # moving base first
  for n in list(self.enabled):
   try:self.release(n)
   except Exception as e:errors.append(str(e))
  if self.engine:self.engine.active=False
  if self.paddle_profile:self.camera_gate.paused_at=self.camera_gate.initial_seq=None # a camera pause only applies while holding; the next enable needs a fresh feed
  if reason not in ('Operator STOP','Hardware-owner exit','Local probe complete','Release requested during movement'):self.state['root_failure']=reason
  if record:self.state.update(last_stop={'time':self.wall(),'reason':reason,'command_id':self.current_command,'released':not errors,'release_errors':errors},stop_count=self.state.get('stop_count',0)+1,error=reason)
  self.current_command=None
  self.state.update(ok=not bool(errors),operator_armed=not self.read_only,release_errors=errors,released=not errors,stop_latched=False);self.publish()
 def setpoints(self,goals):
  for n,q in goals.items():
   if n not in self.enabled or n.startswith('base_'):raise RuntimeError('Direct position target is not an enabled arm/head joint')
   lo,hi=self.ranges[n]
   if type(q)is not int or not lo+4<=q<=hi-4 or abs(q-self.goals[n])>(96 if self.paddle_profile else 68):raise RuntimeError('Position target leaves saved limits or sample bound')
   if q!=self.goals[n]:self.write(n,'Goal_Position',q);self.goals[n]=q
 def command(self,c):
  if type(c.get('id'))is not int or c['id']<=0 or c.get('session_started')!=self.started:raise ValueError('Command belongs to another hardware-owner session')
  op=c.get('op')
  if self.read_only and op not in ('stop','enable_motors','hold'):raise ValueError('READ_ONLY_OWNER: motion commands disabled')
  if op=='stop':self.release_all('Operator STOP');self.state['completed']=c['id'];return
  if op=='enable_motors':self.enable(c.get('names'),c.get('enabled'));self.state['completed']=c['id'];return
  if op=='local_gripper_probe':
   if self.paddle_profile:raise ValueError('Legacy diagnostic probe unavailable under pickup profile')
   if self.enabled or any(r.get('Torque_Enable')!=0 for r in self.rows.values()) or len(self.rows)!=16:raise ValueError('Probe requires all16 observed released')
   if c.get('authorization')!='one-shot-right-gripper-48' or self.state.get('probe_used'):raise ValueError('Local diagnostic authorization missing or already used')
   n='right_arm_gripper';current=int(self.rows[n]['Present_Position'])
   from gripper_response_probe import GripperResponseProbe
   probe=GripperResponseProbe(current,c.get('target_ticks'),self.ranges[n],self.setpoints,clock=self.clock,wall=self.wall)
   self.state['probe_used']=True
   self.enable([n],True)
   applied=self.state['enable_register_diagnostics'][n]['applied']['registers']
   if any(applied[f]!=v for f,v in {'Torque_Limit':250,'Goal_Velocity':100,'Acceleration':10,'P_Coefficient':32,'Operating_Mode':0}.items()):raise RuntimeError('Probe applied settings differ from authorized values')
   self.engine=probe;self.current_command=c['id'];self.state.update(probe.start(c));self.last_tick=self.clock();self.lease=self.clock()+30;self.publish();return
  if op=='hold':self.lease=self.clock()+(120 if self.paddle_profile else 30);self.state['completed']=c['id'];return
  if op=='halt':
   # Stop advancing and hold where the arm is (wheels: brake and release). Unlike stop, nothing is released.
   if self.engine and self.engine.active:
    if self.driving():self.engine.stopped_early='halted by operator'
    elif hasattr(self.engine,'halt'):
     update=self.engine.halt({n:self.rows[n]['Present_Position'] for n in self.engine.joints});self.state.update(update,halted_command_id=self.current_command)
     if self.paddle_profile:self.lease=self.clock()+120
    else:raise ValueError('The active motion cannot be halted; use stop')
   self.state['completed']=c['id'];self.publish();return
  if op=='base_pulse':
   if not self.wheel_names:raise ValueError('UNSUPPORTED_OWNER_SCOPE: owner started without --wheels; base drive disabled')
   if self.engine and self.engine.active:raise ValueError('Previous motion has not completed')
   candidate=WheelPulseExecutor(self.read,self.write,self.camera_fresh,clock=self.clock,wall=self.wall)
   update=candidate.start(c,self.rows,session_started=self.started)
   self.engine=candidate;self.current_command=c['id'];self.state.update(update);self.last_tick=self.clock();self.publish();return
  if op not in ('direct_joint','gripper_target'):raise ValueError('Unsupported hardware command')
  positions=c.get('positions')
  if c.get('waypoints') is not None:
   if not self.paddle_profile or op!='direct_joint':raise ValueError('Waypoint paths require the pickup profile')
   positions=c['waypoints'][0] if isinstance(c['waypoints'],list) and c['waypoints'] else None
  if not isinstance(positions,dict) or not positions or not set(positions)<=self.enabled or not set(positions)<=set(self.position_names):raise ValueError('Targets require already-enabled arm/head motors; wheels do not accept position-motion requests')
  if self.engine and self.engine.active:
   # replace=true swaps a running arm motion for this one, starting from the held goals (no stop in between).
   if not c.get('replace') or self.driving() or not hasattr(self.engine,'halt'):raise ValueError('Previous motion has not completed; send replace=true to change it, or halt first')
   replaced=self.current_command;self.state.update(self.engine.halt({n:self.rows[n]['Present_Position'] for n in self.engine.joints}),replaced_command_id=replaced)
  executor=DirectJointExecutor
  if self.paddle_profile:
   from paddle_joint_executor import PaddleJointExecutor
   executor=PaddleJointExecutor
  elif op=='gripper_target':
   from gripper_waypoint_executor import GripperWaypointExecutor
   executor=GripperWaypointExecutor
  candidate=executor(list(positions),{n:self.ranges[n] for n in positions},self.setpoints,clock=self.clock,wall=self.wall)
  current={n:self.rows[n]['Present_Position'] for n in positions}
  if self.paddle_profile:
   if set(self.position_names)!=self.enabled:raise ValueError('Pickup requires all six right-arm motors explicitly enabled')
   if not self.camera_gate.update(holding=True):raise ValueError('Pickup phone feed paused; no new target accepted')
  update=candidate.start(c,current,session_started=self.started,**({'held_goals':self.goals} if self.paddle_profile else {}))
  if self.paddle_profile:self.motion_count+=1
  self.state['pickup_motion_segments_used']=self.motion_count
  self.state['local_gripper_probe']=False
  self.engine=candidate;self.current_command=c['id'];self.state.update(update);self.lease=self.clock()+(candidate.deadline+5 if op=='gripper_target' or self.paddle_profile else 30);self.last_tick=self.clock();self.publish()

def atomic(path,value):
 temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,allow_nan=False));temp.replace(path)

def main():
 old=Path('/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20');utility=old/'work/carton-visual-controller/software';root=Path('/Users/teachera/Documents/Codex/2026-10-05/m');folder=root/'work/gemma-hardware-session';folder.mkdir(exist_ok=True)
 os.environ.update(CARTON_LIVE_SOFTWARE=str(old/'xlerobot-farm/software'),CARTON_UTILITY_SOFTWARE=str(utility),CARTON_WORKSPACE_ROOT=str(root),CARTON_PROFILE='paper-tray-v0')
 sys.path.insert(0,str(utility/'scripts/carton_robot'))
 from carton_runtime import PROFILE
 from farm.config import load_profile
 from farm.adapters.robot_lerobot import LeRobotXLeRobot
 from servo_ownership import ServoOwnership
 from strict_servo_replies import guard_replies
 from coherent_servo_telemetry import decode_telemetry, ADDRESS, LENGTH
 def observed_telemetry(bus,name):
  motor=bus.motors[name];started=time.time()
  transaction={'motor':name,'servo_id':motor.id,'model':motor.model,'port':bus.port,'address':ADDRESS,'length':LENGTH,'started_at':started}
  try:
   data,communication,packet_error=bus.packet_handler.readTxRx(bus.port_handler,motor.id,ADDRESS,LENGTH)
   transaction.update(communication=communication,packet_error=packet_error,payload_bytes=list(data),reply=getattr(bus,'last_reply_evidence',None),finished_at=time.time())
   decoded=decode_telemetry(data,communication,packet_error);bus.last_telemetry_sample=transaction;return decoded
  except Exception as exc:
   transaction.update(finished_at=time.time(),error=str(exc));bus.last_telemetry_failure=transaction
   raise RuntimeError(str(exc)+'; transaction='+json.dumps(transaction)) from exc
 r=LeRobotXLeRobot(load_profile(PROFILE).robot).robot;buses=[r.bus1,r.bus2];owner=None;ownership=None;stop=threading.Event()
 signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())
 last=None
 try:
  ownership=ServoOwnership([b.port for b in buses]).acquire()
  for b in buses:guard_replies(b);b.connect(handshake=False)
  owner=HardwareOwner(buses,r.calibration,observed_telemetry,read_only='--read-only' in sys.argv,position_scope=[n for b in buses for n in b.motors if n.startswith('right_arm_')] if '--right-arm-only' in sys.argv else None,paddle_profile='--paddle-profile' in sys.argv,wheels='--wheels' in sys.argv);owner.inspect();atomic(folder/'status.json',owner.state)
  if(folder/'command.json').exists():last=json.loads((folder/'command.json').read_text()).get('id')
  print('Hardware owner ready:16 motor reads, all torque off.',flush=True)
  while not stop.is_set():
   try:owner.poll()
   except RuntimeError as e:
    owner.state['failed_command_id']=owner.current_command
    if owner.engine:
     owner.state['direct_fault_diagnostics']=owner.engine.diagnostics
     owner.state['direct_fault_samples']=owner.engine.samples
    owner.state['fault_active_rows']={n:dict(row) for n,row in owner.rows.items()}
    owner.state['fault_at']=time.time()
    owner.release_all(str(e))
    print('Hardware command stopped: '+str(e),flush=True)
   p=folder/'command.json'
   if p.exists():
    c=json.loads(p.read_text())
    if c.get('id')!=last:
     last=c.get('id')
     try:owner.command(c)
     except ValueError as e:owner.state['last_rejected']={'id':last,'reason':str(e)}
   atomic(folder/'status.json',owner.state);time.sleep(.02)
 except BaseException as e:
  print('Hardware owner stopped: '+str(e),flush=True)
  if owner:
   owner.state['root_failure']=str(e);owner.state['failed_command_id']=owner.current_command
   owner.release_all(str(e))
 finally:
  if owner:
   owner.release_all('Hardware-owner exit');atomic(folder/'status.json',owner.state)
  for b in buses:
   if b.is_connected:b.disconnect(disable_torque=False)
  if ownership:ownership.close()
if __name__=='__main__':main()
