"""Hardware file client: no serial access, owner creation or automatic activation."""
import fcntl,json,math,threading,time,contextlib,copy
from pathlib import Path
from gemma_control_limits import SOFTWARE_TEMPERATURE_LIMIT_C


def atomic_json(path,value):
    temp=path.with_name('.'+path.name+'.'+str(time.time_ns())+'.tmp')
    try:
        with temp.open('x') as output:
            json.dump(value,output,allow_nan=False);output.flush();__import__('os').fsync(output.fileno())
        temp.replace(path)
    finally:temp.unlink(missing_ok=True)


class DirectJointClient:
    def __init__(self,session,calibration,*,clock=time.time,sleep=time.sleep):
        self.folder=Path(session);self.calibration=calibration;self.clock=clock;self.sleep=sleep
        self.lock=threading.Lock();self.cancel_generation=0
        self.sequence_lock=threading.RLock();self.sequence_local=threading.local()
    def status(self):
        state=json.loads((self.folder/'status.json').read_text());state['status_age_s']=self.clock()-state['time'];return state
    def readiness(self):
        result={'mode':'direct_joint','control_mode':'direct_joint','hardware_server':True,
            'execution_adapter_bound':True,'execution_adapter_source':'work/gemma_direct_client.py:DirectJointClient',
            'motion_ready':False,'available_to_accept_authorized_command':False,
            'operator_armed':False,'local_operator_gate':False,'motor_owner_active':False,
            'supported_joints':[],'supported_motors':[],'joint_blockers':{},'blockers':[],
            'remote_owner_start':False,'stop_latched':False,'owner_restart_required_after_stop':False,'continuous_profile_required':False,
            'cartesian_transform_required':False,'camera_gate_required':False,'software_temperature_limit_c':SOFTWARE_TEMPERATURE_LIMIT_C}
        try:
            state=self.status();selected=state.get('supportsselectedjoints',[])
            result.update(owner_started=state.get('started'),owner_phase=state.get('phase'),
                owner_status_age_s=state['status_age_s'],supported_joints=selected,
                supported_motors=state.get('supported_motors',[]),enabled_motors=state.get('enabled_motors',[]),
                operator_armed=state.get('operator_armed') is True,local_operator_gate=state.get('operator_armed') is True,
                motor_owner_active=state.get('hardware_server') is True and 0<=state['status_age_s']<=1)
            result.update(pickup_required_enabled_motors=state.get('pickup_required_enabled_motors',[]),pickup_motion_segments_used=state.get('pickup_motion_segments_used',0),pickup_motion_segment_budget=state.get('pickup_motion_segment_budget'),pickup_idle_hold_seconds=state.get('pickup_idle_hold_seconds'),camera_pause_active=state.get('camera_pause_active',False),camera_supervision_required=state.get('camera_supervision_required',False),execution_profile=state.get('execution_profile','legacy-direct'),base_drive_supported=state.get('base_drive_supported') is True,base_drive_limits=state.get('base_drive_limits'),read_only=state.get('read_only') is True,calibration_mismatches=state.get('calibration_mismatches',{}),last_stop=state.get('last_stop'),stop_count=state.get('stop_count',0),release_errors=state.get('release_errors',[]))
            if (state.get('teleop') or {}).get('active'):result['blockers'].append('MANUAL_CONTROL_ACTIVE: Joy-Con session owns the robot')
            if state.get('read_only') is True:result['blockers'].append('READ_ONLY_OWNER: calibration mismatch blocks activation')
            if state.get('control_mode')!='direct_joint' or state.get('hardware_server') is not True:result['blockers'].append('HARDWARE_OWNER_PROTOCOL_UNAVAILABLE')
            if not 0<=state['status_age_s']<=1:result['blockers'].append('OWNER_STATUS_STALE')
            if state.get('phase') not in ('idle','holding'):result['blockers'].append('OWNER_BUSY: '+str(state.get('phase')))
            # No STOP latch; ok is false only when a torque-off release failed (hardware problem).
            if state.get('ok') is not True:result['blockers'].append('OWNER_NOT_HEALTHY'+(': release failed '+json.dumps(state['release_errors']) if state.get('release_errors') else ''))
            if state.get('operator_armed') is not True:result['blockers'].append('AUTHORIZED_INTERFACE_NOT_AVAILABLE')
            if type(state.get('started')) not in (int,float) or not math.isfinite(state['started']):result['blockers'].append('OWNER_IDENTITY_MISSING')
            if not isinstance(selected,list) or not selected or len(set(selected))!=len(selected):result['blockers'].append('POSITION_SCOPE_INVALID');selected=[]
            rows=state.get('rows',{});torque={}
            for name in state.get('supported_motors',[]):torque[name]=rows.get(name,{}).get('Torque_Enable')
            result['torque_enabled_by_joint']=torque
            result['lease_remaining_s']=state.get('lease_remaining',0)
            result['lease_applies_while_enabled']=True
            result['explicit_enable_renews_idle_lease']=state.get('execution_profile')=='paddle-success-v1' or not bool(state.get('enabled_motors'))
            for name in selected:
                issues=[];row=rows.get(name,{})
                if name not in self.calibration or name.startswith('base_'):issues.append('UNSUPPORTED_POSITION_JOINT')
                else:
                    lo,hi=self.calibration[name]['range_min'],self.calibration[name]['range_max']
                    if state.get('ranges',{}).get(name)!=[lo,hi]:issues.append('SAVED_RANGE_MISMATCH')
                    q=row.get('Present_Position')
                    if type(q) not in (int,float) or not math.isfinite(q) or not lo<=q<=hi:issues.append('CURRENT_POSITION_OUTSIDE_SAVED_RANGE')
                for field,valid in [('Status',lambda x:x==0),*([('Present_Temperature',lambda x:x<=SOFTWARE_TEMPERATURE_LIMIT_C)] if state.get('execution_profile')!='paddle-success-v1' else []),('Present_Load',lambda x:abs(x)<=(800 if state.get('execution_profile')=='paddle-success-v1' and not name.endswith('gripper') else 500))]:
                    v=row.get(field)
                    if type(v) not in (int,float) or not math.isfinite(v) or not valid(v):issues.append('UNSAFE_OR_INVALID_'+field)
                if type(row.get('Torque_Enable')) is not int or row['Torque_Enable'] not in (0,1):issues.append('INVALID_TORQUE_TELEMETRY')
                if issues:result['joint_blockers'][name]=issues
            # Unrequested joint faults/range violations do not block healthy scopes.
            result['available_to_accept_authorized_command']=not result['blockers']
            result['motion_ready']=result['available_to_accept_authorized_command']
        except (OSError,ValueError,KeyError,TypeError) as exc:result['blockers'].append('HARDWARE_OWNER_UNAVAILABLE: '+str(exc))
        result['blocker']='; '.join(result['blockers']) if result['blockers'] else None
        return result
    def stop(self,*,expected_started=None):
        self.cancel_generation+=1
        try:state=self.status()
        except Exception:return {'stop_requested':False,'owner_available':False}
        if expected_started is not None and state.get('started')!=expected_started:return {'stop_requested':False,'reason':'BOUND_OWNER_CHANGED'}
        command={'id':time.time_ns(),'op':'stop','session_started':state.get('started')}
        atomic_json(self.folder/'command.json',command)
        # No latch: STOP releases all motors, which stay released until an explicit enable; no owner restart needed.
        result={'stop_requested':True,'command_id':command['id'],'release':'torque eases off over about 2 s, then off','release_confirmed':False,'stop_reset_supported':True,'stop_latched':False,'owner_restart_required':False,'motors_stay_released_until':'explicit robot_set_motor_enable'}
        deadline=self.clock()+6  # soft release eases torque off over ~2 s first
        while self.clock()<deadline:
            try:
                latest=self.status()
                if latest.get('started')!=state.get('started'):
                    result['release_reason']='BOUND_OWNER_CHANGED';return result
                if (latest.get('last_rejected') or {}).get('id')==command['id']:
                    result['release_reason']='Owner rejected STOP: '+str(latest['last_rejected'].get('reason'));return result
                rows=latest.get('rows',{})
                if 0<=latest['status_age_s']<=1 and latest.get('time',0)>=command['id']/1e9 and latest.get('completed')==command['id']:
                    if latest.get('release_errors'):
                        result.update(release_reason='Release failed; owner not healthy',release_errors=latest['release_errors']);return result
                    if rows and len(rows)==len(latest.get('supported_motors') or rows) and not latest.get('enabled_motors') and all(r.get('Torque_Enable')==0 for r in rows.values()):
                        result.update(release_confirmed=True,release_owner_time=latest['time'],owner_started=latest['started'],owner_phase=latest.get('phase'));return result
            except (OSError,ValueError,KeyError,TypeError):pass
            self.sleep(.02)
        result['release_reason']='Fresh same-session all16 torque-zero readback not observed';return result
    def set_motor_enable(self,names,enabled):
        if type(enabled) is not bool or not isinstance(names,list) or not names or len(set(names))!=len(names) or any(not isinstance(n,str) for n in names):raise ValueError('Distinct motor names and boolean enabled required')
        return self._command({'op':'enable_motors','names':names,'enabled':enabled})
    def execute(self,positions,duration_s,wait=True,replace=False):
        if type(duration_s) not in (int,float) or not math.isfinite(duration_s) or not 0<duration_s<=25:raise ValueError('Duration must be finite in (0,25]')
        if not isinstance(positions,dict) or not positions or any(type(q) is not int for q in positions.values()):raise ValueError('Nonempty integer encoder targets required')
        return self._command({'op':'direct_joint','positions':positions,'duration_s':duration_s,'replace':replace is True},wait=wait)
    def execute_path(self,waypoints,duration_s,wait=True,replace=False):
        """Continuous motion through waypoints (pickup profile); intermediate points are passed without stopping."""
        if type(duration_s) not in (int,float) or not math.isfinite(duration_s) or not 0<duration_s<=60:raise ValueError('Path duration must be finite in (0,60]')
        if not isinstance(waypoints,list) or not waypoints or any(not isinstance(w,dict) or not w or any(type(q) is not int for q in w.values()) for w in waypoints):raise ValueError('Nonempty list of integer waypoint targets required')
        return self._command({'op':'direct_joint','waypoints':waypoints,'duration_s':duration_s,'replace':replace is True},wait=wait)
    def halt(self):
        """Stop the running motion and hold where the arm is (wheels brake and release); unlike stop, nothing is released."""
        return self._command({'op':'halt'})
    def motion(self):
        """Compact live view of the running or last motion, for monitoring while it moves."""
        state=self.status();d=state.get('direct_settle_diagnostics') or {}
        return {'phase':state.get('phase'),'status_age_s':round(state['status_age_s'],3),'moving':state.get('phase')=='moving',
            'running_command_id':state.get('accepted') if state.get('phase')=='moving' else None,'last_completed_command_id':state.get('completed'),
            'closure_outcome':state.get('closure_outcome'),'endpoint_reached':state.get('endpoint_reached'),'settle_residual_ticks':state.get('settle_residual_ticks'),
            'waypoint':d.get('leg'),'waypoints':d.get('legs'),'elapsed_s':d.get('elapsed_s'),'final_targets':d.get('final_targets'),
            'joints':{n:{k:v.get(k) for k in ('current_ticks','goal_ticks','target_ticks','following_error_ticks')} for n,v in (d.get('joints') or {}).items()},
            'base_drive_phase':state.get('base_drive_phase'),'enabled_motors':state.get('enabled_motors'),'lease_remaining_s':state.get('lease_remaining'),
            'last_stop':state.get('last_stop'),'positions':{n:r.get('Present_Position') for n,r in state.get('rows',{}).items() if n.startswith(('right_arm_','left_arm_'))}}
    def drive_base(self,linear_m_s,angular_rad_s,duration_s):
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (linear_m_s,angular_rad_s,duration_s)) or not 0<duration_s<=3:raise ValueError('Finite linear_m_s, angular_rad_s and duration_s in (0,3] required')
        return self._command({'op':'base_pulse','linear_m_s':linear_m_s,'angular_rad_s':angular_rad_s,'duration_s':duration_s})
    def _validate(self,request,state):
        if (state.get('teleop') or {}).get('active'):
            raise ValueError('MANUAL_CONTROL_ACTIVE: Joy-Con session owns the robot; STOP remains available')
        if request['op']=='halt':return
        if request['op']=='base_pulse':
            if state.get('base_drive_supported') is not True:raise ValueError('UNSUPPORTED_OWNER_SCOPE: owner started without --wheels; base drive disabled')
            for n in ('base_left_wheel','base_right_wheel'):
                if state.get('rows',{}).get(n,{}).get('Torque_Enable')!=0:raise ValueError('Wheel must be released before a base pulse: '+n)
            from wheel_pulse_executor import WheelPulseExecutor
            WheelPulseExecutor.check_request(request)
            return
        if request['op']=='enable_motors':
            names=request['names']
            if not set(names)<=set(state.get('supported_motors',[])):raise ValueError('Unknown motor names')
            if not request['enabled']:return
            readonly=sorted(set(names)-set(state.get('commandable_motors',state.get('supported_motors',[]))))
            if readonly:raise ValueError('UNSUPPORTED_OWNER_SCOPE: these motors are read-only and are never powered: '+', '.join(readonly)+
                                         '. The head cannot be moved (aim cameras by moving the arm instead); the wheels move only through robot_move_base, which needs no enable. Enable only arm joints.')
            for n in names:
                if n not in state.get('supportsselectedjoints',[]):
                    row=state.get('rows',{}).get(n,{})
                    if row.get('Operating_Mode')!=0:raise ValueError('Wheel hold activation requires actual position mode0: '+n)
                    q=row.get('Present_Position')
                    if type(q) not in (int,float) or not math.isfinite(q):raise ValueError('Wheel current encoder missing: '+n)
                    bounds=state.get('wheel_hold_ranges',{}).get(n) or row.get('firmware_position_limits')
                    if bounds is not None and (not isinstance(bounds,list) or len(bounds)!=2 or not bounds[0]<bounds[1] or not bounds[0]<=q<=bounds[1]):raise ValueError('Wheel current position/actual firmware limits invalid: '+n)
        else:
            targets=request.get('waypoints') or [request['positions']];names=list(targets[0])
            if any(set(t)!=set(names) for t in targets):raise ValueError('Every waypoint must name the same joints')
            if not set(names)<=set(state.get('supportsselectedjoints',[])):raise ValueError('Unsupported position motor or wheel target')
            for n in names:
                c=self.calibration[n]
                if any(not c['range_min']+4<=t[n]<=c['range_max']-4 for t in targets):raise ValueError('Target outside saved range plus4tickmargin: '+n)
                if state['rows'][n].get('Torque_Enable')!=1:raise ValueError('Requested motor is released; explicitly enable it first: '+n)
        if request['op']!='enable_motors' and state.get('execution_profile')=='paddle-success-v1':
            arms={n.split('_arm_')[0] for n in names};required=[m for m in state.get('supportsselectedjoints',[]) if m.split('_arm_')[0] in arms]
            if not set(required)<=set(state.get('enabled_motors',[])):raise ValueError('Pickup requires all six joints of the commanded arm explicitly enabled: '+json.dumps(sorted(set(required)-set(state.get('enabled_motors',[])))))
            from paddle_joint_executor import PaddleJointExecutor
            dry=PaddleJointExecutor(names,{n:state['ranges'][n] for n in names},lambda _:None)
            dry.start(dict(request,id=1,session_started=state['started']),{n:state['rows'][n]['Present_Position'] for n in names},session_started=state['started'],held_goals=state.get('goals'))
        ready=self.readiness()
        faults={n:ready['joint_blockers'][n] for n in names if n in ready['joint_blockers']}
        if faults:raise ValueError('Requested motor health/range blockers: '+json.dumps(faults))
    @contextlib.contextmanager
    def serialized(self):
        if not self.sequence_lock.acquire(blocking=False):raise RuntimeError('Hardware action sequence active')
        try:
            if getattr(self.sequence_local,'writer',None) is not None:
                yield
            else:
                with (self.folder/'hardware-client.lock').open('a') as writer:
                    try:fcntl.flock(writer,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    except BlockingIOError:raise RuntimeError('Another hardware command writer is active')
                    self.sequence_local.writer=writer
                    try:yield
                    finally:self.sequence_local.writer=None
        finally:self.sequence_lock.release()

    def set_gripper(self,arm,position,duration):
        if arm not in ('left','right') or type(position) is not int or type(duration) not in (int,float) or not math.isfinite(duration) or not 0<duration<=25:
            raise ValueError('Valid arm, integer gripper target and finite duration (0,25] required before activation')
        name=arm+'_arm_gripper';request={'op':'gripper_target' if arm=='right' else 'direct_joint','positions':{name:position},'duration_s':duration}
        with self.serialized():
            state=self.status();ready=self.readiness();started=state['started'];generation=self.cancel_generation
            if not ready['available_to_accept_authorized_command']:raise RuntimeError('Gripper owner unavailable: '+json.dumps(ready))
            # The pickup profile moves an arm only with all six of its joints enabled, so auto-enable takes the
            # whole arm (released joints hold where they are); other owners enable just the gripper.
            arm_joints=[m for m in state.get('supportsselectedjoints',[]) if m.startswith(arm+'_arm_')]
            to_enable=[m for m in (arm_joints if state.get('execution_profile')=='paddle-success-v1' else [name]) if state['rows'][m].get('Torque_Enable')==0]
            prospective=copy.deepcopy(state)
            for m in to_enable:prospective['rows'][m]['Torque_Enable']=1
            prospective['enabled_motors']=sorted(set(prospective.get('enabled_motors',[]))|set(to_enable))
            self._validate(request,prospective)
            from direct_joint_executor import DirectJointExecutor
            executor=DirectJointExecutor
            if state.get('execution_profile')=='paddle-success-v1':
                from paddle_joint_executor import PaddleJointExecutor
                executor=PaddleJointExecutor
            elif arm=='right':
                from gripper_waypoint_executor import GripperWaypointExecutor
                executor=GripperWaypointExecutor
            dry=executor([name],{name:state['ranges'][name]},lambda _: (_ for _ in ()).throw(AssertionError('Validation wrote')))
            dry.start(dict(request,id=1,session_started=started),{name:state['rows'][name]['Present_Position']},session_started=started)
            if state.get('enabled_motors') and state.get('lease_remaining',0)<=(0 if state.get('execution_profile')=='paddle-success-v1' else dry.duration+5):raise ValueError('Insufficient owner lease before gripper activation')
            enabled_here=bool(to_enable);enable_attempted=False;phase='validated'
            try:
                if enabled_here:
                    self._validate({'op':'enable_motors','names':to_enable,'enabled':True},state)
                    phase='enabling';enable_attempted=True
                    activation=self.set_motor_enable(to_enable,True)
                    if not activation.get('completed'):raise RuntimeError('Gripper enable not completed: '+json.dumps(activation))
                latest=self.status()
                if latest['started']!=started or generation!=self.cancel_generation:raise RuntimeError('Owner/STOP changed during gripper sequence')
                phase='moving';result=self._command(request) if arm=='right' else self.execute({name:position},duration)
                if not result.get('completed'):
                    # Stopping short while still holding is a motion outcome (often the jaws met an object), reported
                    # like robot_move_joint_targets does, not a failure; anything else is still an error.
                    if result.get('holding') and result.get('closure_outcome') in ('settled_short','contact_halt','stationary_closure_unverified'):
                        return dict(result,gripper_auto_enabled=enabled_here,auto_enabled_motors=to_enable,gripper=name,sequence_phase='stopped_short',
                                    note='The jaw stopped before the target and is holding (often an object between the jaws). Look at the wrist camera before the next step.')
                    raise RuntimeError('Gripper move not completed: '+json.dumps(result))
                return dict(result,gripper_auto_enabled=enabled_here,auto_enabled_motors=to_enable,gripper=name,sequence_phase='completed')
            except BaseException as exc:
                cleanup='not_requested'
                if enable_attempted:
                    try:cleanup=self.stop(expected_started=started)
                    except Exception as cleanup_error:cleanup={'release_confirmed':False,'error':str(cleanup_error)}
                raise RuntimeError('Gripper sequence failed in '+phase+'; cleanup='+json.dumps(cleanup)+'; '+str(exc)) from exc

    def _command(self,request,wait=True):
        with self.serialized():return self._command_locked(request,wait)

    def _command_locked(self,request,wait=True):
        release=request['op']=='enable_motors' and request['enabled'] is False
        # halt and replace=true are the two requests meant for a motion that is still running.
        busy_ok=request['op']=='halt' or request.get('replace') is True;phases=('idle','holding','moving') if busy_ok else ('idle','holding')
        ready=self.readiness()
        if busy_ok and ready['blockers'] and all(b.startswith('OWNER_BUSY') for b in ready['blockers']):ready=dict(ready,available_to_accept_authorized_command=True)
        if not ready['available_to_accept_authorized_command'] and not release:return {'accepted':False,'motor_writes':0,'reason':ready['blocker'],'readiness':ready}
        if not self.lock.acquire(blocking=False):raise RuntimeError('Hardware command active; STOP remains independently available')
        dispatched=False;started=None
        try:
            with contextlib.nullcontext(self.sequence_local.writer) as writer:
                state=self.status();started=state['started'];generation=self.cancel_generation
                if state.get('hardware_server') is not True or not 0<=state['status_age_s']<=1:raise RuntimeError('Fresh hardware owner unavailable')
                if not release and (state.get('phase') not in phases or state.get('operator_armed') is not True or state.get('ok') is not True):raise RuntimeError('Owner busy or unsafe')
                self._validate(request,state)
                if not release and state.get('enabled_motors') and state.get('lease_remaining',0)<=(0 if state.get('execution_profile')=='paddle-success-v1' else request.get('duration_s',0)+5):raise RuntimeError('Insufficient owner lease')
                command_file=self.folder/'command.json';old=json.loads(command_file.read_text()) if command_file.exists() else None
                command_id=max(time.time_ns(),int((old or {}).get('id',0))+1)
                command={**request,'id':command_id,'session_started':started}
                latest=self.status();stops=latest.get('stop_count',0)
                # A STOP written but not yet read by the owner must never be overwritten.
                if (old or {}).get('op')=='stop' and old.get('session_started')==latest.get('started') and latest.get('completed')!=old.get('id') and (latest.get('last_rejected') or {}).get('id')!=old.get('id'):raise RuntimeError('STOP pending; not dispatching')
                if latest['started']!=started or not 0<=latest['status_age_s']<=1 or (not release and (latest.get('phase') not in phases or latest.get('operator_armed') is not True or latest.get('ok') is not True)):raise RuntimeError('Owner changed before dispatch')
                if generation!=self.cancel_generation:raise RuntimeError('STOP interrupted dispatch')
                if (json.loads(command_file.read_text()) if command_file.exists() else None)!=old:raise RuntimeError('Another writer changed command file')
                atomic_json(command_file,command);dispatched=True
                deadline=self.clock()+(request['duration_s']+10 if request['op']=='base_pulse' else 90 if state.get('execution_profile')=='paddle-success-v1' and request['op'] in ('direct_joint','gripper_target') else 60 if request['op']=='gripper_target' else 30 if request['op']=='direct_joint' else 5)
                while self.clock()<deadline:
                    if generation!=self.cancel_generation:raise RuntimeError('STOP cancelled goal; no automatic resume')
                    current=self.status()
                    if current.get('started')!=started:raise RuntimeError('Bound owner restarted')
                    if not 0<=current['status_age_s']<=1:raise RuntimeError('Owner telemetry stale')
                    # No latch: a fault or STOP returns the owner to idle, so detect it by its stop record.
                    last_stop=current.get('last_stop') or {}
                    if not release and (last_stop.get('command_id')==command_id or current.get('stop_count',0)>stops or current.get('phase')=='stopped' or current.get('ok') is not True):raise RuntimeError('Owner stopped: '+str(last_stop.get('reason') or current.get('error')))
                    if (current.get('last_rejected') or {}).get('id')==command_id:raise RuntimeError('Owner rejected: '+str(current['last_rejected'].get('reason')))
                    if json.loads(command_file.read_text()).get('id')!=command_id:raise RuntimeError('Command overwritten; cancelled')
                    if request['op']=='halt' and current.get('completed')==command_id:
                        return {'accepted':True,'completed':True,'halted':True,'command_id':command_id,'halted_command_id':current.get('halted_command_id'),'phase':current.get('phase'),
                            'positions':{n:r.get('Present_Position') for n,r in current.get('rows',{}).items() if n.startswith(('right_arm_','left_arm_'))},'base_drive_phase':current.get('base_drive_phase'),
                            'note':'Holding where it stopped (wheels brake, then release). Nothing was released; send a new move to continue.'}
                    if not wait and request['op']=='direct_joint' and command_id in (current.get('accepted'),current.get('completed')):
                        return {'accepted':True,'started':True,'completed':current.get('completed')==command_id,'command_id':command_id,'owner_started':started,'phase':current.get('phase'),
                            'deadline_s':current.get('direct_deadline_s'),'waypoints':current.get('motion_waypoints'),
                            'note':'Motion is running. Monitor with robot_get_motion and cameras; robot_halt_motion stops and holds; a new move with replace=true changes course.','mode':'direct_joint'}
                    if current.get('completed')==command_id and current.get('phase') in ('idle','holding'):
                        if request['op']=='base_pulse':
                            result=current.get('base_result') or {}
                            if result.get('released') is not True or any(current['rows'][n].get('Torque_Enable')!=0 for n in ('base_left_wheel','base_right_wheel')):
                                self.sleep(.02);continue
                            return {'accepted':True,'completed':True,'command_id':command_id,'owner_started':started,'base_result':result,
                                'owner_status_time':current['time'],'motor_writes':'canonical owner only','mode':'base_pulse'}
                        if request['op']=='enable_motors':
                            measured={n:current['rows'][n]['Torque_Enable'] for n in request['names']}
                            if any(v!=int(request['enabled']) for v in measured.values()):
                                # The owner echoes completion before its next all-motor
                                # read poll; require actual readback before replying.
                                self.sleep(.02)
                                continue
                        else:
                            final=(request.get('waypoints') or [request.get('positions')])[-1]
                            measured={n:current['rows'][n]['Present_Position'] for n in final}
                            if current.get('closure_outcome') in ('halted','contact_halt'):
                                return {'accepted':True,'completed':False,'halted':True,'endpoint_reached':False,'closure_outcome':current['closure_outcome'],'holding':True,'command_id':command_id,'readbacks':measured,
                                    'settle_residual_ticks':current.get('settle_residual_ticks'),'contact':current.get('contact'),'contact_note':current.get('contact_note'),'mode':'direct_joint'}
                            if current.get('execution_profile')=='paddle-success-v1' and current.get('closure_outcome')=='settled_short':
                                # At rest short of target after bounded corrections: holding, not a success and not a STOP.
                                if any(abs(q-final[n])>96+57 for n,q in measured.items()):raise RuntimeError('Pickup settled_short contradicts measured endpoint')
                                return {'accepted':True,'completed':False,'endpoint_reached':False,'closure_outcome':'settled_short','holding':True,'command_id':command_id,'owner_started':started,'readbacks':measured,
                                    'settle_residual_ticks':current.get('settle_residual_ticks'),'execution_profile':current.get('execution_profile'),'grasp_verified':False,'owner_status_time':current['time'],
                                    'reason':'Joint came to rest short of its target after bounded goal corrections; motors are holding at the measured position. Re-plan from fresh readbacks or STOP.',
                                    'motor_writes':'canonical owner only','mode':'direct_joint'}
                            if current.get('execution_profile')=='paddle-success-v1' and current.get('closure_outcome')=='stationary_closure_unverified' and all(n.endswith('gripper') for n in final):
                                # The jaws stopped on something before the target (a possible grasp): holding, not a failure.
                                # It cannot have closed past its target.
                                if any(q<final[n]-96 for n,q in measured.items()):raise RuntimeError('Pickup closure went past its target; contradicts measured endpoint')
                                return {'accepted':True,'completed':False,'endpoint_reached':False,'closure_outcome':'stationary_closure_unverified','holding':True,'command_id':command_id,'owner_started':started,'readbacks':measured,
                                    'settle_residual_ticks':current.get('settle_residual_ticks'),'execution_profile':current.get('execution_profile'),'grasp_verified':False,'owner_status_time':current['time'],
                                    'reason':'The jaws stopped on something before the target and are holding there (possible grasp, not verified). Check the wrist camera.',
                                    'motor_writes':'canonical owner only','mode':'direct_joint'}
                            if current.get('execution_profile')=='paddle-success-v1':
                                for n,q in measured.items():
                                    tolerance=30 if n.endswith('gripper') else 57
                                    contact=n.endswith('gripper') and current.get('closure_outcome')=='stationary_closure_unverified'
                                    if abs(q-final[n])>(96 if contact else tolerance):raise RuntimeError('Pickup completion contradicts measured endpoint')
                            elif any(abs(measured[n]-final[n])>(20 if request['op']=='gripper_target' else 5) for n in measured):raise RuntimeError('Completion contradicts measured endpoint')
                        return {'accepted':True,'completed':True,'command_id':command_id,'owner_started':started,'readbacks':measured,
                            'endpoint_reached':current.get('endpoint_reached'),'settle_residual_ticks':current.get('settle_residual_ticks'),'execution_profile':current.get('execution_profile'),'grasp_verified':current.get('grasp_verified',False),'closure_outcome':current.get('closure_outcome'),'gripper_result':current.get('gripper_result'),'owner_status_time':current['time'],'duration_s_actual':current.get('direct_duration_s'),
                            'motor_writes':'canonical owner only','mode':'direct_joint'}
                    self.sleep(.02)
                raise RuntimeError('Owner completion timed out')
        except BaseException:
            if dispatched:self.stop(expected_started=started)
            raise
        finally:self.lock.release()
