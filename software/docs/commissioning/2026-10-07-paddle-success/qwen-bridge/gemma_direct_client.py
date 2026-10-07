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
            'remote_owner_start_or_stop_reset':False,'continuous_profile_required':False,
            'cartesian_transform_required':False,'camera_gate_required':False,'software_temperature_limit_c':SOFTWARE_TEMPERATURE_LIMIT_C}
        try:
            state=self.status();selected=state.get('supportsselectedjoints',[])
            result.update(owner_started=state.get('started'),owner_phase=state.get('phase'),
                owner_status_age_s=state['status_age_s'],supported_joints=selected,
                supported_motors=state.get('supported_motors',[]),enabled_motors=state.get('enabled_motors',[]),
                operator_armed=state.get('operator_armed') is True,local_operator_gate=state.get('operator_armed') is True,
                motor_owner_active=state.get('hardware_server') is True and 0<=state['status_age_s']<=1)
            result.update(execution_profile=state.get('execution_profile','legacy-direct'),read_only=state.get('read_only') is True,calibration_mismatches=state.get('calibration_mismatches',{}))
            if state.get('read_only') is True:result['blockers'].append('READ_ONLY_OWNER: calibration mismatch blocks activation')
            if state.get('control_mode')!='direct_joint' or state.get('hardware_server') is not True:result['blockers'].append('HARDWARE_OWNER_PROTOCOL_UNAVAILABLE')
            if not 0<=state['status_age_s']<=1:result['blockers'].append('OWNER_STATUS_STALE')
            if state.get('phase') not in ('idle','holding'):result['blockers'].append('OWNER_BUSY_OR_STOP_LATCHED: '+str(state.get('phase')))
            if state.get('ok') is not True:result['blockers'].append('OWNER_NOT_HEALTHY')
            if state.get('operator_armed') is not True:result['blockers'].append('AUTHORIZED_INTERFACE_NOT_AVAILABLE')
            if type(state.get('started')) not in (int,float) or not math.isfinite(state['started']):result['blockers'].append('OWNER_IDENTITY_MISSING')
            if not isinstance(selected,list) or not selected or len(set(selected))!=len(selected):result['blockers'].append('POSITION_SCOPE_INVALID');selected=[]
            rows=state.get('rows',{});torque={}
            for name in state.get('supported_motors',[]):torque[name]=rows.get(name,{}).get('Torque_Enable')
            result['torque_enabled_by_joint']=torque
            result['lease_remaining_s']=state.get('lease_remaining',0)
            result['lease_applies_while_enabled']=True
            result['explicit_enable_renews_idle_lease']=not bool(state.get('enabled_motors'))
            for name in selected:
                issues=[];row=rows.get(name,{})
                if name not in self.calibration or name.startswith('base_'):issues.append('UNSUPPORTED_POSITION_JOINT')
                else:
                    lo,hi=self.calibration[name]['range_min'],self.calibration[name]['range_max']
                    if state.get('ranges',{}).get(name)!=[lo,hi]:issues.append('SAVED_RANGE_MISMATCH')
                    q=row.get('Present_Position')
                    if type(q) not in (int,float) or not math.isfinite(q) or not lo<=q<=hi:issues.append('CURRENT_POSITION_OUTSIDE_SAVED_RANGE')
                for field,valid in [('Status',lambda x:x==0),('Present_Temperature',lambda x:x<=SOFTWARE_TEMPERATURE_LIMIT_C),('Present_Load',lambda x:abs(x)<500)]:
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
        result={'stop_requested':True,'command_id':command['id'],'release_confirmed':False,'stop_reset_supported':False}
        deadline=self.clock()+2
        while self.clock()<deadline:
            try:
                latest=self.status()
                if latest.get('started')!=state.get('started'):
                    result['release_reason']='BOUND_OWNER_CHANGED';return result
                rows=latest.get('rows',{})
                if 0<=latest['status_age_s']<=1 and latest.get('time',0)>=command['id']/1e9 and len(rows)==16 and latest.get('phase')=='stopped' and not latest.get('enabled_motors') and all(r.get('Torque_Enable')==0 for r in rows.values()):
                    result.update(release_confirmed=True,release_owner_time=latest['time'],owner_started=latest['started']);return result
            except (OSError,ValueError,KeyError,TypeError):pass
            self.sleep(.02)
        result['release_reason']='Fresh same-session all16 torque-zero readback not observed';return result
    def set_motor_enable(self,names,enabled):
        if type(enabled) is not bool or not isinstance(names,list) or not names or len(set(names))!=len(names) or any(not isinstance(n,str) for n in names):raise ValueError('Distinct motor names and boolean enabled required')
        return self._command({'op':'enable_motors','names':names,'enabled':enabled})
    def execute(self,positions,duration_s):
        if type(duration_s) not in (int,float) or not math.isfinite(duration_s) or not 0<duration_s<=25:raise ValueError('Duration must be finite in (0,25]')
        if not isinstance(positions,dict) or not positions or any(type(q) is not int for q in positions.values()):raise ValueError('Nonempty integer encoder targets required')
        return self._command({'op':'direct_joint','positions':positions,'duration_s':duration_s})
    def _validate(self,request,state):
        if request['op']=='enable_motors':
            names=request['names']
            if not set(names)<=set(state.get('supported_motors',[])):raise ValueError('Unknown motor names')
            if not request['enabled']:return
            if not set(names)<=set(state.get('commandable_motors',state.get('supported_motors',[]))):raise ValueError('UNSUPPORTED_OWNER_SCOPE: requested motors are read-only')
            for n in names:
                if n not in state.get('supportsselectedjoints',[]):
                    row=state.get('rows',{}).get(n,{})
                    if row.get('Operating_Mode')!=0:raise ValueError('Wheel hold activation requires actual position mode0: '+n)
                    q=row.get('Present_Position')
                    if type(q) not in (int,float) or not math.isfinite(q):raise ValueError('Wheel current encoder missing: '+n)
                    bounds=state.get('wheel_hold_ranges',{}).get(n) or row.get('firmware_position_limits')
                    if bounds is not None and (not isinstance(bounds,list) or len(bounds)!=2 or not bounds[0]<bounds[1] or not bounds[0]<=q<=bounds[1]):raise ValueError('Wheel current position/actual firmware limits invalid: '+n)
        else:
            names=list(request['positions'])
            if not set(names)<=set(state.get('supportsselectedjoints',[])):raise ValueError('Unsupported position motor or wheel target')
            for n,q in request['positions'].items():
                c=self.calibration[n]
                if not c['range_min']+4<=q<=c['range_max']-4:raise ValueError('Target outside saved range plus4tickmargin: '+n)
                if state['rows'][n].get('Torque_Enable')!=1:raise ValueError('Requested motor is released; explicitly enable it first: '+n)
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
            prospective=copy.deepcopy(state);prospective['rows'][name]['Torque_Enable']=1
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
            enabled_here=state['rows'][name]['Torque_Enable']==0;enable_attempted=False;phase='validated'
            try:
                if enabled_here:
                    self._validate({'op':'enable_motors','names':[name],'enabled':True},state)
                    phase='enabling';enable_attempted=True
                    activation=self.set_motor_enable([name],True)
                    if not activation.get('completed'):raise RuntimeError('Gripper enable not completed: '+json.dumps(activation))
                latest=self.status()
                if latest['started']!=started or generation!=self.cancel_generation:raise RuntimeError('Owner/STOP changed during gripper sequence')
                phase='moving';result=self._command(request) if arm=='right' else self.execute({name:position},duration)
                if not result.get('completed'):raise RuntimeError('Gripper move not completed: '+json.dumps(result))
                return dict(result,gripper_auto_enabled=enabled_here,gripper=name,sequence_phase='completed')
            except BaseException as exc:
                cleanup='not_requested'
                if enable_attempted:
                    try:cleanup=self.stop(expected_started=started)
                    except Exception as cleanup_error:cleanup={'release_confirmed':False,'error':str(cleanup_error)}
                raise RuntimeError('Gripper sequence failed in '+phase+'; cleanup='+json.dumps(cleanup)+'; '+str(exc)) from exc

    def _command(self,request):
        with self.serialized():return self._command_locked(request)

    def _command_locked(self,request):
        release=request['op']=='enable_motors' and request['enabled'] is False
        ready=self.readiness()
        if not ready['available_to_accept_authorized_command'] and not release:return {'accepted':False,'motor_writes':0,'reason':ready['blocker'],'readiness':ready}
        if not self.lock.acquire(blocking=False):raise RuntimeError('Hardware command active; STOP remains independently available')
        dispatched=False;started=None
        try:
            with contextlib.nullcontext(self.sequence_local.writer) as writer:
                state=self.status();started=state['started'];generation=self.cancel_generation
                if state.get('hardware_server') is not True or not 0<=state['status_age_s']<=1:raise RuntimeError('Fresh hardware owner unavailable')
                if not release and (state.get('phase') not in ('idle','holding') or state.get('operator_armed') is not True or state.get('ok') is not True):raise RuntimeError('Owner busy, unsafe or STOP latched')
                self._validate(request,state)
                if not release and state.get('enabled_motors') and state.get('lease_remaining',0)<=(0 if state.get('execution_profile')=='paddle-success-v1' else request.get('duration_s',0)+5):raise RuntimeError('Insufficient owner lease')
                command_file=self.folder/'command.json';old=json.loads(command_file.read_text()) if command_file.exists() else None
                command_id=max(time.time_ns(),int((old or {}).get('id',0))+1)
                command={**request,'id':command_id,'session_started':started}
                latest=self.status()
                if latest['started']!=started or not 0<=latest['status_age_s']<=1 or (not release and (latest.get('phase') not in ('idle','holding') or latest.get('operator_armed') is not True or latest.get('ok') is not True)):raise RuntimeError('Owner changed before dispatch')
                if generation!=self.cancel_generation:raise RuntimeError('STOP interrupted dispatch')
                if (json.loads(command_file.read_text()) if command_file.exists() else None)!=old:raise RuntimeError('Another writer changed command file')
                atomic_json(command_file,command);dispatched=True
                deadline=self.clock()+(60 if request['op']=='gripper_target' or state.get('execution_profile')=='paddle-success-v1' and request['op']=='direct_joint' else 30 if request['op']=='direct_joint' else 5)
                while self.clock()<deadline:
                    if generation!=self.cancel_generation:raise RuntimeError('STOP cancelled goal; no automatic resume')
                    current=self.status()
                    if current.get('started')!=started:raise RuntimeError('Bound owner restarted')
                    if not 0<=current['status_age_s']<=1:raise RuntimeError('Owner telemetry stale')
                    if not release and (current.get('phase')=='stopped' or current.get('ok') is not True):raise RuntimeError('Owner stopped: '+str(current.get('error')))
                    if (current.get('last_rejected') or {}).get('id')==command_id:raise RuntimeError('Owner rejected: '+str(current['last_rejected'].get('reason')))
                    if json.loads(command_file.read_text()).get('id')!=command_id:raise RuntimeError('Command overwritten; cancelled')
                    if current.get('completed')==command_id and current.get('phase') in ('idle','holding','stopped'):
                        if request['op']=='enable_motors':
                            measured={n:current['rows'][n]['Torque_Enable'] for n in request['names']}
                            if any(v!=int(request['enabled']) for v in measured.values()):
                                # The owner echoes completion before its next all-motor
                                # read poll; require actual readback before replying.
                                self.sleep(.02)
                                continue
                        else:
                            measured={n:current['rows'][n]['Present_Position'] for n in request['positions']}
                            if current.get('execution_profile')=='paddle-success-v1':
                                for n,q in measured.items():
                                    tolerance=30 if n.endswith('gripper') else 57
                                    contact=n.endswith('gripper') and current.get('closure_outcome')=='stationary_closure_unverified'
                                    if abs(q-request['positions'][n])>(96 if contact else tolerance):raise RuntimeError('Pickup completion contradicts measured endpoint')
                            elif any(abs(measured[n]-request['positions'][n])>(20 if request['op']=='gripper_target' else 5) for n in measured):raise RuntimeError('Completion contradicts measured endpoint')
                        return {'accepted':True,'completed':True,'command_id':command_id,'owner_started':started,'readbacks':measured,
                            'execution_profile':current.get('execution_profile'),'grasp_verified':current.get('grasp_verified',False),'closure_outcome':current.get('closure_outcome'),'gripper_result':current.get('gripper_result'),'owner_status_time':current['time'],'duration_s_actual':current.get('direct_duration_s'),
                            'motor_writes':'canonical owner only','mode':'direct_joint'}
                    self.sleep(.02)
                raise RuntimeError('Owner completion timed out')
        except BaseException:
            if dispatched:self.stop(expected_started=started)
            raise
        finally:self.lock.release()
