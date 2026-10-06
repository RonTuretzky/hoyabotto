"""Guarded physical PR3282 launcher. Imports/connects only when explicitly executed."""
import argparse, json, pathlib, shutil, sys, time
SOFTWARE = pathlib.Path(__file__).resolve().parents[2]
PORTS = ['/dev/cu.usbmodem5B790186401', '/dev/cu.usbmodem5B790182091']
CAL = pathlib.Path('/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json')
# Servo temperature is deliberately not read or checked anywhere here (owner's decision, 2026-10-05).
class CalibrationAbort(BaseException):
    """Not catchable by vendor COMM_ERR recovery/suppression."""

class ExpectedCalibrationOverload(RuntimeError):
    """Only servo BIT5; vendor mechanical-stop recovery may handle this."""

def install_calibration_reply_guard(bus):
    """Strict lengths and fail-closed transport before SDK decoders/retries."""
    ph=bus.packet_handler
    old_tx,old_rx,old_txrx=ph.txPacket,ph.rxPacket,ph.txRxPacket
    expected={'size':None,'address':None,'motor_id':None}
    bus.calibration_reply_evidence=[]
    def tx(port,packet):
        if not port.is_using:
            expected.update(size=packet[6] if packet[4] in (2,130) else 0,address=packet[5] if packet[4] in (2,130) else None,motor_id=packet[2])
        result=old_tx(port,packet)
        if result!=0:raise CalibrationAbort(f'SDK transmit failure {result}')
        return result
    def rx(port):
        try:packet,result=old_rx(port)
        except Exception as e:raise CalibrationAbort(f'SDK malformed receive: {e}') from e
        evidence={'time':time.time(),'packet':list(packet),'communication':result,'expected_payload':expected['size'],'request_address':expected['address'],'request_motor_id':expected['motor_id']}
        bus.calibration_reply_evidence.append(evidence)
        del bus.calibration_reply_evidence[:-64]
        if result!=0:raise CalibrationAbort(f'SDK receive communication failure {result}; {evidence}')
        size=expected['size']
        if len(packet)<6 or (size is not None and (packet[3]!=size+2 or len(packet)!=size+6)):
            raise CalibrationAbort(f'SDK malformed reply length; {evidence}')
        flags=packet[4]
        if flags & ~32:raise CalibrationAbort(f'SDK unexpected servo fault {flags}; {evidence}')
        if flags==32 and not (expected['address']==56 and expected['size']==15):raise ExpectedCalibrationOverload(f'Expected mechanical-stop BIT5 overload; {evidence}')
        return packet,result
    def txrx(port,packet):
        try:reply,result,error=old_txrx(port,packet)
        except (CalibrationAbort,ExpectedCalibrationOverload):raise
        except Exception as e:raise CalibrationAbort(f'SDK transaction failed: {e}') from e
        if result!=0:raise CalibrationAbort(f'SDK transaction communication failure {result}')
        if error & ~32:raise CalibrationAbort(f'SDK transaction unexpected fault {error}')
        if error==32 and not (len(packet)>6 and packet[4]==2 and packet[5:7]==[56,15]):raise ExpectedCalibrationOverload('Expected mechanical-stop BIT5 overload')
        return reply,result,error
    ph.txPacket,ph.rxPacket,ph.txRxPacket=tx,rx,txrx

def validate_health(rows):
    if len(rows) != 6:
        raise CalibrationAbort('Incomplete arm health snapshot')
    for name, r in rows.items():
        # Overload BIT5 is an intentional mechanical-stop signal; other faults abort.
        if int(r['status']) & ~0x20:
            raise CalibrationAbort(f'{name} non-calibration hardware fault {r["status"]}')

def coherent_health(bus,name):
    motor_id=bus.motors[name].id
    try:
        data,comm,error=bus.packet_handler.readTxRx(bus.port_handler,motor_id,56,15)
    except CalibrationAbort:raise
    except Exception as e:raise CalibrationAbort(f'Coherent health read failed {name}/ID{motor_id}: {e}') from e
    evidence={'time':time.time(),'motor':name,'motor_id':motor_id,'address':56,'length':15,'payload':list(data),'communication':comm,'packet_error':error,'raw_replies':list(getattr(bus,'calibration_reply_evidence',[])[-1:])}
    if comm!=0 or error & ~32 or len(data)!=15 or any(type(v) is not int or not 0<=v<=255 for v in data):
        bus.health_fault_snapshot=evidence
        raise CalibrationAbort(f'Invalid coherent health reply: {evidence}')
    row={'status':data[9]|error,'evidence':evidence}
    return row

def poll_health(bus,names):
    rows={}
    try:
        for name in names:
            rows[name]=coherent_health(bus,name)
            if rows[name]['status'] & ~32:
                raise CalibrationAbort(f'{name} health invalid status={rows[name]["status"]}')
        validate_health(rows)
    except BaseException:
        if not hasattr(bus,'health_fault_snapshot'):
            bus.health_fault_snapshot=json.loads(json.dumps({'time':time.time(),'rows':rows,'raw_replies':getattr(bus,'calibration_reply_evidence',[])}))
        raise
    return rows

def validate_measured_range(motor,result,nominal):
    if motor not in nominal:raise CalibrationAbort(f'No independent plausibility bounds for {motor}')
    _,minimum,maximum=nominal[motor]
    span=result[1]-result[0];degrees=span*360/4096
    # Lower NOMINAL bound is a reporting heuristic, not a hardware specification.
    if not 20<=degrees<=maximum:
        raise CalibrationAbort(f'{motor} ambiguous/implausible measured span {span} ticks ({degrees:.2f}deg); validated travel floor20deg / conservative model maximum{maximum}deg; no EEPROM writes accepted')
    if degrees<minimum:
        print(f'WARNING {motor}: measured {degrees:.2f}deg below nominal-report minimum{minimum}deg; retaining actual narrower range, no clamping',flush=True)
    return result

def directed_samples(positions,velocity):
    deltas=[((b-a+2048)%4096)-2048 for a,b in zip(positions,positions[1:])]
    expected=1 if velocity>0 else -1
    return sum(d*expected for d in deltas),sum(max(0,-d*expected) for d in deltas)

def settle_limit_position(bus,name,*,clock=time.monotonic,sleep=time.sleep,timeout_s=1.0):
    """Resample actual endpoint after zero velocity; don't retain pre-stop drift."""
    started=clock();previous=None;stable=0;trace=[]
    while clock()-started<timeout_s:
        if hasattr(bus,'_health'):bus._health()
        row=coherent_health(bus,name);data=row['evidence']['payload']
        if row['status'] & ~32:
            bus.health_fault_snapshot={'rows':{name:row}}
            raise CalibrationAbort(f'{name} invalid endpoint health: {row}')
        pos=data[0]|data[1]<<8;v=data[2]|data[3]<<8;velocity=-(v&32767) if v&32768 else v
        trace.append({'time':clock(),'position':pos,'velocity':velocity,'moving':data[10]})
        if name in getattr(bus,'_direction_samples',{}):bus._direction_samples[name].append(pos)
        stationary=previous is not None and abs(((pos-previous+2048)%4096)-2048)<=3 and abs(velocity)<3 and data[10]==0
        stable=stable+1 if stationary else 0;previous=pos
        if stable>=3:
            bus.last_settled_limit={'motor':name,'position':pos,'samples':trace};return pos
        sleep(0.05)
    raise CalibrationAbort(f'{name} endpoint failed settling after Goal_Velocity0: {trace}')

def wait_guarded_limits(bus,motors,confirm_samples,timeout_s,interval_s,*,velocity_threshold=3,position_delta_threshold=3,clock=time.monotonic,sleep=time.sleep,startup_grace_s=1.0,max_travel_ticks=None,no_progress_s=3.0):
    started=clock();pending=set(motors);previous={};stable={n:0 for n in motors};overload={n:0 for n in motors};reasons={};positions={}
    trace={'started':started,'deadline_s':timeout_s,'samples':{n:[] for n in motors},'pending':list(pending)}
    bus.limit_wait_evidence=getattr(bus,'limit_wait_evidence',[])+[trace]
    travel={n:0 for n in motors};last_progress={n:started for n in motors};progress_position={}
    while pending and clock()-started<timeout_s:
        if hasattr(bus,'_health'):bus._health()
        for n in list(pending):
            row=coherent_health(bus,n);data=row['evidence']['payload']
            if row['status'] & ~32:
                bus.health_fault_snapshot={'rows':{n:row}}
                raise CalibrationAbort(f'{n} health fault during limit seeking: {row}')
            pos=data[0]|data[1]<<8;raw_vel=data[2]|data[3]<<8
            vel=-(raw_vel & 32767) if raw_vel & 32768 else raw_vel
            if n in getattr(bus,'_direction_samples',{}):bus._direction_samples[n].append(pos)
            trace['samples'][n].append({'time':clock(),'position':pos,'velocity':vel,'moving':data[10],'status':row['status']})
            if n in previous:travel[n]+=abs(((pos-previous[n]+2048)%4096)-2048)
            if n not in progress_position or abs(((pos-progress_position[n]+2048)%4096)-2048)>=3:
                progress_position[n]=pos;last_progress[n]=clock()
            if max_travel_ticks and travel[n]>max_travel_ticks[n]:raise CalibrationAbort(f'{n} limit seek exceeded conservative travel cap {travel[n]}>{max_travel_ticks[n]}')
            if clock()-last_progress[n]>no_progress_s and (data[10]!=0 or abs(vel)>=velocity_threshold):raise CalibrationAbort(f'{n} Moving/velocity reported without actual progress for {no_progress_s}s')
            overload[n]=overload[n]+1 if row['status'] & 32 else 0
            stationary=abs(vel)<velocity_threshold and n in previous and abs(((pos-previous[n]+2048)%4096)-2048)<position_delta_threshold and data[10]==0
            stable[n]=stable[n]+1 if stationary and clock()-started>=startup_grace_s else 0
            previous[n]=pos
            if overload[n]>=confirm_samples or stable[n]>=confirm_samples:
                reasons[n]='stall confirmed: Status BIT5 overload' if overload[n]>=confirm_samples else 'limit confirmed after startup grace: stable position + near-zero velocity + Moving=0'
                bus.write('Goal_Velocity',n,0)
                positions[n]=settle_limit_position(bus,n,clock=clock,sleep=sleep)
                pending.remove(n)
                trace['pending']=list(pending);trace.setdefault('settled',{})[n]=positions[n]
        if pending:sleep(interval_s)
    if pending:raise CalibrationAbort(f'Limit-seeking timeout without verified stop: {sorted(pending)}')
    return reasons,positions

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', choices=['left','right'], default='right')
    p.add_argument('--backup-dir', type=pathlib.Path, required=True)
    p.add_argument('--leg-timeout-s',type=float,default=45.0)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--clearance-confirmed',action='store_true',help='Operator confirmed entire selected-arm sweep, other arm, cables and battery switch')
    a=p.parse_args()
    if not 1<=a.leg_timeout_s<=45:p.error('--leg-timeout-s must be1..45')
    if not a.execute:
        print(json.dumps({'plan_only':True,'arm':a.arm,'port':PORTS[0 if a.arm=='left' else 1],'velocity':100,'leg_timeout_s':a.leg_timeout_s,'backup_dir':str(a.backup_dir),'requires_clearance_confirmation':True}));return 0
    if not a.clearance_confirmed:
        p.error('--execute requires --clearance-confirmed')
    a.backup_dir.mkdir(parents=True, exist_ok=False)
    shutil.copy2(CAL, a.backup_dir/'calibration-before.json')
    sys.path.insert(0,str(SOFTWARE)); sys.path.insert(0,str(SOFTWARE/'scripts'))
    from carton_robot.servo_ownership import ServoOwnership
    from farm.vendor.autocal import workflow as w
    from farm.tools.auto_calibrate import merge_arm, CHECKLIST
    from farm.tools.calibration_report import NOMINAL
    names=list(w.MOTOR_NAMES); raw_bus=w.FeetechMotorsBus
    buses=[]
    registers=['Torque_Enable','Operating_Mode','Homing_Offset','Min_Position_Limit','Max_Position_Limit','Torque_Limit','Max_Torque_Limit','Acceleration','P_Coefficient','I_Coefficient','D_Coefficient','Return_Delay_Time','Lock']
    class GuardedBus(raw_bus):
        _last_health=0
        _closing=False
        def disconnect(self,*args,**kwargs):
            if self._closing:return super().disconnect(*args,**kwargs)
            # Vendor finalizer requests disconnect; outer release owns shutdown.
            return None
        def _wait_for_stall(self,motor,stall_confirm_samples,timeout_s,sample_interval_s,**kwargs):
            reasons,_=wait_guarded_limits(self,[motor],stall_confirm_samples,timeout_s,sample_interval_s,max_travel_ticks={motor:int(NOMINAL[motor][2]*4096/360)+20},**kwargs)
            return reasons[motor]
        def _wait_for_stall_multi(self,motors,stall_confirm_samples,timeout_s,sample_interval_s,**kwargs):
            return wait_guarded_limits(self,motors,stall_confirm_samples,timeout_s,sample_interval_s,max_travel_ticks={n:int(NOMINAL[n][2]*4096/360)+20 for n in motors},**kwargs)
        def _compute_mid_and_range_from_limits(self,motor,pos_cw,pos_ccw,**kwargs):
            result=super()._compute_mid_and_range_from_limits(motor,pos_cw,pos_ccw,**kwargs)
            return validate_measured_range(motor,result,NOMINAL)
        def _run_direction_until_stall(self,motors,velocity,**kwargs):
            values=velocity if isinstance(velocity,dict) else dict.fromkeys(motors,velocity)
            initial={n:self.read('Present_Position',n,normalize=False) for n in motors}
            self._direction_samples={n:[initial[n]] for n in motors}
            try:
                reasons,positions=super()._run_direction_until_stall(motors,velocity,**kwargs)
                audit={}
                for n in motors:
                    samples=self._direction_samples[n]+[positions[n]]
                    progress,opposite=directed_samples(samples,values[n])
                    audit[n]={'velocity':values[n],'samples':samples,'directed_progress':progress,'opposite_travel':opposite,'reason':reasons[n]}
                    self.limit_fault_snapshot={'motor':n,'leg':audit[n]}
                    (a.backup_dir/'limit-leg-current.json').write_text(json.dumps(self.limit_fault_snapshot,indent=2))
                    if 'timeout' in reasons[n] or 'communication error' in reasons[n] or opposite>20:
                        raise CalibrationAbort(f'{n} invalid mechanical-stop leg: {audit[n]}')
                    previous=getattr(self,'_prior_limit_leg',{}).get(n)
                    if previous is not None:
                        minimum=20*4096/360
                        if progress<minimum:raise CalibrationAbort(f'{n} reversal stalled without spanning plausible joint travel: {audit[n]}, minimum {minimum:.1f}ticks')
                self._prior_limit_leg=getattr(self,'_prior_limit_leg',{})|audit
                (a.backup_dir/'limit-leg-evidence.json').write_text(json.dumps(self._prior_limit_leg,indent=2))
                return reasons,positions
            finally:self._direction_samples={}
        def _health(self):
            poll_health(self,names)
            self._last_health=time.monotonic()
        def read(self,*args,**kwargs):
            try:
                if getattr(self,'_guard_enabled',False) and time.monotonic()-self._last_health>=0.05:self._health()
                value=super().read(*args,**kwargs)
                if len(args)>=2 and args[0]=='Present_Position' and args[1] in getattr(self,'_direction_samples',{}):self._direction_samples[args[1]].append(value)
                return value
            except ExpectedCalibrationOverload:raise
            except Exception as e:raise CalibrationAbort(f'Communication read abort {args}: {e}') from e
        def write(self,reg,*args,**kwargs):
            try:
                self._health()
                return super().write(reg,*args,**kwargs)
            except (CalibrationAbort,ExpectedCalibrationOverload):raise
            except Exception as e:raise CalibrationAbort(f'Communication write abort {reg}: {e}') from e
        def sync_write(self,*args,**kwargs):
            try:
                self._health(); return super().sync_write(*args,**kwargs)
            except (CalibrationAbort,ExpectedCalibrationOverload):raise
            except Exception as e:raise CalibrationAbort(f'Communication sync-write abort: {e}') from e
    def connect(port,motor_names=None):
        if motor_names is not None and set(motor_names)!=set(names):raise CalibrationAbort("Full-arm launcher requires exactly the six arm motors")
        b=GuardedBus(port=port,motors=w.SO_FOLLOWER_MOTORS.copy()); buses.append(b)
        install_calibration_reply_guard(b)
        b.connect()
        before={n:{r:b.read(r,n,normalize=False) for r in registers} for n in names}
        (a.backup_dir/'registers-before.json').write_text(json.dumps(before,indent=2))
        for n,r in before.items():
            if r['Torque_Enable'] != 0:raise CalibrationAbort(f'{n} unexpectedly powered')
        b._health(); b._guard_enabled=True; return b
    print(CHECKLIST.format(arm=a.arm),flush=True)
    if input('Type yes after clearing the entire arm sweep: ').strip().lower() != 'yes':return 2
    lock=ServoOwnership(PORTS).acquire()
    try:
        w._connect_and_clear=connect
        sink={}
        rc=w.run_full_calibration(PORTS[0 if a.arm=='left' else 1],save=True,velocity_limit=100,timeout_s=a.leg_timeout_s,interactive=True,result_sink=sink)
        if rc or set(sink)!=set(names):raise CalibrationAbort(f'Incomplete calibration rc={rc}')
        # Persist via wrapper only after all six results returned.
        merge_arm(CAL,a.arm,sink)
        (a.backup_dir/'calibration-after.json').write_text(CAL.read_text())
        return 0
    except BaseException as e:
        (a.backup_dir/'failure.json').write_text(json.dumps({'error':str(e),'type':type(e).__name__,'time':time.time(),'calibration_valid':False,'restore_status':'Not automatically restored; partial homing/EEPROM changes may exist. Preserve registers-before and recalibrate or deliberately restore after inspecting failure.'}))
        print(f'Calibration aborted: {e}',file=sys.stderr);return 1
    finally:
        for b in buses:
            if hasattr(b,'limit_wait_evidence'):(a.backup_dir/'limit-wait-evidence.json').write_text(json.dumps(b.limit_wait_evidence,indent=2))
            if hasattr(b,'health_fault_snapshot'):
                (a.backup_dir/'health-fault-frozen.json').write_text(json.dumps(b.health_fault_snapshot,indent=2))
        # Use raw writes for release even if health checks/transport already failed.
        release={}
        packets={}
        for b in buses:
            packets[str(b.port if hasattr(b,'port') else 'arm')]=getattr(b,'calibration_reply_evidence',[])
            for n in names:
                try:
                    raw_bus.write(b,'Goal_Velocity',n,0);raw_bus.write(b,'Torque_Enable',n,0)
                    release[n]=raw_bus.read(b,'Torque_Enable',n,normalize=False)==0
                except BaseException as e:release[n]={'confirmed':False,'error':str(e)}
            try:
                b._closing=True
                b.disconnect()
            except BaseException:pass
        (a.backup_dir/'release.json').write_text(json.dumps(release,indent=2))
        (a.backup_dir/'raw-replies.json').write_text(json.dumps(packets,indent=2))
        lock.close()
        if any(value is not True for value in release.values()):
            return 1
if __name__=='__main__':sys.exit(main())
