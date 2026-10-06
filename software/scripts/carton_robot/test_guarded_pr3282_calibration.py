import importlib.util, pathlib, tempfile, unittest, types, sys
from unittest.mock import patch
spec=importlib.util.spec_from_file_location('guard',pathlib.Path(__file__).with_name('guarded_pr3282_calibration.py'));g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
class Tests(unittest.TestCase):
 def run_case(self,problem=None):
  with tempfile.TemporaryDirectory() as d:
   root=pathlib.Path(d);cal=root/'cal.json';cal.write_text('{}');writes=[];locks=[]
   class Bus:
    def __init__(self,**kw):
     self.motors={n:types.SimpleNamespace(id=i+1) for i,n in enumerate('abcdef')};self.port_handler=object()
     def coherent(*args):
      data=[0]*15;data[7]=132 if problem=='hot' else 38;data[9]=4 if problem=='fault' else 0;return data,0,0
     self.packet_handler=types.SimpleNamespace(txPacket=lambda *a:0,rxPacket=lambda *a:([255,255,1,2,0,0],0),txRxPacket=lambda *a:([],0,0),readTxRx=coherent)
    def connect(self):pass
    def disconnect(self):pass
    def read(self,r,n,**kw):
     if problem=='transport' and r=='Present_Position':raise IOError('corrupt')
     return {'Torque_Enable':0,'Present_Temperature':90 if problem=='hot' else 38,'Status':4 if problem=='fault' else 0,'Max_Temperature_Limit':70,'Unloading_Condition':44}.get(r,100)
    def write(self,r,n,v,**kw):writes.append((r,n,v))
    def sync_write(self,*args,**kw):pass
    def _compute_mid_and_range_from_limits(self,*args,**kw):return (6,4088,1207,3247,3261,-841)
   class Lock:
    def __init__(self,p):locks.extend(p)
    def acquire(self):return self
    def close(self):pass
   w=types.ModuleType('farm.vendor.autocal.workflow');w.FeetechMotorsBus=Bus;w.MOTOR_NAMES=list('abcdef');w.SO_FOLLOWER_MOTORS={n:object() for n in w.MOTOR_NAMES}
   def workflow(port,**kw):
    b=w._connect_and_clear(port)
    if problem=='badrange':b._compute_mid_and_range_from_limits('elbow_flex',3261,3247,reference_pos=2952)
    if problem=='transport':b.read('Present_Position','a')
    if problem=='protected':b.write('Max_Temperature_Limit','a',100)
    b.write('Goal_Position','a',200)
    kw['result_sink'].update({n:{} for n in w.MOTOR_NAMES});b.disconnect();return 0
   w.run_full_calibration=workflow
   ac=types.ModuleType('farm.tools.auto_calibrate');ac.CHECKLIST='clear {arm}';ac.merge_arm=lambda *a:None
   report=types.ModuleType('farm.tools.calibration_report');report.NOMINAL={'elbow_flex':(194,115,225)}
   owner=types.ModuleType('carton_robot.servo_ownership');owner.ServoOwnership=Lock
   auto=types.ModuleType('farm.vendor.autocal');auto.workflow=w
   mods={'farm':types.ModuleType('farm'),'farm.vendor':types.ModuleType('farm.vendor'),'farm.vendor.autocal':auto,'farm.tools':types.ModuleType('farm.tools'),'farm.tools.auto_calibrate':ac,'farm.tools.calibration_report':report,'carton_robot':types.ModuleType('carton_robot'),'carton_robot.servo_ownership':owner}
   with patch.dict(sys.modules,mods),patch.object(g,'CAL',cal),patch('builtins.input',return_value='yes'),patch.object(sys,'argv',['launch','--execute','--clearance-confirmed','--backup-dir',str(root/'backup')]):rc=g.main()
   return rc,writes,locks,json_load(root/'backup')
 def test_raw_transport_cannot_be_mechanical_stop(self):
  bus=types.SimpleNamespace(packet_handler=types.SimpleNamespace(txPacket=lambda *a:0,rxPacket=lambda *a:([],-7),txRxPacket=lambda *a:([],-7,0)))
  g.install_calibration_reply_guard(bus)
  with self.assertRaises(g.CalibrationAbort):bus.packet_handler.txRxPacket(None,[255,255,1,3,2,56,15])
 def test_raw_short_reply_aborts(self):
  bus=types.SimpleNamespace(packet_handler=types.SimpleNamespace(txPacket=lambda *a:0,rxPacket=lambda *a:([255,255,1,2,0,0],0),txRxPacket=lambda *a:([],0,0)))
  g.install_calibration_reply_guard(bus);port=types.SimpleNamespace(is_using=False)
  bus.packet_handler.txPacket(port,[255,255,1,4,2,56,15])
  with self.assertRaises(g.CalibrationAbort):bus.packet_handler.rxPacket(port)
 def test_raw_overload_is_expected_stop(self):
  bus=types.SimpleNamespace(packet_handler=types.SimpleNamespace(txPacket=lambda *a:0,rxPacket=lambda *a:([255,255,1,2,32,0],0),txRxPacket=lambda *a:([],0,32)))
  g.install_calibration_reply_guard(bus)
  with self.assertRaises(g.ExpectedCalibrationOverload):bus.packet_handler.rxPacket(None)
  with self.assertRaises(g.ExpectedCalibrationOverload):bus.packet_handler.txRxPacket(None,[])
 def test_coherent_offsets_and_overload(self):
  data=[0]*15;data[7]=39;data[9]=32;data[6]=121
  bus=types.SimpleNamespace(motors={'a':types.SimpleNamespace(id=3)},port_handler=None,packet_handler=types.SimpleNamespace(readTxRx=lambda *a:(data,0,32)))
  row=g.coherent_health(bus,'a');self.assertEqual(row['temperature'],39);self.assertEqual(row['status'],32);self.assertEqual(row['evidence']['length'],15)
 def test_coherent_corrupt_bytes_abort(self):
  bus=types.SimpleNamespace(motors={'a':types.SimpleNamespace(id=3)},port_handler=None,packet_handler=types.SimpleNamespace(readTxRx=lambda *a:([256]*15,0,0)))
  with self.assertRaises(g.CalibrationAbort):g.coherent_health(bus,'a')
 def test_hot_snapshot_frozen(self):
  rc,w,l,e=self.run_case('hot');snapshot=e['health-fault-frozen.json'];self.assertEqual(snapshot['rows']['a']['temperature'],132);self.assertEqual(snapshot['rows']['a']['evidence']['payload'][7],132);self.assertFalse(any(r=='Goal_Position' for r,n,v in w))
 def test_range_guard_blocks_workflow_before_eeprom(self):
  rc,w,l,e=self.run_case('badrange');self.assertEqual(rc,1);self.assertFalse(any(r in ['Goal_Position','Homing_Offset','Min_Position_Limit','Max_Position_Limit'] for r,n,v in w));self.assertTrue(all(e['release.json'].values()))
 def test_false_near_full_elbow_range_rejected(self):
  with self.assertRaises(g.CalibrationAbort):g.validate_measured_range('elbow_flex',(6,4088,1207,3247,3261,-841),{'elbow_flex':(194,115,225)})
 def test_tiny_range_not_clamped(self):
  with self.assertRaises(g.CalibrationAbort):g.validate_measured_range('elbow_flex',(2041,2055,0,0,14,0),{'elbow_flex':(194,115,225)})
 def test_plausible_range_preserved(self):
  result=(1000,3100,2050,0,0,2);self.assertIs(g.validate_measured_range('elbow_flex',result,{'elbow_flex':(194,115,225)}),result)
 def test_direction_wrap_tracking(self):
  self.assertEqual(g.directed_samples([4080,4094,10,40],100),(56,0))
  self.assertEqual(g.directed_samples([10,4094,4080],-100),(26,0))
 def test_opposite_progress_visible(self):self.assertEqual(g.directed_samples([3200,3210,3220],-100),(-20,20))
 def wait_trace(self,trace,timeout=2,**kwargs):
  now=[0.0];writes=[]
  def read(*args):
   pos,vel,moving,status=trace(now[0]);data=[0]*15;data[0]=pos&255;data[1]=pos>>8;data[2]=vel&255;data[3]=vel>>8;data[7]=38;data[9]=status;data[10]=moving;return data,0,0
  bus=types.SimpleNamespace(motors={'a':types.SimpleNamespace(id=1)},port_handler=None,packet_handler=types.SimpleNamespace(readTxRx=read),write=lambda *a:writes.append((now[0],a)))
  result=g.wait_guarded_limits(bus,['a'],2,timeout,0.05,clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt),**kwargs)
  return result,now[0],writes
 def test_delayed_start_not_false_stationary_stop(self):
  def trace(t):
   if t<0.3:return 100,0,0,0
   if t<1.3:return 100+int((t-0.3)*200),100,1,0
   return 300,0,0,0
  result,elapsed,writes=self.wait_trace(trace);self.assertEqual(result[1]['a'],300);self.assertGreater(elapsed,1.3);self.assertEqual(len(writes),1)
 def test_true_start_at_stop_waits_grace(self):
  result,elapsed,writes=self.wait_trace(lambda t:(100,0,0,0));self.assertGreaterEqual(elapsed,1.0);self.assertEqual(result[1]['a'],100)
 def test_confirmed_overload_stops_without_grace(self):
  result,elapsed,writes=self.wait_trace(lambda t:(100,0,0,32));self.assertLess(elapsed,1);self.assertIn('BIT5',result[0]['a'])
 def test_moving_flag_zero_during_velocity_travel_not_stop(self):
  with self.assertRaises(g.CalibrationAbort):self.wait_trace(lambda t:(100+int(t*100),100,0,0),timeout=1.5)
 def test_narrower_than_nominal_measured_range_warns_not_clamps(self):
  result=(1401,2695,0,1499,2793,0)
  self.assertIs(g.validate_measured_range('elbow_flex',result,{'elbow_flex':(194,115,225)}),result)
 def test_near_zero_and_near_full_remain_rejected(self):
  for span in [14,4082]:
   with self.assertRaises(g.CalibrationAbort):g.validate_measured_range('elbow_flex',(0,span,0,0,span,0),{'elbow_flex':(194,115,225)})
 def test_endpoint_resampled_after_zero_velocity_drift(self):
  now=[0.0];positions=iter([1564,1510,1499,1499,1499,1499]);last=[1499]
  def read(*a):
   pos=next(positions,last[0]);last[0]=pos;data=[0]*15;data[0]=pos&255;data[1]=pos>>8;data[7]=38;return data,0,0
  bus=types.SimpleNamespace(motors={'a':types.SimpleNamespace(id=1)},port_handler=None,packet_handler=types.SimpleNamespace(readTxRx=read))
  actual=g.settle_limit_position(bus,'a',clock=lambda:now[0],sleep=lambda d:now.__setitem__(0,now[0]+d));self.assertEqual(actual,1499);self.assertGreaterEqual(len(bus.last_settled_limit['samples']),6)
 def test_endpoint_unsettled_aborts(self):
  now=[0.0]
  def read(*a):
   pos=100+int(now[0]*100);data=[0]*15;data[0]=pos&255;data[1]=pos>>8;data[2]=100;data[7]=38;return data,0,0
  bus=types.SimpleNamespace(motors={'a':types.SimpleNamespace(id=1)},port_handler=None,packet_handler=types.SimpleNamespace(readTxRx=read))
  with self.assertRaises(g.CalibrationAbort):g.settle_limit_position(bus,'a',clock=lambda:now[0],sleep=lambda d:now.__setitem__(0,now[0]+d))
 def test_slow_leg_runs_beyond_20_then_stops_before_45(self):
  result,elapsed,writes=self.wait_trace(lambda t:(100+int(min(t,25)*80),80 if t<25 else 0,1 if t<25 else 0,0),timeout=45,max_travel_ticks={'a':2580});self.assertGreater(elapsed,25);self.assertLess(elapsed,45);self.assertEqual(result[1]['a'],2100)
 def test_never_stops_deadline_is_failure(self):
  with self.assertRaises(g.CalibrationAbort):self.wait_trace(lambda t:(100+int(t*50),50,1,0),timeout=45)
 def test_excessive_travel_aborts_before_deadline(self):
  with self.assertRaises(g.CalibrationAbort):self.wait_trace(lambda t:(100+int(t*100),100,1,0),timeout=45,max_travel_ticks={'a':240})
 def test_reported_moving_without_progress_aborts(self):
  with self.assertRaises(g.CalibrationAbort):self.wait_trace(lambda t:(100,100,1,0),timeout=45)
 def test_success(self):
  rc,w,l,e=self.run_case();self.assertEqual(rc,0);self.assertEqual(l,g.PORTS);self.assertTrue(all(e['release.json'].values()));self.assertIn('registers-before.json',e)
 def test_transport_aborts(self):
  rc,w,l,e=self.run_case('transport');self.assertEqual(rc,1);self.assertFalse(any(r=='Goal_Position' for r,n,v in w));self.assertFalse(e['failure.json']['calibration_valid'])
 def test_hot_aborts_before_goal(self):
  rc,w,l,e=self.run_case('hot');self.assertEqual(rc,1);self.assertFalse(any(r=='Goal_Position' for r,n,v in w))
 def test_other_fault_aborts(self):self.assertEqual(self.run_case('fault')[0],1)
 def test_protection_write_blocked(self):
  rc,w,l,e=self.run_case('protected');self.assertEqual(rc,1);self.assertFalse(any(r=='Max_Temperature_Limit' for r,n,v in w))
 def test_stall_bit_allowed(self):g.validate_health({str(i):{'temperature':55,'status':32} for i in range(6)})
def json_load(root):
 import json
 return {p.name:json.loads(p.read_text()) for p in root.glob('*.json')}
if __name__=='__main__':unittest.main()
