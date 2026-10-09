"""Original controls -> physical adapter -> actual owner/API, all fake motors.

No robot transport, serial access or network permitted. References are generated
inside tests and explicitly synthetic; none is a deployable calibration.
"""
import copy
import json
import contextlib
import io
import tempfile
from pathlib import Path
import time
import unittest
from unittest.mock import patch

from bridge import Bridge
from simulator import SimulatedRobot
from upstream_hardware import UpstreamHardware,PhysicalReference
from joycon_reference import template,POSITION_NAMES,SOURCE_BINDING,digest
from test_upstream import original_frame


def reference_record():
    calibration={n:dict(range_min=100,range_max=4000,homing_offset=0) for n in POSITION_NAMES}
    record=template(calibration,'SYNTHETIC_TEST_ONLY')
    record.update(verified=True,evidence='Synthetic unit-test geometry, not hardware evidence')
    for n,row in record['joints'].items():
        row['evidence']='Synthetic unit-test reference'
        if n.endswith('gripper'):row.update(closed_tick=2048,open_tick=3048)
        else:row.update(reference_tick=2048,reference_degrees=0.,model_sign=1)
    return record


class ReferenceTests(unittest.TestCase):
    def test_installation_lock_rejects_before_transport_or_reader(self):
        import bridge
        with tempfile.TemporaryDirectory() as folder:
            lock=Path(folder)/'MOTOR_CONTROL_DISABLED';lock.touch()
            with patch.object(bridge,'MOTOR_CONTROL_LOCK',lock),patch.object(bridge,'Robot',side_effect=AssertionError('No connection')),patch.object(bridge,'Bridge',side_effect=AssertionError('No input reader')),patch('sys.argv',['bridge.py','--connect-robot']),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as e:bridge.main()
            self.assertEqual(e.exception.code,2)

    def test_requires_measured_identity_all_joints_and_matching_calibration(self):
        record=reference_record();reference=PhysicalReference(record)
        reference.validate_calibration(record['calibration'])
        bad=[]
        for key,value in (('verified',False),('source_binding','wrong'),('evidence','')):
            r=copy.deepcopy(record);r[key]=value;bad.append(r)
        r=copy.deepcopy(record);del r['joints']['head_motor_2'];bad.append(r)
        for name,field,value in (('left_arm_shoulder_lift','model_sign',None),('right_arm_gripper','open_tick',2048),('right_arm_wrist_roll','reference_tick',float('nan'))):
            r=copy.deepcopy(record);r['joints'][name][field]=value;bad.append(r)
        for r in bad:
            with self.assertRaises((ValueError,KeyError)):PhysicalReference(r)
        changed=copy.deepcopy(record['calibration']);changed['head_motor_1']['homing_offset']=1
        with self.assertRaisesRegex(ValueError,'calibration'):reference.validate_calibration(changed)

    def test_source_binding_and_invalid_reference_precede_transport(self):
        rows=json.loads(Path(__file__).with_name('vendor').joinpath('manifest.json').read_text())
        self.assertEqual(SOURCE_BINDING,digest({r['path']:r['sha256'] for r in rows}))
        import bridge
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'unverified.json';record=reference_record();record['verified']=False
            path.write_text(json.dumps(record))
            argv=['bridge.py','--connect-robot','--control-mode','upstream','--input-backend','hid','--upstream-reference',str(path)]
            with patch('sys.argv',argv),patch.object(bridge,'Robot',side_effect=AssertionError('Must not load credentials or connect')):
                with self.assertRaisesRegex(ValueError,'not verified'):bridge.main()
            with patch('sys.argv',argv[:-2]),patch.object(bridge,'Robot',side_effect=AssertionError('Must not connect')),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):bridge.main()

    def test_encoder_direction_gripper_endpoints_and_roundtrip(self):
        r=reference_record();r['joints']['right_arm_wrist_roll']['model_sign']=-1
        r['joints']['left_arm_gripper'].update(closed_tick=3000,open_tick=1500)
        ref=PhysicalReference(r)
        self.assertLess(ref.to_ticks('right_arm_wrist_roll',30),2048)
        self.assertEqual(ref.raw_limits('right_arm_wrist_roll'),(140,3960))
        self.assertEqual(ref.to_ticks('left_arm_gripper',90),1500)
        for n in POSITION_NAMES:
            for value in (0,30,60):self.assertAlmostEqual(ref.from_ticks(n,ref.to_ticks(n,value)),value)
        with self.assertRaises(ValueError):ref.to_ticks('right_arm_shoulder_pan',999)
        with self.assertRaises(ValueError):ref.to_ticks('right_arm_gripper',91)


class HardwareAdapterTests(unittest.TestCase):
    def make_reference(self):return PhysicalReference(reference_record())

    def test_slow_claim_returns_fresh_post_enable_feedback(self):
        original=self.plant.call
        def delayed(path,body=None,timeout=None):
            if path=='claim':time.sleep(.3)
            return original(path,body,timeout)
        with patch.object(self.plant,'call',side_effect=delayed):
            result=self.robot.call('claim',{'scope':'wholebody'})
        self.assertIn('status',result)
        self.assertTrue(self.robot.check_feedback()['ok'])
        self.assertEqual(set(self.robot.state['enabled_motors']),set(POSITION_NAMES))
        self.robot.call('release',{'token':result['token']})
    def setUp(self):
        self.no_network=patch('socket.socket',side_effect=AssertionError('Network forbidden in hardware adapter tests'));self.no_network.start()
        self.ref=self.make_reference()
        self.plant=SimulatedRobot(upstream_reference=self.ref)
        self.robot=UpstreamHardware(self.plant,self.ref);self.robot.refresh()
        self.mapping=self.robot.make_mapping();self.bridge=Bridge(self.robot,None,start=False,mapping=self.mapping,input_backend='hid')
        self.f=original_frame();self.c=self.f['controllers'][0]
        for side in ('Left','Right'):
            for b in ('SL','SR'):self.c['buttons'][side+' Rail '+b]={'pressed':False}
        self.rails(True);self.feed();self.rails(False);self.feed()
    def tearDown(self):self.bridge.close();self.plant.close();self.no_network.stop()
    def feed(self):
        self.f['timestamp']=time.time();self.f['sequence']+=1;self.bridge.ui_seen=time.monotonic();self.bridge.receive(self.f)
    def rails(self,held):
        for side in ('Left','Right'):self.c['buttons'][side+' Rail SL']['pressed']=held
    def start(self):
        self.feed();self.bridge.action({'op':'arm','scope':'wholebody'});self.assertTrue(self.bridge.armed)
    def tick(self,n=1):
        for _ in range(n):
            self.feed();self.bridge.step();time.sleep(.035)
            self.assertTrue(self.bridge.armed,self.bridge.reason)
    def test_actual_owner_requires_exact_installed_reference_before_enabling(self):
        before={n:r['Torque_Enable'] for n,r in self.plant.bus.r.items()}
        for body in ({'scope':'wholebody'},{'scope':'wholebody','control_mode':'upstream','reference_id':'0'*64}):
            with self.assertRaises(ValueError):self.plant.call('claim',body)
            self.assertEqual(before,{n:r['Torque_Enable'] for n,r in self.plant.bus.r.items()})
        self.start();self.assertTrue(self.plant.owner.teleop.neutral_seen)
        self.assertEqual(self.plant.owner.teleop.control_mode,'upstream')

    def test_all_position_joints_and_wheels_through_original_controller_and_owner(self):
        self.start();before={n:r['Present_Position'] for n,r in self.plant.bus.r.items()};self.rails(True)
        for side in ('Left','Right'):
            for axis in ('x','y'):self.c['pads'][side+' Thumbstick'][axis]['raw']=.5
            self.c['buttons'][side+' Shoulder']['pressed']=True
            self.c['buttons'][side+' Trigger']['pressed']=True
            self.c['independent_motion'][side.lower()]['windows_attitude'].update(roll=.3,pitch=.3)
        for axis in ('x','y'):self.c['pads']['Direction Pad'][axis]['filtered']=1.
        self.c['buttons']['Button Menu']['pressed']=True;self.c['buttons']['Button Y']['pressed']=True
        self.tick(20)
        moved={n for n in POSITION_NAMES if abs(self.plant.bus.r[n]['Present_Position']-before[n])>2}
        self.assertEqual(moved,set(POSITION_NAMES))
        self.assertGreater(self.plant.bus.pose['x'],0.)
        command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertLessEqual(max(map(abs,command['rates'].values())),80)
        self.assertLessEqual(abs(command['linear'])+abs(command['angular'])*.125,.020000001)
        self.rails(False);self.tick(10)
        self.assertTrue(all(self.plant.bus.r[n]['Goal_Velocity']==0 for n in self.plant.bus.wheels))
        self.assertFalse(any(self.mapping.command(self.bridge.decoded,'wholebody')['rates'].values()))

    def test_no_holding_rails_no_motion_and_no_queued_tilt_on_rehold(self):
        self.start();before={n:r['Present_Position'] for n,r in self.plant.bus.r.items()}
        self.c['buttons']['Right Shoulder']['pressed']=True
        self.c['independent_motion']['right']['windows_attitude']['roll']=1.
        self.tick(8)
        self.assertEqual(before,{n:r['Present_Position'] for n,r in self.plant.bus.r.items()})
        self.c['buttons']['Right Shoulder']['pressed']=False;self.rails(True);self.feed()
        command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertLess(abs(command['rates']['right_arm_wrist_roll']),.01)

    def test_turns_use_reported_owner_track_and_pass_actual_wheel_guard(self):
        from wheel_pulse_executor import WheelPulseExecutor,WHEELBASE_M
        self.start();self.rails(True);self.c['buttons']['Button Menu']['pressed']=True
        self.c['buttons']['Button X']['pressed']=True;self.tick(3)
        self.assertEqual(self.robot.state['teleop']['wheelbase_m'],WHEELBASE_M)
        command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertNotEqual(command['angular'],0)
        WheelPulseExecutor.check_request(dict(linear_m_s=command['linear'],angular_rad_s=command['angular'],duration_s=.2))
        self.assertLessEqual(abs(command['angular'])*WHEELBASE_M/2,.020000001)
        # The adapter uses reported geometry even when it differs from its local copy.
        self.robot.state['teleop']['wheelbase_m']=.6
        command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertLessEqual(abs(command['angular'])*.3,.020000001)

    def test_plus_without_both_rails_neither_drives_nor_uses_x_hand_shortcut(self):
        self.start();self.c['buttons']['Right Rail SL']['pressed']=True
        self.c['buttons']['Button Menu']['pressed']=True;self.c['buttons']['Button X']['pressed']=True
        self.feed();command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertEqual((command['linear'],command['angular']),(0.,0.))
        self.assertLess(max(map(abs,command['rates'].values())),.01)

    def test_demo_profile_through_bridge_api_owner_and_restoration(self):
        self.bridge.robot_state=self.robot.state
        self.bridge.action({'op':'speed_profile','profile':'demo'})
        self.start()
        self.assertEqual(self.plant.bus.r['right_arm_wrist_roll']['Goal_Velocity'],300)
        self.assertEqual(self.plant.bus.r['right_arm_gripper']['Goal_Velocity'],200)
        self.assertEqual(self.plant.bus.r['head_motor_1']['Goal_Velocity'],100)
        with self.assertRaisesRegex(ValueError,'Stop before'):
            self.bridge.action({'op':'speed_profile','profile':'normal'})
        self.robot.dead={'left':True,'right':True}
        q=self.robot.state['motors']['right_arm_wrist_roll']['Present_Position']
        command=self.robot.encode_upstream_command({'positions':{'right_arm_wrist_roll':self.ref.from_ticks('right_arm_wrist_roll',q+40)},'linear':0,'angular':0},{})
        self.assertEqual(command['rates']['right_arm_wrist_roll'],300)
        self.bridge.release('Test stop')
        self.assertTrue(all(r['Torque_Enable']==0 for r in self.plant.bus.r.values()))
        self.assertEqual(self.plant.bus.r['right_arm_wrist_roll']['Goal_Velocity'],0)
        self.bridge.action({'op':'speed_profile','profile':'normal'})
        self.start()
        self.assertEqual(self.plant.bus.r['right_arm_wrist_roll']['Goal_Velocity'],100)

    def test_stale_feedback_and_input_disconnect_stop_real_owner_path(self):
        self.start();self.rails(True);self.c['buttons']['Right Shoulder']['pressed']=True;self.tick(3)
        self.robot.received-=1;self.feed();self.bridge.step()
        self.assertFalse(self.bridge.armed);self.assertFalse(self.plant.owner.teleop.active)
        self.assertTrue(all(r['Torque_Enable']==0 for r in self.plant.bus.r.values()))

if __name__=='__main__':unittest.main()
