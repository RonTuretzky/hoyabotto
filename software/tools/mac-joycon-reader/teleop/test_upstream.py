"""Offline source parity and full adapter -> MuJoCo behavior. No devices/network."""
import ast
import hashlib
import json
import importlib.util
import math
import struct
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from bridge import Bridge
from hid_reader import Sensor
from test_hid_reader import imu_cal
from test_mapping import frame
HAVE=importlib.util.find_spec('mujoco') is not None
if HAVE:
    import numpy as np
    from upstream import to_model,from_model,JOINTS
    from upstream_simulator import UpstreamSimulator
    from vendor.windows_attitude import JoyConHIDAPIReader

VENDOR=Path(__file__).with_name('vendor')


def original_frame():
    f=frame();f['backend']='Nintendo HID';c=f['controllers'][0]
    for n in ('Left Shoulder','Right Shoulder','Left Thumbstick Button','Right Thumbstick Button','Button A','Button B','Button X','Button Y','Button Home','Button Capture'):
        c['buttons'][n]={'pressed':False}
    for p in c['pads'].values():
        for a in p.values():a['raw']=0.
    c['pads']['Direction Pad']={a:{'filtered':0.,'raw':0.} for a in ('x','y')}
    c['independent_motion']={s:dict(ready=True,age_s=.01,session=s,windows_attitude=dict(roll=0.,pitch=0.,yaw=0.)) for s in ('left','right')}
    return f


@unittest.skipUnless(HAVE,'MuJoCo environment required')
class SourceTests(unittest.TestCase):
    def test_extracted_definitions_match_pinned_originals(self):
        for row in json.loads((VENDOR/'manifest.json').read_text()):
            raw=(VENDOR/row['source']).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(),row['sha256'])
            original=ast.parse(raw.decode());vendored=ast.parse((VENDOR/row['output']).read_text())
            old={n.name:n for n in original.body if isinstance(n,(ast.ClassDef,ast.FunctionDef))}
            new={n.name:n for n in vendored.body if isinstance(n,(ast.ClassDef,ast.FunctionDef))}
            for name,methods in row['classes'].items():
                if methods is None:self.assertEqual(ast.dump(old[name]),ast.dump(new[name]))
                else:
                    a={n.name:n for n in old[name].body if isinstance(n,ast.FunctionDef)}
                    b={n.name:n for n in new[name].body if isinstance(n,ast.FunctionDef)}
                    self.assertEqual(set(b),set(methods))
                    for m in methods:self.assertEqual(ast.dump(a[m]),ast.dump(b[m]),name+'.'+m)
            for name in row['functions']:self.assertEqual(ast.dump(old[name]),ast.dump(new[name]))

    def test_gravity_filter_and_mirrored_left_sensor_match_original(self):
        sensors={s:Sensor(imu_cal(),s) for s in ('left','right')}
        expected=JoyConHIDAPIReader()
        for s,sensor in sensors.items():
            sign=-1 if s=='left' else 1
            for _ in range(200):sensor.update(struct.pack('<6h',0,0,-4096*sign,0,0,0))
            for _ in range(200):sensor.update(struct.pack('<6h',0,2048*sign,-3547*sign,0,0,0))
        expected.accel[:]=[0,.5,-3547/4096]
        for _ in range(100):expected._update_attitude()
        for sensor in sensors.values():
            self.assertAlmostEqual(sensor.attitude.roll,expected.roll)
            self.assertGreater(math.degrees(sensor.attitude.roll),20)
        self.assertAlmostEqual(sensors['left'].attitude.roll,sensors['right'].attitude.roll)

    def test_coordinate_roundtrip(self):
        for s in ('left','right'):
            for j in JOINTS:
                for q in (-30,0,30):self.assertAlmostEqual(from_model(s+'_arm_'+j,to_model(s+'_arm_'+j,q)),q)


@unittest.skipUnless(HAVE,'MuJoCo environment required')
class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.network=patch('socket.socket',side_effect=AssertionError('Network forbidden'));self.network.start()
        self.robot=UpstreamSimulator(render=False,start=False);self.mapping=self.robot.make_mapping()
        self.bridge=Bridge(self.robot,None,start=False,mapping=self.mapping,input_backend='hid')
        self.f=original_frame();self.c=self.f['controllers'][0];self.feed()
        self.bridge.action({'op':'practice','scope':'wholebody'})
    def tearDown(self):self.bridge.close();self.network.stop()
    def feed(self):
        self.f['timestamp']=time.time();self.f['sequence']+=1;self.bridge.ui_seen=time.monotonic();self.bridge.receive(self.f)
    def tick(self,n=1):
        for _ in range(n):
            self.feed();self.bridge.step();self.robot.advance(.02)
            self.assertTrue(self.bridge.armed,self.bridge.reason)
    def q(self,name):return float(self.robot.bus.data.qpos[self.robot.bus.map[name][1]])

    def test_start_holds_position_without_homing(self):
        before=self.robot.get_observation();self.tick(30);after=self.robot.get_observation()
        self.assertLess(max(abs(after[k]-before[k]) for k in before),1.)

    def test_actual_wrist_tracks_30_degree_request_in_one_second(self):
        self.tick(5);before={n:self.q(n) for n in ('left_arm_wrist_roll','left_arm_wrist_flex','right_arm_wrist_roll')}
        a=self.c['independent_motion']['left']['windows_attitude'];a['roll']=30/45;a['pitch']=30/90
        self.tick(50)
        for j in ('wrist_roll','wrist_flex'):
            actual=math.degrees(self.q('left_arm_'+j)-before['left_arm_'+j])
            self.assertAlmostEqual(actual,30.,delta=2.,msg=j+': '+str(actual))
        self.assertLess(abs(math.degrees(self.q('right_arm_wrist_roll')-before['right_arm_wrist_roll'])),1.)

    def test_measured_forward_sideways_and_lift_for_both_hands(self):
        self.tick(10)
        for axis in range(3):
            for sign in (1,-1):
                before={side:self.robot.bus.data.site(side+'_grip_center').xpos.copy() for side in ('left','right')}
                for side in ('Left','Right'):
                    if axis==2:self.c['buttons'][side+(' Shoulder' if sign==1 else ' Thumbstick Button')]['pressed']=True
                    else:self.c['pads'][side+' Thumbstick']['y' if axis==0 else 'x']['raw']=sign
                self.tick(5)
                for side in ('Left','Right'):
                    for button in (' Shoulder',' Thumbstick Button'):self.c['buttons'][side+button]['pressed']=False
                    for a in ('x','y'):self.c['pads'][side+' Thumbstick'][a]['raw']=0.
                self.tick(20)
                for side in ('left','right'):
                    delta=self.robot.bus.data.site(side+'_grip_center').xpos-before[side]
                    # +X is toward the workbench; +stick X is robot-right (-Y).
                    self.assertGreater(delta[axis]*sign*(-1 if axis==1 else 1),.012,(axis,sign,side,delta))

    def test_original_lift_lower_gripper_and_capture_home(self):
        initial={s:list(c.position) for s,c in self.mapping.controllers.items()}
        for s in ('Left','Right'):self.c['buttons'][s+' Shoulder']['pressed']=True
        self.tick(3)
        for s,c in self.mapping.controllers.items():self.assertGreater(c.position[2],initial[s][2]+.007)
        for s in ('Left','Right'):
            self.c['buttons'][s+' Shoulder']['pressed']=False
            self.c['buttons'][s+' Thumbstick Button']['pressed']=True
        self.tick(3)
        for s,c in self.mapping.controllers.items():np.testing.assert_allclose(c.position,initial[s],atol=1e-6)
        for s in ('Left','Right'):
            self.c['buttons'][s+' Thumbstick Button']['pressed']=False
            self.c['buttons'][s+' Trigger']['pressed']=True
        self.tick(20)
        for s in ('left','right'):self.assertGreater(from_model(s+'_arm_gripper',self.q(s+'_arm_gripper')),85.)
        for s in ('Left','Right'):self.c['buttons'][s+' Trigger']['pressed']=False
        self.tick()
        for s in ('Left','Right'):self.c['buttons'][s+' Trigger']['pressed']=True
        self.tick(20)
        for s in ('left','right'):self.assertLess(from_model(s+'_arm_gripper',self.q(s+'_arm_gripper')),5.)
        for s in ('Left','Right'):self.c['buttons'][s+' Trigger']['pressed']=False
        self.c['pads']['Left Thumbstick']['y']['raw']=1.;self.c['pads']['Right Thumbstick']['x']['raw']=1.
        self.tick(5)
        self.c['pads']['Left Thumbstick']['y']['raw']=0.;self.c['pads']['Right Thumbstick']['x']['raw']=0.
        self.c['buttons']['Button Capture']['pressed']=True;self.c['buttons']['Button Home']['pressed']=True
        self.tick()
        for s,c in self.mapping.controllers.items():np.testing.assert_allclose(c.position,initial[s],atol=1e-6)

    def test_head_and_drive_modifier_keep_original_x_shortcut_separate(self):
        initial=self.mapping.controllers['right'].position[0]
        self.c['buttons']['Button X']['pressed']=True;self.tick()
        self.assertAlmostEqual(self.mapping.controllers['right'].position[0],initial+.003,places=5)
        self.assertTrue(all(self.robot.bus.r[n]['Goal_Velocity']==0 for n in self.robot.bus.wheel_map))
        initial=self.mapping.controllers['right'].position[0]
        self.c['buttons']['Button Menu']['pressed']=True;self.c['buttons']['Button Y']['pressed']=True
        for a in ('x','y'):self.c['pads']['Direction Pad'][a]['filtered']=1.
        self.tick(10)
        self.assertAlmostEqual(self.mapping.controllers['right'].position[0],initial,places=5)
        self.assertTrue(any(self.robot.bus.r[n]['Goal_Velocity'] for n in self.robot.bus.wheel_map))
        self.assertGreater(self.robot.bus.pose['x'],.001)
        obs=self.robot.get_observation();self.assertLess(obs['head_motor_1.pos'],-10);self.assertGreater(obs['head_motor_2.pos'],10)
        self.c['buttons']['Button Menu']['pressed']=False;self.c['buttons']['Button X']['pressed']=False;self.tick()
        self.assertTrue(all(self.robot.bus.r[n]['Goal_Velocity']==0 for n in self.robot.bus.wheel_map))

    def test_gyro_disconnect_focus_stop_and_watchdog(self):
        self.c['independent_motion']['right']['session']='reconnect';self.feed();self.bridge.step()
        self.assertFalse(self.bridge.armed);self.assertFalse(self.robot.call('status')['teleop']['active'])
        self.feed();self.bridge.action({'op':'practice','scope':'wholebody'})
        self.bridge.ui_seen=0;self.bridge.step();self.assertFalse(self.bridge.armed)
        self.feed();self.bridge.action({'op':'practice','scope':'wholebody'})
        self.c['buttons']['Button Menu']['pressed']=True;self.c['buttons']['Button Y']['pressed']=True;self.tick()
        self.robot.last_input-=.3;self.robot.advance(.02)
        self.assertFalse(self.robot.call('status')['teleop']['active'])
        self.assertTrue(all(self.robot.bus.r[n]['Goal_Velocity']==0 for n in self.robot.bus.wheel_map))
        self.feed();self.bridge.step();self.assertFalse(self.bridge.armed)

    def test_bad_positions_rejected_before_writes(self):
        session=dict(self.bridge.session);before={n:r['Goal_Position'] for n,r in self.robot.bus.r.items()}
        with self.assertRaisesRegex(ValueError,'Invalid simulated position'):
            self.robot.call('input',{k:session[k] for k in ('token','permit','owner_started')}|dict(sequence=2,positions={'left_arm_wrist_roll':float('nan')},linear=0.,angular=0.))
        self.assertFalse(self.robot.call('status')['teleop']['active'])
        self.assertTrue(all(math.isfinite(r['Goal_Position']) for r in self.robot.bus.r.values()))

if __name__=='__main__':unittest.main()
