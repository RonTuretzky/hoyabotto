"""Optional MuJoCo integration checks; no motors, serial ports or robot network."""
import importlib.util
import json
import time
import unittest
from unittest.mock import patch
from pathlib import Path
from bridge import Bridge
from test_mapping import frame

HAVE_MUJOCO=importlib.util.find_spec('mujoco') is not None
if HAVE_MUJOCO:
    from mujoco_simulator import MujocoRobot, DEFAULT_MODEL, POSITION_MAP

@unittest.skipUnless(HAVE_MUJOCO,'MuJoCo environment required')
class PhysicsTests(unittest.TestCase):
    def test_controller_to_physics_all_components_and_timeout(self):
        if not DEFAULT_MODEL.exists():self.skipTest('Local XLeRobot model missing')
        with patch('socket.socket',side_effect=AssertionError('Robot network forbidden')):
            robot=MujocoRobot(render=False);bridge=Bridge(robot,None,start=False)
            f=frame();report={}
            def feed():
                f['timestamp']=time.time();f['sequence']+=1;bridge.ui_seen=time.monotonic();bridge.receive(f)
            def neutral():
                for b in ('Left Trigger','Right Trigger'):f['controllers'][0]['buttons'][b]['pressed']=False
                for p in f['controllers'][0]['pads'].values():
                    for a in p.values():a['filtered']=0.
                feed()
            def held(value=1):
                for b in ('Left Trigger','Right Trigger'):f['controllers'][0]['buttons'][b]['pressed']=True
                for p in f['controllers'][0]['pads'].values():
                    for a in p.values():a['filtered']=value
            def tick(n):
                for _ in range(n):feed();bridge.step();time.sleep(.04)
                self.assertTrue(bridge.armed,bridge.reason)
            try:
                held();feed();neutral()
                changed=set()
                for scope in ('both','head'):
                    bridge.action({'op':'practice','scope':scope})
                    for layer in range(3 if scope=='both' else 1):
                        neutral();bridge.action({'op':'layer','layer':layer})
                        before=robot.call('status')['motors']
                        held();tick(12)
                        after=robot.call('status')['motors']
                        delta={n:after[n]['Present_Position']-before[n]['Present_Position'] for n in POSITION_MAP}
                        names={n for n,d in delta.items() if d>10};changed|=names
                        report[f'{scope}_layer_{layer}']={n:delta[n] for n in names}
                        held(-1);tick(12)
                        reverse=robot.call('status')['motors']
                        self.assertTrue(all(reverse[n]['Present_Position']<after[n]['Present_Position']-10 for n in names))
                    neutral();bridge.release('Test stop')
                self.assertEqual(changed,set(POSITION_MAP),report)
                bridge.action({'op':'practice','scope':'drive'})
                before=robot.call('status')['virtual_pose'];held(0)
                f['controllers'][0]['pads']['Left Thumbstick']['y']['filtered']=1
                tick(20);after=robot.call('status')['virtual_pose']
                self.assertGreater(after['x']-before['x'],.005)
                report['drive_delta_m']=after['x']-before['x']
                neutral();tick(15)
                held(0);f['controllers'][0]['pads']['Right Thumbstick']['x']['filtered']=1
                before=robot.call('status')['virtual_pose'];tick(20);after=robot.call('status')['virtual_pose']
                self.assertLess(after['heading']-before['heading'],-.02)
                report['turn_delta_rad']=after['heading']-before['heading']
                time.sleep(.6)
                s=robot.call('status');self.assertFalse(s['teleop']['active'])
                self.assertTrue(all(v['Torque_Enable']==0 for v in s['motors'].values()))
                report['timeout_release']=True
                print('MuJoCo measured results:',json.dumps(report,sort_keys=True))
            finally:bridge.close()

    def test_renderer_returns_actual_scene(self):
        if not DEFAULT_MODEL.exists():self.skipTest('Local XLeRobot model missing')
        robot=MujocoRobot(render=True)
        try:
            deadline=time.monotonic()+5
            while robot.frame() is None and time.monotonic()<deadline:time.sleep(.05)
            img=robot.frame();self.assertIsNotNone(img,robot.bus.render_error)
            self.assertTrue(img.startswith(b'\xff\xd8'));self.assertGreater(len(img),10000)
            sequence=robot.bus.frame_sequence;robot.set_view('top')
            deadline=time.monotonic()+2
            while robot.bus.frame_sequence<=sequence and time.monotonic()<deadline:time.sleep(.05)
            self.assertNotEqual(robot.frame(),img)
            self.assertIsNone(robot.bus.render_error)
        finally:robot.close()

if __name__=='__main__':unittest.main()
