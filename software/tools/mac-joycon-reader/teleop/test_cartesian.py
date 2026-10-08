"""Measured hand-space behavior, full-body ownership and input-loss tests."""
import importlib.util
import math
import time
import unittest
from unittest.mock import patch
from test_mapping import frame
from bridge import Bridge

HAVE=importlib.util.find_spec('mujoco') is not None
if HAVE:
    import numpy as np
    from mujoco_simulator import MujocoRobot,DEFAULT_MODEL


@unittest.skipUnless(HAVE,'MuJoCo environment required')
class CartesianTests(unittest.TestCase):
    def setUp(self):
        if not DEFAULT_MODEL.exists():self.skipTest('Local model required')
        self.network=patch('socket.socket',side_effect=AssertionError('Robot network forbidden'));self.network.start()
        self.robot=MujocoRobot(render=False);self.mapping=self.robot.make_cartesian_mapping();self.mapping.feedback=self.robot.controller_feedback
        self.bridge=Bridge(self.robot,None,start=False,mapping=self.mapping)
        self.f=frame();self.c=self.f['controllers'][0]
        for name in ('Left Shoulder','Right Shoulder','Left Thumbstick Button','Right Thumbstick Button','Button A','Button B','Button X','Button Y','Button Home'):
            self.c['buttons'][name]={'pressed':False}
        self.c['pads']['Direction Pad']={a:{'filtered':0.} for a in ('x','y')}
        self.hold(True);self.feed();self.hold(False);self.feed()
        self.bridge.action({'op':'practice','scope':'wholebody'})

    def tearDown(self):self.bridge.close();self.network.stop()
    def hold(self,value):
        for s in ('Left','Right'):self.c['buttons'][s+' Shoulder']['pressed']=value
    def feed(self):
        self.f['timestamp']=time.time();self.f['sequence']+=1;self.bridge.ui_seen=time.monotonic();self.bridge.receive(self.f)
    def tick(self,n=20):
        for _ in range(n):self.feed();self.bridge.step();time.sleep(.03)
        self.assertTrue(self.bridge.armed,self.bridge.reason)
    def neutral(self):
        for b in self.c['buttons'].values():b['pressed']=False
        for p in self.c['pads'].values():
            for a in p.values():a['filtered']=0.
        self.tick(4)
    def pose(self):
        return {s:self.robot.bus.data.site_xpos[sid].copy() for s,sid in self.mapping.sites.items()}

    def test_xyz_both_directions_for_both_hands(self):
        report={}
        for axis in range(3):
            for sign in (1,-1):
                self.neutral();before=self.pose();self.hold(True)
                for s in ('Left','Right'):
                    self.c['pads'][s+' Thumbstick']['x' if axis==1 else 'y']['filtered']=-sign if axis==1 else sign
                    self.c['buttons'][s+' Thumbstick Button']['pressed']=axis==2
                self.tick(25);after=self.pose()
                for side in ('left','right'):
                    delta=after[side]-before[side];report[f'{side}_{axis}_{sign}']=delta.tolist()
                    self.assertGreater(sign*delta[axis],.001,report)
        print('Measured Cartesian hand deltas (m):',report)

    def test_simultaneous_grippers_head_wheels_and_release(self):
        before=self.robot.call('status');self.hold(True)
        for s in ('Left','Right'):self.c['buttons'][s+' Trigger']['pressed']=True
        self.c['pads']['Direction Pad']['x']['filtered']=1
        self.c['pads']['Direction Pad']['y']['filtered']=1
        self.c['buttons']['Button Y']['pressed']=True
        self.c['pads']['Left Thumbstick']['x']['filtered']=1
        self.c['pads']['Right Thumbstick']['y']['filtered']=1
        self.tick(30);after=self.robot.call('status')
        for n in ('left_arm_gripper','right_arm_gripper','head_motor_1','head_motor_2'):
            self.assertGreater(abs(after['motors'][n]['Present_Position']-before['motors'][n]['Present_Position']),10,n)
        self.assertGreater(after['virtual_pose']['x']-before['virtual_pose']['x'],.003)
        self.hold(False);self.feed()
        command=self.mapping.command(self.bridge.decoded,'wholebody')
        self.assertFalse(any(command['rates'].values()));self.assertEqual(command['linear'],0.)
        self.bridge.step();time.sleep(.65)
        state=self.robot.call('status');self.assertFalse(state['teleop']['active'])
        self.assertTrue(all(not r['Torque_Enable'] for r in state['motors'].values()))

    def test_wrist_modifier_and_gyro_loss(self):
        before=self.robot.call('status');self.hold(True);self.c['buttons']['Button Menu']['pressed']=True
        for s in ('Left','Right'):self.c['pads'][s+' Thumbstick']['x']['filtered']=1
        self.tick(30);after=self.robot.call('status')
        for s in ('left','right'):self.assertGreater(abs(after['motors'][s+'_arm_wrist_roll']['Present_Position']-before['motors'][s+'_arm_wrist_roll']['Present_Position']),10)
        self.neutral();self.bridge.release('Change gyro mode')
        self.c['independent_motion']={s:dict(ready=True,age_s=.01,quaternion=[1.,0.,0.,0.],session=s) for s in ('left','right')}
        self.feed();self.bridge.action({'op':'gyro','enabled':True});self.bridge.action({'op':'practice','scope':'wholebody'})
        self.hold(True);self.tick(2)
        self.c['independent_motion']['right']['age_s']=.3
        self.feed();self.bridge.step();self.assertFalse(self.bridge.armed)
        self.assertIn('gyro',self.bridge.reason.lower())

    def test_wholebody_unavailable_to_hardware_owner_by_default(self):
        self.bridge.release('Check boundary');self.robot.owner.simulation_wholebody=False
        with self.assertRaisesRegex(ValueError,'simulation-only'):self.bridge.action({'op':'practice','scope':'wholebody'})
        self.assertTrue(all(not r['Torque_Enable'] for r in self.robot.call('status')['motors'].values()))

    def test_independent_gyro_moves_only_selected_wrist_and_home_returns(self):
        self.bridge.release('Enable tilt')
        self.c['independent_motion']={s:dict(ready=True,age_s=.01,quaternion=[1.,0.,0.,0.],session=s) for s in ('left','right')}
        self.feed();self.bridge.action({'op':'gyro','enabled':True});self.bridge.action({'op':'practice','scope':'wholebody'})
        start=self.robot.call('status')['motors'];self.hold(True);self.tick(2)
        self.c['independent_motion']['left']['quaternion']=[math.cos(.2),math.sin(.2),0.,0.]
        self.tick(25);after=self.robot.call('status')['motors']
        self.assertGreater(abs(after['left_arm_wrist_roll']['Present_Position']-start['left_arm_wrist_roll']['Present_Position']),10)
        self.assertLess(abs(after['right_arm_wrist_roll']['Present_Position']-start['right_arm_wrist_roll']['Present_Position']),5)
        self.c['buttons']['Button Home']['pressed']=True;self.tick(1)
        self.c['buttons']['Button Home']['pressed']=False;self.tick(10)
        returning=self.robot.call('status')['motors']
        self.assertLess(abs(returning['left_arm_wrist_roll']['Present_Position']-start['left_arm_wrist_roll']['Present_Position']),abs(after['left_arm_wrist_roll']['Present_Position']-start['left_arm_wrist_roll']['Present_Position']))
        self.hold(False);self.feed();self.bridge.step();self.assertFalse(self.mapping.returning)

    def test_new_gripper_press_reverses_direction_and_held_trigger_blocks_claim(self):
        self.hold(True);self.c['buttons']['Left Trigger']['pressed']=True;self.tick(12)
        first=self.robot.call('status')['motors']['left_arm_gripper']['Present_Position']
        self.c['buttons']['Left Trigger']['pressed']=False;self.tick(2)
        self.c['buttons']['Left Trigger']['pressed']=True;self.tick(12)
        second=self.robot.call('status')['motors']['left_arm_gripper']['Present_Position']
        self.assertGreater(second,first+10)
        self.bridge.release('Check neutral');self.hold(False);self.feed()
        with self.assertRaisesRegex(ValueError,'center sticks'):self.bridge.action({'op':'practice','scope':'wholebody'})

    def test_hand_relative_translation(self):
        self.bridge.release('Select hand frame');self.feed()
        self.bridge.action({'op':'reference_frame','frame':'hand'})
        self.bridge.action({'op':'practice','scope':'wholebody'})
        directions={s:-self.robot.bus.data.site_xmat[i].reshape(3,3)[:,1].copy() for s,i in self.mapping.sites.items()}
        before=self.pose();self.hold(True)
        for s in ('Left','Right'):self.c['pads'][s+' Thumbstick']['y']['filtered']=1
        self.tick(25);after=self.pose()
        for s in directions:self.assertGreater(float((after[s]-before[s])@directions[s]),.001)

if __name__=='__main__':unittest.main()
