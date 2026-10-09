"""Whole local control chain with virtual servos; no hardware or sockets."""
import threading
import time
import unittest
from unittest.mock import patch
from bridge import Bridge, PreviewRobot
from simulator import SimulatedRobot
from test_mapping import frame

class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.no_socket=patch('socket.socket',side_effect=AssertionError('Simulation must not open a socket'))
        self.no_socket.start()
        self.robot=SimulatedRobot()
        self.bridge=Bridge(self.robot,None,start=False)
        self.f=frame()
        for b in ('Left Trigger','Right Trigger'):self.f['controllers'][0]['buttons'][b]['pressed']=True
        self.feed()
        for b in ('Left Trigger','Right Trigger'):self.f['controllers'][0]['buttons'][b]['pressed']=False
        self.feed()
    def tearDown(self):
        self.bridge.close();self.no_socket.stop()
    def feed(self):
        self.f['timestamp']=time.time();self.f['sequence']+=1
        self.bridge.ui_seen=time.monotonic();self.bridge.receive(self.f)
    def tick(self,count=5):
        for _ in range(count):
            self.feed();self.bridge.step();time.sleep(.035)
    def start(self,scope):
        self.feed();self.bridge.action({'op':'practice','scope':scope})
        self.assertTrue(self.bridge.armed)
        self.assertTrue(self.robot.call('status')['teleop']['neutral_seen'])
    def neutral(self):
        for b in ('Left Trigger','Right Trigger'):self.f['controllers'][0]['buttons'][b]['pressed']=False
        for p in self.f['controllers'][0]['pads'].values():
            for a in p.values():a['filtered']=0
        self.feed()
    def sticks(self,value):
        for b in ('Left Trigger','Right Trigger'):self.f['controllers'][0]['buttons'][b]['pressed']=True
        for p in self.f['controllers'][0]['pads'].values():
            for a in p.values():a['filtered']=value
    def test_all_14_position_joints_through_mapping_api_and_owner(self):
        moved=set()
        for scope in ('both','head'):
            self.start(scope)
            for layer in range(3 if scope=='both' else 1):
                self.neutral();self.bridge.action({'op':'layer','layer':layer})
                before={n:v['Present_Position'] for n,v in self.robot.call('status')['motors'].items()}
                self.sticks(1);self.tick()
                after={n:v['Present_Position'] for n,v in self.robot.call('status')['motors'].items()}
                names={n for n in before if after[n]>before[n]};moved|=names
                self.sticks(-1);self.tick(7)
                reverse=self.robot.call('status')['motors']
                self.assertTrue(all(reverse[n]['Present_Position']<after[n] for n in names))
            self.neutral();self.bridge.release('Test stop')
        self.assertEqual(len(moved),14)
    def test_drive_then_drop_input_stops_both_wheels(self):
        self.start('drive');self.sticks(0)
        self.f['controllers'][0]['pads']['Left Thumbstick']['y']['filtered']=1
        self.tick(8)
        state=self.robot.call('status');self.assertGreater(state['virtual_pose']['x'],0)
        self.assertTrue(state['teleop']['active'])
        # Abandon the bridge without sending stop; the independent owner expires it.
        time.sleep(.55)
        state=self.robot.call('status');self.assertFalse(state['teleop']['active'])
        self.assertTrue(all(m['Torque_Enable']==0 for m in state['motors'].values()))
        self.feed();self.bridge.step();self.assertFalse(self.bridge.armed)
    def test_focus_loss_and_reader_disconnect_release(self):
        for reason in ('focus','disconnect'):
            self.neutral();self.start('left')
            if reason=='focus':self.bridge.ui_seen=0
            else:self.f['controllers'][0]['connected']=False;self.feed()
            self.bridge.step();self.assertFalse(self.bridge.armed)
            self.assertFalse(self.robot.call('status')['teleop']['active'])
            self.f['controllers'][0]['connected']=True
    def test_background_tab_does_not_cancel_focused_tab_but_lease_still_expires(self):
        self.start('left')
        self.bridge.heartbeat(True);seen=self.bridge.ui_seen
        self.bridge.heartbeat(False)
        self.assertEqual(self.bridge.ui_seen,seen)
        self.bridge.step();self.assertTrue(self.bridge.armed)
        self.bridge.ui_seen-=.81
        self.bridge.step();self.assertFalse(self.bridge.armed)
        self.assertFalse(self.robot.call('status')['teleop']['active'])
    def test_stop_during_claim_does_not_arm_late(self):
        started=threading.Event();proceed=threading.Event();orig=self.robot.call;errors=[]
        def delayed(path,*args,**kwargs):
            if path=='claim':started.set();proceed.wait(1)
            return orig(path,*args,**kwargs)
        self.robot.call=delayed
        def claim():
            try:self.bridge.action({'op':'practice','scope':'left'})
            except ValueError as e:errors.append(str(e))
        worker=threading.Thread(target=claim);worker.start();self.assertTrue(started.wait(1))
        self.bridge.action({'op':'stop'});proceed.set();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertFalse(self.bridge.armed)
        self.assertTrue(any('STOP cancelled' in e for e in errors))
        self.assertFalse(orig('status')['teleop']['active'])
    def test_competing_client_command_does_not_cancel_practice(self):
        self.start('left')
        ready=self.robot.client.readiness()
        self.assertFalse(ready['motion_ready'])
        self.assertTrue(any('MANUAL_CONTROL_ACTIVE' in x for x in ready['blockers']))
        result=self.robot.client.set_motor_enable(['right_arm_gripper'],True)
        self.assertFalse(result['accepted'])
        self.assertIn('MANUAL_CONTROL_ACTIVE',result['reason'])
        with self.assertRaisesRegex(ValueError,'MANUAL_CONTROL_ACTIVE'):
            self.robot.client.set_motor_enable(['left_arm_gripper'],False)
        self.assertTrue(self.robot.call('status')['teleop']['active'])
        self.assertTrue(self.bridge.armed)
    def test_preview_cannot_request_real_arm(self):
        with self.assertRaisesRegex(ValueError,'cannot activate'):
            self.bridge.action({'op':'arm','scope':'left'})
        self.assertTrue(all(m['Torque_Enable']==0 for m in self.robot.call('status')['motors'].values()))

if __name__=='__main__':unittest.main()
