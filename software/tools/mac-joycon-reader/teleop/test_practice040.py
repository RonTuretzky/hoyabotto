"""Real gyro frames move only the local SO-101 model; live transport is forbidden."""
import copy
import time
import threading
import unittest
from unittest.mock import patch
from bridge import Bridge
from practice040 import Practice040
from upstream import WindowsMapping
from test_upstream import original_frame


class LiveForbidden:
    preview=False
    simulation=False
    supports_upstream=True
    def call(self,*args,**kwargs):raise AssertionError('No physical transport in practice')


class Practice040Tests(unittest.TestCase):
    def test_shared_input_gyro_moves_virtual_wrist_without_live_calls(self):
        with patch('socket.socket',side_effect=AssertionError('No network')):
            live=LiveForbidden();b=Bridge(live,None,start=False,mapping=WindowsMapping(live),practice_factory=Practice040,input_backend='hid')
            try:
                b.action({'op':'control_target','target':'practice'})
                frame=original_frame()
                def feed():
                    frame['sequence']+=1;frame['timestamp']=time.time();b.ui_seen=time.monotonic();b.receive(frame)
                stop_feed=threading.Event()
                def stream():
                    while not stop_feed.is_set():feed();stop_feed.wait(.01)
                feeder=threading.Thread(target=stream,daemon=True);feeder.start()
                feed();b.action({'op':'practice','scope':'wholebody'})
                before=b.robot.positions['left_arm_wrist_roll']
                frame['controllers'][0]['independent_motion']['left']['windows_attitude']['roll']+=.4
                for _ in range(8):feed();b.step();time.sleep(.02)
                self.assertGreater(abs(b.robot.positions['left_arm_wrist_roll']-before),5)
                self.assertTrue(b.snapshot()['simulation'])
                self.assertEqual(b.robot.bus.model.njnt,16)
                with self.assertRaisesRegex(ValueError,'Stop controls'):b.action({'op':'control_target','target':'robot'})
                stop_feed.set();feeder.join(timeout=1)
                b.release('test stopped practice')
                b.action({'op':'control_target','target':'robot'})
                self.assertIs(b.robot,live)
                self.assertFalse(b.armed)
                b.ui_seen=0;feed();b.ui_seen=0
                with self.assertRaisesRegex(ValueError,'lost focus'):b.valid()
                self.assertIsNone(b.proc)
            finally:
                if 'stop_feed' in locals():stop_feed.set();feeder.join(timeout=1)
                b.close()


if __name__=='__main__':unittest.main()
