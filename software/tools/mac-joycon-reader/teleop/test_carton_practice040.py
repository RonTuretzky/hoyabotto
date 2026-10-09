"""Exercise capture/slip/springback and shared-reader isolation from live transport."""
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from bridge import Bridge
from carton_practice040 import CartonCourse,CartonPractice040,FoldingPractice
from carton_practice_scene import BOX,CENTER
from test_practice040 import LiveForbidden
from test_upstream import original_frame
from upstream import WindowsMapping
from types import SimpleNamespace


class FoldingTests(unittest.TestCase):
    def inputs(self,f,angle=None):
        return {'left':np.array([2.,2.,2.]),'right':f.point('right',angle=angle)}
    def capture(self,f):
        f.update(.05,self.inputs(f),{'left':90.,'right':90.},{'left':0.,'right':0.})
        f.update(.05,self.inputs(f),{'left':90.,'right':0.},{'left':0.,'right':0.})
        self.assertEqual(f.flaps['right']['held_by'],'right')
    def fold(self,f,angle=110):
        for a in np.linspace(-11,angle,80):
            f.update(.05,self.inputs(f,a),{'left':90.,'right':0.},{'left':0.,'right':0.})
    def test_empty_close_does_not_capture_or_fold(self):
        f=FoldingPractice();far={'left':np.ones(3)*2,'right':np.ones(3)*2}
        f.update(.05,far,{'left':90.,'right':0.},{'left':0.,'right':0.})
        self.assertIsNone(f.flaps['right']['held_by']);self.assertEqual(f.flaps['right']['angle'],-11.)
        self.assertIn('air',f.note)
    def test_brief_flat_fold_springs_back_on_release(self):
        f=FoldingPractice();self.capture(f);self.fold(f,90)
        f.update(.05,self.inputs(f,90),{'left':90.,'right':90.},{'left':0.,'right':0.})
        for _ in range(60):f.update(.05,self.inputs(f),{'left':90.,'right':90.},{'left':0.,'right':0.})
        self.assertLess(f.flaps['right']['angle'],30);self.assertFalse(f.verified)
    def test_hinge_arc_and_continuous_overfold_hold_then_release_passes(self):
        f=FoldingPractice();self.capture(f);self.fold(f)
        for _ in range(220):f.update(.05,self.inputs(f,110),{'left':90.,'right':0.},{'left':0.,'right':0.})
        self.assertGreaterEqual(f.flaps['right']['hold_s'],10)
        f.update(.05,self.inputs(f,110),{'left':90.,'right':90.},{'left':0.,'right':0.})
        self.assertFalse(f.verified)  # Release is not a success verdict yet.
        for _ in range(45):f.update(.05,self.inputs(f),{'left':90.,'right':90.},{'left':0.,'right':0.})
        self.assertTrue(f.verified);self.assertGreater(f.flaps['right']['angle'],75)
    def test_pulling_off_arc_slips_and_cannot_set_crease(self):
        f=FoldingPractice();self.capture(f)
        tips=self.inputs(f);tips['right']+=np.array([0.,0.,.08])
        f.update(.05,tips,{'left':90.,'right':0.},{'left':0.,'right':0.})
        self.assertIsNone(f.flaps['right']['held_by']);self.assertIn('slipped',f.note)
    def test_major_requires_quarter_turned_pads(self):
        f=FoldingPractice('near');tips={'left':np.ones(3)*2,'right':f.point('near')}
        for r in (0.,90.):
            f.update(.05,tips,{'left':90.,'right':90.},{'left':0.,'right':r})
            f.update(.05,tips,{'left':90.,'right':0.},{'left':0.,'right':r})
            self.assertEqual(f.flaps['near']['held_by'],'right' if r else None)


class CartonIntegrationTests(unittest.TestCase):
    def test_course_requires_continuous_hold_release_and_delayed_verdict(self):
        state=dict(target='right',label='R12',hand='right',distance_cm=10.,angle_deg=-11.,hold_s=0.,held_by=None,verified=False,note='')
        folding=SimpleNamespace(flaps={'right':{}},released_at=None,elapsed=0.)
        c=CartonCourse(SimpleNamespace(fold_status=lambda:state,folding=folding))
        positions={s+'_arm_'+j:0. for s in ('left','right') for j in ('wrist_roll','wrist_flex')}
        positions.update(right_arm_gripper=90.)
        c.begin(positions,0.)
        c.update(positions|{'left_arm_wrist_roll':15,'right_arm_wrist_roll':15});c.update(positions)
        self.assertEqual(c.step,1)
        state['distance_cm']=1.;c.update(positions);self.assertEqual(c.step,2)
        state['held_by']='right';c.update(positions);self.assertEqual(c.step,3)
        state['angle_deg']=110.;c.update(positions);self.assertEqual(c.step,4)
        state['hold_s']=9.9;c.update(positions);self.assertEqual(c.step,4)
        state['hold_s']=10.;c.update(positions);self.assertEqual(c.step,5)
        state['held_by']=None;c.update(positions);self.assertEqual(c.step,6);self.assertFalse(c.complete)
        state['verified']=True;c.update(positions);self.assertTrue(c.complete)

    def test_exact_station_dimensions_and_default_rear_view(self):
        p=CartonPractice040(render=False)
        try:
            with p.lock:
                m=p.bus.model
                np.testing.assert_allclose(m.geom('practice_table_top').size[:2],[.24,.25])
                self.assertAlmostEqual(m.body('practice_table').pos[2]+m.geom('practice_table_top').size[2],.7)
                self.assertEqual(m.njnt,20);self.assertEqual(p.bus.view,'behind')
                np.testing.assert_allclose(m.body('fold_carton').pos,CENTER)
                np.testing.assert_allclose(m.geom('fold_panel_right').size[[0,2]],[BOX.width/2,.07])
                np.testing.assert_allclose(m.joint('fold_hinge_right').range,np.radians([-60,125]))
                self.assertEqual(p.status()['simulator']['station']['carton_mm'],[379,283,108])
                p.lesson_action('target:left');self.assertEqual(p.fold_status()['target'],'left')
                p.lesson_action('reset_box');self.assertFalse(p.held);self.assertEqual(p.course.step,1)
        finally:p.close()
    def test_stop_releases_virtual_pinches(self):
        p=CartonPractice040(render=False)
        try:
            with p.lock:
                p.folding.flaps['right']['held_by']='right';p.stop('test')
                self.assertIsNone(p.folding.flaps['right']['held_by'])
        finally:p.close()
    def test_shared_input_moves_carton_scene_without_robot_transport(self):
        with patch('socket.socket',side_effect=AssertionError('No robot connection in practice')):
            live=LiveForbidden()
            b=Bridge(live,None,start=False,mapping=WindowsMapping(live),input_backend='hid',practice_factory=lambda:CartonPractice040(render=False))
            feeder=None;done=threading.Event()
            try:
                b.action({'op':'control_target','target':'practice'});frame=original_frame()
                def feed():
                    frame['sequence']+=1;frame['timestamp']=time.time();b.receive(frame)
                def stream():
                    while not done.is_set():feed();done.wait(.01)
                feeder=threading.Thread(target=stream,daemon=True);feeder.start();feed()
                b.action({'op':'practice','scope':'wholebody'})
                before=b.robot.positions['right_arm_wrist_roll']
                frame['controllers'][0]['independent_motion']['right']['windows_attitude']['roll']+=.4
                for _ in range(8):feed();b.step();time.sleep(.02)
                self.assertGreater(abs(b.robot.positions['right_arm_wrist_roll']-before),5)
                self.assertEqual(b.snapshot()['robot']['folding']['target'],'right')
                with self.assertRaisesRegex(ValueError,'Stop controls'):b.action({'op':'control_target','target':'robot'})
                b.release('test stopped');b.action({'op':'control_target','target':'robot'})
                with self.assertRaisesRegex(ValueError,'practice-only'):b.action({'op':'lesson','action':'repeat'})
                self.assertFalse(b.armed)
            finally:
                done.set()
                if feeder:feeder.join(timeout=1)
                b.close()


if __name__=='__main__':unittest.main()
