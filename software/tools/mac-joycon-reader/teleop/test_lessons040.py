"""Guided milestones and nearby-only box manipulation, local and network-free."""
import time
import unittest
from unittest.mock import patch
from lessons040 import BoxCourse
from practice040 import Practice040


class LessonTests(unittest.TestCase):
    def test_course_requires_observed_wrist_reach_grasp_lift_and_placement(self):
        positions={'left_arm_wrist_roll':0.,'right_arm_wrist_roll':0.,'right_arm_gripper':90.}
        c=BoxCourse();c.begin(positions,1.)
        c.update(positions,.2,True,1.2,True);self.assertEqual(c.step,0)
        c.update(positions|{'left_arm_wrist_roll':15},.2,False,1.,False)
        c.update(positions|{'right_arm_wrist_roll':15},.2,False,1.,False)
        c.update(positions,.2,False,1.,False);self.assertEqual(c.step,1)
        c.update(positions,.01,False,1.,False);self.assertEqual(c.step,2)
        c.update(positions,.01,True,1.,False);self.assertEqual(c.step,3)
        c.update(positions,.01,True,1.02,False);self.assertEqual(c.step,3)
        c.update(positions,.01,True,1.05,False);self.assertEqual(c.step,4)
        c.update(positions,.2,False,1.,False);self.assertFalse(c.complete)
        c.update(positions,.2,False,1.,True);self.assertTrue(c.complete)

    def test_box_cannot_attach_from_far_away_and_release_on_target_places_it(self):
        with patch('socket.socket',side_effect=AssertionError('No robot network')):
            p=Practice040()
            try:
                # Keep the controlled scenario deterministic while testing the same geometry methods.
                with p.lock:
                    p.positions['right_arm_gripper']=0;p.previous_grip=90
                    p._apply_positions(p.positions);p._update_box();self.assertFalse(p.held)
                    p.positions['right_arm_shoulder_pan']=-18;p.positions['right_arm_gripper']=90
                    p._apply_positions(p.positions);p._update_box()
                    self.assertLess(p._grip_distance(),.005)
                    p.positions['right_arm_gripper']=0;p.previous_grip=90
                    p._apply_positions(p.positions);p._update_box();self.assertTrue(p.held)
                    p.positions['right_arm_shoulder_pan']=12
                    p._apply_positions(p.positions);p._update_box()
                    p.positions['right_arm_gripper']=90
                    p._apply_positions(p.positions);p._update_box()
                    self.assertFalse(p.held);self.assertTrue(p.placed)
                    self.assertEqual(p.bus.view,'shoulder')
                    self.assertEqual(p.bus.model.njnt,16)
            finally:p.close()


if __name__=='__main__':unittest.main()
