"""Real render from read-only feedback, with commands/network forbidden."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from PIL import Image
import io
from readback_preview import ReadbackPreview
from joycon_commissioning import native_binding, MOTOR_NAMES
from mujoco_simulator import MujocoBus


class PreviewTests(unittest.TestCase):
    def test_encoder_preview_renders_without_commands_or_physics_steps(self):
        calibration={n:dict(range_min=100,range_max=4000,homing_offset=0,drive_mode=0) for n in MOTOR_NAMES}
        with patch('socket.socket',side_effect=AssertionError('No network')),patch.object(MujocoBus,'step',side_effect=AssertionError('No physics stepping')),patch.object(MujocoBus,'write',side_effect=AssertionError('No register writes')):
            p=ReadbackPreview(native_binding(calibration))
            try:
                p.update(dict(status_age_s=0,motors={n:dict(Present_Position=2048) for n in MOTOR_NAMES}))
                deadline=time.monotonic()+5
                while (p.frame() is None or p.info()['render_error'] is not None) and time.monotonic()<deadline:time.sleep(.02)
                self.assertIsNotNone(p.frame(),p.info())
                self.assertEqual(Image.open(io.BytesIO(p.frame())).size,(960,640))
                self.assertTrue(p.info()['read_only']);self.assertFalse(p.info()['physics'])
                self.assertEqual(p.bus.model.njnt,16)
                self.assertGreaterEqual(p.bus.model.mesh('moving_jaw_so101_v1').id,0)
                self.assertGreaterEqual(p.bus.model.joint('left_shoulder_pan').id,0)
                self.assertIn('0.4 kit layout',p.info()['model_hardware'])
                self.assertIsNone(p.info()['render_error'])
                p.set_view('top')
                p.update(dict(status_age_s=20,motors={}))
                time.sleep(.15)
                self.assertIn('fresh encoder',p.info()['render_error'])
            finally:p.close()


if __name__=='__main__':unittest.main()
