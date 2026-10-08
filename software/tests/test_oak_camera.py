"""Depth evidence must preserve missing pixels and metric units."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from farm.oak_camera import depth_stats, patch_stats, save_capture


class OakDepthTests(unittest.TestCase):
    def test_missing_depth_is_not_zero_distance(self):
        stats = depth_stats(np.zeros((4, 4), dtype=np.uint16))
        self.assertIsNone(stats["median_mm"])
        self.assertEqual(stats["valid_fraction"], 0)

    def test_invalid_pixels_excluded_and_edge_patch_clipped(self):
        depth = np.array([[0, 1000], [1200, 1400]], dtype=np.uint16)
        stats = patch_stats(depth, 0, 0)
        self.assertEqual(stats["median_mm"], 1200)
        self.assertEqual(stats["valid_fraction"], .75)
        with self.assertRaises(ValueError):
            patch_stats(depth, -1, 0)

    def test_capture_preserves_16_bit_metric_depth(self):
        depth = np.array([[0, 300], [1250, 12000]], dtype=np.uint16)
        with tempfile.TemporaryDirectory() as tmp:
            dest = save_capture(Path(tmp), np.zeros((2, 2, 3), dtype=np.uint8), depth, {})
            np.testing.assert_array_equal(cv2.imread(str(dest / "depth_mm.png"), cv2.IMREAD_UNCHANGED), depth)
            np.testing.assert_array_equal(np.load(dest / "depth_mm.npy"), depth)
            self.assertFalse(json.loads((dest / "capture.json").read_text())["robot_frame_calibrated"])

    def test_misaligned_capture_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                save_capture(Path(tmp), np.zeros((2, 3, 3), dtype=np.uint8),
                             np.ones((2, 2), dtype=np.uint16), {})


if __name__ == "__main__":
    unittest.main()
