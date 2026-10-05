import copy
import os
import time

import cv2
import numpy as np
import pytest

from carton.servo.common import Refused, atomic_json
from carton.servo.depth import DepthObserver, camera_point, fit_table, verify_depth_lift
from farm.oak_camera import StreamWriter


def spec(folder):
    names = ["tool", "paddle", "bottom", *[f"table{i}" for i in range(6)]]
    ref = folder / "ref.png"
    cv2.imwrite(str(ref), np.random.default_rng(15).integers(0, 255, (100, 100, 3), dtype=np.uint8))
    return {"manifest": str(folder / "oak.json"), "camera_id": "oak-test", "reference": str(ref),
            "regions": {n: {"roi": [10, 10, 24, 24], "point": [22, 22], "anchor": n.startswith("table")} for n in names},
            "table_features": names[3:], "tool": "tool", "paddle": "paddle", "bottom": "bottom",
            "up_hint_camera": [0, 0, -1], "min_lift_mm": 10, "min_clearance_mm": 10, "max_slip_mm": 5}


def stream(folder, keep=90):
    return StreamWriter(folder, {"device_id": "test", "alignment": "CAM_A RGB", "projection": "rectified_pinhole",
                                "coordinate_frame": "CAM_A_optical", "intrinsics": [[100, 0, 50], [0, 100, 50], [0, 0, 1]]}, keep)


def publish(writer, depth=None):
    if depth is None:
        depth = np.full((100, 100), 500, dtype=np.uint16)
    now = time.time()-.01
    return writer.publish(np.zeros((100, 100, 3), np.uint8), depth, now, now+.001)


def test_atomic_depth_stream_roundtrip_and_bounded_storage(tmp_path):
    s = spec(tmp_path)
    writer = stream(tmp_path, keep=3)
    for _ in range(10):
        m = publish(writer)
    observer = DepthObserver(s)
    record, rgb, depth = observer.read()
    assert record["seq"] == 10 and rgb.shape == (100, 100, 3)
    assert depth.dtype == np.uint16 and np.all(depth == 500)
    assert len(list(tmp_path.glob("*-depth.png"))) == 3
    assert record["robot_frame_calibrated"] is False
    assert record["host"] == os.uname().nodename


@pytest.mark.parametrize("mutation", [
    {"depth_units": "m"}, {"camera_id": "other"}, {"host": "other-Mac"}, {"seq": -1},
    {"captured_at": 0}, {"projection": "distorted"}, {"depth_sha256": "incorrect"},
    {"depth_image": "../outside.png"}, {"rgb_captured_at": time.time()+1000}, {"width": 200},
])
def test_wrong_units_stale_time_wrong_camera_and_corrupt_frames_refuse(tmp_path, mutation):
    s = spec(tmp_path)
    m = publish(stream(tmp_path))
    atomic_json(tmp_path / "oak.json", {**m, **mutation})
    with pytest.raises(Refused):
        DepthObserver(s).read()


def test_restart_requires_registration_and_intrinsics_are_bound(tmp_path):
    s = spec(tmp_path)
    writer = stream(tmp_path)
    publish(writer)
    observer = DepthObserver(s)
    observer.read()
    publish(stream(tmp_path))
    with pytest.raises(Refused, match="restarted"):
        observer.read()


def test_metric_camera_point_rejects_missing_and_mixed_background_depth():
    k = [[100, 0, 50], [0, 100, 50], [0, 0, 1]]
    depth = np.full((100, 100), 500, np.uint16)
    assert camera_point(depth, [60, 50], k) == pytest.approx([50, 0, 500])
    depth[48:51, 58:61] = 0
    with pytest.raises(Refused, match="valid pixels"):
        camera_point(depth, [60, 50], k)
    depth[48:51, 58:61] = 1000
    with pytest.raises(Refused, match="dispersion"):
        camera_point(depth, [60, 50], k)


def test_table_plane_and_lift_distinguish_sliding_from_lifting():
    points = [[x, y, 500] for x in (-100, 0, 100) for y in (-100, 100)]
    normal, center = fit_table(points, [0, 0, -1])
    assert normal == pytest.approx([0, 0, -1])
    s = {"tool": "tool", "paddle": "paddle", "min_lift_mm": 10, "min_clearance_mm": 10, "max_slip_mm": 5}
    before = {"stream": "x", "seq": 1, "normal": normal.tolist(), "points": {"tool": [0, 0, 490], "paddle": [20, 0, 490]}}
    after = {"stream": "x", "seq": 2, "points": {"tool": [0, 0, 470], "paddle": [20, 0, 470]}, "bottom_clearance_mm": 20}
    assert verify_depth_lift(before, after, s)["paddle_lift_mm"] == 20
    after["points"] = {"tool": [20, 0, 490], "paddle": [40, 0, 490]}
    with pytest.raises(Refused, match="did not verify"):
        verify_depth_lift(before, after, s)
    with pytest.raises(Refused, match="clustered"):
        fit_table(np.zeros((6, 3)), [0, 0, -1])


def test_publisher_rejects_old_captures_instead_of_timestamping_them_as_new(tmp_path):
    writer = stream(tmp_path)
    with pytest.raises(ValueError, match="stale"):
        writer.publish(np.zeros((100, 100, 3), np.uint8), np.full((100, 100), 500, np.uint16), 1, 1)
    assert not (tmp_path / "oak.json").exists()
