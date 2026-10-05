"""Camera-only OAK-D Lite diagnostic. Never imports or connects robot controls.

DepthAI v2 pipeline using Luxonis' stereo-to-RGB alignment API.
Depth is aligned to RGB, uint16 millimetres; zero means missing, not contact.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

import numpy as np


def depth_stats(depth: np.ndarray) -> dict:
    valid = depth[depth > 0]
    return {
        "valid_fraction": float(valid.size / depth.size) if depth.size else 0.0,
        "median_mm": float(np.median(valid)) if valid.size else None,
        "p10_mm": float(np.percentile(valid, 10)) if valid.size else None,
        "p90_mm": float(np.percentile(valid, 90)) if valid.size else None,
    }


def patch_stats(depth: np.ndarray, x: int, y: int, radius: int = 5) -> dict:
    h, w = depth.shape
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError("Pixel is outside the depth image")
    return depth_stats(depth[max(0, y-radius):min(h, y+radius+1),
                             max(0, x-radius):min(w, x+radius+1)])


def save_capture(root: Path, rgb: np.ndarray, depth: np.ndarray, metadata: dict) -> Path:
    import cv2
    if rgb.shape[:2] != depth.shape or depth.dtype != np.uint16:
        raise ValueError("Expected matching RGB/depth dimensions and uint16 depth")
    dest = root / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    dest.mkdir(parents=True, exist_ok=False)
    for name, data in (("rgb.png", rgb), ("depth_mm.png", depth)):
        if not cv2.imwrite(str(dest / name), data):
            raise RuntimeError(f"Failed to write {name}")
    np.save(dest / "depth_mm.npy", depth)
    record = {**metadata, "captured_at_utc": datetime.now(timezone.utc).isoformat(),
              "depth_units": "mm", "invalid_depth": 0,
              "shape_hw": list(depth.shape), "whole_image": depth_stats(depth),
              "centre_patch": patch_stats(depth, depth.shape[1]//2, depth.shape[0]//2),
              "robot_frame_calibrated": False,
              "note": "Camera-frame axial depth; not robot coordinates or assembly accuracy."}
    (dest / "capture.json").write_text(json.dumps(record, indent=2) + "\n")
    return dest


class StreamWriter:
    """Bounded immutable RGB/depth files, exposed together by an atomic manifest."""
    def __init__(self, folder, metadata, keep=90):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.metadata = metadata
        self.stream = uuid.uuid4().hex
        self.seq, self.keep, self.files = 0, keep, deque()

    def publish(self, rgb, depth, captured_at, depth_captured_at):
        import cv2
        if rgb.shape[:2] != depth.shape or depth.dtype != np.uint16:
            raise ValueError("Expected aligned uint16 millimetre depth")
        if abs(captured_at-depth_captured_at) > .033:
            raise ValueError("Unsynchronized OAK frames")
        if not all(0 <= time.time()-stamp <= 1 for stamp in (captured_at, depth_captured_at)):
            raise ValueError("OAK capture timestamps are stale or in the future")
        self.seq += 1
        files = []
        hashes = []
        for suffix, im in (("rgb.jpg", rgb), ("depth.png", depth)):
            path = self.folder / f"{self.stream}-{self.seq:09d}-{suffix}"
            encoded = cv2.imencode(path.suffix, im)[1].tobytes()
            with path.open("xb") as f:
                f.write(encoded)
            files.append(path)
            hashes.append(hashlib.sha256(encoded).hexdigest())
        record = {**self.metadata, "schema": 1, "camera_id": "oak-"+self.metadata["device_id"],
                  "stream_id": self.stream, "seq": self.seq,
                  "captured_at": min(captured_at, depth_captured_at), "rgb_captured_at": captured_at,
                  "depth_captured_at": depth_captured_at, "host": os.uname().nodename,
                  "width": depth.shape[1], "height": depth.shape[0], "image": files[0].name,
                  "sha256": hashes[0], "depth_image": files[1].name, "depth_sha256": hashes[1],
                  "depth_units": "mm", "invalid_depth": 0, "robot_frame_calibrated": False}
        tmp = self.folder / f".{self.stream}.manifest.tmp"
        tmp.write_text(json.dumps(record, allow_nan=False))
        tmp.replace(self.folder / "oak.json")
        self.files.append(files)
        while len(self.files) > self.keep:
            for old in self.files.popleft():
                old.unlink(missing_ok=True)
        return record


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["list", "capture", "preview", "stream"])
    parser.add_argument("--device", help="Device ID; required if multiple OAKs are connected")
    parser.add_argument("--usb2", action="store_true", help="Force USB 2 as a connection diagnostic")
    parser.add_argument("--output", type=Path, default=Path("data/oak-captures"))
    parser.add_argument("--timeout", type=float, default=20, help="No-frame timeout in seconds")
    parser.add_argument("--seconds", type=float, default=60, help="Preview or stream duration")
    parser.add_argument("--capture-seconds", type=float, default=2, help="Stream duration before saving a capture")
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.seconds <= 0 or args.capture_seconds <= 0:
        parser.error("Timeout and duration must be positive")
    import depthai as dai
    if dai.__version__.split(".")[0] != "2":
        parser.error("This utility requires DepthAI 2.x; see requirements-oak.txt")
    devices = dai.Device.getAllAvailableDevices()
    if args.mode == "list":
        print(json.dumps([{"id": d.getMxId(), "name": d.name,
                           "state": str(d.state)} for d in devices], indent=2))
        return 0 if devices else 2
    matches = [d for d in devices if not args.device or d.getMxId() == args.device]
    if len(matches) != 1:
        parser.error("Need exactly one matching OAK camera. Run 'list'; use --device with multiple cameras.")
    import cv2
    speed = dai.UsbSpeed.HIGH if args.usb2 else dai.UsbSpeed.SUPER
    with dai.Device(matches[0], speed) as device:
        pipeline = dai.Pipeline()
        rgb = pipeline.create(dai.node.Camera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setSize(640, 360)
        rgb.setVideoSize(640, 360)
        rgb.setFps(15)
        rgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)
        # Keep autofocus from changing the calibrated RGB/depth geometry.
        calibration = device.readCalibration2()
        lens_position = calibration.getLensPosition(dai.CameraBoardSocket.CAM_A)
        if lens_position:
            rgb.initialControl.setManualFocus(lens_position)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        for cam, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
            cam.setBoardSocket(socket)
            cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            cam.setFps(15)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=33))
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 360)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        rgb.video.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])
        output = pipeline.create(dai.node.XLinkOut)
        output.setStreamName("rgbd")
        sync.out.link(output.input)
        metadata = {"device_id": device.getMxId(), "depthai_version": dai.__version__,
                    "usb_speed": str(device.getUsbSpeed()), "alignment": "CAM_A RGB",
                    "stereo_size": [640, 400], "extended_disparity": True,
                    "left_right_check": True, "subpixel": False, "fps": 15,
                    "rgb_undistortion": "factory calibration", "calibrated_lens_position": lens_position,
                    "intrinsics": calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 360),
                    "projection": "rectified_pinhole", "coordinate_frame": "CAM_A_optical"}
        print(json.dumps(metadata), flush=True)
        device.startPipeline(pipeline)
        queue = device.getOutputQueue("rgbd", maxSize=2, blocking=False)
        start = last_frame = time.monotonic()
        frames = 0
        max_sync_skew = 0.0
        point = [320, 180]
        writer = StreamWriter(args.output, metadata) if args.mode == "stream" else None
        if args.mode == "preview":
            cv2.namedWindow("OAK RGB", cv2.WINDOW_AUTOSIZE)
            def click(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    point[:] = [x, y]
            cv2.setMouseCallback("OAK RGB", click)
            print("Click RGB to measure an 11x11 patch. S saves; Q exits. Black depth = missing.", flush=True)
        try:
            while True:
                now = time.monotonic()
                if now - last_frame > args.timeout:
                    raise TimeoutError("No fresh synchronized RGB/depth frames")
                if args.mode in ("preview", "stream") and now - start >= args.seconds:
                    break
                msg = queue.tryGet()
                if msg is None:
                    if args.mode == "preview" and cv2.waitKey(1) & 0xff == ord("q"):
                        break
                    time.sleep(0.01)
                    continue
                last_frame = now
                frames += 1
                color = msg["rgb"].getCvFrame()
                depth = msg["depth"].getFrame()
                if color.shape[:2] != depth.shape:
                    raise RuntimeError(f"RGB {color.shape} and aligned depth {depth.shape} shapes differ; refusing pixel measurements")
                frame_meta = {**metadata, "rgb_device_timestamp_s": msg["rgb"].getTimestampDevice().total_seconds(),
                              "depth_device_timestamp_s": msg["depth"].getTimestampDevice().total_seconds()}
                max_sync_skew = max(max_sync_skew, abs(frame_meta["rgb_device_timestamp_s"] - frame_meta["depth_device_timestamp_s"]))
                frame_meta.update(received_frames=frames, elapsed_s=now-start, max_sync_skew_s=max_sync_skew)
                if writer is not None:
                    # SDK timestamps share dai.Clock.now(); account for USB/queue
                    # delay instead of pretending the frame was just captured.
                    wall, host_clock = time.time(), dai.Clock.now()
                    writer.publish(color, depth,
                                   wall-(host_clock-msg["rgb"].getTimestamp()).total_seconds(),
                                   wall-(host_clock-msg["depth"].getTimestamp()).total_seconds())
                # Allow exposure to settle; a missing-depth capture remains explicitly invalid.
                if args.mode == "capture" and frames >= 30 and now-start >= args.capture_seconds:
                    dest = save_capture(args.output, color, depth, frame_meta)
                    print(f"Saved: {dest}", flush=True)
                    print(json.dumps(depth_stats(depth)), flush=True)
                    return 0 if np.any(depth > 0) else 3
                if args.mode == "preview":
                    stat = patch_stats(depth, *point)
                    median = stat["median_mm"]
                    label = "NO DEPTH" if median is None else f"Z {median:.0f} mm"
                    label += f" | valid {stat['valid_fraction']:.0%}"
                    view = color.copy()
                    cv2.rectangle(view, (point[0]-5, point[1]-5), (point[0]+5, point[1]+5), (255,255,255), 2)
                    cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,0,0), 3)
                    cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1)
                    # Fixed grayscale scale, no dependence on colour discrimination.
                    gray = (np.clip(depth.astype(float), 0, 2000) / 2000 * 255).astype(np.uint8)
                    cv2.imshow("OAK RGB", view)
                    cv2.imshow("Depth: black=missing, white=2000mm+", gray)
                    key = cv2.waitKey(1) & 0xff
                    if key == ord("q"):
                        break
                    if key == ord("s"):
                        print(f"Saved: {save_capture(args.output, color, depth, frame_meta)}", flush=True)
        finally:
            if args.mode == "preview":
                cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
