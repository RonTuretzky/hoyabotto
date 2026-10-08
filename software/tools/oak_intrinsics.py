"""Print the head OAK-D Lite's factory RGB intrinsics at a chosen resolution, as the JSON that
tools/camera_pose_from_tag.py --intrinsics expects. Reads the camera's stored calibration only:
no motors, no streaming. Run from Terminal on the robot Mac (macOS camera permission is per app).

    python tools/oak_intrinsics.py --width 640 --height 480 --out oak-640x480.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    if abs(args.width / args.height - 4 / 3) > 1e-6:
        raise SystemExit('The fold policy expects a 4:3 full-sensor image (e.g. 640x480)')
    import depthai as dai
    with dai.Device() as device:
        cal = device.readCalibration2()
        k = cal.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, args.width, args.height)
        dist = cal.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        serial = device.getMxId()
    fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
    out = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'width': args.width, 'height': args.height,
           'distortion': list(dist), 'source': f'OAK factory calibration, device {serial}',
           'vertical_fov_degrees': math.degrees(2 * math.atan(args.height / (2 * fy)))}
    args.out.write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
