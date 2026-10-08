"""Calibrate a camera (e.g. a wrist camera) from saved photos of a printed checkerboard. Offline: it opens no
camera. Writes the intrinsics JSON used by the fold-policy tools and reports the field of view, which the
simulation assumed to be 90 degrees vertical for the wrist cameras.

    python tools/calibrate_camera_checkerboard.py --images 'left_wrist/*.png' --inner 9 6 --square-mm 25 \
        --out left_wrist-640x480.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path

import cv2
import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--images', required=True, help='glob of 15-20 photos, all at the policy resolution')
    ap.add_argument('--inner', type=int, nargs=2, default=(9, 6), help='inner corners across and down')
    ap.add_argument('--square-mm', type=float, required=True, help='measured size of one printed square')
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    cols, rows = args.inner
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square_mm / 1000
    obj, img, size, used = [], [], None, []
    for path in sorted(glob.glob(args.images)):
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        if size is not None and gray.shape[::-1] != size:
            raise SystemExit(f'{path}: all photos must have the same resolution')
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if not ok:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        obj.append(grid)
        img.append(corners)
        used.append(path)
    if len(used) < 10:
        raise SystemExit(f'Only {len(used)} photos showed the whole board; take 15-20 from different angles')
    rms, k, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    w, h = size
    out = {'fx': k[0, 0], 'fy': k[1, 1], 'cx': k[0, 2], 'cy': k[1, 2], 'width': w, 'height': h,
           'distortion': dist.ravel().tolist(), 'rms_reprojection_px': rms, 'photos_used': len(used),
           'vertical_fov_degrees': math.degrees(2 * math.atan(h / (2 * k[1, 1]))),
           'horizontal_fov_degrees': math.degrees(2 * math.atan(w / (2 * k[0, 0])))}
    args.out.write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(out, indent=1))
    if rms > 1.0:
        print('Warning: reprojection error above 1 px; retake blurry or partial photos')


if __name__ == '__main__':
    main()
