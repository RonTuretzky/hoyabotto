"""Measure a policy camera's pose in the arm-base frame from one saved image of flat AprilTags. Offline only.

Lay printed tag36h11 tags flat on the tabletop at taped positions in the arm_base frame (see
docs/carton-fold-policy-station-gap.md: origin midway between the SO101 base_link origins, +x robot
right, +y toward the table, +z up), face up and read the right way round from the robot's side (printed top
edge away from the robot, as the defaults assume; a tag turned 180 degrees mirrors the result). Capture one
still from the camera with its own tools (this script opens no camera), then:

    PYTHONPATH=. python tools/camera_pose_from_tag.py --image front.png --camera front \
        --intrinsics oak-640x480.json --tag-size .060 \
        --tag 1 -.12 .25 -.12 --tag 20 .12 .25 -.12 --tag 26 -.12 .40 -.12 --tag 27 .12 .40 -.12 \
        --update station-measurement.json

`--intrinsics` is a JSON with fx, fy, cx, cy, width, height (and optional `distortion`, OpenCV order)
for the exact resolution of the image. One tag works for an oblique camera; a camera looking almost straight
down needs several tags spread over the view (a single small tag there is ill-conditioned: a 100 mm tag at
0.8 m gave 3 cm and 2 degree errors in the rendered check). The result is a camera entry (position_m,
rotation_cv, intrinsics), printed and, with --update, written into the measurement file's `cameras`.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from farm.perception.tag_geometry import square_points
from farm.perception.tags import detect_tags
from farm.status import Reading, Status


def tag_frame(right_axis, top_axis):
    """Rotation from the detector's tag frame to the arm_base frame for a tag lying face up.

    `right_axis`/`top_axis`: arm_base directions of the printed tag's right and top edges as you read it.
    With farm.perception.tag_geometry.square_points and the pupil_apriltags corner order (the same as
    pupil's own pose output), the tag frame's x points to the printed tag's left, y to its top and z into
    the tag (checked against MuJoCo renders of carton.folding_sim.marker, which draws the printed image).
    """
    r = np.asarray(right_axis, float); t = np.asarray(top_axis, float)
    r /= np.linalg.norm(r)
    t = t - r * (r @ t); t /= np.linalg.norm(t)
    return np.column_stack((-r, t, -np.cross(r, t)))


def camera_in_base(image_rgb, intrinsics, tags, tag_size, tag_right=(1, 0, 0), tag_top=(0, 1, 0)):
    """Camera position and OpenCV camera-to-base rotation from tags {id: centre in arm_base}.

    Returns the pose, the tags used, their mean side in pixels and the maximum corner reprojection error.
    """
    h, w = image_rgb.shape[:2]
    if (int(intrinsics['width']), int(intrinsics['height'])) != (w, h):
        raise ValueError(f'Intrinsics are for {intrinsics["width"]}x{intrinsics["height"]}, image is {w}x{h}')
    found = detect_tags(Reading(image_rgb, Status.OK))
    if found.status is not Status.OK:
        raise ValueError(f'Tag detection not trustworthy: {found.note}')
    used = [t for t in tags if t in found.value]
    if not used:
        raise ValueError(f'None of tags {sorted(tags)} detected (seen: {sorted(found.value)})')
    k = np.array([[intrinsics['fx'], 0, intrinsics['cx']], [0, intrinsics['fy'], intrinsics['cy']], [0, 0, 1]], float)
    dist = np.asarray(intrinsics.get('distortion') or [], float)
    dist = dist if dist.size else None
    r_bt = tag_frame(tag_right, tag_top)
    square = square_points(tag_size)
    img = np.concatenate([np.asarray(found.value[t]['corners'], float) for t in used])
    obj = np.concatenate([square @ r_bt.T + np.asarray(tags[t], float) for t in used])
    # Seed from the largest tag's planar (IPPE) solution, then refine on every corner.
    sides = {t: float(np.mean(np.linalg.norm(np.asarray(found.value[t]['corners'])
                                             - np.roll(found.value[t]['corners'], 1, axis=0), axis=1))) for t in used}
    best = max(used, key=sides.get)
    ok, rvec, tvec = cv2.solvePnP(square, np.asarray(found.value[best]['corners'], float), k, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError('solvePnP failed')
    r_ct, _ = cv2.Rodrigues(rvec)
    # Tag-frame solution -> base-frame extrinsics: X_c = R_ct R_bt^T (X_b - p_t) + t.
    r_cb = r_ct @ r_bt.T
    t_cb = tvec.reshape(3) - r_cb @ np.asarray(tags[best], float)
    rvec, tvec = cv2.Rodrigues(r_cb)[0], t_cb.reshape(3, 1)
    if len(used) > 1:
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, k, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, dist)
    reproj = float(np.max(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)))
    r_cb, _ = cv2.Rodrigues(rvec)
    position = -r_cb.T @ tvec.reshape(3)
    rotation_cv = r_cb.T
    forward = rotation_cv[:, 2]
    return {'position_m': position.round(5).tolist(), 'rotation_cv': rotation_cv.round(6).tolist(),
            'pitch_below_horizontal_deg': math.degrees(math.asin(max(-1., min(1., -forward[2])))),
            'yaw_deg_from_plus_y': math.degrees(math.atan2(forward[0], forward[1])),
            'tags_used': used, 'tag_side_px': {str(t): round(s, 1) for t, s in sides.items()},
            'max_reprojection_px': reproj}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image', type=Path, required=True)
    ap.add_argument('--camera', choices=('top', 'front'), required=True)
    ap.add_argument('--intrinsics', type=Path, required=True)
    ap.add_argument('--tag-size', type=float, required=True, help='black square side, metres (measure the print)')
    ap.add_argument('--tag', nargs=4, action='append', required=True, metavar=('ID', 'X', 'Y', 'Z'),
                    help='tag id and its centre in the arm_base frame, metres; repeat for more tags')
    ap.add_argument('--tag-right-axis', type=float, nargs=3, default=(1., 0., 0.),
                    help="arm_base direction of the printed tags' right edge (default: robot right)")
    ap.add_argument('--tag-top-axis', type=float, nargs=3, default=(0., 1., 0.),
                    help="arm_base direction of the printed tags' top edge (default: away from the robot)")
    ap.add_argument('--update', type=Path, help='measurement JSON whose cameras.<camera> entry is replaced')
    args = ap.parse_args(argv)
    bgr = cv2.imread(str(args.image))
    if bgr is None:
        raise SystemExit(f'Cannot read {args.image}')
    tags = {int(t[0]): [float(v) for v in t[1:]] for t in args.tag}
    intrinsics = json.loads(args.intrinsics.read_text())
    pose = camera_in_base(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), intrinsics, tags, args.tag_size,
                          args.tag_right_axis, args.tag_top_axis)
    entry = {'position_m': pose['position_m'], 'rotation_cv': pose['rotation_cv'],
             'intrinsics': {k: intrinsics[k] for k in ('fx', 'fy', 'cx', 'cy', 'width', 'height')},
             'sources': (f'tools/camera_pose_from_tag.py: {args.image.name}, tags {pose["tags_used"]} '
                         f'{args.tag_size * 1000:.0f} mm; reprojection {pose["max_reprojection_px"]:.2f} px')}
    print(json.dumps({**pose, 'camera_entry': entry}, indent=1))
    if args.update:
        m = json.loads(args.update.read_text())
        m.setdefault('cameras', {})[args.camera] = entry
        args.update.write_text(json.dumps(m, indent=1))


if __name__ == '__main__':
    main()
