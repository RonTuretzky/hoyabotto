"""Check the fold-policy station from one head-camera (OAK) frame and say how to fix it. Read-only.

Replaces the tape-measure checks of setup steps 2-4 (docs/auto-station-check.md): from the 12 printed box
tags it finds the carton in the arm_base frame and prints plain instructions ("Raise the table 6 mm",
"Move the carton 8 mm to the robot's left", "Turn the carton 3 degrees clockwise (seen from above)") or OK.

From a saved frame (no robot contact at all):

    cd software && PYTHONPATH=. python tools/check_station.py --image front.jpg --camera-json head-pose.json \
        [--intrinsics oak-640x360.json] [--depth depth.png] [--json report.json]

From the robot, one OAK frame through the chat pilot's authenticated client (robot_get_cameras, or with
--with-depth robot_get_depth; both read-only, no motor):

    cd software && PYTHONPATH=. python tools/check_station.py --pilot-root "$PILOT" --camera-json head-pose.json \
        [--with-depth] [--save-frame front.jpg]

The head-camera pose JSON is documented in carton/station_check.py (schema xlerobot-head-camera-pose/1:
position_m, rotation_cv, fx, fy, cx, cy, dist). Without --camera-json the model-derived 'front' camera of
profiles/fold-station-xlerobot-220.json is used and the report says it is unmeasured.

Intrinsics, first available wins: the OAK manifest bound to the fetched frame; --intrinsics; the camera JSON;
the model's 54 deg vertical field of view (4:3 frames only).

Exit status: 0 everything within tolerance (OK), 1 adjustments needed, 2 cannot check or the box does not
match the training carton.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carton import station_check as sc  # noqa: E402

READ_ONLY_TOOLS = ('robot_get_cameras', 'robot_get_depth')


def _decode(record, flags):
    raw = base64.b64decode(record.get('data_base64') or record.get('base64') or '', validate=True)
    if not raw:
        raise ValueError('camera record carries no pixels')
    if record.get('sha256') and hashlib.sha256(raw).hexdigest() != record['sha256']:
        raise ValueError('camera image hash mismatch')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), flags)
    if image is None:
        raise ValueError('cannot decode camera image')
    return image, raw


def frame_from_payload(payload, tool):
    """(bgr, depth_mm or None, manifest, rgb_record) from a robot_get_cameras / robot_get_depth payload."""
    if not isinstance(payload, dict) or payload.get('ok') is not True:
        raise RuntimeError(f'{tool} failed: {str(payload)[:300]}')
    result = payload.get('result') or {}
    manifest = (result.get('manifest') if tool == 'robot_get_depth'
                else (result.get('cameras') or {}).get('oak')) or {}
    rgb = depth = None
    for record in payload.get('images') or []:
        if not isinstance(record, dict) or not str(record.get('camera_id', '')).startswith('oak'):
            continue
        if record.get('mime_type') == 'image/jpeg' and rgb is None:
            rgb = record
        elif record.get('mime_type') == 'image/png' and str(record.get('camera_id')).endswith(':depth'):
            depth = record
    if rgb is None:
        errors = result.get('camera_errors') or {}
        raise RuntimeError(f'{tool} returned no OAK RGB frame {errors or ""}')
    bgr, _ = _decode(rgb, cv2.IMREAD_COLOR)
    depth_mm = _decode(depth, cv2.IMREAD_UNCHANGED)[0] if depth is not None else None
    return bgr, depth_mm, manifest, rgb


def fetch_frame(pilot_root, with_depth=False):
    """One OAK frame through the chat pilot's read-only client."""
    pilot_root = Path(pilot_root).resolve()
    sys.path.insert(0, str(pilot_root))
    robot = importlib.import_module('chat_server').Robot(pilot_root / '.private/robot.json')
    tool = 'robot_get_depth' if with_depth else 'robot_get_cameras'
    assert tool in READ_ONLY_TOOLS
    payload = robot.call(tool, {} if with_depth else {'cameras': ['oak']})
    return frame_from_payload(payload, tool)


def manifest_intrinsics(manifest, record, shape):
    """(K, dist, note) bound to the fetched frame (farm.perception.tag_geometry.camera_calibration), or None."""
    from farm.perception.tag_geometry import camera_calibration
    try:
        k, dist, binding = camera_calibration(manifest, record, shape)
    except (ValueError, TypeError, KeyError) as exc:
        return None, str(exc)
    return (k, dist, f'OAK manifest ({binding["projection"]}, {shape[1]}x{shape[0]})'), None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--image', type=Path, help='saved head-camera frame (JPEG/PNG)')
    src.add_argument('--pilot-root', type=Path, help='chat pilot checkout: fetch one OAK frame (read-only)')
    ap.add_argument('--camera-json', type=Path, help='head-camera pose in arm_base (xlerobot-head-camera-pose/1)')
    ap.add_argument('--profile', type=Path, default=sc.PROFILE,
                    help='fallback model camera when --camera-json is absent (default: %(default)s)')
    ap.add_argument('--intrinsics', type=Path, help='JSON with fx, fy, cx, cy[, width, height, dist]')
    ap.add_argument('--depth', type=Path, help='aligned OAK depth PNG (uint16 mm) for a second table estimate')
    ap.add_argument('--with-depth', action='store_true', help='with --pilot-root: use robot_get_depth (RGB+depth)')
    ap.add_argument('--save-frame', type=Path, help='with --pilot-root: save the fetched frame here')
    ap.add_argument('--json', type=Path, help='write the full report here')
    args = ap.parse_args(argv)

    camera = sc.load_camera(args.camera_json) if args.camera_json else sc.model_camera(args.profile)
    depth_mm, bound, bound_error = None, None, None
    if args.image:
        bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if bgr is None:
            ap.error(f'cannot read {args.image}')
        if args.depth:
            depth_mm = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
            if depth_mm is None or depth_mm.ndim != 2:
                ap.error(f'cannot read a single-channel depth PNG from {args.depth}')
    else:
        bgr, depth_mm, manifest, record = fetch_frame(args.pilot_root, args.with_depth)
        bound, bound_error = manifest_intrinsics(manifest, record, bgr.shape)
        if args.save_frame:
            cv2.imwrite(str(args.save_frame), bgr)
    if bound is not None:
        k, dist, note = bound
    else:
        intr = sc._intrinsics_from(json.loads(args.intrinsics.read_text())) if args.intrinsics else camera['intrinsics']
        try:
            k, dist, note = sc.camera_matrix(intr, bgr.shape, camera.get('fovy_deg'))
        except ValueError as exc:
            ap.error(str(exc) + (f' (OAK manifest: {bound_error})' if bound_error else ''))
    if depth_mm is not None and depth_mm.shape[:2] != bgr.shape[:2]:
        print(f'Warning: depth {depth_mm.shape[1]}x{depth_mm.shape[0]} does not match the RGB frame; ignored.',
              file=sys.stderr)
        depth_mm = None
    report = sc.check_frame(bgr, camera, k, dist, depth_mm=depth_mm)
    report['intrinsics'] = {'source': note, 'K': np.round(k, 3).tolist(),
                            'dist': np.round(np.ravel(dist), 6).tolist()}
    report['motor_writes'] = 0
    print(sc.format_report(report))
    if args.json:
        args.json.write_text(json.dumps(report, indent=1, default=float))
    return {'OK': 0, 'ADJUST': 1}.get(report['status'], 2)


if __name__ == '__main__':
    raise SystemExit(main())
