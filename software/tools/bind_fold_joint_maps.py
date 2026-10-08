"""Bind the fold-policy joint maps (profiles/fold-joint-maps/) to the robot Mac's live calibration file. No motors.

The maps were written from the calibration read live on 2026-10-08 (calibration-2026-10-08.json). The runner ties a
map to the exact calibration file it runs with, by path and SHA-256. This tool checks that the live file has the same
homing offset and range for all twelve arm motors, then writes bound copies that point at it. Any difference refuses:
the arms were recalibrated and the maps must be redone.

    python tools/bind_fold_joint_maps.py --calibration /path/to/live/farm_xlerobot.json --out bound-maps/
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / 'profiles' / 'fold-joint-maps'
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
KEYS = ('homing_offset', 'range_min', 'range_max')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--calibration', type=Path, required=True, help='the live saved calibration on the robot Mac')
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    live = json.loads(args.calibration.read_text())
    snap = json.loads((HERE / 'calibration-2026-10-08.json').read_text())
    diffs = []
    for arm in ('left', 'right'):
        for j in JOINTS:
            name = f'{arm}_arm_{j}'
            for k in KEYS:
                a, b = (snap.get(name) or {}).get(k), (live.get(name) or {}).get(k)
                if a != b:
                    diffs.append(f'{name}.{k}: maps {a} vs live {b}')
    if diffs:
        raise SystemExit('Refused: the live calibration differs from the one the maps were written for '
                         '(re-read it and regenerate the maps):\n  ' + '\n  '.join(diffs))
    sha = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    args.out.mkdir(parents=True, exist_ok=True)
    for arm in ('left', 'right'):
        m = json.loads((HERE / f'{arm}-joint-map.json').read_text())
        m['calibration_file'] = str(args.calibration.resolve())
        m['calibration_sha256'] = sha
        (args.out / f'{arm}-joint-map.json').write_text(json.dumps(m, indent=1) + '\n')
    print(f'Bound both maps to {args.calibration} (sha256 {sha[:12]}...) in {args.out}')


if __name__ == '__main__':
    main()
