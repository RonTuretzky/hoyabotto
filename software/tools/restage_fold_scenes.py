"""Re-stage recorded fold demonstrations with measured policy cameras, without re-recording. Simulation only.

The scripted controller never looks through the policy cameras (`overhead` = `top`, `front` = `front`; it
registers through `station`), so changing those cameras, the arm/table colours, the table size or hiding
the printed markers does not change any recorded state. This writes a mirror batch whose trials link the
original `demo.npz`/`demo.json`/`status.json` and carry a restaged `run/scene.xml`; point the unchanged
tools/fold_demos_to_lerobot.py (`--batches MIRROR`) and tools/eval_fold_policy.py (its holdout.json then
names mirror trials) at it.

A measurement whose station (base height, base line to table edge, base spacing) differs from the
recorded scene is refused: that moves the arms relative to the carton and needs new demonstrations
(tools/record_measured_fold_demos.py).

    PYTHONPATH=. python tools/restage_fold_scenes.py --measurement station.json \
        --batches .../fold-demos/batch-01 .../fold-demos/batch-02 --out .../fold-demos/restaged-01

`--in-place` instead rewrites each trial's `run/scene.xml`, keeping the recorded one as
`run/scene.recorded.xml` and always restaging from that copy, so it can be repeated.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from carton.folding_station_measured import load_measurement, restage_scene_xml

LINKED = ('demo.npz', 'demo.json', 'status.json')


def restage_batch(batch, measurement, out, *, copy_files=False):
    out.mkdir(parents=True, exist_ok=True)
    done = []
    for trial in sorted(batch.glob('trial-*')):
        scene = trial / 'run/scene.xml'
        if not (trial / 'demo.json').exists() or not scene.exists():
            continue
        target = out / trial.name
        if target.exists():
            raise SystemExit(f'{target} exists; choose a new output directory')
        (target / 'run').mkdir(parents=True)
        for name in LINKED:
            if (trial / name).exists():
                if copy_files:
                    (target / name).write_bytes((trial / name).read_bytes())
                else:
                    os.symlink((trial / name).resolve(), target / name)
        report = restage_scene_xml(scene, target / 'run/scene.xml', measurement)
        (target / 'run/measured-station.json').write_text(json.dumps(report, indent=1))
        done.append(trial.name)
    return done


def restage_in_place(batch, measurement):
    done = []
    for trial in sorted(batch.glob('trial-*')):
        scene = trial / 'run/scene.xml'
        if not (trial / 'demo.json').exists() or not scene.exists():
            continue
        recorded = trial / 'run/scene.recorded.xml'
        if not recorded.exists():
            recorded.write_bytes(scene.read_bytes())
        report = restage_scene_xml(recorded, scene, measurement)
        (trial / 'run/restaged-cameras.json').write_text(json.dumps(report, indent=1))
        done.append(trial.name)
    return done


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--measurement', type=Path, required=True)
    ap.add_argument('--batches', type=Path, nargs='+', required=True)
    ap.add_argument('--out', type=Path, help='mirror batch directory (or use --in-place)')
    ap.add_argument('--in-place', action='store_true')
    ap.add_argument('--copy', action='store_true', help='copy demo files instead of symlinking them')
    ap.add_argument('--allow-unmeasured', action='store_true',
                    help='accept a file with "measured": false (examples, rendering checks)')
    args = ap.parse_args(argv)
    measurement = load_measurement(args.measurement)
    if not (measurement.get('measured') or measurement.get('model_derived')) and not args.allow_unmeasured:
        raise SystemExit('Measurement file is marked "measured": false; pass --allow-unmeasured for a check run')
    if args.in_place:
        for batch in args.batches:
            trials = restage_in_place(batch, measurement)
            (batch / 'restage.json').write_text(json.dumps({'measurement': measurement, 'trials': trials,
                                                            'measurement_file': str(args.measurement.resolve()),
                                                            'simulation_only': True}, indent=1))
            print(f'{batch}: {len(trials)} trials restaged in place', flush=True)
        return
    if args.out is None:
        raise SystemExit('Give --out or --in-place')
    if args.out.exists():
        raise SystemExit(f'{args.out} exists; choose a new output directory')
    args.out.mkdir(parents=True)
    summary = {'measurement': measurement, 'measurement_file': str(args.measurement.resolve()),
               'simulation_only': True, 'batches': {}}
    # Several batches may reuse trial names; keep one sub-batch per source batch.
    for batch in args.batches:
        sub = args.out / batch.name if len(args.batches) > 1 else args.out
        summary['batches'][str(batch)] = {'out': str(sub), 'trials': restage_batch(batch, measurement, sub,
                                                                                    copy_files=args.copy)}
        print(f'{batch}: {len(summary["batches"][str(batch)]["trials"])} trials restaged into {sub}', flush=True)
    (args.out / 'restage.json').write_text(json.dumps(summary, indent=1))


if __name__ == '__main__':
    sys.exit(main())
