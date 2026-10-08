"""Summarize recorded bounded short-stroke commands in each panel's fold frame.

Read-only diagnostic over completed simulation trials. For every trial and
arm it reports how far the sensed contact target led the actual CAD vertex
along the fold, radial and hinge directions, how far target and vertex moved
along the fold direction, the measured short angles, and which robot/panel
pairs touched during the stroke in the applied-contact log. It does not
replay physics, alter any run, or score success: partial strokes stay partial
and no hardware readiness follows from it.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from carton.folding_partial_short_probe import _fold_frame

SIDES = ('left', 'right')
AXES = ('fold', 'radial', 'hinge')


def _stats(values):
    values = np.asarray(values, dtype=float)*1e3
    if values.size == 0:
        return None
    return dict(mean_mm=float(values.mean()), std_mm=float(values.std()),
                min_mm=float(values.min()), max_mm=float(values.max()), count=int(values.size))


def _side_summary(stroke, side):
    gaps = {axis: [] for axis in AXES}
    frames, targets, actuals, corrections = [], [], [], 0
    for row in stroke:
        detail = row['robot_substeps'][side]
        target = np.asarray(detail['sensor_target_world'], dtype=float)
        actual = np.asarray(detail['actual_point_world'], dtype=float)
        angle = row['sources'][side]['measured_degrees']
        frame = _fold_frame(np.asarray(row['sources'][side]['world_from_box']), side, angle)
        for axis in AXES:
            gaps[axis].append(float((target-actual) @ frame[axis]))
        frames.append(frame['fold'])
        targets.append(target)
        actuals.append(actual)
        if detail.get('radial_correction_m') or detail.get('hinge_correction_m'):
            corrections += 1
    summary = {axis+'_gap': _stats(gaps[axis]) for axis in AXES}
    if frames:
        fold = frames[0]
        summary.update(net_target_fold_travel_mm=float(1e3*(targets[-1]-targets[0]) @ fold),
                       net_actual_fold_travel_mm=float(1e3*(actuals[-1]-actuals[0]) @ fold),
                       measured_degrees_first=stroke[0]['sources'][side]['measured_degrees'],
                       measured_degrees_last=stroke[-1]['sources'][side]['measured_degrees'],
                       commands_with_recorded_offaxis_correction=corrections)
    return summary


def _stroke_contacts(run, start, end):
    path = run / 'applied-contact-steps.jsonl.gz'
    if not path.exists() or start is None:
        return None
    pairs = {}
    with gzip.open(path, 'rt') as handle:
        for line in handle:
            row = json.loads(line)
            if not start <= row['step_started_at'] <= end:
                continue
            for contact in row['contacts']:
                key = contact['geom1']+' <> '+contact['geom2']
                entry = pairs.setdefault(key, dict(steps=0, max_normal_force_n=0.))
                entry['steps'] += 1
                entry['max_normal_force_n'] = max(entry['max_normal_force_n'],
                                                  abs(contact['wrench_contact_N_Nm'][0]))
    return dict(sorted(pairs.items(), key=lambda item: -item[1]['steps']))


def summarize_run(run):
    run = Path(run)
    path = run / 'result.json'
    result = json.loads(path.read_text())
    probe = result.get('partial_short_probe')
    out = dict(run=str(run), result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
               simulation_only=result.get('simulation_only'), full_task_complete=result.get('full_task_complete'))
    if not isinstance(probe, dict):
        out['partial_short_probe'] = 'absent'
        return out
    commands = probe.get('contact_commands', [])
    stroke = [row for row in commands if row.get('stage') == 'stroke']
    out.update(contact_policy=probe.get('contact_policy'), approach_policy=probe.get('approach_policy'),
               trajectory_policy=probe.get('trajectory_policy'),
               stroke_step_degrees=probe.get('stroke_step_degrees', .25),
               stage=probe.get('stage'), fault=probe.get('fault'),
               bounded_target_verified=probe.get('bounded_target_verified', False),
               approach_commands=len(commands)-len(stroke), stroke_commands=len(stroke),
               independent_final_angles=probe.get('independent_final_angles'),
               max_horizontal_translation_mm=probe.get('motion', {}).get('max_horizontal_translation_mm'),
               sides={side: _side_summary(stroke, side) for side in SIDES})
    if stroke:
        start = stroke[0].get('event_time')
        end = stroke[-1].get('event_time')
        if start is not None and end is not None:
            # The first stroke command's recorded time is after its motion; use
            # the previous command's time as the stroke's physical start.
            index = commands.index(stroke[0])
            start = commands[index-1]['event_time'] if index else start
            out['stroke_contacts'] = _stroke_contacts(run, start, end+.1)
    return out


def summarize(paths):
    runs = []
    for item in paths:
        item = Path(item)
        if (item / 'sweep.json').exists():
            runs += sorted(p / 'run' for p in item.glob('trial-*') if (p / 'run' / 'result.json').exists())
        else:
            runs.append(item)
    return [summarize_run(run) for run in runs]


def _print(rows):
    for row in rows:
        print(f"{row['run']}")
        if 'sides' not in row:
            print('  no partial short probe recorded')
            continue
        print(f"  policy={row['trajectory_policy']} stroke_commands={row['stroke_commands']} "
              f"verified={row['bounded_target_verified']} fault={row['fault']!r}")
        for side in SIDES:
            s = row['sides'][side]
            if not s.get('fold_gap'):
                print(f'  {side}: no stroke commands')
                continue
            print(f"  {side:5s} fold lead {s['fold_gap']['mean_mm']:+.2f}±{s['fold_gap']['std_mm']:.2f} mm, "
                  f"radial {s['radial_gap']['std_mm']:.2f} mm sd, hinge {s['hinge_gap']['std_mm']:.2f} mm sd; "
                  f"vertex moved {s['net_actual_fold_travel_mm']:+.1f} mm along fold; "
                  f"measured {s['measured_degrees_first']:+.2f} -> {s['measured_degrees_last']:+.2f} deg; "
                  f"off-axis corrections {s['commands_with_recorded_offaxis_correction']}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run', nargs='+', required=True, type=Path,
                        help='trial run directories or batch roots containing sweep.json')
    parser.add_argument('--out', type=Path, help='optional JSON summary path')
    args = parser.parse_args(argv)
    rows = summarize(args.run)
    _print(rows)
    if args.out:
        args.out.write_text(json.dumps(dict(read_only_summary=True, physics_replayed=False,
                                            full_task_complete=False, runs=rows), indent=1))
    return rows


if __name__ == '__main__':
    main()
