"""Offline friction-solver controls, with unchanged geometry and torque limits.

Every test keeps the ordinary paddle slip guard enabled. Passing the static
hold is not a folding pass or physical grip validation. No hardware access.
"""
import argparse
import json
import math
from pathlib import Path

from carton.folding_material import CartonMaterial
from carton.folding_paddle import PaddleFoldingSimulation, PaddleSpec
from carton.folding_solver import FoldingSolver
from carton.folding_station import FoldingStation


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root', required=True)
    p.add_argument('--out', required=True)
    args = p.parse_args()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    cases = [
        ('legacy', FoldingSolver(tolerance=1e-10), .8),
        ('impratio10', FoldingSolver(impratio=10, tolerance=1e-10), .8),
        ('impratio50', FoldingSolver(impratio=50, tolerance=1e-10), .8),
        ('noslip3', FoldingSolver.friction(), .8),
        ('noslip3-half-step', FoldingSolver.friction(.001), .8),
        ('zero-friction', FoldingSolver.friction(), 0.),
    ]
    results = []
    for name, solver, friction in cases:
        s = PaddleFoldingSimulation(Path(args.simulation_root), out/name,
            station=FoldingStation(.06, .15, .01,
                table_marker_xy=(-.5, .55), backup_table_marker_xy=(.45, .70)),
            material=CartonMaterial(), paddle=PaddleSpec(friction=friction),
            solver=solver, offset=(0, .0757925946355), yaw=math.pi/6,
            initial_right_roll=1.5, initial_arm_targets={'right': [.28, -.16, .34]})
        row = dict(case=name, task_complete=False, static_hold_passed=False,
                   requested_seconds=30, stop=None)
        try:
            event = s.move({}, 30, 'Stationary gravity hold', capture=False)
            row['static_hold_passed'] = (event['duration_s'] >= 30
                                        and event['bad_penetration_mm'] <= 1.)
        except ValueError as exc:
            row['stop'] = str(exc)
        row['time'] = float(s.data.time)
        row['physics'] = s.save('hold')
        (out/name/'result.json').write_text(json.dumps(row, indent=2))
        results.append(row)
        print(name, row['static_hold_passed'], row['time'], row['stop'], flush=True)
    (out/'solver-controls.json').write_text(json.dumps(dict(
        simulation_only=True, physical_validation=False,
        scope='Unloaded stationary gravity hold only; no folding success claim',
        results=results), indent=2))


if __name__ == '__main__':
    main()
