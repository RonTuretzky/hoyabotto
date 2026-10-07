"""Scan actual fixed-jaw contact vertices; static diagnostics, no executed fold."""
import argparse
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from carton.folding_far_contact import (FarContactGeometry, far_contact_target, trace_far_sweep,
                                        validate_static_sweep, plan_static_approach,
                                        scan_bridge_clearance, scan_diagonal_bridges)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--side', choices=('left', 'right'), default='right')
    p.add_argument('--bridge-scan', choices=('translated', 'diagonal'))
    p.add_argument('--support-height', type=float, default=.111)
    p.add_argument('--trace-from', type=Path)
    p.add_argument('--candidate-index', type=int, default=0)
    p.add_argument('--end-along', type=float, default=.12)
    p.add_argument('--near-degrees', type=float)
    p.add_argument('--short-degrees', type=float)
    p.add_argument('--other-arm-park', action='store_true')
    p.add_argument('--degrees', type=float, default=-5.6)
    p.add_argument('--clearance', type=float, default=.0015)
    p.add_argument('--radii', nargs='+', type=float, default=[.03, .06, .09, .12, .14])
    p.add_argument('--alongs', nargs='+', type=float, default=[-.175, -.12, -.06, 0., .06, .12, .175])
    p.add_argument('--seeds', type=int, default=8)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--limit-vertices', type=int, default=0)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    model = mujoco.MjModel.from_xml_path(str(args.run / 'scene.xml'))
    states = json.loads((args.run / 'folding-frames.json').read_text())
    if args.bridge_scan:
        result = (scan_bridge_clearance(model, states[-1]['qpos'], height=args.support_height)
                  if args.bridge_scan == 'translated' else
                  scan_diagonal_bridges(model, states[-1]['qpos'], seed=args.seed, random_seeds=args.seeds,
                                        height=args.support_height))
        (args.out/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(dict(static_geometry_only=True, rows=len(result['rows']))))
        return
    geometry = FarContactGeometry(model, states[-1]['qpos'], args.side,
                                 near_degrees=args.near_degrees, other_arm_park=args.other_arm_park,
                                 short_degrees=args.short_degrees)
    if args.trace_from:
        candidate = json.loads(args.trace_from.read_text())['candidates'][args.candidate_index]
        result = trace_far_sweep(geometry, candidate, start_degrees=args.degrees,
                                adapt_clearance=True, end_along=args.end_along)
        result['hypothetical_changes'] = geometry.hypothetical_changes
        result['dense_validation'] = validate_static_sweep(geometry, result)
        try:
            result['approach'] = plan_static_approach(geometry, candidate, np.array(states[0]['qpos'])[geometry.ix])
        except ValueError as error:
            result['approach_error'] = str(error)
        (args.out/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({key: value for key, value in result.items() if key not in ('candidate','waypoints')}))
        return
    rng = np.random.default_rng(args.seed)
    seeds = [geometry.data.qpos[geometry.ix].copy()]
    seeds += [rng.uniform(geometry.limits[:, 0], geometry.limits[:, 1]) for _ in range(args.seeds-1)]
    vertices = geometry.vertices[:args.limit_vertices or None]
    started = time.monotonic()
    rows = []
    for radius in args.radii:
        for along in args.alongs:
            target = far_contact_target(geometry.box, args.degrees, along, radius, args.clearance)
            for vi, vertex in enumerate(vertices):
                for si, seed in enumerate(seeds):
                    row = geometry.solve(target, vertex, seed)
                    row.update(radius_m=radius, along_m=along, vertex=vertex, vertex_index=vi,
                               seed_index=si, target_world=target.tolist())
                    if row['clear']:
                        row['exact_jaw_distance'] = geometry.contact_geometry(row['q'])
                    rows.append(row)
            group = [row for row in rows if row['radius_m'] == radius and row['along_m'] == along]
            print(json.dumps(dict(radius=radius, along=along, clear=sum(row['clear'] for row in group),
                                  best_error_mm=min(row['error_mm'] for row in group))), flush=True)
    valid = sorted([row for row in rows if row['clear']],
                   key=lambda row: (abs(row['exact_jaw_distance']['distance_mm']), row['error_mm']))
    result = dict(static_geometry_only=True, full_task_complete=False, executed_fold=False,
                  hardware_commands=0, source_run=str(args.run), source_state_time=states[-1]['time'],
                  configuration={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                  hypothetical_changes=geometry.hypothetical_changes, candidate_count=len(rows),
                  clear_count=len(valid), elapsed_seconds=time.monotonic()-started, candidates=valid, all_results=rows)
    (args.out/'result.json').write_text(json.dumps(result, indent=2))
    if valid:
        renderer = mujoco.Renderer(model, 720, 1280)
        option = mujoco.MjvOption(); option.geomgroup[3] = 0
        for i, candidate in enumerate(valid[:4]):
            geometry.data.qpos[geometry.ix] = candidate['q']
            mujoco.mj_forward(model, geometry.data)
            for camera in ('side', 'station'):
                renderer.update_scene(geometry.data, camera=camera, scene_option=option)
                im = Image.fromarray(renderer.render().copy()); draw = ImageDraw.Draw(im)
                draw.rectangle((0,0,1280,44),fill='white')
                draw.text((10,8),'STATIC HYPOTHETICAL GEOMETRY - NO FOLD EXECUTED',fill='black')
                draw.text((10,26),f"{args.side} contact radius {candidate['radius_m']:.3f} along {candidate['along_m']:.3f}; near {args.near_degrees}",fill='black')
                im.save(args.out/f'candidate-{i}-{camera}.png')
        renderer.close()
    print(json.dumps({k:v for k,v in result.items() if k not in ('candidates','all_results')}))


if __name__ == '__main__':
    main()
