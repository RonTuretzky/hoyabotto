"""Bounded parallel offline claw-controller trials, with isolated worker files.

This searches deterministic controller candidates and camera-noise seeds. It
does not train neural-network weights or connect to any robot/camera device.
Every trial starts from the declared open carton; no checkpoints reset physics.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def snapshot_sources(software, target):
    """Freeze all local Python dependencies; concurrent edits cannot change a run."""
    target.mkdir()
    hashes = {}
    for package in ('carton', 'farm', 'tools'):
        for source in sorted((software / package).rglob('*.py')):
            relative = source.relative_to(software)
            content = source.read_bytes()
            dest = target / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(content)
            hashes[str(relative)] = hashlib.sha256(content).hexdigest()
    return hashes


def make_jobs(seeds, offsets, near_targets, support_heights):
    if (not seeds or len(set(seeds)) != len(seeds) or any(s < 0 for s in seeds)
            or not offsets or not near_targets or not support_heights
            or len(set(offsets)) != len(offsets) or len(set(near_targets)) != len(near_targets)
            or any(not math.isfinite(x) or abs(x) > .15 for x in offsets)
            or any(not math.isfinite(d) or not -35 <= d <= -10 for d in near_targets)
            or len(set(support_heights)) != len(support_heights)
            or any(not math.isfinite(h) or not .1085 <= h <= .120 for h in support_heights)):
        raise ValueError('Use unique nonnegative seeds, offsets within 150 mm and near targets -35..-10 degrees')
    return [dict(id=f'trial-{i:03d}', seed=seed, carton_offset_x=x, prepare_near_degrees=d,
                 support_height=h)
            for i, (x, d, h, seed) in enumerate(itertools.product(offsets, near_targets, support_heights, seeds))]


def run_trial(job, *, root, snapshot, simulation_root, python, video, timeout):
    work = root / job['id']
    work.mkdir()
    out = work / 'run'
    command = [str(python), '-B', str(snapshot / 'tools/diagnose_short_flap_brace.py'),
               '--simulation-root', str(simulation_root), '--out', str(out),
               '--tool', 'claws', '--fold-right', '--press-left', '--radius', '.115',
               '--carton-yaw-degrees', '0', '--park-back', '--normal-only',
               '--floor-marker-x', '.08', '--center-floor-marker',
               '--width', '1280', '--height', '720', '--open-claw-transfer',
               '--prepare-near-degrees', str(job['prepare_near_degrees']),
               '--support-height', str(job['support_height']),
               '--carton-offset-x', str(job['carton_offset_x']), '--seed', str(job['seed'])]
    if video:
        command.append('--video')
    if job.get('near_release_angle') is not None:
        command += ['--near-after-open-claw', '--near-release-angle',str(job['near_release_angle']),
                    '--near-press-along',str(job.get('near_press_along',0.)),
                    '--near-pre-out','.02','--near-pre-up','0']
    env = dict(os.environ, PYTHONPATH=str(snapshot), PYTHONDONTWRITEBYTECODE='1',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
               VECLIB_MAXIMUM_THREADS='1', NUMEXPR_NUM_THREADS='1', PYTHONUNBUFFERED='1')
    started = time.monotonic()
    record = dict(job, command=command, cwd=str(work), start_utc=datetime.now(timezone.utc).isoformat(),
                  simulation_only=True, hardware_commands=False, full_task_complete=False)
    atomic_json(work / 'worker.json', record)
    print(json.dumps({'started': job['id'], 'pid_parent': os.getpid(), **job}), flush=True)
    with (work / 'stdout.log').open('w') as log:
        try:
            completed = subprocess.run(command, cwd=work, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, timeout=timeout, check=False)
            record['exit_code'] = completed.returncode
        except subprocess.TimeoutExpired:
            record.update(exit_code=None, worker_error=f'Trial exceeded {timeout:g} wall seconds')
    result_path = out / 'result.json'
    if result_path.exists():
        result = json.loads(result_path.read_text())
        record.update(stage=result.get('stage'), error=result.get('error'),
                      angles=result.get('angles'), motion=result.get('motion'),
                      physics_seconds=result.get('time'), result_path=str(result_path),
                      completed_without_error=record.get('exit_code') == 0 and not result.get('error'),
                      near_major_held=bool(result.get('near_major_transfer')),
                      partial_support_passed=bool(record.get('exit_code') == 0
                          and (result.get('open_claw_transfer') or {}).get(
                              'both_shorts_retained_by_right_claw')))
    else:
        record.update(partial_support_passed=False,
                      worker_error=record.get('worker_error', 'No result.json; inspect stdout.log'))
    record['wall_seconds'] = time.monotonic() - started
    record['end_utc'] = datetime.now(timezone.utc).isoformat()
    atomic_json(work / 'worker.json', record)
    print(json.dumps({key: record.get(key) for key in (
        'id', 'partial_support_passed', 'error', 'worker_error', 'wall_seconds')}), flush=True)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-root', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    parser.add_argument('--offsets', type=float, nargs='+', default=[0.])
    parser.add_argument('--near-targets', type=float, nargs='+', default=[-15.])
    parser.add_argument('--support-heights', type=float, nargs='+', default=[.1094])
    parser.add_argument('--near-release-angles', type=float, nargs='+',
                        help='Attempt a major fold after the verified short-flap handoff')
    parser.add_argument('--near-press-along', type=float, default=0.)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--timeout', type=float, default=1200.)
    args = parser.parse_args()
    if not 1 <= args.workers <= 4 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Use 1..4 workers and a positive finite timeout')
    jobs = make_jobs(args.seeds, args.offsets, args.near_targets, args.support_heights)
    if args.near_release_angles:
        if any(not math.isfinite(d) or not -10<=d<=60 for d in args.near_release_angles):
            parser.error('Near release angle must be between -10 and 60 degrees')
        if not math.isfinite(args.near_press_along) or abs(args.near_press_along)>.18:
            parser.error('Near press location must be within 180 mm of the panel center')
        jobs=[dict(job,near_release_angle=angle,near_press_along=args.near_press_along)
              for job in jobs for angle in args.near_release_angles]
        for i,job in enumerate(jobs):job['id']=f'trial-{i:03d}'
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=False)
    software = Path(__file__).resolve().parents[1]
    snapshot = root / 'source-snapshot'
    hashes = snapshot_sources(software, snapshot)
    manifest = dict(simulation_only=True, neural_network_training=False, hardware_commands=False,
                    workers=args.workers, jobs=jobs, source_sha256=hashes,
                    source_root=str(software), python=sys.executable,
                    simulation_root=str(args.simulation_root.resolve()),
                    video_recorded=args.video, results=[], full_task_complete=False)
    atomic_json(root / 'sweep.json', manifest)
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {executor.submit(run_trial, job, root=root, snapshot=snapshot,
                   simulation_root=args.simulation_root.resolve(), python=Path(sys.executable),
                   video=args.video, timeout=args.timeout): job for job in jobs}
        for future in as_completed(futures):
            try:
                row = future.result()
            except Exception as exc:
                row = dict(futures[future], worker_error=str(exc), partial_support_passed=False,
                           full_task_complete=False)
            manifest['results'].append(row)
            manifest['results'].sort(key=lambda r: r['id'])
            atomic_json(root / 'sweep.json', manifest)
    manifest.update(wall_seconds=time.monotonic() - started,
                    partial_support_passes=sum(r['partial_support_passed'] for r in manifest['results']),
                    completed_trials=len(manifest['results']))
    # This is measured worker overlap, not a benchmarked sequential speedup.
    manifest['worker_overlap_factor'] = (sum(r.get('wall_seconds', 0) for r in manifest['results'])
                                         / manifest['wall_seconds'])
    atomic_json(root / 'sweep.json', manifest)
    print(json.dumps({key: manifest[key] for key in (
        'completed_trials', 'partial_support_passes', 'wall_seconds', 'worker_overlap_factor')}), flush=True)


if __name__ == '__main__':
    main()
