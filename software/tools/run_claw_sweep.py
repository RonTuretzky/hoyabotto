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
    # Camera collision hulls are runtime dependencies, not optional visuals.
    for source in sorted((software / 'carton/assets/wrist-camera').glob('*')):
        if not source.is_file():
            continue
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
               '--tool', 'claws', '--radius', '.115',
               '--carton-yaw-degrees', '0', '--park-back', '--normal-only',
               '--floor-marker-x', '.08', '--center-floor-marker',
               '--width', '1280', '--height', '720',
               '--prepare-near-degrees', str(job['prepare_near_degrees']),
               '--support-height', str(job['support_height']),
               '--carton-offset-x', str(job['carton_offset_x']), '--seed', str(job['seed'])]
    if job.get('open_short_angle') is not None:
        command += ['--open-shorts-first','--open-short-angle',str(job['open_short_angle']),
                    '--near-hold-degrees',str(job.get('near_hold_degrees',90.)),
                    '--near-press-along',str(job.get('near_press_along',0.)),
                    '--near-pre-out',str(job.get('near_pre_out',.02)),'--near-pre-up',str(job.get('near_pre_up',0.))]
    else:
        command += ['--fold-right','--press-left','--open-claw-transfer']
        if job.get('close_majors_after_open_claw'):
            command += ['--close-majors-after-open-claw', '--far-hold-degrees', str(job['majors_far_target'])]
        if job.get('extra_wall_markers'):
            command.append('--extra-wall-markers')
        if job.get('release_far'):
            command.append('--release-far')
        if job.get('left_pinch_opening') is not None:
            command += ['--left-pinch-opening', str(job['left_pinch_opening'])]
        if job.get('pinch_clearance') is not None:
            command += ['--clearance', str(job['pinch_clearance'])]
    if job.get('base_height') is not None:
        command += ['--base-height', str(job['base_height'])]
    for key in ('hinge_stiffness', 'hinge_friction', 'hinge_rest_degrees'):
        if job.get(key) is not None:
            command += ['--'+key.replace('_', '-'), str(job[key])]
    if job.get('far_open_degrees') is not None:
        command += ['--far-open-degrees', str(job['far_open_degrees'])]
    if job.get('far_after_near'):
        command += ['--far-after-near','--far-hold-degrees',str(job.get('far_hold_degrees',90.)),
                    '--far-contact-profile',job.get('far_contact_profile','edge'),
                    '--far-normal-extra',str(job.get('far_normal_extra',0.)),
                    '--far-startup-lift',str(job.get('far_startup_lift',0.)),
                    '--far-gripper-opening',str(job.get('far_gripper_opening',-.17))]
        if job.get('release_far_after'): command.append('--release-far-after')
        if job.get('additional_far_view'): command.append('--additional-far-view')
        if job.get('release_near_after_far'): command.append('--release-near-after-far')
        if job.get('probe_shorts_after_release'):
            command += ['--probe-shorts-after-release', '--short-view-camera',
                        job.get('short_view_camera', 'front'), '--short-contact-policy',
                        job.get('short_contact_policy', 'measured_v2'), '--short-approach-policy',
                        job.get('short_approach_policy', 'elevated_v2'), '--short-stroke-step-degrees',
                        str(job.get('short_stroke_step_degrees', .25))]
            if job.get('allow_primary_carton_absence'):
                command.append('--allow-primary-carton-absence')
            if job.get('observe_primary_open_shorts'):
                command.append('--observe-primary-open-shorts')
    if video:
        command.append('--video')
    if job.get('privileged_near_angle'):
        command.append('--privileged-near-angle')
    if job.get('near_release_angle') is not None:
        command += ['--near-after-open-claw', '--near-release-angle',str(job['near_release_angle']),
                    '--near-press-along',str(job.get('near_press_along',0.)),
                    '--near-pre-out',str(job.get('near_pre_out',.02)),'--near-pre-up',str(job.get('near_pre_up',0.))]
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
                      near_clearance_held=bool(result.get('near_major_clearance')),
                      far_major_held=bool(result.get('far_major_transfer')),
                      far_clearance_held=bool(result.get('far_major_clearance')),
                      both_partial_majors_released=bool((result.get('partial_major_release') or {}).get(
                          'both_majors_passively_retained')),
                      bounded_short_probe_held=bool((result.get('partial_short_probe') or {}).get(
                          'bounded_target_verified')
                          and not (result.get('partial_short_probe') or {}).get('fault')),
                      shorts_opened=bool((result.get('short_opening') or {}).get('physically_opened_and_released')),
                      four_flaps_closed_and_held=bool((result.get('majors_over_shorts') or {}).get('four_flaps_closed_and_held')),
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
        'id', 'partial_support_passed', 'near_major_held', 'near_clearance_held', 'far_major_held',
        'far_clearance_held', 'both_partial_majors_released', 'bounded_short_probe_held',
        'error', 'worker_error', 'wall_seconds')}), flush=True)
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
    parser.add_argument('--open-short-angles', type=float, nargs='+',
                        help='Try physically opening shorts before a major-first fold')
    parser.add_argument('--near-press-along', type=float, default=0.)
    parser.add_argument('--near-pre-out', type=float, default=.02)
    parser.add_argument('--near-pre-up', type=float, default=0.)
    parser.add_argument('--far-after-near', action='store_true')
    parser.add_argument('--additional-far-view', action='store_true',
                        help='Explicit hypothetical second calibrated rendered camera during far stage')
    parser.add_argument('--far-hold-degrees', type=float, default=90.)
    parser.add_argument('--far-contact-profile', choices=['edge','central'], default='edge')
    parser.add_argument('--far-normal-extra', type=float, nargs='+', default=[0.])
    parser.add_argument('--far-startup-lift', type=float, nargs='+', default=[0.])
    parser.add_argument('--far-gripper-openings', type=float, nargs='+', default=[-.17])
    parser.add_argument('--release-far-after', action='store_true')
    parser.add_argument('--release-near-after-far', action='store_true')
    parser.add_argument('--probe-shorts-after-release', action='store_true',
                        help='Bounded paired short probe with explicit additional front-camera assumption')
    parser.add_argument('--short-view-camera', choices=('front','front_left_back','front_right_back'),
                        default='front', help='Explicit hypothetical camera mount for the short probe only')
    parser.add_argument('--allow-primary-carton-absence', action='store_true',
                        help='Require full fresh additional-view geometry when only primary carton identity is absent')
    parser.add_argument('--close-majors-after-open-claw', action='store_true',
                        help='After the open-claw short hold, close far then near majors over the shorts')
    parser.add_argument('--pinch-clearance', type=float, default=None,
                        help='Left-minor pinch tip offset from the panel midplane (m); default -.002. '
                             '-.007 keeps the fixed jaw 8 mm clear of the panel instead of 3.5 mm')
    parser.add_argument('--left-pinch-opening', type=float, default=None,
                        help='Left claw opening (rad) while inserting around the left minor; default .6')
    parser.add_argument('--extra-wall-markers', action='store_true',
                        help='Add proposed printed carton markers 26/27 (near wall) and 28 (left wall)')
    parser.add_argument('--majors-far-target', type=float, default=18.,
                        help='Far angle that pins the shorts before the right claw releases them')
    parser.add_argument('--release-far', action='store_true',
                        help='Comparison variant: let go of the far major before closing the near one')
    parser.add_argument('--hinge-stiffness', type=float, default=None,
                        help='Assumed crease stiffness (N m/rad); default .018')
    parser.add_argument('--hinge-rest-degrees', type=float, default=None,
                        help='Assumed crease rest angle from upright; default 0 (pre-folded cartons rest partway closed)')
    parser.add_argument('--hinge-friction', type=float, default=None,
                        help='Assumed crease dry friction (N m); default .004')
    parser.add_argument('--base-height', type=float, default=.06,
                        help='Assumed arm-base origin height above the tabletop (m)')
    parser.add_argument('--far-open-degrees', type=float, default=None,
                        help='Initial far-flap presentation angle; default keeps the original -5.73 degrees')
    parser.add_argument('--short-stroke-step-degrees', type=float, choices=(.25, .5, 1.), default=.25,
                        help='Declared measured-angle advance per bounded short stroke command')
    parser.add_argument('--short-contact-policy', choices=('measured_v2','setpoint_feedback_v3','tangent_deadband_v4','jaw_surface_v5'),
                        default='measured_v2', help='Explicit bounded Cartesian feedback variant for the short probe')
    parser.add_argument('--observe-primary-open-shorts', action='store_true',
                        help='Use strict hinge-plane evidence from current primary pixels during the short probe')
    parser.add_argument('--short-approach-policy', choices=('elevated_v2', 'whole_jaw_normal_v1'),
                        default='elevated_v2', help='Explicit checked normal approach using original jaw surfaces')
    parser.add_argument('--near-hold-degrees', type=float, nargs='+', default=[90.])
    parser.add_argument('--privileged-near-angle', action='store_true',
                        help='Explicit mechanics-only diagnostic; cannot verify perception or hardware readiness')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--timeout', type=float, default=1200.)
    args = parser.parse_args()
    if not 1 <= args.workers <= 4 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Use 1..4 workers and a positive finite timeout')
    jobs = make_jobs(args.seeds, args.offsets, args.near_targets, args.support_heights)
    if args.open_short_angles and (args.near_release_angles or args.privileged_near_angle):
        parser.error('Major-first opening cannot be combined with the short-hold release profile')
    if args.open_short_angles:
        if any(not math.isfinite(d) or not -35<=d<=-10 for d in args.open_short_angles):
            parser.error('Short opening must be between -35 and -10 degrees')
        jobs=[dict(job,open_short_angle=angle,near_press_along=args.near_press_along)
              for job in jobs for angle in args.open_short_angles]
        for i,job in enumerate(jobs):job['id']=f'trial-{i:03d}'
    if any(not math.isfinite(d) or not 40<=d<=90 for d in args.near_hold_degrees):
        parser.error('Near hold targets must be 40..90 degrees')
    if args.near_hold_degrees != [90.] and not args.open_short_angles:
        parser.error('Partial near holds require physically opened shorts')
    if args.open_short_angles:
        jobs=[dict(job, near_hold_degrees=angle) for job in jobs for angle in args.near_hold_degrees]
        for i,job in enumerate(jobs): job['id']=f'trial-{i:03d}'
    if args.privileged_near_angle:
        if not args.near_release_angles:
            parser.error('Privileged near-angle probing requires a near-major attempt')
        for job in jobs:job['privileged_near_angle']=True
    if args.near_release_angles:
        if any(not math.isfinite(d) or not -10<=d<=60 for d in args.near_release_angles):
            parser.error('Near release angle must be between -10 and 60 degrees')
        if not math.isfinite(args.near_press_along) or abs(args.near_press_along)>.18:
            parser.error('Near press location must be within 180 mm of the panel center')
        jobs=[dict(job,near_release_angle=angle,near_press_along=args.near_press_along)
              for job in jobs for angle in args.near_release_angles]
        for i,job in enumerate(jobs):job['id']=f'trial-{i:03d}'
    if (not math.isfinite(args.near_press_along) or abs(args.near_press_along)>.18
            or any(not math.isfinite(v) or not 0<=v<=.1 for v in (args.near_pre_out,args.near_pre_up))):
        parser.error('Invalid major-flap contact approach geometry')
    if args.far_after_near and not args.open_short_angles:
        parser.error('Far-after-near currently requires the physically opened short-flap profile')
    if args.additional_far_view and not args.far_after_near:
        parser.error('Additional far view requires the explicit far-after-near experiment')
    if args.near_release_angles or args.open_short_angles:
        for job in jobs:
            job.update(near_pre_out=args.near_pre_out,near_pre_up=args.near_pre_up,
                       far_after_near=args.far_after_near, far_hold_degrees=args.far_hold_degrees,
                       release_far_after=args.release_far_after, far_contact_profile=args.far_contact_profile,
                       additional_far_view=args.additional_far_view,
                       release_near_after_far=args.release_near_after_far,
                       probe_shorts_after_release=args.probe_shorts_after_release,
                       short_view_camera=args.short_view_camera,
                       short_contact_policy=args.short_contact_policy,
                       short_approach_policy=args.short_approach_policy,
                       short_stroke_step_degrees=args.short_stroke_step_degrees,
                       observe_primary_open_shorts=args.observe_primary_open_shorts,
                       allow_primary_carton_absence=args.allow_primary_carton_absence)
    if args.close_majors_after_open_claw:
        if args.open_short_angles or args.near_release_angles:
            parser.error('Closing majors over shorts follows the open-claw short hold only')
        if not 15 <= args.majors_far_target <= 45:
            parser.error('Far pinning angle before releasing the shorts must be 15..45 degrees')
        for job in jobs:
            job.update(close_majors_after_open_claw=True, majors_far_target=args.majors_far_target,
                       extra_wall_markers=args.extra_wall_markers, left_pinch_opening=args.left_pinch_opening,
                       pinch_clearance=args.pinch_clearance, release_far=args.release_far)
    for job in jobs:
        job.update(hinge_stiffness=args.hinge_stiffness, hinge_friction=args.hinge_friction,
                   hinge_rest_degrees=args.hinge_rest_degrees)
        if args.base_height != .06:
            job['base_height'] = args.base_height
        if args.far_open_degrees is not None:
            job['far_open_degrees'] = args.far_open_degrees
    if not math.isfinite(args.far_hold_degrees) or not 20<=args.far_hold_degrees<=90:
        parser.error('Far hold target must be20..90 degrees')
    if args.release_far_after and (not args.far_after_near or not 20<=args.far_hold_degrees<=45):
        parser.error('Far release requires a far-after-near target20..45 degrees')
    if args.release_near_after_far and (not args.release_far_after or args.near_hold_degrees != [40.] or args.far_hold_degrees != 35.):
        parser.error('Near release requires the verified near40/far35 passive far-release profile')
    if args.probe_shorts_after_release and not args.release_near_after_far:
        parser.error('Paired short probe requires the both-hands-parked partial release')
    if args.short_view_camera != 'front' and not args.probe_shorts_after_release:
        parser.error('Alternative short camera requires the explicit paired-short probe')
    if args.allow_primary_carton_absence and not args.probe_shorts_after_release:
        parser.error('Fresh-view fallback requires the explicit paired-short probe')
    if args.short_contact_policy != 'measured_v2' and not args.probe_shorts_after_release:
        parser.error('Alternative short contact policy requires the explicit paired-short probe')
    if args.observe_primary_open_shorts and not args.probe_shorts_after_release:
        parser.error('Primary open-short observation requires the explicit paired-short probe')
    if args.short_approach_policy != 'elevated_v2' and not args.probe_shorts_after_release:
        parser.error('Alternative short approach requires the explicit paired-short probe')
    if args.short_stroke_step_degrees != .25 and not args.probe_shorts_after_release:
        parser.error('Alternative short stroke increment requires the explicit paired-short probe')
    if args.far_contact_profile == 'central' and args.far_hold_degrees > 45:
        parser.error('Central contact profile is only proposed through45 degrees')
    if any(not math.isfinite(v) or not 0<=v<=.002 for v in args.far_normal_extra):
        parser.error('Far normal offsets must be 0..2 mm')
    if (any(not math.isfinite(v) or not 0<=v<=.002 for v in args.far_startup_lift)
            or (args.far_contact_profile != 'central' and any(args.far_startup_lift))):
        parser.error('Far startup lift must be 0..2 mm and nonzero only for central contact')
    if any(v not in (-.17,.6,1.4) for v in args.far_gripper_openings):
        parser.error('Choose one of the statically tested far gripper openings')
    if args.far_after_near:
        jobs=[dict(job,far_normal_extra=extra,far_gripper_opening=opening,far_startup_lift=lift)
              for job in jobs for extra in args.far_normal_extra for opening in args.far_gripper_openings
              for lift in args.far_startup_lift]
        for i,job in enumerate(jobs):job['id']=f'trial-{i:03d}'
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=False)
    software = Path(__file__).resolve().parents[1]
    snapshot = root / 'source-snapshot'
    hashes = snapshot_sources(software, snapshot)
    manifest = dict(simulation_only=True, neural_network_training=False, hardware_commands=False,
                    privileged_mechanics_probe=args.privileged_near_angle,
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
                    completed_trials=len(manifest['results']),
                    shorts_opened=sum(bool(r.get('shorts_opened')) for r in manifest['results']),
                    near_major_holds=sum(bool(r.get('near_major_held')) for r in manifest['results']),
                    near_clearance_holds=sum(bool(r.get('near_clearance_held')) for r in manifest['results']),
                    far_major_holds=sum(bool(r.get('far_major_held')) for r in manifest['results']),
                    far_clearance_holds=sum(bool(r.get('far_clearance_held')) for r in manifest['results']),
                    both_partial_major_releases=sum(bool(r.get('both_partial_majors_released')) for r in manifest['results']),
                    bounded_short_probe_holds=sum(bool(r.get('bounded_short_probe_held')) for r in manifest['results']),
                    four_flap_holds=sum(bool(r.get('four_flaps_closed_and_held')) for r in manifest['results']))
    # This is measured worker overlap, not a benchmarked sequential speedup.
    manifest['worker_overlap_factor'] = (sum(r.get('wall_seconds', 0) for r in manifest['results'])
                                         / manifest['wall_seconds'])
    atomic_json(root / 'sweep.json', manifest)
    print(json.dumps({key: manifest[key] for key in (
        'completed_trials', 'partial_support_passes', 'near_major_holds', 'near_clearance_holds',
        'far_major_holds', 'far_clearance_holds', 'both_partial_major_releases', 'bounded_short_probe_holds',
        'four_flap_holds', 'wall_seconds', 'worker_overlap_factor')}), flush=True)


if __name__ == '__main__':
    main()
