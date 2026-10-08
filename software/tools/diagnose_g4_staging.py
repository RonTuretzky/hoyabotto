"""Compare explicit G4 staging hypotheses with frozen contact/vision episodes.

The staging sweep is a privileged dynamics diagnostic. The task episodes use
the existing rendered RGB/depth detector, independent 8/3 registration, real
joint actuators and complete pickup/hold/release scorer. No policy is exported.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

import mujoco
import numpy as np
from PIL import Image

from carton.servo.common import atomic_json
from farm.kinematics.lerobot import pose_error
from farm.perception.gemma_calibration import CalibrationRobot
from planter.g4_sim import G4Owner, G4Simulation, asset_report, sha
from tools.train_g4_pusher import run_episode


LEGACY_PLACEMENTS = [(0., 0.), (.004, .003), (-.004, .006), (.003, -.005)]
EXPANDED_PLACEMENTS = [(x / 1000, y / 1000) for x in (-8, 0, 8) for y in (-8, 0, 8)]
FAULTS = ['missing_tag', 'occluded_tag', 'stale_frame', 'missing_depth',
          'incorrect_registration', 'open_jaws', 'zero_friction', 'disabled_motion']


def snapshot_sources(out):
    software = Path(__file__).resolve().parents[1]
    sources = {Path(__file__).resolve()}
    for module in list(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if filename:
            path = Path(filename).resolve()
            if path.suffix == '.py' and path.is_relative_to(software) and '.venv' not in path.parts:
                sources.add(path)
    hashes = {}
    for source in sorted(sources):
        target = out / 'source' / source.relative_to(software)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[str(source)] = sha(target)
    return hashes


def settled_diagnostic(source, cad, out, geometry, offset, seconds=.75):
    """Robot remains at its initial pose; object offset is episode reset only."""
    sim = G4Simulation(source, cad, out, **geometry)
    address = sim.model.joint('pusher_free').qposadr[0]
    sim.data.qpos[address:address + 2] += offset
    mujoco.mj_forward(sim.model, sim.data)
    before = sim.score_state()
    intersections = sim.intersections(.1)
    stopped = None
    try:
        if intersections:
            raise ValueError(f'Initial intersections after reset: {intersections}')
        sim.advance(seconds, 'staging_settle_diagnostic')
    except ValueError as exc:
        stopped = str(exc)
    after = sim.score_state()
    support = sum(max(0., c['normal_force_n']) for c in after['contacts']
                  if 'staging_rest' in c['geoms'])
    drift = float(np.linalg.norm(np.array(after['grip_world_m']) - before['grip_world_m']) * 1000)
    stable = stopped is None and abs(after['rest_clearance_mm']) < 1 and drift < 2 and support > .005
    result = dict(diagnostic_only=True, deployable_visual_controller=False,
                  offset_m=list(offset), initial_intersections=intersections,
                  seconds=seconds, stable=stable, stop_reason=stopped,
                  grip_drift_mm=drift, staging_support_normal_force_n=support,
                  before=before, after=after, physical_success=False)
    atomic_json(out / 'result.json', result)
    with (out / 'physics.jsonl').open('w') as handle:
        for row in sim.records:
            handle.write(json.dumps(row, allow_nan=False) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-root', type=Path, required=True)
    parser.add_argument('--cad', type=Path, required=True)
    parser.add_argument('--model-directory', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--rest-near-edge', type=float, default=.393)
    parser.add_argument('--rest-far-edge', type=float, default=.455)
    parser.add_argument('--rest-half-width', type=float, default=.055)
    parser.add_argument('--rest-height', type=float, default=.040)
    parser.add_argument('--timestep', type=float, default=.002)
    parser.add_argument('--noslip', type=int, default=3)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--suite', choices=('legacy', 'expanded'), default='legacy')
    parser.add_argument('--evidence-role', choices=('regression', 'development', 'confirmation'), default='regression')
    parser.add_argument('--seed-base', type=int, help='Predeclared independent depth-noise seed series')
    parser.add_argument('--offset-mm', action='append', nargs=2, type=float,
                        help='Explicit diagnostic placement(s), overrides suite; never training offsets')
    parser.add_argument('--settle-only', action='store_true')
    parser.add_argument('--controls', action='store_true')
    parser.add_argument('--correction-mm',nargs=3,type=float,default=[0,-4,4])
    parser.add_argument('--close-angle',type=float,default=-.080)
    parser.add_argument('--grasp-axis',nargs=3,type=float,default=[0,0,1])
    parser.add_argument('--release-height',type=float,default=.008)
    parser.add_argument('--withdrawal-distance',type=float,default=.045)
    parser.add_argument('--disengage-drop',type=float,default=.012)
    parser.add_argument('--direct-withdrawal',action='store_true')
    parser.add_argument('--cartesian-step',type=float,default=0.)
    parser.add_argument('--require-clear-approach',action='store_true')
    parser.add_argument('--require-clean-pickup',action='store_true')
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Use a new output directory; evidence is immutable')
    args.out.mkdir(parents=True)
    sources = snapshot_sources(args.out)
    geometry = dict(rest_near_edge=args.rest_near_edge, rest_far_edge=args.rest_far_edge,
                    rest_half_width=args.rest_half_width, rest_height=args.rest_height,
                    timestep=args.timestep, noslip=args.noslip)
    placements = LEGACY_PLACEMENTS if args.suite == 'legacy' else EXPANDED_PLACEMENTS
    seeds = [1, 100, 101, 102] if args.suite == 'legacy' else list(range(200, 209))
    if args.offset_mm:
        placements = [tuple(np.array(offset) / 1000) for offset in args.offset_mm]
        seeds = list(range(300, 300 + len(placements)))
    if args.seed_base is not None:
        seeds = list(range(args.seed_base, args.seed_base + len(placements)))
    frozen = dict(correction=tuple(np.array(args.correction_mm)/1000),close_angle=args.close_angle,
                  release_height=args.release_height,vertical_withdrawal=not args.direct_withdrawal,
                  grasp_axis=tuple(args.grasp_axis),withdrawal_distance=args.withdrawal_distance,
                  disengage_drop=args.disengage_drop,cartesian_step=args.cartesian_step,
                  require_clear_approach=args.require_clear_approach,require_clean_pickup=args.require_clean_pickup)
    atomic_json(args.out / 'invocation.json', dict(
        argv=sys.argv, source_hashes=sources, simulation_only=True, mujoco=mujoco.__version__,
        geometry=geometry, placements_m=placements, noise_seeds=seeds, evidence_role=args.evidence_role,
        frozen_controller=frozen,
        assumptions=dict(physical_fixture_fit_verified=False, physical_station_measured=False,
                         rest_is_rigid_station_fixture=True, pusher_mass_kg=.012,
                         friction=.8, depth_noise_sigma_mm=.8, depth_dropout=.25),
        limits=dict(policy_exported=False, hardware_released=False, paper_modeled=False,
                    full_planter_success=False, physical_success=False)))
    atomic_json(args.out / 'assets.json', asset_report(args.simulation_root, args.cad, args.model_directory))
    if args.settle_only:
        results = [settled_diagnostic(args.simulation_root, args.cad, args.out / f'settle-{i:02d}',
                                      geometry, offset) for i, offset in enumerate(placements)]
        atomic_json(args.out / 'result.json', dict(diagnostic_only=True, results=results,
                    stable_count=sum(r['stable'] for r in results), total=len(results),
                    policy_exported=False, physical_success=False))
        print(json.dumps(dict(stable_count=sum(r['stable'] for r in results), total=len(results))))
        return
    owner = G4Owner(args.simulation_root, args.cad, args.model_directory, args.out,
                    width=args.width, **geometry)
    owner.sim.record = False
    try:
        Image.fromarray(owner.camera.render()).save(args.out / 'scene.png')
        cfg = dict(schema=1, arm='right', joints=['shoulder_pan', 'wrist_flex'], camera='sim',
                   model_directory=str(args.model_directory.resolve()), lock_file=str(args.out / 'simulation.lock'))
        config_path = args.out / '.private/tag-calibration.json'
        atomic_json(config_path, cfg)
        robot = CalibrationRobot(owner.tagged, config_path, clock=owner.clock)
        robot.catalog()
        response = robot.call('robot_calibrate_tags', {'mode': 'registration'})
        atomic_json(args.out / 'calibration.json', response)
        if not response.get('ok') or response['result']['status'] != 'REGISTRATION_VALIDATED':
            atomic_json(args.out / 'result.json', dict(status='CALIBRATION_FAILED', policy_exported=False))
            return
        owner.registration = response['result']
        error = pose_error(np.array(owner.registration['base_from_camera']), owner.camera_ground_truth())
        atomic_json(args.out / 'calibration-diagnostics.json', dict(
            independent_camera_error_mm=error[0] * 1000, independent_camera_angle_deg=error[1],
            residuals=owner.registration['residuals'], commands=owner.commands))
        sim = owner.sim
        snapshot = {field: getattr(sim.data, field).copy() for field in ('qpos', 'qvel', 'ctrl')}
        snapshot.update(time=float(sim.data.time), qseed=sim.qseed.copy(),
                        friction=sim.model.geom_friction.copy(), rgba=sim.model.geom_rgba.copy())
        role = {'regression': 'development_regression', 'development': 'development',
                'confirmation': 'fresh_confirmation'}[args.evidence_role]
        trials = [run_episode(owner, robot, snapshot, args.out / f'placement-{i:02d}',
                              offset=offset, seed=seeds[i], evaluation_role=role,
                              **frozen) for i, offset in enumerate(placements)]
        controls = []
        if args.controls:
            controls = [run_episode(owner, robot, snapshot, args.out / f'control-{fault}',
                                   fault=fault, seed=1, evaluation_role='fault_control', **frozen) for fault in FAULTS]
        atomic_json(args.out / 'result.json', dict(
            status='STAGING_COMPARISON_COMPLETE', suite=args.suite, evidence_role=args.evidence_role,
            trials=trials, controls=controls,
            passed=sum(r['success'] for r in trials), total=len(trials),
            controls_compared_to_passing_nominal=any(r['success'] and r['object_reset_offset_m'] == [0., 0.] for r in trials),
            policy_exported=False, full_planter_success=False, physical_success=False))
    finally:
        owner.camera.renderer.close()


if __name__ == '__main__':
    main()
