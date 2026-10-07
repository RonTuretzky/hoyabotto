"""Joint-actuated eight-fit/three-validation G4 rendered registration diagnostic.

No hardware backend, installation, or task targets exist here. The complete
rigid station remains dynamic, with original actuator/joint limits and contact
gates. Encoder bytes and camera timing are simulated, not hardware validation.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
import shutil
import traceback
import uuid
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from carton.servo.common import Limits, digest
from carton.servo.tag_calibration import registration_offsets
from farm.kinematics.assets import verified_model
from farm.kinematics.lerobot import LeRobotSO101, pose_error
from farm.kinematics.tag_registration import assemble_dataset, fit_registration
from farm.perception.gemma_tags import TagObserver
from farm.perception.tag_sampling import stationary_sample
from planter.g4_station import JOINTS, SIDES, StationConfig, StationSimulation, sha
from planter.g4_station_vision import capture, geometry_for_cameras


def save(path, value): path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def telemetry(sim, raw_ranges):
    rows = []
    stamp = 1000.+float(sim.data.time)
    for side in SIDES:
        for name in JOINTS:
            joint = sim.model.joint(side+'_'+name)
            ticks = int(round(2048.+sim.data.qpos[joint.qposadr[0]]*4096/(2*math.pi)))
            velocity = float(sim.data.qvel[joint.dofadr[0]])*4096/(2*math.pi)
            rows.append(dict(name=side+'_arm_'+name, Present_Position=ticks,
                Present_Velocity=velocity, Moving=int(abs(velocity)>1), Status=0, captured_at=stamp))
    rows += [dict(name='head_motor_'+str(i), Present_Position=2048,
                  Present_Velocity=0, Moving=0, Status=0, captured_at=stamp) for i in (1, 2)]
    return dict(ok=True, result=dict(cached=False, motors=rows, raw_calibration_ranges=raw_ranges),
                simulation_only=True, note='Arm velocity/position from dynamics; head/status are static fixtures')


def move_joints(sim, target, phase, seconds=.35):
    target = np.asarray(target)
    if (target.shape != sim.data.ctrl.shape or not np.isfinite(target).all()
            or np.any(target<sim.model.actuator_ctrlrange[:, 0])
            or np.any(target>sim.model.actuator_ctrlrange[:, 1])):
        raise ValueError('Original actuator limits reject joint target')
    start = sim.data.ctrl.copy(); count = math.ceil(seconds/sim.model.opt.timestep)
    sim.commands.append(dict(phase=phase, time_s=float(sim.data.time), ctrl=target.tolist(), duration_s=seconds))
    for step in range(count):
        f = (step+1)/count; f = f*f*(3-2*f)
        sim.data.ctrl[:] = start+(target-start)*f; sim.step(phase)


def settle(sim, phase, maximum_s=1.2):
    quiet = 0
    dofs = [sim.model.joint(s+'_'+j).dofadr[0] for s in SIDES for j in JOINTS]
    for _ in range(math.ceil(maximum_s/sim.model.opt.timestep)):
        sim.step(phase)
        quiet = quiet+1 if np.max(np.abs(sim.data.qvel[dofs]))*4096/(2*math.pi) < .8 else 0
        if quiet*sim.model.opt.timestep >= .05: return
    raise ValueError('Actual arm encoder velocity did not settle within bounded hold')


def camera_truth_in_base(model, data, arm, camera):
    # Evaluation only; never supplied to detector, sampler, encoder FK or fitter.
    root = int(model.jnt_bodyid[model.joint(arm+'_shoulder_pan').id])
    while model.body_parentid[root] != 0: root = int(model.body_parentid[root])
    world_base = np.eye(4); world_base[:3, :3] = data.xmat[root].reshape(3, 3); world_base[:3, 3] = data.xpos[root]
    cid = model.camera(camera).id
    world_camera = np.eye(4); world_camera[:3, :3] = data.cam_xmat[cid].reshape(3, 3)@np.diag([1, -1, -1])
    world_camera[:3, 3] = data.cam_xpos[cid]
    return np.linalg.inv(world_base)@world_camera


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', type=Path, required=True); p.add_argument('--model-directory', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True); p.add_argument('--camera', default='station')
    p.add_argument('--arm', choices=SIDES, default='left'); p.add_argument('--baseline-wrist', type=float, default=-.3)
    p.add_argument('--grid-radius-ticks', type=int, default=96)
    p.add_argument('--seed', type=int, default=921)
    a = p.parse_args()
    # Keep the existing production commissioning envelope; never widen Limits.
    limits = Limits(trust_ticks=a.grid_radius_ticks, max_path_ticks=max(1500, 13*a.grid_radius_ticks))
    a.out.mkdir(parents=True, exist_ok=False)
    source = {str(a.scene.resolve()): sha(a.scene)}
    source.update({m.get('file'): sha(m.get('file')) for m in E.parse(a.scene).findall('./asset/mesh')})
    shutil.copyfile(a.scene, a.out/'scene.xml')
    model = mujoco.MjModel.from_xml_path(str(a.out/'scene.xml'))
    fk = LeRobotSO101(a.model_directory); urdf, manifest = verified_model(a.model_directory)
    source[str(urdf)] = sha(urdf)
    save(a.out/'invocation.json', dict(scene=str(a.scene), source_hashes=source, seed=a.seed,
         arm=a.arm, camera=a.camera, baseline_wrist=a.baseline_wrist,
         grid_radius_ticks=a.grid_radius_ticks, maximum_path_ticks=limits.max_path_ticks, hardware_commands=0,
         declared_sizes_and_mounts=geometry_for_cameras([a.camera]),
         scope=__doc__, installs_registration=False, full_planter_success=False))
    config = StationConfig(width=int(model.vis.global_.offwidth), timestep_s=float(model.opt.timestep),
                           noslip_iterations=int(model.opt.noslip_iterations))
    initial = {s:[0, 0, 0, 0, 0, .55] for s in SIDES}; initial[a.arm][3] = a.baseline_wrist
    sim = StationSimulation(model, a.out, config=config, initial_joint_positions=initial)
    ranges = {s+'_arm_'+j:dict(min_ticks=0, max_ticks=4096) for s in SIDES for j in JOINTS}
    for side in SIDES:
        for j in fk.names:
            radius = math.floor(min(abs(fk.ranges[j]))*4096/360)
            ranges[side+'_arm_'+j] = dict(min_ticks=2048-radius, max_ticks=2048+radius)
    status = dict(ok=True, result=dict(configuration=dict(config=dict(arm=a.arm,
        mapping='feetech_degrees_v1', calibration_sha256=digest(ranges)),
        model_assets=dict(verified=True, revision=manifest['revision']))))
    plan = registration_offsets([a.arm+'_arm_shoulder_pan', a.arm+'_arm_wrist_flex'], limits)
    save(a.out/'plan.json', plan)
    stream = 'g4-actuated-registration-'+uuid.uuid4().hex; rng = np.random.default_rng(a.seed)
    observer = TagObserver(clock=lambda:1000.+float(sim.data.time), geometry=geometry_for_cameras([a.camera]))
    captures = []; result = dict(status='INCOMPLETE', physics_replay_verified=False,
        registration_installed=False, hardware_commands=0, physical_success=False, full_planter_success=False)
    try:
        settle(sim, 'calibration_initial_settle')
        base_command = sim.data.ctrl.copy()
        for i, pose in enumerate(plan['poses']):
            target = base_command.copy()
            for name, offset in pose['offset_ticks'].items():
                actuator = model.actuator(name.replace('_arm_', '_')).id
                target[actuator] += offset*2*math.pi/4096
            move_joints(sim, target, f'calibration_move_{i:02d}')
            settle(sim, f'calibration_settle_{i:02d}')
            before = telemetry(sim, ranges); sim.step(f'calibration_bracket_{i:02d}')
            payload, depth, binding = capture(sim.renderer, model, sim.data, a.camera,
                option=sim.option, stream_id=stream, seq=i+1, timestamp=1000.+float(sim.data.time), rng=rng)
            observed = observer.observe(payload, [a.camera], ids=[1, 2, 4, 41], include_images=False)
            sim.step(f'calibration_bracket_{i:02d}'); after = telemetry(sim, ranges)
            row = observed['result']['observations'][a.camera]
            directory = a.out/f'pose-{i:02d}'; directory.mkdir()
            (directory/'rgb.png').write_bytes(base64.b64decode(payload['images'][0]['data_base64']))
            np.savez_compressed(directory/'depth.npz', aligned_depth_m=depth); save(directory/'depth-binding.json', binding)
            entry = dict(before=before, after=after, observation=observed, arm_geometry_status=status, sample=None)
            try:
                entry['sample'] = stationary_sample(before, after, row, a.arm)
                entry['sample']['split'] = pose['split']
            finally: save(directory/'capture.json', entry)
            captures.append(entry)
            print(json.dumps(dict(pose=i, split=pose['split'], time_s=float(sim.data.time),
                                  sample_accepted=True)), flush=True)
        move_joints(sim, base_command, 'calibration_return'); settle(sim, 'calibration_return_settle')
        dataset = assemble_dataset(captures, a.model_directory); save(a.out/'dataset.json', dataset)
        fitted = fit_registration(dataset); save(a.out/'fit.json', fitted)
        result.update(status=fitted['status'], registration_fit=fitted)
        if fitted['status']=='REGISTRATION_VALIDATED':
            truth = camera_truth_in_base(model, sim.data, a.arm, a.camera)
            error = pose_error(np.asarray(fitted['base_from_camera']), truth)
            result['independent_camera_error'] = dict(translation_mm=error[0]*1000, rotation_deg=error[1],
                true_base_from_camera=truth.tolist(), scope='Ground truth evaluation only; not fitter input')
    except Exception as error:
        result['error'] = str(error); (a.out/'error.txt').write_text(traceback.format_exc())
        print(traceback.format_exc(), flush=True)
    finally:
        sim.save()
    result.update(accepted_samples=len(captures), physics_steps=sim.step_index,
                  max_force_n=sim.max_force, max_penetration_mm=sim.max_penetration*1000,
                  final_physics_time_s=float(sim.data.time),
                  source_hashes_unchanged=all(sha(path)==h for path, h in source.items()))
    save(a.out/'result.json', result)
    print(json.dumps({k:v for k,v in result.items() if k!='registration_fit'}), flush=True)


if __name__ == '__main__': main()
