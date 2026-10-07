"""Replay recorded bench joint controls in the complete G4 rigid station.

This is a transferred-command mechanical experiment, not a visual controller.
Only initial joint positions are set; all subsequent changes use the original
joint actuators. No recorded object pose is applied to the running simulation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import traceback
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from planter.g4_station import JOINTS, SIDES, StationConfig, StationSimulation, sha


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def source_rows(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def checked_command(model, command):
    if (command.shape != (model.nu,) or not np.isfinite(command).all()
            or np.any(command < model.actuator_ctrlrange[:, 0])
            or np.any(command > model.actuator_ctrlrange[:, 1])):
        raise ValueError('Recorded command violates original joint actuator limits')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--transfer', type=Path, required=True)
    p.add_argument('--station-initial', type=Path, required=True)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--initial-settle', type=float, default=.3)
    p.add_argument('--no-render', action='store_true')
    a = p.parse_args()
    if not 0 <= a.initial_settle <= 1:
        p.error('Initial settle must be within0..1seconds')
    a.out.mkdir(parents=True, exist_ok=False)
    scene = a.transfer/'scene.xml'
    predeclared = json.loads((a.transfer/'predeclared.json').read_text())
    geometry = json.loads((a.transfer/'result.json').read_text())
    if (not geometry['source_inputs_unchanged']
            or geometry['samples_with_unexpected_penetration_over_1um']
            or geometry['transformed_scene_sha256'] != sha(scene)):
        raise ValueError('Transferred static scene is changed or has known sampled intersections')
    expected_trace = str((a.episode/'physics.jsonl').resolve())
    if predeclared['source_hashes'].get(expected_trace) != sha(expected_trace):
        raise ValueError('Recorded control trace is not the sampled source trajectory')
    inputs = [scene, a.transfer/'predeclared.json', a.transfer/'result.json',
              a.station_initial, a.episode/'physics.jsonl', a.episode/'commands.json',
              a.episode/'result.json', Path(__file__)]
    inputs += [Path(m.get('file')) for m in E.parse(scene).findall('./asset/mesh')]
    hashes = {str(path.resolve()): sha(path) for path in inputs}
    shutil.copyfile(scene, a.out/'scene.xml')
    model = mujoco.MjModel.from_xml_path(str(a.out/'scene.xml'))
    station_initial = np.load(a.station_initial)
    rows = source_rows(a.episode/'physics.jsonl')
    first = next(rows)
    initial = {s:station_initial['qpos'][[int(model.joint(s+'_'+name).qposadr[0]) for name in JOINTS]].tolist() for s in SIDES}
    initial['right'] = first['encoder_radians']
    configuration = StationConfig(width=int(model.vis.global_.offwidth),
        timestep_s=float(model.opt.timestep), noslip_iterations=int(model.opt.noslip_iterations))
    invocation = dict(argv=sys.argv, scope=__doc__, source_hashes=hashes,
        transferred_geometry=predeclared, current_perception_used=False,
        controller='Recorded source-episode per-step actuator controls for right arm; left arm holds initial command.',
        source_episode_name=a.episode.name, source_control_trace=expected_trace,
        source_control_sha256=sha(expected_trace),
        intended_manipulated_body='pusher', intended_support_geom='transferred_pusher_rest',
        source_phase_mapping='Each source label preserved with pusher_ prefix.',
        post_initial_state_writes='Only ctrl; no qpos/qvel/object-pose/free-force writes.',
        nominal_support_top_z_m=.040, initial_settle_s=a.initial_settle,
        simulated_inertia_difference_retained=predeclared['differences'],
        mechanical_gates=dict(maximum_point_or_body_pair_resultant_n=8., maximum_penetration_mm=.2),
        hardware_commands=0, full_planter_success=False, physical_success=False)
    save(a.out/'invocation.json', invocation)
    sim = None
    result = dict(status='INCOMPLETE', completed_command_sequence=False, primitive_success=False,
                  physics_replay_verified=False, independent_retention_release_audit='pending',
                  current_perception_used=False, hardware_commands=0, physical_success=False,
                  full_planter_success=False)
    try:
        sim = StationSimulation(model, a.out, config=configuration,
            initial_joint_positions=initial, manipulated_object='pusher', render=not a.no_render)
        indices = sim.control_indices['right']
        command = sim.data.ctrl.copy()
        command[indices] = first['ctrl']
        checked_command(model, command)
        sim.commands.append(dict(phase='pusher_initial_settle', time_s=0., ctrl=command.tolist(),
                                 duration_s=a.initial_settle))
        sim.data.ctrl[:] = command
        for _ in range(round(a.initial_settle / model.opt.timestep)):
            sim.step('pusher_initial_settle')
        previous = first
        phase = None
        for source_index, row in enumerate(rows, start=1):
            dt = row['time_s'] - previous['time_s']
            if not np.isclose(dt, model.opt.timestep, rtol=0, atol=1e-9):
                raise ValueError(f'Source control timestep differs at row{source_index}: {dt}')
            command = sim.data.ctrl.copy()
            command[indices] = row['ctrl']
            checked_command(model, command)
            label = 'pusher_'+row['phase']
            if label != phase:
                if phase:
                    sim.capture(phase+' complete; audit pending')
                print(json.dumps(dict(phase_started=label, time_s=float(sim.data.time),
                                      source_row=source_index)), flush=True)
                phase = label
            sim.commands.append(dict(phase=label, time_s=float(sim.data.time),
                ctrl=command.tolist(), duration_s=float(model.opt.timestep),
                source_row=source_index, source_time_s=row['time_s']))
            sim.data.ctrl[:] = command
            sim.step(label)
            previous = row
        result.update(status='COMMAND_SEQUENCE_ENDED_AUDIT_PENDING', completed_command_sequence=True)
    except Exception as error:
        result.update(status='STOPPED', stop_reason=str(error))
        (a.out/'error.txt').write_text(traceback.format_exc())
        print(traceback.format_exc(), flush=True)
    finally:
        if sim is not None:
            sim.save(final_label='STOPPED: '+result['stop_reason'] if result['status']=='STOPPED'
                     else 'Transferred command sequence ended; independent audit pending')
    if sim is not None:
        result.update(physics_steps=sim.step_index, max_force_n=sim.max_force,
            max_penetration_mm=sim.max_penetration*1000, final_time_s=float(sim.data.time))
    result['source_inputs_unchanged'] = all(sha(path)==digest for path,digest in hashes.items())
    save(a.out/'result.json', result)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
