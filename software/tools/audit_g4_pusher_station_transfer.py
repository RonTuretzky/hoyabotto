#!/usr/bin/env python3
"""Privileged sampled geometry transfer of a recorded pusher bench trajectory.

No dynamics, perception, task success, or swept-path qualification is claimed.
The recorded arm and object states are rigidly transformed into the full station
to identify new environmental collisions before attempting joint actuation.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def words(values):
    return ' '.join(format(float(x), '.17g') for x in values)


def quaternion(matrix):
    result = np.empty(4)
    mujoco.mju_mat2Quat(result, np.asarray(matrix).ravel())
    return result


def rotated_quaternion(rotation, old_quaternion):
    result = np.empty(4)
    mujoco.mju_mulQuat(result, quaternion(rotation), np.array(old_quaternion, dtype=float))
    return result


def asset_paths(tree, scene):
    compiler = tree.find('compiler')
    meshdir = Path(compiler.get('meshdir', '.')) if compiler is not None else Path('.')
    paths = []
    for mesh in tree.findall('./asset/mesh'):
        path = Path(mesh.get('file'))
        if not path.is_absolute():
            path = scene.parent / meshdir / path
        paths.append(path.resolve())
        mesh.set('file', str(path.resolve()))
    if compiler is not None:
        compiler.attrib.pop('meshdir', None)
    return paths


def category(model, geom):
    bid = int(model.geom_bodyid[geom])
    while bid:
        name = model.body(bid).name
        if name == 'pusher':
            return 'pusher'
        if name in ('left_base_link', 'right_base_link'):
            return name.split('_')[0] + '_arm'
        bid = int(model.body_parentid[bid])
    return 'environment'


def select_samples(path, interval):
    selected, previous, next_time = {}, None, None
    total = 0
    phase_counts = {}
    with path.open() as stream:
        for index, line in enumerate(stream):
            row = json.loads(line)
            total += 1
            phase_counts[row['phase']] = phase_counts.get(row['phase'], 0) + 1
            if next_time is None:
                next_time = row['time_s']
            if previous is None or previous[1]['phase'] != row['phase']:
                if previous is not None:
                    selected[previous[0]] = previous[1]
                selected[index] = row
            if row['time_s'] >= next_time - 1e-9:
                selected[index] = row
                next_time = row['time_s'] + interval
            previous = index, row
        if previous:
            selected[previous[0]] = previous[1]
    if not selected:
        raise ValueError('Empty physics trace')
    return sorted(selected.items()), total, phase_counts


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bench-scene', type=Path, required=True)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--station-scene', type=Path, required=True)
    p.add_argument('--station-initial', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--interval', type=float, default=.050)
    p.add_argument('--no-render', action='store_true')
    args = p.parse_args()
    if args.interval <= 0:
        p.error('Positive sample interval required')
    args.out.mkdir(parents=True, exist_ok=False)
    bench_tree, station_tree = ET.parse(args.bench_scene), ET.parse(args.station_scene)
    meshes = asset_paths(bench_tree, args.bench_scene) + asset_paths(station_tree, args.station_scene)
    inputs = [args.bench_scene, args.station_scene, args.station_initial,
              args.episode/'physics.jsonl', args.episode/'commands.json', args.episode/'result.json',
              Path(__file__), *meshes]
    hashes = {str(x.resolve()): sha(x) for x in inputs}
    bench_model = mujoco.MjModel.from_xml_path(str(args.bench_scene))
    original_model = mujoco.MjModel.from_xml_path(str(args.station_scene))
    bench_data, original_data = mujoco.MjData(bench_model), mujoco.MjData(original_model)
    mujoco.mj_kinematics(bench_model, bench_data)
    mujoco.mj_kinematics(original_model, original_data)
    rotation = original_data.body('right_base_link').xmat.reshape(3, 3) @ bench_data.body('base_link').xmat.reshape(3, 3).T
    translation = original_data.body('right_base_link').xpos - rotation @ bench_data.body('base_link').xpos
    world = station_tree.find('worldbody')
    pusher = world.find("body[@name='pusher']")
    bench_pusher = bench_tree.find("./worldbody/body[@name='pusher']")
    position = rotation @ np.fromstring(bench_pusher.get('pos'), sep=' ') + translation
    pusher.set('pos', words(position))
    pusher.set('quat', words(rotated_quaternion(rotation, np.fromstring(bench_pusher.get('quat', '1 0 0 0'), sep=' '))))
    pusher.attrib.pop('euler', None)
    for element in bench_pusher:
        if element.tag == 'site' or (element.tag == 'body' and element.get('name') == 'tag31'):
            pusher.append(copy.deepcopy(element))
    rest = copy.deepcopy(bench_tree.find("./worldbody/geom[@name='staging_rest']"))
    if rest is None:
        raise ValueError('The input benchmark must declare its staging rest')
    rest.set('name', 'transferred_pusher_rest')
    rest.set('pos', words(rotation @ np.fromstring(rest.get('pos'), sep=' ') + translation))
    rest.set('quat', words(rotated_quaternion(rotation, np.fromstring(rest.get('quat', '1 0 0 0'), sep=' '))))
    rest.attrib.pop('euler', None)
    world.append(rest)
    ET.indent(station_tree)
    transformed_scene = args.out/'scene.xml'
    station_tree.write(transformed_scene, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(transformed_scene))
    data = mujoco.MjData(model)
    initial = np.load(args.station_initial)
    if data.qpos.shape != initial['qpos'].shape:
        raise ValueError('Transfer unexpectedly changed generalized coordinates')
    initial_positions = initial['qpos'].copy()
    arm_indices = [int(model.joint('right_' + name).qposadr[0]) for name in JOINTS]
    pusher_index = int(model.joint('pusher_free').qposadr[0])
    bench_indices = [int(bench_model.joint(name).qposadr[0]) for name in JOINTS]
    categories = [category(model, g) for g in range(model.ngeom)]
    table = model.geom('table').id
    right_geoms = [g for g, c in enumerate(categories) if c == 'right_arm' and model.geom_contype[g]]
    samples, total, phase_counts = select_samples(args.episode/'physics.jsonl', args.interval)
    assumptions = dict(
        scope='Privileged static sampled geometry; recorded object poses are explicitly reset. No dynamics or deployable controller.',
        rotation_world_from_bench=rotation.tolist(), translation_m=translation.tolist(),
        pusher_nominal_root_m=position.tolist(), transformed_rest=rest.attrib,
        sample_interval_s=args.interval, source_trace_rows=total,
        phase_source_counts=phase_counts, argv=sys.argv, hardware_commands=0,
        source_hashes=hashes,
        retained='Full station table, both arms, all other objects, original joint ranges, collision covers and materials.',
        differences=dict(pusher_inertial_bench=bench_pusher.find('inertial').attrib,
                         pusher_inertial_station=pusher.find('inertial').attrib,
                         table='Full station extends closer to the robot; this audit checks added collisions.',
                         tag='Pusher18mm marker31 transferred from bench as a visual mount hypothesis; no detection attempted.'),
        limitations=['Sampled endpoints cannot establish swept-path clearance.',
                     'Contact positions use current collision covers; no source-normal or force qualification here.',
                     'Other parts remain at prepared initial poses, not states produced by earlier assembly stages.',
                     'Changes in pusher inertia and surrounding scene require new actuated retention/release trials.',
                     'Fixture and marker layout are unmeasured hypotheses.'])
    (args.out/'predeclared.json').write_text(json.dumps(assumptions, indent=2)+'\n')
    rows, phase_ends = [], {}
    for sample_index, (source_index, row) in enumerate(samples):
        data.qpos[:] = initial_positions
        data.qpos[arm_indices] = row['encoder_radians']
        data.qpos[pusher_index:pusher_index+3] = rotation @ np.array(row['object_pose'][:3]) + translation
        data.qpos[pusher_index+3:pusher_index+7] = rotated_quaternion(rotation, row['object_pose'][3:])
        mujoco.mj_fwdPosition(model, data)
        bench_data.qpos[bench_indices] = row['encoder_radians']
        mujoco.mj_kinematics(bench_model, bench_data)
        expected_tip = rotation @ bench_data.site('grasp').xpos + translation
        tip_error = float(np.linalg.norm(data.site('right_tip').xpos-expected_tip)*1000)
        recorded_tip_error = float(np.linalg.norm(bench_data.site('grasp').xpos-row['grasp_world_m'])*1000)
        distances = [(float(mujoco.mj_geomDistance(model, data, g, table, .5, None)*1000), model.geom(g).name) for g in right_geoms]
        relevant, unexpected = [], []
        for contact in data.contact:
            ids = (int(contact.geom1), int(contact.geom2))
            cats = [categories[g] for g in ids]
            if not any(c in ('right_arm', 'pusher') for c in cats):
                continue
            names = [model.geom(g).name for g in ids]
            intended = (set(cats) == {'right_arm', 'pusher'} or
                        ('pusher' in cats and 'transferred_pusher_rest' in names))
            item = dict(geoms=names, categories=cats, distance_mm=float(contact.dist*1000),
                        position_m=contact.pos.tolist(), intended_bench_contact=intended)
            relevant.append(item)
            if not intended and contact.dist < -1e-6:
                unexpected.append(item)
        output = dict(source_row_index=source_index, phase=row['phase'], time_s=row['time_s'],
                      encoder_radians=row['encoder_radians'], object_pose=data.qpos[pusher_index:pusher_index+7].tolist(),
                      arm_tip_world_m=data.site('right_tip').xpos.tolist(),
                      transformed_FK_error_mm=tip_error, recorded_FK_error_mm=recorded_tip_error,
                      minimum_arm_table_distance_mm=min(distances)[0], closest_arm_table_geom=min(distances)[1],
                      relevant_contacts=relevant, unexpected_contacts_over_1um=unexpected)
        rows.append(output)
        phase_ends[row['phase']] = sample_index
    (args.out/'samples.json').write_text(json.dumps(rows, indent=2)+'\n')
    image_paths = []
    if not args.no_render:
        from PIL import Image, ImageDraw
        renderer = mujoco.Renderer(model, 720, 960)
        options = mujoco.MjvOption(); options.geomgroup[3] = 0
        for phase, index in phase_ends.items():
            sample = rows[index]
            data.qpos[:] = initial_positions
            data.qpos[arm_indices] = sample['encoder_radians']
            data.qpos[pusher_index:pusher_index+7] = sample['object_pose']
            mujoco.mj_fwdPosition(model, data)
            renderer.update_scene(data, camera='overview', scene_option=options)
            picture = Image.fromarray(renderer.render().copy())
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 960, 48), fill='white')
            draw.text((10, 7), 'PRIVILEGED STATIC TRANSFER | '+phase+' | NO ACTUATED TRAJECTORY', fill='black')
            draw.text((10, 26), 'Recorded robot/object poses reset. Sampled geometry only; no task success.', fill='black')
            image_path = args.out / ('pose-'+phase+'.png')
            picture.save(image_path); image_paths.append(str(image_path))
        renderer.close()
    differences = {path:dict(before=digest, after=sha(path)) for path,digest in hashes.items() if sha(path) != digest}
    result = dict(status='sampled_static_audit_complete', full_planter_success=False,
                  robot_trajectories_validated=0, physical_success=False, hardware_commands=0,
                  selected_samples=len(rows), source_trace_rows=total, phases=list(phase_ends),
                  maximum_transformed_FK_error_mm=max(r['transformed_FK_error_mm'] for r in rows),
                  maximum_recorded_FK_error_mm=max(r['recorded_FK_error_mm'] for r in rows),
                  minimum_arm_table_distance_mm=min(r['minimum_arm_table_distance_mm'] for r in rows),
                  samples_with_unexpected_penetration_over_1um=sum(bool(r['unexpected_contacts_over_1um']) for r in rows),
                  maximum_unexpected_penetration_mm=max([0]+[-c['distance_mm'] for r in rows for c in r['unexpected_contacts_over_1um']]),
                  source_inputs_unchanged=not differences, source_changes=differences,
                  source_hashes=hashes, transformed_scene_sha256=sha(transformed_scene),
                  images=image_paths, assumptions=assumptions)
    (args.out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    with (args.out/'summary.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_row','phase','time_s','table_distance_mm','unexpected_contacts_gt1um','FK_error_mm'])
        for row in rows:
            writer.writerow([row['source_row_index'],row['phase'],row['time_s'],row['minimum_arm_table_distance_mm'],len(row['unexpected_contacts_over_1um']),row['transformed_FK_error_mm']])
    shutil.copy2(__file__, args.out/Path(__file__).name)
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_hashes','assumptions','images')}, indent=2))
    if differences:
        raise RuntimeError('Input files changed during audit')


if __name__ == '__main__':
    main()
