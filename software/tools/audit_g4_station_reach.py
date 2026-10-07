#!/usr/bin/env python3
"""Bounded privileged CAD/IK staging inventory, never a robot grasp or trajectory."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_station_module(path):
    spec = importlib.util.spec_from_file_location('reach_frozen_station', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def line_hits(mesh, point, direction):
    """Intersect the actual source triangles with an infinite grip-centre line."""
    tri = mesh.triangles
    u, v = tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]
    n = np.cross(u, v)
    den = (n*direction).sum(axis=1)
    keep = abs(den) > 1e-10
    tri, u, v, n, den = tri[keep], u[keep], v[keep], n[keep], den[keep]
    t = ((tri[:, 0]-point)*n).sum(axis=1)/den
    w = point+t[:, None]*direction-tri[:, 0]
    uu, uv, vv = (u*u).sum(axis=1), (u*v).sum(axis=1), (v*v).sum(axis=1)
    wu, wv = (w*u).sum(axis=1), (w*v).sum(axis=1)
    det = uu*vv-uv*uv
    aa, bb = (wu*vv-wv*uv)/det, (wv*uu-wu*uv)/det
    hits = np.sort(t[(aa >= -1e-7) & (bb >= -1e-7) & (aa+bb <= 1+1e-7)])
    unique = []
    for value in hits:
        if not unique or abs(value-unique[-1]) > 1e-5: unique.append(float(value))
    return unique


def proposed_xml(original, out):
    tree = ET.parse(original)
    root, world = tree.getroot(), tree.getroot().find('worldbody')
    changes = {}
    for i, x in enumerate((-.235, -.165, .165, .235)):
        changes[f'carrier{i}'] = [x, -.125, .0206]
    changes['pusher'] = [.320, -.075, .040]
    changes['roller'] = [-.320, -.075, .048]
    for name, position in changes.items():
        body = world.find(f"body[@name='{name}']")
        if body is None: raise ValueError(f'Missing full-station body {name}')
        body.set('pos', ' '.join(map(str, position)))
    for geom in list(world.findall('geom')):
        if geom.get('name') == 'roller_handle_staging_pad': world.remove(geom)
    supports = [
        dict(name='hypothesis_pusher_40mm_rest', center_m=[.305, -.075, .020], halfsize_m=[.030, .055, .020],
             reason='40mm rest with near edge15mm past pusher grip point; support blade/crossbar, expose underside of handle'),
        dict(name='hypothesis_roller_wheel_rest', center_m=[-.312, -.075, .020], halfsize_m=[.004, .015, .020],
             reason='Raise wheel bottom40mm; assumed human axle assembly remains'),
        dict(name='hypothesis_roller_handle_rest', center_m=[-.290, -.075, .0215], halfsize_m=[.002, .004, .0215],
             reason='Support handle sourceZ30mm, leaving grip atZ48mm overhanging by18mm'),
    ]
    for item in supports:
        ET.SubElement(world, 'geom', name=item['name'], type='box',
                      pos=' '.join(map(str, item['center_m'])), size=' '.join(map(str, item['halfsize_m'])),
                      friction='.6 .003 .0001', rgba='.34 .34 .34 1')
    ET.indent(tree)
    tree.write(out, encoding='unicode')
    return changes, supports


def body_point(model, data, name, local_mm):
    body = data.body(name)
    return body.xpos+body.xmat.reshape(3, 3)@(np.array(local_mm)/1000)


def arm_contacts(model, data, side):
    rows = []
    for c in data.contact:
        names = [model.geom(int(g)).name for g in (c.geom1, c.geom2)]
        if not any(n.startswith(side+'_') for n in names): continue
        if c.dist < -.0001:
            rows.append(dict(geoms=names, penetration_mm=float(-1000*c.dist)))
    return sorted(rows, key=lambda x: x['penetration_mm'], reverse=True)


def table_clearance(model, data, side):
    table = model.geom('table').id
    values = []
    for gid in range(model.ngeom):
        name = model.geom(gid).name
        if not name.startswith(side+'_') or not model.geom_contype[gid]: continue
        distance = mujoco.mj_geomDistance(model, data, gid, table, .5, None)
        values.append((float(distance*1000), name))
    distance, geom = min(values)
    return dict(minimum_distance_mm=distance, arm_geom=geom, method='Native MuJoCo collider distance; source-jaw/model approximation applies')


def specs():
    values = [dict(name=f'carrier{i}', body=f'carrier{i}', source='carrier', local_mm=[0, 0, -5],
                   local_axis=[1, 0, 0], side='left' if i < 2 else 'right', axis=[-1 if i < 2 else 1, 0, 0],
                   grasp_note='Pinch7.1mm outer body across two hollow plastic walls; wall deformation and retention unvalidated') for i in range(4)]
    values += [dict(name='pusher', body='pusher', source='pusher', local_mm=[-60, 0, 4], local_axis=[0, 0, 1],
                    side='right', axis=[0, 0, 1], grasp_note='Previously used local grip and vertical closing axis; full station transfer of40mm fixture remains unvalidated'),
               dict(name='roller', body='roller', source='roller', local_mm=[0, 0, 48], local_axis=[1, 0, 0],
                    side='left', axis=[0, 0, 1], grasp_note='10mm handle thickness; lying handle needs underside access and fixture, jaw retention unvalidated')]
    for name, y, z in [('holder', -40.75, 2.5), ('trough', -41.5, -12.)]:
        for side, sign in [('left', -1), ('right', 1)]:
            values.append(dict(name=f'{name}_{side}_rim', body=name, source=name, local_mm=[sign*55, y, z],
                               local_axis=[0, 1, 0], side=side, axis=[0, 1, 0],
                               grasp_note='Thin front wall/rim only; provisional dual-arm point. Current holder is preassembled, so robot receives no placement credit.'))
    return values


def evaluate_variant(name, model, station, initial, inventory, out):
    kin = station.StationKinematics(model)
    parked = mujoco.MjData(model)
    for side in station.SIDES:
        parked.qpos[kin.indices[side]] = initial['qpos'][kin.indices[side]]
    mujoco.mj_fwdPosition(model, parked)
    origin = parked.qpos.copy()
    cases = []
    for spec in inventory:
        point = body_point(model, parked, spec['body'], spec['local_mm'])
        base = parked.body(spec['side']+'_base_link').xpos.copy()
        for phase, offset in [('approach50mm', .050), ('grasp_open', 0.)]:
            target = point+[0, 0, offset]
            kin.seeds[spec['side']] = np.radians([0, 50, -30, -20, 0])
            q, errors = kin.ik(spec['side'], target, spec['axis'], strict=False)
            feasible = errors['position_error_mm'] <= 1.5 and errors['axis_error_scaled'] <= .004
            data = mujoco.MjData(model)
            data.qpos[:] = origin
            data.qpos[kin.indices[spec['side']]] = np.r_[q, .55]
            mujoco.mj_fwdPosition(model, data)
            ranges = kin.ranges[spec['side']]
            margin = np.minimum(q-ranges[:, 0], ranges[:, 1]-q)
            axis_cos = np.clip(np.dot(np.array(errors['rotation_world'])[:, 0], spec['axis']), -1, 1)
            result = dict(variant=name, name=spec['name'], phase=phase, side=spec['side'], target_m=target.tolist(),
                          base_to_target_m=float(np.linalg.norm(target-base)), position_error_mm=errors['position_error_mm'],
                          axis_error_deg=float(np.degrees(np.arccos(axis_cos))), joint_angles_rad=q.tolist(),
                          minimum_joint_limit_margin_deg=float(np.degrees(margin.min())),
                          pose_found_within_declared_IK_tolerances=bool(feasible),
                          contacts_over_0p1mm=arm_contacts(model, data, spec['side']), table_clearance=table_clearance(model, data, spec['side']),
                          limitation='Independent open-jaw endpoint only; no closing, contact retention, path clearance or coordinated dual-arm motion demonstrated')
            if not feasible:
                qpoint, epoint = kin.ik(spec['side'], target, None, strict=False)
                result['position_only_IK_error_mm'] = epoint['position_error_mm']
                result['position_only_joint_angles_rad'] = qpoint.tolist()
            cases.append(result)
            print(json.dumps({k:result[k] for k in ['variant', 'name', 'phase', 'position_error_mm', 'axis_error_deg', 'pose_found_within_declared_IK_tolerances']}), flush=True)
    renderer = mujoco.Renderer(model, 720, 960)
    option = mujoco.MjvOption(); option.geomgroup[3] = 0
    camera = mujoco.MjvCamera(); camera.lookat[:] = [0, -.065, .025]; camera.distance=.82; camera.azimuth=90; camera.elevation=-75
    renderer.update_scene(parked, camera=camera, scene_option=option)
    from PIL import Image, ImageDraw
    im=Image.fromarray(renderer.render().copy()); renderer.close()
    ImageDraw.Draw(im).text((18,18), f'{name}: privileged staging hypothesis, no robot motion', fill='white', stroke_width=2, stroke_fill='black')
    im.save(out/f'layout-{name}.png')
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--initial', type=Path, required=True)
    parser.add_argument('--station-module', type=Path, required=True)
    parser.add_argument('--cad', type=Path, required=True)
    parser.add_argument('--holder', type=Path, required=True)
    parser.add_argument('--trough', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(args.station_module.parent.parent))
    station = load_station_module(args.station_module)
    source_paths = dict(carrier=args.cad/'carrier_PROTOTYPE.stl', pusher=args.cad/'feeder_print_PROTOTYPE.stl',
                        roller=args.cad/'roller_handle_PROTOTYPE.stl', holder=args.holder, trough=args.trough)
    inventory = specs()
    for item in inventory:
        mesh = trimesh.load_mesh(source_paths[item['source']])
        item['source_path'] = str(source_paths[item['source']])
        item['source_sha256'] = sha(source_paths[item['source']])
        item['source_bounds_mm'] = mesh.bounds.tolist()
        item['actual_source_grip_line_intersections_mm_from_center'] = line_hits(mesh, np.array(item['local_mm']), np.array(item['local_axis']))
    shutil.copy2(args.scene, args.out/'current-scene.xml')
    changes, supports = proposed_xml(args.scene, args.out/'proposed-scene.xml')
    initial = np.load(args.initial)
    inputs = {str(p.resolve()): sha(p) for p in [args.scene, args.initial, args.station_module, Path(__file__), *source_paths.values()]}
    (args.out/'predeclared.json').write_text(json.dumps(dict(inputs=inputs,argv=sys.argv,variants=['current','proposed'],
            proposed_positions_m=changes, proposed_supports=supports,grip_inventory=inventory,approach_offset_m=.05,
            jaw_angle_rad=.55,IK_tolerances=dict(position_mm=1.5,scaled_axis_error=.004),
            scope='Privileged static CAD+IK inventory only; no hardware, actuation, task success, grasp validation or swept-path proof'),indent=2)+'\n')
    all_cases, ranges = [], {}
    for variant in ['current', 'proposed']:
        model = mujoco.MjModel.from_xml_path(str(args.out/f'{variant}-scene.xml'))
        ranges[variant] = {s:{j:model.joint(s+'_'+j).range.tolist() for j in station.JOINTS} for s in station.SIDES}
        all_cases.extend(evaluate_variant(variant, model, station, initial, inventory, args.out))
        (args.out/'partial.json').write_text(json.dumps(all_cases,indent=2)+'\n')
    assert all(sha(p)==digest for p,digest in inputs.items()), 'Inputs changed during read-only audit'
    result = dict(status='offline_inventory_complete',cases=all_cases,grip_inventory=inventory,joint_limits_rad=ranges,
                  input_hashes=inputs,proposed_positions_m=changes,proposed_supports=supports,
                  hardware_commands=0,robot_grasps_validated=0,robot_trajectories_validated=0,
                  limitations=['IK tolerances are scene diagnostics, not carrier insertion tolerances.',
                               'Endpoint collision checks use current MuJoCo covers and other arm parked; source contact fidelity and swept paths remain separate.',
                               'Raised fixtures are unmeasured layout hypotheses; stability and clearance under closing must be simulated.',
                               'Holder/trough grip points lie on thin walls; wall stress, pinch retention and source-normal accuracy are unvalidated.',
                               'Current scene initializes holder in trough; this provides no robot assembly evidence.'])
    (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    with (args.out/'inventory.csv').open('w') as file:
        writer=csv.writer(file);writer.writerow(['variant','part','phase','side','IK_position_mm','IK_axis_deg','joint_margin_deg','table_distance_mm','contacts_gt_0p1mm','pose_found'])
        for c in all_cases:writer.writerow([c['variant'],c['name'],c['phase'],c['side'],c['position_error_mm'],c['axis_error_deg'],c['minimum_joint_limit_margin_deg'],c['table_clearance']['minimum_distance_mm'],len(c['contacts_over_0p1mm']),c['pose_found_within_declared_IK_tolerances']])
    shutil.copy2(__file__,args.out/Path(__file__).name)
    print(str(args.out/'result.json'))


if __name__ == '__main__': main()
