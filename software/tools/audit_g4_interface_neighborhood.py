#!/usr/bin/env python3
"""Static source-contact checks near the repaired G4 guide/holder interface.

This is privileged geometry inspection, not an assembly or grasp controller.
No integration, actuator command, source-CAD modification, or fit widening occurs.
"""
from __future__ import annotations

import argparse
import copy
import csv
import gzip
import importlib.metadata
import json
from pathlib import Path
import shutil
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import manifold3d as M
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from planter import g4_collision_assets as assets
from planter.g4_carrier import sha, solid


def declared_poses():
    # Translation is millimetres; extrinsic xyz Euler angles are degrees.
    values = [
        ('center_z025', (0, 0, .25), (0, 0, 0)),
        ('center_z200', (0, 0, 2), (0, 0, 0)),
        ('yp010_z050', (0, .1, .5), (0, 0, 0)),
        ('ym010_z050', (0, -.1, .5), (0, 0, 0)),
        ('yp020_z050', (0, .2, .5), (0, 0, 0)),
        ('ym020_z050', (0, -.2, .5), (0, 0, 0)),
        ('yp020_z200', (0, .2, 2), (0, 0, 0)),
        ('ym020_z200', (0, -.2, 2), (0, 0, 0)),
        ('xp020_z050', (.2, 0, .5), (0, 0, 0)),
        ('xm020_z050', (-.2, 0, .5), (0, 0, 0)),
        ('rollp050_z050', (0, 0, .5), (.5, 0, 0)),
        ('rollm050_z050', (0, 0, .5), (-.5, 0, 0)),
        ('pitchp050_z050', (0, 0, .5), (0, .5, 0)),
        ('pitchm050_z050', (0, 0, .5), (0, -.5, 0)),
        ('yp010_rollp050_z150', (0, .1, 1.5), (.5, 0, 0)),
        ('ym010_rollm050_z150', (0, -.1, 1.5), (-.5, 0, 0)),
        ('yawp050_z150', (0, 0, 1.5), (0, 0, .5)),
        ('yawm050_z150', (0, 0, 1.5), (0, 0, -.5)),
    ]
    return [dict(id=name, translation_mm=xyz, euler_xyz_deg=angles)
            for name, xyz, angles in values]


def isolated_scene(source_path, output_path):
    """Preserve all interface colliders/options; fix holder, free guide only."""
    root = ET.parse(source_path).getroot()
    result = ET.Element('mujoco', model='G4_static_interface_neighborhood')
    for tag in ('compiler', 'default', 'size', 'option', 'visual'):
        element = root.find(tag)
        if element is not None:
            result.append(copy.deepcopy(element))
    asset, world = ET.SubElement(result, 'asset'), ET.SubElement(result, 'worldbody')
    needed = set()
    for name in ('holder', 'guide'):
        body = copy.deepcopy(root.find(f"./worldbody/body[@name='{name}']"))
        body.set('pos', '0 0 0')
        body.set('quat', '1 0 0 0')
        if name == 'holder':
            body.remove(body.find('freejoint'))
        # Decorative tag bodies contain no colliders and are immaterial here.
        for child in list(body.findall('body')):
            body.remove(child)
        world.append(body)
        needed.update(g.get('mesh') for g in body.iter('geom') if g.get('mesh'))
    for mesh in root.findall('./asset/mesh'):
        if mesh.get('name') in needed:
            asset.append(copy.deepcopy(mesh))
    ET.indent(result)
    ET.ElementTree(result).write(output_path, encoding='unicode')


def check_summary(checks):
    return dict(
        endpoint_checks=len(checks),
        max_abs_source_distance_mm=max((abs(c['signed_source_distance_mm']) for c in checks), default=None),
        max_cone_residual=max((c['normal_cone_residual'] for c in checks), default=None),
        outward_2um_inside_count=sum(c['outward_probe_2um_inside_source'] for c in checks),
        source_distance_over_1um_count=sum(abs(c['signed_source_distance_mm']) > .001 for c in checks),
        cone_residual_over_001_count=sum(c['normal_cone_residual'] > .01 for c in checks),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--station-scene', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    snapshot = args.out/'source'
    (snapshot/'tools').mkdir(parents=True)
    (snapshot/'planter').mkdir()
    sources = [Path(__file__).resolve(), Path(assets.__file__).resolve(),
               Path(assets.__file__).resolve().with_name('g4_carrier.py')]
    source_hashes = {str(p): sha(p) for p in sources}
    for path in sources:
        shutil.copy2(path, snapshot/('tools' if path.name == Path(__file__).name else 'planter')/path.name)
    (snapshot/'planter/__init__.py').write_text('')
    bundle = assets.load_bundle(args.manifest)
    poses = declared_poses()
    predeclared = dict(
        scope='Static privileged contact-model sensitivity only; no time integration, robot policy, assembly or physical success',
        poses=poses, units='source CAD millimetres; world physics metres',
        holder='Fixed at original CAD origin; guide is the sole free body',
        initial_velocities='Zero independently for each pose',
        force_interpretation='mj_forward constraint response to reset overlaps and gravity; not a physical measured insertion force',
        source_endpoint_scope='All contacts with normal force >1e-5N, at most1000contacts per pose; coverage recorded',
        review_flags='Outward2um probe inside original solid, absolute source distance>1um, or local normal-cone residual>.01; numerical review flags, not task acceptance gates',
        source_hashes=source_hashes,
        manifest=str(args.manifest.resolve()), manifest_sha256=sha(args.manifest),
        station_scene=str(args.station_scene.resolve()), station_scene_sha256=sha(args.station_scene),
        original_source_sha256={k:sha(v['visual_stl']) for k,v in bundle['parts'].items()},
        versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy','trimesh','manifold3d')},
    )
    (args.out/'predeclared.json').write_text(json.dumps(predeclared, indent=2)+'\n')
    isolated_scene(args.station_scene, args.out/'scene.xml')
    model = mujoco.MjModel.from_xml_path(str(args.out/'scene.xml'))
    original = {n:solid(trimesh.load_mesh(bundle['parts'][n]['visual_stl'])) for n in ('holder','guide')}
    rows = []
    for pose in poses:
        data = mujoco.MjData(model)
        rotation = Rotation.from_euler('xyz', pose['euler_xyz_deg'], degrees=True)
        quat = rotation.as_quat()
        data.qpos[:3] = np.array(pose['translation_mm'])/1000
        data.qpos[3:7] = quat[[3,0,1,2]]
        mujoco.mj_forward(model, data)
        raw = assets.resting_sample(model, data)
        if any(w.number for w in data.warning):
            raise RuntimeError(f"MuJoCo warning at {pose['id']}")
        transform = np.c_[rotation.as_matrix(), pose['translation_mm']]
        overlap = original['holder'] ^ original['guide'].transform(transform)
        if overlap.status() != M.Error.NoError:
            raise RuntimeError(f"Source intersection Boolean failed at {pose['id']}")
        checks = assets.audit_loaded_source_contacts(bundle, raw, max_contacts=1000)
        loaded = sum(c['wrench_local'][0] > 1e-5 for c in raw['contacts'])
        normal = sum(max(0., c['wrench_local'][0]) for c in raw['contacts'])
        resultant = sum(float(np.linalg.norm(c['wrench_local'][:3])) for c in raw['contacts'])
        row = dict(**pose, original_source_overlap_mm3=float(overlap.volume()),
                   contact_count=len(raw['contacts']), loaded_contact_count=loaded,
                   summed_normal_n=normal, summed_resultant_n=resultant,
                   max_single_normal_n=max((c['wrench_local'][0] for c in raw['contacts']), default=0.),
                   max_penetration_mm=max((max(0., -c['distance_mm']) for c in raw['contacts']), default=0.),
                   guide_net_force_n=raw['totals'].get('guide', {}).get('net_force_n', [0.,0.,0.]),
                   coverage_complete=len(checks) == loaded*2,
                   **check_summary(checks))
        row['review_required'] = (row['outward_2um_inside_count'] > 0 or
                                  row['source_distance_over_1um_count'] > 0 or
                                  row['cone_residual_over_001_count'] > 0 or not row['coverage_complete'])
        with gzip.open(args.out/f"{pose['id']}.json.gz", 'wt') as handle:
            json.dump(dict(summary=row, raw_state=raw, source_endpoint_checks=checks), handle)
        rows.append(row)
        print(json.dumps(row), flush=True)
    result = dict(scope=predeclared['scope'], status='complete', cases=rows,
                  pose_count=len(rows), review_pose_count=sum(r['review_required'] for r in rows),
                  complete_loaded_contact_coverage=all(r['coverage_complete'] for r in rows),
                  scene_sha256=sha(args.out/'scene.xml'), source_hashes=source_hashes,
                  source_unchanged=all(sha(p)==value for p,value in source_hashes.items()),
                  options={k:getattr(model.opt,k) for k in ['timestep','iterations','noslip_iterations','ccd_iterations','ccd_tolerance','enableflags','disableflags','solver','integrator','cone','impratio']},
                  runtime_s=time.monotonic()-started)
    (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    with (args.out/'cases.csv').open('w') as handle:
        writer=csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases','source_hashes')}),flush=True)


if __name__ == '__main__':
    main()
