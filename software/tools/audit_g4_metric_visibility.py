"""Metric production-tag coverage for explicit G4 pose/camera hypotheses.

Pose resets here are privileged offline diagnostics, not executed trajectories,
stationary encoder samples, a hand-eye fit, or usable robot registrations.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
from pathlib import Path
import uuid
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from farm.perception.gemma_tags import TagObserver
from planter.g4_station import JOINTS
from planter.g4_station_vision import capture, geometry_for_cameras


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['scene', 'state', 'poses', 'out']:
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--camera', default='station'); p.add_argument('--seed', type=int, default=791)
    a = p.parse_args(); a.out.mkdir(parents=True, exist_ok=False)
    source = {str(x.resolve()): sha(x) for x in [a.scene, a.state, a.poses, Path(__file__),
        Path(__file__).resolve().parents[1]/'planter/g4_station_vision.py',
        *[Path(__file__).resolve().parents[1]/('farm/perception/'+f) for f in ['gemma_tags.py', 'tags.py', 'tag_geometry.py']]]}
    source.update({m.get('file'): sha(m.get('file')) for m in E.parse(a.scene).findall('./asset/mesh')})
    (a.out/'sources.json').write_text(json.dumps(source, indent=2)+'\n')
    model = mujoco.MjModel.from_xml_path(str(a.scene)); data = mujoco.MjData(model)
    initial = np.load(a.state); poses = json.loads(a.poses.read_text())
    renderer = mujoco.Renderer(model, width=model.vis.global_.offwidth, height=model.vis.global_.offheight)
    opt = mujoco.MjvOption(); opt.geomgroup[3] = 0
    geometry = geometry_for_cameras([a.camera]); rng = np.random.default_rng(a.seed)
    stream = 'g4-metric-visibility-'+uuid.uuid4().hex; rows = []
    for i, pose in enumerate(poses):
        data.qpos[:] = initial['qpos']; data.qvel[:] = 0; data.ctrl[:] = initial['ctrl']
        for side, angles in pose.get('joint_angles_rad', {}).items():
            if side not in ['left', 'right'] or len(angles) != len(JOINTS):
                raise ValueError('Explicit six original joint angles required for each selected arm')
            for joint, angle in zip(JOINTS, angles):
                j = model.joint(side+'_'+joint)
                if not np.isfinite(angle) or not j.range[0] <= angle <= j.range[1]:
                    raise ValueError('Pose hypothesis outside original joint limits')
                data.qpos[j.qposadr[0]] = angle
                data.ctrl[model.actuator(side+'_'+joint).id] = angle
        mujoco.mj_forward(model, data)
        prefix = a.out/f'pose-{i:02d}'
        intersections = [dict(geoms=[model.geom(int(g)).name for g in c.geom], penetration_mm=-float(c.dist)*1000)
                         for c in data.contact if c.dist < -.0001]
        payload, depth, binding = capture(renderer, model, data, a.camera, option=opt,
            stream_id=stream, seq=i+1, timestamp=1000.+i, rng=rng)
        prefix.with_suffix('.png').write_bytes(base64.b64decode(payload['images'][0]['data_base64']))
        observer = TagObserver(clock=lambda:1000.+i, geometry=geometry)
        result = observer.observe(payload, [a.camera], ids=[1, 2, 4, 41], include_images=False)
        # Keep a real aligned depth frame and explicit RGB binding for the baseline.
        if i == 0:
            np.savez_compressed(a.out/'baseline-depth.npz', aligned_depth_m=depth)
            (a.out/'baseline-depth-binding.json').write_text(json.dumps(binding, indent=2)+'\n')
            controls = {}
            stale = copy.deepcopy(payload); stale['images'][0]['captured_at'] -= 5
            stale['result']['cameras'][a.camera]['rgb_captured_at'] -= 5
            wrong_frame = copy.deepcopy(payload); wrong_frame['result']['cameras'][a.camera]['seq'] += 1
            for name, bad in [('stale', stale), ('mismatched_metric_frame', wrong_frame)]:
                controls[name] = TagObserver(clock=lambda:1000.+i, geometry=geometry).observe(
                    bad, [a.camera], ids=[1, 2, 4, 41], include_images=False)
            (a.out/'controls.json').write_text(json.dumps(controls, indent=2)+'\n')
        row = dict(pose=pose, initial_intersections_over_point1_mm=intersections, observation=result)
        rows.append(row); prefix.with_suffix('.json').write_text(json.dumps(row, indent=2)+'\n')
        metric = result['result']['observations'][a.camera].get('pose_3d') or {}
        print(json.dumps(dict(pose=i, name=pose['name'], intersections=len(intersections),
            metric=[(t['tag_id'], t['status']) for t in metric.get('tags', [])])), flush=True)
    renderer.close()
    assert all(sha(path)==value for path, value in source.items()), 'Inputs changed during diagnostic'
    report = dict(scope=__doc__, scene=str(a.scene), camera=a.camera, poses=rows,
        geometry=geometry, seed=a.seed, source_hashes='sources.json',
        hardware_commands=0, joint_actuation=False, hand_eye_registration=False,
        full_planter_success=False, physical_success=False,
        artifact_hashes={x.name:sha(x) for x in a.out.iterdir() if x.is_file()})
    (a.out/'result.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__': main()
