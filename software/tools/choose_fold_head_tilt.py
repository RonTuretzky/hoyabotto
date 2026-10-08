"""Choose the head tilt whose `front` view keeps all four flaps and both claws in frame. Simulation only.

For each candidate head tilt (carton/xlerobot_cameras.py head chain, OAK-D Lite 4:3 field of view), every
recorded sample of the given trials is projected into a 320x240 `front` image: the free-edge corners of
the four flaps and both claw tips (`left_tip`/`right_tip`) plus the jaw/gripper bodies. A sample counts
as covered when all of these points are inside the image by `--margin` pixels. The tilt with the highest
coverage wins; ties go to the most centred view.

    PYTHONPATH=. python tools/choose_fold_head_tilt.py --trials .../check-220-a/trial-0* --tilts 40 70 1
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from carton.folding_station_measured import camera_pose, scene_station
from carton.xlerobot_cameras import OAK_D_LITE_FOVY_DEG, head_camera_spec

FLAPS = ('short_left', 'short_right', 'long_far', 'long_near')
GRIPPER_BODIES = ('gripper_link', 'moving_jaw_so101_v1_link')


def key_points(model, data):
    """World points that must be visible: flap free-edge corners, claw tips, gripper bodies."""
    pts = {}
    for flap in FLAPS:
        g = model.geom(flap + '_cardboard').id
        size, pos, mat = model.geom_size[g], data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3)
        ax = int(np.argmax(size))  # long in-plane axis; the free edge is +z of the flap (hinge at -z)
        for s in (-1, 1):
            local = np.zeros(3)
            local[ax] = s * size[ax]
            local[2] = size[2]
            pts[f'{flap}_edge_{s:+d}'] = pos + mat @ local
    for side in ('left', 'right'):
        pts[f'{side}_tip'] = data.site(f'{side}_tip').xpos.copy()
        for b in GRIPPER_BODIES:
            pts[f'{side}_{b}'] = data.body(f'{side}_{b}').xpos.copy()
    return pts


def project(points_arm, position, rotation_cv, fovy_deg, width=320, height=240):
    f = height / 2 / math.tan(math.radians(fovy_deg) / 2)
    cam = (points_arm - position) @ rotation_cv
    z = cam[:, 2]
    u = f * cam[:, 0] / np.where(z > 1e-6, z, np.nan) + width / 2
    v = f * cam[:, 1] / np.where(z > 1e-6, z, np.nan) + height / 2
    return u, v, z


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', type=Path, nargs='+', required=True)
    ap.add_argument('--tilts', type=float, nargs=3, default=(40., 72., 1.), help='start stop step, degrees')
    ap.add_argument('--pan', type=float, default=0.)
    ap.add_argument('--fovy', type=float, default=OAK_D_LITE_FOVY_DEG)
    ap.add_argument('--margin', type=float, default=4., help='pixels inside the 320x240 border')
    ap.add_argument('--stride', type=int, default=5, help='use every Nth 10 Hz sample')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args(argv)
    clouds = []
    for trial in args.trials:
        if not (trial / 'demo.npz').exists():
            continue
        demo = json.loads((trial / 'demo.json').read_text()) if (trial / 'demo.json').exists() else {}
        if demo and not demo.get('success'):
            continue
        model = mujoco.MjModel.from_xml_path(str(trial / 'run/scene.xml'))
        import xml.etree.ElementTree as ET
        origin = np.asarray(scene_station(ET.parse(trial / 'run/scene.xml').getroot())['arm_base_origin_world_m'])
        data = mujoco.MjData(model)
        z = np.load(trial / 'demo.npz')
        for q in z['qpos'][::args.stride]:
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            pts = key_points(model, data)
            clouds.append((list(pts), np.array(list(pts.values())) - origin))
    if not clouds:
        raise SystemExit('No successful trials with demo.npz')
    names = clouds[0][0]
    results = []
    for tilt in np.arange(args.tilts[0], args.tilts[1] + 1e-9, args.tilts[2]):
        spec = head_camera_spec(float(tilt), args.pan, args.fovy)
        position, right, up, fovy = camera_pose(spec, 'front')
        rot = np.column_stack((right, -up, np.cross(right, -up)))
        covered, worst, misses = 0, [], {}
        groups = {'flap_edges': [i for i, n in enumerate(names) if '_edge_' in n],
                  'claw_tips': [i for i, n in enumerate(names) if n.endswith('_tip')]}
        group_hits = {g: 0 for g in groups}
        visible = 0.
        for _, pts in clouds:
            u, v, depth = project(pts, position, rot, fovy)
            inside = (depth > 0) & (u >= args.margin) & (u <= 320 - args.margin) & (v >= args.margin) & (v <= 240 - args.margin)
            covered += bool(inside.all())
            visible += float(inside.mean())
            for g, idx in groups.items():
                group_hits[g] += bool(inside[idx].all())
            for n, ok in zip(names, inside):
                if not ok:
                    misses[n] = misses.get(n, 0) + 1
            worst.append(float(np.nanmax(np.abs(v - 120))))
        results.append({'tilt_deg': float(tilt), 'coverage': covered / len(clouds),
                        'mean_fraction_of_points_in_frame': visible / len(clouds),
                        **{f'{g}_all_in_frame': h / len(clouds) for g, h in group_hits.items()},
                        'median_worst_vertical_offset_px': float(np.median(worst)),
                        'most_missed': sorted(misses.items(), key=lambda kv: -kv[1])[:4],
                        'camera_position_arm_base_m': spec['position_m']})
        r = results[-1]
        print(f"tilt {tilt:5.1f}: all points {100 * r['coverage']:5.1f}%  mean points "
              f"{100 * r['mean_fraction_of_points_in_frame']:5.1f}%  flap edges {100 * r['flap_edges_all_in_frame']:5.1f}%  "
              f"claw tips {100 * r['claw_tips_all_in_frame']:5.1f}%  misses {r['most_missed']}", flush=True)
    best = max(results, key=lambda r: (round(r['coverage'], 3), r['mean_fraction_of_points_in_frame']))
    print(json.dumps({'samples': len(clouds), 'best': best}, indent=1))
    if args.out:
        args.out.write_text(json.dumps({'samples': len(clouds), 'margin_px': args.margin, 'pan_deg': args.pan,
                                        'fovy_deg': args.fovy, 'results': results, 'best': best}, indent=1))


if __name__ == '__main__':
    main()
