"""Hold-settle comparison of guide/holder colliders from the recorded contact09 stop state.

A single forward from a state recorded under another collider overstates soft
contact loads (more contacts at the same recorded depth). This steps physics
from that state with the recorded controls held, under each collider, and
applies the station's own per-step gates (8 N per contact or body pair, 0.2 mm
penetration, no MuJoCo warnings). At the end the guide/holder overlap is
measured independently from the original STLs. Simulation only: no hardware,
no trajectory, no placement or release credit.
"""
from __future__ import annotations
import argparse
import json
import resource
import sys
import time
import xml.etree.ElementTree as E
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from planter import g4_exact_sdf as X  # noqa: E402
import audit_g4_exact_sdf as A  # noqa: E402
import audit_g4_sdf_pairing as P  # noqa: E402

VARIANTS = {
    'sdf_sdf': ('sdf_sdf', False),
    'both_mesh_sdf_pairs': ('both_mesh_sdf_pairs', False),
    'both_mesh_exact_collider': ('both_mesh_exact_collider', True),
}
GATE_FORCE_N, GATE_PENETRATION_MM = 8., .2


def loads(model, data):
    """Station gate quantities: per-contact and per-body-pair resultants, depth."""
    point, pairs, depth, named = 0., {}, 0., {}
    for i in range(data.ncon):
        c = data.contact[i]
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        force = float(np.linalg.norm(wrench[:3]))
        bodies = tuple(sorted(int(model.geom_bodyid[g]) for g in (c.geom1, c.geom2)))
        pairs[bodies] = pairs.get(bodies, 0.)+force
        point = max(point, force)
        depth = max(depth, -float(c.dist)*1000)
    for key, value in pairs.items():
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in key]
        if {'guide', 'holder'} & set(names):
            named[' / '.join(names)] = value
    worst = max(pairs.items(), key=lambda kv: kv[1], default=((), 0.))
    return dict(max_point_n=point, max_pair_n=worst[1],
                max_pair=' / '.join(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in worst[0]),
                max_penetration_mm=depth, ncon=int(data.ncon), guide_holder_pairs_n=named)


def settle(model, data, state, steps):
    mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    guide = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'guide')
    start = data.xpos[guide].copy()
    trace, failure, t = [], None, time.monotonic()
    first = loads(model, data)
    for step in range(1, steps+1):
        mujoco.mj_step(model, data)
        warnings = [int(w.number) for w in data.warning]
        row = loads(model, data)
        row.update(step=step, time_s=float(data.time), guide_drift_mm=float(np.linalg.norm(data.xpos[guide]-start)*1000))
        trace.append(row)
        if any(warnings):
            failure = f'MuJoCo warning {warnings}'
        elif max(row['max_point_n'], row['max_pair_n']) > GATE_FORCE_N:
            failure = f'8 N gate: {max(row["max_point_n"], row["max_pair_n"]):.3f} N on {row["max_pair"]}'
        elif row['max_penetration_mm'] > GATE_PENETRATION_MM:
            failure = f'0.2 mm gate: {row["max_penetration_mm"]:.4f} mm'
        elif not np.isfinite(data.qpos).all():
            failure = 'nonfinite state'
        if failure:
            break
    return dict(first_forward=first, trace=trace, failure=failure, steps_completed=len(trace), wall_s=time.monotonic()-t)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--native-scene', type=Path, default=A.OUTPUT/'g4-native-sdf-01/depth6/scene.xml')
    p.add_argument('--holder-stl', type=Path, default=A.OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/cressmaster-holder.stl')
    p.add_argument('--guide-stl', type=Path, default=A.OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/guide_PROTOTYPE.stl')
    p.add_argument('--state', type=Path, default=A.OUTPUT/'g4-assembly-next/scoring-station-contact09-stop-row.json')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=.3)
    p.add_argument('--fw-starts', type=int, default=4, help='Frank-Wolfe starts per face for the exact collider')
    p.add_argument('--cpu-limit-s', type=int, default=3600)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    resource.setrlimit(resource.RLIMIT_CPU, (a.cpu_limit_s, a.cpu_limit_s))
    sources = dict(holder=a.holder_stl, guide=a.guide_stl)
    state = json.loads(a.state.read_text())
    manifest = X.build(a.out/'plugin')
    handle = X.load(manifest['library'])
    base = A.write_scene(a.native_scene, a.out/'scene-base.xml', sources, with_files=True)
    scenes = {}
    for name, (layout, exact) in VARIANTS.items():
        scene = P.variant_scene(base, a.out/f'scene-{name}.xml', layout)
        if exact:
            tree = E.parse(scene)
            for inst in tree.getroot().iter('instance'):
                E.SubElement(inst, 'config', key='starts', value=str(a.fw_starts))
            tree.write(scene, encoding='unicode')
        scenes[name] = scene
    inputs = {str(k): X.sha(k) for k in [Path(__file__), A.__file__, P.__file__, X.SOURCE, manifest['library'], a.native_scene, a.state, *sources.values(), *scenes.values()]}
    pre = dict(scope='Hold-settle from the recorded contact09 stop state under each collider; station gates unchanged; no trajectory, hardware or task credit',
               variants={k: dict(layout=v[0], exact_collider=v[1]) for k, v in VARIANTS.items()}, seconds=a.seconds, fw_starts=a.fw_starts,
               gates=dict(force_n=GATE_FORCE_N, penetration_mm=GATE_PENETRATION_MM, warnings='none'), inputs=inputs, promotion_allowed=False)
    (a.out/'predeclared.json').write_text(json.dumps(pre, indent=2)+'\n')
    rng = np.random.default_rng(20261007)
    results = {}
    for name, (layout, exact) in VARIANTS.items():
        X.set_collider(handle, exact)
        try:
            model = mujoco.MjModel.from_xml_path(str(scenes[name]))
            data = mujoco.MjData(model)
            steps = int(round(a.seconds/model.opt.timestep))
            A.log(f'variant {name}: {steps} steps of {model.opt.timestep} s')
            run = settle(model, data, state, steps)
            final = run['trace'][-1] if run['trace'] else None
            independent = A.independent_overlap(model, data, sources, rng)
            gh = (final or run['first_forward'])['guide_holder_pairs_n']
            results[name] = dict(**run, sdf_iterations=int(model.opt.sdf_iterations), sdf_initpoints=int(model.opt.sdf_initpoints),
                                 final_independent_overlap=independent,
                                 peak_point_n=max((r['max_point_n'] for r in run['trace']), default=None),
                                 peak_pair_n=max((r['max_pair_n'] for r in run['trace']), default=None),
                                 peak_penetration_mm=max((r['max_penetration_mm'] for r in run['trace']), default=None))
            A.log(f'  failure={run["failure"]} steps={run["steps_completed"]} wall={run["wall_s"]:.1f}s first-forward max pair {run["first_forward"]["max_pair_n"]:.3f} N '
                  f'peak pair {results[name]["peak_pair_n"]} N final guide/holder {gh} independent overlap {independent["max_sampled_penetration_mm"]} mm')
        finally:
            X.set_collider(handle, False)
        del model, data
    summary = {k: dict(failure=v['failure'], steps_completed=v['steps_completed'], first_forward_max_pair_n=v['first_forward']['max_pair_n'],
                       first_forward_max_pair=v['first_forward']['max_pair'], peak_point_n=v['peak_point_n'], peak_pair_n=v['peak_pair_n'],
                       peak_penetration_mm=v['peak_penetration_mm'],
                       final_reported_penetration_mm=v['trace'][-1]['max_penetration_mm'] if v['trace'] else None,
                       final_guide_holder_pairs_n=v['trace'][-1]['guide_holder_pairs_n'] if v['trace'] else None,
                       final_guide_drift_mm=v['trace'][-1]['guide_drift_mm'] if v['trace'] else None,
                       final_independent_guide_holder_penetration_mm=v['final_independent_overlap']['max_sampled_penetration_mm'],
                       wall_s=v['wall_s']) for k, v in results.items()}
    summary.update(promotion_allowed=False, full_planter_success=False, physical_success=False)
    (a.out/'result.json').write_text(json.dumps(dict(scope=pre['scope'], status='completed_settle_audit', summary=summary, variants=results,
                                                      plugin=manifest, inputs=inputs, mujoco_version=mujoco.__version__), indent=2)+'\n')
    (a.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    A.log(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
