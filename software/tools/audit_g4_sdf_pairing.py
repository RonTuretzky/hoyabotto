"""Output-only comparison of guide/holder collision pairings on the exact SDF station.

MuJoCo searches SDF/SDF pairs from Halton points inside the AABB overlap, but
mesh/SDF pairs run Frank-Wolfe over every real triangle of the mesh geom. This
audit keeps the exact source-triangle SDF on both parts and adds explicit
mesh/SDF pairs built from the same original STLs, then compares what each
pairing reports at the recorded contact09 stop state (and small vertical guide
offsets) against the independent original-STL overlap measurement. It does not
step physics, move hardware, or award task credit.
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

CONTACT = dict(condim='4', friction='.6 .003 .0001', solref='0.004 1', solimp='.95 .99 .001')
VARIANTS = {
    'sdf_sdf': 'native SDF/SDF pairing (Halton starts in the AABB overlap)',
    'holder_mesh_vs_guide_sdf': 'explicit pair: original holder triangles against the exact guide field',
    'both_mesh_sdf_pairs': 'explicit pairs in both directions: holder triangles vs guide field and guide triangles vs holder field',
    'both_mesh_exact_collider': 'the same two explicit pairs through the plugin\'s exhaustive mesh/exact-SDF narrowphase instead of mjc_MeshSDF',
}
EXACT_COLLIDER = {'both_mesh_exact_collider'}


def variant_scene(base_scene, out, variant):
    tree = E.parse(base_scene)
    root = tree.getroot()
    if variant != 'sdf_sdf':
        for geom in root.iter('geom'):
            if geom.get('contype') == '0' or geom.get('name') in ('guide_source_sdf', 'holder_source_sdf'):
                continue
            geom.set('conaffinity', '3')  # keep pairing with the holder field (bit 2) and everything else (bit 1)
        holder = root.find("./worldbody/body[@name='holder']")
        guide = root.find("./worldbody/body[@name='guide']")
        holder.find("geom[@name='holder_source_sdf']").set('contype', '2')
        holder.find("geom[@name='holder_source_sdf']").set('conaffinity', '2')  # never SDF/SDF against the guide (bits 1)
        E.SubElement(holder, 'geom', name='holder_source_mesh', type='mesh', mesh='g4_holder_visual', contype='0', conaffinity='0', group='3', mass='0')
        contact = root.find('contact')
        if contact is None:
            contact = E.SubElement(root, 'contact')
        E.SubElement(contact, 'pair', geom1='holder_source_mesh', geom2='guide_source_sdf', **CONTACT)
        if variant in ('both_mesh_sdf_pairs', 'both_mesh_exact_collider'):
            E.SubElement(guide, 'geom', name='guide_source_mesh', type='mesh', mesh='g4_guide_visual', contype='0', conaffinity='0', group='3', mass='0')
            E.SubElement(contact, 'pair', geom1='guide_source_mesh', geom2='holder_source_sdf', **CONTACT)
    E.indent(root)
    tree.write(out, encoding='unicode')
    return out


def pair_counts(model, data):
    pairs = {}
    for i in range(data.ncon):
        c = data.contact[i]
        key = ' / '.join(sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) for g in (c.geom1, c.geom2)))
        pairs[key] = pairs.get(key, 0)+1
    return pairs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--native-scene', type=Path, default=A.OUTPUT/'g4-native-sdf-01/depth6/scene.xml')
    p.add_argument('--holder-stl', type=Path, default=A.OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/cressmaster-holder.stl')
    p.add_argument('--guide-stl', type=Path, default=A.OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/guide_PROTOTYPE.stl')
    p.add_argument('--state', type=Path, default=A.OUTPUT/'g4-assembly-next/scoring-station-contact09-stop-row.json')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--settings', default='10:40,50:100')
    p.add_argument('--offsets-mm', default='-0.05,0,0.05,0.2')
    p.add_argument('--cpu-limit-s', type=int, default=3600)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    resource.setrlimit(resource.RLIMIT_CPU, (a.cpu_limit_s, a.cpu_limit_s))
    sources = dict(holder=a.holder_stl, guide=a.guide_stl)
    settings = [tuple(int(v) for v in s.split(':')) for s in a.settings.split(',')]
    offsets = [float(v) for v in a.offsets_mm.split(',')]
    state = json.loads(a.state.read_text())
    manifest = X.build(a.out/'plugin')
    handle = X.load(manifest['library'])
    base = A.write_scene(a.native_scene, a.out/'scene-base.xml', sources, with_files=True)
    scenes = {v: variant_scene(base, a.out/f'scene-{v}.xml', v) for v in VARIANTS}
    inputs = {str(k): X.sha(k) for k in [Path(__file__), A.__file__, X.SOURCE, manifest['library'], a.native_scene, a.state, *sources.values(), base, *scenes.values()]}
    pre = dict(scope='Privileged static pairing comparison at the recorded contact09 state; no stepping, hardware or task qualification',
               variants=VARIANTS, settings=settings, guide_z_offsets_mm=offsets, gates=A.GATES, inputs=inputs, promotion_allowed=False)
    (a.out/'predeclared.json').write_text(json.dumps(pre, indent=2)+'\n')
    rng = np.random.default_rng(20261007)
    truths = {}
    results = {}
    for variant, scene in scenes.items():
        A.log(f'variant {variant}')
        X.set_collider(handle, variant in EXACT_COLLIDER)
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        instances = {n: X.instance_info(handle, model, data, f'{n}_source_sdf') for n in A.PARTS}
        guide = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'guide')
        adr = int(model.jnt_qposadr[int(model.body_jntadr[guide])])
        cases = []
        for offset in offsets:
            if offset not in truths:
                mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
                data.qpos[adr+2] += offset*.001
                mujoco.mj_kinematics(model, data)
                truths[offset] = A.independent_overlap(model, data, sources, rng)
                A.log(f'  independent overlap at {offset:+.3f} mm: penetration {truths[offset]["max_sampled_penetration_mm"]} mm, min gap {truths[offset]["min_sampled_gap_mm"]:.6f} mm')
            runs = []
            for iterations, starts in settings:
                model.opt.sdf_iterations, model.opt.sdf_initpoints = iterations, starts
                reports, timings = [], []
                for repeat in range(2):
                    mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
                    data.qpos[adr+2] += offset*.001
                    t = time.monotonic()
                    mujoco.mj_forward(model, data)
                    timings.append(time.monotonic()-t)
                    queries = {n: X.PluginQuery(model, data, f'{n}_source_sdf') for n in A.PARTS}
                    reports.append(A.collider_report(model, data, queries))
                gh = reports[0]['pairs'].get('guide / holder')
                witnesses = [c for c in reports[0]['contacts'] if {'guide', 'holder'} == set(c['bodies'])]
                endpoint = max((abs(f['signed_distance_at_inferred_endpoint_mm']) for c in witnesses for f in c['plugin_fields'].values()), default=None)
                runs.append(dict(iterations=iterations, starts=starts, forward_s=timings, deterministic_repeat=json.dumps(reports[0], sort_keys=True) == json.dumps(reports[1], sort_keys=True),
                                 guide_holder=gh, guide_holder_detected=gh is not None, all_pairs=pair_counts(model, data),
                                 max_abs_endpoint_field_mm=endpoint, warnings=reports[0]['warnings'], report=reports[0]))
                A.log(f'  {offset:+.3f} mm, {iterations}:{starts}: forward {timings[0]:.2f} s, guide/holder {gh}, endpoint |field| {endpoint}')
            cases.append(dict(guide_z_offset_mm=offset, runs=runs))
        X.set_collider(handle, False)
        results[variant] = dict(description=VARIANTS[variant], exact_collider=variant in EXACT_COLLIDER, instances=instances, ngeom=int(model.ngeom), npair=int(model.npair), cases=cases)
        del model, data
    table = {}
    for offset in offsets:
        truth = truths[offset]
        row = dict(independent_max_sampled_penetration_mm=truth['max_sampled_penetration_mm'], independent_min_sampled_gap_mm=truth['min_sampled_gap_mm'])
        for variant in VARIANTS:
            for run in next(c for c in results[variant]['cases'] if c['guide_z_offset_mm'] == offset)['runs']:
                gh = run['guide_holder'] or {}
                row[f'{variant}@{run["iterations"]}:{run["starts"]}'] = dict(detected=run['guide_holder_detected'], contacts=gh.get('contacts', 0),
                                                                            reported_max_penetration_mm=gh.get('reported_max_penetration_mm'), normal_n=gh.get('normal_n'),
                                                                            reported_minus_independent_mm=(gh['reported_max_penetration_mm']-truth['max_sampled_penetration_mm']) if gh else None,
                                                                            forward_s=run['forward_s'][0], endpoint_on_surface_mm=run['max_abs_endpoint_field_mm'])
        table[f'{offset:+.3f}'] = row
    summary = dict(comparison_by_offset=table, promotion_allowed=False, full_planter_success=False, physical_success=False)
    (a.out/'result.json').write_text(json.dumps(dict(scope=pre['scope'], status='completed_pairing_audit', summary=summary, variants=results,
                                                      independent={f'{k:+.3f}': v for k, v in truths.items()}, plugin=manifest, inputs=inputs,
                                                      mujoco_version=mujoco.__version__), indent=2)+'\n')
    (a.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    A.log(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
