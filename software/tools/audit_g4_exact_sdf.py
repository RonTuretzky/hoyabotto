"""Output-only audit of the exact source-triangle SDF plugin on the G4 station.

Stages: build the plugin, analytic box controls, static field checks against
an independent double-precision reference on predeclared points, the exact
recorded contact09 stop state (reported contacts versus an independent
guide/holder overlap measurement from the original STLs), and collision-search
coverage across sdf_iterations/sdf_initpoints and small vertical guide
offsets. Nothing steps physics, moves hardware, or awards task credit.
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
import trimesh
from scipy.optimize import nnls

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from planter import g4_exact_sdf as X  # noqa: E402

OUTPUT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output')
PARTS = ('holder', 'guide')
GATES = dict(distance_error_mm=1e-6, wrong_sign_truth_band_mm=1e-6, unique_gradient_error=1e-4, gradient_surface_band_mm=1e-5,
             frame_distance_difference_mm=1e-9, frame_gradient_difference=1e-6,
             simulation_penetration_mm=.2, simulation_contact_force_n=8.)


def log(message):
    print(message, flush=True)


def write_scene(native_scene, out, files, *, with_files):
    tree = E.parse(native_scene)
    root = tree.getroot()
    ext = root.find('extension')
    if ext is None:
        ext = E.SubElement(root, 'extension')
    plugin = E.SubElement(ext, 'plugin', plugin=X.PLUGIN_NAME)
    for name in PARTS:
        inst = E.SubElement(plugin, 'instance', name=f'{name}_exact_mesh')
        if with_files:
            E.SubElement(inst, 'config', key='file', value=str(files[name]))
        geom = root.find(f"./worldbody/body[@name='{name}']/geom[@name='{name}_source_sdf']")
        if geom is None:
            raise ValueError(f'native scene lacks {name}_source_sdf')
        E.SubElement(geom, 'plugin', instance=f'{name}_exact_mesh')
    E.indent(root)
    tree.write(out, encoding='unicode')
    return out


def box_control(handle, out_dir, *, with_file):
    extents = np.array([.020, .012, .008])
    stl = out_dir/'source-box.stl'
    trimesh.creation.box(extents=extents).export(stl)
    root = E.Element('mujoco')
    inst = E.SubElement(E.SubElement(E.SubElement(root, 'extension'), 'plugin', plugin=X.PLUGIN_NAME), 'instance', name='box')
    if with_file:
        E.SubElement(inst, 'config', key='file', value=str(stl))
    E.SubElement(E.SubElement(root, 'asset'), 'mesh', name='source', file=str(stl))
    body = E.SubElement(E.SubElement(root, 'worldbody'), 'body', name='box', pos='.1 -.2 .3', euler='.2 -.3 .4')
    E.SubElement(body, 'freejoint')
    E.SubElement(body, 'inertial', pos='0 0 0', mass='.02', diaginertia='.000001 .000001 .000001')
    E.SubElement(E.SubElement(body, 'geom', name='box', type='sdf', mesh='source'), 'plugin', instance='box')
    scene = out_dir/f'box-{"source" if with_file else "compiled"}.xml'
    E.ElementTree(root).write(scene, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    query = X.PluginQuery(model, data, 'box')
    rng = np.random.default_rng(84216)
    points = np.vstack([rng.uniform(-.02, .02, (2000, 3)), [[0, 0, 0], [.009999, 0, 0], [.010001, 0, 0], [.01, .006, .004], [.01, 0, 0], [.012, .008, .006]]])
    worst = wrong = grad = 0
    for i, point in enumerate(points):
        q = np.abs(point)-extents/2
        truth = float(np.linalg.norm(np.maximum(q, 0))+min(float(q.max()), 0))
        value, gradient = query.body(point)
        worst = max(worst, abs(value-truth))
        wrong += truth*value < 0 and abs(truth) > 1e-8
        if i < 2000:
            eps = 1e-8
            finite = np.array([(query.body(point+np.eye(3)[k]*eps)[0]-query.body(point-np.eye(3)[k]*eps)[0])/(2*eps) for k in range(3)])
            grad = max(grad, float(np.linalg.norm(finite-gradient)))
    info = X.instance_info(handle, model, data, 'box')
    summary = dict(mode=info['mode'], points=len(points), max_abs_distance_error_m=worst, wrong_signs=int(wrong),
                   max_gradient_difference_away_from_explicit_edge_controls=grad, compiled_deviation_m=info['compiled_deviation_m'])
    summary['passed'] = bool(worst < 1e-8 and wrong == 0 and grad < 1e-5)
    return summary


def field_audit(model, data, handle, points, sources, state):
    """Plugin field versus independent reference on predeclared body-frame points (mm).

    Points are mapped into the geom frame with the exact compiler transform
    (``R(mesh_quat)^T (p - mesh_pos)``) so that the plugin's arithmetic is
    scored alone. MuJoCo's own ``geom_xmat`` differs from that rotation by
    about 1e-9 rad; its effect on the body-frame query path is reported
    separately as ``kinematics_roundtrip``.
    """
    mujoco.mj_resetData(model, data)
    mujoco.mj_kinematics(model, data)
    # Exact binary STL triangles moved into each geom frame in double (m).
    triangles = {n: X.source_triangles_in_geom_frame(model, f'{n}_source_sdf', sources[n])*1000 for n in PARTS}
    normals = {n: X.face_normals(triangles[n]) for n in PARTS}
    query = {n: X.PluginQuery(model, data, f'{n}_source_sdf') for n in PARTS}
    frames = {}
    for n in PARTS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'{n}_source_sdf')
        mid = int(model.geom_dataid[gid])
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.mesh_quat[mid])
        frames[n] = dict(rotation=rotation.reshape(3, 3), position=model.mesh_pos[mid].copy(),
                         geom_pos_equals_mesh_pos=bool(np.array_equal(model.geom_pos[gid], model.mesh_pos[mid])),
                         geom_quat_equals_mesh_quat=bool(np.array_equal(model.geom_quat[gid], model.mesh_quat[mid])),
                         geom_xmat_vs_exact_rotation=float(np.abs(data.geom_xmat[gid].reshape(3, 3)-data.xmat[int(model.geom_bodyid[gid])].reshape(3, 3)@rotation.reshape(3, 3)).max()),
                         scale=model.mesh_scale[mid].tolist())
        if not (frames[n]['geom_pos_equals_mesh_pos'] and frames[n]['geom_quat_equals_mesh_quat']):
            raise ValueError(f'{n}: body frame is not the scaled source frame; reference would be misaligned')
    def to_local(part, p_mm):
        return frames[part]['rotation'].T@(np.asarray(p_mm, dtype=float)*.001-frames[part]['position'])
    records = []
    roundtrip = 0.
    for i, entry in enumerate(points):
        p = np.asarray(entry['point_mm'], dtype=float)
        part = entry['part']
        local = to_local(part, p)
        ref = X.reference_field(triangles[part], local*1000, unique_band=1e-9, gradient_band=GATES['gradient_surface_band_mm'])
        v, g = query[part].local(local)
        v *= 1000
        roundtrip = max(roundtrip, abs(query[part].body(p*.001)[0]*1000-v))
        unit = g/max(np.linalg.norm(g), 1e-30)
        near = ref['face_distances'] <= ref['face_distances'].min()+.001
        cone = float(nnls(normals[part][near].T, unit)[1])
        gradient_error = float(np.linalg.norm(g-ref['gradient'])) if ref['gradient'] is not None else None
        fd = None
        if i % 31 == 0 and ref['unsigned_distance'] > 1e-4 and ref['unique']:
            h = 1e-5
            numerical = np.array([(query[part].local(to_local(part, p+s))[0]-query[part].local(to_local(part, p-s))[0])*1000/(2*h) for s in np.eye(3)*h])
            # The body-frame finite difference equals R(mesh_quat) times the local gradient.
            fd = dict(h_mm=h, gradient_body=numerical.tolist(), callback_error=float(np.linalg.norm(numerical-frames[part]['rotation']@g)))
        records.append(dict(**entry, source_signed_distance_mm=ref['signed_distance'], plugin_signed_distance_mm=v,
                            error_mm=v-ref['signed_distance'],
                            wrong_sign=bool(ref['unsigned_distance'] > GATES['wrong_sign_truth_band_mm'] and (v < 0) != ref['inside']),
                            plugin_gradient_local=g.tolist(), unique_projection=ref['unique'],
                            unique_gradient_error=gradient_error, source_normal_cone_residual=cone, finite_difference=fd))
    frame_checks = []
    chosen = [i for i in range(len(points)) if i % 37 == 0]
    for label in ['actual_contact09', 'arbitrary_rigid_pose']:
        mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
        if label == 'arbitrary_rigid_pose':
            for j, n in enumerate(PARTS):
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                adr = int(model.jnt_qposadr[int(model.body_jntadr[bid])])
                data.qpos[adr:adr+3] = [.11*j-.07, .09, .3]
                quat = np.array([.81, .12, -.31, .47])
                data.qpos[adr+3:adr+7] = quat/np.linalg.norm(quat)
        mujoco.mj_kinematics(model, data)
        for i in chosen:
            row = records[i]
            local = to_local(row['part'], row['point_mm'])
            v, g = query[row['part']].local(local)
            rg = data.geom_xmat[query[row['part']].g].reshape(3, 3)
            world_value, world_gradient = query[row['part']].world(rg@local+data.geom_xpos[query[row['part']].g])
            frame_checks.append(dict(state=label, point_index=i, distance_difference_mm=v*1000-row['plugin_signed_distance_mm'],
                                     gradient_difference=float(np.linalg.norm(g-row['plugin_gradient_local'])),
                                     world_path_distance_difference_mm=world_value*1000-row['plugin_signed_distance_mm'],
                                     world_path_gradient_difference=float(np.linalg.norm(rg.T@world_gradient-row['plugin_gradient_local']))))
    summary = {}
    for part in PARTS:
        rows = [r for r in records if r['part'] == part]
        summary[part] = dict(points=len(rows), max_abs_distance_error_mm=max(abs(r['error_mm']) for r in rows),
                             distance_error_over_gate_count=sum(abs(r['error_mm']) > GATES['distance_error_mm'] for r in rows),
                             wrong_sign_count=sum(r['wrong_sign'] for r in rows),
                             unique_gradient_points=sum(r['unique_gradient_error'] is not None for r in rows),
                             max_unique_gradient_error=max((r['unique_gradient_error'] or 0) for r in rows),
                             unique_gradient_error_over_gate_count=sum((r['unique_gradient_error'] or 0) > GATES['unique_gradient_error'] for r in rows),
                             max_normal_cone_residual=max(r['source_normal_cone_residual'] for r in rows),
                             max_normal_cone_residual_beyond_10nm_band=max(r['source_normal_cone_residual'] for r in rows if abs(r['source_signed_distance_mm']) > 1e-5),
                             finite_difference_points=sum(r['finite_difference'] is not None for r in rows),
                             max_finite_difference_callback_error=max((r['finite_difference'] or {}).get('callback_error', 0) for r in rows))
    frame_summary = dict(checks=len(frame_checks), max_distance_difference_mm=max(abs(r['distance_difference_mm']) for r in frame_checks),
                         max_gradient_difference=max(r['gradient_difference'] for r in frame_checks),
                         world_path_max_distance_difference_mm=max(abs(r['world_path_distance_difference_mm']) for r in frame_checks),
                         world_path_max_gradient_difference=max(r['world_path_gradient_difference'] for r in frame_checks))
    kinematics = dict(body_path_max_abs_difference_mm=roundtrip, geom_xmat_vs_exact_rotation={n: frames[n]['geom_xmat_vs_exact_rotation'] for n in PARTS},
                      note='MuJoCo geom_xmat differs from the exact mesh_quat rotation by ~1e-9 rad; the collider sees the exact field composed with that pose rounding')
    # Local-frame queries cannot depend on pose; the world path through MuJoCo's own geom_xpos/geom_xmat is what is gated.
    passed = all(s['distance_error_over_gate_count'] == 0 and s['wrong_sign_count'] == 0 and s['unique_gradient_error_over_gate_count'] == 0 for s in summary.values()) \
        and frame_summary['world_path_max_distance_difference_mm'] <= GATES['frame_distance_difference_mm'] and frame_summary['world_path_max_gradient_difference'] <= GATES['frame_gradient_difference']
    frame_identity = {n: {k: v for k, v in frames[n].items() if k not in ('rotation', 'position')} for n in PARTS}
    return dict(summary=summary, frame_summary=frame_summary, kinematics_roundtrip=kinematics, frame_identity=frame_identity,
                passed=bool(passed), records=records, frame_checks=frame_checks)


def surface_samples(triangles, rng, per_face=2):
    """Vertices, edge midpoints, centroids and random interior points of every face."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    samples = [a, b, c, (a+b)/2, (b+c)/2, (c+a)/2, (a+b+c)/3]
    for _ in range(per_face):
        bary = rng.dirichlet(np.ones(3), len(triangles))
        samples.append(np.einsum('fk,fkj->fj', bary, triangles))
    return np.unique(np.vstack(samples), axis=0)


def world_triangles(model, data, geom, stl_path):
    """Original STL triangles in world millimetres at the current state."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
    local = X.source_triangles_in_geom_frame(model, gid, stl_path)
    rotation = data.geom_xmat[gid].reshape(3, 3)
    return (np.einsum('ij,fkj->fki', rotation, local)+data.geom_xpos[gid])*1000


def independent_overlap(model, data, sources, rng, *, margin_mm=1.):
    """Signed distance of each part's surface samples to the other original solid.

    Negative values are overlap. Surface sampling bounds the true maximum
    penetration from below; the sample density is recorded.
    """
    tri = {n: world_triangles(model, data, f'{n}_source_sdf', sources[n]) for n in PARTS}
    result = {}
    for own, other in (('guide', 'holder'), ('holder', 'guide')):
        samples = surface_samples(tri[own], rng)
        lo, hi = tri[other].reshape(-1, 3).min(axis=0)-margin_mm, tri[other].reshape(-1, 3).max(axis=0)+margin_mm
        keep = np.all((samples >= lo) & (samples <= hi), axis=1)
        candidates = samples[keep]
        values = np.array([X.reference_field(tri[other], p)['signed_distance'] for p in candidates]) if len(candidates) else np.array([])
        order = np.argsort(values)[:10] if len(values) else []
        result[f'{own}_surface_vs_{other}_solid'] = dict(
            surface_samples=int(len(samples)), samples_near_other=int(len(candidates)),
            min_signed_distance_mm=float(values.min()) if len(values) else None,
            samples_inside_other=int((values < 0).sum()) if len(values) else 0,
            deepest_points_world_mm=[dict(point=candidates[i].tolist(), signed_distance_mm=float(values[i])) for i in order])
    depths = [-v['min_signed_distance_mm'] for v in result.values() if v['min_signed_distance_mm'] is not None]
    result['max_sampled_penetration_mm'] = max(max(depths), 0.) if depths else None
    result['min_sampled_gap_mm'] = min(v['min_signed_distance_mm'] for v in result.values() if isinstance(v, dict) and v.get('min_signed_distance_mm') is not None)
    return result


def collider_report(model, data, queries):
    """Guide/holder and jaw/guide contacts with plugin field values at their witnesses."""
    rows, pairs = [], {}
    for i in range(data.ncon):
        c = data.contact[i]
        bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) for g in (c.geom1, c.geom2)]
        if not ({'guide', 'holder'} & set(bodies)):
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        normal = c.frame.reshape(3, 3)[0]
        key = ' / '.join(sorted(bodies))
        v = pairs.setdefault(key, dict(contacts=0, normal_n=0., resultant_n=0., reported_max_penetration_mm=0.))
        v['contacts'] += 1
        v['normal_n'] += max(0., float(wrench[0]))
        v['resultant_n'] += float(np.linalg.norm(wrench[:3]))
        v['reported_max_penetration_mm'] = max(v['reported_max_penetration_mm'], -float(c.dist)*1000)
        fields = {}
        for n in PARTS:
            if n in bodies:
                at = queries[n].world(c.pos)
                sign = -1 if bodies.index(n) == 0 else 1
                end = queries[n].world(c.pos+sign*c.dist*.5*normal)
                fields[n] = dict(signed_distance_at_witness_mm=at[0]*1000, gradient_dot_normal_at_witness=float(np.dot(at[1], normal)),
                                 signed_distance_at_inferred_endpoint_mm=end[0]*1000)
        rows.append(dict(bodies=bodies, geoms=[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (c.geom1, c.geom2)],
                         position_m=c.pos.tolist(), normal=normal.tolist(), distance_mm=float(c.dist)*1000,
                         normal_force_n=float(wrench[0]), resultant_n=float(np.linalg.norm(wrench[:3])), plugin_fields=fields))
    return dict(ncon=int(data.ncon), pairs=pairs, contacts=rows, warnings=[int(w.number) for w in data.warning])


def state_audit(model, data, handle, sources, state, settings, offsets_mm, rng):
    guide = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'guide')
    adr = int(model.jnt_qposadr[int(model.body_jntadr[guide])])
    cases = []
    for offset in offsets_mm:
        mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
        data.qpos[adr+2] += offset*.001
        mujoco.mj_kinematics(model, data)
        t = time.monotonic()
        truth = independent_overlap(model, data, sources, rng)
        truth_s = time.monotonic()-t
        log(f'  offset {offset:+.3f} mm: independent overlap {truth["max_sampled_penetration_mm"]} mm, min gap {truth["min_sampled_gap_mm"]:.6f} mm ({truth_s:.1f} s)')
        runs = []
        for iterations, starts in settings:
            model.opt.sdf_iterations, model.opt.sdf_initpoints = iterations, starts
            timings, reports = [], []
            for repeat in range(2):
                mujoco.mj_setState(model, data, np.asarray(state['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
                data.qpos[adr+2] += offset*.001
                t = time.monotonic()
                mujoco.mj_forward(model, data)
                timings.append(time.monotonic()-t)
                queries = {n: X.PluginQuery(model, data, f'{n}_source_sdf') for n in PARTS}
                reports.append(collider_report(model, data, queries))
            same = json.dumps(reports[0], sort_keys=True) == json.dumps(reports[1], sort_keys=True)
            gh = reports[0]['pairs'].get('guide / holder')
            runs.append(dict(iterations=iterations, starts=starts, forward_s=timings, deterministic_repeat=bool(same),
                             guide_holder=gh, guide_holder_detected=gh is not None, report=reports[0]))
            log(f'    sdf_iterations={iterations} sdf_initpoints={starts}: forward {timings[0]:.2f} s, guide/holder {gh}')
        cases.append(dict(guide_z_offset_mm=offset, independent=truth, runs=runs))
    model.opt.sdf_iterations, model.opt.sdf_initpoints = 10, 40
    return cases


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--native-scene', type=Path, default=OUTPUT/'g4-native-sdf-01/depth6/scene.xml')
    p.add_argument('--holder-stl', type=Path, default=OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/cressmaster-holder.stl')
    p.add_argument('--guide-stl', type=Path, default=OUTPUT/'g4-assembly-next/collision-interface-11/source_assets/guide_PROTOTYPE.stl')
    p.add_argument('--state', type=Path, default=OUTPUT/'g4-assembly-next/scoring-station-contact09-stop-row.json')
    p.add_argument('--points', type=Path, default=OUTPUT/'g4-exact-sdf-audit-01/run-02-stable-reference/predeclared.json',
                   help='previous predeclared point set, reused unchanged for comparability')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--settings', default='10:40,20:40,50:100,100:500')
    p.add_argument('--offsets-mm', default='-0.05,0,0.05')
    p.add_argument('--cpu-limit-s', type=int, default=3600)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    resource.setrlimit(resource.RLIMIT_CPU, (a.cpu_limit_s, a.cpu_limit_s))
    sources = dict(holder=a.holder_stl, guide=a.guide_stl)
    settings = [tuple(int(v) for v in s.split(':')) for s in a.settings.split(',')]
    offsets = [float(v) for v in a.offsets_mm.split(',')]
    points = json.loads(a.points.read_text())['points']
    state = json.loads(a.state.read_text())
    log('building plugin')
    manifest = X.build(a.out/'plugin')
    handle = X.load(manifest['library'])
    scene = write_scene(a.native_scene, a.out/'scene.xml', sources, with_files=True)
    scene_compiled = write_scene(a.native_scene, a.out/'scene-compiled-mode.xml', sources, with_files=False)
    inputs = {str(k): X.sha(k) for k in [Path(__file__), X.SOURCE, manifest['library'], a.native_scene, a.state, a.points, *sources.values(), scene, scene_compiled]}
    pre = dict(scope='Privileged static field, exact-state and collision-search audit of the exact source-triangle SDF; no stepping, hardware, or task qualification',
               plugin=manifest, gates=GATES, settings=settings, guide_z_offsets_mm=offsets, point_count=len(points), seed=20261007,
               reference='All original source triangles, plane/edge nearest point, closed-solid winding sign; unique nearest-point gradient only beyond a 10 nm surface band',
               independent_overlap='Original STL surface samples (vertices, edge midpoints, centroids, two random interior points per face) against the other original solid; a lower bound on penetration',
               inputs=inputs, promotion_allowed=False)
    (a.out/'predeclared.json').write_text(json.dumps(pre, indent=2)+'\n')
    log('box controls')
    box = {mode: box_control(handle, a.out, with_file=(mode == 'source')) for mode in ('source', 'compiled')}
    log(f'  {box}')
    if not box['source']['passed']:
        raise SystemExit('analytic source-mode control failed; audit stopped')
    log('compiling station')
    t = time.monotonic()
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    compile_s = time.monotonic()-t
    instances = {n: X.instance_info(handle, model, data, f'{n}_source_sdf') for n in PARTS}
    log(f'  {instances}')
    for n, info in instances.items():
        if info['mode'] != 'source_double' or info['compiled_deviation_m'] > X.COMPILED_DEVIATION_LIMIT_M:
            raise SystemExit(f'{n}: plugin did not enter source_double mode')
    log('field audit (source_double)')
    field = field_audit(model, data, handle, points, sources, state)
    log(f'  {field["summary"]} frame {field["frame_summary"]} passed={field["passed"]}')
    log('field audit (compiled_float32 decomposition)')
    model_c = mujoco.MjModel.from_xml_path(str(scene_compiled))
    data_c = mujoco.MjData(model_c)
    field_c = field_audit(model_c, data_c, handle, points, sources, state)
    log(f'  {field_c["summary"]}')
    del model_c, data_c
    log('exact contact09 state and search coverage')
    rng = np.random.default_rng(20261007)
    cases = state_audit(model, data, handle, sources, state, settings, offsets, rng)
    actual = next(c for c in cases if c['guide_z_offset_mm'] == 0)
    detection = {f'{r["iterations"]}:{r["starts"]}': {f'{c["guide_z_offset_mm"]:+.3f}': (c['independent']['max_sampled_penetration_mm'] > 0, r2['guide_holder_detected'])
                 for c in cases for r2 in c['runs'] if (r2['iterations'], r2['starts']) == (r['iterations'], r['starts'])} for r in actual['runs']}
    summary = dict(
        plugin_modes={n: i['mode'] for n, i in instances.items()},
        compiled_deviation_m={n: i['compiled_deviation_m'] for n, i in instances.items()},
        box_controls={k: v['passed'] for k, v in box.items()},
        field_passed=field['passed'],
        field_max_abs_distance_error_mm={n: field['summary'][n]['max_abs_distance_error_mm'] for n in PARTS},
        field_max_unique_gradient_error={n: field['summary'][n]['max_unique_gradient_error'] for n in PARTS},
        compiled_mode_max_abs_distance_error_mm={n: field_c['summary'][n]['max_abs_distance_error_mm'] for n in PARTS},
        compiled_mode_max_unique_gradient_error={n: field_c['summary'][n]['max_unique_gradient_error'] for n in PARTS},
        actual_state_independent_max_sampled_penetration_mm=actual['independent']['max_sampled_penetration_mm'],
        actual_state_independent_min_gap_mm=actual['independent']['min_sampled_gap_mm'],
        actual_state_guide_holder_by_setting={f'{r["iterations"]}:{r["starts"]}': r['guide_holder'] for r in actual['runs']},
        actual_state_forward_s_by_setting={f'{r["iterations"]}:{r["starts"]}': r['forward_s'][0] for r in actual['runs']},
        detection_table_overlap_vs_detected=detection,
        deterministic_repeats=all(r['deterministic_repeat'] for c in cases for r in c['runs']),
        simulation_gates_unchanged=dict(penetration_mm=GATES['simulation_penetration_mm'], contact_force_n=GATES['simulation_contact_force_n']),
        promotion_allowed=False, full_planter_success=False, physical_success=False)
    result = dict(scope=pre['scope'], status='completed_exact_sdf_audit', summary=summary, plugin=manifest, instances=instances,
                  station_compile_s=compile_s, box_controls=box, field=field, field_compiled_mode={k: v for k, v in field_c.items() if k != 'records'},
                  state_cases=cases, inputs=inputs, mujoco_version=mujoco.__version__,
                  maxrss_bytes_mac=int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
    (a.out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    (a.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    log(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
