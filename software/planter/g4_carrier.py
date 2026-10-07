"""G4 empty-carrier gravity diagnostic; privileged reset/evaluation, no robot.

Collision solids are convex partitions of actual source STLs, never their whole
convex hulls. Only the first funnel corridor is retained for static collisions;
leaving that corridor fails the trial. The original full meshes remain visual.
All lengths in geometry preparation are mm, and MuJoCo uses metres.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import manifold3d as M
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull
import trimesh

X_SLOT = -45.0
ROI_X = (-61.0, -29.0)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def solid(mesh):
    value = M.Manifold(M.Mesh(np.asarray(mesh.vertices, dtype=np.float32),
                            np.asarray(mesh.faces, dtype=np.uint32)))
    if value.status() != M.Error.NoError:
        raise ValueError(f'Invalid manifold geometry: {value.status()}')
    return value


def mesh_of(obj):
    m = obj.to_mesh()
    return trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts), process=False)


def clip_x(obj):
    return obj.trim_by_plane([1, 0, 0], ROI_X[0]).trim_by_plane([-1, 0, 0], -ROI_X[1])


def guide_sectors(obj):
    """Split actual guide at its CAD layer planes and 96-facet cap rays.

    These are cuts of the source solid, not reconstructed dimensions or a CAD
    regeneration. Preserving the exact hole boundaries avoids voxel/VHACD fill.
    """
    pieces = []
    for za, zb in [(0, 4), (4, 22), (22, 34.01)]:
        layer = obj.trim_by_plane([0, 0, 1], za).trim_by_plane([0, 0, -1], -zb)
        center = layer.trim_by_plane([0, 1, 0], -30).trim_by_plane([0, -1, 0], -30)
        pieces.extend(center.split_by_plane([1, 0, 0], X_SLOT))
        for cy, lo, hi in [(30, 0, 180), (-30, 180, 360)]:
            for a0 in np.arange(lo, hi, 3.75):
                t0, t1 = np.radians([a0, a0 + 3.75])
                n0 = np.array([-np.sin(t0), np.cos(t0), 0])
                n1 = np.array([np.sin(t1), -np.cos(t1), 0])
                origin = np.array([X_SLOT, cy, 0])
                pieces.append(layer.trim_by_plane(n0, float(np.sum(n0 * origin))).trim_by_plane(
                    n1, float(np.sum(n1 * origin))))
    return pieces


def convex_partition(initial, *, max_pieces=2500, max_hull_extra_mm3=.005,
                     min_piece_mm3=.0001, plane_sign_tol_mm=.001):
    """Source-face BSP with bounded numerical sliver removal, then hull audit.

    A rejected decomposition raises instead of quietly exporting a closed hole.
    The output's volume equality and critical local openings are audited again.
    """
    stack = [(p, 0) for p in initial]
    result, dropped, hull_extra = [], 0.0, 0.0
    while stack:
        obj, depth = stack.pop()
        volume = obj.volume()
        if volume < min_piece_mm3:
            dropped += volume
            continue
        m = obj.to_mesh()
        v = np.asarray(m.vert_properties)[:, :3].astype(float)
        f = np.asarray(m.tri_verts)
        hull = ConvexHull(v)
        extra = max(0., hull.volume - volume)
        if extra <= max_hull_extra_mm3:
            faces = hull.simplices.copy()
            tri = v[faces]
            normals = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
            reverse = np.sum(normals*hull.equations[:,:3], axis=1) < 0
            faces[reverse] = faces[reverse][:, ::-1]
            convex = trimesh.Trimesh(v, faces, process=False)
            convex.remove_unreferenced_vertices()
            solid(convex)  # fail closed on any malformed tiny hull
            result.append(convex)
            hull_extra += extra
            continue
        if depth > 80 or len(result) + len(stack) > max_pieces:
            raise ValueError('Convex partition exceeded bounded complexity; scene rejected')
        tris = v[f]
        cross = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        area = np.linalg.norm(cross, axis=1)
        normals = cross / np.maximum(area[:, None], 1e-20)
        offsets = np.einsum('ij,ij->i', normals, tris[:, 0])
        keys = np.unique(np.round(np.column_stack((normals, offsets)), 6), axis=0, return_index=True)[1]
        candidates = []
        for j in keys:
            # Elementwise sum avoids a platform-specific strided BLAS warning.
            d = np.sum(v * normals[j], axis=1) - offsets[j]
            pos, neg = np.count_nonzero(d > plane_sign_tol_mm), np.count_nonzero(d < -plane_sign_tol_mm)
            if pos and neg:
                candidates.append((min(pos, neg) * area[j], normals[j], offsets[j]))
        if not candidates:
            raise ValueError(f'Unresolved concavity {extra:.6g} mm3; scene rejected')
        candidate = max(candidates, key=lambda c: c[0])
        aa, bb = obj.split_by_plane(candidate[1], candidate[2])
        for component in aa.decompose() + bb.decompose():
            stack.append((component, depth + 1))
    if not result:
        raise ValueError('Empty collision decomposition')
    return result, dict(discarded_sliver_mm3=dropped, summed_hull_extra_mm3=hull_extra)


def union_solids(parts):
    result = M.Manifold.batch_boolean([solid(p) for p in parts], M.OpType.Add)
    if result.status() != M.Error.NoError or result.volume() <= 0:
        raise ValueError(f'Invalid collision union: {result.status()}')
    return result


def holder_wall_cover(source):
    """Overlapping convex walls using the original holder's cavity face planes.

    The vertical slot and chamfer share continuous sections. Splitting them at
    z=-0.5 creates a buried horizontal contact face. Every section is checked
    against the actual source; this is neither a reconstructed capsule nor a
    contact filter. Only this audited original-holder topology is supported.
    """
    levels = np.unique(np.round(mesh_of(source).vertices[:, 2], 5))
    if not np.array_equal(levels, [-1.5, -.5, 0., 5.]):
        raise ValueError('Unsupported holder height topology for wall cover')
    plane_groups, raised, records = [], [], []
    for za, zb in zip(levels[:-1], levels[1:]):
        layer = source.trim_by_plane([0, 0, 1], float(za)).trim_by_plane([0, 0, -1], float(-zb))
        if za >= 0:
            raised, _ = convex_partition([layer], max_hull_extra_mm3=1e-6,
                                         min_piece_mm3=1e-7, plane_sign_tol_mm=1e-6)
            continue
        mesh = mesh_of(layer)
        centers, normals = mesh.triangles_center, mesh.face_normals
        radial = centers.copy()
        radial[:, 0] -= X_SLOT
        radial[:, 1] -= np.clip(radial[:, 1], -30, 30)
        radial[:, 2] = 0
        inward = ((radial * normals).sum(axis=1) < -.01) & (abs(centers[:, 0]-X_SLOT) < 8) & (abs(centers[:, 1]) < 37)
        planes = np.column_stack((normals[inward], (normals[inward]*centers[inward]).sum(axis=1)))
        _, indices = np.unique(np.round(planes, 6), axis=0, return_index=True)
        planes = planes[indices]
        if len(planes) == 0:
            raise ValueError('No source cavity planes for holder wall cover')
        plane_groups.append(planes)
        records.append(dict(z_mm=[float(za), float(zb)], cavity_planes=planes.tolist()))
    plate = source.trim_by_plane([0, 0, -1], 0)
    outer = solid(mesh_of(plate).convex_hull)
    parts, pairs = [], []
    for ti, t in enumerate(plane_groups[1]):
        tu = t[:2]/np.linalg.norm(t[:2])
        for vi, v in enumerate(plane_groups[0]):
            vu = v[:2]/np.linalg.norm(v[:2])
            if tu @ vu < math.cos(math.radians(7)):
                continue
            wall = outer.trim_by_plane(-t[:3], float(-t[3])).trim_by_plane(-v[:3], float(-v[3]))
            if wall.volume() < 1e-8:
                continue
            hull = mesh_of(wall).convex_hull
            extra = (solid(hull)-plate).volume()
            if extra > 1e-5:
                raise ValueError('Holder wall extends outside source solid')
            parts.append(hull)
            pairs.append(dict(chamfer_plane=ti, vertical_plane=vi, extra_mm3=extra))
    parts.extend(raised)
    cover = union_solids(parts)
    extra, missing = (cover-source).volume(), (source-cover).volume()
    if extra > 1e-4 or missing > 1e-4:
        raise ValueError('Holder wall cover does not reproduce source exterior')
    return parts, dict(method='Continuous source-plane convex holder wall cover',
                       source_layers=records, wall_pairs=pairs,
                       strict_extra_mm3=extra, strict_missing_mm3=missing)


def box_probe(center, size):
    m = trimesh.creation.box(size)
    m.apply_translation(center)
    return solid(m)


def local_clearance_probes(sources, collisions):
    """Audit functional hollows and the 0.2mm collar gap, not volume alone."""
    records = []
    def check(name, part, probe, expected):
        truth = (sources[part] ^ probe).volume()
        approx = (collisions[part] ^ probe).volume()
        # Local tolerance 0.001 mm3, with >95% material where a rim is expected.
        ok = abs(approx - truth) <= .001 and (truth < .001 if expected == 'clear' else truth > .95 * probe.volume())
        records.append(dict(name=name, source_overlap_mm3=truth, collision_overlap_mm3=approx,
                            probe_volume_mm3=probe.volume(), expected=expected, passed=bool(ok)))
    for xsign in (-1, 1):
        # Side gap interior: collar half-width 4.35, throat half-width 4.55.
        check(f'guide_collar_gap_{xsign}', 'guide',
              box_probe([X_SLOT + xsign * 4.45, 0, .15], [.10, 58, .10]), 'clear')
        check(f'carrier_top_rim_{xsign}', 'carrier',
              box_probe([xsign * 4.15, 0, -.20], [.10, 58, .10]), 'material')
    check('guide_lower_channel', 'guide', box_probe([X_SLOT, 0, 10], [9.09, 59, 19]), 'clear')
    check('carrier_beveled_paper_mouth', 'carrier', box_probe([0, 0, -.20], [7.50, 59, .05]), 'clear')
    check('carrier_lower_paper_hole', 'carrier', box_probe([0, 0, -2], [5.50, 59, .10]), 'clear')
    for z in [0, .3, 1, 4, 10, 20, 30, 40, 56]:
        moving = sources['carrier'].translate([X_SLOT, 0, z])
        for part in ('guide', 'holder'):
            truth = (sources[part] ^ moving).volume()
            approx = (collisions[part] ^ moving).volume()
            records.append(dict(name=f'aligned_carrier_descent_z{z}_{part}', source_overlap_mm3=truth,
                                collision_overlap_mm3=approx, expected='clear', passed=bool(truth < .005 and approx < .005)))
    return records


def prepare_geometry(cad, holder_path, out, *, holder_cover='partition'):
    out.mkdir(parents=True, exist_ok=False)
    sources, collisions, records, files = {}, {}, {}, {}
    paths = dict(guide=cad/'guide_PROTOTYPE.stl', carrier=cad/'carrier_PROTOTYPE.stl', holder=holder_path)
    for name, path in paths.items():
        mesh = trimesh.load_mesh(path)
        if not mesh.is_watertight or not mesh.is_volume:
            raise ValueError(f'{name}: source mesh is not a closed positive solid')
        source = solid(mesh)
        if name != 'carrier':
            source = clip_x(source)
        sources[name] = source
        initial = guide_sectors(source) if name == 'guide' else [source]
        pieces, stats = (holder_wall_cover(source) if name == 'holder' and holder_cover == 'continuous'
                         else convex_partition(initial))
        collision = union_solids(pieces)
        collisions[name] = collision
        extra, missing = (collision - source).volume(), (source - collision).volume()
        record = dict(path=str(path), sha256=sha(path), original_bounds_mm=mesh.bounds.tolist(),
                      original_volume_mm3=float(mesh.volume), retained_volume_mm3=source.volume(),
                      collision_volume_mm3=collision.volume(), convex_pieces=len(pieces),
                      extra_mm3=extra, missing_mm3=missing, **stats)
        record['passed'] = bool(extra < .5 and missing < .05 and abs(collision.volume()-source.volume()) < .5)
        records[name] = record
        partdir = out/name
        partdir.mkdir()
        files[name] = []
        for i, piece in enumerate(pieces):
            dest = partdir/f'{i:04d}.obj'
            piece.export(dest)
            files[name].append(dest)
    probes = local_clearance_probes(sources, collisions)
    audit = dict(status='pass' if all(r['passed'] for r in records.values()) and all(p['passed'] for p in probes) else 'fail',
                 parts=records, local_probes=probes, retained_static_corridor_x_mm=list(ROI_X),
                 collision_scale_to_m=.001, rotation='Source assembly coordinates unchanged',
                 assumptions=['Guide and original holder are fixed rigid fixtures.',
                              'Only first-funnel x corridor is collision modeled; leaving it fails.',
                              'No trough, robot, grasp, paper, fluid, or visual controller is modeled.'],
                 holder_cover=holder_cover,
                 method='Source-STL collision covers with Boolean difference and local clearance checks; cover mode recorded per part')
    (out/'geometry-audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    if audit['status'] != 'pass':
        raise ValueError('Functional collision-geometry audit failed; no simulation permitted')
    return files, audit


@dataclass(frozen=True)
class DropCase:
    name: str
    x_mm: float = 0
    y_mm: float = 0
    yaw_deg: float = 0
    friction: float = .4
    timestep: float = .0001
    solver_iterations: int = 100
    contact_timeconstant_s: float = .0002
    disabled_gravity: bool = False
    multiccd: bool = True
    start_z_mm: float = 56


def cases():
    return [DropCase('nominal'), DropCase('x_plus_3mm', x_mm=3), DropCase('x_minus_3mm', x_mm=-3),
            DropCase('y_plus_3mm', y_mm=3), DropCase('yaw_plus_3deg', yaw_deg=3),
            DropCase('offset_yaw', x_mm=3, y_mm=2, yaw_deg=3),
            DropCase('zero_friction', friction=0), DropCase('strong_friction', friction=2),
            DropCase('half_timestep', timestep=.00005), DropCase('solver_200', solver_iterations=200), DropCase('softer_contact', contact_timeconstant_s=.0004),
            DropCase('disabled_gravity', disabled_gravity=True)]



def release_search_cases():
    """Predeclared geometry-constrained release/compliance hypotheses.

    These starts are inside the guide, often already through the holder slot.
    They test passive final seating, never full-height funnel capture or how a
    robot achieves the initial pose. Time constants are unmeasured contact
    proxies, not fitted plastic material properties.
    """
    return [DropCase(f'guided_z{z:g}_tau{tau:g}', start_z_mm=z, contact_timeconstant_s=tau)
            for z in (1., 2., 4., 8., 16., 28.)
            for tau in (.0004, .0008, .0016, .0032)]



def release_refine_cases():
    """Bounded refinement around the first positive-clearance search pass."""
    return [DropCase(f'refine_z{z:g}_tau{tau:g}', start_z_mm=z, contact_timeconstant_s=tau)
            for z in (1., 1.5) for tau in (.0009, .0010, .0012, .0014)]


def guided_control_cases(release_height_mm, contact_timeconstant_s):
    """Frozen controls for a selected positive-clearance guided release."""
    from dataclasses import replace
    base=DropCase('guided_nominal', start_z_mm=release_height_mm,
                  contact_timeconstant_s=contact_timeconstant_s)
    return [base,
            replace(base,name='guided_height_minus_0p25mm',start_z_mm=release_height_mm-.25),
            replace(base,name='guided_height_plus_0p25mm',start_z_mm=release_height_mm+.25),
            replace(base,name='guided_height_plus_0p5mm',start_z_mm=release_height_mm+.5),
            replace(base,name='guided_contact_minus10pct',contact_timeconstant_s=contact_timeconstant_s*.9),
            replace(base,name='guided_contact_plus10pct',contact_timeconstant_s=contact_timeconstant_s*1.1),
            replace(base,name='guided_x_plus_0p15mm',x_mm=.15),
            replace(base,name='guided_x_minus_0p15mm',x_mm=-.15),
            replace(base,name='guided_y_plus_0p15mm',y_mm=.15),
            replace(base,name='guided_y_minus_0p15mm',y_mm=-.15),
            replace(base,name='guided_yaw_plus_0p1deg',yaw_deg=.1),
            replace(base,name='guided_yaw_minus_0p1deg',yaw_deg=-.1),
            replace(base,name='guided_offset_yaw',x_mm=.10,y_mm=.10,yaw_deg=.08),
            replace(base,name='guided_zero_friction',friction=0),
            replace(base,name='guided_strong_friction',friction=2),
            replace(base,name='guided_half_timestep',timestep=.00005),
            replace(base,name='guided_solver_200',solver_iterations=200),
            replace(base,name='guided_stiffer_contact',contact_timeconstant_s=contact_timeconstant_s/2),
            replace(base,name='guided_softer_contact',contact_timeconstant_s=contact_timeconstant_s*2),
            replace(base,name='guided_disabled_gravity',disabled_gravity=True),
            replace(base,name='guided_known_intersection',x_mm=1.0)]


def guided_confirmation_cases(release_height_mm, contact_timeconstant_s):
    """Four fresh samples of a bounded already-guided envelope, not coverage proof."""
    return [DropCase(name,x_mm=x,y_mm=y,yaw_deg=yaw,start_z_mm=release_height_mm+dz,
                     contact_timeconstant_s=contact_timeconstant_s)
            for name,x,y,yaw,dz in [
                ('guided_fresh_a', .075, .075, .04, -.15),
                ('guided_fresh_b', -.075, -.075, -.04, .15),
                ('guided_fresh_c', .040, -.040, .06, .10),
                ('guided_fresh_d', -.040, .040, -.06, -.10)]]


def words(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def build_scene(cad, holder_path, collision_files, case, out):
    root = ET.Element('mujoco', model='G4_passive_carrier_diagnostic')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    option=ET.SubElement(root, 'option', timestep=str(case.timestep), gravity='0 0 0' if case.disabled_gravity else '0 0 -9.81',
                  integrator='implicitfast', cone='elliptic', iterations=str(case.solver_iterations), tolerance='1e-10', noslip_iterations='0')
    # Opt-in single-contact mode is supported by a saved two-piece collider
    # discrepancy reproducer; shapes/material/gates are unchanged.
    ET.SubElement(option,'flag',nativeccd='enable',multiccd='enable' if case.multiccd else 'disable')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction=words([case.friction, .001 if case.friction else 0, .0001 if case.friction else 0]),
                  condim='4', solref=f'{case.contact_timeconstant_s} 1', solimp='.95 .99 .0001', margin='0', gap='0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'map', znear='.0001')
    asset, world = ET.SubElement(root, 'asset'), ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -.2 .3', dir='0 0 -1', diffuse='.8 .8 .8', ambient='.5 .5 .5')
    ET.SubElement(world, 'geom', name='visual_floor', type='plane', size='.3 .3 .001', pos='0 0 -.035',
                  rgba='.88 .88 .86 1', contype='0', conaffinity='0')
    carrier = ET.SubElement(world, 'body', name='carrier', pos=words([(X_SLOT+case.x_mm)/1000, case.y_mm/1000, case.start_z_mm/1000]),
                            quat=words([math.cos(math.radians(case.yaw_deg)/2), 0, 0, math.sin(math.radians(case.yaw_deg)/2)]))
    ET.SubElement(carrier, 'freejoint', name='carrier_free')
    # Hypothetical 2.5g mass; uniform solid source inertia scaled to that mass.
    mesh = trimesh.load_mesh(cad/'carrier_PROTOTYPE.stl')
    inertia = mesh.moment_inertia * (.0025/mesh.mass) * 1e-6
    ET.SubElement(carrier, 'inertial', pos=words(mesh.center_mass/1000), mass='.0025',
                  fullinertia=words([inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]]))
    colors = dict(guide='.76 .57 .18 .7', holder='.60 .63 .68 1', carrier='.10 .31 .52 1')
    for name, files in collision_files.items():
        body = carrier if name == 'carrier' else world
        path = holder_path if name == 'holder' else cad/f'{name}_PROTOTYPE.stl'
        ET.SubElement(asset, 'mesh', name=name+'_visual', file=str(path), scale='.001 .001 .001')
        ET.SubElement(body, 'geom', name=name+'_visual', type='mesh', mesh=name+'_visual',
                      contype='0', conaffinity='0', group='1', mass='0', rgba=colors[name])
        for i, file in enumerate(files):
            meshname = f'{name}_{i:04d}'
            ET.SubElement(asset, 'mesh', name=meshname, file=str(file), scale='.001 .001 .001')
            ET.SubElement(body, 'geom', name=meshname, type='mesh', mesh=meshname, group='3', mass='0',
                          contype='1' if name == 'carrier' else '2', conaffinity='2' if name == 'carrier' else '1')
    pos = np.array([-.17, -.21, .19]); target = np.array([-.044, 0, .018])
    back = pos-target;back /= np.linalg.norm(back)
    right = np.cross([0,0,1], back);right /= np.linalg.norm(right); up=np.cross(back,right)
    ET.SubElement(world,'camera',name='diagnostic',pos=words(pos),xyaxes=words(np.r_[right,up]),fovy='33')
    ET.indent(root)
    ET.ElementTree(root).write(out/'scene.xml', encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(out/'scene.xml'))
    if bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)):
        raise ValueError('Native CCD must remain enabled')
    if bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)) == case.multiccd:
        raise ValueError('Compiled contact pipeline does not match declared case')
    if np.any(model.geom_margin != 0):
        raise ValueError('Contact margins must remain zero for audited native pipeline')
    if model.nu != 0 or model.nq != 7:
        raise ValueError('Diagnostic must contain only a free carrier and no actuators')
    return model


THRESHOLDS = dict(seat_z_error_mm=.35, xy_error_mm=.4, tilt_deg=2, linear_speed_mm_s=1,
                  angular_speed_rad_s=.1, hold_seconds=.5, max_penetration_mm=.1,
                  abort_normal_force_n=2., duration_s=2.)


def evaluate(records, *, stop_reason=None, thresholds=THRESHOLDS, timestep=.001,
             initial_origin_mm=(X_SLOT, 0, 56), initial_yaw_deg=0, carrier_vertices_mm=None):
    """Independently verify release, complete sampled drop, and loaded settle.

    Raw contact wrench/frame and declared source geometry drive scoring. Logged
    summary values cannot supply missing contacts or overwrite force outcomes.
    """
    def fail(reason):
        return dict(status='fail', reason=reason, metrics={})
    if not records or not np.isfinite(timestep) or timestep <= 0:
        return fail('Missing records or invalid declared timestep')
    if carrier_vertices_mm is None:
        # Conservative source-bounds envelope; run_case passes actual vertices.
        carrier_vertices_mm=np.array([[x,y,z] for x in (-4.35,4.35) for y in (-34.35,34.35) for z in (-20.6,-.1)])
    vertices=np.asarray(carrier_vertices_mm, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        return fail('Invalid declared carrier source geometry')
    checked=[]
    scalar_keys=('time_s','tilt_deg','linear_speed_mm_s','angular_speed_rad_s',
                 'normal_force_n','holder_normal_force_n','penetration_mm')
    try:
        for row in records:
            vals=np.array([row[k] for k in scalar_keys],dtype=float)
            origin=np.asarray(row['origin_mm'],dtype=float);quat=np.asarray(row['quat'],dtype=float)
            if origin.shape!=(3,) or quat.shape!=(4,) or not np.isfinite(np.r_[vals,origin,quat]).all():
                return fail('Nonfinite or malformed observation')
            if abs(np.linalg.norm(quat)-1)>1e-6 or np.any(vals<0):
                return fail('Invalid quaternion or negative observation magnitude')
            rot=np.zeros(9);mujoco.mju_quat2Mat(rot,quat);rot=rot.reshape(3,3)
            tilt=math.degrees(math.acos(float(np.clip(rot[2,2],-1,1))))
            if abs(tilt-row['tilt_deg'])>1e-5:
                return fail('Quaternion and tilt disagree')
            world=np.einsum('ij,kj->ki',rot,vertices)+origin
            bounds=np.array([world.min(axis=0),world.max(axis=0)])
            normal=holder=penetration=0.;net=np.zeros(3);holder_up=0.;single=0.
            for c in row['contacts']:
                names=c['geoms'];frame=np.asarray(c['frame'],dtype=float).reshape(3,3)
                wrench=np.asarray(c['wrench_local'],dtype=float);pos=np.asarray(c['position_m'],dtype=float)
                f=float(c['normal_force_n']);pen=float(c['penetration_mm'])
                if len(names)!=2 or wrench.shape!=(6,) or pos.shape!=(3,) or not np.isfinite(np.r_[frame.ravel(),wrench,pos,f,pen]).all():
                    return fail('Malformed raw contact')
                if f<0 or pen<0 or wrench[0]<-1e-9 or not np.allclose(frame@frame.T,np.eye(3),atol=1e-5) or abs(np.linalg.det(frame)-1)>1e-5:
                    return fail('Invalid contact magnitude or coordinate frame')
                is_carrier=[n.startswith('carrier_') for n in names]
                if sum(is_carrier)!=1 or not any(n.startswith(('holder_','guide_')) for n in names):
                    return fail('Raw contact does not belong to carrier and validated fixture')
                if abs(f-max(0.,wrench[0]))>1e-8:
                    return fail('Normal contact force disagrees with raw wrench')
                sign=1. if is_carrier[1] else -1.
                force=sign*np.einsum('ij,i->j',frame,wrench[:3])
                normal+=f;single=max(single,f);net+=force;penetration=max(penetration,pen)
                if any(n.startswith('holder_') for n in names):
                    holder+=f;holder_up+=force[2]
            for key,value in [('normal_force_n',normal),('holder_normal_force_n',holder),('penetration_mm',penetration)]:
                if abs(row[key]-value)>1e-8:
                    return fail('Contact summary disagrees with raw contacts')
            checked.append(dict(row=row,normal=normal,holder=holder,holder_up=holder_up,net=net,
                                penetration=penetration,single=single,count=len(row['contacts']),bounds=bounds))
    except (KeyError,TypeError,ValueError,AttributeError):
        return fail('Required raw evidence is missing or malformed')
    t=np.array([r['row']['time_s'] for r in checked])
    if abs(t[0])>1e-9 or not np.allclose(np.diff(t),timestep,rtol=1e-7,atol=1e-9):
        return fail('Missing or irregular physics-step observations for declared timestep')
    first=checked[0];initial=first['row'];final=checked[-1];last=final['row']
    expected_quat=np.array([math.cos(math.radians(initial_yaw_deg)/2),0,0,math.sin(math.radians(initial_yaw_deg)/2)])
    if (not np.allclose(initial['origin_mm'],initial_origin_mm,atol=1e-6,rtol=0) or
        abs(abs(np.dot(initial['quat'],expected_quat))-1)>1e-6 or initial_origin_mm[2]<1 or
        first['normal']>.001 or first['penetration']>.005 or initial['linear_speed_mm_s']>1e-6 or initial['angular_speed_rad_s']>1e-6):
        return fail('Initial free release does not match declared pose, clearance and zero motion')
    reason=stop_reason
    if any(r['bounds'][0,0]<ROI_X[0] or r['bounds'][1,0]>ROI_X[1] for r in checked):
        reason=reason or 'Whole carrier left source-validated collision corridor'
    if not reason and last['time_s'] < thresholds['duration_s']-1e-7:
        reason='Complete two-second release-to-seat timeline missing'
    if not reason and max(r['penetration'] for r in checked)>thresholds['max_penetration_mm']:
        reason='Penetration accuracy gate exceeded'
    if not reason and max(r['normal'] for r in checked)>thresholds['abort_normal_force_n']:
        reason='Contact diagnostic gate exceeded'
    def seat(r):
        q=r['row']
        return (abs(q['origin_mm'][2])<=thresholds['seat_z_error_mm'] and
                math.hypot(q['origin_mm'][0]-X_SLOT,q['origin_mm'][1])<=thresholds['xy_error_mm'] and
                q['tilt_deg']<=thresholds['tilt_deg'] and q['linear_speed_mm_s']<=thresholds['linear_speed_mm_s'] and
                q['angular_speed_rad_s']<=thresholds['angular_speed_rad_s'] and r['holder']>.005 and r['holder_up']>.005)
    recent=[r for r in checked if r['row']['time_s']>=last['time_s']-thresholds['hold_seconds']-1e-8]
    full_hold=(recent[-1]['row']['time_s']-recent[0]['row']['time_s']>=thresholds['hold_seconds']-1e-7 and all(seat(r) for r in recent))
    if not reason and not full_hold:
        reason='Carrier did not seat and settle continuously for final 0.5 seconds'
    metrics=dict(descent_mm=initial['origin_mm'][2]-last['origin_mm'][2],initial_origin_mm=initial['origin_mm'],
                 final_origin_mm=last['origin_mm'],final_tilt_deg=last['tilt_deg'],
                 final_linear_speed_mm_s=last['linear_speed_mm_s'],final_holder_normal_force_n=final['holder'],
                 max_normal_force_n=max(r['normal'] for r in checked),max_penetration_mm=max(r['penetration'] for r in checked),
                 max_single_contact_n=max(r['single'] for r in checked),max_contact_count=max(r['count'] for r in checked),
                 final_contact_count=final['count'],final_net_contact_force_n=final['net'].tolist(),
                 final_holder_upward_force_n=final['holder_up'],final_carrier_bounds_mm=final['bounds'].tolist(),
                 final_hold_pass=bool(full_hold),observed_seconds=last['time_s'],sample_count=len(records))
    return dict(status='pass' if reason is None else 'fail',reason=reason,metrics=metrics)


def run_case(cad, holder_path, files, case, out, *, render=True):
    out.mkdir()
    model=build_scene(cad,holder_path,files,case,out);data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    carrier_id=model.body('carrier').id
    carrier_vertices=trimesh.load_mesh(cad/'carrier_PROTOTYPE.stl').vertices
    renderer=None; frames=[]; records=[]
    if render:
        renderer=mujoco.Renderer(model, height=720, width=960)
        options=mujoco.MjvOption();options.geomgroup[3]=0
    def sample():
        contacts=[];normal=0.;holder=0.;penetration=0.;net=np.zeros(3)
        for i,c in enumerate(data.contact):
            names=[model.geom(c.geom1).name,model.geom(c.geom2).name]
            force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force)
            f=max(0.,float(force[0]));normal+=f
            if any(n.startswith('holder_') for n in names):holder+=f
            pen=max(0.,float(-c.dist*1000));penetration=max(penetration,pen)
            # Contact normal points geom1->geom2, world force on geom2 is +frame.T@wrench.
            # https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjcontact
            sign=1. if model.geom_bodyid[c.geom2]==carrier_id else -1.
            force_world=sign*np.einsum('ij,i->j',c.frame.reshape(3,3),force[:3]);net+=force_world
            contacts.append(dict(geoms=names,position_m=c.pos.tolist(),normal_force_n=f,penetration_mm=pen,
                                 frame=c.frame.tolist(),wrench_local=force.tolist(),force_on_carrier_n=force_world.tolist()))
        world=np.einsum('ij,kj->ki',data.xmat[carrier_id].reshape(3,3),carrier_vertices)+data.xpos[carrier_id]*1000
        bounds=np.array([world.min(axis=0),world.max(axis=0)])
        r=dict(time_s=float(data.time),origin_mm=(data.xpos[carrier_id]*1000).tolist(),quat=data.xquat[carrier_id].tolist(),
               tilt_deg=math.degrees(math.acos(float(np.clip(data.xmat[carrier_id,8],-1,1)))),
               linear_speed_mm_s=float(np.linalg.norm(data.qvel[:3])*1000),angular_speed_rad_s=float(np.linalg.norm(data.qvel[3:])),
               normal_force_n=normal,holder_normal_force_n=holder,penetration_mm=penetration,contacts=contacts,
               carrier_bounds_mm=bounds.tolist(),net_contact_force_n=net.tolist())
        records.append(r)
        return r
    def capture(label):
        if renderer:
            renderer.update_scene(data,camera='diagnostic',scene_option=options)
            im=Image.fromarray(renderer.render().copy());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,960,72),fill='white')
            draw.text((12,8),f'G4 GRAVITY CONTACT DIAGNOSTIC | {case.name} | t={data.time:.3f}s | ~5x slow',fill='black')
            draw.text((12,29),label+' | fixed guide/holder; privileged reset; no robot/paper',fill='black')
            scope='FULL-HEIGHT FUNNEL CAPTURE' if case.start_z_mm>=54.6 else 'ALREADY-GUIDED SEATING; preceding lowering/grasp release unmodeled'
            draw.text((12,50),f'{scope} | release offset {case.start_z_mm:g} mm above nominal seat',fill='black')
            frames.append(im)
    initial=sample();capture('INITIAL STATE')
    reason=None
    if initial['penetration_mm']>.005:reason='Initial carrier intersection exceeds 0.005mm; not run'
    if initial['normal_force_n']>.001:reason='Initial loaded contact; not a free release'
    for i in range(round(THRESHOLDS['duration_s']/case.timestep)):
        if reason:break
        mujoco.mj_step(model,data);mujoco.mj_forward(model,data);r=sample()
        if r['penetration_mm']>THRESHOLDS['max_penetration_mm']:reason='Simulation penetration exceeds 0.1mm accuracy gate'
        if r['normal_force_n']>THRESHOLDS['abort_normal_force_n']:reason='Simulation contact force exceeds 2N diagnostic gate; no physical threshold implied'
        if r['carrier_bounds_mm'][0][0]<ROI_X[0] or r['carrier_bounds_mm'][1][0]>ROI_X[1]:reason='Whole carrier left validated collision corridor'
        if r['origin_mm'][2]<-1:reason='Carrier fell below expected holder seat'
        if not np.isfinite(data.qpos).all():reason='Nonfinite dynamics'
        if i%max(1,round(.01/case.timestep))==0:capture('PASSIVE GRAVITY; every physics step logged')
    result=evaluate(records,stop_reason=reason,timestep=case.timestep,
                    initial_origin_mm=[X_SLOT+case.x_mm,case.y_mm,case.start_z_mm],initial_yaw_deg=case.yaw_deg,
                    carrier_vertices_mm=carrier_vertices);capture(result['status'].upper()+': '+(result['reason'] or 'seat + settle complete'))
    if renderer:renderer.close()
    if frames:frames[0].save(out/'timeline.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
    with (out/'observations.jsonl').open('w') as f:
        for r in records:f.write(json.dumps(r)+'\n')
    result.update(name=case.name,parameters=asdict(case),initial_intersection_mm=initial['penetration_mm'],
                  scene=str(out/'scene.xml'),observations=str(out/'observations.jsonl'),
                  contact_generation=dict(nativeccd=True,multiccd=case.multiccd,geom_margin_m=0.,
                                          ccd_iterations=int(model.opt.ccd_iterations),ccd_tolerance_m=float(model.opt.ccd_tolerance)),
                  gif=str(out/'timeline.gif') if frames else None,frames=len(frames),
                  release_scope=('Full-height funnel capture' if case.start_z_mm>=54.6 else 'Already-guided positive-clearance passive seating; robot placement unmodeled'),
                  evidence_boundary='Privileged passive rigid-body diagnostic. Not visual control, robot training, assembly or physical success.')
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
