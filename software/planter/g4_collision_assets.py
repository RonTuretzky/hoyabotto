"""Full G4 source collision assets, in original assembly axes and millimetres.

This module builds geometry, not a robot policy. Boolean volume and functional
probes qualify the exported cover for scene construction; they do not certify
every contact normal, material, printed fit, or an assembly task.
"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import manifold3d as M
import mujoco
import numpy as np
import trimesh
from scipy.spatial import ConvexHull

from planter.g4_carrier import (box_probe, convex_partition, guide_sectors,
                               holder_wall_cover, mesh_of, sha, solid, union_solids)

SLOTS_MM = (-45., -15., 15., 45.)
SCHEMA = "g4-source-collision-bundle-v1"
LIMITS = dict(extra_mm3=.5, missing_mm3=.05, volume_difference_mm3=.5,
              local_difference_mm3=.001, aligned_overlap_mm3=.005)


def precise_partition(initial):
    """Double-precision source cuts avoid float32 sliver/concavity ambiguity.

    This is the same source-face BSP principle as the baseline. Mesh64 retains
    newly cut vertex precision; no source face is deleted or contact filtered.
    """
    stack, pieces, dropped, total_extra = list(initial), [], 0., 0.
    while stack:
        obj = stack.pop()
        volume = obj.volume()
        if volume < 1e-8:
            dropped += max(0., volume)
            continue
        mm = obj.to_mesh64()
        vertices = np.asarray(mm.vert_properties)[:, :3]
        faces = np.asarray(mm.tri_verts)
        hull = ConvexHull(vertices)
        extra = max(0., hull.volume-volume)
        if extra <= 1e-6:
            triangles = hull.simplices.copy()
            vv = vertices[triangles]
            normal = np.cross(vv[:, 1]-vv[:, 0], vv[:, 2]-vv[:, 0])
            flip = (normal*hull.equations[:, :3]).sum(axis=1) < 0
            triangles[flip] = triangles[flip][:, ::-1]
            mesh = trimesh.Trimesh(vertices, triangles, process=False)
            mesh.remove_unreferenced_vertices()
            check = M.Manifold(M.Mesh64(np.array(mesh.vertices, dtype=np.float64, order='C', copy=True),
                                        np.array(mesh.faces, dtype=np.uint64, order='C', copy=True)))
            _valid(check, 'Double-precision convex cell')
            pieces.append(mesh)
            total_extra += extra
            continue
        if len(pieces)+len(stack) > 8000:
            raise ValueError('Double-precision partition complexity limit')
        triangles = vertices[faces]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        area = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(area[:, None], 1e-20)
        offsets = (normals*triangles[:, 0]).sum(axis=1)
        indices = np.unique(np.round(np.c_[normals, offsets], 10), axis=0, return_index=True)[1]
        candidates = []
        for index in indices:
            d = (vertices*normals[index]).sum(axis=1)-offsets[index]
            positive, negative = (d > 1e-8).sum(), (d < -1e-8).sum()
            if positive and negative:
                candidates.append((min(positive, negative)*area[index], index))
        if not candidates:
            raise ValueError(f'Unresolved precise source concavity {extra} mm3')
        index = max(candidates)[1]
        aa, bb = obj.split_by_plane(normals[index], float(offsets[index]))
        stack.extend(aa.decompose()+bb.decompose())
    if not pieces:
        raise ValueError('Empty double-precision partition')
    return pieces, dict(discarded_sliver_mm3=dropped, summed_hull_extra_mm3=total_extra,
                        construction_precision='Manifold Mesh64 and float64 source-cut vertices', max_cell_hull_extra_mm3=1e-6)


def _slab(obj, axis, lo, hi):
    n = np.eye(3)[axis]
    return obj.trim_by_plane(n, float(lo)).trim_by_plane(-n, float(-hi))


def _valid(obj, label):
    if obj.status() != M.Error.NoError or not np.isfinite(obj.volume()) or obj.volume() <= 0:
        raise ValueError(f"{label}: invalid or empty Boolean solid")


def full_guide_cover(source, *, precise=False):
    """Keep every corridor plus the actual upper handles, without clipping CAD."""
    parts, stats = [], []
    boundaries = (-75., -30., 0., 30., 75.)
    for x, a, b in zip(SLOTS_MM, boundaries[:-1], boundaries[1:]):
        corridor = _slab(source, 0, a, b).translate([-45-x, 0, 0])
        seeds = guide_sectors(corridor)
        upper = corridor.trim_by_plane([0, 0, 1], 34.01)
        if upper.volume() > 1e-8:
            seeds.extend(upper.decompose())
        pieces, audit = precise_partition(seeds) if precise else convex_partition(seeds)
        for piece in pieces:
            piece.apply_translation([x+45, 0, 0])
        parts.extend(pieces)
        stats.append(dict(slot_x_mm=x, corridor_x_mm=[a, b], pieces=len(pieces), **audit))
    return parts, dict(method="All four source-cut guide corridors and full upper handles",
                       sectors=stats, artificial_seams_mm=dict(x=list(boundaries[1:-1]), z=[4, 22, 34.01]),
                       contact_limitation="Internal cover boundaries remain; union equality is not contact-normal equivalence.")


def full_holder_cover(source):
    """Use continuous source-plane walls at every slot; retain original ends."""
    parts, stats = [], []
    boundaries = (-61., -30., 0., 30., 61.)
    for x, a, b in zip(SLOTS_MM, boundaries[:-1], boundaries[1:]):
        corridor = _slab(source, 0, a, b).translate([-45-x, 0, 0])
        pieces, audit = holder_wall_cover(corridor)
        for piece in pieces:
            piece.apply_translation([x+45, 0, 0])
        parts.extend(pieces)
        stats.append(dict(slot_x_mm=x, corridor_x_mm=[a, b], pieces=len(pieces), **audit))
    ends = [source.trim_by_plane([-1, 0, 0], 61), source.trim_by_plane([1, 0, 0], 61)]
    for end in ends:
        if end.volume() > 1e-8:
            pieces, audit = convex_partition(end.decompose(), max_pieces=5000,
                                             max_hull_extra_mm3=.005,
                                             min_piece_mm3=.0001, plane_sign_tol_mm=.001)
            parts.extend(pieces)
            stats.append(dict(region="Original end, rim and mounting detail", pieces=len(pieces), **audit))
    return parts, dict(method="Four continuous source-plane slot walls plus original end detail",
                       regions=stats, artificial_seams_x_mm=list(boundaries),
                       contact_limitation="Approximate convex contact normals; known near-edge micrometre neighbourhood sensitivity remains.")


def full_trough_cover(source):
    """Actual open reservoir walls and floor, split at the source floor plane."""
    seeds = []
    for region in (source.trim_by_plane([0, 0, -1], 21.5), source.trim_by_plane([0, 0, 1], -21.5)):
        if region.volume() > 1e-8:
            seeds.extend(region.decompose())
    # Shared BSP uses float32 source vertices. Large floor pieces can have
    # sub-.005 mm3 hull-volume roundoff; the final Boolean/local audits remain.
    parts, stats = convex_partition(seeds, max_pieces=5000, max_hull_extra_mm3=.005,
                                    min_piece_mm3=.0001, plane_sign_tol_mm=.001)
    return parts, dict(method="Source-face convex partition of reservoir floor and complete wall/rim", **stats,
                       artificial_seam_z_mm=-21.5,
                       contact_limitation="Floor-wall decomposition seam and convex edge contacts require dynamic source-normal audit.")


def full_holder_angular_cover(source, *, continuous_rims=False):
    """Source-cut angular cells reduce redundant flat support constraints.

    Unlike the historical z-layer BSP, each slot wall is continuous from the
    bottom of the plate to its top. Source Boolean audits still gate export.
    This alternate geometry must be compared at loaded contact states.
    """
    seeds, detail = [], []
    boundaries = (-61., -30., 0., 30., 61.)
    for x, lo, hi in zip(SLOTS_MM, boundaries[:-1], boundaries[1:]):
        corridor = _slab(source, 0, lo, hi)
        plate = corridor.trim_by_plane([0, 0, -1], 0)
        central = _slab(plate, 1, -30, 30)
        seeds.extend(central.split_by_plane([1, 0, 0], x))
        for cy, lo_angle, hi_angle in ((30, 0, 180), (-30, 180, 360)):
            # The union of vertex rays from the source's 12deg and 11.25deg
            # rings includes their transitions without unnecessary .75deg cells.
            angles = np.unique(np.r_[np.arange(lo_angle, hi_angle+.01, 12),
                                     np.arange(lo_angle, hi_angle+.01, 11.25)])
            for a0, a1 in zip(angles[:-1], angles[1:]):
                t0, t1 = np.radians([a0, a1])
                n0, n1 = np.array([-np.sin(t0), np.cos(t0), 0]), np.array([np.sin(t1), -np.cos(t1), 0])
                origin = np.array([x, cy, 0])
                seeds.append(plate.trim_by_plane(n0, float(n0@origin)).trim_by_plane(n1, float(n1@origin)))
        if not continuous_rims:
            detail.extend(corridor.trim_by_plane([0, 0, 1], 0).decompose())
    for end in [source.trim_by_plane([-1, 0, 0], 61), source.trim_by_plane([1, 0, 0], 61)]:
        detail.extend((end.trim_by_plane([0, 0, -1], 0) if continuous_rims else end).decompose())
    parts, stats = precise_partition(seeds)
    rim_stats = None
    if continuous_rims:
        raised = source.trim_by_plane([0, 0, 1], 0)
        # Original holder has a1.5mm outer rim: inner Y=±40mm,
        # outer Y=±41.5mm, top Z=5mm. Preserve these full source strips.
        rims = [raised.trim_by_plane([0, -1, 0], 40), raised.trim_by_plane([0, 1, 0], 40)]
        rim_parts, rim_stats = precise_partition(rims)
        parts.extend(rim_parts)
        detail.extend(_slab(raised, 1, -40, 40).decompose())
    ends, end_stats = convex_partition(detail, max_pieces=3000)
    parts.extend(ends)
    return parts, dict(method='Nonoverlapping source-cut angular cells, continuous plate height',
                       angular_rays_deg='Union of original 12 and 11.25 degree ring vertex rays', slot_partition=stats, detail_partition=end_stats,
                       continuous_front_back_rims=continuous_rims, rim_partition=rim_stats,
                       contact_limitation='Artificial radial seams and numerical hull approximation remain; loaded source-normal audit required.')


def full_guide_interface_cover(source):
    """Keep the actual lower front/back flange bars continuous across slots."""
    base = source.trim_by_plane([0, 0, -1], -4)
    bars = [base.trim_by_plane([0, -1, 0], 36.9), base.trim_by_plane([0, 1, 0], 36.9)]
    # Clip the previously audited convex cover instead of re-triangulating
    # the full Boolean remainder. Convex halfspace clipping cannot add hull
    # fill and retains the known upper funnel pieces byte-for-byte in memory.
    original, audit = full_guide_cover(source)
    parts = []
    for piece in original:
        if piece.bounds[0, 2] >= 4-1e-8:
            parts.append(piece)
            continue
        obj = M.Manifold(M.Mesh64(np.array(piece.vertices, dtype=np.float64, order='C', copy=True),
                                 np.array(piece.faces, dtype=np.uint64, order='C', copy=True)))
        regions = [_slab(obj.trim_by_plane([0, 0, -1], -4), 1, -36.9, 36.9)]
        if piece.bounds[1, 2] > 4+1e-8: regions.append(obj.trim_by_plane([0, 0, 1], 4))
        for clipped in regions:
            if clipped.volume() <= 1e-8: continue
            _valid(clipped, 'Clipped convex guide cell')
            mm = clipped.to_mesh64()
            parts.append(trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False))
    bar_parts, bar_audit = precise_partition(bars)
    parts.extend(bar_parts)
    return parts, dict(method='Source-cut full guide with continuous front/back lower flange bars',
                       remainder=audit, continuous_bar_partition=bar_audit,
                       source_bar_region='Z<=4mm and |Y|>=36.9mm; original outside edge remains±39.9mm',
                       contact_limitation='Other internal cover boundaries remain; loaded whole-source normal audits still required')


def audit_union(source, pieces):
    _valid(source, "source")
    if not pieces:
        raise ValueError("Empty collision cover")
    # Exported OBJ coordinates are double precision. Rounding the continuous
    # long bars back to float32 before the Boolean audit can spuriously drop
    # an unrelated1.792mm3 upper funnel cell from the union. Audit the actual
    # exported coordinate precision instead, retaining all previous gates.
    solids = []
    for piece in pieces:
        obj = M.Manifold(M.Mesh64(np.array(piece.vertices, dtype=np.float64, order='C', copy=True),
                                 np.array(piece.faces, dtype=np.uint64, order='C', copy=True)))
        _valid(obj, 'Export-precision collision cell')
        solids.append(obj)
    collision = M.Manifold.batch_boolean(solids, M.OpType.Add)
    _valid(collision, "collision")
    extra_obj, missing_obj = collision-source, source-collision
    if extra_obj.status() != M.Error.NoError or missing_obj.status() != M.Error.NoError:
        raise ValueError("Boolean difference error")
    extra, missing = extra_obj.volume(), missing_obj.volume()
    record = dict(source_volume_mm3=source.volume(), collision_volume_mm3=collision.volume(),
                  extra_mm3=extra, missing_mm3=missing, convex_pieces=len(pieces),
                  boolean_status="NoError", union_precision='Float64 OBJ coordinates via Manifold Mesh64')
    record['passed'] = bool(extra < LIMITS['extra_mm3'] and missing < LIMITS['missing_mm3'] and
                            abs(collision.volume()-source.volume()) < LIMITS['volume_difference_mm3'])
    return collision, record


def functional_probes(sources, collisions):
    """Current Boolean probes of all four actual slots, thin rims and reservoir."""
    records = []

    def check(name, part, center, size, expected, slot=None):
        probe = box_probe(center, size)
        truth, approx = (sources[part] ^ probe).volume(), (collisions[part] ^ probe).volume()
        ok = abs(approx-truth) <= LIMITS['local_difference_mm3']
        ok &= truth < .001 if expected == 'clear' else truth > .95*probe.volume()
        records.append(dict(name=name, part=part, slot_x_mm=slot, center_mm=center, size_mm=size,
                            expected=expected, source_overlap_mm3=truth, collision_overlap_mm3=approx,
                            probe_volume_mm3=probe.volume(), passed=bool(ok)))

    for x in SLOTS_MM:
        for sign in (-1, 1):
            check(f"slot{x:g}_collar_gap_{sign}", 'guide', [x+sign*4.45, 0, .15], [.10, 58, .10], 'clear', x)
            check(f"slot{x:g}_holder_wall_{sign}", 'holder', [x+sign*4.15, 0, -1], [.10, 58, .10], 'material', x)
        check(f"slot{x:g}_guide_lower_channel", 'guide', [x, 0, 10], [9.09, 59, 19], 'clear', x)
        check(f"slot{x:g}_holder_slot", 'holder', [x, 0, -1], [7.09, 59, .1], 'clear', x)
        check(f"slot{x:g}_water_tail_corridor", 'trough', [x, 0, -12], [8.8, 68.8, 17], 'clear', x)
        for z in (0., .3, 1., 4., 10., 20., 30., 40., 56.):
            moving_source = sources['carrier'].translate([x, 0, z])
            moving_collision = collisions['carrier'].translate([x, 0, z])
            for part in ('guide', 'holder', 'trough'):
                truth = (sources[part] ^ moving_source).volume()
                approx = (collisions[part] ^ moving_collision).volume()
                records.append(dict(name=f"slot{x:g}_aligned_descent_z{z:g}_{part}", part=part, slot_x_mm=x,
                                    carrier_translation_mm=[x, 0, z], expected='clear', source_overlap_mm3=truth,
                                    collision_overlap_mm3=approx,
                                    passed=bool(truth < LIMITS['aligned_overlap_mm3'] and approx < LIMITS['aligned_overlap_mm3'])))
    for sign in (-1, 1):
        check(f"carrier_top_rim_{sign}", 'carrier', [sign*4.15, 0, -.2], [.1, 58, .1], 'material')
        check(f"trough_long_wall_{sign}", 'trough', [0, sign*41.5, -10], [100, .2, 15], 'material')
    check('carrier_paper_mouth', 'carrier', [0, 0, -.2], [7.5, 59, .05], 'clear')
    check('carrier_paper_channel', 'carrier', [0, 0, -2], [5.5, 59, .1], 'clear')
    check('trough_reservoir_opening', 'trough', [0, 0, -10], [140, 76, 20], 'clear')
    check('trough_floor', 'trough', [0, 0, -22.5], [140, 76, .2], 'material')
    for a, b in (('holder', 'trough'), ('guide', 'holder'), ('guide', 'trough')):
        truth = (sources[a] ^ sources[b]).volume()
        approx = (collisions[a] ^ collisions[b]).volume()
        records.append(dict(name=f"nominal_assembly_{a}_{b}", expected='clear', source_overlap_mm3=truth,
                            collision_overlap_mm3=approx, passed=bool(truth < .005 and approx < .005)))
    return records


def inertial_properties(mesh, mass_kg):
    if not np.isfinite(mass_kg) or mass_kg <= 0 or not mesh.is_volume:
        raise ValueError("Positive finite mass and source volume required")
    inertia = mesh.moment_inertia*(mass_kg/mesh.mass)*1e-6
    if np.min(np.linalg.eigvalsh(inertia)) <= 0:
        raise ValueError("Non-positive source inertia")
    return dict(mass_kg=float(mass_kg), center_of_mass_m=(mesh.center_mass*.001).tolist(), inertia_kg_m2=inertia.tolist())


def build_bundle(cad_dir, trough_stl, holder_stl, out_dir, *, progress=None, holder_cover='angular'):
    cad_dir, out_dir = Path(cad_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema=SCHEMA, status='building', units='mm', collision_scale_to_m=.001,
                    geometry_scope='Complete original source solids; all four slots and original end/handle detail retained',
                    evidence_boundary='Reusable diagnostic geometry; no task, policy, visual controller or physical qualification',
                    parts={}, limits=LIMITS, holder_cover=holder_cover, source_code_sha256={
                        str(Path(__file__).resolve()): sha(__file__),
                        str(Path(__file__).with_name('g4_carrier.py').resolve()): sha(Path(__file__).with_name('g4_carrier.py'))},
                    assumptions=['All source STLs are in nominal CAD assembly axes and millimetres.',
                                 'Uniform solid effective density 1000 kg/m3 for trough/holder/guide; carrier mass 2.5 g retained from baseline. No masses measured.',
                                 'Rigid convex contact cover preserves cavities but is an approximation near facets and internal piece boundaries.',
                                 'Geometry qualification requires successful nonempty Boolean union and all functional probes. Dynamic assembly still requires independent source-normal and task checks.'])
    path = out_dir/'manifest.json'
    (out_dir/'source_code').mkdir()
    for source_path in manifest['source_code_sha256']:
        shutil.copy2(source_path, out_dir/'source_code'/Path(source_path).name)
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    paths = dict(trough=Path(trough_stl), holder=Path(holder_stl), guide=cad_dir/'guide_PROTOTYPE.stl', carrier=cad_dir/'carrier_PROTOTYPE.stl')
    sources, collisions = {}, {}
    (out_dir/'source_assets').mkdir()
    for name, src in paths.items():
        if progress: progress(f"Building full {name}")
        mesh = trimesh.load_mesh(src)
        if not mesh.is_watertight or not mesh.is_volume:
            raise ValueError(f"{name}: source must be a closed positive solid")
        source = solid(mesh)
        _valid(source, name)
        sources[name] = source
        holder_method = {'continuous': full_holder_cover, 'angular': full_holder_angular_cover,
                         'interface': lambda s: full_holder_angular_cover(s, continuous_rims=True)}[holder_cover]
        guide_method = full_guide_interface_cover if holder_cover == 'interface' else full_guide_cover
        method = dict(trough=full_trough_cover, holder=holder_method, guide=guide_method,
                      carrier=lambda s: convex_partition([s]))[name]
        pieces, stats = method(source)
        collisions[name], audit = audit_union(source, pieces)
        audit.update(stats)
        audit['source_bounds_mm'] = mesh.bounds.tolist()
        part_dir = out_dir/'meshes'/name
        part_dir.mkdir(parents=True)
        files = []
        hashes = {}
        for i, piece in enumerate(pieces):
            dest = part_dir/f'{i:04d}.obj'
            piece.export(dest)
            relative = str(dest.relative_to(out_dir))
            files.append(relative)
            hashes[relative] = sha(dest)
        copy = out_dir/'source_assets'/src.name
        shutil.copy2(src, copy)
        relative = str(copy.relative_to(out_dir))
        hashes[relative] = sha(copy)
        mass_kg = .0025 if name == 'carrier' else mesh.volume*1e-6
        manifest['parts'][name] = dict(visual_stl=relative, collision_meshes=files, sha256=hashes,
                                      original_source=str(src.resolve()), original_source_sha256=sha(src),
                                      source_to_assembly=np.eye(4).tolist(), translation_units='mm',
                                      inertial_assumption='Uniform source-solid inertia; unmeasured mass hypothesis',
                                      **inertial_properties(mesh, mass_kg), audit=audit)
        path.write_text(json.dumps(manifest, indent=2)+'\n')
        if progress: progress(f"{name}: {len(pieces)} pieces, extra {audit['extra_mm3']:.8g} mm3, missing {audit['missing_mm3']:.8g} mm3")
    probes = functional_probes(sources, collisions)
    manifest['functional_probes'] = probes
    manifest['assembly_nominal_instances'] = [dict(name=n, part=n, position_mm=[0, 0, 0], quaternion_wxyz=[1, 0, 0, 0]) for n in ('trough', 'holder', 'guide')]
    manifest['assembly_nominal_instances'] += [dict(name=f'carrier_{i+1}', part='carrier', position_mm=[x, 0, 0], quaternion_wxyz=[1, 0, 0, 0]) for i, x in enumerate(SLOTS_MM)]
    manifest['status'] = 'complete' if all(p['audit']['passed'] for p in manifest['parts'].values()) and all(p['passed'] for p in probes) else 'rejected'
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def load_bundle(path, *, verify_hashes=True):
    """Reject incomplete/malformed/tampered bundles; return absolute asset paths."""
    path = Path(path).resolve()
    data = json.loads(path.read_text())
    if data.get('schema') != SCHEMA or data.get('status') != 'complete':
        raise ValueError('Bundle is not complete and geometry-qualified')
    if set(data.get('parts', {})) != {'trough', 'holder', 'guide', 'carrier'}:
        raise ValueError('Full four-part inventory is required')
    if not data.get('functional_probes') or not all(p.get('passed') is True for p in data['functional_probes']):
        raise ValueError('Functional probes did not pass')
    for name, part in data['parts'].items():
        if not part.get('audit', {}).get('passed') or not part.get('collision_meshes'):
            raise ValueError(f'{name}: empty or rejected collision assets')
        for relative in [part['visual_stl'], *part['collision_meshes']]:
            target = (path.parent/relative).resolve()
            if not target.is_relative_to(path.parent):
                raise ValueError('Asset path escapes bundle')
            if verify_hashes and sha(target) != part['sha256'].get(relative):
                raise ValueError(f'Asset hash mismatch: {relative}')
        transform = np.asarray(part['source_to_assembly'], float)
        if transform.shape != (4, 4) or not np.isfinite(transform).all() or not np.allclose(transform, np.eye(4)):
            raise ValueError('Unexpected source coordinate transform')
        part['visual_stl'] = str((path.parent/part['visual_stl']).resolve())
        part['collision_meshes'] = [str((path.parent/p).resolve()) for p in part['collision_meshes']]
    data['manifest_path'] = str(path)
    return data


def _words(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def resting_scene(bundle, path):
    """Privileged nominal reset; fixed trough, free holder/guide/four carriers."""
    root = ET.Element('mujoco', model='G4_full_assets_resting_diagnostic')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'size', memory='512M')
    option = ET.SubElement(root, 'option', timestep='.0001', gravity='0 0 -9.81', integrator='implicitfast',
                           cone='elliptic', iterations='100', tolerance='1e-10')
    ET.SubElement(option, 'flag', nativeccd='enable', multiccd='disable')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction='.4 .001 .0001', condim='4', solref='.0009 1',
                  solimp='.95 .99 .0001', margin='0', gap='0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'map', znear='.0001')
    asset, world = ET.SubElement(root, 'asset'), ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -.15 .3', dir='0 0 -1', diffuse='.8 .8 .8', ambient='.5 .5 .5')
    colors = dict(trough='.45 .55 .65 1', holder='.82 .76 .60 1', guide='.8 .52 .3 .38', carrier='.35 .35 .35 1')
    for name, part in bundle['parts'].items():
        ET.SubElement(asset, 'mesh', name=f'{name}_visual', file=part['visual_stl'], scale='.001 .001 .001')
        for i, p in enumerate(part['collision_meshes']):
            ET.SubElement(asset, 'mesh', name=f'{name}_c{i}', file=p, scale='.001 .001 .001')
    for instance in bundle['assembly_nominal_instances']:
        name, kind = instance['name'], instance['part']
        part = bundle['parts'][kind]
        body = ET.SubElement(world, 'body', name=name, pos=_words(np.array(instance['position_mm'])/1000))
        if kind != 'trough':
            ET.SubElement(body, 'freejoint', name=f'{name}_free')
            inertia = np.array(part['inertia_kg_m2'])
            ET.SubElement(body, 'inertial', pos=_words(part['center_of_mass_m']), mass=str(part['mass_kg']),
                          fullinertia=_words([inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]]))
        ET.SubElement(body, 'geom', name=f'{name}_visual', type='mesh', mesh=f'{kind}_visual',
                      contype='0', conaffinity='0', mass='0', rgba=colors[kind], group='2')
        for i in range(len(part['collision_meshes'])):
            ET.SubElement(body, 'geom', name=f'{name}_c{i}', type='mesh', mesh=f'{kind}_c{i}', rgba='0 0 0 0', group='3')
    ET.ElementTree(root).write(path, encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(path))


def resting_sample(model, data):
    contacts, totals = [], {}
    for i in range(data.ncon):
        c = data.contact[i]
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        a, b = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, v) for v in (a, b)]
        frame = c.frame.reshape(3, 3)
        force = frame.T@wrench[:3]
        for name, sign in zip(names, (-1, 1)):
            value = totals.setdefault(name, dict(normal_sum_n=0., net_force_n=[0., 0., 0.]))
            value['normal_sum_n'] += max(0., float(wrench[0]))
            value['net_force_n'] = (np.array(value['net_force_n'])+sign*force).tolist()
        contacts.append(dict(bodies=names, geoms=[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (c.geom1, c.geom2)],
                             distance_mm=float(c.dist*1000), position_m=c.pos.tolist(), frame=c.frame.tolist(), wrench_local=wrench.tolist()))
    bodies = {}
    for bid in range(1, model.nbody):
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
        bodies[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)] = dict(position_m=data.xpos[bid].tolist(),
                            rotation=data.xmat[bid].reshape(3, 3).tolist(), spatial_velocity=velocity.tolist())
    return dict(time_s=float(data.time), bodies=bodies, contacts=contacts, totals=totals)


def _inside_solid_angle(mesh, point):
    """Generalized winding number for a closed source mesh; no spatial index."""
    vectors = mesh.triangles-np.asarray(point)
    a, b, c = vectors[:, 0], vectors[:, 1], vectors[:, 2]
    la, lb, lc = [np.linalg.norm(v, axis=1) for v in (a, b, c)]
    top = np.einsum('ij,ij->i', a, np.cross(b, c))
    bottom = la*lb*lc+np.einsum('ij,ij->i', a, b)*lc+np.einsum('ij,ij->i', b, c)*la+np.einsum('ij,ij->i', c, a)*lb
    return bool(abs(np.arctan2(top, bottom).sum()) > np.pi)


def audit_loaded_source_contacts(bundle, row, *, max_contacts=80):
    """Sample contact endpoints against original whole source, not convex pieces.

    Signed distances are positive outside. Cone residual is a local numerical
    approximation diagnostic, not a validated physical acceptance threshold.
    """
    from scipy.optimize import nnls
    source = {n: trimesh.load_mesh(p['visual_stl']) for n, p in bundle['parts'].items()}
    # Include each support pair, so the heavily loaded trough cannot hide the
    # lighter guide and any of the four carriers in a global top-N sample.
    eligible = sorted([c for c in row['contacts'] if c['wrench_local'][0] > .00001],
                      key=lambda c: c['wrench_local'][0], reverse=True)
    indices = set(range(min(max_contacts, len(eligible))))
    pair_counts = {}
    for i, c in enumerate(eligible):
        pair = tuple(c['bodies'])
        count = pair_counts.get(pair, 0)
        if count < 8: indices.add(i)
        pair_counts[pair] = count+1
    loaded = [eligible[i] for i in sorted(indices)]
    records = []
    for c in loaded:
        normal = np.array(c['frame']).reshape(3, 3)[0]
        for index, body in enumerate(c['bodies']):
            kind = 'carrier' if body.startswith('carrier_') else body
            pose = row['bodies'][body]
            rotation = np.array(pose['rotation'])
            sign = -1 if index == 0 else 1
            endpoint = np.array(c['position_m'])+sign*c['distance_mm']*.0005*normal
            local = rotation.T@(endpoint-np.array(pose['position_m']))*1000
            outward = rotation.T@(-sign*normal)
            mesh = source[kind]
            points = trimesh.triangles.closest_point(mesh.triangles, np.tile(local, (len(mesh.faces), 1)))
            distances = np.linalg.norm(points-local, axis=1)
            nearest = float(distances.min())
            inside = _inside_solid_angle(mesh, local)
            incident = distances <= nearest+.001
            normals = mesh.face_normals[incident]
            residual = float(nnls(normals.T, outward)[1])
            records.append(dict(body=body, geom=c['geoms'][index], normal_force_n=c['wrench_local'][0],
                                source_endpoint_mm=local.tolist(), source_outward_normal=outward.tolist(),
                                signed_source_distance_mm=-nearest if inside else nearest,
                                normal_cone_neighbourhood_mm=.001, normal_cone_residual=residual,
                                outward_probe_2um_inside_source=_inside_solid_angle(mesh, local+.002*outward),
                                inward_probe_2um_inside_source=_inside_solid_angle(mesh, local-.002*outward)))
    return records


def run_resting_diagnostic(manifest_path, out_dir, *, duration_s=.25):
    """Unactuated nominal resting evidence, never a release or assembly success."""
    import gzip
    from PIL import Image
    bundle = load_bundle(manifest_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    module_hash = sha(__file__)
    shutil.copy2(__file__, out_dir/'g4_collision_assets.py')
    (out_dir/'predeclared.json').write_text(json.dumps(dict(duration_s=duration_s,
        manifest_sha256=sha(manifest_path), diagnostic_module_sha256=module_hash,
        nativeccd=True, multiccd=False, timestep_s=.0001, fixed_body='trough',
        reset_scope='Nominal assembly pose, no placement or release controller'), indent=2)+'\n')
    model = resting_scene(bundle, out_dir/'scene.xml')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial = resting_sample(model, data)
    if any(w.number for w in data.warning):
        raise RuntimeError('MuJoCo warning at initialization; resting evidence rejected')
    peak, worst_pen, frames = initial, initial, []
    renderer = mujoco.Renderer(model, 720, 960)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [-.01, 0, .01]
    camera.distance = .34
    camera.azimuth, camera.elevation = 125, -28
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    def force(row):
        return max((v['normal_sum_n'] for v in row['totals'].values()), default=0.)
    def penetration(row):
        return max((max(0., -c['distance_mm']) for c in row['contacts']), default=0.)
    with gzip.open(out_dir/'observations.jsonl.gz', 'wt') as stream:
        for step in range(round(duration_s/model.opt.timestep)+1):
            if step:
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                if any(w.number for w in data.warning):
                    raise RuntimeError('MuJoCo warning during resting diagnostic; evidence rejected')
            row = resting_sample(model, data)
            stream.write(json.dumps(row, separators=(',', ':'))+'\n')
            if force(row) > force(peak): peak = row
            if penetration(row) > penetration(worst_pen): worst_pen = row
            if step % 100 == 0 or step == round(duration_s/model.opt.timestep):
                renderer.update_scene(data, camera=camera, scene_option=option)
                frames.append(Image.fromarray(renderer.render().copy()))
    renderer.close()
    frames[0].save(out_dir/'initial.png')
    frames[-1].save(out_dir/'final.png')
    frames[0].save(out_dir/'timeline.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)
    selected = dict(initial=initial, peak_force=peak, peak_penetration=worst_pen, final=row)
    source_checks = {name: audit_loaded_source_contacts(bundle, sample) for name, sample in selected.items()}
    result = dict(status='diagnostic_complete', scope='Privileged nominal resting reset with fixed trough; all other bodies free and unactuated. No placement trajectory or task success.',
                  manifest_path=str(Path(manifest_path).resolve()), manifest_sha256=sha(manifest_path),
                  diagnostic_module_sha256=module_hash, duration_s=duration_s, timestep_s=float(model.opt.timestep),
                  contact_model=dict(nativeccd=True, multiccd=False, margin_m=0., solref=[.0009, 1], friction=[.4, .001, .0001], measured=False),
                  maximum_body_normal_sum_n=force(peak), maximum_penetration_mm=penetration(worst_pen),
                  selected_states=selected, source_contact_checks=source_checks,
                  visual_note='Actual MuJoCo frames; guide opacity reduced solely for technical inspection. GIF slowed 10x.')
    (out_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    return result
