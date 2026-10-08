"""Source-backed G4 roller with an unactuated wheel on an ideal M5 axle.

The purchased axle, masses and bearing losses are declared hypotheses. A hinge
models retention and radial constraint, not measured clearance or axle wear.
Geometry is in the unchanged G4 assembly frame, converted from mm to SI.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as E

import numpy as np
import trimesh

from planter.g4_carrier import convex_partition, solid, union_solids


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def words(values):
    return ' '.join(format(float(x), '.12g') for x in values)


def _handle_sectors(source):
    # Source cuts follow the actual 96-facet axle bore, avoiding hull fill of
    # that bore. Above the fork, the source-face partition retains the grip.
    lower = source.trim_by_plane([0, 0, -1], -22)
    initial = source.trim_by_plane([0, 0, 1], 22).decompose()
    origin = np.array([0., 0., 8.])
    for angle in np.arange(0., 360., 3.75):
        a, b = np.radians([angle, angle + 3.75])
        na = np.array([-np.sin(a), 0, np.cos(a)])
        nb = np.array([np.sin(b), 0, -np.cos(b)])
        initial.extend(lower.trim_by_plane(na, float(na @ origin)).trim_by_plane(
            nb, float(nb @ origin)).decompose())
    return initial


def _probe_box(center, extent):
    mesh = trimesh.creation.box(extent)
    mesh.apply_translation(center)
    return solid(mesh)


def prepare_roller(cad: Path, out: Path):
    """Export convex source partitions and a checked, hashed assembly manifest."""
    cad, out = Path(cad).resolve(), Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    parts, solids = {}, {}
    for key, filename, mass in [('wheel', 'roller_PROTOTYPE.stl', .010),
                                ('handle', 'roller_handle_PROTOTYPE.stl', .014)]:
        source_path = cad / filename
        mesh = trimesh.load(source_path, force='mesh')
        source = solid(mesh)
        initial = _handle_sectors(source) if key == 'handle' else source.decompose()
        pieces, stats = convex_partition(initial)
        union = union_solids(pieces)
        extra, missing = (union - source).volume(), (source - union).volume()
        # A volume audit is necessary, but not a proof of dynamic normal fidelity.
        if max(extra, missing) > .001:
            raise ValueError(f'{key} collision union differs from CAD: {extra}, {missing} mm3')
        paths = []
        for i, piece in enumerate(pieces):
            path = out / f'{key}-{i:03d}.obj'
            piece.export(path)
            paths.append(dict(path=str(path), sha256=sha(path)))
        mesh.apply_scale(.001)
        mesh.density = mass / mesh.volume
        parts[key] = dict(source=str(source_path), source_sha256=sha(source_path),
                          collisions=paths, mass_kg=mass, mass_status='assumption',
                          center_m=mesh.center_mass.tolist(),
                          inertia_kg_m2=mesh.moment_inertia.tolist(),
                          extra_mm3=extra, missing_mm3=missing, partition=stats)
        solids[key] = union
    probes = []
    for key in ('wheel', 'handle'):
        # The entire smooth 5 mm shaft path must remain open in both CAD parts.
        for y in ([-24., 0., 24.] if key == 'wheel' else [-27.3, 27.3]):
            probe = _probe_box([0, y, 8], [3.5, .5, 3.5])
            overlap = (solids[key] ^ probe).volume()
            probes.append(dict(part=key, kind='open_axle_bore', y_mm=y,
                               overlap_mm3=overlap, passed=overlap < 1e-6))
    for y in (-25.3, 25.3):
        probe = _probe_box([0, y, 8], [14, .4, 14])
        overlap = sum((obj ^ probe).volume() for obj in solids.values())
        probes.append(dict(kind='wheel_fork_end_gap', y_mm=y,
                           overlap_mm3=overlap, passed=overlap < 1e-6))
    if not all(row['passed'] for row in probes):
        raise ValueError('Roller source aperture or end-gap audit failed')
    result = dict(schema=1, parts=parts, geometry_probes=probes,
                  local_frame='Unchanged G4 assembly CAD frame; SI meters',
                  hinge_position_m=[0, 0, .008], hinge_axis=[0, 1, 0],
                  axle_diameter_m=.005, axle_length_m=.070,
                  axle_mass_kg=.01079, axle_hardware_status='purchased hardware hypothesis',
                  wheel_damping_Nm_s=2e-6, wheel_frictionloss_Nm=1e-5,
                  assumptions=['Uniform source-derived inertia at assumed printed masses',
                               'Ideal hinge constrains radial motion and retains wheel axially',
                               'No measured axle play, washers, fastener preload or friction',
                               'Collision union volume and local apertures audited; dynamic normals remain approximate'],
                  module_sha256=sha(__file__), physical_validation=False,
                  wheel_actuator=False)
    (out / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def add_roller(root, manifest, *, name='roller', position_m=(0, 0, 0),
               quaternion=(1, 0, 0, 0), free=True, damping=None, frictionloss=None):
    """Add one handle/free root and passive wheel child; never add an actuator."""
    pose = np.asarray([*position_m, *quaternion], dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1) > 1e-6:
        raise ValueError('Roller pose requires finite SI xyz and unit wxyz quaternion')
    for part in manifest['parts'].values():
        for item in [dict(path=part['source'], sha256=part['source_sha256']), *part['collisions']]:
            if sha(item['path']) != item['sha256']:
                raise ValueError('Roller source/collision hash mismatch')
    asset, world = root.find('asset'), root.find('worldbody')
    if asset is None:
        asset = E.SubElement(root, 'asset')
    if world is None:
        world = E.SubElement(root, 'worldbody')
    body = E.SubElement(world, 'body', name=name, pos=words(position_m), quat=words(quaternion))
    if free:
        E.SubElement(body, 'freejoint', name=name + '_free')
    wheel = E.SubElement(body, 'body', name=name + '_wheel')
    damping = manifest['wheel_damping_Nm_s'] if damping is None else damping
    frictionloss = manifest['wheel_frictionloss_Nm'] if frictionloss is None else frictionloss
    if not np.isfinite([damping, frictionloss]).all() or min(damping, frictionloss) < 0:
        raise ValueError('Bearing losses must be finite and nonnegative')
    joint_name = name + '_wheel_hinge'
    E.SubElement(wheel, 'joint', name=joint_name, type='hinge', limited='false',
                 pos=words(manifest['hinge_position_m']), axis='0 1 0',
                 damping=str(damping), frictionloss=str(frictionloss))
    metadata = dict(body=name, wheel_body=name + '_wheel', wheel_joint=joint_name,
                    free_joint=name + '_free' if free else None, geoms=[])
    for key, parent in [('handle', body), ('wheel', wheel)]:
        part = manifest['parts'][key]
        # Add the ideal shaft's inertial contribution to the handle analytically.
        mass = part['mass_kg']; center = np.array(part['center_m'])
        inertia = np.array(part['inertia_kg_m2'])
        if key == 'handle':
            shaft_mass = manifest['axle_mass_kg']; shaft_center = np.array([0, 0, .008])
            new_center = (mass*center + shaft_mass*shaft_center)/(mass+shaft_mass)
            radial, length = .0025, .070
            shaft_i = np.diag([shaft_mass*(3*radial**2+length**2)/12,
                               shaft_mass*radial**2/2,
                               shaft_mass*(3*radial**2+length**2)/12])
            total = np.zeros((3,3))
            for m, c, i in [(mass, center, inertia), (shaft_mass, shaft_center, shaft_i)]:
                d = c-new_center
                total += i + m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
            mass += shaft_mass; center = new_center; inertia = total
        E.SubElement(parent, 'inertial', mass=str(mass), pos=words(center),
                     fullinertia=words([*np.diag(inertia), inertia[0,1], inertia[0,2], inertia[1,2]]))
        visual_name = name + '_' + key + '_visual'
        E.SubElement(asset, 'mesh', name=visual_name, file=part['source'], scale='.001 .001 .001')
        E.SubElement(parent, 'geom', name=visual_name, type='mesh', mesh=visual_name,
                     contype='0', conaffinity='0', mass='0',
                     rgba='.25 .38 .56 1' if key == 'handle' else '.25 .50 .52 1')
        for i, item in enumerate(part['collisions']):
            partname = f'{name}_{key}_{i:03d}'
            E.SubElement(asset, 'mesh', name=partname, file=item['path'], scale='.001 .001 .001')
            E.SubElement(parent, 'geom', name=partname, type='mesh', mesh=partname,
                         group='3', mass='0', condim='4', friction='.6 .0001 .00001',
                         solref='.004 1', solimp='.95 .99 .001')
            metadata['geoms'].append(partname)
    E.SubElement(body, 'geom', name=name + '_axle', type='cylinder',
                 fromto='0 -.035 .008 0 .035 .008', size='.0025', mass='0', rgba='.6 .6 .62 1')
    E.SubElement(body, 'site', name=name + '_grip', pos='0 0 .045', size='.001', rgba='0 0 0 0')
    # Prevent the ideal constrained bearing from also becoming a contact bearing.
    # This pair is intentionally modeled by the passive hinge, not external grips.
    contact = root.find('contact')
    if contact is None:
        contact = E.SubElement(root, 'contact')
    E.SubElement(contact, 'exclude', body1=name, body2=name + '_wheel')
    metadata['grip_site'] = name + '_grip'
    return metadata
