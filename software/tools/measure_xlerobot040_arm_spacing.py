"""Extract the exported 0.4.0 arm spacing from pinned STEP geometry. No robot access.

Requires CadQuery 2.8. Give the XLeRobot040_armbase STEP and SO101 Base STEP.
The result describes the exported plate orientation, not every adjustable mounting.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np

XL_REV = 'b017b5e6354bd9f61f4247a920c72622ca0aade0'
SO_REV = '5f6d2b876a53a4872e405b991dd925556c9e38a4'


def cylinders(solid, radius, axis):
    rows = []
    for face in solid.Faces():
        if face.geomType() != 'CYLINDER':
            continue
        c = face._geomAdaptor().Cylinder()
        d, p = c.Axis().Direction(), c.Location()
        direction = np.array([d.X(), d.Y(), d.Z()])
        if abs(c.Radius() - radius) < 0.0001 and abs(abs(direction[axis]) - 1) < 1e-6:
            rows.append([p.X(), p.Y(), p.Z()])
    return np.unique(np.round(rows, 6), axis=0)


def measure(armbase, base):
    plates = cq.importers.importStep(str(armbase)).solids().vals()
    bases = cq.importers.importStep(str(base)).solids().vals()
    if len(plates) != 14 or len(bases) != 1:
        raise ValueError('Unexpected STEP layout; inspect source revision before interpreting solid indexes')
    mounting = cylinders(bases[0], 2.5, 1)
    # The two symmetric mounting pairs, selected in the pinned SO101 base's X/Z plane.
    expected = np.array([[-31.75, -7.5], [31.75, -7.5], [-27.7765, 62.275], [27.7765, 62.275]])
    holes = []
    for point in expected:
        matches = mounting[np.linalg.norm(mounting[:, [0, 2]] - point, axis=1) < 0.001]
        if len(matches) != 1:
            raise ValueError('SO101 mounting hole pattern changed')
        holes.append(matches[0, [0, 2]])
    holes = np.array(holes)
    pan = cylinders(bases[0], 13.2069, 1)
    pan_xz = np.unique(pan[:, [0, 2]], axis=0)
    if len(pan_xz) != 1:
        raise ValueError('SO101 pan-axis cylinder is not unique')
    # Local Y is up. This proper 3D rotation maps local X -> -CAD Y,
    # local Z -> -CAD X. Fit translation from all four actual mating holes.
    rotated = np.column_stack([-holes[:, 1], -holes[:, 0]])
    rows = []
    for index in (10, 12):
        centres = cylinders(plates[index], 10, 2)
        if len(centres) != 1:
            raise ValueError('Plate rotation centre not unique')
        centre = centres[0, :2]
        all_holes = cylinders(plates[index], 1.7, 2)
        top = all_holes[all_holes[:, 2] > all_holes[:, 2].max() - 0.001, :2]
        if len(top) != 4:
            raise ValueError('Expected four upper SO101 mating holes')
        translation = top.mean(axis=0) - rotated.mean(axis=0)
        predicted = rotated + translation
        nearest = np.linalg.norm(predicted[:, None, :] - top[None, :, :], axis=2)
        if len(set(nearest.argmin(axis=1))) != 4 or nearest.min(axis=1).max() > 0.001:
            raise ValueError('SO101 mounting holes do not mate the plate')
        pan_xy = [-pan_xz[0, 1] + translation[0], -pan_xz[0, 0] + translation[1]]
        rows.append({'solid_index_0based': index, 'rotation_centre_xy_mm': centre.tolist(),
                     'mating_holes_xy_mm': top.tolist(), 'base_to_cad_xy_translation_mm': translation.tolist(),
                     'maximum_hole_residual_mm': float(nearest.min(axis=1).max()),
                     'shoulder_pan_xy_mm': pan_xy, 'pan_offset_from_rotation_centre_mm': (np.array(pan_xy) - centre).tolist()})
    axes = np.array([r['shoulder_pan_xy_mm'] for r in rows])
    return {'sources': [
        {'file': armbase.name, 'sha256': hashlib.sha256(armbase.read_bytes()).hexdigest(),
         'url': f'https://github.com/Vector-Wangel/XLeRobot/blob/{XL_REV}/hardware/step/XLeRobot_040/XLeRobot040_armbase.step'},
        {'file': base.name, 'sha256': hashlib.sha256(base.read_bytes()).hexdigest(),
         'url': f'https://github.com/TheRobotStudio/SO-ARM100/blob/{SO_REV}/STEP/SO101/Base_SO101.step'}],
        'units': 'mm', 'solid_count': len(plates), 'local_pan_xz_mm': pan_xz[0].tolist(), 'plates': rows,
        'shoulder_pan_separation_mm': float(np.linalg.norm(axes[0] - axes[1])),
        'orientation': 'Both top plates in the STEP export orientation; local SO101 Y upward, X to -CAD Y, Z to -CAD X',
        'limitation': 'The mounts rotate. This is not a measurement of installed mount angles or a live kinematic calibration.',
        'motor_writes': 0}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--armbase', type=Path, required=True)
    parser.add_argument('--so101-base', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = measure(args.armbase, args.so101_base)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(f"Exported shoulder-pan separation: {result['shoulder_pan_separation_mm']:.3f} mm")
