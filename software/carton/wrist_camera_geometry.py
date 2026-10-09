"""Registered XLeRobot wrist camera CAD, with conservative convex collision hulls.

These meshes describe the upstream mount, not a measured physical assembly.
The fixed-jaw to SO101 registration has about 1.7 mm RMS error. See the asset
manifest for provenance. MuJoCo uses a separate convex hull for each of the
two parts; it does not treat the visible camera shell as collision-free.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).with_name('assets') / 'wrist-camera'


def add_wrist_camera(asset, gripper, side):
    for part, color in ((1, '.82 .82 .79 1'), (2, '.08 .08 .09 1')):
        name = f'{side}_wrist_camera_{part}'
        ET.SubElement(asset, 'mesh', name=name, file=str(ASSETS / f'{side}-{part}.obj'))
        # Put exact CAD in the normal render group, independently of colliders.
        ET.SubElement(gripper, 'geom', name=name+'_visual', type='mesh', mesh=name,
                      rgba=color, contype='0', conaffinity='0', mass='0', group='1')
        # The gripper has an explicit inertial: preserve it. Added module mass
        # is not calibrated and is deliberately not claimed by this geometry.
        ET.SubElement(gripper, 'geom', name=name+'_collision', type='mesh', mesh=name,
                      contype='1', conaffinity='1', mass='0', group='3',
                      friction='.8 .003 .0001', solref='.004 1', solimp='.95 .99 .001')
