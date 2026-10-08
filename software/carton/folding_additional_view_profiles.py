"""Explicit renderer-only camera placements, never measured calibration.

Adding a camera adds no mass, collision geometry, marker, actuator or constraint.
Nominal renderer extrinsics are checked for provenance only. Observers must
recover camera transforms from fresh calibrated marker pixels independently.
"""
from __future__ import annotations

import copy
import xml.etree.ElementTree as ET

import numpy as np


ADDITIONAL_VIEW_CAMERAS = ('front', 'front_left_back', 'front_right_back')
_POSITIONS = {'front_left_back': (-.20, -.85, 1.05),
              'front_right_back': (.20, -.85, 1.05)}
_TARGET = (0., .10, .12)


def additional_view_camera_declaration(camera_name):
    if camera_name not in ADDITIONAL_VIEW_CAMERAS:
        raise ValueError('Unsupported explicitly declared additional-view camera')
    result = dict(camera=camera_name, renderer_only=True, physical_installation_verified=False,
                  fovy_degrees=48., world_frame='declared simulated station tabletop')
    if camera_name == 'front':
        result['placement'] = 'existing front camera, unchanged'
    else:
        result.update(profile_id='offline:'+camera_name+'-v1',
                      optical_center_m=list(_POSITIONS[camera_name]), look_at_m=list(_TARGET),
                      mounting_hardware_modelled=False, physical_calibration_verified=False)
    return result


def _rotation(camera_name):
    back = np.asarray(_POSITIONS[camera_name]) - _TARGET
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back); right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(back, right), back))


def add_additional_view_camera(root, camera_name):
    """Append only the requested named profile to a scene XML root.

    ``front`` is a no-op on an existing front camera, preserving its exact
    attributes. New named profiles must not already exist. Return the explicit
    hypothetical declaration for result metadata, not an observed transform.
    """
    declaration = additional_view_camera_declaration(camera_name)
    world = root.find('worldbody')
    if world is None:
        raise ValueError('Scene worldbody required for a declared camera')
    matching = root.findall(f".//camera[@name='{camera_name}']")
    if camera_name == 'front':
        if len(matching) != 1:
            raise ValueError('Exactly one existing front camera required')
        return declaration
    if matching:
        raise ValueError('Do not replace or duplicate an existing named camera')
    rotation = _rotation(camera_name)
    words = lambda values: ' '.join(format(float(v), '.17g') for v in values)
    ET.SubElement(world, 'camera', name=camera_name,
        pos=words(_POSITIONS[camera_name]), xyaxes=words(np.r_[rotation[:, 0], rotation[:, 1]]),
        fovy='48', mode='fixed')
    return declaration


def verify_additional_view_camera(camera_name, model_camera):
    """Bind nominal renderer metadata; do not use it as pixel calibration.

    Nondefault profiles require a fixed world-attached camera at the exact
    declared pose. A replay facade must expose the corresponding immutable
    model-camera metadata, never an object MjData transform.
    """
    declaration = additional_view_camera_declaration(camera_name)
    try:
        fovy = np.asarray(model_camera.fovy, dtype=float).reshape(-1)
        if fovy.shape != (1,) or not np.isfinite(fovy).all() or fovy[0] != 48.:
            raise ValueError('Additional camera intrinsics differ from declared profile')
        if camera_name == 'front':
            return declaration
        position = np.asarray(model_camera.pos, dtype=float)
        quat = np.asarray(model_camera.quat, dtype=float)
        body_id = np.asarray(model_camera.bodyid).reshape(-1)
        mode = np.asarray(model_camera.mode).reshape(-1)
    except AttributeError as exc:
        raise ValueError('Named camera profile requires immutable renderer pose metadata') from exc
    if (position.shape != (3,) or quat.shape != (4,) or not np.isfinite(position).all()
            or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat), 1., atol=1e-8)
            or not np.allclose(position, _POSITIONS[camera_name], atol=1e-9, rtol=0)
            or body_id.shape != (1,) or body_id[0] != 0 or mode.shape != (1,) or mode[0] != 0):
        raise ValueError('Named additional camera pose or fixed world attachment differs from declaration')
    w, x, y, z = quat
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    if not np.allclose(rotation, _rotation(camera_name), atol=1e-8, rtol=0):
        raise ValueError('Named additional camera orientation differs from declaration')
    return copy.deepcopy(declaration)
