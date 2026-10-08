"""Recover a free carton pose from two freshly observed short-flap tags.

Offline declared geometry only. Both tag mounts must be measured before any
physical use. This observes the hinge origins; it never assumes the box stayed
still, remained level, or that a flap followed a commanded angle.
"""
import numpy as np
from carton.geometry import Box


def short_flap_tag_mount(side):
    if side not in ('left', 'right'):
        raise ValueError('Declare left or right short flap')
    sign = -1 if side == 'left' else 1
    u = np.array([0., sign, 0.])
    v = np.array([0., 0., 1.])
    normal = np.cross(u, v)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((-u, v, -normal))
    # Paper centre is 1.8 mm beyond the hinge plane, printed face another
    # 0.3 mm outward, matching the existing short-flap marker geometry.
    pose[:3, 3] = np.array([0., .07, .09]) + .0021 * normal
    return pose


def carton_pose_from_short_flaps(tags):
    if not {11, 12}.issubset(tags):
        raise ValueError('Both fresh short-flap markers required for hinge registration')
    frames = []
    for tag, side in ((11, 'left'), (12, 'right')):
        pose = np.asarray(tags[tag], dtype=float)
        if (pose.shape != (4, 4) or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
                or np.linalg.det(pose[:3, :3]) < .999):
            raise ValueError('Finite rigid short-flap marker poses required')
        frames.append(pose @ np.linalg.inv(short_flap_tag_mount(side)))
    left, right = frames
    baseline = right[:3, 3] - left[:3, 3]
    distance = np.linalg.norm(baseline)
    box = Box()
    baseline_error = abs(distance - box.length)
    if baseline_error > .012:
        raise ValueError('Observed short-flap hinge spacing disagrees by over 12 mm')
    x_axis = baseline / distance
    left_y, right_y = left[:3, 1], right[:3, 1]
    agreement = np.degrees(np.arccos(np.clip(left_y @ right_y, -1, 1)))
    orthogonal_error = max(abs(float(x_axis @ left_y)), abs(float(x_axis @ right_y)))
    if agreement > 8 or orthogonal_error > np.sin(np.radians(8)):
        raise ValueError('Observed short-flap hinge axes disagree with carton geometry')
    y_axis = left_y + right_y
    y_axis -= (x_axis @ y_axis) * x_axis
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    pose[:3, 3] = (left[:3, 3] + right[:3, 3]) / 2 - box.height * z_axis
    return pose, {
        'source': 'paired short-flap hinges from fresh RGB-D tags',
        'visible_ids': [11, 12], 'selected_id': None,
        'hinge_spacing_error_mm': float(baseline_error * 1000),
        'hinge_axis_disagreement_degrees': float(agreement),
        'hinge_perpendicular_error_degrees': float(np.degrees(np.arcsin(orthogonal_error))),
        'physical_mounts_measured': False,
    }
