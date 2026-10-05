"""Explicit units and carton hinge paths; these functions never command motors."""
from __future__ import annotations

import math

import numpy as np

from .common import Refused, finite, vector
from farm.kinematics.units import JointUnits  # compatibility export




def hinge_path(hinge, axis, contact, start_deg, end_deg, step_deg=5):
    """3D contact arc about a measured crease, in metres. Planning only.

    A paddle's orientation and contact force are not solved by this point path.
    The robot Mac must register it to the camera/robot and validate contact.
    """
    hinge, axis, contact = vector(hinge, 3), vector(axis, 3), vector(contact, 3)
    length = np.linalg.norm(axis)
    if length < 1e-8:
        raise Refused("Hinge axis is zero")
    axis /= length
    start_deg, end_deg, step_deg = map(finite, (start_deg, end_deg, step_deg))
    if not 0 < step_deg <= 10 or abs(end_deg-start_deg) > 180:
        raise Refused("Invalid fold angular envelope")
    offset = contact-hinge
    radial = offset-axis*axis.dot(offset)
    if np.linalg.norm(radial) < .001:
        raise Refused("Contact point lies on the hinge")
    angles = np.linspace(start_deg, end_deg, max(2, math.ceil(abs(end_deg-start_deg)/step_deg)+1))
    points = []
    for angle in angles:
        theta = math.radians(angle-start_deg)
        rotated = offset*math.cos(theta) + np.cross(axis, offset)*math.sin(theta)
        rotated += axis*axis.dot(offset)*(1-math.cos(theta))
        points.append((hinge+rotated).tolist())
    return {"units": "metres", "status": "GEOMETRY_ONLY", "angles_deg": angles.tolist(), "contact_points": points,
            "motor_commands": [], "contact_validated": False}


def verify_grasp(before, after, gripper_ticks, empty_closed_ticks, min_aperture_ticks,
                 tool_key="tool", object_key="target", min_lift_px=8.0, max_slip_px=3.0):
    """Independent evidence after a separately bounded test lift in fixed head view.

    Alignment or a blocked jaw alone cannot establish a grasp. Both the tool and
    object must move together, and the jaw must stop short of its empty closure.
    Inputs are tracked points from actual observations, never plan stage names.
    """
    min_aperture_ticks, min_lift_px, max_slip_px = map(finite, (min_aperture_ticks, min_lift_px, max_slip_px))
    if min(min_aperture_ticks, min_lift_px, max_slip_px) <= 0 or max_slip_px >= min_lift_px:
        raise Refused("Grasp needs positive thresholds and slip smaller than the test displacement")
    if not all(0 <= finite(v) <= 4095 for v in (gripper_ticks, empty_closed_ticks)):
        raise Refused("Grasp jaw positions must be encoder ticks")
    if tool_key == object_key:
        raise Refused("Tool and object must be independently tracked")
    bt, bo = vector(before[tool_key], 2), vector(before[object_key], 2)
    at, ao = vector(after[tool_key], 2), vector(after[object_key], 2)
    tool_motion, object_motion = at-bt, ao-bo
    aperture = abs(finite(gripper_ticks)-finite(empty_closed_ticks))
    slip = float(np.linalg.norm(tool_motion-object_motion))
    ok = (np.linalg.norm(tool_motion) >= min_lift_px and np.linalg.norm(object_motion) >= min_lift_px
          and slip <= max_slip_px and aperture >= min_aperture_ticks)
    return {"status": "GRASP_EVIDENCE_PASSED" if ok else "GRASP_NOT_VERIFIED", "verified": bool(ok),
            "tool_motion_px": tool_motion.tolist(), "object_motion_px": object_motion.tolist(),
            "slip_px": slip, "aperture_ticks": aperture}
