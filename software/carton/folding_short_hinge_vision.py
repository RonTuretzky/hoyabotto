"""Fresh RGB-D hinge planes for outward/upright/partially inward short flaps.

This deliberately does not identify closed, nearly horizontal short flaps.
The open-angle interval is part of the observation method, not a clipping rule.
No simulator state, segmentation, contact, commanded angle or default pose is
an input. Caller owns fresh calibration, synchronization and carton registration.
"""
from __future__ import annotations

import cv2
import numpy as np

from carton.folding_hinge_vision import _candidate, _rigid
from carton.geometry import Box


OPEN_SHORT_ANGLE_BOUNDS_DEGREES = (-40., 30.)


def depth_open_short_flap_angles(rgb, depth, k, world_from_camera, world_from_box,
                                 priors=None, *, box=None):
    """Return supported short hinge planes strictly between -40 and +30 deg.

    Reuses the major observer's unconstrained plane, axis/offset, span and
    six-patch support gates. A second distinct plane with at least 35% of the
    leading support refuses the identity. Priors only reject discontinuities.
    Returned omissions must not be filled from priors or a commanded angle.
    """
    rgb, depth, k = np.asarray(rgb), np.asarray(depth), np.asarray(k, dtype=float)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('uint8 RGB image required')
    if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
        raise ValueError('Metric depth must be aligned to RGB')
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
        raise ValueError('Finite zero-skew pinhole intrinsics required')
    world_from_camera = _rigid(world_from_camera, 'Camera pose')
    world_from_box = _rigid(world_from_box, 'Carton pose')
    box = Box() if box is None else box
    dimensions = np.asarray([box.length, box.width, box.height, box.flap])
    if (not np.isfinite(dimensions).all() or np.any(dimensions <= 0)
            or box.flap >= box.length / 2 or box.flap <= .050 or box.width <= .100):
        raise ValueError('Finite positive measured carton dimensions with a short-flap gap required')
    for name, prior in (priors or {}).items():
        if name in ('short_left', 'short_right') and not np.isfinite(prior):
            raise ValueError('Finite angle priors required')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    good = ((hsv[:, :, 0] >= 8) & (hsv[:, :, 0] <= 32)
            & (hsv[:, :, 1] >= 45) & (hsv[:, :, 1] <= 185)
            & np.isfinite(depth) & (depth > .1) & (depth < 2.))
    yy, xx = np.nonzero(good)
    z = depth[yy, xx]
    camera_points = np.column_stack(((xx - k[0, 2]) * z / k[0, 0],
                                     (yy - k[1, 2]) * z / k[1, 1], z))
    transform = np.linalg.inv(world_from_box) @ world_from_camera
    points = (np.einsum('ij,kj->ik', camera_points, transform[:3, :3])
              + transform[:3, 3])
    result = {}
    lower, upper = OPEN_SHORT_ANGLE_BOUNDS_DEGREES
    for name, inward in (('short_left', 1), ('short_right', -1)):
        # Local coordinates: along the short hinge, inward, above the hinge.
        local = np.column_stack((points[:, 1], inward * points[:, 0] + box.length / 2,
                                 points[:, 2] - box.height))
        radius = np.hypot(local[:, 1], local[:, 2])
        observed_angle = np.degrees(np.arctan2(local[:, 1], local[:, 2]))
        keep = ((np.abs(local[:, 0]) < box.width / 2 - .004)
                & (radius > .045) & (radius < box.flap - .002)
                & (observed_angle > lower) & (observed_angle < upper))
        local, observed_angle = local[keep], observed_angle[keep]
        if len(local) < 60:
            continue
        hist, edges = np.histogram(observed_angle, bins=np.arange(-42, 36, 3))
        candidates = []
        for index in np.argsort(hist)[::-1]:
            if hist[index] < 35:
                continue
            seed = float(edges[index] + 1.5)
            if any(abs(seed - item['degrees']) < 7 for item in candidates):
                continue
            candidate = _candidate(local, seed)
            if candidate is None or not lower < candidate['degrees'] < upper:
                continue
            if priors and name in priors and abs(candidate['degrees'] - priors[name]) >= 35:
                continue
            candidate.pop('_inliers')
            if not any(abs(candidate['degrees'] - item['degrees']) < 4 for item in candidates):
                candidates.append(candidate)
        candidates.sort(key=lambda item: item['pixel_support'], reverse=True)
        if not candidates or (len(candidates) > 1
                and candidates[1]['pixel_support'] >= .35 * candidates[0]['pixel_support']):
            continue
        row = candidates[0]
        row.update(method='aligned_depth_open_short_hinge_consistent_plane',
                   valid_angle_bounds_degrees=list(OPEN_SHORT_ANGLE_BOUNDS_DEGREES),
                   competing_plane_support_ratio=(candidates[1]['pixel_support'] / row['pixel_support']
                                                   if len(candidates) > 1 else 0.))
        result[name] = row
    return result
