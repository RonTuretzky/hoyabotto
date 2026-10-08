"""Observe major flap planes from aligned RGB-D and calibrated task geometry.

This module accepts pixels, pinhole intrinsics and measured rigid transforms.
It has no simulator dependency. Priors can reject discontinuous observations;
they never supply missing pixels or turn an occluded panel into an observation.
The carton frame is the folding station's bottom-centred, z-up frame.
"""
from __future__ import annotations

import cv2
import numpy as np

from carton.geometry import Box


def _rigid(value, name):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1])
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            or np.linalg.det(pose[:3, :3]) < .999):
        raise ValueError(f'{name} must be a finite proper rigid transform')
    return pose


def _plane(points):
    center = np.mean(points, axis=0)
    _, _, axes = np.linalg.svd(points - center, full_matrices=False)
    normal = axes[-1]
    # Coordinates are along hinge, inward from hinge, upwards from hinge.
    angle = np.degrees(np.arctan2(-normal[2], normal[1]))
    if angle < -77 or angle > 103:
        normal = -normal
    return normal, float(center @ normal)


def _candidate(points, seed):
    angle = np.radians(seed)
    normal = np.array([0., np.cos(angle), -np.sin(angle)])
    distance = np.einsum('ij,j->i', points, normal)
    # A seed uses the hinge only to find a neighbourhood; the plane fit is
    # unconstrained, so an arbitrary arm surface cannot inherit hinge alignment.
    select = np.abs(distance) < .007
    if np.count_nonzero(select) < 60:
        return None
    for _ in range(4):
        normal, offset = _plane(points[select])
        residual = np.abs(np.einsum('ij,j->i', points, normal) - offset)
        select = residual < .0025
        if np.count_nonzero(select) < 60:
            return None
    cloud = points[select]
    normal, offset = _plane(cloud)
    angle = float(np.degrees(np.arctan2(-normal[2], normal[1])))
    hinge_axis_error = float(np.degrees(np.arcsin(np.clip(abs(normal[0]), 0, 1))))
    if not -40 < angle < 103 or hinge_axis_error > 4. or abs(offset) > .006:
        return None
    direction = np.array([0., np.sin(np.radians(angle)), np.cos(np.radians(angle))])
    radius = np.einsum('ij,j->i', cloud, direction)
    along_span = float(np.ptp(np.percentile(cloud[:, 0], [5, 95])))
    radial_span = float(np.ptp(np.percentile(radius, [5, 95])))
    if along_span < .075 or radial_span < .030:
        return None
    # Dense pixels on a small tool face do not replace distributed panel area.
    patches = np.floor(np.column_stack((cloud[:, 0], radius)) / .025).astype(int)
    _, counts = np.unique(patches, axis=0, return_counts=True)
    patch_count = int(np.count_nonzero(counts >= 8))
    if patch_count < 6:
        return None
    residual = np.abs(np.einsum('ij,j->i', cloud, normal) - offset)
    return {
        'degrees': angle, 'pixel_support': int(len(cloud)),
        'median_deviation_deg': float(np.degrees(np.arctan2(np.median(residual), np.median(radius)))),
        'plane_rms_mm': float(np.sqrt(np.mean(residual ** 2)) * 1000),
        'hinge_plane_offset_mm': float(offset * 1000),
        'hinge_axis_error_deg': hinge_axis_error,
        'along_span_mm': along_span * 1000, 'radial_span_mm': radial_span * 1000,
        'supported_patches': patch_count,
        'method': 'aligned_depth_hinge_consistent_plane',
        '_inliers': select,
    }


def depth_major_flap_angles(rgb, depth, k, world_from_camera, world_from_box,
                           priors=None, *, box=None, hinge_height_offset=.0035):
    """Return fresh visible ``long_near``/``long_far`` angles, omitting uncertainty.

    ``box`` supplies measured dimensions (defaults to the declared task carton).
    ``hinge_height_offset`` is the measured major/minor hinge height difference.
    RGB must be uint8 RGB and depth optical-axis metres registered to that RGB.
    Callers remain responsible for synchronization and fresh registration.

    A broad cardboard colour mask proposes surfaces, then unconstrained plane
    fits must align with the declared hinge and have distributed two-dimensional
    support. Near-horizontal planes additionally need visible cardboard in the
    gap between the short flaps, where those underlying flaps cannot explain it.
    This intentionally omits a fully obscured major even with a closed prior.
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
    dimensions = np.asarray([box.length, box.width, box.height, box.flap, hinge_height_offset])
    if (not np.isfinite(dimensions).all() or np.any(dimensions[:4] <= 0)
            or not 0 <= hinge_height_offset < .02 or box.flap >= box.length / 2):
        raise ValueError('Finite measured carton dimensions with a short-flap gap required')
    for name, prior in (priors or {}).items():
        if name in ('long_near', 'long_far') and not np.isfinite(prior):
            raise ValueError('Finite angle priors required')

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    good = ((hsv[:, :, 0] >= 8) & (hsv[:, :, 0] <= 32)
            & (hsv[:, :, 1] >= 45) & (hsv[:, :, 1] <= 185)
            & np.isfinite(depth) & (depth > .1) & (depth < 2.))
    yy, xx = np.nonzero(good)
    z = depth[yy, xx]
    camera_points = np.column_stack(((xx - k[0, 2]) * z / k[0, 0],
                                     (yy - k[1, 2]) * z / k[1, 1], z))
    box_from_camera = np.linalg.inv(world_from_box) @ world_from_camera
    points = (np.einsum('ij,kj->ik', camera_points, box_from_camera[:3, :3])
              + box_from_camera[:3, 3])
    result = {}
    for name, inward in (('long_near', 1), ('long_far', -1)):
        local = points.copy()
        local[:, 1] = inward * points[:, 1] + box.width / 2
        local[:, 2] -= box.height + hinge_height_offset
        radius = np.hypot(local[:, 1], local[:, 2])
        observed_angle = np.degrees(np.arctan2(local[:, 1], local[:, 2]))
        keep = ((np.abs(local[:, 0]) < box.length / 2 - .008)
                & (radius > .025) & (radius < box.flap - .004)
                & (observed_angle > -40) & (observed_angle < 103))
        local, observed_angle = local[keep], observed_angle[keep]
        if len(local) < 60:
            continue
        histogram, edges = np.histogram(observed_angle, bins=np.arange(-42, 108, 3))
        peaks = [index for index in np.argsort(histogram)[::-1]
                 if histogram[index] >= 35]
        candidates = []
        for index in peaks:
            seed = float(edges[index] + 1.5)
            if any(abs(seed - candidate['degrees']) < 7 for candidate in candidates):
                continue
            candidate = _candidate(local, seed)
            if candidate is None:
                continue
            if priors and name in priors and abs(candidate['degrees'] - priors[name]) >= 35:
                continue
            inliers = candidate.pop('_inliers')
            # Only nearly horizontal short flaps can mimic a plane parallel
            # to the major hinge. Require identity-bearing pixels in their gap.
            cloud = local[inliers]
            central = np.abs(cloud[:, 0]) < box.length / 2 - box.flap - .008
            candidate['short_gap_pixel_support'] = int(np.count_nonzero(central))
            if candidate['degrees'] > 65:
                if np.count_nonzero(central) < 35:
                    continue
                theta = np.radians(candidate['degrees'])
                gap_radius = cloud[central, 1] * np.sin(theta) + cloud[central, 2] * np.cos(theta)
                # A thin intersection with an unrelated centre occluder is
                # insufficient to label the two underlying short-flap planes.
                if np.ptp(np.percentile(gap_radius, [5, 95])) < .030:
                    continue
            if not any(abs(candidate['degrees'] - item['degrees']) < 4 for item in candidates):
                candidates.append(candidate)
        candidates.sort(key=lambda item: item['pixel_support'], reverse=True)
        if not candidates:
            continue
        # Two credible different hinge planes mean unresolved identity. A
        # prior is a rejection gate, never permission to choose a weak patch.
        if (len(candidates) > 1
                and candidates[1]['pixel_support'] >= .35 * candidates[0]['pixel_support']):
            continue
        result[name] = candidates[0]
    return result
