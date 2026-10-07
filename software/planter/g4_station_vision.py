"""Frame-bound rendered RGB and aligned depth for G4 perception experiments.

Camera intrinsics and marker sizes are explicit simulation mount assumptions.
This adapter returns images, never object poses or robot motion targets.
"""
from __future__ import annotations

import base64
import hashlib
import io
import math

import numpy as np
from PIL import Image


def geometry_for_cameras(cameras):
    geometry = dict(schema=1, family='tag36h11', camera_ids=list(cameras), tags={})
    for tag, size in [(1, 40), (2, 40), (4, 40), (41, 20)]:
        geometry['tags'][str(tag)] = dict(
            black_square_mm=size,
            source='Declared g4_station.py MJCF marker mount; physical size/mount unmeasured')
    for tag, arm in [(2, 'right'), (4, 'left')]:
        geometry['tags'][str(tag)]['mount'] = dict(
            arm=arm, body='fixed_gripper_housing',
            source='Rigid marker body attached to original SO101 gripper_link in G4 MJCF')
    return geometry


def camera_envelope(rgb, *, camera, stream_id, seq, timestamp, intrinsics):
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('Expected RGB uint8 image')
    if not camera or not stream_id or type(seq) is not int or seq < 1 or not math.isfinite(timestamp):
        raise ValueError('Explicit camera, stream, sequence and finite capture time required')
    k = np.asarray(intrinsics, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError('Finite matching camera intrinsics required')
    buf = io.BytesIO(); Image.fromarray(rgb).save(buf, format='PNG'); raw = buf.getvalue()
    identity = dict(camera_id=camera, stream_id=stream_id, seq=seq,
                    sha256=hashlib.sha256(raw).hexdigest())
    frame = dict(**identity, captured_at=timestamp, received_at=timestamp,
                 projection='rectified_pinhole', mime_type='image/png',
                 data_base64=base64.b64encode(raw).decode())
    metadata = dict(**identity, width=rgb.shape[1], height=rgb.shape[0],
                    rgb_captured_at=timestamp, projection='rectified_pinhole',
                    coordinate_frame=camera+'_optical', intrinsics=k.tolist())
    return dict(ok=True, result=dict(cameras={camera: metadata}), images=[frame], simulation_only=True)


def capture(renderer, model, data, camera, *, option, stream_id, seq,
            timestamp, rng, noise_sigma_m=.0008, dropout_probability=.25):
    """One unchanged simulator state supplies RGB and its aligned optical-Z depth."""
    if noise_sigma_m < 0 or not math.isfinite(noise_sigma_m) or not 0 <= dropout_probability <= 1:
        raise ValueError('Invalid explicit depth-noise hypothesis')
    cid = model.camera(camera).id
    renderer.disable_depth_rendering()
    renderer.update_scene(data, camera=camera, scene_option=option)
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    try:
        renderer.update_scene(data, camera=camera, scene_option=option)
        depth = renderer.render().copy()
    finally:
        renderer.disable_depth_rendering()
    height, width = rgb.shape[:2]
    focal = height/(2*math.tan(math.radians(model.cam_fovy[cid])/2))
    k = [[focal, 0, width/2], [0, focal, height/2], [0, 0, 1]]
    payload = camera_envelope(rgb, camera=camera, stream_id=stream_id, seq=seq,
                              timestamp=timestamp, intrinsics=k)
    depth += rng.normal(0, noise_sigma_m, depth.shape)
    depth[rng.random(depth.shape) < dropout_probability] = np.nan
    frame = payload['images'][0]
    binding = {key: frame[key] for key in ('camera_id', 'stream_id', 'seq', 'sha256', 'captured_at')}
    binding.update(rgb_sha256=binding.pop('sha256'), depth_captured_at=timestamp,
                   aligned_to='rgb', units='metres', depth_kind='optical_z',
                   noise_sigma_m=noise_sigma_m, dropout_probability=dropout_probability,
                   simulation_only=True, physics_time_s=float(data.time))
    return payload, depth, binding
