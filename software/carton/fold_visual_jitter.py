"""Per-episode visual perturbations for the fold policy's cameras. Simulation only.

The physical head camera pose, the wrist camera mounts and the lighting are not calibrated to the model
(docs/carton-fold-policy-station-gap.md). Training on small random camera-pose, light and image-gain changes
makes the policy tolerate those registration errors instead of memorising one exact view. Closed-loop
evaluation applies the same jitter, so a reported success rate includes it. AprilTags, geometry, the camera
intrinsics (the OAK contract) and the contact gates are unchanged.
"""
from __future__ import annotations

import math

import numpy as np

CAMERAS = ('front', 'left_wrist', 'right_wrist')
# Magnitudes at scale 1: uniform +/- these.
SPEC = dict(front_pos_m=.010, front_rot_deg=2.0, wrist_pos_m=.003, wrist_rot_deg=2.0,
            light=.20, gain=.15, contrast=.10, channel=.03)


def episode_rng(seed, salt=0x5EED):
    return np.random.default_rng([int(seed), int(salt)])


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


def _axis_angle_quat(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[math.cos(angle / 2), axis * math.sin(angle / 2)]


def jitter_cameras(model, rng, scale=1.0, cameras=CAMERAS, spec=SPEC):
    """Perturb camera poses (model.cam_pos / cam_quat, parent-body frame) and light intensity in place."""
    applied = {}
    if scale <= 0:
        return applied
    for name in cameras:
        try:
            cid = model.camera(name).id
        except KeyError:
            continue
        wrist = 'wrist' in name
        dpos = rng.uniform(-1, 1, 3) * (spec['wrist_pos_m'] if wrist else spec['front_pos_m']) * scale
        axis = rng.normal(size=3)
        angle = math.radians(rng.uniform(-1, 1) * (spec['wrist_rot_deg'] if wrist else spec['front_rot_deg']) * scale)
        model.cam_pos[cid] += dpos
        model.cam_quat[cid] = _quat_mul(model.cam_quat[cid], _axis_angle_quat(axis, angle))
        applied[name] = dict(dpos_m=dpos.round(5).tolist(), angle_deg=round(math.degrees(angle), 3))
    if model.nlight:
        factor = 1 + rng.uniform(-1, 1) * spec['light'] * scale
        model.light_diffuse[:] *= factor
        applied['light_factor'] = round(float(factor), 4)
    return applied


class ImageJitter:
    """Fixed per-episode gain, contrast and per-channel balance, like one camera's exposure for a whole run."""
    def __init__(self, rng, scale=1.0, spec=SPEC):
        self.gain = 1 + rng.uniform(-1, 1) * spec['gain'] * scale
        self.contrast = 1 + rng.uniform(-1, 1) * spec['contrast'] * scale
        self.channel = 1 + rng.uniform(-1, 1, 3) * spec['channel'] * scale
        self.identity = scale <= 0

    def __call__(self, image):
        if self.identity:
            return image
        x = image.astype(np.float32)
        mean = x.mean()
        x = ((x - mean) * self.contrast + mean) * self.gain * self.channel
        return np.clip(x, 0, 255).astype(np.uint8)

    def report(self):
        return dict(gain=round(float(self.gain), 4), contrast=round(float(self.contrast), 4),
                    channel=np.round(self.channel, 4).tolist())
