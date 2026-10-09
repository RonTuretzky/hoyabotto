"""Turn the measured OAK-to-arm registration into the simulated head camera pose. Offline.

The pilot's tag registration (tag-registration.json, `base_from_camera`) gives the OAK optical frame in the right
arm's SO-101 base frame (+x forward, +y the arm's left, +z up), validated on held-out poses to about 1 mm / 0.3 deg.
The fold scene places that base at `right_base_link`, so the camera's world pose is base_pose * base_from_camera.
MuJoCo cameras look along -z with +y up, the optical frame looks along +z with +y down, so x stays and y, z flip.

    PYTHONPATH=. python tools/front_camera_from_registration.py --registration <pilot>/.private/tag-registration.json \
        --scene <fold trial>/run/scene.xml --out config/front-camera-measured.json

The output is `record_refit_fold_demos.py --front-camera-pose`. It reports the difference from the scene's assumed
front camera so the change is visible before any demonstration is recorded.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def camera_pose(base_from_camera, scene):
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = model.body('right_base_link').id
    world_from_base = np.eye(4)
    world_from_base[:3, :3] = data.xmat[base].reshape(3, 3)
    world_from_base[:3, 3] = data.xpos[base]
    world_from_camera = world_from_base @ np.asarray(base_from_camera, float)
    rotation = world_from_camera[:3, :3]
    x_axis, y_axis = rotation[:, 0], -rotation[:, 1]          # optical -> MuJoCo camera axes
    front = model.camera('front').id
    assumed = data.cam_xmat[front].reshape(3, 3)
    forward_measured, forward_assumed = -np.cross(x_axis, y_axis), -assumed[:, 2]
    return dict(pos=world_from_camera[:3, 3].round(5).tolist(),
                xyaxes=np.r_[x_axis, y_axis].round(6).tolist(),
                tilt_down_deg=round(math.degrees(math.asin(-forward_measured[2])), 2),
                assumed_pos=data.cam_xpos[front].round(5).tolist(),
                assumed_tilt_down_deg=round(math.degrees(math.asin(-forward_assumed[2])), 2),
                position_change_mm=(1000 * (world_from_camera[:3, 3] - data.cam_xpos[front])).round(1).tolist(),
                view_direction_change_deg=round(math.degrees(math.acos(float(np.clip(
                    forward_measured @ forward_assumed, -1, 1)))), 2))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--registration', type=Path, required=True)
    ap.add_argument('--scene', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    registration = json.loads(args.registration.read_text())
    if registration.get('status') != 'REGISTRATION_VALIDATED' or registration['binding']['arm'] != 'right':
        raise SystemExit('Need a validated right-arm registration')
    pose = camera_pose(registration['base_from_camera'], args.scene)
    pose['source'] = dict(file=str(args.registration), method=registration.get('method'),
                          sha256=hashlib.sha256(args.registration.read_bytes()).hexdigest(),
                          camera_id=registration['binding']['camera_id'],
                          frame='right_base_link * base_from_camera; optical y,z flipped to MuJoCo camera axes')
    args.out.write_text(json.dumps(pose, indent=1))
    print(json.dumps({k: pose[k] for k in ('pos', 'tilt_down_deg', 'assumed_pos', 'assumed_tilt_down_deg',
                                            'position_change_mm', 'view_direction_change_deg')}))


if __name__ == '__main__':
    main()
