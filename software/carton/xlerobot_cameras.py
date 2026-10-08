"""Robot cameras and arm-base layout from the XLeRobot model, for the fold-policy simulation. Simulation only.

Source of truth (owner, 2026-10-08): the upstream XLeRobot MJCF
`/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/xlerobot/xlerobot.xml`
(`XLEROBOT_MODEL`). The constants below are copied from it, so this module works without the file. The
tests and `model_check()` recompute them from the file when it is present.

Model frame: chassis world frame, robot facing -x (the head camera link's +x and the drive-wheel side),
+y to the robot's right, +z up from the floor.

Mapping to the measurement `arm_base` frame of carton/folding_station_measured.py (+x robot right, +y
forward toward the table, +z up; origin midway between the base origins on their mounting plane):
- x_arm = y_model;
- y_arm = -(x_model - BASE_X);
- z_arm = z_model - BASE_PLANE_Z.

Arm bases: model `Base`/`Base_2` origins at (-0.09, -/+0.11, 0.775). The base spacing is therefore 220 mm.
The legacy `Base` origin sits 45.9 mm above the SO101 base_link mounting plane: the shoulder-pan anchor is
16.5 mm above `Base` but 62.4 mm above SO101 `base_link`. So the mounting plane is at z = 0.7915 - 0.0624
= 0.7291. The model clocks its bases sideways: each shoulder-pan axis is 45.2 mm outboard (+/-y), and the
pan axes are 310 mm apart. With the bases clocked forward (SO101 base_link facing the table, as simulated)
and their origins at +/-0.11, the pan axes are 220 mm apart. Which one the real robot has must be checked
with a tape.

Head camera: pan joint at (-0.103, 0, 1.053), tilt link at +(0.001, 0.002, 0.09815) turned 180 deg about z,
tilt joint about the link's y (positive tilt looks down), camera link at +(0.025, 0, 0.03) in the tilt
link. The camera link is ROS-style: x forward, z up. The MJCF's `head_camera_rgb_frame` copies a URDF rpy
into MuJoCo `euler` and so points along -y; the camera link axes are used instead. Intrinsics: the
model has no <camera>, so the OAK-D Lite colour spec is used. At 4:3 (full IMX214 sensor) that is
VFOV 54 deg / HFOV 69 deg (Luxonis).

Wrist cameras: `Left_Arm_Camera`/`Right_Arm_Camera` meshes on each `Fixed_Jaw`. The legacy fixed jaw and
servo meshes were registered to the simulated SO101 `gripper_link` meshes by ICP (1.7 mm RMS). The camera
module's lens (front face 12 mm across) then sits at (3.5, 68.0, -13.8) mm in `gripper_link`, looking along
-z toward the jaw tips; the claws are offset toward -y in the image. The model has no wrist intrinsics and
no wrist metadata is saved (`STATUS.md`: 640x480 USB boards). The vertical FOV is an assumption
(`WRIST_FOVY_DEG`, 90 deg so the claws are in view as on the real wrist frames); re-render to change it.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

XLEROBOT_MODEL = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/'
                      'robots/xlerobot/xlerobot.xml')
BASE_X = -.09
BASE_Y = .11
BASE_Z = .775
SHOULDER_PAN_ANCHOR_Z = .7915
SO101_PAN_ABOVE_BASE_LINK = .0624
BASE_PLANE_Z = SHOULDER_PAN_ANCHOR_Z - SO101_PAN_ABOVE_BASE_LINK
BASE_SPACING_M = 2 * BASE_Y
HEAD_PAN = np.array([-.103, 0., 1.053])
HEAD_TILT_FROM_PAN = np.array([.001, .002, .09815])
HEAD_CAMERA_FROM_TILT = np.array([.025, 0., .03])
HEAD_TILT_RANGE_RAD = (-.76, 1.45)
OAK_D_LITE_FOVY_DEG = 54.   # colour camera, 4:3 full sensor (Luxonis spec: DFOV 81, HFOV 69, VFOV 54)
WRIST_LENS_IN_GRIPPER = np.array([.0035, .0680, -.0138])
WRIST_AXIS_IN_GRIPPER = np.array([.0037, -.0020, -1.])
WRIST_UP_IN_GRIPPER = np.array([0., 1., 0.])  # image top away from the jaws (claws at the bottom)
# ASSUMPTION: no wrist-camera intrinsics in the model or saved metadata. With the model's lens 68 mm off the
# jaw axis the claw tips sit about 40 deg off the optical axis; the real right-wrist frame (2026-10-07,
# seville-v2/.context/right_wrist.jpg) shows its claw in view, so the vertical half-angle must exceed that.
WRIST_FOVY_DEG = 90.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def model_to_arm_base(p):
    p = np.asarray(p, float)
    return np.array([p[1], -(p[0] - BASE_X), p[2] - BASE_PLANE_Z])


def model_dir_to_arm_base(v):
    v = np.asarray(v, float)
    return np.array([v[1], -v[0], v[2]])


def head_camera_model(tilt_rad, pan_rad=0.):
    """Head camera link position and rotation (columns: forward, left, up) in the model frame."""
    lo, hi = HEAD_TILT_RANGE_RAD
    if not lo <= tilt_rad <= hi:
        raise ValueError(f'head tilt {tilt_rad:.3f} rad outside the model range {HEAD_TILT_RANGE_RAD}')
    r_pan = _rz(pan_rad)
    r_tilt = r_pan @ _rz(math.pi) @ _ry(tilt_rad)
    position = HEAD_PAN + r_pan @ HEAD_TILT_FROM_PAN + r_tilt @ HEAD_CAMERA_FROM_TILT
    return position, r_tilt


def head_camera_spec(tilt_deg, pan_deg=0., fovy_deg=OAK_D_LITE_FOVY_DEG):
    """Measurement-file camera entry (arm_base frame) for the head camera at the given head pan/tilt."""
    position, r = head_camera_model(math.radians(tilt_deg), math.radians(pan_deg))
    p = model_to_arm_base(position)
    forward, up = model_dir_to_arm_base(r[:, 0]), model_dir_to_arm_base(r[:, 2])
    right = np.cross(forward, up)
    rotation_cv = np.column_stack((right, -up, forward))
    return {'position_m': p.round(6).tolist(), 'rotation_cv': rotation_cv.round(9).tolist(),
            'fovy_deg': fovy_deg, 'head_tilt_deg': tilt_deg, 'head_pan_deg': pan_deg,
            'sources': f'{XLEROBOT_MODEL.name}: head_camera_link at head_tilt_joint {tilt_deg} deg, '
                       f'head_pan_joint {pan_deg} deg; OAK-D Lite colour VFOV {fovy_deg} deg (4:3 spec)'}


def wrist_camera_spec(side, fovy_deg=WRIST_FOVY_DEG):
    forward = WRIST_AXIS_IN_GRIPPER / np.linalg.norm(WRIST_AXIS_IN_GRIPPER)
    up = WRIST_UP_IN_GRIPPER - forward * (forward @ WRIST_UP_IN_GRIPPER)
    up /= np.linalg.norm(up)
    right = np.cross(forward, up)
    return {'frame': f'{side}_gripper_link', 'position_m': WRIST_LENS_IN_GRIPPER.round(6).tolist(),
            'rotation_cv': np.column_stack((right, -up, forward)).round(9).tolist(), 'fovy_deg': fovy_deg,
            'sources': f'{XLEROBOT_MODEL.name}: {"Left" if side == "left" else "Right"}_Arm_Camera lens via ICP of '
                       f'Fixed_Jaw onto SO101 gripper_link (1.7 mm RMS); vertical FOV {fovy_deg} deg assumed'}


def station_measurement(tilt_deg, pan_deg=0., base_height=.12, setback=.15, wrist_fovy_deg=WRIST_FOVY_DEG):
    """Measurement dict for the model-derived 220 mm station with the robot's three policy cameras."""
    from carton.folding_station_measured import SCHEMA
    return {
        'schema': SCHEMA, 'measured': False, 'model_derived': True,
        'note': ('Derived from the XLeRobot model (owner: source of truth), not measured on the robot. '
                 'Base spacing from the model Base origins; base height above the carton support and setback '
                 'kept from the simulation. Policy cameras: front (head OAK), left_wrist, right_wrist. '
                 'The overhead camera does not exist on the robot and is not a policy input.'),
        'station': {'base_height_above_table_m': base_height, 'base_line_to_table_edge_m': setback,
                    'base_spacing_m': BASE_SPACING_M, 'carton_near_wall_to_table_edge_m': .01,
                    'park_targets_follow_bases': True,
                    'sources': f'{XLEROBOT_MODEL.name}: Base/Base_2 at y=-/+{BASE_Y}; height/setback: simulation'},
        'cameras': {'front': head_camera_spec(tilt_deg, pan_deg),
                    'left_wrist': wrist_camera_spec('left', wrist_fovy_deg),
                    'right_wrist': wrist_camera_spec('right', wrist_fovy_deg)},
    }


def model_check(path=XLEROBOT_MODEL, tilt_rad=.9, pan_rad=.2):
    """Recompute the head chain and base layout from the MJCF; returns max deviations (m, rad)."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    d.qpos[m.jnt_qposadr[m.joint('head_tilt_joint').id]] = tilt_rad
    d.qpos[m.jnt_qposadr[m.joint('head_pan_joint').id]] = pan_rad
    mujoco.mj_forward(m, d)
    position, r = head_camera_model(tilt_rad, pan_rad)
    cam = d.body('head_camera_link')
    return {'head_position_m': float(np.abs(cam.xpos - position).max()),
            'head_rotation': float(np.abs(cam.xmat.reshape(3, 3) - r).max()),
            'bases_m': float(max(np.abs(d.body('Base').xpos - [BASE_X, -BASE_Y, BASE_Z]).max(),
                                 np.abs(d.body('Base_2').xpos - [BASE_X, BASE_Y, BASE_Z]).max())),
            'pan_anchor_z_m': float(abs(d.xanchor[m.joint('Rotation_L').id][2] - SHOULDER_PAN_ANCHOR_Z))}
