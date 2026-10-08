"""Read-only digital twin: render the XLeRobot's current arm/head pose from encoder ticks.

Give it the raw STS3215 ticks the robot reports and the saved calibration ranges; it poses the
upstream XLeRobot MuJoCo model and returns third-person JPEGs ('front', 'left', 'right', 'top') so an
LLM pilot can see how the arms are placed. It never opens a bus, calls the robot API or writes
anything: rendering only.

Mapping (``feetech_degrees_v1``, the same candidate tag registration uses): angle 0 at the middle
of the saved range, sign +1, 4096 ticks per turn (``candidate_degrees`` in
farm/kinematics/tag_registration.py goes through LeRobot, which divides by 4095: within 0.05 deg;
this module avoids the LeRobot import so it runs in any venv with mujoco). That angle is read as
an SO-101 ``so101_new_calib`` URDF joint angle and converted to this model's joint convention by
the fixed per-joint offset/sign in ``JOINT_TABLE`` (a model-to-model conversion, fitted
geometrically; it is not a robot calibration). Grippers follow the repo convention (LeRobot
RANGE_0_100: 0 at range_min) mapped linearly onto the model jaw range; ``angles_deg`` reports a
gripper as its jaw opening in degrees from closed. Head: zero = model forward/level, sign +1, a
pure guess. Nothing about this mapping has been checked against the physical robot:
``mapping_validated`` stays False until a human compares the render with a photo and writes a
joint map with ``validated: true``.

A joint map (``{'validated': bool, 'joints': {motor: {'zero_tick': int, 'sign': 1|-1}}}``)
replaces the midpoint/sign of any listed motor. For a gripper it gives the opening angle from the
model's closed jaw: ``sign * (tick - zero_tick)`` in degrees.

``claw_positions`` poses the same model and reports where each gripper tip is, by forward
kinematics alone (no renderer). The tip is a site added at load time to each ``Fixed_Jaw`` body,
``TIP_POS`` along the jaw: the point where the two jaw tips meet when the gripper is closed (a fixed
point of the fixed jaw, so it does not move when the gripper opens). Positions are given in the
ROBOT frame (``FRAME``): origin on the floor directly below the midpoint between the two shoulder-pan
axes, +forward the robot's front, +left the robot's left, +up, metres. ``reach_m`` is the
straight-line distance from the arm's shoulder point (where its shoulder-pan axis crosses the
shoulder-lift axis height, the centre of the arm's workspace) to the tip; ``shoulder_up_m`` is that
point's height. ``render_twin`` returns the same dict under ``'claws'``.

``camera_pose`` poses the model the same way and reports the OAK head camera's optical frame in the
robot frame: ``position_m`` [forward, left, up] of the lens and a 3x3 ``rotation`` whose columns are the
optical x (image right), y (image down) and z (out of the lens, the viewing direction) axes in robot
coordinates, so a camera point ``[x, y, z]`` maps to ``position + rotation @ [x, y, z]``. The frame is a
site added at load time (``HEAD_SITE``) to the model's ``head_camera_link`` body (ROS camera_link
convention: +x out of the lens, +z up) with the ROS optical convention. The vendored model's own
``head_camera_rgb_optical_frame`` site is NOT used: its ``euler="-1.5708 0 -1.5708"`` is the URDF's
extrinsic rpy, but MuJoCo applies euler intrinsically, so at zero head that site's z axis points to the
robot's LEFT (its x is up and its y forward). Head sign assumptions, both unvalidated: at the mapped
zero (midpoint of each head motor's saved range) the camera looks exactly forward and level; model
``head_tilt_joint`` positive = look DOWN (the axis is the tilt link's +y, the robot's left), so
``head_motor_2`` ticks above its midpoint tilt the camera down; model ``head_pan_joint`` positive = look
LEFT, so ``head_motor_1`` ticks above its midpoint pan left. Checked numerically against the model; a
joint map with ``sign: -1`` for either head motor flips the assumption.

All MuJoCo/OpenGL work runs on one dedicated worker thread that owns the model and renderers,
so ``render_twin`` and ``claw_positions`` may be called from any thread (e.g. a threaded HTTP
server); calls are serialised. On macOS (CGL, the default; leave MUJOCO_GL unset or 'cgl', not
'glfw') a Renderer works in a non-main thread, but one created on a thread and used from another
hangs, hence the single owner thread. First call ~0.5-1.3 s (model load), later 3-view 640x480
calls ~50 ms; ``claw_positions`` well under 1 ms of kinematics plus the thread hand-off.

Model lookup: ``$XLEROBOT_TWIN_MODEL`` (path to an MJCF), else the vendored copy in
``farm/sim/assets/xlerobot/`` (see its README for source and licence).
"""
from __future__ import annotations

import io
import math
import os
import sys
import threading
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

VIEWS = ('front', 'left', 'right', 'top')
VIEW_ALIASES = {'side': 'right'}  # the first version had one side view, from the robot's right
MAPPING = 'feetech_degrees_v1'
TICKS_PER_TURN = 4096
MODEL_ENV = 'XLEROBOT_TWIN_MODEL'
VENDORED_MODEL = Path(__file__).resolve().parent / 'assets' / 'xlerobot' / 'xlerobot.xml'
VENDORED_ID = 'xlerobot.xml (Vector-Wangel/MuJoCo-GS-Web@0d60421, vendored)'
IGNORED_MOTORS = frozenset({'base_left_wheel', 'base_right_wheel'})

# The one table: canonical motor -> (model joint, model offset deg, model sign).
# model joint angle (deg) = offset + sign * feetech_degrees_v1 angle.
# Offsets/signs make the model reproduce the SO-101 so101_new_calib URDF pose for the same joint
# angles (upper arm up, forearm and gripper pointing forward at all zeros). Fitted by comparing
# joint positions/axes of both models (<= 1 mm residual). Wrist roll carries the URDF's 2.79 deg
# wrist_roll frame tilt. Grippers use offset/sign None: linear over the calibrated range.
# The robot faces model -x; the robot's left arm (*_L, model -y) is left_arm_*.
JOINT_TABLE = {
    'left_arm_shoulder_pan':   ('Rotation_L', 90.0, 1),
    'left_arm_shoulder_lift':  ('Pitch_L', 90.0, -1),
    'left_arm_elbow_flex':     ('Elbow_L', 90.0, 1),
    'left_arm_wrist_flex':     ('Wrist_Pitch_L', 0.0, 1),
    'left_arm_wrist_roll':     ('Wrist_Roll_L', 2.79, -1),
    'left_arm_gripper':        ('Jaw_L', None, None),
    'right_arm_shoulder_pan':  ('Rotation_R', -90.0, 1),
    'right_arm_shoulder_lift': ('Pitch_R', 90.0, -1),
    'right_arm_elbow_flex':    ('Elbow_R', 90.0, 1),
    'right_arm_wrist_flex':    ('Wrist_Pitch_R', 0.0, 1),
    'right_arm_wrist_roll':    ('Wrist_Roll_R', 2.79, -1),
    'right_arm_gripper':       ('Jaw_R', None, None),
    'head_motor_1':            ('head_pan_joint', 0.0, 1),   # pan (farm/tools/robot_test.py)
    'head_motor_2':            ('head_tilt_joint', 0.0, 1),  # tilt
}

# Fixed virtual cameras (MuJoCo free camera: lookat, distance, azimuth, elevation) and captions.
# Robot faces -x, its left is -y, arm shoulders sit about 0.78 m above the floor.
CAMERAS = {
    'front': dict(lookat=(-0.2, 0.0, 0.8), distance=1.4, azimuth=0.0, elevation=-10.0,
                  caption="FRONT: facing the robot. Robot's LEFT arm (orange) is on image RIGHT"),
    'left': dict(lookat=(-0.25, 0.0, 0.8), distance=1.4, azimuth=90.0, elevation=-8.0,
                 caption="LEFT SIDE: from robot's left. Robot faces image LEFT; LEFT arm (orange) nearest"),
    'right': dict(lookat=(-0.25, 0.0, 0.8), distance=1.4, azimuth=-90.0, elevation=-8.0,
                  caption="RIGHT SIDE: from robot's right. Robot faces image RIGHT; RIGHT arm (blue) nearest"),
    'top': dict(lookat=(-0.25, 0.0, 0.75), distance=1.25, azimuth=180.0, elevation=-89.9,
                caption="TOP: robot front at image TOP, LEFT arm (orange) on image LEFT. Floor grid 10 cm"),
}
LEFT_RGBA = (0.95, 0.55, 0.15, 1.0)
RIGHT_RGBA = (0.25, 0.55, 0.95, 1.0)
MAX_SIZE = (1920, 1440)
JPEG_QUALITY = 85

# Claw positions. Robot frame axes in model coordinates (the robot faces model -x, its left is -y).
FORWARD, LEFT, UP = (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)
FRAME = ("Origin is the point on the floor directly below the midpoint between the two shoulder-pan joint axes; "
         "+forward is the robot's front (the direction the OAK head camera faces at zero pan), +left is the "
         "robot's left, +up is height above the floor; metres.")
# Tip site: added to each arm's Fixed_Jaw body (the wrist-roll output) at load time. In that body the jaw runs
# along -y; x=+0.008 is where the fixed jaw's tip face (x 0.008..0.037) meets the moving jaw's tip (x<0.008) when
# closed, y=-0.105 is 1.4 mm short of the end of the jaw. Checked against the mesh vertices of the vendored model.
TIP_POS = '0.008 -0.105 0'
# arm -> (shoulder-pan joint, shoulder-lift joint, jaw body, tip site)
ARMS = {
    'left_arm': ('Rotation_L', 'Pitch_L', 'Fixed_Jaw', 'twin_tip_L'),
    'right_arm': ('Rotation_R', 'Pitch_R', 'Fixed_Jaw_2', 'twin_tip_R'),
}
# Head camera: the OAK's optical frame as a site added at load time to the model's head_camera_link body
# (+x out of the lens, +z up). xyaxes gives the ROS optical convention: x = -link y (image right),
# y = -link z (image down), z = x cross y = +link x (out of the lens). See the module docstring for why the
# vendored head_camera_rgb_optical_frame site is not used.
HEAD_CAMERA_BODY = 'head_camera_link'
HEAD_SITE = 'twin_head_optical'
HEAD_SITE_XYAXES = '0 -1 0 0 0 -1'
HEAD_JOINTS = {'pan': 'head_pan_joint', 'tilt': 'head_tilt_joint'}
CAMERAS_BY_NAME = {'oak': HEAD_SITE}
CAMERA_FRAME = (FRAME + " rotation columns are the camera's optical x (image right), y (image down) and z (out of "
                "the lens) axes in that frame; a camera point [x, y, z] in metres sits at position_m + rotation @ [x, y, z].")
HEAD_SIGN_NOTE = ('head zero = midpoint of each head motor\'s saved range, camera level and forward; head_motor_2 '
                  'ticks above the midpoint tilt the camera DOWN, head_motor_1 ticks above pan it LEFT (sign +1, '
                  'unvalidated; a joint map sign of -1 flips either)')


# ---------------------------------------------------------------- mapping (pure, no MuJoCo)

def _range(value):
    if value is None:
        return None
    if isinstance(value, dict):
        value = (value.get('range_min', value.get('min_ticks')), value.get('range_max', value.get('max_ticks')))
    lo, hi = value
    if lo is None or hi is None or not float(hi) > float(lo):
        return None
    return float(lo), float(hi)


def _tick(value):
    if value is None or isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _check_joint_map(joint_map):
    if joint_map is None:
        return {}
    if not isinstance(joint_map, dict):
        raise ValueError('joint_map must be a dict like {"validated": false, "joints": {...}}')
    joints = joint_map.get('joints') or {}
    out = {}
    for motor, entry in joints.items():
        if motor not in JOINT_TABLE:
            raise ValueError(f'joint_map names unknown motor {motor!r}; known: {", ".join(JOINT_TABLE)}')
        entry = entry or {}
        sign = entry.get('sign', 1)
        if sign not in (1, -1) or isinstance(sign, bool):
            raise ValueError(f'joint_map {motor}: sign must be 1 or -1')
        zero = entry.get('zero_tick')
        if zero is not None and (isinstance(zero, bool) or not isinstance(zero, (int, float))
                                 or not 0 <= zero <= TICKS_PER_TURN - 1):
            raise ValueError(f'joint_map {motor}: zero_tick must be an encoder tick 0..4095')
        out[motor] = (None if zero is None else float(zero), int(sign))
    return out


def motor_angles(positions_ticks, ranges, joint_map=None, jaw_range_deg=None):
    """Map ticks to angles without touching MuJoCo.

    Returns (angles_deg, unmapped, model_deg): angles_deg per mapped motor (feetech_degrees_v1, or
    jaw opening from closed for grippers), unmapped motor names, and the model joint angle (deg)
    for every mapped motor. ``jaw_range_deg`` maps joint name -> (closed, open) model degrees.
    """
    overrides = _check_joint_map(joint_map)
    angles, model, unmapped = {}, {}, []
    names = list(dict.fromkeys(list(positions_ticks) + list(JOINT_TABLE)))
    for motor in names:
        if motor in IGNORED_MOTORS:
            continue
        if motor not in JOINT_TABLE:
            unmapped.append(motor)
            continue
        joint, offset, sign = JOINT_TABLE[motor]
        tick = _tick(positions_ticks.get(motor))
        rng = _range(ranges.get(motor))
        zero, map_sign = overrides.get(motor, (None, 1))
        if tick is None or (rng is None and zero is None):
            unmapped.append(motor)
            continue
        if offset is None:  # gripper
            closed, opened = (jaw_range_deg or {}).get(joint, (-21.46, 100.0))
            if zero is None and motor not in overrides:
                fraction = (tick - rng[0]) / (rng[1] - rng[0])
                angle = fraction * (opened - closed)
            else:
                if zero is None:
                    zero = rng[0]
                angle = map_sign * (tick - zero) * 360.0 / TICKS_PER_TURN
            angles[motor] = angle
            model[joint] = closed + angle
            continue
        if zero is None:
            zero = (rng[0] + rng[1]) / 2.0
        angle = map_sign * (tick - zero) * 360.0 / TICKS_PER_TURN
        angles[motor] = angle
        model[joint] = offset + sign * angle
    return angles, unmapped, model


# ---------------------------------------------------------------- model lookup

def find_model():
    """Return (path, short id) of the MJCF to render, or raise FileNotFoundError."""
    override = os.environ.get(MODEL_ENV)
    if override:
        path = Path(override).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'{MODEL_ENV}={override} does not name an MJCF file')
        return path.resolve(), str(path)
    if VENDORED_MODEL.is_file():
        return VENDORED_MODEL, VENDORED_ID
    raise FileNotFoundError(
        f'XLeRobot twin model not found: set {MODEL_ENV} to xlerobot.xml (with its assets/ folder) '
        f'or restore the vendored copy at {VENDORED_MODEL} (see farm/sim/assets/xlerobot/README.md)')


def _scene_xml(path):
    """Upstream MJCF made fixed-base and render-ready (floor, lights, offscreen buffer)."""
    root = ET.parse(path).getroot()
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    meshdir = compiler.get('meshdir', '')
    compiler.set('meshdir', str((Path(path).parent / meshdir).resolve()))
    world = root.find('worldbody')
    tip_sites = {jaw: site for _, _, jaw, site in ARMS.values()}
    for body in world.iter('body'):
        for free in body.findall('freejoint'):
            body.remove(free)  # base and wheels are fixed
        site = tip_sites.get(body.get('name'))
        if site is not None:  # claw tip marker; group 4 is never drawn (sitegroup is all off anyway)
            ET.SubElement(body, 'site', name=site, pos=TIP_POS, size='0.003', group='4', rgba='1 0 0 0')
        if body.get('name') == HEAD_CAMERA_BODY:  # OAK optical frame (ROS convention) at the camera link origin
            ET.SubElement(body, 'site', name=HEAD_SITE, pos='0 0 0', xyaxes=HEAD_SITE_XYAXES, size='0.003',
                          group='4', rgba='0 1 0 0')
    for tag in ('actuator', 'tendon', 'keyframe', 'sensor'):  # wheel tendons/actuators unused
        for el in root.findall(tag):
            root.remove(el)
    visual = root.find('visual')
    if visual is None:
        visual = ET.Element('visual')
        root.insert(0, visual)
    ET.SubElement(visual, 'global', offwidth=str(MAX_SIZE[0]), offheight=str(MAX_SIZE[1]))
    ET.SubElement(visual, 'headlight', ambient='0.45 0.45 0.45', diffuse='0.55 0.55 0.55', specular='0.05 0.05 0.05')
    ET.SubElement(visual, 'quality', shadowsize='2048')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', name='twin_sky', type='skybox', builtin='gradient',
                  rgb1='0.82 0.86 0.9', rgb2='0.55 0.6 0.66', width='256', height='256')
    ET.SubElement(asset, 'texture', name='twin_grid', type='2d', builtin='checker', rgb1='0.78 0.78 0.76',
                  rgb2='0.7 0.7 0.68', mark='edge', markrgb='0.45 0.45 0.45', width='200', height='200')
    ET.SubElement(asset, 'material', name='twin_floor', texture='twin_grid', texrepeat='5 5', texuniform='true',
                  reflectance='0')
    ET.SubElement(world, 'geom', name='twin_floor', type='plane', size='3 3 0.01', pos='0 0 0', material='twin_floor',
                  contype='0', conaffinity='0', group='0')
    ET.SubElement(world, 'light', pos='-1.5 -1.0 2.5', dir='0.5 0.35 -0.8', directional='true',
                  diffuse='0.35 0.35 0.35', castshadow='false')
    return ET.tostring(root, encoding='unicode')


# ---------------------------------------------------------------- renderer (one worker thread)

class _Twin:
    """Owns MjModel/MjData/Renderers. Only ever touched from the worker thread."""

    def __init__(self, path):
        if sys.platform == 'darwin':
            os.environ.setdefault('MUJOCO_GL', 'cgl')
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_string(_scene_xml(path))
        self.data = mujoco.MjData(self.model)
        self.renderers = {}
        self.qadr, self.jaw_deg = {}, {}
        for motor, (joint, offset, _) in JOINT_TABLE.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0:
                continue  # an override model without this joint: reported as unmapped
            self.qadr[joint] = int(self.model.jnt_qposadr[jid])
            if offset is None:
                lo, hi = (float(v) for v in np.degrees(self.model.jnt_range[jid]))
                self.jaw_deg[joint] = (lo, hi) if self.model.jnt_limited[jid] else (0.0, 90.0)
        self._colour_arms()
        self.option = mujoco.MjvOption()
        self.option.sitegroup[:] = 0
        self.cameras = {}
        for name, spec in CAMERAS.items():
            cam = mujoco.MjvCamera()
            cam.lookat[:] = spec['lookat']
            cam.distance, cam.azimuth, cam.elevation = spec['distance'], spec['azimuth'], spec['elevation']
            self.cameras[name] = cam
        self._find_arms()

    def _find_arms(self):
        """Resolve each arm's pan/lift joints and tip site; fix the robot frame from the (static) pan axes."""
        mj, np = self.mj, self.np
        self.arms = {}
        for arm, (pan, lift, jaw, site) in ARMS.items():
            ids = (mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, pan),
                   mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, lift),
                   mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, site))
            if min(ids) >= 0:
                self.arms[arm] = ids + (f'{jaw}/{site}',)
        self.origin = None
        if len(self.arms) == len(ARMS):
            self.pose({})  # the pan joints sit on the fixed base, so their anchors never move
            anchors = np.array([self.data.xanchor[ids[0]] for ids in self.arms.values()])
            self.origin = anchors.mean(axis=0)
            self.origin[2] = 0.0
        self.axes = np.array([FORWARD, LEFT, UP])
        self.head_site = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, HEAD_SITE)  # -1: no head camera body

    def camera(self):
        """The OAK optical frame in the robot frame for the pose set by ``pose``; None if the model lacks the
        head camera body or the robot frame is undefined."""
        if self.origin is None or self.head_site < 0:
            return None
        rotation = self.axes @ self.data.site_xmat[self.head_site].reshape(3, 3)
        position = self.axes @ (self.data.site_xpos[self.head_site] - self.origin)
        return {'position_m': [float(v) for v in position],
                'rotation': [[float(v) for v in row] for row in rotation],
                'site': f'{HEAD_CAMERA_BODY}/{HEAD_SITE}', 'frame': CAMERA_FRAME}

    def pose(self, model_deg):
        """Set the joint angles (deg, model convention) and run forward kinematics; no rendering."""
        self.data.qpos[:] = 0
        for joint, deg in model_deg.items():
            if joint in self.qadr:
                self.data.qpos[self.qadr[joint]] = math.radians(deg)
        self.mj.mj_kinematics(self.model, self.data)

    def claws(self):
        """Claw tips in the robot frame for the pose set by ``pose``; None if the frame is undefined."""
        if self.origin is None:
            return None
        np = self.np
        out = {}
        for arm, (pan, lift, site, tip_site) in self.arms.items():
            tip = self.data.site_xpos[site]
            shoulder = self.data.xanchor[pan].copy()
            shoulder[2] = self.data.xanchor[lift][2]  # on the pan axis, at the lift axis height
            f, l, u = (self.axes @ (tip - self.origin)).tolist()
            _, sl, su = (self.axes @ (shoulder - self.origin)).tolist()
            out[arm] = {'forward_m': f, 'left_m': l, 'up_m': u,
                        'reach_m': float(np.linalg.norm(tip - shoulder)),
                        'shoulder_up_m': su, 'shoulder_left_m': sl, 'tip_site': tip_site}
        out['frame'] = FRAME
        return out

    def _colour_arms(self):
        m, mj = self.model, self.mj
        roots = {}
        for side, base in (('L', 'Base'), ('R', 'Base_2')):
            bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, base)
            if bid >= 0:
                roots[bid] = LEFT_RGBA if side == 'L' else RIGHT_RGBA
        for g in range(m.ngeom):
            body = int(m.geom_bodyid[g])
            while body > 0 and body not in roots:
                body = int(m.body_parentid[body])
            if body in roots and m.geom_group[g] == 2:
                matid = int(m.geom_matid[g])
                dark = matid >= 0 and m.mat_rgba[matid][:3].max() < 0.2  # keep servo bodies dark
                if not dark:
                    m.geom_matid[g] = -1
                    m.geom_rgba[g] = roots[body]

    def render(self, model_deg, views, size):
        mj, np = self.mj, self.np
        self.pose(model_deg)
        renderer = self.renderers.get(size)
        if renderer is None:
            renderer = self.renderers[size] = mj.Renderer(self.model, height=size[1], width=size[0])
        images = []
        for view in views:
            renderer.update_scene(self.data, camera=self.cameras[view], scene_option=self.option)
            pixels = renderer.render()
            images.append({'view': view, 'mime_type': 'image/jpeg',
                           'data': _jpeg(np.ascontiguousarray(pixels), CAMERAS[view]['caption'])})
        return images


def _jpeg(pixels, caption):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        import cv2
        bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(bgr, caption, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        return buf.tobytes()
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)
    height = max(14, image.height // 32)
    try:
        font = ImageFont.load_default(size=height - 3)
    except TypeError:  # Pillow < 10.1
        font = ImageFont.load_default()
    draw.rectangle((0, 0, image.width, height + 4), fill=(0, 0, 0))
    draw.text((6, 2), caption, fill=(255, 255, 255), font=font)
    out = io.BytesIO()
    image.save(out, format='JPEG', quality=JPEG_QUALITY)
    return out.getvalue()


_LOCK = threading.Lock()
_EXECUTOR = None
_TWINS = {}


def _executor():
    global _EXECUTOR
    with _LOCK:
        if _EXECUTOR is None:
            _EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix='xlerobot-twin')
        return _EXECUTOR


def _twin(path):  # worker thread only
    twin = _TWINS.get(path)
    if twin is None:
        twin = _TWINS[path] = _Twin(path)
    return twin


def _work(path, positions, ranges, joint_map, views, size):
    """Worker-thread body: pose the model; render ``views`` (None: kinematics only) and read the claws."""
    twin = _twin(path)
    angles, unmapped, model_deg = motor_angles(positions, ranges, joint_map, twin.jaw_deg)
    for motor in list(angles):
        if JOINT_TABLE[motor][0] not in twin.qadr:
            angles.pop(motor)
            unmapped.append(motor)
            model_deg.pop(JOINT_TABLE[motor][0], None)
    if views is None:
        twin.pose(model_deg)
        images = None
    else:
        images = twin.render(model_deg, views, size)
    return angles, unmapped, images, twin.claws(), twin.camera()


def _mapping_fields(joint_map):
    return {
        'mapping': MAPPING if joint_map is None else MAPPING + '+joint_map',
        'mapping_validated': bool(joint_map is not None and joint_map.get('validated') is True),
    }


def render_twin(positions_ticks, ranges, *, views=VIEWS, size=(640, 480), joint_map=None):
    """Render the robot's pose from encoder ticks. See module docstring.

    Returns {'images': [{'view', 'mime_type', 'data'}...], 'angles_deg', 'unmapped', 'mapping',
    'mapping_validated', 'model', 'claws'} ('claws' as from ``claw_positions``, or None when the
    model lacks the arms).
    """
    views = tuple(VIEW_ALIASES.get(v, v) for v in views)
    bad = [v for v in views if v not in CAMERAS]
    if bad or not views:
        raise ValueError(f'unknown view(s) {bad}; choose from {VIEWS}')
    width, height = (int(v) for v in size)
    if not (16 <= width <= MAX_SIZE[0] and 16 <= height <= MAX_SIZE[1]):
        raise ValueError(f'size must be within 16x16..{MAX_SIZE[0]}x{MAX_SIZE[1]}')
    _check_joint_map(joint_map)  # fail on the caller's thread with a clear message
    path, model_id = find_model()
    angles, unmapped, images, claws, _ = _executor().submit(
        _work, path, dict(positions_ticks), dict(ranges), joint_map, views, (width, height)).result()
    fields = _mapping_fields(joint_map)
    if claws is not None:
        claws.update(fields, unmapped=list(unmapped), model=model_id)
    return {
        'images': images,
        'angles_deg': angles,
        'unmapped': unmapped,
        **fields,
        'model': model_id,
        'claws': claws,
    }


def claw_positions(positions_ticks, ranges, *, joint_map=None):
    """Where each gripper tip is, in the robot frame, from encoder ticks: forward kinematics, no rendering.

    Returns {'left_arm': {'forward_m', 'left_m', 'up_m', 'reach_m', 'shoulder_up_m', 'shoulder_left_m',
    'tip_site'}, 'right_arm': {...}, 'frame', 'mapping', 'mapping_validated', 'unmapped', 'model'}. An arm
    the model lacks is simply absent. Raises RuntimeError if the model has neither shoulder-pan joint
    (the frame is undefined), ValueError for a bad joint_map, FileNotFoundError for a missing model.
    """
    _check_joint_map(joint_map)
    path, model_id = find_model()
    _, unmapped, _, claws, _ = _executor().submit(
        _work, path, dict(positions_ticks), dict(ranges), joint_map, None, None).result()
    if claws is None:
        raise RuntimeError(f'model {model_id} lacks the shoulder-pan joints/jaw bodies of {", ".join(ARMS)}: '
                           'no robot frame for claw positions')
    claws.update(_mapping_fields(joint_map), unmapped=list(unmapped), model=model_id)
    return claws


def camera_pose(positions_ticks, ranges, *, joint_map=None, camera='oak'):
    """The head camera's optical frame in the robot frame, from encoder ticks: forward kinematics, no rendering.

    Returns {'position_m': [forward, left, up], 'rotation': 3x3 (columns = optical x, y, z axes in the robot
    frame), 'frame', 'site', 'camera', 'head_angles_deg': {'pan', 'tilt'} (feetech_degrees_v1, None when that
    motor is unmapped and the model keeps it at zero), 'head_sign_note', 'mapping', 'mapping_validated',
    'unmapped', 'model'}. Only camera='oak' (the head camera) exists; ValueError otherwise, or for a bad
    joint_map; RuntimeError if the model lacks the head camera body or the shoulder-pan joints (no robot frame);
    FileNotFoundError for a missing model.
    """
    if camera not in CAMERAS_BY_NAME:
        raise ValueError(f'unknown camera {camera!r}; the model has: {", ".join(CAMERAS_BY_NAME)}')
    _check_joint_map(joint_map)
    path, model_id = find_model()
    angles, unmapped, _, _, pose = _executor().submit(
        _work, path, dict(positions_ticks), dict(ranges), joint_map, None, None).result()
    if pose is None:
        raise RuntimeError(f'model {model_id} lacks the {HEAD_CAMERA_BODY} body or the shoulder-pan joints of '
                           f'{", ".join(ARMS)}: no camera pose in the robot frame')
    pose.update(_mapping_fields(joint_map), camera=camera, unmapped=list(unmapped), model=model_id,
                head_angles_deg={'pan': angles.get('head_motor_1'), 'tilt': angles.get('head_motor_2')},
                head_sign_note=HEAD_SIGN_NOTE)
    return pose


# ---------------------------------------------------------------- CLI (renders to files)

def _load_snapshot(path):
    """Accept {'positions': {...}, 'ranges': {...}} or a farm encoder dump {'motors': [...]}."""
    import json
    raw = json.loads(Path(path).read_text())
    if 'motors' in raw:
        positions = {m['name']: m.get('Present_Position') for m in raw['motors']}
        ranges = {m['name']: tuple(m['range']) for m in raw['motors'] if m.get('range')}
        return positions, ranges
    return raw['positions'], {k: tuple(v) if isinstance(v, list) else v for k, v in raw['ranges'].items()}


def main(argv=None):
    import argparse
    import json
    ap = argparse.ArgumentParser(description='Render the XLeRobot twin from an encoder snapshot (no hardware).')
    ap.add_argument('snapshot', help='JSON: {"positions":{motor:tick}, "ranges":{motor:[min,max]}} or an encoder dump')
    ap.add_argument('--out', default='data/twin', help='folder for <view>.jpg (default data/twin)')
    ap.add_argument('--joint-map', help='JSON joint map {"validated":false,"joints":{motor:{"zero_tick":..,"sign":..}}}')
    ap.add_argument('--size', default='640x480')
    args = ap.parse_args(argv)
    positions, ranges = _load_snapshot(args.snapshot)
    joint_map = json.loads(Path(args.joint_map).read_text()) if args.joint_map else None
    size = tuple(int(v) for v in args.size.lower().split('x'))
    result = render_twin(positions, ranges, joint_map=joint_map, size=size)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for image in result['images']:
        (out / f"{image['view']}.jpg").write_bytes(image['data'])
    summary = {k: v for k, v in result.items() if k != 'images'}
    summary['files'] = [str(out / f"{i['view']}.jpg") for i in result['images']]
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
