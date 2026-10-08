"""Read-only digital twin: render the XLeRobot's current arm/head pose from encoder ticks.

Give it the raw STS3215 ticks the robot reports and the saved calibration ranges; it poses the
upstream XLeRobot MuJoCo model and returns third-person JPEGs ('front', 'side', 'top') so an
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

All MuJoCo/OpenGL work runs on one dedicated worker thread that owns the model and renderers,
so ``render_twin`` may be called from any thread (e.g. a threaded HTTP server); calls are
serialised. On macOS (CGL, the default; leave MUJOCO_GL unset or 'cgl', not 'glfw') a Renderer works
in a non-main thread, but one created on a thread and used from another hangs, hence the single
owner thread. First call ~0.5-1.3 s (model load), later 3-view 640x480 calls ~50 ms.

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

VIEWS = ('front', 'side', 'top')
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
    'side': dict(lookat=(-0.25, 0.0, 0.8), distance=1.4, azimuth=-90.0, elevation=-8.0,
                 caption="SIDE: from robot's right. Robot faces image RIGHT; RIGHT arm (blue) nearest"),
    'top': dict(lookat=(-0.25, 0.0, 0.75), distance=1.25, azimuth=180.0, elevation=-89.9,
                caption="TOP: robot front at image TOP, LEFT arm (orange) on image LEFT. Floor grid 10 cm"),
}
LEFT_RGBA = (0.95, 0.55, 0.15, 1.0)
RIGHT_RGBA = (0.25, 0.55, 0.95, 1.0)
MAX_SIZE = (1920, 1440)
JPEG_QUALITY = 85


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
    for body in world.iter('body'):
        for free in body.findall('freejoint'):
            body.remove(free)  # base and wheels are fixed
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
        self.data.qpos[:] = 0
        for joint, deg in model_deg.items():
            if joint in self.qadr:
                self.data.qpos[self.qadr[joint]] = math.radians(deg)
        mj.mj_kinematics(self.model, self.data)
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
    twin = _twin(path)
    angles, unmapped, model_deg = motor_angles(positions, ranges, joint_map, twin.jaw_deg)
    for motor in list(angles):
        if JOINT_TABLE[motor][0] not in twin.qadr:
            angles.pop(motor)
            unmapped.append(motor)
            model_deg.pop(JOINT_TABLE[motor][0], None)
    return angles, unmapped, twin.render(model_deg, views, size)


def render_twin(positions_ticks, ranges, *, views=VIEWS, size=(640, 480), joint_map=None):
    """Render the robot's pose from encoder ticks. See module docstring.

    Returns {'images': [{'view', 'mime_type', 'data'}...], 'angles_deg', 'unmapped', 'mapping',
    'mapping_validated', 'model'}.
    """
    views = tuple(views)
    bad = [v for v in views if v not in CAMERAS]
    if bad or not views:
        raise ValueError(f'unknown view(s) {bad}; choose from {VIEWS}')
    width, height = (int(v) for v in size)
    if not (16 <= width <= MAX_SIZE[0] and 16 <= height <= MAX_SIZE[1]):
        raise ValueError(f'size must be within 16x16..{MAX_SIZE[0]}x{MAX_SIZE[1]}')
    _check_joint_map(joint_map)  # fail on the caller's thread with a clear message
    path, model_id = find_model()
    angles, unmapped, images = _executor().submit(
        _work, path, dict(positions_ticks), dict(ranges), joint_map, views, (width, height)).result()
    return {
        'images': images,
        'angles_deg': angles,
        'unmapped': unmapped,
        'mapping': MAPPING if joint_map is None else MAPPING + '+joint_map',
        'mapping_validated': bool(joint_map is not None and joint_map.get('validated') is True),
        'model': model_id,
    }


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
