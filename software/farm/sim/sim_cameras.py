"""Simulated camera tools for the MuJoCo box-grab bench: robot_get_cameras, robot_get_clip, robot_get_depth.

``SimCameras(world)`` renders the four cameras that ``farm.sim.box_scene.build_scene_xml`` adds (``oak`` 640x360,
``left_wrist``/``right_wrist``/``phone`` 640x480) and answers the three camera tools with the same result and image
shapes as the real robot server (software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/
gemma_robot_tools.py ``cameras()``/``depth_snapshot()`` and frame_clips.py ``burst()``; captured samples are in
farm/sim/assets/sim_robot/real_*.json), so the pilot's eyes model, depth-scene tool and twin view run unchanged:

- ``cameras(names)`` -> ``({'cameras': {name: manifest}, 'camera_errors': {}, 'all_requested_cameras_fresh': True,
  ...}, [image records])``: rendered NOW. Image records carry camera_id/camera_name/mime_type 'image/jpeg'/
  data_base64/captured_at/seq/width/height/sha256/stream_id.
- ``clip({'camera', 'seconds', 'fps', 'max_width'})`` -> a burst from a per-camera ring that the physics step hook
  fills every ``RECORD_INTERVAL_S`` of simulation time (4 s / 32 frames per camera, JPEG bytes), oldest first,
  evenly spaced to the requested fps (at most 12 frames), downscaled to max_width; same limits and defaults as
  frame_clips (seconds 0.5-4, fps 1-8, max_width 160-640; defaults 2/4/480). Frame ages are simulation seconds.
- ``depth()`` -> the OAK RGB JPEG plus a 16-bit PNG of millimetres aligned to it (0 = invalid: closer than
  ``DEPTH_MIN_M`` or beyond ``DEPTH_MAX_M`` like the stereo blind zone, plus ``SPECKLE_FRACTION`` random dropouts and
  a little range noise), with a manifest that ``farm.perception.twin_robot._read_depth`` accepts (projection
  'rectified_pinhole', intrinsics from the camera fovy, depth_units 'mm', invalid_depth 0, width/height, seq,
  stream_id, captured_at/depth_captured_at, rgb_depth_pixel_registration_verified True).

MuJoCo's depth renderer returns metres along the camera z axis (a linearised z-buffer, not ray range), which is
exactly the pinhole depth ``farm.perception.depth_scene`` back-projects; the intrinsics are
``box_scene.intrinsics_for`` (fx = fy from fovy, principal point at the image centre).

Timestamps: ``captured_at``/``received_at`` are wall-clock epoch seconds (``time.time()``) so the pilot's 1-s
freshness checks pass; every record also carries ``sim_time_s`` (the world clock when it was rendered).

``world`` must provide ``.model``, ``.data``, ``.lock`` (a context manager serialising physics and reads),
``.now()`` (simulation seconds) and ``.step_hook(fn)`` (register ``fn`` to be called after every physics step).
``StaticWorld`` is a minimal such world (no controller) for tests and previews. Rendering happens on one worker
thread that owns the renderers (a CGL renderer created on one thread and used from another hangs on macOS);
callers may use any thread. ``render(name, size)`` returns raw RGB pixels for the bench's screenshots.
"""
from __future__ import annotations

import atexit
import base64
import hashlib
import io
import math
import struct
import threading
import time
import uuid
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from farm.sim import box_scene
from farm.sim import xlerobot_twin as twin

CAMERA_NAMES = box_scene.CAMERA_NAMES
CAMERA_SIZES = dict(box_scene.CAMERA_SIZES)
RECORD_INTERVAL_S = 0.125      # ring sampling period in simulation seconds (8 Hz like frame_clips.SAMPLE_HZ)
KEEP_S = 4.0
MAX_FRAMES = 32
STREAMING_S = 2.0              # a burst needs a frame younger than this (simulation seconds)
MIN_FRAMES = 2
MAX_BURST = 12
RING_JPEG_QUALITY = 75
LIVE_JPEG_QUALITY = 85
CLIP_JPEG_QUALITY = 75
LIMITS = {'seconds': (0.5, 4.0), 'fps': (1.0, 8.0), 'max_width': (160, 640)}
DEFAULTS = {'seconds': 2.0, 'fps': 4.0, 'max_width': 480}
NOTE = ('oldest first; timestamps from the camera publisher; receipt time is not capture time for the phone '
        '(simulated cameras: age_s and span_s are simulation seconds)')
DEPTH_MIN_M = 0.25             # stereo blind zone
DEPTH_MAX_M = 8.0
SPECKLE_FRACTION = 0.02
DEPTH_NOISE_FRACTION = 0.004   # 1-sigma range noise as a fraction of the depth
OAK_CAMERA_ID = 'oak-sim'
PHONE_CAMERA_ID = 'phone_overview'
WRIST_CAMERA_IDS = {'left_wrist': 'sim-left-wrist', 'right_wrist': 'sim-right-wrist'}
DEPTH_REASON = ('simulated OAK: depth is the MuJoCo z-buffer in millimetres along the camera axis, registered to the '
                'RGB by construction; the robot transform is the twin camera pose')


class ClipUnavailable(RuntimeError):
    """The ring cannot give a burst for this camera right now (ok false, never an image)."""


# ---------------------------------------------------------------- frame_clips helpers
# Copied from software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/frame_clips.py (same repo) so the
# sim does not import from a docs folder: evenly_spaced, jpeg_size, downscale (PIL path) and check_arguments.

class Frame:
    __slots__ = ('stamp', 'captured_at', 'received_at', 'seq', 'camera_id', 'data', 'sampled_at', 'width', 'height')

    def __init__(self, stamp, captured_at, received_at, seq, camera_id, data, sampled_at, width, height):
        self.stamp, self.captured_at, self.received_at, self.seq = stamp, captured_at, received_at, seq
        self.camera_id, self.data, self.sampled_at, self.width, self.height = camera_id, data, sampled_at, width, height


def evenly_spaced(frames, n):
    """Up to n frames (sorted oldest first) whose stamps best match n evenly spaced times between first and last."""
    if len(frames) <= n:
        return list(frames)
    t0, t1 = frames[0].stamp, frames[-1].stamp
    chosen, j = [], 0
    for k in range(n):
        target = t0 + (t1 - t0) * k / (n - 1)
        while j + 1 < len(frames) and abs(frames[j + 1].stamp - target) <= abs(frames[j].stamp - target):
            j += 1
        if not chosen or frames[j] is not chosen[-1]:
            chosen.append(frames[j])
    return chosen


SOF = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def jpeg_size(data):
    """(width, height) from the JPEG header without decoding, or None."""
    if len(data) < 4 or data[:2] != b'\xff\xd8':
        return None
    i = 2
    while i + 9 <= len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker == 0xFF:
            i += 1
            continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        length = struct.unpack('>H', data[i + 2:i + 4])[0]
        if marker in SOF:
            h, w = struct.unpack('>HH', data[i + 5:i + 9])
            return w, h
        i += 2 + length
    return None


def downscale(data, max_width, quality=CLIP_JPEG_QUALITY):
    """JPEG bytes no wider than max_width -> (bytes, width, height, scaled)."""
    size = jpeg_size(data)
    if size and size[0] <= max_width:
        return data, size[0], size[1], False
    from PIL import Image
    img = Image.open(io.BytesIO(data))
    img.draft('RGB', (max_width, max_width))
    img = img.convert('RGB')
    w, h = img.size
    if w > max_width:
        img = img.resize((max_width, max(1, round(h * max_width / w))), Image.BILINEAR)
    out = io.BytesIO()
    img.save(out, 'JPEG', quality=quality)
    return out.getvalue(), img.width, img.height, True


def check_arguments(args):
    """Fill defaults and bound seconds/fps/max_width (ValueError outside the schema's limits)."""
    out = {}
    for key, (lo, hi) in LIMITS.items():
        value = args.get(key, DEFAULTS[key])
        if type(value) not in (int, float) or not lo <= value <= hi:
            raise ValueError(f'{key} must be a number in {lo}..{hi}')
        if key == 'max_width' and type(value) is not int:
            raise ValueError('max_width must be an integer')
        out[key] = value
    return out


# ---------------------------------------------------------------- encoding

def encode_jpeg(pixels, quality=LIVE_JPEG_QUALITY):
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(pixels)).save(out, format='JPEG', quality=quality)
    return out.getvalue()


def encode_depth_png(depth_mm):
    """uint16 millimetres -> 16-bit grayscale PNG bytes (cv2.imdecode IMREAD_UNCHANGED gives uint16 back)."""
    from PIL import Image
    depth_mm = np.ascontiguousarray(depth_mm, dtype=np.uint16)
    out = io.BytesIO()
    Image.fromarray(depth_mm).save(out, format='PNG')
    return out.getvalue()


def depth_to_mm(depth_m, rng=None, min_m=DEPTH_MIN_M, max_m=DEPTH_MAX_M, speckle=SPECKLE_FRACTION,
                noise=DEPTH_NOISE_FRACTION):
    """MuJoCo depth (float metres along the camera axis) -> uint16 millimetres, 0 where invalid."""
    depth = np.asarray(depth_m, dtype=np.float64)
    valid = np.isfinite(depth) & (depth >= min_m) & (depth <= max_m)
    if rng is not None:
        if noise:
            depth = depth * (1.0 + noise * rng.standard_normal(depth.shape))
        if speckle:
            valid &= rng.random(depth.shape) >= speckle
    mm = np.where(valid, np.rint(depth * 1000.0), 0.0)
    return np.clip(mm, 0, 65535).astype(np.uint16)


# ---------------------------------------------------------------- a world for tests and previews

class StaticWorld:
    """The box scene with no controller: ``step(n)`` advances physics and calls the step hooks.

    ``pose_ticks(ticks, ranges, joint_map=None)`` sets the arm/head joints from encoder ticks with the twin's
    mapping (grippers over the model jaw range) and runs forward kinematics; ``set_joint(name, deg)`` sets one model
    joint. Reads and writes should go through ``with world.lock:``.
    """

    def __init__(self, xml=None, **scene):
        import mujoco
        self.mujoco = mujoco
        self.xml = xml if xml is not None else box_scene.build_scene_xml(**scene)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self._hooks = []
        self.jaw_deg = {}
        for motor, (joint, offset, _) in twin.JOINT_TABLE.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if jid >= 0 and offset is None:
                lo, hi = (float(v) for v in np.degrees(self.model.jnt_range[jid]))
                self.jaw_deg[joint] = (lo, hi) if self.model.jnt_limited[jid] else (0.0, 90.0)
        mujoco.mj_forward(self.model, self.data)

    def now(self):
        return float(self.data.time)

    def step_hook(self, fn):
        self._hooks.append(fn)
        return fn

    def step(self, n=1):
        with self.lock:
            for _ in range(int(n)):
                self.mujoco.mj_step(self.model, self.data)
                for fn in self._hooks:
                    fn(self)

    def set_joint(self, joint, deg):
        jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint)
        if jid < 0:
            raise ValueError(f'no joint {joint!r}')
        with self.lock:
            self.data.qpos[self.model.jnt_qposadr[jid]] = math.radians(deg)
            self.mujoco.mj_forward(self.model, self.data)

    def pose_ticks(self, ticks, ranges, joint_map=None):
        _, unmapped, model_deg = twin.motor_angles(dict(ticks), dict(ranges), joint_map, self.jaw_deg)
        with self.lock:
            for joint, deg in model_deg.items():
                jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint)
                if jid >= 0:
                    self.data.qpos[self.model.jnt_qposadr[jid]] = math.radians(deg)
            self.mujoco.mj_forward(self.model, self.data)
        return unmapped


# ---------------------------------------------------------------- the cameras

class SimCameras:
    """See the module docstring. ``clock`` returns wall-clock epoch seconds (default time.time)."""

    def __init__(self, world, clock=None, *, record=True, seed=0):
        self.world = world
        self.clock = clock or time.time
        self.model, self.data = world.model, world.data
        self.lock = getattr(world, 'lock', None) or threading.RLock()
        box_scene.colour_arms(self.model)
        self.sizes = dict(CAMERA_SIZES)
        self.stream_id = {name: uuid.uuid4().hex for name in CAMERA_NAMES}
        self.seq = {name: 0 for name in CAMERA_NAMES}
        self.rings = {name: deque() for name in CAMERA_NAMES}
        self.samples = {name: 0 for name in CAMERA_NAMES}
        self._ring_lock = threading.Lock()
        self._last_record_s = None
        self._rng = np.random.default_rng(seed)
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='sim-cameras')
        self._renderers = {}         # (width, height) -> Renderer; worker thread only
        self._depth_renderer = None  # worker thread only
        self._closed = False
        self.intrinsics = box_scene.intrinsics_for(self.model, 'oak', self.sizes['oak'])
        import mujoco
        for name in CAMERA_NAMES:
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                raise ValueError(f'the model has no camera {name!r}: build it with box_scene.build_scene_xml')
        if record and hasattr(world, 'step_hook'):
            world.step_hook(self._on_step)
        atexit.register(self.close)

    # ---- worker-thread rendering --------------------------------------------------------------------------
    def _submit(self, fn, *args):
        if self._closed:
            raise RuntimeError('SimCameras is closed')
        return self._executor.submit(fn, *args).result()

    def _renderer(self, size):
        import mujoco
        renderer = self._renderers.get(size)
        if renderer is None:
            renderer = self._renderers[size] = mujoco.Renderer(self.model, height=size[1], width=size[0])
        return renderer

    def _render_rgb(self, name, size):
        renderer = self._renderer(size)
        renderer.update_scene(self.data, camera=name)
        return np.ascontiguousarray(renderer.render())

    def _render_depth(self, name, size):
        import mujoco
        if self._depth_renderer is None or (self._depth_renderer.width, self._depth_renderer.height) != size:
            if self._depth_renderer is not None:
                self._depth_renderer.close()
            self._depth_renderer = mujoco.Renderer(self.model, height=size[1], width=size[0])
            self._depth_renderer.enable_depth_rendering()
        self._depth_renderer.update_scene(self.data, camera=name)
        return np.array(self._depth_renderer.render(), dtype=np.float32)

    def _render_many(self, names, with_depth):
        out = {}
        for name in names:
            size = self.sizes[name]
            rgb = self._render_rgb(name, size)
            depth = self._render_depth(name, size) if (with_depth and name == 'oak') else None
            out[name] = (rgb, depth)
        return out

    def _close_renderers(self):
        for renderer in list(self._renderers.values()):
            try:
                renderer.close()
            except Exception:  # noqa: BLE001 - shutdown must not raise
                pass
        self._renderers.clear()
        if self._depth_renderer is not None:
            try:
                self._depth_renderer.close()
            except Exception:  # noqa: BLE001
                pass
            self._depth_renderer = None

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._executor.submit(self._close_renderers).result(timeout=10)
        except Exception:  # noqa: BLE001 - interpreter shutdown: the worker is gone, close on this thread
            self._close_renderers()
        self._executor.shutdown(wait=False)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- public rendering ----------------------------------------------------------------------------------
    def render(self, name, size=None):
        """RGB pixels (height, width, 3) uint8 of camera ``name`` at the world's current state (no stepping)."""
        if name not in CAMERA_NAMES:
            raise ValueError(f'Unsupported camera: {name}')
        size = tuple(int(v) for v in (size or self.sizes[name]))
        with self.lock:
            return self._submit(self._render_rgb, name, size)

    def render_jpeg(self, name, size=None, quality=LIVE_JPEG_QUALITY):
        return encode_jpeg(self.render(name, size), quality)

    def render_depth(self, name='oak', size=None):
        """Raw MuJoCo depth (metres along the camera axis), float32 (height, width); no blind zone or noise."""
        if name not in CAMERA_NAMES:
            raise ValueError(f'Unsupported camera: {name}')
        size = tuple(int(v) for v in (size or self.sizes[name]))
        with self.lock:
            return self._submit(self._render_depth, name, size)

    # ---- ring (step hook) ----------------------------------------------------------------------------------
    def _on_step(self, *args, **kwargs):
        now = self.world.now()
        if self._last_record_s is not None and now - self._last_record_s < RECORD_INTERVAL_S - 1e-9:
            return
        self.record(now)

    def record(self, sim_time=None):
        """Render every camera into its ring now (called by the step hook; callable directly in tests)."""
        if self._closed:
            return
        sim_time = self.world.now() if sim_time is None else float(sim_time)
        self._last_record_s = sim_time
        wall = self.clock()
        rendered = self._submit(self._render_many, list(CAMERA_NAMES), False)
        with self._ring_lock:
            for name, (rgb, _) in rendered.items():
                self.seq[name] += 1
                data = encode_jpeg(rgb, RING_JPEG_QUALITY)
                frame = Frame(sim_time, wall, wall, self.seq[name], self._camera_id(name), data, wall,
                              rgb.shape[1], rgb.shape[0])
                ring = self.rings[name]
                ring.append(frame)
                self.samples[name] += 1
                while len(ring) > 1 and (len(ring) > MAX_FRAMES or sim_time - ring[0].stamp > KEEP_S):
                    ring.popleft()

    def _camera_id(self, name):
        if name == 'oak':
            return OAK_CAMERA_ID
        if name == 'phone':
            return PHONE_CAMERA_ID
        return WRIST_CAMERA_IDS[name]

    def buffer_status(self, name, now=None):
        now = self.world.now() if now is None else now
        with self._ring_lock:
            ring = self.rings[name]
            return {'frames': len(ring), 'bytes': sum(len(f.data) for f in ring), 'samples': self.samples[name],
                    'newest_age_s': round(now - ring[-1].stamp, 3) if ring else None,
                    'oldest_age_s': round(now - ring[0].stamp, 3) if ring else None,
                    'error': None, 'running': not self._closed}

    # ---- robot_get_cameras ---------------------------------------------------------------------------------
    def cameras(self, names, revive=True):
        """(result, images): every named camera rendered now. Unknown names go to camera_errors."""
        if isinstance(names, str):
            names = [names]
        names = list(names)
        if not names:
            raise ValueError('cameras: name at least one camera')
        errors = {name: {'error': 'Unsupported camera', 'fresh': False} for name in names if name not in CAMERA_NAMES}
        wanted = [n for n in dict.fromkeys(names) if n in CAMERA_NAMES]
        metadata, images = {}, []
        if wanted:
            with self.lock:
                sim_time = self.world.now()
                rendered = self._submit(self._render_many, wanted, True)
            wall = self.clock()
            for name in wanted:
                rgb, depth = rendered[name]
                if name == 'oak':
                    manifest, records = self._oak_records(rgb, depth, wall, sim_time)
                    metadata[name] = manifest
                    images.append(records[0])
                else:
                    manifest, record = self._plain_records(name, rgb, wall, sim_time)
                    metadata[name] = manifest
                    images.append(record)
        result = {'cameras': metadata, 'camera_errors': errors, 'all_requested_cameras_fresh': not errors,
                  'continuous_visual_registration_ready': False, 'simulated': True}
        return result, images

    def _plain_records(self, name, rgb, wall, sim_time):
        self.seq[name] += 1
        seq = self.seq[name]
        data = encode_jpeg(rgb, LIVE_JPEG_QUALITY)
        sha = hashlib.sha256(data).hexdigest()
        height, width = rgb.shape[:2]
        stream = self.stream_id[name]
        camera_id = self._camera_id(name)
        if name == 'phone':
            manifest = {'camera_id': camera_id, 'seq': seq, 'received_at': wall, 'captured_at': wall,
                        'stream_id': stream, 'width': width, 'height': height, 'sha256': sha, 'image': 'latest.jpg',
                        'sim_time_s': sim_time, 'simulated': True}
            record = {'camera_id': camera_id, 'camera_name': name, 'mime_type': 'image/jpeg', 'captured_at': wall,
                      'received_at': wall, 'seq': seq, 'stream_id': stream, 'width': width, 'height': height,
                      'timestamp_semantics': 'simulated: capture and receipt coincide', 'sim_time_s': sim_time,
                      'data_base64': base64.b64encode(data).decode(), 'sha256': sha}
            return manifest, record
        manifest = {'active_min_frame_s': RECORD_INTERVAL_S, 'camera_id': camera_id, 'captured_at': wall,
                    'device_format_fourcc': 'rgb ', 'height': height, 'image': f'{name}-{stream}-{seq}.jpg',
                    'received_at': wall, 'requested_fps': round(1.0 / RECORD_INTERVAL_S), 'sample_pts_s': sim_time,
                    'schema': 1, 'seq': seq, 'sha256': sha, 'stream_id': stream, 'width': width,
                    'sim_time_s': sim_time, 'simulated': True}
        record = {'camera_id': camera_id, 'camera_name': name, 'arm': name.split('_')[0], 'mime_type': 'image/jpeg',
                  'captured_at': wall, 'received_at': wall, 'seq': seq, 'stream_id': stream, 'width': width,
                  'height': height, 'robot_frame_calibrated': True, 'identity_verified': True,
                  'sim_time_s': sim_time, 'data_base64': base64.b64encode(data).decode(), 'sha256': sha}
        return manifest, record

    def _oak_records(self, rgb, depth_m, wall, sim_time):
        """(manifest, [rgb record, depth record]) for one OAK capture."""
        self.seq['oak'] += 1
        seq = self.seq['oak']
        stream = self.stream_id['oak']
        jpeg = encode_jpeg(rgb, LIVE_JPEG_QUALITY)
        depth_mm = depth_to_mm(depth_m, self._rng)
        png = encode_depth_png(depth_mm)
        sha, depth_sha = hashlib.sha256(jpeg).hexdigest(), hashlib.sha256(png).hexdigest()
        height, width = rgb.shape[:2]
        manifest = {
            'device_id': 'sim', 'depthai_version': None, 'usb_speed': None, 'alignment': 'CAM_A RGB',
            'stereo_size': [width, height], 'extended_disparity': False, 'left_right_check': False, 'subpixel': False,
            'fps': round(1.0 / RECORD_INTERVAL_S), 'rgb_undistortion': 'not needed: simulated pinhole camera',
            'intrinsics': [list(row) for row in self.intrinsics], 'distortion_coefficients': [0.0] * 14,
            'distortion_model': 'CameraModel.Perspective', 'projection': 'rectified_pinhole',
            'coordinate_frame': 'CAM_A_optical', 'schema': 1, 'camera_id': OAK_CAMERA_ID, 'stream_id': stream,
            'seq': seq, 'captured_at': wall, 'rgb_captured_at': wall, 'depth_captured_at': wall, 'host': 'sim',
            'width': width, 'height': height, 'image': f'{stream}-{seq:09d}-rgb.jpg', 'sha256': sha,
            'depth_image': f'{stream}-{seq:09d}-depth.png', 'depth_sha256': depth_sha, 'depth_units': 'mm',
            'invalid_depth': 0, 'robot_frame_calibrated': True, 'rgb_depth_pixel_registration_verified': True,
            'depth_range_m': [DEPTH_MIN_M, DEPTH_MAX_M], 'sim_time_s': sim_time, 'simulated': True,
        }
        rgb_record = {'camera_id': OAK_CAMERA_ID, 'camera_name': 'oak', 'mime_type': 'image/jpeg', 'captured_at': wall,
                      'received_at': None, 'seq': seq, 'stream_id': stream, 'width': width, 'height': height,
                      'source': 'sim', 'projection': 'rectified_pinhole', 'rgb_depth_pixel_registration_verified': True,
                      'robot_frame_calibrated': True, 'sim_time_s': sim_time,
                      'data_base64': base64.b64encode(jpeg).decode(), 'sha256': sha}
        depth_record = {'camera_id': OAK_CAMERA_ID + ':depth', 'camera_name': 'oak', 'mime_type': 'image/png',
                        'captured_at': wall, 'received_at': None, 'stream_id': stream, 'seq': seq, 'width': width,
                        'height': height, 'sha256': depth_sha, 'sim_time_s': sim_time,
                        'data_base64': base64.b64encode(png).decode()}
        return manifest, [rgb_record, depth_record]

    # ---- robot_get_depth -----------------------------------------------------------------------------------
    def depth(self, with_rgb=True):
        """(result, images): OAK RGB JPEG (unless with_rgb is False) and the aligned depth PNG, rendered now."""
        with self.lock:
            sim_time = self.world.now()
            rgb, depth_m = self._submit(self._render_many, ['oak'], True)['oak']
        wall = self.clock()
        manifest, records = self._oak_records(rgb, depth_m, wall, sim_time)
        result = {'manifest': manifest, 'source': 'sim', 'powered_depth_observer_ready': False, 'reason': DEPTH_REASON,
                  'sim_time_s': sim_time, 'simulated': True}
        return result, (records if with_rgb else records[1:])

    # ---- robot_get_clip ------------------------------------------------------------------------------------
    def clip(self, args=None):
        """(result, images) like frame_clips.burst: ValueError for bad arguments, ClipUnavailable without frames."""
        args = dict(args or {})
        camera = args.pop('camera', None)
        if camera not in CAMERA_NAMES:
            raise ValueError('Unsupported camera: ' + str(camera))
        want = check_arguments(args)
        now = self.world.now()
        with self._ring_lock:
            frames = list(self.rings[camera])
        if not frames:
            raise ClipUnavailable(f'{camera} camera not streaming: no frames buffered (no physics step recorded yet)')
        newest_age = now - frames[-1].stamp
        if newest_age > STREAMING_S:
            raise ClipUnavailable(f'{camera} camera not streaming: newest frame is {newest_age:.1f} s old')
        window = [f for f in frames if now - f.stamp <= want['seconds'] + 1e-9]
        if len(window) < MIN_FRAMES:
            raise ClipUnavailable(f'{camera}: only {len(window)} frame in the last {want["seconds"]:g} s; a burst needs '
                                  f'{MIN_FRAMES} (buffer holds {len(frames)}, oldest {now - frames[0].stamp:.1f} s old)')
        chosen = evenly_spaced(window, max(MIN_FRAMES, min(MAX_BURST, round(want['seconds'] * want['fps']))))
        rows, images = [], []
        for i, f in enumerate(chosen):
            data, w, h, _ = downscale(f.data, want['max_width'])
            rows.append({'index': i, 'captured_at': f.captured_at, 'received_at': f.received_at, 'seq': f.seq,
                         'age_s': round(now - f.stamp, 3), 'sim_time_s': f.stamp})
            images.append({'camera_id': f.camera_id, 'camera_name': camera, 'mime_type': 'image/jpeg',
                           'data_base64': base64.b64encode(data).decode(), 'captured_at': f.captured_at,
                           'received_at': f.received_at, 'seq': f.seq, 'frame_index': i, 'width': w, 'height': h,
                           'sim_time_s': f.stamp})
        result = {'camera': camera, 'frames': rows, 'count': len(rows),
                  'span_s': round(chosen[-1].stamp - chosen[0].stamp, 3), 'requested': want, 'downscaled_with': 'PIL',
                  'buffer': self.buffer_status(camera, now), 'note': NOTE, 'sim_time_s': now, 'simulated': True}
        return result, images

    # ---- helpers for the bench -----------------------------------------------------------------------------
    def camera_pose(self, name='oak'):
        """The camera's optical frame in the robot frame (same form as xlerobot_twin.camera_pose)."""
        with self.lock:
            return box_scene.camera_pose_from_model(self.model, self.data, name)

    def save_previews(self, folder, names=CAMERA_NAMES):
        """Write <folder>/<name>.jpg for each camera at the current state; returns the paths."""
        from pathlib import Path
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        paths = []
        for name in names:
            path = folder / f'{name}.jpg'
            path.write_bytes(self.render_jpeg(name))
            paths.append(path)
        return paths
