"""Frame bursts: the last few seconds of one camera as timestamped JPEG frames (video for the vision model).

The camera publishers (OAK, phone, both wrists) each write their newest frame to disk with a small JSON manifest.
A sampler thread polls those manifests at about 8 Hz and keeps the newest frames of each camera in a bounded ring
(4 s, 32 frames, ~1.6 MB per camera). The hot path only reads files and compares manifest keys: nothing is decoded or
re-encoded until robot_get_clip asks for a burst, which is then downscaled. Reads files only; never opens a camera.
"""
import base64
import hashlib
import io
import json
import struct
import threading
import time
from collections import deque
from pathlib import Path

CAMERAS = ('oak', 'phone', 'left_wrist', 'right_wrist')
SAMPLE_HZ = 8
KEEP_S = 4.0
MAX_FRAMES = 32
BUDGET_BYTES = 32 * 50 * 1024    # per camera; larger frames mean fewer of them are kept
STREAMING_S = 2.0                # a burst needs a frame younger than this
MIN_FRAMES = 2
MAX_BURST = 12
JPEG_QUALITY = 75
LOG_EVERY_S = 60
LIMITS = {'seconds': (0.5, 4.0), 'fps': (1.0, 8.0), 'max_width': (160, 640)}
DEFAULTS = {'seconds': 2.0, 'fps': 4.0, 'max_width': 480}
NOTE = 'oldest first; timestamps from the camera publisher; receipt time is not capture time for the phone'


class ClipUnavailable(RuntimeError):
    """The buffer cannot give a burst for this camera right now (ok false, never an image)."""


class Source:
    """How one camera's newest frame is found. read() returns (manifest dict, image Path) or raises; the manifest
    must carry the stamp field (captured_at, or received_at for the phone) and seq; sha256 is checked when present."""
    def __init__(self, read, stamp='captured_at', camera_id=None):
        self.read, self.stamp, self.camera_id = read, stamp, camera_id


def read_manifest(manifest, image=None):
    """Manifest JSON plus the image path next to it: image is a fixed file name (the phone's latest.jpg) or, when
    None, the manifest's own image field (hashed OAK/wrist files). The path must stay inside the manifest's folder."""
    manifest = Path(manifest)
    meta = json.loads(manifest.read_text())
    path = (manifest.parent / (image or meta['image'])).resolve()
    if not path.is_relative_to(manifest.parent.resolve()):
        raise ValueError('image path outside camera directory')
    return meta, path


class Frame:
    __slots__ = ('stamp', 'captured_at', 'received_at', 'seq', 'camera_id', 'data', 'sampled_at')

    def __init__(self, stamp, captured_at, received_at, seq, camera_id, data, sampled_at):
        self.stamp, self.captured_at, self.received_at, self.seq = stamp, captured_at, received_at, seq
        self.camera_id, self.data, self.sampled_at = camera_id, data, sampled_at


class FrameRing:
    """Bounded per-camera frame rings filled by one daemon thread. select()/status() lock only around the deques."""
    def __init__(self, sources, hz=SAMPLE_HZ, keep_s=KEEP_S, max_frames=MAX_FRAMES, budget_bytes=BUDGET_BYTES,
                 log=None, clock=time.time):
        self.sources, self.hz, self.keep_s = dict(sources), hz, keep_s
        self.max_frames, self.budget_bytes, self.clock = max_frames, budget_bytes, clock
        self.log = log or (lambda m: print(m, flush=True))
        self.lock = threading.Lock()
        self.rings = {name: deque() for name in self.sources}
        self.bytes = {name: 0 for name in self.sources}
        self.last_key = {name: None for name in self.sources}
        self.errors = {name: None for name in self.sources}          # {'error','count','since'} while a camera fails
        self._last_log = {name: 0.0 for name in self.sources}
        self.samples = {name: 0 for name in self.sources}
        self._stop = threading.Event()
        self.thread = None
        self.started_at = None

    # ---- sampler thread -------------------------------------------------------------------------------------
    def start(self):
        if self.thread and self.thread.is_alive():
            return self
        self._stop.clear(); self.started_at = self.clock()
        self.thread = threading.Thread(target=self.run, name='frame-clips', daemon=True)
        self.thread.start()
        return self

    def stop(self, timeout=2.0):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout)

    def running(self):
        return bool(self.thread and self.thread.is_alive())

    def run(self):
        period = 1.0 / self.hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.tick()
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def tick(self):
        """One sampling pass over every camera; a failing camera never stops the others or the thread."""
        for name in self.sources:
            try:
                self.sample(name)
            except Exception as exc:  # noqa: BLE001 - anything a publisher can do to a file must be survived
                self._complain(name, exc)
            else:
                if self.errors[name]:
                    with self.lock:
                        self.errors[name] = None

    def sample(self, name):
        """Store the camera's newest frame if the manifest changed since the last sample. Returns True when stored."""
        source = self.sources[name]
        meta, path = source.read()
        stamp = meta[source.stamp]
        key = (meta.get('seq'), meta.get('sha256'), stamp)
        if key == self.last_key[name]:
            return False
        data = path.read_bytes()
        if meta.get('sha256'):
            if hashlib.sha256(data).hexdigest() != meta['sha256']:
                return False                       # publisher mid-write: try again next tick
        else:
            again, _ = source.read()               # mutable image file (phone): the manifest must not have moved
            if again.get(source.stamp) != stamp or again.get('seq') != meta.get('seq'):
                return False
        if len(data) > self.budget_bytes:
            raise ValueError(f'frame of {len(data)} bytes exceeds the {self.budget_bytes}-byte ring budget')
        frame = Frame(stamp, meta.get('captured_at'), meta.get('received_at'), meta.get('seq'),
                      meta.get('camera_id') or source.camera_id, data, self.clock())
        self.last_key[name] = key
        with self.lock:
            ring = self.rings[name]
            ring.append(frame); self.bytes[name] += len(data); self.samples[name] += 1
            self._prune(name, frame.stamp)
        return True

    def _prune(self, name, newest_stamp):
        """Under self.lock: drop frames beyond the age, count or byte budget (the newest frame always stays)."""
        ring = self.rings[name]
        while len(ring) > 1 and (len(ring) > self.max_frames or self.bytes[name] > self.budget_bytes
                                 or newest_stamp - ring[0].stamp > self.keep_s):
            self.bytes[name] -= len(ring.popleft().data)

    def _complain(self, name, exc):
        now = self.clock(); message = f'{type(exc).__name__}: {exc}'
        with self.lock:
            old = self.errors[name] or {'count': 0, 'since': now}
            self.errors[name] = {'error': message, 'count': old['count'] + 1, 'since': old['since']}
        if now - self._last_log[name] >= LOG_EVERY_S:
            self._last_log[name] = now
            try:
                self.log(f'frame clips: {name} not sampled ({message}); logged at most once a minute')
            except Exception:  # noqa: BLE001 - logging must never kill the sampler
                pass

    # ---- readers ----------------------------------------------------------------------------------------------
    def snapshot(self, name):
        with self.lock:
            return list(self.rings[name])

    def status(self, name=None, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            names = [name] if name else list(self.sources)
            out = {}
            for n in names:
                ring = self.rings[n]
                out[n] = {'frames': len(ring), 'bytes': self.bytes[n], 'samples': self.samples[n],
                          'newest_age_s': round(now - ring[-1].stamp, 3) if ring else None,
                          'oldest_age_s': round(now - ring[0].stamp, 3) if ring else None,
                          'error': dict(self.errors[n]) if self.errors[n] else None}
        if name:
            return dict(out[name], running=self.running())
        return {'running': self.running(), 'sample_hz': self.hz, 'keep_s': self.keep_s, 'cameras': out}

    def select(self, name, seconds, fps, now=None):
        """Frames of the last `seconds`, evenly spaced to about `fps` (at most MAX_BURST), oldest first."""
        if name not in self.sources:
            raise ValueError('Unsupported camera: ' + str(name))
        now = self.clock() if now is None else now
        frames = self.snapshot(name)
        if not frames:
            why = self.errors[name]['error'] if self.errors[name] else ('sampler not running' if not self.running() else 'no frame seen yet')
            raise ClipUnavailable(f'{name} camera not streaming: no frames buffered ({why})')
        newest_age = now - frames[-1].stamp
        if newest_age > STREAMING_S:
            raise ClipUnavailable(f'{name} camera not streaming: newest frame is {newest_age:.1f} s old')
        window = [f for f in frames if now - f.stamp <= seconds]
        if len(window) < MIN_FRAMES:
            raise ClipUnavailable(f'{name}: only {len(window)} frame in the last {seconds:g} s; a burst needs {MIN_FRAMES} '
                                  f'(buffer holds {len(frames)}, oldest {now - frames[0].stamp:.1f} s old)')
        return evenly_spaced(window, max(MIN_FRAMES, min(MAX_BURST, round(seconds * fps))))


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


# ---- downscaling (only on a tool call) -----------------------------------------------------------------------------
def _find_scaler():
    try:
        import PIL.Image  # noqa: F401
        return 'PIL'
    except ImportError:
        pass
    try:
        import cv2, numpy  # noqa: F401,E401
        return 'cv2'
    except ImportError:
        return None


SCALER = _find_scaler()
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
            i += 1; continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2; continue
        length = struct.unpack('>H', data[i + 2:i + 4])[0]
        if marker in SOF:
            h, w = struct.unpack('>HH', data[i + 5:i + 9])
            return w, h
        i += 2 + length
    return None


def downscale(data, max_width, quality=JPEG_QUALITY, scaler='auto'):
    """JPEG bytes no wider than max_width -> (bytes, width, height, scaled). Frames already narrow enough pass through
    untouched; without PIL or OpenCV (scaler None) the full frame is returned, width/height from the header when readable."""
    scaler = SCALER if scaler == 'auto' else scaler
    size = jpeg_size(data)
    if size and size[0] <= max_width:
        return data, size[0], size[1], False
    if scaler == 'PIL':
        from PIL import Image
        img = Image.open(io.BytesIO(data))
        img.draft('RGB', (max_width, max_width))   # DCT-domain reduction: decodes at 1/2, 1/4 or 1/8 size cheaply
        img = img.convert('RGB')
        w, h = img.size
        if w > max_width:
            img = img.resize((max_width, max(1, round(h * max_width / w))), Image.BILINEAR)
        out = io.BytesIO(); img.save(out, 'JPEG', quality=quality)
        return out.getvalue(), img.width, img.height, True
    if scaler == 'cv2':
        import cv2, numpy as np  # noqa: E401
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError('undecodable JPEG')
        h, w = img.shape[:2]
        if w > max_width:
            img = cv2.resize(img, (max_width, max(1, round(h * max_width / w))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            raise ValueError('JPEG encode failed')
        return buf.tobytes(), img.shape[1], img.shape[0], True
    return data, size[0] if size else None, size[1] if size else None, False


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


def burst(ring, camera, args=None, now=None):
    """The robot_get_clip tool: (result, images) or ClipUnavailable/ValueError."""
    want = check_arguments(args or {})
    now = ring.clock() if now is None else now
    frames = ring.select(camera, want['seconds'], want['fps'], now)
    rows, images = [], []
    for i, f in enumerate(frames):
        data, w, h, scaled = downscale(f.data, want['max_width'])
        rows.append({'index': i, 'captured_at': f.captured_at, 'received_at': f.received_at, 'seq': f.seq,
                     'age_s': round(now - f.stamp, 3)})
        images.append({'camera_id': f.camera_id, 'camera_name': camera, 'mime_type': 'image/jpeg',
                       'data_base64': base64.b64encode(data).decode(), 'captured_at': f.captured_at,
                       'received_at': f.received_at, 'seq': f.seq, 'frame_index': i, 'width': w, 'height': h})
    result = {'camera': camera, 'frames': rows, 'count': len(rows), 'span_s': round(frames[-1].stamp - frames[0].stamp, 3),
              'requested': want, 'downscaled_with': SCALER, 'buffer': ring.status(camera, now), 'note': NOTE}
    return result, images
