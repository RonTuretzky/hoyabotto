"""robot_get_clip: sampling from fake publisher folders, spacing, downscale, refusals, bounded memory, a sampler that
survives broken manifests, and the API registration (schema, argument validation, redeploy lists). No camera, no network."""
import ast, base64, hashlib, json, os, re, tempfile, threading, time
from pathlib import Path
import frame_clips
from frame_clips import FrameRing, Source, ClipUnavailable, read_manifest, burst, downscale, jpeg_size, evenly_spaced
from wrist_cameras import select_wrist_manifest, WRIST_CAMERA_IDS

HERE = Path(__file__).resolve().parent


def make_jpeg(i, w=640, h=480):
    """A JPEG with a block that moves with i (PIL, else OpenCV, else header-only bytes the scaler passes through)."""
    if frame_clips.SCALER == 'PIL':
        from PIL import Image, ImageDraw
        img = Image.new('RGB', (w, h), (40, 60 + i % 100, 90)); d = ImageDraw.Draw(img)
        x = (i * 23) % (w - 80); d.rectangle([x, h // 3, x + 80, h // 3 + 80], fill=(230, 230, 230))
        out = __import__('io').BytesIO(); img.save(out, 'JPEG', quality=85); return out.getvalue()
    if frame_clips.SCALER == 'cv2':
        import cv2, numpy as np
        img = np.full((h, w, 3), (90, 60 + i % 100, 40), np.uint8); x = (i * 23) % (w - 80)
        img[h // 3:h // 3 + 80, x:x + 80] = 230
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
    return b'\xff\xd8' + bytes([0xff, 0xc0, 0, 17, 8]) + h.to_bytes(2, 'big') + w.to_bytes(2, 'big') + bytes(2000 + i)


class Publisher:
    """Writes manifests the way the real publishers do: hashed immutable files (oak, wrists) or a rewritten latest.jpg (phone)."""
    def __init__(self, tmp):
        self.oak, self.phone, self.wrist = tmp / 'oak', tmp / 'phone', tmp / 'wrist'
        for d in (self.oak, self.phone, self.wrist): d.mkdir()
        self.seq = {'oak': 0, 'phone': 0, 'right_wrist': 0, 'left_wrist': 0}
        self.rates = {'oak': 15, 'phone': 4, 'right_wrist': 5, 'left_wrist': 5}
        self.enabled = {n: True for n in self.seq}
        self.stop = threading.Event(); self.thread = threading.Thread(target=self.run, daemon=True)

    @staticmethod
    def atomic(path, meta):   # the real publishers replace the manifest atomically
        tmp = path.with_suffix('.tmp'); tmp.write_text(json.dumps(meta)); os.replace(tmp, path)

    def publish(self, name, at=None):
        at = time.time() if at is None else at; self.seq[name] += 1; seq = self.seq[name]; data = make_jpeg(seq)
        if name == 'oak':
            image = f'oak-{seq}.jpg'; (self.oak / image).write_bytes(data)
            self.atomic(self.oak / 'oak.json', {'camera_id': 'oak-test', 'stream_id': 's', 'seq': seq, 'captured_at': at, 'image': image,
                                                'sha256': hashlib.sha256(data).hexdigest(), 'depth_image': 'none.png', 'depth_sha256': ''})
        elif name == 'phone':
            (self.phone / 'latest.jpg').write_bytes(data)
            self.atomic(self.phone / 'latest.json', {'seq': seq, 'received_at': at})
        else:
            image = f'{name}-s-{seq}.jpg'; (self.wrist / image).write_bytes(data)
            self.atomic(self.wrist / (name + '.json'), {'schema': 1, 'camera_id': WRIST_CAMERA_IDS[name], 'stream_id': 's', 'seq': seq, 'captured_at': at,
                                                        'received_at': at + .01, 'image': image, 'sha256': hashlib.sha256(data).hexdigest(), 'width': 640, 'height': 480})

    def run(self):
        due = {n: time.monotonic() for n in self.seq}
        while not self.stop.is_set():
            now = time.monotonic()
            for n, hz in self.rates.items():
                if self.enabled[n] and now >= due[n]:
                    self.publish(n); due[n] += 1 / hz
            time.sleep(.005)


def wrist_source(name, dirs):
    def read():
        folder, m = select_wrist_manifest(name, dirs); return m, folder / m['image']
    return read


logged = []
with tempfile.TemporaryDirectory() as tmp:
  tmp = Path(tmp); pub = Publisher(tmp)
  ring = FrameRing({'oak': Source(lambda: read_manifest(pub.oak / 'oak.json')),
                    'phone': Source(lambda: read_manifest(pub.phone / 'latest.json', image='latest.jpg'), stamp='received_at', camera_id='phone_overview'),
                    'left_wrist': Source(wrist_source('left_wrist', [pub.wrist])),
                    'right_wrist': Source(wrist_source('right_wrist', [pub.wrist]))}, log=logged.append)
  try:
    # Nothing published yet: the sampler must run anyway and refuse cleanly.
    ring.start(); time.sleep(.3)
    assert ring.running()
    for name in frame_clips.CAMERAS:
        try: ring.select(name, 2, 4)
        except ClipUnavailable as e: assert 'not streaming' in str(e), e
        else: raise AssertionError('empty ring gave a clip')
    assert len(logged) == 4 and all('at most once a minute' in m for m in logged), logged   # one line per camera, not per tick
    logged.clear()
    pub.thread.start(); t_start = time.time(); time.sleep(2.6); elapsed = time.time() - t_start
    st = ring.status()
    assert st['running'] and all(st['cameras'][n]['error'] is None for n in frame_clips.CAMERAS), st
    oak_n, phone_n = st['cameras']['oak']['frames'], st['cameras']['phone']['frames']
    assert 12 <= oak_n <= 8 * elapsed + 2, (oak_n, elapsed)           # 15 fps publisher sampled at <= 8 Hz
    assert 6 <= phone_n <= 4 * elapsed + 2, (phone_n, elapsed)        # 4 fps publisher: every frame, none twice
    for n in frame_clips.CAMERAS:
        seqs = [f.seq for f in ring.snapshot(n)]; stamps = [f.stamp for f in ring.snapshot(n)]
        assert len(set(seqs)) == len(seqs) and stamps == sorted(stamps), n
    # The burst: oldest first, about fps frames per second, never more than 12, downscaled, chat-compatible ids.
    result, images = burst(ring, 'oak', {'seconds': 2, 'fps': 4, 'max_width': 320})
    assert 6 <= result['count'] <= 8 and result['count'] == len(images) == len(result['frames']), result['count']
    assert 1.4 <= result['span_s'] <= 2.05, result['span_s']
    ages = [f['age_s'] for f in result['frames']]
    assert ages == sorted(ages, reverse=True) and ages[-1] <= 2 and ages[0] <= 2.05, ages
    gaps = [a - b for a, b in zip(ages, ages[1:])]
    assert max(gaps) <= .5 and min(gaps) >= .1, gaps                   # roughly even at 4 fps from an 8 Hz ring
    assert [f['index'] for f in result['frames']] == list(range(result['count']))
    assert set(result) >= {'camera', 'frames', 'count', 'span_s', 'requested', 'note'} and result['requested'] == {'seconds': 2, 'fps': 4, 'max_width': 320}
    assert set(result['frames'][0]) == {'index', 'captured_at', 'received_at', 'seq', 'age_s'}
    for i, im in enumerate(images):
        assert set(im) == {'camera_id', 'camera_name', 'mime_type', 'data_base64', 'captured_at', 'received_at', 'seq', 'frame_index', 'width', 'height'}, set(im)
        assert im['camera_id'] == 'oak-test' and im['camera_name'] == 'oak' and im['mime_type'] == 'image/jpeg' and im['frame_index'] == i
        raw = base64.b64decode(im['data_base64']); assert raw[:2] == b'\xff\xd8'
        if frame_clips.SCALER:
            assert (im['width'], im['height']) == (320, 240) and jpeg_size(raw) == (320, 240) and len(raw) < 30000, (im['width'], im['height'], len(raw))
        else:
            assert (im['width'], im['height']) == (640, 480)           # no PIL/cv2: full frames, header dimensions
    r4, im4 = burst(ring, 'oak', {'seconds': 2.5, 'fps': 8}); assert r4['count'] <= 12 and all(i['width'] == (480 if frame_clips.SCALER else 640) for i in im4)
    r1, _ = burst(ring, 'oak', {'seconds': 1, 'fps': 1}); assert r1['count'] == 2 and r1['span_s'] >= .7, r1   # never fewer than 2
    for name, cid in (('phone', 'phone_overview'), ('right_wrist', WRIST_CAMERA_IDS['right_wrist']), ('left_wrist', WRIST_CAMERA_IDS['left_wrist'])):
        r, ims = burst(ring, name, {'seconds': 2, 'fps': 4}); assert 4 <= r['count'] <= 8 and ims[0]['camera_id'] == cid and ims[0]['camera_name'] == name, (name, r['count'])
    assert burst(ring, 'phone', {})[0]['frames'][0]['captured_at'] is None and burst(ring, 'phone', {})[0]['frames'][0]['received_at']
    # Refusals: too few frames in the window; newest frame older than 2 s (publisher stopped); unknown camera; bad arguments.
    newest = ring.snapshot('oak')[-1].stamp
    try: ring.select('oak', .5, 8, now=newest + 1.9)
    except ClipUnavailable as e: assert 'only' in str(e) and 'needs 2' in str(e), e
    else: raise AssertionError('thin window accepted')
    try: ring.select('oak', 4, 4, now=newest + 2.5)
    except ClipUnavailable as e: assert 'not streaming' in str(e) and '2.5 s old' in str(e), e
    else: raise AssertionError('stale ring accepted')
    assert isinstance(ClipUnavailable('x'), RuntimeError)             # the API turns it into ok false (409), no images
    for bad in ({'camera': 'head'}, {'seconds': 5}, {'seconds': .1}, {'fps': 0}, {'max_width': 100}, {'max_width': 320.5}, {'max_width': True}):
        try: burst(ring, bad.pop('camera', 'oak'), bad)
        except ValueError: pass
        else: raise AssertionError(f'{bad} accepted')
    # A tool call never waits on the sampler: selecting while the publisher and sampler run stays within a few ms.
    t0 = time.perf_counter(); [ring.select('oak', 2, 4) for _ in range(50)]; assert (time.perf_counter() - t0) / 50 < .005
    # Broken manifests: the thread survives, logs once, and resumes when the publisher recovers.
    pub.enabled['oak'] = False; time.sleep(.1); (pub.oak / 'oak.json').write_text('{not json'); time.sleep(.6)
    assert ring.running() and ring.status('oak')['error']['count'] >= 3 and 'JSONDecodeError' in ring.status('oak')['error']['error'], ring.status('oak')
    assert logged == [], logged                                         # oak logged less than a minute ago: throttled
    ring._last_log['oak'] = 0; time.sleep(.3)                           # a minute later: one line, then quiet again
    assert len(logged) == 1 and 'oak' in logged[0] and 'JSONDecodeError' in logged[0], logged
    (pub.oak / 'oak.json').write_text(json.dumps({'camera_id': 'oak-test', 'seq': 10**6, 'captured_at': time.time(), 'image': 'missing.jpg', 'sha256': ''})); time.sleep(.3)
    assert ring.running() and 'FileNotFoundError' in ring.status('oak')['error']['error'] and len(logged) == 1
    (pub.oak / 'oak.json').write_text(json.dumps({'camera_id': 'oak-test', 'seq': 10**6, 'captured_at': time.time(), 'image': '../phone/latest.jpg', 'sha256': ''})); time.sleep(.3)
    assert 'outside camera directory' in ring.status('oak')['error']['error']
    before = ring.status('oak')['samples']; torn = make_jpeg(1)
    (pub.oak / 'torn.jpg').write_bytes(torn[: len(torn) // 2])          # publisher mid-write: hash mismatch, frame skipped
    (pub.oak / 'oak.json').write_text(json.dumps({'camera_id': 'oak-test', 'seq': 10**6 + 1, 'captured_at': time.time(), 'image': 'torn.jpg', 'sha256': hashlib.sha256(torn).hexdigest()})); time.sleep(.3)
    assert ring.status('oak')['samples'] == before and ring.running()
    pub.enabled['oak'] = True; time.sleep(.5)
    assert ring.status('oak')['error'] is None and ring.status('oak')['samples'] > before and ring.running()
    # A wrist whose manifest names another camera is refused (select_wrist_manifest identity check), the others continue.
    pub.enabled['left_wrist'] = False; time.sleep(.1); before = ring.status('left_wrist')['samples']
    m = json.loads((pub.wrist / 'left_wrist.json').read_text()); m.update(camera_id=WRIST_CAMERA_IDS['right_wrist'], seq=m['seq'] + 1, captured_at=time.time())
    (pub.wrist / 'left_wrist.json').write_text(json.dumps(m)); time.sleep(.4)
    assert ring.status('left_wrist')['samples'] == before and 'not the expected left_wrist' in ring.status('left_wrist')['error']['error']
    assert ring.status('right_wrist')['error'] is None and ring.running()
  finally:
    pub.stop.set(); pub.thread.join(1); ring.stop()
  assert not ring.running()

# Bounded memory: count, age and byte caps, each keeping the newest frame; oversize frames are refused.
class Fake:
    def __init__(self): self.meta, self.path = None, None
    def read(self): return self.meta, self.path
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp); fake = Fake(); clock = [1000.0]
    r = FrameRing({'oak': Source(fake.read)}, max_frames=5, keep_s=100, budget_bytes=10**6, clock=lambda: clock[0], log=logged.append)
    def push(i, size=1000, at=None):
        data = bytes([i % 256]) * size; p = tmp / f'{i}.jpg'; p.write_bytes(data)
        fake.meta, fake.path = {'seq': i, 'captured_at': clock[0] if at is None else at, 'sha256': hashlib.sha256(data).hexdigest()}, p
        clock[0] += .125; return r.sample('oak')
    assert all(push(i) for i in range(1, 9)) and not r.sample('oak')     # unchanged manifest: not stored twice
    assert [f.seq for f in r.snapshot('oak')] == [4, 5, 6, 7, 8] and r.status('oak')['bytes'] == 5000
    r = FrameRing({'oak': Source(fake.read)}, max_frames=32, keep_s=100, budget_bytes=3500, clock=lambda: clock[0], log=logged.append)
    for i in range(1, 9): push(i)
    assert [f.seq for f in r.snapshot('oak')] == [6, 7, 8] and r.status('oak')['bytes'] == 3000 <= 3500
    try: push(9, size=4000)
    except ValueError as e: assert 'exceeds' in str(e)
    else: raise AssertionError('oversize frame stored')
    r = FrameRing({'oak': Source(fake.read)}, max_frames=32, keep_s=4, budget_bytes=10**6, clock=lambda: clock[0], log=logged.append)
    for i in range(1, 41): push(i)                                       # 40 frames at 8 Hz = 5 s: only the last 4 s stay
    kept = r.snapshot('oak'); assert 32 <= len(kept) <= 33 and kept[-1].stamp - kept[0].stamp <= 4 and kept[-1].seq == 40, len(kept)
    assert frame_clips.MAX_FRAMES * 50 * 1024 == frame_clips.BUDGET_BYTES and frame_clips.KEEP_S == 4 and frame_clips.SAMPLE_HZ == 8
    # Even spacing picks the first and last frames and the nearest to each target in between.
    class F:
        def __init__(self, t): self.stamp = t
    fr = [F(t / 10) for t in range(0, 41)]                               # 0..4 s at 10 Hz
    assert [f.stamp for f in evenly_spaced(fr, 5)] == [0, 1, 2, 3, 4] and evenly_spaced(fr[:3], 5) == fr[:3]
    assert [f.stamp for f in evenly_spaced([F(0), F(.1), F(3.9), F(4)], 3)] == [0, 3.9, 4]   # gaps: nearest, no duplicates

# Downscale: pass-through when already narrow or without a scaler; header parser.
full = make_jpeg(3); assert jpeg_size(full) == (640, 480) and jpeg_size(b'junk') is None
assert downscale(full, 640) == (full, 640, 480, False) and downscale(full, 320, scaler=None) == (full, 640, 480, False)
if frame_clips.SCALER:
    small, w, h, scaled = downscale(full, 160); assert scaled and (w, h) == (160, 120) and jpeg_size(small) == (160, 120)
    assert downscale(small, 320) == (small, 160, 120, False)

# API registration: the tool is in TOOLS (not retired or hidden), its schema matches, validate_arguments bounds it,
# dispatch routes it to frame_clips.burst, main starts the ring, and the redeploy script installs and tests the module.
src = (HERE / 'gemma_robot_tools.py').read_text(); tree = ast.parse(src)
space = {'frame_clips': frame_clips}
exec(compile(ast.Module([n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('tool', 'validate_arguments')], []), 'gemma_robot_tools.py', 'exec'), space)
calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'tool' and n.args and getattr(n.args[0], 'value', '') == 'robot_get_clip']
assert len(calls) == 1
entry = eval(compile(ast.Expression(calls[0]), 'gemma_robot_tools.py', 'eval'), space)
props = entry['function']['parameters']['properties']
assert entry['function']['parameters']['required'] == ['camera'] and props['camera']['enum'] == ['oak', 'phone', 'left_wrist', 'right_wrist']
assert (props['seconds']['minimum'], props['seconds']['maximum'], props['fps']['minimum'], props['fps']['maximum'], props['max_width']['type'], props['max_width']['minimum'], props['max_width']['maximum']) == (0.5, 4, 1, 8, 'integer', 160, 640)
assert entry['function']['description'].startswith('Several recent frames of one camera as a short burst') and 'Not for distances' in entry['function']['description']
sets = {n.targets[0].id: eval(compile(ast.Expression(n.value), 'x', 'eval')) for n in tree.body if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') in ('RETIRED', 'PILOT_HIDDEN')}
assert 'robot_get_clip' not in sets['RETIRED'] | sets['PILOT_HIDDEN']
space['SCHEMAS'] = {'robot_get_clip': entry['function']['parameters']}; space['validate_poses'] = None
validate = space['validate_arguments']
validate('robot_get_clip', {'camera': 'oak'}); validate('robot_get_clip', {'camera': 'right_wrist', 'seconds': 0.5, 'fps': 8, 'max_width': 640})
for bad in ({}, {'camera': 'head'}, {'camera': 'oak', 'seconds': 4.5}, {'camera': 'oak', 'seconds': 0.2}, {'camera': 'oak', 'fps': 9}, {'camera': 'oak', 'max_width': 100}, {'camera': 'oak', 'max_width': 320.0}, {'camera': 'oak', 'extra': 1}):
    try: validate('robot_get_clip', bad)
    except ValueError: pass
    else: raise AssertionError(f'{bad} passed validation')
assert re.search(r"if name == 'robot_get_clip':\n\s+return frame_clips\.burst\(CLIPS, args\['camera'\], args\)", src) and 'CLIPS.start()' in src
deploy = (HERE / 'redeploy_robot_server.py').read_text()
lists = {n.targets[0].id: eval(compile(ast.Expression(n.value), 'x', 'eval')) for n in ast.parse(deploy).body if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') in ('API_ONLY', 'INSTALL', 'TESTS')}
assert 'frame_clips.py' in lists['API_ONLY'] and 'frame_clips.py' in lists['INSTALL'] and 'test_frame_clips.py' in lists['TESTS']
print(f"Frame clips: 8 Hz sampling of 4 fake publishers, even spacing, downscale ({frame_clips.SCALER or 'no scaler: full frames'}), refusals, bounded memory, "
      f"sampler survives broken manifests, tool registered and validated; no camera access")
