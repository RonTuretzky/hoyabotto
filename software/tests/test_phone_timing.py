"""Offline phone timing checks: fake requests/clocks only; no sockets or hardware.

The bridge is AST-isolated because importing its module reads deployment secrets
and initializes hardware clients. Only its actual camera read/metadata functions
are executed, against temporary manifests and synthetic JPEG bytes.
"""
import ast
import asyncio
import base64
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from aiohttp import web
from PIL import Image

SOFTWARE = Path(__file__).resolve().parents[1]
PHONE = SOFTWARE / 'docs/session-archive-2026-10-05/phone_camera'
BRIDGE = SOFTWARE / 'docs/commissioning/2026-10-07-paddle-success/qwen-bridge/gemma_robot_tools.py'


class Clock:
    wall = 1000.0
    mono = 100.0

    def time(self):
        return self.wall

    def monotonic(self):
        return self.mono

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds

    def sleep(self, seconds):
        self.advance(seconds)


class Request:
    secure = True
    content_type = 'image/jpeg'

    def __init__(self, data=b'', **headers):
        self.headers = {'Authorization': 'Bearer offline-test', 'X-Camera-Client': 'test-client',
                        'X-Camera-Stream': 'test-stream', **headers}
        self.data = data

    async def read(self):
        return self.data


def call(function, *args):
    return asyncio.run(function(*args))


def json_body(response):
    return json.loads(response.body)


@pytest.fixture
def server(tmp_path):
    spec = importlib.util.spec_from_file_location('offline_phone_server', PHONE / 'server.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # CONFIG must not read deployment files here.
    assert module.CONFIG == {}
    module.CONFIG = {'publisher_token': 'offline-test', 'viewer_token': 'offline-viewer'}
    module.BASE = tmp_path
    module.LATEST = tmp_path / 'latest.jpg'
    module.META = tmp_path / 'latest.json'
    module.time = Clock()
    return module


@pytest.fixture
def jpeg():
    out = io.BytesIO()
    Image.new('RGB', (80, 64), 'white').save(out, format='JPEG')
    return out.getvalue()


def timing(server, count=1, media=1.0):
    challenge = json_body(call(server.challenge, Request()))
    return {'protocol': challenge['protocol'], 'challenge_id': challenge['challenge_id'],
            'browser_stream_id': 'test-stream', 'presented_frames': count, 'media_time_s': media,
            'challenge_received_ms': 5000.0, 'presentation_time_ms': 5010.0, 'callback_now_ms': 5020.0}


def upload(server, jpeg, value):
    headers = {} if value is None else {'X-Camera-Timing': json.dumps(value)}
    return call(server.frame, Request(jpeg, **headers))


def accepted(server, jpeg):
    value = timing(server)
    server.time.advance(.1)
    return json_body(upload(server, jpeg, value))


@pytest.fixture
def bridge(tmp_path, server):
    names = {'phone_frame_metadata', 'camera_status', 'cameras_strict'}
    tree = ast.parse(BRIDGE.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    env = {'time': server.time, 'math': math, 'json': json, 'hashlib': hashlib, 'base64': base64,
           'ROOT': tmp_path, 'OAK_RECTIFIED_DIR': tmp_path / 'missing_rectified',
           'OAK_RAW_DIR': tmp_path / 'missing_raw', 'WRIST_DIRS': {},
           'wrist_status': lambda _: {}, 'setup_report': lambda _: {}}
    exec(compile(ast.Module(functions, []), str(BRIDGE), 'exec', optimize=2), env)
    return SimpleNamespace(**env)


def test_challenge_bound_keeps_sensor_exposure_unknown(server, jpeg, bridge):
    info = accepted(server, jpeg)
    assert info['captured_at'] is None
    assert info['frame_timing']['sensor_exposure_at'] is None
    assert info['frame_timing']['sensor_exposure_age_s'] is None
    assert info['browser_frame_age_upper_bound_s'] == pytest.approx(.1)
    assert info['sha256'] == hashlib.sha256(jpeg).hexdigest()
    assert info['stream_id'].startswith(server.SERVER_INSTANCE + ':')
    server.time.advance(.55)
    # Ignore persisted age/fresh flags; age the actual challenge at read time.
    info.update(browser_frame_age_upper_bound_s=0, age_s=0, live=True)
    measured = bridge.phone_frame_metadata(info)
    assert measured['browser_frame_age_upper_bound_s'] == pytest.approx(.65)
    assert measured['browser_frame_fresh']
    assert measured['captured_at'] is None
    server.time.advance(.4)
    assert not bridge.phone_frame_metadata(info)['fresh']
    assert not server.metadata()['browser_frame_fresh']


def test_legacy_upload_and_bridge_remain_explicitly_receipt_only(server, jpeg, bridge):
    info = json_body(upload(server, jpeg, None))
    info['captured_at'] = None
    for meta in (info, {'received_at': 1000.0, 'seq': 1}):
        result = bridge.phone_frame_metadata(meta)
        assert result['fresh']  # Existing receipt gate is preserved, not a capture proof.
        assert result['freshness_basis'] == 'server_receipt_only'
        assert result['frame_timing'] is None
        assert result['browser_frame_age_upper_bound_s'] is None
        assert not result['browser_frame_fresh']
        assert result['captured_at'] is None
    server.time.advance(2)
    assert not server.metadata()['live']
    assert not bridge.phone_frame_metadata(info)['fresh']


def test_replay_consumed_challenge_cannot_replace_latest_frame(server, jpeg):
    value = timing(server)
    server.time.advance(.1)
    upload(server, jpeg, value)
    old = server.META.read_bytes()
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, value)
    assert server.STATE['seq'] == 1
    assert server.META.read_bytes() == old


@pytest.mark.parametrize('count,media', [(1, 2), (2, 1), (0, 2), (2, .5)])
def test_new_challenge_does_not_permit_repeated_or_regressed_frame_identity(server, jpeg, count, media):
    accepted(server, jpeg)
    value = timing(server, count, media)
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, value)
    assert server.STATE['seq'] == 1


@pytest.mark.parametrize('key,value', [
    ('protocol', 'unknown'), ('challenge_id', 'replayed'), ('browser_stream_id', 'other-stream'),
    ('presented_frames', True), ('presented_frames', 1.5), ('presented_frames', 2**53),
    ('media_time_s', float('nan')), ('media_time_s', float('inf')), ('media_time_s', -1),
    ('media_time_s', 10**1000), ('media_time_s', None), ('media_time_s', '1'),
    ('callback_now_ms', False), ('callback_now_ms', 5005), ('callback_now_ms', 8000),
    ('presentation_time_ms', 4999), ('challenge_received_ms', 5011),
])
def test_malformed_frame_timing_fails_without_publishing(server, jpeg, key, value):
    candidate = timing(server)
    candidate[key] = value
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, candidate)
    assert server.STATE['seq'] == 0
    assert not server.META.exists()


@pytest.mark.parametrize('elapsed', [1.001, -0.1])
def test_stale_or_future_challenge_is_rejected(server, jpeg, elapsed):
    value = timing(server)
    server.time.advance(elapsed)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, value)


def test_challenge_supersession_and_client_binding(server, jpeg):
    first = timing(server)
    second = timing(server)
    assert first['challenge_id'] != second['challenge_id']
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, first)
    candidate = timing(server)
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        call(server.frame, Request(jpeg, **{'X-Camera-Client': 'different', 'X-Camera-Timing': json.dumps(candidate)}))


def test_bad_jpeg_consumes_challenge_and_cannot_replace_frame(server, jpeg):
    accepted(server, jpeg)
    value = timing(server, 2, 2)
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, b'not a JPEG', value)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, value)
    assert server.STATE['seq'] == 1


@pytest.mark.parametrize('raw', ['{', 'null', '[]', 'true', 'x' * 2049])
def test_invalid_timing_header_is_never_legacy_fallback(server, jpeg, raw):
    timing(server)
    server.time.advance(.1)
    with pytest.raises(web.HTTPBadRequest):
        call(server.frame, Request(jpeg, **{'X-Camera-Timing': raw}))
    assert server.STATE['seq'] == 0


def test_timed_stream_cannot_silently_downgrade(server, jpeg):
    accepted(server, jpeg)
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, None)


@pytest.mark.parametrize('action', ['challenge', 'frame'])
def test_authentication_and_https_still_required(server, jpeg, action):
    request = Request(jpeg, Authorization='invalid')
    with pytest.raises(web.HTTPUnauthorized):
        call(getattr(server, action), request)
    request = Request(jpeg)
    request.secure = False
    with pytest.raises(web.HTTPForbidden):
        call(getattr(server, action), request)


def test_active_publisher_ownership_is_preserved(server, jpeg):
    accepted(server, jpeg)
    for function in (server.challenge, server.frame):
        with pytest.raises(web.HTTPConflict):
            call(function, Request(jpeg, **{'X-Camera-Client': 'another-phone'}))


@pytest.mark.parametrize('delta', [-5, 5])
def test_server_clock_jump_during_upload_fails_closed(server, jpeg, delta):
    value = timing(server)
    server.time.advance(.1)
    server.time.wall += delta
    with pytest.raises(web.HTTPBadRequest):
        upload(server, jpeg, value)


@pytest.mark.parametrize('delta', [-5, 5])
def test_clock_jump_after_upload_invalidates_bound(server, jpeg, bridge, delta):
    info = accepted(server, jpeg)
    server.time.wall += delta
    assert not server.metadata()['live']
    assert server.metadata()['browser_frame_age_upper_bound_s'] is None
    with pytest.raises(ValueError):
        bridge.phone_frame_metadata(info)


@pytest.mark.parametrize('path,value', [
    ('received_at', float('nan')), ('received_at', float('inf')), ('received_at', True),
    ('received_at', 1001.0), ('captured_at', 1000.0), ('seq', True), ('sha256', 'bad'),
    ('stream_id', 'wrong-server:wrong-stream'), ('frame_timing', {}), ('frame_timing', []),
    ('frame_timing', None), ('frame_timing.clock_domain', 'another-host'),
    ('frame_timing.protocol', 'unknown'), ('frame_timing.sensor_exposure_at', 1000),
    ('frame_timing.browser_frame_age_upper_bound_s_at_receipt', 0),
    ('frame_timing.received_monotonic_s', 200), ('frame_timing.challenge_issued_at', 1001),
    ('frame_timing.presentation_time_ms', 4999), ('frame_timing.media_time_s', float('nan')),
])
def test_bridge_rejects_malformed_timestamps_and_contract(server, jpeg, bridge, path, value):
    info = accepted(server, jpeg)
    target = info
    keys = path.split('.')
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = value
    with pytest.raises(ValueError):
        bridge.phone_frame_metadata(info)


def publish_bridge_fixture(bridge, info, jpeg):
    folder = bridge.ROOT / 'work/phone_camera'
    folder.mkdir(parents=True)
    (folder / 'latest.json').write_text(json.dumps(info))
    (folder / 'latest.jpg').write_bytes(jpeg)
    return folder


def test_real_bridge_camera_path_carries_bound_and_rejects_stale_frame(server, jpeg, bridge):
    info = accepted(server, jpeg)
    publish_bridge_fixture(bridge, info, jpeg)
    server.time.advance(.4)
    meta, images = bridge.cameras_strict(['phone'], False)
    assert images[0]['frame_timing'] == info['frame_timing']
    assert images[0]['browser_frame_age_upper_bound_s'] == pytest.approx(.5)
    assert images[0]['sha256'] == info['sha256']
    assert images[0]['captured_at'] is None
    assert bridge.camera_status()['phone']['browser_frame_fresh']
    server.time.advance(.6)  # Receipt alone remains <=1 s, but challenge age >1 s.
    with pytest.raises(RuntimeError, match='stale'):
        bridge.cameras_strict(['phone'], False)
    assert not bridge.camera_status()['phone']['fresh']


def test_real_bridge_rejects_image_manifest_replace_window(server, jpeg, bridge):
    info = accepted(server, jpeg)
    publish_bridge_fixture(bridge, info, jpeg + b'different bytes')
    with pytest.raises(RuntimeError, match='consistent snapshot unavailable'):
        bridge.cameras_strict(['phone'], False)


def test_progress_advances_only_after_valid_upload(server, jpeg):
    accepted(server, jpeg)
    value = timing(server, 2, 2)
    server.time.advance(.1)
    result = json_body(upload(server, jpeg, value))
    assert result['seq'] == 2
    assert result['frame_timing']['presented_frames'] == 2
    assert result['frame_timing']['media_time_s'] == 2


def test_same_pixels_can_be_a_new_frame_but_identity_must_advance(server, jpeg):
    first = accepted(server, jpeg)
    value = timing(server, 2, 2)
    server.time.advance(.1)
    second = json_body(upload(server, jpeg, value))
    assert first['sha256'] == second['sha256']
    assert first['seq'] < second['seq']
    # A motionless scene need not change bytes; pixels are not a physical liveness proof.


@pytest.mark.parametrize('invalid', [[], None, 'not a manifest', {'seq': 1, 'received_at': 10**1000}])
def test_camera_status_rejects_malformed_manifest_without_crashing(server, jpeg, bridge, invalid):
    publish_bridge_fixture(bridge, invalid, jpeg)
    status = bridge.camera_status()['phone']
    assert not status['available'] and not status['fresh']
