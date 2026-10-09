"""Emoji show web service: kiosk (pick emojis, type a name), screen (whose turn it is) and operator page.

  python -m robot_emoji                      real robot, through the chat Mac's paired API certificate
  python -m robot_emoji --fake               no robot: simulated moves at the owner's pace
  python -m robot_emoji --host 0.0.0.0       let phones on the LAN reach the kiosk

Pages: /  (kiosk)   /screen  (big display)   /operator  (arm/pause, STOP, log)
The robot only moves while the operator page has the show armed. Operator actions are accepted from this
Mac (loopback) or with the token printed at startup (/operator?token=...). Visitors cannot arm the show.
"""
import argparse
import collections
import hmac
import json
import secrets
import threading
import time
import unicodedata
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from . import gestures as G
from .performer import Busy, PerformError, Performer
from .robot import FakeRobot, RobotClient, RobotError

STATIC = Path(__file__).with_name('static')
PAGES = {'/': 'kiosk.html', '/screen': 'screen.html', '/operator': 'operator.html'}
NAME_MAX = 24
MAX_GESTURES = 3
QUEUE_MAX = 30
BUSY_RETRY_S = 3
THANKS_S = 4
SITE_ORIGINS = ('https://hoyabotto.com', 'https://www.hoyabotto.com', 'https://ronturetzky.github.io')


def clean_name(raw):
    """Printable, single-spaced, at most NAME_MAX characters; pages render it as text, never as HTML."""
    if not isinstance(raw, str):
        raise ValueError('Name required')
    text = ''.join(ch for ch in unicodedata.normalize('NFC', raw) if not unicodedata.category(ch).startswith('C'))
    text = ' '.join(text.split())
    if not text:
        raise ValueError('Type your name')
    if len(text) > NAME_MAX:
        raise ValueError(f'Name is too long (at most {NAME_MAX} characters)')
    return text


class Show:
    """Queue, current performance and the worker that runs them one at a time."""

    def __init__(self, robot, catalog, armed=False, clock=time.time):
        self.robot = robot
        self.catalog = catalog
        self.clock = clock
        self.lock = threading.Condition()
        self.queue = collections.deque()
        self.requests = {}                       # id -> request (bounded)
        self.current = None
        self.recent = collections.deque(maxlen=8)
        self.armed = armed
        self.robot_note = 'idle'
        self.log_lines = collections.deque(maxlen=200)
        self.performer = Performer(robot, catalog, log=self.log)
        self.closed = False

    def log(self, message):
        line = time.strftime('%H:%M:%S') + ' ' + message
        print(line, flush=True)
        with self.lock:
            self.log_lines.append(line)

    # visitors
    def submit(self, name, keys):
        name = clean_name(name)
        if not isinstance(keys, list) or not keys or len(keys) > MAX_GESTURES or any(k not in self.catalog for k in keys):
            raise ValueError(f'Pick 1 to {MAX_GESTURES} emojis')
        if len({self.catalog[k].arm for k in keys}) != 1:
            raise ValueError('Those emojis use different arms; pick them separately')
        with self.lock:
            if len(self.queue) >= QUEUE_MAX:
                raise ValueError('The line is full, try again in a few minutes')
            req = {'id': uuid.uuid4().hex[:12], 'name': name, 'gestures': list(keys), 'state': 'queued',
                   'created': self.clock(), 'phase': None, 'error': None}
            self.queue.append(req)
            self.requests[req['id']] = req
            while len(self.requests) > 500:
                self.requests.pop(next(iter(self.requests)))
            self.lock.notify_all()
        self.log(f"queued {req['id']} {name!r} {''.join(self.catalog[k].emoji for k in keys)}")
        return self.public_request(req)

    def public_request(self, req):
        with self.lock:
            ahead = next((i for i, r in enumerate(self.queue) if r is req), None)
            return {'id': req['id'], 'name': req['name'], 'emojis': [self.catalog[k].emoji for k in req['gestures']],
                    'state': req['state'], 'phase': req['phase'], 'error': req['error'],
                    'position': None if ahead is None else ahead + 1 + (self.current is not None)}

    def request_status(self, rid):
        req = self.requests.get(rid)
        return None if req is None else self.public_request(req)

    def snapshot(self):
        with self.lock:
            view = lambda r: {'id': r['id'], 'name': r['name'], 'emojis': [self.catalog[k].emoji for k in r['gestures']],
                              'state': r['state'], 'phase': r['phase'], 'error': r['error']}
            return {'armed': self.armed, 'robot': self.robot_note, 'current': self.current and view(self.current),
                    'queue': [view(r) for r in self.queue], 'recent': [view(r) for r in self.recent],
                    'catalog': [g.public() for g in self.catalog.values()],
                    'unverified': [g.key for g in self.catalog.values() if not g.verified_on_hardware]}

    # operator
    def set_armed(self, armed):
        with self.lock:
            self.armed = bool(armed)
            self.lock.notify_all()
        self.log('show ARMED: the robot will perform queued requests' if armed else 'show paused: the current performance finishes, nothing new starts')

    def remove(self, rid):
        with self.lock:
            for r in list(self.queue):
                if r['id'] == rid:
                    self.queue.remove(r)
                    r['state'] = 'removed'
                    return True
        return False

    def stop(self):
        """Operator STOP: robot_stop now (releases every motor), pause the show."""
        with self.lock:
            self.armed = False
        try:
            result = self.performer.stop()
            self.log('STOP sent: every motor released; show paused')
            return {'ok': True, 'result': result}
        except RobotError as e:
            self.log(f'STOP failed: {e}. Use the 12 V switch.')
            return {'ok': False, 'error': str(e)}

    # worker
    def _phase(self, req, phase, gesture=None):
        with self.lock:
            req['phase'] = phase if gesture is None else f'{phase}:{gesture}'
        self.log(f"{req['id']} {req['phase']}")

    def run(self):
        while True:
            with self.lock:
                while not self.closed and not (self.armed and self.queue):
                    self.robot_note = 'paused' if not self.armed else 'idle'
                    self.lock.wait(1)
                if self.closed:
                    return
                req = self.queue[0]
            try:
                self.performer.preflight(req['gestures'])
            except Busy as e:
                with self.lock:
                    self.robot_note = f'waiting: {e}'
                    self.lock.wait(BUSY_RETRY_S)
                continue
            except (PerformError, RobotError) as e:
                self._finish(req, 'failed', str(e), dequeue=True)
                self._cooldown()
                continue
            with self.lock:
                if not self.armed or not self.queue or self.queue[0] is not req:
                    continue
                self.queue.popleft()
                req['state'], self.current, self.robot_note = 'performing', req, 'performing'
            self.log(f"performing {req['id']} for {req['name']!r}")
            try:
                summary = self.performer.perform(req['gestures'], lambda p, g=None: self._phase(req, p, g))
                self.log(f"done {req['id']}: {json.dumps(summary['paths'])}")
                self._finish(req, 'done', None)
            except Busy as e:  # someone took the robot between the check and the start; back to the front
                with self.lock:
                    req['state'], self.current = 'queued', None
                    self.queue.appendleft(req)
                self.log(f'robot taken before start: {e}')
            except (PerformError, RobotError) as e:
                self._finish(req, 'failed', str(e))
                self._cooldown()

    def _finish(self, req, state, error, dequeue=False):
        with self.lock:
            if dequeue and req in self.queue:
                self.queue.remove(req)
            req['state'], req['error'], req['phase'] = state, error, None
            self.current = req if state == 'done' else None
            self.recent.appendleft(req)
        if error:
            self.log(f"{req['id']} failed: {error}")
        if state == 'done':
            time.sleep(THANKS_S)         # keep the name up while the screen says thanks
            with self.lock:
                if self.current is req:
                    self.current = None

    def _cooldown(self):
        """After a failure the show pauses: a person looks at the robot before re-arming."""
        with self.lock:
            self.armed = False
            self.robot_note = 'paused after a failure; check the robot, then re-arm'
        self.log('show paused after a failure')

    def start(self):
        threading.Thread(target=self.run, name='emoji-show-worker', daemon=True).start()

    def close(self):
        with self.lock:
            self.closed = True
            self.lock.notify_all()


def make_handler(show, token, public_only=False, allowed_origins=SITE_ORIGINS):
    submissions = {}
    submissions_lock = threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, kind='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            origin = self.headers.get('Origin')
            if public_only and origin in allowed_origins:
                self.send_header('Access-Control-Allow-Origin', origin)
                self.send_header('Vary', 'Origin')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()
            self.wfile.write(data)

        def operator_ok(self, query):
            # A tunnel connects over loopback too. Its dedicated listener must NEVER
            # authorize operator actions, even with a correct operator token.
            if public_only:
                return False
            forwarded = any(self.headers.get(h) for h in ('CF-Connecting-IP', 'X-Forwarded-For', 'Forwarded'))
            origin = self.headers.get('Origin')
            if origin:
                local_origins = (f'http://localhost:{self.server.server_port}', f'http://127.0.0.1:{self.server.server_port}')
                forwarded = forwarded or origin not in local_origins
            if self.client_address[0] in ('127.0.0.1', '::1') and not forwarded:
                return True
            given = self.headers.get('X-Operator-Token') or query.get('token', '')
            return hmac.compare_digest(given.encode(), token.encode())

        def body(self):
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 4096:
                raise ValueError('Invalid request size')
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict):
                raise ValueError('JSON object required')
            return data

        def do_OPTIONS(self):
            if public_only and self.headers.get('Origin') in allowed_origins:
                return self.send(200, {'ok': True})
            return self.send(403, {'error': 'Origin not allowed'})

        def do_GET(self):
            url = urlsplit(self.path)
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            if url.path == '/emoji-config.js':
                return self.send(200, b'window.ROBOT_EMOJI_API = "";\n', 'application/javascript')
            if url.path == '/emoji-api.js':
                return self.send(200, (STATIC / 'emoji-api.js').read_bytes(), 'application/javascript')
            if url.path in PAGES:
                if url.path == '/operator' and public_only:
                    return self.send(403, {'error': 'Operator controls are local only'})
                return self.send(200, (STATIC / PAGES[url.path]).read_bytes(), 'text/html; charset=utf-8')
            if url.path == '/api/state':
                state = show.snapshot()
                if self.operator_ok(query):
                    with show.lock:
                        state['log'] = list(show.log_lines)[-60:]
                    state['robot_api'] = show.robot.describe()
                return self.send(200, state)
            if url.path.startswith('/api/requests/'):
                found = show.request_status(url.path.rsplit('/', 1)[1])
                return self.send(200, found) if found else self.send(404, {'error': 'Unknown request'})
            return self.send(404, {'error': 'Not found'})

        def do_POST(self):
            url = urlsplit(self.path)
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            try:
                if public_only and self.headers.get('Origin') not in (None, *allowed_origins):
                    return self.send(403, {'error': 'Origin not allowed'})
                if url.path == '/api/requests':
                    data = self.body()
                    if public_only:
                        # Only trust Cloudflare's visitor header on the tunnel's loopback listener.
                        peer = self.headers.get('CF-Connecting-IP') or self.client_address[0]
                        with submissions_lock:
                            now = time.monotonic()
                            for key in list(submissions):
                                if now - submissions[key] >= 30:
                                    del submissions[key]
                            if peer in submissions:
                                return self.send(429, {'error': 'Please wait 30 seconds before sending another request'})
                            result = show.submit(data.get('name'), data.get('gestures'))
                            submissions[peer] = now
                        return self.send(201, result)
                    return self.send(201, show.submit(data.get('name'), data.get('gestures')))
                if url.path.startswith('/api/operator/'):
                    if not self.operator_ok(query):
                        return self.send(403, {'error': 'Operator token required'})
                    action = url.path.rsplit('/', 1)[1]
                    if action == 'stop':
                        return self.send(200, show.stop())
                    data = self.body()
                    if action == 'arm':
                        show.set_armed(data.get('armed') is True)
                        return self.send(200, {'armed': show.armed})
                    if action == 'remove':
                        return self.send(200, {'removed': show.remove(data.get('id'))})
                return self.send(404, {'error': 'Not found'})
            except ValueError as e:
                return self.send(400, {'error': str(e)})
    return Handler


def serve(show, host, port, token, public_only=False):
    server = ThreadingHTTPServer((host, port), make_handler(show, token, public_only=public_only))
    server.daemon_threads = True
    return server


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8790)
    ap.add_argument('--public-port', type=int, help='separate visitor-only loopback listener for an HTTPS tunnel')
    ap.add_argument('--fake', action='store_true', help='simulated robot, no API calls')
    ap.add_argument('--gestures', default=str(G.DEFAULT_PATH))
    ap.add_argument('--robot-config', help='robot.json with the paired client certificate (default: $XLEROBOT_ADMIN_CONFIG or the chat Mac path)')
    ap.add_argument('--token', help='operator token for other devices (default: random, printed)')
    a = ap.parse_args(argv)
    catalog = G.load(a.gestures)
    robot = FakeRobot() if a.fake else RobotClient(a.robot_config)
    if not a.fake:
        try:
            health = robot.health()
            print(f"robot API: ok={health.get('ok')} motion_ready={health.get('motion_ready')} owner_active={health.get('motor_owner_active')}")
        except RobotError as e:
            print(f'robot API not reachable yet ({e}); requests wait until it is')
    show = Show(robot, catalog)
    show.start()
    token = a.token or secrets.token_urlsafe(9)
    server = serve(show, a.host, a.port, token)
    public_server = None
    if a.public_port:
        public_server = serve(show, '127.0.0.1', a.public_port, token, public_only=True)
        threading.Thread(target=public_server.serve_forever, name='emoji-public-http', daemon=True).start()
        print(f'public visitor API http://127.0.0.1:{a.public_port}/ (no operator controls)')
    shown = 'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host
    print(f'kiosk     http://{shown}:{a.port}/\nscreen    http://{shown}:{a.port}/screen\n'
          f'operator  http://{shown}:{a.port}/operator?token={token}\n'
          f"gestures  {', '.join(g.emoji + ' ' + g.key + ('' if g.verified_on_hardware else ' (not yet verified on hardware)') for g in catalog.values())}\n"
          'The show starts PAUSED: arm it on the operator page once the arm has room to move.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        show.close()
        server.server_close()
        if public_server:
            public_server.shutdown()
            public_server.server_close()


if __name__ == '__main__':
    main()
