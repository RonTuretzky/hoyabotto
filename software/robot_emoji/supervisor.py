"""Restart-safe show supervisor. Reconnects the fixed cloud API without editing the slides."""
from pathlib import Path as _P
RELAY_AGENT = _P.home() / 'Library/LaunchAgents/com.hoyabotto.robot-relay.plist'
import argparse
import json
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

STOP = threading.Event()


def listening(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1):
            return True
    except OSError:
        return False


def register(api_url, origin, token):
    request = urllib.request.Request(api_url + '/api/bridge/register',
        json.dumps({'origin': origin}).encode(), {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + token,
                                                'User-Agent': 'HoyaBotto-Bridge/1.0'})
    with urllib.request.urlopen(request, timeout=10) as response:
        if not json.load(response).get('ok'):
            raise RuntimeError('Cloud registration was rejected')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    root = args.config.parent
    logs = root / 'logs'
    logs.mkdir(exist_ok=True)
    token = Path(config['token_file']).read_text().strip()
    robot_config = json.loads(Path(config['robot_config']).read_text())
    processes = {}
    handles = {}
    origin = None
    registered = None
    last_register = 0
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: STOP.set())

    def start(key, argv, truncate=False):
        if key in handles:
            handles[key].close()
        path = logs / (key + '.log')
        handles[key] = path.open('wb' if truncate else 'ab', buffering=0)
        path.chmod(0o600)
        processes[key] = subprocess.Popen(argv, cwd=root, stdout=handles[key], stderr=handles[key], stdin=subprocess.DEVNULL)
        print(f'{key} started (pid {processes[key].pid})', flush=True)

    def down(key):
        return key not in processes or processes[key].poll() is not None

    try:
        while not STOP.is_set():
            # The robot's existing pinned-certificate internet relay works across separate networks.
            # Do not start a second listener when another robot app already owns its local relay port.
            # robot_emoji.relay's launch agent, when installed, is the one owner of that forwarder.
            if not RELAY_AGENT.exists() and not listening(1242) and down('robot-relay'):
                start('robot-relay', [config['cloudflared'], 'access', 'tcp', '--hostname', robot_config['relay_hostname'], '--url', '127.0.0.1:1242'])
            if down('service'):
                start('service', [sys.executable, '-m', 'robot_emoji', '--port', '8790', '--public-port', '8791',
                    '--robot-config', config['robot_config'], '--proxy-token-file', config['token_file'],
                    '--state-file', str(root / 'queue.json')])
            if down('visitor-tunnel'):
                origin = registered = None
                start('visitor-tunnel', [config['cloudflared'], 'tunnel', '--no-autoupdate', '--url', 'http://127.0.0.1:8791'], truncate=True)
            if not origin:
                match = re.search(r'https://[a-z0-9-]+\.trycloudflare\.com', (logs / 'visitor-tunnel.log').read_text(errors='replace'))
                if match:
                    origin = match.group()
            if origin and listening(8791) and (registered != origin or time.monotonic() - last_register > 60):
                try:
                    register(config['api_url'], origin, token)
                    registered = origin
                    last_register = time.monotonic()
                    (root / 'connection.json').write_text(json.dumps({'api_url': config['api_url'], 'upstream_origin': origin}) + '\n')
                    print('Fixed cloud API connected to the visitor service', flush=True)
                except Exception as exc:
                    # No robot command is retried here; this is only a network registration.
                    print(f'Cloud registration waiting ({type(exc).__name__})', flush=True)
            STOP.wait(2)
    finally:
        for process in processes.values():
            if process.poll() is None:
                process.terminate()
        for process in processes.values():
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
        for handle in handles.values():
            handle.close()


if __name__ == '__main__':
    main()
