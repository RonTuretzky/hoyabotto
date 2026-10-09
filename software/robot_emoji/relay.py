"""Keep this Mac's link to the robot's Cloudflare relay alive.

The robot Mac publishes its API through a Cloudflare quick tunnel (a random *.trycloudflare.com name that
changes whenever its tunnel restarts). This Mac reaches it through `cloudflared access tcp`, listening on the
local port named in robot.json's "url" (127.0.0.1:1242). The robot client re-reads robot.json on every call,
prefers the direct LAN address when both Macs share a network, and falls back to this relay otherwise.

  watch                 run by the launch agent: keeps the forwarder running for robot.json's relay_hostname,
                        restarts it if it exits or the hostname changes, and logs when the name stops resolving.
  reconnect NAME        the robot Mac's tunnel restarted: store its new name (a hostname, URL or log line)
                        in robot.json, wait for the forwarder, then check the relay read-only.
  check                 read-only health over the relay and over the LAN.
  install               install and start the launch agent (replaces a hand-started forwarder).
"""
import argparse
import json
import os
import plistlib
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from urllib.parse import urlsplit

from .robot import DEFAULT_CONFIG

ROOT = Path.home() / 'Library/Application Support/HoyaBotto'
LABEL = 'com.hoyabotto.robot-relay'
STATUS = ROOT / 'relay-status.json'
NAME = re.compile(r'([a-z0-9-]+\.trycloudflare\.com)')


def log(msg):
    print(time.strftime('%Y-%m-%d %H:%M:%S'), msg, flush=True)


def read_config(path):
    c = json.loads(Path(path).read_text())
    u = urlsplit(c['url'])
    if u.hostname not in ('127.0.0.1', 'localhost'):
        raise SystemExit('robot.json url is not a local forwarder port; nothing to keep alive')
    return c.get('relay_hostname'), u.port


def resolves(name):
    try:
        socket.getaddrinfo(name, 443)
        return True
    except OSError:
        return False


def listener_pids(port):
    out = subprocess.run(['lsof', '-nP', '-t', f'-iTCP:{port}', '-sTCP:LISTEN'], capture_output=True, text=True).stdout
    return [int(p) for p in out.split()]


def write_status(**fields):
    STATUS.write_text(json.dumps(dict(fields, time=time.time()), indent=1) + '\n')


def watch(config):
    cloudflared = shutil.which('cloudflared') or '/opt/homebrew/bin/cloudflared'
    child, current, last_dns, next_dns = None, None, None, 0
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while not stop:
        try:
            name, port = read_config(config)
        except (OSError, ValueError, KeyError) as e:
            log(f'cannot read {config}: {e}'); time.sleep(5); continue
        if name != current or child is None or child.poll() is not None:
            if child is not None and child.poll() is None:
                child.terminate(); child.wait(5)
            for pid in listener_pids(port):          # a hand-started forwarder from before the agent
                if child is None or pid != child.pid:
                    log(f'stopping earlier forwarder pid {pid} on port {port}')
                    os.kill(pid, signal.SIGTERM)
            time.sleep(1)
            if not name or not NAME.fullmatch(name):
                log('robot.json has no trycloudflare relay_hostname; run: hoyabotto-reconnect <name>')
                child, current = None, name; time.sleep(10); continue
            child = subprocess.Popen([cloudflared, 'access', 'tcp', '--hostname', name, '--url', f'127.0.0.1:{port}'],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            current = name
            log(f'forwarder pid {child.pid}: 127.0.0.1:{port} -> {name}')
        if time.time() >= next_dns:
            ok = resolves(current)
            if ok != last_dns:
                log(f'relay name {current} ' + ('resolves' if ok else 'NO LONGER RESOLVES: the robot Mac tunnel restarted; '
                    'get its new *.trycloudflare.com name and run: hoyabotto-reconnect <name>'))
                last_dns = ok
            write_status(relay_hostname=current, port=port, forwarder_pid=child.pid, name_resolves=ok)
            next_dns = time.time() + 30
        time.sleep(2)
    if child is not None and child.poll() is None:
        child.terminate()


def health(config, lan):
    from .robot import RobotClient
    try:
        client = RobotClient(config, prefer_lan=lan)
        ok = bool(client.health())
        return ok, client.describe().get('link')
    except Exception as e:  # report, never raise: this is a read-only check
        return False, f'{type(e).__name__}: {str(e)[:100]}'


def check(config):
    for lan in (False, True):
        ok, link = health(config, lan)
        print(f"{'relay' if not lan else 'LAN-first'}: {'OK' if ok else 'FAIL'} ({link})")


def reconnect(config, text):
    m = NAME.search(text.lower())
    if not m:
        raise SystemExit('Give the new *.trycloudflare.com name (or the URL / log line containing it)')
    name = m.group(1)
    path = Path(config)
    data = json.loads(path.read_text())
    if data.get('relay_hostname') != name:
        backup = path.with_name(path.name + f'.before-{time.strftime("%Y%m%d-%H%M%S")}')
        backup.write_bytes(path.read_bytes()); backup.chmod(0o600)
        data['relay_hostname'] = name
        tmp = path.with_name(path.name + '.tmp')
        tmp.write_text(json.dumps(data, indent=2) + '\n'); tmp.chmod(0o600); tmp.replace(path)
        print(f'robot.json relay_hostname -> {name} (backup {backup.name})')
    if not resolves(name):
        print(f'warning: {name} does not resolve yet; check it is the robot Mac\'s current tunnel')
    for _ in range(20):          # the agent notices within a few seconds
        time.sleep(1.5)
        try:
            if json.loads(STATUS.read_text()).get('relay_hostname') == name:
                break
        except (OSError, ValueError):
            pass
    time.sleep(3)
    ok, link = health(config, False)
    print(f"relay check: {'OK' if ok else 'FAIL'} ({link})")
    raise SystemExit(0 if ok else 1)


def install(config):
    python = sys.executable
    agent = Path.home() / 'Library/LaunchAgents' / (LABEL + '.plist')
    (ROOT / 'logs').mkdir(parents=True, exist_ok=True)
    settings = {'Label': LABEL, 'ProgramArguments': [python, '-m', 'robot_emoji.relay', '--config', str(config), 'watch'],
                'WorkingDirectory': str(ROOT), 'RunAtLoad': True, 'KeepAlive': True, 'ThrottleInterval': 5,
                'StandardOutPath': str(ROOT / 'logs/relay.log'), 'StandardErrorPath': str(ROOT / 'logs/relay.log'),
                'ProcessType': 'Background'}
    agent.write_bytes(plistlib.dumps(settings)); agent.chmod(0o600)
    domain = f'gui/{os.getuid()}'
    subprocess.run(['launchctl', 'bootout', f'{domain}/{LABEL}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for attempt in range(5):
        if subprocess.run(['launchctl', 'bootstrap', domain, str(agent)]).returncode == 0:
            break
        time.sleep(3)
    else:
        raise SystemExit('launchctl bootstrap failed')
    command = Path('/opt/homebrew/bin/hoyabotto-reconnect')
    command.write_text(f'#!/bin/sh\n[ $# -eq 0 ] && set -- check\ncd "{ROOT}" && exec "{python}" -m robot_emoji.relay --config "{config}" "$@"\n')
    command.chmod(0o755)
    print(f'Installed {LABEL}; command: hoyabotto-reconnect <name> | hoyabotto-reconnect check')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('action', nargs='?', default='check')
    parser.add_argument('name', nargs='?')
    a = parser.parse_args()
    if a.action == 'watch':
        watch(a.config)
    elif a.action == 'check':
        check(a.config)
    elif a.action == 'install':
        install(a.config)
    elif a.action == 'reconnect' or NAME.search(a.action.lower()):
        reconnect(a.config, a.name if a.action == 'reconnect' else a.action)
    else:
        parser.error('action: watch | check | reconnect NAME | install')


if __name__ == '__main__':
    main()
