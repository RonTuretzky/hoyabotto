"""Install the show outside a Conductor workspace and start it automatically at Mac login."""
import argparse
import json
import os
import plistlib
import shutil
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlsplit

from .robot import DEFAULT_CONFIG

LABEL = 'com.hoyabotto.emoji-show'
ROOT = Path.home() / 'Library/Application Support/HoyaBotto'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api-url', required=True)
    parser.add_argument('--robot-config', default=DEFAULT_CONFIG)
    parser.add_argument('--token-file', type=Path, default=ROOT / 'bridge-token')
    args = parser.parse_args()
    target = urlsplit(args.api_url)
    if target.scheme != 'https' or target.path not in ('', '/') or target.query or target.fragment or target.username or target.password:
        parser.error('--api-url must be a public HTTPS origin')
    if not args.token_file.exists():
        parser.error('A configured cloud bridge token file is required')
    ROOT.mkdir(parents=True, exist_ok=True)
    ROOT.chmod(0o700)
    (ROOT / 'logs').mkdir(exist_ok=True)
    token_file = ROOT / 'bridge-token'
    if args.token_file.resolve() != token_file.resolve():
        shutil.copyfile(args.token_file, token_file)
    token_file.chmod(0o600)
    shutil.copytree(Path(__file__).parent, ROOT / 'robot_emoji', dirs_exist_ok=True,
        ignore=shutil.ignore_patterns('__pycache__', 'cloud'))
    cloudflared = shutil.which('cloudflared')
    if not cloudflared:
        parser.error('cloudflared must be installed')
    configuration = {'api_url': args.api_url.rstrip('/'), 'token_file': str(token_file),
        'robot_config': str(Path(args.robot_config).resolve()), 'cloudflared': cloudflared}
    (ROOT / 'runtime.json').write_text(json.dumps(configuration, indent=2) + '\n')
    python = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/.venv/bin/python'
    if not Path(python).exists():
        parser.error('The existing paired-client Python environment is required')
    agent = Path.home() / 'Library/LaunchAgents' / (LABEL + '.plist')
    settings = {'Label': LABEL, 'ProgramArguments': [python, '-m', 'robot_emoji.supervisor', '--config', str(ROOT / 'runtime.json')],
        'WorkingDirectory': str(ROOT), 'RunAtLoad': True, 'KeepAlive': True, 'ThrottleInterval': 5,
        'StandardOutPath': str(ROOT / 'logs/supervisor.log'), 'StandardErrorPath': str(ROOT / 'logs/supervisor.log'),
        'ProcessType': 'Background'}
    agent.parent.mkdir(exist_ok=True)
    agent.write_bytes(plistlib.dumps(settings))
    agent.chmod(0o600)
    domain = f'gui/{os.getuid()}'
    subprocess.run(['launchctl', 'bootout', domain + '/' + LABEL], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['launchctl', 'bootstrap', domain, str(agent)], check=True)
    print(f'Installed {LABEL}; starts automatically at login after a restart.')
    print(f'Runtime: {ROOT}; visitor API: {args.api_url}; operator: http://127.0.0.1:8790/operator')


if __name__ == '__main__':
    main()
