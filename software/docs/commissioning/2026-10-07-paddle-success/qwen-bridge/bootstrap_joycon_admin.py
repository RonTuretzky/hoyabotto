"""Install only the fixed admin modes; preserve the live API code and motor owner.

Used from an isolated bootstrap Git ref through the existing authenticated
deployment endpoint. Does not load the Joy-Con adapter or touch motor settings.
"""
import argparse
import hashlib
import json
import shutil
import time
from pathlib import Path
import redeploy_robot_server as deploy
from joycon_commissioning import require_released, require_no_pending_command


BASELINE_ADMIN_SHA256 = '0a7230b3be7ee9f3af524ee4921e1ce80c72a355995d7082aaa0a21dbd1a3edf'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    before = require_released(deploy.read_status())
    require_no_pending_command(before, deploy.SESSION / 'command.json')
    owner_pids = deploy.processes('gemma_hardware_owner.py')
    if len(owner_pids) != 1:
        raise ValueError('Exactly one existing hardware owner required')
    source = deploy.BRIDGE / 'remote_admin.py'
    target = deploy.WORK / 'remote_admin.py'
    old_bytes = target.read_bytes()
    new_bytes = source.read_bytes()
    if hashlib.sha256(old_bytes).hexdigest() not in (BASELINE_ADMIN_SHA256, hashlib.sha256(new_bytes).hexdigest()):
        raise ValueError('Installed admin module differs from the reviewed baseline; preserve it for inspection')
    receipt = {'owner_pid': owner_pids[0], 'owner_started': before['started'],
               'old_sha256': hashlib.sha256(old_bytes).hexdigest(),
               'new_sha256': hashlib.sha256(new_bytes).hexdigest(),
               'changed_file': 'remote_admin.py', 'dry_run': args.dry_run}
    print(json.dumps(receipt, indent=2), flush=True)
    if args.dry_run:
        return
    # Stop only the API so no command can arrive while its admin module changes.
    if deploy.stop(deploy.processes('gemma_robot_tools.py'), 'API', 10):
        raise RuntimeError('API did not exit; no file changed')
    backup = deploy.WORK / 'backups' / time.strftime('joycon-admin-%Y%m%d-%H%M%S')
    installed = False
    try:
        require_no_pending_command(require_released(deploy.read_status()), deploy.SESSION / 'command.json')
        backup.mkdir(parents=True)
        shutil.copy2(target, backup / 'remote_admin.py')
        shutil.copy2(source, target)
        installed = True
        api = deploy.start_api()
        after = require_released(deploy.read_status())
        if deploy.processes('gemma_hardware_owner.py') != owner_pids or after['started'] != before['started']:
            raise RuntimeError('Hardware owner changed during API bootstrap')
        deploy.record_deploy('joycon-admin-bootstrap')
        print(json.dumps(receipt | {'api_pid': api.pid, 'backup': str(backup),
                                  'owner_unchanged': True, 'all_released': True}, indent=2), flush=True)
    except BaseException:
        if installed:
            if deploy.stop(deploy.processes('gemma_robot_tools.py'), 'API rollback', 10):
                raise RuntimeError('API rollback requires inspection; owner untouched')
            shutil.copy2(backup / 'remote_admin.py', target)
        if not deploy.processes('gemma_robot_tools.py'):
            deploy.start_api()
        raise


if __name__ == '__main__':
    main()
