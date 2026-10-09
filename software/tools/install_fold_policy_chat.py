"""Install the fold-policy chat tools (carton/fold_policy_chat.py) into the chat Mac's pilot; never restarts it.

Patches <pilot>/chat_server.py (backup in .private/fold-policy-backups):
- loads FoldPolicyRobot from this checkout by path (the pilot's `farm` is an older checkout; nothing is shadowed);
- wraps the robot chain: chat=Chat(FoldPolicyRobot(CalibrationRobot(...)));
- adds robot_fold_policy_dry_run / robot_fold_policy_run to MOVE_TOOLS, so supervisor `move` decisions can use them.
Writes <pilot>/.private/fold-policy.json if it does not exist, with execution DISABLED; the owner edits it.

    python tools/install_fold_policy_chat.py --pilot <pilot> --checkpoint <ckpt>/pretrained_model \\
        --joint-map left=bound-maps/left-joint-map.json --joint-map right=bound-maps/right-joint-map.json
    python tools/install_fold_policy_chat.py --pilot <pilot> --uninstall

Restart the chat server afterwards (it prints restart_required). No motor command, no robot call.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
from pathlib import Path

SOFTWARE = Path(__file__).resolve().parents[1]
MODULE = SOFTWARE / 'carton/fold_policy_chat.py'
MARK = '# fold-policy chat tools (xlerobot-farm carton/fold_policy_chat.py)'
ANCHOR = 'from farm.perception.gemma_calibration import CalibrationRobot\n'
CHAT = re.compile(r'chat=Chat\((?!FoldPolicyRobot\()(.*?Robot\(args\.config\)\)*)\);')
MOVE = "'robot_move_base','robot_stop')"
MOVE_AFTER = "'robot_move_base','robot_stop','robot_fold_policy_dry_run','robot_fold_policy_run')"
DEFAULT_BLOCKERS = [
    'owner stream mode (jaw closures, no STOP on a refused streamed target) is not deployed and validated: '
    'docs/carton-fold-policy-chat-mac-handoff.md section 4',
    'the policy has not been retrained for the measured station and cameras (handoff section 3)',
]


def loader_line(module):
    return f"FoldPolicyRobot=__import__('runpy').run_path({str(module)!r})['FoldPolicyRobot']  {MARK}\n"


def patch_source(source, module=MODULE):
    """Return the patched chat_server.py source (idempotent); refuse an unexpected layout."""
    if MARK in source:
        return source
    if source.count(ANCHOR) != 1 or len(CHAT.findall(source)) != 1 or source.count(MOVE) != 1:
        raise ValueError('Inspect the changed pilot layout before installing the fold-policy tools')
    updated = source.replace(ANCHOR, ANCHOR + loader_line(module))
    updated = CHAT.sub(lambda m: f'chat=Chat(FoldPolicyRobot({m.group(1)}));', updated)
    updated = updated.replace(MOVE, MOVE_AFTER)
    ast.parse(updated)
    return updated


def unpatch_source(source):
    if MARK not in source:
        return source
    lines = [line for line in source.splitlines(keepends=True) if MARK not in line]
    updated = ''.join(lines).replace(MOVE_AFTER, MOVE)
    updated = re.sub(r'chat=Chat\(FoldPolicyRobot\((.*?Robot\(args\.config\)\)*)\)\);', r'chat=Chat(\1);', updated)
    ast.parse(updated)
    return updated


def write_source(pilot, transform):
    path = Path(pilot) / 'chat_server.py'
    original = path.read_text()
    updated = transform(original)
    if updated != original:
        folder = Path(pilot) / '.private/fold-policy-backups'
        folder.mkdir(parents=True, exist_ok=True, mode=0o700)
        backup = folder / f'chat_server-{hashlib.sha256(original.encode()).hexdigest()[:16]}.py'
        if not backup.exists():
            backup.write_text(original)
            backup.chmod(0o600)
        if path.read_text() != original:
            raise RuntimeError('Pilot changed during installation')
        stage = path.with_suffix('.fold-policy.tmp')
        stage.write_text(updated)
        stage.chmod(path.stat().st_mode & 0o777)
        stage.replace(path)
    return original != updated


def default_config(pilot, checkpoint, joint_maps, python=None, runs_dir=None):
    pilot = Path(pilot).resolve()
    return {
        'schema': 1,
        'software_root': str(SOFTWARE),
        'python': str(python or pilot.parent / '.venv/bin/python'),
        'pilot_root': str(pilot),
        'checkpoint': str(Path(checkpoint).resolve()),
        'joint_maps': {arm: str(Path(p).resolve()) for arm, p in sorted(joint_maps.items())},
        'cameras': {'front': 'oak', 'left_wrist': 'left_wrist', 'right_wrist': 'right_wrist'},
        'runs_dir': str(runs_dir or pilot.parent / 'fold-policy-runs'),
        'device': 'cpu',
        'transport': 'api',
        'gripper_mode': 'hold',
        'dry_run_max_steps': 300,
        'dry_run_valid_s': 900,
        'execute_enabled': False,
        'execute_max_steps': 10,
        'operators': [],
        'blockers': list(DEFAULT_BLOCKERS),
    }


def install(pilot, checkpoint=None, joint_maps=None, python=None, runs_dir=None):
    pilot = Path(pilot)
    config = pilot / '.private/fold-policy.json'
    wrote_config = False
    if not config.exists():
        if not checkpoint or not joint_maps or set(joint_maps) != {'left', 'right'}:
            raise SystemExit('First install needs --checkpoint and --joint-map left=... --joint-map right=...')
        missing = [str(p) for p in [Path(checkpoint) / 'model.safetensors', *map(Path, joint_maps.values())]
                   if not p.exists()]
        if missing:
            raise SystemExit(f'Missing files: {missing}')
        config.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        config.write_text(json.dumps(default_config(pilot, checkpoint, joint_maps, python, runs_dir), indent=1) + '\n')
        wrote_config = True
    changed = write_source(pilot, patch_source)
    return {'changed': changed, 'config_written': wrote_config, 'config': str(config),
            'restart_required': changed, 'motor_writes': 0}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pilot', type=Path, required=True)
    ap.add_argument('--checkpoint', type=Path, help='ACT pretrained_model directory (first install)')
    ap.add_argument('--joint-map', action='append', default=[], metavar='ARM=PATH', help='bound joint maps (first install)')
    ap.add_argument('--python', type=Path, help='Python with torch + lerobot (default: the pilot venv)')
    ap.add_argument('--runs-dir', type=Path)
    ap.add_argument('--uninstall', action='store_true', help='remove the patch (keeps the config and runs)')
    args = ap.parse_args(argv)
    if args.uninstall:
        changed = write_source(args.pilot, unpatch_source)
        print(json.dumps({'changed': changed, 'restart_required': changed, 'motor_writes': 0}))
        return
    maps = dict(item.split('=', 1) for item in args.joint_map)
    print(json.dumps(install(args.pilot, args.checkpoint, maps, args.python, args.runs_dir)))


if __name__ == '__main__':
    main()
