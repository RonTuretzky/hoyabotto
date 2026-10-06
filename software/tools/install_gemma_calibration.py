"""Install calibration hooks into the existing tag-enabled pilot; never restart it."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

IMPORT = 'from farm.perception.gemma_calibration import CalibrationRobot\n'
ANCHOR = 'from farm.perception.gemma_tags import TagRobot\n'
BEFORE = 'chat=Chat(TagRobot(Robot(args.config)))'
AFTER = 'chat=Chat(CalibrationRobot(TagRobot(Robot(args.config))))'


def patch_source(source):
    if IMPORT in source and AFTER in source:
        return source
    if IMPORT in source or AFTER in source or source.count(ANCHOR) != 1 or source.count(BEFORE) != 1:
        raise ValueError('Inspect changed pilot layout before installing calibration hooks')
    updated = source.replace(ANCHOR, ANCHOR+IMPORT).replace(BEFORE, AFTER)
    ast.parse(updated)
    return updated


def install(pilot):
    path = Path(pilot)/'chat_server.py'
    original = path.read_text()
    updated = patch_source(original)
    if updated != original:
        folder = Path(pilot)/'.private/calibration-adapter-backups'
        folder.mkdir(parents=True, exist_ok=True, mode=0o700)
        backup = folder/f'chat_server-{hashlib.sha256(original.encode()).hexdigest()[:16]}.py'
        if not backup.exists():
            backup.write_text(original)
            backup.chmod(0o600)
        if path.read_text() != original:
            raise RuntimeError('Pilot changed during installation')
        stage = path.with_suffix('.calibration.tmp')
        stage.write_text(updated)
        stage.chmod(path.stat().st_mode & 0o777)
        stage.replace(path)
    return {'changed': original != updated, 'restart_required': original != updated, 'motor_writes': 0}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pilot', type=Path, required=True)
    print(json.dumps(install(p.parse_args().pilot)))
