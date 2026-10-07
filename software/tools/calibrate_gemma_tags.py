"""Read-only preflight or explicit automatic calibration via the existing Gemma client."""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import sys

from farm.perception.gemma_tags import TagRobot
from farm.perception.gemma_calibration import CalibrationRobot


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['status', 'registered', 'paddle_target', 'local_model', 'registration'])
    p.add_argument('--pilot-root', type=Path, required=True)
    p.add_argument('--execute', action='store_true', help='Explicitly enable and move the configured positioning joints')
    args = p.parse_args()
    read_only = args.command in ('status', 'registered', 'paddle_target')
    if not read_only and not args.execute:
        p.error('Movement requires --execute; use status for a read-only check')
    sys.path.insert(0, str(args.pilot_root.resolve()))
    raw = importlib.import_module('chat_server').Robot(args.pilot_root/'.private/robot.json')
    robot = CalibrationRobot(TagRobot(raw))
    robot.catalog()
    name = {'status':'robot_calibration_status', 'registered':'robot_get_registered_tags',
            'paddle_target':'robot_get_paddle_target'}.get(args.command, 'robot_calibrate_tags')
    result = robot.call(name, {} if read_only else {'mode': args.command})
    result.pop('images', None)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
