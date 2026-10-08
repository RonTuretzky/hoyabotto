"""Check local Gemma's use of live tags with a strictly read-only tool catalog.

Uses the existing pilot client/parser and keeps its conversation untouched.
Only robot_get_tags can be dispatched; motor tools are never exposed.
"""
import argparse
import importlib
import json
from pathlib import Path
import sys
import time

import jsonschema

from farm.perception.gemma_tags import TagRobot, TOOL_NAME


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot-root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Choose a new evidence filename')
    sys.path.insert(0, str(args.pilot_root.resolve()))
    pilot = importlib.import_module('chat_server')
    robot = TagRobot(pilot.Robot(args.pilot_root / '.private/robot.json'))
    tools = [t for t in robot.catalog()['tools'] if t['function']['name'] == TOOL_NAME]
    if len(tools) != 1:
        raise ValueError('Expected the existing camera-backed tag tool')
    messages = [dict(role='system', content='You are inspecting live AprilTag measurements. '
        'Use exactly one tool per turn. Only the read-only tag tool is available. '
        'Report the measurements and their limitations; do not claim a grasp, calibrated robot coordinates or movement.'),
        dict(role='user', content='Read robot_get_tags for camera oak, include_images false. '
        'Report visible tag IDs, camera-relative tag-centre distances in millimetres if available, '
        'the gripper-to-paddle tag-centre separation, and whether these data suffice to command a grasp. '
        'Call the tool once, then answer without requesting movement.')]
    report = {'model': pilot.MODEL, 'motor_tools_exposed': False, 'motor_writes': 0, 'calls': []}
    try:
        for _ in range(3):
            start = time.monotonic()
            body = pilot.validated_body(dict(model=pilot.MODEL, messages=messages, tools=tools,
                temperature=0, max_tokens=2048, stream=False))
            response = pilot.request_json('http://127.0.0.1:1234/v1/chat/completions', body, timeout=90)
            message = response['choices'][0]['message']
            if not message.get('tool_calls'):
                native = pilot.native_tool_call(message.get('content'))
                if native:
                    message = {**message, 'content': None, 'tool_calls': [native]}
            messages.append(message)
            calls = message.get('tool_calls') or []
            if not calls:
                report['answer'] = pilot.final_text(message.get('content'))
                break
            if len(calls) != 1 or calls[0]['function']['name'] != TOOL_NAME or report['calls']:
                raise ValueError('Only one read-only tag call is allowed in this probe')
            call = calls[0]
            arguments = pilot.strict_json(call['function']['arguments'])
            jsonschema.validate(arguments, tools[0]['function']['parameters'])
            if arguments.get('cameras') != ['oak']:
                raise ValueError('This probe reads only the commissioned OAK view')
            result = robot.call(TOOL_NAME, arguments)
            compact = {k: v for k, v in result.items() if k != 'images'}
            report['calls'].append(dict(name=TOOL_NAME, arguments=arguments, result=compact,
                                         latency_s=round(time.monotonic()-start, 3)))
            messages.append(dict(role='tool', tool_call_id=call['id'], content=json.dumps(compact)))
            if not result.get('ok'):
                raise ValueError('Live tag observation was refused')
        report['passed'] = bool(report.get('answer') and len(report['calls']) == 1
            and report['calls'][0]['result'].get('result', {}).get('metric_pose_available'))
    except Exception as exc:
        report.update(passed=False, error=str(exc))
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'calls'}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
