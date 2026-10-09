"""Patch the existing pilot's read-only carton sensing hooks, with source backups.

Use --pilot on a staged copy first. Runtime must import this checkout's
farm.perception.carton_tags. No process restart, camera read or motion occurs here.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

IMPORT = "from farm.perception.carton_tags import CartonTagRobot, carton_tag_lines\n"
SENSE = """        if 'tags' in what:
            try:
                out += carton_tag_lines(self.call_tool('robot_get_carton_tags', {'include_images': False}, send))
            except Exception as error:
                out.append('tags unavailable: '+f'{type(error).__name__}: {str(error)[:160]}')
"""
ROUTE = """            if self.path=='/api/carton-tags':
                try:
                    result=chat.robot.call('robot_get_carton_tags', {'include_images':True})
                    return self.reply(200 if result.get('ok') is not False else 502,result)
                except Exception as error:
                    return self.reply(502,{'ok':False,'error':str(error)[:300]})
"""
GUIDANCE = """
Carton AprilTags: sense with what ["tags"] to read the printed carton IDs from fresh camera frames. ID 12 is the right short flap; 11 left short flap; 13 far long flap; 14 near long flap; 10/26/27 near wall; 21/28 left wall; 22 right wall; 24/25 floor. Use the pixel observations to identify panels and ask the eyes a specific question. A tag centre is NOT a pinch point. Missing/rejected tags are unknown, never proof of a folded flap. Tag pixels are not robot coordinates or clearance measurements. No automatic head/base movement is part of sense tags. Current head-to-arm registration is unverified: do not drive the base from box-only pose translation.
"""


def replace_once(source, old, new):
    if new in source:
        return source
    if source.count(old) != 1:
        raise ValueError(f"Pilot layout changed; expected one hook: {old[:100]!r}")
    return source.replace(old, new, 1)


def patch_sources(sources):
    out = dict(sources)
    s = out['chat_server.py']
    s = replace_once(s, 'from farm.perception.gemma_tags import TagRobot\n',
                     'from farm.perception.gemma_tags import TagRobot\n' + IMPORT)
    s = replace_once(s, 'chat=Chat(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config)))))',
                     'chat=Chat(CartonTagRobot(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config))))))')
    s = replace_once(s, "('state','motion','scene','heights')", "('state','motion','scene','heights','tags')")
    s = replace_once(s, 'and/or "heights"', 'and/or "heights", "tags"')
    s = replace_once(s, "        if 'heights' in what:\n", SENSE + "        if 'heights' in what:\n")
    s = replace_once(s, "            if self.path=='/api/status':return self.reply(200,chat.status())\n",
                     "            if self.path=='/api/status':return self.reply(200,chat.status())\n" + ROUTE)
    s = replace_once(s, 'Cameras and what their directions mean:\n', 'Cameras and what their directions mean:\n' + GUIDANCE)
    s = replace_once(s, '# no AprilTag workflow for the pilot', '# legacy paddle commissioning remains hidden; carton sensing is separate')
    out['chat_server.py'] = s
    s = out['model_backend.py']
    s = replace_once(s, "['state', 'motion', 'scene', 'heights']", "['state', 'motion', 'scene', 'heights', 'tags']")
    s = replace_once(s, 'sense only (heights:', 'sense only (tags: fresh carton tag IDs and per-image pixels; heights:')
    out['model_backend.py'] = s
    s = out['bench/bench.py']
    s = replace_once(s, '    from farm.perception.twin_robot import TwinRobot\n',
                     '    from farm.perception.twin_robot import TwinRobot\n    from farm.perception.carton_tags import CartonTagRobot\n')
    s = replace_once(s, "return sim, TwinRobot(sim, joint_map_path=Path(root) / '.private/twin-joint-map.json')",
                     "return sim, CartonTagRobot(TwinRobot(sim, joint_map_path=Path(root) / '.private/twin-joint-map.json'))")
    out['bench/bench.py'] = s
    for source in out.values():
        ast.parse(source)
    return out


def install(pilot):
    paths = ['chat_server.py', 'model_backend.py', 'bench/bench.py']
    originals = {p: (pilot / p).read_text() for p in paths}
    updated = patch_sources(originals)  # Validate every hook before writing anything.
    receipt = {}
    for name in paths:
        target, old, new = pilot / name, originals[name], updated[name]
        before = hashlib.sha256(old.encode()).hexdigest()
        if new != old:
            backup = pilot / '.private/carton-tag-backups' / f'{target.stem}-{before}.py'
            backup.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if not backup.exists():
                backup.write_text(old)
                backup.chmod(0o600)
            if target.read_text() != old:
                raise RuntimeError(f'{name} changed during installation; inspect before retrying')
            temp = target.with_suffix('.carton-tags.tmp')
            temp.write_text(new)
            temp.chmod(target.stat().st_mode & 0o777)
            temp.replace(target)
        receipt[name] = {'changed': old != new, 'before_sha256': before,
                         'after_sha256': hashlib.sha256(new.encode()).hexdigest()}
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(install(args.pilot.resolve()), indent=2))
