"""Patch the pilot with the read-only robot_get_carton_pose tool and `sense carton`, with source backups.

Idempotent; validates every hook before writing anything. Use --pilot on a staged copy first. The
running chat must be restarted by its owner afterwards (this script never restarts or calls it), and
the pilot's apriltag-geometry.json needs the carton sizes
(farm.perception.carton_pose.install_carton_tag_sizes) for robot_get_tags to give metric carton poses.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

IMPORT = "from farm.perception.carton_pose import CartonPoseRobot, carton_pose_lines\n"
ANCHOR = "from farm.perception.carton_tags import CartonTagRobot, carton_tag_lines\n"
STACK_BEFORE = "chat=Chat(CartonTagRobot(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config))))))"
STACK_AFTER = "chat=Chat(CartonPoseRobot(CartonTagRobot(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config)))))))"
SENSE = """        if 'carton' in what:
            try:
                out += carton_pose_lines(self.call_tool('robot_get_carton_pose', {'include_images': False}, send))
            except Exception as error:
                out.append('carton pose unavailable: '+f'{type(error).__name__}: {str(error)[:160]}')
"""
ROUTE = """            if self.path=='/api/carton-pose':
                try:
                    result=chat.robot.call('robot_get_carton_pose', {'include_images':True})
                    return self.reply(200 if result.get('ok') is not False else 502,result)
                except Exception as error:
                    return self.reply(502,{'ok':False,'error':str(error)[:300]})
"""
GUIDANCE = """
Carton pose: sense with what ["carton"] to read the near face, rim and right short flap hinge in reach-model forward/left/up cm. When tag 11 is visible, top_edge_midpoint_if_flat already accounts for outward lean, carton yaw and the cosine height change; do not recompute it with a signed lateral shortcut. Treat that point as observed geometry for approach planning, not an executable contact target. Current motor/model/range bindings and gripper consistency are not revalidated by this coarse tool. Deliberate pad/claw-body push-folding is permitted under the current owner policy; pinch is optional. Check wrist-camera module clearance and use a wrist look to establish the actual edge between the pads before closing. If tag 11 is absent, lean and top edge are unknown: inspect visually, do not assume vertical. Keep the reported up_cm unchanged; the workspace rim difference is diagnostic, not a height correction. If the head moved from its registered pose, metric carton sensing refuses; do not use old coordinates or automatically move the head during contact. Re-establish the registered pose in a clear setup or re-register. Tags identify panels and estimate geometry; tag disappearance, a stalled close, or an edge beside the pads does not prove a pinch or a completed fold.
"""


def replace_once(source, old, new):
    if new in source:
        if source.count(new) != 1 or old in source.replace(new, "", 1):
            raise ValueError("Duplicate pilot hook; inspect source before installing")
        return source
    if source.count(old) != 1:
        raise ValueError(f"Pilot layout changed; expected one hook: {old[:100]!r}")
    return source.replace(old, new, 1)


def patch_sources(sources):
    out = dict(sources)
    s = out['chat_server.py']
    s = replace_once(s, ANCHOR, ANCHOR + IMPORT)
    s = replace_once(s, STACK_BEFORE, STACK_AFTER)
    s = replace_once(s, "('state','motion','scene','heights','tags')", "('state','motion','scene','heights','tags','carton')")
    s = replace_once(s, 'and/or "heights", "tags"', 'and/or "heights", "tags", "carton"')
    s = replace_once(s, "        if 'heights' in what:\n", SENSE + "        if 'heights' in what:\n")
    s = replace_once(s, "            if self.path=='/api/carton-tags':\n", ROUTE + "            if self.path=='/api/carton-tags':\n")
    s = replace_once(s, 'Carton AprilTags: sense with what ["tags"]', GUIDANCE.strip('\n') + '\nCarton AprilTags: sense with what ["tags"]')
    out['chat_server.py'] = s
    s = out['model_backend.py']
    s = replace_once(s, "['state', 'motion', 'scene', 'heights', 'tags']", "['state', 'motion', 'scene', 'heights', 'tags', 'carton']")
    s = replace_once(s, 'sense only (tags:', 'sense only (carton: the carton near face, rim and right-flap hinge in robot cm from the tags; tags:')
    out['model_backend.py'] = s
    for source in out.values():
        ast.parse(source)
    return out


def install(pilot):
    paths = ['chat_server.py', 'model_backend.py']
    originals = {p: (pilot / p).read_text() for p in paths}
    updated = patch_sources(originals)  # Validate every hook before writing anything.
    receipt = {}
    for name in paths:
        target, old, new = pilot / name, originals[name], updated[name]
        before = hashlib.sha256(old.encode()).hexdigest()
        if new != old:
            backup = pilot / '.private/carton-pose-backups' / f'{target.stem}-{before}.py'
            backup.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if not backup.exists():
                backup.write_text(old)
                backup.chmod(0o600)
            if target.read_text() != old:
                raise RuntimeError(f'{name} changed during installation; inspect before retrying')
            temp = target.with_suffix('.carton-pose.tmp')
            temp.write_text(new)
            temp.chmod(target.stat().st_mode & 0o777)
            temp.replace(target)
        receipt[name] = {'changed': old != new, 'before_sha256': before,
                         'after_sha256': hashlib.sha256(new.encode()).hexdigest()}
    receipt['restart_required'] = any(r['changed'] for r in receipt.values() if isinstance(r, dict))
    receipt['motor_writes'] = 0
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(install(args.pilot.resolve()), indent=2))
