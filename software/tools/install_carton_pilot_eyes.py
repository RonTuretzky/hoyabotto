"""Replace the pilot eyes prompt, optionally reconciling supervisor guidance.

Use --reconcile-supervisor after the push-policy installer to remove conflicting
overlay/grasp heuristics and explain mirrored panels and the existing demo profile.
All changes are text-only, with AST guards and source backups.

No imports of the pilot, model calls, device access or restarts occur here.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
from pathlib import Path

SPEED_GUIDANCE = ('Arm speed profiles: the owner accepts speed_profile "normal" or "demo". '
                  'Demo means a maximum arm command rate of 300 ticks/second (about 26.37 degrees/second); '
                  'normal is 100 ticks/second. A 300-tick waypoint step is a position increment, not a speed profile. '
                  'When the user requests the 300 ticks/second profile, select "demo" explicitly on '
                  'robot_set_motor_enable for all six joints of the chosen, already released arm at its next '
                  'authorized enable. Never send "300" or "300 tick profile" as the profile value. '
                  'The reach/fold/drive convenience enables omit this field and default to normal; to use the '
                  'requested demo profile, perform the explicit enable first and inspect the owner arm_speed_profiles '
                  'readback. Do not release a holding arm just to switch profiles. The Joy-Con speed picker does '
                  'not configure pilot speed. Profile selection alone does not authorize enabling or moving motors. '
                  'Longer requested move durations can still make motion slower; the rate ceiling is not measured '
                  'achieved speed. Grippers, head, wheels and all owner guards retain their settings.')

VISION_SECTIONS = (
    ('- left_wrist / right_wrist:', '\n- phone:',
     'f0cd922b62e2ddca5c32dd90459aa0697742bc04a283d765537601d50e96e9b1',
     '- left_wrist / right_wrist: use the actual visible pads, tips, cardboard edge and any visible gap to '
     'assess contact. Wrist image directions depend on wrist roll, pitch and perspective; image-up is not '
     'world-up. The JAW ZONE and 2 cm scale are unvalidated overlays, not measured pad boundaries or metric '
     'distances. A projected edge inside that box is only a candidate alignment, not proof it lies between '
     'the real pads. Ask the eyes to describe pixels or qualitative separation and occlusion; never convert '
     'overlay pixels into centimetres or issue a contact move from the overlay alone. Choose a guarded '
     'approach using fresh geometry and camera clearance; use another view when the contact is hidden.'),
    ('6) Open the gripper before the approach;', '7) If a grasp is needed,',
     'd69d3272b2376f974b1fb4b0f7eeed0b952980e34a22d88f03b5a2f569540d4f',
     '6) For an optional pinch, choose an opening and approach that safely straddle the observed edge. '
     'Locate the edge relative to the actual pads before closing. The overlay cannot establish a 1-2 cm '
     'insertion depth or a contact target. PINCH LIKELY is a servo heuristic, not proof that the intended '
     'flap is held. Reconcile it with fresh views and telemetry; conflicting or occluded evidence is '
     'uncertain. Do not automatically descend, lift, open or retry a close to resolve uncertainty. '),
    ('Jaw orientation for the optional pinch approach', '\n\n',
     '93632cb1fa4dc7897e63d69a5cd39e9fd16e9dc95a5fa0bb8179a19dfccd46bf',
     'Jaw orientation for an optional pinch: identify the selected flap edge and the real pad opening '
     'direction in the current wrist view. A near/far edge may require wrist rotation, but no fixed 90 '
     'degree roll or overlay position proves alignment. Plan rotation from the observed wrist pose, '
     'ranges and camera/arm clearance. Rolling the wrist also rotates the image; re-establish directions '
     'with fresh views before translating image directions into robot motion. Do not roll automatically '
     'while holding or pressing the flap.'),
)


def reconcile_supervisor(source: str) -> str:
    """Text-only reconciliation after the push-policy installer, before activation."""
    matches = [n for n in ast.parse(source).body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == 'SUPERVISOR_SYSTEM' for t in n.targets)]
    if len(matches) != 1 or not isinstance(matches[0].value, ast.Constant) or not isinstance(matches[0].value.value, str):
        raise ValueError('Expected literal SUPERVISOR_SYSTEM')
    node = matches[0].value
    prompt = node.value
    for begin, end, digest, replacement in VISION_SECTIONS:
        if replacement in prompt:
            if prompt.count(replacement) != 1:
                raise ValueError('Duplicate supervisor vision guidance')
            continue
        start = prompt.index(begin)
        stop = prompt.index(end, start + len(begin))
        old = prompt[start:stop]
        if hashlib.sha256(old.encode()).hexdigest() != digest:
            raise ValueError('Supervisor vision guidance changed; install push policy first and inspect source')
        prompt = prompt[:start] + replacement + prompt[stop:]
    old = 'ID 12 is the right short flap; 11 left short flap; 13 far long flap; 14 near long flap; 10/26/27 near wall; 21/28 left wall; 22 right wall; 24/25 floor.'
    new = ('For the current mirrored physical carton: 11 is the robot-right short flap and 12 robot-left; '
           '13 far long flap; 14 near long flap; 10/26/27 near wall; 21/28 robot-right wall; 22 robot-left wall; '
           '24/25 floor. Raw tag labels describe the print plan; use carton pose robot_side_role for this placement.')
    if old in prompt and new not in prompt and prompt.count(old) == 1:
        prompt = prompt.replace(old, new, 1)
    elif old in prompt or prompt.count(new) != 1:
        raise ValueError('Carton panel guidance changed')
    if SPEED_GUIDANCE not in prompt:
        prompt += '\n\n' + SPEED_GUIDANCE
    elif prompt.count(SPEED_GUIDANCE) != 1:
        raise ValueError('Duplicate speed guidance')
    if prompt == node.value:
        return source
    lines = source.encode().splitlines(keepends=True)
    start = sum(map(len, lines[:node.lineno - 1])) + node.col_offset
    end = sum(map(len, lines[:node.end_lineno - 1])) + node.end_col_offset
    # Preserve literal newlines so other reviewed source hooks remain readable.
    literal = '"""' + prompt.replace('\\', '\\\\').replace('"""', '\\"\\"\\"') + '"""'
    result = (source.encode()[:start] + literal.encode() + source.encode()[end:]).decode()
    ast.parse(result)
    return result


def replace_prompt(source: str, prompt: str) -> str:
    tree = ast.parse(source)
    matches = [node for node in tree.body if isinstance(node, ast.Assign)
               and any(isinstance(target, ast.Name) and target.id == "EYES_SYSTEM"
                       for target in node.targets)]
    if len(matches) != 1 or not isinstance(matches[0].value, ast.Constant) or not isinstance(matches[0].value.value, str):
        raise ValueError("Expected exactly one literal EYES_SYSTEM assignment; inspect pilot layout")
    node = matches[0]
    if node.value.value == prompt:
        return source
    lines = source.splitlines(keepends=True)
    before = "".join(lines[:node.lineno - 1]) + lines[node.lineno - 1].encode()[:node.col_offset].decode()
    after = lines[node.end_lineno - 1].encode()[node.end_col_offset:].decode() + "".join(lines[node.end_lineno:])
    updated = before + "EYES_SYSTEM=" + repr(prompt) + after
    ast.parse(updated)
    return updated


def install(pilot: Path, prompt_path: Path, *, reconcile: bool = False) -> dict:
    target = pilot / "chat_server.py"
    source = target.read_text()
    prompt = prompt_path.read_text()
    updated = replace_prompt(source, prompt)
    if reconcile:
        updated = reconcile_supervisor(updated)
    digest = hashlib.sha256(source.encode()).hexdigest()
    if source != updated:
        backup = pilot / ".private/carton-eyes-backups" / f"chat_server-{digest}.py"
        backup.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if not backup.exists():
            backup.write_text(source)
            backup.chmod(0o600)
        if target.read_text() != source:
            raise RuntimeError("Pilot changed during installation; inspect before retrying")
        temp = target.with_suffix(".eyes.tmp")
        temp.write_text(updated)
        temp.chmod(target.stat().st_mode & 0o777)
        temp.replace(target)
    return {"changed": source != updated, "before_sha256": digest,
            "after_sha256": hashlib.sha256(updated.encode()).hexdigest(),
            "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest()}


if __name__ == "__main__":
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot", type=Path, required=True)
    parser.add_argument("--prompt", type=Path,
                        default=Path(__file__).resolve().parents[1] / "docs/prompts/carton-pilot-eyes.txt")
    parser.add_argument('--reconcile-supervisor', action='store_true',
                        help='After installing push policy, align supervisor vision, mirrored tags and speed guidance')
    args = parser.parse_args()
    print(json.dumps(install(args.pilot.resolve(), args.prompt.resolve(), reconcile=args.reconcile_supervisor), indent=2))
