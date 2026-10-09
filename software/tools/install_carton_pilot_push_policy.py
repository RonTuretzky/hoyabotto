"""Install the current push-fold policy into an explicitly supplied pilot directory.

Source-only: never imports the pilot, opens devices, calls models or restarts anything.
Stage copies first. Both source files are AST-checked before writes; executable
structure and non-string constants must remain identical. Backups and a receipt
support exact, drift-guarded rollback. An idle parent-controlled reload and fresh
task context are separate activation steps.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import tempfile

POLICY_PATH = Path(__file__).resolve().parents[1] / "docs/prompts/carton-pilot-push-authorized-v1.txt"
POLICY_MARKER = "Folding a flap (push-authorized owner policy, 10 October 2026):"
LEGACY_FOLD_PREFIX = "Folding a flap (after a verified pinch on it):"
LEGACY_FOLD_SHA256 = "f4cf5b2727c4c08aeb7b10049dca0c393049f7a1f72553ae0e55ff2f07643fa5"
FILES = ("chat_server.py", "model_backend.py")
BACKUPS = ".private/carton-push-policy-backups"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def replace_once(source: str, old: str, new: str, count: int = 1) -> str:
    """Accept only the known old text or the exact already-installed text."""
    if old not in source and source.count(new) == count:
        return source
    if source.count(old) != count or new in source:
        raise ValueError(f"Pilot layout changed; inspect hook {old[:100]!r}")
    return source.replace(old, new)


def assignment(tree: ast.Module, name: str) -> ast.Assign:
    matches = [node for node in tree.body if isinstance(node, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {name} assignment")
    return matches[0]


def replace_node(source: str, node: ast.AST, new: str) -> str:
    # AST columns count UTF-8 bytes, including when a prior line contains Unicode.
    lines = source.encode().splitlines(keepends=True)
    start = sum(map(len, lines[:node.lineno - 1])) + node.col_offset
    end = sum(map(len, lines[:node.end_lineno - 1])) + node.end_col_offset
    return (source.encode()[:start] + new.encode() + source.encode()[end:]).decode()


def text_only_ast(source: str) -> str:
    """Ignore human-facing string contents, preserve every executable expression."""
    class MaskText(ast.NodeTransformer):
        def visit_Constant(self, node):
            if isinstance(node.value, str):
                node.value = "<text>"
            return node

        def visit_JoinedStr(self, node):
            node.values = [self.visit(n) for n in node.values if not isinstance(n, ast.Constant)]
            return node

    return ast.dump(MaskText().visit(ast.parse(source)), include_attributes=False)


PROMPT_EDITS = (
    ("FOLDS a pinched flap about its hinge:", "runs a geometric claw-tip arc for a planned push-fold or pinch-fold:"),
    ("radius_cm (pinch height minus the hinge height, the box top edge)",
     "radius_cm (claw-tip height minus the hinge height, assuming a vertical start above the box top edge)"),
    ("after the last step it holds hold_s seconds with the gripper closed",
     "after the last step it holds hold_s seconds with the gripper command unchanged"),
    ("(any lean, any pinch depth)", "(any modeled claw-tip position)"),
    ("a closed gripper gives only a sliver, so open it before asking",
     "a closed gripper gives only a sliver; for a planned pinch, choose a safe opening before asking"),
    ("'edge INSIDE the jaw zone' means the edge is between the pads: close the gripper now",
     "'edge INSIDE the jaw zone' identifies a candidate pinch: close only if a pinch is the selected safe plan"),
    ("How to reach and grasp (learned on this robot):",
     "How to reach and grasp when a pinch is chosen (not a prerequisite for push-folding):"),
    ("Jaw orientation (learned 8 October):", "Jaw orientation for the optional pinch approach (learned 8 October):"),
)
LEGACY_LIFT_PREFIX = "7) Verify a grasp by lifting:"
LEGACY_LIFT_SHA256 = "2dd1dc57984b7495f38ed4820848ade4b6c15b0d25f28ad12f8a58cfb45027e9"
NEW_GRASP = ("7) If a grasp is needed, assess fresh views and gripper telemetry together; a PINCH LIKELY verdict alone "
             "does not prove the flap is held or folded. A push-fold requires neither a pinch verdict nor a lift test. "
             "Do not lift the carton to verify a push. Any separately needed grasp-verification motion requires a safe "
             "plan and clearance; do not treat contact_halt as payload evidence or resend it automatically.")


def patch_prompt(prompt: str, policy: str) -> str:
    policy = policy.strip()
    if not policy.startswith(POLICY_MARKER):
        raise ValueError("Unexpected push policy artifact")
    if POLICY_MARKER in prompt:
        if prompt.count(POLICY_MARKER) != 1 or policy not in prompt or LEGACY_FOLD_PREFIX in prompt:
            raise ValueError("Installed policy differs; inspect before updating")
    else:
        old = [p for p in prompt.split("\n\n") if p.startswith(LEGACY_FOLD_PREFIX)]
        if len(old) != 1 or sha(old[0].encode()) != LEGACY_FOLD_SHA256:
            raise ValueError("Legacy folding paragraph changed; inspect before installing")
        prompt = prompt.replace(old[0], policy, 1)
    if LEGACY_LIFT_PREFIX in prompt:
        start = prompt.index(LEGACY_LIFT_PREFIX)
        end = prompt.index(" 8) Step sizes:", start)
        legacy = prompt[start:end]
        if sha(legacy.encode()) != LEGACY_LIFT_SHA256:
            raise ValueError("Legacy grasp paragraph changed; inspect before installing")
        prompt = prompt[:start] + NEW_GRASP + prompt[end:]
    elif prompt.count(NEW_GRASP) != 1:
        raise ValueError("Missing grasp-verification hook")
    for old, new in PROMPT_EDITS:
        # The optional eyes reconciliation replaces these complete paragraphs
        # after this installer. Preserve its stricter observation guidance.
        if ('The JAW ZONE and 2 cm scale are unvalidated overlays' in prompt
                and 'Jaw orientation for an optional pinch:' in prompt
                and old in ('a closed gripper gives only a sliver, so open it before asking',
                            "'edge INSIDE the jaw zone' means the edge is between the pads: close the gripper now",
                            'Jaw orientation (learned 8 October):')):
            continue
        prompt = replace_once(prompt, old, new)
    return prompt


# These edits affect text only inside the named helper/method, never control flow.
FOLD_EDITS = {
    "parse_fold": (("(pinch height minus the hinge height)", "(claw-tip height minus the hinge height for a vertical start)"),),
    "fold_arc": (("so a flap that already leans\n    is carried from where it is", "the arc begins at the current tip\n    without inferring flap contact"),
                 ("straight below the pinch", "straight below the claw tip")),
    "fold_words": (("below the pinch, turning", "below the claw tip, commanded arc"),),
    "fold_steps": (("the pinch straight above", "the claw tip straight above"),
                   ("and turns it by degrees", "and moves the tip by degrees")),
    "execute_fold": (
        ("fold: the pinched point on a circular arc", "fold: the claw tip on a geometric circular arc"),
        ("the flap is turned about {a:.0f} deg", "last completed commanded arc step {a:.0f} deg; flap angle unverified"),
        ("contact halt (the crease or the box resists); the flap is turned between {a:.0f} and {b:.0f} deg; the arm is holding",
         "contact halt during commanded arc step {a:.0f} to {b:.0f} deg; cause and flap angle unverified; the arm is holding; inspect before a new safe plan, no automatic retry"),
        ("holding the flap {hold:g} s with the gripper closed", "holding the arc endpoint {hold:g} s with the gripper command unchanged"),
        ("held {hold:g} s at the end of the arc with the gripper closed", "held {hold:g} s at the end of the arc with the gripper command unchanged"),
        ("fold complete:", "fold arc finished: commanded angle"),
        ("deg; the gripper is still closed: open it, raise 5 cm and look whether the flap stays down",
         "deg; flap angle and closure unverified; gripper command unchanged. Observe carton displacement and tipping, then plan safe disengagement and inspect springback; no automatic release or retreat"),
    ),
    "execute_fold_path": (
        ("the flap has not turned", "flap motion unverified; inspect fresh views and telemetry"),
        ("contact halt during the path (the crease or the box resists); the arm is holding",
         "contact halt during the path; cause and flap angle unverified; the arm is holding; inspect before a new safe plan, no automatic retry"),
        ("holding the flap {hold:g} s with the gripper closed", "holding the arc endpoint {hold:g} s with the gripper command unchanged"),
        ("held {hold:g} s at the end of the arc with the gripper closed", "held {hold:g} s at the end of the arc with the gripper command unchanged"),
        ("fold complete:", "fold arc finished: commanded angle"),
        ("deg; the gripper is still closed: open it, raise 5 cm and look whether the flap stays down",
         "deg; flap angle and closure unverified; gripper command unchanged. Observe carton displacement and tipping, then plan safe disengagement and inspect springback; no automatic release or retreat"),
    ),
    "pinch_contradiction": (
        ("The wrist camera cannot see a thin flap held edge-on between closed pads; the load says something is there. Do not conclude the gripper is empty from the image alone: verify by lifting.",
         "A thin edge can be hard to see; load and images must be assessed together. Grasp remains uncertain without corroborating evidence. Push-folding needs no pinch or lift test. Inspect fresh views and telemetry before any separately planned safe verification motion; never automatically retry contact_halt."),
    ),
    "pinch_verdict": (
        ("this is NOT a pinch: send the same close again (closing again is safe) and read the new verdict",
         "this does not establish a pinch; inspect fresh views and telemetry before choosing a new safe plan. Do not automatically retry the close or any contact_halt"),
        ("lift 8 cm and watch the phone to find out",
         "inspect fresh views and telemetry; any grasp verification needs a separate safe plan. Push-folding needs no pinch or lift test; never automatically retry contact_halt", 2),
    ),
}
SCHEMA_EDITS = (
    ("fold: turn a pinched flap about its hinge on an arc toward the box centre",
     "fold: run a geometric claw-tip arc for a planned push-fold or pinch-fold toward the box centre; gripper unchanged; no pinch/lift prerequisite or automatic contact_halt retry"),
    ("fold only: pinch height minus the hinge (box top) height, 4 to 15 cm.",
     "fold only: claw-tip height minus the hinge (box top) height for a vertical start, 4 to 15 cm; actual pad/body contact may be offset."),
    ("fold only: seconds to hold the flap at the end of the arc with the gripper closed, 0 to 30 (null = 0).",
     "fold only: seconds to hold the arc endpoint with the gripper command unchanged, 0 to 30 (null = 0); observe flap closure separately."),
)


def patch_sources(sources: dict[str, str], policy: str | None = None) -> dict[str, str]:
    policy = POLICY_PATH.read_text() if policy is None else policy
    out = dict(sources)
    source = out["chat_server.py"]
    node = assignment(ast.parse(source), "SUPERVISOR_SYSTEM").value
    if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
        raise ValueError("Expected literal SUPERVISOR_SYSTEM")
    prompt = patch_prompt(node.value, policy)
    if prompt != node.value:
        # Keep actual newlines and JSON quotes for compatibility with other installers.
        literal = '"""' + prompt.replace("\\", "\\\\").replace('"""', '\\"\\"\\"') + '"""'
        source = replace_node(source, node, literal)
    for name, edits in FOLD_EDITS.items():
        matches = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name == name]
        if len(matches) != 1:
            raise ValueError(f"Expected one fold helper {name}")
        node = matches[0]
        part = ast.get_source_segment(source, node)
        for old, new, *count in edits:
            part = replace_once(part, old, new, count[0] if count else 1)
        source = replace_node(source, node, part)
    out["chat_server.py"] = source
    source = out["model_backend.py"]
    node = assignment(ast.parse(source), "DECISION_SCHEMA")
    part = ast.get_source_segment(source, node)
    for old, new in SCHEMA_EDITS:
        part = replace_once(part, old, new)
    out["model_backend.py"] = replace_node(source, node, part)
    for name in FILES:
        compile(out[name], name, "exec")  # Parse only; no module execution.
        if text_only_ast(sources[name]) != text_only_ast(out[name]):
            raise ValueError(f"Executable AST changed in {name}; refusing text-only installation")
    return out


def atomic_write(path: Path, data: bytes, mode: int) -> None:
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.push-", dir=path.parent)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_pair(pilot: Path, before: dict, after: dict, modes: dict) -> None:
    """Validate both files before replacement; undo completed writes on an error."""
    for name in FILES:
        if (pilot / name).read_bytes() != before[name]:
            raise RuntimeError(f"{name} changed; inspect before retrying")
    written = []
    try:
        for name in FILES:
            if before[name] == after[name]:
                continue
            if (pilot / name).read_bytes() != before[name]:
                raise RuntimeError(f"{name} changed during installation")
            atomic_write(pilot / name, after[name], modes[name])
            written.append(name)
        for name in FILES:
            if (pilot / name).read_bytes() != after[name]:
                raise RuntimeError(f"{name} changed after replacement")
    except Exception:
        for name in reversed(written):
            if (pilot / name).read_bytes() == after[name]:
                atomic_write(pilot / name, before[name], modes[name])
        raise


def install(pilot: Path, dry_run: bool = False) -> dict:
    pilot = pilot.resolve()
    if any((pilot / name).is_symlink() for name in FILES):
        raise ValueError("Refusing symlinked pilot source")
    before = {name: (pilot / name).read_bytes() for name in FILES}
    patched = patch_sources({name: data.decode() for name, data in before.items()})
    after = {name: patched[name].encode() for name in FILES}
    modes = {name: (pilot / name).stat().st_mode & 0o777 for name in FILES}
    rows = {name: {"changed": before[name] != after[name], "before_sha256": sha(before[name]),
                   "after_sha256": sha(after[name]), "mode": modes[name],
                   "backup": f"{BACKUPS}/{Path(name).stem}-{sha(before[name])}.py"} for name in FILES}
    changed = any(row["changed"] for row in rows.values())
    receipt = {"format": "carton-push-policy-v1", "pilot": str(pilot), "files": rows,
               "policy_sha256": sha(POLICY_PATH.read_bytes()), "changed": changed,
               "dry_run": dry_run, "runtime_activated": False, "motor_writes": 0,
               "activation": "Parent must separately reload an idle pilot and start fresh task context."}
    if changed and not dry_run:
        directory = pilot / BACKUPS
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        for name, row in rows.items():
            backup = pilot / row["backup"]
            if backup.exists():
                if backup.read_bytes() != before[name]:
                    raise ValueError(f"Backup hash mismatch for {name}")
            else:
                atomic_write(backup, before[name], 0o600)
        token = sha(json.dumps(rows, sort_keys=True).encode())[:20]
        receipt_path = directory / f"receipt-{token}.json"
        receipt["receipt_path"] = str(receipt_path)
        # Persist rollback instructions before replacement, including crash recovery.
        atomic_write(receipt_path, (json.dumps(receipt, indent=2) + "\n").encode(), 0o600)
        write_pair(pilot, before, after, modes)
    return receipt


def rollback(pilot: Path, receipt_path: Path) -> dict:
    pilot = pilot.resolve()
    receipt = json.loads(receipt_path.read_text())
    if receipt.get("format") != "carton-push-policy-v1" or receipt.get("pilot") != str(pilot):
        raise ValueError("Receipt does not match this pilot")
    rows = receipt["files"]
    if set(rows) != set(FILES):
        raise ValueError("Unexpected rollback file set")
    before, after, modes = {}, {}, {}
    for name in FILES:
        row = rows[name]
        expected_backup = f"{BACKUPS}/{Path(name).stem}-{row['before_sha256']}.py"
        if row["backup"] != expected_backup or (pilot / name).is_symlink():
            raise ValueError(f"Invalid rollback path for {name}")
        original = (pilot / expected_backup).read_bytes()
        if sha(original) != row["before_sha256"]:
            raise ValueError(f"Corrupt backup for {name}")
        current = (pilot / name).read_bytes()
        if sha(current) not in (row["before_sha256"], row["after_sha256"]):
            raise ValueError(f"{name} drifted since installation; refusing rollback")
        compile(original, name, "exec")
        before[name], after[name], modes[name] = current, original, row["mode"]
    write_pair(pilot, before, after, modes)
    return {"rolled_back": any(before[n] != after[n] for n in FILES),
            "files": {n: sha(after[n]) for n in FILES}, "runtime_activated": False, "motor_writes": 0}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot", required=True, type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--rollback", type=Path, metavar="RECEIPT")
    args = parser.parse_args()
    result = rollback(args.pilot, args.rollback) if args.rollback else install(args.pilot, args.dry_run)
    print(json.dumps(result, indent=2))
