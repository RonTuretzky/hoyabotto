"""Add the tag adapter to an existing Gemma pilot without replacing its code.

Run using the pilot's Python, after installing requirements-carton-vision.txt.
Writes one .pth path and two exact source hooks, with a content-addressed backup.
Does not start/restart a server, read TLS keys or contact any device.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
from pathlib import Path
import site
import sys

IMPORT = "from farm.perception.gemma_tags import TagRobot\n"
ANCHOR = "from gemma_gateway import MODEL,validated_body,strict_json\n"
BEFORE = "chat=Chat(Robot(args.config))"
AFTER = "chat=Chat(TagRobot(Robot(args.config)))"


def patch_source(source):
    if IMPORT in source and AFTER in source:
        return source
    if IMPORT in source or AFTER in source or source.count(ANCHOR) != 1 or source.count(BEFORE) != 1:
        raise ValueError("Gemma chat layout changed; inspect before applying the two adapter hooks")
    updated = source.replace(ANCHOR, ANCHOR + IMPORT).replace(BEFORE, AFTER)
    ast.parse(updated)
    return updated


def install(pilot, software, site_dir):
    # Check dependencies and the adapter before touching the existing pilot.
    sys.path.insert(0, str(software))
    from farm.perception.tags import _get_detector
    _get_detector()
    from farm.perception.gemma_tags import TagRobot  # noqa: F401

    target = pilot / "chat_server.py"
    original = target.read_text()
    updated = patch_source(original)
    digest = hashlib.sha256(original.encode()).hexdigest()
    if updated != original:
        backup_dir = pilot / ".private" / "tag-adapter-backups"
        backup_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        backup = backup_dir / f"chat_server-{digest[:16]}.py"
        if not backup.exists():
            backup.write_text(original)
            backup.chmod(0o600)
        if target.read_text() != original:
            raise RuntimeError("Gemma chat changed during installation; retry against the current file")
        staged = target.with_name("chat_server.apriltags.tmp")
        staged.write_text(updated)
        staged.chmod(target.stat().st_mode & 0o777)
        staged.replace(target)
    path_file = site_dir / "xlerobot_apriltags.pth"
    path_file.write_text(str(software) + "\n")
    return {"pilot": str(pilot), "software": str(software), "changed": updated != original,
            "source_sha256": hashlib.sha256(updated.encode()).hexdigest(), "restart_required": True}


def main():
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot", type=Path, required=True)
    args = parser.parse_args()
    software = Path(__file__).resolve().parents[1]
    if sys.prefix == sys.base_prefix:
        parser.error("Use the Gemma pilot's virtual-environment Python")
    print(json.dumps(install(args.pilot.resolve(), software, Path(site.getsitepackages()[0])), indent=2))


if __name__ == "__main__":
    main()
