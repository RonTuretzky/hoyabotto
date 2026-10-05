"""Fetch and verify the unchanged, pinned upstream SO-101 model and its license."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from urllib.request import urlopen
from uuid import uuid4


def manifest():
    return json.loads(Path(__file__).with_name("so101-assets.json").read_text())


def verified_model(folder):
    folder = Path(folder).resolve()
    m = manifest()
    for item in m["files"]:
        path = folder / item["path"]
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"Missing/changed upstream model asset: {path}; run model-fetch into a new directory")
    return folder / m["urdf"], m


def fetch_model(folder):
    """Explicit network operation. Preserve any preexisting nonmatching files."""
    folder = Path(folder).resolve()
    m = manifest()
    for item in m["files"]:
        path = folder / item["path"]
        if path.exists():
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise ValueError(f"Preserving conflicting local model asset: {path}")
            continue
        url = f'https://raw.githubusercontent.com/{m["repository"]}/{m["revision"]}/{item["source_path"]}'
        with urlopen(url, timeout=30) as reply:
            data = reply.read(item["bytes"] + 1)
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError(f"Upstream model checksum mismatch: {item['path']}")
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
        try:
            temp.write_bytes(data)
            # Exclusive publication: never replace a file created concurrently.
            os.link(temp, path)
        finally:
            temp.unlink(missing_ok=True)
    urdf, _ = verified_model(folder)
    return {"status": "UPSTREAM_MODEL_VERIFIED", "urdf": str(urdf),
            "repository": m["repository"], "revision": m["revision"], "files": len(m["files"]),
            "motor_writes": 0, "physical_calibration_verified": False}
