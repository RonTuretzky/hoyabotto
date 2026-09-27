"""Named joint keyframes. Written by the LLM-servo loop when it reaches a goal,
by `farm keyframe save`, or by hand. Read by every skill."""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import yaml


class KeyframeStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._d: dict[str, dict[str, Any]] = {}
        if self.path.exists():
            self._d = yaml.safe_load(self.path.read_text()) or {}

    def names(self) -> list[str]:
        return sorted(self._d)

    def get(self, name: str) -> dict[str, float] | None:
        kf = self._d.get(name)
        return dict(kf["joints"]) if kf else None

    def meta(self, name: str) -> dict[str, Any] | None:
        return self._d.get(name)

    def save(self, name: str, joints: dict[str, float], arm: str = "", note: str = "", learned_by: str = "") -> None:
        self._d[name] = {"joints": {k: float(v) for k, v in joints.items()}, "arm": arm, "note": note, "learned_by": learned_by, "t": time.time()}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(yaml.safe_dump(self._d, sort_keys=True))

    def delete(self, name: str) -> None:
        self._d.pop(name, None)
        self.path.write_text(yaml.safe_dump(self._d, sort_keys=True))
