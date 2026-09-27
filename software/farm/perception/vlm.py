"""Vision-model perception with typed outputs. Every field has an unknown value,
and the cycle treats unknown as 'stop and ask'."""
from __future__ import annotations

import json
import logging
from typing import Any

import numpy as np

from ..llm.backends import LLMError
from ..status import Reading, Status, unknown

log = logging.getLogger(__name__)
EXTRACTOR_VERSION = "vlm-1"

SYSTEM = (
    "You inspect camera frames from a small robot that cares for paper-grown cress in shallow trays. "
    "Answer only what the images show. If a region is not visible, dark, blurred or occluded, answer 'unknown'. "
    "Never infer 'clear' or 'wet' for something you cannot see."
)

JUDGE_PROMPT = """Judge tray {tray} ({kind}) from these frames. Return JSON exactly:
{{
 "tray_present": true|false|"unknown",
 "opening_visible": true|false|"unknown",     // the refill opening / water reserve region is visible and unobstructed
 "water_visible": true|false|"unknown",       // free water visible in the reserve
 "paper_edge": "dry"|"damp"|"wet"|"unknown",  // condition of an unseeded paper edge
 "spill": "CLEAR"|"SPILL"|"UNKNOWN",          // water outside the tray; UNKNOWN if the rim area is not fully visible
 "obstruction": true|false|"unknown",         // anything (cable, tool, hand) between the gripper path and the opening
 "growth": "none"|"germinating"|"seedlings"|"canopy"|"unknown",
 "confidence": 0-1,
 "notes": "one sentence"
}}"""

VIEW_CHECK_PROMPT = """These frames are labelled with the camera name the software believes they come from. For each label say what the image actually looks like.
Return JSON: {{"views": {{"<label>": "head"|"left_wrist"|"right_wrist"|"unknown"}}, "why": "one sentence"}}.
A wrist camera looks along a gripper (gripper fingers visible at the bottom edge); the head camera shows the table and both arms from above/front."""


def _tri(v: Any) -> bool | str:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in ("true", "yes"):
        return True
    if s in ("false", "no"):
        return False
    return "unknown"


class VLMPerception:
    def __init__(self, backend, store=None, cycle_id: str | None = None):
        self.backend = backend
        self.store = store
        self.cycle_id = cycle_id

    def judge_tray(self, tray_id: str, kind: str, frames: list[tuple[str, Reading[np.ndarray]]]) -> Reading[dict[str, Any]]:
        imgs = [(n, r.value) for n, r in frames if r.status is Status.OK and r.value is not None]
        if not imgs:
            return unknown("perception.vlm", "no OK frames")
        try:
            d, meta = self.backend.complete_json(JUDGE_PROMPT.format(tray=tray_id, kind=kind), images=imgs, system=SYSTEM)
        except LLMError as e:
            return unknown("perception.vlm", f"model error: {e}")
        out = {
            "tray_present": _tri(d.get("tray_present", "unknown")),
            "opening_visible": _tri(d.get("opening_visible", "unknown")),
            "water_visible": _tri(d.get("water_visible", "unknown")),
            "paper_edge": str(d.get("paper_edge", "unknown")).lower() if str(d.get("paper_edge", "")).lower() in ("dry", "damp", "wet") else "unknown",
            "spill": str(d.get("spill", "UNKNOWN")).upper() if str(d.get("spill", "")).upper() in ("CLEAR", "SPILL") else "UNKNOWN",
            "obstruction": _tri(d.get("obstruction", "unknown")),
            "growth": str(d.get("growth", "unknown")).lower() if str(d.get("growth", "")).lower() in ("none", "germinating", "seedlings", "canopy") else "unknown",
            "confidence": float(d.get("confidence", 0) or 0),
            "notes": str(d.get("notes", ""))[:300],
            "model": meta.model, "latency_ms": meta.latency_ms, "cost_usd": meta.cost_usd,
        }
        if self.store is not None:
            self.store.decision(self.cycle_id, "vlm_judge", f"tray {tray_id}", json.dumps([n for n, _ in imgs]), out["spill"], {"confidence": out["confidence"]},
                                EXTRACTOR_VERSION, meta.model, meta.latency_ms, meta.cost_usd, honoured=True, note=out["notes"])
        return Reading(out, Status.OK, source="perception.vlm", meta={"extractor": EXTRACTOR_VERSION, "frames": [n for n, _ in imgs]})

    def check_views(self, frames: list[tuple[str, np.ndarray]]) -> Reading[dict[str, str]]:
        try:
            d, meta = self.backend.complete_json(VIEW_CHECK_PROMPT, images=frames, system=SYSTEM)
        except LLMError as e:
            return unknown("perception.views", f"model error: {e}")
        views = {str(k): str(v).lower() for k, v in (d.get("views") or {}).items()}
        mismatched = [k for k, v in views.items() if v not in ("unknown", k)]
        return Reading(views, Status.OK if not mismatched else Status.INVALID, source="perception.views",
                       note=("camera identity mismatch: " + ", ".join(f"{k} looks like {views[k]}" for k in mismatched)) if mismatched else "",
                       meta={"why": d.get("why", ""), "model": meta.model, "cost_usd": meta.cost_usd})
