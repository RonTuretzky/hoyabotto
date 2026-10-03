"""Typed judgement of the carton from the head camera (and a wrist view when useful).

Every field has an explicit unknown. The cycle only moves on when the field it just worked
on is True; unknown pauses and asks a person, exactly as the farm does with water."""
from __future__ import annotations

import json
from typing import Any

import numpy as np

from farm.llm.backends import LLMError
from farm.status import Reading, Status, unknown

EXTRACTOR_VERSION = "carton-judge-1"
SYSTEM = ("You inspect a cardboard shipping carton on a table from a robot's cameras. Answer ONLY with the JSON object requested. "
          "Use \"unknown\" whenever the image does not let you tell. Never guess.")
PROMPT = (
    "The carton is {L:.0f} x {W:.0f} x {H:.0f} cm with 14 cm flaps: two SHORT flaps on the short ends (left/right in the head view), "
    "two LONG flaps on the long sides (near = bottom of the head view, far = top). A flap is 'folded' when it lies flat over the box, "
    "'open' when it stands up or leans out. Return JSON: {{\"box_present\": true|false|\"unknown\", "
    "\"short_left\": \"open\"|\"folded\"|\"unknown\", \"short_right\": \"open\"|\"folded\"|\"unknown\", "
    "\"long_far\": \"open\"|\"folded\"|\"unknown\", \"long_near\": \"open\"|\"folded\"|\"unknown\", "
    "\"seam_closed\": true|false|\"unknown\" (the two long flaps meet edge to edge), "
    "\"tape_on_seam\": true|false|\"unknown\" (a tape strip crosses the seam), \"tape_pressed\": true|false|\"unknown\" (the strip lies flat, no lifted ends), "
    "\"paddle_held\": true|false|\"unknown\" (the right gripper holds a flat paddle), \"obstruction\": true|false|\"unknown\" (a hand, cable or object in the way), "
    "\"confidence\": 0-1, \"notes\": short string}}."
)
FLAPS = ("short_left", "short_right", "long_far", "long_near")
BOOLS = ("box_present", "seam_closed", "tape_on_seam", "tape_pressed", "paddle_held", "obstruction")


def _tri(v: Any) -> bool | str:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    return True if s in ("true", "yes") else False if s in ("false", "no") else "unknown"


def _flap(v: Any) -> str:
    s = str(v).strip().lower()
    return s if s in ("open", "folded") else "unknown"


def judge(backend, box, frames: list[tuple[str, Reading[np.ndarray]]], store=None, cycle_id: str | None = None) -> Reading[dict[str, Any]]:
    imgs = [(n, r.value) for n, r in frames if r.status is Status.OK and r.value is not None]
    if not imgs:
        return unknown("carton.judge", "no OK frames")
    try:
        d, meta = backend.complete_json(PROMPT.format(L=box.length * 100, W=box.width * 100, H=box.height * 100), images=imgs, system=SYSTEM)
    except LLMError as e:
        return unknown("carton.judge", f"model error: {e}")
    out: dict[str, Any] = {k: _tri(d.get(k, "unknown")) for k in BOOLS}
    out.update({k: _flap(d.get(k, "unknown")) for k in FLAPS})
    out["confidence"] = float(d.get("confidence", 0) or 0)
    out["notes"] = str(d.get("notes", ""))[:300]
    out.update(model=meta.model, latency_ms=meta.latency_ms, cost_usd=meta.cost_usd)
    if store is not None:
        store.decision(cycle_id, "carton_judge", "carton state", json.dumps([n for n, _ in imgs]), json.dumps({k: out[k] for k in FLAPS}),
                       {"confidence": out["confidence"]}, EXTRACTOR_VERSION, meta.model, meta.latency_ms, meta.cost_usd, honoured=True, note=out["notes"])
    return Reading(out, Status.OK, source="carton.judge", meta={"extractor": EXTRACTOR_VERSION, "frames": [n for n, _ in imgs]})


def satisfied(judgement: dict[str, Any], check: str) -> bool | None:
    """True / False / None(unknown) for a plan step's check field."""
    if check.endswith("_folded"):
        v = judgement.get(check[: -len("_folded")], "unknown")
        return None if v == "unknown" else v == "folded"
    v = judgement.get(check, "unknown")
    return None if v == "unknown" else bool(v)
