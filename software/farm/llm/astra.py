"""Astra: reads the last day of evidence and proposes ONE change with evidence,
expected benefit and a rollback condition. Authority levels:

  shadow      - proposals are recorded only
  propose     - proposals are recorded and shown in the viewer for a person to accept
  apply-safe  - numeric proposals inside SAFE_KEYS bounds are applied to data/overrides.yaml
                automatically and recorded; anything else still waits for a person
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any

import yaml

from .backends import LLMError

log = logging.getLogger(__name__)

# key -> (min, max) relative to the current value; only these may be auto-applied
SAFE_KEYS: dict[str, tuple[float, float]] = {
    "deadlines.identify_s": (0.5, 2.0), "deadlines.inspect_s": (0.5, 2.0), "deadlines.measure_s": (0.5, 2.0), "deadlines.verify_s": (0.5, 2.0),
    "light.samples": (0.5, 2.0), "light.settle_s": (0.5, 3.0),
    "authority.jev_approve_min_p": (1.0, 1.15),   # may only get stricter
}

SYSTEM = (
    "You are Astra, an offline reviewer for a small robot plant-care station. You read a summary of the last day of "
    "recorded evidence (cycles, pauses, unknown actions, human interventions, model disagreements, costs) and propose "
    "exactly ONE specific change. Prefer changes that reduce human minutes or false pauses without loosening safety. "
    "You never write motor commands and you cannot loosen safety limits. Respond with JSON: "
    "{\"title\": str, \"change\": {\"key\": str|null, \"from\": any, \"to\": any, \"description\": str}, "
    "\"evidence\": str, \"expected\": str, \"rollback\": str, \"priority\": \"low\"|\"medium\"|\"high\"}. "
    "Use key=null for non-numeric proposals (a procedure, a print, a test)."
)


def summarize_day(store) -> dict[str, Any]:
    since = time.time() - 86400
    q = store.query
    cycles = q("SELECT result, COUNT(*) n FROM cycles WHERE started>? GROUP BY result", (since,))
    pauses = q("SELECT payload_json FROM events WHERE kind='paused' AND t>? ORDER BY t DESC LIMIT 30", (since,))
    unknown_actions = q("SELECT skill, note FROM actions WHERE result='UNKNOWN' AND intent_t>?", (since,))
    interventions = q("SELECT who, checked, done, minutes FROM interventions WHERE t>?", (since,))
    jev = q("SELECT question, choice, honoured, probabilities_json FROM decisions WHERE kind LIKE 'jev%' AND t>? ORDER BY t DESC LIMIT 60", (since,))
    rules = q("SELECT question, choice FROM decisions WHERE kind='rule' AND t>? ORDER BY t DESC LIMIT 60", (since,))
    cost = q("SELECT COALESCE(SUM(cost_usd),0) c, COUNT(*) n FROM decisions WHERE t>?", (since,))
    return {
        "window_h": 24,
        "cycles_by_result": {r["result"]: r["n"] for r in cycles},
        "pause_reasons": [json.loads(p["payload_json"]).get("reason") for p in pauses],
        "unknown_actions": unknown_actions,
        "interventions": interventions,
        "human_minutes": sum(float(i.get("minutes") or 0) for i in interventions),
        "jev_recent": jev[:30],
        "rule_recent": rules[:30],
        "llm_cost_usd": float(cost[0]["c"]) if cost else 0.0,
        "llm_calls": int(cost[0]["n"]) if cost else 0,
    }


class Astra:
    def __init__(self, backend, store, authority: str = "propose", overrides_path: Path | None = None):
        self.backend = backend
        self.store = store
        self.authority = authority
        self.overrides_path = overrides_path

    def review(self, profile_raw: dict[str, Any]) -> dict[str, Any] | None:
        summary = summarize_day(self.store)
        prompt = "Evidence summary (JSON):\n" + json.dumps(summary, default=str, indent=1) + "\n\nCurrent numeric settings (JSON):\n" + json.dumps(
            {"deadlines": profile_raw.get("deadlines"), "light": profile_raw.get("light"), "authority": profile_raw.get("authority")}, default=str)
        try:
            d, meta = self.backend.complete_json(prompt, system=SYSTEM)
        except LLMError as e:
            log.warning("astra failed: %s", e)
            self.store.event(None, "astra_failed", {"error": str(e)})
            return None
        change = d.get("change") or {}
        pid = self.store.proposal("astra", str(d.get("title", ""))[:200], change, "summary:24h", str(d.get("expected", ""))[:500], str(d.get("rollback", ""))[:500])
        self.store.decision(None, "astra_review", "daily", "summary:24h", str(d.get("priority", "")), None, "farm-astra-1", meta.model, meta.latency_ms, meta.cost_usd, honoured=False, note=str(d.get("title", ""))[:200])
        applied = False
        if self.authority == "apply-safe":
            applied = self._apply_safe(pid, change, profile_raw)
        return {"proposal_id": pid, "applied": applied, **d}

    def _apply_safe(self, pid: str, change: dict[str, Any], profile_raw: dict[str, Any]) -> bool:
        key = change.get("key")
        if not key or key not in SAFE_KEYS or self.overrides_path is None:
            return False
        try:
            to = float(change.get("to"))
        except (TypeError, ValueError):
            return False
        sect, name = key.split(".", 1)
        cur = (profile_raw.get(sect) or {}).get(name)
        if cur is None:
            return False
        lo, hi = SAFE_KEYS[key]
        if not (float(cur) * lo <= to <= float(cur) * hi):
            self.store.event(None, "astra_rejected", {"proposal_id": pid, "key": key, "to": to, "reason": "outside safe bounds"})
            return False
        ov = yaml.safe_load(self.overrides_path.read_text()) if self.overrides_path.exists() else {}
        ov.setdefault(sect, {})[name] = to
        self.overrides_path.write_text(yaml.safe_dump(ov, sort_keys=True))
        self.store.decide_proposal(pid, "APPLIED", "astra:apply-safe")
        self.store.event(None, "astra_applied", {"proposal_id": pid, "key": key, "from": cur, "to": to})
        return True
