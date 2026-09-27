"""Who may authorize a pour. Rules always run first and can only say no.

Ladder (per profile `authority.jev`, the maximum level allowed):
  shadow  - Jev is asked and recorded; a person authorizes every pour
  route   - Jev's next_review choice is honoured (reinspect / pause routes); a person still authorizes pours
  approve - Jev may authorize a routine pour when rules pass and p >= jev_approve_min_p; people are notified, not asked

The system starts at shadow and promotes itself with evidence:
  shadow -> route   after `jev_shadow_cycles` cycles where Jev's route agreed with the rules/human outcome >= 80%
  route  -> approve after another `jev_shadow_cycles` cycles with zero disagreements on pour decisions
A person can demote at any time from the viewer; a false approval (human later marks a pour as wrong) demotes to route.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any

from ..config import AuthorityCfg

log = logging.getLogger(__name__)
LEVELS = ["shadow", "route", "approve"]


@dataclass
class Verdict:
    authorized: bool
    by: str            # rules | human:<name> | jev
    reason: str
    ask_human: bool    # the cycle must still ask a person
    route: str = "routine"


class Authority:
    def __init__(self, cfg: AuthorityCfg, store):
        self.cfg = cfg
        self.store = store
        self.max_level = cfg.jev if cfg.jev in LEVELS else "shadow"
        self.level = self._load_level()

    # ---- persistence of the current level ---------------------------------------
    def _load_level(self) -> str:
        rows = self.store.query("SELECT payload_json FROM events WHERE kind='authority_level' ORDER BY t DESC LIMIT 1")
        if rows:
            lvl = json.loads(rows[0]["payload_json"]).get("level", "shadow")
            return lvl if LEVELS.index(lvl) <= LEVELS.index(self.max_level) else self.max_level
        return "shadow"

    def set_level(self, level: str, who: str, why: str) -> None:
        if level not in LEVELS:
            return
        if LEVELS.index(level) > LEVELS.index(self.max_level):
            level = self.max_level
        self.level = level
        self.store.event(None, "authority_level", {"level": level, "who": who, "why": why})
        log.info("Jev authority -> %s (%s: %s)", level, who, why)

    # ---- evidence-based promotion ----------------------------------------------
    def consider_promotion(self) -> None:
        n = self.cfg.jev_shadow_cycles
        if self.level == "shadow" and LEVELS.index(self.max_level) >= 1:
            rows = self.store.query("SELECT choice, note FROM decisions WHERE kind='jev_route' ORDER BY t DESC LIMIT ?", (n,))
            if len(rows) >= n:
                agree = sum(1 for r in rows if (r.get("note") or "").startswith("agree"))
                if agree / len(rows) >= 0.8:
                    self.set_level("route", "auto", f"{agree}/{len(rows)} route agreements")
        elif self.level == "route" and LEVELS.index(self.max_level) >= 2:
            rows = self.store.query("SELECT choice, note FROM decisions WHERE kind='jev_pour' ORDER BY t DESC LIMIT ?", (n,))
            if len(rows) >= n and all((r.get("note") or "").startswith("agree") for r in rows):
                self.set_level("approve", "auto", f"{n} pour decisions agreed with the human")

    def demote_after_false_approval(self, why: str) -> None:
        if self.level == "approve":
            self.set_level("route", "auto", f"false approval: {why}")

    # ---- the decision ---------------------------------------------------------
    def decide_pour(self, cycle_id: str, rules_ok: bool, rules_reason: str, jev_pour, jev_route) -> Verdict:
        """jev_pour / jev_route are jev.Choice or None. Records the decision; returns who authorizes."""
        if not rules_ok:
            self.store.decision(cycle_id, "rule", "pour_preconditions", "", "block", None, "farm-rules-1", honoured=True, note=rules_reason)
            return Verdict(False, "rules", rules_reason, ask_human=False, route="reinspect")
        self.store.decision(cycle_id, "rule", "pour_preconditions", "", "pass", None, "farm-rules-1", honoured=True, note=rules_reason)
        route = "routine"
        if jev_route is not None:
            honoured = self.level in ("route", "approve")
            self.store.decision(cycle_id, "jev_route", jev_route.question, "", jev_route.choice, jev_route.probabilities, "farm-jev-1",
                                jev_route.meta.model, jev_route.meta.latency_ms, jev_route.meta.cost_usd, honoured=honoured, note=("pending:" + jev_route.why)[:300])
            if honoured and jev_route.choice not in ("routine", "unknown"):
                return Verdict(False, "jev", f"route={jev_route.choice}: {jev_route.why}", ask_human=True, route=jev_route.choice)
            route = jev_route.choice
        if jev_pour is not None:
            honoured = self.level == "approve" and jev_pour.choice == "pour" and jev_pour.p >= self.cfg.jev_approve_min_p
            self.store.decision(cycle_id, "jev_pour", jev_pour.question, "", jev_pour.choice, jev_pour.probabilities, "farm-jev-1",
                                jev_pour.meta.model, jev_pour.meta.latency_ms, jev_pour.meta.cost_usd, honoured=honoured, note=("pending:" + jev_pour.why)[:300])
            if honoured:
                return Verdict(True, "jev", f"p={jev_pour.p:.2f}: {jev_pour.why}", ask_human=False, route=route)
            if self.level == "approve" and jev_pour.choice in ("skip", "reinspect"):
                return Verdict(False, "jev", f"{jev_pour.choice}: {jev_pour.why}", ask_human=(jev_pour.choice == "reinspect"), route=jev_pour.choice)
        return Verdict(False, "rules", "rules pass; a person must authorize", ask_human=True, route=route)

    def record_agreement(self, cycle_id: str, kind: str, agreed: bool, detail: str = "") -> None:
        """Rewrites the pending note on the latest Jev decision of this cycle to agree/disagree (append-only elsewhere;
        this is the one bookkeeping field that exists to make promotion auditable)."""
        rows = self.store.query("SELECT decision_id FROM decisions WHERE cycle_id=? AND kind=? ORDER BY t DESC LIMIT 1", (cycle_id, kind))
        if rows:
            self.store._conn.execute("UPDATE decisions SET note=? WHERE decision_id=?", (("agree:" if agreed else "disagree:") + detail[:280], rows[0]["decision_id"]))
