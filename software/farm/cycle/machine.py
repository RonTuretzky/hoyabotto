"""The care-cycle state machine: states, allowed transitions, deadlines.

The runner drives it; this module only knows what is legal. Any state may
enter PAUSED. Only a person (or a reconciled UNKNOWN) leaves PAUSED.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from ..config import DeadlinesCfg


class S(str, Enum):
    IDLE = "IDLE"
    IDENTIFY = "IDENTIFY"
    INSPECT = "INSPECT"
    MEASURE_LIGHT = "MEASURE_LIGHT"
    DECIDE = "DECIDE"           # rules + Jev + (maybe) a person
    PICK_BOTTLE = "PICK_BOTTLE"
    APPROACH = "APPROACH"
    POUR = "POUR"
    RETURN_UPRIGHT = "RETURN_UPRIGHT"
    VERIFY = "VERIFY"
    PARK = "PARK"
    PAUSED = "PAUSED"
    DONE = "DONE"


TRANSITIONS: dict[S, set[S]] = {
    S.IDLE: {S.IDENTIFY, S.PAUSED},
    S.IDENTIFY: {S.INSPECT, S.PAUSED},
    S.INSPECT: {S.MEASURE_LIGHT, S.DECIDE, S.PAUSED},
    S.MEASURE_LIGHT: {S.DECIDE, S.PAUSED},
    S.DECIDE: {S.PICK_BOTTLE, S.PARK, S.INSPECT, S.PAUSED},
    S.PICK_BOTTLE: {S.APPROACH, S.PAUSED},
    S.APPROACH: {S.POUR, S.PAUSED},
    S.POUR: {S.RETURN_UPRIGHT, S.PAUSED},
    S.RETURN_UPRIGHT: {S.VERIFY, S.PAUSED},
    S.VERIFY: {S.PARK, S.PAUSED},
    S.PARK: {S.DONE, S.PAUSED},
    S.PAUSED: {S.INSPECT, S.PARK, S.DONE, S.VERIFY},
    S.DONE: set(),
}

DEADLINE_FIELD = {
    S.IDENTIFY: "identify_s", S.INSPECT: "inspect_s", S.MEASURE_LIGHT: "measure_s", S.DECIDE: "ask_s",
    S.PICK_BOTTLE: "pick_s", S.APPROACH: "approach_s", S.POUR: "pour_s", S.RETURN_UPRIGHT: "pour_s",
    S.VERIFY: "verify_s", S.PARK: "park_s",
}


class IllegalTransition(RuntimeError):
    pass


@dataclass
class Machine:
    deadlines: DeadlinesCfg
    state: S = S.IDLE
    entered_t: float = field(default_factory=time.time)
    history: list[dict[str, Any]] = field(default_factory=list)
    pause_reason: str = ""
    pause_evidence: dict[str, Any] = field(default_factory=dict)

    def go(self, new: S, why: str = "") -> None:
        if new not in TRANSITIONS[self.state]:
            raise IllegalTransition(f"{self.state.value} -> {new.value}")
        self.history.append({"from": self.state.value, "to": new.value, "t": time.time(), "why": why})
        self.state = new
        self.entered_t = time.time()

    def pause(self, reason: str, evidence: dict[str, Any] | None = None) -> None:
        self.pause_reason = reason
        self.pause_evidence = evidence or {}
        self.go(S.PAUSED, reason)

    def deadline_s(self) -> float | None:
        f = DEADLINE_FIELD.get(self.state)
        return getattr(self.deadlines, f) if f else None

    def overdue(self) -> bool:
        d = self.deadline_s()
        return d is not None and (time.time() - self.entered_t) > d

    def elapsed(self) -> float:
        return time.time() - self.entered_t
