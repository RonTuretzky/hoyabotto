"""Carton-specific Jev/Astra handoff and execution contract.

The motor-owning session registers validated primitives as ordinary callbacks.
An Astra proposal cannot register/validate its own trajectory. Vision supplies
observed facts; Jev only resolves which eligible primitive is useful next.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

from farm.llm.jev import Jev, question

ROUTES = {
    "execute": "Execute ONE available, locally validated primitive whose preconditions match the fresh observation.",
    "refresh_view": "Acquire a fresh useful camera observation; geometry cannot currently be seen clearly.",
    "request_plan": "Ask Astra to establish missing geometry or motion primitives, or change a repeatedly failing strategy.",
    "verify_result": "Inspect the result of the last motion before moving again or claiming progress.",
    "finish": "The requested folding (and taping ONLY if requested) is visibly verified complete.",
    "stop": "Stop on a stop request, obstruction, lost control or motor fault.",
    "unknown": "Insufficient or contradictory evidence for any other action.",
}
INSTRUCTIONS = (
    "Choose the next step for this specific carton task. The right arm holds a 15 cm paddle; "
    "the left can stabilize/fold flaps and handle tape. The head observes. Wheels reposition "
    "only before contact with arms stowed and invalidate station registration. "
    "Never infer grasp or folded flaps from commands sent. Stop overrides progress; "
    "stale/occluded evidence requires refresh_view; completed but unverified motion requires verify_result; "
    "missing geometry or primitives, pose drift, or repeated failure requires request_plan. "
    "Use execute only for available validated matching primitives. Finish must match the goal scope."
)


def routing_questions(candidate_descriptions: dict[str, str]) -> dict:
    questions = {"route": question(INSTRUCTIONS, ROUTES)}
    if candidate_descriptions:
        questions["primitive"] = question(
            "If a motion should execute, which ONE available primitive best advances the current carton stage? "
            "Use unknown if none is supported. This answer is ignored unless route is execute.",
            {**candidate_descriptions, "unknown": "No available primitive is supported by the evidence."})
    return questions


@dataclass(frozen=True)
class Primitive:
    name: str
    description: str
    station_revision: str
    calibration_id: str
    stage: str
    execute: Callable[[Callable[[], None]], dict]
    eligible: Callable[[dict], bool]
    # This object is registered by local commissioning code, never deserialized
    # from a model reply. Driving is a station-setup primitive, not a flap nudge.
    drives: bool = False


class CartonSupervisor:
    def __init__(self, backend, primitives: list[Primitive], stop: Callable[[], None],
                 goal: str = "fold_only", max_age_s: float = 3.0, max_decisions: int = 40,
                 max_cost_usd: float = 0.05):
        if goal not in ("fold_only", "fold_and_tape"):
            raise ValueError("goal must be fold_only or fold_and_tape")
        if not math.isfinite(max_age_s) or not 0 < max_age_s <= 5:
            raise ValueError("invalid evidence age")
        if type(max_decisions) is not int or not 1 <= max_decisions <= 100:
            raise ValueError("invalid decision budget")
        if not math.isfinite(max_cost_usd) or not 0 < max_cost_usd <= 1:
            raise ValueError("invalid cost budget")
        self.jev, self.stop, self.goal, self.max_age_s = Jev(backend), stop, goal, max_age_s
        self.primitives = {p.name: p for p in primitives}
        if len(self.primitives) != len(primitives) or "unknown" in self.primitives:
            raise ValueError("duplicate or reserved primitive name")
        self.pending = None
        self.last_sequence = -1
        self._motion_finished_at = 0.0
        self._busy = threading.Lock()
        self.stopped = threading.Event()
        self.decisions, self.cost_usd = 0, 0.0
        self.max_decisions, self.max_cost_usd = max_decisions, max_cost_usd
        self.registration_invalid = False
        self.replan_required = False

    def request_stop(self):
        self.stopped.set()
        self.stop()

    def _gate(self, s):
        if self.stopped.is_set() or s.get("stop_requested") is True or s.get("health_ok") is False or s.get("obstruction") is True:
            return "stop", "stop, fault or obstruction"
        stamp = s.get("observed_at")
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= time.time() - stamp <= self.max_age_s:
            return "refresh_view", "stale or invalid capture timestamp"
        if s.get("camera_ok") is not True or s.get("box_visible") is not True or s.get("obstruction") is not False:
            return "refresh_view", "visibility or clearance is unknown"
        if s.get("health_ok") is not True or s.get("stop_requested") is not False:
            return "unknown", "health or stop state is unknown"
        if self.registration_invalid:
            return "request_plan", "base moved; create a new registered supervisor before any arm motion"
        if self.replan_required:
            return "request_plan", "failed primitive needs a newly commissioned plan"
        if s.get("motion_pending_verification") is True:
            return "verify_result", "last command lacks post-motion verification"
        if self.pending:
            verification = s.get("verification", {})
            if (verification.get("primitive") != self.pending or verification.get("result") not in ("success", "failure")
                    or s["observed_at"] <= self._motion_finished_at):
                return "verify_result", "need a new observation verifying the last primitive"
            self.pending = None
            if verification["result"] == "failure":
                self.replan_required = True
                return "request_plan", "primitive failed; no blind retry"
        folded = s.get("flaps_folded")
        if (isinstance(folded, dict) and set(folded) == {"short_1", "short_2", "long_1", "long_2"}
                and all(v is True for v in folded.values())
                and (self.goal == "fold_only" or s.get("tape_verified") is True)):
            return "finish", "all requested outcomes observed"
        failures = s.get("consecutive_failures", 0)
        if type(failures) is not int or failures < 0:
            return "unknown", "invalid failure history"
        if failures >= 2 or s.get("geometry_known") is not True:
            return "request_plan", "geometry missing or repeated lack of progress"
        return None

    def step(self, snapshot: dict) -> dict:
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("carton decision already in flight")
        result = {"executed": False, "route": "unknown", "source": "rules"}
        try:
            s = copy.deepcopy(snapshot)
            if not isinstance(s, dict) or type(s.get("sequence")) is not int or s["sequence"] <= self.last_sequence:
                result["reason"] = "duplicate, out-of-order or invalid observation"
                return result
            self.last_sequence = s["sequence"]
            result["sequence"] = s["sequence"]
            gate = self._gate(s)
            if gate:
                result.update(route=gate[0], reason=gate[1])
                if gate[0] in ("stop", "finish"):
                    self.stopped.set()
                return result
            eligible = {name: p for name, p in self.primitives.items()
                        if p.station_revision == s.get("station_revision") and p.calibration_id == s.get("calibration_id")
                        and p.stage == s.get("stage") and p.eligible(s) is True
                        and (not p.drives or (s.get("stage") == "position_base" and s.get("arms_stowed") is True
                                             and s.get("base_clear") is True and s.get("in_contact") is False))}
            if not eligible:
                result.update(route="request_plan", reason="no commissioned primitive matches this station and stage")
                return result
            if self.decisions >= self.max_decisions or self.cost_usd >= self.max_cost_usd:
                result.update(route="stop", reason="carton inference budget exhausted")
                self.stopped.set()
                return result
            self.decisions += 1
            answers = self.jev.ask_many(routing_questions({n: p.description for n, p in eligible.items()}),
                                       {"goal": self.goal, "carton": s})
            route, primitive = answers["route"], answers["primitive"]
            self.cost_usd += route.meta.cost_usd + primitive.meta.cost_usd
            result.update(source="jev", route=route.choice, primitive=primitive.choice,
                          latency_ms=route.meta.latency_ms, model=route.meta.model,
                          cost_usd=route.meta.cost_usd, confidence=route.confidence, probability=route.p)
            gate = self._gate(s)
            if gate:
                result.update(route=gate[0], reason=gate[1])
                return result
            if route.choice == "stop":
                self.stopped.set()
            if route.choice != "execute":
                # Jev's finish suggestion is never completion evidence.
                if route.choice == "finish":
                    result.update(route="verify_result", reason="model completion claim requires observed outcomes")
                return result
            scores = (route.p, primitive.p, route.confidence, primitive.confidence)
            if (route.error or primitive.error or not all(math.isfinite(v) for v in scores)
                    or min(route.p, primitive.p) < 0.9 or min(route.confidence, primitive.confidence) < 0.8 or primitive.choice not in eligible):
                result.update(route="request_plan", reason="uncertain or unsupported motion selection")
                return result
            p = eligible[primitive.choice]

            def guard():
                # The primitive additionally reads health, pose and local clearance
                # inside the motor owner immediately before and during every move.
                if self.stopped.is_set() or not 0 <= time.time() - s["observed_at"] <= self.max_age_s:
                    raise RuntimeError("carton observation expired or stop requested")

            guard()
            self.pending = p.name  # even a partial/failed execution must be inspected
            try:
                feedback = p.execute(guard)
            finally:
                self._motion_finished_at = time.time()
                if p.drives:
                    self.registration_invalid = True
            result.update(executed=True, feedback=feedback, reason="inspect a fresh post-motion observation next")
            return result
        finally:
            try:
                self.stop()
            finally:
                self._busy.release()
