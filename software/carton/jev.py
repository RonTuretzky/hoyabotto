"""Motor-free carton advice and a non-blocking shadow observer.

The caller provides text observations. This module neither opens devices nor
executes commands. Suggestions never change the carton state machine's authority.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import math
import threading
import time

from farm.llm.jev import Jev

OPTIONS = {
    "continue_plan": "Continue the existing validated procedure through the local controller, which still checks every motion precondition. Do not invent targets.",
    "refresh_view": "Acquire a new useful observation because current evidence is unclear or occluded.",
    "request_plan": "The current strategy is unsuitable or repeatedly failing; request a new plan or establish missing station-specific movements.",
    "stop": "Stop the task through the controller because an obstruction or unsafe condition is indicated.",
    "unknown": "The observations do not support any of the other choices.",
}


class CartonAdvisor:
    def __init__(self, backend, max_age_s: float = 5.0, clock=time.time):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("Observation maximum age must be finite and positive")
        self.jev = Jev(backend)
        self.max_age_s, self.clock = max_age_s, clock

    def advise(self, snapshot: dict) -> dict:
        if not isinstance(snapshot, dict):
            raise ValueError("A carton observation must be a JSON object")
        observation_id = snapshot.get("observation_id")
        if not isinstance(observation_id, str) or not observation_id or len(observation_id) > 256:
            raise ValueError("A bounded observation_id is required")
        observed_at = snapshot.get("observed_at")
        if type(observed_at) not in (float, int) or not math.isfinite(observed_at):
            raise ValueError("observed_at must be the finite capture timestamp in Unix seconds")
        judgement = snapshot.get("judgement")
        if not isinstance(judgement, dict):
            raise ValueError("A structured judgement object is required")
        # Project onto a bounded evidence schema. No arbitrary command/credential fields are forwarded.
        packet = {k: snapshot[k] for k in ("observation_id", "observed_at", "stage", "step", "attempt",
                  "judgement", "missing_keyframes", "held", "stop_requested", "health_ok") if k in snapshot}
        base = {"observation_id": observation_id, "observed_at": observed_at,
                "mode": "shadow", "motion_authorized": False}

        def local(action, reason):
            return {**base, "action": action, "source": "rules", "reason": reason,
                    "probabilities": {}, "confidence": None, "latency_ms": 0.0, "cost_usd": 0.0}

        if snapshot.get("stop_requested") is True or snapshot.get("health_ok") is False or judgement.get("obstruction") is True:
            return local("stop", "STOP, a machine fault or an obstruction was reported")
        if not 0 <= self.clock() - observed_at <= self.max_age_s:
            return local("refresh_view", "Observation is stale or future-dated")
        if judgement.get("obstruction") is not False or judgement.get("box_present") is not True:
            return local("refresh_view", "Box visibility and clear workspace are not established")
        if snapshot.get("health_ok") is not True:
            return local("unknown", "Machine health is not established")
        if snapshot.get("missing_keyframes"):
            return local("request_plan", "Station-specific keyframes are missing")
        answer = self.jev.ask("Which workflow action best advances carton closing from this observation? "
                              "Use the semantic observations and recent attempt outcome. "
                              "Do not claim a flap is folded unless it was observed; fixed sequence and motor checks belong to the controller.",
                              OPTIONS, packet)
        out = {**base, "action": answer.choice, "source": "jev", "reason": answer.why,
               "probabilities": answer.probabilities, "confidence": answer.confidence,
               "model": answer.meta.model, "latency_ms": answer.meta.latency_ms,
               "cost_usd": answer.meta.cost_usd, "request_id": answer.meta.tokens.get("request_id"),
               "error": answer.error}
        if not 0 <= self.clock() - observed_at <= self.max_age_s:
            out.update(action="refresh_view", proposed_action=answer.choice, reason="Observation expired while awaiting Jev", stale=True)
        return out


class ShadowObserver:
    """At most one request in flight and zero backlog; never waits in motion code."""
    def __init__(self, advisor: CartonAdvisor, store, state: dict, over_budget):
        self.advisor, self.store, self.state, self.over_budget = advisor, store, state, over_budget
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="carton-jev-shadow")
        self._future = None
        self._lock = threading.Lock()
        self._version = 0
        self._closed = False

    def submit(self, snapshot: dict, cycle_id: str) -> bool:
        # Copy now: the robot's dictionaries may change before inference completes.
        snapshot = json.loads(json.dumps(snapshot, allow_nan=False))
        with self._lock:
            self._version += 1
            self.state.pop("jev_advice", None)
            version = self._version
            if self._closed or self.over_budget() or (self._future is not None and not self._future.done()):
                return False
            self._future = self._pool.submit(self._review, snapshot, cycle_id, version)
            return True

    def _review(self, snapshot, cycle_id, version):
        try:
            advice = self.advisor.advise(snapshot)
            with self._lock:
                superseded = self._closed or version != self._version
                advice["superseded"] = superseded
                if not superseded:
                    self.state["jev_advice"] = advice
            self.store.decision(cycle_id, "carton_jev_shadow", "next carton workflow action",
                                json.dumps(snapshot), advice["action"], advice["probabilities"], "carton-jev-1",
                                advice.get("model", "rules"), advice["latency_ms"], advice["cost_usd"],
                                honoured=False, note=advice["reason"])
            self.store.event(cycle_id, "carton_jev_shadow", advice)
        except Exception as exc:  # no advisory failure is allowed to interrupt the controller
            self.store.event(cycle_id, "carton_jev_error", {"error_type": type(exc).__name__})

    def close(self):
        with self._lock:
            self._closed = True
            self.state.pop("jev_advice", None)
            self._version += 1
        self._pool.shutdown(wait=False, cancel_futures=True)


def stream_advice(source, destination, advisor, max_cost_usd: float = 0.05) -> int:
    """One JSON observation per line, one JSON recommendation per line. No devices.

The cap prevents dispatch after cumulative *reported* usage reaches the cap;
one in-flight request may overshoot it. Provider key limits remain authoritative.
"""
    if not math.isfinite(max_cost_usd) or max_cost_usd < 0:
        raise ValueError("Budget must be finite and nonnegative")
    spent, errors = 0.0, 0
    while True:
        line = source.readline(64_001)
        if not line:
            break
        if len(line) > 64_000:
            destination.write(json.dumps({"error": "observation_line_too_large", "motion_authorized": False}) + "\n")
            destination.flush()
            return 1
        if not line.strip():
            continue
        try:
            if spent >= max_cost_usd:
                raise ValueError("Session inference budget reached")
            advice = advisor.advise(json.loads(line))
            spent += advice["cost_usd"]
            errors += int(bool(advice.get("error")))
            advice["session_cost_usd"] = spent
            destination.write(json.dumps(advice, allow_nan=False) + "\n")
        except (ValueError, TypeError, KeyError):
            errors += 1
            destination.write(json.dumps({"error": "invalid_observation_or_budget_exhausted", "motion_authorized": False}) + "\n")
        destination.flush()
    return int(errors > 0)
