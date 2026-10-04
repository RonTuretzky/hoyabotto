"""Jev selects one locally constructed motion; the sole motor owner executes it.

All position values here are raw encoder ticks, NOT degrees or normalized units.
The perception/supervision process supplies fresh, action-specific clearance.
Neither a model answer nor a confidence value establishes physical clearance.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from ..adapters.base import ARM_JOINTS, HEAD_JOINTS, arm_joint
from ..llm.jev import Jev

JOINTS = tuple(arm_joint(a, j) for a in ("left", "right") for j in ARM_JOINTS) + tuple(HEAD_JOINTS)
WHEELS = ("base_left_wheel", "base_right_wheel")
MOTORS = JOINTS + WHEELS


class MotionRefused(RuntimeError):
    pass


def finite(v):
    return type(v) in (int, float) and math.isfinite(v)


@dataclass(frozen=True)
class Limits:
    step_ticks: int = 32             # 2.8125 physical degrees on a 4096-count encoder
    joint_speed: int = 64            # raw ticks / second
    wheel_speed: int = 128           # raw ticks / second
    pulse_s: float = 0.20
    move_timeout_s: float = 1.25
    observation_age_s: float = 3.0
    state_age_s: float = 0.50
    drift_ticks: int = 8
    max_temperature: float = 55.0
    max_load: float = 500.0
    min_probability: float = 0.90
    min_confidence: float = 0.80
    session_s: float = 120.0
    max_decisions: int = 40
    max_cost_usd: float = 0.05

    def __post_init__(self):
        # Hard ceilings remain in code even if callers configure a larger step.
        ranges = {"step_ticks": (1, 64), "joint_speed": (1, 128), "wheel_speed": (1, 256),
                  "pulse_s": (0.02, 0.30), "move_timeout_s": (0.1, 2.0),
                  "observation_age_s": (0.1, 5.0), "state_age_s": (0.05, 0.5),
                  "drift_ticks": (0, 8), "max_temperature": (1, 55), "max_load": (1, 500),
                  "min_probability": (0.9, 1), "min_confidence": (0.8, 1),
                  "session_s": (1, 300), "max_decisions": (1, 100), "max_cost_usd": (0.00001, 1)}
        for key, (lo, hi) in ranges.items():
            v = getattr(self, key)
            if not finite(v) or not lo <= v <= hi:
                raise ValueError(f"invalid motion limit: {key}")
        for key in ("step_ticks", "joint_speed", "wheel_speed", "drift_ticks", "max_decisions"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"motion limit must be an integer: {key}")


@dataclass(frozen=True)
class Action:
    name: str
    description: str
    joints: dict[str, int] = field(default_factory=dict)  # relative raw encoder ticks
    wheels: dict[str, int] = field(default_factory=dict)  # raw ticks / second


def action_catalog(limits: Limits) -> dict[str, Action]:
    out = {}
    for motor in JOINTS:
        for suffix, sign in (("increase", 1), ("decrease", -1)):
            name = f"{motor}_{suffix}"
            out[name] = Action(name, f"{suffix.capitalize()} {motor} by {limits.step_ticks} raw encoder ticks; "
                              "joint-space nudge, no inferred Cartesian direction.", {motor: sign * limits.step_ticks})
    # Left wheel is mirrored on XLeRobot 2 Wheels. Commission this sign mapping
    # on the actual build before the supervisor marks driving actions clear.
    speeds = {"drive_forward": (-1, 1), "drive_backward": (1, -1),
              "turn_left": (1, 1), "turn_right": (-1, -1),
              "left_wheel_forward": (-1, 0), "left_wheel_backward": (1, 0),
              "right_wheel_forward": (0, 1), "right_wheel_backward": (0, -1)}
    for name, signs in speeds.items():
        out[name] = Action(name, f"{name.replace('_', ' ')} for a {limits.pulse_s}s command window, then stop both wheels.",
                           wheels=dict(zip(WHEELS, (s * limits.wheel_speed for s in signs))))
    return out


class Actuator(Protocol):
    def snapshot(self) -> dict: ...
    def execute(self, action: Action, before: dict, limits: Limits, guard: Callable[[], None]) -> dict: ...
    def stop(self) -> None: ...


class MotionController:
    def __init__(self, actuator: Actuator, backend, goal: str, limits: Limits | None = None):
        if not isinstance(goal, str) or not goal.strip() or len(goal) > 2000:
            raise ValueError("provide a goal of 1..2000 characters")
        self.actuator, self.jev, self.goal = actuator, Jev(backend), goal
        self.limits = limits or Limits()
        self.actions = action_catalog(self.limits)
        self.stopped = threading.Event()
        self._step_lock = threading.Lock()
        self._expires = time.monotonic() + self.limits.session_s
        self._last_seq = -1
        self.decisions, self.cost_usd = 0, 0.0

    def request_stop(self):
        """May be called by the motor owner's STOP handler while Jev is waiting."""
        self.stopped.set()
        self.actuator.stop()

    def _guard(self, observation):
        if self.stopped.is_set() or time.monotonic() >= self._expires:
            raise MotionRefused("stopped or session lease expired")
        stamp = observation.get("observed_at")
        if not finite(stamp) or not 0 <= time.time() - stamp <= self.limits.observation_age_s:
            raise MotionRefused("observation expired or timestamp invalid")
        if observation.get("stop_requested") is not False or observation.get("camera_ok") is not True:
            raise MotionRefused("stop requested or camera evidence unavailable")

    def _snapshot(self):
        s = self.actuator.snapshot()
        stamp = s.get("observed_at")
        if not finite(stamp) or not 0 <= time.time() - stamp <= self.limits.state_age_s:
            raise MotionRefused("motor readback stale or invalid")
        if not isinstance(s.get("calibration_id"), str) or not s["calibration_id"]:
            raise MotionRefused("calibration identity missing")
        for key, expected in (("positions", JOINTS), ("bounds", JOINTS), ("wheel_velocities", WHEELS), ("health", MOTORS)):
            if not isinstance(s.get(key), dict) or set(s[key]) != set(expected):
                raise MotionRefused(f"incomplete motor {key}")
        for name in JOINTS:
            b, value = s["bounds"][name], s["positions"][name]
            if not isinstance(b, (tuple, list)) or len(b) != 2 or not all(finite(v) for v in b):
                raise MotionRefused("invalid calibrated range")
            if not 0 <= b[0] < b[1] <= 4095 or not finite(value) or not b[0] <= value <= b[1]:
                raise MotionRefused("joint outside calibrated range")
        for name, h in s["health"].items():
            if not isinstance(h, dict) or not all(finite(h.get(k)) for k in ("temperature", "load", "status")):
                raise MotionRefused("invalid servo health")
            if not -20 <= h["temperature"] <= self.limits.max_temperature or abs(h["load"]) > self.limits.max_load or h["status"] != 0:
                raise MotionRefused("servo health limit")
            if type(h.get("torque_enabled")) is not int or h["torque_enabled"] not in (0, 1) or type(h.get("mode")) is not int:
                raise MotionRefused("invalid servo torque/mode state")
        if any(not finite(v) or abs(v) > 8 for v in s["wheel_velocities"].values()):
            raise MotionRefused("wheels must be stopped between decisions")
        return s

    def step(self, observation: dict) -> dict:
        """Consumes one fresh observation once. Never queues or replays commands."""
        if not self._step_lock.acquire(blocking=False):
            raise MotionRefused("another decision is already in flight")
        result = {"executed": False, "choice": "unknown", "mode": "motion", "reason": ""}
        try:
            observation = copy.deepcopy(observation)
            if not isinstance(observation, dict):
                raise MotionRefused("observation must be an object")
            seq = observation.get("sequence")
            if type(seq) is not int or seq <= self._last_seq:
                raise MotionRefused("duplicate or out-of-order observation")
            self._last_seq = seq
            result["sequence"] = seq
            self._guard(observation)
            if self.decisions >= self.limits.max_decisions or self.cost_usd >= self.limits.max_cost_usd:
                raise MotionRefused("session decision or spending budget exhausted")
            before = self._snapshot()
            allowed = observation.get("allowed_actions")
            if not isinstance(allowed, list) or any(not isinstance(a, str) or a not in self.actions for a in allowed):
                raise MotionRefused("invalid locally cleared action list")
            candidates = {}
            for name in allowed:
                a = self.actions[name]
                selected = a.wheels or a.joints
                if any(before["health"][m]["torque_enabled"] != 1 or before["health"][m]["mode"] != (1 if m in WHEELS else 0) for m in selected):
                    continue
                if a.wheels and (observation.get("base_clear") is not True or observation.get("arms_stowed") is not True):
                    continue
                if any(not before["bounds"][j][0] <= before["positions"][j] + d <= before["bounds"][j][1] for j, d in a.joints.items()):
                    continue
                candidates[name] = a
            if not candidates:
                raise MotionRefused("no locally cleared motion; refresh observation or plan")
            criteria = {k: a.description for k, a in candidates.items()}
            criteria.update(hold="Remain still when the goal is satisfied or no movement is useful.",
                            stop="Stop the session on danger, a stop request or contradictory evidence.",
                            unknown="The evidence does not establish which bounded movement advances the goal.")
            packet = {"goal": self.goal, "observation": observation, "robot": before,
                      "units": "raw encoder ticks for position; raw ticks/second for wheels"}
            self.decisions += 1
            answer = self.jev.ask("Which SINGLE available bounded action should the robot execute next to advance the goal? "
                                  "Only use supplied evidence. Do not invent joint directions or claim task success. "
                                  "Stop overrides progress. If geometry or direction is unclear choose unknown.", criteria, packet)
            self.cost_usd += answer.meta.cost_usd
            result.update(choice=answer.choice, probability=answer.p, confidence=answer.confidence,
                          latency_ms=answer.meta.latency_ms, model=answer.meta.model, cost_usd=answer.meta.cost_usd)
            self._guard(observation)
            if answer.choice == "stop":
                self.request_stop()
                result["reason"] = "Jev requested stop"
                return result
            if answer.error or answer.choice not in candidates:
                result["reason"] = answer.error or "Jev selected no motion"
                return result
            if not finite(answer.p) or not finite(answer.confidence) or answer.p < self.limits.min_probability or answer.confidence < self.limits.min_confidence:
                raise MotionRefused("Jev uncertainty below motion thresholds")
            now = self._snapshot()
            selected = candidates[answer.choice].wheels or candidates[answer.choice].joints
            if any(now["health"][m]["torque_enabled"] != 1 or now["health"][m]["mode"] != (1 if m in WHEELS else 0) for m in selected):
                raise MotionRefused("selected motor configuration changed during decision")
            if before["calibration_id"] != now["calibration_id"] or before["bounds"] != now["bounds"]:
                raise MotionRefused("calibration changed during decision")
            if any(abs(now["positions"][j] - before["positions"][j]) > self.limits.drift_ticks for j in JOINTS):
                raise MotionRefused("robot moved during decision; reacquire evidence")
            self._guard(observation)
            feedback = self.actuator.execute(candidates[answer.choice], now, self.limits, lambda: self._guard(observation))
            result.update(executed=True, reason="bounded command completed; observe again before next move", feedback=feedback)
            return result
        except MotionRefused as exc:
            result["reason"] = str(exc)
            return result
        finally:
            # Stop on API errors, stale evidence, execution exceptions and normal completion.
            # Stop failures propagate: a missing acknowledgement must never look successful.
            try:
                self.actuator.stop()
            finally:
                self._step_lock.release()


class SimActuator:
    """Deterministic motor fake. No physics, camera, calibration or collision claim."""
    def __init__(self):
        self.positions = dict.fromkeys(JOINTS, 2048)
        self.velocities = dict.fromkeys(WHEELS, 0)
        self.sent = []
        self.stop_count = 0

    def snapshot(self):
        return {"observed_at": time.time(), "calibration_id": "sim-only",
                "positions": dict(self.positions), "bounds": {j: [64, 4031] for j in JOINTS},
                "wheel_velocities": dict(self.velocities),
                "health": {m: {"temperature": 30, "load": 0, "status": 0, "torque_enabled": 1,
                               "mode": 1 if m in WHEELS else 0} for m in MOTORS}}

    def execute(self, action, before, limits, guard):
        guard()
        self.sent.append(action.name)
        for j, delta in action.joints.items():
            self.positions[j] += delta
        if action.wheels:
            self.velocities.update(action.wheels)
            try:
                until = time.monotonic() + limits.pulse_s
                while time.monotonic() < until:
                    guard()
                    time.sleep(0.01)
            finally:
                self.stop()
        return {"simulated": True, "positions": dict(self.positions), "wheel_velocities": dict(self.velocities)}

    def stop(self):
        self.stop_count += 1
        self.velocities = dict.fromkeys(WHEELS, 0)
