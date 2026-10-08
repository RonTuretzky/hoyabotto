"""Local safety rules. Code, not config; no model output can relax them.

- joint targets are clamped to the calibrated normalized range and to a max step per tick
- a servo load ceiling stops motion
- a stale joint reading (watchdog) stops motion
- pour parameters are bounded
"""
from __future__ import annotations

from dataclasses import dataclass

from ..config import LimitsCfg
from ..status import Reading, Status


class SafetyStop(RuntimeError):
    """Raised by the skill runner when a rule trips. The caller must stop the robot."""


@dataclass
class HealthVerdict:
    ok: bool
    reason: str = ""


def joint_bounds(joint: str) -> tuple[float, float]:
    return (0.0, 100.0) if joint.endswith("gripper") else (-100.0, 100.0)


def clamp_targets(current: dict[str, float], targets: dict[str, float], step_max: float) -> dict[str, float]:
    out = {}
    for j, v in targets.items():
        lo, hi = joint_bounds(j)
        v = max(lo, min(hi, float(v)))
        if j in current:
            c = current[j]
            v = c + max(-step_max, min(step_max, v - c))
        out[j] = v
    return out


def check_health(h: Reading, limits: LimitsCfg) -> HealthVerdict:
    if h.status is Status.NOT_APPLICABLE:
        return HealthVerdict(True, "health not reported")
    if h.status is not Status.OK or not h.value:
        return HealthVerdict(False, f"health {h.status.value}: {h.note}")
    for m, d in h.value.items():
        if abs(d.get("load", 0)) >= limits.servo_load_max:
            return HealthVerdict(False, f"{m} load {d['load']:.0f} >= {limits.servo_load_max}")
    return HealthVerdict(True)


def check_watchdog(j: Reading, limits: LimitsCfg) -> HealthVerdict:
    if j.status is not Status.OK:
        return HealthVerdict(False, f"joints {j.status.value}: {j.note}")
    if j.age() > limits.watchdog_s:
        return HealthVerdict(False, f"joint reading {j.age():.2f}s old > watchdog {limits.watchdog_s}s")
    return HealthVerdict(True)


def bound_pour(tilt_deg: float, seconds: float, limits: LimitsCfg) -> tuple[float, float]:
    return (max(0.0, min(limits.pour_tilt_max_deg, float(tilt_deg))), max(0.0, min(limits.pour_s_max, float(seconds))))
