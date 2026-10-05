"""Adapter interfaces. Real devices and simulator fakes both implement these.

Every read returns a Reading with a Status; the orchestrator never touches a
device object directly. Joint names follow XLerobot2Wheels exactly, e.g.
"right_arm_wrist_flex". Positions are LeRobot-normalized (-100..100, gripper 0..100).
"""
from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import numpy as np

from ..status import Reading

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
HEAD_JOINTS = ["head_motor_1", "head_motor_2"]


def arm_joint(arm: str, j: str) -> str:
    return f"{arm}_arm_{j}"


@runtime_checkable
class RobotAdapter(Protocol):
    name: str

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def joints(self) -> Reading[dict[str, float]]:
        """All joint positions keyed by motor name (no '.pos' suffix)."""
    def health(self) -> Reading[dict[str, dict[str, float]]]:
        """Per-motor {load} when the bus supports it."""
    def move_to(self, targets: dict[str, float], max_step: float | None = None) -> Reading[dict[str, float]]:
        """Command goal positions (clamped by the robot's max_relative_target). Returns what was sent."""
    def stop(self) -> None:
        """Halt: rewrite present positions as goals, zero wheel velocity."""
    def torque_off(self, motors: list[str] | None = None) -> None: ...
    def torque_on(self, motors: list[str] | None = None) -> None: ...


@runtime_checkable
class CameraAdapter(Protocol):
    name: str

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def frame(self) -> Reading[np.ndarray]:
        """Latest RGB frame (H, W, 3) with capture time; STALE if older than the profile's max_age."""


@runtime_checkable
class LightAdapter(Protocol):
    name: str

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def latest(self) -> Reading[float]:
        """Most recent lux sample; STALE on serial silence / sequence gap; INVALID on sensor error."""
    def measure(self, samples: int, settle_s: float) -> Reading[dict[str, Any]]:
        """Collect `samples` distinct conversions after `settle_s`; returns median/spread/saturation."""


@runtime_checkable
class HumanAdapter(Protocol):
    name: str

    def ask(self, question_id: str, text: str, options: list[str], evidence: dict[str, Any], timeout_s: float) -> Reading[dict[str, Any]]:
        """Block up to timeout_s for a named person's answer; STALE on timeout (never assume)."""
    def notify(self, text: str, evidence: dict[str, Any] | None = None) -> None: ...
