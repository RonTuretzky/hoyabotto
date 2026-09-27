"""LLM-driven visual servoing: the model teaches the robot instead of a person.

Loop: capture wrist + head frames and the arm's Cartesian estimate -> ask the
vision model for ONE bounded step toward a stated goal -> apply it through the
IK under the safety rules -> repeat until the model says done (verified from
its own view) or the step budget is spent. A successful goal is saved as a
named keyframe so the skill runner can replay it without the model next time.

The model never sees raw joint commands; it emits small deltas in millimetres
and degrees, clamped here, and may only open/close the gripper of the arm it
is driving. A person never drives the arm.
"""
from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any

from ..adapters.base import CameraAdapter, arm_joint
from ..llm.backends import LLMError
from ..safety.rules import SafetyStop
from ..status import Status
from .arm import ArmPose
from .runner import SkillRunner

log = logging.getLogger(__name__)

SYSTEM = (
    "You drive one arm of a small two-arm robot (SO-101 arm, parallel gripper, wrist camera) toward a stated goal. "
    "You see the wrist camera (mounted on the moving hand) and the head camera (overview). Coordinates: +x is forward from the "
    "shoulder, +y is up, pitch tilts the tool nose down/up, pan rotates the arm about the base, roll rotates the wrist. "
    "Each turn you return ONE small step as JSON: {\"action\": \"move\"|\"grip\"|\"release\"|\"done\"|\"abort\", "
    "\"dx_mm\": number, \"dy_mm\": number, \"dpitch_deg\": number, \"dpan_deg\": number, \"droll_deg\": number, "
    "\"gripper\": 0-100 or null, \"confidence\": 0-1, \"why\": string}. Steps are clamped to +-15 mm and +-6 degrees. "
    "Say \"done\" only when the goal is visibly satisfied in the wrist view. Say \"abort\" if the target is not visible, "
    "something is in the way, water or wires could be hit, or you are unsure after several steps. Never guess: abort is cheap."
)


@dataclass
class ServoOutcome:
    ok: bool
    reason: str
    steps: int
    joints: dict[str, float] = field(default_factory=dict)
    trace: list[dict[str, Any]] = field(default_factory=list)
    cost_usd: float = 0.0


class LLMServo:
    def __init__(self, runner: SkillRunner, cameras: dict[str, CameraAdapter], backend, store=None, cycle_id: str | None = None,
                 max_steps: int = 40, min_confidence: float = 0.4):
        self.runner = runner
        self.cameras = cameras
        self.backend = backend
        self.store = store
        self.cycle_id = cycle_id
        self.max_steps = max_steps
        self.min_confidence = min_confidence

    def _frames(self, arm: str) -> list[tuple[str, Any]]:
        out = []
        for name in (f"{arm}_wrist", "head"):
            cam = self.cameras.get(name)
            if cam is None:
                continue
            r = cam.frame()
            if r.status is Status.OK:
                out.append((name, r.value))
        return out

    def run(self, arm: str, goal: str, save_as: str | None = None, allow_gripper: bool = True) -> ServoOutcome:
        m = self.runner.models[arm]
        self.runner._read_joints()   # sync the Cartesian estimate from the real joints
        trace: list[dict[str, Any]] = []
        cost = 0.0
        stalls = 0
        for step in range(1, self.max_steps + 1):
            frames = self._frames(arm)
            if not any(n.endswith("wrist") for n, _ in frames):
                self.runner.stop()
                return ServoOutcome(False, "wrist camera frame not OK", step - 1, trace=trace, cost_usd=cost)
            pose = m.pose
            prompt = (f"Goal: {goal}\nArm: {arm}. Step {step}/{self.max_steps}. "
                      f"Current estimate: x={pose.x*1000:.0f} mm, y={pose.y*1000:.0f} mm, pitch={pose.pitch:.0f}, pan={pose.pan:.0f}, roll={pose.roll:.0f}, gripper={pose.gripper:.0f}.\n"
                      f"Previous steps: {json.dumps(trace[-4:], default=str)}")
            try:
                d, meta = self.backend.complete_json(prompt, images=frames, system=SYSTEM)
            except LLMError as e:
                self.runner.stop()
                return ServoOutcome(False, f"model error: {e}", step - 1, trace=trace, cost_usd=cost)
            cost += meta.cost_usd
            action = str(d.get("action", "abort")).lower()
            conf = float(d.get("confidence", 0) or 0)
            rec = {"step": step, "action": action, "conf": conf, "why": str(d.get("why", ""))[:160],
                   "dx": d.get("dx_mm", 0), "dy": d.get("dy_mm", 0), "dpitch": d.get("dpitch_deg", 0), "dpan": d.get("dpan_deg", 0), "droll": d.get("droll_deg", 0), "gripper": d.get("gripper")}
            trace.append(rec)
            if self.store is not None:
                self.store.decision(self.cycle_id, "llm_servo", goal, json.dumps({"frames": [n for n, _ in frames]}), action, {"confidence": conf},
                                    "farm-servo-1", meta.model, meta.latency_ms, meta.cost_usd, honoured=True, note=rec["why"])
            if action == "abort":
                self.runner.stop()
                return ServoOutcome(False, f"model aborted: {rec['why']}", step, trace=trace, cost_usd=cost)
            if action == "done":
                joints = self.runner._read_joints()
                if save_as and self.runner.kf is not None:
                    self.runner.kf.save(save_as, {k: v for k, v in joints.items() if k.startswith(f"{arm}_arm_") or k.startswith("head_")}, arm=arm,
                                        note=goal, learned_by=f"llm_servo:{meta.model}")
                return ServoOutcome(True, rec["why"], step, joints, trace, cost)
            if conf < self.min_confidence:
                stalls += 1
                if stalls >= 3:
                    self.runner.stop()
                    return ServoOutcome(False, "model confidence stayed low", step, trace=trace, cost_usd=cost)
                continue
            try:
                if action == "move":
                    m.apply_delta(rec["dx"] or 0, rec["dy"] or 0, rec["dpitch"] or 0, rec["dpan"] or 0, rec["droll"] or 0,
                                  gripper=(rec["gripper"] if allow_gripper and rec["gripper"] is not None else None))
                    self.runner.move_arm_pose(arm, m.pose, max_s=3)
                elif action in ("grip", "release") and allow_gripper:
                    g = 0.0 if action == "grip" else 60.0
                    cur = self.runner._read_joints()
                    cur[arm_joint(arm, "gripper")] = g
                    self.runner.move_joints({arm_joint(arm, "gripper"): g}, max_s=3, settle_tol=1.0)
                    m.pose.gripper = self.runner.gripper_of(arm)
                else:
                    continue
            except SafetyStop as e:
                return ServoOutcome(False, f"safety stop: {e}", step, trace=trace, cost_usd=cost)
            time.sleep(0.2)
        self.runner.stop()
        return ServoOutcome(False, "step budget exhausted", self.max_steps, trace=trace, cost_usd=cost)
