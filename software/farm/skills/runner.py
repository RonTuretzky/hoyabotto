"""Skills: eight motions with contracts, executed against a RobotAdapter under the safety rules.

Every skill:
  - checks its preconditions, or refuses
  - moves in bounded steps at a fixed tick, re-reading health and the watchdog
  - verifies its postcondition from measured joints (and, for grips, gripper position)
  - returns a SkillResult; the orchestrator records intent/attempt/result around it
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any

from ..adapters.base import ARM_JOINTS, HEAD_JOINTS, RobotAdapter, arm_joint
from ..config import ArmsCfg, LimitsCfg
from ..safety.rules import SafetyStop, bound_pour, check_health, check_watchdog, clamp_targets
from ..status import Status
from .arm import ArmModel, ArmPose
from .keyframes import KeyframeStore

log = logging.getLogger(__name__)

GRIP_CLOSED_EMPTY = 8.0     # gripper position (0..100) reached when closing on nothing
GRIP_HOLDING_MIN = 15.0     # closing on the bottle/paddle stops above this
GRIP_OPEN = 60.0
TICK_S = 1 / 30


@dataclass
class SkillResult:
    ok: bool
    skill: str
    note: str = ""
    joints: dict[str, float] = field(default_factory=dict)
    steps: int = 0
    data: dict[str, Any] = field(default_factory=dict)


class SkillRunner:
    def __init__(self, robot: RobotAdapter, limits: LimitsCfg, arms: ArmsCfg, keyframes: KeyframeStore, on_tick=None):
        self.robot = robot
        self.limits = limits
        self.arms = arms
        self.kf = keyframes
        self.models = {"left": ArmModel("left"), "right": ArmModel("right")}
        self.held: dict[str, str | None] = {"left": None, "right": None}   # tool currently gripped per arm
        self.on_tick = on_tick   # callback(joints) for evidence/telemetry
        self._stopped = False

    # ---- low-level -----------------------------------------------------------
    def _read_joints(self) -> dict[str, float]:
        j = self.robot.joints()
        v = check_watchdog(j, self.limits)
        if not v.ok:
            self.stop()
            raise SafetyStop(v.reason)
        for arm, m in self.models.items():
            m.sync_from_joints(j.value)
        return j.value

    def _guard(self, tick: int) -> None:
        if tick % 10 == 0:
            h = check_health(self.robot.health(), self.limits)
            if not h.ok:
                self.stop()
                raise SafetyStop(h.reason)

    def move_joints(self, targets: dict[str, float], max_s: float = 6.0, settle_tol: float = 2.0) -> SkillResult:
        """Interpolate to targets in clamped steps; done when within tolerance or when max_s elapses."""
        t0 = time.time(); tick = 0
        cur = self._read_joints()
        goal = {k: float(v) for k, v in targets.items() if k in cur}
        while True:
            tick += 1
            self._guard(tick)
            step = clamp_targets(cur, goal, self.limits.step_deg_max)
            r = self.robot.move_to(step, max_step=self.limits.step_deg_max)
            if r.status is not Status.OK:
                self.stop(); raise SafetyStop(f"move refused: {r.note}")
            time.sleep(TICK_S)
            cur = self._read_joints()
            if self.on_tick:
                self.on_tick(cur)
            # grippers stall on whatever they hold; they never gate settling
            err = max((abs(cur[k] - goal[k]) for k in goal if not k.endswith("gripper")), default=0.0)
            if err <= settle_tol:
                return SkillResult(True, "move_joints", joints=cur, steps=tick)
            if time.time() - t0 > max_s:
                return SkillResult(False, "move_joints", f"did not settle: max err {err:.1f}", joints=cur, steps=tick)

    def move_arm_pose(self, arm: str, pose: ArmPose, max_s: float = 6.0) -> SkillResult:
        m = self.models[arm]
        m.pose = pose.copy()
        return self.move_joints(m.joints(), max_s=max_s)

    def stop(self) -> None:
        self._stopped = True
        try:
            self.robot.stop()
        except Exception as e:  # noqa: BLE001
            log.error("stop failed: %s", e)

    def keyframe_or_fail(self, name: str) -> dict[str, float]:
        kf = self.kf.get(name)
        if kf is None:
            raise SafetyStop(f"keyframe {name!r} not taught yet (run `farm teach` with the LLM servo)")
        return kf

    # ---- contracts -----------------------------------------------------------
    def gripper_of(self, arm: str) -> float:
        return self._read_joints()[arm_joint(arm, "gripper")]

    def _require_empty(self, arm: str) -> None:
        if self.held[arm] is not None:
            raise SafetyStop(f"{arm} arm already holds {self.held[arm]}")

    # ---- the eight skills -----------------------------------------------------
    def go_rest(self) -> SkillResult:
        rest = self.kf.get("rest_both")
        if rest is None:
            rest = {**self.models["left"].joints_for(ArmPose()), **self.models["right"].joints_for(ArmPose()), **{h: 0.0 for h in HEAD_JOINTS}}
            for arm in ("left", "right"):  # keep whatever the gripper holds
                rest[arm_joint(arm, "gripper")] = self._read_joints()[arm_joint(arm, "gripper")]
        r = self.move_joints(rest, max_s=8)
        r.skill = "go_rest"
        return r

    def look_at(self, tray_id: str, pose_name: str) -> SkillResult:
        kf = self.keyframe_or_fail(pose_name or f"look_{tray_id}")
        head = {k: v for k, v in kf.items() if k in HEAD_JOINTS}
        r = self.move_joints(head, max_s=4)
        r.skill = "look_at"; r.data = {"tray": tray_id}
        return r

    def pick_tool(self, tool: str, rest_pose: str, grip_pose: str) -> SkillResult:
        arm = self.arms.bottle if tool == "bottle" else self.arms.paddle
        self._require_empty(arm)
        approach = self.keyframe_or_fail(rest_pose)          # above the tool rest, gripper open
        grasp = self.keyframe_or_fail(grip_pose)             # around the tool, gripper open
        approach[arm_joint(arm, "gripper")] = GRIP_OPEN
        grasp[arm_joint(arm, "gripper")] = GRIP_OPEN
        r = self.move_joints(approach, max_s=8)
        if not r.ok:
            return SkillResult(False, "pick_tool", f"approach: {r.note}")
        r = self.move_joints(grasp, max_s=6)
        if not r.ok:
            return SkillResult(False, "pick_tool", f"grasp pose: {r.note}")
        close = dict(grasp); close[arm_joint(arm, "gripper")] = 0.0
        self.move_joints(close, max_s=3, settle_tol=1.0)
        g = self.gripper_of(arm)
        if g < GRIP_HOLDING_MIN:  # closed all the way: nothing in the gripper
            open_ = dict(grasp); open_[arm_joint(arm, "gripper")] = GRIP_OPEN
            self.move_joints(open_, max_s=3)
            return SkillResult(False, "pick_tool", f"gripper closed to {g:.0f}: no {tool} in hand", data={"gripper": g})
        self.held[arm] = tool
        lift = dict(approach); lift[arm_joint(arm, "gripper")] = g
        r = self.move_joints(lift, max_s=6)
        return SkillResult(r.ok, "pick_tool", r.note, r.joints, data={"tool": tool, "arm": arm, "gripper": g})

    def place_tool(self, tool: str, rest_pose: str, grip_pose: str) -> SkillResult:
        arm = self.arms.bottle if tool == "bottle" else self.arms.paddle
        if self.held[arm] != tool:
            return SkillResult(False, "place_tool", f"{arm} arm does not hold {tool}")
        g = self.gripper_of(arm)
        approach = self.keyframe_or_fail(rest_pose); approach[arm_joint(arm, "gripper")] = g
        grasp = self.keyframe_or_fail(grip_pose); grasp[arm_joint(arm, "gripper")] = g
        r = self.move_joints(approach, max_s=8)
        if not r.ok:
            return SkillResult(False, "place_tool", f"approach: {r.note}")
        r = self.move_joints(grasp, max_s=6)
        if not r.ok:
            return SkillResult(False, "place_tool", f"lower: {r.note}")
        open_ = dict(grasp); open_[arm_joint(arm, "gripper")] = GRIP_OPEN
        self.move_joints(open_, max_s=3)
        self.held[arm] = None
        r = self.move_joints({**approach, arm_joint(arm, "gripper"): GRIP_OPEN}, max_s=6)
        return SkillResult(r.ok, "place_tool", r.note, r.joints, data={"tool": tool, "arm": arm})

    def measure_pose(self, tray_id: str, pose_name: str) -> SkillResult:
        arm = self.arms.paddle
        if self.held[arm] != "paddle":
            return SkillResult(False, "measure_pose", "paddle not held")
        kf = self.keyframe_or_fail(pose_name or f"measure_{tray_id}")
        kf[arm_joint(arm, "gripper")] = self.gripper_of(arm)
        r = self.move_joints(kf, max_s=8)
        return SkillResult(r.ok, "measure_pose", r.note, r.joints, data={"tray": tray_id})

    def approach(self, tray_id: str, pose_name: str) -> SkillResult:
        arm = self.arms.bottle
        if self.held[arm] != "bottle":
            return SkillResult(False, "approach", "bottle not held")
        kf = self.keyframe_or_fail(pose_name or f"pour_{tray_id}")
        kf[arm_joint(arm, "gripper")] = self.gripper_of(arm)
        r = self.move_joints(kf, max_s=8)
        return SkillResult(r.ok, "approach", r.note, r.joints, data={"tray": tray_id})

    def pour(self, tilt_deg: float, seconds: float, on_attempt=None) -> SkillResult:
        """Tilt the held bottle by rolling the wrist, hold, then return upright. Bounded by limits."""
        arm = self.arms.bottle
        if self.held[arm] != "bottle":
            return SkillResult(False, "pour", "bottle not held")
        tilt, secs = bound_pour(tilt_deg, seconds, self.limits)
        cur = self._read_joints()
        roll_j = arm_joint(arm, "wrist_roll")
        upright = dict(cur)
        tilted = dict(cur); tilted[roll_j] = max(-100.0, min(100.0, cur[roll_j] + tilt))
        if on_attempt:
            on_attempt()   # ATTEMPT is written before water can move
        if hasattr(self.robot, "faults"):
            self.robot.faults.set("poured", True)   # simulator only: the scene gets wet
        r = self.move_joints(tilted, max_s=4)
        held_ok = r.ok
        time.sleep(secs)
        back = self.return_upright(upright)
        return SkillResult(held_ok and back.ok, "pour", "" if held_ok and back.ok else f"tilt:{r.note} upright:{back.note}", back.joints,
                           data={"tilt_deg": tilt, "seconds": secs, "upright_ok": back.ok})

    def return_upright(self, upright: dict[str, float] | None = None) -> SkillResult:
        arm = self.arms.bottle
        cur = self._read_joints()
        if upright is None:
            kf = self.kf.get("bottle_upright") or {}
            upright = dict(cur); upright.update({k: v for k, v in kf.items() if k.startswith(f"{arm}_arm_") and not k.endswith("gripper")})
            if not kf:
                upright[arm_joint(arm, "wrist_roll")] = 0.0
        upright[arm_joint(arm, "gripper")] = cur[arm_joint(arm, "gripper")]
        r = self.move_joints(upright, max_s=4)
        r.skill = "return_upright"
        return r
