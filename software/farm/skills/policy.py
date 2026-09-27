"""Run a learned policy (ACT / SmolVLA / …) as a skill, inside the same safety envelope as every
other skill. The policy never talks to motors: it proposes joint targets for ONE arm; the skill
runner clamps each tick to the profile's step limit and the watchdog/health rules still apply.

Feature mapping (profile `policy:` section or PolicyRunner.input_spec()):
  - state: the policy's state vector is built from a list of our joint names (e.g. the six
    right_arm_* joints for a single-arm SO-101 policy trained on so101_follower data)
  - cameras: policy image keys -> our camera names (e.g. observation.images.wrist -> right_wrist)
  - actions: same joint list, absolute normalized positions (LeRobot convention)
Termination: max_steps / max_s, a stall detector (no joint moved for N ticks), an optional
`done_check` callback (e.g. a VLM judgement), and the postcondition of the calling skill.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from ..adapters.base import CameraAdapter
from ..safety.rules import SafetyStop
from ..status import Status
from .runner import SkillRunner

log = logging.getLogger(__name__)


@dataclass
class PolicyOutcome:
    ok: bool
    reason: str
    steps: int
    joints: dict[str, float] = field(default_factory=dict)
    trace: list[dict[str, Any]] = field(default_factory=list)


class PolicySkill:
    def __init__(self, runner: SkillRunner, cameras: dict[str, CameraAdapter], policy, state_joints: list[str],
                 camera_map: dict[str, str], hz: float = 10.0, max_steps: int = 300, stall_ticks: int = 40, stall_eps: float = 0.5,
                 done_check: Callable[[dict[str, float]], bool] | None = None, store=None, cycle_id: str | None = None, name: str = "policy"):
        """policy must expose reset() and act(state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray (len == len(state_joints))."""
        self.runner = runner
        self.cameras = cameras
        self.policy = policy
        self.state_joints = list(state_joints)
        self.camera_map = dict(camera_map)     # policy image key -> our camera name
        self.hz = hz
        self.max_steps = max_steps
        self.stall_ticks = stall_ticks
        self.stall_eps = stall_eps
        self.done_check = done_check
        self.store = store
        self.cycle_id = cycle_id
        self.name = name

    def _observe(self) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, float]] | None:
        cur = self.runner._read_joints()
        state = np.array([cur[j] for j in self.state_joints], dtype=np.float32)
        images = {}
        for key, cam_name in self.camera_map.items():
            cam = self.cameras.get(cam_name)
            r = cam.frame() if cam is not None else None
            if r is None or r.status is not Status.OK:
                return None
            images[key] = r.value
        return state, images, cur

    def run(self, goal: str = "") -> PolicyOutcome:
        self.policy.reset()
        trace: list[dict[str, Any]] = []
        last = None
        stalled = 0
        t0 = time.time()
        dt = 1.0 / self.hz
        for step in range(1, self.max_steps + 1):
            tick_t = time.time()
            obs = self._observe()
            if obs is None:
                self.runner.stop()
                return PolicyOutcome(False, "a policy camera frame was not OK", step - 1, trace=trace)
            state, images, cur = obs
            try:
                a = np.asarray(self.policy.act(state, images), dtype=np.float32).reshape(-1)
            except Exception as e:  # noqa: BLE001
                self.runner.stop()
                return PolicyOutcome(False, f"policy error: {e}", step - 1, trace=trace)
            if a.shape[0] != len(self.state_joints) or not np.all(np.isfinite(a)):
                self.runner.stop()
                return PolicyOutcome(False, f"policy returned {a.shape} / non-finite", step, trace=trace)
            targets = {j: float(v) for j, v in zip(self.state_joints, a)}
            try:
                self.runner.move_joints(targets, max_s=max(dt, 0.2), settle_tol=1e9)   # one clamped step, no settling wait
            except SafetyStop as e:
                return PolicyOutcome(False, f"safety stop: {e}", step, trace=trace)
            moved = 0.0 if last is None else float(np.max(np.abs(state - last)))
            last = state
            stalled = stalled + 1 if moved < self.stall_eps and step > 5 else 0
            trace.append({"step": step, "moved": round(moved, 2), "a0": round(float(a[0]), 1)})
            if self.store is not None and step % 10 == 0:
                self.store.event(self.cycle_id, "policy_step", {"skill": self.name, "step": step, "moved": moved})
            if self.done_check is not None and self.done_check(cur):
                return PolicyOutcome(True, "done_check satisfied", step, self.runner._read_joints(), trace)
            if stalled >= self.stall_ticks:
                return PolicyOutcome(False, f"policy stalled for {self.stall_ticks} ticks", step, self.runner._read_joints(), trace)
            time.sleep(max(0.0, dt - (time.time() - tick_t)))
        self.runner.stop()
        return PolicyOutcome(False, "max_steps reached", self.max_steps, self.runner._read_joints(), trace)


class ScriptedPolicy:
    """Test double: plays a fixed list of action vectors, then repeats the last one."""

    def __init__(self, actions: list[list[float]]):
        self.actions = [np.array(a, dtype=np.float32) for a in actions]
        self.i = 0
        self.calls: list[tuple[np.ndarray, list[str]]] = []

    def reset(self) -> None:
        self.i = 0

    def act(self, state, images):
        self.calls.append((state.copy(), sorted(images)))
        a = self.actions[min(self.i, len(self.actions) - 1)]
        self.i += 1
        return a
