"""One carton: look, then run the plan step by step, judging the box after every step.

Same discipline as the farm's care cycle: INTENT is written before a step moves, ATTEMPT when
it moves, RESULT after the camera confirms it. A judgement that cannot tell pauses the cycle
and asks a person; a step whose check fails is retried once, then a person is asked. STOP
from the viewer ends everything at once. Nothing here drives wheels or lifts the box.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from farm.adapters.base import arm_joint
from farm.safety.rules import SafetyStop
from farm.skills.runner import GRIP_HOLDING_MIN, GRIP_OPEN
from farm.status import Status

from . import perception
from .geometry import Box
from .plan import GRIP_STEPS, RELEASE_AFTER, STEPS, Step

log = logging.getLogger(__name__)


class S(str, Enum):
    IDLE = "IDLE"
    LOOK = "LOOK"
    STEP = "STEP"
    VERIFY = "VERIFY"
    PARK = "PARK"
    PAUSED = "PAUSED"
    DONE = "DONE"


TRANSITIONS = {
    S.IDLE: {S.LOOK, S.PAUSED}, S.LOOK: {S.STEP, S.PARK, S.PAUSED}, S.STEP: {S.VERIFY, S.PAUSED},
    S.VERIFY: {S.STEP, S.PARK, S.PAUSED}, S.PARK: {S.DONE, S.PAUSED}, S.PAUSED: {S.STEP, S.VERIFY, S.PARK, S.DONE}, S.DONE: set(),
}


@dataclass
class Outcome:
    cycle_id: str
    result: str                      # CLOSED | NOT_CLOSED | STOPPED | CRASHED
    note: str = ""
    steps_done: list[str] = field(default_factory=list)
    history: list[dict[str, Any]] = field(default_factory=list)


class CartonCycle:
    def __init__(self, system, box: Box, ask_timeout_s: float | None = None, judge_frames: tuple[str, ...] = ("head",)):
        self.sys = system
        self.box = box
        self.store = system.store
        self.skills = system.skills
        self.cameras = system.cameras
        self.human = system.human
        self.ask_timeout_s = ask_timeout_s if ask_timeout_s is not None else float(system.profile.deadlines.ask_s or 600)   # the profile decides how long a person gets
        self.judge_frames = judge_frames
        self.state = S.IDLE
        self.history: list[dict[str, Any]] = []
        self.cycle_id = self.store.start_cycle("carton", system.profile.name, system.profile.config_hash, getattr(system.robot, "calibration_id", "?"), system.profile.simulated)
        self.judgement: dict[str, Any] = {}
        self.steps_done: list[str] = []
        self.last_frames: dict[str, str] = {}
        self.vision = system.backends.vision

    # ---- bookkeeping ---------------------------------------------------------------------
    def _go(self, new: S, why: str = "") -> None:
        assert new in TRANSITIONS[self.state], f"{self.state.value} -> {new.value}"
        self.history.append({"from": self.state.value, "to": new.value, "t": time.time(), "why": why})
        self.state = new
        self._publish()

    def _publish(self, **kw) -> None:
        self.sys.state.update({"task": "carton", "cycle_id": self.cycle_id, "state": self.state.value, "judgement": self.judgement,
                               "steps_done": list(self.steps_done), "frames": self.last_frames, **kw})

    def _frames(self):
        out = []
        for n in self.judge_frames:
            cam = self.cameras.get(n)
            if cam is None:
                continue
            r = cam.frame()
            if r.status is Status.OK:
                h = self.store.save_image(r.value)
                self.last_frames[n] = h
                self.store.observation(self.cycle_id, f"camera.{n}", r.status.value, {"age_s": r.age()}, r.t, image_hash=h)
            out.append((n, r))
        return out

    def _look(self) -> dict[str, Any] | None:
        """Head to the box keyframe (if taught), judge. None when the model cannot tell."""
        if "look_box" in self.sys.keyframes.names():
            try:
                self.skills.move_joints(self.sys.keyframes.get("look_box"), max_s=4)
            except SafetyStop as e:
                self._pause(f"safety stop while looking: {e}")
                return None
        j = perception.judge(self.vision, self.box, self._frames(), self.store, self.cycle_id)
        if not j.ok:
            return None
        self.judgement = j.value
        self._publish()
        return j.value

    def _pause(self, reason: str) -> None:
        log.warning("PAUSED: %s", reason)
        try:
            self.skills.stop()
        except Exception:  # noqa: BLE001
            pass
        self.store.event(self.cycle_id, "paused", {"reason": reason, "state_before": self.state.value})
        self._go(S.PAUSED, reason)
        self._publish(pause_reason=reason)

    def _ask(self, step: Step, question: str) -> str:
        """Ask a person; returns 'done', 'redo' or 'stop'."""
        r = self.human.ask(f"carton:{step.name}", question, ["done", "redo", "stop"], {"cycle_id": self.cycle_id, "frames": dict(self.last_frames), "judgement": self.judgement}, self.ask_timeout_s)
        if r.status is not Status.OK or not r.value:
            return "stop"
        self.store.intervention(self.cycle_id, r.value.get("who", "?"), f"carton:{step.name}", question, r.value["choice"])
        return r.value["choice"]

    # ---- motion ---------------------------------------------------------------------
    def _play(self, step: Step) -> str | None:
        """Run a step's keyframes with the usual clamps. Returns an error string or None."""
        kf = self.sys.keyframes
        grip = arm_joint(step.arm, "gripper") if step.arm in ("left", "right") else None
        for name in step.keyframes:
            pose = kf.get(name)
            if pose is None:
                return f"keyframe {name!r} not taught yet (run `carton teach-all`)"
            if self.skills.estop.is_set():
                raise SafetyStop("STOP pressed in the viewer")
            target = dict(pose)
            if grip and step.name in GRIP_STEPS and name in step.keyframes[: step.keyframes.index(GRIP_STEPS[step.name]) + 1]:
                target[grip] = GRIP_OPEN                       # approach and grasp poses with the gripper open
            r = self.skills.move_joints(target, max_s=step.max_s)
            if not r.ok:
                return f"{name}: {r.note}"
            if grip and GRIP_STEPS.get(step.name) == name:
                self.skills.move_joints({grip: 0.0}, max_s=3, settle_tol=1.0)
                g = self.skills.gripper_of(step.arm)
                if step.name == "pick_paddle":
                    if g < GRIP_HOLDING_MIN:
                        self.skills.move_joints({grip: GRIP_OPEN}, max_s=3)
                        return f"gripper closed to {g:.0f}: no paddle in hand"
                    self.skills.held[step.arm] = "paddle"
            if grip and RELEASE_AFTER.get(step.name) == name:
                self.skills.move_joints({grip: GRIP_OPEN}, max_s=3)
        return None

    # ---- the cycle ---------------------------------------------------------------------
    def run(self) -> Outcome:
        rec = getattr(self.sys, "recorder", None)
        if rec is not None:
            try:
                rec.start_episode("close the carton: fold four flaps, tape the seam")
            except Exception as e:  # noqa: BLE001
                log.warning("recorder start failed: %s", e)
                rec = None
        try:
            out = self._run()
            if rec is not None:
                rec.end_episode(save=out.result == "CLOSED")
            return out
        except Exception as e:  # noqa: BLE001
            log.exception("carton cycle crashed")
            try:
                self.skills.stop()
            except Exception:  # noqa: BLE001
                pass
            self.store.end_cycle(self.cycle_id, "CRASHED", str(e))
            if rec is not None:
                try:
                    rec.end_episode(save=False)
                except Exception:  # noqa: BLE001
                    pass
            return Outcome(self.cycle_id, "CRASHED", str(e), self.steps_done, self.history)

    def _run(self) -> Outcome:
        self._publish()
        self._go(S.LOOK)
        j = self._look()
        if j is None:
            self._pause("the model cannot tell what the box looks like")
            return self._finish("NOT_CLOSED", "could not judge the box before starting")
        if j.get("box_present") is not True:
            return self._finish("NOT_CLOSED", "no box in view")
        if j.get("obstruction") is True:
            self._pause("something is in the way")
            return self._finish("NOT_CLOSED", "obstruction")
        for step in STEPS:
            already = perception.satisfied(j, step.check)
            if already is True and (step.name != "pick_paddle" or self.skills.held.get("right") == "paddle"):
                self.steps_done.append(step.name)
                continue
            outcome = self._do_step(step)
            if outcome != "ok":
                return self._finish("STOPPED" if outcome.startswith("stop") else "NOT_CLOSED", f"{step.name}: {outcome}")
            j = self.judgement
        self._go(S.PARK)
        self._park()
        self._go(S.DONE)
        return self._finish("CLOSED", "all steps verified")

    def _do_step(self, step: Step) -> str:
        """'ok', 'stop', or a reason the step could not be completed."""
        for attempt in (1, 2):
            self._go(S.STEP, step.name)
            aid = self.store.intent(self.cycle_id, step.name, {"arm": step.arm, "keyframes": list(step.keyframes), "attempt": attempt})
            self.store.attempt(aid)
            try:
                err = self._play(step)
            except SafetyStop as e:
                self.store.result(aid, "ABORTED", note=f"safety stop: {e}")
                self._pause(f"safety stop during {step.name}: {e}")
                return f"stop: {e}"
            if err:
                self.store.result(aid, "ABORTED", note=err)
                self._pause(f"{step.name}: {err}")
                return err
            self._go(S.VERIFY, step.name)
            if hasattr(self.vision, "mark"):          # simulator stand-in: tell it the step happened
                self.vision.mark(step.name)
            j = self._look()
            ok = None if j is None else perception.satisfied(j, step.check)
            if ok is True:
                self.store.result(aid, "VERIFIED", self.last_frames.get(self.judge_frames[0], ""))
                self.steps_done.append(step.name)
                self.store.event(self.cycle_id, "step_done", {"step": step.name, "attempt": attempt})
                return "ok"
            if ok is None:
                self.store.result(aid, "UNKNOWN", note="model could not tell")
                self._pause(f"after {step.name}: the model cannot tell whether it worked")
                choice = self._ask(step, f"After '{step.name}', is the result correct ({step.check.replace('_', ' ')})? Look at the head view.")
            else:
                self.store.result(aid, "ABORTED", note=f"{step.check} still false")
                if attempt == 1:
                    self.store.event(self.cycle_id, "retry", {"step": step.name})
                    continue
                self._pause(f"{step.name} did not achieve {step.check} after two attempts")
                choice = self._ask(step, f"'{step.name}' failed twice. Fix the box by hand and say 'done', or 'redo' to try again, or 'stop'.")
            if choice == "done":
                self.steps_done.append(step.name)
                self.store.event(self.cycle_id, "person_confirmed", {"step": step.name})
                return "ok"
            if choice == "redo":
                continue
            return "stop"
        return f"{step.check} not achieved"

    def _park(self) -> None:
        kf = self.sys.keyframes
        try:
            for name in ("rest_left", "rest_right"):
                if name in kf.names():
                    self.skills.move_joints(kf.get(name), max_s=8)
        except SafetyStop as e:
            log.warning("park: %s", e)

    def _finish(self, result: str, note: str) -> Outcome:
        if self.state not in (S.DONE,):
            try:
                self.skills.stop()
            except Exception:  # noqa: BLE001
                pass
        self.store.end_cycle(self.cycle_id, result, note)
        self._publish(result=result, note=note)
        return Outcome(self.cycle_id, result, note, self.steps_done, self.history)
