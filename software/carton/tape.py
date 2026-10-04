"""Guarded tape pickup from a dispenser and placement; no hard-coded wrist rotation.

Vision checks are fallible observations, not force sensing or collision guarantees.
Real poses must be taught at the actual station. Simulator marks are discrete test
fixtures and do not model adhesion, slip, flexible tape or contact dynamics.
"""
from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass

from farm.adapters.base import ARM_JOINTS, arm_joint
from farm.safety.rules import SafetyStop
from farm.skills.llm_servo import LLMServo
from farm.skills.runner import GRIP_OPEN
from farm.status import Status

from .plan import keyframe

POSES = ("tape_dispenser_above", "tape_dispenser_grip", "tape_lift_clear", "tape_over_seam", "tape_down", "tape_retract")
LEFT = {arm_joint("left", j) for j in ARM_JOINTS}
GRIPPER = arm_joint("left", "gripper")
VERSION = "carton-dispenser-v2"
SYSTEM = "Inspect this robot tape operation. Return only JSON. Use 'unknown' for anything not visibly established."
PROMPT = """TAPE_CHECK stage={stage}. A left robot gripper picks a fully cut strip by its exposed end
from a stationary tape dispenser support. No folded tab is required. The strip must start ADHESIVE
DOWN and remain that way throughout pickup and placement. There is NO flip/turnover operation.
A visible mark on the NONSTICKY BACKING identifies its face. Do not infer grip from jaw width,
adhesive direction from a requested motion, or clearance from a target pose. Inspect head and left
wrist images. Return each field as true, false, or "unknown":
left_empty (no tape/object in left jaws), end_accessible (exposed end reachable outside cutter/feed
mechanism), strip_ready (fully cut separate strip supported at pickup, not connected to roll),
end_in_jaws (end visibly between open fingers), tape_held (end visibly pinched),
clear_of_dispenser (entire strip and gripper clear of support, cutter/feed mechanism and cover),
adhesive_down, seam_ready (closed flaps form a supported seam),
tape_supported (strip lies on carton while end remains pinched), tape_on_seam (strip crosses seam),
obstruction (hands, cables, collision risk, dispenser movement, snagged/folded tape, or slipping grip).
Also return confidence as a number from 0 to 1 and notes as a short string.
Images alone cannot prove the dispenser is stopped; station setup must disable automatic cycling.
"""


@dataclass
class TapeOutcome:
    ok: bool
    stage: str
    note: str


class _FreshCamera:
    def __init__(self, camera):
        self.camera = camera

    def frame(self):
        r = self.camera.frame()
        if r.status is not Status.OK or r.value is None or not 0 <= r.age() <= 1.0:
            raise SafetyStop("tape requires fresh head and left-wrist images")
        return r


class _LockedRunner:
    """Scope LLM teaching to the left arm and an externally controlled grip."""
    def __init__(self, tape, grip):
        self.tape, self.grip = tape, grip

    def __getattr__(self, name):
        return getattr(self.tape.skills, name)

    def _read_joints(self):
        self.tape.guard()
        return self.tape.skills._read_joints()

    def move_arm_pose(self, arm, pose, max_s=3):
        if arm != "left":
            raise SafetyStop("tape teaching may move only the left arm")
        pose = pose.copy()
        pose.gripper = self.grip
        return self.move_joints(self.models[arm].joints_for(pose), max_s=max_s)

    def move_joints(self, targets, **kwargs):
        self.tape.guard()
        self.tape.validate_pose(targets)
        r = self.tape.skills.move_joints({**targets, GRIPPER: self.grip}, **kwargs)
        self.tape.guard()
        if not r.ok:
            raise SafetyStop(f"tape motion did not settle: {r.note}")
        return r


class TapeMotion:
    def __init__(self, system, cycle_id=None):
        self.sys = system
        self.skills = system.skills
        self.cycle_id = cycle_id
        self.stage = "preflight"
        self.pending = {}
        self.tag = f"{VERSION}:{system.profile.config_hash}:{system.robot.calibration_id}:"

    def guard(self):
        self.skills._guard(10)  # STOP plus health on each tape command/check

    @staticmethod
    def validate_pose(pose):
        if set(pose) != LEFT:
            raise SafetyStop("tape pose must contain exactly the six left-arm joints")
        for name, value in pose.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise SafetyStop(f"invalid tape joint value: {name}")
            if not (0 if name == GRIPPER else -100) <= value <= 100:
                raise SafetyStop(f"tape joint outside normalized range: {name}")

    def _cameras(self):
        if not all(n in self.sys.cameras for n in ("head", "left_wrist")):
            raise SafetyStop("tape requires head and left_wrist cameras")
        return {n: _FreshCamera(self.sys.cameras[n]) for n in ("head", "left_wrist")}

    def check(self, stage, *required):
        self.stage = stage
        self.guard()
        frames, hashes = [], {}
        for name, cam in self._cameras().items():
            r = cam.frame()
            frames.append((name, r.value))
            hashes[name] = self.sys.store.save_image(r.value)
        if self.sys.profile.simulated and hasattr(self.sys.backends.vision, "mark_tape"):
            self.sys.backends.vision.mark_tape(stage)
        d, meta = self.sys.backends.vision.complete_json(PROMPT.format(stage=stage), images=frames, system=SYSTEM)
        self.guard()  # STOP while the vision call was in flight must win
        confidence = d.get("confidence") if isinstance(d, dict) else None
        valid_confidence = type(confidence) in (int, float) and math.isfinite(confidence) and 0.8 <= confidence <= 1
        good = (valid_confidence and d.get("obstruction") is False and all(d.get(k) is True for k in required))
        self.sys.store.event(self.cycle_id, "tape_check", {"stage": stage, "required": required,
                             "judgement": d, "model": meta.model, "frames": hashes, "ok": bool(good)})
        self.sys.state.update(task="carton_tape", state=stage, frames=hashes, judgement=d)
        if not good:
            raise SafetyStop(f"tape check {stage} failed or unknown; need {', '.join(required)} and no obstruction")

    def gripper(self, target):
        """Wait for actual opening or stable contact; jaw width does not prove tape grip.

        SkillRunner's ordinary settling test excludes the gripper, so repeat its
        bounded ticks here until it opens fully or closing stops on contact.
        """
        self.guard()
        start = time.monotonic()
        stable_since = start
        last = self.skills.gripper_of("left")
        while time.monotonic() - start < 3:
            self.guard()
            r = self.skills.move_joints({GRIPPER: target}, max_s=0.2)
            if not r.ok:
                raise SafetyStop(f"tape gripper failed: {r.note}")
            now = self.skills.gripper_of("left")
            if abs(now - target) <= 1:
                return
            if abs(now - last) > 0.1:
                stable_since = time.monotonic()
            if target == 0 and time.monotonic() - stable_since >= 0.3:
                return  # only the subsequent image check can establish tape held
            last = now
        raise SafetyStop("tape gripper did not reach open position or stable closing contact")

    def _move(self, name, teach, grip):
        self.stage = name
        self.guard()
        if teach:
            servo = LLMServo(_LockedRunner(self, grip), self._cameras(), self.sys.backends.vision,
                             self.sys.store, self.cycle_id, max_steps=self.sys.profile.limits.llm_servo_max_steps)
            out = servo.run("left", keyframe(name).goal + " Do not change the gripper opening; it is controlled by the sequence.",
                            save_as=None, allow_gripper=False)
            self.guard()
            if not out.ok:
                raise SafetyStop(f"teaching {name}: {out.reason}")
            pose = {j: v for j, v in self.skills._read_joints().items() if j in LEFT}
            self.validate_pose(pose)
            self.pending[name] = pose
        else:
            pose = self.sys.keyframes.get(name)
            self.validate_pose(pose or {})
            r = self.skills.move_joints({**pose, GRIPPER: grip}, max_s=8)
            if not r.ok:
                raise SafetyStop(f"{name}: {r.note}")
        self.guard()
        self.sys.store.event(self.cycle_id, "tape_pose", {"name": name, "teaching": teach, "gripper_target": grip})

    def _preflight_poses(self):
        tags = set()
        for name in POSES:
            meta = self.sys.keyframes.meta(name) or {}
            tag = meta.get("learned_by", "")
            if not tag.startswith(self.tag):
                raise SafetyStop(f"{name}: missing/current-station tape sequence not taught; run carton tape-test --teach")
            tags.add(tag)
            self.validate_pose(self.sys.keyframes.get(name) or {})
        if len(tags) != 1:
            raise SafetyStop("tape poses came from different teaching runs; re-teach the complete sequence")

    def run(self, teach=False):
        """Complete one strip, no automatic retries or motion recovery on failure."""
        try:
            self.guard()
            if not teach:
                self._preflight_poses()  # fail before moving, including before opening the jaws
            self.check("ready", "left_empty", "end_accessible", "strip_ready", "adhesive_down", "seam_ready")
            self.gripper(GRIP_OPEN)
            self._move("tape_dispenser_above", teach, GRIP_OPEN)
            self._move("tape_dispenser_grip", teach, GRIP_OPEN)
            self.check("before_pinch", "end_in_jaws", "strip_ready", "adhesive_down")
            self.gripper(0)
            self.skills.held["left"] = "tape_unverified"
            self.check("pinched", "tape_held", "adhesive_down")
            self.skills.held["left"] = "tape"
            self._move("tape_lift_clear", teach, 0)
            self.check("lifted", "tape_held", "clear_of_dispenser", "adhesive_down")
            self._move("tape_over_seam", teach, 0)
            self.check("over_seam", "tape_held", "adhesive_down", "seam_ready")
            self._move("tape_down", teach, 0)
            self.check("supported", "tape_held", "adhesive_down", "tape_supported", "tape_on_seam")
            self.gripper(GRIP_OPEN)
            self.skills.held["left"] = None
            self._move("tape_retract", teach, GRIP_OPEN)
            self.check("released", "left_empty", "tape_on_seam")
            if teach:
                tag = self.tag + uuid.uuid4().hex
                for name in POSES:  # only publish a teaching bundle after the entire sequence passes
                    self.sys.keyframes.save(name, self.pending[name], "left", keyframe(name).goal, tag)
            return TapeOutcome(True, self.stage, "tape placed and released; pressing is a separate step")
        except (Exception, KeyboardInterrupt) as e:
            self.sys.store.event(self.cycle_id, "tape_stopped", {"stage": self.stage, "reason": str(e)})
            self.sys.state.update(state="PAUSED", pause_reason=f"tape {self.stage}: {e}")
            return TapeOutcome(False, self.stage, str(e) or "interrupted")
        finally:
            self.skills.stop()  # hold; do not retry, release a held strip, or retreat blindly
