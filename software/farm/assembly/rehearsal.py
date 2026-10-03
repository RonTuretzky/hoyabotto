"""Offline rehearsal of the R2a supervisor, including failures between pick and release.

This models discrete object states, NOT geometry, contacts, vision, or robot dynamics.
No hardware adapter can be passed to it. Its trace is never a training demonstration.
The ordered checks are the acceptance specification for the future hardware executor.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from ..status import Reading, Status
from .r2a import (
    AssemblyMachine, AssemblyProfile, EpisodeAccount, EvidenceMissing, HeldState,
    Stage, check_paper_pick, required_evidence,
)

FAULTS = (
    "none", "preflight_unknown", "no_pick", "lost_carrier", "seat_unknown",
    "double_paper", "no_paper", "paper_unknown", "paper_on_jaws",
    "frame_failed", "stale_evidence", "stop_during_transfer", "deadline", "intervention",
)
PARTS = {
    Stage.PLACE_PREPARED_CARRIER: "carrier",
    Stage.PLACE_REAL_TOP_PAPER: "paper",
    Stage.PLACE_RETAINER: "retainer",
}
POSES = ("approach", "grasp", "lift", "transfer", "seat", "retreat")


def required_keyframes(variant) -> list[str]:
    """Names only: every pose must later be taught and validated on the actual station."""
    from .r2a import stage_order
    return [f"r2a_{part}_{pose}" for stage, part in PARTS.items()
            if stage in stage_order(variant) for pose in POSES] + ["r2a_safe"]


class Rehearsal:
    def __init__(self, profile: AssemblyProfile, fault: str = "none"):
        if fault not in FAULTS:
            raise ValueError(f"unknown fault {fault!r}")
        if fault == "frame_failed" and Stage.PLACE_RETAINER not in AssemblyMachine(profile.variant).order():
            raise ValueError("frame_failed requires R2a_with_frame")
        self.profile, self.fault = profile, fault
        self.machine = AssemblyMachine(profile.variant, profile.deadlines_s, profile.max_attempts)
        # No real paper is present in an offline rehearsal, even on the nominal path.
        self.account = EpisodeAccount(profile.variant, real_paper=False)
        self.held = HeldState.UNKNOWN
        self.events: list[dict] = []
        self.used = False

    def log(self, kind: str, **data):
        self.events.append({"seq": len(self.events), "t": time.time(),
                            "stage": self.machine.stage.value, "kind": kind, **data})

    def require(self, name: str, value=True, status=Status.OK, age=0.0):
        """Fresh, affirmative observations only; None/numbers/strings never mean yes."""
        now = time.time()
        reading = Reading(value, status, t=now - age, source="synthetic_scene")
        self.log("observation", name=name, reading=reading.to_record())
        if (status is not Status.OK or value is not True or not math.isfinite(reading.t)
                or not 0 <= reading.age(now) <= 1.0):
            raise EvidenceMissing(self.machine.stage, {name: f"{status.value}: {value!r}, age={age}"})
        return reading

    def action(self, name: str, part: str = ""):
        if self.machine.overdue():
            raise EvidenceMissing(self.machine.stage, {"deadline": "expired before action"})
        if name == "release" and self.held is not HeldState.HELD:
            raise EvidenceMissing(self.machine.stage, {"held_object": "release requires a known held part"})
        self.log("intent", action=name, part=part)
        self.log("synthetic_action", action=name, part=part)

    def place(self, part: str):
        self.require("gripper_empty_before_pick", self.held is HeldState.EMPTY)
        self.action("approach", part)
        self.action("grasp", part)
        self.action("close", part)
        self.held = HeldState.UNKNOWN
        self.require("pick_present", not (self.fault == "no_pick" and part == "carrier"))
        self.held = HeldState.HELD
        self.action("lift", part)
        if part == "paper":
            count = {"double_paper": 2, "no_paper": 0, "paper_unknown": None}.get(self.fault, 1)
            pick = check_paper_pick(count)
            self.require("sheet_count_correct_before_transfer", pick.value, pick.status)
        if part == "carrier" and self.fault == "lost_carrier":
            self.held = HeldState.UNKNOWN
            self.require("carrier_held_after_lift", None, Status.UNKNOWN)
        self.require("held_after_lift")
        if part == "carrier" and self.fault == "stop_during_transfer":
            self.account.safety_faults.append("synthetic STOP during transfer")
            raise RuntimeError("synthetic STOP during transfer")
        if part == "carrier" and self.fault == "deadline":
            self.machine.entered_t -= self.machine.deadline_s() + 1
        self.action("transfer", part)
        self.require("held_after_transfer")
        self.action("seat", part)
        if self.fault == "seat_unknown" and part == "carrier":
            self.require("supported_before_release", None, Status.UNKNOWN)
        if self.fault == "frame_failed" and part == "retainer":
            self.require("supported_before_release", False)
        self.require("supported_before_release")
        self.action("release", part)
        self.held = HeldState.UNKNOWN
        self.require("jaws_clear_after_release", not (self.fault == "paper_on_jaws" and part == "paper"))
        self.held = HeldState.EMPTY
        self.require("part_stays_after_release")
        self.action("retreat", part)

    def run(self) -> dict:
        if self.used:
            raise RuntimeError("a rehearsal instance cannot resume or retry; create a new episode")
        self.used = True
        reason = ""
        self.log("start", simulated=True, evidence_scope="discrete supervisor only")
        try:
            while self.machine.stage is not Stage.DONE:
                stage = self.machine.stage
                self.machine.begin_attempt()
                if stage is Stage.PREFLIGHT:
                    if self.fault == "preflight_unknown":
                        self.require("trough_secured_empty", None, Status.UNKNOWN)
                    self.held = HeldState.EMPTY
                if stage in PARTS:
                    self.place(PARTS[stage])
                if self.fault == "intervention" and stage is Stage.VERIFY_CARRIER:
                    self.account.intervene("synthetic_operator", stage, "fixture adjusted")
                    raise RuntimeError("intervention ends autonomous attempt; physical reset required")
                if stage is Stage.VERIFY_DRY_ASSEMBLY:
                    self.require("empty_before_safe_pose", self.held is HeldState.EMPTY)
                    self.action("safe_pose")
                evidence = {}
                for name in required_evidence(stage, self.machine.variant):
                    age = 5 if self.fault == "stale_evidence" and stage is Stage.VERIFY_CARRIER else 0
                    evidence[name] = self.require(name, age=age)
                self.machine.advance(evidence)
                self.account.record(stage, "VERIFIED")
                self.log("stage_verified", verified_stage=stage.value)
        except (EvidenceMissing, RuntimeError) as exc:
            reason = str(exc)
            if self.machine.stage.value not in self.account.stage_results:
                self.account.record(self.machine.stage, "FAILED")
            self.log("stop_hold", held=self.held.value, reason=reason)
            # No release, park, torque-off, generic recovery or blind retry here.
            self.machine.fail(reason)
        complete = self.machine.stage is Stage.DONE
        result = {"simulated": True, "evidence_scope": "discrete supervisor; no geometry, physics, vision or hardware",
                  "result": "SIMULATED_COMPLETE" if complete else "SIMULATED_STOPPED",
                  "physical_success": False, "training_eligible": False,
                  "fault": self.fault, "reason": reason, "variant": self.machine.variant.value,
                  "held": self.held.value, "attempts": self.machine.attempts,
                  "account": self.account.to_dict(), "events": self.events}
        return result


def save_trace(result: dict, path: Path):
    """Never overwrite an earlier attempt's evidence."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
