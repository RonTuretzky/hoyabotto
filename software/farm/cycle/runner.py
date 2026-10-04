"""One care cycle for one tray, end to end, with evidence written at every step.

identify -> inspect -> measure light -> decide (rules, Jev, maybe a person)
-> pick bottle -> approach -> pour (INTENT/ATTEMPT/RESULT) -> return upright
-> verify -> park. Any failure pauses; pauses are resolved by a person from
the viewer; an UNKNOWN pour blocks any retry until reconciled.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any

from ..adapters.base import arm_joint
from ..config import Profile, TrayCfg
from ..llm.backends import NoLLM
from ..llm.jev import Jev
from ..perception.basic import frame_quality, green_fraction
from ..perception.vlm import VLMPerception
from ..safety.rules import SafetyStop
from ..skills.runner import SkillResult
from ..status import Reading, Status
from .authority import Authority
from .machine import Machine, S

log = logging.getLogger(__name__)

PAUSE_OPTIONS = ["reinspect", "checked: water ok, no pour needed", "checked: needs water, authorize one pour", "resolved: park and end cycle", "caretaker takes over"]
ASK_POUR_OPTIONS = ["authorize one pour", "skip this cycle", "reinspect"]
RECONCILE_OPTIONS = ["I looked: water reached the tray", "I looked: no water delivered", "I looked: spill — cleaned"]


@dataclass
class CycleOutcome:
    cycle_id: str
    tray_id: str
    result: str
    note: str = ""
    poured: bool = False
    history: list[dict[str, Any]] = field(default_factory=list)


class CareCycle:
    def __init__(self, system, tray: TrayCfg):
        self.sys = system
        self.p: Profile = system.profile
        self.tray = tray
        self.store = system.store
        self.human = system.human
        self.skills = system.skills
        self.cameras = system.cameras
        self.light = system.light
        self.m = Machine(self.p.deadlines)
        self.cycle_id = self.store.start_cycle(tray.id, self.p.name, self.p.config_hash, getattr(system.robot, "calibration_id", "?"), self.p.simulated)
        self.vlm = VLMPerception(system.backends.vision, self.store, self.cycle_id) if not isinstance(system.backends.vision, NoLLM) else None
        self.jev = Jev(system.backends.jev) if not isinstance(system.backends.jev, NoLLM) else None
        self.authority: Authority = system.authority
        self.judgement: dict[str, Any] = {}
        self.light_stats: dict[str, Any] | None = None
        self.pour_action_id: str | None = None
        self.last_frames: dict[str, str] = {}   # camera -> image hash (for the viewer)
        self.system_state = system.state         # shared dict the viewer reads

    # ---- helpers ---------------------------------------------------------------
    def _publish(self, **kw) -> None:
        self.system_state.update({"cycle_id": self.cycle_id, "tray": self.tray.id, "state": self.m.state.value, "since": self.m.entered_t,
                                  "pause_reason": self.m.pause_reason, "judgement": self.judgement, "frames": self.last_frames, **kw})

    def _frames(self, names=("head", f"{'right'}_wrist")) -> list[tuple[str, Reading]]:
        out = []
        for n in names:
            cam = self.cameras.get(n)
            if cam is None:
                continue
            r = cam.frame()
            if r.status is Status.OK:
                h = self.store.save_image(r.value)
                self.last_frames[n] = h
                self.store.observation(self.cycle_id, f"camera.{n}", r.status.value, {"age_s": r.age()}, r.t, image_hash=h)
            else:
                self.store.observation(self.cycle_id, f"camera.{n}", r.status.value, None, r.t, note=r.note)
            out.append((n, r))
        return out

    def _run_skill(self, fn, *a, **k) -> SkillResult:
        try:
            return fn(*a, **k)
        except SafetyStop as e:
            return SkillResult(False, getattr(fn, "__name__", "skill"), f"safety stop: {e}")

    def _pause(self, reason: str, evidence: dict[str, Any] | None = None) -> None:
        log.warning("PAUSED: %s", reason)
        try:
            self.skills.stop()
        except Exception:  # noqa: BLE001
            pass
        self.m.pause(reason, evidence)
        self.store.event(self.cycle_id, "paused", {"reason": reason, "state_before": self.m.history[-1]["from"], **(evidence or {})})
        self._publish()

    # ---- the cycle ---------------------------------------------------------------
    def run(self) -> CycleOutcome:
        rec = getattr(self.sys, "recorder", None)
        if rec is not None:
            try:
                rec.start_episode(f"care cycle tray {self.tray.id}")
            except Exception as e:  # noqa: BLE001
                log.warning("recorder start failed: %s", e)
                rec = None
        try:
            out = self._run()
            if rec is not None:
                rec.end_episode(save=out.result in ("POURED", "NO_POUR", "RESOLVED"))
            return out
        except Exception as e:  # noqa: BLE001
            log.exception("cycle crashed")
            try:
                self.skills.stop()
            except Exception:  # noqa: BLE001
                pass
            self.store.end_cycle(self.cycle_id, "CRASHED", str(e))
            self.system_state["last_error"] = str(e)
            if rec is not None:
                try:
                    rec.end_episode(save=False)
                except Exception:  # noqa: BLE001
                    pass
            return CycleOutcome(self.cycle_id, self.tray.id, "CRASHED", str(e), history=self.m.history)

    def _run(self) -> CycleOutcome:
        t = self.tray
        wrist = f"{self.p.arms.bottle}_wrist"
        self._publish()

        # Refuse to start while any earlier action is UNKNOWN: physics may already have poured.
        open_ = self.store.open_actions()
        if open_:
            self.m.go(S.IDENTIFY); self._pause(f"{len(open_)} unresolved action(s) from earlier: reconcile in the viewer first", {"open_actions": [a["action_id"] for a in open_]})
            return self._await_resolution()

        # IDENTIFY: the tray lives in a fixed nest; look at it, confirm it is there and the view is fresh.
        self.m.go(S.IDENTIFY); self._publish()
        r = self._run_skill(self.skills.look_at, t.id, t.look_pose)
        if not r.ok and "not taught" not in r.note:
            self._pause(f"look_at failed: {r.note}"); return self._await_resolution()
        frames = self._frames(("head", wrist))
        q = {n: frame_quality(f) for n, f in frames}
        for n, qq in q.items():
            self.store.observation(self.cycle_id, f"quality.{n}", qq.status.value, qq.value, note=qq.note, extractor_version="basic-1")
        bad = [f"{n}: {qq.note or qq.status.value}" for n, qq in q.items() if qq.status is not Status.OK]
        if bad:
            self._pause("camera view not usable: " + "; ".join(bad)); return self._await_resolution()
        if t.tag_id is not None:
            from ..perception.tags import confirm_tray
            head = dict(frames).get("head")
            tag = confirm_tray(head, t.tag_id) if head is not None else None
            if tag is not None:
                self.store.observation(self.cycle_id, "tray.tag", tag.status.value, tag.value, note=tag.note, extractor_version="apriltag-1")
                if tag.status is Status.OK and tag.value is False:
                    self._pause(f"wrong tray in nest {t.nest}: {tag.note}"); return self._await_resolution()

        # INSPECT: typed judgement from the vision model (or UNKNOWN without one).
        self.m.go(S.INSPECT); self._publish()
        if self.vlm is not None:
            j = self.vlm.judge_tray(t.id, t.kind, frames)
            self.judgement = j.value or {}
            self.store.observation(self.cycle_id, "vlm.judgement", j.status.value, self.judgement, note=j.note, extractor_version="vlm-1")
            if j.status is not Status.OK:
                self._pause(f"inspection unavailable: {j.note}"); return self._await_resolution()
        else:
            self.judgement = {"tray_present": "unknown", "opening_visible": "unknown", "water_visible": "unknown", "paper_edge": "unknown", "spill": "UNKNOWN", "obstruction": "unknown", "growth": "unknown", "confidence": 0.0, "notes": "no vision model configured"}
            self.store.observation(self.cycle_id, "vlm.judgement", "UNKNOWN", self.judgement, note="no vision backend")
        for n, f in frames:
            if n == "head":
                g = green_fraction(f)
                self.store.observation(self.cycle_id, "growth.green_fraction", g.status.value, g.value, extractor_version="basic-1")
        if self.judgement.get("tray_present") is False:
            self._pause(f"tray {t.id} not in its nest"); return self._await_resolution()
        if self.m.overdue():
            self._pause("inspect deadline passed"); return self._await_resolution()

        # MEASURE_LIGHT: optional; never decides thirst.
        self.m.go(S.MEASURE_LIGHT); self._publish()
        if self.light is not None and self.p.light.enabled:
            lr = self.light.latest()
            if lr.status is Status.OK and self.skills.held.get(self.p.arms.paddle) == "paddle" and t.measure_pose:
                r = self._run_skill(self.skills.measure_pose, t.id, t.measure_pose)
                if r.ok:
                    ms = self.light.measure(self.p.light.samples, self.p.light.settle_s)
                    self.light_stats = ms.value
                    self.store.observation(self.cycle_id, "light.measure", ms.status.value, ms.value, note=ms.note)
            else:
                self.store.observation(self.cycle_id, "light.latest", lr.status.value, lr.value, lr.t, note=lr.note or "paddle not in hand; ambient sample only")
                if lr.status is Status.OK:
                    self.light_stats = {"median_lux": lr.value, "n": 1, "ambient": True}

        # DECIDE: rules, then Jev, then (if needed) a person. Water never moves without one of them.
        self.m.go(S.DECIDE); self._publish()
        verdict = self._decide(frames)
        if verdict is None:
            return self._await_resolution()
        if not verdict.authorized:
            self.store.event(self.cycle_id, "no_pour", {"by": verdict.by, "reason": verdict.reason})
            self.m.go(S.PARK, verdict.reason)
            return self._park_and_finish("NO_POUR", verdict.reason)

        # PICK_BOTTLE
        self.m.go(S.PICK_BOTTLE); self._publish()
        if self.skills.held.get(self.p.arms.bottle) != "bottle":
            aid = self.store.intent(self.cycle_id, "pick_tool", {"tool": "bottle"})
            self.store.attempt(aid)
            r = self._run_skill(self.skills.pick_tool, "bottle", "bottle_rest_above", "bottle_rest_grip")
            self.store.result(aid, "VERIFIED" if r.ok else "ABORTED", note=r.note)
            if not r.ok:
                self._pause(f"pick bottle failed: {r.note}"); return self._await_resolution()

        # APPROACH
        self.m.go(S.APPROACH); self._publish()
        aid = self.store.intent(self.cycle_id, "approach", {"tray": t.id, "pose": t.pour_pose})
        self.store.attempt(aid)
        r = self._run_skill(self.skills.approach, t.id, t.pour_pose)
        self.store.result(aid, "VERIFIED" if r.ok else "ABORTED", note=r.note)
        if not r.ok:
            self._pause(f"approach failed: {r.note}"); return self._await_resolution()
        # Fresh look before water moves: the opening must still be visible and unobstructed.
        if self.vlm is not None:
            j2 = self.vlm.judge_tray(t.id, t.kind, self._frames((wrist,)))
            if j2.status is not Status.OK or j2.value.get("opening_visible") is not True or j2.value.get("obstruction") is True:
                self._pause(f"pre-pour view blocked: {j2.note or j2.value}"); return self._await_resolution()

        # POUR: INTENT -> ATTEMPT (before tilt) -> RESULT after verify
        self.m.go(S.POUR); self._publish()
        tilt, secs = self._dose()
        self.pour_action_id = self.store.intent(self.cycle_id, "pour", {"tray": t.id, "tilt_deg": tilt, "seconds": secs, "authorized_by": verdict.by})
        r = self._run_skill(self.skills.pour, tilt, secs, on_attempt=lambda: self.store.attempt(self.pour_action_id))
        self.m.go(S.RETURN_UPRIGHT); self._publish()
        if not r.data.get("upright_ok", r.ok):
            # Bottle may still be tilted: the most dangerous state we can be in. Try once more, then pause and hold.
            r2 = self._run_skill(self.skills.return_upright)
            if not r2.ok:
                self.store.result(self.pour_action_id, "UNKNOWN", note="upright recovery failed; bottle state unknown")
                self._pause("bottle may be tilted: upright recovery failed", {"action_id": self.pour_action_id}); return self._await_resolution()

        # VERIFY
        self.m.go(S.VERIFY); self._publish()
        after = self._frames((wrist, "head"))
        verified, why = self._verify(after)
        if verified is True:
            self.store.result(self.pour_action_id, "VERIFIED", note=why)
        elif verified is False:
            self.store.result(self.pour_action_id, "ABORTED", note=why)
        else:
            self.store.result(self.pour_action_id, "UNKNOWN", note=why)
            self._pause(f"delivery UNKNOWN: {why}", {"action_id": self.pour_action_id}); return self._await_resolution()
        if self.judgement.get("spill") == "SPILL":
            self._pause("spill detected after pour", {"action_id": self.pour_action_id}); return self._await_resolution()

        # PARK
        self.m.go(S.PARK); self._publish()
        return self._park_and_finish("POURED" if verified else "POUR_FAILED", why, poured=bool(verified))

    # ---- decision ----------------------------------------------------------------
    def _packet(self) -> dict[str, Any]:
        h = self.sys.robot.health()
        return {
            "tray": self.tray.id, "kind": self.tray.kind, "nest": self.tray.nest,
            "judgement": self.judgement, "light": self.light_stats,
            "last_pour_min_ago": self._minutes_since_last_pour(),
            "robot_health": (h.status.value if h.status is not Status.OK else "OK"),
            "held": dict(self.skills.held), "simulated": self.p.simulated,
            "authority_level": self.authority.level,
        }

    def _minutes_since_last_pour(self) -> float | None:
        rows = self.store.query("SELECT verify_t FROM actions WHERE skill='pour' AND result='VERIFIED' AND params_json LIKE ? ORDER BY verify_t DESC LIMIT 1", (f'%"tray": "{self.tray.id}"%',))
        return round((time.time() - rows[0]["verify_t"]) / 60, 1) if rows and rows[0]["verify_t"] else None

    def _rules(self) -> tuple[bool, str]:
        j = self.judgement
        if self.sys.backends.over_budget():
            return False, "LLM daily budget exhausted"
        if j.get("tray_present") is not True:
            return False, f"tray_present={j.get('tray_present')}"
        if j.get("opening_visible") is not True:
            return False, f"opening_visible={j.get('opening_visible')}"
        if j.get("obstruction") is not False:
            return False, f"obstruction={j.get('obstruction')}"
        if j.get("spill") == "SPILL":
            return False, "spill already present"
        if j.get("paper_edge") not in ("dry", "damp", "wet"):
            return False, f"paper_edge={j.get('paper_edge')}"
        m = self._minutes_since_last_pour()
        if m is not None and m < 30:
            return False, f"last verified pour {m:.0f} min ago (< 30)"
        h = self.sys.robot.health()
        if h.status not in (Status.OK, Status.NOT_APPLICABLE):
            return False, f"robot health {h.status.value}"
        return True, "tray present, opening visible, no obstruction, no spill, paper edge judged"

    def _decide(self, frames):
        ok, reason = self._rules()
        packet = self._packet()
        jev_route = jev_pour = None
        if self.jev is not None and ok:
            jev_route, jev_pour = self.jev.care_decisions(packet)
        v = self.authority.decide_pour(self.cycle_id, ok, reason, jev_pour, jev_route)
        self.store.event(self.cycle_id, "verdict", {"authorized": v.authorized, "by": v.by, "reason": v.reason, "ask_human": v.ask_human, "route": v.route})
        if v.authorized:
            self.human.notify(f"Jev authorized one pour into tray {self.tray.id} ({v.reason})", {"cycle_id": self.cycle_id, "frames": self.last_frames})
            return v
        if not v.ask_human:
            return v
        # Ask a person. Jev's suggestion is shown, never assumed.
        qid = f"pour:{self.cycle_id}"
        text = (f"Tray {self.tray.id}: paper edge {self.judgement.get('paper_edge')}, water visible {self.judgement.get('water_visible')}, "
                f"spill {self.judgement.get('spill')}. Rules: {reason}. Jev suggests: {jev_pour.choice if jev_pour else 'n/a'}"
                f"{f' (p={jev_pour.p:.2f})' if jev_pour else ''}; route {v.route}. Authorize one pour?")
        ans = self.human.ask(qid, text, ASK_POUR_OPTIONS, {"cycle_id": self.cycle_id, "frames": dict(self.last_frames), "judgement": self.judgement}, self.p.deadlines.ask_s)
        if ans.status is not Status.OK:
            self.store.event(self.cycle_id, "no_answer", {"question": qid})
            from .authority import Verdict
            return Verdict(False, "nobody", "no answer within the deadline: park, no water", ask_human=False, route=v.route)
        who = ans.value["who"]; choice = ans.value["choice"]
        self.store.intervention(self.cycle_id, who, qid, "viewer decision", choice, note=ans.value.get("note", ""))
        if jev_pour is not None:
            self.authority.record_agreement(self.cycle_id, "jev_pour", (choice == "authorize one pour") == (jev_pour.choice == "pour"), f"human={choice} jev={jev_pour.choice}")
        if jev_route is not None:
            self.authority.record_agreement(self.cycle_id, "jev_route", (jev_route.choice == "routine") == (choice != "reinspect"), f"human={choice} jev={jev_route.choice}")
        from .authority import Verdict
        if choice == "authorize one pour":
            return Verdict(True, f"human:{who}", "authorized in viewer", ask_human=False, route=v.route)
        if choice == "reinspect":
            self._pause(f"{who} asked for reinspection", {"question": qid}); return None
        return Verdict(False, f"human:{who}", "skipped", ask_human=False, route=v.route)

    def _dose(self) -> tuple[float, float]:
        cal = self.sys.pour_calibration()   # {tilt_deg, seconds} learned from cup tests, else conservative defaults
        return float(cal.get("tilt_deg", 25.0)), float(cal.get("seconds", 1.5))

    def _verify(self, after) -> tuple[bool | None, str]:
        if self.vlm is None:
            return None, "no vision model: delivery cannot be verified"
        j = self.vlm.judge_tray(self.tray.id, self.tray.kind, after)
        if j.status is not Status.OK:
            return None, f"post-pour judgement unavailable: {j.note}"
        self.judgement = j.value
        self.store.observation(self.cycle_id, "vlm.judgement.after", "OK", j.value, extractor_version="vlm-1")
        if j.value.get("spill") == "UNKNOWN":
            return None, "spill area not visible after pour"
        if j.value.get("water_visible") is True or j.value.get("paper_edge") in ("damp", "wet"):
            return True, f"after: water_visible={j.value.get('water_visible')} paper_edge={j.value.get('paper_edge')} spill={j.value.get('spill')}"
        if j.value.get("water_visible") is False and j.value.get("paper_edge") == "dry":
            return False, "no visible change after pour"
        return None, f"ambiguous after-pour view: {j.value}"

    # ---- endings ------------------------------------------------------------------
    def _park_and_finish(self, result: str, note: str, poured: bool = False) -> CycleOutcome:
        aid = self.store.intent(self.cycle_id, "park", {})
        self.store.attempt(aid)
        r = self._run_skill(self.skills.go_rest)
        self.store.result(aid, "VERIFIED" if r.ok else "ABORTED", note=r.note)
        if not r.ok:
            self._pause(f"park failed: {r.note}"); return self._await_resolution()
        self.m.go(S.DONE); self._publish()
        self.store.end_cycle(self.cycle_id, result, note)
        self.authority.consider_promotion()
        return CycleOutcome(self.cycle_id, self.tray.id, result, note, poured, self.m.history)

    def _await_resolution(self) -> CycleOutcome:
        """PAUSED: ask a person; only their answer moves the machine."""
        ev = dict(self.m.pause_evidence)
        options = list(PAUSE_OPTIONS)
        if ev.get("action_id"):
            options = RECONCILE_OPTIONS + ["caretaker takes over"]
        qid = f"pause:{self.cycle_id}"
        ans = self.human.ask(qid, f"PAUSED on tray {self.tray.id}: {self.m.pause_reason}", options, {"cycle_id": self.cycle_id, "frames": dict(self.last_frames), **ev}, self.p.deadlines.ask_s)
        if ans.status is not Status.OK:
            self.store.end_cycle(self.cycle_id, "PAUSED", self.m.pause_reason)
            self.system_state["needs_person"] = True
            return CycleOutcome(self.cycle_id, self.tray.id, "PAUSED", self.m.pause_reason, history=self.m.history)
        who, choice = ans.value["who"], ans.value["choice"]
        self.store.intervention(self.cycle_id, who, qid, choice, choice, note=ans.value.get("note", ""))
        if ev.get("action_id"):
            aid = ev["action_id"]
            if choice.startswith("I looked: water reached"):
                self.store.result(aid, "RECONCILED_OK", note=f"{who}: water reached the tray")
            elif choice.startswith("I looked: no water"):
                self.store.result(aid, "RECONCILED_FAIL", note=f"{who}: no water delivered")
            elif choice.startswith("I looked: spill"):
                self.store.result(aid, "RECONCILED_OK", note=f"{who}: spill cleaned")
                self.authority.demote_after_false_approval("spill after authorized pour")
        if choice == "reinspect" and self.m.state is S.PAUSED:
            self.m.go(S.INSPECT, f"{who}: reinspect")
            self.store.end_cycle(self.cycle_id, "REINSPECT", f"{who} asked for reinspection")
            return CycleOutcome(self.cycle_id, self.tray.id, "REINSPECT", who, history=self.m.history)
        if choice.startswith("checked: needs water"):
            self.store.end_cycle(self.cycle_id, "HUMAN_AUTHORIZED_NEXT", f"{who} authorized a pour on the next cycle")
            self.system_state["preauthorized"] = {"tray": self.tray.id, "who": who, "t": time.time()}
        self.m.go(S.PARK, f"{who}: {choice}")
        return self._park_and_finish("RESOLVED", f"{who}: {choice}")
