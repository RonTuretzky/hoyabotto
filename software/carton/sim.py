"""Stand-in for the vision model when the carton task runs on the simulator. It tracks what the
cycle has done (the farm's FakeRobot has no box) and answers the judgement prompt from that,
with faults to force 'open', 'unknown' or an obstruction."""
from __future__ import annotations

from farm.adapters.sim import Faults
from farm.llm.backends import Meta


class SimCartonVision:
    name = "sim-carton-vision"
    model = "sim-carton-vision"

    def __init__(self, faults: Faults):
        self.faults = faults
        self.calls = 0
        self.done: set[str] = set()         # steps the cycle reports as executed

    def mark(self, step: str) -> None:
        self.done.add(step)

    def complete_json(self, prompt: str, images=None, system: str = "", model: str | None = None):
        self.calls += 1
        meta = Meta(self.name, self.model, 1.0, 0.0, "")
        if "views" in prompt and "label" in prompt:
            return {"views": {n: n for n, _ in (images or [])}, "why": "sim"}, meta
        if "action" in system and "dx_mm" in system:          # LLM-servo request while teaching
            step = self.faults.get("servo_step", 0) + 1
            self.faults.set("servo_step", step)
            if step >= 3:
                self.faults.clear("servo_step")
                return {"action": "done", "confidence": 0.9, "why": "sim: goal reached"}, meta
            return {"action": "move", "dx_mm": 10, "dy_mm": -5, "confidence": 0.8, "why": "sim: approaching"}, meta
        if self.faults.get("judge_unknown"):
            return {"box_present": "unknown", "confidence": 0.1, "notes": "sim: cannot tell"}, meta
        stuck = self.faults.get("flap_stuck")                 # a step whose flap refuses to fold
        f = lambda step, key: ("open" if (stuck == step) else "folded") if step in self.done else "open"  # noqa: E731
        j = {
            "box_present": not self.faults.get("no_box"),
            "short_left": f("fold_short_left", "short_left"), "short_right": f("fold_short_right", "short_right"),
            "long_far": f("fold_long_far", "long_far"), "long_near": f("fold_long_near", "long_near"),
            "seam_closed": "fold_long_near" in self.done and stuck not in ("fold_long_far", "fold_long_near"),
            "tape_on_seam": "tape" in self.done and not self.faults.get("tape_missed"),
            "tape_pressed": "press" in self.done and not self.faults.get("tape_missed"),
            "paddle_held": "pick_paddle" in self.done and not self.faults.get("paddle_dropped"),
            "obstruction": bool(self.faults.get("obstruction")),
            "confidence": 0.9, "notes": "sim",
        }
        return j, meta
