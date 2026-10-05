import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from farm.adapters.sim import Faults, ScriptedHuman  # noqa: E402
from farm.config import load_profile  # noqa: E402
from farm.llm.backends import LLMError, Meta, ScriptedLLM  # noqa: E402
from farm.system import System  # noqa: E402


class ScriptedDecisions:
    name = "scripted-decisions"
    model = "typesafe/jev-test"

    def __init__(self, replies=None, cost=0.0):
        self.replies = list(replies or [])
        self.calls = []
        self.cost = cost

    def decide(self, state, questions):
        self.calls.append({"state": state, "questions": questions})
        if len(self.replies) < len(questions):
            raise LLMError("scripted decisions exhausted")
        answers = {}
        for name in questions:
            reply = self.replies.pop(0)
            answers[name] = {"type": "choice", "confidence": 0.9, **reply}
        return answers, Meta(self.name, self.model, 1.0, self.cost)


class FakeBackends:
    """Backends object whose vision/jev/astra are ScriptedLLMs (no network)."""

    def __init__(self, vision=None, jev=None, astra=None, budget_exhausted=False):
        self.vision = vision or ScriptedLLM()
        self.jev = jev if hasattr(jev, "decide") else ScriptedDecisions(getattr(jev, "replies", []))
        self.astra = astra or ScriptedLLM()
        self._over = budget_exhausted

    def over_budget(self):
        return self._over


GOOD_JUDGEMENT = {"tray_present": True, "opening_visible": True, "water_visible": True, "paper_edge": "dry", "spill": "CLEAR", "obstruction": False, "growth": "seedlings", "confidence": 0.9, "notes": "ok"}
AFTER_JUDGEMENT = {**GOOD_JUDGEMENT, "paper_edge": "wet"}


def jev_reply(choice, p=0.9, choices=("routine", "inspect_water", "inspect_image", "review_machine", "review_hygiene", "unknown")):
    probs = {c: (1 - p) / (len(choices) - 1) for c in choices}
    probs[choice] = p
    return {"choice": choice, "probabilities": probs, "why": "scripted"}


@pytest.fixture
def sim(tmp_path):
    def make(script=None, vision=None, jev=None, faults=None, seed=True, backends=None):
        p = load_profile("sim")
        p.llm.backend = "none"          # tests inject scripted backends; never the sim vision
        p.data_dir = str(tmp_path / "data")
        p.raw["data_dir"] = p.data_dir
        faults = faults or Faults()
        human = ScriptedHuman(script if script is not None else {"pour:": "authorize one pour"})
        b = backends or FakeBackends(vision=vision, jev=jev)
        s = System(p, human=human, faults=faults, backends=b)
        s.connect()
        if seed:
            from farm.cli import _seed_sim_keyframes
            _seed_sim_keyframes(s)
        return s
    return make
