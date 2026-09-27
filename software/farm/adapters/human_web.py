"""Human adapter: questions are posted to the local viewer (and an optional chat
webhook); answers arrive from the viewer's buttons. A person is an input device
with a name and a timestamp. No answer means STALE, never assumed."""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from ..status import Reading, Status

log = logging.getLogger(__name__)


@dataclass
class Question:
    question_id: str
    text: str
    options: list[str]
    evidence: dict[str, Any]
    asked_t: float = field(default_factory=time.time)
    answer: dict[str, Any] | None = None
    event: threading.Event = field(default_factory=threading.Event)


class WebHuman:
    name = "human"

    def __init__(self, webhook: str = ""):
        self.webhook = webhook
        self._q: dict[str, Question] = {}
        self._lock = threading.Lock()
        self.feed: list[dict[str, Any]] = []   # notifications shown in the viewer

    # ---- orchestrator side ----------------------------------------------------
    def ask(self, question_id: str, text: str, options: list[str], evidence: dict[str, Any], timeout_s: float) -> Reading[dict[str, Any]]:
        q = Question(question_id, text, options, evidence)
        with self._lock:
            self._q[question_id] = q
        self.notify(f"QUESTION {question_id}: {text} options={options}", evidence)
        answered = q.event.wait(timeout=timeout_s)
        with self._lock:
            self._q.pop(question_id, None)
        if not answered or q.answer is None:
            return Reading(None, Status.STALE, source=self.name, note=f"no answer within {timeout_s:.0f}s")
        return Reading(q.answer, Status.OK, source=self.name, meta={"question_id": question_id})

    def notify(self, text: str, evidence: dict[str, Any] | None = None) -> None:
        item = {"t": time.time(), "text": text, "evidence": evidence or {}}
        with self._lock:
            self.feed.append(item)
            self.feed = self.feed[-200:]
        if self.webhook:
            try:
                import httpx
                httpx.post(self.webhook, json={"text": text, "evidence": json.loads(json.dumps(evidence or {}, default=str))}, timeout=5)
            except Exception as e:  # noqa: BLE001
                log.warning("webhook failed: %s", e)

    # ---- viewer side ------------------------------------------------------------
    def pending(self) -> list[dict[str, Any]]:
        with self._lock:
            return [{"question_id": q.question_id, "text": q.text, "options": q.options, "evidence": q.evidence, "asked_t": q.asked_t}
                    for q in self._q.values() if q.answer is None]

    def answer(self, question_id: str, choice: str, who: str, note: str = "") -> bool:
        with self._lock:
            q = self._q.get(question_id)
            if q is None or q.answer is not None:
                return False
            if choice not in q.options:
                return False
            if not who.strip():
                return False
            q.answer = {"choice": choice, "who": who.strip(), "note": note, "t": time.time()}
            q.event.set()
        return True
