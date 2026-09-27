"""Fan a question out to several human channels (viewer, Telegram, ...) and take
the first OK answer. The person who happens to be nearest their phone or the
laptop decides; nobody has to be at a particular screen.

Each channel's `ask` blocks on its own Event, so every channel runs in its own
thread. When one wins, the others are unblocked by calling their `answer()`
with note="answered elsewhere" so they release immediately instead of waiting
out the deadline. If every channel comes back non-OK the result is STALE.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Any

from ..status import Reading, Status

log = logging.getLogger(__name__)


class MultiHuman:
    name = "human"

    def __init__(self, *channels: Any):
        if not channels:
            raise ValueError("MultiHuman needs at least one channel")
        self.channels = list(channels)

    @property
    def feed(self) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        for ch in self.channels:
            items.extend(getattr(ch, "feed", []) or [])
        items.sort(key=lambda i: i.get("t", 0))
        return items[-200:]

    def ask(self, question_id: str, text: str, options: list[str], evidence: dict[str, Any], timeout_s: float) -> Reading[dict[str, Any]]:
        results: queue.Queue[tuple[Any, Reading[dict[str, Any]]]] = queue.Queue()

        def worker(ch: Any) -> None:
            try:
                r = ch.ask(question_id, text, options, evidence, timeout_s)
            except Exception as e:  # noqa: BLE001
                log.warning("%s.ask failed: %s", getattr(ch, "name", ch), e)
                r = Reading(None, Status.INVALID, source=getattr(ch, "name", "human"), note=f"ask failed: {e}")
            results.put((ch, r))

        threads = [threading.Thread(target=worker, args=(ch,), name=f"ask-{i}", daemon=True) for i, ch in enumerate(self.channels)]
        for t in threads:
            t.start()

        deadline = time.time() + timeout_s + 2.0   # channels enforce timeout_s themselves; the grace only covers scheduling
        winner: tuple[Any, Reading[dict[str, Any]]] | None = None
        outcomes: list[Reading[dict[str, Any]]] = []
        while len(outcomes) < len(self.channels):
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            try:
                ch, r = results.get(timeout=remaining)
            except queue.Empty:
                break
            outcomes.append(r)
            if r.ok and r.value is not None:
                winner = (ch, r)
                break

        if winner is not None:
            ch, r = winner
            ans = r.value
            for other in self.channels:
                if other is ch:
                    continue
                try:
                    other.answer(question_id, ans["choice"], ans["who"], note="answered elsewhere")
                except Exception as e:  # noqa: BLE001
                    log.warning("%s.answer (release) failed: %s", getattr(other, "name", other), e)
            for t in threads:
                t.join(timeout=2.0)
            meta = dict(r.meta)
            meta["channel"] = getattr(ch, "name", type(ch).__name__)
            meta["channel_type"] = type(ch).__name__
            return Reading(ans, Status.OK, t=r.t, source=self.name, meta=meta)

        notes = "; ".join(f"{o.source}: {o.note or o.status.value}" for o in outcomes) or "no channel replied"
        return Reading(None, Status.STALE, source=self.name, note=f"no answer within {timeout_s:.0f}s ({notes})")

    def notify(self, text: str, evidence: dict[str, Any] | None = None) -> None:
        for ch in self.channels:
            try:
                ch.notify(text, evidence)
            except Exception as e:  # noqa: BLE001
                log.warning("%s.notify failed: %s", getattr(ch, "name", ch), e)

    def pending(self) -> list[dict[str, Any]]:
        seen: dict[str, dict[str, Any]] = {}
        for ch in self.channels:
            fn = getattr(ch, "pending", None)
            if fn is None:
                continue
            for item in fn():
                seen.setdefault(item["question_id"], item)
        return list(seen.values())

    def answer(self, question_id: str, choice: str, who: str, note: str = "") -> bool:
        accepted = False
        for ch in self.channels:
            fn = getattr(ch, "answer", None)
            if fn is None:
                continue
            try:
                accepted = fn(question_id, choice, who, note) or accepted
            except Exception as e:  # noqa: BLE001
                log.warning("%s.answer failed: %s", getattr(ch, "name", ch), e)
        return accepted
