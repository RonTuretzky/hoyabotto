"""Human adapter over the Telegram Bot API: questions become inline keyboards,
button presses become answers. Same contract as WebHuman: a person is an input
device with a name and a timestamp; silence is STALE, never an assumption.

Why long polling and not a webhook: the robot sits behind home NAT and must
keep working when nobody has a tunnel up. One daemon thread calls getUpdates
with a 20 s server-side timeout and hands every callback_query to `answer()`.
Network errors are logged and retried with backoff; they never reach the
orchestrator, whose only signal is the Reading it gets back from `ask()`.

Only chats listed in TELEGRAM_CHAT_IDS may answer. A press from any other chat
is logged and dropped, so a leaked bot username cannot authorize a pour.
"""
from __future__ import annotations

import itertools
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any

import httpx

from ..status import Reading, Status
from .human_web import Question

log = logging.getLogger(__name__)

TELEGRAM_API = "https://api.telegram.org"
MAX_EVIDENCE_PHOTOS = 2
POLL_TIMEOUT_S = 20
BACKOFF_MAX_S = 30.0


def _parse_chat_ids(raw: str | None) -> set[int]:
    ids: set[int] = set()
    for part in (raw or "").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            ids.add(int(part))
        except ValueError:
            log.warning("TELEGRAM_CHAT_IDS: ignoring non-integer entry %r", part)
    return ids


class TelegramHuman:
    name = "human"

    def __init__(
        self,
        token: str | None = None,
        chat_ids: set[int] | None = None,
        image_dir: Path | None = None,
        base_url: str = TELEGRAM_API,
        client: httpx.Client | None = None,
        poll_timeout_s: int = POLL_TIMEOUT_S,
    ):
        self.token = token if token is not None else os.environ.get("TELEGRAM_BOT_TOKEN", "")
        self.chat_ids = chat_ids if chat_ids is not None else _parse_chat_ids(os.environ.get("TELEGRAM_CHAT_IDS"))
        if not self.token:
            raise RuntimeError("TELEGRAM_BOT_TOKEN is not set")
        if not self.chat_ids:
            raise RuntimeError("TELEGRAM_CHAT_IDS is empty: nobody would be allowed to answer")
        self.image_dir = image_dir
        self._base = f"{base_url.rstrip('/')}/bot{self.token}"
        self._client = client or httpx.Client(timeout=poll_timeout_s + 10)
        self._poll_timeout_s = poll_timeout_s
        self._q: dict[str, Question] = {}
        self._token_to_qid: dict[str, str] = {}        # short callback token -> question_id
        self._messages: dict[str, list[tuple[int, int, str]]] = {}   # question_id -> [(chat_id, message_id, text)]
        self._counter = itertools.count(1)
        self._lock = threading.Lock()
        self.feed: list[dict[str, Any]] = []
        self._offset: int | None = None
        self._run = False
        self._thread: threading.Thread | None = None

    # ---- lifecycle --------------------------------------------------------------
    def start(self) -> None:
        with self._lock:
            if self._run:
                return
            self._run = True
            self._thread = threading.Thread(target=self._poll_loop, name="telegram-poll", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._run = False
        t = self._thread
        if t is not None:
            t.join(timeout=self._poll_timeout_s + 5)
        self._thread = None

    def connect(self) -> None:
        self.start()

    def disconnect(self) -> None:
        self.stop()

    # ---- orchestrator side ----------------------------------------------------
    def ask(self, question_id: str, text: str, options: list[str], evidence: dict[str, Any], timeout_s: float) -> Reading[dict[str, Any]]:
        self.start()
        q = Question(question_id, text, options, evidence)
        # callback_data must stay under 64 bytes; question ids are long, so map a short token.
        tok = f"q{next(self._counter):x}"
        with self._lock:
            self._q[question_id] = q
            self._token_to_qid[tok] = question_id
            self._messages[question_id] = []
        keyboard = {"inline_keyboard": [[{"text": opt, "callback_data": f"{tok}|{i}"}] for i, opt in enumerate(options)]}
        body = f"QUESTION {question_id}\n{text}"
        for chat in sorted(self.chat_ids):
            self._send_photos(chat, evidence)
            msg = self._call("sendMessage", {"chat_id": chat, "text": body, "reply_markup": keyboard})
            if msg and isinstance(msg.get("message_id"), int):
                with self._lock:
                    self._messages[question_id].append((chat, msg["message_id"], body))
        with self._lock:
            self.feed.append({"t": time.time(), "text": body, "evidence": evidence or {}})
            self.feed = self.feed[-200:]
        answered = q.event.wait(timeout=timeout_s)
        with self._lock:
            self._q.pop(question_id, None)
            self._token_to_qid.pop(tok, None)
            sent = self._messages.pop(question_id, [])
        if not answered or q.answer is None:
            for chat, mid, orig in sent:
                self._call("editMessageText", {"chat_id": chat, "message_id": mid, "text": f"{orig}\n\n(no answer within {timeout_s:.0f}s; the robot did not act)"})
            return Reading(None, Status.STALE, source=self.name, note=f"no answer within {timeout_s:.0f}s")
        return Reading(q.answer, Status.OK, source=self.name, meta={"question_id": question_id})

    def notify(self, text: str, evidence: dict[str, Any] | None = None) -> None:
        with self._lock:
            self.feed.append({"t": time.time(), "text": text, "evidence": evidence or {}})
            self.feed = self.feed[-200:]
        for chat in sorted(self.chat_ids):
            self._send_photos(chat, evidence)
            self._call("sendMessage", {"chat_id": chat, "text": text})

    # ---- viewer / poller side ---------------------------------------------------
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
            sent = list(self._messages.get(question_id, []))
        for chat, mid, orig in sent:
            self._call("editMessageText", {"chat_id": chat, "message_id": mid, "text": f"{orig}\n\nAnswered: {choice} - {who.strip()}" + (f" ({note})" if note else "")})
        return True

    # ---- Telegram plumbing --------------------------------------------------------
    def _call(self, method: str, payload: dict[str, Any], timeout: float | None = None) -> dict[str, Any] | None:
        """POST one Bot API method. Returns the `result` dict/list or None; never raises."""
        try:
            r = self._client.post(f"{self._base}/{method}", json=payload, timeout=timeout if timeout is not None else 15)
            data = r.json()
        except Exception as e:  # noqa: BLE001
            log.warning("telegram %s failed: %s", method, e)
            return None
        if not data.get("ok"):
            log.warning("telegram %s rejected: %s", method, data.get("description", data))
            return None
        return data.get("result")

    def _send_photos(self, chat: int, evidence: dict[str, Any] | None) -> None:
        if not evidence or self.image_dir is None:
            return
        frames = evidence.get("frames")
        if not isinstance(frames, dict):
            return
        for cam, h in list(frames.items())[:MAX_EVIDENCE_PHOTOS]:
            if not isinstance(h, str):
                continue
            path = self.image_dir / f"{h}.jpg"
            if not path.exists():
                log.warning("evidence image missing for %s: %s", cam, path)
                continue
            try:
                with path.open("rb") as f:
                    r = self._client.post(f"{self._base}/sendPhoto", data={"chat_id": str(chat), "caption": cam},
                                          files={"photo": (path.name, f, "image/jpeg")}, timeout=30)
                if not r.json().get("ok"):
                    log.warning("telegram sendPhoto rejected: %s", r.text[:200])
            except Exception as e:  # noqa: BLE001
                log.warning("telegram sendPhoto failed: %s", e)

    def _poll_loop(self) -> None:
        backoff = 1.0
        while self._run:
            payload: dict[str, Any] = {"timeout": self._poll_timeout_s, "allowed_updates": ["callback_query"]}
            if self._offset is not None:
                payload["offset"] = self._offset
            try:
                r = self._client.post(f"{self._base}/getUpdates", json=payload, timeout=self._poll_timeout_s + 10)
                data = r.json()
                if not data.get("ok"):
                    raise RuntimeError(data.get("description", "getUpdates not ok"))
            except Exception as e:  # noqa: BLE001
                if self._run:
                    log.warning("telegram getUpdates failed (%s); retrying in %.0fs", e, backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, BACKOFF_MAX_S)
                continue
            backoff = 1.0
            for upd in data.get("result") or []:
                uid = upd.get("update_id")
                if isinstance(uid, int):
                    self._offset = uid + 1
                cq = upd.get("callback_query")
                if cq:
                    try:
                        self._handle_callback(cq)
                    except Exception as e:  # noqa: BLE001
                        log.warning("telegram callback handling failed: %s", e)

    def _handle_callback(self, cq: dict[str, Any]) -> None:
        cq_id = cq.get("id")
        chat_id = ((cq.get("message") or {}).get("chat") or {}).get("id")
        frm = cq.get("from") or {}
        if chat_id not in self.chat_ids:
            log.warning("ignoring callback from unlisted chat %s (user %s)", chat_id, frm.get("username") or frm.get("id"))
            return
        data = cq.get("data") or ""
        tok, _, idx_s = data.partition("|")
        with self._lock:
            qid = self._token_to_qid.get(tok)
            q = self._q.get(qid) if qid else None
        if q is None:
            self._call("answerCallbackQuery", {"callback_query_id": cq_id, "text": "That question has expired."})
            return
        try:
            choice = q.options[int(idx_s)]
        except (ValueError, IndexError):
            self._call("answerCallbackQuery", {"callback_query_id": cq_id, "text": "Unknown option."})
            return
        who = (frm.get("first_name") or "").strip()
        if frm.get("username"):
            who = f"{who} @{frm['username']}".strip()
        if not who:
            who = f"tg:{frm.get('id', '?')}"
        if self.answer(q.question_id, choice, who):
            self._call("answerCallbackQuery", {"callback_query_id": cq_id, "text": f"Recorded: {choice}"})
        else:
            self._call("answerCallbackQuery", {"callback_query_id": cq_id, "text": "Already answered."})
