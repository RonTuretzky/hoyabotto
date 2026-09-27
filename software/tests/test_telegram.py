"""TelegramHuman and MultiHuman against a fake Bot API served through httpx.MockTransport."""
import json
import threading
import time
from pathlib import Path

import httpx
import numpy as np
import pytest

from farm.adapters.human_multi import MultiHuman
from farm.adapters.human_telegram import TelegramHuman
from farm.adapters.human_web import WebHuman
from farm.status import Status

TOKEN = "123:TEST"
ALLOWED = 4242
STRANGER = 999


class FakeBotAPI:
    """Just enough of api.telegram.org: records calls, hands out updates, blocks briefly on getUpdates."""

    def __init__(self):
        self.calls: list[tuple[str, dict]] = []
        self.photos: list[dict] = []
        self._updates: list[dict] = []
        self._next_update_id = 100
        self._next_message_id = 1
        self._cv = threading.Condition()

    def transport(self) -> httpx.MockTransport:
        return httpx.MockTransport(self.handle)

    # ---- test side ----------------------------------------------------------------
    def messages(self, method="sendMessage") -> list[dict]:
        return [p for m, p in self.calls if m == method]

    def wait_for(self, method: str, n: int = 1, timeout: float = 5.0) -> list[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            got = self.messages(method)
            if len(got) >= n:
                return got
            time.sleep(0.01)
        raise AssertionError(f"{method} not called {n}x; calls={[m for m, _ in self.calls]}")

    def push_callback(self, chat_id: int, data: str, first_name="Ron", username="ron", message_id=1) -> None:
        with self._cv:
            self._updates.append({
                "update_id": self._next_update_id,
                "callback_query": {
                    "id": f"cq{self._next_update_id}",
                    "from": {"id": 7, "first_name": first_name, "username": username},
                    "message": {"message_id": message_id, "chat": {"id": chat_id}},
                    "data": data,
                },
            })
            self._next_update_id += 1
            self._cv.notify_all()

    # ---- server side --------------------------------------------------------------
    def handle(self, request: httpx.Request) -> httpx.Response:
        parts = request.url.path.strip("/").split("/")
        assert parts[0] == f"bot{TOKEN}", request.url
        method = parts[1]
        ctype = request.headers.get("content-type", "")
        if ctype.startswith("multipart/form-data"):
            body = request.read()
            assert b'name="photo"' in body and b"\xff\xd8" in body   # JPEG magic present in the multipart payload
            self.photos.append({"method": method, "size": len(body)})
            self.calls.append((method, {"multipart": True}))
            return httpx.Response(200, json={"ok": True, "result": {"message_id": 0}})
        payload = json.loads(request.read() or b"{}")
        self.calls.append((method, payload))
        if method == "sendMessage":
            mid = self._next_message_id
            self._next_message_id += 1
            return httpx.Response(200, json={"ok": True, "result": {"message_id": mid, "chat": {"id": payload["chat_id"]}}})
        if method == "getUpdates":
            offset = payload.get("offset")
            with self._cv:
                if not self._updates:
                    self._cv.wait(timeout=0.05)     # emulate long poll without spinning the test CPU
                out = [u for u in self._updates if offset is None or u["update_id"] >= offset]
                # Telegram forgets updates once a higher offset is confirmed.
                if offset is not None:
                    self._updates = [u for u in self._updates if u["update_id"] >= offset]
            return httpx.Response(200, json={"ok": True, "result": out})
        if method in ("answerCallbackQuery", "editMessageText", "editMessageReplyMarkup"):
            return httpx.Response(200, json={"ok": True, "result": True})
        return httpx.Response(200, json={"ok": False, "description": f"unknown method {method}"})


@pytest.fixture
def api():
    return FakeBotAPI()


@pytest.fixture
def tg(api, tmp_path):
    t = TelegramHuman(token=TOKEN, chat_ids={ALLOWED}, image_dir=tmp_path, client=httpx.Client(transport=api.transport()), poll_timeout_s=1)
    yield t
    t.stop()


def _ask_in_thread(human, qid, options, timeout_s, evidence=None):
    box = {}

    def go():
        box["r"] = human.ask(qid, "Pour 30 ml into tray B?", options, evidence or {}, timeout_s)

    th = threading.Thread(target=go, daemon=True)
    th.start()
    return th, box


def test_question_sent_with_keyboard_and_callback_resolves(api, tg):
    qid = "act_0123456789abcdef_pour_tray_B_long_identifier"
    th, box = _ask_in_thread(tg, qid, ["authorize one pour", "skip"], 5.0)
    msg = api.wait_for("sendMessage")[0]
    assert msg["chat_id"] == ALLOWED and qid in msg["text"]
    rows = msg["reply_markup"]["inline_keyboard"]
    assert [r[0]["text"] for r in rows] == ["authorize one pour", "skip"]
    for r in rows:
        assert len(r[0]["callback_data"].encode()) < 64
    tok, idx = rows[1][0]["callback_data"].split("|")
    assert idx == "1"
    assert tg.pending() and tg.pending()[0]["question_id"] == qid

    api.push_callback(ALLOWED, f"{tok}|1")
    th.join(timeout=5)
    assert not th.is_alive()
    r = box["r"]
    assert r.status is Status.OK
    assert r.value["choice"] == "skip" and r.value["who"] == "Ron @ron" and r.value["t"] > 0
    api.wait_for("answerCallbackQuery")
    edits = api.wait_for("editMessageText")
    assert "Ron" in edits[-1]["text"] and "skip" in edits[-1]["text"]
    assert edits[-1]["message_id"] == 1 and edits[-1]["chat_id"] == ALLOWED
    assert tg.pending() == []


def test_callback_from_unlisted_chat_is_ignored_then_stale(api, tg, caplog):
    th, box = _ask_in_thread(tg, "q1", ["yes", "no"], 1.0)
    msg = api.wait_for("sendMessage")[0]
    tok = msg["reply_markup"]["inline_keyboard"][0][0]["callback_data"].split("|")[0]
    with caplog.at_level("WARNING", logger="farm.adapters.human_telegram"):
        api.push_callback(STRANGER, f"{tok}|0", first_name="Mallory", username="mal")
        time.sleep(0.3)
    assert th.is_alive(), "a stranger's press must not resolve the question"
    assert api.messages("answerCallbackQuery") == []
    assert any("unlisted chat" in rec.message for rec in caplog.records)
    th.join(timeout=3)
    assert not th.is_alive()
    assert box["r"].status is Status.STALE and box["r"].value is None
    assert "no answer" in box["r"].note


def test_evidence_photos_sent_as_multipart(api, tg, tmp_path):
    import cv2
    img = np.zeros((16, 16, 3), np.uint8)
    for h in ("aaa", "bbb", "ccc"):
        cv2.imwrite(str(tmp_path / f"{h}.jpg"), img)
    tg.notify("hello", {"frames": {"head": "aaa", "wrist": "bbb", "third": "ccc", "missing": "zzz"}})
    assert len(api.photos) == 2, "at most two evidence images per message"
    assert api.messages("sendMessage")[-1]["text"] == "hello"


def test_network_errors_do_not_crash_caller(tmp_path):
    def boom(request):
        raise httpx.ConnectError("no network")

    tg = TelegramHuman(token=TOKEN, chat_ids={ALLOWED}, client=httpx.Client(transport=httpx.MockTransport(boom)), poll_timeout_s=1)
    try:
        tg.notify("still fine")
        r = tg.ask("q", "text", ["a"], {}, 0.2)
        assert r.status is Status.STALE
    finally:
        tg.stop()


def test_multi_human_first_answer_wins_and_releases_others(api, tg):
    web = WebHuman()
    multi = MultiHuman(web, tg)
    th, box = _ask_in_thread(multi, "q-multi", ["yes", "no"], 5.0)
    api.wait_for("sendMessage")
    deadline = time.time() + 2
    while not web.pending() and time.time() < deadline:
        time.sleep(0.01)
    assert web.answer("q-multi", "yes", "Ron (viewer)")
    th.join(timeout=5)
    assert not th.is_alive(), "MultiHuman must return as soon as one channel answers"
    r = box["r"]
    assert r.ok and r.value["choice"] == "yes" and r.value["who"] == "Ron (viewer)"
    assert r.meta["channel_type"] == "WebHuman"
    time.sleep(0.2)
    assert tg.pending() == [] and web.pending() == []
    edits = api.wait_for("editMessageText")
    assert "answered elsewhere" in edits[-1]["text"]


def test_multi_human_telegram_wins_and_unblocks_web(api, tg):
    web = WebHuman()
    multi = MultiHuman(web, tg)
    th, box = _ask_in_thread(multi, "q-multi-2", ["yes", "no"], 5.0)
    msg = api.wait_for("sendMessage")[0]
    tok = msg["reply_markup"]["inline_keyboard"][0][0]["callback_data"].split("|")[0]
    api.push_callback(ALLOWED, f"{tok}|0")
    th.join(timeout=5)
    assert not th.is_alive()
    r = box["r"]
    assert r.ok and r.value["choice"] == "yes" and r.value["who"] == "Ron @ron" and r.meta["channel_type"] == "TelegramHuman"
    time.sleep(0.2)
    assert web.pending() == []


def test_multi_human_all_silent_is_stale(api, tg):
    web = WebHuman()
    multi = MultiHuman(web, tg)
    t0 = time.time()
    r = multi.ask("q-silent", "anyone?", ["yes"], {}, 0.3)
    assert r.status is Status.STALE and time.time() - t0 < 3
    assert multi.pending() == []
    multi.notify("fan out")
    assert web.feed[-1]["text"] == "fan out" and api.messages("sendMessage")[-1]["text"] == "fan out"
    assert multi.feed[-1]["text"] == "fan out"
