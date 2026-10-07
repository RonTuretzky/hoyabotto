"""Hold-to-run trigger from the Mac Joy-Con reader (tools/mac-joycon-reader, JSON Lines contract v1).

Input only: this never talks to motors. `is_held()` is true only while the chosen button is pressed on a
live, connected controller AND the reader's frames are fresh. Anything else counts as released:
a stale stream, a disconnect or shutdown frame, the reader exiting, demo frames, or unreadable lines.
"""
from __future__ import annotations

import json
import subprocess
import threading
import time
from typing import Any, Callable, Iterable


class JoyConHold:
    def __init__(self, button: str, stale_s: float = 0.3, allow_demo: bool = False,
                 clock: Callable[[], float] = time.monotonic):
        self.button = button.strip().lower()
        self.stale_s = stale_s
        self.allow_demo = allow_demo
        self.clock = clock
        self._held = False
        self._last = float("-inf")
        self.ended = False
        self.seen_buttons: set[str] = set()
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()

    # ---- parsing ---------------------------------------------------------------
    def _matches(self, name: str, info: dict[str, Any]) -> bool:
        names = [name, *(info.get("physical_names") or [])]
        return any(str(n).strip().lower() == self.button for n in names)

    def feed(self, line: str) -> None:
        try:
            frame = json.loads(line)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(frame, dict) or frame.get("input_only") is not True:
            return
        with self._lock:
            if frame.get("source") != "live" and not self.allow_demo:
                self._held = False
                return
            self._last = self.clock()
            if frame.get("event") in ("disconnect", "shutdown"):
                self._held = False
                if frame.get("event") == "shutdown":
                    self.ended = True
                return
            held = False
            for c in frame.get("controllers") or []:
                if not c.get("connected"):
                    continue
                for name, info in (c.get("buttons") or {}).items():
                    self.seen_buttons.add(name)
                    if isinstance(info, dict) and self._matches(name, info) and info.get("pressed") is True:
                        held = True
            self._held = held

    def feed_all(self, lines: Iterable[str]) -> None:
        for line in lines:
            self.feed(line)

    def is_held(self) -> bool:
        with self._lock:
            return self._held and not self.ended and self.clock() - self._last <= self.stale_s

    # ---- running the reader -------------------------------------------------------
    def start(self, command: list[str]) -> None:
        """Run the reader (e.g. `.../run.command --json --hz 30`) and follow its stdout in a thread."""
        self._proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

        def follow():
            assert self._proc and self._proc.stdout
            for line in self._proc.stdout:
                self.feed(line)
            with self._lock:
                self.ended = True
                self._held = False

        threading.Thread(target=follow, daemon=True).start()

    def close(self) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
