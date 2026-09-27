"""Camera adapter over LeRobot's OpenCVCamera with a capture timestamp on every frame.

Identity: on Linux, `match` is a substring of the device name/path reported by
OpenCVCamera.find_cameras(); on macOS device names are generic, so the profile
sets `index_or_path` explicitly and `farm devices` shows a snapshot per index
so the assignment can be checked. At connect we also keep a boot snapshot so
the viewer (and the VLM view check) can catch a swapped camera.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

import numpy as np

from ..config import CameraCfg
from ..status import Reading, Status, invalid

log = logging.getLogger(__name__)


def list_cameras() -> list[dict[str, Any]]:
    from lerobot.cameras.opencv import OpenCVCamera
    try:
        return OpenCVCamera.find_cameras()
    except Exception as e:  # noqa: BLE001
        log.warning("find_cameras failed: %s", e)
        return []


class OpenCVCameraAdapter:
    def __init__(self, cfg: CameraCfg):
        self.cfg = cfg
        self.name = cfg.name
        self._cam = None
        self._last: Reading[np.ndarray] | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._run = False
        self.boot_frame: np.ndarray | None = None

    def _resolve(self):
        if self.cfg.index_or_path is not None:
            v = self.cfg.index_or_path
            return int(v) if isinstance(v, str) and v.isdigit() else v
        if self.cfg.match:
            for c in list_cameras():
                if self.cfg.match.lower() in str(c.get("name", "")).lower() or self.cfg.match in str(c.get("id", "")):
                    return c["id"]
        raise RuntimeError(f"camera {self.name}: no device matches match={self.cfg.match!r}; set index_or_path in the profile")

    def connect(self) -> None:
        from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
        ident = self._resolve()
        self._cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=ident, fps=self.cfg.fps, width=self.cfg.width, height=self.cfg.height))
        self._cam.connect()
        self.boot_frame = self._cam.read()
        self._run = True
        self._thread = threading.Thread(target=self._loop, name=f"cam-{self.name}", daemon=True)
        self._thread.start()
        log.info("camera %s connected on %r", self.name, ident)

    def _loop(self) -> None:
        while self._run:
            try:
                f = self._cam.read()
                r = Reading(f, Status.OK, t=time.time(), source=self.name)
            except Exception as e:  # noqa: BLE001
                r = invalid(self.name, f"read failed: {e}")
                time.sleep(0.2)
            with self._lock:
                self._last = r

    def frame(self) -> Reading[np.ndarray]:
        with self._lock:
            r = self._last
        if r is None:
            return invalid(self.name, "no frame yet")
        return r.aged(self.cfg.max_age_s)

    def disconnect(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._cam is not None:
            try:
                self._cam.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._cam = None
