"""Fakes behind the same adapter interfaces, for the simulator and the test suite.

FakeRobot moves joints toward goals at a bounded rate. FakeCamera renders a
synthetic tray scene (or replays saved frames) with a capture time. FakeLight
follows a script. Faults are injected by name so every failure path has a test.
"""
from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from ..status import Reading, Status, invalid, not_applicable
from .base import ARM_JOINTS, HEAD_JOINTS, arm_joint

ALL_JOINTS = [arm_joint(a, j) for a in ("left", "right") for j in ARM_JOINTS] + HEAD_JOINTS


class Faults:
    """Shared fault switchboard. Set flags from tests: faults.set('camera_stale')."""

    def __init__(self):
        self._f: dict[str, Any] = {}
        self._lock = threading.Lock()

    def set(self, name: str, value: Any = True) -> None:
        with self._lock:
            self._f[name] = value

    def clear(self, name: str | None = None) -> None:
        with self._lock:
            if name is None:
                self._f.clear()
            else:
                self._f.pop(name, None)

    def get(self, name: str, default: Any = None) -> Any:
        with self._lock:
            return self._f.get(name, default)


class FakeRobot:
    name = "robot"

    def __init__(self, faults: Faults | None = None, rate_per_s: float = 120.0):
        self.faults = faults or Faults()
        self.rate = rate_per_s
        self.pos = {j: 0.0 for j in ALL_JOINTS}
        self.goal = dict(self.pos)
        self.temp = {j: 35.0 for j in ALL_JOINTS}
        self.load = {j: 50.0 for j in ALL_JOINTS}
        self._t = time.time()
        self._connected = False
        self.calibration_id = "sim"
        self.torque = {j: True for j in ALL_JOINTS}
        self.sent: list[dict[str, float]] = []

    def connect(self) -> None:
        if self.faults.get("robot_connect_fail"):
            raise RuntimeError("simulated: motor bus not found")
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def _step(self) -> None:
        now = time.time(); dt = max(0.0, now - self._t); self._t = now
        for j in ALL_JOINTS:
            d = self.goal[j] - self.pos[j]
            step = self.rate * dt
            self.pos[j] += max(-step, min(step, d))

    def joints(self) -> Reading[dict[str, float]]:
        if not self._connected or self.faults.get("robot_disconnect"):
            return invalid(self.name, "simulated: bus disconnected")
        self._step()
        return Reading(dict(self.pos), Status.OK, source=self.name)

    def health(self) -> Reading[dict[str, dict[str, float]]]:
        if not self._connected:
            return invalid(self.name, "not connected")
        hot = self.faults.get("servo_overtemp")
        out = {j: {"temperature": (70.0 if hot else self.temp[j]), "load": self.load[j]} for j in ALL_JOINTS}
        return Reading(out, Status.OK, source=self.name)

    def move_to(self, targets: dict[str, float], max_step: float | None = None) -> Reading[dict[str, float]]:
        if not self._connected or self.faults.get("robot_disconnect"):
            return invalid(self.name, "simulated: bus disconnected")
        self._step()
        sent = {}
        for j, v in targets.items():
            if j not in self.pos:
                continue
            v = float(v)
            if max_step is not None:
                v = self.pos[j] + max(-max_step, min(max_step, v - self.pos[j]))
            lo, hi = (0.0, 100.0) if j.endswith("gripper") else (-100.0, 100.0)
            if j.endswith("gripper") and not self.faults.get("empty_gripper"):
                lo = self.faults.get("held_width", 32.0)   # something is in the gripper: fingers stop on it
            self.goal[j] = max(lo, min(hi, v)); sent[j] = self.goal[j]
        self.sent.append(sent)
        return Reading(sent, Status.OK, source=self.name)

    def stop(self) -> None:
        self._step(); self.goal = dict(self.pos)

    def torque_off(self, motors: list[str] | None = None) -> None:
        for j in (motors or ALL_JOINTS):
            self.torque[j] = False

    def torque_on(self, motors: list[str] | None = None) -> None:
        for j in (motors or ALL_JOINTS):
            self.torque[j] = True


class FakeCamera:
    name = "camera"

    def __init__(self, name: str, faults: Faults | None = None, scene: dict[str, Any] | None = None):
        self.name = name
        self.faults = faults or Faults()
        self.scene = scene or {}
        self.boot_frame = None
        self._connected = False

    def connect(self) -> None:
        if self.faults.get(f"camera_missing:{self.name}") or self.faults.get("camera_missing"):
            raise RuntimeError(f"simulated: camera {self.name} unplugged")
        self._connected = True
        self.boot_frame = self._render()

    def disconnect(self) -> None:
        self._connected = False

    def _render(self) -> np.ndarray:
        """A synthetic tray: grey table, a dark rectangle (tray), a blue patch (water) unless 'dry',
        a brown blotch (spill) when requested, green speckle proportional to 'growth'.
        If faults['replay_dir'] points at a folder of JPEG/PNG photos, those are replayed instead (round-robin),
        so real station photos can drive the whole program before the robot exists."""
        rd = self.faults.get("replay_dir")
        if rd:
            import cv2
            from pathlib import Path as _P
            files = sorted([f for f in _P(rd).glob("*") if f.suffix.lower() in (".jpg", ".jpeg", ".png")])
            if files:
                self._replay_i = getattr(self, "_replay_i", -1) + 1
                img = cv2.imread(str(files[self._replay_i % len(files)]))
                if img is not None:
                    return cv2.cvtColor(cv2.resize(img, (640, 480)), cv2.COLOR_BGR2RGB)
        h, w = 480, 640
        img = np.full((h, w, 3), 150, np.uint8)
        wet = not self.faults.get("tray_dry")
        img[140:340, 180:460] = (60, 60, 60)                          # tray body
        img[300:330, 200:440] = (40, 90, 200) if wet else (90, 80, 70)  # water reserve visible band
        g = float(self.faults.get("growth", self.scene.get("growth", 0.3)))
        rng = np.random.default_rng(int(g * 1000))
        n = int(g * 4000)
        ys = rng.integers(160, 290, n); xs = rng.integers(200, 440, n)
        img[ys, xs] = (40, 160, 50)
        if self.faults.get("spill"):
            img[340:380, 300:420] = (70, 60, 40)
        if self.faults.get("occluded"):
            img[:, : w // 2] = (20, 20, 20)
        if self.faults.get("dark"):
            img = (img * 0.08).astype(np.uint8)
        return img

    def frame(self) -> Reading[np.ndarray]:
        if not self._connected or self.faults.get(f"camera_missing:{self.name}"):
            return invalid(self.name, "simulated: camera unplugged")
        t = time.time() - (5.0 if self.faults.get("camera_stale") else 0.0)
        r = Reading(self._render(), Status.OK, t=t, source=self.name)
        return r.aged(1.0)


class FakeLight:
    name = "light"

    def __init__(self, faults: Faults | None = None, lux: float = 420.0, enabled: bool = True):
        self.faults = faults or Faults()
        self.lux = lux
        self.enabled = enabled
        self._seq = 0

    def connect(self) -> None:
        if self.faults.get("esp32_missing"):
            raise RuntimeError("simulated: ESP32 not found")

    def disconnect(self) -> None: ...

    def latest(self) -> Reading[float]:
        if not self.enabled:
            return not_applicable(self.name)
        if self.faults.get("esp32_silent"):
            return Reading(self.lux, Status.STALE, t=time.time() - 3, source=self.name, note="simulated serial silence")
        if self.faults.get("esp32_error"):
            return invalid(self.name, "simulated i2c error")
        self._seq += 1
        return Reading(float(self.lux + np.random.default_rng(self._seq).normal(0, 3)), Status.OK, seq=self._seq, source=self.name)

    def measure(self, samples: int = 10, settle_s: float = 0.0) -> Reading[dict[str, Any]]:
        if not self.enabled:
            return not_applicable(self.name)
        vals = []
        for _ in range(samples):
            r = self.latest()
            if r.status is not Status.OK:
                return invalid(self.name, r.note)
            vals.append(r.value)
        return Reading({"median_lux": float(np.median(vals)), "spread_lux": float(max(vals) - min(vals)), "n": len(vals), "errors": 0, "saturated": False, "samples": vals}, Status.OK, source=self.name)


class ScriptedHuman:
    """Answers questions from a script: {question_id_prefix: choice} or a callable. Unscripted -> STALE (silence)."""
    name = "human"

    def __init__(self, script: dict[str, str] | None = None, who: str = "sim-operator"):
        self.script = script or {}
        self.who = who
        self.log: list[dict[str, Any]] = []
        self.feed: list[dict[str, Any]] = []

    def ask(self, question_id: str, text: str, options: list[str], evidence: dict[str, Any], timeout_s: float) -> Reading[dict[str, Any]]:
        self.log.append({"question_id": question_id, "text": text, "options": options})
        for k, v in self.script.items():
            if question_id.startswith(k) and v in options:
                return Reading({"choice": v, "who": self.who, "note": "scripted", "t": time.time()}, Status.OK, source=self.name)
        return Reading(None, Status.STALE, source=self.name, note="scripted silence")

    def notify(self, text: str, evidence: dict[str, Any] | None = None) -> None:
        self.feed.append({"t": time.time(), "text": text, "evidence": evidence or {}})

    def pending(self) -> list[dict[str, Any]]:
        return []

    def answer(self, *a, **k) -> bool:
        return False


class SimVision:
    """A stand-in for the vision model in the simulator: judges the synthetic scene from the fault switchboard,
    in the same typed JSON the real model returns. Never used outside simulated profiles."""
    name = "sim-vision"
    model = "sim-vision"

    def __init__(self, faults: Faults):
        self.faults = faults
        self.calls = 0

    def complete_json(self, prompt: str, images=None, system: str = "", model: str | None = None):
        from ..llm.backends import Meta
        self.calls += 1
        if "views" in prompt and "label" in prompt:
            return {"views": {n: n for n, _ in (images or [])}, "why": "sim"}, Meta(self.name, self.model, 1.0, 0.0, "")
        if "action" in system and "dx_mm" in system:   # servo request: walk toward the goal, then finish
            step = self.faults.get("servo_step", 0) + 1
            self.faults.set("servo_step", step)
            if step >= 3:
                self.faults.clear("servo_step")
                return {"action": "done", "confidence": 0.9, "why": "sim: goal reached"}, Meta(self.name, self.model, 1.0, 0.0, "")
            return {"action": "move", "dx_mm": 10, "dy_mm": -5, "confidence": 0.8, "why": "sim: approaching"}, Meta(self.name, self.model, 1.0, 0.0, "")
        occluded = bool(self.faults.get("occluded") or self.faults.get("dark"))
        poured = bool(self.faults.get("poured"))
        j = {
            "tray_present": True, "opening_visible": not occluded, "water_visible": (not self.faults.get("tray_dry")) or poured,
            "paper_edge": "unknown" if occluded else ("wet" if poured else ("dry" if self.faults.get("paper_dry", True) else "damp")),
            "spill": "UNKNOWN" if occluded else ("SPILL" if self.faults.get("spill") else "CLEAR"),
            "obstruction": "unknown" if occluded else bool(self.faults.get("obstruction")),
            "growth": "seedlings", "confidence": 0.9, "notes": "sim judgement",
        }
        if "after" in prompt.lower() or self.faults.get("pour_started"):
            j["paper_edge"] = "wet" if not occluded else "unknown"
        return j, Meta(self.name, self.model, 1.0, 0.0, "")
