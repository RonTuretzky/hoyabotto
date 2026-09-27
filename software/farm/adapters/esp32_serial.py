"""ESP32 light-sensor adapter: one JSON line per reading over USB serial.

Firmware (firmware/esp32_light/esp32_light.ino) prints at ~5 Hz:
  {"seq":812,"t_ms":93410,"lux":412.5,"ok":true}
  {"seq":813,"t_ms":93610,"lux":null,"ok":false,"err":"i2c"}
A gap in seq or silence longer than max_age_s becomes STALE on this side;
ok=false becomes INVALID. Nothing in this file invents a number.
"""
from __future__ import annotations

import json
import logging
import statistics
import threading
import time
from typing import Any

from ..config import LightCfg
from ..status import Reading, Status, invalid, not_applicable

log = logging.getLogger(__name__)

BH1750_SATURATION_LUX = 54612.0  # high-res mode ceiling at default MTreg
USB_SERIAL_VIDS = {0x10C4: "CP210x", 0x1A86: "CH340", 0x303A: "Espressif", 0x0403: "FTDI"}


def find_esp32_port() -> str | None:
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid in USB_SERIAL_VIDS and "usbmodem" not in (p.device or "") or (p.vid in USB_SERIAL_VIDS and "wchusbserial" in (p.device or "")):
            return p.device
    for p in list_ports.comports():
        if p.vid in USB_SERIAL_VIDS:
            return p.device
    return None


class ESP32Light:
    name = "light"

    def __init__(self, cfg: LightCfg):
        self.cfg = cfg
        self._ser = None
        self._lock = threading.Lock()
        self._last: Reading[float] | None = None
        self._last_seq: int | None = None
        self._gap = False
        self._run = False
        self._thread: threading.Thread | None = None
        self.port: str | None = None

    def connect(self) -> None:
        if not self.cfg.enabled:
            return
        import serial
        self.port = self.cfg.port or find_esp32_port()
        if not self.port:
            raise RuntimeError("ESP32 not found: set light.port in the profile or plug the data cable into the hub")
        self._ser = serial.Serial(self.port, self.cfg.baud, timeout=1)
        time.sleep(0.3)
        self._ser.reset_input_buffer()
        self._run = True
        self._thread = threading.Thread(target=self._loop, name="esp32", daemon=True)
        self._thread.start()
        log.info("ESP32 light sensor on %s", self.port)

    def _loop(self) -> None:
        while self._run:
            try:
                line = self._ser.readline().decode("utf-8", "replace").strip()
            except Exception as e:  # noqa: BLE001
                with self._lock:
                    self._last = invalid(self.name, f"serial error: {e}")
                time.sleep(0.5)
                continue
            if not line.startswith("{"):
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            seq = d.get("seq")
            gap = self._last_seq is not None and isinstance(seq, int) and seq != self._last_seq + 1
            self._last_seq = seq if isinstance(seq, int) else self._last_seq
            if d.get("ok") and isinstance(d.get("lux"), (int, float)):
                r = Reading(float(d["lux"]), Status.OK, t=time.time(), seq=seq, source=self.name, meta={"t_ms": d.get("t_ms"), "gap": gap})
            else:
                r = Reading(None, Status.INVALID, t=time.time(), seq=seq, source=self.name, note=f"sensor error: {d.get('err', 'ok=false')}")
            with self._lock:
                self._last = r

    def latest(self) -> Reading[float]:
        if not self.cfg.enabled:
            return not_applicable(self.name)
        with self._lock:
            r = self._last
        if r is None:
            return invalid(self.name, "no sample yet")
        return r.aged(self.cfg.max_age_s)

    def measure(self, samples: int | None = None, settle_s: float | None = None) -> Reading[dict[str, Any]]:
        """Collect `samples` distinct fresh conversions after the arm has settled."""
        if not self.cfg.enabled:
            return not_applicable(self.name)
        n = samples or self.cfg.samples
        time.sleep(settle_s if settle_s is not None else self.cfg.settle_s)
        vals: list[float] = []
        seqs: set[int] = set()
        errors = 0
        deadline = time.time() + max(3.0, n * 0.4)
        while len(vals) < n and time.time() < deadline:
            r = self.latest()
            if r.status is Status.INVALID:
                errors += 1
            elif r.ok and r.seq not in seqs:
                seqs.add(r.seq if r.seq is not None else len(vals))
                vals.append(r.value)
            time.sleep(0.05)
        if len(vals) < max(3, n // 2):
            return invalid(self.name, f"only {len(vals)}/{n} fresh conversions (errors={errors})")
        med = statistics.median(vals)
        spread = (max(vals) - min(vals))
        sat = max(vals) >= BH1750_SATURATION_LUX * 0.98
        status = Status.INVALID if sat else Status.OK
        return Reading({"median_lux": med, "spread_lux": spread, "n": len(vals), "errors": errors, "saturated": sat, "samples": vals},
                       status, source=self.name, note="saturated" if sat else "")

    def disconnect(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
