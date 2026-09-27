"""Bench monitor for the ESP32 light sensor: one line per second so a person can
watch lux, status and sequence gaps while waving a hand over the sensor, plus
the same 10-sample measure() the care cycle uses every 10 s. Nothing is
printed as a bare number: every lux value carries its Status.

    python -m farm.tools.light_monitor [--port /dev/tty.usbserial-XXX] [--baud 115200] [--seconds 60]
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from ..adapters.esp32_serial import ESP32Light
from ..config import LightCfg
from ..status import Reading, Status

SUMMARY_EVERY_S = 10.0
SUMMARY_SAMPLES = 10


def format_latest(r: Reading[float]) -> str:
    lux = f"{r.value:8.1f}" if r.value is not None else "     ---"
    gap = "GAP" if r.meta.get("gap") else "   "
    seq = f"{r.seq}" if r.seq is not None else "-"
    return f"lux={lux} status={r.status.value:<8} seq={seq:>6} {gap} {r.note}".rstrip()


def format_summary(r: Reading[dict]) -> str:
    if r.value is None:
        return f"measure: status={r.status.value} {r.note}"
    v = r.value
    return (f"measure: status={r.status.value} median={v['median_lux']:.1f} spread={v['spread_lux']:.1f} "
            f"n={v['n']} errors={v['errors']} saturated={v['saturated']}")


def run(port: str | None, baud: int, seconds: float | None, out=sys.stdout) -> int:
    cfg = LightCfg(enabled=True, port=port or "", baud=baud)
    light = ESP32Light(cfg)
    try:
        light.connect()
    except Exception as e:  # noqa: BLE001
        print(f"connect failed: {e}", file=out)
        return 2
    print(f"ESP32 light on {light.port} @ {baud} baud; Ctrl-C to stop", file=out)
    t0 = time.time()
    next_line = t0
    next_summary = t0 + SUMMARY_EVERY_S
    try:
        while seconds is None or time.time() - t0 < seconds:
            now = time.time()
            if now >= next_line:
                print(f"{now - t0:7.1f}s  {format_latest(light.latest())}", file=out, flush=True)
                next_line += 1.0
            if now >= next_summary:
                r = light.measure(samples=SUMMARY_SAMPLES, settle_s=0.0)
                print(f"{time.time() - t0:7.1f}s  {format_summary(r)}", file=out, flush=True)
                next_summary = time.time() + SUMMARY_EVERY_S
                next_line = max(next_line, time.time())
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped", file=out)
    finally:
        light.disconnect()
    last = light.latest()
    return 0 if last.status in (Status.OK, Status.STALE) else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=None, help="serial port; default auto-detects by USB VID")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=None, help="stop after this many seconds (default: run until Ctrl-C)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    return run(a.port, a.baud, a.seconds)


if __name__ == "__main__":
    sys.exit(main())
