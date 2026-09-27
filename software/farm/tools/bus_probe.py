"""Find out which USB serial port is which Feetech bus without guessing from
/dev names (they reorder on every replug). Each port is opened with an empty
motor table and broadcast-pinged; the set of motor IDs that answers tells us:

  bus1 (left arm + head)     IDs 1..8   (6 arm joints + 2 head motors)
  bus2 (right arm + wheels)  IDs 9..    (6 arm joints + wheel motors)

    python -m farm.tools.bus_probe /dev/tty.usbmodem58*  /dev/tty.usbmodem59*

Errors are reported per port so one unplugged cable does not hide the other bus.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from typing import Any, Callable

log = logging.getLogger(__name__)

BUS1_IDS = set(range(1, 9))
BUS2_MARKERS = {9, 10}


def guess_bus(ids: set[int]) -> str:
    if ids == BUS1_IDS:
        return "bus1 (left arm + head)"
    if ids >= BUS2_MARKERS:
        return "bus2 (right arm + wheels)"
    return "unknown"


def _model_name(model_number: int) -> str:
    try:
        from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE
        for name, num in MODEL_NUMBER_TABLE.items():
            if num == model_number:
                return name
    except Exception:  # noqa: BLE001
        pass
    return f"model#{model_number}"


def _default_bus(port: str) -> Any:
    from lerobot.motors.feetech import FeetechMotorsBus
    bus = FeetechMotorsBus(port, motors={})
    bus.connect(handshake=False)   # nothing to handshake against: we do not know the motors yet
    return bus


def probe_ports(ports: list[str], bus_factory: Callable[[str], Any] | None = None) -> list[dict[str, Any]]:
    """For each port: {port, ids, models, guess} or {port, error}."""
    make = bus_factory or _default_bus
    out: list[dict[str, Any]] = []
    for port in ports:
        bus = None
        try:
            bus = make(port)
            found = bus.broadcast_ping(num_retry=2)
            if found is None:
                out.append({"port": port, "error": "broadcast ping got no response (no motors powered, or wrong bus)"})
                continue
            ids = sorted(int(i) for i in found)
            out.append({
                "port": port,
                "ids": ids,
                "models": {int(i): _model_name(int(m)) for i, m in found.items()},
                "guess": guess_bus(set(ids)),
            })
        except Exception as e:  # noqa: BLE001
            log.debug("probe %s failed", port, exc_info=True)
            out.append({"port": port, "error": f"{type(e).__name__}: {e}"})
        finally:
            if bus is not None:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception:  # noqa: BLE001
                    pass
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("ports", nargs="*", help="serial ports; default: every USB serial port found")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    ports = a.ports
    if not ports:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports() if p.vid is not None]
    if not ports:
        print("no serial ports found", file=sys.stderr)
        return 2
    results = probe_ports(ports)
    if a.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            if "error" in r:
                print(f"{r['port']}: ERROR {r['error']}")
            else:
                models = ", ".join(f"{i}:{m}" for i, m in r["models"].items())
                print(f"{r['port']}: {r['guess']}  ids={r['ids']}  [{models}]")
    return 0 if all("error" not in r for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
