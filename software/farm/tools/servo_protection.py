"""Turn off the STS3215 servo's own temperature protection, in the servo's EEPROM.

The firmware compares its temperature sensor with Max_Temperature_Limit (register 13,
default 70 C) and, when the temperature bit of Unloading_Condition (register 19) is set,
drops torque and raises a fault flag. LED_Alarm_Condition (register 20) uses the same bit
layout for the LED. This tool raises the limit to the register's top value (100 C) and
clears the temperature bit in both masks, so the servo neither unloads nor flags on heat.

EEPROM writes are permanent and need torque off and Lock cleared; every value is read back.
Nothing here moves a joint. Bit layout (Feetech SMS/STS memory table):
bit0 voltage, bit1 sensor, bit2 temperature, bit3 current, bit4 angle, bit5 overload.
"""
from __future__ import annotations

from typing import Any, Callable

TEMPERATURE_BIT = 1 << 2
MAX_TEMPERATURE_LIMIT = 100          # the register's range is 0..100 C
REGISTERS = ("Max_Temperature_Limit", "Unloading_Condition", "LED_Alarm_Condition")


def plan(current: dict[str, int]) -> dict[str, int]:
    """What the three registers should hold, given what they hold now."""
    return {
        "Max_Temperature_Limit": MAX_TEMPERATURE_LIMIT,
        "Unloading_Condition": int(current["Unloading_Condition"]) & ~TEMPERATURE_BIT,
        "LED_Alarm_Condition": int(current["LED_Alarm_Condition"]) & ~TEMPERATURE_BIT,
    }


def done(values: dict[str, int]) -> bool:
    return plan(values) == {k: int(values[k]) for k in REGISTERS}


def read(bus, motor: str) -> dict[str, int]:
    return {r: int(bus.read(r, motor, normalize=False, num_retry=2)) for r in REGISTERS}


def apply(bus, motor: str) -> dict[str, int]:
    """Write the planned values for one motor and return what the servo reads back."""
    target = plan(read(bus, motor))
    bus.write("Torque_Enable", motor, 0, num_retry=2)
    bus.write("Lock", motor, 0, num_retry=2)
    try:
        for r in REGISTERS:
            bus.write(r, motor, target[r], normalize=False, num_retry=2)
    finally:
        bus.write("Lock", motor, 1, num_retry=2)
    after = read(bus, motor)
    if after != target:
        raise RuntimeError(f"{motor}: wrote {target}, servo reads back {after}")
    return after


def run(buses: list[Any], write: bool = False, only: list[str] | None = None,
        out: Callable[[str], None] = print) -> dict[str, Any]:
    """Read (and with write=True, change) every motor on the given connected buses.
    Returns {ok, written, motors: {name: {before, target, after}}}."""
    motors: dict[str, dict[str, Any]] = {}
    ok = True
    out(f"{'motor':<24}{'reg':<22}{'now':>5}{'target':>8}")
    for bus in buses:
        for name in bus.motors:
            if only and name not in only:
                continue
            try:
                before = read(bus, name)
            except Exception as e:  # noqa: BLE001 - report the servo, keep going with the rest
                motors[name] = {"error": str(e)}
                out(f"{name:<24}read failed: {e}")
                ok = False
                continue
            target = plan(before)
            entry: dict[str, Any] = {"before": before, "target": target, "after": None}
            for r in REGISTERS:
                out(f"{name:<24}{r:<22}{before[r]:>5}{target[r]:>8}")
            if write and before != target:
                try:
                    entry["after"] = apply(bus, name)
                    out(f"{name:<24}written and read back")
                except Exception as e:  # noqa: BLE001
                    entry["error"] = str(e)
                    out(f"{name:<24}WRITE FAILED: {e}")
                    ok = False
            elif before == target:
                entry["after"] = before
                out(f"{name:<24}already done")
            motors[name] = entry
    pending = [n for n, m in motors.items() if m.get("after") is None]
    if pending and not write:
        out(f"\n{len(pending)} motor(s) still have temperature protection; run again with --write to change them.")
    elif not pending and ok:
        out("\nALL DONE: no servo on these buses unloads or flags on temperature.")
    return {"ok": ok, "written": bool(write), "motors": motors}
