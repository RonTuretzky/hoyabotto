"""Read a saved calibration and say whether it looks like a complete, sane sweep.

No motion, no connection to the robot: it only reads the JSON that
`farm calibrate` wrote. A bad calibration is otherwise silent until something
moves wrong.

What is checked per joint (encoder is 4096 ticks per turn):
  wrapped    the recorded range touches the encoder edge (0 or 4095): the joint
             was near a stop when "middle" was taken, so the reading wrapped
  short      the swept range is well below what the joint can travel
  too wide   the swept range is more than the joint can travel
  off-centre the "middle" pose was far from the middle of the swept range
  mismatch   the same joint on the left and right arm differ by a lot

Nominal travel comes from the SO-101 design limits (shoulder pan ±110°, shoulder
lift ±100°, elbow ±97°, wrist flex ±95°, wrist roll about 320°, gripper about
110°). Thresholds are loose on purpose: this is a sanity check, not a tolerance.
The head has no published limits; RoboCrew's XLeRobot driver uses pan ±120° and
tilt 0-85°, which is used as the reference.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable

TICKS_PER_TURN = 4096
DEG = 360.0 / TICKS_PER_TURN

# joint suffix -> (nominal travel in degrees, minimum plausible, maximum plausible)
NOMINAL: dict[str, tuple[float, float, float]] = {
    "shoulder_pan": (220, 130, 250),
    "shoulder_lift": (200, 120, 230),
    "elbow_flex": (194, 115, 225),
    "wrist_flex": (190, 110, 220),
    "wrist_roll": (320, 180, 350),
    "gripper": (110, 50, 140),
    "head_motor_1": (240, 90, 330),      # pan
    "head_motor_2": (85, 40, 130),       # tilt
}
EDGE_TICKS = 8            # this close to 0 / 4095 counts as touching the encoder edge
OFF_CENTRE_DEG = 55.0     # middle pose this far from the centre of the sweep is worth a note
MISMATCH_DEG = 30.0       # left vs right arm difference in swept range


def _kind(joint: str) -> str | None:
    if joint in ("head_motor_1", "head_motor_2"):
        return joint
    for k in NOMINAL:
        if joint.endswith(k):
            return k
    return None


def analyse(cal: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Returns {rows: [...], problems: [...], notes: [...]}. Wheels are skipped (full-turn by design)."""
    rows, problems, notes = [], [], []
    ranges: dict[str, float] = {}
    for joint, c in cal.items():
        kind = _kind(joint)
        if kind is None:
            continue
        lo, hi = int(c["range_min"]), int(c["range_max"])
        span = (hi - lo) * DEG
        centre_off = ((lo + hi) / 2 - (TICKS_PER_TURN / 2 - 1)) * DEG
        nominal, lo_ok, hi_ok = NOMINAL[kind]
        flags = []
        if lo <= EDGE_TICKS or hi >= TICKS_PER_TURN - 1 - EDGE_TICKS:
            flags.append("wrapped")
            problems.append(f"{joint}: range touches the encoder edge ({lo}..{hi}). The reading wrapped: switch 12 V off, put this joint at mid-travel, switch on, calibrate again.")
        elif span < lo_ok:
            flags.append("short")
            problems.append(f"{joint}: swept only {span:.0f}° (expected about {nominal:.0f}°). Sweep this joint to both stops and calibrate again.")
        elif span > hi_ok:
            flags.append("too wide")
            problems.append(f"{joint}: swept {span:.0f}°, more than the joint can travel (about {nominal:.0f}°). Check that the right joint was moved and nothing slipped.")
        if "wrapped" not in flags and abs(centre_off) > OFF_CENTRE_DEG:
            flags.append("off-centre")
            notes.append(f"{joint}: the middle pose was {abs(centre_off):.0f}° from the centre of the sweep. Usable, but the next calibration should start nearer mid-travel.")
        ranges[joint] = span
        rows.append({"joint": joint, "id": c.get("id"), "min": lo, "max": hi, "range_deg": span, "nominal_deg": nominal, "centre_off_deg": centre_off, "flags": flags})
    for joint, span in ranges.items():
        if joint.startswith("left_arm_"):
            other = "right_arm_" + joint[len("left_arm_"):]
            if other in ranges and abs(span - ranges[other]) > MISMATCH_DEG:
                problems.append(f"{joint} and {other}: swept {span:.0f}° and {ranges[other]:.0f}°. Identical arms should match; one sweep is incomplete.")
                for r in rows:
                    if r["joint"] in (joint, other) and "mismatch" not in r["flags"]:
                        r["flags"].append("mismatch")
    expected = {"left_arm_" + k for k in list(NOMINAL)[:6]} | {"right_arm_" + k for k in list(NOMINAL)[:6]} | {"head_motor_1", "head_motor_2"}
    missing = sorted(expected - set(cal))
    if missing:
        problems.append("missing from the calibration file: " + ", ".join(missing))
    return {"rows": rows, "problems": problems, "notes": notes}


def report(path: Path, out: Callable[[str], None] = print) -> dict[str, Any]:
    if not path.is_file():
        out(f"No calibration at {path}. Run `farm calibrate` first.")
        return {"rows": [], "problems": [f"no calibration file at {path}"], "notes": [], "ok": False}
    cal = json.loads(path.read_text())
    res = analyse(cal)
    out(f"calibration: {path}")
    out(f"{'joint':<26}{'id':>3}{'min':>7}{'max':>7}{'range':>8}{'expected':>10}  flags")
    for r in res["rows"]:
        out(f"{r['joint']:<26}{r['id'] or '':>3}{r['min']:>7}{r['max']:>7}{r['range_deg']:>7.0f}°{r['nominal_deg']:>9.0f}°  {', '.join(r['flags'])}")
    for n in res["notes"]:
        out("note: " + n)
    out("\n" + ("LOOKS COMPLETE" if not res["problems"] else "PROBLEMS:\n  " + "\n  ".join(res["problems"])))
    res["ok"] = not res["problems"]
    return res


def calibration_path(robot_cfg) -> Path:
    """Where `farm calibrate` saves for this profile (no serial port is opened)."""
    from ..vendor.config_xlerobot_2wheels import XLerobot2WheelsConfig
    from ..vendor.xlerobot_2wheels import XLerobot2Wheels
    rc = XLerobot2WheelsConfig(id=robot_cfg.id, port1=robot_cfg.port1 or "unset1", port2=robot_cfg.port2 or "unset2", cameras={})
    if robot_cfg.calibration_dir:
        rc.calibration_dir = Path(robot_cfg.calibration_dir)
    return Path(XLerobot2Wheels(rc).calibration_fpath)
