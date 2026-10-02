"""Soak test: hold a pose and log every servo's temperature and load until the time is up
or a servo reaches the ceiling. Answers "how hot does a held pose get, and is it still rising?"
before the robot is left alone.

The idea is from the XLeRobot-Pro measurement protocols (hold a pose under load, log per-servo
telemetry, abort at the ceiling); no code is shared.

The caller puts the robot in the pose (a taught keyframe, or wherever it already is) and decides
what to do afterwards. This module only reads, writes the CSV and says when to stop.
"""
from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, Callable


def run(robot, minutes: float, csv_path: Path, interval_s: float = 2.0, temp_max_c: float = 55.0,
        out: Callable[[str], None] = print, sleep: Callable[[float], None] = time.sleep,
        now: Callable[[], float] = time.time, should_stop: Callable[[], bool] = lambda: False) -> dict[str, Any]:
    """Returns {reason, minutes, samples, peak_c, peak_joint, slope_c_per_min, minutes_to_ceiling, bad_reads, csv}."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    t0 = now()
    series: dict[str, list[tuple[float, float]]] = {}
    samples = bad = 0
    reason = "time up"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "joint", "temperature_c", "load"])
        while True:
            t = now() - t0
            h = robot.health()
            if not h.ok:
                bad += 1
                if bad >= 5:
                    reason = f"health read failed 5 times: {h.note}"
                    break
            else:
                bad = 0
                samples += 1
                for joint, v in h.value.items():
                    temp = float(v.get("temperature", float("nan")))
                    w.writerow([f"{t:.1f}", joint, f"{temp:.1f}", f"{float(v.get('load', 0)):.0f}"])
                    series.setdefault(joint, []).append((t, temp))
                f.flush()
                hot_joint, hot = max(((j, s[-1][1]) for j, s in series.items()), key=lambda x: x[1])
                if samples == 1 or samples % 15 == 0:
                    out(f"{t / 60:5.1f} min  hottest {hot_joint} {hot:.0f} C")
                if hot >= temp_max_c:
                    reason = f"{hot_joint} reached {hot:.0f} C (ceiling {temp_max_c:.0f} C)"
                    break
            if should_stop():
                reason = "stopped"
                break
            if t >= minutes * 60:
                break
            sleep(interval_s)
    elapsed = (now() - t0) / 60
    res: dict[str, Any] = {"reason": reason, "minutes": round(elapsed, 2), "samples": samples, "csv": str(csv_path), "bad_reads": bad,
                           "peak_c": None, "peak_joint": None, "slope_c_per_min": None, "minutes_to_ceiling": None}
    if series:
        peak_joint, peak = max(((j, max(v for _, v in s)) for j, s in series.items()), key=lambda x: x[1])
        s = series[peak_joint]
        tail = s[len(s) * 2 // 3:] if len(s) >= 6 else s          # slope over the last third: is it still rising at the end?
        slope = 0.0
        if len(tail) >= 2 and tail[-1][0] > tail[0][0]:
            slope = (tail[-1][1] - tail[0][1]) / ((tail[-1][0] - tail[0][0]) / 60)
        res.update(peak_c=round(peak, 1), peak_joint=peak_joint, slope_c_per_min=round(slope, 2))
        if slope > 0.05 and s[-1][1] < temp_max_c:
            res["minutes_to_ceiling"] = round((temp_max_c - s[-1][1]) / slope, 1)
    return res


def summary(res: dict[str, Any], temp_max_c: float = 55.0) -> str:
    if res["peak_c"] is None:
        return f"no temperature samples ({res['reason']})"
    line = f"ended: {res['reason']} after {res['minutes']:.1f} min. Peak {res['peak_c']:.0f} C on {res['peak_joint']}."
    if res["slope_c_per_min"] is not None and res["slope_c_per_min"] > 0.05:
        line += f" Still rising {res['slope_c_per_min']:.2f} C/min at the end"
        line += f"; at that rate the {temp_max_c:.0f} C ceiling is about {res['minutes_to_ceiling']:.0f} min away." if res["minutes_to_ceiling"] is not None else "."
    else:
        line += " Level at the end."
    return line + f" Log: {res['csv']}"
