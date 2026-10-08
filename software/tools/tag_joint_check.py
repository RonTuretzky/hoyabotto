"""Read-only zero/sign check for one arm (docs/tag-registration-slides.html step 6).

Prints the candidate joint degrees for the current encoder ticks (the feetech_degrees_v1 mapping that tag
registration uses: farm.kinematics.tag_registration.candidate_degrees) and, for each of the five positioning
joints, which way the SO-101 model says the gripper moves when that joint alone gets +40 ticks
(farm.kinematics.lerobot.LeRobotSO101.forward, the FK registration uses). Compare each prediction with what the
arm actually does on a +40 tick move, and the angles with a protractor or inclinometer.

No motor access and no robot API calls: ticks come from the command line or a saved robot_get_state reply.

  cd software && PYTHONPATH=. python tools/tag_joint_check.py --model-directory <verified SO-101 dir> \\
      --state right-state.json
  cd software && PYTHONPATH=. python tools/tag_joint_check.py --model-directory <dir> \\
      --calibration farm_xlerobot.json --ticks shoulder_pan=2048 shoulder_lift=1500 elbow_flex=2900 \\
      wrist_flex=2100 wrist_roll=2048

Arm-base frame (SO-101 URDF): +x is forward along the arm at the zero pose, +y is the arm's left, +z is up.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

from farm.kinematics.assets import manifest
from farm.kinematics.tag_registration import candidate_degrees

JOINTS = tuple(manifest()["joint_names"])  # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
WORDS = (("forward", "back"), ("left", "right"), ("up", "down"))
MIN_SHOWN_M = .0005


def describe(delta_m):
    """'2.1 cm (up 1.8 cm, forward 1.1 cm)': base-frame components of at least 0.5 mm, largest first."""
    parts = sorted(((abs(v), WORDS[i][0 if v > 0 else 1]) for i, v in enumerate(delta_m) if abs(v) >= MIN_SHOWN_M), reverse=True)
    if not parts:
        return "under 0.5 mm"
    return f"{np.linalg.norm(delta_m)*100:.1f} cm ({', '.join(f'{w} {m*100:.1f} cm' for m, w in parts)})"


def rotation_deg(before, after):
    r = before[:3, :3].T @ after[:3, :3]
    return math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1))))


def ticks_from_state(payload, arm):
    """Encoder ticks of the arm's five positioning joints from a robot_get_state reply (bare or {'ok','result'})."""
    state = payload.get("result", payload)
    rows = {r["name"]: r for r in state.get("motors", [])} or state.get("rows") or state.get("live_rows") or {}
    out = {}
    for n in JOINTS:
        row = rows.get(f"{arm}_arm_{n}")
        if not row or type(row.get("Present_Position")) not in (int, float):
            raise ValueError(f"State has no Present_Position for {arm}_arm_{n}")
        out[f"{arm}_arm_{n}"] = row["Present_Position"]
    return out


def ranges_from_state(payload):
    ranges = payload.get("result", payload).get("raw_calibration_ranges")
    if not isinstance(ranges, dict):
        raise ValueError("State has no raw_calibration_ranges; pass --calibration")
    return ranges


def ranges_from_calibration(path):
    """LeRobot calibration file (range_min/range_max) -> {name: {min_ticks, max_ticks}}."""
    return {n: {"min_ticks": v["range_min"], "max_ticks": v["range_max"]} for n, v in json.loads(Path(path).read_text()).items()}


def predict(ticks, ranges, solver, arm="right", step=40):
    """Candidate degrees now, and the model's gripper motion when each joint alone moves +step ticks."""
    names = [f"{arm}_arm_{n}" for n in JOINTS]
    missing = [n for n in names if n not in ticks or n not in ranges]
    if missing:
        raise ValueError(f"Need ticks and calibration ranges for {missing}")
    q = candidate_degrees(ticks, ranges, names)
    base = solver.forward(q)
    joints = []
    for i, (short, name) in enumerate(zip(JOINTS, names)):
        row = {"joint": short, "ticks": ticks[name], "candidate_degrees": q[i], "step_ticks": step,
               "range_ticks": [ranges[name]["min_ticks"], ranges[name]["max_ticks"]]}
        try:
            moved = candidate_degrees({**ticks, name: ticks[name]+step}, ranges, names)
            pose = solver.forward(moved)
        except ValueError as exc:
            row.update(predicted=None, note=f"+{step} ticks not evaluated: {exc}")
            joints.append(row)
            continue
        delta = pose[:3, 3]-base[:3, 3]
        row.update(step_degrees=moved[i]-q[i], gripper_delta_m=delta.tolist(), gripper_rotation_deg=rotation_deg(base, pose),
                   predicted=f"{short} +{step} ticks ({moved[i]-q[i]:+.1f} deg) -> gripper moves {describe(delta)}, "
                             f"turns {rotation_deg(base, pose):.1f} deg")
        joints.append(row)
    return {"arm": arm, "mapping": "feetech_degrees_v1 candidate (0 deg = middle of saved range, sign +1; unvalidated)",
            "frame": "arm base (SO-101 URDF): +x forward, +y left, +z up; metres", "joints": joints,
            "gripper_position_m": base[:3, 3].tolist(), "motor_writes": 0}


def parse_ticks(items, arm):
    out = {}
    for item in items:
        name, _, value = item.partition("=")
        name = name if name.startswith(f"{arm}_arm_") else f"{arm}_arm_{name}"
        if name.removeprefix(f"{arm}_arm_") not in JOINTS or not value:
            raise ValueError(f"Expected joint=ticks for one of {', '.join(JOINTS)}; got {item!r}")
        out[name] = int(value)
    return out


def main(argv=None, solver_factory=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=("right", "left"), default="right")
    p.add_argument("--model-directory", type=Path, required=True, help="verified SO-101 assets (python -m carton.servo model-fetch)")
    p.add_argument("--state", type=Path, help="saved robot_get_state JSON (ticks and raw_calibration_ranges)")
    p.add_argument("--ticks", nargs="+", default=[], metavar="JOINT=TICKS", help="current encoder ticks; override --state")
    p.add_argument("--calibration", type=Path, help="LeRobot calibration JSON for the ranges (needed without --state)")
    p.add_argument("--step", type=int, default=40)
    p.add_argument("--json", action="store_true", help="print the full result as JSON")
    a = p.parse_args(argv)
    try:
        payload = json.loads(a.state.read_text()) if a.state else None
        ticks = ticks_from_state(payload, a.arm) if payload else {}
        ticks.update(parse_ticks(a.ticks, a.arm))
        ranges = ranges_from_calibration(a.calibration) if a.calibration else ranges_from_state(payload or {})
        if solver_factory is None:
            from farm.kinematics.lerobot import LeRobotSO101 as solver_factory
        result = predict(ticks, ranges, solver_factory(a.model_directory), a.arm, a.step)
    except (OSError, ValueError, KeyError) as exc:
        print(f"tag_joint_check: {exc}", file=sys.stderr)
        return 1
    if a.json:
        print(json.dumps(result, indent=2))
        return 0
    print(f"{a.arm} arm, {result['mapping']}")
    print(f"frame: {result['frame']}")
    for j in result["joints"]:
        print(f"  {j['joint']:<14} {j['ticks']:>5} ticks  {j['candidate_degrees']:+7.1f} deg  (range {j['range_ticks'][0]}..{j['range_ticks'][1]})")
    print("predicted motion (model only; nothing was moved):")
    for j in result["joints"]:
        print("  " + (j["predicted"] or f"{j['joint']}: {j['note']}"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
