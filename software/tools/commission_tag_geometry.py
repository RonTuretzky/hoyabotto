"""Read-only AprilTag observations, stationary samples and registration fitting.

The capture command reuses the Gemma pilot's authenticated client. It contains
no enable, move, stop, reset or calibration-write operation.
"""
from __future__ import annotations

import argparse
import base64
import importlib
import json
from pathlib import Path
import sys

from farm.kinematics.tag_registration import fit_registration, assemble_dataset
from farm.perception.gemma_tags import TagObserver, TagRobot
from farm.perception.tag_sampling import stationary_sample


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("observe", "capture"):
        p = sub.add_parser(command)
        p.add_argument("--pilot-root", type=Path, required=True)
        p.add_argument("--geometry", type=Path, required=True)
        p.add_argument("--camera", default="oak")
        p.add_argument("--arm", choices=["left", "right"], default="left")
        p.add_argument("--out", type=Path, required=True)
        if command == "capture":
            p.add_argument("--split", choices=["train", "validation"], required=True)
    p = sub.add_parser("fit")
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("assemble")
    p.add_argument("--captures", type=Path, nargs="+", required=True)
    p.add_argument("--model-directory", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("Choose a new output path; existing evidence is preserved")
    if args.command == "fit":
        result = fit_registration(json.loads(args.dataset.read_text()))
        save(args.out, result)
        print(json.dumps(result, indent=2))
        return 0 if result["status"] == "REGISTRATION_VALIDATED" else 1
    if args.command == "assemble":
        captures = [{"sample": json.loads((p / "sample.json").read_text()),
                     "before": json.loads((p / "before.json").read_text()),
                     "after": json.loads((p / "after.json").read_text()),
                     "arm_geometry_status": json.loads((p / "arm-geometry-status.json").read_text())} for p in args.captures]
        result = assemble_dataset(captures, args.model_directory)
        save(args.out, result)
        print(json.dumps({"samples": len(result["samples"]), "motor_writes": 0, "out": str(args.out)}))
        return 0
    sys.path.insert(0, str(args.pilot_root.resolve()))
    client = importlib.import_module("chat_server").Robot(args.pilot_root / ".private/robot.json")
    observer = TagObserver(geometry=args.geometry)
    robot = TagRobot(client, observer=observer)
    robot.catalog()
    args.out.mkdir(parents=True)
    before = client.call("robot_get_state", {"fresh": True}) if args.command == "capture" else None
    observation = robot.call("robot_get_tags", {"cameras": [args.camera]})
    after = client.call("robot_get_state", {"fresh": True}) if args.command == "capture" else None
    save(args.out / "observation.json", {k: v for k, v in observation.items() if k != "images"})
    for i, image in enumerate(observation.get("images", [])):
        (args.out / f"annotated-{i}.jpg").write_bytes(base64.b64decode(image["data_base64"]))
    if args.command == "capture":
        save(args.out / "before.json", before)
        save(args.out / "after.json", after)
        pose = client.call("robot_get_arm_pose", {"arm": args.arm})
        save(args.out / "arm-geometry-status.json", pose)
        try:
            row = observation.get("result", {}).get("observations", {}).get(args.camera, {})
            sample = stationary_sample(before, after, row, args.arm)
            sample["split"] = args.split
            save(args.out / "sample.json", sample)
        except (ValueError, KeyError, TypeError) as exc:
            save(args.out / "sample-rejected.json", {"reason": str(exc), "motor_writes": 0})
            print(json.dumps({"status": "SAMPLE_REJECTED", "reason": str(exc), "out": str(args.out)}))
            return 1
    print(json.dumps({"ok": observation["ok"], "metric_pose_available": observation.get("result", {}).get("metric_pose_available"),
                      "out": str(args.out), "motor_writes": 0}))
    return 0 if observation["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
