"""Move both arms to the fold policy's training start pose (replaces posing the released arms by hand, step 8b).

Dry-run by default: reads the owner (robot_get_execution), prints current vs target ticks per joint and the planned,
collision-checked sequence, sends nothing. --execute --operator NAME moves the arms (both must be enabled and
holding), one arm and one leg at a time with blocking robot_move_joint_targets, verifies every leg and ends holding.
It never calls robot_stop. See docs/auto-start-pose.md and carton/fold_start_pose.py.

Robot (chat Mac):
  python tools/move_to_start_pose.py --pilot-root "$PILOT" --scene <fold trial dir or scene.xml> \\
      [--joint-map left=... --joint-map right=...] [--execute --operator NAME]
Simulation (FakeOwner running the deployed owner code over the MuJoCo fold scene; no robot):
  python tools/move_to_start_pose.py --sim <fold trial dir> --sim-start random:3 --execute --operator sim
"""
from __future__ import annotations

import argparse
import importlib
import math
import os
import signal
import sys
import tempfile
import threading
from pathlib import Path

SOFTWARE = Path(__file__).resolve().parents[1]
if str(SOFTWARE) not in sys.path:
    sys.path.insert(0, str(SOFTWARE))

import numpy as np  # noqa: E402

from carton import fold_start_pose as S  # noqa: E402
from carton.fold_policy_runner import (ARMS, OWNER_JOINTS, POLICY_JOINTS, TrainingEnvelope, _clean,  # noqa: E402
                                       load_arm_maps)
from carton.servo.common import Refused, atomic_json  # noqa: E402

DEFAULT_MAPS = {arm: SOFTWARE / f"profiles/fold-joint-maps/{arm}-joint-map.json" for arm in ARMS}


def _pairs(values):
    out = {}
    for v in values or []:
        if "=" not in v:
            raise SystemExit(f"expected ARM=PATH, got {v!r}")
        k, p = v.split("=", 1)
        out[k] = p
    return out


def scene_path(value) -> Path:
    p = Path(value)
    return p / "run/scene.xml" if p.is_dir() else p


def load_pose(args) -> S.StartPose:
    if args.dataset:
        return S.start_pose_from_dataset(args.dataset)
    if args.demo:
        return S.start_pose_from_demo(args.demo)
    return S.builtin_start_pose()


def print_pose(pose: S.StartPose, maps, out=print):
    ticks = S.pose_ticks(pose, maps)
    out(f"Training start pose from {pose.source}" + (f" ({pose.episodes} episodes agree)" if pose.episodes else ""))
    out(f"  {'joint':22s} {'rad':>9s} {'deg':>8s} {'ticks':>6s}")
    for i, (policy_name, owner_name) in enumerate(zip(POLICY_JOINTS, OWNER_JOINTS)):
        out(f"  {policy_name:22s} {pose.rad[i]:9.4f} {math.degrees(pose.rad[i]):8.2f} {ticks[owner_name]:6d}")
    builtin = np.asarray(S.TRAINING_START_RAD)
    diff = float(np.abs(np.asarray(pose.rad) - builtin).max())
    if diff > math.radians(0.1):
        out(f"  NOTE: differs from the built-in training start pose by up to {math.degrees(diff):.2f} deg")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], epilog=__doc__.split("\n\n", 1)[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pilot-root", type=Path, help="chat pilot checkout with chat_server.Robot and .private/robot.json")
    ap.add_argument("--joint-map", action="append", metavar="ARM=PATH",
                    help="per-arm joint map (default: profiles/fold-joint-maps/; bind them to the live calibration)")
    ap.add_argument("--scene", help="MuJoCo fold training scene: a fold-demos trial dir or its run/scene.xml")
    ap.add_argument("--carton", choices=["nominal", "absent"], default="nominal",
                    help="'absent' only when the operator has removed the carton from the station")
    ap.add_argument("--clearance-mm", type=float, default=S.DEFAULT_CLEARANCE_M * 1000,
                    help=f"minimum clearance to the station, carton and other arm (>= {S.MIN_CLEARANCE_M * 1000:.0f})")
    ap.add_argument("--demo", help="take the start pose from this demonstration (trial dir or demo.npz)")
    ap.add_argument("--dataset", help="take the start pose from frame 0 of this LeRobot training dataset")
    ap.add_argument("--checkpoint", help="pretrained_model dir: refuse if the pose lies outside its training state range")
    ap.add_argument("--execute", action="store_true", help="move the arms (default: dry-run, nothing sent)")
    ap.add_argument("--operator", help="name of the person holding STOP (required with --execute)")
    ap.add_argument("--speed-ticks-s", type=float, default=S.DEFAULT_SPEED_TICKS_S,
                    help=f"per-leg speed (<= {S.MAX_SPEED_TICKS_S:.0f}, the owner's ramp)")
    ap.add_argument("--no-jaws", action="store_true", help="leave the jaws where they are")
    ap.add_argument("--out", type=Path, help="write plan and summary.json here")
    ap.add_argument("--parent-pid", type=int, help="stop before the next leg if this parent process exits")
    ap.add_argument("--sim", help="simulation: fold-demos trial dir (FakeOwner + MuJoCo); no robot")
    ap.add_argument("--sim-start", default="random:0",
                    help="simulation start: random:SEED (clear random pose), training (already there)")
    ap.add_argument("--sim-workdir", type=Path)
    args = ap.parse_args(argv)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stopped = stop.is_set if not args.parent_pid else (lambda: stop.is_set() or os.getppid() != args.parent_pid)
    config = S.MoverConfig(execute=args.execute, operator=args.operator, speed_ticks_s=args.speed_ticks_s,
                           move_jaws=not args.no_jaws)
    rig = None
    try:
        config.validate()
        envelope = TrainingEnvelope.from_pretrained_dir(args.checkpoint) if args.checkpoint else None
        pose = load_pose(args)
        if args.sim:
            from carton import fold_policy_fakes as F
            workdir = args.sim_workdir or Path(tempfile.mkdtemp(prefix="start-pose-sim-")) / "rig"
            rig = F.build_sim_rig(Path(args.sim), workdir, owner=True)
            maps = rig.arm_maps
            checker = S.SceneChecker(scene_path(args.scene or args.sim), maps, clearance_m=args.clearance_mm / 1000,
                                     carton=args.carton == "nominal")
            ranges = {n: (rig.calibration[n]["range_min"], rig.calibration[n]["range_max"]) for n in OWNER_JOINTS}
            if args.sim_start.startswith("random:"):
                start = S.random_safe_start(checker, maps, ranges, np.random.default_rng(int(args.sim_start[7:])))
                S.place_sim_arms(rig, start)
            elif args.sim_start != "training":
                raise SystemExit("--sim-start must be random:SEED or training")
            if args.execute:
                rig.enable_all()   # the operator's step on the robot
            robot, clock = rig.owner, rig.clock
        else:
            maps = load_arm_maps(_pairs(args.joint_map) or DEFAULT_MAPS)
            checker = (S.SceneChecker(scene_path(args.scene), maps, clearance_m=args.clearance_mm / 1000,
                                      carton=args.carton == "nominal") if args.scene else None)
            if args.pilot_root is None:
                raise SystemExit("--pilot-root is required (the robot API client), or use --sim")
            sys.path.insert(0, str(args.pilot_root.resolve()))
            robot = importlib.import_module("chat_server").Robot(args.pilot_root / ".private/robot.json")
            import time
            clock = time.time
    except Refused as exc:
        print(f"REFUSED: {exc}")
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            atomic_json(args.out / "summary.json", {"execute": args.execute, "aborted": f"refused before motion: {exc}",
                                                    "legs_sent": 0, "at_start_pose": False})
        return 1
    print_pose(pose, maps)
    mover = S.StartPoseMover(robot, maps, pose, checker, config, clock=clock, stop_requested=stopped, log=print,
                             envelope=envelope)
    summary = mover.run()
    if rig is not None:
        summary["simulation"] = rig.score()
        sim = summary["simulation"]
        print("Simulation: owner faults", sim.get("owner_faults"), "| stop_count", sim.get("owner_stop_count"),
              "| robot-carton penetration mm", sim.get("max_robot_flap_penetration_mm"),
              "| other penetration mm", sim.get("max_robot_other_penetration_mm"),
              "| carton moved mm", sim.get("max_carton_translation_mm"))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        atomic_json(args.out / "summary.json", _clean(summary))
    if summary["aborted"]:
        return 1
    if args.execute and not summary.get("at_start_pose"):
        return 2      # arms there, a jaw stopped short (reported, not retried)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
