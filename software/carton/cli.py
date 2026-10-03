"""`carton` command line: closing a carton with the parked XLeRobot.

  carton geometry [--setback 0.06 --height 0.15 --spacing 0.30]   reach check for the measured box, no hardware
  carton check        -p carton-v0     connect, judge the box once, print the typed judgement
  carton teach-all    -p carton-v0     the vision model teaches every keyframe the plan needs (no hands)
  carton teach        -p ... --name far_touch     re-teach one keyframe
  carton once         -p carton-v0 [--record]     close one carton; viewer at :8765 with STOP
  carton run          -p carton-v0 [--record]     close a carton, ask for the next one, repeat
  carton sim          [--faults ...]              whole task on the simulator
  carton train        [--steps 8000]              train ACT on the public SO-101 box-closing episodes
"""
from __future__ import annotations

import argparse
import json
import logging
import sys

from farm.config import load_env


def _log():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def _box_and_stance(profile):
    from .geometry import Box, Stance
    c = (profile.raw or {}).get("carton") or {}
    box = Box(length=c.get("length_m", 0.379), width=c.get("width_m", 0.283), height=c.get("height_m", 0.108), flap=c.get("flap_m", 0.14), mass_kg=c.get("mass_kg", 0.96))
    st = c.get("stance") or {}
    stance = Stance(setback=st.get("setback_m", 0.06), spacing=st.get("spacing_m", 0.30), height=st.get("height_m", 0.15), paddle=st.get("paddle_m", 0.15))
    return box, stance


def _system(a):
    from farm.config import load_profile
    from farm.system import System
    p = load_profile(a.profile)
    s = System(p)
    if p.simulated:
        from .sim import SimCartonVision
        s.backends.vision = SimCartonVision(s.faults)
    problems = s.connect()
    if any(x.startswith("robot") for x in problems):
        sys.exit("robot not connected: " + "; ".join(problems))
    return s


def cmd_geometry(a):
    from farm.config import load_profile
    from .geometry import Stance, format_report, reach_report
    box, stance = _box_and_stance(load_profile(a.profile))
    for k in ("setback", "height", "spacing", "paddle"):
        v = getattr(a, k)
        if v is not None:
            setattr(stance, k, v)
    r = reach_report(box, stance)
    print(format_report(r))
    sys.exit(0 if r["all_within_reach"] else 1)


def cmd_check(a):
    _log()
    s = _system(a)
    box, _ = _box_and_stance(s.profile)
    from . import perception
    frames = [(n, c.frame()) for n, c in s.cameras.items() if n == "head"]
    j = perception.judge(s.backends.vision, box, frames, s.store, None)
    print("judgement:", json.dumps(j.value if j.ok else {"status": j.status.value, "note": j.note}, indent=1, default=str))
    print("keyframes taught:", s.keyframes.names())
    s.disconnect()


def _teach_one(s, kf):
    from farm.skills.llm_servo import LLMServo
    servo = LLMServo(s.skills, s.cameras, s.backends.vision, s.store, None, max_steps=s.profile.limits.llm_servo_max_steps)
    out = servo.run(kf.arm, kf.goal, save_as=kf.name, allow_gripper=(kf.arm != "head"))
    print(f"{kf.name}: {'OK' if out.ok else 'FAILED'} in {out.steps} steps, ${out.cost_usd:.3f}: {out.reason}")
    return out.ok


def cmd_teach_all(a):
    _log()
    from farm.viewer.app import serve_in_thread
    from .plan import KEYFRAMES
    s = _system(a)
    serve_in_thread(s, s.profile.viewer_port)
    failed = []
    for kf in KEYFRAMES:
        if kf.name in s.keyframes.names() and not a.force:
            print(f"{kf.name}: already taught, skipping")
            continue
        if not _teach_one(s, kf):
            failed.append(kf.name)
    s.disconnect()
    print("failed:", failed or "none")
    sys.exit(1 if failed else 0)


def cmd_teach(a):
    _log()
    from .plan import keyframe
    s = _system(a)
    ok = _teach_one(s, keyframe(a.name))
    s.disconnect()
    sys.exit(0 if ok else 1)


def _close_one(s, record):
    from .cycle import CartonCycle
    box, _ = _box_and_stance(s.profile)
    if record and getattr(s, "recorder", None) is None:
        s.enable_recording(repo_id="farm/carton-runs")
    out = CartonCycle(s, box).run()
    print(json.dumps({"result": out.result, "note": out.note, "steps_done": out.steps_done}, indent=1))
    return out


def cmd_once(a):
    _log()
    from farm.viewer.app import serve_in_thread
    s = _system(a)
    serve_in_thread(s, s.profile.viewer_port)
    print(f"viewer: http://localhost:{s.profile.viewer_port}")
    try:
        out = _close_one(s, a.record)
    finally:
        s.disconnect()
    sys.exit(0 if out.result == "CLOSED" else 1)


def cmd_run(a):
    _log()
    from farm.status import Status
    from farm.viewer.app import serve_in_thread
    s = _system(a)
    serve_in_thread(s, s.profile.viewer_port)
    print(f"viewer: http://localhost:{s.profile.viewer_port}")
    try:
        while True:
            out = _close_one(s, a.record)
            if out.result == "STOPPED":
                break
            r = s.human.ask("carton:next", "Place the next open carton, then answer.", ["next", "stop"], {"last": out.result}, 3600)
            if r.status is not Status.OK or not r.value or r.value["choice"] != "next":
                break
    finally:
        s.disconnect()


def cmd_sim(a):
    _log()
    from farm.adapters.sim import Faults, ScriptedHuman
    from farm.config import load_profile
    from farm.system import System
    from .sim import SimCartonVision
    faults = Faults()
    for f in (a.faults or []):                      # name, or name=value (e.g. flap_stuck=fold_long_far)
        k, _, v = f.partition("=")
        faults.set(k, v or True)
    s = System(load_profile("carton-sim"), human=ScriptedHuman({"carton:": "done"}) if a.auto_answer else None, faults=faults)
    s.backends.vision = SimCartonVision(s.faults)
    s.connect()
    seed_sim_keyframes(s)
    try:
        out = _close_one(s, a.record)
    finally:
        s.disconnect()
    sys.exit(0 if out.result == "CLOSED" else 1)


def seed_sim_keyframes(s):
    """The simulator has no box to learn; give every plan keyframe a plausible pose so the cycle can run."""
    from farm.adapters.base import HEAD_JOINTS
    from farm.skills.arm import ArmPose
    from .plan import KEYFRAMES
    poses = {"touch": ArmPose(0.22, 0.12), "done": ArmPose(0.18, 0.06), "above": ArmPose(0.18, 0.12), "grip": ArmPose(0.20, 0.08),
             "carry": ArmPose(0.16, 0.15), "over_seam": ArmPose(0.21, 0.10), "down": ArmPose(0.21, 0.07), "press": ArmPose(0.20, 0.07), "rest": ArmPose()}
    for kf in KEYFRAMES:
        if kf.name in s.keyframes.names():
            continue
        if kf.arm == "head":
            s.keyframes.save(kf.name, {HEAD_JOINTS[0]: 0.0, HEAD_JOINTS[1]: -20.0}, "head", "sim seed", "sim")
            continue
        key = next((k for k in poses if k in kf.name), "rest")
        s.keyframes.save(kf.name, s.skills.models[kf.arm].joints_for(poses[key]), kf.arm, "sim seed", "sim")


def cmd_train(a):
    """ACT on the public bimanual SO-100 box-closing episodes (same joint names as our arms)."""
    from pathlib import Path
    from farm.learning import train as T
    root = Path(T.DEFAULT_DATA_TRAIN) / "datasets" / a.repo_id.replace("/", "__")
    if not (root / "meta" / "info.json").is_file():
        from huggingface_hub import snapshot_download
        print("downloading", a.repo_id)
        snapshot_download(a.repo_id, repo_type="dataset", local_dir=str(root))
    out = Path(T.DEFAULT_DATA_TRAIN) / a.output
    cmd = T.build_command(a.repo_id, "act", a.steps, a.device, out, root, None, a.batch_size, a.save_freq, 100, 2, None, None, False, [])
    print(" ".join(cmd))
    rc = T.run(cmd, out.parent / f"{out.name}_train.log")
    if out.is_dir():
        T.parse_loss_log(out.parent / f"{out.name}_train.log", out / "loss.csv")
    sys.exit(rc)


def main(argv=None):
    load_env()
    ap = argparse.ArgumentParser(prog="carton", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn, extra in [
        ("geometry", cmd_geometry, [("--setback", {"type": float}), ("--height", {"type": float}), ("--spacing", {"type": float}), ("--paddle", {"type": float})]),
        ("check", cmd_check, []), ("teach-all", cmd_teach_all, [("--force", {"action": "store_true"})]), ("teach", cmd_teach, [("--name", {"required": True})]),
        ("once", cmd_once, [("--record", {"action": "store_true"})]), ("run", cmd_run, [("--record", {"action": "store_true"})]),
        ("sim", cmd_sim, [("--faults", {"nargs": "*"}), ("--auto-answer", {"action": "store_true"}), ("--record", {"action": "store_true"})]),
        ("train", cmd_train, [("--repo-id", {"default": "yoshikokulala/box_closing3"}), ("--steps", {"type": int, "default": 8000}), ("--device", {"default": "mps"}),
                              ("--output", {"default": "act_box_closing"}), ("--batch-size", {"type": int, "default": 8}), ("--save-freq", {"type": int, "default": 2000})]),
    ]:
        sp = sub.add_parser(name)
        if name not in ("sim", "train"):
            sp.add_argument("-p", "--profile", default="carton-v0")
        for flag, kw in extra:
            sp.add_argument(flag, **kw)
        sp.set_defaults(fn=fn)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
