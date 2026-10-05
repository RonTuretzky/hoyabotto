"""CLI preparation, camera audit, guarded experiments, and evidence reports."""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from .common import Limits, Refused, Trace, atomic_json, binding, camera_names, read_json, validate_config
from .controller import Experiment
from .geometry import hinge_path, verify_grasp
from .transport import SessionTransport
from .vision import ManifestCamera, ManifestDepthCamera, Observer


def load_config(path):
    path = Path(path).resolve()
    c = read_json(path)
    for key in ("session_dir", "calibration_file"):
        c[key] = str((path.parent / c[key]).resolve())
    for cam in c["cameras"].values():
        for key in ("manifest", "reference"):
            cam[key] = str((path.parent / cam[key]).resolve())
    return c


def prepare(a):
    """Read files and snapshots only. No device connections or command writes."""
    folder = Path(a.out).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "experiment.json"
    if path.exists():
        raise Refused("Existing experiment.json preserved; use a new output directory")
    calibration = Path(a.calibration).resolve()
    cal = read_json(calibration)
    suffixes = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
    ranges = {f"{a.arm}_arm_{n}": [cal[f"{a.arm}_arm_{n}"]["range_min"], cal[f"{a.arm}_arm_{n}"]["range_max"]] for n in suffixes}
    cameras = {}
    pair = getattr(a, 'camera_pair', 'head_wrist')
    for name in sorted(camera_names(dict(arm=a.arm, camera_pair=pair))):
        base = getattr(a, 'oak_frames', None) if name == 'oak' else None
        manifest = Path(base or a.frames).resolve() / f"{name}.json"
        m = read_json(manifest)
        frame = (ManifestDepthCamera if name=='oak' else ManifestCamera)(manifest, m.get("camera_id")).read()
        ref = folder / f"{name}-reference.png"
        if not cv2.imwrite(str(ref), frame.image):
            raise Refused("Cannot save seed image")
        cameras[name] = {"camera_id": frame.camera_id, "manifest": str(manifest),
                         "reference": str(ref), "regions": {}}
    c = {"schema": 1, "units": "encoder_ticks", "arm": a.arm,
         "session_dir": str(Path(a.session).resolve()), "calibration_file": str(calibration),
         "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
         "joints": [f"{a.arm}_arm_{n}" for n in suffixes[:4]], "ranges": ranges,
         "cameras": cameras, "measurements": [], "target": [0, 0, 0, 0],
         "notes": "Select tool/target/anchor patches, then inspect. Targets are image-feature goals, not a verified grasp."}
    for camera in cameras:
        for axis in (0, 1):
            c["measurements"].append({"name": f"{camera}_{'xy'[axis]}", "camera": camera,
                                      "a": "target", "b": "tool", "axis": axis})
    if pair != 'head_wrist':
        c['camera_pair'] = pair
    atomic_json(path, c)
    return {"status": "NEEDS_VISUAL_SEEDS", "config": str(path), "motor_writes": 0}


def seed(a):
    path = Path(a.config).resolve()
    c = load_config(path)
    cam = c["cameras"][a.camera]
    for value in a.region:
        name, raw = value.split(":", 1)
        roi = [int(v) for v in raw.split(",")]
        if len(roi) != 4:
            raise Refused("Region format: name:x,y,width,height")
        cam["regions"][name] = {"type": "patch", "roi": roi, "anchor": name == "anchor"}
    for value in a.tag:
        name, raw = value.split(":", 1)
        cam["regions"][name] = {"type": "apriltag", "tag_id": int(raw), "anchor": name == "anchor",
                                "min_edge_px": a.min_tag_edge_px}
    for value in a.point:
        name, raw = value.split(":", 1)
        point = [float(v) for v in raw.split(",")]
        if len(point) != 2:
            raise Refused("Point format: name:x,y")
        cam["regions"][name]["point"] = point
    if a.target:
        c["target"] = [float(v) for v in a.target.split(",")]
    # Validate the actual patches/tags now, before replacing the config.
    from .features import FeatureTracks
    im = cv2.imread(cam["reference"])
    if im is None:
        raise Refused("Seed image is unreadable")
    FeatureTracks(im, cam["regions"]).locate(im)
    atomic_json(path, c)
    return {"status": "SEEDS_SAVED", "camera": a.camera, "regions": list(cam["regions"]), "motor_writes": 0}


def camera_check(a):
    if not 2 <= a.seconds <= 60:
        raise Refused("Camera audit duration must be 2–60 seconds")
    readers = {}
    pair = getattr(a, 'camera_pair', 'head_wrist')
    for name in sorted(camera_names(dict(arm=a.arm, camera_pair=pair))):
        base = getattr(a, 'oak_frames', None) if name == 'oak' else None
        path = Path(base or a.frames) / f"{name}.json"
        m = read_json(path)
        readers[name] = (ManifestDepthCamera if name=='oak' else ManifestCamera)(path, m.get("camera_id"))
    if len({r.camera_id for r in readers.values()}) != 2:
        raise Refused("Registered cameras must be distinct devices")
    start = time.monotonic()
    samples = {n: [] for n in readers}
    last = {}
    skews = []
    while time.monotonic()-start < a.seconds:
        frames = {n: c.read() for n, c in readers.items()}
        for name, f in frames.items():
            if f.seq != last.get(name):
                samples[name].append({"seq": f.seq, "stamp": f.stamp, "age_s": time.time()-f.stamp})
                last[name] = f.seq
        skews.append(max(f.stamp for f in frames.values())-min(f.stamp for f in frames.values()))
        time.sleep(.02)
    summary = {}
    for name, rows in samples.items():
        if len(rows) < 3:
            raise Refused(f"{name}: too few distinct capture frames")
        dt = np.diff([r["stamp"] for r in rows])
        summary[name] = {"frames": len(rows), "measured_fps": (len(rows)-1)/(rows[-1]["stamp"]-rows[0]["stamp"]),
                         "longest_gap_s": float(max(dt)), "maximum_age_s": max(r["age_s"] for r in rows)}
    passed = max(skews) <= .3 and all(v["measured_fps"] >= 4 and v["longest_gap_s"] <= .5 for v in summary.values())
    return {"status": "CAMERA_TIMING_PASSED" if passed else "CAMERA_TIMING_FAILED",
            "motor_writes": 0, "duration_s": time.monotonic()-start,
            "cameras": summary, "max_pair_skew_s": max(skews),
            "note": "This checks delivered frame timing and identity, not USB bus topology or physical grasp success."}


def experiment(a):
    config = load_config(a.config)
    limits = validate_config(config)
    fingerprint = binding(config)
    trace = Trace(Path(a.out).resolve())
    transport = SessionTransport(config, limits, execute=a.execute)
    try:
        arm = None
        if getattr(a, "kinematics", None):
            from .kinematics import load_arm
            arm, geometry_fingerprint = load_arm(a.kinematics, config)
        trace.write("start", command=a.command, execute=a.execute, fingerprint=fingerprint, limits=vars(limits))
        with transport:
            observer = Observer(config, limits)
            e = Experiment(config, transport, observer, trace, fingerprint)
            if a.command in ("inspect", "plan-reach"):
                obs, q = e.observe()
                result = {"status": "OBSERVED_ONLY", "features": obs.values.tolist(), "joints": q,
                          "motor_writes": 0, "fingerprint": fingerprint}
                if arm is not None:
                    result.update(tool_pose_estimate=arm.forward(q).tolist(), pose_frame="arm_base", pose_units="metres",
                                  kinematics_fingerprint=geometry_fingerprint, source=arm.solver.provenance(),
                                  physical_calibration_verified=False)
                if a.command == "plan-reach":
                    result.update(arm.plan(q, read_json(a.request)))
                    # A proposal must remain tied to the still-observed start.
                    fresh, after = transport.status()
                    if (fresh["phase"] != "holding" or any(abs(after[n]-q[n]) > limits.settle_ticks for n in q)
                            or time.time()-obs.captured_at > limits.frame_age_s):
                        raise Refused("Scene or joint start became stale while computing the proposal")
                    result.update(captured_at=obs.captured_at, camera_streams=obs.streams, camera_sequences=obs.sequences,
                                  session_started=transport.started)
            elif a.command == "calibrate":
                if not a.execute:
                    raise Refused("Calibration requires bounded physical probes; inspect first, then use --execute")
                result = e.calibrate()
                atomic_json(trace.folder / "model.json", result)
            elif a.command == "align":
                result = e.align(read_json(a.model), shadow=not a.execute)
            else:
                raise Refused("Unknown experiment")
            result["trace"] = str(trace.path)
            result["environment"] = "GUARDED_SESSION"
            result["target"] = config["target"]
            result["motor_writes"] = transport.path_ticks > 0
            atomic_json(trace.folder / "result.json", result)
            trace.write("finish", result=result)
            return result
    except BaseException as exc:
        # The existing owner defines STOP/release behavior. Never issue a blind
        # recovery motion after lost tracking or stale telemetry.
        try:
            transport.abort()
        except Exception as stop_error:
            trace.write("stop_request_failed", reason=str(stop_error))
        result = {"status": "REFUSED", "reason": str(exc), "physical_task_completed": False,
                  "path_ticks": transport.path_ticks, "trace": str(trace.path)}
        atomic_json(trace.folder / "result.json", result)
        trace.write("refused", **result)
        raise
    finally:
        trace.close()


def parser():
    p = argparse.ArgumentParser(description="Measured local visual control; no motors unless --execute is explicit.")
    sub = p.add_subparsers(dest="command", required=True)
    s = sub.add_parser("prepare", help="Create a robot-local experiment from saved calibration and coherent camera frames")
    s.add_argument("--session", required=True); s.add_argument("--frames", required=True)
    s.add_argument("--calibration", required=True); s.add_argument("--arm", choices=["left", "right"], default="right")
    s.add_argument("--out", required=True)
    s.add_argument('--camera-pair', choices=['head_wrist','head_oak'], default='head_wrist')
    s.add_argument('--oak-frames', help='Separate directory containing oak.json; otherwise --frames')
    s = sub.add_parser("seed", help="Identify regions on the saved image; does not move the robot")
    s.add_argument("--config", required=True); s.add_argument("--camera", required=True)
    s.add_argument("--region", action="append", default=[], help="name:x,y,width,height; head needs anchor, tool, target")
    s.add_argument("--tag", action="append", default=[], help="name:tag36h11_ID; reuse the farm's AprilTag detector")
    s.add_argument("--min-tag-edge-px", type=float, default=24,
                   help="Shortest tag edge required on every observation (default 24 pixels)")
    s.add_argument("--point", action="append", default=[], help="Optional tracked reference point name:x,y inside its ROI")
    s.add_argument("--target", help="Desired measurement values in config order, comma separated")
    s = sub.add_parser("camera-check", help="Audit coherent image streams without connecting to motors")
    s.add_argument("--frames", required=True); s.add_argument("--arm", choices=["left", "right"], default="right")
    s.add_argument("--seconds", type=float, default=20)
    s.add_argument('--camera-pair', choices=['head_wrist','head_oak'], default='head_wrist')
    s.add_argument('--oak-frames', help='Separate directory containing oak.json; otherwise --frames')
    s = sub.add_parser("tag-kit", help="Generate verified tag36h11 markers and a printable vector sheet; no devices")
    s.add_argument("--out", required=True)
    s.add_argument("--anchor-mm", type=float, default=60)
    s.add_argument("--tool-mm", type=float, default=40)
    s.add_argument("--target-mm", type=float, default=40)
    s = sub.add_parser("tag-check", help="Audit AprilTag visibility/tracking using existing image streams; no motor session needed")
    source = s.add_mutually_exclusive_group(required=True)
    source.add_argument("--frames", help="Existing head/wrist manifest directory; IDs 1=anchor, 2=tool, 3=target")
    source.add_argument("--config", help="Seeded experiment; check its exact tag IDs and reference points")
    s.add_argument("--arm", choices=["left", "right"], default="right")
    s.add_argument("--seconds", type=float, default=20)
    s.add_argument("--min-edge-px", type=float, default=24)
    s.add_argument("--out", required=True)
    for command in ("inspect", "plan-reach", "calibrate", "align"):
        s = sub.add_parser(command)
        s.add_argument("--config", required=True); s.add_argument("--out", required=True)
        if command in ("calibrate", "align"):
            s.add_argument("--execute", action="store_true", help="Send bounded commands to an already-running guarded session")
        else:
            s.set_defaults(execute=False)
        if command == "align":
            s.add_argument("--model", required=True)
        if command in ("inspect", "plan-reach"):
            s.add_argument("--kinematics", required=command == "plan-reach", help="Measured URDF zero/sign and tool/station configuration")
        if command == "plan-reach":
            s.add_argument("--request", required=True, help="Explicit metre-frame tool poses; proposals only")
    s = sub.add_parser("model-fetch", help="Download the pinned upstream SO-101 URDF, meshes and license; no devices")
    s.add_argument("--out", required=True)
    s = sub.add_parser("kinematics-template", help="Prepare a draft requiring measured geometric calibration")
    s.add_argument("--config", required=True); s.add_argument("--model-dir", required=True); s.add_argument("--out", required=True)
    s = sub.add_parser("kinematics-check", help="Run actual LeRobot FK/IK on the pinned model without hardware")
    s.add_argument("--model-dir", required=True); s.add_argument("--out", required=True)
    s = sub.add_parser("simulate", help="Run rendered-pixel regression; no motors and no carton physics")
    s.add_argument("--out", required=True)
    s = sub.add_parser("hinge-plan", help="Generate a measured crease arc; no motor commands")
    s.add_argument("--spec", required=True); s.add_argument("--out", required=True)
    s = sub.add_parser("verify-grasp", help="Evaluate observed jaw aperture and co-motion from a bounded lift")
    s.add_argument("--evidence", required=True); s.add_argument("--out", required=True)
    s = sub.add_parser("save-skill", help="Archive a measured visual alignment; never exports blind motor replay")
    s.add_argument("--config", required=True); s.add_argument("--run", required=True)
    s.add_argument("--name", required=True); s.add_argument("--out", required=True)
    s = sub.add_parser("report", help="Summarize an experiment's measured error and outcome")
    s.add_argument("run")
    return p


def main(argv=None):
    a = parser().parse_args(argv)
    try:
        if a.command == "prepare": result = prepare(a)
        elif a.command == "seed": result = seed(a)
        elif a.command == "camera-check": result = camera_check(a)
        elif a.command == "tag-kit":
            from .tag_kit import make_kit
            result = make_kit(a.out, a.anchor_mm, a.tool_mm, a.target_mm)
        elif a.command == "tag-check":
            from .tag_check import check_tags
            result = check_tags(a.out, a.seconds, a.min_edge_px, frames_dir=a.frames, arm=a.arm,
                                config=load_config(a.config) if a.config else None)
        elif a.command in ("inspect", "plan-reach", "calibrate", "align"): result = experiment(a)
        elif a.command == "model-fetch":
            from farm.kinematics.assets import fetch_model
            result = fetch_model(a.out)
        elif a.command == "kinematics-template":
            from .kinematics import template
            result = template(load_config(a.config), a.model_dir, a.out)
        elif a.command == "kinematics-check":
            from .kinematics import check_model
            result = check_model(a.model_dir); atomic_json(a.out, result)
        elif a.command == "simulate":
            from .simulation import run
            result = run(a.out)
        elif a.command == "hinge-plan":
            result = hinge_path(**read_json(a.spec)); atomic_json(a.out, result)
        elif a.command == "verify-grasp":
            result = verify_grasp(**read_json(a.evidence)); atomic_json(a.out, result)
        elif a.command == "save-skill":
            from .skills import save_alignment
            result = save_alignment(load_config(a.config), a.run, a.out, a.name)
        else:
            folder = Path(a.run)
            result = read_json(folder / "result.json")
            events = [json.loads(line) for line in (folder / "trace.jsonl").read_text().splitlines()]
            errors = [e["norm_px"] for e in events if e["event"] == "error"]
            result.update(commands=sum(e["event"] == "command" for e in events),
                          first_error_px=errors[0] if errors else None, last_error_px=errors[-1] if errors else None)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 2 if result.get("status") in ("GRASP_NOT_VERIFIED", "CAMERA_TIMING_FAILED", "TAG_CHECK_FAILED", "REFUSED") else 0
    except (Refused, ValueError, KeyError, OSError) as exc:
        print(json.dumps({"status": "REFUSED", "reason": str(exc), "physical_task_completed": False}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
