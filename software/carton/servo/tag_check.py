"""Camera-only commissioning of the same tag tracker used by the controller.

This module never imports a motor transport. Failed detections are recorded and
the audit continues for diagnosis; any such failure makes the whole audit fail.
It never substitutes an old point for a missing observation.
"""
from __future__ import annotations

import copy
import hashlib
import math
import time
from pathlib import Path

import cv2
import numpy as np

from .common import Limits, Refused, Trace, atomic_json, finite, read_json, validate_config
from .features import TagTracker, tags_from_bgr
from .vision import ManifestCamera


class TagView:
    def __init__(self, specs, min_edge_px=24, reference=None):
        if not specs or any(s.get("type") != "apriltag" for s in specs.values()):
            raise Refused("tag-check requires AprilTag regions for every feature; seed tags first")
        self.specs = copy.deepcopy(specs)
        ids = [s["tag_id"] for s in specs.values()]
        if any(type(i) is not int or not 0 <= i < 587 for i in ids) or len(set(ids)) != len(ids):
            raise Refused("Each feature needs a distinct tag36h11 ID from 0 to 586")
        self.trackers = {}
        for s in self.specs.values():
            s["min_edge_px"] = max(finite(s.get("min_edge_px", 8)), min_edge_px)
        if reference is not None:
            found = tags_from_bgr(reference)
            self.trackers = {n: TagTracker(reference, s, found) for n, s in self.specs.items()}

    def measure(self, image):
        try:
            found = tags_from_bgr(image)
        except Refused as exc:
            return {"ok": False, "reason": str(exc), "features": {}, "seen_ids": []}
        rows = {}
        for name, spec in self.specs.items():
            tag_id = spec["tag_id"]
            row = {"tag_id": tag_id, "ok": False, "required_edge_px": spec["min_edge_px"]}
            if tag_id in found:
                d = found[tag_id]
                row.update(d)
                corners = np.asarray(d["corners"])
                row["min_edge_px"] = float(np.min(np.linalg.norm(corners-np.roll(corners, 1, axis=0), axis=1)))
            try:
                if name not in self.trackers:
                    self.trackers[name] = TagTracker(image, spec, found)
                row["point"] = self.trackers[name].locate(image, found).tolist()
                row["ok"] = True
            except Refused as exc:
                row["reason"] = str(exc)
            rows[name] = row
        return {"ok": all(r["ok"] for r in rows.values()), "features": rows, "seen_ids": sorted(found)}


def default_regions(head):
    roles = [("tool", 2), ("target", 3)] + ([("anchor", 1)] if head else [])
    return {n: {"type": "apriltag", "tag_id": i, "anchor": n == "anchor"} for n, i in roles}


def annotate(image, measurement):
    image = image.copy()
    for name, row in measurement["features"].items():
        if "corners" not in row:
            continue
        pts = np.round(row["corners"]).astype(np.int32)
        cv2.polylines(image, [pts], True, (0, 0, 0), 4)
        cv2.polylines(image, [pts], True, (255, 255, 255), 2)
        x, y = pts.min(axis=0)
        text = f'{name}:{row["tag_id"]} {"OK" if row["ok"] else "FAIL"} {row["min_edge_px"]:.0f}px'
        xy = (max(2, min(int(x), image.shape[1]-250)), max(18, int(y)-6))
        cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3)
        cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    label = "TAGS OK" if measurement["ok"] else "TAG CHECK FAILED - see trace.jsonl"
    cv2.rectangle(image, (0, 0), (image.shape[1], 24), (255, 255, 255), -1)
    cv2.putText(image, label, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1)
    return image


def save_raw(folder, index, name, frame):
    path = folder / f"{index:04d}-{name}-raw.png"
    if not cv2.imwrite(str(path), frame.image):
        raise Refused("Cannot preserve raw camera pixels")
    return {"raw_image": path.name, "raw_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "seq": frame.seq, "captured_at": frame.stamp, "stream_id": frame.stream,
            "camera_id": frame.camera_id}


def check_tags(out, seconds=20, min_edge_px=24, *, frames_dir=None, arm="right", config=None,
               clock=time.time, monotonic=time.monotonic, sleep=time.sleep):
    seconds, min_edge_px = finite(seconds), finite(min_edge_px)
    if not 2 <= seconds <= 60 or not 8 <= min_edge_px <= 200:
        raise Refused("Use 2–60 seconds and a minimum tag edge of 8–200 pixels")
    if (config is None) == (frames_dir is None):
        raise Refused("Choose either a seeded config or an existing frame directory")
    if arm not in ("left", "right"):
        raise Refused("Arm must be left or right")
    folder = Path(out).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    trace = Trace(folder)
    started = monotonic()
    samples, failures, summary, cameras, views, metadata = {}, [], {}, {}, {}, {}
    last = {}
    count = failed_pairs = 0
    max_skew = 0.0
    mode = "SEEDED_TAG_TRACKING" if config is not None else "UNSEEDED_TAG_VISIBILITY"
    try:
        if config is not None:
            limits = validate_config(config)
            cameras = config["cameras"]
        else:
            limits = Limits()
            for name in ("head", f"{arm}_wrist"):
                path = Path(frames_dir).resolve() / f"{name}.json"
                cameras[name] = {"manifest": str(path), "camera_id": read_json(path).get("camera_id"),
                                 "regions": default_regions(name == "head")}
        if len({c["camera_id"] for c in cameras.values()}) != 2 or any(not c["camera_id"] for c in cameras.values()):
            raise Refused("Head and wrist must have distinct, nonempty camera identities")
        readers = {n: ManifestCamera(c["manifest"], c["camera_id"], limits.frame_age_s, clock) for n, c in cameras.items()}
        for name, c in cameras.items():
            reference = None
            if config is not None:
                reference = cv2.imread(c["reference"])
                if reference is None:
                    raise Refused(f"Cannot read {name} seed image")
            views[name] = TagView(c["regions"], min_edge_px, reference)
            metadata[name] = {"camera_id": c["camera_id"], "regions": views[name].specs,
                              "reference_sha256": hashlib.sha256(Path(c["reference"]).read_bytes()).hexdigest() if config is not None else None}
        samples = {n: [] for n in cameras}
        trace.write("start", mode=mode, cameras=metadata, motor_writes=0)
        # Detector initialization / reference decoding must not consume the audit.
        started = monotonic()
        while monotonic()-started < seconds:
            frames = {}
            try:
                for name, reader in readers.items():
                    frames[name] = reader.read()
            except (Refused, OSError, KeyError, ValueError) as exc:
                partial = {n: save_raw(folder, count, n, f) for n, f in frames.items()}
                trace.write("camera_failure", reason=str(exc), partial_frames=partial)
                raise
            if not all(f.seq > last.get(n, -1) for n, f in frames.items()):
                sleep(.02)
                continue
            last = {n: f.seq for n, f in frames.items()}
            raw = {n: save_raw(folder, count, n, f) for n, f in frames.items()}
            measured = {n: views[n].measure(f.image) for n, f in frames.items()}
            skew = max(f.stamp for f in frames.values())-min(f.stamp for f in frames.values())
            max_skew = max(max_skew, skew)
            ok = skew <= limits.frame_skew_s and all(v["ok"] for v in measured.values())
            failed_pairs += not ok
            for name, frame in frames.items():
                raw[name]["age_s_at_read"] = clock()-frame.stamp
                samples[name].append(raw[name])
                path = folder / f"{count:04d}-{name}-annotated.jpg"
                if not cv2.imwrite(str(path), annotate(frame.image, measured[name])):
                    raise Refused("Cannot save annotated tag evidence")
            trace.write("pair", index=count, ok=ok, skew_s=skew, frames=raw, detections=measured)
            count += 1
            sleep(.005)
        if failed_pairs:
            failures.append(f"{failed_pairs} camera pairs failed tag tracking or synchronization; see trace.jsonl")
        for name, rows in samples.items():
            stamps = [r["captured_at"] for r in rows]
            fps = (len(rows)-1)/(stamps[-1]-stamps[0]) if len(rows) >= 2 else 0.0
            gap = max(np.diff(stamps)) if len(rows) >= 2 else None
            summary[name] = {"frames": len(rows), "capture_fps": fps,
                             "longest_capture_gap_s": float(gap) if gap is not None else None}
            if len(rows) < math.ceil(seconds*4) or fps < 4 or gap is None or gap > .5:
                failures.append(f"{name}: need >=4 checked pairs/s with capture gaps <=0.5s")
    except (Refused, OSError, KeyError, ValueError) as exc:
        failures.append(str(exc))
        trace.write("refused", reason=str(exc))
    finally:
        trace.close()
    result = {"schema": 1, "status": "TAG_CHECK_FAILED" if failures else "TAG_CHECK_PASSED", "mode": mode,
              "family": "tag36h11", "duration_s": monotonic()-started, "pairs": count,
              "failed_pairs": failed_pairs, "failures": failures, "max_pair_skew_s": max_skew,
              "cameras": summary, "setup": metadata, "motor_writes": 0, "physical_task_completed": False,
              "trace": str(folder / "trace.jsonl"),
              "note": "Camera-only visibility/tracking evidence. No motor readiness, metric pose, grasp or clearance validation."}
    atomic_json(folder / "result.json", result)
    return result
