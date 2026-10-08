"""OAK-D Lite observations in its own optical frame; no robot-frame guesses."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import time

import cv2
import numpy as np

from .common import Refused, finite, read_json, vector
from .features import FeatureTracks


def validate_depth_spec(spec):
    for key in ("manifest", "camera_id", "reference", "regions", "table_features", "tool", "paddle", "bottom"):
        if not spec.get(key):
            raise Refused(f"Depth commissioning needs {key}")
    if len(set(spec["table_features"])) < 6:
        raise Refused("Depth table plane needs at least six independently seeded points")
    if len({spec[k] for k in ("tool", "paddle", "bottom")}) != 3:
        raise Refused("Depth tool, paddle and paddle-bottom tracks must be independent")
    if set(spec["table_features"]) & {spec[k] for k in ("tool", "paddle", "bottom")}:
        raise Refused("Moving features cannot define the table plane")
    for name in [*spec["table_features"], *(spec[k] for k in ("tool", "paddle", "bottom"))]:
        if name not in spec["regions"]:
            raise Refused(f"Missing OAK RGB seed: {name}")
    for name in spec["table_features"]:
        if spec["regions"][name].get("anchor") is not True:
            raise Refused("Depth table points must be stationary anchors")
    if abs(np.linalg.norm(vector(spec["up_hint_camera"], 3))-1) > .001:
        raise Refused("Depth up hint must be a unit direction in the OAK optical frame")
    for name in ("min_lift_mm", "min_clearance_mm"):
        if not 10 <= finite(spec[name]) <= 100:
            raise Refused(f"{name} must be a measured 10–100mm clearance")
    if not 0 < finite(spec["max_slip_mm"]) <= 5:
        raise Refused("Depth co-motion tolerance cannot exceed 5mm")


def camera_point(depth, point, intrinsics):
    """Small quality-checked patch; zero/holes/background edges are not contact."""
    x, y = np.rint(vector(point, 2)).astype(int)
    h, w = depth.shape
    if not 2 <= x < w-2 or not 2 <= y < h-2:
        raise Refused("Depth feature patch extends outside the aligned image")
    patch = depth[y-2:y+3, x-2:x+3]
    valid = patch[(patch >= 200) & (patch <= 2000)]
    if len(valid) < .9*patch.size:
        raise Refused("Depth feature has insufficient valid pixels in the 200–2000mm measurement range")
    if np.percentile(valid, 90)-np.percentile(valid, 10) > 10:
        raise Refused("Depth patch crosses an edge or has excessive dispersion")
    z = float(np.median(valid))
    k = np.asarray(intrinsics, float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
        raise Refused("Invalid rectified pinhole intrinsics")
    return np.array([(point[0]-k[0, 2])*z/k[0, 0], (point[1]-k[1, 2])*z/k[1, 1], z])


def fit_table(points, up_hint):
    points = np.asarray(points, float)
    center = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points-center)
    if len(points) < 6 or singular[1] < 30:
        raise Refused("Table depth samples are too clustered or collinear")
    normal = vt[-1]
    if normal @ up_hint < 0:
        normal = -normal
    if normal @ up_hint < .5 or max(abs((points-center) @ normal)) > 3:
        raise Refused("Depth table plane is ambiguous or has excessive residual")
    return normal, center


class DepthObserver:
    def __init__(self, spec, clock=time.time, sleep=time.sleep):
        validate_depth_spec(spec)
        self.spec, self.clock, self.sleep = spec, clock, sleep
        self.path = Path(spec["manifest"]).resolve()
        reference = cv2.imread(spec["reference"])
        if reference is None:
            raise Refused("Cannot read OAK reference image")
        self.tracks = FeatureTracks(reference, spec["regions"])
        self.stream, self.seq, self.intrinsics, self.plane = None, -1, None, None

    def _image(self, name, sha, flags):
        path = (self.path.parent / name).resolve()
        if not path.is_relative_to(self.path.parent):
            raise Refused("Depth frame must stay within its manifest directory")
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != sha:
            raise Refused("Depth frame hash mismatch")
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), flags)
        if image is None:
            raise Refused("Cannot decode depth stream image")
        return image

    def read(self):
        m = read_json(self.path)
        if (m.get("schema") != 1 or m.get("camera_id") != self.spec["camera_id"]
                or m.get("host") != os.uname().nodename or m.get("depth_units") != "mm"
                or m.get("invalid_depth") != 0 or m.get("alignment") != "CAM_A RGB"
                or m.get("projection") != "rectified_pinhole" or m.get("coordinate_frame") != "CAM_A_optical"):
            raise Refused("OAK identity, local clock, alignment or metric units do not match")
        stamps = [finite(m[k]) for k in ("captured_at", "rgb_captured_at", "depth_captured_at")]
        if not all(0 <= self.clock()-stamp <= 1 for stamp in stamps) or max(stamps)-min(stamps) > .033:
            raise Refused("Depth timestamps are stale, future or unsynchronized")
        if type(m["seq"]) is not int or m["seq"] < self.seq or m["seq"] < 0 or not m.get("stream_id"):
            raise Refused("Invalid or regressing depth sequence")
        if self.stream is not None and self.stream != m["stream_id"]:
            raise Refused("Depth stream restarted; redo scene registration")
        rgb = self._image(m["image"], m["sha256"], cv2.IMREAD_COLOR)
        depth = self._image(m["depth_image"], m["depth_sha256"], cv2.IMREAD_UNCHANGED)
        if rgb.shape[:2] != depth.shape or depth.shape != (m["height"], m["width"]) or depth.dtype != np.uint16:
            raise Refused("Depth dimensions or encoding changed")
        if self.intrinsics is not None and m["intrinsics"] != self.intrinsics:
            raise Refused("Depth intrinsics changed")
        self.stream, self.intrinsics = m["stream_id"], m["intrinsics"]
        return m, rgb, depth

    def observe(self, paired_at):
        # Bound the entire head/wrist/OAK group, not distance to its oldest
        # timestamp alone (which could admit 600 ms of total camera skew).
        paired = list(paired_at.values()) if isinstance(paired_at, dict) else [paired_at]
        paired = [finite(t) for t in paired]
        if not paired or max(paired)-min(paired) > .3:
            raise Refused("Invalid registered RGB camera pairing")
        end = self.clock()+1
        while True:
            m, rgb, depth = self.read()
            stamps = paired+[m["rgb_captured_at"], m["depth_captured_at"]]
            if m["seq"] > self.seq and max(stamps)-min(stamps) <= .3:
                break
            if self.clock() >= end or m["captured_at"] > min(paired)+.3:
                raise Refused("No distinct depth frame synchronized with the registered RGB pair")
            self.sleep(.02)
        self.seq = m["seq"]
        pixels = self.tracks.locate(rgb)
        xyz = {n: camera_point(depth, p, self.intrinsics) for n, p in pixels.items()}
        normal, center = fit_table([xyz[n] for n in self.spec["table_features"]], vector(self.spec["up_hint_camera"], 3))
        if self.plane is not None:
            old_normal, old_center = self.plane
            if normal @ old_normal < np.cos(np.deg2rad(2)) or abs((center-old_center) @ old_normal) > 5:
                raise Refused("Table/camera registration moved; recapture scene before motion")
        if self.plane is None:
            self.plane = (normal, center)
        return {"seq": self.seq, "stream": self.stream, "captured_at": m["captured_at"],
                "coordinate_frame": "CAM_A_optical", "units": "mm", "robot_frame_calibrated": False,
                "points": {n: p.tolist() for n, p in xyz.items()}, "normal": normal.tolist(), "table_center": center.tolist(),
                "bottom_clearance_mm": float((xyz[self.spec["bottom"]]-center) @ normal)}


def verify_depth_lift(before, after, spec):
    if before["stream"] != after["stream"] or after["seq"] <= before["seq"]:
        raise Refused("Depth lift needs distinct observations from one stream")
    normal = vector(before["normal"], 3)
    movement = {k: vector(after["points"][spec[k]], 3)-vector(before["points"][spec[k]], 3)
                for k in ("tool", "paddle")}
    if (min(v @ normal for v in movement.values()) < spec["min_lift_mm"]
            or np.linalg.norm(movement["tool"]-movement["paddle"]) > spec["max_slip_mm"]
            or after["bottom_clearance_mm"] < spec["min_clearance_mm"]):
        raise Refused("Depth did not verify paddle lift and tabletop clearance")
    return {"bottom_clearance_mm": after["bottom_clearance_mm"],
            "paddle_lift_mm": float(movement["paddle"] @ normal)}
