"""Local tracking from reviewed seed images; no model call in the control loop."""
from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .common import Limits, Observation, Refused, finite, read_json, vector


@dataclass
class Frame:
    image: np.ndarray
    stamp: float
    seq: int
    stream: str
    camera_id: str


class ManifestCamera:
    """Read an immutable JPEG selected by an atomically published, hashed manifest.

    Deliberately rejects the old loose JPEG/timestamp pair for powered operation.
    A receive timestamp alone cannot bind an observation to a post-motion image.
    """
    def __init__(self, manifest, camera_id, max_age_s=1.0, clock=time.time):
        self.path = Path(manifest)
        self.camera_id = camera_id
        self.max_age_s = max_age_s
        self.clock = clock
        self.stream = None
        self.last_seq = -1
        self.last_stamp = None
        self.last_hash = None

    def read(self):
        m = read_json(self.path)
        if m.get("schema") != 1 or m.get("camera_id") != self.camera_id:
            raise Refused(f"{self.path.name}: coherent frame identity missing or changed")
        if not isinstance(m.get("stream_id"), str) or not m["stream_id"]:
            raise Refused("Frame has no capture stream identity")
        if self.stream is not None and m["stream_id"] != self.stream:
            raise Refused("Camera restarted: invalidate the current experiment")
        self.stream = m["stream_id"]
        stamp = finite(m.get("captured_at"), "captured_at")
        age = self.clock() - stamp
        if not 0 <= age <= self.max_age_s:
            raise Refused(f"{self.path.name}: frame age {age:.3f}s outside bound")
        seq = m.get("seq")
        if type(seq) is not int or seq < 0 or seq < self.last_seq:
            raise Refused("Invalid or regressing capture sequence")
        if self.last_stamp is not None:
            if seq > self.last_seq and stamp <= self.last_stamp:
                raise Refused("Capture timestamps must advance with the sequence")
            if seq == self.last_seq and (stamp != self.last_stamp or m.get("sha256") != self.last_hash):
                raise Refused("Frame sequence was reused for different bytes or capture time")
        image_path = (self.path.parent / m["image"]).resolve()
        if image_path.parent != self.path.parent.resolve():
            raise Refused("Frame path must stay inside the stream directory")
        try:
            data = image_path.read_bytes()
        except OSError as exc:
            raise Refused(f"Frame unavailable: {exc}") from exc
        if hashlib.sha256(data).hexdigest() != m.get("sha256"):
            raise Refused("Frame bytes do not match their timestamp/sequence manifest")
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None or list(image.shape[:2][::-1]) != [m.get("width"), m.get("height")]:
            raise Refused("Frame dimensions do not match the manifest")
        self.last_seq = seq
        self.last_stamp, self.last_hash = stamp, m.get("sha256")
        return Frame(image, stamp, seq, self.stream, self.camera_id)


class RegionTracker:
    """Track a textured rigid patch relative to its original image, avoiding drift.

    Forward/backward optical flow, affine inliers and patch displacement bound
    reject occlusion and inconsistent tracks. This does NOT recognize objects:
    the agent must review the selected regions and annotated output beforehand.
    """
    def __init__(self, reference, spec):
        self.reference = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        self.spec = spec
        if not 1 <= finite(spec.get("max_displacement_px", 100)) <= 100:
            raise Refused("Patch tracking is limited to 100 pixels around the reviewed seed")
        x, y, w, h = vector(spec["roi"], 4).astype(int)
        height, width = self.reference.shape
        if min(w, h) < 8 or x < 0 or y < 0 or x + w > width or y + h > height:
            raise Refused("Seed ROI lies outside the image or is too small")
        self.centre = vector(spec.get("point", [x + w / 2, y + h / 2]), 2)
        self.roi = (x, y, w, h)
        self.template = self.reference[y:y+h, x:x+w].copy()
        if not x <= self.centre[0] <= x + w or not y <= self.centre[1] <= y + h:
            raise Refused("Tracked point must lie inside its seed ROI")
        mask = np.zeros_like(self.reference)
        mask[y:y+h, x:x+w] = 255
        self.points = cv2.goodFeaturesToTrack(self.reference, 80, .02, 3, mask=mask)
        if self.points is None or len(self.points) < 8:
            raise Refused("Seed patch has too little texture; select a better patch or use a printed marker")

    def locate(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if gray.shape != self.reference.shape:
            raise Refused("Camera resolution changed")
        # Seed optical flow with a bounded template search. Starting every flow
        # solve at zero displacement fails once the hand has moved several patch
        # widths; unconstrained re-detection could jump to a different object.
        x, y, w, h = self.roi
        radius = int(self.spec.get("max_displacement_px", 100))
        x0, y0 = max(0, x-radius), max(0, y-radius)
        x1, y1 = min(gray.shape[1], x+w+radius), min(gray.shape[0], y+h+radius)
        scores = cv2.matchTemplate(gray[y0:y1, x0:x1], self.template, cv2.TM_CCOEFF_NORMED)
        _, best, _, where = cv2.minMaxLoc(scores)
        alternatives = scores.copy()
        bx, by = where
        alternatives[max(0, by-h//2):by+h//2+1, max(0, bx-w//2):bx+w//2+1] = -1
        second = float(np.max(alternatives))
        if best < .75 or best-second < .08:
            raise Refused("Patch appearance lost or multiple similar targets are ambiguous")
        shift = np.array([x0+bx-x, y0+by-y], np.float32)
        opts = dict(winSize=(21, 21), maxLevel=4,
                    flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                    criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01))
        nxt, ok1, err = cv2.calcOpticalFlowPyrLK(self.reference, gray, self.points, self.points+shift, **opts)
        if nxt is None:
            raise Refused("Tracker lost its patch")
        back, ok2, _ = cv2.calcOpticalFlowPyrLK(gray, self.reference, nxt, self.points.copy(), **opts)
        if back is None:
            raise Refused("Tracker backward verification failed")
        valid = (ok1.ravel() != 0) & (ok2.ravel() != 0)
        valid &= np.linalg.norm(back[:, 0] - self.points[:, 0], axis=1) <= 1.0
        valid &= err.ravel() <= 25
        valid &= np.isfinite(nxt).all(axis=(1, 2))
        if valid.sum() < max(6, .5 * len(self.points)):
            raise Refused("Tracker lost too many consistent image features")
        matrix, inliers = cv2.estimateAffinePartial2D(self.points[valid], nxt[valid],
                                                     method=cv2.RANSAC, ransacReprojThreshold=1.5)
        if matrix is None or inliers.sum() < max(6, .7 * valid.sum()):
            raise Refused("Tracked patch does not move as one rigid region")
        scale = np.linalg.norm(matrix[:, 0])
        if not .65 <= scale <= 1.5:
            raise Refused("Patch scale is outside its local tracking envelope")
        point = matrix @ np.r_[self.centre, 1.0]
        if not (0 <= point[0] < gray.shape[1] and 0 <= point[1] < gray.shape[0]):
            raise Refused("Tracked point left the image")
        displacement = np.linalg.norm(point - self.centre)
        if displacement > self.spec.get("max_displacement_px", 100):
            raise Refused("Patch left its reviewed local image envelope")
        if self.spec.get("anchor"):
            # Rotation/zoom around the centre is also camera motion.
            predicted = cv2.transform(self.points, matrix)
            if np.max(np.linalg.norm(predicted - self.points, axis=2)) > 2.0:
                raise Refused("Fixed head-camera/background anchor moved")
        return point


class Observer:
    def __init__(self, config, limits: Limits, clock=time.time, sleep=time.sleep):
        from .features import FeatureTracks
        self.config, self.limits, self.clock, self.sleep = config, limits, clock, sleep
        self.cameras, self.trackers = {}, {}
        for name, cam in config["cameras"].items():
            reference = cv2.imread(cam["reference"])
            if reference is None:
                raise Refused(f"Cannot read {name} seed image")
            self.cameras[name] = ManifestCamera(cam["manifest"], cam["camera_id"], limits.frame_age_s, clock)
            self.trackers[name] = FeatureTracks(reference, cam["regions"])
        self.last_sequences = {}
        self.last_frames = {}

    def observe(self, after=0.0, timeout=2.0):
        end = self.clock() + timeout
        while True:
            frames = {n: c.read() for n, c in self.cameras.items()}
            fresh = all(f.stamp > after and f.seq > self.last_sequences.get(n, -1) for n, f in frames.items())
            skew = max(f.stamp for f in frames.values()) - min(f.stamp for f in frames.values())
            if fresh and skew <= self.limits.frame_skew_s:
                break
            if self.clock() >= end:
                raise Refused("No distinct synchronized post-command camera pair arrived")
            self.sleep(.02)
        points = {}
        for name, frame in frames.items():
            points[name] = self.trackers[name].locate(frame.image)
        values = []
        for m in self.config["measurements"]:
            p = points[m["camera"]]
            v = p[m["a"]].copy()
            if m.get("b") is not None:
                v -= p[m["b"]]
            values.append(float(v[m["axis"]]))
        self.last_sequences = {n: f.seq for n, f in frames.items()}
        self.last_frames = frames
        return Observation(vector(values), min(f.stamp for f in frames.values()),
                           dict(self.last_sequences), {c: {n: p.tolist() for n, p in ps.items()} for c, ps in points.items()},
                           {n: f.stream for n, f in frames.items()}, {n: f.stamp for n, f in frames.items()})

    def evidence(self, folder, observation):
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        for name, frame in self.last_frames.items():
            image = frame.image.copy()
            for region, p in observation.points[name].items():
                xy = tuple(np.round(p).astype(int))
                cv2.drawMarker(image, xy, (255, 255, 255), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(image, region, (xy[0]+7, xy[1]-7), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
                cv2.putText(image, region, (xy[0]+7, xy[1]-7), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            if not cv2.imwrite(str(folder / f"{name}.jpg"), image):
                raise Refused("Could not save annotated evidence")
