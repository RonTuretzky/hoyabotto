"""Rendered-pixel regression rig, explicitly NOT a carton/contact simulator."""
from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np

from .common import Limits, Refused, Trace, atomic_json, binding
from .controller import Experiment
from .vision import Observer


class PixelRig:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.now, self.seq = 1000000.0, 0
        self.started = self.now
        self.limits = Limits(max_seconds=300)
        self.joints = [f"right_arm_{s}" for s in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")]
        self.q = {n: 2048 for n in self.joints + ["right_arm_wrist_roll", "right_arm_gripper"]}
        self.origin = self.q.copy()
        self.path_ticks = 0
        self.fail = None
        rng = np.random.default_rng(394)
        self.patches = {n: rng.integers(35, 230, (24, 24, 3), np.uint8) for n in ("tool", "target", "anchor")}
        self.matrix = np.array([[.7, 0, .2, 0], [0, -.6, .15, 0], [.2, 0, .65, .1], [0, .1, .15, .7]])
        self.images, self.points = self.render()
        cameras = {}
        for name in ("head", "right_wrist"):
            ref = self.folder / f"{name}-reference.png"
            cv2.imwrite(str(ref), self.images[name])
            regions = {n: {"roi": [p[0]-12, p[1]-12, 24, 24], "point": p,
                           "anchor": n == "anchor"} for n, p in self.points[name].items()}
            cameras[name] = {"camera_id": "synthetic-"+name, "manifest": str(self.folder / f"{name}.json"),
                             "reference": str(ref), "regions": regions}
        self.config = {"schema": 1, "units": "encoder_ticks", "arm": "right", "joints": self.joints,
                       "ranges": {n: [1000, 3100] for n in self.q}, "calibration_sha256": "synthetic-only",
                       "cameras": cameras, "measurements": [], "target": [20, -20, 190, -80]}
        for camera in cameras:
            for axis in (0, 1):
                self.config["measurements"].append({"name": f"{camera}_{axis}", "camera": camera,
                                                    "a": "target", "b": "tool", "axis": axis})

    def clock(self):
        return self.now

    def render(self):
        dq = np.array([self.q[n]-2048 for n in self.joints])
        # Nonlinear plant differs from the controller's fitted linear model.
        offset = self.matrix @ dq + .00008 * dq**2
        points = {"head": {"tool": (np.array([260, 240])+offset[:2]).tolist(), "target": [300, 180], "anchor": [65, 65]},
                  "right_wrist": {"tool": [120, 310], "target": (np.array([320, 220])+offset[2:]).tolist()}}
        images = {}
        for cam, p in points.items():
            im = np.full((480, 640, 3), 24, np.uint8)
            for name, xy in p.items():
                if self.fail == "occlusion" and name == "target":
                    continue
                x, y = np.round(xy).astype(int)
                if self.fail == "camera_shift" and name == "anchor":
                    x += 12
                im[y-12:y+12, x-12:x+12] = self.patches[name]
            images[cam] = im
        return images, points

    def publish(self):
        self.now += .2
        self.seq += 1
        images, _ = self.render()
        for name, im in images.items():
            data = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
            fn = f"{name}-current.jpg"
            (self.folder / fn).write_bytes(data)
            atomic_json(self.folder / f"{name}.json", {"schema": 1, "camera_id": "synthetic-"+name,
                        "stream_id": "synthetic", "seq": self.seq, "captured_at": self.now,
                        "width": 640, "height": 480, "image": fn, "sha256": hashlib.sha256(data).hexdigest()})

    def positions(self):
        return self.q.copy()

    def move(self, joint, ticks):
        if type(ticks) is not int or abs(ticks) > 68 or joint not in self.joints:
            raise Refused("Synthetic motor command rejected")
        if abs(self.q[joint]+ticks-self.origin[joint]) > self.limits.trust_ticks:
            raise Refused("Synthetic motor envelope exceeded")
        if self.fail != "stuck_motor":
            self.q[joint] += ticks
        self.path_ticks += abs(ticks)
        self.now += .15
        return self.positions(), self.now

    def abort(self):
        pass


class RenderedObserver:
    def __init__(self, rig):
        self.rig = rig
        self.inner = Observer(rig.config, rig.limits, clock=rig.clock)

    def observe(self, after=0.0):
        self.rig.publish()
        return self.inner.observe(after=after)

    def evidence(self, folder, obs):
        self.inner.evidence(folder, obs)


def run(folder):
    folder = Path(folder).resolve()
    rig = PixelRig(folder / "synthetic-inputs")
    trace = Trace(folder / "run")
    try:
        experiment = Experiment(rig.config, rig, RenderedObserver(rig), trace, binding(rig.config), rig.clock)
        model = experiment.calibrate()
        atomic_json(folder / "synthetic-model.json", model)
        result = experiment.align(model)
        result.update(environment="SYNTHETIC_RENDERED_PIXELS", robot_connected=False,
                      carton_contact_simulated=False, path_ticks=rig.path_ticks)
        atomic_json(folder / "result.json", result)
        return result
    finally:
        trace.close()
