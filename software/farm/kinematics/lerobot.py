"""LeRobot's URDF/Placo solver, with explicit units and checked results.

No analytical IK or robot driver lives here. All FK/IK goes through upstream
RobotKinematics. These proposals never certify collision or contact safety.
"""
from __future__ import annotations

import hashlib
import json
import math
from importlib.metadata import version
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .assets import verified_model
from .units import JointUnits, finite


def transform(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]):
        raise ValueError("Expected a finite 4x4 rigid transform in metres")
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError("Transform rotation must be orthonormal and right-handed")
    return t.copy()


def pose_error(actual, desired):
    position = float(np.linalg.norm(actual[:3, 3] - desired[:3, 3]))
    cosine = (np.trace(actual[:3, :3].T @ desired[:3, :3]) - 1) / 2
    return position, math.degrees(math.acos(float(np.clip(cosine, -1, 1))))


class LeRobotSO101:
    def __init__(self, folder):
        urdf, self.manifest = verified_model(folder)
        try:
            from lerobot.model.kinematics import RobotKinematics
            self.names = self.manifest["joint_names"]
            self.engine = RobotKinematics(str(urdf), target_frame_name=self.manifest["target_frame"], joint_names=self.names)
            self.engine.solver.enable_joint_limits(True)
        except (ImportError, RuntimeError) as exc:
            raise ValueError("LeRobot kinematics unavailable; install the pinned carton kinematics extra in an isolated environment") from exc
        joints = {j.attrib["name"]: j for j in ET.parse(urdf).getroot().findall("joint")}
        self.ranges = {name: np.degrees([float(joints[name].find("limit").attrib[k]) for k in ("lower", "upper")])
                       for name in self.names}

    def check_joints(self, q):
        q = np.asarray(q, float)
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError("Supply all five arm joints in explicit URDF order/degrees")
        for name, value in zip(self.names, q):
            lo, hi = self.ranges[name]
            if not lo <= value <= hi:
                raise ValueError(f"{name}: outside upstream URDF joint limits")
        return q

    def forward(self, q):
        return transform(self.engine.forward_kinematics(self.check_joints(q)))

    def solve(self, current, desired, *, position_only=False):
        q, target = self.check_joints(current), transform(desired)
        # LeRobot 0.6.1 performs one numerical solve per call. Repeated upstream
        # steps with an explicit residual check also work with later versions.
        for iteration in range(80):
            error_m, error_deg = pose_error(self.forward(q), target)
            if error_m <= .001 and (position_only or error_deg <= 2):
                return {"degrees": q.tolist(), "position_error_m": error_m,
                        "orientation_error_deg": error_deg, "orientation_constrained": not position_only,
                        "iterations": iteration}
            try:
                q = self.check_joints(self.engine.inverse_kinematics(q, target, position_weight=1,
                                                                     orientation_weight=0 if position_only else .1))
            except RuntimeError as exc:
                raise ValueError(f"Upstream IK refused the target: {exc}") from exc
        raise ValueError(f"Upstream IK did not converge: {error_m:.4f}m / {error_deg:.2f}deg; target was not clamped")

    def provenance(self):
        return {"solver": "lerobot.model.kinematics.RobotKinematics", "lerobot": version("lerobot"),
                "placo": version("placo"), "model_repository": self.manifest["repository"],
                "model_revision": self.manifest["revision"], "frame": self.manifest["target_frame"]}


class CalibratedArm:
    """Bind upstream geometry to this arm's saved raw encoder calibration."""
    def __init__(self, config):
        if config.get("schema") != 1 or config.get("arm") not in ("right", "left"):
            raise ValueError("Expected an explicit single-arm kinematics configuration")
        self.config = config
        cal_path = Path(config["calibration_file"])
        if hashlib.sha256(cal_path.read_bytes()).hexdigest() != config["calibration_sha256"]:
            raise ValueError("Motor calibration changed since kinematics configuration was prepared")
        raw = json.loads(cal_path.read_text())
        self.units = {}
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
        if set(config["joints"]) != set(names):
            raise ValueError("Five named arm joint zero/sign measurements are required")
        for name in names:
            j = f'{config["arm"]}_arm_{name}'
            s = config["joints"][name]
            if s.get("model_zero_tick") is None or s.get("model_sign") is None:
                raise ValueError(f"{name}: measure the upstream model zero/sign; do not infer it from range endpoints")
            self.units[name] = JointUnits(raw[j]["range_min"], raw[j]["range_max"],
                                          finite(s["model_zero_tick"]), s["model_sign"])
        self.base_from_station = (None if config.get("base_from_station") is None
                                  else transform(config["base_from_station"]))
        self.gripper_from_tool = transform(config["gripper_from_tool"])
        self.bounds = np.asarray(config["workspace_bounds_m"], float)
        if (self.bounds.shape != (2, 3) or not np.isfinite(self.bounds).all()
                or not np.all(self.bounds[0] < self.bounds[1]) or np.max(np.abs(self.bounds)) > 1):
            raise ValueError("Specify measured workspace min/max in arm-base metres within a one-metre envelope")
        self.solver = LeRobotSO101(config["model_directory"])

    def degrees(self, ticks):
        return [self.units[n].ticks_to_model_degrees(ticks[f'{self.config["arm"]}_arm_{n}'])
                for n in self.solver.names]

    def forward(self, ticks):
        return self.solver.forward(self.degrees(ticks)) @ self.gripper_from_tool

    def plan(self, current_ticks, request):
        if request.get("frame") not in ("arm_base", "station") or request.get("units") != "metres":
            raise ValueError("Declare arm_base or station frame and metres")
        mode = request.get("orientation")
        if mode not in ("constrained", "position_only"):
            raise ValueError("Choose constrained or position_only orientation explicitly")
        if mode == "position_only" and np.linalg.norm(self.gripper_from_tool[:3, 3]) > 1e-9:
            raise ValueError("An offset tool requires constrained orientation to preserve its contact point")
        poses = request.get("tool_poses", [])
        if not 1 <= len(poses) <= 100:
            raise ValueError("Supply 1–100 tool poses; no implicit motion sequence")
        q = self.degrees(current_ticks)
        output = []
        for pose in poses:
            desired_tool = transform(pose)
            if request["frame"] == "station":
                if self.base_from_station is None:
                    raise ValueError("Measure base_from_station before requesting station-frame poses")
                desired_tool = self.base_from_station @ desired_tool
            if not np.all((desired_tool[:3, 3] >= self.bounds[0]) & (desired_tool[:3, 3] <= self.bounds[1])):
                raise ValueError("Target outside the measured workspace; not clamped")
            desired_gripper = desired_tool @ np.linalg.inv(self.gripper_from_tool)
            solved = self.solver.solve(q, desired_gripper, position_only=mode == "position_only")
            q = solved["degrees"]
            ticks = {f'{self.config["arm"]}_arm_{n}': int(round(self.units[n].model_degrees_to_ticks(v)))
                     for n, v in zip(self.solver.names, q)}
            # Recheck after quantization. Use the complete tool offset, not only
            # the URDF gripper origin, when deciding whether the target was met.
            actual_tool = self.forward(ticks)
            q = self.degrees(ticks)
            error_m, error_deg = pose_error(actual_tool, desired_tool)
            if error_m > .002 or (mode == "constrained" and error_deg > 2.5):
                raise ValueError("Rounded encoder proposal misses the requested tool pose")
            output.append({"joint_targets_ticks": ticks, "tool_pose": actual_tool.tolist(),
                           "position_error_m": error_m, "orientation_error_deg": error_deg})
        return {"status": "KINEMATIC_PROPOSAL_ONLY", "units": "encoder_ticks", "waypoints": output,
                "pose_frame": "arm_base", "pose_units": "metres", "starting_ticks": current_ticks,
                "source": self.solver.provenance(), "motor_writes": 0, "collision_checked": False,
                "orientation_constrained": mode == "constrained", "contact_validated": False,
                "physical_task_completed": False}
