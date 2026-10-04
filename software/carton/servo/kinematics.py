"""Thin commissioning adapter for the farm's shared LeRobot kinematics.

Only the explicit model-fetch command accesses the network. Nothing here writes
motor commands. Geometric waypoints are proposals, not executable saved skills.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from farm.kinematics.assets import verified_model
from farm.kinematics.lerobot import CalibratedArm, LeRobotSO101
from .common import Refused, atomic_json, digest, read_json


def template(experiment, model_directory, output):
    path = Path(output).resolve()
    if path.exists():
        raise Refused("Existing kinematics configuration preserved; choose a new file")
    _, model = verified_model(model_directory)
    calibration = Path(experiment["calibration_file"]).resolve()
    if hashlib.sha256(calibration.read_bytes()).hexdigest() != experiment["calibration_sha256"]:
        raise Refused("Experiment motor calibration changed")
    config = {"schema": 1, "arm": experiment["arm"], "model_directory": str(Path(model_directory).resolve()),
              "calibration_file": str(calibration), "calibration_sha256": experiment["calibration_sha256"],
              "joints": {n: {"model_zero_tick": None, "model_sign": None} for n in model["joint_names"]},
              "gripper_from_tool": None, "base_from_station": None, "workspace_bounds_m": None,
              "notes": "Measure URDF zeros/signs, gripper-to-tool transform and arm-base workspace. "
                       "Station requests additionally need base_from_station. No inferred midpoint zeros."}
    atomic_json(path, config)
    return {"status": "NEEDS_GEOMETRIC_CALIBRATION", "config": str(path), "motor_writes": 0}


def load_arm(path, experiment):
    path = Path(path).resolve()
    config = read_json(path)
    for key in ("calibration_file", "model_directory"):
        config[key] = str((path.parent / config[key]).resolve())
    if (config["arm"] != experiment["arm"]
            or config["calibration_sha256"] != experiment["calibration_sha256"]
            or Path(config["calibration_file"]) != Path(experiment["calibration_file"]).resolve()):
        raise Refused("Kinematics belongs to a different arm or saved calibration")
    arm = CalibratedArm(config)
    for name, units in arm.units.items():
        if list(experiment["ranges"][f'{config["arm"]}_arm_{name}']) != [units.range_min, units.range_max]:
            raise Refused("Experiment ranges differ from the shared motor calibration")
    return arm, digest(config)


def check_model(folder):
    """Actual upstream FK→IK→FK regression, with no physical robot claims."""
    solver = LeRobotSO101(folder)
    cases = []
    for target in ([10, -15, 25, -10, 5], [-20, 25, -30, 15, -10], [0, 0, 0, 0, 0]):
        desired = solver.forward(target)
        solved = solver.solve(np.zeros(5), desired)
        cases.append({"target_degrees": target, **solved})
    return {"status": "UPSTREAM_KINEMATICS_CHECKED", "environment": "URDF_NUMERICAL_ONLY",
            "source": solver.provenance(), "cases": cases, "motor_writes": 0,
            "robot_connected": False, "physical_calibration_verified": False,
            "collision_checked": False, "physical_task_completed": False}
