"""Archive achieved visual goals without turning them into blind joint replay."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from .common import Refused, atomic_json, binding, read_json, validate_config, vector


def save_alignment(config, run, output, name):
    limits = validate_config(config)
    folder, output = Path(run).resolve(), Path(output).resolve()
    if output.exists():
        raise Refused("Existing skill preserved; use a new name/version")
    result = read_json(folder / "result.json")
    if result.get("status") != "ALIGNED_ONLY" or result.get("environment") != "GUARDED_SESSION":
        raise Refused("Only a measured guarded-session alignment can become a physical visual goal")
    fingerprint = binding(config)
    if result.get("fingerprint") != fingerprint or result.get("target") != config["target"]:
        raise Refused("Successful run belongs to a different configuration/target")
    trace_path = folder / "trace.jsonl"
    events = [json.loads(line) for line in trace_path.read_text().splitlines()]
    if not events or events[-1].get("event") != "finish" or events[-1].get("result") != result:
        raise Refused("Run has no matching final evidence record")
    observations = [e for e in events if e["event"] == "observation"]
    errors = [e for e in events if e["event"] == "error"]
    if len(observations) < 3 or len(errors) < 3:
        raise Refused("Three final observations are required")
    for row in errors[-3:]:
        if np.max(np.abs(vector(row["error_px"], len(config["target"])))) > limits.tolerance_px:
            raise Refused("Final observed error exceeds the alignment tolerance")
    for before, after in zip(observations[-3:-1], observations[-2:]):
        if any(after["sequences"][cam] <= before["sequences"][cam] for cam in config["cameras"]):
            raise Refused("Final evidence repeats an old frame")
    skill = {"schema": 1, "name": name, "status": "VERIFIED_VISUAL_ALIGNMENT_ONLY",
             "fingerprint": fingerprint, "target": config["target"], "arm": config["arm"],
             "source_run": str(folder), "trace_sha256": hashlib.sha256(trace_path.read_bytes()).hexdigest(),
             "execution": "RECALIBRATE_LOCALLY_THEN_ALIGN", "blind_joint_replay_allowed": False,
             "grasp_verified": False, "fold_verified": False, "physical_task_completed": False}
    atomic_json(output, skill)
    return skill
