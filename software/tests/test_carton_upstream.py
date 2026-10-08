"""Real upstream perception/URDF tests; no robot or camera device connections."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
import pytest

from carton.servo import cli
from carton.servo.common import Refused, atomic_json
from carton.servo.features import FeatureTracks, TagTracker, tags_from_bgr
from carton.servo.kinematics import check_model, load_arm, template
from carton.servo.simulation import PixelRig
from carton.servo.vision import Observer
from farm.kinematics.assets import fetch_model, manifest, verified_model
from farm.kinematics.lerobot import CalibratedArm, LeRobotSO101, pose_error, transform
from farm.perception.tags import detect_tags
from farm.status import Reading, Status


def tag_image(entries=((1, 30, 30), (2, 250, 180), (3, 470, 320))):
    image = np.full((480, 640, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id, x, y in entries:
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 80)
        image[y:y+80, x:x+80] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


def tag_specs():
    return {name: {"type": "apriltag", "tag_id": tag_id, "anchor": name == "anchor"}
            for name, tag_id in (("anchor", 1), ("tool", 2), ("target", 3))}


def test_shared_detector_tracks_identity_and_subpixel_translation():
    reference = tag_image()
    tracks = FeatureTracks(reference, tag_specs())
    before = tracks.locate(reference)
    shifted = tag_image(((1, 30, 30), (2, 261, 173), (3, 470, 320)))
    after = tracks.locate(shifted)
    assert after["tool"]-before["tool"] == pytest.approx([11, -7], abs=.3)
    assert after["anchor"] == pytest.approx(before["anchor"])
    assert after["target"] == pytest.approx(before["target"])


@pytest.mark.parametrize("entries,reason", [
    (((1, 30, 30), (3, 470, 320)), "missing"),
    (((1, 35, 30), (2, 250, 180), (3, 470, 320)), "anchor moved"),
    (((1, 30, 30), (2, 360, 180), (3, 470, 320)), "envelope"),
    (((1, 30, 30), (4, 250, 180), (3, 470, 320)), "missing"),
    (((1, 30, 30), (2, 250, 180), (2, 470, 320)), "duplicate"),
])
def test_tag_faults_refuse_instead_of_switching_targets(entries, reason):
    tracks = FeatureTracks(tag_image(), tag_specs())
    with pytest.raises(Refused, match=reason):
        tracks.locate(tag_image(entries))


def test_anchor_rotation_and_corrected_decode_refuse():
    reference = tag_image()
    tracks = FeatureTracks(reference, tag_specs())
    rotated = reference.copy()
    rotated[30:110, 30:110] = np.rot90(reference[30:110, 30:110])
    with pytest.raises(Refused, match="anchor moved"):
        tracks.locate(rotated)
    detections = tags_from_bgr(reference)
    tracker = TagTracker(reference, tag_specs()["tool"], detections)
    detections[2]["hamming"] = 1
    with pytest.raises(Refused, match="corrected"):
        tracker.locate(reference, detections)


def test_tag_detector_errors_and_duplicate_config_are_not_silent(monkeypatch):
    specs = tag_specs(); specs["target"]["tag_id"] = 2
    with pytest.raises(Refused, match="distinct"):
        FeatureTracks(tag_image(), specs)
    import farm.perception.tags as shared
    def unavailable(): raise ImportError("not installed")
    monkeypatch.setattr(shared, "_get_detector", unavailable)
    assert detect_tags(Reading(tag_image(), Status.OK)).status is Status.UNKNOWN
    with pytest.raises(Refused, match="unavailable"):
        FeatureTracks(tag_image(), tag_specs())


def test_manifest_observer_and_seed_cli_use_shared_tags(tmp_path):
    rig = PixelRig(tmp_path / "frames")
    image = tag_image()
    for cam in rig.config["cameras"].values():
        cv2.imwrite(cam["reference"], image)
        cam["regions"] = {}
    # Feed real detector frames through the existing coherent-image publisher.
    rig.render = lambda: ({n: image for n in rig.config["cameras"]}, {})
    rig.publish()
    config_path = tmp_path / "experiment.json"
    rig.config.update(session_dir=str(tmp_path / "session"), calibration_file=str(tmp_path / "calibration.json"))
    atomic_json(config_path, rig.config)
    for camera in rig.config["cameras"]:
        assert cli.main(["seed", "--config", str(config_path), "--camera", camera,
                         "--tag", "anchor:1", "--tag", "tool:2", "--tag", "target:3"]) == 0
    config = json.loads(config_path.read_text())
    observer = Observer(config, rig.limits, clock=rig.clock)
    observation = observer.observe()
    assert observation.values == pytest.approx([220, 140, 220, 140], abs=.4)
    assert observation.sequences == {"head": 1, "right_wrist": 1}
    assert rig.path_ticks == 0


@pytest.fixture(scope="module")
def model_folder():
    folder = Path(os.environ.get("CARTON_MODEL_DIR", "data-carton/models/so101")).resolve()
    if not folder.exists():
        pytest.skip("Run carton.servo model-fetch and set CARTON_MODEL_DIR for real URDF integration tests")
    pytest.importorskip("placo")
    verified_model(folder)
    return folder


@pytest.fixture(scope="module")
def solver(model_folder):
    return LeRobotSO101(model_folder)


def calibrated_config(tmp_path, model_folder):
    calibration = tmp_path / "calibration.json"
    raw = {f"right_arm_{n}": {"range_min": 1000, "range_max": 3100}
           for n in manifest()["joint_names"] + ["gripper"]}
    atomic_json(calibration, raw)
    return {"schema": 1, "arm": "right", "model_directory": str(model_folder),
            "calibration_file": str(calibration), "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
            "joints": {n: {"model_zero_tick": 2048, "model_sign": -1 if n == "elbow_flex" else 1}
                       for n in manifest()["joint_names"]},
            "gripper_from_tool": np.eye(4).tolist(), "base_from_station": None,
            "workspace_bounds_m": [[-.5, -.5, -.1], [.6, .6, .7]]}


@pytest.mark.parametrize("target", [[10, -15, 25, -10, 5], [-20, 25, -30, 15, -10], [35, -20, 40, -20, 15]])
def test_actual_lerobot_fk_ik_residual(solver, target):
    desired = solver.forward(target)
    solved = solver.solve([0]*5, desired)
    position, orientation = pose_error(solver.forward(solved["degrees"]), desired)
    assert position <= .001 and orientation <= 2
    assert solved["iterations"] < 20
    assert solver.provenance()["solver"] == "lerobot.model.kinematics.RobotKinematics"


def test_upstream_rejects_unreachable_and_out_of_joint_range(solver):
    desired = solver.forward([0]*5); desired[:3, 3] = [3, 3, 3]
    with pytest.raises((ValueError, RuntimeError)):
        solver.solve([0]*5, desired)
    with pytest.raises(ValueError, match="joint limits"):
        solver.forward([0, 0, 200, 0, 0])


@pytest.mark.parametrize("kind", ["reflection", "scale", "nan", "last_row"])
def test_nonrigid_frames_refuse(kind):
    frame = np.eye(4)
    if kind == "reflection": frame[0, 0] = -1
    elif kind == "scale": frame[1, 1] = 2
    elif kind == "nan": frame[0, 3] = np.nan
    else: frame[3, 3] = 2
    with pytest.raises(ValueError): transform(frame)


def test_calibrated_reach_includes_tool_and_station_transforms(tmp_path, model_folder):
    config = calibrated_config(tmp_path, model_folder)
    config["gripper_from_tool"][0][3] = .035
    station = np.eye(4); station[:3, 3] = [.02, -.03, .01]
    config["base_from_station"] = station.tolist()
    arm = CalibratedArm(config)
    start = {f"right_arm_{n}": 2048 for n in arm.solver.names}
    target = arm.solver.forward([10, -15, 25, -10, 5]) @ arm.gripper_from_tool
    request = {"frame": "station", "units": "metres", "orientation": "constrained",
               "tool_poses": [(np.linalg.inv(station) @ target).tolist()]}
    result = arm.plan(start, request)
    row = result["waypoints"][0]
    assert row["position_error_m"] < .002 and row["orientation_error_deg"] < 2.5
    assert row["joint_targets_ticks"]["right_arm_elbow_flex"] < 2048
    assert result["motor_writes"] == 0 and result["collision_checked"] is False
    assert pose_error(np.array(row["tool_pose"]), target)[0] < .002
    request["orientation"] = "position_only"
    with pytest.raises(ValueError, match="offset tool"):
        arm.plan(start, request)


def test_missing_zeros_hash_or_registration_and_workspace_refuse(tmp_path, model_folder):
    config = calibrated_config(tmp_path, model_folder)
    missing = copy.deepcopy(config); missing["joints"]["shoulder_pan"]["model_zero_tick"] = None
    with pytest.raises(ValueError, match="measure"):
        CalibratedArm(missing)
    with pytest.raises(ValueError, match="calibration changed"):
        CalibratedArm({**config, "calibration_sha256": "wrong"})
    arm = CalibratedArm(config)
    start = {f"right_arm_{n}": 2048 for n in arm.solver.names}
    request = {"frame": "station", "units": "metres", "orientation": "constrained",
               "tool_poses": [arm.forward(start).tolist()]}
    with pytest.raises(ValueError, match="base_from_station"):
        arm.plan(start, request)
    request["frame"] = "arm_base"; request["tool_poses"][0][0][3] = .7
    with pytest.raises(ValueError, match="workspace"):
        arm.plan(start, request)


def test_asset_checks_preserve_conflicts_and_verify_downloads(tmp_path, monkeypatch):
    import farm.kinematics.assets as assets
    data = b"pinned upstream bytes"
    model = {"repository": "example/model", "revision": "a"*40, "urdf": "model.urdf",
             "files": [{"path": "model.urdf", "source_path": "model.urdf", "bytes": len(data),
                        "sha256": hashlib.sha256(data).hexdigest()}]}
    monkeypatch.setattr(assets, "manifest", lambda: model)
    from io import BytesIO
    monkeypatch.setattr(assets, "urlopen", lambda *a, **k: BytesIO(b"corrupt"))
    with pytest.raises(ValueError, match="checksum"): fetch_model(tmp_path)
    assert not (tmp_path / "model.urdf").exists()
    monkeypatch.setattr(assets, "urlopen", lambda *a, **k: BytesIO(data))
    assert fetch_model(tmp_path)["status"] == "UPSTREAM_MODEL_VERIFIED"
    (tmp_path / "model.urdf").write_text("local edits")
    with pytest.raises(ValueError, match="conflicting"): fetch_model(tmp_path)
    assert (tmp_path / "model.urdf").read_text() == "local edits"
    with pytest.raises(ValueError, match="changed"): verified_model(tmp_path)


def test_real_model_assets_and_check_evidence(model_folder, tmp_path):
    import shutil
    folder = tmp_path / "model"
    shutil.copytree(model_folder, folder)
    mesh = next(v for v in manifest()["files"] if v["path"].endswith(".stl"))
    (folder / mesh["path"]).write_bytes(b"changed mesh")
    with pytest.raises(ValueError, match="changed"): LeRobotSO101(folder)
    result = check_model(model_folder)
    assert result["environment"] == "URDF_NUMERICAL_ONLY"
    assert result["robot_connected"] is False and result["physical_task_completed"] is False


def test_read_only_cli_binds_real_kinematics_to_existing_session(tmp_path, model_folder):
    config = calibrated_config(tmp_path, model_folder)
    rig = PixelRig(tmp_path / "frames")
    session = tmp_path / "session"; session.mkdir()
    rig.config.update(session_dir=str(session), calibration_file=config["calibration_file"],
                      calibration_sha256=config["calibration_sha256"])
    experiment_path = tmp_path / "experiment.json"; atomic_json(experiment_path, rig.config)
    kin_path = tmp_path / "kinematics.json"
    assert template(rig.config, model_folder, kin_path)["status"] == "NEEDS_GEOMETRIC_CALIBRATION"
    with pytest.raises(ValueError, match="measure"): load_arm(kin_path, rig.config)
    with pytest.raises(Refused, match="preserved"): template(rig.config, model_folder, kin_path)
    atomic_json(kin_path, config)
    arm, _ = load_arm(kin_path, rig.config)
    request = {"frame": "arm_base", "units": "metres", "orientation": "constrained",
               "tool_poses": [arm.solver.forward([3, -3, 3, -3, 3]).tolist()]}
    request_path = tmp_path / "request.json"; atomic_json(request_path, request)
    status = {"arm": "right", "phase": "holding", "ok": True, "started": time.time()-10, "lease_remaining": 180,
              "rows": {n: {"Present_Position": q, "Present_Load": 0, "Status": 0}
                       for n, q in rig.q.items()}}
    rig.now = time.time()-.2; rig.publish()
    atomic_json(session / "status.json", {**status, "time": time.time()})
    # Like the real session/camera owners, this synthetic publisher must live
    # outside the IK process: native model construction can hold Python's GIL.
    publisher = subprocess.Popen([sys.executable, "-c", """
import json, pathlib, sys, time
paths = [pathlib.Path(p) for p in sys.argv[1:]]
while True:
    for path in paths:
        value = json.loads(path.read_text())
        if 'seq' in value:
            value.update(seq=value['seq']+1, captured_at=time.time())
        else:
            value['time'] = time.time()
        temporary = path.with_suffix('.publishing')
        temporary.write_text(json.dumps(value)); temporary.replace(path)
    time.sleep(.08)
""", str(session / "status.json"), *[c["manifest"] for c in rig.config["cameras"].values()]])
    output = tmp_path / "proposal"
    try:
        assert cli.main(["plan-reach", "--config", str(experiment_path), "--kinematics", str(kin_path),
                         "--request", str(request_path), "--out", str(output)]) == 0
    finally:
        publisher.terminate(); publisher.wait(timeout=2)
    result = json.loads((output / "result.json").read_text())
    assert result["status"] == "KINEMATIC_PROPOSAL_ONLY"
    assert result["kinematics_fingerprint"] and result["camera_sequences"]
    assert not (session / "command.json").exists()
    with pytest.raises(Refused, match="different arm"):
        load_arm(kin_path, {**rig.config, "arm": "left"})
    wrong = copy.deepcopy(rig.config); wrong["ranges"]["right_arm_shoulder_pan"] = [900, 3100]
    with pytest.raises(Refused, match="ranges differ"): load_arm(kin_path, wrong)
