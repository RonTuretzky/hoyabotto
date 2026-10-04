"""Evidence-bound visual controller tests. No camera or motor device is opened."""
import hashlib
import json
import math
from dataclasses import replace

import cv2
import numpy as np
import pytest

from carton.servo.common import Limits, Refused, Trace, atomic_json, binding, read_json, validate_config
from carton.servo.controller import Experiment, fit_model
from carton.servo.geometry import JointUnits, hinge_path, verify_grasp
from carton.servo.simulation import PixelRig, RenderedObserver, run
from carton.servo.transport import SessionTransport
from carton.servo.vision import ManifestCamera, Observer, RegionTracker
from farm.vendor.so101_kinematics import SO101Kinematics


def test_rendered_pixels_identification_holdout_and_control(tmp_path):
    result = run(tmp_path)
    assert result["status"] == "ALIGNED_ONLY"
    assert result["steps"] < 30
    assert max(abs(x) for x in result["error_px"]) <= 3
    assert result["environment"] == "SYNTHETIC_RENDERED_PIXELS"
    assert result["robot_connected"] is False
    assert result["grasp_verified"] is False and result["physical_task_completed"] is False
    model = json.loads((tmp_path / "synthetic-model.json").read_text())
    assert len(model["holdout"]) == 4 and len(model["samples"]) == 8
    assert model["condition"] < 10
    assert len(list((tmp_path / "run").glob("frame-*/head.jpg"))) > 20


@pytest.mark.parametrize("xy", [(.15, .1), (.1629, .1131), (.2, .05), (.1, .15), (.08, .04)])
def test_real_inverse_forward_regressions(xy):
    kin = SO101Kinematics()
    q = kin.inverse_kinematics(*xy)
    assert kin.forward_kinematics(*q) == pytest.approx(xy, abs=1e-10)


@pytest.mark.parametrize("xy", [(0, 0), (1, 1), (.001, .001), (math.nan, .1)])
def test_inverse_does_not_silently_clamp_unreachable_targets(xy):
    with pytest.raises(ValueError):
        SO101Kinematics().inverse_kinematics(*xy)


def test_units_require_measured_zero_and_are_not_degrees():
    u = JointUnits(1000, 3000)
    assert u.normalized_to_ticks(50) == 2500
    assert u.ticks_to_normalized(2500) == 50
    with pytest.raises(Refused, match="zero"):
        u.ticks_to_model_degrees(2500)
    u = JointUnits(1000, 3000, model_zero_tick=2048, model_sign=-1)
    assert u.ticks_to_model_degrees(2500) != 50
    assert u.model_degrees_to_ticks(u.ticks_to_model_degrees(2500)) == pytest.approx(2500)
    assert JointUnits(1000, 3000, gripper=True).normalized_to_ticks(50) == 2000
    with pytest.raises(Refused): u.normalized_to_ticks(101)


@pytest.mark.parametrize("override", [{"step_ticks": 69}, {"probe_ticks": True}, {"temperature_c": 56},
                                      {"load_raw": 501}, {"frame_age_s": math.nan}, {"max_seconds": 1000}])
def test_safety_limits_cannot_be_relaxed(override):
    with pytest.raises(Refused): Limits(**override)


def test_hinge_arc_has_constant_radius_and_no_motor_commands():
    plan = hinge_path([0, 0, 0], [1, 0, 0], [0, .14, 0], 0, 90)
    points = np.array(plan["contact_points"])
    assert np.linalg.norm(points, axis=1) == pytest.approx(np.full(len(points), .14))
    assert points[-1] == pytest.approx([0, 0, .14])
    assert plan["motor_commands"] == [] and plan["contact_validated"] is False


def test_grasp_needs_object_motion_and_nonempty_aperture():
    before = {"tool": [100, 100], "target": [110, 110]}
    moving = {"tool": [100, 80], "target": [110, 90]}
    stationary = {"tool": [100, 80], "target": [110, 110]}
    assert verify_grasp(before, moving, 1500, 1400, 30)["verified"]
    assert not verify_grasp(before, stationary, 1500, 1400, 30)["verified"]
    assert not verify_grasp(before, moving, 1400, 1400, 30)["verified"]
    assert not verify_grasp(before, before, 1500, 1400, 30)["verified"]


def test_manifest_rejects_corruption_stale_frames_wrong_identity_and_restart(tmp_path):
    rig = PixelRig(tmp_path)
    rig.publish()
    path = tmp_path / "head.json"
    reader = ManifestCamera(path, "synthetic-head", clock=rig.clock)
    assert reader.read().seq == 1
    original = json.loads(path.read_text())
    for changes, pattern in [({"sha256": "bad"}, "bytes"), ({"captured_at": rig.now-4}, "age"),
                             ({"captured_at": rig.now+1}, "age"), ({"seq": -1}, "sequence"),
                             ({"camera_id": "wrong"}, "identity"), ({"image": "../secret.jpg"}, "inside")]:
        atomic_json(path, {**original, **changes})
        with pytest.raises(Refused, match=pattern): reader.read()
    atomic_json(path, {**original, "stream_id": "restarted"})
    with pytest.raises(Refused, match="restarted"): reader.read()


def test_distinct_post_motion_camera_frames_are_required(tmp_path):
    rig = PixelRig(tmp_path)
    rig.publish()
    obs = Observer(rig.config, rig.limits, clock=rig.clock, sleep=lambda s: setattr(rig, "now", rig.now+s))
    obs.observe()
    with pytest.raises(Refused, match="distinct"):
        obs.observe(timeout=.1)
    rig.publish()
    with pytest.raises(Refused, match="post-command"):
        obs.observe(after=rig.now+1, timeout=.1)


@pytest.mark.parametrize("fault", ["occlusion", "camera_shift"])
def test_tracker_stops_on_target_loss_or_camera_movement(tmp_path, fault):
    rig = PixelRig(tmp_path)
    obs = RenderedObserver(rig)
    obs.observe()
    rig.fail = fault
    with pytest.raises(Refused): obs.observe()


def test_duplicate_patch_cannot_be_silently_selected(tmp_path):
    rig = PixelRig(tmp_path)
    im = rig.images["head"].copy()
    spec = rig.config["cameras"]["head"]["regions"]["target"]
    tracker = RegionTracker(im, spec)
    im[168:192, 338:362] = rig.patches["target"]
    with pytest.raises(Refused, match="ambiguous"):
        tracker.locate(im)


def test_fit_rejects_unobservable_and_inconsistent_responses():
    samples = [{"dq": [32, 0], "dy": [10, 0]}, {"dq": [-32, 0], "dy": [-10, 0]},
               {"dq": [0, 32], "dy": [10, 0]}, {"dq": [0, -32], "dy": [-10, 0]}]
    with pytest.raises(Refused, match="distinguish"):
        fit_model(samples, ["j1", "j2"], Limits())
    samples[2]["dy"], samples[3]["dy"] = [0, 10], [0, 10]
    with pytest.raises(Refused): fit_model(samples, ["j1", "j2"], Limits())


def test_stuck_motor_refuses_calibration_and_no_fake_model(tmp_path):
    rig = PixelRig(tmp_path / "rig"); rig.fail = "stuck_motor"
    trace = Trace(tmp_path / "trace")
    try:
        e = Experiment(rig.config, rig, RenderedObserver(rig), trace, binding(rig.config), rig.clock)
        with pytest.raises(Refused, match="insufficient"):
            e.calibrate()
    finally: trace.close()


def test_model_binding_and_scene_change_prevent_reuse(tmp_path):
    rig = PixelRig(tmp_path / "rig")
    trace = Trace(tmp_path / "trace")
    try:
        e = Experiment(rig.config, rig, RenderedObserver(rig), trace, binding(rig.config), rig.clock)
        model = e.calibrate()
        before = rig.path_ticks
        with pytest.raises(Refused, match="changed"):
            e.align({**model, "fingerprint": "different"})
        assert rig.path_ticks == before
        rig.started += 1
        with pytest.raises(Refused, match="session changed"):
            e.align(model)
    finally: trace.close()


class FileMotorOwner:
    """Emulate the actual file protocol, independently of the controller math."""
    def __init__(self, folder, mode="ok"):
        self.folder, self.mode, self.now, self.seen = folder, mode, 1000.0, None
        folder.mkdir()
        self.q = {f"right_arm_{n}": 2000 for n in
                  ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
        self.s = {"arm": "right", "phase": "holding", "ok": True, "started": 900, "lease_remaining": 180}
        self.publish()
        cal = folder / "calibration.json"; cal.write_text('{}')
        self.config = {"session_dir": str(folder), "arm": "right", "calibration_file": str(cal),
                       "calibration_sha256": hashlib.sha256(cal.read_bytes()).hexdigest(),
                       "joints": ["right_arm_shoulder_pan"], "ranges": {n: [1000, 3000] for n in self.q}}

    def clock(self): return self.now

    def publish(self):
        self.s.update(time=self.now, rows={n: {"Present_Position": v, "Present_Load": 20,
                                              "Present_Temperature": 35, "Status": 0} for n, v in self.q.items()})
        atomic_json(self.folder / "status.json", self.s)

    def sleep(self, seconds):
        self.now += seconds
        path = self.folder / "command.json"
        if path.exists():
            cmd = json.loads(path.read_text())
            if cmd["id"] != self.seen:
                self.seen = cmd["id"]
                if cmd["op"] == "move":
                    n, delta = next(iter(cmd["delta_ticks"].items()))
                    if self.mode != "stuck": self.q[n] += delta
                    if self.mode == "drift": self.q["right_arm_elbow_flex"] += 20
                    self.s["completed"] = cmd["id"] if self.mode != "wrong_ack" else cmd["id"]-1
        self.publish()


def test_protocol_requires_exact_ack_and_measured_settling(tmp_path):
    owner = FileMotorOwner(tmp_path / "session")
    with SessionTransport(owner.config, Limits(), True, owner.clock, owner.sleep) as t:
        q, stamp = t.move("right_arm_shoulder_pan", 16)
        assert q["right_arm_shoulder_pan"] == 2016 and stamp > 1000
        with pytest.raises(Refused): t.move("base_left_wheel", 1)
        with pytest.raises(Refused): t.move("right_arm_gripper", 1)


@pytest.mark.parametrize("mode", ["wrong_ack", "stuck", "drift"])
def test_protocol_faults_do_not_look_like_success(tmp_path, mode):
    owner = FileMotorOwner(tmp_path / "session", mode)
    with SessionTransport(owner.config, Limits(command_timeout_s=.3), True, owner.clock, owner.sleep) as t:
        with pytest.raises(Refused): t.move("right_arm_shoulder_pan", 16)
        t.abort()
    assert json.loads((owner.folder / "command.json").read_text())["op"] == "stop"


def test_read_only_and_competing_writer_send_no_move(tmp_path):
    owner = FileMotorOwner(tmp_path / "session")
    with SessionTransport(owner.config, Limits(), False, owner.clock, owner.sleep) as t:
        with pytest.raises(Refused, match="Read-only"): t.move("right_arm_shoulder_pan", 16)
    assert not (owner.folder / "command.json").exists()
    with SessionTransport(owner.config, Limits(), True, owner.clock, owner.sleep) as t:
        atomic_json(owner.folder / "command.json", {"id": 999, "op": "hold"})
        with pytest.raises(Refused, match="writer"): t.move("right_arm_shoulder_pan", 16)
    assert json.loads((owner.folder / "command.json").read_text())["id"] == 999


def test_stale_status_and_changed_calibration_refuse_before_motion(tmp_path):
    owner = FileMotorOwner(tmp_path / "session")
    t = SessionTransport(owner.config, Limits(), False, owner.clock, owner.sleep)
    owner.now += 3
    with pytest.raises(Refused, match="stale"): t.positions()
    owner.publish()
    (owner.folder / "calibration.json").write_text('{"changed":true}')
    with pytest.raises(Refused, match="calibration changed"): t.positions()
    assert not (owner.folder / "command.json").exists()


def test_duplicate_cameras_and_missing_anchor_are_rejected(tmp_path):
    rig = PixelRig(tmp_path)
    validate_config(rig.config)
    rig.config["cameras"]["right_wrist"]["camera_id"] = "synthetic-head"
    with pytest.raises(Refused, match="distinct"): validate_config(rig.config)
    rig.config["cameras"]["right_wrist"]["camera_id"] = "synthetic-right_wrist"
    del rig.config["cameras"]["head"]["regions"]["anchor"]
    with pytest.raises(Refused, match="anchor"): validate_config(rig.config)


def test_capture_timestamp_cannot_be_refreshed_without_a_new_frame(tmp_path):
    rig = PixelRig(tmp_path); rig.publish()
    path = tmp_path / "head.json"
    reader = ManifestCamera(path, "synthetic-head", clock=rig.clock)
    reader.read()
    manifest = json.loads(path.read_text())
    rig.now += .1
    atomic_json(path, {**manifest, "captured_at": rig.now})
    with pytest.raises(Refused, match="reused"): reader.read()
    atomic_json(path, {**manifest, "seq": manifest["seq"]+1})
    with pytest.raises(Refused, match="timestamps"): reader.read()


def test_stale_or_moved_robot_cannot_use_an_old_observation(tmp_path):
    rig = PixelRig(tmp_path / "rig"); trace = Trace(tmp_path / "trace")
    try:
        e = Experiment(rig.config, rig, RenderedObserver(rig), trace, binding(rig.config), rig.clock)
        e.observe(); rig.now += 2
        with pytest.raises(Refused, match="stale"): e.move(rig.joints[0], 16)
        e.observe(); rig.q[rig.joints[1]] += 10
        with pytest.raises(Refused, match="moved after"): e.move(rig.joints[0], 16)
        assert rig.path_ticks == 0
    finally: trace.close()


def test_model_must_match_samples_holdouts_and_camera_stream(tmp_path):
    import copy
    rig = PixelRig(tmp_path / "rig"); trace = Trace(tmp_path / "trace")
    try:
        e = Experiment(rig.config, rig, RenderedObserver(rig), trace, binding(rig.config), rig.clock)
        model = e.calibrate(); traveled = rig.path_ticks
        for field in ("jacobian", "camera_streams", "holdout"):
            bad = copy.deepcopy(model)
            if field == "jacobian": bad[field][0][0] += .1
            elif field == "camera_streams": bad[field]["head"] = "restarted"
            else: bad[field][0]["dy"][0] += 20
            with pytest.raises(Refused): e.align(bad)
        assert rig.path_ticks == traveled
    finally: trace.close()


def test_lock_released_on_failed_entry_and_abort_on_exception(tmp_path):
    owner = FileMotorOwner(tmp_path / "session")
    owner.now += 3
    t = SessionTransport(owner.config, Limits(), True, owner.clock, owner.sleep)
    with pytest.raises(Refused):
        with t: pass
    assert t.lock is None
    owner.publish()
    with pytest.raises(ValueError):
        with SessionTransport(owner.config, Limits(), True, owner.clock, owner.sleep) as second:
            second.move("right_arm_shoulder_pan", 16)
            raise ValueError("tracking failed")
    assert read_json(owner.folder / "command.json")["op"] == "stop"


@pytest.mark.parametrize("changes", [{"min_aperture_ticks": -1}, {"min_lift_px": math.nan},
                                     {"max_slip_px": 10}, {"object_key": "tool"}])
def test_bad_grasp_evidence_thresholds_do_not_pass(changes):
    args = dict(before={"tool": [0, 0], "target": [1, 1]}, after={"tool": [0, 20], "target": [1, 21]},
                gripper_ticks=1500, empty_closed_ticks=1400, min_aperture_ticks=30)
    with pytest.raises(Refused): verify_grasp(**{**args, **changes})


def test_legacy_cartesian_and_llm_paths_refuse_before_physical_motion(tmp_path):
    from farm.adapters.sim import FakeRobot
    from farm.config import LimitsCfg, ArmsCfg
    from farm.skills.keyframes import KeyframeStore
    from farm.skills.runner import SkillRunner
    from farm.skills.arm import ArmPose
    from farm.skills.llm_servo import LLMServo
    from farm.safety.rules import SafetyStop
    robot = FakeRobot()
    runner = SkillRunner(robot, LimitsCfg(), ArmsCfg(), KeyframeStore(tmp_path / "poses.yaml"))
    with pytest.raises(SafetyStop, match="normalized"): runner.move_arm_pose("right", ArmPose())
    with pytest.raises(SafetyStop, match="rest pose"): runner.go_rest()
    outcome = LLMServo(runner, {}, None).run("right", "grasp")
    assert not outcome.ok and outcome.steps == 0 and robot.sent == []


def test_synthetic_result_cannot_be_saved_as_a_physical_skill(tmp_path):
    from carton.servo.skills import save_alignment
    rig = PixelRig(tmp_path / "rig")
    run_dir = tmp_path / "run"; run_dir.mkdir()
    atomic_json(run_dir / "result.json", {"status": "ALIGNED_ONLY", "environment": "SYNTHETIC_RENDERED_PIXELS"})
    with pytest.raises(Refused, match="guarded-session"):
        save_alignment(rig.config, run_dir, tmp_path / "skill.json", "paddle-approach")
    assert not (tmp_path / "skill.json").exists()
