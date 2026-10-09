"""Offline contract tests: no Robot import, camera, network, or motor access."""
from __future__ import annotations

import copy
import json
import signal
import time

import cv2
import numpy as np
import pytest

from carton.servo.common import Refused, digest
from carton.servo.gemma import FRAME_READ_ATTEMPTS, GemmaTransport
from farm.kinematics import tag_registration
from farm.perception.tag_sampling import ARM_JOINTS
from farm.perception.gemma_tags import TagObserver
from test_gemma_calibration import Owner
from tools import collect_stationary_tag_pose as collector
from tools.collect_stationary_tag_pose import ReadOnlyRobot, collect_stationary_pose


class StationaryOwner(Owner):
    """The fake owner has fresh independent wall/monotonic clocks and no I/O."""

    def __init__(self, arm="right"):
        super().__init__(arm)
        self.responses = []
        self.sleeps = []
        self.on_response = None
        self.on_sleep = None
        self.monotonic_now = 0.0
        self.config_hash = digest("unchanged motor calibration")
        self.model_revision = "offline-pinned-model"
        self.call_delay = 0.02

    def monotonic(self):
        return self.monotonic_now

    def sleep(self, seconds):
        assert 0 < seconds <= 1.0
        self.sleeps.append(seconds)
        self.now += seconds
        self.monotonic_now += seconds
        self.lease = max(0, self.lease - seconds)
        if self.on_sleep:
            self.on_sleep(self)

    def call(self, name, args, request_id=None):
        # This assertion fires before Owner's deliberately available motor calls.
        assert name in {
            "robot_get_capabilities", "robot_get_state", "robot_get_execution",
            "robot_get_arm_pose", "robot_get_tags", "robot_get_cameras",
        }, f"The offline collector tried a forbidden operation: {name}"
        self.now += self.call_delay - 0.02
        self.monotonic_now += self.call_delay
        payload = super().call(name, args, request_id)
        if name == "robot_get_arm_pose":
            configuration = payload["result"]["configuration"]
            configuration["config"]["calibration_sha256"] = self.config_hash
            configuration["model_assets"] = {
                "verified": True, "revision": self.model_revision,
            }
        if name == "robot_get_tags":
            row = payload["result"]["observations"]["oak"]
            row["frame"]["received_at"] = self.now
            row["pose_3d"]["calibration_sha256"] = digest("camera calibration")
            row["pose_3d"]["geometry_config_sha256"] = digest("tag geometry")
        if self.on_response:
            self.on_response(self, name, payload)
        self.responses.append((name, copy.deepcopy(payload)))
        return payload


def collect(owner, path, **kwargs):
    return collect_stationary_pose(
        owner, path, arm=owner.arm, clock=owner.clock,
        monotonic=owner.monotonic, sleep=owner.sleep,
        tag_robot_factory=lambda read_only: read_only, **kwargs,
    )


def read_json(path):
    return json.loads(path.read_text())


def samples_in(path):
    return [read_json(p) for p in sorted(path.glob("sample-*/sample.json"))]


def assert_read_only(owner):
    assert owner.writes == owner.command == 0
    assert not owner.stopped and not owner.enabled
    assert all(name.startswith("robot_get_") for name, _ in owner.calls)
    for name, args in owner.calls:
        if name == "robot_get_tags":
            assert args == {
                "cameras": ["oak"], "tag_ids": [1, owner.tag_id], "include_images": False,
            }
        elif name == "robot_get_cameras":
            assert args == {"cameras": ["oak"], "revive": False}


def assert_rejected(owner, path, report):
    assert report["status"] == "REJECTED"
    assert report["calibration_ready"] is False
    assert report["distinct_calibration_poses"] == 0
    assert report["physical_mapping_validated"] is False
    assert report["error"]
    assert read_json(path / "report.json")["status"] == "REJECTED"
    assert (path / "failure.json").is_file()
    assert_read_only(owner)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_five_independent_brackets_preserve_original_evidence_and_one_pose(tmp_path, arm):
    owner = StationaryOwner(arm)
    path = tmp_path / "settled"
    report = collect(owner, path)
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert report["accepted_samples"] == 5
    assert report["distinct_calibration_poses"] == 1
    assert report["calibration_ready"] is report["physical_mapping_validated"] is False
    assert report["encoder_same"] is True and report["motor_writes"] == 0
    samples = samples_in(path)
    assert len(samples) == len({s["sample_id"] for s in samples}) == 5
    assert all(s["split"] is None and s["base_from_gripper"] is None for s in samples)
    assert report["seq"] == [s["frame"]["seq"] for s in samples]
    limits = read_json(path / "invocation.json")["limits"]
    assert limits["frame_age_s"] == 1.0 and limits["status_age_s"] == .75
    assert all(b["frame"]["captured_at"] - a["frame"]["captured_at"] >= 1.0
               for a, b in zip(samples, samples[1:]))
    assert owner.sleeps and report["elapsed_s"] >= 4.0
    tags = [p["result"]["observations"]["oak"] for n, p in owner.responses
            if n == "robot_get_tags"]
    for folder, sample, observed in zip(sorted(path.glob("sample-*")), samples, tags):
        assert sample["frame"] == observed["frame"]
        before, after = read_json(folder / "before.json"), read_json(folder / "after.json")
        assert before in [p for n, p in owner.responses if n == "robot_get_state"]
        assert after in [p for n, p in owner.responses if n == "robot_get_state"]
        assert max(r["captured_at"] for r in before["result"]["motors"]) <= sample["frame"]["captured_at"]
        assert min(r["captured_at"] for r in after["result"]["motors"]) >= sample["frame"]["captured_at"]
        assert len(before["result"]["motors"]) == len(after["result"]["motors"]) == 16
        assert read_json(folder / "observation.json")["result"]["observations"]["oak"] == observed
        assert read_json(folder / "arm-geometry-status.json")["result"]["configuration"]["model_assets"]["verified"]
        assert (folder / "owner-before.json").is_file() and (folder / "owner-after.json").is_file()
        capture = read_json(folder / "capture.json")
        assert capture["sample"] == sample and capture["before"] == before and capture["after"] == after
        assert "arm_geometry_status" in capture
    assert_read_only(owner)


def test_actual_pose_variation_is_reported_without_calling_it_encoder_motion(tmp_path):
    owner = StationaryOwner()
    translations = []
    def varied(o, name, payload):
        if name != "robot_get_tags":
            return
        pose = np.eye(4)
        pose[:3, 3] = [.2 + o.seq * .001, .02, .3]
        pose[:3, :3] = cv2.Rodrigues(np.array([0.0, 0.0, np.deg2rad(o.seq)]))[0]
        payload["result"]["observations"]["oak"]["pose_3d"]["tags"][1]["camera_from_tag"] = pose.tolist()
        translations.append(pose[:3, 3] * 1000)
    owner.on_response = varied
    report = collect(owner, tmp_path / "variation")
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert report["position_std_mm"] == pytest.approx(np.std(translations, axis=0))
    assert report["position_peak_to_peak_mm"] == pytest.approx([4, 0, 0])
    assert report["pairwise_rotation_deg_max"] == pytest.approx(4)
    assert report["encoder_same"] is True and report["distinct_calibration_poses"] == 1


def test_saved_capture_is_usable_by_existing_dataset_assembler(tmp_path, monkeypatch):
    owner = StationaryOwner()
    path = tmp_path / "compatible"
    assert collect(owner, path)["status"] == "SETTLED_POSE_CAPTURED"
    model = tmp_path / "fake.urdf"
    model.write_text("offline model fixture")
    class Solver:
        names = ARM_JOINTS[:-1]
        def __init__(self, folder):
            pass
        def forward(self, q):
            return np.eye(4)
    monkeypatch.setattr(tag_registration, "LeRobotSO101", Solver)
    monkeypatch.setattr(tag_registration, "verified_model", lambda _: (model, {"revision": owner.model_revision}))
    monkeypatch.setattr(tag_registration, "candidate_degrees", lambda ticks, ranges, names: [0.] * len(names))
    dataset = tag_registration.assemble_dataset(
        [read_json(p) for p in sorted(path.glob("sample-*/capture.json"))], tmp_path,
    )
    assert len(dataset["samples"]) == 5
    assert dataset["binding"]["motor_calibration_sha256"] == owner.config_hash
    assert dataset["binding"]["arm"] == owner.arm
    assert all(sample["split"] is None for sample in dataset["samples"])
    with pytest.raises(ValueError, match="eight fitting poses"):
        tag_registration.fit_registration(dataset)


def test_eleven_distinct_frames_at_one_pose_cannot_be_fitted_as_diverse_poses(tmp_path):
    owner = StationaryOwner()
    path = tmp_path / "one-pose-eleven-frames"
    report = collect(owner, path, samples=11)
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert report["accepted_samples"] == 11 and report["distinct_calibration_poses"] == 1
    assert report["calibration_ready"] is False
    samples = samples_in(path)
    assert all(sample["split"] is None for sample in samples)
    for i, sample in enumerate(samples):
        sample["split"] = "train" if i < 8 else "validation"
        sample["base_from_gripper"] = np.eye(4).tolist()
    first = samples[0]
    binding = {
        "arm": owner.arm, "camera_id": first["frame"]["camera_id"],
        "camera_calibration_sha256": first["camera_calibration_sha256"],
        "tag_geometry_sha256": first["tag_geometry_sha256"],
        "robot_model_sha256": digest("offline model"),
        "motor_calibration_sha256": owner.config_hash,
        "encoder_mapping_source": "offline candidate model fixture",
        "gripper_tag_id": owner.tag_id, "gripper_tag_mount": first["gripper_tag_mount"],
    }
    with pytest.raises(ValueError, match="independent axes|spatially diverse"):
        tag_registration.fit_registration({
            "schema": 1, "binding": binding, "samples": samples,
            "source_stream_id": first["frame"]["stream_id"],
        })


@pytest.mark.parametrize("cached_reads", [1, 3])
def test_asynchronous_cached_frames_keep_original_bracket_until_new_frame(tmp_path, cached_reads):
    owner = StationaryOwner()
    cached = owner.call("robot_get_tags", {})["result"]["observations"]["oak"]
    reads = 0
    def delayed(o, name, payload):
        nonlocal reads
        if name == "robot_get_tags":
            if reads < cached_reads:
                payload["result"]["observations"]["oak"] = copy.deepcopy(cached)
            reads += 1
    owner.on_response = delayed
    owner.calls.clear()
    path = tmp_path / "async"
    report = collect(owner, path, samples=1)
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    capture = read_json(path / "sample-00" / "capture.json")
    assert capture["frame_reads"] == cached_reads + 1
    stamp = capture["sample"]["frame"]["captured_at"]
    assert cached["frame"]["captured_at"] < max(r["captured_at"] for r in capture["before"]["result"]["motors"]) <= stamp
    assert capture["sample"]["frame"] != cached["frame"]
    assert_read_only(owner)


def test_asynchronous_owner_telemetry_catches_up_without_restamping_frame(tmp_path):
    owner = StationaryOwner()
    lagged = False
    frame = None
    def delayed(o, name, payload):
        nonlocal lagged, frame
        if name == "robot_get_tags":
            frame = copy.deepcopy(payload["result"]["observations"]["oak"]["frame"])
        elif name == "robot_get_state" and frame and not lagged:
            for row in payload["result"]["motors"]:
                row["captured_at"] = frame["captured_at"] - .001
            lagged = True
    owner.on_response = delayed
    path = tmp_path / "catchup"
    assert collect(owner, path, samples=1)["status"] == "SETTLED_POSE_CAPTURED"
    assert lagged and samples_in(path)[0]["frame"] == frame
    assert_read_only(owner)


@pytest.mark.parametrize("motor", ["right_arm_wrist_roll", "left_arm_gripper", "head_motor_1", "base_right_wheel"])
def test_even_one_tick_of_any_motor_during_dwell_refuses(tmp_path, motor):
    owner = StationaryOwner()
    def moved(o):
        o.q[motor] += 1
    owner.on_sleep = moved
    path = tmp_path / "moved"
    report = collect(owner, path)
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] == 1


def test_transient_one_tick_move_inside_bracket_cannot_hide_by_returning(tmp_path):
    owner = StationaryOwner()
    changed = False
    def transient(o, name, payload):
        nonlocal changed
        if name == "robot_get_state" and o.seq and not changed:
            row = next(r for r in payload["result"]["motors"] if r["name"] == "left_arm_gripper")
            row["Present_Position"] += 1
            changed = True
    owner.on_response = transient
    path = tmp_path / "transient"
    report = collect(owner, path, samples=1)
    assert changed
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] == 0
    assert sum(name == "robot_get_tags" for name, _ in owner.calls) == 1


@pytest.mark.parametrize("marker", ["started", "accepted", "completed", "motor_writes", "stop_count"])
def test_owner_generation_write_and_stop_markers_must_match_across_dwell(tmp_path, marker):
    owner = StationaryOwner()
    changed = False
    owner.on_sleep = lambda _: setattr(owner, "on_sleep", None)
    def change_marker(o, name, payload):
        nonlocal changed
        if name == "robot_get_execution" and o.sleeps:
            payload["result"][marker] += 1
            changed = True
    owner.on_response = change_marker
    path = tmp_path / marker
    report = collect(owner, path)
    assert changed
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] == 1


@pytest.mark.parametrize("change", ["missing_tag", "ambiguous", "mount", "stream", "camera", "intrinsics", "geometry", "anchor", "metric_anchor", "model_revision", "motor_hash", "raw_ranges"])
def test_binding_and_anchor_changes_during_dwell_are_rejected(tmp_path, change):
    owner = StationaryOwner()
    def changed(o, name, payload):
        if not o.sleeps:
            return
        result = payload["result"]
        if name == "robot_get_arm_pose":
            if change == "model_revision":result["configuration"]["model_assets"]["revision"] = "replacement"
            if change == "motor_hash":result["configuration"]["config"]["calibration_sha256"] = digest("replacement")
        if name == "robot_get_state" and change == "raw_ranges":
            result["raw_calibration_ranges"]["right_arm_wrist_roll"]["max_ticks"] -= 1
        if name != "robot_get_tags":
            return
        row = result["observations"]["oak"]
        if change == "missing_tag":row["tags"].pop()
        if change == "ambiguous":row["pose_3d"]["tags"][1]["orientation_ambiguous"] = True
        if change == "mount":row["pose_3d"]["tags"][1]["mount"]["source"] = "replacement fixture"
        if change == "stream":row["frame"]["stream_id"] = "replacement stream"
        if change == "camera":row["frame"]["camera_id"] = "replacement camera"
        if change == "intrinsics":row["pose_3d"]["calibration_sha256"] = digest("replacement intrinsics")
        if change == "geometry":row["pose_3d"]["geometry_config_sha256"] = digest("replacement geometry")
        if change == "anchor":row["tags"][0]["corners_px"][0][0] += 3
        if change == "metric_anchor":row["pose_3d"]["tags"][0]["center_camera_mm"][0] += 6
    owner.on_response = changed
    path = tmp_path / change
    report = collect(owner, path)
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] == 1


@pytest.mark.parametrize("change", ["sequence_regression", "time_regression", "same_sequence_new_time", "new_sequence_same_time", "duplicate_hash"])
def test_reordered_or_relabelled_frames_are_rejected_and_retained(tmp_path, change):
    owner = StationaryOwner()
    first = None
    offending = None
    def changed(o, name, payload):
        nonlocal first, offending
        if name != "robot_get_tags":
            return
        frame = payload["result"]["observations"]["oak"]["frame"]
        if first is None:
            first = copy.deepcopy(frame)
            return
        if change == "sequence_regression":frame["seq"] = first["seq"] - 1
        if change == "time_regression":frame["captured_at"] = first["captured_at"] - .01
        if change == "same_sequence_new_time":frame["seq"] = first["seq"]
        if change == "new_sequence_same_time":frame["captured_at"] = first["captured_at"]
        if change == "duplicate_hash":frame["sha256"] = first["sha256"]
        offending = copy.deepcopy(frame)
    owner.on_response = changed
    path = tmp_path / change
    report = collect(owner, path, samples=2)
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] == 1
    assert offending is not None
    # Failed observations must survive even when GemmaTagObserver raises before
    # publishing a capture. Inspect the collector's persisted call journal.
    assert any(json.dumps(offending, sort_keys=True) in json.dumps(read_json(p), sort_keys=True)
               for p in path.rglob("*.json"))


def test_frozen_frame_uses_bounded_existing_attempt_limit(tmp_path):
    owner = StationaryOwner()
    frozen = owner.call("robot_get_tags", {})["result"]["observations"]["oak"]
    def frozen_frame(o, name, payload):
        if name == "robot_get_tags":payload["result"]["observations"]["oak"] = copy.deepcopy(frozen)
    owner.on_response = frozen_frame
    owner.calls.clear()
    path = tmp_path / "frozen"
    report = collect(owner, path, samples=1)
    assert_rejected(owner, path, report)
    assert sum(n == "robot_get_tags" for n, _ in owner.calls) <= FRAME_READ_ATTEMPTS
    assert len(owner.calls) < 50
    assert report["accepted_samples"] == 0


def test_existing_output_is_refused_without_overwrite_or_robot_calls(tmp_path):
    owner = StationaryOwner()
    path = tmp_path / "existing"
    path.mkdir()
    sentinel = path / "keep.txt"
    sentinel.write_text("original evidence")
    with pytest.raises((FileExistsError, ValueError, Refused)):
        collect(owner, path)
    assert sentinel.read_text() == "original evidence" and owner.calls == []


@pytest.mark.parametrize("options", [
    {"samples": 0}, {"samples": -1}, {"samples": True}, {"samples": 1.5}, {"samples": 10_000},
    {"interval_s": -1}, {"interval_s": float("nan")}, {"interval_s": float("inf")},
    {"max_seconds": 0}, {"max_seconds": float("nan")}, {"max_seconds": float("inf")},
    {"max_seconds": 1_000_000},
])
def test_invalid_or_unbounded_requests_fail_before_owner_calls(tmp_path, options):
    owner = StationaryOwner()
    try:
        report = collect(owner, tmp_path / "invalid", **options)
    except (ValueError, Refused):
        pass
    else:
        assert report["status"] == "REJECTED"
    assert owner.calls == []


def test_monotonic_time_budget_is_not_renewed_across_dwell(tmp_path):
    owner = StationaryOwner()
    owner.call_delay = .1
    path = tmp_path / "budget"
    report = collect(owner, path, samples=2, max_seconds=1.1)
    assert_rejected(owner, path, report)
    assert report["accepted_samples"] < 2
    assert owner.monotonic_now <= 2.0 and len(owner.calls) < 50


def test_expired_motion_lease_is_never_renewed_or_used_for_movement(tmp_path):
    owner = StationaryOwner()
    owner.lease = .05
    report = collect(owner, tmp_path / "lease")
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert owner.lease == 0
    assert_read_only(owner)


def test_dwell_does_not_extend_transport_deadline_or_retimestamp_observation(tmp_path, monkeypatch):
    owner = StationaryOwner()
    checks = []
    original = GemmaTransport.status
    def inspect(self):
        checks.append((self.deadline, self.observed_at, owner.seq))
        return original(self)
    monkeypatch.setattr(GemmaTransport, "status", inspect)
    report = collect(owner, tmp_path / "lifetime", samples=2)
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert len({deadline for deadline, _, _ in checks}) == 1
    during_dwell = [stamp for _, stamp, seq in checks if seq == 1 and stamp is not None]
    assert len(during_dwell) >= 3 and len(set(during_dwell)) == 1
    assert_read_only(owner)


@pytest.mark.parametrize("name,args", [
    ("robot_stop", {}), ("robot_move_motor_targets", {"positions": {}}),
    ("robot_set_motor_enable", {"enabled": False, "names": []}),
    ("robot_restart", {}), ("robot_get_unknown", {}),
    ("robot_get_cameras", {"cameras": ["oak"], "revive": True}),
    ("robot_get_cameras", {"cameras": ["phone"], "revive": False}),
    ("robot_get_tags", {"cameras": ["oak"], "tag_ids": [1, 2], "include_images": True}),
    ("robot_get_tags", {"cameras": ["oak"], "tag_ids": [1, 4]}),
    ("robot_get_tags", {"cameras": ["oak"], "tag_ids": [True, 2]}),
    ("robot_get_tags", {"cameras": ["wrist"], "tag_ids": [1, 2]}),
    ("robot_get_state", {"fresh": False}), ("robot_get_state", {"fresh": True, "revive": False}),
    ("robot_get_execution", {"restart": False}), ("robot_get_arm_pose", {"arm": "left"}),
])
def test_positive_allowlist_rejects_writes_restarts_revive_and_unlisted_arguments(name, args):
    owner = StationaryOwner()
    safe = ReadOnlyRobot(owner, arm="right")
    with pytest.raises((ValueError, Refused, PermissionError)):
        safe.call(name, args)
    assert owner.calls == []


def test_read_only_wrapper_does_not_forward_raw_attributes_or_generic_get():
    owner = StationaryOwner()
    safe = ReadOnlyRobot(owner, arm="right")
    for name in ("get", "stop", "restart", "config", "q", "enabled"):
        with pytest.raises((AttributeError, ValueError, Refused, PermissionError)):
            getattr(safe, name)
    assert owner.calls == []


def test_execute_false_transport_blocks_motion_even_with_valid_readonly_preflight(tmp_path, monkeypatch):
    owner = StationaryOwner()
    seen = []
    original = GemmaTransport.__init__
    def inspect(self, *args, **kwargs):
        seen.append(kwargs.get("execute", False))
        return original(self, *args, **kwargs)
    monkeypatch.setattr(GemmaTransport, "__init__", inspect)
    assert collect(owner, tmp_path / "transport", samples=1)["status"] == "SETTLED_POSE_CAPTURED"
    assert seen and not any(seen)
    assert_read_only(owner)


def test_default_tag_robot_native_catalog_path_is_read_only(tmp_path):
    owner = StationaryOwner()
    catalog_calls = []
    def catalog():
        catalog_calls.append(True)
        return {"tools": [
            {"type": "function", "function": {"name": "robot_get_tags"}},
            {"type": "function", "function": {"name": "robot_stop"}},
            {"type": "function", "function": {"name": "robot_restart"}},
        ]}
    owner.catalog = catalog
    report = collect_stationary_pose(
        owner, tmp_path / "native", arm=owner.arm, samples=1,
        clock=owner.clock, monotonic=owner.monotonic, sleep=owner.sleep,
    )
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert catalog_calls == [True]
    assert_read_only(owner)
    safe = ReadOnlyRobot(owner, arm=owner.arm)
    assert [t["function"]["name"] for t in safe.catalog()["tools"]] == ["robot_get_tags"]


def test_default_local_decoder_reads_only_oak_without_revive_or_images(tmp_path, monkeypatch):
    owner = StationaryOwner()
    template = owner.call("robot_get_tags", {})
    owner.calls.clear()
    original = owner.call
    def camera_call(name, args, request_id=None):
        if name != "robot_get_cameras":
            return original(name, args, request_id)
        owner.now += .02
        owner.monotonic_now += .02
        owner.calls.append((name, copy.deepcopy(args)))
        payload = {"ok": True, "result": {"cameras": {"oak": {}}}, "images": []}
        owner.responses.append((name, copy.deepcopy(payload)))
        return payload
    owner.call = camera_call
    owner.catalog = lambda: {"tools": [{"function": {
        "name": "robot_get_cameras", "parameters": {"properties": {
            "cameras": {"items": {"enum": ["oak", "phone", "wrist"]}},
            "revive": {"type": "boolean"},
        }},
    }}]}
    decoded = []
    def decode(self, payload, cameras, ids, include_images):
        assert cameras == ["oak"] and ids == [1, 2] and include_images is False
        assert payload["images"] == []
        response = copy.deepcopy(template)
        frame = response["result"]["observations"]["oak"]["frame"]
        frame.update(captured_at=owner.now, received_at=owner.now, seq=len(decoded) + 1,
                     sha256=digest(("local decoder", len(decoded))))
        decoded.append(copy.deepcopy(frame))
        return response
    monkeypatch.setattr(TagObserver, "observe", decode)
    path = tmp_path / "local"
    report = collect_stationary_pose(
        owner, path, arm=owner.arm, samples=2,
        clock=owner.clock, monotonic=owner.monotonic, sleep=owner.sleep,
    )
    assert report["status"] == "SETTLED_POSE_CAPTURED"
    assert [s["frame"] for s in samples_in(path)] == decoded
    assert sum(n == "robot_get_cameras" for n, _ in owner.calls) == 2
    assert not any(n == "robot_get_tags" for n, _ in owner.calls)
    assert_read_only(owner)


@pytest.mark.parametrize("budget,value", [
    ("MAX_CALLS", 3), ("MAX_RESPONSE_BYTES", 32), ("MAX_EVIDENCE_BYTES", 200),
])
def test_call_and_evidence_storage_limits_preserve_rejection_journal(tmp_path, monkeypatch, budget, value):
    owner = StationaryOwner()
    monkeypatch.setattr(collector, budget, value)
    path = tmp_path / budget
    report = collect(owner, path, samples=1)
    assert_rejected(owner, path, report)
    assert len(owner.calls) <= 3
    assert list((path / "calls").glob("*.json"))
    if budget != "MAX_CALLS":
        records = [read_json(p) for p in (path / "calls").glob("*.json")]
        assert any(r["status"] == "RESOURCE_REJECTED" and r["response_bytes"] > 0 for r in records)


def test_hard_deadline_interrupts_blocking_fake_call_and_restores_signal(tmp_path):
    owner = StationaryOwner()
    original = owner.call
    def blocking(name, args, request_id=None):
        time.sleep(.2)
        return original(name, args, request_id)
    owner.call = blocking
    handler = signal.getsignal(signal.SIGALRM)
    began = time.monotonic()
    path = tmp_path / "blocking"
    report = collect(owner, path, samples=1, max_seconds=.04)
    elapsed = time.monotonic() - began
    assert_rejected(owner, path, report)
    assert .02 <= elapsed < .2
    assert "hard time budget" in report["error"]
    assert owner.calls == []
    assert signal.getsignal(signal.SIGALRM) is handler
    assert signal.getitimer(signal.ITIMER_REAL) == (0., 0.)


def test_existing_signal_timer_is_preserved_without_owner_calls(tmp_path):
    owner = StationaryOwner()
    signal.setitimer(signal.ITIMER_REAL, 10.)
    try:
        report = collect(owner, tmp_path / "timer", samples=1)
        assert report["status"] == "REJECTED"
        assert "existing process timer" in report["error"]
        assert 9. < signal.getitimer(signal.ITIMER_REAL)[0] <= 10.
        assert owner.calls == []
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)


@pytest.mark.parametrize("marker", ["accepted", "completed", "motor_writes", "stop_count"])
def test_missing_owner_markers_fail_before_observation(tmp_path, marker):
    owner = StationaryOwner()
    def missing(o, name, payload):
        if name == "robot_get_execution":payload["result"].pop(marker)
    owner.on_response = missing
    path = tmp_path / marker
    assert_rejected(owner, path, collect(owner, path, samples=1))
    assert not any(name == "robot_get_tags" for name, _ in owner.calls)
