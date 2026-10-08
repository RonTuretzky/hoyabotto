"""Offline-tested paired trajectory component for an existing sole motor owner.

No device, camera, file, thread, enable or recovery operation is created here.
The owner supplies independently checked local bindings, coherent telemetry,
scene/camera context, synchronous writes and verified per-arm STOP callbacks.
Protocol v2 is separate from the existing single-arm continuous protocol v1.
"""
from __future__ import annotations

import copy
import re
import time

from .common import Refused, digest, finite, vector
from .trajectory import JointTrajectory

PROTOCOL = "carton_bimanual_trajectory"
VERSION = 2
ARMS = ("left", "right")
JOINT_SUFFIXES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARM_JOINTS = {arm: tuple(f"{arm}_arm_{joint}" for joint in JOINT_SUFFIXES) for arm in ARMS}
JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"]


def trajectory_digest(command):
    """Identity to certify AFTER independent collision checking of this proposal.

    Includes rate/corridor profile, geometry bindings, scene and exact waypoints.
    Computing a digest does not perform or certify a collision check.
    """
    return digest({key: command.get(key) for key in
                   ("profile_sha256", "bindings_sha256", "scene_revision", "scene_sha256", "waypoints")})


def _hash(value, label):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise Refused(f"{label} needs a SHA256 identity")


def _exact(value, names, label):
    if not isinstance(value, dict) or set(value) != set(names):
        raise Refused(f"{label} must cover exactly {', '.join(names)}")


def _session_identity(value):
    if isinstance(value, str):
        if not value.strip():
            raise Refused("Owner session identity needs a nonempty nonce")
    elif finite(value, "owner session identity") <= 0:
        raise Refused("Owner session identity needs a positive finite timestamp")


def validate_profile(profile, bindings):
    """Validate independently loaded owner configuration, never client limits.

    Evidence references/hashes bind the caller's commissioning record. This
    library cannot certify that a declared record is a physical measurement.
    """
    if not isinstance(profile, dict) or not isinstance(bindings, dict):
        raise Refused("Profile and independent bindings must be objects")
    if (profile.get("protocol") != PROTOCOL or profile.get("schema") != VERSION
            or profile.get("units") != "encoder_ticks"):
        raise Refused("Expected bimanual trajectory v2 in encoder_ticks")
    if (not isinstance(profile.get("joints"), list) or len(profile["joints"]) != 12
            or set(profile["joints"]) != set(JOINTS)):
        raise Refused("Profile requires exactly both arms' twelve joints")
    for key in ("calibration_sha256", "registration_sha256", "kinematics_sha256"):
        _exact(bindings.get(key), ARMS, key)
        for arm, value in bindings[key].items():
            _hash(value, f"{arm} {key}")
    for key in ("station_sha256", "commissioning_sha256"):
        _hash(bindings.get(key), key)
    cameras = bindings.get("camera_ids")
    if (not isinstance(cameras, dict) or not 2 <= len(cameras) <= 4
            or any(not isinstance(n, str) or not n or not isinstance(v, str) or not v for n, v in cameras.items())
            or len(set(cameras.values())) != len(cameras)):
        raise Refused("Bind two to four independently registered distinct cameras")
    _exact(bindings.get("camera_streams"), cameras, "camera streams")
    if any(not isinstance(s, str) or not s for s in bindings["camera_streams"].values()):
        raise Refused("Bind each registered camera capture stream")
    _exact(bindings.get("ranges"), JOINTS, "saved ranges")
    evidence = profile.get("commissioning_evidence", {})
    if (not isinstance(evidence, dict) or evidence.get("status") != "COMMISSIONED" or not isinstance(evidence.get("reference"), str)
            or not evidence["reference"].strip() or evidence.get("sha256") != bindings["commissioning_sha256"]):
        raise Refused("Profile lacks bound commissioning evidence")
    if profile.get("bindings") != bindings:
        raise Refused("Profile does not match independently loaded owner bindings")
    bounds = {
        "following_ticks": (1, 24), "start_ticks": (1, 5), "settle_ticks": (1, 5),
        "vision_age_s": (.01, 1), "max_camera_skew_s": (.001, .3),
        "max_tick_gap_s": (.001, .2), "telemetry_age_s": (.001, .2),
        "max_telemetry_skew_s": (.001, .1), "max_dispatch_skew_s": (.001, .05),
        "settle_timeout_s": (.1, 5), "max_sample_ticks": (1, 68),
    }
    for key, (lo, hi) in bounds.items():
        if not lo <= finite(profile.get(key), key) <= hi:
            raise Refused(f"Invalid commissioned {key}")
    if (profile["telemetry_age_s"] > profile["max_tick_gap_s"]
            or profile["max_dispatch_skew_s"] > profile["max_tick_gap_s"]):
        raise Refused("Telemetry/dispatch timing cannot exceed the owner watchdog")
    for key in ("corridor", "velocity", "acceleration", "load_raw"):
        _exact(profile.get(key), JOINTS, key)
    for name in JOINTS:
        a, b = vector(bindings["ranges"][name], 2)
        lo, hi = vector(profile["corridor"][name], 2)
        if not 0 <= a < b <= 4095 or not a + 4 <= lo < hi <= b - 4:
            raise Refused(f"{name}: corridor leaves saved range margins")
        for key, cap in (("velocity", 100), ("acceleration", 200),
                         ("load_raw", 250 if name.endswith("gripper") else 500)):
            if not 0 < finite(profile[key][name]) <= cap:
                raise Refused(f"{name}: invalid commissioned {key}")
    return digest(profile)


class BimanualTrajectoryOwner:
    """One paired clock, lease and fault latch; never two independently ticking arms.

    Construct only inside the sole owner. Constructor and start issue no goals.
    start/tick refusals latch STOP for BOTH arms; no reset/retry method exists.
    The containing owner must continue guard/lease supervision while holding.
    """
    def __init__(self, profile, bindings, *, write_arm, stop_arm, guard,
                 clock=time.monotonic, wall=time.time):
        self._profile, self._bindings = copy.deepcopy(profile), copy.deepcopy(bindings)
        self.profile_sha256 = validate_profile(self._profile, self._bindings)
        self.bindings_sha256 = digest(self._bindings)
        self.write_arm, self.stop_arm, self.guard = write_arm, stop_arm, guard
        self.clock, self.wall = clock, wall
        self.active = self.fault_latched = False
        self.last_command_id = self.last_scene_revision = -1
        self.command = None
        self.frame_history, self.telemetry_history = {}, {}
        self.stop_result = None
        self.metrics = {"paired_dispatches": 0, "arm_write_attempts": 0,
                        "max_dispatch_span_s": 0., "max_dispatch_start_skew_s": 0.,
                        "max_telemetry_skew_s": 0., "max_tick_gap_s": 0.}

    def capabilities(self):
        return {"bimanual_trajectory_protocol": PROTOCOL, "bimanual_trajectory_version": VERSION,
                "profile_sha256": self.profile_sha256, "bindings_sha256": self.bindings_sha256,
                "joint_names": list(JOINTS), "automatic_gripper_reenable": False,
                "hardware_deployment_verified": False}

    def _local_guard(self):
        if self.fault_latched:
            raise Refused("Bimanual STOP is latched")
        if digest(self._profile) != self.profile_sha256 or digest(self._bindings) != self.bindings_sha256:
            raise Refused("Owner profile/bindings changed")
        self.guard()

    def _context(self, context, command, *, new_scene=False):
        if not isinstance(context, dict):
            raise Refused("Independent owner context must be an object")
        _session_identity(context.get("owner_started"))
        if (context.get("owner_started") != command["session_started"]
                or context.get("bindings") != self._bindings):
            raise Refused("Current owner/calibration/registration/station bindings changed")
        generations = context.get("gripper_release_generation")
        _exact(generations, ARMS, "gripper release generations")
        if (any(type(v) is not int or v < 0 for v in generations.values())
                or generations != command.get("gripper_release_generation")):
            raise Refused("Gripper release generation changed or missing")
        scene = context.get("scene", {})
        if not isinstance(scene, dict):
            raise Refused("Observed scene context must be an object")
        _hash(scene.get("sha256"), "scene")
        if (scene.get("valid") is not True or scene.get("collision_checked") is not True
                or scene.get("observation_only") is not True
                or type(scene.get("revision")) is not int or scene["revision"] < 0
                or scene["revision"] != command.get("scene_revision")
                or scene["sha256"] != command.get("scene_sha256")):
            raise Refused("Current observed scene is unvalidated or differs from the planned scene")
        if new_scene and scene["revision"] <= self.last_scene_revision:
            raise Refused("Replayed scene revision; obtain a new observed planning scene")
        _hash(scene.get("trajectory_sha256"), "independent trajectory collision certificate")
        if scene["trajectory_sha256"] != trajectory_digest(command):
            raise Refused("Collision certificate differs from the exact proposed trajectory")
        frames = context.get("frames")
        _exact(frames, self._bindings["camera_ids"], "registered camera frames")
        _exact(scene.get("frame_sequences"), frames, "scene validation frame sequences")
        stamps = []
        for name, frame in frames.items():
            if not isinstance(frame, dict):
                raise Refused("Registered camera frame must be an object")
            if (frame.get("camera_id") != self._bindings["camera_ids"][name]
                    or frame.get("stream_id") != self._bindings["camera_streams"][name]):
                raise Refused("Registered camera identity/stream changed")
            seq, stamp = frame.get("seq"), finite(frame.get("captured_at"))
            _hash(frame.get("sha256"), "frame")
            if type(seq) is not int or seq < 0 or scene["frame_sequences"][name] != seq:
                raise Refused("Scene validation is not bound to the current camera frames")
            if not 0 <= self.wall() - stamp <= self._profile["vision_age_s"]:
                raise Refused("Vision is stale or future-dated")
            identity = (seq, stamp, frame["sha256"])
            previous = self.frame_history.get(name)
            if previous and (seq < previous[0] or seq == previous[0] and identity != previous
                             or seq > previous[0] and stamp <= previous[1]):
                raise Refused("Camera frame replay, rollback or timestamp relabel")
            if new_scene and previous and seq <= previous[0]:
                raise Refused("New scene needs a new frame from every registered camera")
            stamps.append(stamp)
        if max(stamps) - min(stamps) > self._profile["max_camera_skew_s"]:
            raise Refused("Camera capture skew exceeds commissioned bound")
        self.metrics["last_scene_validation"] = {"revision": scene["revision"], "sha256": scene["sha256"],
            "trajectory_sha256": scene["trajectory_sha256"],
            "frames": copy.deepcopy(frames), "camera_skew_s": max(stamps) - min(stamps)}

    def _telemetry(self, rows, telemetry_at):
        _exact(rows, JOINTS, "coherent telemetry")
        _exact(telemetry_at, JOINTS, "per-joint telemetry capture times")
        q, stamps = {}, {}
        for name in JOINTS:
            row = rows[name]
            if not isinstance(row, dict):
                raise Refused(f"{name}: coherent telemetry must be an object")
            stamp = finite(telemetry_at[name])
            if not 0 <= self.wall() - stamp <= self._profile["telemetry_age_s"]:
                raise Refused(f"{name}: telemetry is stale or future-dated")
            if (type(row.get("Status")) is not int or row["Status"] != 0
                    or type(row.get("Torque_Enable")) is not int or row["Torque_Enable"] != 1):
                raise Refused(f"{name}: motor fault or unexpected torque release")
            if abs(finite(row.get("Present_Load"))) >= self._profile["load_raw"][name]:
                raise Refused(f"{name}: load limit")
            value = row.get("Present_Position")
            if type(value) is not int or not self._profile["corridor"][name][0] <= value <= self._profile["corridor"][name][1]:
                raise Refused(f"{name}: invalid encoder or left commissioned corridor")
            previous = self.telemetry_history.get(name)
            if previous and (stamp < previous[0] or stamp == previous[0] and digest(row) != previous[1]):
                raise Refused(f"{name}: telemetry replay or timestamp relabel")
            q[name], stamps[name] = value, stamp
        skew = max(stamps.values()) - min(stamps.values())
        if skew > self._profile["max_telemetry_skew_s"]:
            raise Refused("Both-arm telemetry capture skew exceeds commissioned bound")
        self.metrics["max_telemetry_skew_s"] = max(self.metrics["max_telemetry_skew_s"], skew)
        self.metrics["last_telemetry"] = {"captured_at": stamps.copy(), "capture_skew_s": skew,
                                           "oldest_age_s": self.wall() - min(stamps.values())}
        return q, stamps

    def _remember(self, rows, stamps, context):
        self.telemetry_history = {n: (stamps[n], digest(rows[n])) for n in JOINTS}
        self.frame_history = {n: (f["seq"], f["captured_at"], f["sha256"]) for n, f in context["frames"].items()}

    def _lease(self, lease_remaining=None):
        if lease_remaining is not None:
            # A containing owner may shorten the lease; this operation never renews it.
            self.lease_deadline = min(self.lease_deadline, self.clock() + finite(lease_remaining))
        if self.clock() >= self.lease_deadline:
            raise Refused("Bimanual supervision lease expired")

    def start(self, command, rows, telemetry_at, context, *, session_started, lease_remaining):
        try:
            self._local_guard()
            _session_identity(session_started)
            if self.active:
                raise Refused("A paired trajectory is already active")
            if not isinstance(command, dict):
                raise Refused("Paired command must be an object")
            _session_identity(command.get("session_started"))
            if (command.get("protocol") != PROTOCOL or command.get("schema") != VERSION
                    or command.get("op") != "bimanual_trajectory"
                    or type(command.get("id")) is not int or command["id"] <= self.last_command_id
                    or command.get("session_started") != session_started
                    or command.get("profile_sha256") != self.profile_sha256
                    or command.get("bindings_sha256") != self.bindings_sha256):
                raise Refused("Invalid paired command identity/version/profile/bindings")
            self._context(context, command, new_scene=True)
            q, stamps = self._telemetry(rows, telemetry_at)
            p = self._profile
            path = JointTrajectory(command.get("waypoints"), JOINTS, p["corridor"], p["velocity"], p["acceleration"])
            if path.duration + p["settle_timeout_s"] > 30:
                raise Refused("Paired trajectory plus settling exceeds 30 seconds")
            if finite(lease_remaining) <= path.duration + p["settle_timeout_s"] + 1:
                raise Refused("Paired trajectory exceeds remaining owner lease")
            path.assert_start(q, p["start_ticks"])
            self.command, self.path = copy.deepcopy(command), path
            self.started = self.last_tick = self.clock()
            self.lease_deadline = self.started + finite(lease_remaining)
            self.previous_goal, self.settle_stamps, self.stable = path.sample(0)[0], stamps, 0
            self._remember(rows, stamps, context)
            self.last_command_id, self.last_scene_revision = command["id"], command["scene_revision"]
            self.active = True
            self.metrics.update(command_id=command["id"], duration_s=path.duration, travel_ticks=path.travel_ticks)
            return {"accepted": command["id"], "phase": "moving", "motor_writes": 0}
        except BaseException as exc:
            self.stop(str(exc))
            raise

    def _ack(self, arm, goals, ack, called, returned):
        if (not isinstance(ack, dict) or ack.get("arm") != arm or ack.get("written") != goals):
            raise Refused(f"{arm}: write did not acknowledge the exact six-joint set")
        started, finished = finite(ack.get("started_at")), finite(ack.get("finished_at"))
        if not called <= started <= finished <= returned:
            raise Refused(f"{arm}: missing actual monotonic dispatch timestamps")
        return {"arm": arm, "started_at": started, "finished_at": finished,
                "callback_started_at": called, "callback_finished_at": returned,
                "written": copy.deepcopy(goals)}

    def tick(self, rows, telemetry_at, context, *, lease_remaining, stop_requested=False):
        try:
            self._local_guard()
            if not self.active:
                raise Refused("No active paired trajectory")
            begin = self.clock()
            if stop_requested:
                raise Refused("Owner STOP requested")
            gap = begin - self.last_tick
            if not 0 <= gap <= self._profile["max_tick_gap_s"]:
                raise Refused("Paired owner tick watchdog expired")
            self.metrics["max_tick_gap_s"] = max(self.metrics["max_tick_gap_s"], gap)
            self._lease(lease_remaining)
            self._context(context, self.command)
            q, stamps = self._telemetry(rows, telemetry_at)
            if any(abs(q[n] - self.previous_goal[n]) > self._profile["following_ticks"] for n in JOINTS):
                raise Refused("Paired encoder following error")
            elapsed = begin - self.started
            if elapsed > self.path.duration + self._profile["settle_timeout_s"]:
                raise Refused("Paired endpoint did not settle")
            raw = self.path.sample(elapsed)[0]
            goals = {n: int(round(raw[n])) for n in JOINTS}
            # Validate all twelve rounded targets BEFORE either bus callback.
            if any(not self._profile["corridor"][n][0] <= goals[n] <= self._profile["corridor"][n][1]
                   or abs(goals[n] - self.previous_goal[n]) > self._profile["max_sample_ticks"] for n in JOINTS):
                raise Refused("Paired sample leaves corridor or increment bound")
            dispatch = {"command_id": self.command["id"], "elapsed_s": elapsed, "arms": []}
            self.metrics["last_dispatch"] = dispatch
            for arm in ARMS:
                self._local_guard()
                self._lease()
                self._context(context, self.command)
                self._telemetry(rows, telemetry_at)
                if self.clock() - begin > self._profile["max_tick_gap_s"]:
                    raise Refused("Paired validation exceeded the owner watchdog")
                if dispatch["arms"] and self.clock() - dispatch["arms"][0]["started_at"] > self._profile["max_dispatch_skew_s"]:
                    raise Refused("Paired dispatch skew exceeded before second arm")
                subset = {n: goals[n] for n in ARM_JOINTS[arm]}
                called = self.clock()
                self.metrics["arm_write_attempts"] += 1
                dispatch["attempting_arm"] = arm
                ack = self.write_arm(arm, subset)
                dispatch["arms"].append(self._ack(arm, subset, ack, called, self.clock()))
            first, second = dispatch["arms"]
            span = second["finished_at"] - first["started_at"]
            skew = second["started_at"] - first["started_at"]
            dispatch.update(span_s=span, start_skew_s=skew)
            self.metrics["max_dispatch_span_s"] = max(self.metrics["max_dispatch_span_s"], span)
            self.metrics["max_dispatch_start_skew_s"] = max(self.metrics["max_dispatch_start_skew_s"], skew)
            if span > self._profile["max_dispatch_skew_s"]:
                raise Refused("Paired dispatch skew exceeded after writes")
            self._local_guard()
            self._lease()
            self._context(context, self.command)
            self._telemetry(rows, telemetry_at)
            self.metrics["paired_dispatches"] += 1
            self.previous_goal, self.last_tick = goals, begin
            self._remember(rows, stamps, context)
            if elapsed >= self.path.duration and all(abs(q[n] - goals[n]) <= self._profile["settle_ticks"] for n in JOINTS):
                if all(stamps[n] > self.settle_stamps[n] for n in JOINTS):
                    self.stable += 1
                    self.settle_stamps = stamps
            else:
                self.stable, self.settle_stamps = 0, stamps
            if self.stable >= 3:
                self.active = False
                self.metrics["elapsed_s"] = self.clock() - self.started
                return {"completed": self.command["id"], "phase": "holding", "trajectory_metrics": copy.deepcopy(self.metrics)}
            return {"phase": "moving", "trajectory_elapsed_s": elapsed, "dispatch": copy.deepcopy(dispatch)}
        except BaseException as exc:
            if "last_dispatch" in self.metrics:
                self.metrics["last_dispatch"]["failure"] = str(exc)
            self.stop(str(exc))
            raise

    def stop(self, reason="Operator STOP"):
        """Latch once, attempt BOTH releases even if the first callback fails.

        Release is reported only from fresh explicit all-six torque-zero proofs.
        This callback acknowledgment is separate from encoder motion completion.
        """
        if self.fault_latched:
            return copy.deepcopy(self.stop_result)
        self.active = False
        self.fault_latched = True
        started = self.clock()
        result = {"phase": "stopped", "fault_latched": True, "reason": reason,
                  "released": False, "release_errors": [], "arms": {}, "command_id": self.last_command_id}
        for arm in ARMS:
            try:
                release_requested_at = self.wall()
                proof = self.stop_arm(arm)
                if (not isinstance(proof, dict) or proof.get("arm") != arm or proof.get("released") is not True
                        or proof.get("cached") is not False
                        or proof.get("release_errors") != [] or proof.get("torque_enable") != dict.fromkeys(ARM_JOINTS[arm], 0)
                        or any(type(v) is not int for v in proof["torque_enable"].values())
                        or not release_requested_at <= finite(proof.get("captured_at")) <= self.wall()
                        or self.wall() - proof["captured_at"] > self._profile["telemetry_age_s"]):
                    raise Refused("Fresh verified all-six torque release missing")
                result["arms"][arm] = copy.deepcopy(proof)
            except BaseException as exc:
                result["release_errors"].append(f"{arm}: {type(exc).__name__}: {exc}")
        for arm, proof in result["arms"].items():
            if not 0 <= self.wall() - proof["captured_at"] <= self._profile["telemetry_age_s"]:
                result["release_errors"].append(f"{arm}: release evidence stale at paired confirmation")
        result["released"] = not result["release_errors"] and set(result["arms"]) == set(ARMS)
        result["release_latency_s"] = self.clock() - started
        self.stop_result = result
        return copy.deepcopy(result)
