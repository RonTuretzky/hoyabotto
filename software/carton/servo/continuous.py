"""Continuous file-protocol client and motor-owner execution component.

No motor port is opened here. The existing sole owner calls tick() in its
control loop and supplies its existing synchronous motor-write and STOP
functions. A separately commissioned profile is loaded by BOTH processes;
the client cannot enlarge owner limits. There are no model calls or endpoint
settles between trajectory samples.
"""
from __future__ import annotations

import copy
import time

import numpy as np

from .common import Refused, atomic_json, digest, finite, read_json, vector
from .trajectory import JointTrajectory
from .transport import SessionTransport

PROTOCOL = 1


def validate_profile(profile, config=None):
    if profile.get("schema") != PROTOCOL or profile.get("units") != "encoder_ticks":
        raise Refused("Expected continuous profile schema 1 in encoder_ticks")
    arm = profile.get("arm")
    expected = {f"{arm}_arm_{j}" for j in
                ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
    if arm not in ("left", "right") or set(profile["joints"]) != expected or len(profile["joints"]) != 6:
        raise Refused("Profile must cover exactly the six selected arm joints")
    if not isinstance(profile.get("commissioning_evidence"), str) or not profile["commissioning_evidence"].strip():
        raise Refused("Profile needs a station commissioning evidence reference")
    for field, minimum, maximum in (("following_ticks", 1, 24), ("start_ticks", 1, 5),
                                    ("settle_ticks", 1, 5), ("vision_age_s", .05, 1),
                                    ("max_tick_gap_s", .01, .2), ("settle_timeout_s", .1, 5)):
        if not minimum <= finite(profile[field], field) <= maximum:
            raise Refused(f"Invalid commissioned {field}")
    for n in profile["joints"]:
        lo, hi = vector(profile["corridor"][n], 2)
        if not 0 <= lo < hi <= 4095:
            raise Refused("Invalid commissioned corridor")
        if finite(profile["velocity"][n]) <= 0 or finite(profile["acceleration"][n]) <= 0:
            raise Refused("Commissioned speed and acceleration must be positive")
        if config is not None:
            a, b = config["ranges"][n]
            if not a+4 <= lo < hi <= b-4:
                raise Refused("Commissioned corridor leaves saved motor range margins")
    if config is not None:
        from .common import binding
        if profile.get("fingerprint") != binding(config) or profile["arm"] != config["arm"]:
            raise Refused("Continuous profile is bound to a different station configuration")
    return digest(profile)


def trajectory(profile, waypoints):
    return JointTrajectory(waypoints, profile["joints"], profile["corridor"],
                           profile["velocity"], profile["acceleration"])


class TrajectoryExecutor:
    """Tick from the sole motor owner's loop; STOP callback owns torque policy.

    Call start() before accepting a command, then tick() on EVERY owner cycle,
    including end-point settling. Persist returned fields into status.json.
    The containing owner must also enforce its lease, single-writer ownership,
    hardware communication checks, and continue health monitoring after finish.
    """
    def __init__(self, profile, write_goals, stop, *, clock=time.monotonic, wall=time.time):
        self.profile = copy.deepcopy(profile)
        self.profile_hash = validate_profile(profile)
        self.write_goals, self.stop = write_goals, stop
        self.clock, self.wall = clock, wall
        self.active = False
        self.last_id = 0
        self.metrics = {}

    def capabilities(self):
        return {"trajectory_protocol": PROTOCOL, "trajectory_profile_sha256": self.profile_hash}

    def _health(self, rows):
        if set(rows) != set(self.profile["joints"]):
            raise Refused("Incomplete continuous motor telemetry")
        q = {}
        for n, row in rows.items():
            if finite(row["Status"]) != 0:
                raise Refused(f"{n}: motor fault")
            if finite(row["Present_Temperature"]) >= 55 or abs(finite(row["Present_Load"])) >= 500:
                raise Refused(f"{n}: thermal/load limit")
            q[n] = finite(row["Present_Position"])
            lo, hi = self.profile["corridor"][n]
            if not lo <= q[n] <= hi:
                raise Refused(f"{n}: left commissioned corridor")
        return q

    def _vision(self, evidence):
        if (evidence.get("command_id") != self.command_id or evidence.get("session_started") != self.session_started
                or evidence.get("ok") is not True or evidence.get("streams") != self.streams):
            raise Refused("Vision watchdog identity or tracking failure")
        stamps = evidence["captured_at"]
        if set(stamps) != set(self.streams) or set(evidence["sequences"]) != set(self.streams):
            raise Refused("Incomplete vision watchdog evidence")
        now = self.wall()
        if any(not 0 <= now-finite(t) <= self.profile["vision_age_s"] for t in stamps.values()):
            raise Refused("Vision watchdog expired")
        if max(stamps.values())-min(stamps.values()) > .3:
            raise Refused("Vision watchdog cameras are not synchronized")
        for n, seq in evidence["sequences"].items():
            if type(seq) is not int or seq < 0 or seq < self.sequences.get(n, -1):
                raise Refused("Vision sequence rolled back")
            if n in self.sequences and seq == self.sequences[n] and stamps[n] != self.stamps[n]:
                raise Refused("Capture timestamp changed without a new camera frame")
            if n in self.sequences and seq > self.sequences[n] and stamps[n] <= self.stamps[n]:
                raise Refused("New camera frame has a non-increasing timestamp")
        self.sequences, self.stamps = dict(evidence["sequences"]), dict(stamps)

    def start(self, command, rows, vision, *, session_started, lease_remaining):
        if self.active:
            raise Refused("Previous trajectory is still active")
        if (command.get("op") != "trajectory" or type(command.get("id")) is not int
                or command["id"] <= self.last_id or command.get("profile_sha256") != self.profile_hash
                or command.get("session_started") != session_started):
            raise Refused("Invalid trajectory command identity or owner profile")
        path = trajectory(self.profile, command["waypoints"])
        if lease_remaining <= path.duration+self.profile["settle_timeout_s"]+1:
            raise Refused("Trajectory exceeds remaining owner lease")
        q = self._health(rows)
        path.assert_start(q, self.profile["start_ticks"])
        self.command_id, self.session_started = command["id"], session_started
        self.streams = command["camera_streams"]
        if set(self.streams) != {"head", f'{self.profile["arm"]}_wrist'} or not all(self.streams.values()):
            raise Refused("Trajectory needs both registered camera streams")
        self.sequences, self.stamps = {}, {}
        self._vision(vision)
        self.path, self.started, self.last_tick = path, self.clock(), self.clock()
        self.previous_goal = path.sample(0)[0]
        self.stable, self.last_telemetry = 0, None
        self.metrics = {"goal_writes": 0, "max_tick_gap_s": 0, "duration_s": path.duration,
                        "travel_ticks": path.travel_ticks, "command_id": self.command_id}
        self.active, self.last_id = True, self.command_id
        return {"accepted": self.command_id, "phase": "moving"}

    def tick(self, rows, vision, *, telemetry_at, stop_requested=False):
        if not self.active:
            raise Refused("No active trajectory")
        try:
            now = self.clock()
            gap = now-self.last_tick
            if stop_requested:
                raise Refused("Owner STOP requested")
            if gap < 0 or gap > self.profile["max_tick_gap_s"]:
                raise Refused("Motor-owner tick watchdog expired")
            self.metrics["max_tick_gap_s"] = max(self.metrics["max_tick_gap_s"], gap)
            if not 0 <= self.wall()-finite(telemetry_at) <= self.profile["max_tick_gap_s"]:
                raise Refused("Continuous motor telemetry is stale")
            q = self._health(rows)
            self._vision(vision)
            # Compare encoders with the preceding dispatched goal, allowing
            # the actuator's measured lag rather than predicting future motion.
            if any(abs(q[n]-self.previous_goal[n]) > self.profile["following_ticks"] for n in q):
                raise Refused("Continuous encoder following error")
            elapsed = now-self.started
            goals = self.path.sample(elapsed)[0]
            if elapsed > self.path.duration+self.profile["settle_timeout_s"]:
                raise Refused("Trajectory endpoint did not settle")
            self.write_goals({n: int(round(v)) for n, v in goals.items()})
            self.metrics["goal_writes"] += 1
            self.previous_goal, self.last_tick = goals, now
            if telemetry_at != self.last_telemetry:
                self.stable = self.stable+1 if elapsed >= self.path.duration and all(
                    abs(q[n]-goals[n]) <= self.profile["settle_ticks"] for n in q) else 0
            self.last_telemetry = telemetry_at
            if self.stable >= 3:
                self.active = False
                self.metrics["elapsed_s"] = elapsed
                return {"completed": self.command_id, "phase": "holding", "trajectory_metrics": dict(self.metrics)}
            return {"phase": "moving", "trajectory_elapsed_s": elapsed}
        except BaseException:
            self.active = False
            self.stop()
            raise


class ContinuousTransport(SessionTransport):
    def __init__(self, config, limits, profile, **kwargs):
        self.profile = copy.deepcopy(profile)
        self.profile_hash = validate_profile(profile, config)
        self.last_observation = None
        super().__init__(config, limits, **kwargs)

    def check_envelope(self, q):
        for n in self.profile["joints"]:
            lo, hi = self.profile["corridor"][n]
            if not lo <= q[n] <= hi:
                raise Refused("Actual joint position left the commissioned trajectory corridor")

    def status(self):
        s, q = super().status()
        if s.get("trajectory_protocol") != PROTOCOL or s.get("trajectory_profile_sha256") != self.profile_hash:
            raise Refused("Owner has not loaded this continuous protocol/profile")
        return s, q

    def move(self, *args, **kwargs):
        raise Refused("Continuous program cannot fall back to six-degree steps")

    move_gripper = move

    def play(self, segment, observe):
        """One dispatch for the whole path; refresh vision while owner runs it."""
        if not self.execute:
            raise Refused("Read-only mode cannot send motor commands")
        path = trajectory(self.profile, segment["waypoints"])
        s, q = self.status()
        if s["phase"] != "holding":
            raise Refused("Previous motor command has not finished")
        path.assert_start(q, self.profile["start_ticks"])
        if self.path_ticks+path.travel_ticks > self.limits.max_path_ticks:
            raise Refused("Total program travel budget exhausted")
        duration = path.duration+self.profile["settle_timeout_s"]+1
        if s["lease_remaining"] <= duration or self.deadline-self.clock() <= duration:
            raise Refused("Insufficient lease/time for the complete trajectory")
        command_file = self.folder / "command.json"
        if command_file.exists() and read_json(command_file).get("id") != self.last_id:
            raise Refused("Another command writer is active")
        command_id = max(time.time_ns(), (self.last_id or 0)+1)
        obs = observe()
        streams = obs.streams.copy()
        start = vector(segment["start_features"], len(obs.values))
        if max(abs(obs.values-start)) > self.limits.tolerance_px:
            raise Refused("Scene no longer matches measured trajectory start")
        previous_sequences = {}

        def publish(observation):
            values = vector(observation.values)
            bounds = np.asarray(segment["feature_bounds"], float)
            if (observation.streams != streams or bounds.shape != (len(values), 2)
                    or not np.isfinite(bounds).all() or np.any(values < bounds[:, 0]) or np.any(values > bounds[:, 1])):
                raise Refused("Trajectory visual corridor/stream mismatch")
            if (set(observation.sequences) != set(streams) or any(type(seq) is not int or
                    seq <= previous_sequences.get(n, -1) for n, seq in observation.sequences.items())):
                raise Refused("Trajectory requires distinct camera observations")
            stamps = observation.captured_times or {n: observation.captured_at for n in streams}
            if set(stamps) != set(streams) or any(not 0 <= self.clock()-t <= self.profile["vision_age_s"] for t in stamps.values()):
                raise Refused("Trajectory camera evidence is stale")
            previous_sequences.update(observation.sequences)
            atomic_json(self.folder / "trajectory-vision.json", {"command_id": command_id,
                        "session_started": self.started, "ok": True, "streams": streams,
                        "sequences": observation.sequences, "captured_at": stamps})
            self.last_observation = observation

        publish(obs)
        # Recheck after camera processing, before sending the path.
        s, current = self.status()
        path.assert_start(current, self.profile["start_ticks"])
        if s["phase"] != "holding" or (command_file.exists() and read_json(command_file).get("id") != self.last_id):
            raise Refused("Owner/writer changed while preparing trajectory")
        atomic_json(command_file, {"id": command_id, "op": "trajectory", "session_started": self.started,
                    "profile_sha256": self.profile_hash, "camera_streams": streams, "waypoints": segment["waypoints"]})
        self.last_id, self.path_ticks = command_id, self.path_ticks+path.travel_ticks
        self.commands_sent += 1
        end = self.clock()+duration
        while self.clock() < end:
            publish(observe())
            s, q = self.status()
            if read_json(command_file).get("id") != command_id:
                raise Refused("Trajectory command overwritten by another client")
            if (s.get("last_rejected") or {}).get("id") == command_id:
                raise Refused(f"Owner rejected trajectory: {s['last_rejected'].get('reason')}")
            if s.get("completed") == command_id and s["phase"] == "holding":
                endpoint = path.sample(path.duration)[0]
                if any(abs(q[n]-endpoint[n]) > self.profile["settle_ticks"] for n in q):
                    raise Refused("Trajectory completion disagrees with encoders")
                if max(abs(self.last_observation.values-vector(segment["end_features"], len(obs.values)))) > self.limits.tolerance_px:
                    raise Refused("Trajectory endpoint visual target not reached")
                return q, self.clock()
            self.sleep(.01)
        raise Refused("Continuous trajectory completion timed out")
