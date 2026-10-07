"""Fake sole owner and clocks only: no serial/camera access or physical claims."""
import copy

import pytest

from carton.servo.bimanual_owner import ARMS, ARM_JOINTS, JOINTS, PROTOCOL, VERSION, BimanualTrajectoryOwner, trajectory_digest
from carton.servo.common import Refused, digest

LEFT = "left_arm_shoulder_pan"
RIGHT = "right_arm_shoulder_pan"


class Rig:
    def __init__(self):
        self.now = 10.
        self.seq = self.scene_revision = 1
        self.q = dict.fromkeys(JOINTS, 2000)
        self.writes, self.stops = [], []
        self.guard_fault = self.write_fault = self.release_fault = None
        self.write_delays = dict.fromkeys(ARMS, .004)
        self.generations = dict.fromkeys(ARMS, 0)
        self.planned_command = None
        self.bindings = {
            **{key: {arm: digest([key, arm]) for arm in ARMS}
               for key in ("calibration_sha256", "registration_sha256", "kinematics_sha256")},
            "station_sha256": digest("station"), "commissioning_sha256": digest("SYNTHETIC TEST ONLY"),
            "camera_ids": {"head": "head-device", "oak": "oak-device"},
            "camera_streams": {"head": "head-stream", "oak": "oak-stream"},
            "ranges": {n: [1000, 3000] for n in JOINTS},
        }
        self.profile = {
            "protocol": PROTOCOL, "schema": VERSION, "units": "encoder_ticks", "joints": list(JOINTS),
            "bindings": copy.deepcopy(self.bindings),
            "commissioning_evidence": {"status": "COMMISSIONED", "reference": "SYNTHETIC TEST ONLY",
                                       "sha256": self.bindings["commissioning_sha256"]},
            "corridor": {n: [1800, 2400] for n in JOINTS}, "velocity": dict.fromkeys(JOINTS, 100),
            "acceleration": dict.fromkeys(JOINTS, 200),
            "load_raw": {n: 250 if n.endswith("gripper") else 500 for n in JOINTS},
            "following_ticks": 24, "start_ticks": 5, "settle_ticks": 5,
            "vision_age_s": .2, "max_camera_skew_s": .02, "max_tick_gap_s": .2,
            "telemetry_age_s": .2, "max_telemetry_skew_s": .02, "max_dispatch_skew_s": .02,
            "settle_timeout_s": 1, "max_sample_ticks": 68,
        }
        self.stamps = dict.fromkeys(JOINTS, self.wall())

    def clock(self):
        return self.now

    def wall(self):
        return 1000 + self.now

    def rows(self):
        return {n: {"Present_Position": q, "Present_Load": 10, "Status": 0, "Torque_Enable": 1} for n, q in self.q.items()}

    def guard(self):
        if self.guard_fault:
            raise Refused("Independent owner guard refused")

    def write(self, arm, goals):
        started = self.clock()
        self.writes.append((arm, goals.copy(), started))
        if self.write_fault == f"{arm}_partial":
            self.q.update(dict(list(goals.items())[:3]))
            raise OSError("Synthetic partial serial write")
        self.q.update(goals)
        self.now += self.write_delays[arm]
        ack = {"arm": arm, "written": goals.copy(), "started_at": started, "finished_at": self.clock()}
        if self.write_fault == f"{arm}_ack":
            ack["written"].pop(next(iter(goals)))
        if self.write_fault == f"{arm}_timestamp":
            ack["started_at"] = started - 1
        if self.write_fault == f"{arm}_guard":
            self.guard_fault = True
        return ack

    def stop(self, arm):
        self.stops.append(arm)
        self.generations[arm] += 1
        if self.release_fault == f"{arm}_error":
            raise OSError("Synthetic release failure")
        proof = {"arm": arm, "released": True, "release_errors": [], "cached": False,
                 "captured_at": self.wall(), "torque_enable": dict.fromkeys(ARM_JOINTS[arm], 0)}
        if self.release_fault == f"{arm}_old":
            proof["captured_at"] -= .01
        if self.release_fault == f"{arm}_unverified":
            proof["torque_enable"][ARM_JOINTS[arm][0]] = 1
        return proof

    def create(self):
        return BimanualTrajectoryOwner(self.profile, self.bindings, write_arm=self.write, stop_arm=self.stop,
                                      guard=self.guard, clock=self.clock, wall=self.wall)

    def context(self, command=None):
        certified = command or self.planned_command or self.command()
        return {"owner_started": 900., "bindings": copy.deepcopy(self.bindings),
                "gripper_release_generation": self.generations.copy(),
                "scene": {"revision": self.scene_revision, "sha256": digest(["scene", self.scene_revision]),
                          "trajectory_sha256": trajectory_digest(certified),
                          "valid": True, "collision_checked": True, "observation_only": True,
                          "frame_sequences": dict.fromkeys(self.bindings["camera_ids"], self.seq)},
                "frames": {n: {"camera_id": device, "stream_id": self.bindings["camera_streams"][n],
                               "captured_at": self.wall(), "seq": self.seq, "sha256": digest([n, self.seq])}
                           for n, device in self.bindings["camera_ids"].items()}}

    def command(self, owner=None, command_id=1):
        return {"protocol": PROTOCOL, "schema": VERSION, "op": "bimanual_trajectory", "id": command_id,
                "session_started": 900., "profile_sha256": owner.profile_sha256 if owner else digest(self.profile),
                "bindings_sha256": owner.bindings_sha256 if owner else digest(self.bindings), "scene_revision": self.scene_revision,
                "scene_sha256": digest(["scene", self.scene_revision]), "gripper_release_generation": self.generations.copy(),
                "waypoints": [{"time_s": 0., "positions": self.q.copy()},
                              {"time_s": 1., "positions": {**self.q, LEFT: 2040, RIGHT: 1970}}]}

    def start(self, owner):
        command = self.command(owner)
        self.planned_command = copy.deepcopy(command)
        result = owner.start(command, self.rows(), self.stamps, self.context(), session_started=900., lease_remaining=10.)
        assert result["accepted"] == 1 and result["motor_writes"] == 0
        return command

    def advance(self, seconds=.02, *, telemetry=True):
        self.now += seconds
        self.seq += 1
        if telemetry:
            self.stamps = dict.fromkeys(JOINTS, self.wall())

    def tick(self, owner, **kwargs):
        return owner.tick(self.rows(), self.stamps, self.context(), lease_remaining=kwargs.pop("lease_remaining", 10), **kwargs)


def test_constructor_and_acceptance_never_write_then_both_arms_finish_together():
    rig = Rig()
    owner = rig.create()
    assert not rig.writes and not rig.stops
    rig.start(owner)
    assert not rig.writes and not rig.stops
    for _ in range(100):
        rig.advance()
        result = rig.tick(owner)
        if not owner.active:
            break
    assert result["completed"] == 1 and result["phase"] == "holding"
    assert rig.q[LEFT] == 2040 and rig.q[RIGHT] == 1970
    assert len(rig.writes) > 50 and not rig.stops
    assert [a for a, _, _ in rig.writes] == list(ARMS) * (len(rig.writes) // 2)
    assert all(set(q) == set(ARM_JOINTS[a]) for a, q, _ in rig.writes)
    assert owner.metrics["max_dispatch_span_s"] == pytest.approx(.008)
    assert owner.metrics["max_dispatch_start_skew_s"] == pytest.approx(.004)
    assert owner.stop()["released"] and rig.stops == list(ARMS)
    assert not owner.capabilities()["hardware_deployment_verified"]


@pytest.mark.parametrize("fault", ["commissioning", "missing_evidence", "binding", "joints", "rates", "corridor", "camera", "jaw_load"])
def test_uncommissioned_or_mismatched_constructor_refuses_without_callbacks(fault):
    rig = Rig()
    if fault == "commissioning":
        rig.profile["commissioning_evidence"]["status"] = "PREPARED"
    if fault == "missing_evidence":
        rig.profile["commissioning_evidence"] = None
    if fault == "binding":
        rig.profile["bindings"]["registration_sha256"]["right"] = digest("other")
    if fault == "joints":
        rig.profile["joints"][-1] = "head_motor_1"
    if fault == "rates":
        rig.profile["velocity"][RIGHT] = 101
    if fault == "corridor":
        rig.profile["corridor"][RIGHT][1] = 3000
    if fault == "camera":
        rig.bindings["camera_ids"]["oak"] = rig.bindings["camera_ids"]["head"]
    if fault == "jaw_load":
        rig.profile["load_raw"]["right_arm_gripper"] = 251
    with pytest.raises(Refused):
        rig.create()
    assert not rig.writes and not rig.stops


@pytest.mark.parametrize("fault", ["right_goal", "right_start", "missing_joint", "profile", "binding", "scene", "lease"])
def test_invalid_second_arm_or_command_refuses_before_either_arm_write(fault):
    rig = Rig()
    owner = rig.create()
    command, rows = rig.command(owner), rig.rows()
    if fault == "right_goal":
        command["waypoints"][-1]["positions"][RIGHT] = 2500
    if fault == "right_start":
        rows[RIGHT]["Present_Position"] = 2010
    if fault == "missing_joint":
        command["waypoints"][-1]["positions"].pop(RIGHT)
    if fault == "profile":
        command["profile_sha256"] = digest("other")
    if fault == "binding":
        command["bindings_sha256"] = digest("other")
    if fault == "scene":
        command["scene_sha256"] = digest("other")
    with pytest.raises(Refused):
        owner.start(command, rows, rig.stamps, rig.context(command), session_started=900., lease_remaining=1 if fault == "lease" else 10)
    assert not rig.writes and rig.stops == list(ARMS) and owner.fault_latched


@pytest.mark.parametrize("fault", ["load", "corridor", "following", "released", "missing_time", "stale", "telemetry_skew",
                                  "vision", "camera_skew", "camera_missing", "registration", "scene_invalid",
                                  "scene_changed", "generation", "owner", "guard", "stop", "loop_gap", "lease"])
def test_inflight_faults_on_either_source_stop_both_before_any_goal(fault):
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    rig.advance()
    rows, stamps, context = rig.rows(), rig.stamps.copy(), rig.context()
    if fault == "load":
        rows[RIGHT]["Present_Load"] = 500
    if fault == "corridor":
        rows[RIGHT]["Present_Position"] = 2401
    if fault == "following":
        rows[RIGHT]["Present_Position"] += 25
    if fault == "released":
        rows[RIGHT]["Torque_Enable"] = 0
    if fault == "missing_time":
        stamps.pop(RIGHT)
    if fault == "stale":
        stamps[RIGHT] -= .3
    if fault == "telemetry_skew":
        stamps[RIGHT] -= .03
    if fault == "vision":
        context["frames"]["oak"]["captured_at"] -= .3
    if fault == "camera_skew":
        context["frames"]["oak"]["captured_at"] -= .03
    if fault == "camera_missing":
        context["frames"].pop("oak")
    if fault == "registration":
        context["bindings"]["registration_sha256"]["right"] = digest("new")
    if fault == "scene_invalid":
        context["scene"]["valid"] = False
    if fault == "scene_changed":
        context["scene"]["revision"] += 1
    if fault == "generation":
        context["gripper_release_generation"]["right"] += 1
    if fault == "owner":
        context["owner_started"] += 1
    if fault == "guard":
        rig.guard_fault = True
    if fault == "loop_gap":
        rig.now += .3
    with pytest.raises(Refused):
        owner.tick(rows, stamps, context, lease_remaining=0 if fault == "lease" else 10, stop_requested=fault == "stop")
    assert not rig.writes and rig.stops == list(ARMS)
    assert owner.fault_latched and not owner.active


@pytest.mark.parametrize("fault,attempts", [("left_partial", 1), ("right_partial", 2), ("left_ack", 1),
                                           ("right_ack", 2), ("left_timestamp", 1), ("left_guard", 1)])
def test_partial_serial_write_bad_ack_or_between_arm_guard_latches_both_stops(fault, attempts):
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    rig.write_fault = fault
    rig.advance()
    with pytest.raises((Refused, OSError)):
        rig.tick(owner)
    assert len(rig.writes) == attempts and rig.stops == list(ARMS)
    assert owner.metrics["arm_write_attempts"] == attempts
    assert owner.stop_result["released"] and owner.fault_latched
    with pytest.raises(Refused, match="latched"):
        rig.tick(owner)
    assert len(rig.writes) == attempts and rig.stops == list(ARMS)


@pytest.mark.parametrize("arm,delay,attempts", [("left", .03, 1), ("right", .02, 2)])
def test_measured_dispatch_skew_stops_and_retains_actual_timestamps(arm, delay, attempts):
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    rig.write_delays[arm] = delay
    rig.advance()
    with pytest.raises(Refused, match="dispatch skew"):
        rig.tick(owner)
    assert len(rig.writes) == attempts and rig.stops == list(ARMS)
    evidence = owner.metrics["last_dispatch"]
    assert len(evidence["arms"]) == attempts
    assert all(a["finished_at"] >= a["started_at"] for a in evidence["arms"])
    if attempts == 2:
        assert evidence["span_s"] == pytest.approx(.024)


@pytest.mark.parametrize("fault", ["left_error", "left_old", "right_unverified"])
def test_failed_or_old_release_proof_never_claims_paired_release_and_still_attempts_both(fault):
    rig = Rig()
    owner = rig.create()
    rig.release_fault = fault
    result = owner.stop()
    assert rig.stops == list(ARMS)
    assert not result["released"] and result["release_errors"]
    assert owner.stop() == result and rig.stops == list(ARMS)


@pytest.mark.parametrize("fresh_frames", [False, True])
def test_next_command_cannot_replay_scene_or_invent_new_scene_using_old_frames(fresh_frames):
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    for _ in range(100):
        rig.advance()
        rig.tick(owner)
        if not owner.active:
            break
    before = len(rig.writes)
    if fresh_frames:
        rig.advance()  # Same scene revision is still a replay.
    else:
        rig.scene_revision += 1  # Revision alone cannot bless old frames.
    command = rig.command(owner, 2)
    context = rig.context(command)
    if not fresh_frames:
        context["frames"] = copy.deepcopy(owner.metrics["last_scene_validation"]["frames"])
    with pytest.raises(Refused, match="scene|frame"):
        owner.start(command, rig.rows(), rig.stamps, context, session_started=900, lease_remaining=10)
    assert len(rig.writes) == before and rig.stops == list(ARMS)


def test_camera_or_telemetry_timestamp_cannot_be_reused_for_changed_data():
    for source in ("camera", "telemetry"):
        rig = Rig()
        owner = rig.create()
        rig.start(owner)
        context, rows = rig.context(), rig.rows()
        rig.now += .01
        if source == "camera":
            context["frames"]["oak"]["sha256"] = digest("replacement")
        else:
            rows[RIGHT]["Present_Position"] += 1
        with pytest.raises(Refused, match="replay|relabel"):
            owner.tick(rows, rig.stamps, context, lease_remaining=10)
        assert not rig.writes and rig.stops == list(ARMS)


def test_next_observed_scene_can_start_another_paired_stage_without_releasing_jaws():
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    for _ in range(100):
        rig.advance()
        rig.tick(owner)
        if not owner.active:
            break
    rig.scene_revision += 1
    rig.advance()
    command = rig.command(owner, 2)
    result = owner.start(command, rig.rows(), rig.stamps, rig.context(command),
                         session_started=900, lease_remaining=10)
    assert result["accepted"] == 2 and owner.active
    assert not rig.stops and rig.generations == {"left": 0, "right": 0}


@pytest.mark.parametrize("mutation", ["endpoint", "time"])
def test_collision_certificate_rejects_changed_in_range_waypoint_before_either_write(mutation):
    rig = Rig()
    owner = rig.create()
    command = rig.command(owner)
    context = rig.context(command)
    if mutation == "endpoint":
        command["waypoints"][-1]["positions"][RIGHT] += 1
    else:
        command["waypoints"][-1]["time_s"] += .1
    with pytest.raises(Refused, match="Collision certificate differs"):
        owner.start(command, rig.rows(), rig.stamps, context, session_started=900, lease_remaining=10)
    assert not rig.writes and rig.stops == list(ARMS)


def test_repeated_telemetry_cannot_supply_three_settling_samples():
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    while owner.stable < 1:
        rig.advance()
        rig.tick(owner)
    for _ in range(3):
        rig.advance(telemetry=False)
        rig.tick(owner)
    assert owner.active and owner.stable == 1
    for _ in range(2):
        rig.advance()
        result = rig.tick(owner)
    assert result["completed"] == 1 and not owner.active


def test_lease_is_shared_and_cannot_be_renewed_by_later_tick_arguments():
    rig = Rig()
    owner = rig.create()
    rig.start(owner)
    rig.advance()
    rig.tick(owner, lease_remaining=.015)
    before = len(rig.writes)
    rig.advance()
    with pytest.raises(Refused, match="lease expired"):
        rig.tick(owner, lease_remaining=1000)
    assert len(rig.writes) == before and rig.stops == list(ARMS)


def test_metrics_keep_real_per_joint_capture_times_instead_of_relabeling_them():
    rig = Rig()
    owner = rig.create()
    rig.stamps[RIGHT] -= .01
    original = rig.stamps.copy()
    rig.start(owner)
    evidence = owner.metrics["last_telemetry"]
    assert evidence["captured_at"] == original
    assert evidence["capture_skew_s"] == pytest.approx(.01)
    assert evidence["oldest_age_s"] == pytest.approx(.01)
    assert owner.metrics["max_telemetry_skew_s"] == pytest.approx(.01)


def test_slow_right_release_does_not_hide_stale_left_release_confirmation():
    rig = Rig()
    original = rig.stop
    def stop(arm):
        if arm == "right":
            rig.now += .3
        return original(arm)
    owner = BimanualTrajectoryOwner(rig.profile, rig.bindings, write_arm=rig.write,
        stop_arm=stop, guard=rig.guard, clock=rig.clock, wall=rig.wall)
    result = owner.stop()
    assert rig.stops == list(ARMS)
    assert not result["released"]
    assert any("left: release evidence stale" in e for e in result["release_errors"])


@pytest.mark.parametrize('identity', [None, '', '   ', True, False, 0, -1, float('inf'), float('-inf'), float('nan'), [], {}])
def test_missing_or_invalid_owner_session_identity_is_refused_before_goals(identity):
    rig = Rig()
    owner = rig.create()
    command = rig.command(owner)
    command['session_started'] = identity
    context = rig.context(command)
    context['owner_started'] = identity
    with pytest.raises(Refused, match='session identity'):
        owner.start(command, rig.rows(), rig.stamps, context, session_started=identity, lease_remaining=10)
    assert rig.writes == [] and rig.stops == list(ARMS) and owner.fault_latched


@pytest.mark.parametrize('identity', [900., 'owner-process-random-nonce'])
def test_positive_timestamp_or_nonempty_nonce_owner_identity_is_accepted_without_goals(identity):
    rig = Rig()
    owner = rig.create()
    command = rig.command(owner)
    command['session_started'] = identity
    context = rig.context(command)
    context['owner_started'] = identity
    result = owner.start(command, rig.rows(), rig.stamps, context, session_started=identity, lease_remaining=10)
    assert result['accepted'] == 1 and rig.writes == [] and rig.stops == []


@pytest.mark.parametrize('field', ['command', 'context'])
def test_boolean_owner_identity_cannot_alias_numeric_timestamp(field):
    rig = Rig()
    owner = rig.create()
    command = rig.command(owner)
    command['session_started'] = 1
    context = rig.context(command)
    context['owner_started'] = 1
    if field == 'command':
        command['session_started'] = True
    else:
        context['owner_started'] = True
    with pytest.raises(Refused, match='session identity'):
        owner.start(command, rig.rows(), rig.stamps, context, session_started=1, lease_remaining=10)
    assert rig.writes == [] and rig.stops == list(ARMS)
