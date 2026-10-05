"""Actual client/owner protocol with a deterministic motor fixture, not physics."""
import numpy as np
import pytest

from carton.servo.common import Limits, Observation, Refused, Trace, binding, read_json
from carton.servo.continuous import ContinuousTransport, TrajectoryExecutor
from carton.servo.controller import Experiment
from carton.servo.program import Program, preflight, template
from test_carton_program import RecipeRig


class ContinuousRig(RecipeRig):
    def __init__(self, folder, failure=None):
        super().__init__(folder)
        self.failure = failure
        self.writes, self.stops, self.held_at, self.placed_at = [], 0, None, 0
        self.profile = {"schema": 1, "units": "encoder_ticks", "arm": "right", "joints": list(self.q),
                        "fingerprint": binding(self.config), "commissioning_evidence": "SYNTHETIC TEST ONLY",
                        "corridor": {n: [1900, 2600] for n in self.q},
                        "velocity": {n: 200 for n in self.q}, "acceleration": {n: 400 for n in self.q},
                        "following_ticks": 16, "start_ticks": 5, "settle_ticks": 5,
                        "vision_age_s": .5, "max_tick_gap_s": .08, "settle_timeout_s": 1}
        self.engine = TrajectoryExecutor(self.profile, self.write, self.stop, clock=self.clock, wall=self.clock)
        self.s.update(self.engine.capabilities())
        self.publish()
        self.recipe = template(folder / "config.json", "pickup", "continuous")
        self.recipe.update(approach=[[456]], gripper={"open_ticks": 2000, "empty_closed_ticks": 1940,
                                                   "goal_ticks": 1984, "min_aperture_ticks": 20})
        self.recipe["lift"].update(targets=[[480]], up_normal=[1, 0], min_clearance_px=8)
        self.recipe["finish"].update(place_targets=[[464]], park_targets=[[0]], park_positions=self.q.copy(),
                                     supported_park_evidence="SYNTHETIC TEST ONLY")
        self.recipe["motion"]["profile"] = self.profile
        current = self.q.copy()
        for stage, x, jaw in (("approach", 456, 2000), ("close_gripper", 456, 1984),
                              ("lift", 480, 1984), ("place", 464, 1984),
                              ("open_gripper", 464, 2000), ("park", 0, 2000)):
            end = {**current, "right_arm_shoulder_pan": 2000+x, "right_arm_gripper": jaw}
            start_x = current["right_arm_shoulder_pan"]-2000
            self.recipe["motion"]["segments"][stage] = {
                "waypoints": [{"time_s": 0, "positions": current}, {"time_s": 2, "positions": end}],
                "start_features": [start_x], "end_features": [x],
                "feature_bounds": [[min(start_x, x)-3, max(start_x, x)+3]]}
            current = end

    def write(self, goals):
        self.writes.append((self.now, goals.copy()))
        if self.failure != "stuck": self.q.update(goals)
        if self.q["right_arm_gripper"] <= 1985 and self.held_at is None:
            self.held_at = self.q["right_arm_shoulder_pan"]-2000
        if self.q["right_arm_gripper"] >= 1998 and self.held_at is not None:
            self.placed_at = self.q["right_arm_shoulder_pan"]-2000-self.held_at
            self.held_at = None

    def stop(self):
        self.stops += 1
        self.s.update(phase="stopped", released=True, release_errors=[], ok=False)

    def sleep(self, seconds):
        remaining = seconds
        while remaining > 1e-10:
            dt = min(.02, remaining)
            self.now += dt
            remaining -= dt
            command_file = self.folder / "command.json"
            if command_file.exists():
                cmd = read_json(command_file)
                if cmd["id"] != self.seen:
                    self.seen = cmd["id"]
                    self.commands.append(cmd)
                    if cmd["op"] == "stop":
                        self.stop(); self.engine.active = False
                        self.s["completed"] = cmd["id"]
                    elif cmd["op"] == "trajectory":
                        self.s.update(self.engine.start(cmd, self.s["rows"], read_json(self.folder / "trajectory-vision.json"),
                                                       session_started=self.s["started"], lease_remaining=self.s["lease_remaining"]))
                if self.engine.active:
                    self.publish()
                    if self.failure == "load" and len(self.writes) > 8:
                        self.s["rows"]["right_arm_gripper"]["Present_Load"] = 600
                    try:
                        self.s.update(self.engine.tick(self.s["rows"], read_json(self.folder / "trajectory-vision.json"),
                                                       telemetry_at=self.now))
                    except Refused:
                        self.publish()
                        raise
            self.publish()

    def observe(self, after=0):
        if self.failure == "vision" and len(self.writes) > 8: self.sleep(.7)
        self.sleep(.08)
        self.seq += 1
        x = self.q["right_arm_shoulder_pan"]-2000
        paddle = x-self.held_at if self.held_at is not None else self.placed_at
        if self.failure == "no_grasp": paddle = 0
        head = {"tool": [x, 0], "target": [paddle+1, 0], "paddle_bottom": [paddle-8, 0],
                "table_edge": [0, 50], "anchor": [0, 0]}
        wrist = {"tool": [0, 0], "target": [2, 0]}
        if self.failure == "slip" and x > 465: wrist["target"] = [20, 0]
        streams = {"head": "synthetic", "right_wrist": "synthetic"}
        if self.failure == "restart" and len(self.writes) > 8: streams["head"] = "restarted"
        return Observation(np.array([x], float), self.now, {n: self.seq for n in streams},
                           {"head": head, "right_wrist": wrist}, streams, {n: self.now for n in streams})


def run(rig, out):
    trace = Trace(out)
    transport = ContinuousTransport(rig.config, Limits(), rig.profile, execute=True, clock=rig.clock, sleep=rig.sleep)
    try:
        with transport:
            e = Experiment(rig.config, transport, rig, trace, binding(rig.config), rig.clock)
            return Program(rig.recipe, e).run(None)
    finally: trace.close()


def test_full_pickup_cycle_is_six_whole_paths_then_verified_release(tmp_path):
    rig = ContinuousRig(tmp_path / "owner")
    assert preflight(rig.recipe, rig.config, None)["problems"] == []
    result = run(rig, tmp_path / "run")
    assert result["status"] == "PADDLE_PICKUP_CYCLE_PASSED"
    assert result["pickup_cycle_completed"] and result["supported_park_released"]
    assert result["model_calls"] == 0
    assert [c["op"] for c in rig.commands] == ["trajectory"]*6+["stop"]
    first = rig.commands[0]["waypoints"]
    assert first[-1]["positions"]["right_arm_shoulder_pan"]-first[0]["positions"]["right_arm_shoulder_pan"] == 456
    assert len(rig.writes) > 100
    assert rig.stops == 1 and rig.held_at is None and rig.placed_at == 8
    assert rig.now-1000 < 30


@pytest.mark.parametrize("failure,reason", [("stuck", "following error"), ("vision", "Vision watchdog"),
                                          ("load", "load limit"), ("restart", "stream mismatch"),
                                          ("no_grasp", "Lift not verified"), ("slip", "retention lost")])
def test_inflight_faults_stop_without_retry(tmp_path, failure, reason):
    rig = ContinuousRig(tmp_path / "owner", failure)
    with pytest.raises(Refused, match=reason): run(rig, tmp_path / "run")
    assert read_json(rig.folder / "command.json")["op"] == "stop"
    assert len([c for c in rig.commands if c["op"] == "trajectory"]) <= 3


@pytest.mark.parametrize("change,reason", [
    (lambda r: r.s.update(trajectory_protocol=0), "protocol/profile"),
    (lambda r: r.s.update(trajectory_profile_sha256="different"), "protocol/profile"),
    (lambda r: r.q.update(right_arm_shoulder_pan=2006), "start differs"),
    (lambda r: r.s.update(lease_remaining=10), "lease"),
])
def test_incompatible_owner_or_start_refused_before_dispatch(tmp_path, change, reason):
    rig = ContinuousRig(tmp_path / "owner")
    change(rig); rig.publish()
    with pytest.raises(Refused, match=reason): run(rig, tmp_path / "run")
    assert rig.commands == []


def started_engine(rig):
    segment = rig.recipe["motion"]["segments"]["approach"]
    vision = {"ok": True, "command_id": 1, "session_started": 900,
              "streams": {"head": "one", "right_wrist": "two"}, "sequences": {"head": 1, "right_wrist": 1},
              "captured_at": {"head": rig.now, "right_wrist": rig.now}}
    command = {"id": 1, "op": "trajectory", "session_started": 900, "profile_sha256": rig.engine.profile_hash,
               "waypoints": segment["waypoints"], "camera_streams": vision["streams"]}
    rig.engine.start(command, rig.s["rows"], vision, session_started=900, lease_remaining=180)
    return vision


@pytest.mark.parametrize("fault,reason", [("stop", "STOP"), ("gap", "tick watchdog"),
                                         ("stale", "telemetry is stale"), ("relabel", "timestamp changed"),
                                         ("rollback", "rolled back"), ("identity", "identity")])
def test_owner_interrupts_before_another_goal_write(tmp_path, fault, reason):
    rig = ContinuousRig(tmp_path / "owner")
    vision = started_engine(rig)
    rig.now += .02
    telemetry_at = rig.now
    if fault == "gap": rig.now += .2
    if fault == "stale": telemetry_at -= 1
    if fault == "relabel": vision["captured_at"]["head"] = rig.now
    if fault == "rollback": vision["sequences"]["head"] = 0
    if fault == "identity": vision["session_started"] = 901
    with pytest.raises(Refused, match=reason):
        rig.engine.tick(rig.s["rows"], vision, telemetry_at=telemetry_at, stop_requested=fault == "stop")
    assert not rig.writes and rig.stops == 1 and not rig.engine.active


def test_client_never_falls_back_to_six_degree_moves(tmp_path):
    rig = ContinuousRig(tmp_path / "owner")
    client = ContinuousTransport(rig.config, Limits(), rig.profile)
    with pytest.raises(Refused, match="fall back"): client.move("right_arm_shoulder_pan", 16)


def test_owner_rejects_wrong_profile_and_insufficient_lease_before_motion(tmp_path):
    rig = ContinuousRig(tmp_path / "owner")
    vision = started_engine(rig)
    rig.engine.active = False
    cmd = {"id": 2, "op": "trajectory", "session_started": 900, "profile_sha256": "wrong",
           "waypoints": rig.recipe["motion"]["segments"]["approach"]["waypoints"], "camera_streams": vision["streams"]}
    with pytest.raises(Refused, match="identity or owner profile"):
        rig.engine.start(cmd, rig.s["rows"], vision, session_started=900, lease_remaining=180)
    cmd["profile_sha256"] = rig.engine.profile_hash
    with pytest.raises(Refused, match="lease"):
        rig.engine.start(cmd, rig.s["rows"], vision, session_started=900, lease_remaining=1)
    assert not rig.writes


def test_missing_terminal_measurements_are_collected_before_motion(tmp_path):
    rig = ContinuousRig(tmp_path / "owner")
    rig.recipe["finish"]["supported_park_evidence"] = None
    ready = preflight(rig.recipe, rig.config, None)
    assert any("supported" in p for p in ready["problems"])
    assert not rig.commands


@pytest.mark.parametrize('terminal', [None, {}, {'mode': 'place_and_park', 'park_positions': None}])
def test_partial_terminal_recipe_refuses_before_any_command(tmp_path, terminal):
    rig = ContinuousRig(tmp_path / 'owner')
    rig.recipe['finish'] = terminal
    ready = preflight(rig.recipe, rig.config, None)
    assert ready['status'] == 'PROGRAM_NOT_READY'
    assert not rig.commands
