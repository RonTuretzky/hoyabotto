"""Local sequence and actual file-protocol regressions; no carton physics claim."""
import copy
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest

from carton.servo.common import Limits, Observation, Refused, Trace, binding, read_json
from carton.servo.controller import Experiment
from carton.servo.program import FLAPS, Program, execute, preflight, template
from carton.servo.transport import SessionTransport
from test_carton_servo import FileMotorOwner


class RecipeRig(FileMotorOwner):
    """Scripted contact outcomes over the real command/status file contract."""
    def __init__(self, folder, failure=None):
        super().__init__(folder)
        self.failure, self.seq, self.commands, self.closed = failure, 0, [], set()
        self.s.update(gripper_release_generation=0, automatic_gripper_reenable=False)
        self.publish()
        self.limits = Limits()
        ref = folder / "reference.txt"
        ref.write_text("synthetic reference, never presented as a camera image")
        regions = {n: {"anchor": n in ("anchor", "table_edge")} for n in
                   ("tool", "target", "paddle_bottom", "table_edge", "anchor", *FLAPS)}
        self.config.update(schema=1, units="encoder_ticks", target=[0],
                           cameras={name: {"camera_id": name, "reference": str(ref),
                                           "manifest": str(folder / (name+".json")), "regions": copy.deepcopy(regions)}
                                    for name in ("head", "right_wrist")},
                           measurements=[{"name": "tool_station_x", "camera": "head", "a": "tool", "b": "anchor", "axis": 0}])
        joint = self.config["joints"][0]
        self.model = {"schema": 1, "units": "encoder_ticks", "status": "LOCAL_MODEL_VALIDATED",
                      "fingerprint": binding(self.config), "joints": [joint], "origin": self.q.copy(),
                      "origin_features": [0], "camera_streams": {"head": "synthetic", "right_wrist": "synthetic"},
                      "created_at": self.now, "motor_session_started": self.s["started"], "limits": asdict(self.limits),
                      "samples": [{"joint": joint, "dq": [d], "dy": [d]} for d in (32, -32)],
                      "holdout": [{"joint": joint, "dq": [16], "dy": [16]}], "jacobian": [[1]]}
        self.recipe = template(folder / "config.json")
        self.recipe.update(approach=[[0]], gripper={"open_ticks": 2000, "empty_closed_ticks": 1940,
                                                  "goal_ticks": 1984, "min_aperture_ticks": 20})
        self.recipe["lift"].update(targets=[[20]], up_normal=[1, 0], min_clearance_px=8)
        for i, fold in enumerate(self.recipe["folds"]):
            fold.update(targets=[[28+i*8]], retract=[[20]],
                        verification={"seconds": .5, "constraints": [
                            {"role": "flap", "camera": "head", "a": fold["name"], "b": "anchor",
                             "direction": [1, 0], "interval_px": [9, 11]},
                            {"role": "tool_clearance", "camera": "head", "a": "tool", "b": fold["name"],
                             "direction": [1, 0], "interval_px": [8, 20]}]})

    def sleep(self, seconds):
        path = self.folder / "command.json"
        if path.exists():
            cmd = read_json(path)
            if cmd["id"] != self.seen:
                self.commands.append(cmd)
        super().sleep(seconds)
        x = self.q["right_arm_shoulder_pan"]-2000
        for i, name in enumerate(FLAPS):
            if x >= 25+i*8:
                self.closed.add(name)
        if self.failure == "temperature" and self.q["right_arm_gripper"] != 2000:
            self.s["rows"]["right_arm_gripper"]["Present_Temperature"] = 94
            from carton.servo.common import atomic_json
            atomic_json(self.folder / "status.json", self.s)

    def observe(self, after=0):
        self.sleep(.2)
        self.seq += 1
        x = self.q["right_arm_shoulder_pan"]-2000
        paddle = 0 if self.failure == "no_grasp" else x
        head = {"tool": [x, 0], "target": [paddle+1, 0], "paddle_bottom": [paddle-8, 0],
                "table_edge": [0, 50], "anchor": [0, 0]}
        for name in FLAPS:
            closed = name in self.closed and not (self.failure == "springback" and x < 25)
            head[name] = [10 if closed else 100, 0]
        wrist = {"tool": [0, 0], "target": [10 if self.failure == "slip" and x >= 25 else 2, 0]}
        return Observation(np.array([x], float), self.now,
                           {"head": self.seq, "right_wrist": self.seq}, {"head": head, "right_wrist": wrist},
                           self.model["camera_streams"])


def run_rig(rig, output):
    trace = Trace(output)
    transport = SessionTransport(rig.config, rig.limits, True, rig.clock, rig.sleep)
    try:
        with transport:
            e = Experiment(rig.config, transport, rig, trace, binding(rig.config), rig.clock)
            return Program(rig.recipe, e).run(rig.model)
    finally:
        trace.close()


def test_continuous_program_uses_one_owner_and_does_not_stop_between_phases(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    result = run_rig(rig, tmp_path / "trace")
    assert result["status"] == "CARTON_VISUAL_CHECKS_PASSED"
    assert result["verified_flaps"] == list(FLAPS)
    assert result["owner_session_started"] == 900
    assert result["model_calls"] == 0
    assert result["physical_task_completed"] is False
    assert len(rig.commands) > 10 and all(c["op"] == "move" for c in rig.commands)
    assert rig.now-1000 < 60
    assert read_json(tmp_path / "trace" / "progress.json")["stage"] == "visual_checks_passed"


@pytest.mark.parametrize("failure,reason", [("no_grasp", "Lift not verified"), ("slip", "retention lost"),
                                            ("springback", "flap evidence failed"), ("temperature", "temperature limit")])
def test_failed_physical_evidence_or_health_stops_without_retry(tmp_path, failure, reason):
    rig = RecipeRig(tmp_path / "owner", failure)
    with pytest.raises(Refused, match=reason):
        run_rig(rig, tmp_path / "trace")
    assert read_json(rig.folder / "command.json")["op"] == "stop"
    assert not any("renew" in c["op"] for c in rig.commands)
    events = [__import__("json").loads(line) for line in (tmp_path / "trace" / "trace.jsonl").read_text().splitlines()]
    assert not any(e.get("stage") == "fold_short_right" for e in events)


def test_collects_all_missing_commissioning_inputs_before_any_owner_access(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    draft = template("unused.json")
    result = preflight(draft, rig.config, None)
    assert result["status"] == "PROGRAM_NOT_READY"
    assert len(result["problems"]) >= 10
    assert any("model:" in x for x in result["problems"])
    assert any("near_long.verification:" in x for x in result["problems"])
    assert not (rig.folder / "command.json").exists()


@pytest.mark.parametrize("mutation,pattern", [
    (lambda r: r["approach"].__setitem__(0, [200]), "neighborhood"),
    (lambda r: r["folds"].pop(), "four flaps"),
    (lambda r: r["gripper"].update(goal_ticks=1940), "between"),
    (lambda r: r["lift"].update(up_normal=[0, 0]), "unit"),
    (lambda r: r["folds"][0]["verification"]["constraints"].pop(), "both"),
])
def test_invalid_recipe_never_sends_a_partial_program(tmp_path, mutation, pattern):
    rig = RecipeRig(tmp_path / "owner")
    mutation(rig.recipe)
    result = execute(rig.recipe, rig.config, rig.model, tmp_path / "trace", execute=True)
    assert result["status"] == "PROGRAM_NOT_READY"
    assert pattern in " ".join(result["problems"])
    assert not (rig.folder / "command.json").exists()


def test_ready_static_check_does_not_claim_live_readiness_or_start_motors(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    result = execute(rig.recipe, rig.config, rig.model, tmp_path / "trace")
    assert result["status"] == "PROGRAM_STATIC_CHECKS_PASSED"
    assert result["requires_live_checks"] is True
    assert not (rig.folder / "command.json").exists()


@pytest.mark.parametrize("failure,reason", [("lease", "lease"), ("session", "session changed"), ("scene", "scene no longer")])
def test_live_preflight_refuses_before_the_first_motor_command(tmp_path, failure, reason):
    rig = RecipeRig(tmp_path / "owner")
    if failure == "lease": rig.s["lease_remaining"] = 30
    elif failure == "session": rig.s["started"] = 901
    else: rig.model["origin_features"] = [20]
    rig.publish()
    with pytest.raises(Refused, match=reason):
        run_rig(rig, tmp_path / "trace")
    assert not (rig.folder / "command.json").exists()


def test_gripper_method_preserves_endpoint_checks_and_excludes_other_joints(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    with SessionTransport(rig.config, rig.limits, True, rig.clock, rig.sleep) as transport:
        q, _ = transport.move_gripper(-16)
        assert q["right_arm_gripper"] == 1984
        with pytest.raises(Refused, match="Invalid"):
            transport.move("right_arm_gripper", -16)
        with pytest.raises(Refused, match="Invalid"):
            transport.move_gripper(69)


def test_gripper_stall_is_not_treated_as_contact_success(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    rig.mode = "stuck"
    with pytest.raises(Refused, match="settling"):
        run_rig(rig, tmp_path / "trace")
    assert read_json(rig.folder / "command.json")["op"] == "stop"


def test_old_owner_cannot_silently_reenergize_the_gripper_during_a_grasp(tmp_path):
    rig = RecipeRig(tmp_path / "owner")
    del rig.s["gripper_release_generation"]
    rig.publish()
    with pytest.raises(Refused, match="release generation"):
        run_rig(rig, tmp_path / "trace")
    assert not (rig.folder / "command.json").exists()
