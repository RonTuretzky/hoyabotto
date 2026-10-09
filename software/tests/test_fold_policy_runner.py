"""carton.fold_policy_runner against the deployed owner code on fake hardware. No robot, network or camera access.

Fast tests use the qwen-bridge HardwareOwner/DirectJointClient/gemma_robot_tools stack over a kinematic plant
(carton.fold_policy_fakes). The MuJoCo tests need the recorded fold demonstrations; the checkpoint test also needs
the trained checkpoint and FOLD_POLICY_E2E=1 (it runs a whole simulated episode).
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from carton import fold_policy_fakes as F
from carton import fold_policy_runner as R
from carton.servo.common import Refused

SOFTWARE = Path(__file__).resolve().parents[1]
HACK = Path("/Users/wk/Documents/ChatGPT/Hackatuson/output")
TRIAL = Path(os.environ.get("FOLD_POLICY_TRIAL", HACK / "fold-demos/batch-01/trial-000"))
CHECKPOINT = Path(os.environ.get("FOLD_POLICY_CHECKPOINT",
                                 HACK / "fold-train/act-both-shorts-v1/checkpoints/last/pretrained_model"))


class Toward:
    """Scripted stand-in policy: move every joint toward `goal` (radians) by at most `rate` per tick."""
    def __init__(self, goal, rate=0.08, on_act=None, cameras=R.CAMERA_KEYS):
        self.goal, self.rate, self.on_act, self.calls = np.asarray(goal, float), rate, on_act, 0
        self.cameras = set(cameras)

    def reset(self):
        self.calls = 0

    def act(self, state, images, task=None):
        assert set(images) == self.cameras and task == R.TASK_TEXT
        self.calls += 1
        if self.on_act:
            self.on_act(self.calls)
        return state + np.clip(self.goal - state, -self.rate, self.rate)


def runner(rig, policy, tmp_path, name="run", **config):
    safety = config.pop("safety", R.SafetyConfig())
    envelope = config.pop("envelope", None)
    stop = config.pop("stop_requested", None)
    return R.FoldPolicyRunner(policy, rig.transport, rig.cameras, rig.arm_maps, R.RunnerConfig(safety=safety, **config),
                              tmp_path / name, envelope=envelope, clock=rig.clock, sleep=rig.sleep, stop_requested=stop)


def ticks_log(tmp_path, name="run"):
    return [json.loads(line) for line in (tmp_path / name / "ticks.jsonl").read_text().splitlines()]


def tools_called(rig):
    return [c[0] for c in rig.owner.calls]


GOAL = np.zeros(12)
GOAL[1], GOAL[7] = 0.3, -0.3          # both shoulder lifts move about 196 ticks


# ------------------------------------------------------------------------------------------ joint maps
def test_unmeasured_joint_maps_are_refused_with_every_gap(tmp_path):
    cal = tmp_path / "farm_xlerobot.json"
    cal.write_text((SOFTWARE / "calibration/farm_xlerobot/farm_xlerobot.json").read_text())
    template = {"schema": 1, "arm": "left", "calibration_file": str(cal),
                "joints": {s: {"model_zero_tick": None, "model_sign": None} for s in R.SUFFIXES[:5]}}
    (tmp_path / "left.json").write_text(json.dumps(template))
    with pytest.raises(Refused) as e:
        R.load_arm_map(tmp_path / "left.json", "left")
    for gap in ("joints.shoulder_pan.model_zero_tick", "joints.wrist_roll.model_sign", "gripper.model_zero_tick",
                "evidence.joints", "evidence.gripper"):
        assert gap in str(e.value)
    # The robot's current right-arm configuration is the unvalidated LeRobot midpoint candidate: refused.
    deployed = SOFTWARE / "docs/commissioning/2026-10-07-paddle-success/qwen-bridge/right-arm-kinematics.json"
    with pytest.raises(Refused) as e:
        R.load_arm_map(deployed, "right")
    assert "feetech_degrees_v1" in str(e.value) and "gripper.model_sign" in str(e.value)
    with pytest.raises(Refused):
        R.load_arm_maps({"left": tmp_path / "left.json"})


def test_joint_map_bound_to_the_saved_calibration(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    path = rig.map_paths["right"]
    cfg = json.loads(path.read_text())
    cfg["calibration_sha256"] = "0" * 64
    path.write_text(json.dumps(cfg))
    with pytest.raises(Refused, match="another calibration"):
        R.load_arm_map(path, "right")


def test_ticks_radians_round_trip_with_both_signs(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    left, right = rig.arm_maps["left"], rig.arm_maps["right"]
    ticks = {n: 2048 + 100 for n in R.OWNER_JOINTS}
    lrad, rrad = left.ticks_to_rad(ticks), right.ticks_to_rad(ticks)
    assert lrad[0] == pytest.approx(-math.radians(100 * 360 / 4096)) and rrad[0] == pytest.approx(-lrad[0])
    assert left.rad_to_ticks(lrad) == pytest.approx([2148] * 6) and right.rad_to_ticks(rrad) == pytest.approx([2148] * 6)
    with pytest.raises(Refused, match="outside saved calibration"):
        right.ticks_to_rad({**ticks, "right_arm_elbow_flex": 4000})


def test_limits_cannot_be_relaxed():
    for bad in (dict(step_ticks=41), dict(min_step_ticks=2), dict(range_margin_ticks=39), dict(joint_max_age_s=2.0),
                dict(frame_max_age_s=1.5)):
        with pytest.raises(Refused):
            R.SafetyConfig(**bad).validate()


# ------------------------------------------------------------------------------------- target planning
def snapshot(ticks, goals=None, ranges=(826, 3268)):
    rows = {n: {"Present_Position": ticks[n], "captured_at": 10.0, "Torque_Enable": 1, "Status": 0, "Present_Load": 0}
            for n in R.OWNER_JOINTS}
    return R.parse_owner_status({"hardware_server": True, "control_mode": "direct_joint", "rows": rows, "time": 10.0,
                                 "status_age_s": 0.0, "started": 1.0, "phase": "holding", "ok": True, "stop_count": 0,
                                 "enabled_motors": list(R.OWNER_JOINTS), "goals": goals or dict(ticks),
                                 "ranges": {n: list(ranges) for n in R.OWNER_JOINTS},
                                 "execution_profile": "paddle-success-v1"}, local_sent=10.0, local_received=10.0)


def planner(tmp_path, **config):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    return runner(rig, Toward(GOAL), tmp_path, **config), rig


def test_targets_are_clamped_per_tick_and_to_the_commandable_range(tmp_path):
    r, rig = planner(tmp_path)
    ticks = {n: 2048 for n in R.OWNER_JOINTS}
    ticks["right_arm_wrist_flex"] = 3200            # 68 ticks inside the saved max, 28 beyond the commandable max
    snap = snapshot(ticks)
    action = np.zeros(12)
    action[R.OWNER_JOINTS.index("right_arm_shoulder_pan")] = 0.5      # +326 ticks wanted
    action[R.OWNER_JOINTS.index("left_arm_shoulder_pan")] = 0.5       # left counts the other way: -326 ticks
    action[R.OWNER_JOINTS.index("right_arm_elbow_flex")] = math.radians(2 * 360 / 4096)   # 2 ticks: below the rule
    action[R.OWNER_JOINTS.index("right_arm_wrist_flex")] = 2.0        # far beyond the range
    send, detail = r.plan_targets(action, snap)
    assert send["right_arm_shoulder_pan"] == 2088 and send["left_arm_shoulder_pan"] == 2008
    assert detail["right_arm_shoulder_pan"]["clamps"] == ["step"]
    assert "right_arm_elbow_flex" not in send and detail["right_arm_elbow_flex"]["clamps"] == ["below_min_step"]
    assert send["right_arm_wrist_flex"] == 3268 - 40 and "commandable_range" in detail["right_arm_wrist_flex"]["clamps"]
    assert all(abs(q - ticks[n]) <= 40 for n, q in send.items())


def test_grippers_hold_by_default_and_never_stream_a_closure(tmp_path):
    r, rig = planner(tmp_path)
    ticks = {n: 2048 for n in R.OWNER_JOINTS}
    action = np.asarray(rig.arm_maps["left"].ticks_to_rad(ticks) + rig.arm_maps["right"].ticks_to_rad(ticks))
    gl, gr = R.OWNER_JOINTS.index("left_arm_gripper"), R.OWNER_JOINTS.index("right_arm_gripper")
    action[gl] -= 0.2          # left counts the other way: opening
    action[gr] -= 0.2          # right: closing
    send, detail = r.plan_targets(action, snapshot(ticks))
    assert not send and detail["left_arm_gripper"]["clamps"] == ["step", "gripper_hold_mode"]
    follow, _ = planner(tmp_path / "follow", gripper_mode="follow")
    send, detail = follow.plan_targets(action, snapshot(ticks))
    assert "left_arm_gripper" in send and "right_arm_gripper" not in send
    assert "gripper_close_not_streamable" in detail["right_arm_gripper"]["clamps"]
    holding, _ = planner(tmp_path / "holding", gripper_mode="follow", holding_arms=("left",))
    send, detail = holding.plan_targets(action, snapshot(ticks))
    assert not send and "gripper_holding_object" in detail["left_arm_gripper"]["clamps"]


def test_stream_gripper_mode_closes_in_small_steps_only_on_a_streaming_transport(tmp_path):
    with pytest.raises(Refused, match="streams jaw closures"):
        planner(tmp_path, gripper_mode="stream")             # the deployed owner (ApiOwnerTransport) cannot
    rig = F.build_kinematic_rig(tmp_path / "rig2")
    rig.transport.streams_gripper_closure = True             # stands in for the owner's stream mode
    r = runner(rig, Toward(GOAL), tmp_path, name="stream", gripper_mode="stream")
    ticks = {n: 2048 for n in R.OWNER_JOINTS}
    action = np.asarray(rig.arm_maps["left"].ticks_to_rad(ticks) + rig.arm_maps["right"].ticks_to_rad(ticks))
    gr = R.OWNER_JOINTS.index("right_arm_gripper")
    action[gr] -= 0.2                                        # right: closing, ~130 ticks
    send, detail = r.plan_targets(action, snapshot(ticks))
    assert send["right_arm_gripper"] == 2048 - R.GRIPPER_STREAM_STEP
    assert "gripper_stream_step" in detail["right_arm_gripper"]["clamps"]


def test_stream_transport_refuses_an_owner_without_stream_mode(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.transport = R.StreamOwnerTransport(rig.owner, clock=rig.clock)
    summary = runner(rig, Toward(GOAL), tmp_path, gripper_mode="stream", max_steps=3).run()
    assert "not in stream mode" in summary["aborted"] and summary["commands_sent"] == 0


def test_stream_owner_executes_both_arms_and_jaw_closures_in_one_call_per_tick(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig", stream=True)
    rig.enable_all()
    start = rig.owner.owner.state["stop_count"]
    goal = np.asarray(rig.arm_maps["left"].ticks_to_rad({n: 2048 for n in R.OWNER_JOINTS})
                      + rig.arm_maps["right"].ticks_to_rad({n: 2048 for n in R.OWNER_JOINTS}))
    gr = R.OWNER_JOINTS.index("right_arm_gripper")
    goal[gr] -= 0.15                                          # close the right jaw ~100 ticks while the arms move
    goal[R.OWNER_JOINTS.index("left_arm_shoulder_pan")] += 0.3
    summary = runner(rig, Toward(goal, rate=0.05), tmp_path, execute=True, gripper_mode="stream", max_steps=40).run()
    assert summary["aborted"] is None and summary["commands_sent"] > 0
    names = [c[0] for c in rig.owner.calls]
    assert "robot_move_joint_targets" not in names and "robot_stop" not in names
    assert names.count("robot_stream_joint_targets") == summary["commands_sent"] and names[-1] == "robot_hold_here"
    sent = [a for n, a, _ in rig.owner.calls if n == "robot_stream_joint_targets"]
    assert any("right_arm_gripper" in a["positions"] and any(k.startswith("left_arm_") for k in a["positions"])
               for a in sent)
    assert rig.owner.owner.state["stop_count"] == start and rig.owner.faults == []
    jaw = rig.owner.owner.rows["right_arm_gripper"]["Present_Position"]
    assert jaw < 2048 - 60                                    # the jaw closed while streaming


def test_race_guard_never_commands_a_target_the_moving_joint_will_pass(tmp_path):
    r, rig = planner(tmp_path)
    n = "right_arm_shoulder_pan"
    ticks = {m: 2048 for m in R.OWNER_JOINTS}
    goals = dict(ticks, **{n: 2088})                 # holding goal 40 ahead: the joint is on its way there
    action = np.asarray(rig.arm_maps["left"].ticks_to_rad(ticks) + rig.arm_maps["right"].ticks_to_rad(ticks))
    action[R.OWNER_JOINTS.index(n)] = math.radians(20 * 360 / 4096)   # 20 ticks: between present and held goal
    send, detail = r.plan_targets(action, snapshot(ticks, goals))
    assert n not in send and detail[n]["clamps"] == ["race_guard_hold"]
    action[R.OWNER_JOINTS.index(n)] = math.radians(-20 * 360 / 4096)  # reversing, clear of the path: allowed
    send, _ = r.plan_targets(action, snapshot(ticks, goals))
    assert send[n] == 2028


def test_training_envelope_clips_actions(tmp_path):
    env = R.TrainingEnvelope(*(np.full(12, v) for v in (-0.1, 0.1, 0.0, 0.05, -0.1, 0.1)))
    r, rig = planner(tmp_path, envelope=env)
    ticks = {n: 2048 for n in R.OWNER_JOINTS}
    action = np.full(12, 1.0)
    send, detail = r.plan_targets(action, snapshot(ticks))
    i = R.OWNER_JOINTS.index("right_arm_elbow_flex")
    assert detail["right_arm_elbow_flex"]["clamps"][0] == "training_envelope"
    assert detail["right_arm_elbow_flex"]["raw_ticks"] == pytest.approx(2048 + math.degrees(0.1 + math.radians(5)) * 4096 / 360)


# ----------------------------------------------------------------------------------- whole loop, fake owner
def test_dry_run_is_the_default_and_sends_nothing(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    writes = rig.owner.bus.writes
    summary = runner(rig, Toward(GOAL), tmp_path, max_steps=12).run()
    assert summary["aborted"] is None and summary["steps"] == 12 and summary["commands_sent"] == 0
    assert "robot_move_joint_targets" not in tools_called(rig) and "robot_halt_motion" not in tools_called(rig)
    assert rig.owner.bus.writes == writes
    log = ticks_log(tmp_path)
    assert len(log) == 12 and all(t["mode"] == "dry-run" and t["acks"] is None for t in log)
    assert log[0]["send"] and set(log[0]["frames"]) == set(R.CAMERA_KEYS) and len(log[0]["frames"]["front"]["sha256"]) == 64
    assert (tmp_path / "run/preflight.json").exists() and (tmp_path / "run/summary.json").exists()


def test_execute_moves_both_arms_within_the_step_bound(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig", rate_ticks_s=400)
    rig.enable_all()
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=40).run()
    assert summary["aborted"] is None and summary["commands_sent"] > 0
    assert rig.owner.faults == [] and rig.owner.owner.state["stop_count"] == 0
    for n in ("left_arm_shoulder_lift", "right_arm_shoulder_lift"):
        assert abs(rig.plant.ticks(n) - 1852) <= 3
    for t in ticks_log(tmp_path):
        state = dict(zip(R.OWNER_JOINTS, t["state_ticks"]))
        assert all(abs(q - state[n]) <= 40 for n, q in t["send"].items())
    moves = rig.owner.moves()
    assert all(m["wait"] is False and m["replace"] is True and m["duration_s"] == 0.4 for m in moves)
    assert {m["arm"] for m in moves} == {"left", "right"}
    assert "robot_stop" not in tools_called(rig)
    # The run ends holding: halted, every arm motor still enabled.
    assert tools_called(rig)[-1] == "robot_halt_motion"
    assert set(rig.owner.owner.enabled) == set(R.OWNER_JOINTS)


def test_policy_holding_still_sends_nothing(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()
    summary = runner(rig, Toward(np.zeros(12), rate=0.0), tmp_path, execute=True, max_steps=8).run()
    assert summary["aborted"] is None and summary["commands_sent"] == 0 and not rig.owner.moves()


def test_execute_refuses_released_arms(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=5).run()
    assert summary["aborted"].startswith("refused before motion: Enable all twelve arm motors")
    assert summary["steps"] == 0 and not rig.owner.moves()


def test_stop_hook_halts_and_releases_nothing(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()
    policy = Toward(GOAL)
    summary = runner(rig, policy, tmp_path, execute=True, max_steps=50, stop_requested=lambda: policy.calls >= 6).run()
    assert "Stop requested" in summary["aborted"] and summary["steps"] == 5
    assert "robot_halt_motion" in tools_called(rig) and "robot_stop" not in tools_called(rig)
    assert set(rig.owner.owner.enabled) == set(R.OWNER_JOINTS) and rig.owner.owner.state["stop_count"] == 0


def test_operator_stop_ends_the_run_without_re_enabling(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()
    sleeps = []

    def sleep(dt):                       # the operator presses STOP while the runner waits for its next tick
        sleeps.append(dt)
        rig.sleep(dt)
        if len(sleeps) == 4:
            rig.owner.call("robot_stop", {})
    r = R.FoldPolicyRunner(Toward(GOAL), rig.transport, rig.cameras, rig.arm_maps, R.RunnerConfig(execute=True, max_steps=50),
                           tmp_path / "run", clock=rig.clock, sleep=sleep)
    summary = r.run()
    assert "Owner STOP or fault" in summary["aborted"] and summary["steps"] < 50
    names = tools_called(rig)
    assert names.count("robot_set_motor_enable") == 2 and not rig.owner.owner.enabled   # released, stays released
    # STOP pressed during the policy call instead: the owner/client refuse that tick's targets; the run also ends.
    rig2 = F.build_kinematic_rig(tmp_path / "rig2")
    rig2.enable_all()
    stop = lambda calls: calls == 6 and rig2.owner.call("robot_stop", {})
    summary = runner(rig2, Toward(GOAL, on_act=stop), tmp_path, name="run2", execute=True, max_steps=50).run()
    assert summary["aborted"] and summary["steps"] == 5 and not rig2.owner.owner.enabled


def test_stale_joint_telemetry_trips_the_watchdog(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")

    def hang(calls):
        if calls == 5:
            rig.owner.paused = True       # the owner loop stops publishing status.json
    summary = runner(rig, Toward(GOAL, on_act=hang), tmp_path, max_steps=50).run()
    assert "Joint telemetry" in summary["aborted"], summary["aborted"]
    # Executing, the owner's own client refuses the same tick's targets on its stale status; the run ends either way.
    rig2 = F.build_kinematic_rig(tmp_path / "rig2")
    rig2.enable_all()
    summary = runner(rig2, Toward(GOAL, on_act=lambda c: setattr(rig2.owner, "paused", c >= 5)), tmp_path, name="run2",
                     execute=True, max_steps=50).run()
    assert summary["aborted"] and summary["steps"] <= 5


def test_stale_or_repeated_frames_abort(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()
    rig.cameras.delay_s = 1.5
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=5).run()
    assert "Camera watchdog" in summary["aborted"]
    rig2 = F.build_kinematic_rig(tmp_path / "rig2")
    rig2.enable_all()
    summary = runner(rig2, Toward(GOAL, on_act=lambda c: setattr(rig2.cameras, "freeze", c >= 3)), tmp_path, name="run2",
                     execute=True, max_steps=20).run()
    assert "has not delivered a new frame" in summary["aborted"]
    assert "robot_halt_motion" in tools_called(rig2) and "robot_stop" not in tools_called(rig2)


def test_policy_failure_aborts_and_holds(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()

    class Broken(Toward):
        def act(self, state, images, task=None):
            out = super().act(state, images, task)
            return out * np.nan if self.calls >= 4 else out
    summary = runner(rig, Broken(GOAL), tmp_path, execute=True, max_steps=20).run()
    assert "non-finite" in summary["aborted"] and "robot_stop" not in tools_called(rig)


def test_contact_halt_ends_the_run(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig", rate_ticks_s=400)
    n = "right_arm_shoulder_lift"
    rig.plant.blocked[n] = 2010          # an obstacle 38 ticks along the motion
    rig.plant.extra_load[n] = 660        # contact load (owner CONTACT_LOAD 600, fault at 800)
    rig.enable_all()
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=40).run()
    assert "Contact" in summary["aborted"] or "contact_halt" in summary["aborted"], summary["aborted"]
    assert set(rig.owner.owner.enabled) == set(R.OWNER_JOINTS) and rig.owner.owner.state["stop_count"] == 0


def test_direct_client_transport_sends_both_arms_in_one_command(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig", rate_ticks_s=400)
    rig.enable_all()
    rig.transport = R.DirectClientOwnerTransport(rig.owner.client, clock=rig.clock)
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=40).run()
    assert summary["aborted"] is None and rig.owner.faults == []
    log = ticks_log(tmp_path)
    assert any(len({n.split("_arm_")[0] for n in t["send"]}) == 2 and len(t["acks"]) == 1 for t in log)
    assert abs(rig.plant.ticks("right_arm_shoulder_lift") - 1852) <= 3


def test_start_state_and_training_constant_dims(tmp_path):
    env = R.TrainingEnvelope(np.full(12, -0.2), np.full(12, 0.2), np.zeros(12), np.full(12, 0.1),
                             np.full(12, -0.3), np.full(12, 0.3))
    env.state_std[11] = 0.0                              # right gripper never moved in training
    env.state_min[1], env.state_max[1] = 0.5, 0.6       # left shoulder lift starts outside the training range
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.enable_all()
    summary = runner(rig, Toward(GOAL), tmp_path, execute=True, max_steps=3, envelope=env).run()
    assert "Start state outside the training range" in summary["aborted"]
    env.state_min[1], env.state_max[1] = -0.2, 0.2
    r = runner(rig, Toward(GOAL), tmp_path, name="run2", execute=True, max_steps=3, envelope=env)
    state = np.zeros(12)
    state[11] = math.radians(3)
    conditioned, notes = r.condition_state(state)
    assert conditioned[11] == 0.0 and notes[0]["joint"] == "right_arm_gripper"
    state[11] = math.radians(8)
    with pytest.raises(R.Abort, match="constant"):
        r.condition_state(state)


def test_api_cameras_through_the_real_camera_tool(tmp_path):
    rgb = {"oak": np.full((48, 64, 3), (200, 30, 30), np.uint8), "phone": np.full((48, 64, 3), (30, 30, 200), np.uint8)}

    def publish(owner, names):
        if "oak" in names:
            owner.publish_oak(rgb["oak"])
        if "phone" in names:
            owner.publish_phone(rgb["phone"])
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.owner.camera_publisher = publish
    cams = R.ApiCameras(rig.owner, {"top": "oak", "front": "phone"})
    frames = cams.frames()
    assert frames["top"].rgb.shape == (48, 64, 3) and frames["top"].timestamp_basis == "capture"
    assert frames["front"].timestamp_basis == "receipt"
    assert abs(int(frames["top"].rgb[..., 0].mean()) - 200) < 8 and abs(int(frames["front"].rgb[..., 2].mean()) - 200) < 8
    with pytest.raises(Refused):
        R.ApiCameras(rig.owner, {"top": "overhead"})     # not a robot camera
    rig.cameras = cams
    summary = runner(rig, Toward(GOAL, cameras=("top", "front")), tmp_path, max_steps=4).run()
    assert summary["aborted"] is None


def test_robot_model_policy_reads_head_and_wrist_cameras_through_the_real_tool(tmp_path):
    """Default mapping of the robot-model policy: front <- oak, left/right_wrist <- the wrist streams (identity-checked)."""
    rgb = {"oak": (200, 30, 30), "left_wrist": (30, 200, 30), "right_wrist": (30, 30, 200)}

    def publish(owner, names):
        if "oak" in names:
            owner.publish_oak(np.full((48, 64, 3), rgb["oak"], np.uint8))
        for name in ("left_wrist", "right_wrist"):
            if name in names:
                owner.publish_wrist(name, np.full((48, 64, 3), rgb[name], np.uint8))
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.owner.camera_publisher = publish
    rig.cameras = R.ApiCameras(rig.owner, R.DEFAULT_ROBOT_CAMERAS)
    frames = rig.cameras.frames()
    assert set(frames) == set(R.CAMERA_KEYS) and all(f.timestamp_basis == "capture" for f in frames.values())
    for key, cam in R.DEFAULT_ROBOT_CAMERAS.items():
        assert np.abs(frames[key].rgb.mean(axis=(0, 1)) - rgb[cam]).max() < 8, key
    with pytest.raises(Refused):
        R.ApiCameras(rig.owner, {"front": "head_usb"})
    summary = runner(rig, Toward(GOAL), tmp_path, max_steps=4).run()
    assert summary["aborted"] is None


class SpecPolicy(Toward):
    """Stand-in with a checkpoint-like input spec (240x320 images, as the fold datasets)."""
    def input_spec(self):
        return {"state_dim": 12, "action_dim": 12, "camera_names": list(self.cameras),
                "cameras": {f"observation.images.{k}": (240, 320) for k in self.cameras}}


def test_policy_camera_keys_must_match_the_camera_source(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    summary = runner(rig, SpecPolicy(GOAL, cameras=("top", "front")), tmp_path, max_steps=2).run()
    assert "cameras" in summary["aborted"] and summary["commands_sent"] == 0


def test_preflight_warns_when_a_camera_aspect_differs_from_training(tmp_path):
    rig = F.build_kinematic_rig(tmp_path / "rig")
    rig.cameras = F.SyntheticCameras(rig.clock, shape=(54, 96, 3))      # 16:9, like the OAK's 1080p output
    summary = runner(rig, SpecPolicy(GOAL), tmp_path, max_steps=2).run()
    warnings = json.loads((tmp_path / "run/preflight.json").read_text())["warnings"]
    assert summary["aborted"] is None
    assert sum(w.startswith("camera_aspect: ") for w in warnings) == 3
    rig2 = F.build_kinematic_rig(tmp_path / "rig2")
    rig2.cameras = F.SyntheticCameras(rig2.clock, shape=(30, 40, 3))    # 4:3 matches 240x320
    runner(rig2, SpecPolicy(GOAL), tmp_path, name="run2", max_steps=2).run()
    assert not any(w.startswith("camera_aspect") for w in json.loads((tmp_path / "run2/preflight.json").read_text())["warnings"])


def test_released_joint_maps_put_the_closed_jaw_where_the_pads_meet():
    """profiles/fold-joint-maps: URDF -10 deg (the simulation's closed jaw) at range_min + 82 (left pads meet ~1355)."""
    folder = SOFTWARE / "profiles/fold-joint-maps"
    maps = R.load_arm_maps({arm: folder / f"{arm}-joint-map.json" for arm in R.ARMS})
    cal = json.loads((folder / "calibration-2026-10-08.json").read_text())
    for arm in R.ARMS:
        name = f"{arm}_arm_gripper"
        meet = cal[name]["range_min"] + 82
        ticks = {n: 2047 for n in R.OWNER_JOINTS} | {name: meet}
        assert abs(math.degrees(maps[arm].ticks_to_rad(ticks)[-1]) + 10) < 0.1
    left = {n: 2047 for n in R.OWNER_JOINTS} | {"left_arm_gripper": 1357}
    assert abs(math.degrees(maps["left"].ticks_to_rad(left)[-1]) + 10) < 0.3   # measured meeting point 1355-1359


# ------------------------------------------------------------------------------------- MuJoCo simulation
needs_trial = pytest.mark.skipif(not (TRIAL / "demo.npz").exists(), reason="recorded fold demonstration not present")


@needs_trial
def test_simulated_station_behind_the_deployed_owner(tmp_path):
    """Replay the demonstration's own targets through runner -> API -> owner -> MuJoCo for 3 s of simulation."""
    rig = F.build_sim_rig(TRIAL, tmp_path / "rig", owner=True, api_cameras=True)
    demo = np.load(TRIAL / "demo.npz")
    rig.enable_all()
    summary = runner(rig, F.ReplayPolicy(demo["ctrl"]), tmp_path, execute=True, max_steps=30,
                     gripper_mode="follow").run()
    assert summary["aborted"] is None, summary["aborted"]
    assert rig.owner.faults == [] and summary["commands_sent"] > 0
    log = ticks_log(tmp_path)
    assert log[0]["frames"]["top"]["shape"] == [240, 320, 3] and log[0]["frames"]["front"]["basis"] == "receipt"
    start = np.asarray(log[0]["state_rad"])
    assert np.allclose(start, demo["qpos"][0][rig.plant.adr], atol=math.radians(0.1))   # tick quantization only


@pytest.mark.skipif(os.environ.get("FOLD_POLICY_E2E") != "1" or not (CHECKPOINT / "config.json").exists()
                    or not (TRIAL / "demo.npz").exists(), reason="set FOLD_POLICY_E2E=1 (needs checkpoint and trial)")
@pytest.mark.parametrize("transport", ["sim-direct", "sim-owner"])
def test_checkpoint_closed_loop_in_simulation(tmp_path, transport):
    rig = F.build_sim_rig(TRIAL, tmp_path / "rig", owner=transport == "sim-owner")
    policy = R.build_policy(str(CHECKPOINT), device=os.environ.get("FOLD_POLICY_DEVICE", "cpu"))
    envelope = R.TrainingEnvelope.from_pretrained_dir(policy.path)
    rig.enable_all()
    steps = int(os.environ.get("FOLD_POLICY_E2E_STEPS", "750"))
    r = runner(rig, F.LatencyPolicy(policy, rig.sleep, latency_s=0.03), tmp_path, execute=True, max_steps=steps,
               gripper_mode="follow", envelope=envelope)
    summary = r.run()
    score = rig.score()
    (tmp_path / "score.json").write_text(json.dumps({"summary": R._clean(summary), "score": score}, indent=1))
    print(json.dumps({"transport": transport, "aborted": summary["aborted"], "steps": summary["steps"],
                      "effective_hz": summary["effective_hz"], "clamps": summary.get("clamp_counts"), **score}))
    assert summary["steps"] > 0
