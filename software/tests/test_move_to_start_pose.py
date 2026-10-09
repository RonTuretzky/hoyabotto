"""tools/move_to_start_pose.py / carton.fold_start_pose against the deployed owner code on fake hardware.

No robot, network or camera access. Fast tests run the qwen-bridge owner stack (FakeOwner) over a kinematic plant
with a stand-in collision checker; the MuJoCo tests need the recorded fold scene (FOLD_START_POSE_TRIAL or the
default trial below) and use the real SceneChecker and the MuJoCo fold plant.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from carton import fold_policy_fakes as F
from carton import fold_start_pose as S
from carton.fold_policy_runner import ARMS, OWNER_JOINTS, load_arm_maps
from carton.servo.common import Refused

SOFTWARE = Path(__file__).resolve().parents[1]
HACK = Path("/Users/wk/Documents/ChatGPT/Hackatuson/output")
TRIAL = Path(os.environ.get("FOLD_START_POSE_TRIAL", HACK / "fold-demos/batch-220-01/trial-020"))
DATASET = HACK / "fold-datasets/both-shorts-220-v1"
CHECKPOINT = HACK / "fold-train/act_carton_both_shorts_220_bs32_lr3e5_chunk100/checkpoints/015000/pretrained_model"
needs_scene = pytest.mark.skipif(not (TRIAL / "run/scene.xml").exists() or not (TRIAL / "demo.npz").exists(),
                                 reason="MuJoCo fold training scene not present")
MOTION = {"robot_move_joint_targets", "robot_move_path", "robot_move_motor_targets", "robot_set_gripper",
          "robot_stream_joint_targets", "robot_set_motor_enable", "robot_stop", "robot_halt_motion", "robot_hold_here"}


class ClearChecker:
    """Stand-in for SceneChecker on the kinematic rig: everything is clear unless `blocked(ticks)` says otherwise."""
    def __init__(self, blocked=None):
        self.blocked = blocked or (lambda q: False)
        self.legs = []

    def describe(self):
        return {"scene": "stand-in", "clearance_mm": 15}

    def hits(self, ticks):
        return [{"between": ["arm", "obstacle"], "distance_mm": 0.0}] if self.blocked(ticks) else []

    def check_leg(self, start, end):
        self.legs.append((dict(start), dict(end)))
        for u in np.linspace(0, 1, 11):
            if self.hits({n: start[n] + (end[n] - start[n]) * u for n in OWNER_JOINTS}):
                return {"clear": False, "hits": self.hits(end), "samples": 11}
        return {"clear": True, "samples": 11}


def kinematic_rig(tmp_path, start=None, name="rig", enable=True):
    rig = F.build_kinematic_rig(tmp_path / name, start)
    if enable:
        rig.enable_all()
    return rig


def target_of(rig):
    return S.pose_ticks(S.builtin_start_pose(), rig.arm_maps)


def random_start(rig, seed, spread=600):
    """A random holding pose within the commandable band (kinematic rig: no geometry)."""
    rng = np.random.default_rng(seed)
    target = target_of(rig)
    out = {}
    for n in OWNER_JOINTS:
        lo, hi = 826 + 40, 3268 - 40
        out[n] = int(np.clip(target[n] + rng.integers(-spread, spread), lo, hi))
    return out


def mover(rig, checker, **config):
    stop = config.pop("stop_requested", None)
    robot = config.pop("robot", rig.owner)
    return S.StartPoseMover(robot, rig.arm_maps, S.builtin_start_pose(), checker, S.MoverConfig(**config),
                            clock=rig.clock, stop_requested=stop)


def called(rig):
    return [c[0] for c in rig.owner.calls]


# ------------------------------------------------------------------------------------------- start pose
def test_built_in_pose_through_the_released_joint_maps_is_the_handoff_table():
    maps = load_arm_maps({arm: SOFTWARE / f"profiles/fold-joint-maps/{arm}-joint-map.json" for arm in ARMS})
    ticks = S.pose_ticks(S.builtin_start_pose(), maps)
    handoff = {"left_arm_shoulder_pan": 1691, "left_arm_shoulder_lift": 909, "left_arm_elbow_flex": 2396,
               "left_arm_wrist_flex": 3128, "left_arm_wrist_roll": 2050, "left_arm_gripper": 1358,
               "right_arm_shoulder_pan": 2401, "right_arm_shoulder_lift": 909, "right_arm_elbow_flex": 2396,
               "right_arm_wrist_flex": 3128, "right_arm_wrist_roll": 3024, "right_arm_gripper": 1354}
    assert all(abs(ticks[n] - v) <= 1 for n, v in handoff.items()), ticks
    ranges = {n: r for m in maps.values() for n, r in m.ranges().items()}
    assert all(S.commandable(ranges, n)[0] <= t <= S.commandable(ranges, n)[1] for n, t in ticks.items())


@pytest.mark.skipif(not (TRIAL / "demo.npz").exists(), reason="demonstration not present")
def test_built_in_pose_is_the_demonstrations_and_the_training_datasets_start():
    demo = S.start_pose_from_demo(TRIAL)
    assert np.allclose(demo.rad, S.TRAINING_START_RAD, atol=1e-5)
    if (DATASET / "data").exists():
        data = S.start_pose_from_dataset(DATASET)
        assert data.episodes > 100 and data.spread_rad < 1e-4
        assert np.allclose(data.rad, S.TRAINING_START_RAD, atol=1e-4)
    if (CHECKPOINT / "config.json").exists():
        from carton.fold_policy_runner import TrainingEnvelope
        env = S.envelope_check(S.builtin_start_pose(), TrainingEnvelope.from_pretrained_dir(CHECKPOINT))
        assert all(v["inside"] for v in env.values())


def test_limits_cannot_be_relaxed(tmp_path):
    with pytest.raises(Refused):
        S.MoverConfig(arm_tolerance_ticks=31).validate()
    with pytest.raises(Refused):
        S.MoverConfig(jaw_tolerance_ticks=40).validate()
    with pytest.raises(Refused):
        S.MoverConfig(speed_ticks_s=150).validate()
    with pytest.raises(Refused):
        S.MoverConfig(execute=True, operator=" ").validate()
    with pytest.raises(Refused):
        S.Planner({}, None, speed_ticks_s=101)
    if (TRIAL / "run/scene.xml").exists():
        rig = kinematic_rig(tmp_path, enable=False)
        with pytest.raises(Refused):
            S.SceneChecker(TRIAL / "run/scene.xml", rig.arm_maps, clearance_m=0.005)


# --------------------------------------------------------------------------------------------- dry-run
def test_dry_run_sends_nothing_and_prints_the_plan(tmp_path):
    for enable in (False, True):
        rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name=f"p{enable}", enable=False), 1),
                            name=f"rig{enable}", enable=enable)
        writes, calls = rig.owner.bus.writes, len(rig.owner.calls)
        lines = []
        m = S.StartPoseMover(rig.owner, rig.arm_maps, S.builtin_start_pose(), ClearChecker(), S.MoverConfig(),
                             clock=rig.clock, log=lines.append)
        summary = m.run()
        assert summary["aborted"] is None and summary["end"] == "dry-run (nothing sent)"
        assert rig.owner.bus.writes == writes
        assert not set(called(rig)[calls:]) & MOTION and set(m.calls) == {"robot_get_execution"}
        assert summary["plan"]["legs"] and summary["target_ticks"] == target_of(rig)
        text = "\n".join(lines)
        assert "current vs target ticks" in text and "Planned sequence" in text and "left_arm_shoulder_lift" in text


def test_dry_run_without_a_scene_plans_but_marks_it_unchecked(tmp_path):
    rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name="p", enable=False), 2))
    summary = mover(rig, None).run()
    assert summary["aborted"] is None and summary["plan"]["collision_checked"] is False
    summary = mover(rig, None, execute=True, operator="ron").run()
    assert "No collision scene" in summary["aborted"] and summary["legs_sent"] == 0


# --------------------------------------------------------------------------------------------- execute
@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_execute_reaches_the_pose_from_random_starts_on_the_kinematic_rig(tmp_path, seed):
    rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name="p", enable=False), seed))
    summary = mover(rig, ClearChecker(), execute=True, operator="ron").run()
    assert summary["aborted"] is None, summary["aborted"]
    assert summary["at_start_pose"] and summary["end"] == "holding"
    assert all(abs(v) <= S.ARM_TOLERANCE_TICKS for v in summary["residual_ticks"].values())
    names = called(rig)
    assert "robot_stop" not in names and "robot_halt_motion" not in names and names.count("robot_set_motor_enable") == 2
    assert rig.owner.owner.state["stop_count"] == 0 and not rig.owner.faults
    assert set(rig.owner.owner.enabled) == set(OWNER_JOINTS)          # ends holding
    ranges = {n: (826, 3268) for n in OWNER_JOINTS}
    for move in rig.owner.moves():
        assert move["wait"] is True and move["replace"] is False and 0 < move["duration_s"] <= 25
        assert all(n.startswith(move["arm"] + "_arm_") for n in move["positions"])      # one arm per call
        jaw = [n for n in move["positions"] if n.endswith("gripper")]
        assert not jaw or len(move["positions"]) == 1                                  # jaws alone
        assert all(S.commandable(ranges, n)[0] <= t <= S.commandable(ranges, n)[1] for n, t in move["positions"].items())
    phases = [leg["phase"] for leg in summary["plan"]["legs"]]
    assert phases == sorted(phases, key=["raise", "pan_roll", "descend", "jaw"].index)   # jaws last


def test_execute_refuses_released_arms(tmp_path):
    rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name="p", enable=False), 4), enable=False)
    summary = mover(rig, ClearChecker(), execute=True, operator="ron").run()
    assert summary["aborted"].startswith("refused before motion: Both arms must be enabled and holding")
    assert not rig.owner.moves() and "robot_set_motor_enable" not in called(rig)


def test_execute_refuses_a_path_that_collides_and_sends_nothing(tmp_path):
    lift = "right_arm_shoulder_lift"
    start = random_start(kinematic_rig(tmp_path, name="p", enable=False), 5)
    target = target_of(kinematic_rig(tmp_path, name="t", enable=False))[lift]
    start[lift] = target + 400
    rig = kinematic_rig(tmp_path, start)
    middle = target + 200
    checker = ClearChecker(lambda q: abs(q[lift] - middle) < 30)     # an obstacle halfway along every ordering
    summary = mover(rig, checker, execute=True, operator="ron").run()
    assert "No collision-free sequence" in summary["aborted"] and not rig.owner.moves()
    assert summary["plan"]["blocked"]["phase"] == "raise"


def test_injected_operator_stop_aborts_cleanly(tmp_path):
    rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name="p", enable=False), 6))

    class OperatorPressesStop:            # the operator presses STOP right after the second leg finishes
        def __init__(self):
            self.moves = 0

        def call(self, name, args, request_id=None):
            out = rig.owner.call(name, args)
            if name == "robot_move_joint_targets":
                self.moves += 1
                if self.moves == 2:
                    rig.owner.call("robot_stop", {})
            return out
    m = mover(rig, ClearChecker(), execute=True, operator="ron", robot=OperatorPressesStop())
    summary = m.run()
    assert "Owner STOP or fault" in summary["aborted"] and summary["legs_sent"] == 2
    assert len(rig.owner.moves()) == 2 and "robot_stop" not in m.calls and not summary["robot_stop_called"]
    assert called(rig).count("robot_set_motor_enable") == 2 and not rig.owner.owner.enabled   # stays released


def test_stop_hook_ends_before_the_next_leg_and_releases_nothing(tmp_path):
    rig = kinematic_rig(tmp_path, random_start(kinematic_rig(tmp_path, name="p", enable=False), 7))
    summary = mover(rig, ClearChecker(), execute=True, operator="ron",
                    stop_requested=lambda: len(rig.owner.moves()) >= 1).run()
    assert "Stop requested" in summary["aborted"] and summary["legs_sent"] == 1
    assert "robot_stop" not in called(rig) and set(rig.owner.owner.enabled) == set(OWNER_JOINTS)


def test_a_stalled_jaw_is_reported_and_never_resent(tmp_path):
    rig = kinematic_rig(tmp_path, enable=False)
    target = target_of(rig)
    start = dict(target, right_arm_gripper=target["right_arm_gripper"] + 160)
    rig = kinematic_rig(tmp_path, start, name="rig2")
    jaw = "right_arm_gripper"
    rig.plant.blocked[jaw] = target[jaw] + 90          # the jaw stalls mid-travel (as the right jaw does today)
    telemetry = rig.owner.owner.telemetry

    def stalled(bus, n):                               # a stalled servo reports at rest, not Moving
        row = telemetry(bus, n)
        if n == jaw and rig.plant.ticks(n) == rig.plant.blocked[jaw]:
            row.update(Moving=0, Present_Velocity=0)
        return row
    rig.owner.owner.telemetry = stalled
    summary = mover(rig, ClearChecker(), execute=True, operator="ron").run()
    assert summary["aborted"] is None, summary["aborted"]
    jaw_moves = [m for m in rig.owner.moves() if "right_arm_gripper" in m["positions"]]
    assert len(jaw_moves) == 1
    assert summary["arms_at_start_pose"] and not summary["at_start_pose"]
    assert summary["jaws"]["right"]["ok"] is False and "not re-sent" in summary["jaws"]["right"]["note"]
    assert "robot_stop" not in called(rig)
    # A stall the owner treats as a fault (it releases everything): the tool aborts and sends nothing more.
    rig = kinematic_rig(tmp_path, start, name="rig3")
    rig.plant.blocked[jaw] = target[jaw] + 90
    summary = mover(rig, ClearChecker(), execute=True, operator="ron").run()
    assert "Leg 1" in summary["aborted"] and len(rig.owner.moves()) == 1 and "robot_stop" not in called(rig)


# ---------------------------------------------------------------------------------- MuJoCo fold scene
def sim_rig(tmp_path, seed):
    rig = F.build_sim_rig(TRIAL, tmp_path / f"sim{seed}", owner=True)
    checker = S.SceneChecker(TRIAL / "run/scene.xml", rig.arm_maps)
    ranges = {n: (rig.calibration[n]["range_min"], rig.calibration[n]["range_max"]) for n in OWNER_JOINTS}
    start = S.random_safe_start(checker, rig.arm_maps, ranges, np.random.default_rng(seed))
    S.place_sim_arms(rig, start)
    return rig, checker, start


@needs_scene
def test_scene_checker_sees_the_carton_and_the_other_arm(tmp_path):
    zero_sign, ranges = F.sim_zero_sign_and_ranges(__import__("mujoco").MjModel.from_xml_path(str(TRIAL / "run/scene.xml")))
    maps, _ = F.write_joint_maps(tmp_path / "maps", F.fake_calibration(ranges), zero_sign, evidence="test")
    checker = S.SceneChecker(TRIAL / "run/scene.xml", maps)
    as_ticks = lambda rad: dict(zip(OWNER_JOINTS, maps["left"].rad_to_ticks(rad[:6]) + maps["right"].rad_to_ticks(rad[6:])))
    assert checker.hits(as_ticks(np.asarray(S.TRAINING_START_RAD))) == []
    folding = np.load(TRIAL / "demo.npz")["qpos"][150][:12]          # mid-fold: the jaws are on the (nominal) flaps
    assert any("short_left" in h["between"] or "long_near" in h["between"] for h in checker.hits(as_ticks(folding)))
    crossed = np.asarray(S.TRAINING_START_RAD).copy()
    crossed[0], crossed[1], crossed[2] = math.radians(100), 0.0, 0.0     # left arm swung across into the right arm
    assert any(h["between"][0].startswith("left") and h["between"][1].startswith("right") or
               h["between"][0].startswith("right") and h["between"][1].startswith("left")
               for h in checker.hits(as_ticks(crossed)))
    no_carton = S.SceneChecker(TRIAL / "run/scene.xml", maps, carton=False).hits(as_ticks(folding))
    assert not [h for h in no_carton if any(b.startswith(("short_", "long_", "carton")) for b in h["between"])]


@needs_scene
@pytest.mark.parametrize("seed", [1, 3, 5])
def test_execute_in_the_mujoco_fold_scene_reaches_the_pose_without_touching_the_carton(tmp_path, seed):
    rig, checker, start = sim_rig(tmp_path, seed)
    rig.enable_all()
    summary = S.StartPoseMover(rig.owner, rig.arm_maps, S.builtin_start_pose(), checker,
                               S.MoverConfig(execute=True, operator="sim"), clock=rig.clock).run()
    assert summary["aborted"] is None, summary["aborted"]
    assert summary["at_start_pose"]
    assert all(abs(v) <= S.ARM_TOLERANCE_TICKS for v in summary["residual_ticks"].values())
    score = rig.score()
    assert score["owner_faults"] == [] and score["owner_stop_count"] == 0
    assert score["max_robot_flap_penetration_mm"] == 0 and score["max_robot_other_penetration_mm"] == 0
    assert score["max_carton_translation_mm"] < 3          # settling only; nothing pushed it
    assert all(leg["check"]["clear"] for leg in summary["plan"]["legs"])


@needs_scene
def test_mujoco_refuses_when_the_arms_already_sit_in_the_carton(tmp_path):
    rig = F.build_sim_rig(TRIAL, tmp_path / "sim", owner=True)
    checker = S.SceneChecker(TRIAL / "run/scene.xml", rig.arm_maps)
    folding = np.load(TRIAL / "demo.npz")["qpos"][150][:12]
    ticks = rig.arm_maps["left"].rad_to_ticks(folding[:6]) + rig.arm_maps["right"].rad_to_ticks(folding[6:])
    S.place_sim_arms(rig, {n: int(round(t)) for n, t in zip(OWNER_JOINTS, ticks)})
    rig.enable_all()
    summary = S.StartPoseMover(rig.owner, rig.arm_maps, S.builtin_start_pose(), checker,
                               S.MoverConfig(execute=True, operator="sim"), clock=rig.clock).run()
    assert "No collision-free sequence: blocked in phase start" in summary["aborted"]
    assert not rig.owner.moves()


@needs_scene
def test_cli_simulation_mode(tmp_path, capsys):
    import importlib.util
    spec = importlib.util.spec_from_file_location("move_to_start_pose", SOFTWARE / "tools/move_to_start_pose.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    code = tool.main(["--sim", str(TRIAL), "--sim-start", "random:2", "--sim-workdir", str(tmp_path / "w"),
                      "--out", str(tmp_path / "out")])
    out = capsys.readouterr().out
    assert code == 0 and "Planned sequence" in out and "left_shoulder_lift" in out
    summary = json.loads((tmp_path / "out/summary.json").read_text())
    assert summary["end"] == "dry-run (nothing sent)" and summary["legs_sent"] == 0
    code = tool.main(["--sim", str(TRIAL), "--sim-start", "random:2", "--sim-workdir", str(tmp_path / "w2"),
                      "--execute", "--operator", "sim"])
    assert code == 0 and "at training start pose" in capsys.readouterr().out


@needs_scene
def test_released_joint_maps_and_saved_ranges_reach_the_handoff_ticks(tmp_path):
    """The robot's own numbers (profiles/fold-joint-maps, 8 Oct ranges) on the kinematic rig, MuJoCo-checked."""
    cal = json.loads((SOFTWARE / "profiles/fold-joint-maps/calibration-2026-10-08.json").read_text())
    real = load_arm_maps({arm: SOFTWARE / f"profiles/fold-joint-maps/{arm}-joint-map.json" for arm in ARMS})
    zero_sign = {f"{arm}_arm_{s}": (u.model_zero_tick, u.model_sign) for arm in ARMS for s, u in real[arm].units.items()}
    ranges = {n: (cal[n]["range_min"], cal[n]["range_max"]) for n in OWNER_JOINTS}
    start = S.random_safe_start(S.SceneChecker(TRIAL / "run/scene.xml", real), real, ranges, np.random.default_rng(4))
    rig = F.build_kinematic_rig(tmp_path / "rig", start, zero_sign=zero_sign, ranges=ranges)
    rig.enable_all()
    summary = S.StartPoseMover(rig.owner, rig.arm_maps, S.builtin_start_pose(),
                               S.SceneChecker(TRIAL / "run/scene.xml", rig.arm_maps),
                               S.MoverConfig(execute=True, operator="sim"), clock=rig.clock).run()
    assert summary["aborted"] is None and summary["at_start_pose"]
    assert summary["target_ticks"]["left_arm_shoulder_lift"] == 909 and summary["target_ticks"]["left_arm_gripper"] == 1358
    jaw_legs = [leg for leg in summary["plan"]["legs"] if leg["phase"] == "jaw"]
    assert jaw_legs and all(leg["expected_s"] > leg["duration_s"] for leg in jaw_legs)     # contact-mode closes
    assert rig.owner.owner.state["stop_count"] == 0 and not rig.owner.faults
