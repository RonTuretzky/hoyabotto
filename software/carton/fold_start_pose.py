"""Move both arms to the fold policy's training start pose through the robot's sole owner (replaces hand posing).

The fold policy (carton/fold_policy_runner.py) only ever started from one pose: every one of the 640 recorded
demonstrations and every episode of the training dataset begins at the same 12 joint angles. This module reads that
pose (built in, or from a demonstration / the LeRobot training dataset), converts it to encoder ticks through the
arms' joint maps (the runner's ArmMap), plans a joint-space path from where the arms are holding now, checks the
whole path for collisions in the MuJoCo fold training scene (carton in its nominal spot, flaps up), and then moves
the arms there with the owner's ordinary blocking motion tool:

    robot_get_execution (read) -> plan (raise / pan+roll / descend / jaws) -> MuJoCo clearance check of every leg
        -> robot_move_joint_targets(arm, positions, duration_s, wait=true, replace=false), one arm and one leg at a time
        -> verify each leg (owner outcome, stop_count, positions) -> final residual per joint

Default is DRY-RUN: the owner is read and the plan printed, nothing is sent. Execution needs execute=True and the
operator's name, all twelve arm motors enabled and holding, the paddle-success-v1 owner and the collision scene.

The plan, per arm (the other arm holds still while one moves, so every checked configuration is the real one):
  1. raise:    shoulder_lift, elbow_flex, wrist_flex to the clearance pose (by default the start pose's own values:
               the start pose is the high tuck, grippers ~0.35 m above the table) with pan and roll unchanged;
  2. pan_roll: shoulder_pan and wrist_roll to the start pose;
  3. descend:  lift / elbow / wrist_flex from the clearance pose to the start pose (nothing with the default);
  4. jaw:      the gripper alone and blocking (the owner closes jaws 10 ticks per 1.5 s); a jaw that stops short
               is reported and never re-sent (the right jaw stalls in free air; a resent open trips the owner's
               no-progress guard).
Several joint orderings and both arm orders are tried per phase; the first collision-free one is used. Moves are
split into legs of <= 280 ticks per joint (the owner's own segment size) and run at <= `speed_ticks_s`.

Limits are the owner's and are only ever tightened: targets 40 ticks inside the saved range, travel 3..341 ticks per
joint per leg, at most the owner's 40-tick ramp step, clearance >= MIN_CLEARANCE_M. The mover never calls robot_stop
and never releases or enables a motor: STOP stays with the operator; an owner fault releases on its own. It ends
HOLDING; the owner's idle lease (120 s) then releases the arms unless the next command or an explicit enable comes.
"""
from __future__ import annotations

import glob
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

from carton.fold_policy_runner import (ARMS, OWNER_JOINTS, OWNER_PROFILE, OWNER_STEP_TICKS, OWNER_TARGET_MARGIN,
                                       POLICY_JOINTS, SUFFIXES, ArmMap, TrainingEnvelope, _ok, parse_owner_status)
from carton.servo.common import Refused

# The training start pose (radians, URDF so101_new_calib, policy order): demo.npz qpos[0] of every recorded
# demonstration (batch-220-01/02, 640 trials) and frame 0 of all 287 episodes of the both-shorts-220 dataset.
TRAINING_START_RAD = (-0.5457, -1.745372, 0.535927, 1.658081, 0.005755, -0.170001,
                      0.542371, -1.745372, 0.535925, 1.658081, 1.5, -0.17)
ARM_SUFFIXES = SUFFIXES[:5]
LIFT_GROUP = ("shoulder_lift", "elbow_flex", "wrist_flex")
PAN_GROUP = ("shoulder_pan", "wrist_roll")
ARM_TOLERANCE_TICKS = 30        # arrival tolerance per arm joint (~2.6 deg)
# A joint whose target sits at the edge of its commandable range can stop short of it on its MECHANICAL stop (the
# right wrist_flex does: target 3128, commandable max 3130, settles at 3086-3089 on 9 Oct). That is not drift, so a
# shortfall TOWARD the range edge of up to this much is accepted and reported, never resent. Any other deviation,
# or a shortfall away from the edge, still aborts. Not a tolerance relaxation: it only applies within EDGE_TICKS
# of the commandable limit, and the fold runner's own start check (+/-10 deg) is unaffected.
MECHANICAL_STOP_SHORT_TICKS = 60  # ~5.3 deg
EDGE_TICKS = 8                    # target this close to the commandable limit counts as "at the edge"
JAW_TOLERANCE_TICKS = 30        # jaw "near its meeting point" (the owner's own gripper settle tolerance)
LEG_MAX_TICKS = 280             # paddle_segments.PADDLE_SEGMENT_TICKS: the owner's own long-move piece
OWNER_MAX_TRAVEL = 341          # paddle_joint_executor.SEGMENT
OWNER_MIN_TRAVEL = 3            # paddle_joint_executor: each joint in a move travels 3..341 ticks
OWNER_RAMP_INTERVAL_S = 0.4     # paddle_joint_executor: one <= 40-tick goal write per >= 0.4 s
OWNER_JAW_CLOSE_STEP = 10       # paddle_joint_executor CONTACT_STEP: a closing jaw moves 10 ticks per step ...
OWNER_JAW_CLOSE_INTERVAL_S = 1.5  # ... one step per >= 1.5 s, alone and blocking
DEFAULT_SPEED_TICKS_S = 60.0    # ~5 deg/s; the owner's own ramp tops out at 40 ticks / 0.4 s = 100 ticks/s
MAX_SPEED_TICKS_S = 100.0
MAX_LEG_DURATION_S = 25.0       # robot_move_joint_targets schema
MIN_CLEARANCE_M = 0.010
DEFAULT_CLEARANCE_M = 0.015
SAMPLE_TICKS = 4                # collision samples along a leg: at most this many ticks apart on any joint


def _owner(arm, suffix):
    return f"{arm}_arm_{suffix}"


# --------------------------------------------------------------------------------------------- start pose
@dataclass
class StartPose:
    rad: tuple                    # 12 radians, policy order
    source: str
    episodes: int | None = None   # how many episodes agreed (dataset / demo sources)
    spread_rad: float | None = None

    def degrees(self):
        return {POLICY_JOINTS[i]: round(math.degrees(v), 3) for i, v in enumerate(self.rad)}


def builtin_start_pose() -> StartPose:
    return StartPose(tuple(TRAINING_START_RAD), "built-in (demo.npz qpos[0] of the 220 mm demonstrations)")


def start_pose_from_demo(path) -> StartPose:
    """qpos[0] of a recorded demonstration (a trial dir or its demo.npz); robot joints come first in the fold scene."""
    path = Path(path)
    npz = path / "demo.npz" if path.is_dir() else path
    demo = np.load(npz)
    q = np.asarray(demo["qpos"][0][:12], dtype=float)
    if "ctrl" in demo.files and np.abs(np.asarray(demo["ctrl"][0][:12]) - q).max() > math.radians(0.5):
        raise Refused(f"{npz}: qpos[0] and ctrl[0] disagree; not a demonstration that starts at rest")
    return StartPose(tuple(float(v) for v in q), f"demo {npz}", 1, 0.0)


def start_pose_from_dataset(root) -> StartPose:
    """Frame 0 of every episode of a LeRobot v3 dataset (the one the checkpoint was trained on); all must agree."""
    import pyarrow.parquet as pq
    files = sorted(glob.glob(str(Path(root) / "data/chunk-*/*.parquet")))
    if not files:
        raise Refused(f"No LeRobot data files under {root}/data")
    starts = []
    for f in files:
        t = pq.read_table(f, columns=["observation.state", "frame_index"])
        first = np.asarray(t.column("frame_index")) == 0
        if first.any():
            starts.append(np.stack(t.column("observation.state").to_numpy(zero_copy_only=False))[first])
    s = np.concatenate(starts).astype(float)
    spread = float((s.max(0) - s.min(0)).max())
    if spread > math.radians(0.5):
        raise Refused(f"{root}: episodes start from different poses (spread {math.degrees(spread):.2f} deg); "
                      "the policy has no single training start pose")
    return StartPose(tuple(float(v) for v in s.mean(0)), f"dataset {root}", len(s), spread)


def envelope_check(pose: StartPose, envelope: TrainingEnvelope, eps_rad=math.radians(0.05)) -> dict:
    """The start pose must sit inside the checkpoint's own training state range (it is that range's corner)."""
    out = {}
    for i, name in enumerate(POLICY_JOINTS):
        lo, hi = float(envelope.state_min[i]), float(envelope.state_max[i])
        out[name] = {"start_deg": math.degrees(pose.rad[i]), "training_deg": [math.degrees(lo), math.degrees(hi)],
                     "inside": lo - eps_rad <= pose.rad[i] <= hi + eps_rad}
    return out


def pose_ticks(pose: StartPose, maps: dict[str, ArmMap]) -> dict[str, int]:
    """Training start pose in encoder ticks through each arm's joint map (ArmMap.rad_to_ticks)."""
    ticks = maps["left"].rad_to_ticks(pose.rad[:6]) + maps["right"].rad_to_ticks(pose.rad[6:])
    return {n: int(round(t)) for n, t in zip(OWNER_JOINTS, ticks)}


# ---------------------------------------------------------------------------------------- collision scene
class SceneChecker:
    """Clearance check in the MuJoCo fold training scene (tools/record_fold_demos.py scene.xml).

    A configuration is clear when no moving arm geom comes within `clearance_m` of the station (table, cart, mount,
    head neck, carton in its nominal pose with the flaps up) or of the other arm. Arm bases are fixed and skipped;
    pairs within one arm are not checked (the joint ranges cover those). Tick -> radian conversion uses the same
    ArmMap as the runner, so the check is in the policy's own joint convention.
    """

    def __init__(self, scene_xml, maps: dict[str, ArmMap], *, clearance_m=DEFAULT_CLEARANCE_M, carton=True):
        import mujoco
        if clearance_m < MIN_CLEARANCE_M:
            raise Refused(f"clearance_m must be >= {MIN_CLEARANCE_M} m; not relaxed")
        self.mujoco, self.maps, self.clearance_m, self.carton = mujoco, maps, float(clearance_m), bool(carton)
        self.scene = str(scene_xml)
        m = self.m = mujoco.MjModel.from_xml_path(self.scene)
        self.d = mujoco.MjData(m)
        self.adr = {}
        for owner_name, policy_name in zip(OWNER_JOINTS, POLICY_JOINTS):
            try:
                self.adr[owner_name] = int(m.jnt_qposadr[m.joint(policy_name).id])
            except KeyError as exc:
                raise Refused(f"{scene_xml}: no joint {policy_name}; not a fold training scene") from exc
        self.body_name = [m.body(i).name for i in range(m.nbody)]
        root = [int(m.body_rootid[i]) for i in range(m.nbody)]
        carton_roots = {i for i in range(m.nbody) if self.body_name[i] == "carton"}
        self.side = []
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            name = self.body_name[b]
            if not (m.geom_contype[g] or m.geom_conaffinity[g]):
                self.side.append(None)
                continue
            if name.startswith(("left_", "right_")) and not name.endswith("base_link"):
                arm = name.split("_", 1)[0]
                self.side.append(arm)
                # collision bits: env 1, left 2, right 4; left<->right, arm<->env, never within one arm
                m.geom_contype[g], m.geom_conaffinity[g] = (2, 1) if arm == "left" else (4, 1 | 2)
                m.geom_margin[g] = self.clearance_m
                m.geom_gap[g] = 0.0
            elif name.endswith("base_link"):
                self.side.append(None)
                m.geom_contype[g] = m.geom_conaffinity[g] = 0
            else:
                in_carton = root[b] in carton_roots or any(self._ancestor(b, c) for c in carton_roots)
                if in_carton and not self.carton:
                    self.side.append(None)
                    m.geom_contype[g] = m.geom_conaffinity[g] = 0
                    continue
                self.side.append("carton" if in_carton else "station")
                m.geom_contype[g], m.geom_conaffinity[g], m.geom_margin[g] = 1, 1, 0.0
        self.qpos0 = m.qpos0.copy()     # the scene's nominal carton pose, flaps at 0 (up)
        self.checks = 0

    def _ancestor(self, b, a):
        while b > 0:
            if b == a:
                return True
            b = int(self.m.body_parentid[b])
        return False

    def describe(self) -> dict:
        return {"scene": self.scene, "clearance_mm": self.clearance_m * 1000, "carton": "nominal" if self.carton else
                "absent (operator removed it)"}

    def _rad_unchecked(self, ticks):
        out = {}
        for arm in ARMS:
            for s in SUFFIXES:
                u = self.maps[arm].units[s]
                n = _owner(arm, s)
                out[n] = math.radians(u.model_sign * (float(ticks[n]) - u.model_zero_tick) * 360 / 4096)
        return out

    def hits(self, ticks: dict[str, int]) -> list[dict]:
        """Every arm geom pair closer than the clearance at this configuration (empty: clear)."""
        mujoco, m, d = self.mujoco, self.m, self.d
        d.qpos[:] = self.qpos0
        for n, v in self._rad_unchecked(ticks).items():
            d.qpos[self.adr[n]] = v
        before = int(d.warning[mujoco.mjtWarning.mjWARN_CONTACTFULL].number)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        self.checks += 1
        if int(d.warning[mujoco.mjtWarning.mjWARN_CONTACTFULL].number) != before:
            return [{"between": ["contact buffer full", "check incomplete"], "distance_mm": None}]
        found = {}
        for c in d.contact[:d.ncon]:
            a, b = self.side[c.geom1], self.side[c.geom2]
            if a is None or b is None or a == b or a not in ARMS and b not in ARMS or c.dist >= self.clearance_m:
                continue
            key = tuple(sorted((self.body_name[m.geom_bodyid[c.geom1]], self.body_name[m.geom_bodyid[c.geom2]])))
            found[key] = min(found.get(key, 1.0), float(c.dist))
        return [{"between": list(k), "distance_mm": round(v * 1000, 1)} for k, v in sorted(found.items(), key=lambda kv: kv[1])]

    def check_leg(self, start: dict[str, int], end: dict[str, int]) -> dict:
        """Sample the straight line in tick space (how the owner ramps all joints of one move together)."""
        span = max(abs(end[n] - start[n]) for n in OWNER_JOINTS)
        count = max(1, math.ceil(span / SAMPLE_TICKS))
        for i in range(count + 1):
            u = i / count
            q = {n: start[n] + (end[n] - start[n]) * u for n in OWNER_JOINTS}
            found = self.hits(q)
            if found:
                return {"clear": False, "samples": i + 1, "at_fraction": round(u, 3), "hits": found[:5],
                        "at_ticks": {n: int(round(v)) for n, v in q.items()}}
        return {"clear": True, "samples": count + 1}


# --------------------------------------------------------------------------------------------- planning
@dataclass
class Leg:
    phase: str
    arm: str
    targets: dict              # owner joint -> target tick (only joints that move >= 3 ticks)
    start: dict                # all 12 joints, planned, before the leg
    end: dict                  # all 12 joints, planned, after the leg
    duration_s: float
    check: dict | None = None

    def expected_s(self) -> float:
        """How long the owner takes: a closing jaw (falling ticks) runs in contact mode, 10 ticks per >= 1.5 s."""
        closing = [self.start[n] - t for n, t in self.targets.items() if n.endswith("gripper") and t < self.start[n]]
        if closing:
            return round(math.ceil(max(closing) / OWNER_JAW_CLOSE_STEP) * OWNER_JAW_CLOSE_INTERVAL_S, 1)
        return self.duration_s

    def summary(self):
        moves = {n.split("_arm_")[1]: [self.start[n], t] for n, t in self.targets.items()}
        return {"phase": self.phase, "arm": self.arm, "moves": moves, "duration_s": self.duration_s,
                "expected_s": self.expected_s(),
                "check": None if self.check is None else {k: v for k, v in self.check.items() if k != "at_ticks"}}


def leg_duration(travel: int, speed_ticks_s: float) -> float:
    steps = max(1, math.ceil(travel / OWNER_STEP_TICKS))
    return round(min(MAX_LEG_DURATION_S, max(OWNER_RAMP_INTERVAL_S * steps, travel / speed_ticks_s)), 2)


def commandable(ranges: dict, name: str) -> tuple[int, int]:
    lo, hi = ranges[name]
    return lo + OWNER_TARGET_MARGIN, hi - OWNER_TARGET_MARGIN


# Joint orderings tried per phase: each is a list of groups moved one after the other (joints in a group together).
ORDERINGS = {
    "raise": ([LIFT_GROUP], [("wrist_flex",), ("shoulder_lift", "elbow_flex")],
              [("elbow_flex",), ("shoulder_lift",), ("wrist_flex",)], [("shoulder_lift",), ("elbow_flex", "wrist_flex")],
              [("shoulder_lift",), ("elbow_flex",), ("wrist_flex",)], [("wrist_flex",), ("elbow_flex",), ("shoulder_lift",)]),
    "pan_roll": ([PAN_GROUP], [("wrist_roll",), ("shoulder_pan",)], [("shoulder_pan",), ("wrist_roll",)]),
    "jaw": ([("gripper",)],),
}
ORDERINGS["descend"] = ORDERINGS["raise"]


class Planner:
    def __init__(self, ranges: dict, checker: SceneChecker | None, *, speed_ticks_s=DEFAULT_SPEED_TICKS_S):
        if not 0 < speed_ticks_s <= MAX_SPEED_TICKS_S:
            raise Refused(f"speed_ticks_s must be in (0, {MAX_SPEED_TICKS_S}] (the owner's ramp); not raised")
        self.ranges, self.checker, self.speed = ranges, checker, float(speed_ticks_s)

    def _legs_for(self, phase, arm, joints, goal, state) -> list[Leg]:
        """Move `joints` of `arm` from state to goal in legs of <= LEG_MAX_TICKS per joint (straight line)."""
        names = [_owner(arm, s) for s in joints]
        travel = {n: goal[n] - state[n] for n in names if abs(goal[n] - state[n]) >= OWNER_MIN_TRAVEL}
        if not travel:
            return []
        # Jaws go as ONE blocking move: the API splits a long closure itself and stops at the first piece that
        # stops short, so a jaw that stalls is never commanded again by this tool.
        pieces = 1 if phase == "jaw" else max(1, math.ceil(max(abs(v) for v in travel.values()) / LEG_MAX_TICKS))
        legs, current = [], dict(state)
        for i in range(1, pieces + 1):
            targets = {}
            for n, v in travel.items():
                t = goal[n] if i == pieces else state[n] + round(v * i / pieces)
                lo, hi = commandable(self.ranges, n)
                targets[n] = min(hi, max(lo, t))
            targets = {n: t for n, t in targets.items() if abs(t - current[n]) >= OWNER_MIN_TRAVEL}
            if not targets:
                continue
            end = dict(current, **targets)
            span = max(abs(t - current[n]) for n, t in targets.items())
            legs.append(Leg(phase, arm, targets, current, end, leg_duration(span, self.speed)))
            current = end
        return legs

    def _arm_phase(self, phase, arm, goal, state):
        """First joint ordering whose legs are all clear; returns (legs, end state) or (None, tried)."""
        tried = []
        for ordering in ORDERINGS[phase]:
            legs, s = [], dict(state)
            ok = True
            for group in ordering:
                for leg in self._legs_for(phase, arm, group, goal, s):
                    leg.check = self.checker.check_leg(leg.start, leg.end) if self.checker else None
                    legs.append(leg)
                    s = leg.end
                    if leg.check is not None and not leg.check["clear"]:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return legs, s
            tried.append({"ordering": [" ".join(g) for g in ordering], "blocked": legs[-1].summary()})
        return None, tried

    def plan(self, current: dict[str, int], target: dict[str, int], clearance: dict[str, int] | None = None) -> dict:
        """Phases raise -> pan_roll -> descend -> jaw, each arm in turn (both arm orders tried)."""
        problems = []
        for n, t in list(target.items()) + list((clearance or {}).items()):
            lo, hi = commandable(self.ranges, n)
            if not lo <= t <= hi:
                problems.append(f"{n}: target {t} outside the commandable range {lo}..{hi} (saved range - 40)")
        if problems:
            raise Refused("Start pose not commandable: " + "; ".join(problems))
        clearance = dict(clearance or {n: target[n] for arm in ARMS for n in (_owner(arm, s) for s in LIFT_GROUP)})
        state = dict(current)
        report = {"legs": [], "blocked": None, "start_check": None}
        if self.checker is not None:
            found = self.checker.hits(state)
            report["start_check"] = {"clear": not found, "hits": found[:5]}
            if found:
                report["blocked"] = {"phase": "start", "reason": "the arms' present configuration is already inside "
                                     "the clearance of the station, the carton or the other arm", "hits": found[:5]}
                return report
        goals = {"raise": clearance, "pan_roll": target, "descend": target, "jaw": target}
        for phase in ("raise", "pan_roll", "descend", "jaw"):
            goal = dict(state, **{n: v for n, v in goals[phase].items()})
            done = None
            attempts = []
            for order in (ARMS, ARMS[::-1]):
                s, legs = dict(state), []
                for arm in order:
                    arm_legs, out = self._arm_phase(phase, arm, goal, s)
                    if arm_legs is None:
                        attempts.append({"arm_order": list(order), "arm": arm, "tried": out})
                        legs = None
                        break
                    legs += arm_legs
                    s = out
                if legs is not None:
                    done = (legs, s)
                    break
            if done is None:
                report["blocked"] = {"phase": phase, "attempts": attempts}
                return report
            report["legs"] += done[0]
            state = done[1]
        report["end"] = state
        return report


# --------------------------------------------------------------------------------------------- execution
class Abort(RuntimeError):
    pass


@dataclass
class MoverConfig:
    execute: bool = False
    operator: str | None = None
    speed_ticks_s: float = DEFAULT_SPEED_TICKS_S
    arm_tolerance_ticks: int = ARM_TOLERANCE_TICKS
    jaw_tolerance_ticks: int = JAW_TOLERANCE_TICKS
    move_jaws: bool = True

    def validate(self):
        if self.execute and not (self.operator or "").strip():
            raise Refused("--execute needs --operator (the person holding STOP)")
        if not 0 < self.arm_tolerance_ticks <= ARM_TOLERANCE_TICKS or not 0 < self.jaw_tolerance_ticks <= JAW_TOLERANCE_TICKS:
            raise Refused("Arrival tolerances may only be tightened")
        if not 0 < self.speed_ticks_s <= MAX_SPEED_TICKS_S:
            raise Refused(f"speed_ticks_s must be in (0, {MAX_SPEED_TICKS_S}]")


class StartPoseMover:
    def __init__(self, robot, maps: dict[str, ArmMap], pose: StartPose, checker: SceneChecker | None,
                 config: MoverConfig, *, clock: Callable[[], float] = time.time,
                 stop_requested: Callable[[], bool] | None = None, log: Callable[[str], None] | None = None,
                 envelope: TrainingEnvelope | None = None):
        config.validate()
        if set(maps) != set(ARMS):
            raise Refused("Both arms' joint maps are required")
        self.robot, self.maps, self.pose, self.checker, self.config = robot, maps, pose, checker, config
        self.clock = clock
        self.stop_requested = stop_requested or (lambda: False)
        self.log = log or (lambda line: None)
        self.envelope = envelope
        self.calls: list[str] = []
        self.initial = None

    # ---------------------------------------------------------------- owner access
    def call(self, name, args):
        self.calls.append(name)
        return self.robot.call(name, args)

    def _stopped_short_at_edge(self, name, measured, target):
        """True when `target` is within EDGE_TICKS of the joint's commandable limit and `measured` fell short of it
        TOWARD the inside of the range by at most MECHANICAL_STOP_SHORT_TICKS: the servo met its mechanical stop."""
        ranges = {n: r for m in self.maps.values() for n, r in m.ranges().items()}
        lo, hi = commandable(ranges, name)
        if abs(target - hi) <= EDGE_TICKS:
            return 0 < target - measured <= MECHANICAL_STOP_SHORT_TICKS
        if abs(target - lo) <= EDGE_TICKS:
            return 0 < measured - target <= MECHANICAL_STOP_SHORT_TICKS
        return False

    def snapshot(self):
        sent = self.clock()
        return parse_owner_status(_ok(self.call("robot_get_execution", {}), "robot_get_execution"),
                                  local_sent=sent, local_received=self.clock())

    def check_owner(self, snap):
        if not snap.ok or snap.release_errors:
            raise Abort(f"Owner not healthy: {snap.release_errors or snap.last_stop}")
        if snap.owner_started != self.initial.owner_started:
            raise Abort("Owner restarted")
        if snap.stop_count != self.initial.stop_count:
            raise Abort(f"Owner STOP or fault (stop_count {self.initial.stop_count} -> {snap.stop_count}): "
                        f"{snap.last_stop}; nothing is retried or re-enabled")
        if snap.phase not in ("idle", "holding"):
            raise Abort(f"Owner phase {snap.phase}, expected a stationary hold between legs")
        bad = {n: snap.status[n] for n in OWNER_JOINTS if snap.status[n] != 0}
        if bad:
            raise Abort(f"Motor status faults: {bad}")
        off = [n for n in OWNER_JOINTS if n not in snap.enabled or snap.torque[n] != 1]
        if off:
            raise Abort(f"Arm motors no longer enabled/holding: {off}")

    # ---------------------------------------------------------------- report helpers
    def table(self, ticks, target, ranges):
        rows = []
        for n in OWNER_JOINTS:
            lo, hi = commandable(ranges, n)
            rows.append({"joint": n, "current": ticks[n], "target": target[n], "delta": target[n] - ticks[n],
                         "commandable": [lo, hi]})
        return rows

    def print_table(self, rows, title):
        self.log(title)
        self.log(f"  {'joint':28s} {'current':>8s} {'target':>8s} {'delta':>7s}  commandable")
        for r in rows:
            self.log(f"  {r['joint']:28s} {r['current']:8d} {r['target']:8d} {r['delta']:+7d}  {r['commandable'][0]}..{r['commandable'][1]}")

    # ---------------------------------------------------------------- run
    def preflight(self):
        cfg = self.config
        snap = self.snapshot()
        self.initial = snap
        if not snap.ok:
            raise Refused(f"Owner not healthy: {snap.last_stop}")
        missing = [n for n in OWNER_JOINTS if n not in snap.ranges]
        if missing:
            raise Refused(f"Owner reports no commandable range for {missing}")
        for arm, m in self.maps.items():
            for name, rng in m.ranges().items():
                if tuple(snap.ranges[name]) != tuple(rng):
                    raise Refused(f"{name}: owner range {snap.ranges[name]} differs from the joint map's calibration "
                                  f"{rng}; the saved calibration changed")
        target = pose_ticks(self.pose, self.maps)
        report = {"execute": cfg.execute, "operator": cfg.operator, "pose_source": self.pose.source,
                  "pose_episodes": self.pose.episodes, "target_rad": list(self.pose.rad),
                  "target_deg": self.pose.degrees(), "target_ticks": target,
                  "joint_maps": {arm: m.config_path for arm, m in self.maps.items()},
                  "owner": snap.summary(), "collision_check": self.checker.describe() if self.checker else None}
        if self.envelope is not None:
            env = envelope_check(self.pose, self.envelope)
            report["checkpoint_envelope"] = env
            outside = [k for k, v in env.items() if not v["inside"]]
            if outside:
                raise Refused(f"The start pose lies outside the checkpoint's training state range: {outside}")
        if cfg.execute:
            if snap.profile != OWNER_PROFILE:
                raise Refused(f"Only the {OWNER_PROFILE} owner is supported; this owner is {snap.profile}")
            off = [n for n in OWNER_JOINTS if n not in snap.enabled or snap.torque[n] != 1]
            if off:
                raise Refused("Both arms must be enabled and holding first (robot_set_motor_enable holds them in "
                              f"place; the operator does this, supported): {off}")
            if snap.phase not in ("idle", "holding"):
                raise Refused(f"Owner is {snap.phase}; start from a stationary hold")
            if self.checker is None:
                raise Refused("No collision scene: execution needs the MuJoCo fold training scene (--scene)")
        return snap, target, report

    def run(self) -> dict:
        summary: dict[str, Any] = {"execute": self.config.execute, "aborted": None, "legs_sent": 0,
                                   "at_start_pose": False}
        try:
            snap, target, report = self.preflight()
            summary.update(report)
            self.print_table(self.table(snap.ticks, target, snap.ranges),
                             f"Training start pose ({self.pose.source}); current vs target ticks:")
            planner = Planner(snap.ranges, self.checker, speed_ticks_s=self.config.speed_ticks_s)
            if not self.config.move_jaws:
                target = dict(target, **{_owner(a, "gripper"): snap.ticks[_owner(a, "gripper")] for a in ARMS})
            plan = planner.plan(snap.ticks, target)
            legs: list[Leg] = plan["legs"]
            summary["plan"] = {"start_check": plan["start_check"], "blocked": plan["blocked"],
                               "legs": [leg.summary() for leg in legs],
                               "total_duration_s": round(sum(leg.expected_s() for leg in legs), 1),
                               "collision_checked": self.checker is not None}
            self.print_plan(summary["plan"])
            if plan["blocked"] is not None:
                raise Refused(f"No collision-free sequence: blocked in phase {plan['blocked']['phase']}")
            if not self.config.execute:
                summary["end"] = "dry-run (nothing sent)"
                return summary
            self.execute(legs, target, summary)
        except Refused as exc:
            summary["aborted"] = f"refused before motion: {exc}"
        except Abort as exc:
            summary["aborted"] = str(exc)
        except KeyboardInterrupt:
            summary["aborted"] = "KeyboardInterrupt"
        finally:
            summary["robot_stop_called"] = "robot_stop" in self.calls
            summary["tool_calls"] = {k: self.calls.count(k) for k in sorted(set(self.calls))}
        if summary["aborted"]:
            self.log(f"ABORTED: {summary['aborted']}")
            if summary["legs_sent"]:
                self.log("Nothing was released by this tool; the arms hold where they stopped (or the owner released "
                         "them on a fault/STOP). Support them before robot_stop.")
        return summary

    def print_plan(self, plan):
        if plan["start_check"] is not None:
            self.log(f"Present configuration clear of station/carton/other arm: {plan['start_check']['clear']}")
        self.log(f"Planned sequence ({len(plan['legs'])} legs, ~{plan['total_duration_s']} s of motion"
                 f"{'' if plan['collision_checked'] else ', NOT collision-checked: no scene'}):")
        for i, leg in enumerate(plan["legs"], 1):
            moves = ", ".join(f"{j} {a}->{b}" for j, (a, b) in leg["moves"].items())
            check = leg["check"]
            status = "unchecked" if check is None else ("clear" if check["clear"] else f"COLLIDES {check['hits'][:2]}")
            self.log(f"  {i:2d}. {leg['phase']:8s} {leg['arm']:5s} {moves}  (~{leg['expected_s']} s, {status})")
        if plan["blocked"] is not None:
            self.log(f"BLOCKED: {json.dumps(plan['blocked'])[:1500]}")

    def execute(self, legs: list[Leg], target, summary):
        cfg = self.config
        jaw_notes = {}
        summary["legs"] = []
        for i, leg in enumerate(legs, 1):
            if self.stop_requested():
                raise Abort("Stop requested by the operator (nothing further is sent; the arms hold)")
            snap = self.snapshot()
            self.check_owner(snap)
            drift = {n: snap.ticks[n] - leg.start[n] for n in OWNER_JOINTS
                     if not n.endswith("gripper") and abs(snap.ticks[n] - leg.start[n]) > cfg.arm_tolerance_ticks
                     and not self._stopped_short_at_edge(n, snap.ticks[n], leg.start[n])}
            if drift:
                raise Abort(f"Leg {i}: the arms are not where the plan expects (ticks off): {drift}")
            actual_start = dict(snap.ticks)
            positions = {n: t for n, t in leg.targets.items() if abs(t - actual_start[n]) >= OWNER_MIN_TRAVEL}
            if not positions:
                summary["legs"].append({"leg": i, "skipped": "already there"})
                continue
            if leg.phase != "jaw" and any(abs(t - actual_start[n]) > OWNER_MAX_TRAVEL for n, t in positions.items()):
                raise Abort(f"Leg {i}: a joint would travel more than {OWNER_MAX_TRAVEL} ticks")
            end = dict(actual_start, **positions)
            check = self.checker.check_leg(actual_start, end)
            if not check["clear"]:
                raise Abort(f"Leg {i}: the path from the measured position is not clear: {check['hits'][:3]}")
            self.log(f"Leg {i}/{len(legs)} {leg.phase} {leg.arm}: {positions} over {leg.duration_s} s")
            payload = self.call("robot_move_joint_targets", {"arm": leg.arm, "positions": positions,
                                                              "duration_s": leg.duration_s, "wait": True,
                                                              "replace": False})
            summary["legs_sent"] += 1
            result = payload.get("result") if isinstance(payload, dict) else None
            record = {"leg": i, "phase": leg.phase, "arm": leg.arm, "positions": positions,
                      "ok": isinstance(payload, dict) and payload.get("ok") is True,
                      "result": {k: (result or {}).get(k) for k in ("accepted", "completed", "no_op", "closure_outcome",
                                                                    "endpoint_reached", "settle_residual_ticks",
                                                                    "readbacks", "error", "contact")}}
            summary["legs"].append(record)
            if not record["ok"] or not isinstance(result, dict) or result.get("accepted") is not True:
                raise Abort(f"Leg {i}: the owner refused or failed the move: {str(payload)[:500]}")
            if leg.phase == "jaw":
                if result.get("completed") is not True and not result.get("no_op"):
                    if result.get("holding") and result.get("closure_outcome") in (
                            "settled_short", "stationary_closure_unverified", "contact_halt"):
                        jaw_notes[leg.arm] = f"stopped short ({result.get('closure_outcome')}); not re-sent"
                    else:
                        raise Abort(f"Leg {i}: jaw move did not complete: {str(result)[:400]}")
            elif result.get("completed") is not True and not result.get("no_op"):
                raise Abort(f"Leg {i}: the move ended {result.get('closure_outcome')} (holding, endpoint not "
                            f"reached); not retried: {str(result)[:400]}")
        final = self.snapshot()
        self.check_owner(final)
        residual = {n: final.ticks[n] - target[n] for n in OWNER_JOINTS}
        short = {n: residual[n] for n in OWNER_JOINTS if not n.endswith("gripper")
                 and abs(residual[n]) > cfg.arm_tolerance_ticks and self._stopped_short_at_edge(n, final.ticks[n], target[n])}
        arms_ok = all(abs(residual[n]) <= cfg.arm_tolerance_ticks or n in short
                      for n in OWNER_JOINTS if not n.endswith("gripper"))
        summary["stopped_short_at_mechanical_stop"] = short
        jaws = {}
        for arm in ARMS:
            n = _owner(arm, "gripper")
            ok = abs(residual[n]) <= cfg.jaw_tolerance_ticks
            jaws[arm] = {"position": final.ticks[n], "target": target[n], "residual": residual[n], "ok": ok,
                         "note": jaw_notes.get(arm) or (None if ok else ("jaw not moved (--no-jaws)" if not cfg.move_jaws
                                                                        else "outside tolerance; not re-sent"))}
        summary["residual_ticks"] = residual
        summary["residual_deg"] = {n: round(v * 360 / 4096, 2) for n, v in residual.items()}
        summary["jaws"] = jaws
        summary["arms_at_start_pose"] = arms_ok
        summary["at_start_pose"] = arms_ok and all(j["ok"] for j in jaws.values())
        summary["lease_remaining_s"] = final.lease_remaining
        summary["end"] = "holding"
        if not arms_ok:
            raise Abort("The arms finished outside the arrival tolerance: " + json.dumps(
                {n: v for n, v in residual.items() if not n.endswith("gripper") and abs(v) > cfg.arm_tolerance_ticks}))
        self.log("at training start pose" if summary["at_start_pose"] else
                 "arms at training start pose; jaws: " + json.dumps({a: j["note"] for a, j in jaws.items() if not j["ok"]}))
        self.log("Residual per joint (ticks): " + ", ".join(f"{n} {v:+d}" for n, v in residual.items()))
        self.log(f"The arms are HOLDING. The owner's idle lease releases them in ~{final.lease_remaining} s unless the "
                 "fold run (or an explicit enable) follows.")


# ------------------------------------------------------------------------------------------ simulation aids
def random_safe_start(checker: SceneChecker, maps: dict[str, ArmMap], ranges: dict, rng: np.random.Generator,
                      target_rad=TRAINING_START_RAD, tries=500) -> dict[str, int]:
    """A random holding pose that is clear in the scene and commandable (simulation and tests only)."""
    deg = np.degrees(np.asarray(target_rad))
    for _ in range(tries):
        rad = []
        for k, arm in enumerate(ARMS):
            o = 6 * k
            rad += [math.radians(deg[o] + rng.uniform(-25, 25)), math.radians(rng.uniform(-98, -20)),
                    math.radians(rng.uniform(-20, 60)), math.radians(rng.uniform(40, 94)),
                    math.radians(deg[o + 4] + rng.uniform(-40, 40)), math.radians(rng.uniform(-9, 25))]
        ticks = maps["left"].rad_to_ticks(rad[:6]) + maps["right"].rad_to_ticks(rad[6:])
        q = {n: int(round(t)) for n, t in zip(OWNER_JOINTS, ticks)}
        if any(not commandable(ranges, n)[0] <= v <= commandable(ranges, n)[1] for n, v in q.items()):
            continue
        if not checker.hits(q):
            return q
    raise RuntimeError("no clear random start found")


def place_sim_arms(rig, ticks: dict[str, int], settle_s=0.5):
    """Put the simulated arms (released, before the first enable) at `ticks`: KinematicPlant or MujocoFoldPlant."""
    plant = rig.plant
    if hasattr(plant, "q"):
        for n, v in ticks.items():
            plant.q[n] = plant.goal[n] = float(v)
    else:
        import mujoco
        for n, v in ticks.items():
            i = plant.index[n]
            rad = plant._rad(i, v)
            plant.d.qpos[plant.adr[i]] = rad
            plant.d.qvel[plant.dof[i]] = 0.0
            plant.d.ctrl[plant.act[i]] = plant.target[i] = plant.ramp_from[i] = rad
        mujoco.mj_forward(plant.m, plant.d)
    for n in ticks:
        rig.owner.bus.r[n]["Goal_Position"] = int(ticks[n])
    rig.sleep(settle_s)
