"""Closed-loop runner for the simulation-trained short-flap fold policy, through the robot's existing sole owner.

The policy (LeRobot ACT, tools/record_fold_demos.py -> fold_demos_to_lerobot.py) was trained ONLY in MuJoCo with
simulated cameras. It reads 12 joint angles in radians (URDF so101_new_calib convention, left arm then right arm,
shoulder_pan .. gripper) plus the RGB images its checkpoint names, and returns 12 absolute joint targets in the
same units at 10 Hz. The demo checkpoint (220 mm robot-model station) reads `front` (head OAK) and `left_wrist`/
`right_wrist`; the first policy read the simulated `top`/`front` cameras. The robot's owner speaks raw encoder ticks. This module is the adapter between the two:

    owner rows (ticks) --per-arm measured zero/sign--> policy radians --ACT--> target radians
        --training envelope, commandable range, per-tick step clamp, 3-tick rule, gripper hold--> target ticks
        --> robot_move_joint_targets(wait=false, replace=true)   (or the owner's DirectJointClient on the robot Mac)

Default is DRY-RUN: everything is read, converted, inferred, clamped and logged, nothing is sent. Execution needs
RunnerConfig.execute=True (CLI --execute) and an owner with all twelve arm motors already enabled by the operator.

Refusals (raised as carton.servo.common.Refused before any motion):
- any arm joint's model zero/sign or the gripper mapping is unmeasured (nothing is inferred from range endpoints);
- the joint-map calibration digest or ranges differ from the saved calibration / the owner's ranges;
- (execute) the start state is outside the training state range, or the owner is not the paddle-success-v1 owner.

Aborts during a run (no retry; the runner HALTS, which holds every joint where it is and releases nothing):
- stop hook (operator), owner STOP or fault (stop_count changed / owner restarted / not ok), contact_halt,
- stale or non-advancing joint telemetry (watchdog) or camera frames, a policy error or non-finite action,
- a tick that overruns RunnerConfig.max_tick_s, a refused owner command.
The runner never calls robot_stop or releases torque itself: STOP stays the operator's, and an owner fault releases
on its own (gemma_hardware_owner.py release_all). Grippers are held by default (gripper_mode='hold'): the deployed
owner cannot stream a gripper closure, and opening one may drop a held flap.

Every tick is logged to <run_dir>/ticks.jsonl (owner snapshot summary, state ticks/radians, image hashes and ages,
raw policy action, every clamp and its reason, what was sent or would have been sent, latencies).
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol

import numpy as np

from carton.servo.common import Refused, atomic_json, digest
from farm.kinematics.units import JointUnits

ARMS = ("left", "right")
SUFFIXES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
OWNER_JOINTS = tuple(f"{arm}_arm_{s}" for arm in ARMS for s in SUFFIXES)   # policy order, owner names
POLICY_JOINTS = tuple(f"{arm}_{s}" for arm in ARMS for s in SUFFIXES)      # dataset / MuJoCo names
CAMERA_KEYS = ("front", "left_wrist", "right_wrist")      # robot-model policy: head OAK + both wrist cameras
LEGACY_CAMERA_KEYS = ("top", "front")                       # first policy: simulated overhead + front cameras
# The robot camera behind each policy key of the robot-model policy (same physical cameras as in the simulation).
DEFAULT_ROBOT_CAMERAS = {"front": "oak", "left_wrist": "left_wrist", "right_wrist": "right_wrist"}
TASK_TEXT = "fold both short carton flaps and hold them"
TICKS_PER_REV = 4096          # farm.kinematics.units.JointUnits convention (360/4096 degrees per tick)

# The deployed owner profile (qwen-bridge, paddle-success-v1). These are the owner's own numbers; the runner never
# asks for more and refuses configuration that would.
OWNER_PROFILE = "paddle-success-v1"
OWNER_STEP_TICKS = 40         # paddle_joint_executor.py STEP: max goal change per ramp write
OWNER_ENVELOPE_TICKS = 96     # paddle_joint_executor.py ENVELOPE: |present - goal| fault (releases all motors)
OWNER_TARGET_MARGIN = 40      # paddle_joint_executor.py MARGIN: targets 40 ticks inside the saved range
OWNER_MIN_TRAVEL = 3          # paddle_joint_executor.py start(): each joint in a move travels 3..341 ticks
OWNER_FRAME_AGE_S = 1.0       # gemma_robot_tools.cameras_strict: images older than 1 s are refused
OWNER_CONTACT_LOAD = 600      # paddle_joint_executor.py CONTACT_LOAD: |Present_Load| treated as contact
OWNER_CONTACT_LAG = 20        # paddle_joint_executor.py CONTACT_PUSH_TICKS: ... on a joint lagging this far
OWNER_MAX_DURATION_S = 25.0   # gemma_robot_tools schema / DirectJointClient.execute
GRIPPER_STREAM_STEP = 10      # gripper_mode 'stream': a closing jaw moves at most this per tick (pickup owner's step)


# --------------------------------------------------------------------------------------------- joint mapping
@dataclass(frozen=True)
class ArmMap:
    """One arm's measured tick <-> policy-radian mapping for all six joints (five positioning joints + jaw)."""
    arm: str
    units: dict            # suffix -> JointUnits (range from the saved calibration, measured zero/sign)
    calibration_file: str
    calibration_sha256: str
    config_path: str
    config_sha256: str
    evidence: dict

    def ranges(self) -> dict[str, tuple[int, int]]:
        return {f"{self.arm}_arm_{s}": (u.range_min, u.range_max) for s, u in self.units.items()}

    def ticks_to_rad(self, ticks: dict[str, int]) -> list[float]:
        out = []
        for s in SUFFIXES:
            name, u = f"{self.arm}_arm_{s}", self.units[s]
            q = ticks[name]
            if not u.range_min <= q <= u.range_max:
                raise Refused(f"{name}: encoder {q} outside saved calibration {u.range_min}..{u.range_max}")
            out.append(math.radians(u.ticks_to_model_degrees(q)))
        return out

    def rad_to_ticks(self, values) -> list[float]:
        """Unclamped float ticks (the same formula as JointUnits.model_degrees_to_ticks, without its range raise)."""
        return [self.units[s].model_zero_tick + self.units[s].model_sign * math.degrees(float(v)) * TICKS_PER_REV / 360
                for s, v in zip(SUFFIXES, values)]


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_arm_map(path, arm: str) -> ArmMap:
    """Load a per-arm joint map. Refuses with EVERY missing measurement listed; nothing is guessed.

    Accepted file (schema 1; the `joints` block is the same as farm.kinematics.lerobot.CalibratedArm's):
      {"schema": 1, "arm": "right", "calibration_file": ".../farm_xlerobot.json", "calibration_sha256": "<hex>",
       "joints": {"shoulder_pan": {"model_zero_tick": <tick>, "model_sign": 1|-1}, ... five joints},
       "gripper": {"model_zero_tick": <tick>, "model_sign": 1|-1},
       "evidence": {"joints": "<where the zero/sign was measured>", "gripper": "<where the jaw mapping was measured>"}}
    model_zero_tick is the encoder tick at which the URDF (so101_new_calib) joint angle is 0; model_sign is +1 when
    increasing ticks increase the URDF angle. The gripper uses the same linear form (jaw driven by the servo horn).
    """
    path = Path(path)
    try:
        raw_text = path.read_text()
        cfg = json.loads(raw_text)
    except (OSError, ValueError) as exc:
        raise Refused(f"{arm} joint map unreadable: {path}: {exc}") from exc
    missing = []
    if cfg.get("schema") != 1:
        missing.append("schema 1")
    if cfg.get("arm") != arm:
        missing.append(f"arm == {arm!r} (file says {cfg.get('arm')!r})")
    if cfg.get("mapping") == "feetech_degrees_v1" and cfg.get("mapping_validated") is not True:
        missing.append("feetech_degrees_v1 is an unvalidated LeRobot midpoint candidate, not a measured zero/sign")
    joints = cfg.get("joints") or {}
    for s in SUFFIXES[:5]:
        j = joints.get(s) or {}
        for key in ("model_zero_tick", "model_sign"):
            if j.get(key) is None:
                missing.append(f"joints.{s}.{key}")
    g = cfg.get("gripper") or {}
    for key in ("model_zero_tick", "model_sign"):
        if g.get(key) is None:
            missing.append(f"gripper.{key}")
    evidence = cfg.get("evidence") or {}
    for key in ("joints", "gripper"):
        if not isinstance(evidence.get(key), str) or not evidence[key].strip():
            missing.append(f"evidence.{key} (reference to the physical measurement)")
    cal_file = cfg.get("calibration_file")
    if not isinstance(cal_file, str):
        missing.append("calibration_file")
    if missing:
        raise Refused(f"{arm} arm joint map {path} is not measured: missing " + "; ".join(missing))
    cal_path = Path(cal_file)
    if not cal_path.is_absolute():
        cal_path = (path.parent / cal_path).resolve()
    try:
        cal_sha = _sha256_file(cal_path)
        cal = json.loads(cal_path.read_text())
    except (OSError, ValueError) as exc:
        raise Refused(f"{arm} saved calibration unreadable: {cal_path}: {exc}") from exc
    if cfg.get("calibration_sha256") != cal_sha:
        raise Refused(f"{arm} joint map was measured against another calibration "
                      f"({cfg.get('calibration_sha256')} != current {cal_sha}); re-measure after any recalibration")
    units = {}
    for s in SUFFIXES:
        name = f"{arm}_arm_{s}"
        spec = g if s == "gripper" else joints[s]
        try:
            units[s] = JointUnits(int(cal[name]["range_min"]), int(cal[name]["range_max"]),
                                  float(spec["model_zero_tick"]), spec["model_sign"], gripper=s == "gripper")
        except (KeyError, ValueError, TypeError) as exc:
            raise Refused(f"{name}: invalid calibration or joint map entry: {exc}") from exc
    return ArmMap(arm, units, str(cal_path), cal_sha, str(path), hashlib.sha256(raw_text.encode()).hexdigest(),
                  dict(evidence))


def load_arm_maps(paths: dict[str, str | Path]) -> dict[str, ArmMap]:
    problems, maps = [], {}
    for arm in ARMS:
        if arm not in paths:
            problems.append(f"{arm}: no joint map given")
            continue
        try:
            maps[arm] = load_arm_map(paths[arm], arm)
        except Refused as exc:
            problems.append(str(exc))
    if problems:
        raise Refused("Refusing to run the fold policy: " + " | ".join(problems))
    if maps["left"].calibration_sha256 != maps["right"].calibration_sha256:
        raise Refused("Left and right joint maps name different saved calibration files")
    return maps


# ----------------------------------------------------------------------------------------- training envelope
@dataclass
class TrainingEnvelope:
    """Per-joint statistics of the training data (radians), from the checkpoint's own normalizer."""
    state_min: np.ndarray
    state_max: np.ndarray
    state_mean: np.ndarray
    state_std: np.ndarray
    action_min: np.ndarray
    action_max: np.ndarray
    source: str = ""

    @classmethod
    def from_pretrained_dir(cls, pretrained_dir) -> "TrainingEnvelope":
        from safetensors.numpy import load_file
        files = sorted(Path(pretrained_dir).glob("policy_preprocessor_step_*_normalizer_processor.safetensors"))
        if not files:
            raise Refused(f"No normalizer statistics in {pretrained_dir}")
        d = load_file(str(files[0]))
        get = lambda k: np.asarray(d[k], dtype=np.float64).reshape(-1)
        return cls(get("observation.state.min"), get("observation.state.max"), get("observation.state.mean"),
                   get("observation.state.std"), get("action.min"), get("action.max"), str(files[0]))

    def degenerate(self, threshold=1e-3) -> list[int]:
        """State dims that were (numerically) constant in training; MEAN_STD normalization divides by std+1e-8."""
        return [i for i, s in enumerate(self.state_std) if s < threshold]


# ------------------------------------------------------------------------------------------- safety config
@dataclass
class SafetyConfig:
    step_ticks: int = OWNER_STEP_TICKS          # per-tick |target - measured| bound
    min_step_ticks: int = OWNER_MIN_TRAVEL      # joints closer than this are not commanded (owner: <=2 is refused/no-op)
    race_margin_ticks: int = 3                  # overshoot allowance around [measured, held goal] (see race guard)
    range_margin_ticks: int = OWNER_TARGET_MARGIN
    action_margin_rad: float = math.radians(5)  # allowed excursion beyond the training action range
    start_margin_rad: float = math.radians(10)  # allowed start-state excursion beyond the training state range
    degenerate_tolerance_rad: float = math.radians(5)
    joint_max_age_s: float = 0.5                # profile limits.watchdog_s
    frame_max_age_s: float = OWNER_FRAME_AGE_S
    max_stale_ticks: int = 2                    # consecutive ticks without new telemetry / a new frame
    max_tick_s: float = 0.3                     # one tick (read + frames + policy + send) may take at most this
    contact_load: int = OWNER_CONTACT_LOAD      # runner-side copy of the owner's contact rule (see check_owner)
    contact_lag_ticks: int = OWNER_CONTACT_LAG
    contact_ticks: int = 2

    def validate(self):
        if not 1 <= self.step_ticks <= OWNER_STEP_TICKS:
            raise Refused(f"step_ticks must be 1..{OWNER_STEP_TICKS} (owner STEP); not relaxed")
        if self.min_step_ticks < OWNER_MIN_TRAVEL or self.race_margin_ticks < 0:
            raise Refused(f"min_step_ticks must be >= {OWNER_MIN_TRAVEL} (owner 3-tick rule), race margin >= 0")
        if self.range_margin_ticks < OWNER_TARGET_MARGIN:
            raise Refused(f"range_margin_ticks must be >= {OWNER_TARGET_MARGIN} (owner MARGIN)")
        if not 0 < self.joint_max_age_s <= 1.0 or not 0 < self.frame_max_age_s <= OWNER_FRAME_AGE_S:
            raise Refused("Telemetry/frame age limits may only be tightened")
        if not 0 < self.max_tick_s <= 1.0 or self.max_stale_ticks < 1:
            raise Refused("Invalid tick watchdog")
        if not 0 < self.contact_load <= OWNER_CONTACT_LOAD or not 0 < self.contact_lag_ticks <= OWNER_CONTACT_LAG \
                or not 1 <= self.contact_ticks <= 2:
            raise Refused("The contact rule may only be tightened")


@dataclass
class RunnerConfig:
    execute: bool = False
    hz: float = 10.0
    max_steps: int = 750
    task: str = TASK_TEXT
    gripper_mode: str = "hold"                  # 'hold': grippers never move; 'follow': opening may be streamed;
                                                # 'stream': closing too, <= GRIPPER_STREAM_STEP per tick, only on a
                                                # transport that streams jaw closures (owner stream mode)
    holding_arms: tuple = ()                    # operator-declared: these grippers never open
    degenerate_state: str = "substitute"        # 'substitute' training mean (within tolerance) | 'refuse'
    abort_on_contact_halt: bool = True
    save_frames_every: int = 0
    duration_s: float = 0.4                     # owner duration per command (pickup ramp interval floor)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def validate(self):
        self.safety.validate()
        if self.gripper_mode not in ("hold", "follow", "stream"):
            raise Refused("gripper_mode must be 'hold', 'follow' or 'stream'")
        if set(self.holding_arms) - set(ARMS):
            raise Refused("holding_arms must name left/right")
        if self.degenerate_state not in ("substitute", "refuse"):
            raise Refused("degenerate_state must be 'substitute' or 'refuse'")
        if not 1 <= self.hz <= 30 or self.max_steps < 1 or not 0 < self.duration_s <= OWNER_MAX_DURATION_S:
            raise Refused("Invalid rate, step count or command duration")


# ------------------------------------------------------------------------------------------- owner transport
@dataclass
class OwnerSnapshot:
    ticks: dict
    captured_at: dict
    torque: dict
    status: dict
    load: dict
    goals: dict
    enabled: list
    ranges: dict
    owner_started: Any
    owner_time: float
    robot_now: float          # owner clock when the snapshot was served
    clock_offset_s: float     # robot clock - local clock (0 when they are the same host/fake)
    stop_count: int
    last_stop: Any
    phase: str
    ok: bool
    profile: str | None
    closure_outcome: Any
    completed: Any
    accepted: Any
    lease_remaining: Any
    raw_keys: list
    contact: Any = None
    release_errors: Any = None

    def summary(self) -> dict:
        stamps = [self.captured_at[n] for n in OWNER_JOINTS if n in self.captured_at]
        return {"owner_started": self.owner_started, "owner_time": self.owner_time, "phase": self.phase,
                "ok": self.ok, "stop_count": self.stop_count, "profile": self.profile,
                "closure_outcome": self.closure_outcome, "completed": self.completed, "accepted": self.accepted,
                "lease_remaining": self.lease_remaining, "enabled_arm_motors": sorted(set(self.enabled) & set(OWNER_JOINTS)),
                "telemetry_oldest_age_s": (self.robot_now - min(stamps)) if stamps else None,
                "telemetry_skew_s": (max(stamps) - min(stamps)) if stamps else None,
                "clock_offset_s": self.clock_offset_s}


def parse_owner_status(s: dict, *, local_sent: float, local_received: float) -> OwnerSnapshot:
    """Owner status as status.json / robot_get_execution serve it (gemma_hardware_owner.py HardwareOwner.state)."""
    if not isinstance(s, dict) or s.get("hardware_server") is not True or s.get("control_mode") != "direct_joint":
        raise Refused("Owner status is not the direct-joint sole owner")
    rows = s.get("rows") or {}
    missing = [n for n in OWNER_JOINTS if n not in rows]
    if missing:
        raise Refused(f"Owner telemetry lacks arm joints: {missing}")
    ticks, stamps, torque, status, load = {}, {}, {}, {}, {}
    for n in OWNER_JOINTS:
        r = rows[n]
        q = r.get("Present_Position")
        if type(q) is not int:
            raise Refused(f"{n}: non-integer encoder reading {q!r}")
        t = r.get("captured_at")
        if type(t) not in (int, float) or not math.isfinite(t):
            raise Refused(f"{n}: telemetry without capture time")
        ticks[n], stamps[n] = q, float(t)
        torque[n], status[n], load[n] = r.get("Torque_Enable"), r.get("Status"), r.get("Present_Load")
    owner_time = float(s.get("time"))
    age = s.get("status_age_s")
    robot_now = owner_time + (float(age) if type(age) in (int, float) else 0.0)
    offset = robot_now - (local_sent + local_received) / 2 if type(age) in (int, float) else 0.0
    ranges = {n: tuple(v) for n, v in (s.get("ranges") or {}).items() if n in OWNER_JOINTS}
    return OwnerSnapshot(ticks, stamps, torque, status, load, dict(s.get("goals") or {}), list(s.get("enabled_motors") or []),
                         ranges, s.get("started"), owner_time, robot_now, offset, int(s.get("stop_count") or 0),
                         s.get("last_stop"), str(s.get("phase")), s.get("ok") is True, s.get("execution_profile"),
                         s.get("closure_outcome"), s.get("completed"), s.get("accepted"), s.get("lease_remaining"),
                         sorted(s), s.get("contact"), s.get("release_errors"))


class OwnerTransport(Protocol):
    name: str
    def snapshot(self) -> OwnerSnapshot: ...
    def send(self, targets: dict[str, int], duration_s: float) -> list[dict]: ...
    def halt(self) -> dict: ...


def _ok(payload, tool):
    if not isinstance(payload, dict) or payload.get("ok") is not True or not isinstance(payload.get("result"), dict):
        raise Refused(f"{tool} refused: {str(payload)[:600]}")
    return payload["result"]


class ApiOwnerTransport:
    """The robot's HTTPS tool API (gemma_robot_tools.py POST /call) through a client with call(name, args).

    On the chat Mac that client is the pilot's chat_server.Robot (mTLS, not in this repo); tests use FakeOwner.
    robot_move_joint_targets takes ONE arm per call, so a tick sends the left arm, then the right arm, each with
    wait=false/replace=true: each call is accepted after one owner loop and replaces (halts) the previous motion,
    whose first <=40-tick ramp step has by then been written.
    """
    name = "api"

    def __init__(self, robot, *, clock=time.time):
        self.robot, self.clock = robot, clock
        self.calls: list[dict] = []

    def _call(self, name, args):
        t0 = self.clock()
        payload = self.robot.call(name, args)
        self.calls.append({"tool": name, "s": round(self.clock() - t0, 4)})
        return payload

    def snapshot(self) -> OwnerSnapshot:
        sent = self.clock()
        result = _ok(self._call("robot_get_execution", {}), "robot_get_execution")
        return parse_owner_status(result, local_sent=sent, local_received=self.clock())

    def send(self, targets, duration_s):
        acks = []
        for arm in ARMS:
            positions = {n: int(q) for n, q in targets.items() if n.startswith(f"{arm}_arm_")}
            if not positions:
                continue
            payload = self._call("robot_move_joint_targets", {"arm": arm, "positions": positions,
                                                               "duration_s": duration_s, "wait": False, "replace": True})
            result = _ok(payload, "robot_move_joint_targets")
            if result.get("accepted") is not True:
                raise Refused(f"Owner did not accept the {arm} target: {str(result)[:400]}")
            acks.append({"arm": arm, "command_id": result.get("command_id"), "no_op": result.get("no_op", False),
                         "completed": result.get("completed")})
        return acks

    def halt(self):
        return _ok(self._call("robot_halt_motion", {}), "robot_halt_motion")


class StreamOwnerTransport(ApiOwnerTransport):
    """The owner's stream mode (on with the pickup profile; qwen-bridge STREAM-MODE.md) through the API.

    One robot_stream_joint_targets call per tick names both arms, jaws included (a jaw moves at most 10 ticks per
    command, closing too). A refusal answers accepted=false and never releases motors; the runner aborts and holds.
    Halt is robot_hold_here: every joint held where it is now (also acknowledges a stream contact_halt).
    """
    name = "api-stream"
    streams_gripper_closure = True

    def preflight(self):
        caps = _ok(self._call("robot_get_capabilities", {}), "robot_get_capabilities")
        if caps.get("stream_mode") is not True:
            raise Refused("The owner is not in stream mode (it predates it: redeploy; see qwen-bridge STREAM-MODE.md)")
        return {"stream_mode": True}

    def send(self, targets, duration_s):
        if not targets:
            return []
        payload = self._call("robot_stream_joint_targets", {"positions": {n: int(q) for n, q in targets.items()}})
        result = _ok(payload, "robot_stream_joint_targets")
        if result.get("accepted") is not True:
            raise Refused(f"Owner did not accept the streamed targets: {str(result)[:400]}")
        return [{"arm": "both", "command_id": result.get("command_id"), "no_op": result.get("no_op", False),
                 **{k: result[k] for k in ("skipped_joints", "jaw_contact", "jaw_limited", "jaw_ignored_closing", "phase")
                    if result.get(k)}}]

    def halt(self):
        return _ok(self._call("robot_hold_here", {}), "robot_hold_here")


class DirectClientOwnerTransport:
    """The owner's own file client (qwen-bridge gemma_direct_client.DirectJointClient), on the robot Mac.

    Both arms go in one direct_joint command (the pickup executor accepts any enabled arm joints), which halves the
    per-tick owner waits compared with the HTTPS API. The client still validates every command against the owner.
    """
    name = "direct-client"

    def __init__(self, client, *, clock=time.time):
        self.client, self.clock = client, clock

    def snapshot(self):
        sent = self.clock()
        state = self.client.status()
        return parse_owner_status(state, local_sent=sent, local_received=self.clock())

    def send(self, targets, duration_s):
        if not targets:
            return []
        result = self.client.execute({n: int(q) for n, q in targets.items()}, duration_s, wait=False, replace=True)
        if result.get("accepted") is not True:
            raise Refused(f"Owner did not accept the target: {str(result)[:400]}")
        return [{"arm": "both", "command_id": result.get("command_id"), "completed": result.get("completed")}]

    def halt(self):
        return self.client.halt()


# ------------------------------------------------------------------------------------------------- cameras
@dataclass
class Frame:
    rgb: np.ndarray
    captured_at: float        # on the robot's clock (the API's own capture or receipt stamp)
    seq: Any
    source: str
    timestamp_basis: str = "capture"

    def sha256(self) -> str:
        return hashlib.sha256(np.ascontiguousarray(self.rgb).tobytes()).hexdigest()


class CameraSource(Protocol):
    def frames(self) -> dict[str, Frame]: ...


ROBOT_CAMERAS = ("oak", "phone", "left_wrist", "right_wrist")


class ApiCameras:
    """Policy camera keys -> robot cameras through robot_get_cameras. The mapping must be given explicitly.

    The robot-model policy's keys are the robot's own cameras (DEFAULT_ROBOT_CAMERAS), rendered in simulation from
    the model's poses and an assumed lens; the first policy's `top`/`front` match no real camera. The phone feed only
    carries receipt time (timestamp_basis 'receipt').
    """
    def __init__(self, robot, mapping: dict[str, str]):
        if not mapping or any(v not in ROBOT_CAMERAS for v in mapping.values()):
            raise Refused(f"Map each policy camera key to one of the robot cameras {ROBOT_CAMERAS}; got {mapping}")
        self.robot, self.mapping = robot, dict(mapping)
        self.keys = tuple(mapping)

    def frames(self):
        import cv2
        names = list(dict.fromkeys(self.mapping.values()))
        payload = self.robot.call("robot_get_cameras", {"cameras": names, "revive": False})
        result = _ok(payload, "robot_get_cameras")
        errors = result.get("camera_errors") or {}
        if errors:
            raise Refused(f"Camera(s) not fresh: {errors}")
        images = payload.get("images") or []
        if len(images) != len(names):
            raise Refused(f"Expected {len(names)} images, got {len(images)}")
        by_name = {}
        for name, image in zip(names, images):  # gemma_robot_tools.cameras() appends images in request order
            data = base64.b64decode(image["data_base64"])
            if hashlib.sha256(data).hexdigest() != image.get("sha256"):
                raise Refused(f"{name}: image hash mismatch")
            bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise Refused(f"{name}: undecodable image")
            stamp, basis = image.get("captured_at"), "capture"
            if stamp is None:
                stamp, basis = image.get("received_at"), "receipt"
            by_name[name] = Frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), float(stamp), image.get("seq"),
                                  str(image.get("camera_id") or name), basis)
        return {key: by_name[name] for key, name in self.mapping.items()}


# ------------------------------------------------------------------------------------------------- policy
def enable_temporal_ensembling(runner, coeff: float = 0.01):
    """ACT temporal ensembling as in tools/eval_fold_policy.py: one inference per tick, chunks averaged."""
    from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
    cfg = runner.policy.config
    cfg.temporal_ensemble_coeff, cfg.n_action_steps = coeff, 1
    runner.policy.temporal_ensembler = ACTTemporalEnsembler(coeff, cfg.chunk_size)
    return runner


def policy_camera_keys(policy) -> tuple[str, ...] | None:
    """The camera keys a checkpoint reads (observation.images.<key>), or None for a stand-in without a spec."""
    spec_fn = getattr(policy, "input_spec", None)
    return None if spec_fn is None else tuple(spec_fn().get("camera_names") or ())


def check_policy_spec(policy, camera_keys=CAMERA_KEYS):
    spec_fn = getattr(policy, "input_spec", None)
    if spec_fn is None:
        return None
    spec = spec_fn()
    cams = set(spec.get("camera_names") or [])
    if spec.get("state_dim") != 12 or spec.get("action_dim") != 12 or cams != set(camera_keys):
        raise Refused(f"Policy expects state {spec.get('state_dim')}, action {spec.get('action_dim')}, cameras {cams}; "
                      f"this runner provides 12/12 and {tuple(camera_keys)}")
    out = {k: (list(v) if isinstance(v, tuple) else v) for k, v in spec.items() if k != "cameras"}
    out["cameras"] = {k: list(v) for k, v in (spec.get("cameras") or {}).items()}
    return out


# --------------------------------------------------------------------------------------------------- runner
class Abort(RuntimeError):
    pass


def _clean(value):
    """JSON-safe copy (numpy scalars/arrays, tuples)."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, (np.floating, float)):
        v = float(value)
        return v if math.isfinite(v) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


class RunLog:
    def __init__(self, run_dir):
        self.dir = Path(run_dir)
        self.dir.mkdir(parents=True, exist_ok=False)
        self._ticks = (self.dir / "ticks.jsonl").open("a")

    def write_json(self, name, value):
        atomic_json(self.dir / name, _clean(value))

    def tick(self, record):
        self._ticks.write(json.dumps(_clean(record), allow_nan=False) + "\n")
        self._ticks.flush()

    def frame(self, k, key, frame: Frame):
        import cv2
        folder = self.dir / "frames"
        folder.mkdir(exist_ok=True)
        cv2.imwrite(str(folder / f"{k:05d}-{key}.jpg"), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))

    def close(self):
        self._ticks.close()


class FoldPolicyRunner:
    def __init__(self, policy, transport: OwnerTransport, cameras: CameraSource, arm_maps: dict[str, ArmMap],
                 config: RunnerConfig, run_dir, *, envelope: TrainingEnvelope | None = None,
                 clock: Callable[[], float] = time.time, sleep: Callable[[float], None] = time.sleep,
                 stop_requested: Callable[[], bool] | None = None, metadata: dict | None = None):
        config.validate()
        if config.gripper_mode == "stream" and not getattr(transport, "streams_gripper_closure", False):
            raise Refused("gripper_mode 'stream' needs a transport that streams jaw closures (owner stream mode); "
                          f"{getattr(transport, 'name', type(transport).__name__)} does not")
        if set(arm_maps) != set(ARMS):
            raise Refused("Both arms' measured joint maps are required")
        self.policy, self.transport, self.cameras = policy, transport, cameras
        self.camera_keys = tuple(getattr(cameras, "keys", None) or CAMERA_KEYS)
        self.maps, self.config, self.envelope = arm_maps, config, envelope
        self.clock, self.sleep = clock, sleep
        self.stop_requested = stop_requested or (lambda: False)
        self.log = RunLog(run_dir)
        self.metadata = metadata or {}
        self.initial: OwnerSnapshot | None = None
        self.sent_ids: set = set()
        self.commands_sent = 0
        self._last_stamp = None
        self._stale_joint_ticks = 0
        self._last_frames: dict[str, tuple] = {}
        self._stale_frame_ticks = dict.fromkeys(self.camera_keys, 0)
        self._loaded = dict.fromkeys(OWNER_JOINTS, 0)
        self._last_sent: dict[str, int] = {}

    # ---------------------------------------------------------------- conversions
    def state_rad(self, snap: OwnerSnapshot) -> np.ndarray:
        return np.asarray(self.maps["left"].ticks_to_rad(snap.ticks) + self.maps["right"].ticks_to_rad(snap.ticks))

    def condition_state(self, state: np.ndarray) -> tuple[np.ndarray, list[dict]]:
        """Training-constant state dims (std ~ 0) would be divided by ~1e-8: feed the training mean, within tolerance."""
        out, notes = state.astype(np.float32).copy(), []
        if self.envelope is None:
            return out, notes
        for i in self.envelope.degenerate():
            deviation = float(state[i] - self.envelope.state_mean[i])
            notes.append({"joint": OWNER_JOINTS[i], "measured_rad": float(state[i]),
                          "training_constant_rad": float(self.envelope.state_mean[i]), "deviation_rad": deviation})
            if self.config.degenerate_state == "refuse" or abs(deviation) > self.config.safety.degenerate_tolerance_rad:
                raise Abort(f"{OWNER_JOINTS[i]} reads {math.degrees(state[i]):.1f} deg but was constant at "
                            f"{math.degrees(self.envelope.state_mean[i]):.1f} deg in training")
            out[i] = self.envelope.state_mean[i]
        return out, notes

    def plan_targets(self, action: np.ndarray, snap: OwnerSnapshot) -> tuple[dict[str, int], dict[str, dict]]:
        """Policy radians -> owner ticks, with every bound applied and recorded."""
        sc = self.config.safety
        action = np.asarray(action, dtype=np.float64)
        if self.envelope is not None:
            lo = self.envelope.action_min - sc.action_margin_rad
            hi = self.envelope.action_max + sc.action_margin_rad
            clipped = np.clip(action, lo, hi)
        else:
            clipped = action
        raw_ticks = self.maps["left"].rad_to_ticks(clipped[:6]) + self.maps["right"].rad_to_ticks(clipped[6:])
        send, detail = {}, {}
        for i, name in enumerate(OWNER_JOINTS):
            arm = name.split("_arm_")[0]
            reasons = []
            if clipped[i] != action[i]:
                reasons.append("training_envelope")
            measured = snap.ticks[name]
            lo_s, hi_s = snap.ranges[name]
            lo_c, hi_c = lo_s + sc.range_margin_ticks, hi_s - sc.range_margin_ticks
            t = raw_ticks[i]
            if not lo_c <= t <= hi_c:
                t = min(hi_c, max(lo_c, t))
                reasons.append("commandable_range")
            if abs(t - measured) > sc.step_ticks:
                t = measured + math.copysign(sc.step_ticks, t - measured)
                reasons.append("step")
            target = int(round(t))
            command = True
            if not lo_c <= target <= hi_c:
                command = False
                reasons.append("outside_commandable_after_step")
            if name.endswith("gripper"):
                if self.config.gripper_mode == "hold":
                    command = False
                    reasons.append("gripper_hold_mode")
                elif arm in self.config.holding_arms:
                    command = False
                    reasons.append("gripper_holding_object")
                elif self.config.gripper_mode == "stream":
                    if abs(target - measured) > GRIPPER_STREAM_STEP:   # the stream owner's jaw limit, both ways
                        target = measured + int(math.copysign(GRIPPER_STREAM_STEP, target - measured))
                        reasons.append("gripper_stream_step")
                elif target < measured - (sc.min_step_ticks - 1):
                    command = False   # the pickup owner only closes a gripper alone and blocking (contact mode)
                    reasons.append("gripper_close_not_streamable")
            if command and abs(target - measured) < sc.min_step_ticks:
                command = False
                reasons.append("below_min_step")
            held = snap.goals.get(name, measured)
            if command and type(held) is int:
                # Race guard. Between this snapshot and the owner processing the command the joint keeps moving
                # from `measured` toward its held goal. If it then sits within 2 ticks of `target`, the owner
                # refuses the command (3-tick rule) and DirectJointClient answers a post-dispatch refusal with
                # STOP, which releases every motor. Only command targets clear of that whole interval.
                lo_p, hi_p = min(measured, held) - sc.race_margin_ticks, max(measured, held) + sc.race_margin_ticks
                if lo_p - sc.min_step_ticks < target < hi_p + sc.min_step_ticks:
                    command = False
                    reasons.append("race_guard_hold")
            if command:
                send[name] = target
            detail[name] = {"action_rad": float(action[i]), "raw_ticks": float(raw_ticks[i]), "measured": measured,
                            "held_goal": snap.goals.get(name), "target": target, "sent": command, "clamps": reasons}
        return send, detail

    # ---------------------------------------------------------------- checks
    def check_owner(self, snap: OwnerSnapshot, *, now: float):
        sc = self.config.safety
        if not snap.ok or snap.release_errors:
            raise Abort(f"Owner not healthy: {snap.release_errors or snap.last_stop}")
        if self.initial is not None:
            if snap.owner_started != self.initial.owner_started:
                raise Abort("Owner restarted during the run")
            if snap.stop_count != self.initial.stop_count:
                raise Abort(f"Owner STOP or fault during the run (no retry, motors stay as the owner left them): "
                            f"{snap.last_stop}")
        if snap.phase not in ("idle", "holding", "moving"):
            raise Abort(f"Owner phase {snap.phase}")
        bad = {n: snap.status[n] for n in OWNER_JOINTS if snap.status[n] != 0}
        if bad:
            raise Abort(f"Motor status faults: {bad}")
        robot_now = now + snap.clock_offset_s
        oldest = min(snap.captured_at[n] for n in OWNER_JOINTS)
        age = robot_now - oldest
        if not -0.05 <= age <= sc.joint_max_age_s:
            raise Abort(f"Joint telemetry watchdog: oldest arm row is {age:.3f} s old (limit {sc.joint_max_age_s} s)")
        newest = max(snap.captured_at[n] for n in OWNER_JOINTS)
        if self._last_stamp is not None and newest <= self._last_stamp:
            self._stale_joint_ticks += 1
            if self._stale_joint_ticks >= sc.max_stale_ticks:
                raise Abort(f"Joint telemetry has not advanced for {self._stale_joint_ticks} ticks")
        else:
            self._stale_joint_ticks = 0
        self._last_stamp = newest
        if self.config.execute:
            off = [n for n in OWNER_JOINTS if n not in snap.enabled or snap.torque[n] != 1]
            if off:
                raise Abort(f"Arm motors not enabled/holding: {off}")
            if (self.config.abort_on_contact_halt and snap.closure_outcome == "contact_halt"
                    and snap.completed in self.sent_ids):
                raise Abort(f"Owner contact_halt (load on a lagging joint): {snap.contact}")
            # The owner's contact guard counts loaded samples per motion (PaddleJointExecutor.loaded starts at 0 for
            # every command), a streamed motion is replaced after about one owner poll, and a contact_halt is
            # overwritten in status by the next command's start. So apply the same rule across ticks here: high
            # load on an arm joint whose held goal or last sent target is >= 20 ticks from where it is = contact.
            hits = {}
            for n in OWNER_JOINTS:
                if n.endswith("gripper"):
                    continue
                load, q = snap.load.get(n), snap.ticks[n]
                goal = snap.goals.get(n, q)
                lag = max(abs(goal - q) if type(goal) is int else 0, abs(self._last_sent.get(n, q) - q))
                pushing = type(load) in (int, float) and abs(load) >= sc.contact_load and lag >= sc.contact_lag_ticks
                self._loaded[n] = self._loaded[n] + 1 if pushing else 0
                if self._loaded[n] >= sc.contact_ticks:
                    hits[n] = {"load": load, "lag_ticks": lag, "position": q}
            if hits:
                raise Abort(f"Contact: high load on joints lagging their goal for {sc.contact_ticks} ticks: {hits}")

    def check_frames(self, frames: dict[str, Frame], snap: OwnerSnapshot, *, now: float) -> dict:
        sc = self.config.safety
        if set(frames) != set(self.camera_keys):
            raise Abort(f"Camera source returned {sorted(frames)}, need {self.camera_keys}")
        out = {}
        for key in self.camera_keys:
            f = frames[key]
            rgb = np.asarray(f.rgb)
            if rgb.ndim != 3 or rgb.shape[-1] != 3 or rgb.dtype != np.uint8:
                raise Abort(f"{key}: expected HxWx3 uint8, got {rgb.shape} {rgb.dtype}")
            age = now + snap.clock_offset_s - f.captured_at
            if not -0.05 <= age <= sc.frame_max_age_s:
                raise Abort(f"Camera watchdog: {key} frame is {age:.3f} s old (limit {sc.frame_max_age_s} s)")
            sha = f.sha256()
            identity = (f.seq, f.captured_at)
            previous = self._last_frames.get(key)
            if previous is not None and (identity == previous[0] or (f.seq is None and sha == previous[1])):
                self._stale_frame_ticks[key] += 1
                if self._stale_frame_ticks[key] >= sc.max_stale_ticks:
                    raise Abort(f"Camera {key} has not delivered a new frame for {self._stale_frame_ticks[key]} ticks")
            else:
                self._stale_frame_ticks[key] = 0
            self._last_frames[key] = (identity, sha)
            out[key] = {"sha256": sha, "seq": f.seq, "age_s": age, "source": f.source, "basis": f.timestamp_basis,
                        "shape": list(rgb.shape)}
        return out

    # ---------------------------------------------------------------- run
    def preflight(self) -> dict:
        cfg = self.config
        report = {"execute": cfg.execute, "transport": getattr(self.transport, "name", type(self.transport).__name__),
                  "warnings": []}
        report["policy_spec"] = check_policy_spec(self.policy, self.camera_keys)
        if hasattr(self.transport, "preflight"):
            report["transport_preflight"] = self.transport.preflight()
        snap = self.transport.snapshot()
        now = self.clock()
        if not snap.ok:
            raise Refused(f"Owner not healthy: {snap.last_stop}")
        missing = [n for n in OWNER_JOINTS if n not in snap.ranges]
        if missing:
            raise Refused(f"Owner reports no commandable range for {missing} (read-only or scope-reduced arm)")
        for arm, m in self.maps.items():
            for name, rng in m.ranges().items():
                if tuple(snap.ranges[name]) != tuple(rng):
                    raise Refused(f"{name}: owner range {snap.ranges[name]} differs from the joint map's calibration "
                                  f"{rng}; the saved calibration changed")
        if cfg.execute:
            if snap.profile != OWNER_PROFILE:
                raise Refused(f"Execution is only adapted to the {OWNER_PROFILE} owner; this owner is {snap.profile}")
            off = [n for n in OWNER_JOINTS if n not in snap.enabled]
            if off:
                raise Refused(f"Enable all twelve arm motors first (robot_set_motor_enable holds them in place): {off}")
            if snap.phase not in ("idle", "holding"):
                raise Refused(f"Owner is {snap.phase}; start from a stationary hold")
        elif snap.enabled:
            report["warnings"].append("Dry-run with motors enabled: the owner releases them when its idle lease "
                                      "(120 s) expires; dry-run normally runs with the arms released and supported")
        self.initial = snap
        self.check_owner(snap, now=now)
        state = self.state_rad(snap)
        report["start_state_rad"] = state
        if self.envelope is not None:
            lo = self.envelope.state_min - cfg.safety.start_margin_rad
            hi = self.envelope.state_max + cfg.safety.start_margin_rad
            outside = {OWNER_JOINTS[i]: {"measured_deg": math.degrees(state[i]),
                                         "training_deg": [math.degrees(self.envelope.state_min[i]),
                                                          math.degrees(self.envelope.state_max[i])]}
                       for i in range(12) if i not in self.envelope.degenerate() and not lo[i] <= state[i] <= hi[i]}
            report["start_outside_training"] = outside
            if outside:
                message = f"Start state outside the training range (+{math.degrees(cfg.safety.start_margin_rad):.0f} deg): {outside}"
                if cfg.execute:
                    raise Refused(message)
                report["warnings"].append(message)
            try:
                _, notes = self.condition_state(state)
                report["degenerate_state"] = notes
            except Abort as exc:
                if cfg.execute:
                    raise Refused(str(exc)) from exc
                report["warnings"].append(str(exc))
        frames = self.cameras.frames()
        report["frames"] = self.check_frames(frames, snap, now=self.clock())
        self._last_frames.clear()
        # The policy squeezes every frame to its trained H x W. A different aspect ratio (e.g. the OAK's 16:9 1080p
        # output against the 4:3 head camera it was rendered with) is a different field of view, not a resize.
        sizes = ((report["policy_spec"] or {}).get("cameras") or {})
        for key in self.camera_keys:
            h, w = sizes.get(f"observation.images.{key}") or (None, None)
            fh, fw = report["frames"][key]["shape"][:2]
            if h and abs(fw / fh - w / h) > 0.02 * (w / h):
                report["warnings"].append(f"camera_aspect: {key} frames are {fw}x{fh} but the policy was trained on "
                                          f"{w}x{h}; its field of view differs from training (re-render or re-measure)")
        # Warm-up inference (the first pass on a fresh accelerator is slow); run() resets the policy afterwards.
        policy_state, _ = self.condition_state(state) if self.envelope is not None and not report["warnings"] else (
            state.astype(np.float32), None)
        t0 = self.clock()
        try:
            self.policy.act(policy_state, {key: frames[key].rgb for key in self.camera_keys}, cfg.task)
        except Exception as exc:  # noqa: BLE001
            raise Refused(f"Policy warm-up failed: {type(exc).__name__}: {exc}") from exc
        report["warmup_inference_s"] = self.clock() - t0
        return report

    def _halt(self, reason):
        if not self.config.execute or self.commands_sent == 0:
            return {"halt": "not needed (nothing was sent)"}
        try:
            return {"halt": self.transport.halt(), "reason": reason}
        except Exception as exc:  # noqa: BLE001 - report; STOP stays with the operator
            return {"halt_failed": f"{type(exc).__name__}: {exc}", "reason": reason,
                    "operator_action": "use robot_stop (releases all motors) or cut 12 V if the arms must not move"}

    def tick(self, k: int) -> dict:
        cfg = self.config
        t0 = self.clock()
        if self.stop_requested():
            raise Abort("Stop requested by the operator hook")
        snap = self.transport.snapshot()
        t_snap = self.clock()
        self.check_owner(snap, now=t_snap)
        frames = self.cameras.frames()
        t_frames = self.clock()
        frame_info = self.check_frames(frames, snap, now=t_frames)
        state = self.state_rad(snap)
        policy_state, notes = self.condition_state(state)
        try:
            action = self.policy.act(policy_state, {key: frames[key].rgb for key in self.camera_keys}, cfg.task)
        except Exception as exc:  # noqa: BLE001
            raise Abort(f"Policy error: {type(exc).__name__}: {exc}") from exc
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != (12,) or not np.isfinite(action).all():
            raise Abort(f"Policy returned shape {action.shape} or non-finite values")
        t_policy = self.clock()
        if self.stop_requested():
            raise Abort("Stop requested by the operator hook")
        send, detail = self.plan_targets(action, snap)
        acks = None
        if cfg.execute and send:
            try:
                acks = self.transport.send(send, cfg.duration_s)
            except Exception as exc:  # noqa: BLE001 - a refused command ends the run
                raise Abort(f"Owner refused the tick's targets: {type(exc).__name__}: {exc}") from exc
            self.commands_sent += 1
            self._last_sent.update(send)
            self.sent_ids.update(a.get("command_id") for a in acks if a.get("command_id") is not None)
        t_end = self.clock()
        if cfg.save_frames_every and k % cfg.save_frames_every == 0:
            for key in self.camera_keys:
                self.log.frame(k, key, frames[key])
        record = {"k": k, "t": t0, "owner": snap.summary(), "state_ticks": [snap.ticks[n] for n in OWNER_JOINTS],
                  "state_rad": state, "degenerate_state": notes, "frames": frame_info, "action_rad": action,
                  "joints": detail, "send": send, "mode": "execute" if cfg.execute else "dry-run",
                  "acks": acks, "latency_s": {"limit": cfg.safety.max_tick_s, "snapshot": t_snap - t0, "frames": t_frames - t_snap,
                                              "policy": t_policy - t_frames, "send": t_end - t_policy,
                                              "tick": t_end - t0}}
        self.log.tick(record)
        if t_end - t0 > cfg.safety.max_tick_s:
            raise Abort(f"Tick took {t_end - t0:.3f} s (limit {cfg.safety.max_tick_s} s)")
        return record

    def run(self) -> dict:
        cfg = self.config
        started = self.clock()
        summary = {"execute": cfg.execute, "steps": 0, "aborted": None, "metadata": self.metadata,
                   "simulation_trained_policy": True}
        try:
            report = self.preflight()
            self.log.write_json("preflight.json", report)
            if hasattr(self.policy, "reset"):
                self.policy.reset()
            period = 1.0 / cfg.hz
            next_t = self.clock()
            clamp_counts: dict[str, int] = {}
            for k in range(cfg.max_steps):
                record = self.tick(k)
                summary["steps"] = k + 1
                for name, d in record["joints"].items():
                    for reason in d["clamps"]:
                        clamp_counts[reason] = clamp_counts.get(reason, 0) + 1
                summary["clamp_counts"] = clamp_counts
                next_t += period
                wait = next_t - self.clock()
                if wait > 0:
                    self.sleep(wait)
                else:
                    next_t = self.clock()   # overrun: do not try to catch up
            summary["end"] = "max_steps"
        except Refused as exc:
            summary["aborted"] = f"refused before motion: {exc}"
        except Abort as exc:
            summary["aborted"] = str(exc)
        except KeyboardInterrupt:
            summary["aborted"] = "KeyboardInterrupt"
        finally:
            if summary["aborted"] is not None or summary.get("end"):
                summary["final"] = self._halt(summary["aborted"] or "run complete; holding")
            summary["commands_sent"] = self.commands_sent
            elapsed = self.clock() - started
            summary["elapsed_s"] = elapsed
            summary["effective_hz"] = summary["steps"] / elapsed if elapsed > 0 and summary["steps"] else None
            self.log.write_json("summary.json", summary)
            self.log.close()
        return summary


# ------------------------------------------------------------------------------------------------------ CLI
def _pairs(values, what):
    out = {}
    for v in values or []:
        if "=" not in v:
            raise SystemExit(f"--{what} expects key=value, got {v!r}")
        k, val = v.split("=", 1)
        out[k] = val
    return out


def build_policy(checkpoint, device="cpu", temporal_ensemble=0.01):
    from farm.learning.infer import PolicyRunner
    runner = PolicyRunner(checkpoint, device=device)
    if temporal_ensemble is not None:
        enable_temporal_ensembling(runner, temporal_ensemble)
    return runner


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Simulation: --transport sim-owner --sim-trial <fold-demos trial dir>. "
                                        "Robot (supervised, see docs/carton-fold-policy-robot.md): --transport api "
                                        "--pilot-root <pilot> (cameras default to front=oak left_wrist=left_wrist right_wrist=right_wrist for "
                                        "the robot-model policy); dry-run unless --execute.")
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--transport", choices=["sim-owner", "sim-owner-stream", "sim-direct", "api", "api-stream",
                                            "direct-client"], required=True,
                    help="api-stream / sim-owner-stream: the owner's stream mode (qwen-bridge STREAM-MODE.md); "
                         "sim-owner simulates the owner before stream mode")
    ap.add_argument("--joint-map", action="append", metavar="ARM=PATH", help="measured per-arm joint map (robot)")
    ap.add_argument("--camera", action="append", metavar="KEY=CAMERA", help="policy key -> robot camera, e.g. front=oak (robot; default for the robot-model policy)")
    ap.add_argument("--sim-trial", type=Path, help="fold-demos trial dir with run/scene.xml and demo.npz")
    ap.add_argument("--pilot-root", type=Path, help="chat pilot checkout with chat_server.Robot and .private/robot.json")
    ap.add_argument("--session", type=Path, help="robot Mac work/gemma-hardware-session (direct-client)")
    ap.add_argument("--calibration", type=Path, help="robot Mac saved calibration (direct-client)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--execute", action="store_true", help="send targets (default: dry-run, nothing sent)")
    ap.add_argument("--operator", help="name of the person holding STOP (required with --execute on the robot)")
    ap.add_argument("--gripper-mode", choices=["hold", "follow", "stream"], default="hold")
    ap.add_argument("--holding-arm", action="append", default=[], choices=list(ARMS))
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--max-tick-s", type=float, default=SafetyConfig.max_tick_s,
                    help="abort if one tick (read + frames + policy + send) takes longer than this; default 0.3 s. "
                         "Over a LAN two round trips alone can take 0.2-0.3 s, so a slower rate (--hz 5) may need up "
                         "to 1.0 s, the hard cap. The value is recorded in the run's metadata and ticks.jsonl.")
    ap.add_argument("--max-steps", type=int, default=750)
    ap.add_argument("--temporal-ensemble", type=float, default=0.01)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--save-frames-every", type=int, default=10)
    ap.add_argument("--owner-period-s", type=float, default=0.05, help="sim-owner: owner loop period")
    ap.add_argument("--rtt-s", type=float, default=0.0, help="sim-owner: API round trip per call")
    ap.add_argument("--policy-latency-s", type=float, help="sim: virtual inference time per tick (default: measured)")
    ap.add_argument("--parent-pid", type=int, help="stop (halt, hold) if this parent process exits, e.g. the chat server")
    args = ap.parse_args(argv)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    if args.parent_pid:
        orphaned = lambda: os.getppid() != args.parent_pid
        stopped = lambda: stop.is_set() or orphaned()
    else:
        stopped = stop.is_set
    config = RunnerConfig(execute=args.execute, hz=args.hz, max_steps=args.max_steps, gripper_mode=args.gripper_mode,
                          holding_arms=tuple(args.holding_arm), save_frames_every=args.save_frames_every,
                          safety=SafetyConfig(max_tick_s=args.max_tick_s))
    policy = build_policy(args.checkpoint, args.device, args.temporal_ensemble)
    envelope = TrainingEnvelope.from_pretrained_dir(policy.path)
    meta = {"checkpoint": policy.path, "transport": args.transport, "argv": sys.argv[1:] if argv is None else argv,
            "max_tick_s": args.max_tick_s, "hz": args.hz,
            "model_sha256": _sha256_file(Path(policy.path) / "model.safetensors")}
    if args.transport.startswith("sim"):
        from carton import fold_policy_fakes as fakes
        if args.sim_trial is None:
            raise SystemExit("--sim-trial is required for simulation transports")
        rig = fakes.build_sim_rig(args.sim_trial, args.out.with_name(args.out.name + "-sim"), owner=args.transport.startswith("sim-owner"),
                                  stream=args.transport == "sim-owner-stream",
                                  owner_period_s=args.owner_period_s, rtt_s=args.rtt_s,
                                  camera_keys=policy_camera_keys(policy))
        policy = fakes.LatencyPolicy(policy, rig.sleep, latency_s=args.policy_latency_s)
        runner = FoldPolicyRunner(policy, rig.transport, rig.cameras, rig.arm_maps, config, args.out, envelope=envelope,
                                  clock=rig.clock, sleep=rig.sleep, stop_requested=stopped, metadata=meta)
        if args.execute:
            rig.enable_all()
        summary = runner.run()
        summary["simulation"] = rig.score()
        atomic_json(args.out / "summary.json", _clean(summary))
    else:
        maps = load_arm_maps(_pairs(args.joint_map, "joint-map"))
        if args.execute and not args.operator:
            raise SystemExit("--execute on the robot needs --operator (the person holding STOP)")
        meta["operator"] = args.operator
        if args.pilot_root is None:
            raise SystemExit("--pilot-root is required: camera frames (and api motion) go through the robot API client")
        # The pilot's authenticated client, exactly as tools/measure_robot_link.py loads it. Not in this repo.
        sys.path.insert(0, str(args.pilot_root.resolve()))
        import importlib
        robot = importlib.import_module("chat_server").Robot(args.pilot_root / ".private/robot.json")
        if args.transport == "api":
            transport = ApiOwnerTransport(robot)
        elif args.transport == "api-stream":
            transport = StreamOwnerTransport(robot)
        else:
            if args.session is None or args.calibration is None:
                raise SystemExit("--session and --calibration are required for --transport direct-client")
            bridge = Path(__file__).resolve().parents[1] / "docs/commissioning/2026-10-07-paddle-success/qwen-bridge"
            sys.path.insert(0, str(bridge))
            from gemma_direct_client import DirectJointClient
            transport = DirectClientOwnerTransport(DirectJointClient(args.session, json.loads(args.calibration.read_text())))
        keys = policy_camera_keys(policy) or CAMERA_KEYS
        if args.camera:
            mapping = _pairs(args.camera, "camera")
        elif set(keys) == set(DEFAULT_ROBOT_CAMERAS):
            mapping = dict(DEFAULT_ROBOT_CAMERAS)
        else:
            raise SystemExit(f"--camera KEY=CAMERA is required for this checkpoint's cameras {keys} (robot cameras "
                             f"{ROBOT_CAMERAS}); only the robot-model policy has a default mapping")
        cameras = ApiCameras(robot, mapping)
        runner = FoldPolicyRunner(policy, transport, cameras, maps, config, args.out, envelope=envelope,
                                  stop_requested=stopped, metadata=meta)
        summary = runner.run()
    print(json.dumps(_clean({k: summary.get(k) for k in ("execute", "steps", "aborted", "end", "effective_hz",
                                                          "commands_sent", "clamp_counts", "simulation")}), indent=1))
    return 0 if summary.get("aborted") is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
