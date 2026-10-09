"""Fake and simulated back ends for carton.fold_policy_runner. No hardware, network, serial port or camera access.

FakeOwner runs the DEPLOYED robot-server code from docs/commissioning/2026-10-07-paddle-success/qwen-bridge on a
virtual clock, the way qwen-bridge/test_tag_registration_contract.py does:

    call(name, args) -> gemma_robot_tools.dispatch (the HTTPS API's tool layer; robot-Mac-only imports stubbed)
                     -> DirectJointClient (status.json / command.json protocol, its validation and STOP-on-failure)
                     -> HardwareOwner(--both-arms --paddle-profile, 2 s soft release) -> PaddleJointExecutor
                     -> FakeBus (STS registers) -> a plant

so tick units, the 3..341-tick rule, 40-tick ramp steps, the 96-tick following envelope, contact_halt, the lease,
replace/halt, STOP without latch and stop_count all come from the owner's own code, not from a re-implementation.

Plants: KinematicPlant (rate-limited follower, for fast tests) and MujocoFoldPlant (the carton fold simulation of
tools/eval_fold_policy.py, so the whole runner loop can run against the trained checkpoint). SimDirectTransport
skips the owner (only the runner's own bounds apply) to separate owner effects from policy effects.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import importlib.util
import json
import math
import os
import sys
import time
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace as C
from typing import Any, Callable
from unittest import mock

import numpy as np

from carton.fold_policy_runner import (ARMS, CAMERA_KEYS, DEFAULT_ROBOT_CAMERAS, LEGACY_CAMERA_KEYS, OWNER_JOINTS, SUFFIXES, TICKS_PER_REV, ArmMap, Frame,
                                       load_arm_maps, parse_owner_status)

SOFTWARE = Path(__file__).resolve().parents[1]
BRIDGE = SOFTWARE / "docs/commissioning/2026-10-07-paddle-success/qwen-bridge"
HEAD_AND_WHEELS = ("head_motor_1", "head_motor_2", "base_left_wheel", "base_right_wheel")
STS_STALL_NM = 2.94          # STS3215 stall torque; also the simulation's actuator forcerange


class VirtualClock:
    """Simulated wall clock, one hour ahead of real time.

    DirectJointClient numbers commands with the REAL time.time_ns() and confirms STOP by comparing the owner's
    status time with that id, so the virtual owner clock must never be behind real time.
    """
    def __init__(self, start: float | None = None):
        self.t = time.time() + 3600.0 if start is None else float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float):
        if dt > 0:
            self.t += dt


# ------------------------------------------------------------------------------------------------ plants
class KinematicPlant:
    """Each torque-enabled joint moves toward its goal at <= rate ticks/s; released joints stay where they are."""
    def __init__(self, start: dict[str, int], rate_ticks_s: float = 200.0, load: int = 40):
        self.q = {n: float(v) for n, v in start.items()}
        self.goal = {n: float(v) for n, v in start.items()}
        self.torque = dict.fromkeys(start, False)
        self.rate, self.base_load = rate_ticks_s, load
        self.extra_load: dict[str, int] = {}
        self.blocked: dict[str, float] = {}     # joint -> encoder value it cannot pass (an obstacle)
        self.velocity = dict.fromkeys(start, 0.0)
        self._side: dict[str, int] = {}

    def ticks(self, n):
        return int(round(self.q[n]))

    def set_goal(self, n, ticks):
        self.goal[n] = float(ticks)

    def set_torque(self, n, on):
        self.torque[n] = bool(on)

    def integrate(self, dt):
        for n in self.q:
            before = self.q[n]
            if self.torque[n]:
                step = self.rate * dt
                target = self.goal[n]
                if n in self.blocked:
                    stop = self.blocked[n]
                    side = self._side.setdefault(n, 1 if before >= stop else -1)
                    target = max(target, stop) if side > 0 else min(target, stop)
                self.q[n] += max(-step, min(step, target - self.q[n]))
            self.velocity[n] = (self.q[n] - before) / dt if dt > 0 else 0.0

    def load(self, n):
        return int(self.base_load + self.extra_load.get(n, 0))

    def present_velocity(self, n):
        return int(round(self.velocity[n]))


class MujocoFoldPlant:
    """The carton fold simulation (tools/eval_fold_policy.Episode) driven through servo goal ticks.

    Tick <-> radian conversion uses the generated simulation joint maps (sim_joint_maps), i.e. the same ArmMap code
    the runner uses. A new Goal_Position is ramped in with the smoothstep the scripted controller and the evaluator
    use (over `ramp_s`). Before the first enable, released joints keep their actuators active (the arm rests where the
    demonstration starts); after a release that follows an enable (STOP or owner fault) the actuator goes limp.
    Present_Load: load_model='constant' reports 40 (default); 'actuator' reports |actuator force| / 2.94 N m * 1000
    (STS3215 stall torque). The simulated kp=300 position actuators saturate during every new goal ramp, so the
    'actuator' figure overstates a real servo's Present_Load; it is recorded as max_actuator_load_estimate either way.
    """
    def __init__(self, trial: Path, maps: dict[str, ArmMap], *, height=240, width=320, ramp_s=0.08,
                 max_speed_ticks_s: float | None = None, load_model: str = "constant"):
        import mujoco
        episode_cls = _load_tool("eval_fold_policy").Episode
        self.mujoco = mujoco
        self.ep = episode_cls(Path(trial), height, width)
        self.m, self.d = self.ep.model, self.ep.data
        self.maps, self.ramp_s, self.max_speed, self.load_model = maps, ramp_s, max_speed_ticks_s, load_model
        self.index = {n: i for i, n in enumerate(OWNER_JOINTS)}
        self.act = np.asarray(self.ep.act_ids)
        self.adr = np.asarray(self.ep.robot_adr)
        self.dof = np.asarray([self.m.jnt_dofadr[self.m.actuator_trnid[a, 0]] for a in self.act])
        self.target = self.d.ctrl[self.act].copy()
        self.ramp_from = self.target.copy()
        self.ramp_t = np.full(12, 1e9)
        self.torque = np.zeros(12, bool)
        self.ever_enabled = np.zeros(12, bool)
        self.gains = (self.m.actuator_gainprm[self.act].copy(), self.m.actuator_biasprm[self.act].copy())
        self.limp = np.zeros(12, bool)
        self.carry = 0.0
        self.sim_time0 = float(self.d.time)
        self.folded_since, self.success_at = None, None
        self.next_score = float(self.d.time)
        self.max_load = np.zeros(12)

    # --- conversions
    def _arm(self, i):
        return self.maps[ARMS[i // 6]], i % 6

    def ticks(self, n):
        i = self.index[n]
        arm_map, j = self._arm(i)
        u = arm_map.units[SUFFIXES[j]]
        rad = float(self.d.qpos[self.adr[i]])
        return int(round(u.model_zero_tick + u.model_sign * math.degrees(rad) * TICKS_PER_REV / 360))

    def _rad(self, i, ticks):
        arm_map, j = self._arm(i)
        u = arm_map.units[SUFFIXES[j]]
        return math.radians(u.model_sign * (ticks - u.model_zero_tick) * 360 / TICKS_PER_REV)

    # --- servo interface
    def set_goal(self, n, ticks):
        i = self.index[n]
        self.ramp_from[i] = self.d.ctrl[self.act[i]]
        self.target[i] = self._rad(i, ticks)
        self.ramp_t[i] = 0.0

    def set_torque(self, n, on):
        i = self.index[n]
        self.torque[i] = bool(on)
        if on:
            self.ever_enabled[i] = True
            self.limp[i] = False
            self.m.actuator_gainprm[self.act[i]] = self.gains[0][i]
            self.m.actuator_biasprm[self.act[i]] = self.gains[1][i]
        elif self.ever_enabled[i]:
            self.limp[i] = True
            self.m.actuator_gainprm[self.act[i]] = 0
            self.m.actuator_biasprm[self.act[i]] = 0

    def integrate(self, dt):
        mujoco, m, d = self.mujoco, self.m, self.d
        self.carry += dt
        h = m.opt.timestep
        while self.carry >= h - 1e-12:
            self.carry -= h
            self.ramp_t += h
            u = np.clip(self.ramp_t / self.ramp_s, 0, 1)
            ctrl = self.ramp_from + (self.target - self.ramp_from) * (u * u * (3 - 2 * u))
            if self.max_speed is not None:
                limit = math.radians(self.max_speed * 360 / TICKS_PER_REV) * h
                ctrl = d.ctrl[self.act] + np.clip(ctrl - d.ctrl[self.act], -limit, limit)
            lo, hi = m.actuator_ctrlrange[self.act].T
            d.ctrl[self.act] = np.clip(ctrl, lo, hi)
            mujoco.mj_step(m, d)
            self._score_contacts()
            if d.time >= self.next_score:
                self.next_score += 0.1
                self._score_fold()
        self.max_load = np.maximum(self.max_load, [self.load_index(i) for i in range(12)])

    def load_index(self, i):
        if self.limp[i]:
            return 0
        return int(min(1000, abs(float(self.d.actuator_force[self.act[i]])) / STS_STALL_NM * 1000))

    def load(self, n):
        return self.load_index(self.index[n]) if self.load_model == "actuator" else 40

    def present_velocity(self, n):
        i = self.index[n]
        return int(round(math.degrees(float(self.d.qvel[self.dof[i]])) * TICKS_PER_REV / 360))

    # --- observation and scoring
    def render(self, camera):
        self.ep.renderer.update_scene(self.d, camera=camera)
        return self.ep.renderer.render().copy()

    def _score_contacts(self):
        ep, d = self.ep, self.d
        for c in d.contact[:d.ncon]:
            g = {int(c.geom1), int(c.geom2)}
            if g & ep.robot_geoms:
                other = g - ep.robot_geoms
                if other and other <= ep.flap_geoms:
                    ep.max_flap_pen = max(ep.max_flap_pen, -c.dist * 1000)
                elif len(g) != 1:
                    ep.max_other_pen = max(ep.max_other_pen, -c.dist * 1000)
        ep.max_carton_mm = max(ep.max_carton_mm, float(np.linalg.norm(d.qpos[12:15] - ep.carton0) * 1000))

    def _score_fold(self):
        t = float(self.d.time)
        if all(self.ep.hinge_degrees(h) >= 80 for h in ("short_left_hinge", "short_right_hinge")):
            self.folded_since = t if self.folded_since is None else self.folded_since
            if self.success_at is None and t - self.folded_since >= 3:
                self.success_at = t
        else:
            self.folded_since = None

    def score(self):
        ep = self.ep
        return {"success_both_shorts_held_3s": self.success_at is not None, "success_at_sim_s": self.success_at,
                "sim_time": round(float(self.d.time) - self.sim_time0, 2),
                "flaps_degrees": {h: round(ep.hinge_degrees(h), 1) for h in
                                  ("short_left_hinge", "short_right_hinge", "long_far_hinge", "long_near_hinge")},
                "max_carton_translation_mm": round(ep.max_carton_mm, 1),
                "max_robot_flap_penetration_mm": round(ep.max_flap_pen, 2),
                "max_robot_other_penetration_mm": round(ep.max_other_pen, 2),
                "max_actuator_load_estimate": {n: int(v) for n, v in zip(OWNER_JOINTS, self.max_load)},
                "load_model": self.load_model,
                "simulation_only": True}


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(f"_fold_tool_{name}", SOFTWARE / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------------------------------- servo bus
class FakeBus:
    """The registers HardwareOwner reads and writes, for the twelve arm motors (plus `extra`, e.g. the head), backed by
    a plant."""
    def __init__(self, plant, calibration: dict, extra: tuple = ()):
        self.plant = plant
        names = (*OWNER_JOINTS, *extra)
        self.motors = {n: C(id=i + 1, model="sts3215") for i, n in enumerate(names)}
        self.port = "fake-bus"
        self.r = {n: dict(Torque_Enable=0, Operating_Mode=0, Homing_Offset=calibration[n]["homing_offset"],
                          Min_Position_Limit=calibration[n]["range_min"], Max_Position_Limit=calibration[n]["range_max"],
                          Goal_Position=plant.ticks(n), Lock=1, Torque_Limit=1000, Goal_Velocity=0, Goal_Time=0,
                          Acceleration=0, P_Coefficient=16, I_Coefficient=0, D_Coefficient=32, Status=0)
                  for n in names}
        self.writes = 0

    def read(self, f, n, **kw):
        if f == "Present_Position":
            return self.plant.ticks(n)
        return self.r[n].get(f, 0)

    def write(self, f, n, v, **kw):
        self.r[n][f] = v
        self.writes += 1
        if f == "Goal_Position":
            self.plant.set_goal(n, v)
        elif f == "Torque_Enable":
            self.plant.set_torque(n, v == 1)

    def disable_torque(self, names, **kw):
        for n in names:
            self.write("Torque_Enable", n, 0)

    def telemetry(self, bus, n):
        v = self.plant.present_velocity(n)
        moving = int(abs(self.r[n]["Goal_Position"] - self.plant.ticks(n)) > 1 and self.r[n]["Torque_Enable"] == 1)
        return dict(Present_Position=self.plant.ticks(n), Present_Load=self.plant.load(n), Present_Voltage=124,
                    Present_Temperature=31, Moving=moving, Present_Velocity=v, Status=self.r[n]["Status"])


# ---------------------------------------------------------------------------- deployed robot server, faked
_TOOLS = None


def _bridge_on_path():
    if str(BRIDGE) not in sys.path:
        sys.path.insert(0, str(BRIDGE))


def import_robot_tools(cal_path: Path):
    """Import the real gemma_robot_tools once, with its robot-Mac-only dependencies stubbed (contract-test recipe)."""
    global _TOOLS
    if _TOOLS is not None:
        return _TOOLS
    _bridge_on_path()
    stubs = {"carton_preflight": dict(run=lambda: {"error": "fake: serial preflight unavailable"}),
             "gemma_execution_binding": dict(TrustedExecutionBinding=lambda session: C(bind=None)),
             "gemma_reach_planner": dict(POSE_SCHEMA={"type": "array"}, validate_poses=lambda poses: None,
                                         inspect_or_plan=lambda *a, **k: {"status": "NEEDS_GEOMETRIC_CONFIGURATION",
                                                                          "motor_writes": 0})}
    for name, attributes in stubs.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        sys.modules.setdefault(name, module)
    original = Path.read_text

    def read_text(self, *args, **kwargs):
        if self.name == "farm_xlerobot.json" and self != cal_path:
            return original(cal_path)
        if self.name == "gateway-server.pem" and "gemma-mtls" in str(self):
            return original(BRIDGE / "gateway-server.pem")
        return original(self, *args, **kwargs)
    import gemma_direct_client, paddle_segments, remote_admin, wrist_cameras  # noqa: F401  bind bridge modules first
    environment, path = dict(os.environ), list(sys.path)
    with mock.patch.object(Path, "read_text", read_text):
        import gemma_robot_tools as tools
    os.environ.clear()
    os.environ.update(environment)
    sys.path[:] = path
    _bridge_on_path()
    _TOOLS = tools
    return tools


def fake_calibration(ranges: dict[str, tuple[int, int]]) -> dict:
    """Saved-calibration JSON for the twelve arm motors plus inert head/wheel entries (the API's schemas list 16)."""
    cal = {n: {"id": i % 6 + 1, "drive_mode": 0, "homing_offset": 0, "range_min": int(lo), "range_max": int(hi)}
           for i, (n, (lo, hi)) in enumerate(ranges.items())}
    for i, n in enumerate(HEAD_AND_WHEELS):
        cal[n] = {"id": 7 + i, "drive_mode": 0, "homing_offset": 0,
                  "range_min": 0 if n.startswith("base_") else 1019, "range_max": 4095 if n.startswith("base_") else 3151}
    return cal


class FakeOwner:
    """The deployed API + DirectJointClient + HardwareOwner, on a fake bus and a virtual clock."""
    def __init__(self, plant, calibration: dict, workdir: Path, *, clock: VirtualClock | None = None,
                 owner_period_s: float = 0.05, rtt_s: float = 0.0, soft_release_s: float = 2.0,
                 camera_publisher: Callable[["FakeOwner", list], None] | None = None, stream: bool = False,
                 head: bool = False):
        self.workdir = Path(workdir)
        self.session = self.workdir / "work/gemma-hardware-session"
        self.session.mkdir(parents=True, exist_ok=False)
        self.cal_path = self.workdir / "farm_xlerobot.json"
        self.cal_path.write_text(json.dumps(calibration, indent=1))
        self.calibration = calibration
        tools = import_robot_tools(self.cal_path)
        from gemma_direct_client import DirectJointClient
        from gemma_hardware_owner import HardwareOwner, atomic
        self.tools, self.atomic = tools, atomic
        self.clock = clock or VirtualClock()
        self.plant, self.period, self.rtt = plant, owner_period_s, rtt_s
        # Head motors are on the bus when the plant has them (read-only unless the owner runs the --head scope).
        heads = tuple(n for n in HEAD_AND_WHEELS[:2] if n in getattr(plant, "q", {}))
        if head and not heads:
            raise ValueError("--head scope needs a plant with head_motor_1/head_motor_2")
        self.bus = FakeBus(plant, calibration, heads)
        self.phone = {"fresh": True, "seq": 0}
        self.owner = HardwareOwner(
            [self.bus], {n: C(range_min=calibration[n]["range_min"], range_max=calibration[n]["range_max"],
                              homing_offset=calibration[n]["homing_offset"]) for n in (*OWNER_JOINTS, *heads)},
            self.bus.telemetry, clock=self.clock, wall=self.clock, position_scope=list(OWNER_JOINTS),
            paddle_profile=True, camera_metadata=self._phone_metadata, wheels=False,
            soft_release_s=soft_release_s, sleep=self._advance_only, stream=stream, head=head)
        self.owner.inspect()
        self.owner.writer = lambda: atomic(self.session / "status.json", self.owner.state)
        atomic(self.session / "status.json", self.owner.state)
        self.last_command = None
        self.next_owner_tick = self.clock() + self.period
        self.paused = False                 # True: the owner loop stops (hung owner -> stale status)
        self.faults: list[str] = []
        self.calls: list[tuple] = []
        self.camera_publisher = camera_publisher
        self.client = DirectJointClient(self.session, calibration, clock=self.clock, sleep=self.sleep)
        self.bind()

    def bind(self):
        """Point the (process-wide) API module at this rig, as the contract test's Rig does."""
        t = self.tools
        t.ROOT, t.SESSION, t.CAL = self.workdir, self.session, self.calibration
        t.OAK_RECTIFIED_DIR, t.OAK_RAW_DIR = self.workdir / "work/oak-rectified-stream", self.workdir / "work/oak-stream"
        t.time = C(time=self.clock, sleep=self.sleep, time_ns=time.time_ns, monotonic=self.clock)
        t.WRIST_DIRS = [self.workdir / "work/wrist-camera-stream"]
        sys.modules["wrist_cameras"].time = t.time      # its 1 s freshness check reads the same (virtual) clock
        t.DIRECT_CLIENT = self.client
        t.REQUESTS.clear()

    def _phone_metadata(self):
        self.phone["seq"] += 1
        return {"received_at": self.clock() if self.phone["fresh"] else self.clock() - 60, "seq": self.phone["seq"]}

    # --- time
    def _advance_only(self, dt):
        """Used by the owner's soft release (it runs inside an owner iteration): time and physics, no new iteration."""
        self.plant.integrate(dt)
        self.clock.advance(dt)

    def sleep(self, dt):
        end = self.clock() + max(0.0, dt)
        while self.clock() < end - 1e-12:
            step = min(end - self.clock(), max(1e-6, self.next_owner_tick - self.clock()))
            self.plant.integrate(step)
            self.clock.advance(step)
            if self.clock() >= self.next_owner_tick - 1e-12:
                self.next_owner_tick += self.period
                self.pump()

    def pump(self):
        """One iteration of gemma_hardware_owner.main's loop (poll, fault -> release_all, command file, status)."""
        if self.paused:
            return
        o = self.owner
        try:
            o.poll()
        except RuntimeError as e:
            o.state["failed_command_id"] = o.current_command
            o.release_all(str(e))
            self.faults.append(str(e))
        p = self.session / "command.json"
        if p.exists():
            c = json.loads(p.read_text())
            if c.get("id") != self.last_command:
                self.last_command = c.get("id")
                try:
                    o.command(c)
                except ValueError as e:
                    o.state["last_rejected"] = {"id": self.last_command, "reason": str(e)}
        self.atomic(self.session / "status.json", o.state)

    # --- the HTTPS API, minus TLS
    def call(self, name, args, request_id=None):
        self.calls.append((name, copy.deepcopy(args), self.clock()))
        self.sleep(self.rtt / 2)
        if name == "robot_get_cameras" and self.camera_publisher is not None:
            self.camera_publisher(self, list(args.get("cameras", ["oak", "phone"])))
        try:
            result, images = self.tools.dispatch(name, copy.deepcopy(args))
            body = {"ok": True, "result": result, **({"images": images} if images is not None else {})}
            body = json.loads(json.dumps(body, allow_nan=False))
        except (ValueError, KeyError, TypeError) as e:
            body = {"ok": False, "result": {"error": str(e)}}
        except Exception as e:  # noqa: BLE001 - the API answers 409 with the error
            body = {"ok": False, "result": {"error": str(e)}}
        self.sleep(self.rtt / 2)
        return body

    def enable_all(self):
        for arm in ARMS:
            names = [n for n in OWNER_JOINTS if n.startswith(f"{arm}_arm_")]
            out = self.call("robot_set_motor_enable", {"names": names, "enabled": True})
            if out.get("ok") is not True or out["result"].get("completed") is not True:
                raise RuntimeError(f"enable failed: {out}")

    def moves(self):
        return [a for n, a, _ in self.calls if n == "robot_move_joint_targets"]

    # --- camera files the API reads
    def publish_oak(self, rgb, extra: dict | None = None):
        """An OAK manifest as the API reads it; `extra` adds fields (intrinsics, width, height, coordinate_frame ...)."""
        import cv2
        folder = self.tools.OAK_RECTIFIED_DIR
        folder.mkdir(parents=True, exist_ok=True)
        seq = getattr(self, "_oak_seq", 0) + 1
        self._oak_seq = seq
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR))
        data = buf.tobytes()
        name = f"oak-{seq:06d}.jpg"
        (folder / name).write_bytes(data)
        manifest = {"camera_id": "oak-sim", "stream_id": "oak-sim-stream", "seq": seq, "image": name,
                    "sha256": hashlib.sha256(data).hexdigest(), "captured_at": self.clock(),
                    "projection": "rectified_pinhole", **(extra or {})}
        tmp = folder / "oak.json.tmp"
        tmp.write_text(json.dumps(manifest))
        tmp.replace(folder / "oak.json")

    def publish_wrist(self, name, rgb):
        """A wrist stream manifest as wrist_cameras.select_wrist_manifest reads it (identity = the configured ID)."""
        import cv2
        folder = self.tools.WRIST_DIRS[0]
        folder.mkdir(parents=True, exist_ok=True)
        seq = getattr(self, "_wrist_seq", {}).get(name, 0) + 1
        self._wrist_seq = {**getattr(self, "_wrist_seq", {}), name: seq}
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR))
        data = buf.tobytes()
        image = f"{name}-{seq:06d}.jpg"
        (folder / image).write_bytes(data)
        manifest = {"camera_id": sys.modules["wrist_cameras"].WRIST_CAMERA_IDS[name], "stream_id": f"{name}-sim",
                    "seq": seq, "image": image, "sha256": hashlib.sha256(data).hexdigest(),
                    "captured_at": self.clock(), "width": int(rgb.shape[1]), "height": int(rgb.shape[0])}
        tmp = folder / f"{name}.json.tmp"
        tmp.write_text(json.dumps(manifest))
        tmp.replace(folder / f"{name}.json")

    def publish_phone(self, rgb):
        import cv2
        folder = self.workdir / "work/phone_camera"
        folder.mkdir(parents=True, exist_ok=True)
        seq = getattr(self, "_phone_seq", 0) + 1
        self._phone_seq = seq
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR))
        (folder / "latest.jpg").write_bytes(buf.tobytes())
        (folder / "latest.json").write_text(json.dumps({"seq": seq, "received_at": self.clock()}))


# --------------------------------------------------------------------------------- simulation-only transport
class SimDirectTransport:
    """No owner: targets go straight to the plant as goals. Isolates the policy and the runner's own bounds."""
    name = "sim-direct"
    streams_gripper_closure = True     # simulation-only diagnostic of gripper_mode 'stream'

    def __init__(self, plant, calibration, clock):
        self.plant, self.calibration, self.clock = plant, calibration, clock
        self.goals = {n: plant.ticks(n) for n in OWNER_JOINTS}
        self.enabled = False

    def enable_all(self):
        for n in OWNER_JOINTS:
            self.goals[n] = self.plant.ticks(n)
            self.plant.set_goal(n, self.goals[n])
            self.plant.set_torque(n, True)
        self.enabled = True

    def snapshot(self):
        now = self.clock()
        rows = {n: {"Present_Position": self.plant.ticks(n), "captured_at": now, "Torque_Enable": int(self.enabled),
                    "Status": 0, "Present_Load": self.plant.load(n)} for n in OWNER_JOINTS}
        status = {"hardware_server": True, "control_mode": "direct_joint", "rows": rows, "time": now, "status_age_s": 0.0,
                  "started": 1.0, "phase": "holding" if self.enabled else "idle", "ok": True, "stop_count": 0,
                  "enabled_motors": list(OWNER_JOINTS) if self.enabled else [], "goals": dict(self.goals),
                  "ranges": {n: [self.calibration[n]["range_min"], self.calibration[n]["range_max"]] for n in OWNER_JOINTS},
                  "execution_profile": "paddle-success-v1"}
        return parse_owner_status(status, local_sent=now, local_received=now)

    def send(self, targets, duration_s):
        for n, q in targets.items():
            self.goals[n] = int(q)
            self.plant.set_goal(n, int(q))
        return [{"arm": "both", "command_id": None}]

    def halt(self):
        return {"halted": True}


# ------------------------------------------------------------------------------------------------ cameras
SCENE_CAMERA = {"top": "overhead"}   # policy key -> MuJoCo camera where they differ (as tools/eval_fold_policy.py)


def scene_camera_keys(model) -> tuple[str, ...]:
    """Policy keys a fold scene can render: the robot-model cameras if it has them, else the first policy's."""
    names = {model.camera(i).name for i in range(model.ncam)}
    return CAMERA_KEYS if set(CAMERA_KEYS) <= names else LEGACY_CAMERA_KEYS


class SimCameras:
    """Renders the policy's cameras from the scene (`top` is the simulated `overhead`), stamped with the virtual clock."""
    def __init__(self, plant, clock, keys=LEGACY_CAMERA_KEYS):
        self.plant, self.clock, self.seq = plant, clock, 0
        self.keys = tuple(keys)

    def frames(self):
        self.seq += 1
        now = self.clock()
        return {key: Frame(self.plant.render(SCENE_CAMERA.get(key, key)), now, self.seq,
                           f"sim:{SCENE_CAMERA.get(key, key)}") for key in self.keys}


class SyntheticCameras:
    """Small deterministic images for fast tests; `freeze` repeats the last frame, `delay_s` ages it."""
    def __init__(self, clock, shape=(24, 32, 3), keys=CAMERA_KEYS):
        self.clock, self.shape, self.seq = clock, shape, 0
        self.keys = tuple(keys)
        self.freeze, self.delay_s = False, 0.0
        self.last = None

    def frames(self):
        if self.freeze and self.last is not None:
            return self.last
        self.seq += 1
        now = self.clock() - self.delay_s
        rng = np.random.default_rng(self.seq)
        self.last = {k: Frame(rng.integers(0, 255, self.shape, dtype=np.uint8), now, self.seq, f"synthetic:{k}")
                     for k in self.keys}
        return self.last


def sim_camera_publisher(plant, keys=LEGACY_CAMERA_KEYS):
    """For ApiCameras through the real robot_get_cameras. First policy: overhead -> 'oak' manifest, front -> phone
    files. Robot-model policy: the head camera `front` -> 'oak', the wrist cameras -> the wrist streams."""
    legacy = tuple(keys) == LEGACY_CAMERA_KEYS
    def publish(owner: FakeOwner, names):
        if "oak" in names:
            owner.publish_oak(plant.render("overhead" if legacy else "front"))
        if "phone" in names:
            owner.publish_phone(plant.render("front"))
        for name in ("left_wrist", "right_wrist"):
            if name in names:
                owner.publish_wrist(name, plant.render(name))
    return publish


# -------------------------------------------------------------------------------------------- joint maps
def write_joint_maps(folder: Path, calibration: dict, zero_sign: dict[str, tuple[float, int]], *, evidence: str):
    """Write the saved calibration and both arms' joint maps in the runner's format."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    cal_path = folder / "farm_xlerobot.json"
    cal_path.write_text(json.dumps(calibration, indent=1))
    sha = hashlib.sha256(cal_path.read_bytes()).hexdigest()
    paths = {}
    for arm in ARMS:
        joints = {s: {"model_zero_tick": zero_sign[f"{arm}_arm_{s}"][0], "model_sign": zero_sign[f"{arm}_arm_{s}"][1]}
                  for s in SUFFIXES[:5]}
        g = zero_sign[f"{arm}_arm_gripper"]
        cfg = {"schema": 1, "arm": arm, "calibration_file": str(cal_path), "calibration_sha256": sha, "joints": joints,
               "gripper": {"model_zero_tick": g[0], "model_sign": g[1]},
               "evidence": {"joints": evidence, "gripper": evidence}}
        paths[arm] = folder / f"{arm}-joint-map.json"
        paths[arm].write_text(json.dumps(cfg, indent=1))
    return load_arm_maps(paths), paths


def sim_zero_sign_and_ranges(model, margin_ticks=60):
    """Simulation-only mapping: zero tick 2048 for every joint; the left arm counts the opposite way (sign -1) so
    both signs are exercised. Ranges cover the MuJoCo joint limits plus `margin_ticks` on each side so the training
    start pose (shoulder_lift at -100 deg, the joint limit) sits inside the owner's 40-tick commandable margin."""
    zero_sign, ranges = {}, {}
    for n in OWNER_JOINTS:
        arm, suffix = n.split("_arm_")
        lo, hi = model.jnt_range[model.joint(f"{arm}_{suffix}").id]
        sign = -1 if arm == "left" else 1
        ends = sorted(2048 + sign * math.degrees(v) * TICKS_PER_REV / 360 for v in (lo, hi))
        zero_sign[n] = (2048.0, sign)
        ranges[n] = (max(0, int(math.floor(ends[0])) - margin_ticks), min(4095, int(math.ceil(ends[1])) + margin_ticks))
    return zero_sign, ranges


# -------------------------------------------------------------------------------------------- assembled rigs
@dataclass
class SimRig:
    clock: VirtualClock
    sleep: Callable[[float], None]
    transport: Any
    cameras: Any
    arm_maps: dict
    plant: Any
    owner: FakeOwner | None
    calibration: dict

    def enable_all(self):
        if self.owner is not None:
            self.owner.enable_all()
        else:
            self.transport.enable_all()

    def score(self):
        out = self.plant.score() if hasattr(self.plant, "score") else {}
        if self.owner is not None:
            out["owner_faults"] = list(self.owner.faults)
            out["owner_stop_count"] = self.owner.owner.state.get("stop_count")
            out["owner_last_stop"] = self.owner.owner.state.get("last_stop")
        return out


def build_sim_rig(trial: Path, workdir: Path, *, owner: bool = True, owner_period_s=0.05, rtt_s=0.0,
                  api_cameras: bool = False, height=240, width=320, max_speed_ticks_s=None, load_model="constant",
                  camera_keys=None, stream: bool = False):
    """MuJoCo carton fold simulation behind the deployed owner (owner=True) or directly (owner=False).

    `camera_keys` are the policy's cameras (default: what the scene has, see scene_camera_keys)."""
    import mujoco
    from carton.fold_policy_runner import ApiCameras, ApiOwnerTransport, StreamOwnerTransport
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=False)
    model = mujoco.MjModel.from_xml_path(str(Path(trial) / "run/scene.xml"))
    keys = tuple(camera_keys or scene_camera_keys(model))
    zero_sign, ranges = sim_zero_sign_and_ranges(model)
    calibration = fake_calibration(ranges)
    maps, _ = write_joint_maps(workdir / "joint-maps", calibration, zero_sign,
                               evidence="SIMULATION ONLY: generated from MuJoCo joint ranges, not a robot measurement")
    plant = MujocoFoldPlant(trial, maps, height=height, width=width, max_speed_ticks_s=max_speed_ticks_s,
                            load_model=load_model)
    clock = VirtualClock()
    if owner:
        fake = FakeOwner(plant, calibration, workdir / "robot", clock=clock, owner_period_s=owner_period_s, rtt_s=rtt_s,
                         camera_publisher=sim_camera_publisher(plant, keys) if api_cameras else None, stream=stream)
        transport = (StreamOwnerTransport if stream else ApiOwnerTransport)(fake, clock=clock)
        mapping = {"top": "oak", "front": "phone"} if keys == LEGACY_CAMERA_KEYS else DEFAULT_ROBOT_CAMERAS
        cameras = ApiCameras(fake, mapping) if api_cameras else SimCameras(plant, clock, keys)
        return SimRig(clock, fake.sleep, transport, cameras, maps, plant, fake, calibration)

    def sleep(dt):
        plant.integrate(dt)
        clock.advance(dt)
    transport = SimDirectTransport(plant, calibration, clock)
    return SimRig(clock, sleep, transport, SimCameras(plant, clock, keys), maps, plant, None, calibration)


def build_kinematic_rig(workdir: Path, start: dict[str, int] | None = None, *, rate_ticks_s=200.0, owner_period_s=0.05,
                        rtt_s=0.0, zero_sign=None, ranges=None, stream=False):
    """Fast rig: deployed owner over a rate-limited kinematic plant and synthetic cameras."""
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=False)
    ranges = ranges or {n: (826, 3268) for n in OWNER_JOINTS}
    zero_sign = zero_sign or {n: (2048.0, -1 if n.startswith("left") else 1) for n in OWNER_JOINTS}
    start = start or {n: 2048 for n in OWNER_JOINTS}
    calibration = fake_calibration(ranges)
    maps, map_paths = write_joint_maps(workdir / "joint-maps", calibration, zero_sign, evidence="fake test fixture")
    plant = KinematicPlant(start, rate_ticks_s)
    clock = VirtualClock()
    fake = FakeOwner(plant, calibration, workdir / "robot", clock=clock, owner_period_s=owner_period_s, rtt_s=rtt_s,
                     stream=stream)
    from carton.fold_policy_runner import ApiOwnerTransport, StreamOwnerTransport
    rig = SimRig(clock, fake.sleep, (StreamOwnerTransport if stream else ApiOwnerTransport)(fake, clock=clock), SyntheticCameras(clock), maps, plant, fake,
                 calibration)
    rig.map_paths = map_paths
    return rig


# ------------------------------------------------------------------------------------------------ policies
class LatencyPolicy:
    """Wraps a policy so its inference time advances the virtual clock (measured wall time, or a fixed value)."""
    def __init__(self, policy, sleep, latency_s: float | None = None):
        self.policy, self.sleep, self.latency_s = policy, sleep, latency_s
        self.path = getattr(policy, "path", None)
        self.wall = []

    def reset(self):
        self.policy.reset()

    def input_spec(self):
        return self.policy.input_spec()

    def act(self, state, images, task=None):
        t0 = time.perf_counter()
        out = self.policy.act(state, images, task)
        elapsed = time.perf_counter() - t0
        self.wall.append(elapsed)
        self.sleep(self.latency_s if self.latency_s is not None else elapsed)
        return out


class ReplayPolicy:
    """Plays a recorded demonstration's commanded targets (radians) as if it were the policy."""
    def __init__(self, ctrl: np.ndarray):
        self.ctrl, self.k = np.asarray(ctrl, float), 0

    def reset(self):
        self.k = 0

    def act(self, state, images, task=None):
        self.k += 1
        return self.ctrl[min(self.k, len(self.ctrl) - 1)].astype(np.float32)


# ------------------------------------------------------------------------------------- head OAK (auto head pose)
@dataclass
class HeadMapping:
    """A fake robot's TRUE head tick -> model angle mapping (tools/auto_head_pose.py must not rely on it).

    tilt = (head_motor_2 - tilt_zero_tick) * tilt_deg_per_tick; pan = (head_motor_1 - pan_zero_tick) * pan_deg_per_tick
    (model convention: tilt positive looks down, pan positive looks to the robot's left)."""
    tilt_zero_tick: float
    pan_zero_tick: float
    tilt_deg_per_tick: float = 360 / TICKS_PER_REV
    pan_deg_per_tick: float = 360 / TICKS_PER_REV

    def angles(self, ticks):
        return ((ticks["head_motor_2"] - self.tilt_zero_tick) * self.tilt_deg_per_tick,
                (ticks["head_motor_1"] - self.pan_zero_tick) * self.pan_deg_per_tick)

    def ticks_for(self, tilt_deg, pan_deg):
        return {"head_motor_1": int(round(self.pan_zero_tick + pan_deg / self.pan_deg_per_tick)),
                "head_motor_2": int(round(self.tilt_zero_tick + tilt_deg / self.tilt_deg_per_tick))}


class HeadOakSim:
    """The head OAK of a fake robot: renders what it sees for given arm and head ticks. Simulation only.

    The camera is the XLeRobot model head (carton/xlerobot_cameras.head_camera_model) at the angles `mapping` gives,
    plus an optional lens offset from the camera link; arm joints come from the joint maps. `scene_xml`: render the
    fold training scene with MuJoCo (its 'front' camera moved to the head pose); without it, a synthetic image of the
    tags only (warped tag36h11 images on grey). carton: 'away' (default), 'nominal' (the training spot of
    carton.head_pose.nominal_carton_pose; needs `station`) or 'scene' (where the scene put it; MuJoCo only)."""
    def __init__(self, arm_maps, mapping: HeadMapping, *, width=640, height=360, fovy_deg=39.2, scene_xml=None,
                 base_spacing_m=.22, optical_offset=(0., 0., 0.), carton="away", station=None):
        self.maps, self.mapping, self.width, self.height, self.fovy = arm_maps, mapping, width, height, fovy_deg
        self.fy = height / 2 / math.tan(math.radians(fovy_deg) / 2)
        if carton not in ("away", "nominal", "scene") or (carton == "nominal" and station is None):
            raise ValueError("carton: 'away', 'nominal' (with station) or 'scene'")
        self.base_spacing, self.offset, self.carton, self.station = base_spacing_m, optical_offset, carton, station
        self.scene_xml = scene_xml
        self._model = None
        self.renders = 0

    def intrinsics(self):
        return {"fx": self.fy, "fy": self.fy, "cx": self.width / 2, "cy": self.height / 2,
                "width": self.width, "height": self.height}

    def manifest_extras(self):
        return {"width": self.width, "height": self.height,
                "intrinsics": [[self.fy, 0, self.width / 2], [0, self.fy, self.height / 2], [0, 0, 1]],
                "coordinate_frame": "CAM_A_optical", "rgb_pipeline": f"simulated head OAK {self.width}x{self.height}"}

    def camera_pose(self, head_ticks):
        """(position, rotation_cv) of the lens in arm_base for these head ticks (the TRUE pose)."""
        from carton.xlerobot_cameras import head_camera_model, model_dir_to_arm_base, model_to_arm_base
        tilt, pan = self.mapping.angles(head_ticks)
        position, r = head_camera_model(math.radians(tilt), math.radians(pan))
        lens = model_to_arm_base(position + r @ np.asarray(self.offset, float))
        forward, up = model_dir_to_arm_base(r[:, 0]), model_dir_to_arm_base(r[:, 2])
        return lens, np.column_stack((np.cross(forward, up), -up, forward))

    def _joint_rad(self, ticks):
        return {side: self.maps[side].ticks_to_rad(ticks)[:5] for side in ARMS}

    def render(self, ticks):
        self.renders += 1
        position, rotation = self.camera_pose(ticks)
        if self.scene_xml is not None:
            return self._render_mujoco(ticks, position, rotation)
        return self._render_synthetic(ticks, position, rotation)

    def _render_mujoco(self, ticks, position, rotation):
        import mujoco
        if self._model is None:
            self._model = mujoco.MjModel.from_xml_path(str(self.scene_xml))
            self._data = mujoco.MjData(self._model)
            self._renderer = mujoco.Renderer(self._model, self.height, self.width)
        m, d = self._model, self._data
        mujoco.mj_resetData(m, d)
        for side, q in self._joint_rad(ticks).items():
            for suffix, v in zip(SUFFIXES[:5], q):
                d.qpos[m.jnt_qposadr[m.joint(f"{side}_{suffix}").id]] = v
        mujoco.mj_forward(m, d)
        origin = (d.body("left_base_link").xpos + d.body("right_base_link").xpos) / 2
        if self.carton != "scene":
            from carton import head_pose
            adr = m.jnt_qposadr[m.joint("carton_free").id]
            centre = (origin + head_pose.nominal_carton_pose(self.station)[1] if self.carton == "nominal"
                      else [0., 2.5, .001])
            d.qpos[adr:adr + 7] = [*centre, 1., 0., 0., 0.]
            mujoco.mj_forward(m, d)
        cid = m.camera("front").id
        m.cam_pos[cid] = origin + position
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.column_stack((rotation[:, 0], -rotation[:, 1], -rotation[:, 2])).ravel())
        m.cam_quat[cid] = quat
        m.cam_fovy[cid] = self.fovy
        mujoco.mj_forward(m, d)
        self._renderer.update_scene(d, camera="front")
        return self._renderer.render().copy()

    def close(self):
        if self._model is not None:
            self._renderer.close()
            self._model = None

    def _render_synthetic(self, ticks, position, rotation):
        import cv2
        from carton import head_pose
        from carton.servo.tag_kit import marker_grid
        mounts = head_pose.gripper_tag_mounts(self._joint_rad(ticks), self.base_spacing)
        if self.carton == "nominal":
            mounts.update(head_pose.box_tag_mounts(self.station))
        image = np.full((self.height, self.width, 3), 140, np.uint8)
        r_cb = rotation.T
        for tag_id in sorted(mounts, key=lambda i: -float((r_cb @ (mounts[i][0] - position))[2])):   # far first
            centre, u, v, size = mounts[tag_id]
            if np.cross(u, v) @ (position - centre) <= 0:
                continue                      # printed side faces away from the camera
            half = size / 2 * 10 / 8          # black square plus one white cell on each side
            outer = np.array([centre - half * u + half * v, centre + half * u + half * v,
                              centre + half * u - half * v, centre - half * u - half * v])
            cam = (outer - position) @ r_cb.T
            if (cam[:, 2] <= .02).any():
                continue
            pix = cam[:, :2] / cam[:, 2:] * self.fy + [self.width / 2, self.height / 2]
            n = 240
            grid = np.pad(np.asarray(marker_grid(tag_id)), 1, constant_values=255).astype(np.uint8)
            tag = cv2.resize(grid, (n, n), interpolation=cv2.INTER_NEAREST)
            # Image coordinate x is the centre of pixel x: the tag image's outer edge is at -0.5.
            h = cv2.getPerspectiveTransform(np.float32([[-.5, -.5], [n - .5, -.5], [n - .5, n - .5], [-.5, n - .5]]),
                                            np.float32(pix))
            warped = cv2.warpPerspective(tag, h, (self.width, self.height), flags=cv2.INTER_LINEAR, borderValue=0)
            mask = cv2.warpPerspective(np.full_like(tag, 255), h, (self.width, self.height), flags=cv2.INTER_LINEAR,
                                       borderValue=0).astype(float)[..., None] / 255
            image = (image * (1 - mask) + warped[..., None] * mask).round().astype(np.uint8)
        return image

    def publisher(self, plant):
        """camera_publisher for FakeOwner: the OAK frame from the plant's current arm and head ticks."""
        def publish(owner, names):
            if "oak" in names:
                owner.publish_oak(self.render({n: plant.ticks(n) for n in plant.q}), self.manifest_extras())
            if "phone" in names:
                owner.publish_phone(np.zeros((24, 32, 3), np.uint8))
        return publish


def build_head_rig(workdir: Path, sim_factory, mapping: HeadMapping, start_head: dict, *, arm_ticks=None,
                   head=True, rate_ticks_s=200.0, owner_period_s=0.05):
    """The deployed owner (--head scope unless head=False) over a kinematic plant whose head carries a simulated OAK.

    Arm maps: zero tick 2047, sign +1 (the owner-accepted feetech_degrees_v1 maps) over the saved calibration ranges in
    profiles/fold-joint-maps. `sim_factory(arm_maps, mapping) -> HeadOakSim`. Arms start at `arm_ticks` (default: the
    measurement pose of carton.head_pose) and stay released, i.e. still."""
    from carton import head_pose
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=False)
    saved = json.loads((SOFTWARE / "profiles/fold-joint-maps/calibration-2026-10-08.json").read_text())
    ranges = {n: (saved[n]["range_min"], saved[n]["range_max"]) for n in OWNER_JOINTS}
    calibration = fake_calibration(ranges)
    for n in HEAD_AND_WHEELS[:2]:
        calibration[n].update(range_min=saved[n]["range_min"], range_max=saved[n]["range_max"])
    maps, map_paths = write_joint_maps(workdir / "joint-maps", calibration, {n: (2047.0, 1) for n in OWNER_JOINTS},
                                       evidence="SIMULATION ONLY: fake head-pose rig")
    arm_ticks = dict(arm_ticks or head_pose.measurement_pose_ticks(maps))
    for n in OWNER_JOINTS:
        arm_ticks.setdefault(n, (ranges[n][0] + ranges[n][1]) // 2)
    plant = KinematicPlant({**arm_ticks, **start_head}, rate_ticks_s)
    sim = sim_factory(maps, mapping)
    clock = VirtualClock()
    fake = FakeOwner(plant, calibration, workdir / "robot", clock=clock, owner_period_s=owner_period_s,
                     camera_publisher=sim.publisher(plant), head=head)
    rig = SimRig(clock, fake.sleep, None, None, maps, plant, fake, calibration)
    rig.map_paths, rig.sim = map_paths, sim
    return rig
