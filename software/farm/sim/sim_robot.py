"""Simulated XLeRobot behind the pilot chat server's exact tool API (MuJoCo, no hardware).

``SimRobot`` stands in for ``pilot/chat_server.py``'s ``Robot`` (``catalog()``, ``get(path)``,
``call(name, args, request_id)``) so an LLM supervisor can be benchmarked on a box-grab task without the
real robot. It speaks the paddle-success-v1 pickup profile of the real server (the qwen-bridge owner in
software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/): the same tool schemas (captured
read-only from the live server into assets/sim_robot/real_tool_catalog.json), the same argument checks
and refusal wording, the same result shapes, the same motion profile (<=341-tick legs, 40-tick commandable
margin, 96-tick following-error fault that releases every motor, bounded settle corrections, 10-tick
contact-mode gripper closure, <=0.02 m/s base pulses).

Physics: the vendored upstream MJCF made fixed-base exactly like the twin (``xlerobot_twin._scene_xml``),
plus a table and a box (``farm.sim.box_scene.build_scene_xml`` when present, else the private fallback
``_fallback_scene_xml`` here). Encoder ticks <-> joint angles use the twin's ``feetech_degrees_v1``
mapping (``xlerobot_twin.motor_angles``, sampled to a per-motor linear map, never reimplemented) and the
calibration ranges from the saved real state sample. Enabled motors are PD servos (torque actuators,
``KP_ARM``/``KP_GRIP``, saturating at the STS3215's 2.94 N m); released motors are passive with gear friction
and sag slowly. A physics thread steps at 500 Hz; all MuJoCo objects are touched only under ``world.lock``.

Where the simulation knowingly differs from the real owner is listed in ``APPROXIMATIONS`` at the bottom.

Usage (bench)::

    from farm.sim.sim_robot import SimRobot
    robot = SimRobot(real_time=False)          # seed=0; scene from farm.sim.box_scene when importable
    catalog = robot.catalog()                  # {'tools': [...], 'metadata': {'stop_tool': 'robot_stop', ...}}
    robot.call('robot_get_state', {'fresh': False})
    robot.call('robot_set_motor_enable', {'names': [...six left joints...], 'enabled': True})
    robot.call('robot_move_joint_targets', {'arm': 'left', 'positions': {...}, 'duration_s': 6})
    robot.score(); robot.snapshot(); robot.reset(seed=1); robot.close()

Run with PYTHONPATH=<this software dir> and MUJOCO_GL=cgl on macOS. Needs mujoco and numpy; the camera tools need
farm.sim.sim_cameras (PIL/cv2) and answer 'cameras not available in this build' without it.
"""
from __future__ import annotations

import inspect
import json
import math
import random
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from farm.sim import xlerobot_twin as twin

ASSETS = Path(__file__).resolve().parent / 'assets' / 'sim_robot'
STATE_SAMPLE = ASSETS / 'real_state_sample.json'
CATALOG_SAMPLE = ASSETS / 'real_tool_catalog.json'

EXECUTION_PROFILE = 'paddle-success-v1'
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
HEAD_NAMES = ('head_motor_1', 'head_motor_2')
WHEELS = ('base_left_wheel', 'base_right_wheel')
MOTION_TOOLS = ('robot_move_joint_targets', 'robot_move_head', 'robot_set_gripper', 'robot_set_motor_enable',
                'robot_move_motor_targets', 'robot_move_path', 'robot_halt_motion', 'robot_move_base',
                'robot_stream_joint_targets', 'robot_hold_here')   # the last two: owner stream mode, not simulated
IMPLEMENTED_TOOLS = ('robot_list_motors', 'robot_set_motor_enable', 'robot_get_state', 'robot_get_cameras',
                     'robot_get_clip', 'robot_get_capabilities', 'robot_get_depth', 'robot_stop',
                     'robot_move_joint_targets', 'robot_move_path', 'robot_get_motion', 'robot_halt_motion',
                     'robot_set_gripper', 'robot_move_base')
CAMERA_TOOLS = ('robot_get_cameras', 'robot_get_clip', 'robot_get_depth')

# Pickup-profile constants (paddle_joint_executor.py / paddle_segments.py / gemma_robot_tools.py).
ENVELOPE = 96
MARGIN = 40
EDGE = 4
SEGMENT = 341
SEGMENT_PIECE = 280
MAX_WAYPOINTS = 24
STEP, CONTACT_STEP = 40, 10
CONTACT_INTERVAL = 1.5
CORRECTION_STEP = 40
CORRECTION_ROOM = 90
MAX_OVERDRIVE = 57
MAX_CORRECTIONS = 3
CORRECTION_BUDGET_S = 5
CONTACT_LOAD = 600
CONTACT_HALT_LOAD = 350
CONTACT_PUSH_TICKS = 50
GRIPPER_CONTACT_LAG = 40       # real contact_stop: jaw quiet 0.3 s and >= 40 ticks behind its goal
GRIPPER_CLOSE_CHUNK = 300
MAX_PATH_WAYPOINTS = 12

# Base pulse constants (wheel_pulse_executor.py).
WHEEL_RADIUS_M = 0.0635
WHEELBASE_M = 0.45
TICKS_PER_REV = 4096
MAX_WHEEL_M_S = 0.02
MAX_DURATION_S = 3.0
BRAKE_SETTLE_S = 0.3

# Simulation constants.
PHYSICS_HZ = 500
NOSLIP_ITERATIONS = 5
POLL_S = 0.1                   # owner telemetry rate the guards are evaluated at
FAST_HOOK_INTERVAL_S = 0.5     # step hooks (camera recording) run at most this often in sim time while fast-forwarding
TICK_RAD = 2 * math.pi / 4096
KP_ARM, KV_ARM = 20.0, 0.6     # N m/rad, N m s/rad
KP_GRIP, KV_GRIP = 6.0, 0.15
FORCE_LIMIT = 2.94             # N m, the vendored model's STS3215 forcerange
LOAD_PER_LAG_TICK = 3.6        # Present_Load per tick of servo error (real contacts: 300-400 at ~96 ticks)
RELEASED_DAMPING, RELEASED_FRICTION = 2.0, 0.4   # released joints: gear friction, slow sag
ENABLED_DAMPING, ENABLED_FRICTION = 0.6, 0.052   # the model's own sts3215 defaults
MIN_LEG_S = 2.0
STILL_TICKS_PER_S = 30         # 'still' = under 3 ticks per 0.1 s poll (the real test is 3 units of Present_Velocity)
FOLDED_DEG = {'shoulder_pan': 0.0, 'shoulder_lift': -78.0, 'elbow_flex': 82.0, 'wrist_flex': 60.0, 'wrist_roll': 0.0}
# Jaws meet at raw range_min + this many ticks: 1355 on the left gripper, whose saved range starts at 1273. Measured on
# 8-9 October: an empty close to 1340 settles at 1355-1359 (the pads meet) and the owner-confirmed flap pinch stopped at
# 1378 (a 3 mm flap is about 25 ticks). Below it the jaws press together; the model's jaw range (-21.5..100 deg) then
# spans ticks 1355..2738 (the saved open end is 2821, 7 deg more). Applied as a twin joint map (grippers only).
GRIPPER_CLOSED_OFFSET_TICKS = 82
# Per arm (9 October): the right gripper's empty pads meet at 1344-1349 (saved range_min 1269 + 79) with ~150 load.
GRIPPER_CLOSED_OFFSET = {'left': 82, 'right': 79}
# Fidelity options that make the sim at least as hard as the real robot on 9 October (SimRobot(fidelity={...}) overrides):
# - meet_jitter_ticks: per closing call the reported meeting point moves by U(lo, hi) ticks (real empty closes read
#   1355-1359 on the left, 1344-1360 on the right);
# - right_grip_sticks: the right gripper sticks mid-travel (a close from above 1700 stops at 1520-1610 with probability
#   p_close; an open stops at 1500-1950 with probability p_open; resending an open after an open stall leaves the jaw
#   stuck with probability p_reopen, which trips the 1 s no-progress guard and releases everything, as twice on 9 Oct);
# - lift_bias_deg: the physical shoulder_lift sits this many degrees from what its ticks say (the arm model reads a few
#   cm high near the box: a claw the model put at 73-75 cm met nothing below the 77 cm rim); positive = claw lower;
# - roll_offset_deg: the physical wrist_roll sits this far from the twin's mapping, so the unrolled jaws open to the
#   robot's left and right as on the real robot (the twin has them opening up and down);
# - plastic (optional): overrides for box_scene.PLASTIC, e.g. a crease that is harder to set than the default.
FIDELITY = {'meet_jitter_ticks': {'left': (0, 4), 'right': (-4, 10)},
            'right_grip_sticks': {'p_close': 0.5, 'p_open': 0.4, 'p_reopen': 0.7, 'close_band': (1520, 1610),
                                  'open_band': (1500, 1950)},
            'lift_bias_deg': {'left': 5.0, 'right': 5.0},
            'roll_offset_deg': {'left': 90.0, 'right': 90.0},
            'head_tilt_deg': 45.0}
PAD_Y_M = -0.035               # jaw frame (Fixed_Jaw: the jaw runs along -y from the hinge at -0.024 to the tip at -0.106):
                               # contacts beyond this are the jaws' gripping length (a flap edge entering the jaws at pitch -35
                               # meets them about 5 cm from the tip); nearer the hinge it is the gripper body and camera mount
FLAP_FOLDED_DEG = 75.0         # score()['flap_folded'] threshold (farm.sim.box_scene.FLAP_FOLDED_DEG)
FOLD_PUSH_ALLOWANCE_DEG = 10.0 # score()['fold_by_pinch']: at most this much of the flap's turn may happen under a non-pad contact
PRESENT_VOLTAGE = 120
PRESENT_TEMPERATURE = 35
RANGE_SEMANTICS = ('raw_calibration_ranges and motor range are saved hardware limits, not command targets; use '
                   'commandable_ranges (inclusive, 40-tick margin) for targets. A released joint resting outside '
                   'commandable_ranges is normal (it sags under gravity); enable holds it there and the next target '
                   'simply has to be inside.')
MOTION_NOTE = ('Motion is running. Monitor with robot_get_motion and cameras; robot_halt_motion stops and holds; a new '
               'move with replace=true changes course.')
HALT_NOTE = 'Holding where it stopped (wheels brake, then release). Nothing was released; send a new move to continue.'
SHORT_REASON = ('Joint came to rest short of its target after bounded goal corrections; motors are holding at the '
                'measured position. Re-plan from fresh readbacks or STOP.')
CONTACT_NOTE = (f'Load >= {CONTACT_HALT_LOAD} on a joint lagging >= {CONTACT_PUSH_TICKS} ticks behind its command: treated '
                'as contact with something (there is no self-collision model). That joint stopped pushing and holds '
                'where it is; nothing was released.')
GRIPPER_NOTE = ('The jaw stopped before the target and is holding (often an object between the jaws). Look at the '
                'wrist camera before the next step.')
READ_ONLY_MESSAGE = ('UNSUPPORTED_OWNER_SCOPE: these motors are read-only and are never powered: {names}. The head '
                     'cannot be moved (aim cameras by moving the arm instead); the wheels move only through '
                     'robot_move_base, which needs no enable. Enable only arm joints.')
ODOMETRY_NOTE = 'wheel encoder estimate only; slip and floor contact unverified'


def tolerance(name):
    return 30 if name.endswith('gripper') else 57


def wheel_ticks_per_s(linear_m_s, angular_rad_s):
    """Differential drive; the left wheel is mounted mirrored, so forward is left negative, right positive."""
    scale = TICKS_PER_REV / (2 * math.pi * WHEEL_RADIUS_M)
    left = linear_m_s - angular_rad_s * WHEELBASE_M / 2
    right = linear_m_s + angular_rad_s * WHEELBASE_M / 2
    return {'base_left_wheel': -round(left * scale), 'base_right_wheel': round(right * scale)}


def check_base_request(c):
    """wheel_pulse_executor.check_request, verbatim wording."""
    lin, ang, duration = c.get('linear_m_s'), c.get('angular_rad_s', 0), c.get('duration_s')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (lin, ang, duration)):
        raise ValueError('Finite linear_m_s, angular_rad_s and duration_s required')
    if not 0 < duration <= MAX_DURATION_S:
        raise ValueError(f'Base pulse duration must be in (0,{MAX_DURATION_S}] s')
    limit = MAX_WHEEL_M_S * TICKS_PER_REV / (2 * math.pi * WHEEL_RADIUS_M) + .5
    commands = wheel_ticks_per_s(lin, ang)
    if any(abs(v) > limit for v in commands.values()):
        raise ValueError(f'Each wheel is limited to {MAX_WHEEL_M_S} m/s; lower linear_m_s or angular_rad_s')
    if not any(commands.values()):
        raise ValueError('Base pulse needs a nonzero linear_m_s or angular_rad_s')
    return commands


def wrap(delta):
    return ((delta + 2048) % 4096) - 2048


def load_calibration(calibration=None):
    """{motor: (min_ticks, max_ticks)} for all 16 motors, from the saved real sample unless given."""
    if calibration is None:
        calibration = json.loads(STATE_SAMPLE.read_text())['result']['raw_calibration_ranges']
    out = {}
    for name, value in calibration.items():
        rng = twin._range(value)
        if rng is None:
            raise ValueError(f'calibration {name}: need (min_ticks, max_ticks) with max > min')
        out[name] = (int(rng[0]), int(rng[1]))
    missing = [n for n in list(twin.JOINT_TABLE) + list(WHEELS) if n not in out]
    if missing:
        raise ValueError(f'calibration lacks {", ".join(missing)}')
    return out


# ---------------------------------------------------------------- segments (paddle_segments.py, ported)

def paddle_target_segments(targets, rows, ranges):
    travel = {}
    for name, target in targets.items():
        start = rows.get(name, {}).get('Present_Position')
        if type(start) is not int:
            raise ValueError('Pickup start encoder unavailable for ' + name + '; refresh robot_get_state')
        if abs(target - start) > 2:
            travel[name] = (start, target)
    closing = {n: v for n, v in travel.items() if n.endswith('gripper') and v[1] < v[0]}
    moving = {n: v for n, v in travel.items() if n not in closing}
    segments = []

    def band(n, q):
        lo, hi = ranges[n]
        return min(max(q, lo + MARGIN), hi - MARGIN)
    for group in (moving, closing):
        if not group:
            continue
        spans = {n: 1 if abs(t - s) <= SEGMENT else -(-abs(t - s) // SEGMENT_PIECE) for n, (s, t) in group.items()}
        count = max(spans.values())
        for i in range(1, count + 1):
            segments.append({n: (t if i == count else band(n, s + round((t - s) * i / count))) for n, (s, t) in group.items()})
    return segments


def expand_path(points, start):
    names = sorted(set(start) & {n for p in points for n in p})
    previous = {n: start[n] for n in names}
    out = []
    for point in points:
        target = {**previous, **{n: point[n] for n in point if n in previous}}
        pieces = max(1, max(0 if abs(target[n] - previous[n]) <= SEGMENT else -(-abs(target[n] - previous[n]) // SEGMENT_PIECE) for n in names))
        for i in range(1, pieces + 1):
            out.append({n: previous[n] + round((target[n] - previous[n]) * i / pieces) for n in names})
        previous = target
    return out


# ---------------------------------------------------------------- scene

ROBOT_ORIGIN_MODEL = (-0.09, 0.0, 0.0)   # the twin's robot-frame origin in model coordinates (checked at load)


def _robot_to_model(forward, left, up):
    ox, oy, oz = ROBOT_ORIGIN_MODEL
    return (ox - forward, oy - left, oz + up)


def _lookat_xyaxes(pos, target, up=(0, 0, 1)):
    """MuJoCo camera xyaxes (x right, y up; it looks along -z) for a camera at pos looking at target."""
    import numpy as np
    pos, target, up = (np.array(v, dtype=float) for v in (pos, target, up))
    z = pos - target
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return ' '.join(f'{v:.6f}' for v in list(x) + list(y))


def _fallback_scene_xml(box_forward_m=0.42, box_left_m=0.21, table_top_m=0.70, box_size_m=(0.20, 0.15, 0.11), seed=0):
    """Private stand-in for farm.sim.box_scene.build_scene_xml: the twin's fixed-base robot plus a table, a box
    (free joint 'box_free', thin 'flap' geom standing on its near top edge) and cameras oak/left_wrist/right_wrist/phone."""
    rng = random.Random(seed)
    jitter = (rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
    root = ET.fromstring(twin._scene_xml(twin.VENDORED_MODEL))
    world = root.find('worldbody')
    bx, by, bz = box_size_m
    table = ET.SubElement(world, 'body', name='table', pos=' '.join(f'{v:.5f}' for v in _robot_to_model(0.58, 0.0, table_top_m - 0.02)))
    ET.SubElement(table, 'geom', name='table_top', type='box', size='0.30 0.45 0.02', rgba='0.62 0.47 0.32 1',
                  contype='1', conaffinity='1', friction='1 0.005 0.0001')
    for lx, ly in ((0.27, 0.42), (0.27, -0.42), (-0.27, 0.42), (-0.27, -0.42)):
        ET.SubElement(table, 'geom', type='box', size=f'0.02 0.02 {(table_top_m - 0.04) / 2:.4f}',
                      pos=f'{lx} {ly} {-(table_top_m - 0.04) / 2 - 0.02:.4f}', rgba='0.5 0.4 0.3 1', contype='0', conaffinity='0')
    box = ET.SubElement(world, 'body', name='box', pos=' '.join(f'{v:.5f}' for v in _robot_to_model(
        box_forward_m + jitter[0], box_left_m + jitter[1], table_top_m + bz / 2 + 0.002)))
    ET.SubElement(box, 'freejoint', name='box_free')
    ET.SubElement(box, 'geom', name='box_body', type='box', size=f'{bx / 2:.4f} {by / 2:.4f} {bz / 2:.4f}', rgba='0.80 0.68 0.42 1',
                  mass='0.12', contype='1', conaffinity='1', friction='1 0.005 0.0001')
    # Flap: a thin plate standing up along the near (robot-facing, model +x) top edge, 5 cm tall.
    ET.SubElement(box, 'geom', name='flap', type='box', size=f'0.0025 {by / 2 - 0.005:.4f} 0.025', pos=f'{bx / 2 - 0.0025:.4f} 0 {bz / 2 + 0.025:.4f}',
                  rgba='0.9 0.8 0.55 1', mass='0.01', contype='1', conaffinity='1', friction='1.5 0.005 0.0001')
    # Cameras. oak: ROS optical frame (x right, y down, z out) == MuJoCo camera (x right, y up, looks along -z) with y, z flipped.
    for body in world.iter('body'):
        if body.get('name') == twin.HEAD_CAMERA_BODY:
            ET.SubElement(body, 'camera', name='oak', pos='0 0 0', xyaxes='0 -1 0 0 0 1', fovy='55')
        if body.get('name') in ('Fixed_Jaw', 'Fixed_Jaw_2'):
            # Wrist camera on the jaw's camera mount, looking down the jaw (-y) and a little toward the tips.
            name = 'left_wrist' if body.get('name') == 'Fixed_Jaw' else 'right_wrist'
            ET.SubElement(body, 'camera', name=name, pos='0 -0.02 0.05', xyaxes=_lookat_xyaxes((0, -0.02, 0.05), (0, -0.12, 0.0), up=(0, 0, 1)), fovy='70')
    phone_pos = _robot_to_model(0.35, 1.1, 1.25)
    ET.SubElement(world, 'camera', name='phone', pos=' '.join(f'{v:.4f}' for v in phone_pos),
                  xyaxes=_lookat_xyaxes(phone_pos, _robot_to_model(0.4, 0.15, 0.78)), fovy='60')
    return ET.tostring(root, encoding='unicode')


def _scene_with_actuators(xml):
    """Add one torque actuator per arm motor (the PD runs in Python so enable/release is per motor)."""
    root = ET.fromstring(xml)
    for el in root.findall('actuator'):
        root.remove(el)
    act = ET.SubElement(root, 'actuator')
    for motor, (joint, _, _) in twin.JOINT_TABLE.items():
        if motor.startswith('head'):
            continue
        ET.SubElement(act, 'motor', name=motor, joint=joint, ctrllimited='true', ctrlrange=f'-{FORCE_LIMIT} {FORCE_LIMIT}')
    option = root.find('option')
    if option is None:
        option = ET.Element('option')
        root.insert(0, option)
    option.set('timestep', f'{1.0 / PHYSICS_HZ:.6f}')
    option.set('noslip_iterations', str(NOSLIP_ITERATIONS))   # pinched objects creep out of soft contacts without it
    return ET.tostring(root, encoding='unicode')


def build_scene(seed=0, preset=None):
    """The box scene from farm.sim.box_scene when present (``preset``: a box_scene.PRESETS key, default the real
    9 October carton), else the private fallback."""
    try:
        from farm.sim.box_scene import build_scene_xml
    except ImportError:
        return _fallback_scene_xml(seed=seed), 'fallback'
    return build_scene_xml(seed=seed, preset=preset), 'box_scene'


# ---------------------------------------------------------------- world

class World:
    """What the camera module gets: ``.model .data .lock .now()`` and ``step_hook(fn)`` (fn(world) or fn()) called after
    every physics step under the lock. ``fast_forward`` is True while a blocking call is fast-forwarding sim time."""

    def __init__(self):
        self.model = None
        self.data = None
        self.lock = threading.RLock()
        self.fast_forward = False
        self._hooks = []
        self._last_fast_hook = -1e9

    def now(self):
        return float(self.data.time) if self.data is not None else 0.0

    def step_hook(self, fn):
        try:
            params = [p for p in inspect.signature(fn).parameters.values()
                      if p.default is inspect.Parameter.empty and p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
            takes_world = len(params) >= 1
        except (TypeError, ValueError):
            takes_world = True
        self._hooks.append((fn, takes_world))
        return fn

    def _run_hooks(self):
        if self.fast_forward and self._hooks:
            # Camera hooks render; while fast-forwarding, give them the step only every FAST_HOOK_INTERVAL_S of sim time.
            now = self.now()
            if now - self._last_fast_hook < FAST_HOOK_INTERVAL_S - 1e-9:
                return
            self._last_fast_hook = now
        for fn, takes_world in self._hooks:
            try:
                fn(self) if takes_world else fn()
            except Exception as error:  # noqa: BLE001 - a camera hook must never stop physics
                sys.stderr.write(f'sim_robot step hook {fn!r} failed: {error}\n')


class _Fault(RuntimeError):
    """An owner fault: everything is released."""


class _Motor:
    def __init__(self, name, joint, qadr, dadr, actuator, a, b, lo, hi, kp, kv):
        self.name, self.joint, self.qadr, self.dadr, self.actuator = name, joint, qadr, dadr, actuator
        self.a, self.b = a, b          # q (rad) = a + b * tick
        self.lo, self.hi = lo, hi      # saved range (ticks)
        self.kp, self.kv = kp, kv
        self.enabled = False
        self.goal = None               # held/commanded goal, ticks (int)
        self.force = 0.0               # last applied torque
        self.present = 0               # last polled present position (ticks)
        self.velocity = 0              # ticks/s
        self.load = 0
        self.moving = 0

    def tick_to_q(self, tick):
        return self.a + self.b * tick

    def q_to_tick(self, q):
        return (q - self.a) / self.b


# ---------------------------------------------------------------- arm motion (PaddleJointExecutor, ported)

class _ArmMotion:
    def __init__(self, robot, command, now):
        self.robot = robot
        path = command.get('waypoints') is not None
        legs = command.get('waypoints') if path else [command.get('positions')]
        duration = command.get('duration_s')
        if not isinstance(legs, list) or not 1 <= len(legs) <= MAX_WAYPOINTS:
            raise ValueError(f'Pickup path needs 1..{MAX_WAYPOINTS} waypoints')
        joints = list(legs[0]) if isinstance(legs[0], dict) else []
        for p in legs:
            if not isinstance(p, dict) or not p or set(p) != set(joints):
                raise ValueError('Pickup command joints must match the executor joints')
            if any(not n.startswith(('right_arm_', 'left_arm_')) or type(t) is not int for n, t in p.items()):
                raise ValueError('Integer arm-joint target required')
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 < duration <= (60 if path else 25):
            raise ValueError('Finite duration (0,25] required (paths: (0,60])')
        self.joints = joints
        motors = robot.motors
        current = {n: motors[n].present for n in joints}
        self.ranges = {n: (motors[n].lo, motors[n].hi) for n in joints}
        goals = {}
        for n in joints:
            q = current[n]
            lo, hi = self.ranges[n]
            if not lo + EDGE <= q <= hi - EDGE:
                raise ValueError(n + ': current position outside saved range')
            g = motors[n].goal if motors[n].goal is not None else q
            if type(g) is not int or abs(g - q) > ENVELOPE or not lo + EDGE <= g <= hi - EDGE:
                raise ValueError(n + ': pickup previous held goal outside envelope')
            goals[n] = g
        previous = dict(current)
        for i, p in enumerate(legs):
            for n, t in p.items():
                lo, hi = self.ranges[n]
                if not lo + MARGIN <= t <= hi - MARGIN or abs(t - previous[n]) > SEGMENT:
                    raise ValueError(f'{n}: waypoint {i + 1} must be 40 ticks inside the saved range and at most 341 ticks from the previous point')
            previous = p
        if not path and not all(2 < abs(t - current[n]) for n, t in legs[0].items()):
            raise ValueError('Each joint in a pickup move must travel 3..341 ticks')
        if path and not any(abs(p[n] - current[n]) > 2 for p in legs for n in joints):
            raise ValueError('Pickup path does not move any joint')
        self.contact = any(n.endswith('gripper') and t < current[n] for n, t in legs[0].items())
        if self.contact and (len(legs[0]) != 1 or path):
            raise ValueError('Gripper closure must be commanded alone, not in a path')
        step = CONTACT_STEP if self.contact else STEP
        origin = goals
        self.leg_steps = []
        for p in legs:
            self.leg_steps.append(max(1, max(math.ceil(max(abs(t - origin[n]), abs(t - current[n]) if p is legs[0] else 0) / step) for n, t in p.items())))
            origin = p
        self.steps = sum(self.leg_steps)
        self.interval = CONTACT_INTERVAL if self.contact else max(.4, float(duration) / self.steps)
        base = self.steps * self.interval + 3
        if base > (55 if self.contact else 80 if path else 28):
            raise ValueError('Pickup motion exceeds API completion deadline; shorten it or its duration')
        self.duration = base - 3
        self.deadline = base + (0 if self.contact else CORRECTION_BUDGET_S)
        if not self.contact:  # the sim ramps linearly with a floor of MIN_LEG_S per leg; the deadline follows
            self.deadline += sum(max(0.0, MIN_LEG_S - s * self.interval) for s in self.leg_steps)
        self.legs = [dict(p) for p in legs]
        self.path = path
        self.final_target = dict(legs[-1])
        self.goal = dict(goals)
        self.start_positions = dict(current)
        self.bias = dict.fromkeys(joints, 0)
        self.corrections = dict.fromkeys(joints, 0)
        self.exhausted = {n: n.endswith('gripper') for n in joints}
        self.stable = dict.fromkeys(joints, 0)
        self.still = dict.fromkeys(joints, 0)
        self.last_q = dict(current)
        self.loaded = dict.fromkeys(joints, 0)
        self.correction_from = {}
        self.possible_contact = set()
        self.progress_q = dict(current)
        self.progress_at = dict.fromkeys(joints, now)
        self.started = now
        self.last_write = now
        self.quiet_since = None
        self.first_step = True
        self.command_id = command['id']
        self.active = True
        self.result = None
        self.outcome = None
        self.done = threading.Event()
        self.diagnostics = {}
        self.leg = 0
        self.ramp_done = False
        self.leg_start_goal = dict(goals)
        self.leg_started = now
        self.leg_time = 0.0
        self.set_leg(0, now)
        self.diagnostics = {'leg': 1, 'legs': len(self.legs), 'elapsed_s': 0.0, 'final_targets': self.final_target,
                            'joints': {n: {'goal_ticks': self.goal[n], 'current_ticks': current[n], 'target_ticks': self.targets[n],
                                           'following_error_ticks': current[n] - self.goal[n], 'stable_samples': 0, 'still_samples': 0,
                                           'overdrive_ticks': 0, 'corrections': 0, 'load': motors[n].load} for n in joints},
                            'contact_stop': False}

    # -- ramp
    def set_leg(self, i, now):
        self.leg = i
        self.targets = dict(self.legs[i])
        self.leg_start_goal = dict(self.goal)
        self.leg_started = now
        if not self.contact:
            self.leg_time = max(MIN_LEG_S, self.leg_steps[i] * self.interval)

    def aim(self, n):
        return self.targets[n] + self.bias[n]

    def advance_ramp(self, now):
        """Every physics step (non-contact): goals move linearly through the legs; intermediate waypoints are passed
        without stopping."""
        if self.contact or self.ramp_done:
            return
        frac = min(1.0, (now - self.leg_started) / self.leg_time) if self.leg_time > 0 else 1.0
        for n in self.joints:
            start = self.leg_start_goal[n]
            self.goal[n] = int(round(start + (self.targets[n] - start) * frac))
        if frac >= 1.0:
            if self.leg < len(self.legs) - 1:
                self.set_leg(self.leg + 1, now)
            else:
                self.ramp_done = True
                self.last_write = now

    def halt(self, current):
        self.legs = [dict(self.goal)]
        self.targets = dict(self.goal)
        self.bias = dict.fromkeys(self.joints, 0)
        self.ramp_done = True
        return self.finish(current, 'halted')

    def correct(self, n, q):
        t = self.targets[n]
        lo, hi = self.ranges[n]
        g = self.goal[n]
        want = t + max(-MAX_OVERDRIVE, min(MAX_OVERDRIVE, self.bias[n] + t - q))
        want = max(q - CORRECTION_ROOM, min(q + CORRECTION_ROOM, want))
        want = max(g - CORRECTION_STEP, min(g + CORRECTION_STEP, want))
        want = max(lo + MARGIN, min(hi - MARGIN, want))
        if want == g or (want - g) * (t - q) <= 0:
            self.exhausted[n] = True
            return None
        self.corrections[n] += 1
        self.exhausted[n] = self.corrections[n] >= MAX_CORRECTIONS
        self.goal[n] = want
        self.bias[n] = want - t
        return want

    def finish(self, current, outcome):
        if getattr(self, 'stalled', False) and outcome == 'endpoint_settled':
            outcome = 'settled_short'   # the jaw stuck mid-travel (SimRobot.FIDELITY right_grip_sticks)
        self.active = False
        self.outcome = outcome
        self.robot.last_completed = self.command_id
        for n in self.joints:  # the owner keeps holding the last commanded goals
            self.robot.motors[n].goal = self.goal[n]
        residual = {n: current[n] - self.final_target[n] for n in self.joints}
        self.result = {'completed': self.command_id, 'phase': 'holding', 'direct_actual_positions': dict(current),
                       'grasp_verified': False, 'closure_outcome': outcome, 'endpoint_reached': outcome == 'endpoint_settled',
                       'settle_residual_ticks': residual, 'possible_contact_joints': sorted(self.possible_contact),
                       'contact': None, 'direct_settle_diagnostics': self.diagnostics}
        self.done.set()
        return self.result

    def poll(self, now, rows):
        """Owner tick at the telemetry rate: guards, contact, settle, corrections, contact-mode stepping."""
        current = {n: rows[n]['Present_Position'] for n in self.joints}
        for n in self.joints:
            if abs(current[n] - self.goal[n]) > ENVELOPE:
                raise _Fault('Pickup following error exceeds96ticks: ' + n)
        if now - self.started > self.deadline:
            raise _Fault('Pickup segment failed to settle before deadline')
        quiet = {}
        for n in self.joints:
            q = current[n]
            row = rows[n]
            still = abs(row['Present_Velocity']) < STILL_TICKS_PER_S and abs(q - self.last_q[n]) <= 3
            quiet[n] = still and (row['Moving'] == 0 or self.bias[n] != 0)
            self.last_q[n] = q
            self.still[n] = self.still[n] + 1 if still else 0
            self.stable[n] = self.stable[n] + 1 if quiet[n] and self.goal[n] == self.aim(n) and abs(q - self.targets[n]) <= tolerance(n) else 0
        c = self.joints[0]
        if self.contact:
            self.quiet_since = (self.quiet_since if self.quiet_since is not None else now) if quiet[c] else None
        contact_stop = self.contact and self.quiet_since is not None and now - self.quiet_since >= .3 and current[c] - self.goal[c] >= GRIPPER_CONTACT_LAG
        self.diagnostics = {'leg': self.leg + 1, 'legs': len(self.legs), 'elapsed_s': round(now - self.started, 2), 'final_targets': self.final_target,
                            'joints': {n: {'goal_ticks': self.goal[n], 'current_ticks': current[n], 'target_ticks': self.targets[n],
                                           'following_error_ticks': current[n] - self.goal[n], 'stable_samples': self.stable[n], 'still_samples': self.still[n],
                                           'overdrive_ticks': self.bias[n], 'corrections': self.corrections[n], 'load': rows[n]['Present_Load']} for n in self.joints},
                            'contact_stop': contact_stop}
        if not self.contact:
            hits = {}
            for n in self.joints:
                if n.endswith('gripper'):
                    continue
                load = abs(rows[n]['Present_Load'])
                self.loaded[n] = self.loaded[n] + 1 if load >= CONTACT_HALT_LOAD else 0
                if self.loaded[n] >= 2 and abs(self.goal[n] - current[n]) >= CONTACT_PUSH_TICKS:
                    hits[n] = load
            if hits:
                for n in hits:
                    lo, hi = self.ranges[n]
                    self.goal[n] = max(lo + EDGE, min(hi - EDGE, current[n]))
                self.ramp_done = True
                out = self.finish(current, 'contact_halt')
                out['contact'] = {n: {'load': hits[n], 'position_ticks': current[n]} for n in hits}
                out['contact_note'] = CONTACT_NOTE
                return out
        final = self.leg == len(self.legs) - 1
        settled = final and all(self.stable[n] >= 3 for n in self.joints)
        if contact_stop:
            out = self.finish(current, 'contact_halt')
            out['gripper_result'] = {'holding': True, 'position_ticks': current[c], 'load': rows[c]['Present_Load']}
            return out
        if settled and (now - self.last_write >= self.interval if not self.contact else self.quiet_since is not None and now - self.quiet_since >= .3):
            return self.finish(current, 'endpoint_settled')
        if self.contact and final and self.goal[c] == self.targets[c] and self.quiet_since is not None and now - self.quiet_since >= .3:
            return self.finish(current, 'settled_short')
        ramp_done = self.ramp_done and all(self.goal[n] == self.aim(n) for n in self.joints)
        if not self.contact and final and ramp_done and now - self.last_write >= self.interval and all(self.still[n] >= 3 for n in self.joints):
            pending = [n for n in self.joints if self.stable[n] < 3]
            for n in pending:
                load = abs(rows[n]['Present_Load'])
                if not self.exhausted[n] and (load >= CONTACT_LOAD or (n in self.correction_from and abs(current[n] - self.correction_from[n]) < 3)):
                    self.exhausted[n] = True
                    self.possible_contact.add(n)
            writes = {n: g for n in pending if not self.exhausted[n] and (g := self.correct(n, current[n])) is not None}
            self.correction_from.update({n: current[n] for n in writes})
            if writes:
                self.last_write = now
                for n in writes:
                    self.stable[n] = self.still[n] = 0
                return None
            if all(self.exhausted[n] for n in pending):
                outside = any(abs(current[n] - self.targets[n]) > tolerance(n) for n in self.joints)
                return self.finish(current, 'settled_short' if outside else 'endpoint_settled')
        for n in self.joints:
            q = current[n]
            if abs(q - self.progress_q[n]) >= 8:
                self.progress_q[n] = q
                self.progress_at[n] = now
            if n.endswith('gripper') and not self.contact and abs(q - self.goal[n]) > 30 and now - self.progress_at[n] > 1:
                raise _Fault('Pickup gripper no-progress guard')
        if self.contact:
            advance = self.goal[c] != self.targets[c] and (self.first_step or self.quiet_since is not None and now - self.quiet_since >= .3)
            if not self.first_step and not advance and now - self.last_write > CONTACT_INTERVAL:
                raise _Fault('Pickup closure did not become stationary')
            if advance:
                delta = self.aim(c) - self.goal[c]
                self.goal[c] += max(-CONTACT_STEP, min(CONTACT_STEP, delta))
                self.stable[c] = self.still[c] = 0
                self.last_write = now
                self.first_step = False
                self.quiet_since = None
        return None


class _BasePulse:
    def __init__(self, robot, command, now):
        self.commands = check_base_request(command)
        self.lin, self.ang, self.duration = command['linear_m_s'], command.get('angular_rad_s', 0), float(command['duration_s'])
        self.command_id = command['id']
        self.started = now
        self.phase = 'driving'
        self.stopped_at = None
        self.released = 0
        self.before = {n: robot.wheel_ticks[n] for n in WHEELS}
        self.travel = {n: 0.0 for n in WHEELS}
        self.active = True
        self.result = None
        self.done = threading.Event()

    def step(self, robot, dt, now):
        if self.phase == 'driving':
            if now - self.started >= self.duration:
                self.phase = 'braking'
                self.stopped_at = now
                return
            for n, v in self.commands.items():
                self.travel[n] += v * dt
                robot.wheel_ticks[n] = int(round(self.before[n] + self.travel[n])) % 4096
            robot._shift_world(-self.lin * dt, -self.ang * dt)

    def poll(self, robot, now):
        if self.phase == 'braking' and now - self.stopped_at >= BRAKE_SETTLE_S:
            self.phase = 'released'
        elif self.phase == 'released':
            self.released += 1
            if self.released >= 5:
                delta = {n: wrap(robot.wheel_ticks[n] - self.before[n]) for n in WHEELS}
                self.active = False
                robot.last_completed = self.command_id
                self.result = {'completed': self.command_id, 'phase': 'idle', 'base_drive_phase': 'done', 'base_result': {
                    'wheel_delta_ticks': delta,
                    'estimated_wheel_travel_cm': {n: abs(v) * 2 * math.pi * WHEEL_RADIUS_M / TICKS_PER_REV * 100 for n, v in delta.items()},
                    'pulse_s': self.stopped_at - self.started, 'stopped_early': None, 'released': True, 'settings_restored': True,
                    'odometry_note': ODOMETRY_NOTE}}
                self.done.set()


# ---------------------------------------------------------------- the robot

class SimRobot:
    """See the module docstring. ``real_time``: pace blocking calls (wait=true moves, gripper, base pulses) at 1x wall
    time; False fast-forwards them. Idle time and wait=false motions always run at 1x so monitoring works like on the
    real robot. ``calibration``: {motor: (min_ticks, max_ticks)} (default: the saved real sample)."""

    def __init__(self, scene_xml=None, seed=0, real_time=True, calibration=None, preset=None, fidelity=None):
        if sys.platform == 'darwin':
            import os
            os.environ.setdefault('MUJOCO_GL', 'cgl')
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.real_time = bool(real_time)
        self.seed = seed
        self.calibration = load_calibration(calibration)
        self._given_xml = scene_xml
        try:
            from farm.sim import box_scene as _bs
            self.preset = preset or _bs.DEFAULT_PRESET
            plastic = _bs.PRESETS.get(self.preset, {}).get('plastic')
            self.plastic = dict(_bs.PLASTIC) if plastic == 'default' else (dict(plastic) if isinstance(plastic, dict) else None)
        except ImportError:
            self.preset, self.plastic = preset, None
        # Fidelity: the real-robot difficulties (FIDELITY); the 'near7' preset keeps the 8 October behaviour.
        base = {} if self.preset == 'near7' else json.loads(json.dumps(FIDELITY))
        if fidelity is not None:
            base.update(fidelity)
        self.fidelity = base
        if self.plastic is not None and isinstance(base.get('plastic'), dict):
            self.plastic.update(base['plastic'])   # e.g. a stiffer crease: {'rate_per_s': 0.3, 'full_set_deg': 112}
        self.rng = random.Random(1000003 * (int(seed) if isinstance(seed, int) else 0) + 17)
        self.world = World()
        self.lock = self.world.lock
        self._command_lock = threading.Lock()
        self._fast = 0
        self._stop_thread = threading.Event()
        self._thread = None
        self.started = time.time()
        self.cancel_generation = 0
        self._requests = {}
        self._load_model(seed)
        self._thread = threading.Thread(target=self._run, name='sim-robot-physics', daemon=True)
        self._thread.start()

    # ---------------------------------------------------------------- model and state

    def _load_model(self, seed):
        mj, np = self.mj, self.np
        if self._given_xml is not None:
            xml, self.scene_source = self._given_xml, 'given'
        else:
            xml, self.scene_source = build_scene(seed, self.preset)
        model = mj.MjModel.from_xml_string(_scene_with_actuators(xml))
        data = mj.MjData(model)
        self.motors = {}
        jaw_deg = {}
        for motor, (joint, offset, _) in twin.JOINT_TABLE.items():
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0:
                raise RuntimeError(f'scene lacks joint {joint} for {motor}')
            if offset is None:
                jaw_deg[joint] = tuple(float(v) for v in np.degrees(model.jnt_range[jid]))
        ranges = {n: self.calibration[n] for n in twin.JOINT_TABLE}
        self.joint_map = {'validated': False, 'joints': {n: {'zero_tick': ranges[n][0] + GRIPPER_CLOSED_OFFSET.get(n.split('_arm_')[0], GRIPPER_CLOSED_OFFSET_TICKS), 'sign': 1}
                                                         for n in twin.JOINT_TABLE if n.endswith('gripper')}}
        # Motors in the owner's bus order (the calibration's order: left arm, head, right arm), as robot_get_state lists them.
        for motor in [n for n in self.calibration if n in twin.JOINT_TABLE]:
            joint, offset, _ = twin.JOINT_TABLE[motor]
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
            lo, hi = ranges[motor]
            # Sample the twin's mapping at both range ends: it is linear in ticks for every joint.
            _, _, m_lo = twin.motor_angles({motor: lo}, {motor: ranges[motor]}, self.joint_map, jaw_deg)
            _, _, m_hi = twin.motor_angles({motor: hi}, {motor: ranges[motor]}, self.joint_map, jaw_deg)
            q_lo, q_hi = math.radians(m_lo[joint]), math.radians(m_hi[joint])
            b = (q_hi - q_lo) / (hi - lo)
            a = q_lo - b * lo
            aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, motor)
            grip = motor.endswith('gripper')
            if '_arm_' in motor and not grip:
                # The vendored MJCF's joint ranges are narrower than the servos' calibrated travel (e.g. Rotation_R stops at
                # feetech -34 deg where the saved range reaches -120): the saved range IS the mechanical travel (calibration
                # drives each joint to both stops), so the model limits follow it. The gripper keeps the model's jaw stops.
                model.jnt_range[jid] = sorted((q_lo, q_hi))
                model.jnt_limited[jid] = 1
            bias = (self.fidelity.get('lift_bias_deg') or {}).get(motor.split('_arm_')[0], 0.0) if motor.endswith('_arm_shoulder_lift') else 0.0
            roll = (self.fidelity.get('roll_offset_deg') or {}).get(motor.split('_arm_')[0], 0.0) if motor.endswith('_arm_wrist_roll') else 0.0
            if roll:
                # the real jaws at the normal wrist_roll (about 2047-2120) open to the robot's left and right (their
                # pads are vertical planes parallel to a side flap; 8-9 October wrist images and pinches); the twin's
                # mapping has them opening up and down in the arm's plane, i.e. 90 degrees off
                a += math.radians(roll)
            if bias:
                # the physical joint sits bias degrees from the angle its ticks say, toward a LOWER claw (the lift's
                # positive direction raises the arm on both sides: checked in the tests)
                a += math.radians(bias) * (1 if b > 0 else -1)
            self.motors[motor] = _Motor(motor, joint, int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]), aid if aid >= 0 else None,
                                        a, b, lo, hi, KP_GRIP if grip else KP_ARM, KV_GRIP if grip else KV_ARM)
            self.motors[motor].a0 = a
        self.arm_motors = [n for n in self.motors if '_arm_' in n]
        self.position_names = list(self.motors)
        self.wheel_ticks = {}
        sample_rows = {}
        try:
            sample_rows = {m['name']: m for m in json.loads(STATE_SAMPLE.read_text())['result']['motors']}
        except (OSError, ValueError, KeyError):
            pass
        for n in WHEELS:
            self.wheel_ticks[n] = int(sample_rows.get(n, {}).get('Present_Position', 0))
        # Robot frame (as the twin defines it) and scene bodies.
        self.model, self.data = model, data
        self._bad_warnings = [int(w) for w in (mj.mjtWarning.mjWARN_BADQPOS, mj.mjtWarning.mjWARN_BADQVEL, mj.mjtWarning.mjWARN_BADQACC)]
        self._bad_seen = 0
        mj.mj_kinematics(model, data)
        pan_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, j) for j in ('Rotation_L', 'Rotation_R')]
        self.origin = np.array([data.xanchor[i] for i in pan_ids]).mean(axis=0)
        self.origin[2] = 0.0
        self.axes = np.array([twin.FORWARD, twin.LEFT, twin.UP], dtype=float)
        self.box_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'box')
        self.flap_bodies = {b for b in (mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, model.body(i).name) for i in range(model.nbody) if model.body(i).name.startswith('box_flap')) if b >= 0}
        self.box_bodies = self.flap_bodies | ({self.box_body} if self.box_body >= 0 else set())
        flap_joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'flap_hinge')
        self.flap_qadr = int(model.jnt_qposadr[flap_joint]) if flap_joint >= 0 else None
        self.flap_spring0 = float(model.qpos_spring[self.flap_qadr]) if self.flap_qadr is not None else None
        self.flap_panel = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'box_flap')
        self.all_box_bodies = {i for i in range(model.nbody) if model.body(i).name == 'box' or model.body(i).name.startswith('box_')}
        # other hinged flaps (e.g. the far flap): one folded over the target flap holds its crease down
        self.other_flap_qadr = {model.joint(j).name: int(model.jnt_qposadr[j]) for j in range(model.njnt)
                                if model.joint(j).name.endswith('flap_hinge') and model.joint(j).name != 'flap_hinge'}
        self.table_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'table')
        self.scene_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == 0 and model.geom_type[g] != mj.mjtGeom.mjGEOM_PLANE
                            and 'floor' not in (mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g) or '')]
        box_joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'box_free')
        self.box_qadr = int(model.jnt_qposadr[box_joint]) if box_joint >= 0 else None
        self.box_dadr = int(model.jnt_dofadr[box_joint]) if box_joint >= 0 else None
        self.jaw_bodies = {arm: (mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, fixed), mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, moving))
                           for arm, (fixed, moving) in (('left_arm', ('Fixed_Jaw', 'Moving_Jaw')), ('right_arm', ('Fixed_Jaw_2', 'Moving_Jaw_2')))}
        self.tip_sites = {arm: mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, site) for arm, (_, _, _, site) in twin.ARMS.items()}
        # Head joints never move (read-only motors): freeze them. Wheels do not spin (fixed base).
        for joint in ('head_pan_joint', 'head_tilt_joint', 'left_wheel_joint', 'right_wheel_joint'):
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
            if jid >= 0:
                model.dof_frictionloss[model.jnt_dofadr[jid]] = 50.0
                model.dof_damping[model.jnt_dofadr[jid]] = 50.0
        self.head_ticks = {n: int(sample_rows.get(n, {}).get('Present_Position', (self.calibration[n][0] + self.calibration[n][1]) // 2)) for n in HEAD_NAMES}
        tilt = self.fidelity.get('head_tilt_deg')
        if tilt is not None and 'head_motor_2' in self.calibration:
            # the real OAK looks down into the open box (9 October: its table-plane fit landed on the box floor);
            # head_motor_2 above its midpoint tilts the head down (twin convention)
            lo, hi = self.calibration['head_motor_2']
            self.head_ticks['head_motor_2'] = int((lo + hi) // 2 + float(tilt) * 4096 / 360)
        self.world.model, self.world.data = model, data
        self._reset_state()

    def _reset_state(self):
        """All motors released, arms folded on the cart, box at its start pose; counters cleared."""
        mj = self.mj
        model, data = self.model, self.data
        mj.mj_resetData(model, data)
        for n, motor in self.motors.items():
            motor.enabled = False
            motor.goal = None
            if n in HEAD_NAMES:
                data.qpos[motor.qadr] = motor.tick_to_q(self.head_ticks[n])
                continue
            side, joint = n.split('_arm_')
            if joint == 'gripper':
                tick = motor.lo + GRIPPER_CLOSED_OFFSET_TICKS + 20   # just closed
            else:
                tick = twin.motor_ticks({n: FOLDED_DEG[joint]}, {n: (motor.lo, motor.hi)})[n]
            data.qpos[motor.qadr] = motor.tick_to_q(tick)
            self._set_passive(motor)
        mj.mj_forward(model, data)
        self.motion = None
        self.base = None
        self.last_motion = None
        self.last_stop = None
        self.stop_count = 0
        self.error = None
        self.command_counter = 0
        self.last_completed = None
        self.last_rejected = None
        self.flap_pinched_ever = False
        if self.flap_qadr is not None:
            model.qpos_spring[self.flap_qadr] = self.flap_spring0
        self.crease = {'yield_s': 0.0, 'press_s': 0.0, 'last_t': None, 'prev_deg': None, 'deg_by_pinch': 0.0, 'deg_by_push': 0.0,
                       'deg_free': 0.0, 'illegal_s': 0.0, 'illegal_ever': False, 'pad_press_s': 0.0, 'max_deg': None,
                       'max_deg_pinched': None, 'pressed_set': False, 'held_over_95_s': 0.0}
        self.grip_stick_at = {}     # name -> (kind, tick): stick when the jaw passes that tick
        self.grip_stuck = {}        # name -> (kind, q, tick): frozen jaw
        self.counts = {'faults': 0, 'refusals': 0, 'moves': 0, 'gripper_closes': 0, 'base_pulses': 0, 'calls': 0, 'sim_resets': 0}
        self._bad_seen = sum(int(data.warning[w].number) for w in self._bad_warnings)
        self.wall_started = time.monotonic()
        self.sim_started = float(data.time)
        self._poll_due = 0.0
        self._poll_rows()
        # Let the folded arms settle on their friction for a moment, then record the box start pose.
        for _ in range(int(0.5 * PHYSICS_HZ)):
            self._physics_step()
        self.box_start = self.data.xpos[self.box_body].copy() if self.box_body >= 0 else None
        self.sim_started = float(data.time)

    def _set_passive(self, motor):
        self.model.dof_damping[motor.dadr] = RELEASED_DAMPING
        self.model.dof_frictionloss[motor.dadr] = RELEASED_FRICTION
        if motor.actuator is not None:
            self.data.ctrl[motor.actuator] = 0.0
        motor.force = 0.0

    def _set_active(self, motor):
        self.model.dof_damping[motor.dadr] = ENABLED_DAMPING
        self.model.dof_frictionloss[motor.dadr] = ENABLED_FRICTION

    def reset(self, seed=None):
        """Back to the start: all released, arms folded, box at its (seeded) start pose. A new seed rebuilds the scene."""
        with self.lock:
            if seed is not None and seed != self.seed and self._given_xml is None:
                self.seed = seed
                self._load_model(seed)
            else:
                self._reset_state()
            self.cancel_generation += 1
            self._requests.clear()

    def close(self):
        self._stop_thread.set()
        if self._thread is not None and self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=2)

    # ---------------------------------------------------------------- physics thread

    def _run(self):
        dt = 1.0 / PHYSICS_HZ
        anchor_wall = time.monotonic()
        anchor_sim = self.data.time
        while not self._stop_thread.is_set():
            with self.lock:
                fast = self._fast > 0 and not self.real_time
                self.world.fast_forward = fast
                if fast:
                    for _ in range(100):
                        self._physics_step()
                    anchor_wall, anchor_sim = time.monotonic(), self.data.time
                else:
                    due = int((time.monotonic() - anchor_wall) / dt) - int(round((self.data.time - anchor_sim) / dt))
                    if due > PHYSICS_HZ // 4:  # fell behind (e.g. the lock was held): resync rather than catch up
                        anchor_wall, anchor_sim = time.monotonic(), self.data.time
                        due = 1
                    for _ in range(max(0, due)):
                        self._physics_step()
            time.sleep(0 if fast else 0.002)
        self.world.fast_forward = False

    def _physics_step(self):
        mj, data = self.mj, self.data
        now = float(data.time)
        dt = self.model.opt.timestep
        if self.motion is not None and self.motion.active:
            self.motion.advance_ramp(now)
            for n in self.motion.joints:
                self.motors[n].goal = self.motion.goal[n]
        if self.base is not None and self.base.active:
            self.base.step(self, dt, now)
        if self.grip_stick_at or self.grip_stuck:
            self._grip_stick_step()
        for n in self.arm_motors:
            motor = self.motors[n]
            if motor.actuator is None:
                continue
            if motor.enabled and motor.goal is not None:
                q = data.qpos[motor.qadr]
                qd = data.qvel[motor.dadr]
                tau = motor.kp * (motor.tick_to_q(motor.goal) - q) - motor.kv * qd
                tau = max(-FORCE_LIMIT, min(FORCE_LIMIT, tau))
            else:
                tau = 0.0
            motor.force = tau
            data.ctrl[motor.actuator] = tau
        mj.mj_step(self.model, data)
        bad = sum(int(data.warning[w].number) for w in self._bad_warnings)
        if bad > self._bad_seen:   # MuJoCo reset the state after a divergence (bad qpos/qvel/qacc): count it for score()
            self._bad_seen = bad
            self.counts['sim_resets'] = self.counts.get('sim_resets', 0) + 1
        now = float(data.time)
        if now >= self._poll_due:
            self._poll_due = now + POLL_S
            self._poll(now)
        self.world._run_hooks()

    def _poll(self, now):
        rows = self._poll_rows()
        if not self.flap_pinched_ever and self.flap_bodies:
            self.flap_pinched_ever = any(all(self._box_touching_jaw(arm, bodies=self.flap_bodies)) for arm in self.jaw_bodies)
        if self.flap_qadr is not None:
            self._crease_poll(now)
        try:
            if self.motion is not None and self.motion.active:
                self.motion.poll(now, rows)
            if self.base is not None and self.base.active:
                self.base.poll(self, now)
        except _Fault as fault:
            self._release_all(str(fault))

    def _poll_rows(self):
        data = self.data
        rows = {}
        for n, motor in self.motors.items():
            q = data.qpos[motor.qadr]
            tick = int(round(motor.q_to_tick(q)))
            previous = motor.present
            motor.present = max(0, min(4095, tick))
            motor.velocity = int(round((motor.present - previous) / POLL_S)) if getattr(self, '_polled', False) else 0
            motor.load = int(max(-1000, min(1000, round(motor.force * LOAD_PER_LAG_TICK / (motor.kp * TICK_RAD))))) if motor.enabled else 0
            motor.moving = 1 if abs(motor.velocity) >= STILL_TICKS_PER_S else 0
            rows[n] = {'Present_Position': motor.present, 'Present_Velocity': motor.velocity, 'Present_Load': motor.load, 'Moving': motor.moving}
        self.rows = rows
        self._polled = True
        return rows

    def _release_all(self, reason, record=True):
        """Owner fault / STOP: every motor released, the running motion cancelled (never resumed)."""
        self.grip_stick_at.clear()
        self.grip_stuck.clear()
        for motor in self.motors.values():
            if motor.enabled:
                motor.enabled = False
                motor.goal = None
                self._set_passive(motor)
        if self.motion is not None and self.motion.active:
            self.motion.active = False
            self.motion.outcome = 'fault' if record and reason != 'Operator STOP' else 'stopped'
            self.motion.result = None
            self.motion.done.set()
        if self.base is not None and self.base.active:
            self.base.active = False
            self.base.result = None
            self.base.done.set()
        if record:
            self.last_stop = {'time': time.time(), 'reason': reason, 'command_id': self.motion.command_id if self.motion else None, 'released': True, 'release_errors': []}
            self.stop_count += 1
            self.error = reason
            if reason != 'Operator STOP':
                self.counts['faults'] += 1

    def _shift_world(self, d_forward, d_angle):
        """Base drive on a fixed-base model: move the table (and the box when it rides on the table) the opposite
        way: translate by d_forward along the robot's forward axis and rotate by d_angle about the robot origin."""
        np = self.np
        ox, oy = self.origin[0], self.origin[1]
        c, s = math.cos(d_angle), math.sin(d_angle)
        forward = np.array(twin.FORWARD) * d_forward

        def shifted(p):
            x, y = p[0] - ox, p[1] - oy
            return np.array([ox + c * x - s * y + forward[0], oy + s * x + c * y + forward[1], p[2]])
        model, data = self.model, self.data
        if self.table_body >= 0:
            model.body_pos[self.table_body] = shifted(model.body_pos[self.table_body])
            quat = model.body_quat[self.table_body].copy()
            self._rotate_quat(quat, d_angle)
            model.body_quat[self.table_body] = quat
        for g in self.scene_geoms:   # static scene geoms attached to the world (table, legs, lamp)
            model.geom_pos[g] = shifted(model.geom_pos[g])
            quat = model.geom_quat[g].copy()
            self._rotate_quat(quat, d_angle)
            model.geom_quat[g] = quat
        if self.box_qadr is not None and not self._box_touching_jaw():
            q = data.qpos[self.box_qadr:self.box_qadr + 7]
            q[0:3] = shifted(q[0:3])
            rot = q[3:7].copy()
            self._rotate_quat(rot, d_angle)
            q[3:7] = rot

    def _rotate_quat(self, quat, angle):
        """In place: quat <- rot_z(angle) * quat."""
        mj = self.mj
        rz = self.np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])
        out = self.np.zeros(4)
        mj.mju_mulQuat(out, rz, quat)
        quat[:] = out

    def _box_touching_jaw(self, arm=None, bodies=None):
        """(fixed jaw touching, moving jaw touching) for ``arm`` (both arms when None), from the current contacts.
        ``bodies``: which box bodies count (default the box and its flap panels)."""
        bodies = self.box_bodies if bodies is None else bodies
        if not bodies:
            return False if arm is None else (False, False)
        model, data = self.model, self.data
        fixed = moving = False
        arms = [arm] if arm else list(self.jaw_bodies)
        for i in range(data.ncon):
            con = data.contact[i]
            b1, b2 = int(model.geom_bodyid[con.geom1]), int(model.geom_bodyid[con.geom2])
            if b1 in bodies:
                other = b2
            elif b2 in bodies:
                other = b1
            else:
                continue
            for a in arms:
                f, m = self.jaw_bodies[a]
                fixed |= other == f
                moving |= other == m
        if arm is None:
            return fixed or moving
        return fixed, moving

    # ---------------------------------------------------------------- real-robot fidelity: sticky gripper, crease

    def _grip_call(self, arm, name, current, position):
        """Start of a robot_set_gripper call (the tool, not its 300-tick parts): per-close meeting-point jitter and the
        right gripper's mid-travel sticking (FIDELITY). Called under the lock."""
        closing = position < current
        sticks = self.fidelity.get('right_grip_sticks') if arm == 'right' else None
        self.grip_stick_at.pop(name, None)
        stuck = self.grip_stuck.pop(name, None)
        if stuck is not None and not closing and stuck[0] in ('open', 'reopen') and sticks and self.rng.random() < sticks['p_reopen']:
            self.grip_stuck[name] = ('reopen', stuck[1], stuck[2])   # a resent open: the jaw does not move at all
        elif sticks:
            if closing and current > 1700 and position < sticks['close_band'][0] and self.rng.random() < sticks['p_close']:
                self.grip_stick_at[name] = ('close', self.rng.uniform(*sticks['close_band']))
            elif not closing:
                lo, hi = max(sticks['open_band'][0], current + 60), min(sticks['open_band'][1], position - 30)
                if lo < hi and self.rng.random() < sticks['p_open']:
                    self.grip_stick_at[name] = ('open', self.rng.uniform(lo, hi))
        jitter = (self.fidelity.get('meet_jitter_ticks') or {}).get(arm)
        if closing and jitter:
            motor = self.motors[name]
            motor.a = motor.a0 - self.rng.uniform(*jitter) * motor.b

    def _grip_stick_step(self):
        """Every physics step: a jaw passing its stick tick freezes there; a frozen jaw stays put. An opening move whose
        jaw froze has its goal capped 25 ticks past the jaw, so it ends 'settled_short' (as the real opens did); a resent
        open on a jaw still stuck is not capped and trips the no-progress guard."""
        data = self.data
        motion = self.motion if self.motion is not None and self.motion.active else None
        for name, (kind, tick) in list(self.grip_stick_at.items()):
            motor = self.motors[name]
            t = motor.q_to_tick(float(data.qpos[motor.qadr]))
            if (kind == 'close' and t <= tick) or (kind == 'open' and t >= tick):
                del self.grip_stick_at[name]
                self.grip_stuck[name] = (kind, float(data.qpos[motor.qadr]), int(round(t)))
        for name, (kind, q, tick) in self.grip_stuck.items():
            motor = self.motors[name]
            data.qpos[motor.qadr] = q
            data.qvel[motor.dadr] = 0.0
            if kind == 'open' and motion is not None and name in motion.joints and not motion.contact:
                cap = tick + 25
                motion.stalled = True
                for leg in motion.legs:
                    leg[name] = min(leg[name], cap)
                motion.targets[name] = min(motion.targets[name], cap)
                motion.goal[name] = min(motion.goal[name], cap)
                motor.goal = motion.goal[name]

    def _flap_contacts(self):
        """Robot contacts on the target flap: {'pinched' (the INNER pad faces of both jaws of one arm touch it: the flap is
        between the pads), 'pads' (some contact on a jaw's distal pad region), 'illegal' (a contact that is not allowed by
        the owner's 9 October rule: any robot geom other than the jaw pads, and the outer faces or tips of the pads while
        no arm pinches the flap, i.e. pushing with a claw; a pad's inner face is always allowed (closing on the flap), and
        while an arm pinches every pad contact of either arm is, which covers pressing the crease with the other claw's
        pads), 'illegal_geoms', 'crease_force' (N, robot contacts
        on the crease panel within PLASTIC press_zone_m of the hinge)}. Pads = jaw-body contacts beyond PAD_Y_M along the
        jaw (Fixed_Jaw frame); inner = the contact normal points from the jaw toward the other jaw."""
        np, mj = self.np, self.mj
        model, data = self.model, self.data
        zone = (self.plastic or {}).get('press_zone_m', 0.05)
        out = {'pinched': False, 'pads': False, 'illegal': False, 'illegal_geoms': set(), 'crease_force': 0.0}
        inner = {arm: [False, False] for arm in self.jaw_bodies}
        hits = []
        f6 = np.zeros(6)
        for i in range(data.ncon):
            con = data.contact[i]
            b1, b2 = int(model.geom_bodyid[con.geom1]), int(model.geom_bodyid[con.geom2])
            if b1 in self.flap_bodies and b2 not in self.all_box_bodies and b2 != 0:
                flap_body, other, og, sign = b1, b2, con.geom2, -1.0   # frame normal points geom1 -> geom2 (flap -> robot)
            elif b2 in self.flap_bodies and b1 not in self.all_box_bodies and b1 != 0:
                flap_body, other, og, sign = b2, b1, con.geom1, 1.0
            else:
                continue
            pad = face = False
            jaw = None
            for arm, (fixed, moving) in self.jaw_bodies.items():
                if other in (fixed, moving):
                    rot = data.xmat[fixed].reshape(3, 3)
                    local = rot.T @ (np.asarray(con.pos) - data.xpos[fixed])
                    pad = bool(local[1] < PAD_Y_M)
                    jaw = (arm, other)
                    if pad:
                        n = rot.T @ (sign * np.asarray(con.frame[:3]))   # jaw -> flap, fixed jaw frame
                        # the pads close along x: the fixed pad's inner face looks toward -x, the moving pad's toward +x
                        face = bool((other == fixed and n[0] < -0.5) or (other == moving and n[0] > 0.5))
                        if face:
                            inner[arm][0 if other == fixed else 1] = True
            hits.append((pad, face, jaw, mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, og) or model.body(other).name))
            if flap_body == self.flap_panel:
                rot = data.xmat[flap_body].reshape(3, 3)
                local = rot.T @ (np.asarray(con.pos) - data.xpos[flap_body])
                if 0.0 <= local[2] < zone:
                    mj.mj_contactForce(model, data, i, f6)
                    out['crease_force'] += abs(float(f6[0]))
        out['pinched'] = any(all(v) for v in inner.values())
        facing = {jaw for pad, face, jaw, _ in hits if face}   # a jaw closing on the flap with its face: its pad edges too
        # an arm whose gripper is moving (closing on the flap, or opening to let it go) grips or releases with its pads
        motion = self.motion if self.motion is not None and self.motion.active else None
        gripping = {arm for arm in self.jaw_bodies if motion is not None and f"{arm.split('_')[0]}_arm_gripper" in motion.joints}
        for pad, face, jaw, name in hits:
            out['pads'] |= pad
            if not (face or (pad and (out['pinched'] or jaw in facing or (jaw is not None and jaw[0] in gripping)))):
                out['illegal'] = True
                out['illegal_geoms'].add(name)
        return out

    def _crease_poll(self, now):
        """10 Hz: fold accounting (how much of the flap's inward turn happened while the pads held it vs while another
        robot part touched it) and the crease plasticity (box_scene.PLASTIC)."""
        c = self.crease
        model, data = self.model, self.data
        th = math.degrees(float(data.qpos[self.flap_qadr]))
        dt = 0.0 if c['last_t'] is None else max(0.0, now - c['last_t'])
        c['last_t'] = now
        info = self._flap_contacts()
        prev, c['prev_deg'] = c['prev_deg'], th
        if prev is not None and th > prev:
            key = 'deg_by_push' if info['illegal'] else 'deg_by_pinch' if (info['pinched'] or info['pads']) else 'deg_free'
            c[key] += th - prev
        if info['illegal']:
            c['illegal_s'] += dt
            c['illegal_ever'] = True
            c.setdefault('illegal_geoms', set()).update(info['illegal_geoms'])
        c['max_deg'] = th if c['max_deg'] is None else max(c['max_deg'], th)
        if info['pinched']:
            c['max_deg_pinched'] = th if c['max_deg_pinched'] is None else max(c['max_deg_pinched'], th)
        P = self.plastic
        if not P:
            return
        th0 = math.degrees(float(model.qpos_spring[self.flap_qadr]))
        loaded = abs(th - th0) > P['yield_deg']
        c['yield_s'] = c['yield_s'] + dt if loaded else 0.0
        if loaded and th > 95.0:
            c['held_over_95_s'] += dt
        new = th0
        if c['yield_s'] > P['delay_s']:
            if th > th0:
                f = min(1.0, max(0.0, (th - P['start_set_deg']) / (P['full_set_deg'] - P['start_set_deg'])))
                target = th - P['springback_deg'] * (1.0 - f)
            else:
                target = th + P['springback_deg']
            if (target - th0) * (th - th0) > 0:
                new += (target - th0) * (1.0 - math.exp(-P['rate_per_s'] * dt))
        covered = th >= P['press_min_deg'] and any(math.degrees(float(data.qpos[q])) >= P['cover_deg'] for q in self.other_flap_qadr.values())
        if covered:
            c['covered'] = True
        pressing = covered or (th >= P['press_min_deg'] and info['crease_force'] >= P['press_force_n'])
        c['press_s'] = c['press_s'] + dt if pressing else 0.0
        if c['press_s'] > P['press_s']:
            target = th - P['press_springback_deg']
            if target > new:
                new += (target - new) * (1.0 - math.exp(-P['press_rate_per_s'] * dt))
                c['pressed_set'] = True
        if new != th0:
            model.qpos_spring[self.flap_qadr] = math.radians(new)

    # ---------------------------------------------------------------- public: catalog, get, call, score

    def catalog(self):
        tools = []
        try:
            raw = json.loads(CATALOG_SAMPLE.read_text())
            raw_tools = raw.get('tools', [])
        except (OSError, ValueError):
            raw_tools = []
        by_name = {t['function']['name']: t for t in raw_tools if t.get('type') == 'function'}
        for name in IMPLEMENTED_TOOLS:
            tool = by_name.get(name) or _minimal_tool(name)
            tools.append(self._patch_schema(json.loads(json.dumps(tool))))
        return {'tools': tools, 'metadata': {'ok': True, 'stop_tool': 'robot_stop', 'simulated': True, 'execution_profile': EXECUTION_PROFILE,
                                             'scene': self.scene_source, 'model': twin.VENDORED_ID}}

    def _patch_schema(self, tool):
        """Joint bounds in the schemas follow this robot's calibration (raw range +-4 for targets, the 40-tick
        commandable band for the gripper tool), exactly as the real server builds them."""
        name = tool['function']['name']
        params = tool['function']['parameters']
        cal = self.calibration

        def patch_targets(obj):
            for key, spec in obj.get('properties', {}).items():
                if key in cal and isinstance(spec, dict) and spec.get('type') == 'integer':
                    spec['minimum'], spec['maximum'] = cal[key][0] + 4, cal[key][1] - 4
                elif isinstance(spec, dict) and spec.get('type') == 'integer' and 'minimum' in spec:
                    for arm in ('left', 'right'):
                        full = f'{arm}_arm_{key}'
                        if full in cal and (obj.get('description') or '').startswith('Selected arm') and full in obj.get('properties', {}):
                            spec['minimum'], spec['maximum'] = cal[full][0] + 4, cal[full][1] - 4
        if name == 'robot_move_joint_targets':
            patch_targets(params['properties']['positions'])
            for clause in params.get('allOf', []):
                patch_targets(clause['then']['properties']['positions'])
        if name == 'robot_move_path':
            patch_targets(params['properties']['waypoints']['items'])
        if name == 'robot_set_gripper':
            bands = {a: self._commandable()[f'{a}_arm_gripper'] for a in ('left', 'right')}
            message = '; '.join(f"{a}: {b['min_ticks']}..{b['max_ticks']} inclusive ticks" for a, b in bands.items())
            params['properties']['position_ticks']['description'] = 'Commandable integer encoder ticks: ' + message
            params['allOf'] = [{'if': {'properties': {'arm': {'const': a}}, 'required': ['arm']},
                               'then': {'properties': {'position_ticks': {'minimum': b['min_ticks'], 'maximum': b['max_ticks']}}}} for a, b in bands.items()]
            desc = tool['function']['description']
            head = desc.split('Commandable gripper ranges: ')[0]
            tool['function']['description'] = head + 'Commandable gripper ranges: ' + message + '. Raw calibration endpoints are invalid command targets; no clamping.'
        return tool

    def get(self, path):
        if path in ('/health', '/healthz'):
            return {'ok': True, 'status': 'ok', 'simulated': True, 'execution_profile': EXECUTION_PROFILE, 'time': time.time()}
        if path == '/status':
            with self.lock:
                return {'ok': True, 'phase': self._phase(), 'enabled_motors': self._enabled(), 'sim_time_s': self.world.now(),
                        'last_stop': self.last_stop, 'stop_count': self.stop_count, 'simulated': True}
        if path == '/tools':
            cat = self.catalog()
            return {'ok': True, 'tools': cat['tools'], **{k: v for k, v in cat['metadata'].items() if k != 'ok'}}
        raise KeyError(f'Unknown route {path}')

    def call(self, name, args, request_id=None):
        """Exactly the real bridge's envelope: {'ok': True, 'result': ..., ['images': ...]} or
        {'ok': False, 'http_status': 400|409, 'result': {'error': ..., ['motor_writes': ...]}}."""
        args = {} if args is None else args
        if request_id is not None:
            cached = self._requests.get(request_id)
            if cached is not None:
                return cached
        self.counts['calls'] += 1
        try:
            result, images = self._dispatch(name, args)
            body = {'ok': True, 'result': result}
            if images is not None:
                body['images'] = images
        except (ValueError, KeyError, TypeError) as error:
            body = {'ok': False, 'http_status': 400, 'result': {'error': str(error)}}
            self.counts['refusals'] += 1
        except Exception as error:  # noqa: BLE001 - mirrors the bridge's catch-all
            body = {'ok': False, 'http_status': 409, 'result': {'error': str(error), 'motor_writes': 'not_observed_by_bridge' if name in MOTION_TOOLS else 0}}
            self.counts['refusals'] += 1
        if request_id is not None:
            self._requests[request_id] = body
            while len(self._requests) > 64:
                del self._requests[next(iter(self._requests))]
        return body

    def score(self):
        with self.lock:
            np = self.np
            lifted = moved = 0.0
            closed = held = False
            if self.box_body >= 0 and self.box_start is not None:
                pos = self.data.xpos[self.box_body]
                lifted = float(pos[2] - self.box_start[2])
                moved = float(np.linalg.norm(pos[:2] - self.box_start[:2]))
                for arm in self.jaw_bodies:
                    f, m = self._box_touching_jaw(arm)
                    if f and m:
                        closed = True
                held = closed and lifted >= 0.03
            flap = self._flap_state()
            return {'box_lifted_m': lifted, 'box_moved_m': moved, 'gripper_closed_on_box': closed, 'box_held_now': held, **flap,
                    'faults': self.counts['faults'], 'refusals': self.counts['refusals'], 'moves': self.counts['moves'],
                    'gripper_closes': self.counts['gripper_closes'], 'base_pulses': self.counts['base_pulses'], 'calls': self.counts['calls'],
                    'sim_resets': self.counts['sim_resets'],
                    'sim_time_s': float(self.data.time - self.sim_started), 'wall_time_s': time.monotonic() - self.wall_started,
                    'released_all': all(not m.enabled for m in self.motors.values()), 'last_stop': self.last_stop}

    def _flap_state(self):
        """flap_angle_deg (0 vertical, 90 flat on the top; None without a hinged flap); flap_pinched_now (both jaws of one
        arm touch the flap); flap_pinched_ever (so at any 10 Hz poll since the reset: a fold without it was a push);
        flap_folded (>= FLAP_FOLDED_DEG and resting on the top: the box upright on the table and no jaw touching the flap,
        i.e. it stays folded on its own)."""
        if self.flap_qadr is None:
            return {'flap_angle_deg': None, 'flap_pinched_now': False, 'flap_pinched_ever': False, 'flap_folded': False}
        angle = math.degrees(float(self.data.qpos[self.flap_qadr]))
        pinched = touched = False
        for arm in self.jaw_bodies:
            f, m = self._box_touching_jaw(arm, bodies=self.flap_bodies)
            pinched |= f and m
            touched |= f or m
        rot = self.data.xmat[self.box_body].reshape(3, 3)
        upright = rot[2, 2] > math.cos(math.radians(10))
        resting = upright and self.box_start is not None and abs(float(self.data.xpos[self.box_body][2] - self.box_start[2])) < 0.01
        folded = bool(angle >= FLAP_FOLDED_DEG and resting and not touched)
        c = self.crease
        rest = math.degrees(float(self.model.qpos_spring[self.flap_qadr]))
        # fold_by_pinch: folded, and the flap turned at most FOLD_PUSH_ALLOWANCE_DEG while a robot part other than the
        # jaw pads touched it (the owner's rule of 9 October: no pushing with the claw body, wrist camera or arm)
        return {'flap_angle_deg': round(angle, 1), 'flap_pinched_now': pinched, 'flap_pinched_ever': bool(self.flap_pinched_ever or pinched),
                'flap_folded': folded, 'fold_by_pinch': bool(folded and c['deg_by_push'] <= FOLD_PUSH_ALLOWANCE_DEG),
                'flap_rest_deg': round(rest, 1), 'flap_deg_by_pinch': round(c['deg_by_pinch'], 1), 'flap_deg_by_push': round(c['deg_by_push'], 1),
                'flap_deg_free': round(c['deg_free'], 1), 'illegal_contact_s': round(c['illegal_s'], 1),
                'illegal_contact_geoms': sorted(c.get('illegal_geoms', ())), 'flap_max_deg': None if c['max_deg'] is None else round(c['max_deg'], 1),
                'flap_max_deg_pinched': None if c['max_deg_pinched'] is None else round(c['max_deg_pinched'], 1),
                'crease_pressed': bool(c['pressed_set']), 'crease_covered': bool(c.get('covered')), 'held_over_95_s': round(c['held_over_95_s'], 1),
                'other_flaps_deg': {n: round(math.degrees(float(self.data.qpos[q])), 1) for n, q in self.other_flap_qadr.items()}}

    def snapshot(self):
        """Positions/ranges for the twin renderer (render_twin(positions, ranges)) plus the box pose in the robot frame."""
        with self.lock:
            self._poll_rows()
            positions = {n: m.present for n, m in self.motors.items()}
            positions.update(self.wheel_ticks)
            box = None
            if self.box_body >= 0:
                f, l, u = (self.axes @ (self.data.xpos[self.box_body] - self.origin)).tolist()
                box = {'forward_m': f, 'left_m': l, 'up_m': u}
            return {'positions': positions, 'ranges': {n: list(v) for n, v in self.calibration.items()}, 'box': box,
                    'joint_map': json.loads(json.dumps(self.joint_map)), 'claws': self._claws(), 'sim_time_s': self.world.now(),
                    'enabled_motors': self._enabled()}

    def _claws(self):
        out = {}
        for arm, site in self.tip_sites.items():
            if site >= 0:
                f, l, u = (self.axes @ (self.data.site_xpos[site] - self.origin)).tolist()
                out[arm] = {'forward_m': f, 'left_m': l, 'up_m': u}
        return out

    # ---------------------------------------------------------------- state views

    def _enabled(self):
        return sorted(n for n, m in self.motors.items() if m.enabled)

    def _phase(self):
        if self.motion is not None and self.motion.active or self.base is not None and self.base.active:
            return 'moving'
        return 'holding' if any(m.enabled for m in self.motors.values()) else 'idle'

    def _commandable(self):
        return {n: {'min_ticks': lo + MARGIN, 'max_ticks': hi - MARGIN, 'margin_ticks': MARGIN} for n, (lo, hi) in self.calibration.items() if n in self.motors}

    def _motor_rows(self):
        now = time.time()
        rows = []
        for n, m in self.motors.items():
            rows.append({'name': n, 'Present_Position': m.present, 'Present_Velocity': m.velocity, 'Present_Load': m.load,
                         'Present_Voltage': PRESENT_VOLTAGE, 'Present_Temperature': PRESENT_TEMPERATURE, 'Status': 0, 'Moving': m.moving,
                         'Present_Current': abs(m.load) // 4, 'Torque_Enable': 1 if m.enabled else 0, 'Operating_Mode': 0, 'captured_at': now,
                         'firmware_position_limits': [m.lo, m.hi]})
        for n in WHEELS:
            rows.append({'name': n, 'Present_Position': self.wheel_ticks[n], 'Present_Velocity': 0, 'Present_Load': 0, 'Present_Voltage': PRESENT_VOLTAGE,
                         'Present_Temperature': PRESENT_TEMPERATURE, 'Status': 0, 'Moving': 0, 'Present_Current': 0, 'Torque_Enable': 0,
                         'Operating_Mode': 0, 'captured_at': now, 'firmware_position_limits': [0, 4095]})
        return rows

    def _state(self):
        with self.lock:
            self._poll_rows()
            rows = self._motor_rows()
            enabled = self._enabled()
        return {'source': 'canonical_hardware_owner', 'time': time.time(), 'read_age_s': 0.0, 'cached': False, 'motors': rows,
                'control_mode': 'direct_joint', 'enabled_motors': enabled, 'all_16_released': not enabled,
                'commandable_ranges': self._commandable(),
                'raw_calibration_ranges': {n: {'min_ticks': lo, 'max_ticks': hi} for n, (lo, hi) in self.calibration.items()},
                'range_semantics': RANGE_SEMANTICS}

    def _readiness(self):
        phase = self._phase()
        blockers = []
        if phase not in ('idle', 'holding'):
            blockers.append('OWNER_BUSY: ' + phase)
        joint_blockers = {}
        for n, m in self.motors.items():
            if '_arm_' in n and not m.lo <= m.present <= m.hi:
                joint_blockers[n] = ['CURRENT_POSITION_OUTSIDE_SAVED_RANGE']
        return {'mode': 'direct_joint', 'control_mode': 'direct_joint', 'hardware_server': True, 'execution_adapter_bound': True,
                'execution_adapter_source': 'farm/sim/sim_robot.py:SimRobot', 'motion_ready': not blockers,
                'available_to_accept_authorized_command': not blockers, 'operator_armed': True, 'local_operator_gate': True,
                'motor_owner_active': True, 'supported_joints': list(self.arm_motors), 'supported_motors': list(self.motors) + list(WHEELS),
                'joint_blockers': joint_blockers, 'blockers': blockers, 'blocker': '; '.join(blockers) if blockers else None,
                'remote_owner_start': False, 'stop_latched': False, 'owner_restart_required_after_stop': False,
                'continuous_profile_required': False, 'cartesian_transform_required': False, 'camera_gate_required': False,
                'owner_started': self.started, 'owner_phase': phase, 'owner_status_age_s': 0.0, 'enabled_motors': self._enabled(),
                'pickup_required_enabled_motors': list(self.arm_motors), 'execution_profile': EXECUTION_PROFILE,
                'base_drive_supported': True, 'base_drive_limits': {'max_wheel_m_s': MAX_WHEEL_M_S, 'max_duration_s': MAX_DURATION_S},
                'read_only': False, 'calibration_mismatches': {}, 'last_stop': self.last_stop, 'stop_count': self.stop_count,
                'release_errors': [], 'torque_enabled_by_joint': {n: (1 if m.enabled else 0) for n, m in self.motors.items()},
                'lease_remaining_s': 120.0, 'lease_applies_while_enabled': True, 'explicit_enable_renews_idle_lease': True, 'simulated': True}

    def _capabilities(self):
        with self.lock:
            ready = self._readiness()
        return {'mode': 'direct_joint', 'control_mode': 'direct_joint', 'execution_adapter_bound': True, 'armed': True,
                'motion_ready': ready['motion_ready'], 'available_to_accept_authorized_command': ready['available_to_accept_authorized_command'],
                'execution_binding': ready, 'joint_blockers': ready['joint_blockers'],
                'camera_status': {'simulated': True, 'cameras': ['oak', 'phone', 'left_wrist', 'right_wrist']},
                'saved_motor_readiness': {'simulated': True}, 'automatic_motion_on_startup': False, 'passive_recovery': False,
                'motion_units': 'encoder_ticks; positioning-joint degrees use4095 ticks/rev', 'commandable_ranges': self._commandable(),
                'canonical_position_joint_names': list(self.motors), 'arm_joint_aliases': sorted(ARM_JOINTS),
                'ranges': {n: {'min_ticks': lo, 'max_ticks': hi} for n, (lo, hi) in self.calibration.items()},
                'read_tools_ready': True, 'stop_tool_ready': True, 'controller': 'SimRobot MuJoCo twin -> simulated sole owner (paddle-success-v1)',
                'execution_profile': EXECUTION_PROFILE, 'base_drive_limits': {'max_wheel_m_s': MAX_WHEEL_M_S, 'max_duration_s': MAX_DURATION_S},
                'wheel_policy': 'robot_move_base guarded velocity pulses (<=0.02 m/s per wheel, <=3 s, released between pulses)',
                'base_drive_supported': True, 'automatic_motor_activation': False, 'remote_owner_start_supported': False,
                'stop_latch': False, 'owner_restart_required_after_stop': False,
                'stop_behavior': ('robot_stop and any owner fault release all motors and cancel the move in progress (never resumed). No STOP latch: '
                                  'the owner returns to idle and motors stay released until an explicit robot_set_motor_enable.'),
                'blockers': ready['blockers'], 'simulated': True}

    # ---------------------------------------------------------------- dispatch

    def _dispatch(self, name, args):
        if name not in IMPLEMENTED_TOOLS or not isinstance(args, dict):
            raise ValueError('Unknown tool or invalid arguments')
        self._validate_arguments(name, args)
        if name == 'robot_get_state':
            return self._state(), None
        if name == 'robot_get_capabilities':
            return self._capabilities(), None
        if name == 'robot_list_motors':
            with self.lock:
                ready = self._readiness()
            return {'state': self._state(), 'saved_calibration': {n: {'range_min': lo, 'range_max': hi} for n, (lo, hi) in self.calibration.items()},
                    'commandable_ranges': self._commandable(), 'readiness': ready}, None
        if name in CAMERA_TOOLS:
            return self._cameras(name, args)
        if name == 'robot_get_motion':
            return self._motion_view(), None
        if name == 'robot_stop':
            return self._stop(), None
        if name == 'robot_set_motor_enable':
            return self._set_motor_enable(args['names'], args['enabled']), None
        if name == 'robot_halt_motion':
            return self._halt(), None
        if name == 'robot_move_base':
            return self._counted('base_pulses', self._drive_base(args['linear_m_s'], args['angular_rad_s'], args['duration_s'])), None
        if name == 'robot_move_joint_targets':
            positions = self._normalize_targets(args['positions'], arm=args['arm'])
            return self._counted('moves', self._execute_targets(positions, args['duration_s'], wait=args.get('wait', True), replace=args.get('replace', False))), None
        if name == 'robot_move_path':
            waypoints = [self._normalize_targets(w, arm=args['arm']) for w in args['waypoints']]
            return self._counted('moves', self._execute_path(waypoints, args['duration_s'], wait=args.get('wait', True), replace=args.get('replace', False))), None
        if name == 'robot_set_gripper':
            n = args['arm'] + '_arm_gripper'
            b = self._commandable()[n]
            if not b['min_ticks'] <= args['position_ticks'] <= b['max_ticks']:
                with self.lock:
                    ready = self._readiness()
                raise ValueError(f"Gripper target out of bounds: {n}={args['position_ticks']}; valid inclusive range [{b['min_ticks']}, {b['max_ticks']}] ticks; readiness={json.dumps(ready)}")
            with self.lock:
                self._poll_rows()
                closing = args['position_ticks'] < self.motors[n].present - 2
            return self._counted('gripper_closes' if closing else 'moves', self._set_gripper(args['arm'], args['position_ticks'], args.get('duration_s', 3))), None
        raise ValueError('Unknown tool or invalid arguments')

    def _counted(self, key, result):
        """Count accepted motion tool calls (what a supervisor sends), not the owner's internal parts."""
        if isinstance(result, dict) and result.get('accepted') is not False:
            self.counts[key] += 1
        return result

    def _validate_arguments(self, name, args):
        """gemma_robot_tools.validate_arguments against the catalog schema."""
        schemas = getattr(self, '_schemas', None)
        if schemas is None:
            schemas = self._schemas = {t['function']['name']: t['function']['parameters'] for t in self.catalog()['tools']}
        schema = schemas[name]
        if set(args) - set(schema['properties']) or set(schema['required']) - set(args):
            raise ValueError('Unexpected or missing tool arguments')
        for key, value in args.items():
            if name == 'robot_move_path' and key == 'waypoints':
                if not isinstance(value, list) or not 1 <= len(value) <= MAX_PATH_WAYPOINTS or any(not isinstance(w, dict) or not w or any(type(q) is not int for q in w.values()) for w in value):
                    raise ValueError('waypoints must be 1..12 objects of joint name to integer ticks')
                continue
            spec = schema['properties'][key]
            kind = spec['type']
            if kind == 'boolean' and type(value) is not bool:
                raise ValueError('Boolean argument required')
            if kind == 'integer' and type(value) is not int:
                raise ValueError('Integer ticks required')
            if kind == 'number' and (type(value) not in (int, float) or not math.isfinite(value)):
                raise ValueError('Finite numeric argument required')
            if kind in ('number', 'integer') and ('maximum' in spec and value > spec['maximum'] or 'minimum' in spec and value < spec['minimum']
                                                 or 'exclusiveMinimum' in spec and value <= spec['exclusiveMinimum']):
                raise ValueError('Numeric argument outside schema bounds')
            if kind == 'string' and (not isinstance(value, str) or value not in spec.get('enum', [value])):
                raise ValueError('Unsupported argument value')
            if kind == 'object' and (not isinstance(value, dict) or not value or any(type(v) is not int for v in value.values())):
                raise ValueError('Integer joint target object required')
            if kind == 'array':
                if (not isinstance(value, list) or not spec.get('minItems', 1) <= len(value) <= spec.get('maxItems', 16) or any(not isinstance(v, str) for v in value)
                        or len(set(value)) != len(value) or any(v not in spec['items']['enum'] for v in value)):
                    raise ValueError('Select distinct supported argument names')

    def _normalize_targets(self, targets, arm=None):
        result = {}
        for key, q in targets.items():
            n = arm + '_arm_' + key if arm and key in ARM_JOINTS else key
            if n not in self.motors:
                raise ValueError('Unknown position joint: ' + key + '; valid canonical names: ' + ', '.join(self.motors))
            if arm and not n.startswith(arm + '_arm_'):
                raise ValueError('Wrong-arm joint: ' + key + '; requested arm: ' + arm)
            if n in result:
                raise ValueError('Duplicate/conflicting aliases for joint: ' + n)
            result[n] = q
        bands = self._commandable()
        for n, q in result.items():
            b = bands[n]
            if not b['min_ticks'] <= q <= b['max_ticks']:
                raise ValueError(f"Target out of bounds: {n}={q}; commandable inclusive range [{b['min_ticks']}, {b['max_ticks']}] ticks ({b['margin_ticks']}-tick margin)")
        return result

    # ---------------------------------------------------------------- cameras (delegated)

    def _cameras(self, name, args):
        try:
            from farm.sim.sim_cameras import SimCameras
        except ImportError:
            raise RuntimeError('cameras not available in this build')
        cams = getattr(self, '_sim_cameras', None)
        if cams is None:
            cams = self._sim_cameras = SimCameras(self.world)
        if name == 'robot_get_cameras':
            return cams.cameras(args.get('cameras', ['oak', 'phone']))
        if name == 'robot_get_clip':
            return cams.clip(args)
        return cams.depth()

    # ---------------------------------------------------------------- owner commands

    def _next_command_id(self):
        self.command_counter += 1
        return max(time.time_ns(), self.command_counter)

    def _stop(self):
        """DirectJointClient.stop(): release everything, cancel any motion; never latched."""
        self.cancel_generation += 1
        with self.lock:
            command_id = self._next_command_id()
            self._release_all('Operator STOP')
            self.last_completed = command_id
            phase = self._phase()
        return {'stop_requested': True, 'command_id': command_id, 'release': 'torque eases off over about 2 s, then off', 'release_confirmed': True,
                'stop_reset_supported': True, 'stop_latched': False, 'owner_restart_required': False,
                'motors_stay_released_until': 'explicit robot_set_motor_enable', 'release_owner_time': time.time(), 'owner_started': self.started,
                'owner_phase': phase}

    def _busy_result(self, ready):
        return {'accepted': False, 'motor_writes': 0, 'reason': ready['blocker'], 'readiness': ready}

    def _completion(self, command_id, readbacks, **extra):
        out = {'accepted': True, 'completed': True, 'command_id': command_id, 'owner_started': self.started, 'readbacks': readbacks,
               'endpoint_reached': None, 'settle_residual_ticks': None, 'execution_profile': EXECUTION_PROFILE, 'grasp_verified': False,
               'closure_outcome': None, 'gripper_result': None, 'owner_status_time': time.time(), 'duration_s_actual': None,
               'motor_writes': 'canonical owner only', 'mode': 'direct_joint'}
        out.update(extra)
        return out

    def _set_motor_enable(self, names, enabled):
        if type(enabled) is not bool or not isinstance(names, list) or not names or len(set(names)) != len(names) or any(not isinstance(n, str) for n in names):
            raise ValueError('Distinct motor names and boolean enabled required')
        supported = set(self.motors) | set(WHEELS)
        if not set(names) <= supported:
            raise ValueError('Unknown motor names')
        with self.lock:
            if not enabled:
                command_id = self._next_command_id()
                for n in names:
                    motor = self.motors.get(n)
                    if motor is not None and motor.enabled:
                        if self.motion is not None and self.motion.active and n in self.motion.joints:
                            self.motion.halt({j: self.motors[j].present for j in self.motion.joints})
                        motor.enabled = False
                        motor.goal = None
                        self._set_passive(motor)
                self.last_completed = command_id
                return self._completion(command_id, {n: 0 for n in names})
            ready = self._readiness()
            if not ready['available_to_accept_authorized_command']:
                return self._busy_result(ready)
            readonly = sorted(set(names) - set(self.arm_motors))
            if readonly:
                raise ValueError(READ_ONLY_MESSAGE.format(names=', '.join(readonly)))
            faults = {n: ready['joint_blockers'][n] for n in names if n in ready['joint_blockers']}
            if faults:
                raise ValueError('Requested motor health/range blockers: ' + json.dumps(faults))
            command_id = self._next_command_id()
            self._poll_rows()
            for n in names:
                motor = self.motors[n]
                if not motor.enabled:
                    motor.enabled = True
                    motor.goal = motor.present   # hold where it is
                    self._set_active(motor)
            self.last_completed = command_id
            return self._completion(command_id, {n: 1 for n in names})

    def _halt(self):
        with self.lock:
            command_id = self._next_command_id()
            halted_id = None
            if self.motion is not None and self.motion.active:
                halted_id = self.motion.command_id
                self.motion.halt({j: self.motors[j].present for j in self.motion.joints})
            self.last_completed = command_id
            self._poll_rows()
            positions = {n: self.motors[n].present for n in self.arm_motors}
            return {'accepted': True, 'completed': True, 'halted': True, 'command_id': command_id, 'halted_command_id': halted_id,
                    'phase': self._phase(), 'positions': positions, 'base_drive_phase': None, 'note': HALT_NOTE}

    def _motion_view(self):
        with self.lock:
            self._poll_rows()
            motion = self.motion
            active = motion is not None and motion.active
            d = motion.diagnostics if motion is not None else {}
            last = motion.result if motion is not None and motion.result is not None else {}
            phase = self._phase()
            return {'phase': phase, 'status_age_s': 0.0, 'moving': phase == 'moving',
                    'running_command_id': motion.command_id if active else None, 'last_completed_command_id': self.last_completed,
                    'closure_outcome': last.get('closure_outcome'), 'endpoint_reached': last.get('endpoint_reached'),
                    'settle_residual_ticks': last.get('settle_residual_ticks'), 'waypoint': d.get('leg'), 'waypoints': d.get('legs'),
                    'elapsed_s': d.get('elapsed_s'), 'final_targets': d.get('final_targets'),
                    'joints': {n: {k: v.get(k) for k in ('current_ticks', 'goal_ticks', 'target_ticks', 'following_error_ticks')} for n, v in (d.get('joints') or {}).items()},
                    'base_drive_phase': self.base.phase if self.base is not None and self.base.active else None, 'enabled_motors': self._enabled(),
                    'lease_remaining_s': 120.0, 'last_stop': self.last_stop,
                    'positions': {n: self.motors[n].present for n in self.arm_motors}}

    def _wait(self, event, generation):
        """Block (fast-forwarding sim time when real_time is False) until the motion/pulse finishes or STOP cancels it."""
        self._fast += 1
        try:
            while not event.wait(0.01):
                if generation != self.cancel_generation:
                    raise RuntimeError('STOP cancelled goal; no automatic resume')
        finally:
            self._fast -= 1
        if generation != self.cancel_generation:
            raise RuntimeError('STOP cancelled goal; no automatic resume')

    def _validate_move(self, request):
        """DirectJointClient._validate for direct_joint requests (ValueError wording verbatim)."""
        targets = request.get('waypoints') or [request['positions']]
        names = list(targets[0])
        if any(set(t) != set(names) for t in targets):
            raise ValueError('Every waypoint must name the same joints')
        if not set(names) <= set(self.arm_motors):
            raise ValueError('Unsupported position motor or wheel target')
        for n in names:
            lo, hi = self.calibration[n]
            if any(not lo + 4 <= t[n] <= hi - 4 for t in targets):
                raise ValueError('Target outside saved range plus4tickmargin: ' + n)
            if not self.motors[n].enabled:
                raise ValueError('Requested motor is released; explicitly enable it first: ' + n)
        arms = {n.split('_arm_')[0] for n in names}
        required = [m for m in self.arm_motors if m.split('_arm_')[0] in arms]
        missing = sorted(set(required) - set(self._enabled()))
        if missing:
            raise ValueError('Pickup requires all six joints of the commanded arm explicitly enabled: ' + json.dumps(missing))
        ready = self._readiness()
        faults = {n: ready['joint_blockers'][n] for n in names if n in ready['joint_blockers']}
        if faults:
            raise ValueError('Requested motor health/range blockers: ' + json.dumps(faults))

    def _command(self, request, wait=True):
        """DirectJointClient._command_locked for direct_joint / base_pulse requests."""
        if not self._command_lock.acquire(blocking=False):
            raise RuntimeError('Hardware command active; STOP remains independently available')
        try:
            busy_ok = request.get('replace') is True
            with self.lock:
                ready = self._readiness()
                if busy_ok and ready['blockers'] and all(b.startswith('OWNER_BUSY') for b in ready['blockers']):
                    ready = dict(ready, available_to_accept_authorized_command=True)
                if not ready['available_to_accept_authorized_command']:
                    return self._busy_result(ready)
                generation = self.cancel_generation
                self._poll_rows()
                if request['op'] == 'base_pulse':
                    check_base_request(request)
                    command_id = self._next_command_id()
                    pulse = _BasePulse(self, dict(request, id=command_id), self.world.now())
                    self.base = pulse
                    event = pulse.done
                else:
                    self._validate_move(request)
                    command_id = self._next_command_id()
                    # Dry run exactly like the real client (raises ValueError before anything moves).
                    _ArmMotion(self, dict(request, id=command_id), self.world.now())
                    if self.motion is not None and self.motion.active:
                        self.motion.halt({j: self.motors[j].present for j in self.motion.joints})
                    motion = _ArmMotion(self, dict(request, id=command_id), self.world.now())
                    self.motion = motion
                    for n in motion.joints:
                        self.motors[n].goal = motion.goal[n]
                    event = motion.done
                    if not wait:
                        return {'accepted': True, 'started': True, 'completed': False, 'command_id': command_id, 'owner_started': self.started,
                                'phase': 'moving', 'deadline_s': motion.deadline, 'waypoints': len(motion.legs), 'note': MOTION_NOTE, 'mode': 'direct_joint'}
            self._wait(event, generation)
            with self.lock:
                return self._finish_command(request, command_id)
        finally:
            self._command_lock.release()

    def _finish_command(self, request, command_id):
        if request['op'] == 'base_pulse':
            pulse = self.base
            if pulse.result is None:
                raise RuntimeError('Owner stopped: ' + str((self.last_stop or {}).get('reason') or self.error))
            self.last_completed = command_id
            return {'accepted': True, 'completed': True, 'command_id': command_id, 'owner_started': self.started, 'base_result': pulse.result['base_result'],
                    'owner_status_time': time.time(), 'motor_writes': 'canonical owner only', 'mode': 'base_pulse'}
        motion = self.motion
        if motion.command_id != command_id:
            raise RuntimeError('Command overwritten; cancelled')
        if motion.result is None:
            raise RuntimeError('Owner stopped: ' + str((self.last_stop or {}).get('reason') or self.error))
        self.last_completed = command_id
        self._poll_rows()
        current = motion.result
        final = (request.get('waypoints') or [request.get('positions')])[-1]
        measured = {n: self.motors[n].present for n in final}
        outcome = current['closure_outcome']
        if outcome in ('halted', 'contact_halt'):
            out = {'accepted': True, 'completed': False, 'halted': True, 'endpoint_reached': False, 'closure_outcome': outcome, 'holding': True,
                   'command_id': command_id, 'readbacks': measured, 'settle_residual_ticks': current['settle_residual_ticks'],
                   'contact': current.get('contact'), 'contact_note': current.get('contact_note'), 'mode': 'direct_joint'}
            if current.get('gripper_result') is not None:
                out['gripper_result'] = current['gripper_result']
            return out
        if outcome == 'settled_short':
            return {'accepted': True, 'completed': False, 'endpoint_reached': False, 'closure_outcome': 'settled_short', 'holding': True,
                    'command_id': command_id, 'owner_started': self.started, 'readbacks': measured, 'settle_residual_ticks': current['settle_residual_ticks'],
                    'execution_profile': EXECUTION_PROFILE, 'grasp_verified': False, 'owner_status_time': time.time(), 'reason': SHORT_REASON,
                    'motor_writes': 'canonical owner only', 'mode': 'direct_joint'}
        return self._completion(command_id, measured, endpoint_reached=current['endpoint_reached'], settle_residual_ticks=current['settle_residual_ticks'],
                                closure_outcome=outcome, duration_s_actual=motion.duration)

    def _execute(self, positions, duration_s, wait=True, replace=False):
        if type(duration_s) not in (int, float) or not math.isfinite(duration_s) or not 0 < duration_s <= 25:
            raise ValueError('Duration must be finite in (0,25]')
        if not isinstance(positions, dict) or not positions or any(type(q) is not int for q in positions.values()):
            raise ValueError('Nonempty integer encoder targets required')
        return self._command({'op': 'direct_joint', 'positions': positions, 'duration_s': duration_s, 'replace': replace is True}, wait=wait)

    def _execute_path_raw(self, waypoints, duration_s, wait=True, replace=False):
        if type(duration_s) not in (int, float) or not math.isfinite(duration_s) or not 0 < duration_s <= 60:
            raise ValueError('Path duration must be finite in (0,60]')
        if not isinstance(waypoints, list) or not waypoints or any(not isinstance(w, dict) or not w or any(type(q) is not int for q in w.values()) for w in waypoints):
            raise ValueError('Nonempty list of integer waypoint targets required')
        return self._command({'op': 'direct_joint', 'waypoints': waypoints, 'duration_s': duration_s, 'replace': replace is True}, wait=wait)

    def _execute_targets(self, positions, duration_s, wait=True, replace=False):
        """gemma_robot_tools.execute_targets: segments, moving joints together, a closing gripper last and alone."""
        with self.lock:
            self._poll_rows()
            rows = {n: {'Present_Position': m.present} for n, m in self.motors.items()}
        segments = paddle_target_segments(positions, rows, self.calibration)
        if not segments:
            return {'accepted': True, 'completed': True, 'no_op': True, 'endpoint_reached': True, 'motor_writes': 0,
                    'reason': 'Every requested joint is already within 2 ticks of its target'}
        closing = {n for n, t in positions.items() if n.endswith('gripper') and t < rows[n]['Present_Position'] - 2}
        closing_segments = [seg for seg in segments if set(seg) <= closing]
        moving_segments = [seg for seg in segments if not set(seg) <= closing]
        if closing_segments and (not wait or replace):
            raise ValueError('A closing gripper runs alone after the arm stops; send it as its own robot_set_gripper/move with wait=true')
        generation = self.cancel_generation
        results = []
        if moving_segments:
            if len(moving_segments) == 1:
                result = self._execute(moving_segments[0], duration_s, wait=wait, replace=replace)
            else:
                result = self._execute_path_raw(moving_segments, min(60, max(duration_s, .4 * len(moving_segments))), wait=wait, replace=replace)
            results.append(result)
            if not wait or not result.get('completed'):
                return dict(result, path_waypoints=moving_segments, simultaneous_joints=sorted({n for seg in moving_segments for n in seg}))
        for seg in closing_segments:
            if self.cancel_generation != generation:
                raise RuntimeError('STOP cancelled the remaining gripper closure; motors released, no automatic resume')
            result = self._execute(seg, duration_s)
            results.append(result)
            if not result.get('completed') or result.get('closure_outcome') == 'stationary_closure_unverified':
                break
        final = dict(results[-1])
        final.update(path_waypoints=moving_segments, closing_steps=closing_segments, simultaneous_joints=sorted({n for seg in moving_segments for n in seg}),
                     part_outcomes=[{'closure_outcome': r.get('closure_outcome'), 'readbacks': r.get('readbacks')} for r in results])
        return final

    def _execute_path(self, waypoints, duration_s, wait=True, replace=False):
        with self.lock:
            self._poll_rows()
            names = sorted({n for w in waypoints for n in w})
            start = {n: self.motors[n].present for n in names}
        path = expand_path(waypoints, start)
        for n in names:
            if any(n.endswith('gripper') and w[n] < p[n] - 2 for p, w in zip([start] + path, path)):
                raise ValueError('A path cannot close the gripper; close it with its own move once the arm has stopped')
        return dict(self._execute_path_raw(path, duration_s, wait=wait, replace=replace), path_waypoints=path)

    def _set_gripper(self, arm, position, duration_s):
        """gemma_robot_tools.set_gripper + DirectJointClient.set_gripper: <=300-tick parts, auto-enable of the whole arm,
        STOP cleanup when a part the sequence enabled fails."""
        if arm not in ('left', 'right') or type(position) is not int or type(duration_s) not in (int, float) or not math.isfinite(duration_s) or not 0 < duration_s <= 25:
            raise ValueError('Valid arm, integer gripper target and finite duration (0,25] required before activation')
        name = arm + '_arm_gripper'
        with self.lock:
            self._poll_rows()
            current = self.motors[name].present
            self._grip_call(arm, name, current, position)
            self._poll_rows()
            current = self.motors[name].present
        if abs(current - position) <= GRIPPER_CLOSE_CHUNK:
            return self._set_gripper_once(arm, position, duration_s)
        generation = self.cancel_generation
        pieces = -(-abs(current - position) // GRIPPER_CLOSE_CHUNK)
        parts = []
        result = None
        for i in range(1, pieces + 1):
            if self.cancel_generation != generation:
                raise RuntimeError('STOP cancelled the remaining gripper closure; motors released, no automatic resume')
            target = current - round((current - position) * i / pieces)
            result = self._set_gripper_once(arm, target, duration_s)
            parts.append({'target': target, 'closure_outcome': result.get('closure_outcome'), 'readback': (result.get('readbacks') or {}).get(name)})
            if not result.get('completed'):
                break
        return dict(result, closure_parts=parts, final_target=position)

    def _set_gripper_once(self, arm, position, duration):
        name = arm + '_arm_gripper'
        request = {'op': 'direct_joint', 'positions': {name: position}, 'duration_s': duration}
        with self.lock:
            ready = self._readiness()
            if not ready['available_to_accept_authorized_command']:
                raise RuntimeError('Gripper owner unavailable: ' + json.dumps(ready))
            arm_joints = [m for m in self.arm_motors if m.startswith(arm + '_arm_')]
            to_enable = [m for m in arm_joints if not self.motors[m].enabled]
            # Validate against the prospective (auto-enabled) state before touching anything.
            was = {m: self.motors[m].enabled for m in to_enable}
            for m in to_enable:
                self.motors[m].enabled = True
            try:
                self._validate_move(request)
                _ArmMotion(self, dict(request, id=1), self.world.now())
            finally:
                for m, v in was.items():
                    self.motors[m].enabled = v
            started = self.started
            generation = self.cancel_generation
        enabled_here = bool(to_enable)
        enable_attempted = False
        phase = 'validated'
        try:
            if enabled_here:
                phase = 'enabling'
                enable_attempted = True
                activation = self._set_motor_enable(to_enable, True)
                if not activation.get('completed'):
                    raise RuntimeError('Gripper enable not completed: ' + json.dumps(activation))
            if generation != self.cancel_generation:
                raise RuntimeError('Owner/STOP changed during gripper sequence')
            phase = 'moving'
            result = self._execute({name: position}, duration)
            if not result.get('completed'):
                if result.get('holding') and result.get('closure_outcome') in ('settled_short', 'contact_halt', 'stationary_closure_unverified'):
                    return dict(result, gripper_auto_enabled=enabled_here, auto_enabled_motors=to_enable, gripper=name, sequence_phase='stopped_short', note=GRIPPER_NOTE)
                raise RuntimeError('Gripper move not completed: ' + json.dumps(result))
            return dict(result, gripper_auto_enabled=enabled_here, auto_enabled_motors=to_enable, gripper=name, sequence_phase='completed')
        except BaseException as exc:
            cleanup = 'not_requested'
            if enable_attempted:
                try:
                    cleanup = self._stop()
                except Exception as cleanup_error:  # noqa: BLE001
                    cleanup = {'release_confirmed': False, 'error': str(cleanup_error)}
            raise RuntimeError('Gripper sequence failed in ' + phase + '; cleanup=' + json.dumps(cleanup) + '; ' + str(exc)) from exc

    def _drive_base(self, linear_m_s, angular_rad_s, duration_s):
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (linear_m_s, angular_rad_s, duration_s)) or not 0 < duration_s <= 3:
            raise ValueError('Finite linear_m_s, angular_rad_s and duration_s in (0,3] required')
        return self._command({'op': 'base_pulse', 'linear_m_s': linear_m_s, 'angular_rad_s': angular_rad_s, 'duration_s': duration_s})

    # ---------------------------------------------------------------- scripting helpers (tests, bench setup)

    def set_box_pose(self, forward_m, left_m, up_m, yaw_rad=0.0):
        """Teleport the box (robot frame, metres; up = box centre height) and zero its velocity."""
        with self.lock:
            if self.box_qadr is None:
                raise RuntimeError('scene has no free box')
            q = self.data.qpos[self.box_qadr:self.box_qadr + 7]
            q[0:3] = self.origin + self.np.array(_robot_to_model(forward_m, left_m, up_m)) - self.np.array(ROBOT_ORIGIN_MODEL)
            q[3:7] = [math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)]
            self.data.qvel[self.box_dadr:self.box_dadr + 6] = 0
            self.mj.mj_forward(self.model, self.data)

    def box_pose(self):
        with self.lock:
            f, l, u = (self.axes @ (self.data.xpos[self.box_body] - self.origin)).tolist()
            return {'forward_m': f, 'left_m': l, 'up_m': u}

    def claw_positions(self):
        with self.lock:
            return self._claws()

    def positions(self):
        with self.lock:
            self._poll_rows()
            return {n: m.present for n, m in self.motors.items()}

    def ranges(self):
        return {n: tuple(v) for n, v in self.calibration.items()}


def _minimal_tool(name):
    """Schema fallback when the captured real catalog is missing (keeps the bench running; shapes match the real server)."""
    arm = {'type': 'string', 'enum': ['left', 'right']}
    targets = {'type': 'object', 'minProperties': 1, 'properties': {n: {'type': 'integer'} for n in list(twin.JOINT_TABLE) + list(ARM_JOINTS)}, 'additionalProperties': False}
    specs = {
        'robot_list_motors': ({}, []),
        'robot_set_motor_enable': ({'names': {'type': 'array', 'items': {'type': 'string', 'enum': list(twin.JOINT_TABLE) + list(WHEELS)}, 'minItems': 1, 'maxItems': 16, 'uniqueItems': True},
                                   'enabled': {'type': 'boolean'}}, ['names', 'enabled']),
        'robot_get_state': ({'fresh': {'type': 'boolean'}}, []),
        'robot_get_cameras': ({'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': ['oak', 'phone', 'left_wrist', 'right_wrist']}, 'minItems': 1, 'maxItems': 4},
                              'revive': {'type': 'boolean'}}, []),
        'robot_get_clip': ({'camera': {'type': 'string', 'enum': ['oak', 'phone', 'left_wrist', 'right_wrist']}, 'seconds': {'type': 'number', 'minimum': 0.5, 'maximum': 4},
                           'fps': {'type': 'number', 'minimum': 1, 'maximum': 8}, 'max_width': {'type': 'integer', 'minimum': 160, 'maximum': 640}}, ['camera']),
        'robot_get_capabilities': ({}, []),
        'robot_get_depth': ({}, []),
        'robot_stop': ({}, []),
        'robot_move_joint_targets': ({'arm': arm, 'positions': targets, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25},
                                      'wait': {'type': 'boolean'}, 'replace': {'type': 'boolean'}}, ['arm', 'positions', 'duration_s']),
        'robot_move_path': ({'arm': arm, 'waypoints': {'type': 'array', 'minItems': 1, 'maxItems': 12, 'items': targets},
                             'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 60}, 'wait': {'type': 'boolean'}, 'replace': {'type': 'boolean'}},
                            ['arm', 'waypoints', 'duration_s']),
        'robot_get_motion': ({}, []),
        'robot_halt_motion': ({}, []),
        'robot_set_gripper': ({'arm': arm, 'position_ticks': {'type': 'integer'}, 'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 25}}, ['arm', 'position_ticks']),
        'robot_move_base': ({'linear_m_s': {'type': 'number', 'minimum': -0.02, 'maximum': 0.02}, 'angular_rad_s': {'type': 'number', 'minimum': -0.16, 'maximum': 0.16},
                             'duration_s': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 3}}, ['linear_m_s', 'angular_rad_s', 'duration_s']),
    }
    props, required = specs[name]
    return {'type': 'function', 'function': {'name': name, 'description': f'{name} (simulated)',
                                             'parameters': {'type': 'object', 'properties': props, 'required': required, 'additionalProperties': False}}}


APPROXIMATIONS = """Where SimRobot differs from the real paddle-success-v1 owner:
- Goals ramp linearly (100 ticks/s at most, >= 2 s per leg) instead of 40-tick writes every >= 0.4 s; guards run at 10 Hz polls.
- The following-error fault (|goal - present| > 96) is checked at each 10 Hz poll, like the owner's tick; no persistence.
- Present_Load is 3.6 x the servo's position error in ticks (from the real 300-400 at 96 ticks), so a joint pushed into
  something rigid trips the 96-tick release before the 350-load contact halt, as on the real robot on 2026-10-08;
  arm contact_halt is therefore practically unreachable here. Present_Velocity is the per-poll average in ticks/s;
  Moving = |velocity| >= 30 ticks/s (3 ticks per 0.1 s poll), which is also the 'still' test for settling.
- A closing gripper reports closure_outcome 'contact_halt' (plus gripper_result) when the jaw is quiet 0.3 s and >= 40 ticks
  behind its goal; the real owner names that 'stationary_closure_unverified'. Steps advance as soon as the jaw is quiet
  (about 0.4 s per 10 ticks), as the real executor does; 1.5 s per step is only the deadline budget. A closure that stops
  less than 40 ticks short reports settled_short (holding), as on the real robot.
- Jaws meet at raw gripper range_min + 82 ticks (1355 on the left, where the real pads met on 8-9 October); the model's jaw
  range then covers ticks 1355..2738. Commanding below 1355 (e.g. the real close target 1340) presses the jaws together (the
  joint limit holds them); the 3.5 mm flap stops them about 25 ticks earlier, as the real 3 mm flap did.
- Servo stiffness KP_ARM=20 N m/rad (saturating at 2.94 N m, i.e. 96 ticks of error), KP_GRIP=6; the real servos' compliance
  is unmeasured. Settle corrections (up to 3 x 40 ticks, 57 max overdrive) are ported from the real executor.
- Released joints keep 0.4 N m of gear friction: the folded arms rest; an extended released arm sags over a few seconds.
- MuJoCo noslip_iterations=5 so a pinched box does not creep out of the soft jaw contacts.
- Default scene (box_scene preset 'real', the 9 October carton): an open box, rim 77 cm, a 16 cm right flap (the target,
  'flap_hinge') and a 16 cm far flap that leans in (flaps never collide with each other). The crease springs back toward a
  rest angle that only moves while the crease is loaded (box_scene.PLASTIC): a flap held flat and released returns most of
  the way, one carried past ~100 deg into the opening and held ~5 s stays down; a crease pressed by a robot contact near the
  hinge, or covered by the far flap folded past 80 deg, also sets. score() splits the flap's inward turn into degrees moved
  while the pads pinched it vs while another robot part touched it (fold_by_pinch allows 10 deg of the latter). FIDELITY:
  per-close meeting-point jitter, the right gripper's mid-travel sticking, a 5 deg shoulder_lift bias (the arm model reads
  ~3 cm high near the box) and a 90 deg wrist_roll offset (unrolled jaws open left/right). None of the crease numbers is
  measured beyond the attempts quoted in box_scene. The 'near7' preset is the 8 October scene below, without FIDELITY.
- near7: the box's flap is 7 cm: a 5 cm panel on the crease hinge (box_scene.FLAP_SEGMENTS_M) plus a 2 cm top strip on a nearly free
  joint that stands in for cardboard crushing between the pads, so a pinch on the top 2 cm can carry the flap round its hinge
  while a deeper pinch locks it to the jaws. The crease folds under about 0.9 N at the edge and stays where it is put (its
  friction beats its spring); none of this is measured on the real carton. Jaw-flap contacts use friction 1.2, 5 mm torsion.
- score(): box_held_now = both jaws of one arm touch the box or flap and the box centre is >= 3 cm up; flap_angle_deg is
  the crease angle (0 vertical, + inward, 90 flat on the top); flap_folded = >= 75 deg with the box upright and resting
  on the table and no jaw touching the flap (it stays folded on its own).
- No lease/idle-hold timeout, no camera-freshness gate, no temperature or voltage checks (constant 12.0 V), no calibration
  mismatch/read-only paths, no robot_get_handoff/robot_get_execution/calibration tools.
- Base pulses move the table and box the opposite way (fixed-base model): no slip, no wheel dynamics; wheel ticks follow the
  command exactly; a box touching a jaw stays with the robot. The pulse blocks for duration + 0.8 s of sim time.
- Sim time between commands follows wall time (1x); only blocking calls fast-forward when real_time=False, so tick-level
  results differ run to run. Step hooks (camera recording) run at most every 0.5 s of sim time while fast-forwarding.
- Head motors are frozen at the sample's positions; the fallback scene's cameras are approximate poses.
- score()['moves'] counts accepted robot_move_joint_targets/robot_move_path/opening robot_set_gripper calls,
  'gripper_closes' closing robot_set_gripper calls, 'base_pulses' robot_move_base calls; internal 300-tick parts are not counted.
"""
