"""Reach IK for the XLeRobot's SO-101 arms: "put the LEFT claw at forward 0.30, left 0.12, up 0.84, level".

Input is a claw-tip target in the digital twin's ROBOT frame (farm/sim/xlerobot_twin.py: origin on the
floor below the midpoint between the two shoulder-pan axes, +forward, +left, +up, metres); output is
encoder ticks for shoulder_pan, shoulder_lift, elbow_flex and wrist_flex in the twin's candidate
``feetech_degrees_v1`` mapping (optionally overridden per motor by a joint map). Nothing here talks
to the robot: the ticks go to the chat's motion tools (``plan_steps`` splits them into
``robot_move_path`` waypoints).

How it solves (``method`` 'analytic+twin-refine'):

1. Shoulder pan from the horizontal geometry. Each arm's pan axis is vertical at forward 0, left
   +-SHOULDER_LEFT_M (273 mm assembly spacing: 0.1365 m each side). The whole arm lies in the vertical plane
   through that axis, so pan = atan2 of the target's lateral offset from the axis over its forward
   offset. A positive feetech pan swings EITHER arm toward the robot's RIGHT (verified on the twin for
   both arms), so mirrored targets give opposite pans.
2. In that plane, the shoulder-lift axis sits LIFT_AHEAD_M (0.0306 m) ahead of the pan axis at
   SHOULDER_UP_M (0.894 m). The claw tip is TIP_ALONG_M (0.165 m) from the wrist-flex axis along the
   hand (wrist-roll) axis and TIP_BELOW_M (0.008 m) below it; the hand's elevation is exactly
   -(lift + elbow + wrist_flex) degrees, so ``pitch_deg`` (elevation of the hand above level,
   positive = claw tip UP, negative = down) fixes the wrist: wrist_flex = -lift - elbow - pitch_deg.
   That is the upstream teleop rule ``wrist_flex = -shoulder_lift - elbow_flex + pitch`` with the
   opposite sign of ``pitch`` (upstream's positive pitch points the claw down). Subtracting the hand
   offset gives the wrist-flex axis point, and the upstream 2-link inverse kinematics (ported below)
   gives lift and elbow; its output degrees ARE our feetech degrees (checked on the twin, see
   ``arm_plane_forward`` and the tests).
3. Degrees -> ticks with the twin's mapping (``xlerobot_twin.motor_ticks``), every tick clamped into the
   commandable band [min + MARGIN_TICKS, max - MARGIN_TICKS] (the pickup profile's 40-tick margin).
4. Refinement on the twin's own forward kinematics (``claw_positions_many``): damped Gauss-Newton on
   the pan/lift/elbow ticks (wrist tracking the pitch rule), Jacobian by finite differences of
   FD_STEP_TICKS, until the predicted tip is within REFINE_TOL_M or MAX_ITERATIONS. The analytic model
   and the twin agree to about a millimetre, so this mostly removes the IK's near-singular sensitivity
   and tick rounding; the reported ``predicted``/``error_m`` are the twin's FK at the returned integer
   ticks. ``ok`` needs ``error_m`` < OK_TOL_M and every joint inside its band.

Upstream reference (Apache-2.0, Vector-Wangel/XLeRobot, commit b017b5e6354bd9f61f4247a920c72622ca0aade0):
``software/src/model/SO101Robot.py`` (class SO101Kinematics: ``inverse_kinematics``/``forward_kinematics``,
l1 = 0.1159, l2 = 0.1350, offsets atan2(0.028, 0.11257) and atan2(0.0052, 0.1349)) and
``software/examples/7_xlerobot_2wheels_teleop_joycon_smooth.py`` (hand target in metres, wrist pitch
rule). Both link lengths and offsets match the twin's joint anchors to 0.1 mm. Note that upstream's
``forward_kinematics`` is NOT the inverse of its ``inverse_kinematics``: its forearm term reads
``theta1 + theta2 - pi`` where the IK (and the robot) has ``theta1 - theta2``, so IK(FK(j2, j3)) returns
(j2, 2 * theta2_offset_deg - j3) = (j2, 32.35 - j3). Both are ported verbatim for reference;
``arm_plane_forward`` is the FK with that term fixed, which the twin agrees with to 0.1 mm. Do not use
``farm/vendor/so101_kinematics.py`` here: its IK was altered to round-trip with the broken FK and gives
elbow angles tens of degrees off the model. Upstream's IK clamps (lift -107.7..95.7 deg, elbow
-101.5..90 deg) are reported as a reachability problem, never used to bend a solution.

Limits of the model: no collision checking beyond the floor and the cart body (a target below the
cart's top deck inside its footprint is refused); the head mast and the other arm are not checked.
Use the cameras for clearance. The tick mapping is the unvalidated candidate until a joint map says
``validated: true`` (``mapping_validated``).

Thread-safe: all MuJoCo work goes through the twin's single worker thread; this module keeps no
mutable state. Warm solve: a few milliseconds (one batched FK call per iteration).
"""
from __future__ import annotations

import math

import numpy as np

from farm.sim import xlerobot_twin as twin
from farm.kinematics.xlerobot_geometry import SHOULDER_LEFT_M, SHOULDER_UP_M

METHOD = 'analytic+twin-refine'
UPSTREAM = ('Vector-Wangel/XLeRobot@b017b5e6354bd9f61f4247a920c72622ca0aade0 software/src/model/SO101Robot.py '
            '(Apache-2.0)')

# Upstream SO101Kinematics constants (metres / radians).
L1 = 0.1159                                            # shoulder-lift axis -> elbow axis
L2 = 0.1350                                            # elbow axis -> wrist-flex axis
THETA1_OFFSET = math.atan2(0.028, 0.11257)             # upper arm leans 13.97 deg forward of vertical at lift 0
THETA2_OFFSET = math.atan2(0.0052, 0.1349) + THETA1_OFFSET
FOREARM_ZERO_DEG = math.degrees(math.atan2(0.0052, 0.1349))  # forearm elevation at lift 0, elbow 0: 2.21 deg
LIFT_LIMITS_DEG = (90 - math.degrees(3.45), 90 - math.degrees(-0.1))     # upstream IK clamps: -107.7 .. 95.7
ELBOW_LIMITS_DEG = (math.degrees(-0.2) - 90, math.degrees(math.pi) - 90)  # -101.5 .. 90

# Twin geometry in the robot frame (measured on the vendored model, see tests/test_so101_reach.py).
LIFT_AHEAD_M = 0.0306                                  # lift axis ahead of the pan axis, along the pan direction
TIP_ALONG_M = 0.165                                    # wrist-flex axis -> claw tip along the hand axis (0.060 + 0.105)
TIP_BELOW_M = 0.008                                    # claw tip below the hand axis (twin TIP_POS x)
CART = {'forward_m': (-0.285, 0.108), 'left_m': (-0.234, 0.234), 'top_m': 0.776}  # Raskog body AABB
ARMS = ('left', 'right')
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex')

MARGIN_TICKS = 40          # the pickup controller's commandable band inside the saved range
MAX_STEP_TICKS = 300       # plan_steps default: ticks per joint per waypoint
FD_STEP_TICKS = 8.0        # finite-difference step for the Jacobian (0.7 deg)
MAX_ITERATIONS = 10
REFINE_TOL_M = 0.0005      # stop refining below this tip error (about one tick of resolution)
OK_TOL_M = 0.005           # ok needs the final error below this
DAMPING = 1e-3             # relative Levenberg damping on J^T J


# ---------------------------------------------------------------- upstream port (verbatim formulas)

def upstream_inverse_kinematics(x, y, l1=L1, l2=L2):
    """SO101Kinematics.inverse_kinematics, ported verbatim: wrist point (x forward, y up, metres, from the
    shoulder-lift axis) -> (shoulder_lift, elbow_flex) degrees. Out-of-reach points are scaled onto the
    boundary and the joints clamped to upstream's URDF limits, exactly as upstream does."""
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    r = math.sqrt(x ** 2 + y ** 2)
    r_max = l1 + l2
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min
    cos_theta2 = -(r ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.pi - math.acos(cos_theta2)
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma
    joint2 = theta1 + theta1_offset
    joint3 = theta2 + theta2_offset
    joint2 = max(-0.1, min(3.45, joint2))
    joint3 = max(-0.2, min(math.pi, joint3))
    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)
    joint2_deg = 90 - joint2_deg
    joint3_deg = joint3_deg - 90
    return joint2_deg, joint3_deg


def upstream_forward_kinematics(joint2_deg, joint3_deg, l1=L1, l2=L2):
    """SO101Kinematics.forward_kinematics, ported verbatim. Not the inverse of the IK above (see module
    docstring); kept for reference and the round-trip test."""
    joint2_rad = math.radians(90 - joint2_deg)
    joint3_rad = math.radians(joint3_deg + 90)
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    theta1 = joint2_rad - theta1_offset
    theta2 = joint3_rad - theta2_offset
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2 - math.pi)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2 - math.pi)
    return x, y


# ---------------------------------------------------------------- consistent arm-plane model

def arm_plane_forward(lift_deg, elbow_deg):
    """Wrist-flex axis (x forward along the pan direction, y up; metres from the shoulder-lift axis) for
    feetech lift/elbow degrees. The forward model ``upstream_inverse_kinematics`` actually inverts, and
    the one the twin follows: upper arm at theta1 = 90 - lift - 13.97 deg, forearm at theta1 - theta2
    with theta2 = elbow + 90 - 16.18 deg (so the forearm's elevation is 2.21 - lift - elbow deg)."""
    theta1 = math.radians(90.0 - lift_deg) - THETA1_OFFSET
    theta2 = math.radians(elbow_deg + 90.0) - THETA2_OFFSET
    return (L1 * math.cos(theta1) + L2 * math.cos(theta1 - theta2),
            L1 * math.sin(theta1) + L2 * math.sin(theta1 - theta2))


def hand_elevation_deg(lift_deg, elbow_deg, wrist_deg):
    """Elevation of the hand (wrist-roll) axis above level, degrees; exact on the twin."""
    return -(lift_deg + elbow_deg + wrist_deg)


def tip_offset(pitch_deg):
    """Claw tip minus wrist-flex axis point in the arm plane (x forward, y up) for a hand elevation."""
    p = math.radians(pitch_deg)
    return (TIP_ALONG_M * math.cos(p) + TIP_BELOW_M * math.sin(p),
            TIP_ALONG_M * math.sin(p) - TIP_BELOW_M * math.cos(p))


def arm_plane_tip(lift_deg, elbow_deg, wrist_deg):
    """Claw tip in the arm plane (x forward, y up, metres from the shoulder-lift axis)."""
    wx, wy = arm_plane_forward(lift_deg, elbow_deg)
    ox, oy = tip_offset(hand_elevation_deg(lift_deg, elbow_deg, wrist_deg))
    return wx + ox, wy + oy


def analytic_tip_robot_frame(arm, pan_deg, lift_deg, elbow_deg, wrist_deg):
    """Claw tip in the robot frame from feetech degrees, by the analytic model alone (no twin)."""
    x, y = arm_plane_tip(lift_deg, elbow_deg, wrist_deg)
    d = LIFT_AHEAD_M + x
    a = math.radians(-pan_deg)  # +pan swings toward the right (negative left)
    return {'forward_m': d * math.cos(a), 'left_m': SHOULDER_LEFT_M[arm] + d * math.sin(a), 'up_m': SHOULDER_UP_M + y}


# ---------------------------------------------------------------- helpers

def _motor(arm, joint):
    return f'{arm}_arm_{joint}'


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _band(ranges, motor):
    rng = twin._range(ranges.get(motor))
    if rng is None:
        raise ValueError(f'no saved range for {motor}; the solver needs ranges for {", ".join(JOINTS)}')
    lo, hi = rng[0] + MARGIN_TICKS, rng[1] - MARGIN_TICKS
    if lo > hi:
        raise ValueError(f'{motor}: saved range {rng} is narrower than twice the {MARGIN_TICKS}-tick margin')
    return lo, hi


def _target(target):
    if not isinstance(target, dict):
        raise ValueError("target must be a dict {'forward_m', 'left_m', 'up_m'} in the robot frame")
    missing = [k for k in ('forward_m', 'left_m', 'up_m') if k not in target]
    if missing:
        raise ValueError(f'target lacks {", ".join(missing)}')
    return {k: _finite(target[k], f'target {k}') for k in ('forward_m', 'left_m', 'up_m')}


def analytic_solve(arm, target, pitch_deg=0.0):
    """Steps 1-2: feetech degrees for the four joints plus geometry diagnostics (no twin, no clamping).

    Returns {'degrees': {motor: deg}, 'pan_deg', 'radial_m', 'height_m', 'wrist_radius_m', 'reach_m',
    'problems': [str, ...]} where reach_m is the tip's distance from the shoulder point (pan axis at lift
    height, as the twin reports) and problems lists the geometric reasons the target cannot be reached.
    """
    if arm not in ARMS:
        raise ValueError(f"arm must be 'left' or 'right', not {arm!r}")
    t = _target(target)
    pitch_deg = _finite(pitch_deg, 'pitch_deg')
    problems = []
    dx, dy = t['forward_m'], t['left_m'] - SHOULDER_LEFT_M[arm]
    d = math.hypot(dx, dy)
    pan_deg = math.degrees(math.atan2(-dy, dx)) if d > 1e-9 else 0.0
    radial = d - LIFT_AHEAD_M
    height = t['up_m'] - SHOULDER_UP_M
    reach = math.sqrt(dx * dx + dy * dy + height * height)
    ox, oy = tip_offset(pitch_deg)
    wx, wy = radial - ox, height - oy
    wr = math.hypot(wx, wy)
    if t['up_m'] < 0:
        problems.append(f"below the floor (up_m {t['up_m']:.3f} < 0)")
    elif (CART['forward_m'][0] <= t['forward_m'] <= CART['forward_m'][1]
          and CART['left_m'][0] <= t['left_m'] <= CART['left_m'][1] and t['up_m'] < CART['top_m']):
        problems.append(f"inside the cart (up_m {t['up_m']:.3f} is below its {CART['top_m']:.3f} m deck within its footprint)")
    if dx < 0:
        problems.append(f"behind the shoulder (forward_m {t['forward_m']:.3f} < 0; pan would be {pan_deg:+.0f} deg)")
    if wr > L1 + L2:
        max_tip = L1 + L2 + math.hypot(ox, oy)
        problems.append(f'unreachable: {reach:.3f} m from the shoulder; max reach about {LIFT_AHEAD_M + max_tip:.3f} m '
                        f'at this pitch (wrist point {wr:.3f} m from the lift axis, arm {L1 + L2:.3f} m)')
    elif wr < abs(L1 - L2):
        problems.append(f'too close to the shoulder (wrist point {wr:.3f} m from the lift axis, minimum {abs(L1 - L2):.3f} m)')
    lift, elbow = upstream_inverse_kinematics(wx, wy)
    for name, value, limits in (('shoulder_lift', lift, LIFT_LIMITS_DEG), ('elbow_flex', elbow, ELBOW_LIMITS_DEG)):
        if value <= limits[0] + 1e-9 or value >= limits[1] - 1e-9:
            problems.append(f'{name} would be at the SO-101 joint limit ({value:.1f} deg, '
                            f'model limits {limits[0]:.1f}..{limits[1]:.1f})')
    wrist = -lift - elbow - pitch_deg
    degrees = {_motor(arm, 'shoulder_pan'): pan_deg, _motor(arm, 'shoulder_lift'): lift,
               _motor(arm, 'elbow_flex'): elbow, _motor(arm, 'wrist_flex'): wrist}
    return {'degrees': degrees, 'pan_deg': pan_deg, 'radial_m': radial, 'height_m': height,
            'wrist_radius_m': wr, 'reach_m': reach, 'problems': problems}


# ---------------------------------------------------------------- twin refinement

class _Pose:
    """Ticks for one arm as the twin wants them, with the pitch rule folded in."""

    def __init__(self, arm, current_ticks, ranges, joint_map, pitch_deg, bands):
        self.arm, self.ranges, self.joint_map, self.pitch, self.bands = arm, ranges, joint_map, pitch_deg, bands
        self.base = {k: v for k, v in current_ticks.items()}
        self.motors = [_motor(arm, j) for j in JOINTS]
        # degree <-> tick slope per motor (sign from the joint map), so the wrist can follow the pitch rule in ticks
        self.slope = {m: twin.motor_ticks({m: 1.0}, ranges, joint_map)[m] - twin.motor_ticks({m: 0.0}, ranges, joint_map)[m]
                      for m in self.motors}

    def degrees(self, ticks3):
        """Feetech degrees of pan/lift/elbow from their ticks, and the wrist from the pitch rule."""
        angles, _, _ = twin.motor_angles({m: t for m, t in zip(self.motors[:3], ticks3)}, self.ranges, self.joint_map)
        pan, lift, elbow = (angles[m] for m in self.motors[:3])
        return pan, lift, elbow, -lift - elbow - self.pitch

    def ticks(self, ticks3):
        pan, lift, elbow, wrist = self.degrees(ticks3)
        wrist_tick = twin.motor_ticks({self.motors[3]: wrist}, self.ranges, self.joint_map)[self.motors[3]]
        full = dict(self.base)
        full.update({m: float(t) for m, t in zip(self.motors[:3], ticks3)})
        full[self.motors[3]] = wrist_tick
        return full

    def clamp3(self, ticks3):
        return np.array([min(max(t, self.bands[m][0]), self.bands[m][1]) for m, t in zip(self.motors[:3], ticks3)])

    def evaluate(self, tick_sets):
        """Twin FK for several tick sets: list of (tip xyz array, claws dict)."""
        claws = twin.claw_positions_many(tick_sets, self.ranges, joint_map=self.joint_map)
        key = f'{self.arm}_arm'
        return [(np.array([c[key]['forward_m'], c[key]['left_m'], c[key]['up_m']]), c) for c in claws]


def _refine(pose, ticks3, goal):
    """Damped Gauss-Newton on pan/lift/elbow ticks against the twin's FK. Returns (ticks3, iterations)."""
    ticks3 = pose.clamp3(np.asarray(ticks3, dtype=float))
    iterations = 0
    tip, _ = pose.evaluate([pose.ticks(ticks3)])[0]
    error = goal - tip
    while np.linalg.norm(error) > REFINE_TOL_M and iterations < MAX_ITERATIONS:
        iterations += 1
        probes = []
        for i in range(3):
            step = ticks3.copy()
            step[i] += FD_STEP_TICKS if step[i] + FD_STEP_TICKS <= pose.bands[pose.motors[i]][1] else -FD_STEP_TICKS
            probes.append(step)
        results = pose.evaluate([pose.ticks(p) for p in probes])
        jac = np.column_stack([(r[0] - tip) / (p[i] - ticks3[i]) for i, (p, r) in enumerate(zip(probes, results))])
        jtj = jac.T @ jac
        lam = DAMPING * (np.trace(jtj) / 3.0 + 1e-18)
        try:
            delta = np.linalg.solve(jtj + lam * np.eye(3), jac.T @ error)
        except np.linalg.LinAlgError:
            break
        candidate = pose.clamp3(ticks3 + delta)
        new_tip, _ = pose.evaluate([pose.ticks(candidate)])[0]
        new_error = goal - new_tip
        if np.linalg.norm(new_error) >= np.linalg.norm(error) - 1e-9:
            # No progress (band edge or a kink): halve the step once, then stop.
            candidate = pose.clamp3(ticks3 + 0.5 * delta)
            new_tip, _ = pose.evaluate([pose.ticks(candidate)])[0]
            new_error = goal - new_tip
            if np.linalg.norm(new_error) >= np.linalg.norm(error) - 1e-9:
                break
        ticks3, tip, error = candidate, new_tip, new_error
    return ticks3, iterations


# ---------------------------------------------------------------- public API

def solve_reach(arm, target, current_ticks, ranges, *, pitch_deg=0.0, joint_map=None, roll_ticks=None):
    """Joint ticks that put ``arm``'s claw tip at ``target`` (robot frame, metres) with the hand at
    ``pitch_deg`` elevation (0 = level, + = tip up).

    ``current_ticks``/``ranges`` are what the twin takes (motor -> ticks, motor -> (min, max) or
    {'range_min', 'range_max'}); the wrist roll, gripper and the other arm stay at ``current_ticks``
    (``roll_ticks`` sets the wrist roll instead). Returns a dict; see the module docstring for the method
    and ``describe`` for a one-line summary:

    {'ok': bool, 'arm', 'target', 'pitch_deg', 'ticks': {motor: int ...} (the four joints, plus
     wrist_roll when roll_ticks is given), 'degrees': {motor: float} (feetech degrees of those ticks),
     'predicted': {'forward_m', 'left_m', 'up_m'} (twin FK at 'ticks'), 'error_m', 'reach_m',
     'analytic_error_m' (the analytic solution's error before refinement), 'iterations', 'clamped': [motor...]
     (joints the band limited), 'band': {motor: [lo, hi]}, 'reason': str | None, 'method', 'mapping',
     'mapping_validated', 'model'}

    Raises ValueError for a bad arm/target/ranges/joint_map; geometric impossibility is reported in
    ``reason`` with ok False (the ticks are still the best in-band attempt, with their predicted tip).
    """
    analytic = analytic_solve(arm, target, pitch_deg)
    twin._check_joint_map(joint_map)
    motors = [_motor(arm, j) for j in JOINTS]
    bands = {m: _band(ranges, m) for m in motors}
    roll_motor = _motor(arm, 'wrist_roll')
    if roll_ticks is not None:
        bands[roll_motor] = _band(ranges, roll_motor)
        roll_ticks = _finite(roll_ticks, 'roll_ticks')
    base = {k: v for k, v in current_ticks.items() if k not in twin.IGNORED_MOTORS}
    if roll_ticks is not None:
        base[roll_motor] = roll_ticks
    pose = _Pose(arm, base, ranges, joint_map, float(pitch_deg), bands)
    goal = np.array([target['forward_m'], target['left_m'], target['up_m']], dtype=float)

    raw = twin.motor_ticks(analytic['degrees'], ranges, joint_map)
    raw3 = np.array([raw[m] for m in motors[:3]])
    if roll_ticks is not None:
        raw[roll_motor] = roll_ticks
    clamped = [m for m in raw if not bands[m][0] <= raw[m] <= bands[m][1]]
    analytic_tip, _ = pose.evaluate([pose.ticks(raw3)])[0] if not clamped else pose.evaluate([pose.ticks(pose.clamp3(raw3))])[0]
    analytic_error = float(np.linalg.norm(goal - analytic_tip))

    ticks3, iterations = _refine(pose, raw3, goal)
    final = pose.ticks(ticks3)
    out_ticks = {m: int(round(final[m])) for m in motors}
    if roll_ticks is not None:
        out_ticks[roll_motor] = int(round(roll_ticks))
    # Rounding and the wrist rule can leave a tick just outside the band: clamp, and note it.
    for m in list(out_ticks):
        lo, hi = bands[m]
        if not lo <= out_ticks[m] <= hi:
            out_ticks[m] = int(min(max(out_ticks[m], lo), hi))
            if m not in clamped:
                clamped.append(m)
    full = dict(base)
    full.update(out_ticks)
    tip, claws = pose.evaluate([full])[0]
    error = float(np.linalg.norm(goal - tip))
    degrees, _, _ = twin.motor_angles({m: out_ticks[m] for m in motors}, ranges, joint_map)

    ok = error < OK_TOL_M and not clamped
    reason = None
    if not ok:
        problems = list(analytic['problems'])
        for m in clamped:
            lo, hi = bands[m]
            problems.append(f'outside joint ranges ({m} needs {raw[m]:.0f} ticks, commandable band {lo:.0f}..{hi:.0f})')
        if not problems:
            problems.append(f'refinement left {error * 1000:.1f} mm of error after {iterations} iterations')
        reason = '; '.join(problems)
    return {
        'ok': ok,
        'arm': arm,
        'target': {k: float(target[k]) for k in ('forward_m', 'left_m', 'up_m')},
        'pitch_deg': float(pitch_deg),
        'ticks': out_ticks,
        'degrees': {m: float(degrees[m]) for m in motors},
        'predicted': {'forward_m': float(tip[0]), 'left_m': float(tip[1]), 'up_m': float(tip[2])},
        'error_m': error,
        'reach_m': float(claws[f'{arm}_arm']['reach_m']),
        'analytic_error_m': analytic_error,
        'iterations': iterations,
        'clamped': clamped,
        'band': {m: [int(lo), int(hi)] for m, (lo, hi) in bands.items()},
        'reason': reason,
        'method': METHOD,
        'mapping': claws['mapping'],
        'mapping_validated': claws['mapping_validated'],
        'model': claws['model'],
    }


def plan_steps(current_ticks, target_ticks, max_step=MAX_STEP_TICKS):
    """Split a move into proportional multi-joint waypoints for robot_move_path.

    Every waypoint names every joint in ``target_ticks`` (the owner refuses paths whose waypoints
    differ), each joint moves the same fraction of its travel per waypoint, no joint moves more than
    ``max_step`` ticks between consecutive waypoints, and the last waypoint is exactly ``target_ticks``.
    Returns a list of {motor: int} (at least one waypoint, even for no motion).
    """
    if not isinstance(max_step, (int, float)) or isinstance(max_step, bool) or not max_step > 0:
        raise ValueError('max_step must be a positive number of ticks')
    missing = [m for m in target_ticks if twin._tick(current_ticks.get(m)) is None]
    if missing:
        raise ValueError(f'plan_steps: current_ticks lacks {", ".join(missing)}')
    start = {m: float(current_ticks[m]) for m in target_ticks}
    goal = {m: int(round(_finite(t, m))) for m, t in target_ticks.items()}
    span = max((abs(goal[m] - start[m]) for m in goal), default=0.0)
    count = max(1, math.ceil(span / max_step - 1e-9))
    steps = []
    for i in range(1, count + 1):
        if i == count:
            steps.append(dict(goal))
        else:
            steps.append({m: int(round(start[m] + (goal[m] - start[m]) * i / count)) for m in goal})
    return steps


def describe(result):
    """One line for a human: target, verdict, predicted error, ticks."""
    t = result['target']
    pitch = result.get('pitch_deg', 0.0)
    attitude = 'level' if abs(pitch) < 0.5 else f'pitch {pitch:+.0f} deg ({"tip up" if pitch > 0 else "tip down"})'
    arm = result['arm']
    ticks = ' '.join(f"{m.replace(f'{arm}_arm_', '')}={v}" for m, v in result['ticks'].items())
    head = (f"{arm.upper()} claw -> forward {t['forward_m']:.3f} left {t['left_m']:.3f} up {t['up_m']:.3f} m, {attitude}: ")
    if result['ok']:
        return head + (f"ok, twin predicts {result['error_m'] * 1000:.1f} mm off target "
                       f"(reach {result['reach_m']:.3f} m, {result['mapping']}"
                       f"{'' if result['mapping_validated'] else ', unvalidated mapping'}); ticks {ticks}")
    return head + f"NOT ok: {result['reason']}; best in-band attempt {result['error_m'] * 1000:.0f} mm off, ticks {ticks}"
