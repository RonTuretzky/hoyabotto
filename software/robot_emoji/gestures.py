"""Gesture definitions (gestures.json) and the robot_move_path plan built from them.

A gesture is checked twice: its shape when the file loads (known joints, integer ticks, legs the owner runs as
one continuous piece), and its targets against the live commandable ranges before any motor is enabled. Targets
are never clamped; only the return to the arm's resting pose is, because a released joint may sag outside the
commandable range and the owner only accepts targets inside it.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

DEFAULT_PATH = Path(__file__).with_name('gestures.json')
ARMS = ('left', 'right', 'head')
HEAD_JOINTS = ('pan', 'tilt')
HEAD_NAMES = {'pan': 'head_motor_1', 'tilt': 'head_motor_2'}
HEAD_MAX_LEG_TICKS = 200
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
GESTURE_JOINTS = ARM_JOINTS[:-1]   # no gripper: a closing gripper cannot run inside a path
MAX_LEG_TICKS = 341                # the pickup owner's per-leg limit; longer legs are split into 280-tick pieces
SPLIT_TICKS = 280
STEP_TICKS, STEP_S = 40, 0.4       # owner ramp: 40-tick steps, no faster than every 0.4 s
MAX_PATH_WAYPOINTS = 12            # robot_move_path input limit
MAX_PATH_S = 60


class GestureError(ValueError):
    pass


@dataclass(frozen=True)
class Gesture:
    key: str
    emoji: str
    label: str
    arm: str
    raise_path: tuple
    motion: tuple
    verified_on_hardware: bool
    relative_joints: tuple = ()

    def joints(self):
        return sorted({j for w in self.raise_path + self.motion for j in w})

    def public(self):
        return {'key': self.key, 'emoji': self.emoji, 'label': self.label}


def _waypoints(key, part, value, allowed=GESTURE_JOINTS):
    if not isinstance(value, list) or not 1 <= len(value) <= MAX_PATH_WAYPOINTS:
        raise GestureError(f'{key}.{part}: 1..{MAX_PATH_WAYPOINTS} waypoints required')
    out = []
    for i, w in enumerate(value):
        if not isinstance(w, dict) or not w:
            raise GestureError(f'{key}.{part}[{i}]: a non-empty joint -> ticks object required')
        for j, q in w.items():
            if j not in allowed:
                raise GestureError(f'{key}.{part}[{i}]: unknown or excluded joint {j!r}; allowed {allowed}')
            if type(q) is not int:
                raise GestureError(f'{key}.{part}[{i}].{j}: integer ticks required')
        out.append(dict(w))
    return tuple(out)


def parse(data):
    gestures = {}
    for key, g in data.items():
        if key.startswith('_'):
            continue
        if not isinstance(g, dict):
            raise GestureError(f'{key}: object required')
        if g.get('arm') not in ARMS:
            raise GestureError(f'{key}: arm must be one of {ARMS}')
        if not isinstance(g.get('emoji'), str) or not g['emoji'] or not isinstance(g.get('label'), str):
            raise GestureError(f'{key}: emoji and label strings required')
        allowed = HEAD_JOINTS if g['arm'] == 'head' else GESTURE_JOINTS
        relative = g.get('relative_joints', [])
        if not isinstance(relative, list) or any(j not in allowed for j in relative) or len(set(relative)) != len(relative):
            raise GestureError(f'{key}: relative_joints must be supported positioning joints')
        gesture = Gesture(key, g['emoji'], g['label'], g['arm'], _waypoints(key, 'raise', g.get('raise'), allowed),
                          _waypoints(key, 'motion', g.get('motion'), allowed), g.get('verified_on_hardware') is True, tuple(relative))
        # The motion starts where the raise ends; every leg must be one continuous owner piece, so the
        # gesture keeps its rhythm instead of being split.
        pose = {}
        for w in gesture.raise_path:
            pose.update(w)
        for i, w in enumerate(gesture.motion):
            for j, q in w.items():
                if j not in pose:
                    raise GestureError(f'{key}.motion[{i}].{j}: joint not set by the raise, so its start is unknown')
                maximum = HEAD_MAX_LEG_TICKS if g['arm'] == 'head' else MAX_LEG_TICKS
                if abs(q - pose[j]) > maximum:
                    raise GestureError(f'{key}.motion[{i}].{j}: leg of {abs(q - pose[j])} ticks exceeds {maximum}')
            pose.update(w)
        gestures[key] = gesture
    if not gestures:
        raise GestureError('No gestures defined')
    return gestures


def load(path=DEFAULT_PATH):
    return parse(json.loads(Path(path).read_text()))


def canonical(arm, waypoint):
    if arm == 'head':
        return {HEAD_NAMES[j]: q for j, q in waypoint.items()}
    return {f'{arm}_arm_{j}': q for j, q in waypoint.items()}


def arm_motors(arm):
    if arm == 'head':
        return list(HEAD_NAMES.values())
    return [f'{arm}_arm_{j}' for j in ARM_JOINTS]


def path_seconds(waypoints, start):
    """The owner's run time at full pace: legs split like expand_path, 40-tick steps every 0.4 s."""
    current, steps = dict(start), 0
    for w in waypoints:
        delta = max([abs(q - current[n]) for n, q in w.items()] or [0])
        pieces = 1 if delta <= MAX_LEG_TICKS else math.ceil(delta / SPLIT_TICKS)
        steps += pieces * max(1, math.ceil(delta / pieces / STEP_TICKS))
        current.update(w)
    return round(max(STEP_S, steps * STEP_S), 1)


def plan(gestures, start, ranges):
    """Paths for a sequence of gestures on one arm: each gesture's raise and motion, then a return to the
    resting pose. start: canonical name -> present ticks. ranges: robot_get_state commandable_ranges.
    Raises GestureError before anything moves if a target is outside the live commandable range."""
    arms = {g.arm for g in gestures}
    if len(arms) != 1:
        raise GestureError('One arm per performance')
    arm = arms.pop()
    for n in arm_motors(arm):
        if type(start.get(n)) is not int:
            raise GestureError(f'No current encoder reading for {n}')
    paths, touched = [], set()
    for g in gestures:
        for part, points in (('raise', g.raise_path), ('motion', g.motion)):
            canon = []
            for waypoint in points:
                values = canonical(arm, waypoint)
                for joint in g.relative_joints:
                    name = HEAD_NAMES[joint] if arm == 'head' else f'{arm}_arm_{joint}'
                    if name in values:
                        values[name] += start[name]
                canon.append(values)
            for w in canon:
                for n, q in w.items():
                    lo, hi = ranges[n]['min_ticks'], ranges[n]['max_ticks']
                    if not lo <= q <= hi:
                        raise GestureError(f'{g.key}.{part}: {n}={q} outside commandable [{lo}, {hi}]')
                    touched.add(n)
            paths.append({'gesture': g.key, 'part': part, 'waypoints': canon})
    home = {n: min(max(start[n], ranges[n]['min_ticks']), ranges[n]['max_ticks']) for n in sorted(touched)}
    paths.append({'gesture': None, 'part': 'return', 'waypoints': [home]})
    current = {n: start[n] for n in touched}
    for p in paths:
        if arm == 'head':
            durations = []
            for w in p['waypoints']:
                delta = max(abs(q - current[n]) for n, q in w.items())
                if delta > HEAD_MAX_LEG_TICKS:
                    raise GestureError(f'Head travel {delta} exceeds {HEAD_MAX_LEG_TICKS} ticks')
                durations.append(max(delta / 100, path_seconds([w], current)))
                current.update(w)
            p['head_durations_s'] = durations
            p['duration_s'] = sum(durations)
            continue
        p['duration_s'] = min(MAX_PATH_S, path_seconds(p['waypoints'], current))
        for w in p['waypoints']:
            current.update(w)
    motors = sorted(touched) if arm == 'head' else arm_motors(arm)
    return {'arm': arm, 'motors': motors, 'paths': paths, 'home': home}
