"""A short, hold-to-run dance: head, wrists and grippers only, small moves around the starting pose.

The dance never moves shoulders or elbows (so the arms stay where they are and cannot swing into the
cart or table) and never the wheels. Every tick goes through the same checks as the skills: STOP, the
health/load ceiling, the joint watchdog and the per-tick step clamp, at a slower step than the skills.
It runs only while `keep_going()` is true; when that turns false (button released, controller lost) it
stops and eases back to the pose it started from.
"""
from __future__ import annotations

import time
from typing import Callable

from ..adapters.base import HEAD_JOINTS, arm_joint
from ..safety.rules import SafetyStop, clamp_targets
from ..status import Status
from .runner import TICK_S, SkillResult, SkillRunner

HEAD_PAN, HEAD_TILT = HEAD_JOINTS
DANCE_STEP = 2.0          # normalized units per tick; slower than the skills' step limit
BEAT_SETTLE = 2.5         # a beat is reached when every joint is within this
BEAT_MAX_S = 1.5          # move on even if a beat is not quite reached

# Offsets from the starting pose, in the adapter's normalized units (-100..100, grippers 0..100).
# Kept small: about +-8 degrees of head and +-25 degrees of wrist roll on this kit.
BEATS: list[dict[str, float]] = [
    {HEAD_PAN: 25, arm_joint("left", "wrist_roll"): 20, arm_joint("right", "wrist_roll"): 20},
    {HEAD_PAN: -25, arm_joint("left", "wrist_roll"): -20, arm_joint("right", "wrist_roll"): -20},
    {HEAD_TILT: 15, arm_joint("left", "gripper"): 20, arm_joint("right", "gripper"): 20},
    {HEAD_TILT: -10, arm_joint("left", "wrist_flex"): 8, arm_joint("right", "wrist_flex"): -8},
    {HEAD_PAN: 15, arm_joint("left", "wrist_flex"): -8, arm_joint("right", "wrist_flex"): 8},
    {},   # back to the start: one loop of the dance ends where it began
]
DANCE_JOINTS = sorted({j for beat in BEATS for j in beat})


def beat_targets(start: dict[str, float], beat: dict[str, float]) -> dict[str, float]:
    """Absolute targets for one beat: start pose plus offsets, for the dance joints only."""
    return {j: start[j] + beat.get(j, 0.0) for j in DANCE_JOINTS if j in start}


def _step_toward(runner: SkillRunner, goal: dict[str, float], keep_going: Callable[[], bool] | None, tick: int) -> tuple[bool, int]:
    """Move toward goal one clamped tick at a time. Returns (reached, tick). Stops early if keep_going() turns false."""
    t0 = time.time()
    cur = runner._read_joints()
    step_max = min(DANCE_STEP, runner.limits.step_deg_max)
    while True:
        if keep_going is not None and not keep_going():
            return False, tick
        tick += 1
        runner._guard(tick)
        step = clamp_targets(cur, goal, step_max)
        r = runner.robot.move_to(step, max_step=step_max)
        if r.status is not Status.OK:
            runner.stop()
            raise SafetyStop(f"move refused: {r.note}")
        time.sleep(TICK_S)
        cur = runner._read_joints()
        if runner.on_tick:
            runner.on_tick(cur)
        # grippers stall on whatever they meet; like the skills, they never gate settling
        err = max((abs(cur[j] - goal[j]) for j in goal if not j.endswith("gripper")), default=0.0)
        if err <= BEAT_SETTLE or time.time() - t0 > BEAT_MAX_S:
            return True, tick


def dance(runner: SkillRunner, keep_going: Callable[[], bool], max_loops: int | None = None) -> SkillResult:
    """Dance while keep_going() is true, then return to the starting pose. Refuses if an arm holds a tool."""
    for arm in ("left", "right"):
        if runner.held[arm] is not None:
            raise SafetyStop(f"{arm} arm holds {runner.held[arm]}; no dancing with a tool in hand")
    start = runner._read_joints()
    home = beat_targets(start, {})
    loops = tick = 0
    # A SafetyStop (STOP, stale reading, load) propagates from here with the robot already stopped:
    # after a stop the robot holds still and nothing moves it back.
    while keep_going() and (max_loops is None or loops < max_loops):
        for beat in BEATS:
            reached, tick = _step_toward(runner, beat_targets(start, beat), keep_going, tick)
            if not reached:
                break
        else:
            loops += 1
    # Released normally: ease back to where we started, still under every check.
    _step_toward(runner, home, None, tick)
    return SkillResult(True, "dance", f"{loops} full loop(s)", joints=runner._read_joints(), data={"loops": loops})
