"""Robot-only self-test: motors and nothing else (no cameras, no LLM, no trays).

Stage 1 (always, no motion beyond holding position): connect, read every joint
and load.

Stage 2 (--move): nudge one joint at a time by a few units and bring it back,
naming the joint first, so a person can watch that the named joint is the one
that moves. This is what catches swapped buses (left/right) and swapped head
motors (pan/tilt).

Positions are LeRobot-normalized: -100..100 per joint, gripper 0..100.
"""
from __future__ import annotations

import time
from typing import Any, Callable

from ..adapters.base import ARM_JOINTS, HEAD_JOINTS, arm_joint

GROUPS: dict[str, list[str]] = {
    "head": list(HEAD_JOINTS),
    "left": [arm_joint("left", j) for j in ARM_JOINTS],
    "right": [arm_joint("right", j) for j in ARM_JOINTS],
}

# What a person should see when the joint is nudged.
EXPECT = {
    "head_motor_1": "head turns left/right (pan)",
    "head_motor_2": "head nods up/down (tilt)",
    "shoulder_pan": "whole arm rotates at its base",
    "shoulder_lift": "upper arm tilts at the shoulder",
    "elbow_flex": "forearm bends at the elbow",
    "wrist_flex": "gripper tilts up/down at the wrist",
    "wrist_roll": "gripper rotates around its own axis",
    "gripper": "gripper jaw opens/closes",
}


def describe(joint: str) -> str:
    if joint in HEAD_JOINTS:
        return f"HEAD: {EXPECT[joint]}"
    arm, _, name = joint.partition("_arm_")
    return f"{arm.upper()} arm: {EXPECT.get(name, name)}"


def _limits(joint: str) -> tuple[float, float]:
    return (0.0, 100.0) if joint.endswith("gripper") else (-100.0, 100.0)


def _wait_near(robot, joint: str, target: float, tol: float, timeout_s: float, sleep: Callable[[float], None]) -> float | None:
    """Poll until the joint is within tol of target and has stopped moving, or timeout; returns the last position read."""
    last = None
    deadline = time.time() + timeout_s
    while True:
        r = robot.joints()
        if r.ok:
            prev, last = last, float(r.value[joint])
            if abs(last - target) <= tol and prev is not None and abs(last - prev) < 0.3:
                return last
        if time.time() >= deadline:
            return last
        sleep(0.05)


def nudge(robot, joint: str, delta: float = 5.0, tol: float = 2.0, timeout_s: float = 2.0,
          sleep: Callable[[float], None] = time.sleep) -> dict[str, Any]:
    """Move one joint by `delta` and back. Returns {joint, start, reached, end, moved, status}."""
    r = robot.joints()
    if not r.ok:
        return {"joint": joint, "status": "read failed", "note": r.note}
    start = float(r.value[joint])
    lo, hi = _limits(joint)
    target = start + delta if start + delta <= hi - 2 else start - delta
    target = max(lo, min(hi, target))
    sent = robot.move_to({joint: target}, max_step=abs(delta))
    if not sent.ok:
        return {"joint": joint, "start": start, "status": "send failed", "note": sent.note}
    reached = _wait_near(robot, joint, target, tol, timeout_s, sleep)
    robot.move_to({joint: start}, max_step=abs(delta))
    end = _wait_near(robot, joint, start, tol, timeout_s, sleep)
    moved = abs((reached if reached is not None else start) - start)
    if reached is None or end is None:
        status = "read failed"
    elif moved < abs(delta) * 0.4:
        status = "did not move"
    elif abs(end - start) > max(tol, 3.0):
        status = "did not return"
    else:
        status = "ok"
    return {"joint": joint, "start": start, "target": target, "reached": reached, "end": end, "moved": moved, "status": status}


def run(robot, move: bool = False, delta: float = 5.0, only: str | None = None,
        ask: Callable[[str], str] | None = None, out: Callable[[str], None] = print,
        sleep: Callable[[float], None] = time.sleep) -> dict[str, Any]:
    """Run the self-test on an already-connected robot adapter. Returns {ok, joints, health, nudges, problems}."""
    problems: list[str] = []
    groups = [only] if only else ["head", "left", "right"]
    joints = [j for g in groups for j in GROUPS[g]]

    pos = robot.joints()
    health = robot.health()
    if not pos.ok:
        problems.append(f"joint read failed: {pos.note}")
        return {"ok": False, "problems": problems, "joints": {}, "health": {}, "nudges": []}
    out(f"{'joint':<26}{'position':>10}{'load':>7}")
    for j in joints:
        h = (health.value or {}).get(j, {}) if health.ok else {}
        out(f"{j:<26}{pos.value[j]:>10.1f}{(str(int(h['load'])) if 'load' in h else '-'):>7}")
    if not health.ok:
        problems.append(f"health read failed: {health.note}")

    nudges: list[dict[str, Any]] = []
    if move:
        out("\nNudging one joint at a time. Keep hands clear. Ctrl-C stops the test.")
        for j in joints:
            out(f"\n{j}  ->  expect: {describe(j)}")
            sleep(1.0)
            res = nudge(robot, j, delta=delta, sleep=sleep)
            if res["status"] == "ok":
                out(f"  moved {res['moved']:.1f} and returned")
            else:
                out(f"  {res['status'].upper()}" + (f" ({res.get('note')})" if res.get("note") else ""))
                problems.append(f"{j}: {res['status']}")
            if ask is not None and res["status"] == "ok":
                a = ask("  Was that the part that moved? [y/n] ").strip().lower()
                res["confirmed"] = a.startswith("y")
                if not res["confirmed"]:
                    problems.append(f"{j}: a different part moved (check bus ports / head motor order)")
            nudges.append(res)

    out("\n" + ("ALL OK" if not problems else "PROBLEMS:\n  " + "\n  ".join(problems)))
    return {"ok": not problems, "problems": problems, "joints": dict(pos.value), "health": health.value if health.ok else {}, "nudges": nudges}
