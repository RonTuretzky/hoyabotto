"""Teach one keyframe by hand: the fallback when the vision model cannot reach a pose.

One arm (or the head) goes limp, a person places it, presses ENTER, and the pose is saved.
The idea is from the XLeRobot kinesthetic recorder. It is human operation, which this project
otherwise rules out, so it is off unless the profile says `teaching: {by_hand: true}` and
every pose taught this way is marked `learned_by: hand:<name>` in the keyframe file and the
evidence store.
"""
from __future__ import annotations

from typing import Any, Callable

from ..adapters.base import ARM_JOINTS, HEAD_JOINTS, arm_joint


def joints_of(arm: str) -> list[str]:
    if arm == "head":
        return list(HEAD_JOINTS)
    if arm not in ("left", "right"):
        raise ValueError("arm must be left, right or head")
    return [arm_joint(arm, j) for j in ARM_JOINTS]


def teach(robot, keyframes, arm: str, name: str, who: str, ask: Callable[[str], str] = input,
          out: Callable[[str], None] = print, store=None, note: str = "") -> dict[str, Any]:
    """Limp -> person places the arm -> ENTER -> hold -> save. Returns {ok, name, joints} or {ok: False, reason}."""
    if not who.strip():
        return {"ok": False, "reason": "a name is required (--who): hand-taught poses are recorded with who taught them"}
    joints = joints_of(arm)
    before = robot.joints()
    if not before.ok:
        return {"ok": False, "reason": f"cannot read joints: {before.note}"}
    out(f"Support the {arm} {'head' if arm == 'head' else 'arm'} now: its motors are about to go limp.")
    ask("Press ENTER when you are holding it... ")
    robot.torque_off(joints)
    try:
        ask(f"Place it in the pose for '{name}', hold it still, and press ENTER... ")
        r = robot.joints()
        if not r.ok:
            return {"ok": False, "reason": f"cannot read joints: {r.note}"}
        pose = {j: float(r.value[j]) for j in joints}
    finally:
        robot.stop()                 # goal := where it is now, so switching torque on does not jump
        robot.torque_on(joints)
    keyframes.save(name, pose, arm=arm, note=note or "placed by hand", learned_by=f"hand:{who.strip()}")
    if store is not None:
        store.event(None, "keyframe_taught", {"name": name, "arm": arm, "learned_by": "hand", "who": who.strip(), "joints": pose})
    out(f"saved '{name}' ({len(pose)} joints), taught by hand by {who.strip()}. The motors are holding it; you can let go.")
    return {"ok": True, "name": name, "joints": pose}
