"""The closing plan: which arm does what, in what order, and what each step must leave behind.

Order follows the video: both short (end) flaps in, then the far long flap, then the near
long flap, then tape across the seam, then press. Left arm works bare-fingered; the right
arm holds the printed paddle for the whole job (far reach, flat pressing face).

Every pose named here is a keyframe the vision model teaches (`carton teach-all`); nobody
positions the arms by hand. A step is a sequence of keyframes; the fold itself is the motion
from the "touch" keyframe to the "done" keyframe, run as clamped joint interpolation.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Keyframe:
    name: str
    arm: str                 # left | right | head
    goal: str                # what the vision model must achieve before saving


@dataclass(frozen=True)
class Step:
    name: str
    arm: str
    keyframes: tuple[str, ...]        # played in order, each a clamped move
    check: str                        # field of the box judgement that must be true afterwards
    max_s: float = 8.0


KEYFRAMES: list[Keyframe] = [
    Keyframe("look_box", "head", "Point the head camera so the whole open carton, all four flaps included, fills the head view."),
    Keyframe("paddle_rest_above", "right", "Position the open gripper 6 cm above the flap paddle lying in its rest, fingers open, ready to descend."),
    Keyframe("paddle_rest_grip", "right", "Lower the open gripper around the flap paddle's handle so closing the fingers would hold it firmly, blade pointing forward."),
    Keyframe("paddle_carry", "right", "Lift the paddle clear of the rest and the box, blade forward and level, at shoulder height."),
    Keyframe("short_left_touch", "left", "Closed fingers touching the OUTSIDE face of the left end flap at half its height, centred along the flap, ready to push it inward."),
    Keyframe("short_left_done", "left", "Closed fingers resting on the left end flap now folded flat over the box contents; the flap lies horizontal."),
    Keyframe("short_right_touch", "right", "Paddle blade flat against the OUTSIDE face of the right end flap at half its height, ready to push it inward."),
    Keyframe("short_right_done", "right", "Paddle blade resting on the right end flap now folded flat over the contents."),
    Keyframe("far_touch", "right", "Paddle blade against the OUTSIDE face of the far long flap at half its height, in line with the right shoulder, ready to pull it toward the robot."),
    Keyframe("far_done", "right", "Paddle blade resting on the far long flap now folded flat, its free edge at the centre of the box."),
    Keyframe("near_touch", "left", "Closed fingers against the OUTSIDE face of the near long flap at half its height, ready to push it away over the box."),
    Keyframe("near_done", "left", "Closed fingers resting on the near long flap now folded flat, its free edge meeting the far flap at the centre seam."),
    Keyframe("tape_rest_above", "left", "Open gripper 5 cm above the tape strip's free tab in the tape rest."),
    Keyframe("tape_rest_grip", "left", "Open gripper around the tape strip's tab so closing would pinch the tab without touching the sticky side."),
    Keyframe("tape_over_seam", "left", "Holding the strip by its tab, hold it level 2 cm above the centre seam, strip crossing the seam at a right angle."),
    Keyframe("tape_down", "left", "Lower the strip onto the box so its middle sits on the seam; fingers still pinching the tab."),
    Keyframe("press_a", "right", "Paddle blade flat on the near end of the tape strip, pressing lightly."),
    Keyframe("press_b", "right", "Paddle blade flat on the far end of the tape strip, pressing lightly."),
    Keyframe("rest_right", "right", "Right arm folded back at rest beside the box, paddle still held, clear of the work area."),
    Keyframe("rest_left", "left", "Left arm folded back at rest beside the box, clear of the work area."),
]

STEPS: list[Step] = [
    Step("pick_paddle", "right", ("paddle_rest_above", "paddle_rest_grip", "paddle_carry"), "paddle_held"),
    Step("fold_short_left", "left", ("short_left_touch", "short_left_done", "rest_left"), "short_left_folded"),
    Step("fold_short_right", "right", ("short_right_touch", "short_right_done", "paddle_carry"), "short_right_folded"),
    Step("fold_long_far", "right", ("far_touch", "far_done", "paddle_carry"), "long_far_folded", max_s=10.0),
    Step("fold_long_near", "left", ("near_touch", "near_done", "rest_left"), "long_near_folded"),
    Step("tape", "left", ("tape_rest_above", "tape_rest_grip", "tape_over_seam", "tape_down"), "tape_on_seam"),
    Step("press", "right", ("press_a", "press_b", "rest_right"), "tape_pressed"),
]

GRIP_STEPS = {"pick_paddle": "paddle_rest_grip", "tape": "tape_rest_grip"}   # gripper closes after this keyframe
RELEASE_AFTER = {"tape": "tape_down"}                                        # gripper opens after this keyframe


def keyframe(name: str) -> Keyframe:
    return next(k for k in KEYFRAMES if k.name == name)


def needed(names_taught: set[str]) -> list[Keyframe]:
    return [k for k in KEYFRAMES if k.name not in names_taught]
