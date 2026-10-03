"""Box geometry and a reach check. Everything in metres, box frame:
origin at the centre of the top rim, x along the box length (the robot's left/right),
y toward the far side (away from the robot), z up. The robot stands along one LONG side.

Measured 2026-10-03 (user): outer 37.9 x 28.3 x 10.8 cm, flaps 14 cm, contents 24 x 40 g.
The long flaps (hinged on the 37.9 cm sides) are 14 cm each and meet at the centre seam
(2 x 14 = 28 vs 28.3 wide); the short flaps (hinged on the 28.3 cm ends) fold inward and
do not meet. That is the order the video shows: short flaps first, then the long flaps.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

L1_L2 = 0.1159 + 0.1350            # SO-101 upper + lower arm (farm/vendor/so101_kinematics.py)


@dataclass
class Box:
    length: float = 0.379
    width: float = 0.283
    height: float = 0.108
    flap: float = 0.14
    mass_kg: float = 0.96              # contents; the robot never lifts it

    @property
    def seam_gap(self) -> float:
        """Gap (or overlap, negative) between the two long flaps when closed."""
        return self.width - 2 * self.flap

    @property
    def short_flap_gap(self) -> float:
        return self.length - 2 * self.flap


@dataclass
class Stance:
    """Where the two shoulders are relative to the box: cart parked along the near long side."""
    setback: float = 0.02              # shoulder line behind the near rim (m); small = leaning over the box
    spacing: float = 0.30              # distance between the two shoulders (m); measure on the cart
    height: float = 0.18               # shoulders above the box's top rim (m); depends on conveyor vs cart height
    tool: float = 0.10                 # wrist point to fingertip, gripper closed (m)
    paddle: float = 0.15               # extra reach when the printed flap paddle is held in the gripper (m)
    margin: float = 0.03               # keep this much reach in hand

    def reach(self, with_paddle: bool = False) -> float:
        return L1_L2 + self.tool + (self.paddle if with_paddle else 0.0) - self.margin

    def shoulder(self, arm: str, box: Box) -> tuple[float, float, float]:
        x = -self.spacing / 2 if arm == "left" else self.spacing / 2
        return (x, -box.width / 2 - self.setback, self.height)


@dataclass
class Target:
    name: str
    arm: str
    point: tuple[float, float, float]          # where the tool tip must be at the hardest moment
    paddle: bool = False                       # held paddle (right arm) instead of bare fingers
    note: str = ""


def targets(box: Box, stance: "Stance | None" = None) -> list[Target]:
    """The hardest point of each contact in the plan. Flaps are pushed at mid height (half the flap),
    where the fold starts with little force; the first touch is the farthest point, the push then
    moves toward the robot or toward the centre. Long flaps are pushed in line with the working
    shoulder (not at the box centre) so the arm does not also reach sideways.

    Left arm, bare fingers: left short flap, near long flap, pick and lay the tape strip.
    Right arm, holding the paddle: right short flap, far long flap, press the tape."""
    L, W, F = box.length, box.width, box.flap
    h = F / 2
    xr = (stance.spacing / 2) if stance else 0.0
    xl = -xr
    return [
        Target("short_flap_left", "left", (-L / 2, 0.0, h), False, "push the left end flap inward at mid height"),
        Target("short_flap_right", "right", (L / 2, 0.0, h), True, "push the right end flap inward with the paddle"),
        Target("long_flap_far", "right", (xr, W / 2, h), True, "first touch on the far flap, in line with the shoulder; the fold comes toward the robot"),
        Target("long_flap_far_end", "right", (xr, 0.0, 0.0), True, "where the far flap ends: at the seam"),
        Target("long_flap_near", "left", (xl, -W / 2, h), False, "push the near flap over; it ends at the seam"),
        Target("tape_centre", "left", (xl / 2, 0.0, 0.0), False, "strip laid across the seam, a little toward the left shoulder"),
        Target("press_end_a", "right", (xl / 2, -0.05, 0.0), True, "press along the strip with the paddle"),
        Target("press_end_b", "right", (xl / 2, 0.05, 0.0), True, "press along the strip with the paddle"),
    ]


def reach_report(box: Box, stance: Stance) -> dict[str, Any]:
    """Distance from the chosen arm's shoulder to each target, against the usable reach."""
    rows = []
    ok = True
    for t in targets(box, stance):
        sx, sy, sz = stance.shoulder(t.arm, box)
        d = math.dist((sx, sy, sz), t.point)
        reach = stance.reach(t.paddle)
        within = d <= reach
        ok &= within
        rows.append({"target": t.name, "arm": t.arm, "tool": "paddle" if t.paddle else "fingers", "distance_m": round(d, 3),
                     "within_reach": within, "margin_m": round(reach - d, 3), "note": t.note})
    # how far back the shoulders may stand and still touch the far flap, with and without the paddle
    far = next(t for t in targets(box, stance) if t.name == "long_flap_far")
    dz = stance.height - far.point[2]

    def max_setback(with_paddle: bool) -> float:
        horiz = math.sqrt(max(0.0, stance.reach(with_paddle) ** 2 - dz ** 2))
        return horiz - box.width
    return {"reach_fingers_m": round(stance.reach(False), 3), "reach_paddle_m": round(stance.reach(True), 3), "all_within_reach": ok, "rows": rows,
            "max_setback_far_flap_fingers_m": round(max_setback(False), 3), "max_setback_far_flap_paddle_m": round(max_setback(True), 3),
            "seam_gap_m": round(box.seam_gap, 3), "short_flap_gap_m": round(box.short_flap_gap, 3)}


def format_report(r: dict[str, Any]) -> str:
    lines = [f"usable reach: fingers {r['reach_fingers_m'] * 100:.0f} cm, with the paddle {r['reach_paddle_m'] * 100:.0f} cm (arm {L1_L2 * 100:.0f} cm + tool, minus margin)",
             f"{'target':<20}{'arm':<7}{'tool':<9}{'dist cm':>8}{'margin':>8}  ok"]
    for row in r["rows"]:
        lines.append(f"{row['target']:<20}{row['arm']:<7}{row['tool']:<9}{row['distance_m'] * 100:>8.1f}{row['margin_m'] * 100:>8.1f}  {'yes' if row['within_reach'] else 'NO'}")
    lines.append(f"long flaps meet with a gap of {r['seam_gap_m'] * 100:.1f} cm; short flaps leave {r['short_flap_gap_m'] * 100:.1f} cm open between them")
    lines.append(f"far flap: shoulders may stand at most {r['max_setback_far_flap_fingers_m'] * 100:.1f} cm behind the near rim with fingers, {r['max_setback_far_flap_paddle_m'] * 100:.1f} cm with the paddle")
    lines.append("ALL WITHIN REACH" if r["all_within_reach"] else "NOT ALL WITHIN REACH: park closer or lower, or lengthen the paddle")
    return "\n".join(lines)
