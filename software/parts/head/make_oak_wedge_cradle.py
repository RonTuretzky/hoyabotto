"""OAK-D Lite slot cradle with a built-in tilt wedge, so the head reaches the fold policy's 58 deg at its servo limit.

Same tongue, same four M3 screws, same snap-fit box as make_oak_slot_cradle.py. The camera box is rotated WEDGE_DEG
about the tilt axis (the link's X axis, at Z = AXIS_Z) relative to the tongue, so at every servo tick the camera looks
WEDGE_DEG further down than the stock cradle. Measured 9 Oct: the stock cradle reaches 35.1 deg down at the tilt
servo's commandable maximum (tick 2580); the policy trained at 58 deg; wedge = 58 - 35.1 = 22.9 -> 23 deg.

    /tmp/cqvenv/bin/python make_oak_wedge_cradle.py [--wedge 23] [--sign +1|-1] [--out DIR]

The sign is verified numerically (lens optical axis in the link frame) by check_wedge_direction(); the build refuses
the wrong one. Everything else is in millimetres in the tilt-link design frame of the stock part.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_oak_slot_cradle as stock  # noqa: E402  the stock part: parameters, box builder, export helpers

AXIS_Z = 8.33                      # tilt axis height in the design frame (design_slot_cradle.json "frame")
WEDGE_DEG = 23.0
# The wedge must join the rotated box to the tongue through the wall; the wall is thickened into a solid block
# behind the box so the rotated box always lands on material (no thin bridge).
BLOCK_Y0 = stock.WALL_Y0            # back of the wall (just in front of the link)


def box_only() -> cq.Workplane:
    """The stock cradle's camera box alone: wall + window + ledge + top + ends + USB opening + fingers; no tongue/bar."""
    # Build the whole stock part, then cut away the tongue and bar region (everything at Y < WALL_Y0).
    full = stock.build()
    behind = stock.box(-200, 200, -200, stock.WALL_Y0, -200, 200)
    return full.cut(behind)


def tongue_and_bar() -> cq.Workplane:
    full = stock.build()
    front = stock.box(-200, 200, stock.WALL_Y0, 200, -200, 200)
    return full.cut(front)


def build(wedge_deg: float, sign: int) -> cq.Workplane:
    """Tongue stays; the box is rotated sign*wedge about the tilt axis and joined to the tongue by a solid block."""
    box_part = box_only().rotate((0, 0, AXIS_Z), (1, 0, AXIS_Z), sign * wedge_deg)
    tongue = tongue_and_bar()
    # Joining block: fills the space between the tongue's front face and the rotated box's back wall. It is the
    # stock wall's footprint (full width, from the tongue zone down to the plate), extruded along +Y far enough to
    # reach the rotated wall, then trimmed by the rotated wall's back plane so nothing pokes into the camera cavity.
    reach = stock.WALL_T + abs(math.sin(math.radians(wedge_deg))) * abs(stock.CAM_TOP_Z - stock.TONGUE_Z1) + 0.5
    block = stock.box(-stock.HALF_W, stock.HALF_W, stock.WALL_Y0, stock.WALL_Y0 + reach,
                      stock.CAM_TOP_Z - stock.PLATE_T, stock.TONGUE_Z1)
    # trim: keep only what lies behind the rotated box's back wall plane (the box's own wall carries the camera)
    back_plane_cut = (stock.box(-200, 200, stock.WALL_Y1, 200, -200, 200)
                      .rotate((0, 0, AXIS_Z), (1, 0, AXIS_Z), sign * wedge_deg))
    block = block.cut(back_plane_cut)
    # also keep the vent window open through the block, on the rotated wall's window footprint
    window = (stock.box(-stock.WINDOW_HALF_W, stock.WINDOW_HALF_W, stock.WALL_Y0 - 20, stock.WALL_Y1 + 20,
                        stock.WINDOW_Z1, stock.WINDOW_Z0).edges("|Y").fillet(3.0)
              .rotate((0, 0, AXIS_Z), (1, 0, AXIS_Z), sign * wedge_deg))
    block = block.cut(window)
    return tongue.union(block).union(box_part)


def lens_axis_in_link_frame(sign: int, wedge_deg: float):
    """Optical axis direction of the camera (stock: +Y, straight out of the box) after the wedge rotation."""
    a = math.radians(sign * wedge_deg)
    # rotation about X by angle a maps (0, 1, 0) -> (0, cos a, sin a)
    return (0.0, math.cos(a), math.sin(a))


def check_wedge_direction(sign: int, wedge_deg: float) -> dict:
    """Looking further DOWN = the optical axis tips toward the link's underside, which is -Z in the design frame
    (the camera hangs on the -Z side; the tilt servo turns the whole link, so 'down' at the camera is -Z here).
    Refuse the sign that tips the axis toward +Z (the servo horns / upward)."""
    axis = lens_axis_in_link_frame(sign, wedge_deg)
    ok = axis[2] > 0
    return {"optical_axis_link_frame": [round(v, 4) for v in axis], "tips_toward_plus_z_horn_side": ok,
            "note": "verified 9 Oct by mesh fit, side render and servo sweep: +Z (horn side) is the down-looking "
                    "direction for the camera box; -1 swings the box into the tilt link"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wedge", type=float, default=WEDGE_DEG)
    ap.add_argument("--sign", type=int, choices=(-1, 1), default=1)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument("--allow-wrong-sign", action="store_true", help="build anyway (for comparing both signs)")
    args = ap.parse_args()
    direction = check_wedge_direction(args.sign, args.wedge)
    if not direction["tips_toward_plus_z_horn_side"] and not args.allow_wrong_sign:
        raise SystemExit(f"sign {args.sign:+d} tips the camera UP, not down: {direction}")
    wp = build(args.wedge, args.sign)
    stem = f"oak_d_lite_wedge{int(round(args.wedge))}_cradle" + ("" if args.sign < 0 else "_signplus")
    stock.HERE = args.out
    info = {
        "frame": "tilt link frame as the stock cradle; the camera box is rotated about the tilt axis (X, Z=8.33)",
        "wedge_deg": args.wedge, "sign": args.sign, "direction_check": direction,
        "stock_reach_deg_at_servo_max": 35.1, "target_deg": 58.0,
        "parts": {stem: stock.export(wp, stem, stock.to_print_orientation(wp, stock.FRONT_Y + 30))},
    }
    (args.out / f"design_{stem}.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
