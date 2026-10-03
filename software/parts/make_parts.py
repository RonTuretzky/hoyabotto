#!/usr/bin/env python
"""
make_parts.py -- 3D-printable fixtures for the XLeRobot micro-farm.

Pure numpy, no trimesh / numpy-stl.  Every part is a union of simple closed
shells (axis-aligned boxes, square-to-circle annuli, tapered rings).  Shells
overlap slightly on purpose; slicers union overlapping closed shells.

Run:   python parts/make_parts.py        (from software/, venv active)
Out:   parts/out/*.stl  + a bounding-box / triangle-count table on stdout.

Printer: AnkerMake M5C, bed 220 x 220 x 250 mm, 0.4 mm nozzle, PLA.
All parts are modelled in their print orientation (Z up), flat bottom,
no overhangs (all features are steps whose upper faces sit on material).
"""
from __future__ import annotations

import math
import os
import struct
import sys

import numpy as np

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
BED_XY = 220.0          # mm, usable bed
EPS = 0.02              # mm, shell overlap so adjacent cells fuse in the slicer
SEGS = 48               # facets per full circle (multiple of 8 -> exact square corners)

# --------------------------------------------------------------------------
# Parameters (mm).  Everything a user might need to tune lives up here.
# --------------------------------------------------------------------------
# 1) Cress trough nest ("Macce" wicking planter, trough ~180 x 85 x 25)
TROUGH_L, TROUGH_W, TROUGH_H = 180.0, 85.0, 25.0
POCKET_CLEAR = 0.75                     # per side
POCKET_L = TROUGH_L + 2 * POCKET_CLEAR  # 181.5
POCKET_W = TROUGH_W + 2 * POCKET_CLEAR  # 86.5
NEST_L, NEST_W = 200.0, 105.0           # base plate under the rim
BASE_T = 4.0                            # base plate thickness
RIM_H = 6.0                             # rim height above the base
RIM_TOP = BASE_T + RIM_H                # 10.0
LEDGE_STEPS = 3                         # stepped 45-degree lead-in (1 mm rise / 1 mm inset)
LEDGE_STEP = 1.0
APRON_D = 48.0                          # front apron (-Y) that carries the AprilTag recess
TAB_L, TAB_D = 16.0, 12.0               # rear corner tabs (+Y)
M4_HOLE_D = 4.5
TAG_SIZE = 40.0                         # tag tile edge
TAG_RECESS_CLEAR = 0.25                 # per side -> recess 40.5 x 40.5
TAG_RECESS_DEPTH = 1.2
NOTCH_W = 30.0                          # refill / finger notch on the -X short side

# 2) AprilTag tiles
TAG_TILE_T = 1.0
TAG_CELL = 5.0                          # 8 x 8 cells -> 40 mm
TAG_RAISE = 0.6                         # black cells stand proud by this much

# 3) BH1750 light paddle
HANDLE_L, HANDLE_W, HANDLE_T = 120.0, 18.0, 8.0
GRIP_GROOVE_POS = (25.0, 45.0)          # from the gripper end
GRIP_GROOVE_W, GRIP_GROOVE_D = 2.0, 1.0
PLAT_L, PLAT_W = 30.0, 24.0
BH_PCB_L, BH_PCB_W = 19.5, 15.0         # BH1750 breakout (GY-302 style) -- VERIFY
BH_CLEAR = 0.5                          # per side
BH_POCKET_D = 3.0
BH_HOLE_D = 2.2                         # M2 clearance
BH_HOLE_SPACING = 14.5                  # ASSUMED, along the PCB long axis -- VERIFY
CABLE_W, CABLE_D = 4.0, 2.5

# 4) Bottle rest
BOTTLE_D = 54.0                         # nominal bottle diameter (52-55 typical)
BOTTLE_CLEAR = 1.0                      # per side -> ID 56
REST_BASE, REST_BASE_T = 70.0, 4.0
RING_H, RING_WALL = 8.0, 4.0
TAPER_ANGLE_DEG, TAPER_H = 40.0, 2.5    # lead-in: 40 deg from vertical over top 2.5 mm

# --------------------------------------------------------------------------
# AprilTag 36h11 bit patterns.
# Source of truth: the official PNGs tag36_11_00001.png / tag36_11_00002.png
# from https://github.com/AprilRobotics/apriltag-imgs (10x10 px, 1 px white
# border stripped), decoded with a pure-python PNG reader and cross-checked
# against the code words in AprilRobotics/apriltag tag36h11.c
# (codedata[1] = 0x0000000dda664ca7, codedata[2] = 0x0000000dc4a1c821) laid
# out with that file's bit_x/bit_y tables.  Both derivations agree exactly.
# '#' = black cell (raised), '.' = white cell.  Row 0 is the TOP of the tag
# image (rendered here at +Y so the printed tile matches the reference image
# when viewed from above).
# --------------------------------------------------------------------------
TAG36H11 = {
    1: [
        "########",
        "#..#..##",
        "##.#...#",
        "#....###",
        "##..####",
        "#.#..#.#",
        "###.##.#",
        "########",
    ],
    2: [
        "########",
        "#..#...#",
        "##.##.##",
        "#.######",
        "###.##.#",
        "####.###",
        "###...##",
        "########",
    ],
}


# --------------------------------------------------------------------------
# Mesh primitives.  Every function returns a list of triangles, each a
# (3, 3) array of vertices, ordered counter-clockwise seen from outside.
# --------------------------------------------------------------------------
def _quad(a, b, c, d):
    a, b, c, d = (np.asarray(p, dtype=float) for p in (a, b, c, d))
    return [np.array([a, b, c]), np.array([a, c, d])]


def box(x0, y0, z0, x1, y1, z1):
    """Axis-aligned closed box."""
    t = []
    t += _quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))   # +Z
    t += _quad((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))   # -Z
    loop = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]                      # CCW from +Z
    for i in range(4):
        p, q = loop[i], loop[(i + 1) % 4]
        t += _quad((p[0], p[1], z0), (q[0], q[1], z0), (q[0], q[1], z1), (p[0], p[1], z1))
    return t


def _circle(cx, cy, r, n=SEGS):
    a = np.arange(n) * (2 * math.pi / n)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _square_on_rays(cx, cy, hs, n=SEGS):
    """Points on the boundary of a square (half-size hs) along the same rays
    as _circle, so an annulus between them has flat outer walls."""
    a = np.arange(n) * (2 * math.pi / n)
    c, s = np.cos(a), np.sin(a)
    rad = hs / np.maximum(np.abs(c), np.abs(s))
    return np.stack([cx + rad * c, cy + rad * s], axis=1)


def _ring_between(inner_xy, outer_xy, z0, z1):
    """Closed solid between an inner loop and an outer loop (both CCW, same
    vertex count, same z0..z1).  Used for square-with-round-hole cells."""
    n = len(inner_xy)
    t = []
    for i in range(n):
        j = (i + 1) % n
        ci, cj, si, sj = inner_xy[i], inner_xy[j], outer_xy[i], outer_xy[j]
        t += _quad((*ci, z1), (*si, z1), (*sj, z1), (*cj, z1))            # top
        t += _quad((*ci, z0), (*cj, z0), (*sj, z0), (*si, z0))            # bottom
        t += _quad((*si, z0), (*sj, z0), (*sj, z1), (*si, z1))            # outer wall
        t += _quad((*cj, z0), (*ci, z0), (*ci, z1), (*cj, z1))            # inner wall
    return t


def square_with_hole(cx, cy, hs, r, z0, z1):
    return _ring_between(_circle(cx, cy, r), _square_on_rays(cx, cy, hs), z0, z1)


def tapered_ring(cx, cy, r_out, inner_profile, z0, z1):
    """Ring with a vertical outer wall (radius r_out) and an inner wall that
    follows inner_profile = [(z, r_inner), ...] from z0 to z1 (monotonic z).
    Radii growing with z give a lead-in taper that never overhangs."""
    assert abs(inner_profile[0][0] - z0) < 1e-9 and abs(inner_profile[-1][0] - z1) < 1e-9
    n = SEGS
    outer = _circle(cx, cy, r_out, n)
    levels = [(_circle(cx, cy, r, n), z) for z, r in inner_profile]
    t = []
    for i in range(n):
        j = (i + 1) % n
        so, sj = outer[i], outer[j]
        c0, cT = levels[0][0], levels[-1][0]
        t += _quad((*cT[i], z1), (*so, z1), (*sj, z1), (*cT[j], z1))       # top annulus
        t += _quad((*c0[i], z0), (*c0[j], z0), (*sj, z0), (*so, z0))       # bottom annulus
        t += _quad((*so, z0), (*sj, z0), (*sj, z1), (*so, z1))             # outer wall
        for k in range(len(levels) - 1):
            ca, za = levels[k]
            cb, zb = levels[k + 1]
            t += _quad((*ca[j], za), (*ca[i], za), (*cb[i], zb), (*cb[j], zb))  # inner wall
    return t


# --------------------------------------------------------------------------
# Height-map plate: a rectangle subdivided by feature edges into cells, each
# cell being a box with its own bottom/top (or void), or a square-with-hole.
# Later features override earlier ones.  All faces are steps -> no overhangs.
# --------------------------------------------------------------------------
class Plate:
    def __init__(self, x0, y0, x1, y1, z0, z1):
        self.bounds = (x0, y0, x1, y1)
        self.z0, self.z1 = z0, z1
        self.feats = []   # (x0,y0,x1,y1, top, bottom, void)
        self.holes = []   # (cx, cy, r, hs)

    def rect(self, x0, y0, x1, y1, top=None, bottom=None, void=False):
        self.feats.append((x0, y0, x1, y1, top, bottom, void))
        return self

    def rect_inset(self, inset, top=None, bottom=None, ref=None):
        bx0, by0, bx1, by1 = ref or self.bounds
        return self.rect(bx0 + inset, by0 + inset, bx1 - inset, by1 - inset, top=top, bottom=bottom)

    def hole(self, cx, cy, d, margin=0.5):
        self.holes.append((cx, cy, d / 2.0, d / 2.0 + margin))
        return self

    def build(self):
        bx0, by0, bx1, by1 = self.bounds
        xs = {bx0, bx1}
        ys = {by0, by1}
        for f in self.feats:
            xs.update((f[0], f[2]))
            ys.update((f[1], f[3]))
        for cx, cy, r, hs in self.holes:
            xs.update((cx - hs, cx + hs))
            ys.update((cy - hs, cy + hs))
        xs = sorted(round(v, 6) for v in xs if bx0 - 1e-9 <= v <= bx1 + 1e-9)
        ys = sorted(round(v, 6) for v in ys if by0 - 1e-9 <= v <= by1 + 1e-9)
        tris = []
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
                if x1 - x0 < 1e-6 or y1 - y0 < 1e-6:
                    continue
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                z0, z1, void = self.z0, self.z1, False
                for fx0, fy0, fx1, fy1, top, bottom, fvoid in self.feats:
                    if fx0 < cx < fx1 and fy0 < cy < fy1:
                        if fvoid:
                            void = True
                        else:
                            void = False
                            if top is not None:
                                z1 = top
                            if bottom is not None:
                                z0 = bottom
                if void or z1 - z0 < 1e-6:
                    continue
                hole = None
                for hx, hy, r, hs in self.holes:
                    if abs(hx - cx) < 1e-6 and abs(hy - cy) < 1e-6:
                        hole = (hx, hy, r, hs)
                if hole:
                    hx, hy, r, hs = hole
                    tris += square_with_hole(hx, hy, hs + EPS, r, z0, z1)
                else:
                    tris += box(max(x0 - EPS, bx0), max(y0 - EPS, by0), z0,
                                min(x1 + EPS, bx1), min(y1 + EPS, by1), z1)
        return tris


# --------------------------------------------------------------------------
# Binary STL I/O + checks
# --------------------------------------------------------------------------
def write_stl(path, tris, name):
    v = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 1e-12, n / np.maximum(ln, 1e-12), 0.0)
    rec = np.zeros(len(v), dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = v
    header = ("binary STL " + name).encode()[:80].ljust(80, b"\0")
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(v)))
        f.write(rec.tobytes())
    return v


def check_stl(path):
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        header = f.read(80)
        (count,) = struct.unpack("<I", f.read(4))
    assert not header.startswith(b"solid"), "binary STL header must not start with 'solid'"
    assert size == 84 + 50 * count, f"{path}: size {size} != 84 + 50*{count}"
    return count


def signed_volume(v):
    return float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6.0)


def bbox(v):
    p = v.reshape(-1, 3)
    return p.min(axis=0), p.max(axis=0)


# --------------------------------------------------------------------------
# Parts
# --------------------------------------------------------------------------
def part_nest_cress():
    """Locating nest for the cress trough."""
    y_front, y_back = -APRON_D, NEST_W + TAB_D
    p = Plate(0.0, y_front, NEST_L, y_back, 0.0, BASE_T)
    # rear: only two corner tabs, void between them
    p.rect(TAB_L, NEST_W, NEST_L - TAB_L, y_back, void=True)
    # rim over the whole 200 x 105 base, then stepped 45-degree lead-in, then pocket
    rim = (0.0, 0.0, NEST_L, NEST_W)
    p.rect(*rim, top=RIM_TOP)
    rim_w = (NEST_L - POCKET_L) / 2.0                    # 9.25
    assert abs(rim_w - (NEST_W - POCKET_W) / 2.0) < 1e-9, "rim must be equal width on all sides"
    for k in range(LEDGE_STEPS):                          # k=0 outermost ledge
        inset = rim_w - (LEDGE_STEPS - k) * LEDGE_STEP
        p.rect_inset(inset, top=RIM_TOP - (k + 1) * LEDGE_STEP, ref=rim)
    p.rect_inset(rim_w, top=BASE_T, ref=rim)              # pocket floor = base top
    # refill / finger notch through the rim on the -X short side
    p.rect(0.0, NEST_W / 2 - NOTCH_W / 2, rim_w + EPS, NEST_W / 2 + NOTCH_W / 2, top=BASE_T)
    # AprilTag recess, centred on the front apron
    rs = TAG_SIZE + 2 * TAG_RECESS_CLEAR
    tag_cy = -APRON_D / 2.0
    p.rect(NEST_L / 2 - rs / 2, tag_cy - rs / 2, NEST_L / 2 + rs / 2, tag_cy + rs / 2,
           top=BASE_T - TAG_RECESS_DEPTH)
    # four M4 holes: two in rear tabs, two in the front apron corners
    hx = (TAB_L / 2.0, NEST_L - TAB_L / 2.0)
    for x in hx:
        p.hole(x, NEST_W + TAB_D / 2.0, M4_HOLE_D)
        p.hole(x, y_front + 8.0, M4_HOLE_D)
    return p.build()


def part_tag(tag_id):
    grid = TAG36H11[tag_id]
    assert len(grid) == 8 and all(len(r) == 8 for r in grid)
    t = box(0, 0, 0, TAG_SIZE, TAG_SIZE, TAG_TILE_T)
    for row, line in enumerate(grid):
        y1 = TAG_SIZE - row * TAG_CELL         # row 0 at the top (+Y)
        y0 = y1 - TAG_CELL
        for col, ch in enumerate(line):
            if ch != "#":
                continue
            x0 = col * TAG_CELL
            t += box(max(x0 - EPS, 0), max(y0 - EPS, 0), TAG_TILE_T - 0.1,
                     min(x0 + TAG_CELL + EPS, TAG_SIZE), min(y1 + EPS, TAG_SIZE),
                     TAG_TILE_T + TAG_RAISE)
    return t


def part_paddle():
    total_l = HANDLE_L + PLAT_L
    p = Plate(0.0, -PLAT_W / 2, total_l, PLAT_W / 2, 0.0, HANDLE_T)
    # handle is narrower than the platform: void the strips beside it
    p.rect(0.0, -PLAT_W / 2, HANDLE_L, -HANDLE_W / 2, void=True)
    p.rect(0.0, HANDLE_W / 2, HANDLE_L, PLAT_W / 2, void=True)
    # grip grooves across the handle, top and bottom
    for gx in GRIP_GROOVE_POS:
        p.rect(gx - GRIP_GROOVE_W / 2, -HANDLE_W / 2, gx + GRIP_GROOVE_W / 2, HANDLE_W / 2,
               top=HANDLE_T - GRIP_GROOVE_D, bottom=GRIP_GROOVE_D)
    # BH1750 pocket, centred on the platform
    pl, pw = BH_PCB_L + 2 * BH_CLEAR, BH_PCB_W + 2 * BH_CLEAR
    pcx = HANDLE_L + PLAT_L / 2
    px0, px1 = pcx - pl / 2, pcx + pl / 2
    p.rect(px0, -pw / 2, px1, pw / 2, top=HANDLE_T - BH_POCKET_D)
    # cable channel from the pocket to the gripper end, along the top centreline
    p.rect(0.0, -CABLE_W / 2, px0 + EPS, CABLE_W / 2, top=HANDLE_T - CABLE_D)
    # M2 holes for the breakout (assumed spacing along the long axis)
    for dx in (-BH_HOLE_SPACING / 2, BH_HOLE_SPACING / 2):
        p.hole(pcx + dx, 0.0, BH_HOLE_D, margin=0.4)
    return p.build()


def part_bottle_rest():
    c = REST_BASE / 2
    p = Plate(0.0, 0.0, REST_BASE, REST_BASE, 0.0, REST_BASE_T)
    p.hole(7.0, 7.0, M4_HOLE_D)
    p.hole(REST_BASE - 7.0, REST_BASE - 7.0, M4_HOLE_D)
    t = p.build()
    r_in = BOTTLE_D / 2 + BOTTLE_CLEAR                  # 28 -> ID 56
    r_out = r_in + RING_WALL
    z0, z1 = REST_BASE_T - EPS, REST_BASE_T + RING_H
    r_top = r_in + TAPER_H * math.tan(math.radians(TAPER_ANGLE_DEG))
    assert r_out - r_top >= 1.2, "ring wall too thin at the lead-in"
    assert c - r_out > 0
    t += tapered_ring(c, c, r_out, [(z0, r_in), (z1 - TAPER_H, r_in), (z1, r_top)], z0, z1)
    return t


# 6) Carton flap paddle: held in the right gripper for the whole carton job. Handle like the light
#    paddle (same grip grooves), then a long flat blade that pushes flaps and presses tape.
FLAP_HANDLE_L, FLAP_BLADE_L, FLAP_BLADE_W, FLAP_T = 60.0, 150.0, 40.0, 6.0
# 7) Tape rest: a block with a shallow slot; a pre-cut masking-tape strip lies sticky-side up
#    across the slot with its tab end free over the gap, so the gripper can pinch the tab.
TAPE_REST_L, TAPE_REST_W, TAPE_REST_T = 90.0, 40.0, 18.0
TAPE_SLOT_L, TAPE_SLOT_D = 30.0, 12.0


def part_flap_paddle():
    total_l = FLAP_HANDLE_L + FLAP_BLADE_L
    p = Plate(0.0, -FLAP_BLADE_W / 2, total_l, FLAP_BLADE_W / 2, 0.0, FLAP_T)
    p.rect(0.0, -FLAP_BLADE_W / 2, FLAP_HANDLE_L, -HANDLE_W / 2, void=True)
    p.rect(0.0, HANDLE_W / 2, FLAP_HANDLE_L, FLAP_BLADE_W / 2, void=True)
    for gx in GRIP_GROOVE_POS:
        if gx < FLAP_HANDLE_L - 5:
            p.rect(gx - GRIP_GROOVE_W / 2, -HANDLE_W / 2, gx + GRIP_GROOVE_W / 2, HANDLE_W / 2,
                   top=FLAP_T - GRIP_GROOVE_D, bottom=GRIP_GROOVE_D)
    return p.build()


def part_tape_rest():
    p = Plate(0.0, 0.0, TAPE_REST_L, TAPE_REST_W, 0.0, TAPE_REST_T)
    x0 = TAPE_REST_L - TAPE_SLOT_L - 10.0
    p.rect(x0, -EPS, x0 + TAPE_SLOT_L, TAPE_REST_W + EPS, top=TAPE_REST_T - TAPE_SLOT_D)   # the gap the tab hangs over
    p.hole(8.0, 8.0, M4_HOLE_D)
    p.hole(8.0, TAPE_REST_W - 8.0, M4_HOLE_D)
    return p.build()


PARTS = [
    ("nest_cress.stl", part_nest_cress),
    ("paddle_flap.stl", part_flap_paddle),
    ("tape_rest.stl", part_tape_rest),
    ("tag_36h11_id1.stl", lambda: part_tag(1)),
    ("tag_36h11_id2.stl", lambda: part_tag(2)),
    ("paddle_bh1750.stl", part_paddle),
    ("bottle_rest.stl", part_bottle_rest),
]


def _selftest():
    v = np.asarray(box(0, 0, 0, 2, 3, 4)).reshape(-1, 3, 3)
    assert abs(signed_volume(v) - 24.0) < 1e-9, "box orientation"
    v = np.asarray(square_with_hole(0, 0, 5, 2, 0, 3)).reshape(-1, 3, 3)
    exp = (100 - SEGS / 2 * 4 * math.sin(2 * math.pi / SEGS)) * 3       # square - inscribed polygon
    assert abs(signed_volume(v) - exp) < 1e-6, "annulus orientation"
    v = np.asarray(tapered_ring(0, 0, 6, [(0, 4), (1, 4), (2, 5)], 0, 2)).reshape(-1, 3, 3)
    assert signed_volume(v) > 0, "ring orientation"


def main():
    _selftest()
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    ok = True
    for name, fn in PARTS:
        path = os.path.join(OUT_DIR, name)
        v = write_stl(path, fn(), name)
        count = check_stl(path)
        lo, hi = bbox(v)
        size = hi - lo
        vol = signed_volume(v)
        fits = size[0] <= BED_XY and size[1] <= BED_XY
        ok &= fits and vol > 0
        rows.append((name, size, lo, count, os.path.getsize(path), fits, vol))
    print(f"{'STL':<22}{'size X x Y x Z (mm)':<28}{'min corner':<24}{'tris':>7}{'bytes':>10}  fits bed")
    for name, size, lo, count, nbytes, fits, vol in rows:
        print(f"{name:<22}{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f}"
              f"{'':<{28 - len(f'{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f}')}}"
              f"({lo[0]:.1f}, {lo[1]:.1f}, {lo[2]:.1f}){'':<{24 - len(f'({lo[0]:.1f}, {lo[1]:.1f}, {lo[2]:.1f})')}}"
              f"{count:>7}{nbytes:>10}  {'yes' if fits else 'NO'}")
    print("\nmarkdown:")
    print("| STL | X (mm) | Y (mm) | Z (mm) | triangles | file bytes |")
    print("|---|---|---|---|---|---|")
    for name, size, lo, count, nbytes, fits, vol in rows:
        print(f"| `{name}` | {size[0]:.2f} | {size[1]:.2f} | {size[2]:.2f} | {count} | {nbytes} |")
    if not ok:
        print("ERROR: a part does not fit the bed or has inverted orientation", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
