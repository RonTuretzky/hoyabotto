"""OAK-D Lite cradle that slots onto the XLeRobot head's stock tilt link (no screws).

The kit's original head is: neck -> pan servo -> pan bracket -> tilt STS3215 -> tilt link
(a U with the horn discs and, under its bridge, two "feet" 7.5 mm wide with a 24 mm slot
between them and a 5.7 mm hole through each foot) -> camera connector tongue in that slot.
This part is that connector, drawn for the OAK-D Lite:

* a tongue (23.7 x 20.9 x 9.5 mm) that slides into the slot between the feet from the front
  and takes the stock connector's four M3 screws (two from each side, through the feet's
  counterbored holes into pilot holes in the tongue) -- the stock attachment, nothing new to buy;
* in front of the link, a box the camera pushes into from the front: back wall with a vent
  window over the fins, ledge, top plate, end walls and four snap fingers that click over
  the camera's front edge.  USB-C and both lenses stay open; the plug hangs below the camera.

Frame = the tilt link's own frame in the XLeRobot 0.3.0 .3mf (object 39): X = tilt axis,
+Y = toward the camera (front), the camera is on the -Z side of the link, the tilt axis is
at Z = 8.33.  Millimetres.  The feet and their holes are symmetric about the feet's
centre (Y = 1), so the same part fits whichever way the link faces.

    .venv-cad/bin/python make_oak_slot_cradle.py
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent

# --- OAK-D Lite --------------------------------------------------------------------------
CAM_W, CAM_H, CAM_D = 91.0, 28.0, 17.45
USB_X, USB_FROM_FRONT, USB_W, USB_H = -15.55, 5.77, 12.8, 6.8  # port centre on the bottom edge
LENS_FROM_BOTTOM = 19.06

# --- stock tilt link (object 39 of XLeRobot_0_3_0.3mf) ---------------------------------
LINK_FRONT_Y = 10.1            # front face of the feet / bridge
LINK_BACK_Y = -8.1             # back face of the feet
FEET_INNER = (-11.3, 12.8)     # slot between the feet (X)
FEET_OUTER = (-18.8, 19.2)     # outer faces of the feet (X)
FEET_BOTTOM_Z = -18.3          # feet end (camera side is -Z)
BRIDGE_UNDER_Z = -11.6         # bridge underside between the feet
HOLES_YZ = ((6.0, -13.6), (-4.0, -14.1))   # M3 holes through the feet: 3.2 mm, counterbored 5.6 x 4.8 from outside
PILOT_D = 2.7                  # M3 self-tapping pilot in the tongue
PILOT_DEPTH = 9.0              # from each side of the tongue
SCREW_LENGTH = 10.0            # M3 x 10: 4.8 in the counterbore, 2.7 through the foot, 2.5+ into the tongue (M3 x 12 also fine)

# --- cradle ------------------------------------------------------------------------------
GAP = 0.2
WALL_T, PLATE_T, END_T = 3.0, 3.0, 2.0
WALL_Y0 = LINK_FRONT_Y + GAP                  # 10.3, back face of the wall
WALL_Y1 = WALL_Y0 + WALL_T                    # 13.3, camera back sits here
CAM_BOTTOM_Z = -13.0                          # camera bottom edge (ledge top)
CAM_TOP_Z = CAM_BOTTOM_Z - CAM_H              # -41.0
CAM_FRONT_Y = WALL_Y1 + CAM_D                 # 30.75
HOOK_Y = CAM_FRONT_Y + GAP                    # 30.95, inner face of the hooks
FRONT_Y = HOOK_Y + 2.0                        # 32.95, front of the part
HALF_W = CAM_W / 2 + 0.25 + END_T             # 47.75
WINDOW_HALF_W, WINDOW_Z0, WINDOW_Z1 = 30.0, -17.5, -36.5
TONGUE_Z0, TONGUE_Z1 = FEET_BOTTOM_Z - PLATE_T, BRIDGE_UNDER_Z - GAP   # -21.3 .. -11.8 (3 mm below the feet, under the pins)
TONGUE_BACK_Y = LINK_BACK_Y - 2.0             # -10.1, so the teardrop pin holes stay inside the tongue
FINGER_W, FINGER_L, FINGER_T, HOOK_OVERLAP, SLIT = 12.0, 14.0, 2.0, 1.0, 1.0
FINGER_X = (-32.0, 32.0)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def build() -> cq.Workplane:
    # back wall in front of the link (from the tongue zone down to the top plate)
    wall = box(-HALF_W, HALF_W, WALL_Y0, WALL_Y1, CAM_TOP_Z - PLATE_T, TONGUE_Z1)
    window = box(-WINDOW_HALF_W, WINDOW_HALF_W, WALL_Y0 - 1, WALL_Y1 + 1, WINDOW_Z1, WINDOW_Z0).edges("|Y").fillet(3.0)
    wall = wall.cut(window)
    ledge = box(-HALF_W, HALF_W, WALL_Y0, FRONT_Y, CAM_BOTTOM_Z, CAM_BOTTOM_Z + PLATE_T)
    top = box(-HALF_W, HALF_W, WALL_Y0, FRONT_Y, CAM_TOP_Z - PLATE_T, CAM_TOP_Z)
    ends = box(-HALF_W, -HALF_W + END_T, WALL_Y0, FRONT_Y, CAM_TOP_Z - PLATE_T, CAM_BOTTOM_Z + PLATE_T).union(
        box(HALF_W - END_T, HALF_W, WALL_Y0, FRONT_Y, CAM_TOP_Z - PLATE_T, CAM_BOTTOM_Z + PLATE_T)
    )
    body = wall.union(ledge).union(top).union(ends)

    # USB-C opening through the ledge, below the camera's port
    usb = box(USB_X - USB_W / 2 - 1, USB_X + USB_W / 2 + 1, CAM_FRONT_Y - USB_FROM_FRONT - USB_H / 2 - 1,
              CAM_FRONT_Y - USB_FROM_FRONT + USB_H / 2 + 1, CAM_BOTTOM_Z - 1, CAM_BOTTOM_Z + PLATE_T + 1)
    body = body.cut(usb)

    # snap fingers: slit the ledge and top plate, thin the finger, add the hook with an entry chamfer
    for fx in FINGER_X:
        for (z_plate0, z_plate1, inward) in ((CAM_BOTTOM_Z, CAM_BOTTOM_Z + PLATE_T, -1), (CAM_TOP_Z - PLATE_T, CAM_TOP_Z, +1)):
            # inward: direction (sign of Z) from the plate into the camera cavity
            y0 = FRONT_Y - FINGER_L
            for sx in (-1, 1):
                body = body.cut(box(fx + sx * FINGER_W / 2 - (SLIT if sx < 0 else 0), fx + sx * FINGER_W / 2 + (SLIT if sx > 0 else 0),
                                    y0, FRONT_Y + 1, z_plate0 - 1, z_plate1 + 1))
            # thin the finger from the outside so it is FINGER_T thick
            if inward < 0:   # bottom plate: cavity is at -Z, remove material at +Z side
                body = body.cut(box(fx - FINGER_W / 2 - 0.01, fx + FINGER_W / 2 + 0.01, y0 - 1, FRONT_Y + 1, z_plate0 + FINGER_T, z_plate1 + 1))
                hook = box(fx - FINGER_W / 2, fx + FINGER_W / 2, HOOK_Y, FRONT_Y, z_plate0 - HOOK_OVERLAP, z_plate0)
                cham = (cq.Workplane("YZ").polyline([(HOOK_Y, z_plate0 - HOOK_OVERLAP), (FRONT_Y, z_plate0 - HOOK_OVERLAP), (FRONT_Y, z_plate0 - HOOK_OVERLAP - 2.0)]).close()
                        .extrude(FINGER_W).translate((fx - FINGER_W / 2, 0, 0)))
            else:            # top plate: cavity is at +Z, remove material at -Z side
                body = body.cut(box(fx - FINGER_W / 2 - 0.01, fx + FINGER_W / 2 + 0.01, y0 - 1, FRONT_Y + 1, z_plate0 - 1, z_plate1 - FINGER_T))
                hook = box(fx - FINGER_W / 2, fx + FINGER_W / 2, HOOK_Y, FRONT_Y, z_plate1, z_plate1 + HOOK_OVERLAP)
                cham = (cq.Workplane("YZ").polyline([(HOOK_Y, z_plate1 + HOOK_OVERLAP), (FRONT_Y, z_plate1 + HOOK_OVERLAP), (FRONT_Y, z_plate1 + HOOK_OVERLAP + 2.0)]).close()
                        .extrude(FINGER_W).translate((fx - FINGER_W / 2, 0, 0)))
            body = body.union(hook.cut(cham))

    # tongue between the feet (on the back of the wall) and a bar under the feet
    tongue = box(FEET_INNER[0] + GAP, FEET_INNER[1] - GAP, TONGUE_BACK_Y, WALL_Y0 + 0.5, TONGUE_Z0, TONGUE_Z1)
    bar = box(FEET_OUTER[0] - GAP, FEET_OUTER[1] + GAP, TONGUE_BACK_Y, WALL_Y0 + 0.5, TONGUE_Z0, FEET_BOTTOM_Z - GAP)
    body = body.union(tongue).union(bar)
    # M3 pilot holes from both sides, on the feet's hole pattern
    for (hy, hz) in HOLES_YZ:
        for x0, length in ((FEET_INNER[0] + GAP - 1.0, PILOT_DEPTH + 1.0), (FEET_INNER[1] - GAP - PILOT_DEPTH, PILOT_DEPTH + 1.0)):
            body = body.cut(cq.Workplane("YZ").center(hy, hz).circle(PILOT_D / 2).extrude(length).translate((x0, 0, 0)))
    return body


def to_print_orientation(wp: cq.Workplane, front_y: float) -> cq.Workplane:
    """Front face (+Y) down on the bed: (x, y, z) -> (x, z, front_y - y)."""
    return wp.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, 0, front_y))


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def export(wp: cq.Workplane, stem: str, print_wp: cq.Workplane | None = None) -> dict:
    import trimesh

    stl = HERE / f"{stem}.stl"
    step = HERE / f"{stem}.step"
    cq.exporters.export(print_wp if print_wp is not None else wp, str(stl), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(wp, str(step))
    m = trimesh.load(stl, force="mesh")
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    m = max(m.split(only_watertight=False), key=lambda q: q.area)
    trimesh.repair.fill_holes(m)
    trimesh.repair.fix_normals(m)
    assert m.is_watertight, f"{stl.name} not watertight"
    m.export(stl)
    s = wp.solids().vals()
    assert len(s) == 1, f"{stem}: {len(s)} solids"
    bb = s[0].BoundingBox()
    return {
        "stl": stl.name, "step": step.name,
        "bbox_min": [round(bb.xmin, 2), round(bb.ymin, 2), round(bb.zmin, 2)],
        "bbox_max": [round(bb.xmax, 2), round(bb.ymax, 2), round(bb.zmax, 2)],
        "volume_mm3": round(s[0].Volume(), 1), "faces": int(len(m.faces)),
        "stl_sha256": sha256(stl), "step_sha256": sha256(step),
    }


def main() -> None:
    info = {
        "frame": "tilt link frame (XLeRobot_0_3_0.3mf object 39): X tilt axis, +Y front (camera side), camera at -Z, axis at Z=8.33; mm",
        "stl_orientation": "STL files are in print orientation: part front face on the bed, +Z up (design +Y -> print -Z); STEP files are in the design frame",
        "camera": {
            "bottom_edge_z": CAM_BOTTOM_Z, "top_edge_z": CAM_TOP_Z, "back_face_y": WALL_Y1, "front_face_y": CAM_FRONT_Y,
            "lens_row_z": round(CAM_BOTTOM_Z - LENS_FROM_BOTTOM, 2),
            "usb_c_opening_centre_xy": [USB_X, round(CAM_FRONT_Y - USB_FROM_FRONT, 2)],
        },
        "link_interface": {
            "tongue_x": [FEET_INNER[0] + GAP, FEET_INNER[1] - GAP], "tongue_y": [TONGUE_BACK_Y, WALL_Y0], "tongue_z": [TONGUE_Z0, TONGUE_Z1],
            "bar_under_feet_z": [TONGUE_Z0, FEET_BOTTOM_Z - GAP],
            "screw_holes_yz": HOLES_YZ, "pilot_d": PILOT_D, "pilot_depth": PILOT_DEPTH, "screw": "M3 x 10 (4 off, from the stock connector)", "screw_length": SCREW_LENGTH,
        },
        "parts": {},
    }
    wp = build()
    info["parts"]["oak_d_lite_slot_cradle"] = export(wp, "oak_d_lite_slot_cradle", to_print_orientation(wp, FRONT_Y))
    (HERE / "design_slot_cradle.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
