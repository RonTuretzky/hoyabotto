"""OAK-D Lite pitch holder for the XLeRobot RGBD head gimbal.

Replaces ``Gimbal_Pitch_Holder_d435`` / ``_d455`` (Lix, XLeRobot hardware/step/RGBD_Gimbal,
Apache-2.0).  The servo arms and the bridge between them are taken unchanged from the
upstream STEP, so the STS3215 horn / idler interface is exactly the stock one.  The
RealSense plate is replaced by a 94 x 31 mm plate carrying the OAK-D Lite on its two
VESA-75 M4 threads (datasheet: 75 mm apart, 18.5 mm above the bottom edge, M4 x 6.5 deep).

Frame (same as the upstream STEP): X = pitch axis, +Y = up, +Z = forward (camera view).
The pitch axis passes through (Y = 0, Z = 0.48).  Millimetres.

Run from this directory with a Python 3.12 venv that has cadquery 2.8:

    uv venv -p 3.12 .venv-cad && uv pip install -p .venv-cad/bin/python cadquery==2.8.0
    .venv-cad/bin/python make_oak_holder.py

Writes oak_d_lite_pitch_holder.stl and .step next to this file.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "source" / "Gimbal_Pitch_Holder_d435_d415.step"
OUT_STL = HERE / "oak_d_lite_pitch_holder.stl"
OUT_STEP = HERE / "oak_d_lite_pitch_holder.step"
OUT_JSON = HERE / "design.json"

# --- OAK-D Lite (Luxonis datasheet Dec 2021 + DM9095 enclosure STEP) -------------------
CAM_W, CAM_H, CAM_D = 91.0, 28.0, 17.45       # width (X), height (Y), depth (Z)
VESA_DX = 37.5                                 # M4 threads at +-37.5 from centre
VESA_DY = 18.51                                # ... and 18.5 above the bottom edge
LENS_DY = 19.06                                # lens row above the bottom edge
USB_DX, USB_DZ, USB_W, USB_H = -15.55, 5.77, 12.8, 6.8   # USB-C opening on the bottom face
TRIPOD_DX, TRIPOD_DZ = 30.18, 12.9             # 1/4-20 on the bottom face (unused here)

# --- holder geometry --------------------------------------------------------------------
PLATE_Z0, PLATE_Z1 = 5.5, 10.5                 # upstream plate thickness, front face at 10.5
CAM_BOTTOM_Y = 24.0                            # camera bottom edge; back face on Z = 10.5
PLATE_HALF_W = 47.0                            # 94 mm plate (camera is 91)
PLATE_Y0 = 22.0                                # overlaps the upstream bridge (Y 15..26.8)
PLATE_Y1 = CAM_BOTTOM_Y + CAM_H + 1.0          # 1 mm above the camera top
PLATE_CORNER_R = 6.0
WINDOW_HALF_W, WINDOW_Y0, WINDOW_Y1, WINDOW_R = 30.0, 29.0, 49.0, 3.0   # lets the fins breathe
M4_CLEARANCE = 4.4
KEEP_HALF_W = 26.5                             # arms are at |X| 18.2..26.2
KEEP_Y_MAX = 26.83                             # top of the upstream bridge block
PIVOT = (0.0, 0.48)                            # (Y, Z) of the pitch axis


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean_stl(path: Path) -> dict:
    """OCC tessellates each face separately, which leaves hairline cracks between faces.
    Weld the vertices, fill the remaining pinholes, drop stray slivers and rewrite the STL
    as one watertight shell (what a slicer wants)."""
    import trimesh

    m = trimesh.load(path, force="mesh")
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    parts = m.split(only_watertight=False)
    m = max(parts, key=lambda q: q.area)
    trimesh.repair.fill_holes(m)
    trimesh.repair.fix_normals(m)
    assert m.is_watertight, "cleaned STL is still not watertight"
    m.export(path)
    return {"faces": int(len(m.faces)), "watertight": bool(m.is_watertight), "mesh_volume_mm3": round(float(m.volume), 1)}


def build() -> cq.Workplane:
    lix = cq.importers.importStep(str(SOURCE))

    # 1. Keep the servo arms and the bridge: |X| <= 26.5, Y <= top of bridge.
    keep_box = (
        cq.Workplane("XY")
        .box(2 * KEEP_HALF_W, KEEP_Y_MAX + 12.0, 24.0, centered=(True, False, True))
        .translate((0, -12.0, 0))
    )
    arms = lix.intersect(keep_box)

    # 2. New camera plate, same thickness and front plane as the upstream plate.
    plate = (
        cq.Workplane("XY")
        .box(2 * PLATE_HALF_W, PLATE_Y1 - PLATE_Y0, PLATE_Z1 - PLATE_Z0, centered=(True, False, False))
        .translate((0, PLATE_Y0, PLATE_Z0))
        .edges("|Z")
        .fillet(PLATE_CORNER_R)
    )

    # 3. Vent window between the two VESA posts.
    window = (
        cq.Workplane("XY")
        .box(2 * WINDOW_HALF_W, WINDOW_Y1 - WINDOW_Y0, 40.0, centered=(True, False, True))
        .translate((0, WINDOW_Y0, PLATE_Z0))
        .edges("|Z")
        .fillet(WINDOW_R)
    )

    # 4. M4 clearance holes on the VESA-75 pattern.
    vesa_y = CAM_BOTTOM_Y + VESA_DY
    holes = (
        cq.Workplane("XY")
        .workplane(offset=-10.0)
        .pushPoints([(-VESA_DX, vesa_y), (VESA_DX, vesa_y)])
        .circle(M4_CLEARANCE / 2)
        .extrude(40.0)
    )

    # 5. Remove the 0.3 mm stubs of the upstream lobes that sit behind the plate.
    stub_cut = (
        cq.Workplane("XY")
        .box(200.0, 40.0, 20.0, centered=(True, False, False))
        .translate((0, PLATE_Y0, PLATE_Z0 - 20.0))
        .cut(cq.Workplane("XY").box(2 * 26.2, 40.0, 60.0, centered=(True, False, True)).translate((0, PLATE_Y0, 0)))
    )

    part = arms.union(plate).cut(window).cut(holes).cut(stub_cut)
    return part


def main() -> None:
    part = build()
    solids = part.solids().vals()
    assert len(solids) == 1, f"expected one solid, got {len(solids)}"
    solid = solids[0]
    bb = solid.BoundingBox()
    cq.exporters.export(part, str(OUT_STL), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(part, str(OUT_STEP))
    mesh_stats = clean_stl(OUT_STL)
    info = {
        "part": OUT_STL.name,
        "frame": "X pitch axis, +Y up, +Z forward; pitch axis at Y=0, Z=0.48; mm",
        "bbox_min": [round(bb.xmin, 2), round(bb.ymin, 2), round(bb.zmin, 2)],
        "bbox_max": [round(bb.xmax, 2), round(bb.ymax, 2), round(bb.zmax, 2)],
        "volume_mm3": round(solid.Volume(), 1),
        "camera": {
            "model": "OAK-D Lite (DM9095)",
            "size_mm": [CAM_W, CAM_H, CAM_D],
            "bottom_edge_y": CAM_BOTTOM_Y,
            "back_face_z": PLATE_Z1,
            "front_face_z": PLATE_Z1 + CAM_D,
            "vesa_m4_holes_xy": [[-VESA_DX, round(CAM_BOTTOM_Y + VESA_DY, 2)], [VESA_DX, round(CAM_BOTTOM_Y + VESA_DY, 2)]],
            "lens_row_y": round(CAM_BOTTOM_Y + LENS_DY, 2),
            "usb_c_opening_centre_xz": [USB_DX, round(PLATE_Z1 + CAM_D - USB_DZ, 2)],
            "tripod_thread_xz": [TRIPOD_DX, round(PLATE_Z1 + CAM_D - TRIPOD_DZ, 2)],
        },
        "screws": "2 x M4 x 10 (5 mm plate + 5 mm into the 6.5 mm deep camera threads); horn/idler screws as the stock holder",
        "mesh": mesh_stats,
        "source_step_sha256": sha256(SOURCE),
        "stl_sha256": sha256(OUT_STL),
        "step_sha256": sha256(OUT_STEP),
    }
    OUT_JSON.write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
