"""Checks for the slot-in OAK-D Lite cradle against the stock tilt link and the camera body.

    .venv-cad/bin/python check_oak_slot_cradle.py --cache /tmp/oak-holder-cache

Needs the stock tilt link mesh (object 39 of XLeRobot_0_3_0.3mf, extracted to
<cache>/stock/tilt_link_obj39.stl) and the Luxonis enclosure STL (<cache>/DM9095_enclosure.STL,
downloaded on demand).  Everything is in the tilt-link design frame.  Writes
geometry-report-slot-cradle.json and preview meshes in <cache>/preview_slot/.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import urllib.request
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
OAK_STL_URL = "https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-Lite/DM9095_enclosure.STL"


def fetch(url: str, dest: Path) -> Path:
    if not dest.exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(url, dest)
    return dest


def step_mesh(step: Path) -> trimesh.Trimesh:
    import cadquery as cq

    tmp = step.with_suffix(".design.stl")
    cq.exporters.export(cq.importers.importStep(str(step)), str(tmp), tolerance=0.02, angularTolerance=0.1)
    m = trimesh.load(tmp, force="mesh")
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    trimesh.repair.fill_holes(m)
    tmp.unlink()
    return m


def box(x0, x1, y0, y1, z0, z1) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def ivol(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    r = trimesh.boolean.intersection([a, b], engine="manifold")
    return 0.0 if r is None or r.is_empty else float(r.volume)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, default=Path("/tmp/oak-holder-cache"))
    args = ap.parse_args()
    d = json.loads((HERE / "design_slot_cradle.json").read_text())
    cam = d["camera"]
    link = trimesh.load(args.cache / "stock" / "tilt_link_obj39.stl", force="mesh")
    oak_stl = fetch(OAK_STL_URL, args.cache / "DM9095_enclosure.STL")
    oak = trimesh.load(oak_stl, force="mesh")
    ob = oak.bounds
    cx = (ob[0][0] + ob[1][0]) / 2
    # OAK (x right-seen-from-behind, y up, z front->back) -> link frame (X = -x, Y = front - z, Z = -(y) + bottom)
    T = np.array([[-1, 0, 0, cx], [0, 0, -1, cam["front_face_y"]], [0, -1, 0, cam["bottom_edge_z"] + ob[0][1]], [0, 0, 0, 1.0]])
    oak_h = oak.copy()
    oak_h.apply_transform(T)
    cam_box = box(-45.5, 45.5, cam["back_face_y"] + 0.02, cam["front_face_y"], cam["top_edge_z"], cam["bottom_edge_z"] - 0.02)
    ux, uy = cam["usb_c_opening_centre_xy"]
    plug = box(ux - 6.5, ux + 6.5, uy - 4.0, uy + 4.0, cam["bottom_edge_z"], cam["bottom_edge_z"] + 20.0)
    report = {"link_mesh_watertight": bool(link.is_watertight), "oak_mesh_bounds_in_link_frame": np.round(oak_h.bounds, 2).tolist()}
    out = args.cache / "preview_slot"
    out.mkdir(exist_ok=True)
    oak_h.export(out / "oak.stl")
    link.export(out / "link.stl")
    plug.export(out / "plug.stl")
    stem = "oak_d_lite_slot_cradle"
    part = step_mesh(HERE / f"{stem}.step")
    li = d["link_interface"]
    # M3 screws from each side: 3 mm shank through the foot into the tongue, modelled as cylinders
    screws = []
    for (hy, hz) in li["screw_holes_yz"]:
        for sign, x_out in ((-1, -18.8), (1, 19.2)):
            c = trimesh.creation.cylinder(radius=1.5, height=li["screw_length"], sections=48)
            c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            c.apply_translation([x_out - sign * li["screw_length"] / 2, hy, hz])
            screws.append(c)
    rep = {
        "part_watertight": bool(part.is_watertight),
        "part_volume_mm3": round(float(part.volume), 1),
        "part_vs_link_mm3": round(ivol(part, link), 2),
        "camera_box_vs_part_mm3": round(ivol(cam_box, part), 2),
        "camera_box_vs_link_mm3": round(ivol(cam_box, link), 2),
        "plug_box_vs_part_mm3": round(ivol(plug, part), 2),
        "plug_box_vs_link_mm3": round(ivol(plug, link), 2),
        "m3_shank_vs_link_mm3": [round(ivol(c, link), 2) for c in screws],
        "m3_shank_in_tongue_mm3": [round(ivol(c, part), 2) for c in screws],
    }
    slot = box(-11.3, 12.8, -8.1, 10.1, -18.3, -11.6)
    rep["tongue_volume_in_slot_mm3"] = round(ivol(part, slot), 1)
    r = trimesh.boolean.intersection([part, link], engine="manifold")
    rep["part_vs_link_overlap_bounds"] = None if r is None or r.is_empty else np.round(r.bounds, 2).tolist()
    report[stem] = rep
    part.export(out / f"{stem}.stl")
    for i, c in enumerate(screws):
        c.export(out / f"screw{i}.stl")
    (HERE / "geometry-report-slot-cradle.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
