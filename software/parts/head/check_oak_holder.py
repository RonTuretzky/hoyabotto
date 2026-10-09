"""Geometry checks for oak_d_lite_pitch_holder.stl against the upstream gimbal CAD.

Needs the same venv as make_oak_holder.py plus trimesh, manifold3d, shapely, rtree, scipy,
networkx.  Downloads (once, into --cache) the upstream XLeRobot gimbal assembly STEP and the
Luxonis OAK-D Lite enclosure STL; neither is committed here.

    .venv-cad/bin/python check_oak_holder.py --cache /tmp/oak-holder-cache

Checks:
  1. the STL is one watertight solid;
  2. the camera body (placed on the plate by the design transform) does not intersect the
     holder, and the holder's M4 holes line up with the camera's VESA threads;
  3. pitch sweep (exact OCC B-rep distances): the first pitch angle (down and up) at which
     holder + camera (+ a straight 20 mm USB-C plug) touch the gimbal yaw block or base,
     compared with the upstream RealSense D435 holder in the same assembly.
Prints a JSON report and writes it next to the STL as geometry-report.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import urllib.request
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = Path(__file__).resolve().parent
ASSEMBLY_URL = (
    "https://raw.githubusercontent.com/Vector-Wangel/XLeRobot/main/hardware/step/RGBD_Gimbal/"
    "Assembly_RGBD_Gimbal.step"
)
OAK_STL_URL = "https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-Lite/DM9095_enclosure.STL"

# Assembly world -> pitch-holder frame (derived from the holder's bounding box in the assembly:
# holder X = world y - 5.5, Y = world z - 1140.3, Z = world x + 213.5).
WORLD_TO_HOLDER = np.array([[0, 1, 0, -5.5], [0, 0, 1, -1140.3], [1, 0, 0, 213.5], [0, 0, 0, 1.0]])
# OAK enclosure STL (x 0..91 left-right seen from behind, y 0..28 up, z 0..17.45 front->back)
# -> holder frame: back face on the plate (Z = 10.5), bottom edge at Y = CAM_BOTTOM_Y.
PIVOT = np.array([0.0, 0.0, 0.48])


def fetch(url: str, dest: Path) -> Path:
    if not dest.exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        print("downloading", url)
        urllib.request.urlretrieve(url, dest)
    return dest


def step_groups(step: Path, cache: Path) -> dict[str, trimesh.Trimesh]:
    """Tessellate the assembly STEP into named groups (cached as STL)."""
    names = ["base", "yaw", "holder", "servo", "camera"]
    if all((cache / f"asm_{n}.stl").exists() for n in names):
        return {n: trimesh.load(cache / f"asm_{n}.stl", force="mesh") for n in names}
    import cadquery as cq

    sol = cq.importers.importStep(str(step)).solids().vals()
    groups: dict[str, list] = {n: [] for n in names}
    for s in sol:
        b = s.BoundingBox()
        dims = (round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1))
        if dims == (62.3, 64.0, 59.7):
            groups["base"].append(s)
        elif dims == (54.0, 48.0, 39.0):
            groups["yaw"].append(s)
        elif dims == (21.0, 66.4, 49.9):
            groups["holder"].append(s)
        elif dims[:2] == (25.1, 89.9):
            groups["camera"].append(s)
        elif dims[0] >= 19 and dims[2] < 200:
            groups["servo"].append(s)
    out = {}
    for n, v in groups.items():
        assert v, f"assembly group {n} not found"
        cq.exporters.export(cq.Workplane().add(cq.Compound.makeCompound(v)), str(cache / f"asm_{n}.stl"), tolerance=0.05, angularTolerance=0.2)
        out[n] = trimesh.load(cache / f"asm_{n}.stl", force="mesh")
    return out


def tidy(m: trimesh.Trimesh) -> trimesh.Trimesh:
    """Drop stray triangles (OCC occasionally emits a sliver reaching to the origin)."""
    m = m.copy()
    edge_len = np.linalg.norm(m.vertices[m.edges[:, 0]] - m.vertices[m.edges[:, 1]], axis=1).reshape(-1, 3)
    m.update_faces(edge_len.max(axis=1) < 100.0)
    m.remove_unreferenced_vertices()
    return m


def hull_in_holder_frame(m: trimesh.Trimesh) -> trimesh.Trimesh:
    h = tidy(m).convex_hull.copy()
    h.apply_transform(WORLD_TO_HOLDER)
    return h


def box(xmin, xmax, ymin, ymax, zmin, zmax) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=[xmax - xmin, ymax - ymin, zmax - zmin])
    b.apply_translation([(xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2])
    return b


def inter_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    r = trimesh.boolean.intersection([a, b], engine="manifold")
    return float(r.volume) if r is not None and not r.is_empty else 0.0


def touches(moving: trimesh.Trimesh, obstacle_hull: trimesh.Trimesh) -> bool:
    """Convex obstacle vs an arbitrary (possibly non-watertight) mesh: any vertex of either
    inside the other.  Good to about one tessellation edge; the sweep runs in 1 deg steps."""
    if obstacle_hull.contains(moving.vertices).any():
        return True
    return bool(moving.is_volume and moving.contains(obstacle_hull.vertices).any())


def first_contact(moving: list[trimesh.Trimesh], obstacles: list[trimesh.Trimesh], angles) -> float | None:
    """Rotate the obstacles by -angle about the pitch axis (= holder pitched down by angle)."""
    for a in angles:
        R = rotation_matrix(np.radians(-a), [1, 0, 0], point=PIVOT)
        for ob in obstacles:
            o = ob.copy()
            o.apply_transform(R)
            if any(touches(mv, o) for mv in moving):
                return float(a)
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, default=Path("/tmp/oak-holder-cache"))
    ap.add_argument("--stl", type=Path, default=HERE / "oak_d_lite_pitch_holder.stl")
    args = ap.parse_args()
    design = json.loads((HERE / "design.json").read_text())
    cam = design["camera"]
    cam_bottom_y = cam["bottom_edge_y"]
    cam_w, cam_h, cam_d = cam["size_mm"]
    plate_front_z = cam["back_face_z"]

    part = trimesh.load(args.stl, force="mesh")
    report = {
        "stl": args.stl.name,
        "stl_sha256": hashlib.sha256(args.stl.read_bytes()).hexdigest(),
        "watertight": bool(part.is_watertight),
        "components": int(len(part.split(only_watertight=False))),
        "volume_mm3": round(float(part.volume), 1),
        "bbox_min": np.round(part.bounds[0], 2).tolist(),
        "bbox_max": np.round(part.bounds[1], 2).tolist(),
    }

    # --- camera placement ---------------------------------------------------------------
    oak_stl = fetch(OAK_STL_URL, args.cache / "DM9095_enclosure.STL")
    oak = trimesh.load(oak_stl, force="mesh")
    report["oak_enclosure_sha256"] = hashlib.sha256(oak_stl.read_bytes()).hexdigest()
    ob = oak.bounds
    cx = (ob[0][0] + ob[1][0]) / 2
    OAK_TO_HOLDER = np.array(
        [[-1, 0, 0, cx], [0, 1, 0, cam_bottom_y - ob[0][1]], [0, 0, -1, plate_front_z + ob[1][2]], [0, 0, 0, 1.0]]
    )
    oak_h = oak.copy()
    oak_h.apply_transform(OAK_TO_HOLDER)
    report["oak_mesh_bounds_in_holder_frame"] = np.round(oak_h.bounds, 2).tolist()
    # VESA threads from the enclosure mesh: slice just inside the back face, find the two ~3.5 mm holes.
    back_z = oak_h.bounds[0][2]
    sec = oak_h.section(plane_origin=[0, 0, back_z + 0.25], plane_normal=[0, 0, 1])
    vesa = []
    if sec is not None:
        p2, T = sec.to_2D()
        for poly in p2.polygons_full:
            for ring in poly.interiors:
                from shapely.geometry import Polygon

                c = Polygon(ring)
                if 7 < abs(c.area) < 14:
                    pt = np.array([c.centroid.x, c.centroid.y, 0, 1]) @ T.T
                    vesa.append(np.round(pt[:2], 2).tolist())
    report["oak_vesa_holes_from_mesh"] = sorted(vesa)
    report["holder_m4_holes"] = cam["vesa_m4_holes_xy"]
    if len(vesa) == 2:
        d = [np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(sorted(vesa), cam["vesa_m4_holes_xy"])]
        report["vesa_alignment_error_mm"] = [round(x, 2) for x in d]

    cam_box = box(-cam_w / 2, cam_w / 2, cam_bottom_y, cam_bottom_y + cam_h, plate_front_z + 0.02, plate_front_z + cam_d)
    report["camera_box_vs_holder_intersection_mm3"] = round(inter_volume(cam_box, part), 2)
    ux, uz = cam["usb_c_opening_centre_xz"]
    plug = box(ux - 6.5, ux + 6.5, cam_bottom_y - 20.0, cam_bottom_y, uz - 4.0, uz + 4.0)
    report["straight_usb_plug_box_vs_holder_mm3"] = round(inter_volume(plug, part), 2)

    # --- pitch sweep against the gimbal (exact B-rep distances, OCC) ----------------------
    asm = fetch(ASSEMBLY_URL, args.cache / "Assembly_RGBD_Gimbal.step")
    report["assembly_step_sha256"] = hashlib.sha256(asm.read_bytes()).hexdigest()
    g = step_groups(asm, args.cache)
    import cadquery as cq

    sol = cq.importers.importStep(str(asm)).solids().vals()

    def to_holder(shape):
        # WORLD_TO_HOLDER is the cyclic axis permutation (x,y,z) -> (y,z,x) plus a translation:
        # a -120 deg rotation about (1,1,1), which keeps OCC's rigid-transform path happy.
        return shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 1, 1), -120).translate(cq.Vector(-5.5, -1140.3, 213.5))

    def pick(dims3):
        for s_ in sol:
            b = s_.BoundingBox()
            if (round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)) == dims3:
                return to_holder(s_)
        raise RuntimeError(f"solid {dims3} not in assembly")

    yaw_s = pick((54.0, 48.0, 39.0))
    base_s = pick((62.3, 64.0, 59.7))
    lix_s = pick((21.0, 66.4, 49.9))
    d435_s = pick((25.1, 89.9, 25.0))
    part_s = cq.importers.importStep(str(args.stl.with_suffix(".step"))).solids().vals()[0]

    def cq_box(b: trimesh.Trimesh):
        lo, hi = b.bounds
        return cq.Solid.makeBox(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], cq.Vector(*lo))

    cam_s, plug_s = cq_box(cam_box), cq_box(plug)
    bb = yaw_s.BoundingBox()
    report["yaw_block_bounds_in_holder_frame"] = [[round(bb.xmin, 1), round(bb.ymin, 1), round(bb.zmin, 1)], [round(bb.xmax, 1), round(bb.ymax, 1), round(bb.zmax, 1)]]
    # sanity: at 0 deg the upstream holder must not interpenetrate the yaw block
    report["upstream_holder_to_yaw_distance_at_0deg_mm"] = round(lix_s.distance(yaw_s), 3)
    report["oak_holder_to_yaw_distance_at_0deg_mm"] = round(part_s.distance(yaw_s), 3)

    def rotated(shape, deg):
        return shape.rotate(cq.Vector(-1, PIVOT[1], PIVOT[2]), cq.Vector(1, PIVOT[1], PIVOT[2]), -deg)

    def first_contact_brep(moving, obstacles, sign):
        """Smallest |angle| (deg, sign = +1 down / -1 up) at which any moving shape comes within
        0.05 mm of an obstacle rotated by -angle about the pitch axis (3 deg coarse, 1 deg fine)."""
        def hit(a):
            for ob in obstacles:
                o = rotated(ob, sign * a)
                if any(mv.distance(o) < 0.05 for mv in moving):
                    return True
            return False
        coarse = next((a for a in range(0, 91, 3) if hit(a)), None)
        if coarse is None:
            return None
        for a in range(max(coarse - 3, 0), coarse + 1):
            if hit(a):
                return float(sign * a)
        return float(sign * coarse)

    obstacles = [yaw_s, base_s]
    report["pitch_sweep_deg"] = {
        "upstream_d435_holder": {
            "first_contact_down": first_contact_brep([lix_s, d435_s], obstacles, +1),
            "first_contact_up": first_contact_brep([lix_s, d435_s], obstacles, -1),
        },
        "oak_holder_no_plug": {
            "first_contact_down": first_contact_brep([part_s, cam_s], obstacles, +1),
            "first_contact_up": first_contact_brep([part_s, cam_s], obstacles, -1),
        },
        "oak_holder_straight_plug_20mm": {
            "first_contact_down": first_contact_brep([part_s, cam_s, plug_s], obstacles, +1),
            "first_contact_up": first_contact_brep([part_s, cam_s, plug_s], obstacles, -1),
        },
    }
    lix = tidy(g["holder"])
    lix.apply_transform(WORLD_TO_HOLDER)
    d435 = hull_in_holder_frame(g["camera"])
    # preview meshes for the renderer
    out = args.cache / "preview"
    out.mkdir(exist_ok=True)
    oak_h.export(out / "oak_in_holder_frame.stl")
    for n in ["yaw", "servo", "base"]:
        mm = tidy(g[n])
        mm.apply_transform(WORLD_TO_HOLDER)
        mm.export(out / f"{n}_in_holder_frame.stl")
    lix.export(out / "upstream_holder.stl")
    d435.export(out / "d435_hull.stl")
    plug.export(out / "plug_box.stl")
    (HERE / "geometry-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
