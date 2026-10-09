"""Render preview PNGs of the OAK-D Lite pitch holder with Blender (workbench, orthographic).

    /Applications/Blender.app/Contents/MacOS/Blender -b -P render_preview.py -- \
        --cache /tmp/oak-holder-cache --out preview

Expects the meshes written by check_oak_holder.py in <cache>/preview/.
"""
import argparse
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1 :]
ap = argparse.ArgumentParser()
ap.add_argument("--cache", default="/tmp/oak-holder-cache")
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "preview"))
ap.add_argument("--stl", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "oak_d_lite_pitch_holder.stl"))
args = ap.parse_args(argv)
pre = os.path.join(args.cache, "preview")
os.makedirs(args.out, exist_ok=True)

COL = {
    "holder": (0.95, 0.55, 0.15, 1),
    "oak": (0.25, 0.25, 0.28, 1),
    "yaw": (0.35, 0.7, 0.35, 1),
    "servo": (0.15, 0.15, 0.15, 1),
    "base": (0.8, 0.3, 0.3, 1),
    "plug": (0.2, 0.4, 0.9, 1),
}


def load(path, name, color, upright=True):
    bpy.ops.wm.stl_import(filepath=path)
    ob = bpy.context.selected_objects[0]
    ob.name = name
    if upright:
        # design frame: X pitch axis, +Y up, +Z forward.  Blender renders with +Z up, so turn
        # the mesh 90 deg about X (design Y -> Blender Z, design Z -> Blender -Y).
        ob.rotation_euler = (1.5707963, 0, 0)
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    b.inputs["Base Color"].default_value = color
    m.diffuse_color = color
    ob.data.materials.append(m)
    return ob


def setup_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "MATERIAL"
    sc.display.shading.show_cavity = True
    sc.render.resolution_x, sc.render.resolution_y = 1200, 900
    sc.render.film_transparent = False
    cam = bpy.data.cameras.new("c")
    cam.type = "ORTHO"
    cam.clip_end = 1e6
    camob = bpy.data.objects.new("c", cam)
    sc.collection.objects.link(camob)
    sc.camera = camob
    return sc, cam, camob


def bounds(objs):
    mn, mx = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for ob in objs:
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            mn, mx = Vector(map(min, mn, w)), Vector(map(max, mx, w))
    return mn, mx


def shoot(sc, cam, camob, objs, direction, path, scale_mul=1.1):
    mn, mx = bounds(objs)
    ctr, size = (mn + mx) / 2, mx - mn
    d = Vector(direction).normalized()
    camob.location = ctr + d * max(size) * 4
    camob.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    cam.ortho_scale = max(size) * scale_mul
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)


# --- 1. holder alone, print orientation (front face down) ----------------------------
sc, cam, camob = setup_scene()
h = load(args.stl, "holder", COL["holder"], upright=False)
h.rotation_euler = (3.14159265, 0, 0)  # flip so the Z=10.5 face is on the bed
bpy.context.view_layer.update()
shoot(sc, cam, camob, [h], (1, -1, 0.9), os.path.join(args.out, "holder_print_orientation.png"))

# --- 2. holder alone, design frame ----------------------------------------------------
sc, cam, camob = setup_scene()
h = load(args.stl, "holder", COL["holder"])
# Blender view directions: front of the camera is -Y, up is +Z.
for name, d in {"front": (0, -1, 0), "back": (0, 1, 0), "iso": (1, -1, 0.8), "side": (1, 0, 0)}.items():
    shoot(sc, cam, camob, [h], d, os.path.join(args.out, f"holder_{name}.png"))

# --- 3. assembly: holder + OAK + gimbal ----------------------------------------------
sc, cam, camob = setup_scene()
objs = [load(args.stl, "holder", COL["holder"])]
for n in ["oak_in_holder_frame", "yaw_in_holder_frame", "servo_in_holder_frame", "base_in_holder_frame", "plug_box"]:
    p = os.path.join(pre, n + ".stl")
    if os.path.exists(p):
        key = n.split("_")[0]
        objs.append(load(p, n, COL.get(key, (0.6, 0.6, 0.6, 1))))
for name, d in {"iso": (1, -1, 0.7), "front": (0, -1, 0), "side": (1, 0, 0), "iso_rear": (-1, 1, 0.7)}.items():
    shoot(sc, cam, camob, objs, d, os.path.join(args.out, f"assembly_{name}.png"), scale_mul=1.05)
print("rendered to", args.out)
