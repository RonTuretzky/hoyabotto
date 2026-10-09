"""Blender previews for the slot-in cradle.  Meshes come from check_oak_slot_cradle.py.

    /Applications/Blender.app/Contents/MacOS/Blender -b -P render_slot_preview.py -- --cache /tmp/oak-holder-cache
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
args = ap.parse_args(argv)
pre = os.path.join(args.cache, "preview_slot")
os.makedirs(args.out, exist_ok=True)
COL = {"cradle": (0.95, 0.55, 0.15, 1), "oak": (0.25, 0.25, 0.28, 1), "link": (0.35, 0.7, 0.35, 1), "plug": (0.2, 0.4, 0.9, 1), "pin": (0.8, 0.3, 0.3, 1)}


def setup():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "MATERIAL"
    sc.display.shading.show_cavity = True
    sc.render.resolution_x, sc.render.resolution_y = 1000, 750
    cam = bpy.data.cameras.new("c")
    cam.type = "ORTHO"
    cam.clip_end = 1e6
    co = bpy.data.objects.new("c", cam)
    sc.collection.objects.link(co)
    sc.camera = co
    return sc, cam, co


def load(path, name, color, design_frame=True):
    bpy.ops.wm.stl_import(filepath=path)
    ob = bpy.context.selected_objects[0]
    ob.name = name
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED").inputs["Base Color"].default_value = color
    m.diffuse_color = color
    ob.data.materials.append(m)
    if design_frame:
        # design: camera side is -Z, front is +Y.  Blender: up +Z, front -Y  ->  rotate 180 deg about X.
        ob.rotation_euler = (3.14159265, 0, 0)
    return ob


def shoot(sc, cam, co, objs, d, path, mul=1.1):
    mn, mx = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for ob in objs:
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            mn, mx = Vector(map(min, mn, w)), Vector(map(max, mx, w))
    ctr, size = (mn + mx) / 2, mx - mn
    d = Vector(d).normalized()
    co.location = ctr + d * max(size) * 4
    co.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    cam.ortho_scale = max(size) * mul
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)


stem = "oak_d_lite_slot_cradle"
sc, cam, co = setup()
objs = [load(os.path.join(pre, f"{stem}.stl"), "cradle", COL["cradle"])]
for n in ["oak", "link", "plug"]:
    objs.append(load(os.path.join(pre, n + ".stl"), n, COL[n]))
for i in range(4):
    objs.append(load(os.path.join(pre, f"screw{i}.stl"), f"screw{i}", COL["pin"]))
for name, d in {"iso": (1, -1, 0.7), "iso_rear": (-1, 1, 0.6), "front": (0, -1, 0), "side": (1, 0, 0), "top": (0, 0, 1)}.items():
    shoot(sc, cam, co, objs, d, os.path.join(args.out, f"slot_{name}.png"))
# cradle alone, print orientation (the STL in the repo is already in print orientation)
sc, cam, co = setup()
h = load(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{stem}.stl"), "cradle", COL["cradle"], design_frame=False)
shoot(sc, cam, co, [h], (1, -1, 0.8), os.path.join(args.out, "slot_print_orientation.png"))
print("rendered")
