"""Blender (background) scene: what "arm base", "pan axis", "220 mm" and "120 mm" mean on the robot.

    Blender -b --python build.py -- [--frames 1,120,300] [--out DIR] [--res 1280x720]
Writes PNG frames plus anchors.json (2D positions of the label points on every frame) for overlay.py.
Training-scene frame: +x robot right, +y toward the table, z = 0 table top, metres.
"""
import json, math, sys
from pathlib import Path
import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
opt = dict(zip(argv[::2], argv[1::2]))
OUT = Path(opt.get('--out', '/tmp/foldviz/frames'))
RES = [int(v) for v in opt.get('--res', '1280x720').split('x')]
FRAMES = [int(f) for f in opt['--frames'].split(',')] if '--frames' in opt else None
info = json.load(open('/tmp/foldviz/station.json'))
LP, RP = Vector(info['left_pan_anchor']), Vector(info['right_pan_anchor'])
BASE_Z = info['left_base_origin'][2]          # 0.12: bottom of the arm bases (SO-101 base_link mounting plane)
Y0 = info['left_base_origin'][1]
FPS, END = 24, 600

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = FPS
scene.frame_start, scene.frame_end = 1, END
scene.render.resolution_x, scene.render.resolution_y = RES
for engine in ('BLENDER_EEVEE', 'BLENDER_EEVEE_NEXT'):
    try:
        scene.render.engine = engine
        break
    except TypeError:
        pass
try:
    scene.view_settings.view_transform = 'AgX'
except TypeError:
    scene.view_settings.view_transform = 'Filmic'
world = bpy.data.worlds.new('w'); scene.world = world; world.use_nodes = True
world.node_tree.nodes['Background'].inputs[0].default_value = (0.80, 0.82, 0.86, 1)
world.node_tree.nodes['Background'].inputs[1].default_value = 0.55

bpy.ops.wm.obj_import(filepath='/tmp/foldviz/station.obj', forward_axis='Y', up_axis='Z')
objs = {o.name: o for o in bpy.context.selected_objects}


def mat(name, color, emit=0.0, alpha=1.0, rough=0.6):
    m = bpy.data.materials.new(name); m.use_nodes = True
    p = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Roughness'].default_value = rough
    if emit:
        p.inputs['Emission Color'].default_value = (*color, 1)
        p.inputs['Emission Strength'].default_value = emit
    if alpha < 1:
        p.inputs['Alpha'].default_value = alpha
        try:
            m.surface_render_method = 'BLENDED'
        except AttributeError:
            m.blend_method = 'BLEND'
    return m


def recolor(obj, m):
    obj.data.materials.clear(); obj.data.materials.append(m)


ARM = mat('robot', (0.035, 0.035, 0.04), rough=0.4)
CART = mat('cart', (0.22, 0.22, 0.24), rough=0.5, alpha=0.999)
WOOD = mat('table', (0.62, 0.48, 0.30), rough=0.7)
BASE = mat('base', (0.035, 0.035, 0.04), rough=0.4)
for name, m in (('cart', CART), ('left_arm', ARM), ('right_arm', ARM), ('table', WOOD),
                ('left_base', BASE), ('right_base', BASE)):
    recolor(objs[name], m)
pb_ = next(n for n in BASE.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
pb_.inputs['Emission Color'].default_value = (1.0, 0.42, 0.02, 1)
for f, c, s in ((1, (0.035, 0.035, 0.04), 0), (78, (0.035, 0.035, 0.04), 0), (96, (1.0, 0.42, 0.02), 1.4), (END, (1.0, 0.42, 0.02), 0.9)):
    pb_.inputs['Base Color'].default_value = (*c, 1); pb_.inputs['Base Color'].keyframe_insert('default_value', frame=f)
    pb_.inputs['Emission Strength'].default_value = s; pb_.inputs['Emission Strength'].keyframe_insert('default_value', frame=f)
# the cart fades during the side view so the 120 mm reads clearly
pc = next(n for n in CART.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
for f, a in ((1, 1), (400, 1), (420, 0.25), (540, 0.25), (560, 1)):
    pc.inputs['Alpha'].default_value = a; pc.inputs['Alpha'].keyframe_insert('default_value', frame=f)

for side, anchor, sign in (('left', LP, 1), ('right', RP, -1)):
    o = objs[f'{side}_arm']
    scene.cursor.location = anchor
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    for f, deg in ((1, 0), (176, 0), (198, 30 * sign), (222, -30 * sign), (246, 30 * sign), (268, 0)):
        o.rotation_euler = (0, 0, math.radians(deg)); o.keyframe_insert('rotation_euler', frame=f)


def pop(o, f_in, f_out=None, dur=10):
    for f, s in ((1, 0), (f_in, 0), (f_in + dur, 1)) + (((f_out, 1), (f_out + dur, 0)) if f_out else ()):
        o.scale = (s, s, s); o.keyframe_insert('scale', frame=f)


def cylinder(a, b, r, m, name):
    a, b = Vector(a), Vector(b)
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=(b - a).length, location=(a + b) / 2, vertices=24)
    o = bpy.context.object; o.name = name
    o.rotation_mode = 'QUATERNION'; o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(b - a)
    recolor(o, m); return o


def cone(tip, direction, m, name, r=0.009, h=0.02):
    d = Vector(direction).normalized()
    bpy.ops.mesh.primitive_cone_add(radius1=r, radius2=0, depth=h, location=Vector(tip) - d * h / 2, vertices=24)
    o = bpy.context.object; o.name = name
    o.rotation_mode = 'QUATERNION'; o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(d)
    recolor(o, m); return o


def group(name, parts, pivot):
    e = bpy.data.objects.new(name, None); bpy.context.collection.objects.link(e); e.location = pivot
    bpy.context.view_layer.update()
    for p in parts:
        p.parent = e; p.matrix_parent_inverse = e.matrix_world.inverted()
    return e


def arrow(a, b, m, name, r=0.0035):
    a, b = Vector(a), Vector(b); d = (b - a).normalized()
    return group(name, [cylinder(a + d * .019, b - d * .019, r, m, name + '_s'), cone(b, d, m, name + '_1'),
                        cone(a, -d, m, name + '_2')], (a + b) / 2)


def dashed(a, b, m, name, dash=.014, gap=.009, r=.0028):
    a, b = Vector(a), Vector(b); L = (b - a).length; d = (b - a).normalized(); parts = []; s = 0
    while s < L:
        parts.append(cylinder(a + d * s, a + d * min(s + dash, L), r, m, f'{name}_{len(parts)}')); s += dash + gap
    return group(name, parts, (a + b) / 2)


PINK, BLUE = (0.86, 0.10, 0.42), (0.05, 0.40, 0.85)
PINK_M, BLUE_M = mat('pink', PINK, emit=1.2), mat('blue', BLUE, emit=1.2)
for side, anchor in (('left', LP), ('right', RP)):
    pop(dashed((anchor.x, anchor.y, BASE_Z - 0.03), (anchor.x, anchor.y, BASE_Z + 0.34), PINK_M, f'axis_{side}'), 176, None, 12)
H220 = BASE_Z + 0.31
pop(arrow((LP.x, LP.y, H220), (RP.x, RP.y, H220), PINK_M, 'arrow_220'), 292)
# 120 mm: a plane through the bottom of both arm bases, carried forward to the table edge, and an arrow down to the top
XA, YA = 0.215, -0.135
bpy.ops.mesh.primitive_plane_add(size=1, location=(0, (Y0 - 0.06 + YA + 0.02) / 2, BASE_Z))
pl = bpy.context.object; pl.name = 'plane_base'
pl.dimensions = (0.48, (YA + 0.02) - (Y0 - 0.06), 0); recolor(pl, mat('plane_base', BLUE, emit=0.4, alpha=0.3)); pop(pl, 410, 555)
pop(arrow((XA, YA, 0.0), (XA, YA, BASE_Z), BLUE_M, 'arrow_120'), 426)

CAM_DATA = bpy.data.cameras.new('cam'); CAM_DATA.lens = 32
CAM = bpy.data.objects.new('cam', CAM_DATA); bpy.context.collection.objects.link(CAM); scene.camera = CAM
TARGET = bpy.data.objects.new('target', None); bpy.context.collection.objects.link(TARGET)
t = CAM.constraints.new('TRACK_TO'); t.target = TARGET; t.track_axis = 'TRACK_NEGATIVE_Z'; t.up_axis = 'UP_Y'
SHOTS = [  # frame, camera, look-at
    (1, (1.25, 1.05, 0.85), (0, -0.18, 0.12)),          # the station from the table side
    (60, (1.05, 0.85, 0.75), (0, -0.20, 0.12)),
    (110, (-0.55, -0.95, 0.62), (0, -0.28, 0.15)),      # behind the robot: the arm bases
    (176, (-0.45, -0.85, 0.60), (0, -0.28, 0.17)),
    (270, (-0.30, -0.85, 0.70), (0, -0.27, 0.20)),
    (300, (0.0, -0.80, 0.78), (0, -0.25, 0.22)),        # straight behind and above: both pan axes, 220 mm
    (390, (0.0, -0.80, 0.78), (0, -0.25, 0.22)),
    (410, (0.85, -0.85, 0.45), (0, -0.22, 0.12)),       # swing around the robot's right side, outside it
    (430, (1.10, -0.20, 0.11), (0, -0.20, 0.07)),       # from the robot's right, low: 120 mm
    (540, (1.10, -0.20, 0.11), (0, -0.20, 0.07)),
    (585, (1.05, 0.85, 0.75), (0, -0.20, 0.12)),
    (END, (1.05, 0.85, 0.75), (0, -0.20, 0.12)),
]
for f, loc, look in SHOTS:
    CAM.location = loc; CAM.keyframe_insert('location', frame=f)
    TARGET.location = look; TARGET.keyframe_insert('location', frame=f)

sun = bpy.data.objects.new('sun', bpy.data.lights.new('sun', 'SUN')); bpy.context.collection.objects.link(sun)
sun.data.energy = 4.0; sun.rotation_euler = (math.radians(35), math.radians(10), math.radians(-35))
fill = bpy.data.objects.new('fill', bpy.data.lights.new('fill', 'AREA')); bpy.context.collection.objects.link(fill)
fill.data.energy = 150; fill.data.size = 2.5; fill.location = (0.4, -1.4, 1.2)
fill.rotation_euler = (math.radians(55), 0, math.radians(-10))

ANCHORS = {
    'base_left': (LP.x, Y0, BASE_Z + 0.075), 'base_right': (RP.x, Y0, BASE_Z + 0.075),
    'axis_left': (LP.x, LP.y, BASE_Z + 0.35), 'axis_right': (RP.x, RP.y, BASE_Z + 0.35),
    'mm220': (0, LP.y, H220), 'mm120': (XA, YA, BASE_Z / 2), 'plane': (XA, YA, BASE_Z),
    'table': (XA, YA, 0.0), 'carton_rim': (0.19, -0.157, 0.108),
}
bpy.ops.wm.save_as_mainfile(filepath='/tmp/foldviz/scene.blend')
OUT.mkdir(parents=True, exist_ok=True)
scene.render.image_settings.file_format = 'PNG'
anchors = {}
for f in (FRAMES or range(1, END + 1)):
    scene.frame_set(f)
    anchors[f] = {k: list(world_to_camera_view(scene, CAM, Vector(v)))[:2] for k, v in ANCHORS.items()}
    scene.render.filepath = str(OUT / f'f{f:04d}.png')
    bpy.ops.render.render(write_still=True)
json.dump(anchors, open(OUT / 'anchors.json', 'w'))
print('rendered', len(anchors))
