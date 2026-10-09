"""Blender (background): one timeline, one segment per setup step; cut into per-step clips afterwards.

    Blender -b --python build_steps.py -- [--frames a,b,c] [--out DIR] [--res 1280x720]
Segments (24 fps): step3 1-300, step4 301-720, tags 721-900, step5 901-1020, step6a 1021-1140, step8b 1141-1260.
Frame: +x robot right, +y toward the table, z = 0 table top, metres. Writes anchors.json for overlay_steps.py.
"""
import json, math, sys
from pathlib import Path
import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector, Matrix, Quaternion

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
opt = dict(zip(argv[::2], argv[1::2]))
OUT = Path(opt.get('--out', '/tmp/foldviz/frames_steps'))
RES = [int(v) for v in opt.get('--res', '1280x720').split('x')]
FRAMES = [int(f) for f in opt['--frames'].split(',')] if '--frames' in opt else None
info = json.load(open('/tmp/foldviz/station_split.json'))
LP, RP = Vector(info['left_pan_anchor']), Vector(info['right_pan_anchor'])
BASE_Z, Y0 = info['left_base_origin'][2], info['left_base_origin'][1]
L, W, H = 0.379, 0.283, 0.108
EDGE_Y = -0.1515
HOME = Vector((-0.010, 0.0, 0.0))
START = Vector((0.62, 0.30, 0.0))
END = 1260

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = 24; scene.frame_start, scene.frame_end = 1, END
scene.render.resolution_x, scene.render.resolution_y = RES
for engine in ('BLENDER_EEVEE', 'BLENDER_EEVEE_NEXT'):
    try:
        scene.render.engine = engine; break
    except TypeError:
        pass
scene.view_settings.view_transform = 'AgX'
world = bpy.data.worlds.new('w'); scene.world = world; world.use_nodes = True
world.node_tree.nodes['Background'].inputs[0].default_value = (0.80, 0.82, 0.86, 1)
world.node_tree.nodes['Background'].inputs[1].default_value = 0.55
bpy.ops.wm.obj_import(filepath='/tmp/foldviz/station_split.obj', forward_axis='Y', up_axis='Z')
objs = {o.name: o for o in bpy.context.selected_objects}


def mat(name, color, emit=0.0, alpha=1.0, rough=0.6):
    m = bpy.data.materials.new(name); m.use_nodes = True
    p = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    p.inputs['Base Color'].default_value = (*color, 1); p.inputs['Roughness'].default_value = rough
    if emit:
        p.inputs['Emission Color'].default_value = (*color, 1); p.inputs['Emission Strength'].default_value = emit
    if alpha < 1:
        p.inputs['Alpha'].default_value = alpha
        try:
            m.surface_render_method = 'BLENDED'
        except AttributeError:
            m.blend_method = 'BLEND'
    return m


def recolor(obj, m):
    obj.data.materials.clear(); obj.data.materials.append(m)


def key_scale(o, pairs, constant=False):
    for f, s in pairs:
        o.scale = (s, s, s); o.keyframe_insert('scale', frame=f)
    if constant and o.animation_data:
        act = o.animation_data.action
        for fc in getattr(act, 'fcurves', []):
            for k in fc.keyframe_points:
                k.interpolation = 'CONSTANT'


def pop(o, f_in, f_out=None, dur=10):
    key_scale(o, ((1, 0), (f_in, 0), (f_in + dur, 1)) + (((f_out, 1), (f_out + dur, 0)) if f_out else ()))


def box(center, size, m, name):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center); o = bpy.context.object; o.name = name
    o.scale = size; bpy.ops.object.transform_apply(scale=True); recolor(o, m); return o


def cylinder(a, b, r, m, name):
    a, b = Vector(a), Vector(b)
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=(b - a).length, location=(a + b) / 2, vertices=20)
    o = bpy.context.object; o.name = name
    o.rotation_mode = 'QUATERNION'; o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(b - a)
    recolor(o, m); return o


def cone(tip, direction, m, name, r=0.008, h=0.018):
    d = Vector(direction).normalized()
    bpy.ops.mesh.primitive_cone_add(radius1=r, radius2=0, depth=h, location=Vector(tip) - d * h / 2, vertices=20)
    o = bpy.context.object; o.name = name
    o.rotation_mode = 'QUATERNION'; o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(d)
    recolor(o, m); return o


def group(name, parts, pivot, parent=None):
    e = bpy.data.objects.new(name, None); bpy.context.collection.objects.link(e); e.location = pivot
    bpy.context.view_layer.update()
    for p in parts:
        p.parent = e; p.matrix_parent_inverse = e.matrix_world.inverted()
    if parent is not None:
        e.parent = parent; e.matrix_parent_inverse = parent.matrix_world.inverted()
    return e


def arrow(a, b, m, name, r=0.003):
    a, b = Vector(a), Vector(b); d = (b - a).normalized()
    return group(name, [cylinder(a + d * .017, b - d * .017, r, m, name + '_s'), cone(b, d, m, name + '_1'),
                        cone(a, -d, m, name + '_2')], (a + b) / 2)


def dashed(a, b, m, name, dash=.014, gap=.009, r=.0028):
    a, b = Vector(a), Vector(b); n = (b - a).length; d = (b - a).normalized(); parts = []; s = 0
    while s < n:
        parts.append(cylinder(a + d * s, a + d * min(s + dash, n), r, m, f'{name}_{len(parts)}')); s += dash + gap
    return group(name, parts, (a + b) / 2)


ARM = mat('robot', (0.035, 0.035, 0.04), rough=0.4)
CARTM = mat('cart', (0.22, 0.22, 0.24), rough=0.5)
for name, m in (('cart', CARTM), ('head_pan', CARTM), ('head_tilt', CARTM), ('left_base', ARM), ('right_base', ARM),
                ('table', mat('table', (0.62, 0.48, 0.30), rough=0.7))):
    recolor(objs[name], m)
for n in ('left_arm', 'right_arm'):              # replaced by the posable arms below
    bpy.data.objects.remove(objs[n], do_unlink=True)

# ---- posable arms: shape keys through 13 poses, rest (0) -> training start (12)
POSES = 12
pose_objs = {'left_arm': [], 'right_arm': []}
for k in range(POSES + 1):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.obj_import(filepath=f'/tmp/foldviz/arms_pose_{k:02d}.obj', forward_axis='Y', up_axis='Z')
    for o in bpy.context.selected_objects:
        pose_objs[o.name.split('.')[0]].append(o)
ARMS = {}
for side, lst in pose_objs.items():
    base = lst[0]; recolor(base, ARM)
    bpy.ops.object.select_all(action='DESELECT')
    for o in lst[1:]:
        o.select_set(True)
    base.select_set(True); bpy.context.view_layer.objects.active = base
    bpy.ops.object.join_shapes()
    for o in lst[1:]:
        bpy.data.objects.remove(o, do_unlink=True)
    keys = base.data.shape_keys.key_blocks
    T = [1150 + 7 * k for k in range(POSES + 1)]          # rest at 1150 .. start pose at 1234
    for k in range(1, POSES + 1):
        kb = keys[k]
        for f, v in ((1, 1 if k == POSES else 0), (1145, 1 if k == POSES else 0), (1146, 0), (T[k - 1], 0), (T[k], 1),
                     (T[k + 1] if k < POSES else END, 0 if k < POSES else 1)):
            kb.value = v; kb.keyframe_insert('value', frame=f)
    ARMS[side] = base

# ---- head: pan link fixed, tilt link turns about its hinge (0 = level, 58 deg = training)
tilt = objs['head_tilt']; ht = info['head_tilt_joint']
scene.cursor.location = ht['anchor']
bpy.ops.object.select_all(action='DESELECT'); tilt.select_set(True); bpy.context.view_layer.objects.active = tilt
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
tilt.rotation_mode = 'AXIS_ANGLE'; tilt.rotation_axis_angle = (0, *ht['axis'])
for f, deg in ((1, 58), (900, 58), (901, 0), (930, 0), (985, 58), (END, 58)):
    tilt.rotation_axis_angle[0] = math.radians(deg); tilt.keyframe_insert('rotation_axis_angle', index=0, frame=f)
for fc in getattr(tilt.animation_data.action, 'fcurves', []):
    for k in fc.keyframe_points:
        if int(k.co[0]) in (900, 901):
            k.interpolation = 'CONSTANT'
# view cone of the head camera (OAK-D Lite 4:3: 69 x 54 deg), built level along +y and carried by the tilt link
eye = Vector(ht['anchor']) + Vector((0, 0.025, 0.03))
dist = 0.75
hw, hh = math.tan(math.radians(34.5)) * dist, math.tan(math.radians(27)) * dist
verts = [eye] + [eye + Vector((sx * hw, dist, sz * hh)) for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
me = bpy.data.meshes.new('cone'); me.from_pydata([tuple(v) for v in verts], [], [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)])
frus = bpy.data.objects.new('view_cone', me); bpy.context.collection.objects.link(frus)
recolor(frus, mat('cone', (1.0, 0.55, 0.1), emit=0.6, alpha=0.18))
frus.parent = tilt; frus.matrix_parent_inverse = tilt.matrix_world.inverted()
pop(frus, 935, 1015)

# ---- carton rig: box + flaps (hinged) + tags (each pops on in the tags segment)
RIG = bpy.data.objects.new('carton_rig', None); bpy.context.collection.objects.link(RIG)
bpy.context.view_layer.update()
FLAPS = ('short_left', 'short_right', 'long_far', 'long_near')
flap = {}
for f in FLAPS:
    o = objs['flap_' + f]; h = info[f + '_hinge']
    scene.cursor.location = h['anchor']
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    o.rotation_mode = 'AXIS_ANGLE'; o.rotation_axis_angle = (0, *h['axis'])
    o.parent = RIG; flap[f] = o
objs['carton'].parent = RIG
TAG_POP = {'box_tag_near_left': 740, 'box_tag': 748, 'box_tag_near_right': 756, 'box_tag_left': 785, 'box_tag_left_near': 793,
           'box_tag_right': 815, 'box_tag_floor_center': 838, 'box_tag_floor': 846, 'short_left_tag': 862,
           'short_right_tag': 868, 'long_far_tag': 874, 'long_near_tag': 880}
tags = {}
for name, owner in info['tags'].items():
    o = objs['tag_' + name]
    # pivot at the tag's own centre so it pops in place
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    o.parent = RIG if owner == 'carton' else flap[owner[5:]]
    o.matrix_parent_inverse = o.parent.matrix_world.inverted()
    key_scale(o, ((1, 0), (TAG_POP[name], 0), (TAG_POP[name] + 8, 1.25), (TAG_POP[name] + 12, 1), (1140, 1), (1141, 0),
                  (1146, 1), (END, 1)))
    tags[name] = o
# flaps: up except while being opened/raised in step 4
for f, o in flap.items():
    for fr, deg in ((1, 0), (300, 0), (301, -95), (380, -95), (450, 0), (END, 0)):
        o.rotation_axis_angle[0] = math.radians(deg); o.keyframe_insert('rotation_axis_angle', index=0, frame=fr)
# carton: absent in step 3 and 6a; beside -> square -> into its spot in step 4; in its spot after
for fr, loc, yaw, s in ((1, START, 90, 0), (300, START, 90, 0), (301, START, 90, 1), (465, START, 90, 1),
                        (515, Vector((HOME.x, 0.32, 0)), 0, 1), (570, Vector((HOME.x, 0.32, 0)), 0, 1), (620, HOME, 0, 1),
                        (1020, HOME, 0, 1), (1021, HOME, 0, 0), (1140, HOME, 0, 0), (1141, HOME, 0, 1), (END, HOME, 0, 1)):
    RIG.location = loc; RIG.rotation_euler = (0, 0, math.radians(yaw)); RIG.scale = (s, s, s)
    for path in ('location', 'rotation_euler', 'scale'):
        RIG.keyframe_insert(path, frame=fr)
for fc in getattr(RIG.animation_data.action, 'fcurves', []):
    for k in fc.keyframe_points:
        if int(k.co[0]) in (300, 1020, 1140):
            k.interpolation = 'CONSTANT'

# ---- step 3: the cart squared to the table edge (111 mm each side), then the taped footprint
PINK, BLUE, ORANGE, GREEN = (0.86, 0.10, 0.42), (0.05, 0.40, 0.85), (0.95, 0.45, 0.05), (0.15, 0.70, 0.35)
PINK_M, BLUE_M, ORANGE_M, GREEN_M = (mat(n, c, emit=1.1) for n, c in (('pink', PINK), ('blue', BLUE), ('orange', ORANGE), ('green', GREEN)))
ROBOT = bpy.data.objects.new('robot', None); bpy.context.collection.objects.link(ROBOT)
ROBOT.location = (0, Y0, 0); bpy.context.view_layer.update()
for o in [objs['cart'], objs['head_pan'], tilt, objs['left_base'], objs['right_base'], ARMS['left_arm'], ARMS['right_arm']]:
    o.parent = ROBOT; o.matrix_parent_inverse = ROBOT.matrix_world.inverted()
for side, a in (('l', LP), ('r', RP)):
    ax = dashed((a.x, a.y, 0.004), (a.x, a.y, BASE_Z + 0.30), PINK_M, f'axis3_{side}'); ax.parent = ROBOT
    ax.matrix_parent_inverse = ROBOT.matrix_world.inverted(); pop(ax, 40, 228)
    ar = arrow((a.x, a.y, BASE_Z + 0.005), (a.x, EDGE_Y, BASE_Z + 0.005), PINK_M, f'set_{side}', r=0.0045); ar.parent = ROBOT
    ar.matrix_parent_inverse = ROBOT.matrix_world.inverted(); pop(ar, 55, 228)
    drop = dashed((a.x, EDGE_Y, 0.002), (a.x, EDGE_Y, BASE_Z + 0.005), ORANGE_M, f'drop_{side}', r=.002); pop(drop, 50, 228)
for f, deg in ((1, 0), (120, 0), (140, 5), (180, 5), (200, 0), (END, 0)):
    ROBOT.rotation_euler = (0, 0, math.radians(deg)); ROBOT.keyframe_insert('rotation_euler', frame=f)
key_scale(ROBOT, ((1, 1), (720, 1), (721, 0), (900, 0), (901, 1), (END, 1)), constant=False)
for fc in getattr(ROBOT.animation_data.action, 'fcurves', []):
    for k in fc.keyframe_points:
        if int(k.co[0]) in (720, 900):
            k.interpolation = 'CONSTANT'
edge = box((0, EDGE_Y + 0.002, .0008), (1.1, 0.004, .0016), ORANGE_M, 'table_edge'); pop(edge, 50)
TAPE = mat('tape', (0.10, 0.45, 0.90), emit=0.25)
t, g = 0.018, 0.003
x0, x1, y0, y1 = HOME.x - L / 2 - g, HOME.x + L / 2 + g, HOME.y - W / 2 - g, HOME.y + W / 2 + g
ghost = box((HOME.x, HOME.y, H / 2), (L, W, H), mat('ghost', GREEN, emit=0.3, alpha=0.18), 'ghost'); pop(ghost, 232, 296)
tape = group('tape', [box(((x0 + x1) / 2, y0 - t / 2, .0006), (x1 - x0 + 2 * t, t, .0012), TAPE, 'tape_n'),
                      box(((x0 + x1) / 2, y1 + t / 2, .0006), (x1 - x0 + 2 * t, t, .0012), TAPE, 'tape_f'),
                      box((x0 - t / 2, (y0 + y1) / 2, .0006), (t, y1 - y0, .0012), TAPE, 'tape_l'),
                      box((x1 + t / 2, (y0 + y1) / 2, .0006), (t, y1 - y0, .0012), TAPE, 'tape_r')], (HOME.x, HOME.y, 0))
key_scale(tape, ((1, 0), (262, 0), (272, 1), (1020, 1), (1021, 0), (1140, 0), (1141, 1), (END, 1)))
# centre lines above the carton rim: midway between the arms (pink) and the carton centre (green)
mid = dashed((0, -0.30, H + 0.03), (0, 0.18, H + 0.03), PINK_M, 'midline', r=.002); pop(mid, 238, 296)
cen = cylinder((HOME.x, -0.20, H + 0.031), (HOME.x, 0.16, H + 0.031), .002, GREEN_M, 'cline'); pop(cen, 244, 296)

# ---- step 6a: floor tags 40-43 (80 mm black square, 100 mm with border), "top edge" away from the robot
for tid, (sx, out) in {40: (-0.12, 0.25), 41: (0.12, 0.25), 42: (-0.12, 0.40), 43: (0.12, 0.40)}.items():
    bpy.ops.mesh.primitive_plane_add(size=0.1, location=(sx, Y0 + out, 0.0008)); p = bpy.context.object; p.name = f'floor{tid}'
    m = bpy.data.materials.new(f'tag{tid}'); m.use_nodes = True
    tex = m.node_tree.nodes.new('ShaderNodeTexImage'); tex.image = bpy.data.images.load(f'/tmp/foldviz/tags/tag{tid}.png')
    tex.interpolation = 'Closest'
    bsdf = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    p.data.materials.append(m)
    key_scale(p, ((1, 0), (1035 + 8 * (tid - 40), 0), (1045 + 8 * (tid - 40), 1), (1140, 1), (1141, 0), (END, 0)))

# ---- camera
CAM_DATA = bpy.data.cameras.new('cam'); CAM_DATA.lens = 32
CAM = bpy.data.objects.new('cam', CAM_DATA); bpy.context.collection.objects.link(CAM); scene.camera = CAM
TARGET = bpy.data.objects.new('target', None); bpy.context.collection.objects.link(TARGET)
c = CAM.constraints.new('TRACK_TO'); c.target = TARGET; c.track_axis = 'TRACK_NEGATIVE_Z'; c.up_axis = 'UP_Y'
SHOTS = [
    # step 3
    (1, (0.95, 0.75, 0.75), (0, -0.20, 0.05)), (40, (0.62, -0.30, 0.42), (0.02, -0.21, 0.09)),
    (228, (0.62, -0.30, 0.42), (0.02, -0.21, 0.09)), (240, (-0.01, -0.55, 1.05), (-0.01, -0.02, 0.0)),
    (300, (-0.01, -0.55, 1.05), (-0.01, -0.02, 0.0)),
    # step 4 (from the old clip, shifted by 300)
    (301, (1.25, 0.95, 0.55), (0.55, 0.28, 0.10)), (450, (1.15, 0.90, 0.50), (0.55, 0.28, 0.12)),
    (485, (0.15, -1.00, 0.85), (0.05, 0.15, 0.08)), (600, (0.05, -0.95, 0.85), (0.0, 0.05, 0.06)),
    (640, (0.42, -0.45, 0.22), (0.17, -0.15, 0.03)), (680, (0.42, -0.45, 0.22), (0.17, -0.15, 0.03)),
    (705, (-1.05, 0.95, 0.70), (0, -0.10, 0.10)), (720, (-1.05, 0.95, 0.70), (0, -0.10, 0.10)),
    # tags, face by face
    (721, (0.0, -0.78, 0.22), (0.0, -0.145, 0.06)), (770, (0.0, -0.78, 0.22), (0.0, -0.145, 0.06)),
    (782, (-0.80, 0.10, 0.30), (-0.19, 0, 0.05)), (800, (-0.80, 0.10, 0.30), (-0.19, 0, 0.05)),
    (812, (0.80, 0.05, 0.30), (0.19, 0, 0.05)), (822, (0.80, 0.05, 0.30), (0.19, 0, 0.05)),
    (832, (-0.01, -0.12, 0.95), (-0.01, 0.0, 0.0)), (850, (-0.01, -0.12, 0.95), (-0.01, 0.0, 0.0)),
    (860, (0.85, 0.85, 0.55), (0, 0, 0.15)), (900, (-0.85, 0.80, 0.55), (0, 0, 0.15)),
    # step 5: head tilt from the side, then the cone onto the carton
    (901, (0.95, -0.55, 0.55), (0, -0.15, 0.25)), (985, (0.95, -0.55, 0.55), (0, -0.15, 0.25)),
    (1020, (1.05, 0.25, 0.75), (0, -0.10, 0.12)),
    # step 6a: floor tags from behind and above the robot
    (1021, (0.0, -0.40, 1.15), (0, 0.03, 0.0)), (1140, (0.0, -0.40, 1.15), (0, 0.03, 0.0)),
    # step 8b: the arms to the start pose
    (1141, (0.70, -0.75, 0.55), (0, -0.25, 0.18)), (1260, (-0.70, -0.70, 0.55), (0, -0.25, 0.18)),
]
for fr, loc, look in SHOTS:
    CAM.location = loc; CAM.keyframe_insert('location', frame=fr)
    TARGET.location = look; TARGET.keyframe_insert('location', frame=fr)
for o in (CAM, TARGET):
    for fc in getattr(o.animation_data.action, 'fcurves', []):
        for k in fc.keyframe_points:
            if int(k.co[0]) in (300, 720, 900, 1020, 1140):
                k.interpolation = 'CONSTANT'        # hard cut between segments

sun = bpy.data.objects.new('sun', bpy.data.lights.new('sun', 'SUN')); bpy.context.collection.objects.link(sun)
sun.data.energy = 4.0; sun.rotation_euler = (math.radians(35), math.radians(10), math.radians(-35))
fill = bpy.data.objects.new('fill', bpy.data.lights.new('fill', 'AREA')); bpy.context.collection.objects.link(fill)
fill.data.energy = 150; fill.data.size = 2.5; fill.location = (0.4, -1.4, 1.2)
fill.rotation_euler = (math.radians(55), 0, math.radians(-10))

A = {   # label anchors: (object or None, local/world point)
    'axl': (ROBOT, (LP.x, LP.y - Y0, BASE_Z + 0.30)), 'axr': (ROBOT, (RP.x, RP.y - Y0, BASE_Z + 0.30)),
    'setl': (ROBOT, (LP.x, (LP.y + EDGE_Y) / 2 - Y0, 0.004)), 'setr': (ROBOT, (RP.x, (RP.y + EDGE_Y) / 2 - Y0, 0.004)),
    'edge': (None, (0.32, EDGE_Y, 0.0)), 'tape': (None, (x1 + t / 2, 0.08, 0.0)), 'gap': (None, (0.16, EDGE_Y + 0.005, 0.0)),
    'mid': (None, (0, 0.15, H + 0.03)), 'cen': (None, (HOME.x, -0.18, H + 0.031)), 'ghost': (None, (HOME.x, 0.10, H)),
    'len': (RIG, (0, -W / 2, H + 0.005)), 'wid': (RIG, (L / 2, 0, H + 0.005)), 'hgt': (RIG, (L / 2, -W / 2, H / 2)),
    'sl': (flap['short_left'], (0, 0, 0.10)), 'sr': (flap['short_right'], (0, 0, 0.10)),
    'ln': (flap['long_near'], (0, 0, 0.12)), 'lf': (flap['long_far'], (0, 0, 0.12)),
    'larm': (None, (LP.x, LP.y, 0.33)), 'rarm': (None, (RP.x, RP.y, 0.33)),
    'head': (tilt, (0, 0.03, 0.02)), 'conefloor': (None, (0.0, 0.05, H)),
    **{f't{tid}': (None, (sx, Y0 + out, 0.0)) for tid, (sx, out) in {40: (-0.12, 0.25), 41: (0.12, 0.25), 42: (-0.12, 0.40), 43: (0.12, 0.40)}.items()},
    'baseline': (None, (0.0, Y0, 0.0)),
    **{'tag_' + n: (o, (0, 0, 0)) for n, o in tags.items()},
    'farL': (RIG, (-L / 2 + 0.004, W / 2, H + 0.145)), 'farR': (RIG, (L / 2 - 0.004, W / 2, H + 0.145)),
}


def world(o, p):
    return (o.matrix_world @ Vector(p)) if o else Vector(p)


bpy.ops.wm.save_as_mainfile(filepath='/tmp/foldviz/scene_steps.blend')
OUT.mkdir(parents=True, exist_ok=True)
scene.render.image_settings.file_format = 'PNG'
anchors = {}
for f in (FRAMES or range(1, END + 1)):
    scene.frame_set(f)
    anchors[f] = {k: list(world_to_camera_view(scene, CAM, world(o, p)))[:2] for k, (o, p) in A.items()}
    if '--anchors-only' in argv:
        continue
    scene.render.filepath = str(OUT / f'f{f:04d}.png')
    bpy.ops.render.render(write_still=True)
json.dump(anchors, open(OUT / 'anchors.json', 'w'))
print('rendered', len(anchors))
