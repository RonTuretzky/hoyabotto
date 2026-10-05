"""Blender render from the pinned SO-101 URDF/STLs, with a proposed marker mount.

Run in a separate Blender background process; never modifies the original scenes.
Blender --background --factory-startup --python THIS.py -- --out DIR [--draft]
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view

ROOT = Path(__file__).resolve().parents[1]
args = argparse.ArgumentParser()
args.add_argument('--out', type=Path, required=True)
args.add_argument('--draft', action='store_true')
args.add_argument('--model-dir', type=Path, default=ROOT/'software/data-carton/models/so101')
args.add_argument('--tag-kit', type=Path, default=ROOT/'blender/carton-marker-grids.json')
args = args.parse_args(sys.argv[sys.argv.index('--')+1:])
OUT = args.out.resolve(); OUT.mkdir(parents=True, exist_ok=True)
MODEL = args.model_dir.resolve()
URDF = ET.parse(MODEL / 'so101_new_calib.urdf').getroot()

# This process was explicitly started with factory-startup; user scenes stay intact.
for obj in list(bpy.data.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
scene = bpy.context.scene
scene.name = 'Actual SO101 gripper - tag mounting proposal'

def choose(owner, prop, value):
    valid = [i.identifier for i in owner.bl_rna.properties[prop].enum_items]
    if value not in valid:
        raise ValueError((prop, value, valid))
    setattr(owner, prop, value)

def material(name, color, metal=0):
    m = bpy.data.materials.new(name); m.use_nodes = True
    p = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = .47
    return m

body = material('Original printed gripper - illustration finish', (.22, .29, .33))
dark = material('Original servo casing', (.025, .035, .045), .15)
jaw = material('Original moving jaw - illustration finish', (.11, .15, .18))
white = material('Matte tag white', (.98, .98, .98))
ink = material('Tag black', (.003, .003, .003))
accent = material('Proposed backing - blue', (.025, .29, .55))

def transform(node):
    origin = node.find('origin')
    if origin is None: return Matrix.Identity(4)
    xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
    rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
    return Matrix.Translation(xyz) @ Euler(rpy, 'XYZ').to_matrix().to_4x4()

joint = {j.get('name'): j for j in URDF.findall('joint')}
# Keep the gripper's physical dimensions and original assembly transforms.
# Rotate the whole assembly for an explanatory view: jaws point along world +X.
display = Matrix.Rotation(-math.pi/2, 4, 'Y')
poses = {'gripper_link': display,
         'wrist_link': display @ transform(joint['wrist_roll']).inverted(),
         'moving_jaw_so101_v1_link': display @ transform(joint['gripper']) @ Matrix.Rotation(.55, 4, 'Z')}
parts = []
for link in URDF.findall('link'):
    name = link.get('name')
    if name not in poses: continue
    for index, visual in enumerate(link.findall('visual')):
        mesh = visual.find('geometry/mesh')
        if mesh is None: continue
        path = MODEL / mesh.get('filename')
        bpy.ops.wm.stl_import(filepath=str(path))
        obj = bpy.context.object
        obj.name = f'ACTUAL | {name} | {path.stem}'
        obj.matrix_world = poses[name] @ transform(visual)
        obj.data.materials.clear()
        mat = dark if visual.find('material').get('name') == 'sts3215' else (jaw if name.startswith('moving_jaw') else body)
        obj.data.materials.append(mat)
        obj['source_stl'] = str(path.relative_to(MODEL))
        obj['source_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        obj['geometry_kind'] = 'unmodified upstream mesh with URDF transform'
        parts.append(obj)
bpy.context.view_layer.update()
bounds = []
for obj in parts:
    points = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    bounds.append({'name': obj.name, 'min_mm': [round(min(p[i] for p in points)*1000, 3) for i in range(3)],
                   'max_mm': [round(max(p[i] for p in points)*1000, 3) for i in range(3)]})
print('ACTUAL_BOUNDS', json.dumps(bounds), flush=True)

world = bpy.data.worlds.new('Studio world'); world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == 'BACKGROUND')
bg.inputs['Color'].default_value = (.95, .96, .97, 1)
bg.inputs['Strength'].default_value = 2
scene.world = world
for loc, energy, size in [((.15, -.2, .30), 1.5, .25), ((-.12, .15, .12), 1, .20), ((.18, .18, .1), .7, .2)]:
    light = bpy.data.lights.new('Softbox', 'AREA')
    light.energy = energy; light.size = size
    ob = bpy.data.objects.new('Softbox', light); scene.collection.objects.link(ob); ob.location = loc
    ob.rotation_euler = (Vector((.035, 0, 0))-ob.location).to_track_quat('-Z', 'Y').to_euler()
cam = bpy.data.cameras.new('Presentation camera'); choose(cam, 'type', 'ORTHO')
cam.clip_start = .001; cam.clip_end = 10; cam.ortho_scale = .23
camera = bpy.data.objects.new('Presentation camera', cam); scene.collection.objects.link(camera); scene.camera = camera
scene.render.resolution_x = 1400; scene.render.resolution_y = 1000; scene.render.resolution_percentage = 100
choose(scene.render.image_settings, 'file_format', 'PNG')
scene.render.film_transparent = False
print('RENDER_ENGINE', scene.render.engine, flush=True)
def render(name, position, target=(.035, 0, -.004)):
    camera.location = position
    camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.filepath = str(OUT / (name+'.png'))
    bpy.ops.render.render(write_still=True)

if args.draft:
    render('bare-side', (.07, -.30, .08))
    render('bare-other-side', (.07, .30, .08))
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'gripper-inspection.blend'), compress=True)
    (OUT/'geometry-bounds.json').write_text(json.dumps(bounds, indent=2))
else:
    def box(name, center, dims, mat):
        sx, sy, sz = [d/2 for d in dims]
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata([(-sx,-sy,-sz),(sx,-sy,-sz),(sx,sy,-sz),(-sx,sy,-sz),
                          (-sx,-sy,sz),(sx,-sy,sz),(sx,sy,sz),(-sx,sy,sz)], [],
                         [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
        mesh.update()
        obj = bpy.data.objects.new(name, mesh); scene.collection.objects.link(obj)
        obj.location = center; obj.data.materials.append(mat)
        return obj

    # Proposed removable backing, deliberately separate from the existing CAD.
    # Its rear supports land on the y=-24 mm fixed-body band below the horn.
    cx, cy, cz = .031, -.032, -.012
    board = box('PROPOSED | rigid backing 52 x 52 x 1.5 mm', (cx, cy, cz), (.052, .0015, .052), accent)
    mounts = [box('PROPOSED | spacer on fixed housing', (x, -.027625, -.008), (.004, .00725, .006), accent)
              for x in (.015, .024)]
    card = box('PRINTED | 50 mm card with white quiet zone', (cx, cy-.00086, cz), (.050, .0002, .050), white)
    # Use the already detector-verified grid; no decorative or invented tag pattern.
    kit = json.loads(args.tag_kit.read_text())
    tag = next(r for r in kit['markers'] if r['tag_id'] == 2)
    cells = []
    for y, row in enumerate(tag['grid_black_is_1']):
        for x, bit in enumerate(row):
            if bit == '1':
                cells.append(box('PRINTED | tag36h11 ID 2 cell',
                                 (cx+(x-3.5)*.005, cy-.000975, cz+(3.5-y)*.005),
                                 (.005, .00002, .005), ink))
    mount_group = [board, *mounts, card, *cells]
    for ob in mount_group:
        ob['geometry_kind'] = 'proposed visualization only - not a validated bracket'

    def tree(obj):
        return BVHTree.FromPolygons([obj.matrix_world@v.co for v in obj.data.vertices],
                                   [p.vertices[:] for p in obj.data.polygons])
    moving = next(o for o in parts if 'moving_jaw_so101_v1_link' in o.name)
    saved = moving.matrix_world.copy()
    jaw_visual = URDF.find("link[@name='moving_jaw_so101_v1_link']/visual")
    bpy.context.view_layer.update()
    mount_trees = [(o.name, tree(o)) for o in [board, *mounts]]
    overlaps = []
    angles = [i*5 for i in range(-2,21)]
    for deg in angles:
        moving.matrix_world = display @ transform(joint['gripper']) @ Matrix.Rotation(math.radians(deg),4,'Z') @ transform(jaw_visual)
        bpy.context.view_layer.update()
        t = tree(moving)
        for name, mt in mount_trees:
            if mt.overlap(t): overlaps.append({'jaw_deg':deg,'object':name})
    moving.matrix_world = saved; bpy.context.view_layer.update()

    # Flat, camera-facing labels stay readable in both views; words carry meaning
    # independently of the blue used to distinguish added geometry.
    labelmat = bpy.data.materials.new('Unlit dark annotation'); labelmat.use_nodes = True
    nodes = labelmat.node_tree.nodes; nodes.clear()
    emission = nodes.new('ShaderNodeEmission'); emission.inputs[0].default_value = (.025,.035,.045,1)
    output = nodes.new('ShaderNodeOutputMaterial'); labelmat.node_tree.links.new(emission.outputs[0],output.inputs[0])
    annotations = []
    def screen_point(u,v):
        width = cam.ortho_scale; height = width*scene.render.resolution_y/scene.render.resolution_x
        return camera.matrix_world @ Vector(((u-.5)*width,(v-.5)*height,-.012))
    def text(body,u,v,size):
        curve=bpy.data.curves.new(body,'FONT');curve.body=body;curve.size=size*cam.ortho_scale
        ob=bpy.data.objects.new('LABEL | '+body,curve);scene.collection.objects.link(ob)
        ob.location=screen_point(u,v);ob.rotation_euler=camera.rotation_euler;ob.data.materials.append(labelmat)
        annotations.append(ob)
    def leader(anchor,u,v):
        uv=world_to_camera_view(scene,camera,Vector(anchor))
        curve=bpy.data.curves.new('Leader','CURVE');curve.dimensions='3D';curve.bevel_depth=cam.ortho_scale*.0006
        s=curve.splines.new('POLY');s.points.add(2)
        for p,co in zip(s.points,[screen_point(uv.x,uv.y),screen_point(u-.012,v),screen_point(u+.003,v)]):p.co=(*co,1)
        ob=bpy.data.objects.new('ANNOTATION | leader',curve);scene.collection.objects.link(ob);ob.data.materials.append(labelmat);annotations.append(ob)
    def clear_labels():
        for ob in annotations:bpy.data.objects.remove(ob,do_unlink=True)
        annotations.clear()
    def compose(name, position, exploded=False):
        clear_labels()
        camera.location=position;camera.rotation_euler=(Vector((.020,-.005,.010))-camera.location).to_track_quat('-Z','Y').to_euler()
        cam.ortho_scale=.285;scene.render.resolution_x=1920;scene.render.resolution_y=1280
        bpy.context.view_layer.update()
        text('TAG 2 / FIXED GRIPPER BODY',.055,.928,.029)
        text('Actual SO-101 meshes  |  Proposed removable marker backing',.055,.884,.014)
        text('Camera mount and cables are not in this model. Visibility and full-arm clearance need a physical check.',.055,.060,.011)
        text('40 mm black square + white margin = 50 mm printed card.  Backing shown: 52 mm square.',.055,.033,.011)
        if exploded:
            text('ATTACH TO THIS FIXED BAND',.59,.20,.017)
            text('Below the pivot; keep the horn free.',.59,.166,.012)
            leader((.0195,-.024,-.008),.59,.23)
            text('Backing shown separated',.055,.76,.015)
            text('to reveal the attachment area.',.055,.726,.012)
        else:
            text('TAG ON FLAT BACKING',.055,.285,.018)
            text('Fixed to the housing below the pivot.',.055,.249,.012)
            leader((cx-.026,cy-.001,cz),.055,.315)
            text('MOVING JAW / LEAVE CLEAR',.59,.77,.017)
            leader((.077,-.018,.059),.59,.747)
            text('GRIP AREA / LEAVE CLEAR',.68,.19,.016)
            leader((.094,-.011,-.022),.68,.23)
        scene.render.filepath=str(OUT/(name+'.png'))
        bpy.ops.render.render(write_still=True)

    # Explode only the card/backing, leaving the support locations attached to CAD.
    explode = Vector((-.064,-.045,.010))
    for ob in [board,card,*cells]:ob.location+=explode
    compose('gripper-tag-attachment',(.18,-.31,.15),exploded=True)
    for ob in [board,card,*cells]:ob.location-=explode
    compose('gripper-tag-installed',(.125,-.31,.13))
    scene['note']='Original SO-101 geometry; proposed backing. No physical camera pose, cable model or full collision certification.'
    scene['backing_center_m']=list((cx,cy,cz))
    scene['original_gripper_to_display_matrix']=json.dumps([list(r) for r in display])
    attribution = bpy.data.texts.new('SOURCE_AND_LIMITATIONS.txt')
    attribution.write('SO-101 meshes: TheRobotStudio/SO-ARM100 at 5f6d2b876a53a4872e405b991dd925556c9e38a4.\n'
                      'https://github.com/TheRobotStudio/SO-ARM100\n'
                      'Marker pattern: OpenCV DICT_APRILTAG_36h11 ID 2, decoded with pupil-apriltags.\n'
                      'Added backing/spacers are an unvalidated placement illustration.\n\n'+(MODEL/'LICENSE').read_text())
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                choose(area.spaces.active.region_3d,'view_perspective','CAMERA')
    scene.render.filepath='//gripper-tag-installed.png'
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'gripper-apriltag-placement.blend'),compress=True)
    report={'source_repository':'TheRobotStudio/SO-ARM100','source_revision':'5f6d2b876a53a4872e405b991dd925556c9e38a4',
            'blender_version':bpy.app.version_string,'actual_geometry':bounds,
            'backing_center_display_frame_m':[cx,cy,cz], 'backing_dimensions_mm':[52,1.5,52],
            'attachment_pad_centers_display_frame_m':[[.015,-.024,-.008],[.024,-.024,-.008]],
            'tag_id':2,'family':'tag36h11','black_square_mm':40,'paper_mm':50,
            'sampled_jaw_angles_deg':angles,'sampled_mesh_surface_intersections':overlaps,
            'limitations':['Surface intersection samples are not a continuous swept-volume or full robot collision test.',
                           'Wrist camera, cables, fasteners and physical mounting are not modeled or validated.',
                           'This is a placement illustration, not a manufacturing-ready bracket.'],
            'physical_validation':False}
    (OUT/'placement-report.json').write_text(json.dumps(report,indent=2))
    print('PLACEMENT_REPORT',json.dumps(report),flush=True)
