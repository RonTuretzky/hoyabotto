"""The box-grab benchmark scene: the twin's XLeRobot plus a table, a cardboard box, a lamp and cameras.

``build_scene_xml`` starts from ``farm.sim.xlerobot_twin._scene_xml`` (upstream MJCF made fixed-base, claw tip
sites, the OAK optical site ``HEAD_SITE`` on ``HEAD_CAMERA_BODY``, floor, lights, offscreen buffer) and adds:

- a table (box geom, 0.60 m deep x 0.90 m wide, top at ``table_top_m``) whose near edge is ``TABLE_NEAR_M`` forward
  of the robot origin; four visual legs;
- a free cardboard box (body ``box``, freejoint ``box_free``, 0.25 kg, friction 1.0, a brown corrugation-striped
  material) resting on the table's near edge. ``preset`` picks the box (``PRESETS``): 'real' (the default) is the
  carton on the robot on 9 October (379 x 283 x 108 mm), an OPEN box (floor and four walls, rim 81 cm) with a 14 cm right flap
  (the target: ``flap_hinge``, bodies ``box_flap*``) whose starting lean is drawn per seed, and a 14 cm far flap
  leaning in; its crease plasticity lives in SimRobot (``PLASTIC``). 'near7' is the 8 October scene described next,
  a closed box with one open carton flap: body ``box_flap`` (geom ``flap``, 7 cm tall,
  3.5 mm thick, the box's full width, 10 g) on hinge joint ``flap_hinge`` along the near top edge (axis along the box
  width). Angle 0 = vertical, positive = folded inward over the box top, 90 = flat on the top (the joint stops at
  ``FLAP_RANGE_DEG``; the flap and the box are parent and child, so MuJoCo does not collide them and the stop is
  the top). It starts ``FLAP_OPEN_DEG`` outward like a real open flap. The crease is modelled as a light spring
  toward that open angle plus a larger hinge friction, so the flap stays where it is put (a folded flap stays
  folded) and starts to fold under about 0.9 N at its edge; its top 2 cm turns freely relative to the rest (see
  ``FLAP_SEGMENTS_M``); ``seed`` jitters the box +-2 cm in forward/left and +-5 deg of yaw;
- a bright lamp on the table behind the box (emissive white sphere on a stand plus a strong spotlight and a weaker
  directional fill, both pointing back at the robot; no shadow maps, which look blocky centimetres from a wrist
  lens), so a wrist camera looking level at the box is backlit;
- a dark floor and a dim sky, like the IKEA-cart setup in the real images;
- MuJoCo cameras: ``oak`` on the head camera body with the OAK optical orientation (``HEAD_SITE_XYAXES`` converted
  from the optical convention: MuJoCo cameras look along their -z with +y up; optical +z out of the lens = camera
  -z, optical +y down = camera -y), fovy from the real OAK intrinsics (fx = fy ~ 505 px at 640x360, 39.2 deg
  vertical / 64.7 deg horizontal); ``left_wrist``/``right_wrist`` on the fixed-jaw bodies (``Fixed_Jaw`` is the
  robot's LEFT arm, ``Fixed_Jaw_2`` the right, as in the twin's ``ARMS``) where the upstream model mounts the wrist
  camera (above the jaw plane, behind the jaw root, looking down the jaws toward the fingertips so both jaw tips
  sit in the lower frame like the real images); ``phone`` fixed at the robot's left rear, 1.2 m up, looking down
  at the box over the left shoulder.

Robot frame (``xlerobot_twin.FRAME``): origin on the floor below the midpoint of the shoulder-pan axes (model
``ROBOT_ORIGIN_MODEL``), +forward = model -x, +left = model -y, +up = +z. ``robot_frame_of_box`` reports where
the box is in that frame; ``colour_arms`` repeats the twin's orange/blue arm colouring on a compiled model.

Rendering only: nothing here talks to motors or the robot API.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from farm.sim import xlerobot_twin as twin

# Robot frame origin stays fixed under the symmetric 273 mm arm-spacing overlay.
ROBOT_ORIGIN_MODEL = np.array([-0.09, 0.0, 0.0])
AXES = np.array([twin.FORWARD, twin.LEFT, twin.UP], dtype=float)  # rows: robot axes in model coordinates

TABLE_NEAR_M = 0.30           # near edge forward of the robot origin
TABLE_SIZE_M = (0.60, 0.90)   # depth (forward), width (left-right)
TABLE_THICKNESS_M = 0.03
FLAP_SEGMENTS_M = (0.05, 0.02)  # bottom up: the flap panel on the crease, then the top strip the pads hold
FLAP_HEIGHT_M = sum(FLAP_SEGMENTS_M)    # 7 cm (preset 'near7'); the real carton's flaps are 16 cm (preset 'real')
FLAP_THICKNESS_M = 0.0035
FLAP_MASS_KG = 0.01
FLAP_FRICTION = 1.2           # sliding friction of the jaw pads on the flap (priority 1: the flap's contact parameters win)
FLAP_TORSION_M = 0.005        # torsional friction (the jaw meshes' own 0.2 m torsion / 0.1 m rolling lock any pinch rigid)
FLAP_OPEN_DEG = -8.0          # rest angle: leaning 8 deg outward (toward the robot), like a real open flap
FLAP_RANGE_DEG = (-60.0, 92.0)
FLAP_STIFFNESS = 0.02         # N m/rad toward FLAP_OPEN_DEG (crease spring)
FLAP_FRICTIONLOSS = 0.06      # N m: about 0.9 N at the free edge starts the fold; more than the spring at 90 deg
                              # (0.034 N m), so a folded flap stays down
FLAP_DAMPING = 0.002          # N m s/rad
FLAP_BEND_STIFFNESS = 0.005   # N m/rad: the top strip is the crushed edge between the pads, nearly free to turn
FLAP_BEND_FRICTIONLOSS = 0.005
FLAP_BEND_DAMPING = 0.001
FLAP_BEND_RANGE_DEG = (-120.0, 120.0)
FLAP_FOLDED_DEG = 75.0        # robot_frame_of_box / SimRobot.score: folded at or past this angle

# Scene presets. 'near7' is the 8 October bench scene (box top 81 cm, one 7 cm near flap that stays where it is put).
# 'real' (the default) is the carton on the robot's table on 9 October: the owner's HACHIYO box, 379 x 283 x 108 mm
# (left-right x forward x height) with 140 mm flaps, so on the 70 cm table the rim is ~81 cm and a standing flap's free
# edge ~95 cm (the morning's depth-camera fit that gave 77 cm was off: its tilt correction swung 27-52 deg). Placement
# (model frame): right wall at about -18 left, left wall at +20, near face at about 21 cm forward (the far rim then at
# ~50); the right flap leaned ~11 deg outward in the morning; the far flap stands leaning in; the near flap is folded
# down and the left flap hangs outward (neither is modelled). The target flap ('flap_hinge', 'box_flap*') is the RIGHT one.
# Its crease springs back: a flap carried to 100 deg and released returned to 5-15 deg from vertical on the robot, and
# 3-6 s holds pressed flat did not set it. SimRobot models that with a moving spring rest angle (PLASTIC below).
# Flap entries: side ('near', 'far', 'left', 'right' face of the box), segments (bottom panel on the crease, top strip
# the pads hold), thickness per segment (the top 2 cm is the crushed edge: 2 mm, so a tip pinch reads about 14 ticks
# above the meeting pads like the real right gripper's 1358-1364), open_deg (rest lean, + = inward), crease spring,
# friction, damping, range, mass; 'target' marks the scored flap.
# The real flaps folded from pinches 2-4 cm deep (9 October): the board bends where it leaves the pads. Segments: a
# 9 cm crease panel, a 3 cm band that bends moderately (the pinch line of a deeper pinch), the 2 cm crushed edge
# (140 mm in all). Range: past flat the free edge dips into the open box (the floor is 10.4 cm below the rim).
REAL_FLAP = {'segments': (0.09, 0.03, 0.02), 'thickness': (0.0035, 0.0035, 0.002), 'bend_stiffness': (0.03, 0.005),
             'bend_frictionloss': (0.004, 0.005), 'stiffness': 0.08, 'frictionloss': 0.008,
             'damping': 0.004, 'range': (-60.0, 125.0), 'mass': 0.012}
PRESETS = {
    'near7': {'box_forward_m': 0.42, 'box_left_m': 0.21, 'box_size_m': (0.20, 0.15, 0.11),
              'phone': (-0.45, 0.55, 1.20), 'plastic': None,
              'flaps': {'near': {'segments': FLAP_SEGMENTS_M, 'thickness': (FLAP_THICKNESS_M, FLAP_THICKNESS_M),
                                 'open_deg': FLAP_OPEN_DEG, 'stiffness': FLAP_STIFFNESS, 'frictionloss': FLAP_FRICTIONLOSS,
                                 'damping': FLAP_DAMPING, 'range': FLAP_RANGE_DEG, 'mass': FLAP_MASS_KG, 'target': True}}},
    'real': {'box_forward_m': 0.355, 'box_left_m': 0.0145, 'box_size_m': (0.283, 0.379, 0.108), 'hollow': True,
             'table_near_m': 0.20,   # the table must reach under the box's near face (~21 cm)
             'lean_jitter_deg': (-12.0, 15.0),
             'phone': (0.05, -0.62, 1.05),   # the real phone stands on the robot's right, looking at the right wall
             'plastic': 'default',
             'flaps': {'right': dict(REAL_FLAP, open_deg=-11.0, target=True),
                       'far': dict(REAL_FLAP, open_deg=17.0, target=False)}},
}
DEFAULT_PRESET = 'real'
# Crease plasticity (SimRobot, 10 Hz): the spring's rest angle theta0 moves only while the crease is loaded, i.e. the
# flap is held |theta - theta0| > yield_deg away from it, continuously for more than delay_s. It then creeps toward
# theta - springback(theta) at rate_per_s: springback is springback_deg up to start_set_deg and falls linearly to 0 at
# full_set_deg (overfolding breaks the crease). Checks against the 9 October robot attempts (rest about -11):
# held flat (86 deg) 6 s -> rest about +10 (robot: 5-15 inward); carried to 100 deg, held 3 s -> about +25 (robot:
# 30-40 inward, with a push sweep); 100 deg for 7 s or 105 deg for 5 s -> about +75, which stays folded.
# A second contact pressing the crease zone (within press_zone_m of the hinge) with >= press_force_n while the flap is
# past press_min_deg for press_s sets the crease almost fully (springback press_springback_deg, rate press_rate_per_s).
# So does another flap folded past cover_deg on top of it (the flaps do not collide in the sim, see build_scene_xml).
PLASTIC = {'yield_deg': 30.0, 'delay_s': 2.0, 'rate_per_s': 0.5, 'springback_deg': 70.0, 'start_set_deg': 85.0,
           'full_set_deg': 105.0, 'press_zone_m': 0.05, 'press_force_n': 0.5, 'press_min_deg': 70.0, 'press_s': 2.0,
           'press_springback_deg': 5.0, 'press_rate_per_s': 1.0, 'cover_deg': 80.0}
BOX_MASS_KG = 0.25
BOX_WALL_M = 0.004
BOX_FRICTION = 1.0
BOX_JITTER_M = 0.02
BOX_JITTER_DEG = 5.0

# Cameras. OAK RGB: the real device reports fx = 504.9, fy = 505.0 at 640x360 (software/farm/sim/assets/sim_robot/
# real_oak_depth_manifest.json), i.e. 64.7 deg horizontal, 39.2 deg vertical. MuJoCo takes the vertical fov.
OAK_SIZE = (640, 360)
OAK_FOCAL_PX = 505.0
OAK_FOVY_DEG = math.degrees(2.0 * math.atan(OAK_SIZE[1] / 2.0 / OAK_FOCAL_PX))
STOCK_HEAD_CAMERA_MESH = 'tophead6'   # the kit's USB head camera on the tilt link (removed from the robot 2026-10-09)
OAK_CAMERA_XYAXES = '0 -1 0 0 0 1'   # HEAD_SITE_XYAXES '0 -1 0 0 0 -1' with the y axis flipped (image up, not down)
WRIST_SIZE = (640, 480)
WRIST_FOVY_DEG = 70.0
# In the Fixed_Jaw frame the jaw runs along -y (tip at y = -0.106, closed tips meet at x = 0.008), the moving jaw
# opens toward -x about a hinge at (-0.020, -0.024, 0), and the upstream wrist-camera mount sits above the jaw
# plane at z 0..0.048, y -0.023..-0.001. The lens is put at the top of that mount, 3 cm down the jaw, looking
# along -y tilted down toward the jaws by WRIST_TILT_DEG: with these numbers the closed tips sit about 10 deg
# below the image centre and the jaw pads run off the bottom edge, as in the real right_wrist image, while the
# hinge 3 cm below the lens stays just out of frame. Image right = jaw -x (the moving jaw's side).
WRIST_CAMERA_POS = (-0.005, -0.030, 0.054)
WRIST_TILT_DEG = 26.0
PHONE_SIZE = (640, 480)
PHONE_FOVY_DEG = 60.0
PHONE_POS_ROBOT = (-0.45, 0.55, 1.20)   # forward, left, up: left rear, looking over the left shoulder
CAMERA_SIZES = {'oak': OAK_SIZE, 'left_wrist': WRIST_SIZE, 'right_wrist': WRIST_SIZE, 'phone': PHONE_SIZE}
CAMERA_NAMES = tuple(CAMERA_SIZES)
WRIST_BODIES = {'left_wrist': twin.ARMS['left_arm'][2], 'right_wrist': twin.ARMS['right_arm'][2]}

LAMP_POS_ROBOT = (0.80, 0.21, 0.98)     # on the table behind the box, bulb a little above a level wrist camera
LAMP_RADIUS_M = 0.06
FLOOR_RGBA = '0.16 0.16 0.17 1'


def _fmt(values):
    return ' '.join(f'{float(v):.6g}' for v in values)


def robot_to_model(forward_m, left_m, up_m):
    """Robot-frame metres -> model coordinates."""
    return ROBOT_ORIGIN_MODEL + AXES.T @ np.array([forward_m, left_m, up_m], dtype=float)


def model_to_robot(xyz):
    """Model coordinates -> robot-frame [forward, left, up]."""
    return AXES @ (np.asarray(xyz, dtype=float) - ROBOT_ORIGIN_MODEL)


def lookat_xyaxes(position, target, up=(0.0, 0.0, 1.0)):
    """MuJoCo camera xyaxes (image right, image up) for a camera at ``position`` looking at ``target``."""
    position, target = np.asarray(position, dtype=float), np.asarray(target, dtype=float)
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(up, dtype=float))
    right /= np.linalg.norm(right)
    image_up = np.cross(right, forward)
    return _fmt(right) + ' ' + _fmt(image_up)


def box_pose(box_forward_m, box_left_m, table_top_m, box_size_m, seed):
    """(model xyz of the box centre, yaw rad) with the seed's jitter (seed None: none)."""
    forward, left, yaw = float(box_forward_m), float(box_left_m), 0.0
    if seed is not None:
        rng = np.random.RandomState(int(seed))
        forward += rng.uniform(-BOX_JITTER_M, BOX_JITTER_M)
        left += rng.uniform(-BOX_JITTER_M, BOX_JITTER_M)
        yaw = math.radians(rng.uniform(-BOX_JITTER_DEG, BOX_JITTER_DEG))
    centre = robot_to_model(forward, left, table_top_m + box_size_m[2] / 2.0)
    return centre, yaw


def build_scene_xml(box_forward_m=None, box_left_m=None, table_top_m=0.70, box_size_m=None,
                    lamp=True, seed=0, path=None, preset=None, lean_jitter_deg=None):
    """MJCF string: the twin's scene plus table, box, lamp, floor and the four cameras. See the module docstring.

    ``preset``: a ``PRESETS`` key (default ``DEFAULT_PRESET``, the real 9 October carton); it supplies the box pose,
    size, flaps and phone position, and the explicit arguments override it (``lean_jitter_deg``: the range the target
    flap's starting lean is drawn from per seed). ``box_size_m`` is (forward depth,
    left-right width, height); the box's near face is at ``box_forward_m - depth/2``. ``seed`` jitters the box
    (None: exactly where asked).
    """
    spec = PRESETS[preset or DEFAULT_PRESET]
    flaps = {side: dict(flap) for side, flap in spec['flaps'].items()}
    lean_jitter_deg = lean_jitter_deg or spec.get('lean_jitter_deg')
    if lean_jitter_deg and seed is not None:
        # the real right flap's lean changed from attempt to attempt (11 deg out in the morning, 5-15 and 25-40 deg in
        # after folds): each seed draws the target flap's starting lean (its crease rest angle) from this range
        rng = np.random.RandomState(int(seed) + 7919)
        for flap in flaps.values():
            if flap.get('target'):
                flap['open_deg'] = float(rng.uniform(*lean_jitter_deg))
    box_forward_m = spec['box_forward_m'] if box_forward_m is None else box_forward_m
    box_left_m = spec['box_left_m'] if box_left_m is None else box_left_m
    box_size_m = spec['box_size_m'] if box_size_m is None else box_size_m
    if path is None:
        path, _ = twin.find_model()
    root = ET.fromstring(twin._scene_xml(path))
    world = root.find('worldbody')
    asset = root.find('asset')
    visual = root.find('visual')
    depth, width, height = (float(v) for v in box_size_m)

    # Near clipping plane: the wrist cameras see jaws 3 cm away; MuJoCo's default znear (1% of the model extent,
    # a few cm here) would clip them. Also a bigger offscreen buffer is not needed: the twin already set one.
    # shadowclip shrinks the directional-light shadow box from the whole model extent (3 mm texels, 20 px steps
    # on a box face 5 cm from a wrist lens) to about 1.5 m around the model centre.
    ET.SubElement(visual, 'map', znear='0.002', zfar='60', shadowclip='0.25')
    quality = visual.find('quality')
    if quality is None:
        quality = ET.SubElement(visual, 'quality')
    quality.set('offsamples', '4')
    quality.set('shadowsize', '4096')   # the lamp's shadow falls on the box face the wrist cameras look at

    # Dark floor and dim room instead of the twin's bright checker and sky.
    # The floor catches a dropped box (collision bit 2, which only the box and flap geoms carry, so the fixed robot's
    # base never touches it); the twin's floor is visual only.
    for geom in world.findall('geom'):
        if geom.get('name') == 'twin_floor':
            geom.attrib.pop('material', None)
            geom.set('rgba', FLOOR_RGBA)
            geom.set('contype', '2')
            geom.set('conaffinity', '2')
    for tex in asset.findall('texture'):
        if tex.get('name') == 'twin_sky':
            tex.set('rgb1', '0.22 0.22 0.24')
            tex.set('rgb2', '0.08 0.08 0.09')

    # Materials: table top, cardboard with corrugation stripes, lamp glow.
    ET.SubElement(asset, 'material', name='scene_table', rgba='0.62 0.55 0.46 1', specular='0.2', shininess='0.3')
    ET.SubElement(asset, 'material', name='scene_leg', rgba='0.25 0.25 0.27 1')
    ET.SubElement(asset, 'texture', name='scene_cardboard', type='2d', builtin='checker', mark='none',
                  rgb1='0.72 0.52 0.32', rgb2='0.66 0.47 0.28', width='64', height='64')
    ET.SubElement(asset, 'material', name='scene_cardboard', texture='scene_cardboard', texrepeat='0.5 24',
                  rgba='1 1 1 1', specular='0.05', shininess='0.05')
    ET.SubElement(asset, 'material', name='scene_flap', rgba='0.75 0.56 0.36 1', specular='0.05', shininess='0.05')
    ET.SubElement(asset, 'material', name='scene_lamp', rgba='1 1 0.92 1', emission='3', specular='0', shininess='0')
    ET.SubElement(asset, 'material', name='scene_lamp_stand', rgba='0.3 0.3 0.32 1')

    # Table: near edge TABLE_NEAR_M forward of the origin, centred on the robot's midline.
    table_near = float(spec.get('table_near_m', TABLE_NEAR_M))
    table_centre = robot_to_model(table_near + TABLE_SIZE_M[0] / 2.0, 0.0, table_top_m - TABLE_THICKNESS_M / 2.0)
    ET.SubElement(world, 'geom', name='table', type='box', pos=_fmt(table_centre),
                  size=_fmt((TABLE_SIZE_M[0] / 2.0, TABLE_SIZE_M[1] / 2.0, TABLE_THICKNESS_M / 2.0)),
                  material='scene_table', contype='1', conaffinity='1', condim='4', friction='1 0.005 0.0001',
                  group='0')
    leg_half = (table_top_m - TABLE_THICKNESS_M) / 2.0
    for i, (df, dl) in enumerate(((0.05, 0.05), (0.05, -0.05), (-0.05, 0.05), (-0.05, -0.05))):
        leg = robot_to_model(table_near + TABLE_SIZE_M[0] / 2.0 + np.sign(df) * (TABLE_SIZE_M[0] / 2.0 - abs(df)),
                             np.sign(dl) * (TABLE_SIZE_M[1] / 2.0 - abs(dl)), leg_half)
        ET.SubElement(world, 'geom', name=f'table_leg_{i}', type='cylinder', pos=_fmt(leg),
                      size=_fmt((0.02, leg_half)), material='scene_leg', contype='0', conaffinity='0', group='0')

    # Cardboard box: free body on the table's near edge; its near face looks at the robot (model +x side).
    centre, yaw = box_pose(box_forward_m, box_left_m, table_top_m, (depth, width, height), seed)
    quat = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    box = ET.SubElement(world, 'body', name='box', pos=_fmt(centre), quat=_fmt(quat))
    ET.SubElement(box, 'freejoint', name='box_free')
    ET.SubElement(box, 'inertial', pos='0 0 0', mass=str(BOX_MASS_KG),
                  diaginertia=_fmt((BOX_MASS_KG / 12.0 * (width ** 2 + height ** 2),
                                    BOX_MASS_KG / 12.0 * (depth ** 2 + height ** 2),
                                    BOX_MASS_KG / 12.0 * (depth ** 2 + width ** 2))))
    if spec.get('hollow'):
        # An open carton (the real one: the depth camera sees its floor at table height): a floor and four 4 mm walls.
        # ``box_body`` stays as the box's outline (size, top height for robot_frame_of_box) but neither collides nor
        # renders (group 3), so a flap can fold past flat into the opening and a claw can reach in.
        ET.SubElement(box, 'geom', name='box_body', type='box', size=_fmt((depth / 2.0, width / 2.0, height / 2.0)),
                      contype='0', conaffinity='0', group='3', rgba='0 0 0 0', mass='0')
        wall = BOX_WALL_M / 2.0
        parts = {'box_floor': ((0.0, 0.0, -height / 2.0 + wall), (depth / 2.0, width / 2.0, wall)),
                 'box_wall_near': ((depth / 2.0 - wall, 0.0, 0.0), (wall, width / 2.0, height / 2.0)),
                 'box_wall_far': ((-depth / 2.0 + wall, 0.0, 0.0), (wall, width / 2.0, height / 2.0)),
                 'box_wall_right': ((0.0, width / 2.0 - wall, 0.0), (depth / 2.0, wall, height / 2.0)),
                 'box_wall_left': ((0.0, -width / 2.0 + wall, 0.0), (depth / 2.0, wall, height / 2.0))}
        for name, (pos, half) in parts.items():
            ET.SubElement(box, 'geom', name=name, type='box', pos=_fmt(pos), size=_fmt(half), mass='0',
                          material='scene_cardboard', contype='3', conaffinity='3', condim='4',
                          friction=f'{BOX_FRICTION} 0.005 0.0001', solref='0.005 1', solimp='0.95 0.99 0.001', group='0')
    else:
        ET.SubElement(box, 'geom', name='box_body', type='box', size=_fmt((depth / 2.0, width / 2.0, height / 2.0)),
                      material='scene_cardboard', contype='3', conaffinity='3', condim='4',
                      friction=f'{BOX_FRICTION} 0.005 0.0001', solref='0.005 1', solimp='0.95 0.99 0.001', group='0')
    # Flaps: plates hinged on a top edge of the box (see ``_add_flap``); the preset's target flap is the scored one
    # (joint ``flap_hinge``, bodies ``box_flap``, ``box_flap_1``, geoms ``flap``, ``flap_1``, site ``box_flap_top``).
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root, 'contact')
    chains = [_add_flap(box, contact, side, flap, (depth, width, height)) for side, flap in flaps.items()]
    # Different flaps never collide: at the far corner an inward-leaning far flap would cut through the folding right
    # flap's far end (real board gives way there; the robot folded the right flap past flat on 9 October with the far
    # flap leaning in). A flap folded on top of another one is credited by SimRobot instead (crease 'covered').
    for i, a in enumerate(chains):
        for b in chains[i + 1:]:
            for x in a:
                for y in b:
                    ET.SubElement(contact, 'exclude', body1=x, body2=y)
    ET.SubElement(box, 'site', name='box_centre', pos='0 0 0', size='0.003', group='4', rgba='0 0 1 0')

    # Lamp behind the box: emissive sphere on a stand plus a strong directional light pointing at the robot.
    if lamp:
        lamp_pos = robot_to_model(*LAMP_POS_ROBOT)
        ET.SubElement(world, 'geom', name='lamp_bulb', type='sphere', pos=_fmt(lamp_pos), size=str(LAMP_RADIUS_M),
                      material='scene_lamp', contype='0', conaffinity='0', group='0')
        stand_top = lamp_pos - np.array([0.0, 0.0, LAMP_RADIUS_M])
        stand_half = (stand_top[2] - table_top_m) / 2.0
        ET.SubElement(world, 'geom', name='lamp_stand', type='cylinder', size=_fmt((0.008, stand_half)),
                      pos=_fmt(stand_top - np.array([0.0, 0.0, stand_half])), material='scene_lamp_stand',
                      contype='0', conaffinity='0', group='0')
        ET.SubElement(world, 'geom', name='lamp_base', type='cylinder', size='0.06 0.006',
                      pos=_fmt(robot_to_model(LAMP_POS_ROBOT[0], LAMP_POS_ROBOT[1], table_top_m + 0.006)),
                      material='scene_lamp_stand', contype='0', conaffinity='0', group='0')
        towards_robot = robot_to_model(0.0, 0.0, 0.85) - lamp_pos
        towards_robot /= np.linalg.norm(towards_robot)
        ET.SubElement(world, 'light', name='lamp_spot', pos=_fmt(lamp_pos), dir=_fmt(towards_robot),
                      directional='false', cutoff='55', exponent='2', diffuse='1 1 0.92', specular='0.7 0.7 0.7',
                      attenuation='1 0 0.3', castshadow='false')  # a shadow map looks blocky 3 cm from a wrist lens
        ET.SubElement(world, 'light', name='lamp_light', pos=_fmt(lamp_pos), dir=_fmt(towards_robot),
                      directional='true', diffuse='0.35 0.35 0.32', specular='0.2 0.2 0.2', castshadow='false')

    # Cameras.
    bodies = {body.get('name'): body for body in world.iter('body')}
    head = bodies.get(twin.HEAD_CAMERA_BODY)
    if head is None:
        raise ValueError(f'model has no {twin.HEAD_CAMERA_BODY} body for the oak camera')
    # at the OAK's optical centre: the slot cradle's offset from the camera link origin (link frame: x out of the lens,
    # y left, z up), as the twin's camera_pose uses it. The stock USB head camera (mesh tophead6 on the tilt link) was
    # removed on 9 October for the OAK; its mesh would sit right under the lens and block the view, so it goes too.
    # The vendored camera-frame marker sites (1 cm translucent boxes at the camera link origin) would sit in front of
    # the lens now that it is offset: they go to the never-drawn group 4.
    for body in world.iter('body'):
        for geom in [g for g in body.findall('geom') if g.get('mesh') == STOCK_HEAD_CAMERA_MESH]:
            body.remove(geom)
    for body in head.iter('body'):
        for site in body.findall('site'):
            if site.get('name') in ('head_camera_rgb_optical_frame', 'head_camera_depth_optical_frame'):
                site.set('group', '4')
    lens = ' '.join(f'{v:.4f}' for v in getattr(twin, 'HEAD_OPTICAL_OFFSET_M', (0.0, 0.0, 0.0)))
    ET.SubElement(head, 'camera', name='oak', pos=lens, xyaxes=OAK_CAMERA_XYAXES, fovy=f'{OAK_FOVY_DEG:.4f}')
    tilt = math.radians(WRIST_TILT_DEG)
    wrist_xyaxes = _fmt((-1.0, 0.0, 0.0, 0.0, -math.sin(tilt), math.cos(tilt)))
    for name, body_name in WRIST_BODIES.items():
        body = bodies.get(body_name)
        if body is None:
            raise ValueError(f'model has no {body_name} body for the {name} camera')
        ET.SubElement(body, 'camera', name=name, pos=_fmt(WRIST_CAMERA_POS), xyaxes=wrist_xyaxes,
                      fovy=str(WRIST_FOVY_DEG))
    phone_pos = robot_to_model(*spec.get('phone', PHONE_POS_ROBOT))
    phone_target = robot_to_model(box_forward_m, box_left_m, table_top_m + height / 2.0)
    ET.SubElement(world, 'camera', name='phone', pos=_fmt(phone_pos), xyaxes=lookat_xyaxes(phone_pos, phone_target),
                  fovy=str(PHONE_FOVY_DEG))
    return ET.tostring(root, encoding='unicode')


FLAP_SIDES = {'near': 0.0, 'right': 0.5 * math.pi, 'far': math.pi, 'left': -0.5 * math.pi}  # yaw of the face's outward normal (box frame: +x faces the robot, +y is the robot's right)


def _add_flap(box, contact, side, flap, size):
    """One hinged flap on the ``side`` top edge of ``box`` (an ET body). The flap body's frame is yawed so its +x
    points out of that face; the hinge axis is its -y, so a positive angle swings the free edge inward over the box.
    It is posed at ``open_deg`` and ``ref`` says so, so qpos reads the angle from vertical (90 = flat over the
    opening, past 90 it dips into the open box; the joint range stands in for the box floor, since parent and child
    never collide). The flap is a chain of panels (``segments``, bottom up): the crease panel on the hinge, then the
    top strip on a nearly free bend joint that stands in for cardboard crushing between the pads. A rigid plate
    pinched between pads that are not parallel to it is locked to the jaws, and this arm cannot tilt its jaws far out
    there, so a rigid flap could never be folded from a pinch: carrying the pinch along the arc would drag the box.
    The strip joint is that pivot: a pinch on the top 2 cm can carry the flap round the hinge, while a deeper pinch
    holds the stiff panel and locks it to the jaws (and drags the box), as stiff board would."""
    depth, width, height = size
    target = bool(flap.get('target'))
    prefix = 'box_flap' if target else f'box_{side}flap'
    hinge = 'flap_hinge' if target else f'{side}flap_hinge'
    yaw = FLAP_SIDES[side]
    half_out = depth / 2.0 if side in ('near', 'far') else width / 2.0
    span = (width if side in ('near', 'far') else depth) - 0.012   # 6 mm short at each end: corners do not touch
    segments, thickness = tuple(flap['segments']), tuple(flap['thickness'])
    total = sum(segments)
    open_rad = math.radians(flap['open_deg'])
    t0 = thickness[0]
    out = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    pos = out * (half_out - t0 / 2.0) + np.array([0.0, 0.0, height / 2.0])
    q_yaw = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
    q_open = np.array([math.cos(open_rad / 2.0), 0.0, -math.sin(open_rad / 2.0), 0.0])
    w1, x1, y1, z1 = q_yaw
    w2, x2, y2, z2 = q_open
    quat = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    parent, chain = box, []
    for i, length in enumerate(segments):
        name = prefix if i == 0 else f'{prefix}_{i}'
        th = thickness[i]
        if i == 0:
            body = ET.SubElement(parent, 'body', name=name, pos=_fmt(pos), quat=_fmt(quat))
            ET.SubElement(body, 'joint', name=hinge, type='hinge', axis='0 -1 0', pos='0 0 0',
                          ref=f'{open_rad:.6g}', springref=f'{open_rad:.6g}', stiffness=str(flap['stiffness']),
                          damping=str(flap['damping']), frictionloss=str(flap['frictionloss']), armature='1e-5',
                          limited='true', range=_fmt([math.radians(v) for v in flap['range']]))
        else:
            body = ET.SubElement(parent, 'body', name=name, pos=_fmt((0.0, 0.0, segments[i - 1])))
            bend_k = tuple(flap.get('bend_stiffness') or ())
            bend_f = tuple(flap.get('bend_frictionloss') or ())
            ET.SubElement(body, 'joint', name=hinge.replace('hinge', f'bend_{i}'), type='hinge', axis='0 -1 0',
                          pos='0 0 0', stiffness=str(bend_k[i - 1] if i - 1 < len(bend_k) else FLAP_BEND_STIFFNESS),
                          damping=str(FLAP_BEND_DAMPING),
                          frictionloss=str(bend_f[i - 1] if i - 1 < len(bend_f) else FLAP_BEND_FRICTIONLOSS),
                          armature='1e-5', limited='true',
                          range=_fmt([math.radians(v) for v in FLAP_BEND_RANGE_DEG]))
        mass = flap['mass'] * length / total
        # the panels share the outer face (flush with the box face); a thinner strip sits on the outer side
        x = (t0 - th) / 2.0
        ET.SubElement(body, 'inertial', pos=_fmt((x, 0.0, length / 2.0)), mass=f'{mass:.6g}',
                      diaginertia=_fmt((mass / 12.0 * (span ** 2 + length ** 2),
                                        mass / 12.0 * (th ** 2 + length ** 2),
                                        mass / 12.0 * (th ** 2 + span ** 2))))
        geom = name.replace('box_', '', 1)   # flap, flap_1 / rightflap ...
        ET.SubElement(body, 'geom', name=geom, type='box', pos=_fmt((x, 0.0, length / 2.0)),
                      size=_fmt((th / 2.0, span / 2.0, length / 2.0)), material='scene_flap',
                      contype='3', conaffinity='3', condim='4', friction=f'{FLAP_FRICTION} {FLAP_TORSION_M} 0.0001',
                      priority='1', solref='0.005 1', solimp='0.95 0.99 0.001', group='0')
        # MuJoCo skips parent-child pairs only; panels further down the chain (and the box) would collide once bent.
        for other in ['box'] + chain[:-1]:
            ET.SubElement(contact, 'exclude', body1=other, body2=name)
        chain.append(name)
        parent = body
    ET.SubElement(parent, 'site', name='box_flap_top' if target else f'box_{side}flap_top', size='0.003', group='4',
                  rgba='0 0 1 0', pos=_fmt((0.0, 0.0, segments[-1])))
    return chain


def colour_arms(model):
    """The twin's arm colouring (left orange, right blue; servo bodies stay dark) on a compiled model."""
    import mujoco
    roots = {}
    for side, base in (('L', 'Base'), ('R', 'Base_2')):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base)
        if bid >= 0:
            roots[bid] = twin.LEFT_RGBA if side == 'L' else twin.RIGHT_RGBA
    for g in range(model.ngeom):
        body = int(model.geom_bodyid[g])
        while body > 0 and body not in roots:
            body = int(model.body_parentid[body])
        if body in roots and model.geom_group[g] == 2:
            matid = int(model.geom_matid[g])
            dark = matid >= 0 and model.mat_rgba[matid][:3].max() < 0.2
            if not dark:
                model.geom_matid[g] = -1
                model.geom_rgba[g] = roots[body]


def robot_frame_of_box(model, data):
    """Where the box is, in the robot frame, from the current ``data`` (run mj_forward/mj_step first).

    Returns {'forward_m', 'left_m', 'up_m' (box centre), 'top_m' (height of the top face), 'flap_top_m' (height of
    the flap's free edge), 'flap_angle_deg' (0 = vertical/open, 90 = folded flat onto the top; None without a hinged
    flap), 'near_face_forward_m', 'yaw_deg', 'size_m': [depth, width, height], 'frame'}.
    """
    import mujoco
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'box')
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'box_body')
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'box_flap_top')
    if min(bid, gid) < 0:
        raise ValueError('model has no box body / box_body geom: build it with build_scene_xml')
    centre = model_to_robot(data.xpos[bid])
    half = np.asarray(model.geom_size[gid], dtype=float)
    rot = np.asarray(data.xmat[bid]).reshape(3, 3)
    # forward axis of the box in robot terms: the box's +x faces the robot, so yaw is about the up axis
    box_x = AXES @ rot[:, 0]
    yaw = math.degrees(math.atan2(box_x[1], -box_x[0]))
    top = centre[2] + half[2] * abs(rot[2, 2]) + half[0] * abs(rot[2, 0]) + half[1] * abs(rot[2, 1])
    flap_top = float(model_to_robot(data.site_xpos[sid])[2]) if sid >= 0 else None
    near_face = float(centre[0] - half[0] * abs(math.cos(math.radians(yaw))) - half[1] * abs(math.sin(math.radians(yaw))))
    return {'forward_m': float(centre[0]), 'left_m': float(centre[1]), 'up_m': float(centre[2]),
            'top_m': float(top), 'flap_top_m': flap_top, 'flap_angle_deg': flap_angle_deg(model, data),
            'near_face_forward_m': near_face, 'yaw_deg': float(yaw),
            'size_m': [float(2 * v) for v in half], 'frame': twin.FRAME}


def flap_angle_deg(model, data):
    """The flap hinge angle in degrees (0 = vertical, + = folded inward, 90 = flat on the top), or None."""
    import mujoco
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'flap_hinge')
    if jid < 0:
        return None
    return float(math.degrees(data.qpos[model.jnt_qposadr[jid]]))


def camera_pose_from_model(model, data, name='oak'):
    """A MuJoCo camera's optical frame in the robot frame, in ``xlerobot_twin.camera_pose`` form:
    {'position_m': [forward, left, up], 'rotation': 3x3 (columns = optical x right, y down, z out of the lens)}."""
    import mujoco
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if cid < 0:
        raise ValueError(f'model has no camera {name!r}')
    cam = np.asarray(data.cam_xmat[cid]).reshape(3, 3)          # columns: camera x (right), y (up), z (backward)
    optical = np.stack([cam[:, 0], -cam[:, 1], -cam[:, 2]], axis=1)
    rotation = AXES @ optical
    position = model_to_robot(data.cam_xpos[cid])
    return {'position_m': [float(v) for v in position], 'rotation': [[float(v) for v in row] for row in rotation],
            'camera': name, 'frame': twin.CAMERA_FRAME}


def intrinsics_for(model, name, size):
    """Pinhole 3x3 for a MuJoCo camera rendered at ``size`` (width, height): fx = fy from its fovy, principal point
    at the image centre (pixel centres at integer + 0.5, so cx = width/2 - 0.5 for integer-indexed back-projection)."""
    import mujoco
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if cid < 0:
        raise ValueError(f'model has no camera {name!r}')
    width, height = int(size[0]), int(size[1])
    fovy = math.radians(float(model.cam_fovy[cid]))
    f = height / 2.0 / math.tan(fovy / 2.0)
    return [[f, 0.0, width / 2.0 - 0.5], [0.0, f, height / 2.0 - 0.5], [0.0, 0.0, 1.0]]
