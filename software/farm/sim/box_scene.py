"""The box-grab benchmark scene: the twin's XLeRobot plus a table, a cardboard box, a lamp and cameras.

``build_scene_xml`` starts from ``farm.sim.xlerobot_twin._scene_xml`` (upstream MJCF made fixed-base, claw tip
sites, the OAK optical site ``HEAD_SITE`` on ``HEAD_CAMERA_BODY``, floor, lights, offscreen buffer) and adds:

- a table (box geom, 0.60 m deep x 0.90 m wide, top at ``table_top_m``) whose near edge is ``TABLE_NEAR_M`` forward
  of the robot origin; four visual legs;
- a free cardboard box (body ``box``, freejoint ``box_free``, 0.25 kg, friction 1.0, a brown corrugation-striped
  material) resting on the table's near edge, with an open carton flap: body ``box_flap`` (geom ``flap``, 7 cm tall,
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

# Robot frame origin in model coordinates: the shoulder-pan axes are at model (-0.09, +-0.1552, 0.7915).
ROBOT_ORIGIN_MODEL = np.array([-0.09, 0.0, 0.0])
AXES = np.array([twin.FORWARD, twin.LEFT, twin.UP], dtype=float)  # rows: robot axes in model coordinates

TABLE_NEAR_M = 0.30           # near edge forward of the robot origin
TABLE_SIZE_M = (0.60, 0.90)   # depth (forward), width (left-right)
TABLE_THICKNESS_M = 0.03
FLAP_SEGMENTS_M = (0.05, 0.02)  # bottom up: the flap panel on the crease, then the top strip the pads hold
FLAP_HEIGHT_M = sum(FLAP_SEGMENTS_M)    # 7 cm, the real carton's open flap is about 8
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
BOX_MASS_KG = 0.25
BOX_FRICTION = 1.0
BOX_JITTER_M = 0.02
BOX_JITTER_DEG = 5.0

# Cameras. OAK RGB: the real device reports fx = 504.9, fy = 505.0 at 640x360 (software/farm/sim/assets/sim_robot/
# real_oak_depth_manifest.json), i.e. 64.7 deg horizontal, 39.2 deg vertical. MuJoCo takes the vertical fov.
OAK_SIZE = (640, 360)
OAK_FOCAL_PX = 505.0
OAK_FOVY_DEG = math.degrees(2.0 * math.atan(OAK_SIZE[1] / 2.0 / OAK_FOCAL_PX))
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


def build_scene_xml(box_forward_m=0.42, box_left_m=0.21, table_top_m=0.70, box_size_m=(0.20, 0.15, 0.11),
                    lamp=True, seed=0, path=None):
    """MJCF string: the twin's scene plus table, box, lamp, floor and the four cameras. See the module docstring.

    ``box_size_m`` is (forward depth, left-right width, height); the box's near face is at
    ``box_forward_m - depth/2``. ``seed`` jitters the box (None: exactly where asked).
    """
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
    table_centre = robot_to_model(TABLE_NEAR_M + TABLE_SIZE_M[0] / 2.0, 0.0, table_top_m - TABLE_THICKNESS_M / 2.0)
    ET.SubElement(world, 'geom', name='table', type='box', pos=_fmt(table_centre),
                  size=_fmt((TABLE_SIZE_M[0] / 2.0, TABLE_SIZE_M[1] / 2.0, TABLE_THICKNESS_M / 2.0)),
                  material='scene_table', contype='1', conaffinity='1', condim='4', friction='1 0.005 0.0001',
                  group='0')
    leg_half = (table_top_m - TABLE_THICKNESS_M) / 2.0
    for i, (df, dl) in enumerate(((0.05, 0.05), (0.05, -0.05), (-0.05, 0.05), (-0.05, -0.05))):
        leg = robot_to_model(TABLE_NEAR_M + TABLE_SIZE_M[0] / 2.0 + np.sign(df) * (TABLE_SIZE_M[0] / 2.0 - abs(df)),
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
    ET.SubElement(box, 'geom', name='box_body', type='box', size=_fmt((depth / 2.0, width / 2.0, height / 2.0)),
                  material='scene_cardboard', contype='3', conaffinity='3', condim='4',
                  friction=f'{BOX_FRICTION} 0.005 0.0001', solref='0.005 1', solimp='0.95 0.99 0.001', group='0')
    # Flap: a plate hinged on the near top edge (the +x face in the box frame faces the robot), its outer face flush
    # with the near face. Joint axis -y: a positive angle swings the flap's free edge toward -x, i.e. inward over
    # the top. The body is posed at FLAP_OPEN_DEG and ``ref`` says so, so qpos reads the angle from vertical.
    # The flap is a chain of panels (FLAP_SEGMENTS_M, bottom up): ``box_flap`` (geom ``flap``) on the crease
    # ``flap_hinge``, then ``box_flap_1`` (geom ``flap_1``, the top strip) on the bend joint ``flap_bend_1``.
    # A rigid plate pinched between pads that are not parallel to it is locked to the jaws, and this arm cannot tilt
    # its jaws past about -60 deg pitch out there, so a rigid flap could never be folded from a pinch: carrying the
    # pinch along the arc would drag the box instead. Real corrugated board crushes between the pads and turns there.
    # The nearly free strip joint is that pivot: a pinch on the top 2 cm can carry the flap round the hinge, while a
    # deeper pinch holds the stiff panel and locks it to the jaws (and drags the box), as stiff board would.
    open_rad = math.radians(FLAP_OPEN_DEG)
    flap_width = width - 0.002
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root, 'contact')
    parent, chain = box, []
    for i, length in enumerate(FLAP_SEGMENTS_M):
        name = 'box_flap' if i == 0 else f'box_flap_{i}'
        if i == 0:
            body = ET.SubElement(parent, 'body', name=name,
                                 pos=_fmt((depth / 2.0 - FLAP_THICKNESS_M / 2.0, 0.0, height / 2.0)),
                                 quat=_fmt((math.cos(open_rad / 2.0), 0.0, -math.sin(open_rad / 2.0), 0.0)))
            ET.SubElement(body, 'joint', name='flap_hinge', type='hinge', axis='0 -1 0', pos='0 0 0',
                          ref=f'{open_rad:.6g}', springref=f'{open_rad:.6g}', stiffness=str(FLAP_STIFFNESS),
                          damping=str(FLAP_DAMPING), frictionloss=str(FLAP_FRICTIONLOSS), armature='1e-5',
                          limited='true', range=_fmt([math.radians(v) for v in FLAP_RANGE_DEG]))
        else:
            body = ET.SubElement(parent, 'body', name=name, pos=_fmt((0.0, 0.0, FLAP_SEGMENTS_M[i - 1])))
            ET.SubElement(body, 'joint', name=f'flap_bend_{i}', type='hinge', axis='0 -1 0', pos='0 0 0',
                          stiffness=str(FLAP_BEND_STIFFNESS), damping=str(FLAP_BEND_DAMPING),
                          frictionloss=str(FLAP_BEND_FRICTIONLOSS), armature='1e-5', limited='true',
                          range=_fmt([math.radians(v) for v in FLAP_BEND_RANGE_DEG]))
        mass = FLAP_MASS_KG * length / FLAP_HEIGHT_M
        ET.SubElement(body, 'inertial', pos=_fmt((0.0, 0.0, length / 2.0)), mass=f'{mass:.6g}',
                      diaginertia=_fmt((mass / 12.0 * (flap_width ** 2 + length ** 2),
                                        mass / 12.0 * (FLAP_THICKNESS_M ** 2 + length ** 2),
                                        mass / 12.0 * (FLAP_THICKNESS_M ** 2 + flap_width ** 2))))
        ET.SubElement(body, 'geom', name='flap' if i == 0 else f'flap_{i}', type='box', pos=_fmt((0.0, 0.0, length / 2.0)),
                      size=_fmt((FLAP_THICKNESS_M / 2.0, flap_width / 2.0, length / 2.0)), material='scene_flap',
                      contype='3', conaffinity='3', condim='4', friction=f'{FLAP_FRICTION} {FLAP_TORSION_M} 0.0001',
                      priority='1', solref='0.005 1', solimp='0.95 0.99 0.001', group='0')
        # MuJoCo skips parent-child pairs only; panels further down the chain (and the box) would collide once bent.
        for other in ['box'] + chain[:-1]:
            ET.SubElement(contact, 'exclude', body1=other, body2=name)
        chain.append(name)
        parent = body
    ET.SubElement(box, 'site', name='box_centre', pos='0 0 0', size='0.003', group='4', rgba='0 0 1 0')
    ET.SubElement(parent, 'site', name='box_flap_top', size='0.003', group='4', rgba='0 0 1 0',
                  pos=_fmt((0.0, 0.0, FLAP_SEGMENTS_M[-1])))

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
    ET.SubElement(head, 'camera', name='oak', pos='0 0 0', xyaxes=OAK_CAMERA_XYAXES, fovy=f'{OAK_FOVY_DEG:.4f}')
    tilt = math.radians(WRIST_TILT_DEG)
    wrist_xyaxes = _fmt((-1.0, 0.0, 0.0, 0.0, -math.sin(tilt), math.cos(tilt)))
    for name, body_name in WRIST_BODIES.items():
        body = bodies.get(body_name)
        if body is None:
            raise ValueError(f'model has no {body_name} body for the {name} camera')
        ET.SubElement(body, 'camera', name=name, pos=_fmt(WRIST_CAMERA_POS), xyaxes=wrist_xyaxes,
                      fovy=str(WRIST_FOVY_DEG))
    phone_pos = robot_to_model(*PHONE_POS_ROBOT)
    phone_target = robot_to_model(box_forward_m, box_left_m, table_top_m + height / 2.0)
    ET.SubElement(world, 'camera', name='phone', pos=_fmt(phone_pos), xyaxes=lookat_xyaxes(phone_pos, phone_target),
                  fovy=str(PHONE_FOVY_DEG))
    return ET.tostring(root, encoding='unicode')


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
