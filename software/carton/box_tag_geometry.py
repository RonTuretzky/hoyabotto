"""3D corners of the twelve fold-policy box tags, in the carton frame. Geometry only: no robot, camera or simulator state.

Carton frame (carton/geometry.Box 379 x 283 x 108 mm, flaps 140 mm; tools/make_fold_box_tags.py): x to the robot's right,
y away from the robot, z up, origin at the bottom centre. The numbers are the training scene's (carton/folding_sim.py
build_scene, carton/folding_markers.BOX_MARKERS, tools/diagnose_short_flap_brace.py extra markers), with the flaps
upright (hinge angle 0). tools/make_fold_box_tags.TAGS prints the same tags at the same places; the tests check both.

Each tag's black square (45 mm on walls and floor, 35 mm on flaps) is the calibration target. Its corners are listed
in the order the shared detector (carton.servo.features.tags_from_bgr -> pupil-apriltags) reports them for the
generated tag36h11 marker seen upright from outside its face:

    0: top-right, 1: top-left, 2: bottom-left, 3: bottom-right

(checked on carton.servo.tag_kit.marker_image: corners (360,40), (40,40), (40,360), (360,360) for a 320 px square).

In the scene the wall and flap tags sit 1.8 mm proud of the cardboard centre plane (the walls are 3 mm boxes centred on
the outer dimension) and the printed black cells another 0.3 mm. On the real carton the paper lies on the outer face
instead, about 2 mm further in; the lens solver (tools/calibrate_wrist_from_tags.py) estimates each tag's placement
with the views, so millimetre-level differences like this, a hand-placed tag or a leaning flap do not bias the lens.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from carton.geometry import Box

BOX = Box()
L, W, H, F = BOX.length, BOX.width, BOX.height, BOX.flap
WALL_TAG, FLAP_TAG = .045, .035          # black square, m
PROUD = .0018                            # scene: tag body off the cardboard centre plane (folding_sim / folding_markers)
INK = .0003                              # scene: top of the black cells above the tag body (marker(): .0002 + .0001)
FLAP_TAG_UP = .090                       # flap tag centre above the hinge line (folding_sim tag_point z)

# Corner signs in the tag's own (right, up) axes, in detector order.
CORNER_SIGNS = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]], float)


@dataclass(frozen=True)
class TagMount:
    tag_id: int
    face: str
    size: float                      # black square side, m
    centre: tuple                    # body origin in the carton frame (flaps upright), m
    right: tuple                     # tag x axis (image right seen from outside), carton frame
    up: tuple                        # tag y axis (image up seen from outside), carton frame
    scene_body: str
    hinge: str | None = None         # flap hinge joint name in the training scene
    hinge_origin: tuple | None = None
    hinge_axis: tuple | None = None

    @property
    def normal(self) -> np.ndarray:
        return np.cross(self.right, self.up)

    @property
    def rigid(self) -> bool:
        return self.hinge is None


def _flap(tag_id, face, name, hinge_origin, hinge_axis, tag_point, right, up):
    n = np.cross(right, up)
    centre = np.asarray(hinge_origin) + np.asarray(tag_point) + n * PROUD
    return TagMount(tag_id, face, FLAP_TAG, tuple(centre), tuple(right), tuple(up), name + '_tag', name + '_hinge',
                    tuple(hinge_origin), tuple(hinge_axis))


NEAR, FAR = -W / 2 - PROUD, W / 2 + PROUD
TAGS: dict[int, TagMount] = {t.tag_id: t for t in [
    TagMount(26, 'near-wall', WALL_TAG, (-.12, NEAR, H / 2), (1, 0, 0), (0, 0, 1), 'box_tag_near_left'),
    TagMount(10, 'near-wall', WALL_TAG, (0., NEAR, H / 2), (1, 0, 0), (0, 0, 1), 'box_tag'),
    TagMount(27, 'near-wall', WALL_TAG, (.12, NEAR, H / 2), (1, 0, 0), (0, 0, 1), 'box_tag_near_right'),
    TagMount(21, 'left-wall', WALL_TAG, (-L / 2 - PROUD, .04, H / 2), (0, -1, 0), (0, 0, 1), 'box_tag_left'),
    TagMount(28, 'left-wall', WALL_TAG, (-L / 2 - PROUD, -.08, H / 2), (0, -1, 0), (0, 0, 1), 'box_tag_left_near'),
    TagMount(22, 'right-wall', WALL_TAG, (L / 2 + PROUD, .04, H / 2), (0, 1, 0), (0, 0, 1), 'box_tag_right'),
    TagMount(25, 'floor', WALL_TAG, (0., 0., .0038), (1, 0, 0), (0, 1, 0), 'box_tag_floor_center'),
    TagMount(24, 'floor', WALL_TAG, (.08, 0., .0038), (1, 0, 0), (0, 1, 0), 'box_tag_floor'),
    _flap(11, 'short-left-flap', 'short_left', (-L / 2, 0, H), (0, 1, 0), (0, .07, FLAP_TAG_UP), (0, -1, 0), (0, 0, 1)),
    _flap(12, 'short-right-flap', 'short_right', (L / 2, 0, H), (0, -1, 0), (0, .07, FLAP_TAG_UP), (0, 1, 0), (0, 0, 1)),
    _flap(13, 'long-far-flap', 'long_far', (0, W / 2, H + .0035), (1, 0, 0), (-.08, 0, FLAP_TAG_UP), (-1, 0, 0), (0, 0, 1)),
    _flap(14, 'long-near-flap', 'long_near', (0, -W / 2, H + .0035), (-1, 0, 0), (.08, 0, FLAP_TAG_UP), (1, 0, 0), (0, 0, 1)),
]}
RIGID_IDS = tuple(i for i, t in TAGS.items() if t.rigid)
FLAP_IDS = tuple(i for i, t in TAGS.items() if not t.rigid)


def _rotation(axis, angle):
    """Rotation matrix about a unit axis (right-hand rule, as a MuJoCo hinge with that axis)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def tag_corners(tag_id: int, flap_angles: dict[str, float] | None = None) -> np.ndarray:
    """4x3 corners (m, carton frame) of the tag's black square, detector order. Flaps upright unless `flap_angles`
    gives a hinge angle (rad, the scene's hinge joint sign) for this tag's flap."""
    t = TAGS[tag_id]
    centre = np.asarray(t.centre, float) + INK * t.normal
    right, up = np.asarray(t.right, float), np.asarray(t.up, float)
    pts = centre + (CORNER_SIGNS[:, :1] * right + CORNER_SIGNS[:, 1:] * up) * (t.size / 2)
    angle = (flap_angles or {}).get(t.hinge, 0.0) if t.hinge else 0.0
    if angle:
        origin = np.asarray(t.hinge_origin, float)
        pts = (pts - origin) @ _rotation(t.hinge_axis, angle).T + origin
    return pts


def all_corners(ids=None, flap_angles=None) -> dict[int, np.ndarray]:
    return {i: tag_corners(i, flap_angles) for i in (TAGS if ids is None else ids)}


def tag_frame(tag_id: int) -> tuple[np.ndarray, np.ndarray]:
    """(R, c): the tag's axes (columns right, up, normal) and black-square centre in the carton frame, flaps upright."""
    t = TAGS[tag_id]
    r = np.column_stack((t.right, t.up, t.normal)).astype(float)
    return r, np.asarray(t.centre, float) + INK * t.normal


def tag_local_corners(tag_id: int) -> np.ndarray:
    """4x3 corners in the tag's own frame (z = 0 plane), detector order."""
    s = TAGS[tag_id].size / 2
    return np.column_stack((CORNER_SIGNS * s, np.zeros(4)))


# ------------------------------------------------------------------------------------------- training scene
def scene_flap_angles(model, data) -> dict[str, float]:
    return {t.hinge: float(data.qpos[model.jnt_qposadr[model.joint(t.hinge).id]]) for t in TAGS.values() if t.hinge}


def scene_carton_pose(model, data) -> tuple[np.ndarray, np.ndarray]:
    """(R, p) of the carton body in the world, after mj_kinematics/mj_forward."""
    b = model.body('carton').id
    return data.xmat[b].reshape(3, 3).copy(), data.xpos[b].copy()


def scene_corners_world(model, data, ids=None) -> dict[int, np.ndarray]:
    r, p = scene_carton_pose(model, data)
    angles = scene_flap_angles(model, data)
    return {i: c @ r.T + p for i, c in all_corners(ids, angles).items()}


def mujoco_camera(model, data, camera, width, height) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(K, R_cv, position): pinhole intrinsics of a MuJoCo camera rendered at width x height, and its pose as an
    OpenCV camera (columns: x right, y down, z forward, world frame). Pixel centres at integer coordinates."""
    cam = model.camera(camera).id if isinstance(camera, str) else int(camera)
    f = height / 2 / math.tan(math.radians(model.cam_fovy[cam]) / 2)
    k = np.array([[f, 0, (width - 1) / 2], [0, f, (height - 1) / 2], [0, 0, 1.]])
    m = data.cam_xmat[cam].reshape(3, 3)
    return k, m @ np.diag([1., -1., -1.]), data.cam_xpos[cam].copy()


def project(k, r_cv, position, points) -> tuple[np.ndarray, np.ndarray]:
    """Pinhole projection of world points; returns (uv Nx2, depth N)."""
    cam = (np.asarray(points, float) - position) @ r_cv
    z = cam[:, 2]
    uv = cam[:, :2] / np.where(np.abs(z) < 1e-9, 1e-9, z)[:, None] @ k[:2, :2].T + k[:2, 2]
    return uv, z
