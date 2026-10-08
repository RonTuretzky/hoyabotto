"""Set the fold policy's simulated cameras and station from a measurement file. Simulation only.

The short-flap fold policy (tools/fold_demos_to_lerobot.py) sees simulated cameras named by
`POLICY_CAMERAS`: `top` (scene camera `overhead`, simulation only), `front` (the head camera) and the
wrist cameras `left_wrist`/`right_wrist` (created inside the gripper bodies). This module sets those
cameras (pose and vertical field of view), optionally the visual appearance (arm/table colour, table size,
printed markers), and the station geometry (arm-base height, base line to table edge, base spacing) from a
measurement or model-derived file (carton/xlerobot_cameras.py), without editing the scene builder or the
scripted controller:

- `restage_scene_xml` rewrites an existing recorded `scene.xml`. Cameras and appearance only: the scripted
  controller never looks through `overhead` or `front`, so recorded demonstrations can be re-rendered with
  measured cameras without re-recording. It refuses when the measured station differs from the recorded one.
- `install` patches `carton.folding_station.FoldingStation` and `carton.folding_sim.build_scene` in the
  current process, so a new recording (tools/record_measured_fold_demos.py) builds the measured station and
  cameras (not the appearance, which would hide the controller's markers; restage afterwards). Station
  changes move the arms relative to the carton and need new demonstrations. When the base spacing differs
  from 300 mm, the controller's explicit parked start targets are moved with the bases (same pose relative
  to each base), because the 300 mm targets leave the left gripper tag hidden from the controller's
  `station` camera at 220 mm (0/8 starts registered).

Measurement frame (`arm_base`): origin midway between the two SO101 `base_link` origins (the base mounting
plane, centre of the base's mounting footprint), +x to the robot's right, +y horizontally toward the table
(away from the robot), +z up; metres. The simulated tabletop is z = -base_height in this frame.
"""
from __future__ import annotations

import atexit
import copy
import json
import math
import weakref
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

SCHEMA = 'xlerobot-fold-station-measurement/1'
# Measurement key -> scene camera name. `top` (the overhead camera) exists only in simulation; the robot's
# policy cameras are the head camera (`front`) and the two wrist cameras.
POLICY_CAMERAS = {'top': 'overhead', 'front': 'front', 'left_wrist': 'left_wrist', 'right_wrist': 'right_wrist'}
# Frames a camera position/orientation may be given in: the arm-base frame, or a gripper body (the camera
# then moves with that gripper).
CAMERA_FRAMES = ('arm_base', 'left_gripper_link', 'right_gripper_link')
# Base spacing of the recorded batch-01/02 station; the scripted controller's parked start targets
# (tools/diagnose_short_flap_brace.py --park-back) were chosen for it.
REFERENCE_BASE_SPACING_M = .30
POLICY_ASPECT = 4 / 3  # the policy's 320x240 images
# The recorded demonstrations place the carton's near wall 10 mm from the table edge
# (tools/diagnose_short_flap_brace.py computes the carton offset with that value).
CARTON_TO_EDGE_M = .01
_STATION_KEYS = ('base_height_above_table_m', 'base_line_to_table_edge_m', 'base_spacing_m')


def _vec(value, name, n=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must be {n} finite numbers')
    return a


def words(values):
    return ' '.join(f'{float(v):.10g}' for v in values)


# ---------------------------------------------------------------- camera geometry

def look_at_axes(position, look_at, up_hint=None, roll_deg=0.):
    """MuJoCo camera x (image right) and y (image up) axes for a camera at `position` looking at `look_at`.

    Same construction as carton.folding_sim.build_scene: right = up_hint x back, with up_hint = +z unless
    the view is within 10 degrees of vertical, where the image top points +y (away from the robot).
    `roll_deg` rotates the image about the optical axis (positive: image content turns counter-clockwise).
    """
    position, look_at = _vec(position, 'position'), _vec(look_at, 'look_at')
    back = position - look_at
    if np.linalg.norm(back) < 1e-6:
        raise ValueError('Camera position and look-at point coincide')
    back /= np.linalg.norm(back)
    if up_hint is None:
        up_hint = [0., 1., 0.] if abs(back[2]) > math.cos(math.radians(10)) else [0., 0., 1.]
    up_hint = _vec(up_hint, 'up_hint')
    right = np.cross(up_hint, back)
    if np.linalg.norm(right) < 1e-6:
        raise ValueError('up_hint is parallel to the viewing direction')
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    if roll_deg:
        r = math.radians(roll_deg)
        right, up = math.cos(r) * right + math.sin(r) * up, -math.sin(r) * right + math.cos(r) * up
    return right, up


def axes_from_rotation_cv(rotation_cv):
    """MuJoCo x/y camera axes from an OpenCV camera-to-frame rotation (columns: optical x right, y down, z forward)."""
    r = np.asarray(rotation_cv, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError('rotation_cv must be a finite 3x3 matrix')
    if not np.allclose(r.T @ r, np.eye(3), atol=2e-3) or np.linalg.det(r) < 0:
        raise ValueError('rotation_cv must be a proper rotation (orthonormal, det +1)')
    u, _, vt = np.linalg.svd(r)
    r = u @ vt
    return r[:, 0], -r[:, 1]


def rotation_cv_from_axes(right, up):
    right, up = np.asarray(right, float), np.asarray(up, float)
    return np.column_stack((right, -up, np.cross(right, -up)))


def policy_crop(intrinsics, fovy_deg=None, aspect=POLICY_ASPECT):
    """Centred-on-principal-point crop of a real image that yields the policy's aspect ratio.

    Without `fovy_deg`, the largest such crop; with it, the crop whose vertical field of view equals
    `fovy_deg` (`fits` is False when the camera is narrower than that). Returns the crop box in source
    pixels (x0, y0, x1, y1) and its vertical field of view. Assumes an undistorted (rectified) image.
    """
    k = {key: float(intrinsics[key]) for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')}
    if not all(math.isfinite(v) and v > 0 for v in k.values()):
        raise ValueError('Intrinsics need positive finite fx, fy, cx, cy, width, height')
    half_h_max = min(k['cy'], k['height'] - k['cy'], min(k['cx'], k['width'] - k['cx']) / aspect)
    fits = True
    if fovy_deg is None:
        half_h = half_h_max
    else:
        half_h = k['fy'] * math.tan(math.radians(fovy_deg) / 2)
        if half_h > half_h_max + 1e-9:
            fits, half_h = False, half_h_max
    half_w = half_h * aspect
    return {'crop_xyxy': [k['cx'] - half_w, k['cy'] - half_h, k['cx'] + half_w, k['cy'] + half_h],
            'fovy_deg': math.degrees(2 * math.atan(half_h / k['fy'])),
            'fovx_deg': math.degrees(2 * math.atan(half_w / k['fx'])),
            'pixel_aspect_error': abs(k['fx'] - k['fy']) / k['fy'], 'fits': fits}


def camera_pose(spec, name):
    """(position, right, up, fovy_deg) of one camera spec in the arm_base frame."""
    if not isinstance(spec, dict):
        raise ValueError(f'camera {name}: object required')
    if spec.get('frame', 'arm_base') not in CAMERA_FRAMES:
        raise ValueError(f'camera {name}: frame must be one of {CAMERA_FRAMES}')
    position = _vec(spec.get('position_m'), f'camera {name} position_m')
    has_look, has_rot = spec.get('look_at_m') is not None, spec.get('rotation_cv') is not None
    if has_look == has_rot:
        raise ValueError(f'camera {name}: give exactly one of look_at_m or rotation_cv')
    if has_look:
        right, up = look_at_axes(position, spec['look_at_m'], spec.get('up_hint'), float(spec.get('roll_deg', 0.)))
    else:
        right, up = axes_from_rotation_cv(spec['rotation_cv'])
    if spec.get('fovy_deg') is not None:
        fovy = float(spec['fovy_deg'])
    elif spec.get('intrinsics') is not None:
        fovy = policy_crop(spec['intrinsics'], spec.get('crop_to_fovy_deg'))['fovy_deg']
    else:
        raise ValueError(f'camera {name}: give fovy_deg or intrinsics')
    if not 10 <= fovy <= 120:
        raise ValueError(f'camera {name}: vertical field of view {fovy:.1f} deg outside 10..120')
    return position, right, up, fovy


# ---------------------------------------------------------------- measurement file

def load_measurement(source):
    """Read and validate a measurement JSON (path or dict). Missing values (null) are refused."""
    m = json.loads(Path(source).read_text()) if not isinstance(source, dict) else copy.deepcopy(source)
    if m.get('schema') != SCHEMA:
        raise ValueError(f'Measurement schema must be {SCHEMA!r}')
    cams = m.get('cameras') or {}
    unknown = set(cams) - set(POLICY_CAMERAS)
    if unknown:
        raise ValueError(f'Unknown policy cameras {sorted(unknown)}; use {sorted(POLICY_CAMERAS)}')
    for key, spec in cams.items():
        camera_pose(spec, key)
    station = m.get('station')
    if station is not None:
        for key in _STATION_KEYS:
            v = station.get(key)
            if v is None or not math.isfinite(float(v)):
                raise ValueError(f'station.{key} must be a measured number')
        if not 0 < float(station['base_spacing_m']) < .8:
            raise ValueError('station.base_spacing_m outside 0..0.8 m')
        if not -.05 <= float(station['base_height_above_table_m']) <= .5:
            raise ValueError('station.base_height_above_table_m outside -0.05..0.5 m')
        edge = station.get('carton_near_wall_to_table_edge_m', CARTON_TO_EDGE_M)
        if abs(float(edge) - CARTON_TO_EDGE_M) > .001:
            raise ValueError('Place the carton 10 mm from the table edge: the recorded carton placement assumes it')
    app = m.get('appearance') or {}
    for key in ('arm_rgba', 'table_rgba'):
        if app.get(key) is not None:
            rgba = _vec(app[key], f'appearance.{key}', 4)
            if not ((rgba >= 0) & (rgba <= 1)).all():
                raise ValueError(f'appearance.{key} values must be 0..1')
    if app.get('table_size_m') is not None:
        size = _vec(app['table_size_m'], 'appearance.table_size_m', 2)
        if not (size > .3).all():
            raise ValueError('appearance.table_size_m must exceed the carton footprint')
    return m


# ---------------------------------------------------------------- scene XML

def _bodies(root):
    return {b.get('name'): b for b in root.iter('body') if b.get('name')}


def scene_station(root):
    """Station geometry of a built folding scene: arm_base origin (world) and the measured quantities."""
    bodies = _bodies(root)
    try:
        left = _vec(bodies['left_base_link'].get('pos').split(), 'left_base_link pos')
        right = _vec(bodies['right_base_link'].get('pos').split(), 'right_base_link pos')
    except (KeyError, AttributeError) as exc:
        raise ValueError('Scene has no left/right_base_link bodies') from exc
    table = next((g for g in root.iter('geom') if g.get('name') == 'table'), None)
    if table is None:
        raise ValueError('Scene has no table geom')
    tpos, tsize = _vec(table.get('pos').split(), 'table pos'), _vec(table.get('size').split(), 'table size')
    origin = (left + right) / 2
    edge_y = tpos[1] - tsize[1]
    return {'arm_base_origin_world_m': origin.tolist(),
            'base_height_above_table_m': float(origin[2]),
            'base_line_to_table_edge_m': float(edge_y - origin[1]),
            'base_spacing_m': float(np.linalg.norm(right - left)),
            'table_edge_y_world_m': float(edge_y),
            'table_top_z_world_m': float(tpos[2] + tsize[2])}


def station_mismatch(recorded, station, tolerance_m=.002):
    return {k: {'recorded': recorded[k], 'measured': float(station[k])} for k in _STATION_KEYS
            if abs(recorded[k] - float(station[k])) > tolerance_m}


def _set_camera(root, name, origin, position, right, up, fovy, frame='arm_base'):
    """Pose camera `name`. In the arm_base frame it must already exist in the worldbody; a gripper-frame
    camera is created (or moved) inside that gripper body."""
    world = root.find('worldbody')
    cams = [c for c in world.iter('camera') if c.get('name') == name]
    if frame == 'arm_base':
        if len(cams) != 1:
            raise ValueError(f'Scene must contain exactly one {name!r} camera')
        cam = cams[0]
    else:
        body = _bodies(root).get(frame)
        if body is None:
            raise ValueError(f'Scene has no {frame!r} body for camera {name!r}')
        if len(cams) > 1:
            raise ValueError(f'Scene has several {name!r} cameras')
        if cams and cams[0] in list(body):
            cam = cams[0]
        else:
            for parent in world.iter():
                for c in list(parent):
                    if c.tag == 'camera' and c.get('name') == name:
                        parent.remove(c)
            cam = ET.SubElement(body, 'camera', name=name)
        origin = np.zeros(3)
    for attr in ('quat', 'axisangle', 'euler', 'zaxis', 'xyaxes', 'focal', 'focalpixel', 'principal',
                 'principalpixel', 'sensorsize', 'resolution', 'ipd'):
        cam.attrib.pop(attr, None)
    cam.set('pos', words(np.asarray(origin) + position))
    cam.set('xyaxes', words(np.r_[right, up]))
    cam.set('fovy', f'{fovy:.10g}')


def _marker_bodies(root):
    names = {g.get('name') for g in root.iter('geom')}
    return [b for b in root.iter('body') if b.get('name') and b.get('name') + '_paper' in names]


def apply_appearance(root, appearance):
    """Visual-only changes: arm colour, table colour/size, printed markers hidden. Returns a change log."""
    log = {}
    if not appearance:
        return log
    if appearance.get('arm_rgba') is not None:
        rgba = words(appearance['arm_rgba'])
        count = 0
        markers = {id(g) for b in _marker_bodies(root) for g in b.findall('geom')}
        for side in ('left', 'right'):
            base = _bodies(root).get(side + '_base_link')
            for g in base.iter('geom') if base is not None else ():
                # Visual arm geoms only: collision geoms are group 3; printed gripper tags keep their colours.
                if g.get('contype') == '0' and g.get('group') != '3' and id(g) not in markers:
                    g.set('rgba', rgba)
                    count += 1
        log['arm_visual_geoms_recoloured'] = count
    table = next((g for g in root.iter('geom') if g.get('name') == 'table'), None)
    if appearance.get('table_rgba') is not None:
        table.set('rgba', words(appearance['table_rgba']))
        log['table_rgba'] = list(appearance['table_rgba'])
    hide = bool(appearance.get('hide_markers'))
    if appearance.get('table_size_m') is not None:
        sx, sy = (float(v) for v in appearance['table_size_m'])
        pos, size = np.asarray(table.get('pos').split(), float), np.asarray(table.get('size').split(), float)
        edge = pos[1] - size[1]  # keep the near edge (and so the carton placement) unchanged
        table.set('pos', words([0., edge + sy / 2, pos[2]]))
        table.set('size', words([sx / 2, sy / 2, size[2]]))
        log['table_size_m'] = [sx, sy]
        # Table anchor tags would float off a smaller table.
        for b in _marker_bodies(root):
            if b.get('name', '').startswith('table_tag'):
                _hide_marker(b)
                log.setdefault('table_tags_hidden', []).append(b.get('name'))
    if hide:
        hidden = [b.get('name') for b in _marker_bodies(root)]
        for b in _marker_bodies(root):
            _hide_marker(b)
        log['markers_hidden'] = hidden
    return log


def _hide_marker(body):
    for g in body.findall('geom'):
        rgba = (g.get('rgba') or '1 1 1 1').split()
        g.set('rgba', words([*map(float, rgba[:3]), 0.]))


def restage_root(root, measurement, *, allow_station_change=False):
    """Apply cameras (+ appearance) of a validated measurement to a parsed scene root, in place."""
    measurement = load_measurement(measurement)
    recorded = scene_station(root)
    report = {'recorded_station': recorded, 'cameras': {}}
    if measurement.get('station') is not None:
        diff = station_mismatch(recorded, measurement['station'])
        report['station_mismatch'] = diff
        if diff and not allow_station_change:
            raise ValueError('Measured station differs from the recorded scene '
                             f'({diff}); record new demonstrations with tools/record_measured_fold_demos.py')
    origin = np.asarray(recorded['arm_base_origin_world_m'])
    for key, spec in (measurement.get('cameras') or {}).items():
        position, right, up, fovy = camera_pose(spec, key)
        frame = spec.get('frame', 'arm_base')
        _set_camera(root, POLICY_CAMERAS[key], origin, position, right, up, fovy, frame)
        report['cameras'][key] = {'scene_camera': POLICY_CAMERAS[key], 'frame': frame,
                                  'position_m': position.tolist(), 'fovy_deg': fovy,
                                  'right': right.tolist(), 'up': up.tolist()}
    report['appearance'] = apply_appearance(root, measurement.get('appearance'))
    return report


def restage_scene_xml(scene_in, scene_out, measurement, *, allow_station_change=False):
    """Write `scene_out`: `scene_in` with the measured policy cameras/appearance. Returns a report dict."""
    tree = ET.parse(scene_in)
    report = restage_root(tree.getroot(), measurement, allow_station_change=allow_station_change)
    scene_out = Path(scene_out)
    scene_out.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree)
    tree.write(scene_out, encoding='unicode')
    report.update(scene_in=str(scene_in), scene_out=str(scene_out))
    return report


def camera_report(model, data, camera, arm_base_origin, height, width):
    """Intrinsics/extrinsics of a compiled MuJoCo camera, in world and arm_base frames, plus its table footprint."""
    cid = model.camera(camera).id
    pos = data.cam_xpos[cid].copy()
    mat = data.cam_xmat[cid].reshape(3, 3).copy()  # columns: camera x, y, z (z points backward)
    fovy = float(model.cam_fovy[cid])
    fy = height / 2 / math.tan(math.radians(fovy) / 2)
    forward = -mat[:, 2]
    rot_cv = np.column_stack((mat[:, 0], -mat[:, 1], forward))
    corners = {}
    table_z = 0.
    for label, (u, v) in {'top_left': (0, 0), 'top_right': (width, 0), 'bottom_left': (0, height),
                          'bottom_right': (width, height), 'centre': (width / 2, height / 2)}.items():
        ray = rot_cv @ np.array([(u - width / 2) / fy, (v - height / 2) / fy, 1.])
        t = (table_z - pos[2]) / ray[2] if ray[2] < -1e-9 else None
        corners[label] = None if t is None else (pos + t * ray - arm_base_origin).round(4).tolist()
    return {'scene_camera': camera, 'width': width, 'height': height, 'fovy_deg': fovy,
            'fovx_deg': math.degrees(2 * math.atan(width / 2 / fy)),
            'K': [[fy, 0, width / 2], [0, fy, height / 2], [0, 0, 1]],
            'position_world_m': pos.round(5).tolist(),
            'position_arm_base_m': (pos - arm_base_origin).round(5).tolist(),
            'rotation_cv_camera_to_arm_base': rot_cv.round(6).tolist(),
            'pitch_below_horizontal_deg': math.degrees(math.asin(max(-1., min(1., -forward[2])))),
            'yaw_deg_from_plus_y': math.degrees(math.atan2(forward[0], forward[1])),
            'height_above_table_m': float(pos[2] - table_z),
            'image_corner_rays_on_tabletop_arm_base_m': corners}


# ---------------------------------------------------------------- live recording patch

_INSTALLED = {}
# Renderers created by simulations in an installed process. A mujoco.Renderer finalized during interpreter
# shutdown segfaults in glDeleteTextures (macOS crash dialogs); close any still open before teardown.
_RENDERERS = weakref.WeakSet()


@atexit.register
def _close_renderers():
    for renderer in list(_RENDERERS):
        try:
            renderer.close()
        except Exception:
            pass


def measured_station_class(base_class, station):
    """FoldingStation subclass that keeps CLI height/setback but enforces them and the measured spacing."""
    height = float(station['base_height_above_table_m'])
    setback = float(station['base_line_to_table_edge_m'])
    spacing = float(station['base_spacing_m'])

    class MeasuredFoldingStation(base_class):
        def __init__(self, base_height, base_to_table_edge, box_from_table_edge, base_spacing=None, **kw):
            if not kw.get('reference_layout'):
                if abs(base_height - height) > 1e-6 or abs(base_to_table_edge - setback) > 1e-6:
                    raise ValueError(f'Run with --base-height {height} --base-to-table-edge {setback} '
                                     '(the measured station)')
                if abs(box_from_table_edge - CARTON_TO_EDGE_M) > 1e-6:
                    raise ValueError('Measured station assumes the carton 10 mm from the table edge')
                base_spacing = spacing
            elif base_spacing is None:
                base_spacing = base_class.__dataclass_fields__['base_spacing'].default
            super().__init__(base_height, base_to_table_edge, box_from_table_edge, base_spacing, **kw)

    MeasuredFoldingStation.__name__ = MeasuredFoldingStation.__qualname__ = 'MeasuredFoldingStation'
    return MeasuredFoldingStation


def install(source):
    """Patch this process so new folding scenes use the measured station and policy cameras."""
    import mujoco
    import carton.folding_sim as folding_sim
    import carton.folding_station as folding_station
    measurement = load_measurement(source)
    uninstall()
    _INSTALLED['FoldingStation'] = folding_station.FoldingStation
    _INSTALLED['build_scene'] = folding_sim.build_scene
    _INSTALLED['FoldingSimulation.__init__'] = folding_sim.FoldingSimulation.__init__
    shift = (measurement.get('station') or {}).get('park_targets_follow_bases', True)
    original_init = _INSTALLED['FoldingSimulation.__init__']

    def __init__(self, source_dir, out, width=960, height=720, initial_right_roll=None, initial_flaps=None,
                 initial_arm_targets=None, **kwargs):
        # Explicit start targets (the controller's --park-back) are world points chosen for 300 mm base
        # spacing; keep each target at the same place relative to its own base when the spacing differs.
        station = kwargs.get('station')
        if (shift and initial_arm_targets and station is not None and not station.reference_layout):
            dx = (station.base_spacing - REFERENCE_BASE_SPACING_M) / 2
            initial_arm_targets = {side: [float(v[0]) + (dx if side == 'right' else -dx), *map(float, v[1:])]
                                   for side, v in initial_arm_targets.items()}
        _INSTALLED['initial_arm_targets'] = initial_arm_targets
        original_init(self, source_dir, out, width, height, initial_right_roll, initial_flaps,
                      initial_arm_targets, **kwargs)

    folding_sim.FoldingSimulation.__init__ = __init__
    _INSTALLED['FoldingSimulation.render'] = folding_sim.FoldingSimulation.render
    original_render = _INSTALLED['FoldingSimulation.render']

    def render(self, *args, **kwargs):
        image = original_render(self, *args, **kwargs)
        if getattr(self, 'renderer', None) is not None:
            _RENDERERS.add(self.renderer)
        return image

    folding_sim.FoldingSimulation.render = render
    if measurement.get('station') is not None:
        folding_station.FoldingStation = measured_station_class(_INSTALLED['FoldingStation'],
                                                               measurement['station'])
    original = _INSTALLED['build_scene']

    def build_scene(source_dir, out, **kwargs):
        original(source_dir, out, **kwargs)
        path = Path(out) / 'scene.xml'
        # Cameras only: the controller registers through the table/carton markers in its own `station`
        # camera, so hiding markers or shrinking the table here would break it. Apply `appearance` to the
        # finished recordings with tools/restage_fold_scenes.py instead.
        report = restage_scene_xml(path, path, {**measurement, 'appearance': None})
        report['measurement'] = measurement
        report['initial_arm_targets_world_m'] = _INSTALLED.get('initial_arm_targets')
        (Path(out) / 'measured-station.json').write_text(json.dumps(report, indent=1))
        return mujoco.MjModel.from_xml_path(str(path))

    folding_sim.build_scene = build_scene
    return measurement


def uninstall():
    import carton.folding_sim as folding_sim
    import carton.folding_station as folding_station
    if 'FoldingStation' in _INSTALLED:
        folding_station.FoldingStation = _INSTALLED.pop('FoldingStation')
    if 'build_scene' in _INSTALLED:
        folding_sim.build_scene = _INSTALLED.pop('build_scene')
    if 'FoldingSimulation.__init__' in _INSTALLED:
        folding_sim.FoldingSimulation.__init__ = _INSTALLED.pop('FoldingSimulation.__init__')
    if 'FoldingSimulation.render' in _INSTALLED:
        folding_sim.FoldingSimulation.render = _INSTALLED.pop('FoldingSimulation.render')


def nominal_measurement():
    """The recorded simulation's own values (batch-01): restaging with these changes nothing."""
    return {
        'schema': SCHEMA,
        'note': 'Simulation nominal values (carton.folding_sim.build_scene with --base-height .12), not measurements.',
        'station': {'base_height_above_table_m': .12, 'base_line_to_table_edge_m': .15, 'base_spacing_m': .30,
                    'carton_near_wall_to_table_edge_m': .01},
        'cameras': {
            'top': {'position_m': [0., .3015, .73], 'look_at_m': [0., .3015, -.06], 'up_hint': [0., 1., 0.],
                    'fovy_deg': 48.},
            'front': {'position_m': [0., -.08, .45], 'look_at_m': [0., .2865, -.02], 'fovy_deg': 48.},
        },
    }


__all__ = ['SCHEMA', 'POLICY_CAMERAS', 'look_at_axes', 'axes_from_rotation_cv', 'rotation_cv_from_axes',
           'policy_crop', 'camera_pose', 'load_measurement', 'scene_station', 'restage_root',
           'restage_scene_xml', 'apply_appearance', 'camera_report', 'install', 'uninstall',
           'measured_station_class', 'nominal_measurement']
