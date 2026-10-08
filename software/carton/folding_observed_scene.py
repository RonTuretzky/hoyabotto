"""Observation-only collision snapshots for the bare-claw folding planner.

There is deliberately no simulator or live MjData input. A declared robot-only
model, surveyed static obstacles/base poses, fresh RGB-D estimates and encoders
are the entire input. This module never sends commands or advances dynamics.
"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


JOINT_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
FLAPS = ('short_left', 'short_right', 'long_far', 'long_near')
ENCODER_NAMES = tuple(f'{side}_{joint}' for side in ('left', 'right') for joint in JOINT_NAMES)
FRAME_CONVENTION = 'bottom_center_z_up_x_length_y_far'
OBSTACLE_MASK = str(2**31 - 1)


def _finite(value, name, minimum=0., maximum=float('inf')):
    if isinstance(value, (bool, str)) or not np.isscalar(value):
        raise ValueError(f'Finite numeric {name} required')
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} outside declared limits')
    return value


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'Explicit {name} required')
    return value


def _pose(value, name):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or np.linalg.det(pose[:3, :3]) < .999999):
        raise ValueError(f'{name} must be a finite proper rigid transform')
    return pose.copy()


def _words(values):
    return ' '.join(f'{float(value):.17g}' for value in values)


def _set_pose(element, pose):
    for key in ('quat', 'euler', 'axisangle', 'xyaxes', 'zaxis'):
        element.attrib.pop(key, None)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, pose[:3, :3].reshape(-1))
    element.set('pos', _words(pose[:3, 3]))
    element.set('quat', _words(quat))


@dataclass(frozen=True)
class RobotCollisionModel:
    """Flattened, explicit-radian robot-only MJCF and verified asset hashes."""
    xml: str
    sha256: str
    model_id: str
    measurement_id: str
    root_body_names: dict
    asset_root: str
    asset_sha256: dict
    geometry_error_m: float


@dataclass(frozen=True)
class StationBox:
    name: str
    world_from_box: object
    half_size_m: tuple
    point_error_m: float


@dataclass(frozen=True)
class MeasuredCarton:
    length_m: float
    width_m: float
    height_m: float
    flap_m: float
    thickness_m: float
    major_hinge_offset_m: float
    dimension_error_m: float
    hinge_limits_deg: dict
    flap_tag_ids: dict
    frame_convention: str
    contents_height_m: float | None


@dataclass(frozen=True)
class SceneCalibration:
    calibration_id: str
    measurement_id: str
    measurements_verified: bool
    world_frame_id: str
    clock_id: str
    sensor_id: str
    robot_id: str
    carton_id: str
    inventory_id: str
    world_from_robot_bases: dict
    base_position_error_m: float
    base_rotation_error_deg: float
    anchor_ids: tuple
    station_boxes: tuple
    carton: MeasuredCarton


@dataclass(frozen=True)
class ObservationLimits:
    max_age_s: float = .150
    max_sensor_skew_s: float = .020
    max_position_error_m: float = .012
    max_rotation_error_deg: float = 3.
    max_flap_error_deg: float = 5.
    max_encoder_error_rad: float = .020


class ObservedScene:
    """Geometric snapshot, never an executor. Use planner() to retain margins."""
    def __init__(self, model, data, metadata, robot_clearance_m):
        self.model, self.data = model, data
        self.arm_indices = {side: [int(model.joint(f'{side}_{joint}').qposadr[0])
                                   for joint in JOINT_NAMES] for side in ('left', 'right')}
        self.metadata = metadata
        self.required_robot_clearance_m = robot_clearance_m

    def require_fresh(self, *, now_s, clock_id, calibration_id):
        now = _finite(now_s, 'current timestamp')
        if clock_id != self.metadata['clock_id'] or calibration_id != self.metadata['calibration_id']:
            raise ValueError('Planning scene clock/calibration identity changed')
        if not self.metadata['captured_at_s'] <= now <= self.metadata['expires_at_s']:
            raise ValueError('Observation-derived planning scene is stale')

    def planner(self, side, *, now_s, clock_id, calibration_id, allowed_flaps=(),
                extra_clearance_m=0., seed=1):
        """Return the existing geometric planner with mandatory uncertainty margin.

        A returned path is still only a proposal. Revalidate this snapshot and
        current observations/encoders before execution; a path cannot extend
        its expiry. No hardware execution adapter is supplied here.
        """
        self.require_fresh(now_s=now_s, clock_id=clock_id, calibration_id=calibration_id)
        if side not in self.arm_indices:
            raise ValueError('Declare left or right planning arm')
        allowed = set(allowed_flaps)
        if not allowed <= {f'{name}_cardboard' for name in self.metadata['observed_flaps']}:
            raise ValueError('Only freshly observed flap faces may be allowed contacts')
        # Preserve the existing 6 mm nominal transit gate even if upstream
        # declares very small errors. This maximum is not an additive promise
        # of 6 mm physical clearance after worst-case uncertainty displacement.
        clearance = max(.006, self.required_robot_clearance_m) + _finite(extra_clearance_m, 'extra clearance', maximum=.02)
        if clearance > .02:
            raise ValueError('Combined uncertainty exceeds planner clearance capacity')
        from carton.folding_paths import JointPathPlanner
        return JointPathPlanner(self, side, allowed_flaps=allowed, seed=seed, clearance=clearance)


class ObservedSceneBuilder:
    """Validate one observation packet and construct all movable obstacle state.

    Explicit units/identity/timestamps/error bounds are mandatory. Current raw
    PixelPort readings do not yet contain this envelope and are refused.
    Missing optional flaps require unknown_flaps='swept'; default is refusal.
    Supplied error bounds are trusted upstream bounds, not estimated here.
    """
    def __init__(self, robot_model: RobotCollisionModel, calibration: SceneCalibration,
                 *, limits=ObservationLimits()):
        self.robot_model, self.calibration, self.limits = copy.deepcopy((robot_model, calibration, limits))
        self.last_observation_seq = self.last_encoder_seq = -1
        self._asset_blobs = {}
        self._validate_calibration()
        self._robot_xml = self._validated_robot_xml()

    def _validate_calibration(self):
        c, r, limits = self.calibration, self.robot_model, self.limits
        for field in ('calibration_id', 'measurement_id', 'world_frame_id', 'clock_id', 'sensor_id',
                      'robot_id', 'carton_id', 'inventory_id'):
            _identifier(getattr(c, field), field)
        for field in ('model_id', 'measurement_id'):
            _identifier(getattr(r, field), field)
        if c.measurements_verified is not True:
            raise ValueError('Verified measured model/station calibration required')
        if set(c.world_from_robot_bases) != {'left', 'right'} or set(r.root_body_names) != {'left', 'right'}:
            raise ValueError('Both measured robot base transforms and root names required')
        if len(set(r.root_body_names.values())) != 2:
            raise ValueError('Robot roots must be distinct')
        for side in ('left', 'right'):
            _pose(c.world_from_robot_bases[side], f'{side} robot base')
        _finite(c.base_position_error_m, 'base position error', maximum=.010)
        _finite(c.base_rotation_error_deg, 'base rotation error', maximum=3.)
        _finite(r.geometry_error_m, 'robot geometry error', maximum=.010)
        if not c.anchor_ids or any(isinstance(x, bool) or not isinstance(x, int) or x < 0 for x in c.anchor_ids):
            raise ValueError('Declared calibration anchor IDs required')
        for field in limits.__dataclass_fields__:
            _finite(getattr(limits, field), field, minimum=1e-9)
        b = c.carton
        if b.frame_convention != FRAME_CONVENTION:
            raise ValueError('Explicit bottom-centred carton frame convention required')
        for field in ('length_m', 'width_m', 'height_m', 'flap_m', 'thickness_m'):
            _finite(getattr(b, field), field, minimum=.000001, maximum=2.)
        if b.thickness_m >= min(b.length_m, b.width_m, b.height_m, b.flap_m) / 4:
            raise ValueError('Carton thickness inconsistent with declared dimensions')
        _finite(b.major_hinge_offset_m, 'major hinge offset', maximum=.020)
        _finite(b.dimension_error_m, 'carton dimension error', maximum=.010)
        if b.contents_height_m is not None:
            _finite(b.contents_height_m, 'declared conservative contents height', minimum=b.thickness_m, maximum=2.)
        if set(b.hinge_limits_deg) != set(FLAPS) or set(b.flap_tag_ids) != set(FLAPS):
            raise ValueError('Declare all four hinge limits and flap marker identities')
        if len(set(b.flap_tag_ids.values())) != 4:
            raise ValueError('Flap marker identities must be distinct')
        if any(isinstance(tag, bool) or not isinstance(tag, int) or tag < 0 for tag in b.flap_tag_ids.values()):
            raise ValueError('Flap marker identities must be nonnegative integers')
        for name, bounds in b.hinge_limits_deg.items():
            if len(bounds) != 2:
                raise ValueError('Two measured hinge limits required')
            lo = _finite(bounds[0], name + ' lower limit', -180., 180.)
            hi = _finite(bounds[1], name + ' upper limit', -180., 180.)
            if lo >= hi:
                raise ValueError('Hinge limits must be ordered')
        if not c.station_boxes:
            raise ValueError('Explicit complete surveyed station obstacle inventory required')
        names = set()
        for obstacle in c.station_boxes:
            if (not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', obstacle.name)
                    or obstacle.name in names or obstacle.name.startswith(('left_', 'right_', 'carton_', 'short_', 'long_'))):
                raise ValueError('Unique non-robot station obstacle names required')
            names.add(obstacle.name)
            _pose(obstacle.world_from_box, obstacle.name)
            if len(obstacle.half_size_m) != 3:
                raise ValueError('Three station box half sizes in metres required')
            for value in obstacle.half_size_m:
                _finite(value, 'station half size', minimum=1e-6, maximum=10.)
            _finite(obstacle.point_error_m, 'station point error', maximum=.020)

    def _validated_robot_xml(self):
        r = self.robot_model
        if not isinstance(r.xml, str) or hashlib.sha256(r.xml.encode()).hexdigest() != r.sha256:
            raise ValueError('Declared robot-model source hash mismatch')
        root = ET.fromstring(r.xml)
        compiler = root.find('compiler')
        if root.tag != 'mujoco' or compiler is None or compiler.get('angle') != 'radian':
            raise ValueError('Robot model must explicitly use radian units')
        if root.findall('.//include') or root.find('equality') is not None or root.findall('.//freejoint'):
            raise ValueError('Flattened robot-only model without free bodies or equality constraints required')
        world = root.find('worldbody')
        if world is None or any(child.tag != 'body' for child in world):
            raise ValueError('Robot template may contain only its two robot roots')
        bodies = {child.get('name'): child for child in world}
        if set(bodies) != set(r.root_body_names.values()):
            raise ValueError('Robot template includes missing roots or undeclared scene obstacles')
        for side, root_name in r.root_body_names.items():
            _set_pose(bodies[root_name], _pose(self.calibration.world_from_robot_bases[side], root_name))
            joints = bodies[root_name].findall('.//joint')
            if {joint.get('name') for joint in joints} != {f'{side}_{joint}' for joint in JOINT_NAMES} or len(joints) != 6:
                raise ValueError('Exactly six named robot joints per arm required')
            if any(joint.get('type', 'hinge') != 'hinge' for joint in joints):
                raise ValueError('Only measured rotational robot joints are supported')
            if any(not geom.get('name', '').startswith(side + '_') for geom in bodies[root_name].iter('geom')):
                raise ValueError('Robot collision geoms must identify their arm')
        # Imported contact settings must never name generated obstacles or
        # suppress arm/arm collisions. Keep only declared adjacent-body
        # exclusions within each robot, as in the measured SO101 model.
        adjacent = set()
        for arm in bodies.values():
            for body in arm.iter('body'):
                adjacent.update(frozenset((body.get('name'), child.get('name')))
                                for child in body.findall('body'))
        for contact in root.findall('contact'):
            for entry in contact:
                if (entry.tag != 'exclude'
                        or frozenset((entry.get('body1'), entry.get('body2'))) not in adjacent):
                    raise ValueError('Only adjacent robot-body contact exclusions are supported')
        # The snapshot is geometric only. Actuator/sensor/user callbacks cannot
        # supply object state; their XML definitions are unnecessary here.
        for tag in ('actuator', 'sensor', 'keyframe', 'extension'):
            for child in root.findall(tag):
                root.remove(child)
        assets = root.find('asset')
        if assets is not None:
            for asset in assets:
                if asset.tag not in ('mesh', 'texture', 'material'):
                    raise ValueError('Unsupported robot asset type')
                file = asset.get('file')
                if file is None:
                    continue
                directory = compiler.get('meshdir' if asset.tag == 'mesh' else 'texturedir', '')
                path = (Path(r.asset_root) / directory / file).resolve()
                expected = r.asset_sha256.get(str(path))
                content = path.read_bytes()
                if expected is None or hashlib.sha256(content).hexdigest() != expected:
                    raise ValueError(f'Unverified robot collision asset: {path.name}')
                frozen_name = expected[:16] + '-' + path.name
                self._asset_blobs[frozen_name] = content
                asset.set('file', frozen_name)
        compiler.attrib.pop('meshdir', None); compiler.attrib.pop('texturedir', None)
        return ET.tostring(root, encoding='unicode')

    def _packet(self, packet, kind, now):
        if not isinstance(packet, dict):
            raise ValueError(f'Explicit {kind} packet required')
        seq = packet.get('seq')
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
            raise ValueError(f'Positive integer {kind} sequence required')
        previous = self.last_observation_seq if kind == 'observation' else self.last_encoder_seq
        if seq <= previous:
            raise ValueError(f'Stale {kind} sequence')
        for key in ('clock_id', 'calibration_id'):
            if packet.get(key) != getattr(self.calibration, key):
                raise ValueError(f'{kind} {key} mismatch')
        stamp = _finite(packet.get('timestamp_s'), kind + ' timestamp')
        if stamp > now or now - stamp > self.limits.max_age_s:
            raise ValueError(f'Stale or future {kind} timestamp')
        return seq, stamp

    def build(self, observation, encoders, *, now_s, required_flaps=FLAPS, unknown_flaps='refuse'):
        c, b, limits = self.calibration, self.calibration.carton, self.limits
        now = _finite(now_s, 'current timestamp')
        seq, stamp = self._packet(observation, 'observation', now)
        encoder_seq, encoder_stamp = self._packet(encoders, 'encoders', now)
        required = set(required_flaps)
        if not required <= set(FLAPS) or unknown_flaps not in ('refuse', 'swept'):
            raise ValueError('Declare supported required flaps and unknown-flap policy')
        for key, expected in (('source', 'calibrated_rgbd'), ('length_unit', 'm'), ('angle_unit', 'deg'),
                              ('world_frame_id', c.world_frame_id), ('sensor_id', c.sensor_id),
                              ('carton_id', c.carton_id), ('inventory_id', c.inventory_id)):
            if observation.get(key) != expected:
                raise ValueError(f'Observation {key} mismatch or missing')
        if observation.get('unmodelled_obstacles') != []:
            raise ValueError('Unmodelled or unverified obstacle inventory; refuse planning')
        depth_stamp = _finite(observation.get('depth_timestamp_s'), 'depth timestamp')
        observation_bound_expiry = _finite(observation.get('uncertainty_valid_until_s'), 'observation bound expiry')
        encoder_bound_expiry = _finite(encoders.get('uncertainty_valid_until_s'), 'encoder bound expiry')
        if min(observation_bound_expiry, encoder_bound_expiry) < now:
            raise ValueError('Expired observation/encoder uncertainty bounds')
        if (depth_stamp > now or now - depth_stamp > limits.max_age_s
                or abs(stamp - depth_stamp) > limits.max_sensor_skew_s
                or abs(stamp - encoder_stamp) > limits.max_sensor_skew_s):
            raise ValueError('RGB/depth/encoders are stale or unsynchronized')
        _pose(observation.get('world_from_camera'), 'Observed camera pose')
        box_pose = _pose(observation.get('world_from_box'), 'Observed carton pose')
        registration = observation.get('box_registration', {})
        if (registration.get('source_seq') != seq or registration.get('identity_verified') is not True
                or registration.get('ambiguous') is not False):
            raise ValueError('Fresh unambiguous carton identity/registration required')
        anchors = observation.get('anchor_ids')
        if not isinstance(anchors, (tuple, list)) or not anchors or not set(anchors) <= set(c.anchor_ids):
            raise ValueError('Fresh declared calibration anchor observations required')
        _finite(observation.get('anchor_fit_rms_mm'), 'anchor fit RMS', maximum=6.)
        position_error = _finite(registration.get('position_error_m'), 'carton position error', maximum=limits.max_position_error_m)
        rotation_error = _finite(registration.get('rotation_error_deg'), 'carton rotation error', maximum=limits.max_rotation_error_deg)
        angles = observation.get('angles')
        if not isinstance(angles, dict) or not set(angles) <= set(FLAPS):
            raise ValueError('Named flap observation map required')
        observed, unknown = {}, {}
        for name in FLAPS:
            row = angles.get(name)
            try:
                if (not isinstance(row, dict) or row.get('observed_seq') != seq
                        or row.get('identity') != name or row.get('unambiguous') is not True):
                    raise ValueError('missing, stale or ambiguous flap identity')
                method = row.get('method')
                if method == 'apriltag_aligned_depth_plane':
                    if c.carton.flap_tag_ids[name] not in observation.get('tags', []):
                        raise ValueError('identified flap tag missing from this frame')
                elif method == 'aligned_depth_hinge_consistent_plane' and name.startswith('long_'):
                    _finite(row.get('pixel_support'), 'plane pixel support', minimum=60)
                    _finite(row.get('supported_patches'), 'plane spatial support', minimum=6)
                    _finite(row.get('hinge_axis_error_deg'), 'hinge axis error', maximum=4.)
                    _finite(abs(row.get('hinge_plane_offset_mm', float('inf'))), 'hinge plane offset', maximum=6.)
                else:
                    raise ValueError('unsupported or identity-ambiguous flap observation method')
                lo, hi = b.hinge_limits_deg[name]
                angle = _finite(row.get('degrees'), name + ' angle', lo, hi)
                error = _finite(row.get('error_bound_deg'), name + ' angle error', maximum=limits.max_flap_error_deg)
                if angle - error < lo or angle + error > hi:
                    raise ValueError('flap uncertainty extends beyond calibrated hinge limits')
                observed[name] = (angle, error)
            except (ValueError, TypeError) as exc:
                if name in required or unknown_flaps == 'refuse':
                    raise ValueError(f'{name}: {exc}') from exc
                unknown[name] = str(exc)
        if (encoders.get('robot_id') != c.robot_id or encoders.get('model_id') != self.robot_model.model_id
                or encoders.get('angle_unit') != 'rad' or encoders.get('position_frame') != 'model_joint_coordinates'):
            raise ValueError('Encoder robot/model identity, frame or radian units missing')
        positions, errors = encoders.get('joint_positions'), encoders.get('error_bounds_rad')
        if not isinstance(positions, dict) or set(positions) != set(ENCODER_NAMES) or not isinstance(errors, dict) or set(errors) != set(ENCODER_NAMES):
            raise ValueError('All twelve named encoder positions and uncertainty bounds required')
        positions = {name: _finite(value, name + ' encoder', -math.pi * 2, math.pi * 2) for name, value in positions.items()}
        errors = {name: _finite(value, name + ' encoder error', maximum=limits.max_encoder_error_rad) for name, value in errors.items()}
        root = ET.fromstring(self._robot_xml)
        world = root.find('worldbody')
        for obstacle in c.station_boxes:
            geom = ET.SubElement(world, 'geom', name=obstacle.name, type='box',
                                 size=_words(np.asarray(obstacle.half_size_m) + obstacle.point_error_m),
                                 contype=OBSTACLE_MASK, conaffinity=OBSTACLE_MASK, gap='0')
            _set_pose(geom, _pose(obstacle.world_from_box, obstacle.name))
        carton = ET.SubElement(world, 'body', name='carton')
        _set_pose(carton, box_pose)
        ET.SubElement(carton, 'freejoint', name='carton_free')
        # Bounding radius covers every carton/flap point for any hinge angle.
        radius = np.linalg.norm([b.length_m / 2 + b.flap_m, b.width_m / 2 + b.flap_m,
                                 max(b.height_m + b.flap_m + b.major_hinge_offset_m,
                                     b.contents_height_m or 0.)]) + b.thickness_m
        # Six independent dimension errors can move the hinge, edge and face
        # together. Summing their displacement bounds is conservative under
        # any rotation; treating one dimension error as the total is not.
        pose_padding = position_error + 2 * radius * math.sin(math.radians(rotation_error) / 2) + 6 * b.dimension_error_m
        t, length, width, height = b.thickness_m / 2, b.length_m, b.width_m, b.height_m
        def geom(parent, name, center, half_size, padding):
            ET.SubElement(parent, 'geom', name=name, type='box', pos=_words(center),
                          size=_words(np.asarray(half_size) + padding), mass='.01',
                          quat='1 0 0 0', contype=OBSTACLE_MASK, conaffinity=OBSTACLE_MASK, gap='0')
        geom(carton, 'bottom', [0, 0, t], [length / 2, width / 2, t], pose_padding)
        if b.contents_height_m is not None:
            geom(carton, 'contents_conservative', [0, 0, b.contents_height_m / 2],
                 [length / 2, width / 2, b.contents_height_m / 2], pose_padding)
        for name, center, size in (
                ('wall_left', [-length / 2, 0, height / 2], [t, width / 2, height / 2]),
                ('wall_right', [length / 2, 0, height / 2], [t, width / 2, height / 2]),
                ('wall_near', [0, -width / 2, height / 2], [length / 2, t, height / 2]),
                ('wall_far', [0, width / 2, height / 2], [length / 2, t, height / 2])):
            geom(carton, name, center, size, pose_padding)
        hinges = {'short_left':([-length / 2, 0, height], [0, 1, 0], 0),
                  'short_right':([length / 2, 0, height], [0, -1, 0], 0),
                  'long_far':([0, width / 2, height + b.major_hinge_offset_m], [1, 0, 0], 1),
                  'long_near':([0, -width / 2, height + b.major_hinge_offset_m], [-1, 0, 0], 1)}
        for name, (center, axis, major) in hinges.items():
            if name in unknown:
                size = ([b.flap_m + t, width / 2 + t, b.flap_m + t] if not major
                        else [length / 2 + t, b.flap_m + t, b.flap_m + t])
                # Full sweep, including outward and inverted positions. There
                # is intentionally no guessed hinge joint or allowed face.
                geom(carton, name + '_unobserved_sweep', center, size, pose_padding)
                continue
            body = ET.SubElement(carton, 'body', name=name, pos=_words(center))
            ET.SubElement(body, 'joint', name=name + '_hinge', type='hinge', axis=_words(axis),
                          pos='0 0 0', ref='0', limited='true', range=_words(np.radians(b.hinge_limits_deg[name])))
            flap_padding = pose_padding + 2 * (b.flap_m + t) * math.sin(math.radians(observed[name][1]) / 2)
            geom(body, name + '_cardboard', [0, 0, b.flap_m / 2],
                 [length / 2, t, b.flap_m / 2] if major else [t, width / 2, b.flap_m / 2], flap_padding)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), assets=self._asset_blobs)
        if model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
            raise ValueError('Robot template disables collision detection')
        if np.any(model.geom_contype < 0) or np.any(model.geom_conaffinity < 0):
            raise ValueError('Only nonnegative 31-bit collision masks are supported')
        active = {}
        for side in ('left', 'right'):
            active[side] = [gid for gid in range(model.ngeom)
                            if model.geom(gid).name.startswith(side + '_')
                            and (model.geom_contype[gid] or model.geom_conaffinity[gid])]
            if not active[side]:
                raise ValueError(f'{side} robot has no active collision geometry')
        for left in active['left']:
            if any(not ((model.geom_contype[left] & model.geom_conaffinity[right])
                        or (model.geom_contype[right] & model.geom_conaffinity[left]))
                   for right in active['right']):
                raise ValueError('Robot masks suppress arm/arm collisions')
        data = mujoco.MjData(model)
        # qpos0 is never copied from a simulator. Every DOF in this newly
        # compiled, bounded inventory is assigned from exposed measurements.
        assigned = set()
        for name, angle in positions.items():
            joint = model.joint(name); adr = int(joint.qposadr[0]); lo, hi = joint.range
            if joint.type[0] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError('Compiled robot joints must use rotational coordinates')
            if not bool(joint.limited[0]) or angle - errors[name] < lo or angle + errors[name] > hi:
                raise ValueError(f'{name}: encoder interval outside model joint limits')
            data.qpos[adr] = angle; assigned.add(adr)
        joint = model.joint('carton_free'); adr = int(joint.qposadr[0])
        data.qpos[adr:adr + 3] = box_pose[:3, 3]
        mujoco.mju_mat2Quat(data.qpos[adr + 3:adr + 7], box_pose[:3, :3].ravel())
        assigned.update(range(adr, adr + 7))
        for name, (angle, _) in observed.items():
            adr = int(model.joint(name + '_hinge').qposadr[0]); data.qpos[adr] = math.radians(angle); assigned.add(adr)
        if assigned != set(range(model.nq)) or model.neq or model.nu:
            raise ValueError('Scene includes unobserved DOFs, constraints or actuators')
        # A global reach bound upper-bounds motion of every arm mesh point
        # under any permitted robot pose. The existing planner's margin AND
        # signed-distance threshold both receive the resulting bound.
        # Compensate the existing planner's 0.1mm signed-distance tolerance
        # so it cannot consume any of the declared uncertainty allowance.
        robot_clearance = .0001
        for side, root_name in self.robot_model.root_body_names.items():
            root_id = model.body(root_name).id; reach = 0.
            for gid in range(model.ngeom):
                body_id = int(model.geom_bodyid[gid]); branch = []
                while body_id and body_id != root_id:
                    branch.append(body_id); body_id = int(model.body_parentid[body_id])
                if body_id != root_id:
                    continue
                bound = float(np.linalg.norm(model.geom_pos[gid]) + model.geom_rbound[gid])
                for body_id in [root_id, *branch]:
                    if body_id != root_id:
                        bound += float(np.linalg.norm(model.body_pos[body_id]))
                    for jid in np.flatnonzero(model.jnt_bodyid == body_id):
                        bound += 2 * float(np.linalg.norm(model.jnt_pos[jid]))
                reach = max(reach, bound)
            angular = sum(errors[f'{side}_{joint}'] for joint in JOINT_NAMES) + math.radians(c.base_rotation_error_deg)
            # Arm/arm separation must cover the uncertainty of BOTH arms.
            # The sum also conservatively covers arm/environment contacts.
            robot_clearance += c.base_position_error_m + self.robot_model.geometry_error_m + reach * angular
        if robot_clearance > .020:
            raise ValueError('Robot uncertainty exceeds planner clearance capacity')
        mujoco.mj_kinematics(model, data)
        metadata = {'source': 'fresh_calibrated_rgbd_and_named_encoders', 'observation_seq': seq,
                    'encoder_seq': encoder_seq, 'calibration_id': c.calibration_id, 'clock_id': c.clock_id,
                    'robot_model_sha256': self.robot_model.sha256, 'inventory_id': c.inventory_id,
                    'captured_at_s': max(stamp, depth_stamp, encoder_stamp),
                    'expires_at_s': min(min(stamp, depth_stamp, encoder_stamp) + limits.max_age_s,
                                        observation_bound_expiry, encoder_bound_expiry),
                    'observed_flaps': sorted(observed), 'conservative_sweeps': unknown,
                    'carton_pose_padding_m': float(pose_padding), 'robot_clearance_m': robot_clearance,
                    'hardware_commands': False, 'dynamics_validated': False}
        digest_payload = {'xml': ET.tostring(root, encoding='unicode'), 'qpos': data.qpos.tolist(), 'metadata': metadata}
        metadata['scene_sha256'] = hashlib.sha256(json.dumps(digest_payload, sort_keys=True).encode()).hexdigest()
        self.last_observation_seq, self.last_encoder_seq = seq, encoder_seq
        return ObservedScene(model, data, metadata, robot_clearance)
