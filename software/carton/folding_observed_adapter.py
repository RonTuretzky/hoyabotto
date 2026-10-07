"""Explicitly offline PixelPort/encoder adapter for observed collision scenes.

This accepts exposed observation records and named encoder captures, never a
simulation, MjData, object qpos, truth angles, or grasp-result input. Times,
calibration identities, complete obstacle inventory and error horizons are
declared simulation assumptions, not measured physical guarantees. Existing
PixelPort observations alone do not establish them.

Tag/anchor fit residuals only check internal image/depth consistency. They are
never substituted for camera-transform or carton-pose accuracy bounds. There
is currently no authoritative measured physical station profile for this
adapter; source camera mounts and rear robot spacing remain offline assumptions.

The separate export_offline_robot_model helper extracts only static robot
model declarations from scene XML. It is offline setup, not observation.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from carton.geometry import Box
from carton.folding_markers import BOX_MARKERS
from carton.folding_observed_scene import (
    ENCODER_NAMES, FLAPS, ObservedSceneBuilder, RobotCollisionModel, _finite, _pose,
)


def _offline_id(value, name):
    if not isinstance(value, str) or not value.startswith('offline:') or not value[8:].strip():
        raise ValueError(f'Explicit offline: identity required for {name}')
    return value


def _ids(value, name):
    if (not isinstance(value, (list, tuple)) or len(set(value)) != len(value)
            or any(isinstance(x, bool) or not isinstance(x, int) or x < 0 for x in value)):
        raise ValueError(f'Unique integer {name} required')
    return set(value)


def _marker_declaration_digest():
    return hashlib.sha256(json.dumps(BOX_MARKERS, sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class OfflineObservationAssumptions:
    run_id: str
    assumption_id: str
    camera_name: str
    carton_position_error_m: float
    carton_rotation_error_deg: float
    flap_error_bounds_deg: dict
    encoder_error_bounds_rad: dict
    uncertainty_horizon_s: float
    allow_paired_short_hinge_registration: bool


@dataclass(frozen=True)
class OfflineSensorCapture:
    """Copy these at capture; do not fill them from a later simulator state.

    world_from_camera is the pixel observer's calibrated pose, not a renderer
    camera-body pose. joint_positions contains only twelve named robot joints.
    All times use the explicitly declared simulation clock.
    """
    run_id: str
    clock_id: str
    observation_seq: int
    rgb_timestamp_s: float
    depth_timestamp_s: float
    world_from_camera: object
    encoder_seq: int
    encoder_timestamp_s: float
    joint_positions: dict


@dataclass(frozen=True)
class OfflineScenePackets:
    observation: dict
    encoders: dict
    audit: dict


@dataclass(frozen=True)
class OfflineSceneBuild:
    scene: object
    packets: OfflineScenePackets


class OfflinePixelSceneAdapter:
    """Bridge PixelPort's current output schema using explicit offline bounds.

    build() requires both a required-flap set and an unknown-flap policy. An
    unsupported legacy cardboard row remains unobserved. The robot/station
    builder's verification flag attests to the declared offline model only;
    all identities and returned scene metadata explicitly forbid interpreting
    this as a verified physical calibration.
    """
    def __init__(self, builder: ObservedSceneBuilder, assumptions: OfflineObservationAssumptions):
        self.builder = builder
        self.assumptions = copy.deepcopy(assumptions)
        self._marker_ids = set(BOX_MARKERS)
        self._marker_digest = _marker_declaration_digest()
        c, r, a = builder.calibration, builder.robot_model, self.assumptions
        for field in ('calibration_id', 'measurement_id', 'world_frame_id', 'clock_id', 'sensor_id',
                      'robot_id', 'carton_id', 'inventory_id'):
            _offline_id(getattr(c, field), field)
        for field in ('model_id', 'measurement_id'):
            _offline_id(getattr(r, field), 'robot ' + field)
        for field in ('run_id', 'assumption_id'):
            _offline_id(getattr(a, field), field)
        if not isinstance(a.camera_name, str) or not a.camera_name.strip():
            raise ValueError('Explicit PixelPort camera name required')
        if not isinstance(a.allow_paired_short_hinge_registration, bool):
            raise ValueError('Declare paired-short-hinge registration policy')
        _finite(a.carton_position_error_m, 'declared carton position error', maximum=builder.limits.max_position_error_m)
        _finite(a.carton_rotation_error_deg, 'declared carton rotation error', maximum=builder.limits.max_rotation_error_deg)
        _finite(a.uncertainty_horizon_s, 'declared uncertainty horizon', minimum=1e-9,
                maximum=builder.limits.max_age_s)
        for values, names, maximum in ((a.flap_error_bounds_deg, FLAPS, builder.limits.max_flap_error_deg),
                                      (a.encoder_error_bounds_rad, ENCODER_NAMES, builder.limits.max_encoder_error_rad)):
            if not isinstance(values, dict) or set(values) != set(names):
                raise ValueError('Explicit error bounds for every flap and named robot encoder required')
            for name, value in values.items():
                _finite(value, name + ' declared error', maximum=maximum)
        # Current PixelPort embeds this declared carton/marker geometry. A
        # differently sized model needs an estimator configured for it first.
        box = Box()
        for field, expected in (('length_m', box.length), ('width_m', box.width),
                                ('height_m', box.height), ('flap_m', box.flap),
                                ('thickness_m', .003), ('major_hinge_offset_m', .0035)):
            if not np.isclose(getattr(c.carton, field), expected, rtol=0, atol=1e-12):
                raise ValueError('Declared carton differs from current PixelPort task geometry')
        if c.carton.flap_tag_ids != dict(zip(FLAPS, (11, 12, 13, 14))):
            raise ValueError('Declared flap identities differ from current PixelPort marker mapping')

    def adapt(self, reading, observer_history_row, capture: OfflineSensorCapture, *,
              inventory_complete, unmodelled_obstacles):
        """Create auditable packets without changing the builder's sequence.

        history is the matching RGBDTagObserver.history row, not arbitrary
        quality from another frame. The caller explicitly supplies the closed
        simulated inventory assertion; no missing-obstacle detection is claimed.
        """
        a, c = self.assumptions, self.builder.calibration
        if _marker_declaration_digest() != self._marker_digest:
            raise ValueError('PixelPort marker declarations changed after adapter creation')
        reading, history, capture = copy.deepcopy((reading, observer_history_row, capture))
        if not isinstance(reading, dict) or not isinstance(history, dict):
            raise ValueError('PixelPort reading and same-frame observer history required')
        seq = reading.get('seq')
        if (isinstance(seq, bool) or not isinstance(seq, int) or seq < 1
                or isinstance(history.get('seq'), bool) or isinstance(capture.observation_seq, bool)
                or seq != history.get('seq') or seq != capture.observation_seq):
            raise ValueError('PixelPort, observer and camera capture sequences must match')
        if (capture.run_id != a.run_id or capture.clock_id != c.clock_id
                or reading.get('camera') != a.camera_name):
            raise ValueError('Offline run, capture clock or PixelPort camera identity mismatch')
        if reading.get('privileged_mechanics_probe'):
            raise ValueError('Privileged mechanics observations cannot enter the pixel scene adapter')
        if reading.get('source') not in (None, 'calibrated_rgbd'):
            raise ValueError('PixelPort source is not calibrated RGB-D')
        if reading.get('paddle') is not None:
            raise ValueError('Observed paddle is outside the declared bare-claw obstacle model')
        if (inventory_complete is not True or unmodelled_obstacles != []
                or reading.get('unmodelled_obstacles', []) != []):
            raise ValueError('Explicit complete simulated obstacle inventory required')
        tags = _ids(reading.get('tags'), 'detected tag IDs')
        if tags != _ids(history.get('detected'), 'observer detected IDs'):
            raise ValueError('PixelPort and observer detections disagree')
        anchors = _ids(history.get('anchor_ids'), 'anchor IDs')
        if not anchors or not anchors <= tags or not anchors <= set(c.anchor_ids):
            raise ValueError('Matching fresh declared anchors required')
        quality = history.get('quality')
        if not isinstance(quality, dict):
            raise ValueError('Fresh aligned-depth tag quality required')
        # JSON history round-trips turn integer dictionary keys into strings.
        def quality_for(tag):
            if tag in quality and str(tag) in quality:
                raise ValueError('Ambiguous duplicate quality keys')
            q = quality.get(tag, quality.get(str(tag)))
            if not isinstance(q, dict):
                raise ValueError('Detected tag lacks same-frame depth quality')
            _finite(q.get('valid_depth_pixels'), 'tag depth support', minimum=30)
            _finite(q.get('square_fit_rms_mm'), 'tag square fit', maximum=4.)
            _finite(q.get('plane_rms_mm'), 'tag plane RMS')
        for tag in tags:
            quality_for(tag)
        rejected = history.get('rejected', {})
        if not isinstance(rejected, dict) or any(tag in rejected or str(tag) in rejected for tag in tags):
            raise ValueError('Detected tag is also rejected in this frame')
        anchor_fit = _finite(history.get('anchor_fit_rms_mm'), 'anchor fit', maximum=6.)
        registration = reading.get('box_registration')
        if not isinstance(registration, dict):
            raise ValueError('Fresh carton marker registration evidence required')
        if (registration.get('ambiguous', False) is not False
                or registration.get('identity_verified', True) is not True
                or registration.get('source_seq', seq) != seq):
            raise ValueError('Carton registration explicitly stale or ambiguous')
        visible = _ids(registration.get('visible_ids'), 'carton registration IDs')
        if not visible or not visible <= tags:
            raise ValueError('Carton registration markers missing from current image')
        if visible <= self._marker_ids:
            if registration.get('source') not in (None, 'fresh_carton_wall_tags'):
                raise ValueError('Unsupported carton wall registration source')
            if registration.get('selected_id') not in visible:
                raise ValueError('Fresh selected carton wall marker required')
            translation = _finite(registration.get('max_translation_disagreement_mm'),
                                  'carton marker disagreement', maximum=12.) / 1000
            rotation = _finite(registration.get('max_rotation_disagreement_degrees'),
                               'carton marker rotation disagreement', maximum=8.)
            registration_method = 'fresh_carton_wall_tags'
        elif (a.allow_paired_short_hinge_registration and visible == {11, 12}
              and registration.get('selected_id') is None
              and registration.get('source') == 'paired short-flap hinges from fresh RGB-D tags'):
            translation = _finite(registration.get('hinge_spacing_error_mm'),
                                  'short-hinge spacing error', maximum=12.) / 1000
            rotation = max(_finite(registration.get(field), field, maximum=8.) for field in
                           ('hinge_axis_disagreement_degrees', 'hinge_perpendicular_error_degrees'))
            registration_method = 'fresh_paired_short_hinge_tags'
        else:
            raise ValueError('Unrecognized, ambiguous or disabled carton registration method')
        angles = reading.get('angles')
        if not isinstance(angles, dict) or not set(angles) <= set(FLAPS):
            raise ValueError('Named PixelPort flap observations required')
        accepted, omitted = {}, {}
        for name in FLAPS:
            row = angles.get(name)
            if row is None:
                omitted[name] = 'not observed in this PixelPort frame'
                continue
            if not isinstance(row, dict):
                raise ValueError('Malformed PixelPort flap row')
            if row.get('unambiguous', True) is not True or row.get('observed_seq', seq) != seq:
                omitted[name] = 'explicitly ambiguous or stale flap observation'
                continue
            method = row.get('method')
            if isinstance(method, str) and 'PRIVILEGED' in method.upper():
                raise ValueError('Privileged flap angles cannot enter the pixel scene adapter')
            if method == 'apriltag_aligned_depth_plane':
                if c.carton.flap_tag_ids[name] not in tags:
                    raise ValueError('Tag-derived flap lacks its current decoded identity')
                result = {'method': method}
            elif method == 'aligned_depth_hinge_consistent_plane' and name.startswith('long_'):
                result = {key: row.get(key) for key in ('method', 'pixel_support', 'supported_patches',
                           'hinge_axis_error_deg', 'hinge_plane_offset_mm')}
            else:
                omitted[name] = 'unsupported or identity-ambiguous PixelPort method'
                continue
            angle = _finite(row.get('degrees'), name + ' PixelPort angle', minimum=-40., maximum=103.)
            if not -40 < angle < 103:
                raise ValueError('Flap angle outside current PixelPort observation gates')
            result.update(degrees=angle, identity=name, observed_seq=seq, unambiguous=True,
                          error_bound_deg=a.flap_error_bounds_deg[name])
            accepted[name] = result
        rgb_time = _finite(capture.rgb_timestamp_s, 'simulated RGB capture time')
        depth_time = _finite(capture.depth_timestamp_s, 'simulated depth capture time')
        encoder_time = _finite(capture.encoder_timestamp_s, 'simulated encoder capture time')
        if not isinstance(capture.joint_positions, dict) or set(capture.joint_positions) != set(ENCODER_NAMES):
            raise ValueError('Exactly twelve named robot encoder coordinates required; no object qpos')
        observation = dict(seq=seq, timestamp_s=rgb_time, depth_timestamp_s=depth_time,
            uncertainty_valid_until_s=min(rgb_time, depth_time) + a.uncertainty_horizon_s,
            clock_id=c.clock_id, calibration_id=c.calibration_id, source='calibrated_rgbd',
            length_unit='m', angle_unit='deg', world_frame_id=c.world_frame_id, sensor_id=c.sensor_id,
            carton_id=c.carton_id, inventory_id=c.inventory_id, unmodelled_obstacles=[],
            world_from_camera=_pose(capture.world_from_camera, 'pixel-derived camera pose').tolist(),
            world_from_box=_pose(reading.get('world_from_box'), 'pixel-derived carton pose').tolist(),
            box_registration=dict(source_seq=seq, identity_verified=True, ambiguous=False,
                position_error_m=max(a.carton_position_error_m, translation),
                rotation_error_deg=max(a.carton_rotation_error_deg, rotation)),
            anchor_ids=sorted(anchors), anchor_fit_rms_mm=anchor_fit, tags=sorted(tags), angles=accepted,
            simulation_only=True, physical_calibration_verified=False, assumption_id=a.assumption_id)
        encoders = dict(seq=capture.encoder_seq, timestamp_s=encoder_time,
            uncertainty_valid_until_s=encoder_time + a.uncertainty_horizon_s,
            clock_id=c.clock_id, calibration_id=c.calibration_id, robot_id=c.robot_id,
            model_id=self.builder.robot_model.model_id, angle_unit='rad', position_frame='model_joint_coordinates',
            joint_positions=copy.deepcopy(capture.joint_positions), error_bounds_rad=copy.deepcopy(a.encoder_error_bounds_rad),
            simulation_only=True, physical_calibration_verified=False, assumption_id=a.assumption_id)
        audit = dict(simulation_only=True, physical_calibration_verified=False, hardware_commands=False,
            run_id=a.run_id, assumption_id=a.assumption_id, observation_seq=seq,
            registration_method=registration_method, omitted_flaps=omitted,
            pixelport_marker_geometry_sha256=self._marker_digest,
            uncertainty_source='explicit offline assumptions, not physical measurements or inferred confidence',
            uncertainty_horizon_s=a.uncertainty_horizon_s,
            object_state_source='PixelPort RGB-D estimates only', encoder_source='named captured robot joint coordinates')
        return OfflineScenePackets(observation, encoders, audit)

    def build(self, reading, observer_history_row, capture, *, now_s, required_flaps,
              unknown_flaps, inventory_complete, unmodelled_obstacles):
        packets = self.adapt(reading, observer_history_row, capture,
                             inventory_complete=inventory_complete, unmodelled_obstacles=unmodelled_obstacles)
        scene = self.builder.build(packets.observation, packets.encoders, now_s=now_s,
                                   required_flaps=required_flaps, unknown_flaps=unknown_flaps)
        # Preserve the builder digest and bind offline provenance into a new
        # deterministic digest. The scene itself carries the evidence boundary
        # even if a caller stores it separately from this return wrapper.
        scene.metadata['builder_scene_sha256'] = scene.metadata['scene_sha256']
        scene.metadata.update(simulation_only=True, physical_calibration_verified=False,
                              offline_adapter=copy.deepcopy(packets.audit))
        payload = {key: value for key, value in scene.metadata.items() if key != 'scene_sha256'}
        scene.metadata['scene_sha256'] = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
        return OfflineSceneBuild(scene, packets)


def export_offline_robot_model(scene_xml, *, root_body_names, asset_root, model_id,
                               measurement_id, geometry_error_m):
    """Extract static robot declarations from flattened simulation XML only.

    No qpos, keyframes, scene obstacle bodies, actuators or sensor data are
    exported. Base transforms are subsequently supplied by an explicit offline
    calibration; carton/flap poses must come from pixel observations. This
    source-model operation does not establish physical robot calibration.
    """
    _offline_id(model_id, 'robot model'); _offline_id(measurement_id, 'robot measurement declaration')
    _finite(geometry_error_m, 'declared offline robot geometry error', maximum=.01)
    if (not isinstance(scene_xml, str) or set(root_body_names) != {'left', 'right'}
            or len(set(root_body_names.values())) != 2
            or any(not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) for name in root_body_names.values())):
        raise ValueError('Static XML and two explicit robot root names required')
    source = ET.fromstring(scene_xml)
    if source.tag != 'mujoco' or source.findall('.//include'):
        raise ValueError('Flattened offline MJCF source required')
    root = ET.Element('mujoco', model='offline_observed_robot_template')
    for tag in ('compiler', 'asset', 'default', 'option'):
        root.extend(copy.deepcopy(source.findall(tag)))
    compiler = root.find('compiler')
    if compiler is None or compiler.get('angle') != 'radian':
        raise ValueError('Explicit radian robot template required')
    world = ET.SubElement(root, 'worldbody')
    for name in root_body_names.values():
        bodies = source.findall(f"./worldbody/body[@name='{name}']")
        if len(bodies) != 1:
            raise ValueError('Unique declared robot root missing in offline XML')
        world.append(copy.deepcopy(bodies[0]))
    names = {body.get('name') for body in world.iter('body')}
    contact = ET.SubElement(root, 'contact')
    for entry in source.findall('./contact/exclude'):
        if entry.get('body1') in names and entry.get('body2') in names:
            contact.append(copy.deepcopy(entry))
    hashes = {}
    for asset in root.findall('./asset/*'):
        if asset.get('file') is not None:
            directory = compiler.get('meshdir' if asset.tag == 'mesh' else 'texturedir', '')
            path = (Path(asset_root) / directory / asset.get('file')).resolve()
            hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    xml = ET.tostring(root, encoding='unicode')
    return RobotCollisionModel(xml=xml, sha256=hashlib.sha256(xml.encode()).hexdigest(),
        model_id=model_id, measurement_id=measurement_id, root_body_names=copy.deepcopy(root_body_names),
        asset_root=str(Path(asset_root).resolve()), asset_sha256=hashes, geometry_error_m=geometry_error_m)
