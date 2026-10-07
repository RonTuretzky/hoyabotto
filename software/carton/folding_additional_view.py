"""Explicit, offline second-camera supplement to the station PixelPort.

The second view is a hypothetical calibrated camera, not verified hardware.
Only rendering, camera intrinsics and the simulation clock are read here. The
primary port still owns its housing/FK checks and every motion/collision guard.
"""
from __future__ import annotations

import copy
from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np

from carton.folding_hinge_vision import depth_major_flap_angles, _rigid
from carton.folding_markers import BOX_MARKERS, carton_pose_from_tags
from carton.folding_observation_status import (
    CartonAvailability, PARTIAL_VIEW_SCHEMA, PARTIAL_VIEW_SOURCE,
)
from carton.folding_observation_recording import LastRGBDFrameCache
from carton.folding_short_hinge_vision import (
    OPEN_SHORT_ANGLE_BOUNDS_DEGREES, depth_open_short_flap_angles,
)
from carton.folding_vision import RGBDTagObserver
from carton.folding_additional_view_profiles import (
    ADDITIONAL_VIEW_CAMERAS, verify_additional_view_camera,
)


MAJORS = ('long_near', 'long_far')
SHORTS = ('short_left', 'short_right')


@dataclass(frozen=True)
class AdditionalViewConfiguration:
    """Declared simulation assumptions and comparison gates, never accuracy."""

    assumption_id: str
    clock_id: str
    required_flaps: tuple[str, ...]
    camera: str = 'front'
    primary_camera: str = 'station'
    camera_fovy_degrees: float = 48.
    max_carton_translation_m: float = .012
    max_carton_rotation_degrees: float = 8.
    max_flap_disagreement_degrees: float = 3.
    observe_open_shorts: bool = False
    allow_primary_carton_absence: bool = False

    def __post_init__(self):
        for field in ('assumption_id', 'clock_id'):
            value = getattr(self, field)
            if not isinstance(value, str) or not value.startswith('offline:') or len(value) <= 8:
                raise ValueError('Explicit offline assumption and simulation clock identities required')
        if self.camera not in ADDITIONAL_VIEW_CAMERAS or self.primary_camera != 'station':
            raise ValueError('Only declared additional views with station primary are supported')
        if not isinstance(self.observe_open_shorts, bool):
            raise ValueError('Explicit boolean open-short opt-in required')
        if not isinstance(self.allow_primary_carton_absence, bool):
            raise ValueError('Explicit boolean primary carton-absence opt-in required')
        supported = MAJORS + SHORTS if self.observe_open_shorts else MAJORS
        if (not isinstance(self.required_flaps, tuple) or not self.required_flaps
                or len(set(self.required_flaps)) != len(self.required_flaps)
                or not set(self.required_flaps) <= set(supported)):
            raise ValueError('Explicit nonempty unique required flap tuple within enabled methods required')
        for field, limit in (('max_carton_translation_m', .012),
                             ('max_carton_rotation_degrees', 8.),
                             ('max_flap_disagreement_degrees', 3.)):
            value = getattr(self, field)
            if isinstance(value, bool) or not np.isfinite(value) or not 0 < value <= limit:
                raise ValueError(f'{field} must be positive and cannot weaken the comparison gate')
        if self.camera_fovy_degrees != 48.:
            raise ValueError('Only the explicitly declared 48 degree front intrinsics are supported')


def _angle(row, name, tags, seq):
    if row is None:
        return None
    if (not isinstance(row, dict) or row.get('unambiguous', True) is not True
            or row.get('observed_seq', seq) != seq):
        raise ValueError(f'{name}: ambiguous or stale major estimate')
    angle = row.get('degrees')
    if isinstance(angle, bool) or not isinstance(angle, (float, int)) or not np.isfinite(angle) or not -40 < angle < 103:
        raise ValueError(f'{name}: fresh finite angle within existing observation range required')
    if name in SHORTS and not OPEN_SHORT_ANGLE_BOUNDS_DEGREES[0] < angle < OPEN_SHORT_ANGLE_BOUNDS_DEGREES[1]:
        raise ValueError(f'{name}: outside explicitly supported open-short angle bounds')
    if row.get('method') == 'apriltag_aligned_depth_plane':
        tag = {'long_near': 14, 'long_far': 13, 'short_left': 11, 'short_right': 12}[name]
        if tag not in tags:
            raise ValueError(f'{name}: decoded flap identity absent from current view')
    elif row.get('method') == ('aligned_depth_open_short_hinge_consistent_plane'
                              if name in SHORTS else 'aligned_depth_hinge_consistent_plane'):
        if name in SHORTS:
            if row.get('valid_angle_bounds_degrees') != list(OPEN_SHORT_ANGLE_BOUNDS_DEGREES):
                raise ValueError(f'{name}: declared open-short angle bounds required')
            competitor = row.get('competing_plane_support_ratio')
            if (not isinstance(competitor, (int, float)) or isinstance(competitor, bool)
                    or not np.isfinite(competitor) or not 0 <= competitor < .35):
                raise ValueError(f'{name}: competing open-short planes are ambiguous')
        for key, minimum in (('pixel_support', 60), ('supported_patches', 6),
                             ('along_span_mm', 75.), ('radial_span_mm', 30.)):
            value = row.get(key)
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not np.isfinite(value) or value < minimum):
                raise ValueError(f'{name}: unchanged hinge plane support gate failed: {key}')
        for key, maximum in (('hinge_axis_error_deg', 4.), ('hinge_plane_offset_mm', 6.)):
            value = row.get(key)
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not np.isfinite(value) or abs(value) > maximum):
                raise ValueError(f'{name}: unchanged hinge plane identity gate failed: {key}')
    else:
        raise ValueError(f'{name}: unsupported major identity method')
    return float(angle)


def _primary_carton_absent(primary, configuration):
    partial = (primary.get('source') == PARTIAL_VIEW_SOURCE
               or 'packet_schema' in primary or 'carton_status' in primary)
    if not partial:
        return False
    if (not configuration.allow_primary_carton_absence
            or primary.get('source') != PARTIAL_VIEW_SOURCE
            or primary.get('packet_schema') != PARTIAL_VIEW_SCHEMA
            or primary.get('carton_status') != CartonAvailability.MISSING_IDENTITY.value
            or any(key in primary for key in ('world_from_box', 'angles', 'box_registration'))):
        raise ValueError('Explicit missing-carton packet schema without cached geometry required')
    return True


def _combine(primary, secondary, configuration):
    """Combine two current pixel packets; unavailable views are never priors."""
    seq = primary['seq']
    if (secondary['seq'] != seq or primary['camera'] != 'station'
            or secondary['camera'] != configuration.camera):
        raise ValueError('Matching sequence and explicitly declared view identities required')
    absent = _primary_carton_absent(primary, configuration)
    s = _rigid(secondary['world_from_box'], 'Additional pixel carton pose')
    translation, rotation = None, None
    if not absent:
        p = _rigid(primary['world_from_box'], 'Primary pixel carton pose')
        translation = float(np.linalg.norm(p[:3, 3] - s[:3, 3]))
        rotation = float(np.degrees(np.arccos(np.clip((np.trace(p[:3, :3].T @ s[:3, :3]) - 1) / 2, -1, 1))))
        if translation > configuration.max_carton_translation_m or rotation > configuration.max_carton_rotation_degrees:
            raise ValueError('Fresh independent camera views disagree on the carton pose')
    angles, comparisons = copy.deepcopy(primary.get('angles', {})), {}
    for name in MAJORS + SHORTS if configuration.observe_open_shorts else MAJORS:
        first, second = primary.get('angles', {}).get(name), secondary['angles'].get(name)
        omitted = None
        if name in SHORTS and first and first.get('method') == 'aligned_depth_cardboard_plane':
            # The legacy stripe lacks hinge identity. It is not a trusted
            # corroborating estimate, and cannot defeat a fresh identified view.
            omitted = first['method']
            first = None
            angles.pop(name, None)
        a = _angle(first, name, primary['tags'], seq)
        b = _angle(second, name, secondary['tags'], seq)
        difference = None if a is None or b is None else abs(a - b)
        if difference is not None and difference > configuration.max_flap_disagreement_degrees:
            raise ValueError(f'Fresh independent camera views disagree on {name}')
        comparisons[name] = dict(primary_degrees=a, additional_degrees=b,
                                 difference_degrees=difference)
        if omitted:
            comparisons[name]['primary_omitted_method'] = omitted
        if a is not None:
            angles[name]['source_camera'] = 'station'
            angles[name]['observed_seq'] = seq
        elif b is not None:
            angles[name] = copy.deepcopy(second)
            angles[name].update(source_camera=configuration.camera, observed_seq=seq,
                                source_method=second['method'],
                                method=('additional_view_open_short_hinge_consistent_plane'
                                        if name in SHORTS else 'additional_view_hinge_consistent_plane'))
        elif name in configuration.required_flaps:
            raise ValueError(f'Fresh required {name} missing from both synchronized views')
    comparison = dict(carton_translation_disagreement_mm=None if translation is None else translation * 1000,
                        carton_rotation_disagreement_degrees=rotation,
                        angles=comparisons)
    if absent:
        comparison.update(carton_comparison_status='primary_carton_unavailable',
                          primary_carton_status=CartonAvailability.MISSING_IDENTITY.value)
    return angles, comparison


class AdditionalViewPixelPort:
    """Opt-in composition; construct the normal station PixelPort first.

    By default both views must freshly register carton and world anchor on
    every call. An explicit post-startup opt-in allows only absent primary
    carton identity after fresh anchor and current visible housing/FK checks.
    The additional view must then supply all required geometry. No missing
    primary anchors or other failures are rescued. If both supply a flap they
    must agree. No estimate is averaged, extrapolated or filled from a prior.
    A failed additional registration refuses the whole observation, even when
    the primary alone could have supplied all required angles.

    ``observer`` and ``arm_tag_checks`` remain the primary's. Additional evidence
    is in ``additional_view_history`` and each returned reading. The current
    single-camera portable scene adapter deliberately rejects this new source.
    """

    def __init__(self, primary, *, configuration: AdditionalViewConfiguration,
                 seed=0, noise=.0008, dropout=.25):
        if not isinstance(configuration, AdditionalViewConfiguration):
            raise ValueError('Explicit hypothetical additional-camera configuration required')
        previous = primary if isinstance(primary, AdditionalViewPixelPort) else None
        if previous is not None:
            if previous._failed:
                raise ValueError('Failed additional-view port requires a new primary startup')
            primary = previous.primary
        if primary.camera != configuration.primary_camera:
            raise ValueError('Station PixelPort must retain primary startup housing registration checks')
        if ((getattr(primary, 'seq', 0) > 0 or primary.readings)
                and (not primary.readings or primary.readings[0]['seq'] != 1)):
            raise ValueError('A failed or missing primary startup cannot be bypassed by adding a view')
        if getattr(primary, '_command_failed', False):
            raise ValueError('Failed primary command requires a new primary startup')
        if getattr(primary, '_observation_failed', False):
            raise ValueError('Failed primary observation requires a new primary startup')
        if configuration.allow_primary_carton_absence:
            if (getattr(primary, 'startup_registration_verified', False) is not True
                    or not primary.readings or primary.readings[0]['seq'] != 1
                    or 'world_from_box' not in primary.readings[0]
                    or primary.readings[0].get('source') not in (None, 'calibrated_rgbd')):
                raise ValueError('Normal primary startup required before allowing carton absence')
            if not callable(getattr(primary, 'observe_with_carton_absence', None)):
                raise ValueError('Explicit primary partial-packet observation API required')
        if (not np.isfinite(noise) or noise < 0 or not np.isfinite(dropout)
                or not 0 <= dropout < 1):
            raise ValueError('Finite nonnegative simulated depth noise and dropout below one required')
        self.primary = primary
        self.configuration = configuration
        self.additional_rgbd_frame = LastRGBDFrameCache(configuration.camera,
                                                        clock_id=configuration.clock_id)
        if hasattr(primary, 'last_rgbd_frame'):
            # Starting a phase can declare its clock identity, but must never
            # relabel a previous phase's frame as current evidence.
            primary.last_rgbd_frame = LastRGBDFrameCache(primary.camera,
                                                         clock_id=configuration.clock_id)
        self.renderer_camera_declaration = verify_additional_view_camera(
            configuration.camera, primary.sim.model.camera(configuration.camera))
        self.rng = np.random.default_rng(seed)
        self.noise, self.dropout = float(noise), float(dropout)
        anchors = copy.deepcopy(primary.observer.anchors)
        if 1 not in anchors:
            raise ValueError('Declared world anchor 1 required')
        anchor = anchors.pop(1)
        # Unlike the primary stationary-camera observer, solve this camera's
        # transform anew from current pixels, with no cached registration.
        self.additional_observer = RGBDTagObserver(anchor, additional_anchors=anchors,
                                                    stationary_camera=False)
        self.additional_priors = copy.deepcopy(previous.additional_priors) if previous else {}
        self.additional_view_history = copy.deepcopy(previous.additional_view_history) if previous else []
        # A stage may enable this mode after a completed primary-only prefix.
        # Preserve that prefix without relabelling it as multiview evidence.
        self.readings = copy.deepcopy(previous.readings if previous else primary.readings)
        self._last_seq = self.readings[-1]['seq'] if self.readings else 0
        self._failed = False
        declaration = dict(configuration=asdict(configuration),
            renderer_camera=copy.deepcopy(self.renderer_camera_declaration),
            activation_after_primary_sequence=self.readings[-1]['seq'] if self.readings else 0,
            preserved_primary_reading_count=len(self.readings),
            world_anchor_poses={str(i): np.asarray(p).tolist()
                                for i, p in self.additional_observer.anchors.items()},
            depth_noise_std_m=self.noise, dropout_fraction=self.dropout, random_seed=int(seed),
            simulation_only=True, physical_camera_verified=False,
            uncertainty_source='comparison gates and synthetic noise, not physical accuracy bounds')
        if configuration.observe_open_shorts:
            declaration['open_short_angle_bounds_degrees'] = list(OPEN_SHORT_ANGLE_BOUNDS_DEGREES)
        if previous:
            declaration['previous_phase'] = dict(assumptions_sha256=previous.assumptions_sha256,
                                                declaration=copy.deepcopy(previous.declaration))
        self.declaration = declaration
        self.assumptions_sha256 = hashlib.sha256(json.dumps(declaration, sort_keys=True,
                                                          allow_nan=False).encode()).hexdigest()

    def __getattr__(self, name):
        return getattr(self.primary, name)

    def _time(self):
        value = float(self.primary.sim.data.time)
        if not np.isfinite(value) or value < 0:
            raise ValueError('Finite nonnegative simulation clock required')
        return value

    def _same_time(self, timestamp):
        if self._time() != timestamp:
            raise ValueError('Additional and primary views are not the same frozen simulated state')

    def _validate_partial(self, primary, timestamp):
        """Reject stale or forged partial provenance before a second view is used."""
        seq = primary['seq']
        history = primary.get('observer_history')
        if (seq <= 1 or primary.get('startup_registration_sequence') != 1
                or primary.get('rgb_timestamp_s') != timestamp
                or primary.get('depth_timestamp_s') != timestamp
                or history != self.primary.observer.history[-1]
                or not history or history.get('seq') != seq
                or primary.get('tags') != history.get('detected')):
            raise ValueError('Fresh partial timestamps, sequence, tags and observer history required')
        tags = set(primary['tags'])
        anchors = set(history.get('anchor_ids', ()))
        if (not anchors or not anchors <= tags & set(self.primary.observer.anchors)
                or tags & set(BOX_MARKERS) or {11, 12} <= tags
                or set(history.get('rejected', ())) & (set(BOX_MARKERS) | {11, 12, 2, 4})):
            raise ValueError('Partial packet requires fresh anchors, valid decoded housing identity and only absent carton identity')
        camera = _rigid(primary.get('world_from_camera'), 'Partial pixel camera pose')
        if not np.array_equal(camera, self.primary.observer.world_from_camera):
            raise ValueError('Partial camera registration must match the current observer')
        calibration = primary.get('calibration', {})
        intrinsics = np.asarray(calibration.get('intrinsics'), dtype=float)
        expected_anchors = {str(i): np.asarray(p).tolist() for i, p in self.primary.observer.anchors.items()}
        if (calibration.get('camera') != 'station'
                or calibration.get('anchor_poses') != expected_anchors
                or intrinsics.shape != (3, 3) or not np.isfinite(intrinsics).all()
                or min(intrinsics[0, 0], intrinsics[1, 1]) <= 0
                or not np.array_equal(intrinsics[2], [0., 0., 1.])
                or primary.get('calibration_sha256') != hashlib.sha256(json.dumps(
                    calibration, sort_keys=True, allow_nan=False).encode()).hexdigest()):
            raise ValueError('Partial calibrated intrinsics and anchor identity required')
        checks = primary.get('housing_fk_checks')
        current = [row for row in self.primary.arm_tag_checks if row['seq'] == seq]
        if (checks != current or {row['tag_id'] for row in checks} != tags & {2, 4}
                or any(not np.isfinite(row['encoder_fk_error_mm'])
                       or not 0 <= row['encoder_fk_error_mm'] <= 12. for row in checks)):
            raise ValueError('All current visible primary housing/FK checks required')
        for key in ('rgb_sha256', 'exposed_depth_sha256'):
            digest = primary.get(key)
            if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError('Current partial RGB/depth provenance hashes required')

    def _checked_command(self, command, *args, **kwargs):
        if (self._failed or getattr(self.primary, '_observation_failed', False)
                or getattr(self.primary, '_command_failed', False)):
            raise ValueError('Additional-view session failure requires reconstruction before motion')
        try:
            return command(*args, **kwargs)
        except Exception as exc:
            self._failed = True
            self.primary._command_failed = True
            self.primary._command_failure_reason = str(exc)
            raise

    def move_arms(self, *args, **kwargs):
        return self._checked_command(self.primary.move_arms, *args, **kwargs)

    def set_grippers(self, *args, **kwargs):
        return self._checked_command(self.primary.set_grippers, *args, **kwargs)

    def observe(self, label):
        self.additional_rgbd_frame.clear()
        primary_cache = getattr(self.primary, 'last_rgbd_frame', None)
        if primary_cache is not None:
            if primary_cache.clock_id != self.configuration.clock_id:
                self.primary.last_rgbd_frame = LastRGBDFrameCache(self.primary.camera,
                    clock_id=self.configuration.clock_id)
            else:
                primary_cache.clear()
        if (self._failed or getattr(self.primary, '_observation_failed', False)
                or getattr(self.primary, '_command_failed', False)):
            raise ValueError('Additional-view port refused an observation; reconstruct and repeat startup checks')
        primary = None
        audit = dict(assumption_id=self.configuration.assumption_id,
            assumptions_sha256=self.assumptions_sha256, clock_id=self.configuration.clock_id,
            simulation_only=True, physical_camera_verified=False, hardware_commands=False,
            additional_camera=self.configuration.camera, primary_camera='station',
            fresh_independent_camera_registration=True,
            required_flaps=list(self.configuration.required_flaps), status='pending')
        try:
            timestamp = self._time()
            audit['synchronized_simulation_time_s'] = timestamp
            # No exception becomes missing data. The explicit API can return
            # an absent-identity packet only after its mandatory checks pass.
            primary = (self.primary.observe_with_carton_absence(label)
                       if self.configuration.allow_primary_carton_absence
                       else self.primary.observe(label))
            self._same_time(timestamp)
            seq = primary['seq']
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= self._last_seq:
                raise ValueError('Fresh primary observation sequence required')
            absent = _primary_carton_absent(primary, self.configuration)
            if primary.get('privileged_mechanics_probe') or (not absent and primary.get('source') not in (None, 'calibrated_rgbd')):
                raise ValueError('Only ordinary pixel observations may use the additional view')
            if self.primary.observer.history[-1].get('seq') != seq:
                raise ValueError('Fresh primary observer history must match the current sequence')
            _rigid(self.primary.observer.world_from_camera, 'Primary pixel camera pose')
            if absent:
                self._validate_partial(primary, timestamp)
            audit.update(seq=seq, primary=copy.deepcopy(primary))
            audit['primary'].update(rgb_timestamp_s=timestamp, depth_timestamp_s=timestamp,
                world_from_camera=np.asarray(self.primary.observer.world_from_camera).tolist(),
                observer_history=copy.deepcopy(self.primary.observer.history[-1]))
            camera = self.configuration.camera
            if verify_additional_view_camera(camera, self.primary.sim.model.camera(camera)) != self.renderer_camera_declaration:
                raise ValueError('Additional camera declaration changed during observation')
            rgb = self.primary.sim.render(camera).copy()
            self._same_time(timestamp)
            depth = self.primary.sim.render(camera, True).copy()
            self._same_time(timestamp)
            fovy = float(self.primary.sim.model.camera(camera).fovy[0])
            if fovy != self.configuration.camera_fovy_degrees:
                raise ValueError('Additional camera intrinsics do not match the explicit setup declaration')
            depth += self.rng.normal(0, self.noise, depth.shape)
            depth[self.rng.random(depth.shape) < self.dropout] = 0
            h, w = depth.shape
            f = h / (2 * math.tan(math.radians(fovy) / 2))
            k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
            self.additional_rgbd_frame.capture(rgb, depth, seq=seq,
                timestamp_s=timestamp, intrinsics=k)
            tags = self.additional_observer.observe(rgb, depth, k, seq=seq,
                timestamp=timestamp, depth_timestamp=timestamp)
            quality = self.additional_observer.history[-1]
            box, registration = carton_pose_from_tags(tags, quality['quality'])
            angles = depth_major_flap_angles(rgb, depth, k,
                self.additional_observer.world_from_camera, box, self.additional_priors)
            if self.configuration.observe_open_shorts:
                angles.update(depth_open_short_flap_angles(rgb, depth, k,
                    self.additional_observer.world_from_camera, box, self.additional_priors))
            secondary = dict(seq=seq, camera=camera, tags=sorted(tags),
                world_from_box=box.tolist(), box_registration=registration, angles=angles,
                world_from_camera=self.additional_observer.world_from_camera.tolist(),
                observer_history=copy.deepcopy(quality), rgb_timestamp_s=timestamp,
                depth_timestamp_s=timestamp, intrinsics=k.tolist(),
                rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
                exposed_depth_sha256=hashlib.sha256(depth.tobytes()).hexdigest())
            audit['additional'] = secondary
            combined, comparison = _combine(primary, secondary, self.configuration)
            self._same_time(timestamp)
            audit.update(status='accepted', comparison=comparison)
            if absent:
                audit.update(primary_carton_status=CartonAvailability.MISSING_IDENTITY.value,
                             pose_source_camera=camera)
                result = copy.deepcopy(secondary)
                result.update(label=label, paddle=None, pose_source_camera=camera,
                              primary_carton_status=CartonAvailability.MISSING_IDENTITY.value)
            else:
                result = copy.deepcopy(primary)
            result.update(angles=combined, source='calibrated_rgbd_additional_view',
                          additional_view=copy.deepcopy(audit))
            self.additional_priors.update({name: row['degrees'] for name, row in angles.items()})
            self._last_seq = seq
            self.readings.append(result)
            self.additional_view_history.append(audit)
            return result
        except Exception as exc:
            # The shared primary/session owns the failure boundary. Otherwise
            # another wrapper, a raw-primary rewrap, or direct primary motion
            # could bypass a secondary-camera registration/identity failure.
            self._failed = True
            self.primary._observation_failed = True
            audit.update(status='refused', refusal=str(exc))
            self.additional_view_history.append(audit)
            if primary is not None:
                refused = copy.deepcopy(primary)
                refused.update(angles={}, source='calibrated_rgbd_additional_view',
                               additional_view=copy.deepcopy(audit))
                self.readings.append(refused)
            raise
