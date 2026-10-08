"""Bounded offline short-contact probe after both partial majors are passive.

The two CAD proposals passed static paired robot-clearance diagnostics. Whether
the shorts can displace the free majors is an unproven dynamics question. This
component stops at a declared short target no greater than +10 degrees and
never labels that component, or a hypothetical panel intersection, full closure.
"""
from contextlib import AbstractContextManager
import gzip
import hashlib
import json
import math
from types import SimpleNamespace
import uuid

import mujoco
import numpy as np

from carton.folding_diagonal import contact_point
from carton.folding_far_contact import RobotVertexIK, mesh_contact_vertices, _verify_target_angle
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_progress import ContactProgressGuard
from carton.folding_hinge_vision import _rigid
from carton.folding_sim import JOINTS, F, W
from carton.folding_panel_audit import (
    PANEL_SCHEMA, PANEL_SOURCE, PANEL_LOG_NAME, INTEGRATORS, event_digest,
)


_PROFILES = {
    'front': {
        'along': -.10,
        'left': ([-.010900730, -.006113951, -.094425959],
                 [-.352499325, -1.544291480, .953627161, .831693404, -1.569852638]),
        'right': ([-.010900730, .005678369, -.094425928],
                  [.416730994, -.876738899, .266623386, 1.308314546, .833325059]),
    },
    'middlefar': {
        'along': .06,
        'left': ([-.016536210, -.003251981, -.094435794],
                 [-.213924120, .045895518, -.257406533, .616806899, -.265937843]),
        'right': ([-.010900730, .005678369, -.094425928],
                  [.219551130, .057894514, -.249991912, .622400643, .843985127]),
    },
}
_SIDES = ('left', 'right')
_FLAPS = ('short_left', 'short_right', 'long_near', 'long_far')
_CONTACT_POLICIES = ('measured_v2', 'setpoint_feedback_v3', 'tangent_deadband_v4', 'jaw_surface_v5')
_SETPOINT_POLICIES = ('setpoint_feedback_v3', 'tangent_deadband_v4', 'jaw_surface_v5')
# V4 corrects errors across the panel (radial) and along the hinge only beyond
# these bounds. In the recorded V3 whole-jaw strokes both components jitter by
# about 1 mm (one sigma) from current registration noise and never exceed
# 3.3 mm; neither direction changes the flap angle.
_TANGENT_DEADBANDS_M = dict(radial=.003, hinge=.003)
_STROKE_STEPS_DEGREES = (.25, .5, 1.)
# V5 drives the nearest permitted jaw surface this far past the sensed outer
# panel plane. The recorded V4 right strokes held the jaw 1.3 mm off the panel
# because the 3.5 mm approach standoff was reused as the stroke target. The
# press stays below the unchanged 1 mm robot/flap penetration stop.
_JAW_PRESS_M = .0005
# Short panels are 3 mm cardboard hinged at their midplane; V5 measures the
# jaw against the outer face (the observed-scene adapter declares the same).
_PANEL_HALF_THICKNESS_M = .0015
_APPROACH_POLICIES = ('elevated_v2', 'whole_jaw_normal_v1')
_WHOLE_JAW = {
    'left': dict(vertex=dict(body='left_gripper_link',
        local=[-.03499957180960571, -.012649502777943581, -.0069244265327669235],
        geometry='left_wrist_roll_follower_so101_v1_part_37'),
        along=0., radius=.14, offset=.0015,
        seed=[-.40179800698637935, .37215188776714014, -.5126514124071644,
              1.1376084149710965, 1.347337488154797]),
    'right': dict(vertex=dict(body='right_gripper_link',
        local=[-.01064218, -.007367827, -.084425503]),
        along=-.10, radius=.14, offset=.0035,
        seed=[.5143840450876404, -.754664796272655, .23038185403013392,
              1.429046286599008, -2.416513149718617]),
}


class _PanelStepAudit(AbstractContextManager):
    """Add a panel-panel stop to the existing actual-step diagnostic callback.

    Runs before render/geometry refresh in FoldingSimulation.move. An earlier
    existing robot/load fault can skip that callback on the final refused step;
    coverage then remains explicitly incomplete and cannot authorize success.
    """
    def __init__(self, sim, report):
        self.sim, self.report = sim, report
        self.previous = sim.step_diagnostic
        self.started = self.last_time = float(sim.data.time)
        self.timestep = float(sim.model.opt.timestep)
        self.event_start_index = len(sim.events)
        self.recording_id = uuid.uuid4().hex
        self.path = sim.out / PANEL_LOG_NAME
        self.row = dict(path=PANEL_LOG_NAME, format='gzip_jsonl', schema=PANEL_SCHEMA,
                        source=PANEL_SOURCE, recording_id=self.recording_id,
                        expected_start_time=self.started, timestep_s=self.timestep,
                        source_report='folding.json', source_event_start_index=self.event_start_index,
                        max_panel_panel_penetration_mm=0., steps=0, complete_coverage=False)
        report['panel_panel_audit'] = self.row

    def __enter__(self):
        if mujoco.mjtIntegrator(self.sim.model.opt.integrator).name not in INTEGRATORS:
            raise ValueError('Panel contact audit requires an original one-pass integrator')
        self.stream = gzip.open(self.path, 'xt', encoding='utf-8')
        self.sim.step_diagnostic = self.diagnose
        return self

    def diagnose(self):
        sim = self.sim
        now, dt = float(sim.data.time), float(sim.model.opt.timestep)
        if (not math.isclose(dt, self.timestep, rel_tol=0., abs_tol=1e-12)
                or not math.isclose(now-self.last_time, dt, rel_tol=1e-8, abs_tol=1e-9)):
            return 'Incomplete actual-step panel contact audit coverage'
        contacts = []
        for index, contact in enumerate(sim.data.contact):
            a, b = sim.model.geom(contact.geom1).name, sim.model.geom(contact.geom2).name
            if not (a.endswith('_cardboard') and b.endswith('_cardboard')):
                continue
            wrench = np.zeros(6)
            mujoco.mj_contactForce(sim.model, sim.data, index, wrench)
            penetration = max(0., -float(contact.dist)*1000)
            self.row['max_panel_panel_penetration_mm'] = max(
                self.row['max_panel_panel_penetration_mm'], penetration)
            contacts.append(dict(contact_id=index, geom1=a, geom2=b, distance_m=float(contact.dist),
                                 position_m=contact.pos.tolist(), contact_frame=contact.frame.tolist(),
                                 wrench_contact_N_Nm=wrench.tolist()))
        self.stream.write(json.dumps(dict(schema=PANEL_SCHEMA, source=PANEL_SOURCE,
            recording_id=self.recording_id, step_index=self.row['steps'],
            step_started_at=self.last_time, step_ended_at=now, timestep_s=dt,
            integrator=mujoco.mjtIntegrator(sim.model.opt.integrator).name,
            total_contact_count=int(sim.data.ncon), contacts=contacts), allow_nan=False)+'\n')
        self.last_time = now
        self.row['steps'] += 1
        prior_fault = self.previous()
        if prior_fault:
            return prior_fault
        if self.row['max_panel_panel_penetration_mm'] > 1.:
            return 'Panel/panel penetration exceeded 1 mm during bounded short probe'
        return None

    def coverage_complete(self):
        expected = round((float(self.sim.data.time)-self.started)/self.sim.model.opt.timestep)
        return (expected > 0 and self.row['steps'] == expected
                and math.isclose(self.last_time, float(self.sim.data.time), abs_tol=1e-9))

    def __exit__(self, *exc):
        self.sim.step_diagnostic = self.previous
        self.row['complete_coverage'] = self.coverage_complete()
        self.stream.close()
        with self.path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        events = self.sim.events[self.event_start_index:]
        self.row.update(sha256=digest, compressed_bytes=self.path.stat().st_size,
                        expected_end_time=float(self.sim.data.time),
                        source_event_count=len(events), source_events_sha256=event_digest(events))
        return False


class _ShortStroke:
    def __init__(self, angles, target, *, step_degrees=.25):
        if not math.isfinite(step_degrees) or step_degrees not in (.25, .5, 1.):
            raise ValueError('Declared short stroke increment must be 0.25, 0.5 or 1 degree')
        self.target = target
        self.step_degrees = step_degrees
        # Keep the previous 12-degree attempted-command budget and 80-degree
        # total budget explicit when smaller observed-angle increments are used.
        self.stall_window = int(math.ceil(12./step_degrees))
        self.max_commands = int(math.ceil(80./step_degrees))
        self.history = {side: [float(angles[side])] for side in _SIDES}
        self.commands = 0

    def next_angles(self):
        latest = {a: self.history[a][-1] for a in _SIDES}
        if all(value >= self.target-1 for value in latest.values()):
            return None
        if self.commands >= self.max_commands:
            raise ValueError(f'Paired short probe exceeded {self.max_commands} bounded commands')
        self.commands += 1
        return {a: min(self.target, value+self.step_degrees) for a, value in latest.items()}

    def observe(self, angles):
        for a in _SIDES:
            value = float(angles[a])
            if not math.isfinite(value) or not -25 <= value <= self.target+5:
                raise ValueError('Observed short angle left bounded probe range')
            self.history[a].append(value)
            if (len(self.history[a]) > self.stall_window and value < self.target-1
                    and value-self.history[a][-self.stall_window-1] < 2.):
                raise ValueError(a+f' short stalled over {self.stall_window} bounded commands')


def _contact_reading(reading, side):
    """Keep a flap angle paired with its own current raw camera registration."""
    flap = 'short_'+side
    angle = reading.get('angles', {}).get(flap)
    audit = reading.get('additional_view')
    seq = reading.get('seq')
    if (not isinstance(angle, dict) or not isinstance(audit, dict)
            or audit.get('status') != 'accepted' or audit.get('seq') != seq
            or angle.get('observed_seq') != seq):
        raise ValueError('Fresh source-bound short angle and accepted independent views required')
    camera = angle.get('source_camera')
    packets = [audit.get(key) for key in ('primary', 'additional')]
    packets = [packet for packet in packets if isinstance(packet, dict)
               and packet.get('camera') == camera]
    if len(packets) != 1 or packets[0].get('seq') != seq:
        raise ValueError('Short contact pose must come from the angle source camera and sequence')
    packet = packets[0]
    source_angle = packet.get('angles', {}).get(flap, {}).get('degrees')
    if (not isinstance(source_angle, (float, int)) or isinstance(source_angle, bool)
            or not math.isfinite(source_angle) or source_angle != angle.get('degrees')):
        raise ValueError('Merged short angle differs from its raw source-camera observation')
    timestamp = audit.get('synchronized_simulation_time_s')
    if (not isinstance(timestamp, (float, int)) or isinstance(timestamp, bool)
            or not math.isfinite(timestamp)
            or any(packet.get(key) != timestamp for key in ('rgb_timestamp_s', 'depth_timestamp_s'))):
        raise ValueError('Current synchronized RGB-D source timestamps required for short contact')
    pose = _rigid(packet.get('world_from_box'), 'Short source-camera carton pose')
    # The wrapper has already checked independent station/additional agreement.
    # Keep both original packets intact; never average, extrapolate, or replace
    # the primary registration used by the existing global drift guards.
    coherent = dict(reading, world_from_box=pose.tolist())
    return coherent, dict(camera=camera, seq=seq, world_from_box=pose.tolist(),
                          measured_degrees=source_angle, timestamp_s=timestamp)


def _actuator_setpoint(sim, side):
    """Read the current command without clipping or changing the live plant."""
    ids = [sim.model.actuator(side+'_'+joint).id for joint in JOINTS[:5]]
    q = np.asarray(sim.data.ctrl[ids], dtype=float).copy()
    limits = np.asarray([sim.model.joint(side+'_'+joint).range for joint in JOINTS[:5]])
    controls = np.asarray(sim.model.actuator_ctrlrange[ids])
    if (q.shape != (5,) or not np.isfinite(q).all()
            or np.any(q < limits[:, 0]) or np.any(q > limits[:, 1])
            or np.any(q < controls[:, 0]) or np.any(q > controls[:, 1])):
        raise ValueError('Finite actuator setpoints within original joint and control ranges required')
    return q


def _fold_frame(box, side, degrees):
    """Unit fold, radial and hinge directions of one short panel, in world.

    The fold direction is the panel's inward face normal: a hinged panel's
    contact point moves along it as the angle grows. Radial runs from hinge to
    tip across the panel and hinge follows the hinge axis; motion along either
    leaves the flap angle unchanged. The frame uses only the current source
    registration and its measured angle; no renderer or simulator state.
    """
    if side not in _SIDES:
        raise ValueError('Unknown short side for fold frame')
    if (isinstance(degrees, bool) or not isinstance(degrees, (float, int))
            or not math.isfinite(degrees) or not -40 <= degrees <= 103):
        raise ValueError('Finite short angle within the observation range required for fold frame')
    rotation = _rigid(box, 'Short fold-frame carton pose')[:3, :3]
    theta, sign = math.radians(degrees), (-1. if side == 'left' else 1.)
    return dict(fold=rotation @ np.array([-sign*math.cos(theta), 0., -math.sin(theta)]),
                radial=rotation @ np.array([-sign*math.sin(theta), 0., math.cos(theta)]),
                hinge=rotation @ np.array([0., 1., 0.]))


def _tangent_deadband_increment(delta, frame, max_step_m):
    """Capped fold-direction lead plus only out-of-deadband radial/hinge error.

    The fold component never retreats: a vertex already ahead of the current
    measured target along the arc is left where it is and recorded, so a flap
    that does not follow shows up as a measured stall rather than as chasing.
    """
    if (not isinstance(frame, dict) or set(frame) != {'fold', 'radial', 'hinge'}
            or any(np.shape(frame[k]) != (3,) or not np.isfinite(frame[k]).all() for k in frame)):
        raise ValueError('Current unit fold frame required for tangent deadband policy')
    axes = {k: np.asarray(frame[k], dtype=float) for k in ('fold', 'radial', 'hinge')}
    gram = np.array([[a @ b for b in axes.values()] for a in axes.values()])
    if not np.allclose(gram, np.eye(3), atol=1e-9):
        raise ValueError('Orthonormal fold frame required for tangent deadband policy')
    gaps = {k: float(delta @ axis) for k, axis in axes.items()}
    corrections = {k: (gaps[k] if abs(gaps[k]) > _TANGENT_DEADBANDS_M[k] else 0.)
                   for k in ('radial', 'hinge')}
    lead = min(max(gaps['fold'], 0.), max_step_m)
    increment = (lead*axes['fold'] + corrections['radial']*axes['radial']
                 + corrections['hinge']*axes['hinge'])
    size = float(np.linalg.norm(increment))
    increment = increment*min(1., max_step_m/max(size, 1e-12))
    return increment, dict(fold_frame_world={k: axis.tolist() for k, axis in axes.items()},
        gap_fold_m=gaps['fold'], gap_radial_m=gaps['radial'], gap_hinge_m=gaps['hinge'],
        fold_lead_increment_m=lead, radial_correction_m=corrections['radial'],
        hinge_correction_m=corrections['hinge'], deadbands_m=dict(_TANGENT_DEADBANDS_M),
        fold_direction_retreat_allowed=False)


def _jaw_points(sim, side):
    """World vertices of the side's permitted jaw collision meshes from encoder FK.

    Uses robot joints only (the same CAD and encoders the physical arm has);
    no carton or flap state enters. Collision meshes are convex, so a plane's
    nearest hull point is one of these vertices.
    """
    data = mujoco.MjData(sim.model)
    data.qpos[sim.arm_indices[side]] = sim.data.qpos[sim.arm_indices[side]]
    mujoco.mj_kinematics(sim.model, data)
    points, names = [], []
    for gid in range(sim.model.ngeom):
        name = sim.model.geom(gid).name
        if (not name.startswith(side+'_') or not sim.model.geom_contype[gid]
                or sim.model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH
                or not any(kind in name for kind in ('wrist_roll_follower', 'moving_jaw'))):
            continue
        mid = sim.model.geom_dataid[gid]
        start, count = sim.model.mesh_vertadr[mid], sim.model.mesh_vertnum[mid]
        world = np.einsum('ij,kj->ki', data.geom_xmat[gid].reshape(3, 3),
                          sim.model.mesh_vert[start:start+count]) + data.geom_xpos[gid]
        points.append(world)
        names += [name]*len(world)
    if not points:
        raise ValueError('Permitted jaw collision meshes required for jaw-surface policy')
    return np.concatenate(points), names


def _jaw_panel_clearance(points, names, surface, frame, *, radius, along):
    """Signed clearance of the jaw hull to the sensed outer panel plane.

    Only jaw points over the sensed 140 mm x 283 mm panel count. Positive is a
    gap on the outside; negative is past the sensed surface.
    """
    surface = np.asarray(surface, dtype=float)
    offsets = points-surface
    radial = radius + np.einsum('ij,j->i', offsets, frame['radial'])
    hinge = along + np.einsum('ij,j->i', offsets, frame['hinge'])
    over = (radial >= 0.) & (radial <= F) & (np.abs(hinge) <= W/2)
    if not over.any():
        raise ValueError('No permitted jaw surface over the sensed short panel')
    gaps = -np.einsum('ij,j->i', offsets, frame['fold'])
    gaps[~over] = np.inf
    index = int(np.argmin(gaps))
    return float(gaps[index]), names[index]


def _jaw_surface_increment(delta, frame, clearance, max_step_m, press_m=_JAW_PRESS_M):
    """Fold lead that closes the measured jaw-to-sensed-panel gap plus a press.

    Radial and hinge error to the declared contact location are corrected only
    beyond the V4 deadbands, inside the same cap. The fold lead never retreats.
    """
    if not math.isfinite(clearance):
        raise ValueError('Finite jaw-to-panel clearance required for jaw-surface policy')
    axes = {k: np.asarray(frame[k], dtype=float) for k in ('fold', 'radial', 'hinge')}
    gaps = {k: float(delta @ axes[k]) for k in ('radial', 'hinge')}
    corrections = {k: (gaps[k] if abs(gaps[k]) > _TANGENT_DEADBANDS_M[k] else 0.) for k in gaps}
    lead = min(max(clearance+press_m, 0.), max_step_m)
    increment = lead*axes['fold'] + corrections['radial']*axes['radial'] + corrections['hinge']*axes['hinge']
    size = float(np.linalg.norm(increment))
    increment = increment*min(1., max_step_m/max(size, 1e-12))
    return increment, dict(fold_frame_world={k: axis.tolist() for k, axis in axes.items()},
        jaw_panel_clearance_m=clearance, jaw_press_m=press_m, fold_lead_increment_m=lead,
        gap_radial_m=gaps['radial'], gap_hinge_m=gaps['hinge'], radial_correction_m=corrections['radial'],
        hinge_correction_m=corrections['hinge'], deadbands_m=dict(_TANGENT_DEADBANDS_M),
        fold_direction_retreat_allowed=False)


def _bounded_contact_goals(sim, ik, targets, *, max_step_m=.0005, contact_policy='measured_v2',
                           frames=None, panels=None):
    """Bound the declared CAD command increment; actual motion is monitored.

    V2 references measured FK. V3 instead adds the capped measured target error
    to the existing actuator-setpoint FK. A steady following offset therefore
    does not get added to every command. V4 keeps the V3 reference but spends
    the increment only on the measured fold-direction lead, correcting radial
    and hinge error beyond explicit deadbands. V5 instead sets the fold lead
    from the encoder-FK jaw hull's clearance to the sensed outer panel plane,
    plus a sub-millimetre press. This is a controller policy,
    never a promise that the physical point moves by the requested increment.
    """
    if contact_policy not in _CONTACT_POLICIES:
        raise ValueError('Unknown bounded short contact policy')
    if not math.isfinite(max_step_m) or not 0 < max_step_m <= .0005:
        raise ValueError('Contact CAD substep must be positive and no greater than 0.5 mm')
    goals, points, details = {}, {}, {}
    for side in _SIDES:
        actual_q = sim.data.qpos[sim.arm_indices[side][:5]].copy()
        actual = ik[side].point(actual_q).copy()
        delta = np.asarray(targets[side])-actual
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise ValueError('Finite current FK and sensed short contact target required')
        distance = float(np.linalg.norm(delta))
        increment = delta*min(1., max_step_m/max(distance, 1e-12))
        decomposition = None
        if contact_policy == 'tangent_deadband_v4':
            increment, decomposition = _tangent_deadband_increment(
                delta, (frames or {}).get(side), max_step_m)
        if contact_policy == 'jaw_surface_v5':
            panel = (panels or {}).get(side)
            if not isinstance(panel, dict):
                raise ValueError('Current sensed panel surface required for jaw-surface policy')
            _tangent_deadband_increment(np.zeros(3), panel['frame'], max_step_m)  # frame validation
            clearance, nearest = _jaw_panel_clearance(*_jaw_points(sim, side), panel['surface'], panel['frame'],
                                                      radius=panel['radius'], along=panel['along'])
            increment, decomposition = _jaw_surface_increment(delta, panel['frame'], clearance, max_step_m)
            decomposition.update(nearest_jaw_geometry=nearest, sensed_surface_world=np.asarray(panel['surface']).tolist())
        seed, reference = actual_q, actual
        if contact_policy in _SETPOINT_POLICIES:
            seed = _actuator_setpoint(sim, side)
            reference = ik[side].point(seed).copy()
            if not np.isfinite(reference).all():
                raise ValueError('Finite actuator-setpoint FK required')
        points[side] = reference + increment
        goals[side], error = ik[side].solve(points[side], seed)
        goal_point = ik[side].point(goals[side]).copy()
        fk_step = float(np.linalg.norm(ik[side].point(goals[side])-actual))
        command_step = float(np.linalg.norm(goal_point-reference))
        if not math.isfinite(command_step) or command_step > max_step_m+1e-6:
            raise ValueError('Short contact IK did not preserve the conservative CAD substep bound')
        details[side] = dict(actual_point_world=actual.tolist(), sensor_target_world=np.asarray(targets[side]).tolist(),
                             command_point_world=points[side].tolist(), requested_distance_m=distance,
                             actual_fk_step_m=fk_step, ik_error_m=error,
                             contact_policy=contact_policy, command_reference_world=reference.tolist(),
                             commanded_increment_world=increment.tolist(), commanded_fk_increment_m=command_step,
                             ik_goal_point_world=goal_point.tolist())
        if contact_policy in _SETPOINT_POLICIES:
            details[side].update(current_actuator_setpoint_radians=seed.tolist(),
                current_setpoint_fk_world=reference.tolist(), actual_minus_setpoint_world=(actual-reference).tolist())
        if decomposition is not None:
            details[side].update(decomposition)
    return goals, points, details


def _point(box, profile, side, degrees, *, offset=None):
    configuration = profile.get('per_side', {}).get(side, {})
    point, _ = contact_point(math.radians(degrees), 0, -1 if side == 'left' else 1,
                             configuration.get('along', profile['along']),
                             configuration.get('radius', .14), 0.,
                             configuration.get('offset', .0015) if offset is None else offset)
    return box[:3, :3] @ point + box[:3, 3]


def _sensed_panels(sources, profile, angles):
    """Outer short-panel face from each arm's own current camera registration."""
    panels = {}
    for a in _SIDES:
        box = np.asarray(sources[a]['world_from_box'])
        configuration = profile.get('per_side', {}).get(a, {})
        panels[a] = dict(surface=_point(box, profile, a, angles[a], offset=_PANEL_HALF_THICKNESS_M),
                         frame=_fold_frame(box, a, angles[a]),
                         radius=configuration.get('radius', .14),
                         along=configuration.get('along', profile['along']))
    return panels


def _validated_jaw_vertex(sim, side, declared):
    """Verify an exact vertex on a permitted original jaw collision mesh.

    This deliberately does not change the older distal-only vertex selector.
    The new approach can use an existing proximal jaw surface, and records
    which original mesh supplied it. External arm/housing meshes are excluded.
    """
    body_name = declared.get('body')
    local = np.asarray(declared.get('local'), dtype=float)
    if (side not in _SIDES or not isinstance(body_name, str) or not body_name.startswith(side+'_')
            or local.shape != (3,) or not np.isfinite(local).all()
            or mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, body_name) < 0):
        raise ValueError('A finite side-bound original jaw vertex is required')
    data = mujoco.MjData(sim.model)
    data.qpos[:] = sim.data.qpos
    mujoco.mj_kinematics(sim.model, data)
    body = data.body(body_name)
    body_id = sim.model.body(body_name).id
    nearest = None
    for gid in range(sim.model.ngeom):
        name = sim.model.geom(gid).name
        if (not name.startswith(side+'_') or not sim.model.geom_contype[gid]
                or sim.model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH
                or not any(kind in name for kind in ('wrist_roll_follower', 'moving_jaw'))
                or sim.model.geom_bodyid[gid] != body_id
                or (declared.get('geometry') is not None and declared['geometry'] != name)):
            continue
        mid = sim.model.geom_dataid[gid]
        start, count = sim.model.mesh_vertadr[mid], sim.model.mesh_vertnum[mid]
        world = np.einsum('ij,kj->ki', data.geom_xmat[gid].reshape(3, 3),
                          sim.model.mesh_vert[start:start+count]) + data.geom_xpos[gid]
        points = np.einsum('ji,kj->ki', body.xmat.reshape(3, 3), world-body.xpos)
        errors = np.linalg.norm(points-local, axis=1)
        index = int(np.argmin(errors))
        if nearest is None or errors[index] < nearest['source_vertex_error_m']:
            nearest = dict(body=body_name, local=local.tolist(), geometry=name,
                           source_mesh_id=int(mid), source_mesh_vertex_index=index,
                           source_vertex_error_m=float(errors[index]))
    if nearest is None or nearest['source_vertex_error_m'] > 1e-6:
        raise ValueError('Declared whole-jaw contact vertex absent from original permitted collision meshes')
    return nearest


def _subdivide_reversed_joint_paths(paths, ik, *, max_step_m=.0005):
    """Subdivide existing joint edges, without solving new inward IK poses."""
    if not math.isfinite(max_step_m) or not 0 < max_step_m <= .0005:
        raise ValueError('Exact approach CAD increment must be at most 0.5 mm')
    if set(paths) != set(_SIDES) or len(paths['left']) != len(paths['right']) or len(paths['left']) < 2:
        raise ValueError('Equal finite paired reverse joint paths required')
    waypoints = [{a: np.asarray(paths[a][0], dtype=float).copy() for a in _SIDES}]

    def append_edge(start, end, depth=0):
        if any(q.shape != (5,) or not np.isfinite(q).all() for q in (*start.values(), *end.values())):
            raise ValueError('Finite original robot joint waypoints required')
        distance = max(float(np.linalg.norm(ik[a].point(end[a])-ik[a].point(start[a]))) for a in _SIDES)
        if not math.isfinite(distance):
            raise ValueError('Finite exact approach CAD FK required')
        if distance <= max_step_m+1e-6:
            waypoints.append(end)
        else:
            if depth >= 8:
                raise ValueError('Exact approach subdivision exceeded finite depth')
            middle = {a: (start[a]+end[a])/2 for a in _SIDES}
            append_edge(start, middle, depth+1)
            append_edge(middle, end, depth+1)

    for index in range(1, len(paths['left'])):
        end = {a: np.asarray(paths[a][index], dtype=float).copy() for a in _SIDES}
        append_edge(waypoints[-1], end)
    return waypoints


def _whole_jaw_preflight(sim, boxes, angles, profile, vertices, points=None):
    """Plan outward once, then check its exact joint reversal in a copy."""
    copy = SimpleNamespace(model=sim.model, data=mujoco.MjData(sim.model),
                           arm_indices=sim.arm_indices, forbidden_contact=sim.forbidden_contact)
    copy.data.qpos[:] = sim.data.qpos
    ik = {a: RobotVertexIK(copy, a, vertices[a]) for a in _SIDES}
    reversed_paths = {}
    for a in _SIDES:
        point = _point(boxes[a], profile, a, angles[a]) if points is None else np.asarray(points[a])
        theta = math.radians(angles[a])
        normal = boxes[a][:3, :3] @ np.array([(-1 if a == 'left' else 1)*math.cos(theta), 0., math.sin(theta)])
        q, _ = ik[a].solve(point, profile['per_side'][a]['seed'])
        planner = JointPathPlanner(copy, a, clearance=.006, allowed_flaps=('short_'+a+'_cardboard',))
        if not planner.valid(q):
            raise ValueError(a+' whole-jaw endpoint collides: '+str(planner.last_collision))
        outward = [q.copy()]
        for gap in np.linspace(.001, .050, 50):
            goal, _ = ik[a].solve(point+gap*normal, q)
            if not planner.edge(q, goal):
                raise ValueError(a+' whole-jaw outward path collides: '+str(planner.last_collision))
            q = goal
            outward.append(q.copy())
        reversed_paths[a] = list(reversed(outward))
    waypoints = _subdivide_reversed_joint_paths(reversed_paths, ik)
    free = {}
    for a in _SIDES:
        free[a] = JointPathPlanner(copy, a, clearance=.006).plan(waypoints[0][a], max_seconds=4.)
        copy.data.qpos[copy.arm_indices[a][:5]] = waypoints[0][a]
    for goals in waypoints[1:]:
        _paired_edge(copy, goals)
        for a, q in goals.items():
            copy.data.qpos[copy.arm_indices[a][:5]] = q
    return free, waypoints


def _exact_waypoint_details(sim, ik, goals, targets):
    """Audit a checked joint waypoint; reject jumps instead of re-solving IK."""
    points, details = {}, {}
    for a in _SIDES:
        actual = ik[a].point(sim.data.qpos[sim.arm_indices[a][:5]]).copy()
        setpoint_q = _actuator_setpoint(sim, a)
        setpoint = ik[a].point(setpoint_q).copy()
        point = ik[a].point(goals[a]).copy()
        increment = point-setpoint
        if not np.isfinite(increment).all() or np.linalg.norm(increment) > .000501:
            raise ValueError('Exact normal waypoint exceeds checked 0.5 mm CAD command increment')
        points[a] = point
        details[a] = dict(actual_point_world=actual.tolist(), sensor_target_world=np.asarray(targets[a]).tolist(),
            command_point_world=point.tolist(), requested_distance_m=float(np.linalg.norm(np.asarray(targets[a])-actual)),
            actual_fk_step_m=float(np.linalg.norm(point-actual)), contact_policy='exact_checked_reverse_joint_path',
            command_reference_world=setpoint.tolist(), commanded_increment_world=increment.tolist(),
            commanded_fk_increment_m=float(np.linalg.norm(increment)), ik_goal_point_world=point.tolist(),
            current_actuator_setpoint_radians=setpoint_q.tolist(), current_setpoint_fk_world=setpoint.tolist(),
            actual_minus_setpoint_world=(actual-setpoint).tolist())
    return points, details


def _entry_route_endpoint_drifts(targets, entry_targets):
    """Fresh geometry can invalidate an entry-bound route, never retarget it."""
    drifts = {}
    for side in _SIDES:
        current, entry = np.asarray(targets[side]), np.asarray(entry_targets[side])
        if current.shape != (3,) or entry.shape != (3,) or not np.isfinite(np.r_[current, entry]).all():
            raise ValueError('Finite fresh and entry-bound short endpoints required')
        drifts[side] = float(np.linalg.norm(current-entry))
        if drifts[side] > .035:
            raise ValueError('Fresh short endpoint drift exceeds original 35 mm Cartesian bound for exact route')
    return drifts


def _paired_edge(sim, goals):
    """Check both interpolated robot arms together in planner copies only."""
    planners = {a: JointPathPlanner(sim, a, clearance=.006,
                                   allowed_flaps=('short_'+a+'_cardboard',)) for a in _SIDES}
    starts = {a: np.clip(sim.data.qpos[sim.arm_indices[a][:5]],
                         planners[a].limits[:, 0], planners[a].limits[:, 1]) for a in _SIDES}
    count = max(1, int(np.ceil(max(np.max(np.abs(goals[a]-starts[a])) for a in _SIDES)/.025)))
    for u in np.linspace(0., 1., count+1):
        values = {a: starts[a]+u*(goals[a]-starts[a]) for a in _SIDES}
        for a, planner in planners.items():
            for other in _SIDES:
                planner.data.qpos[sim.arm_indices[other][:5]] = values[other]
            if not planner.valid(values[a]):
                raise ValueError('Paired '+a+' path collides: '+str(planner.last_collision))


def _preflight(sim, boxes, angles, profile, vertices):
    # Explicit offline obstacle/robot copy. Only robot joints in this copy are
    # assigned. Even the planning flap configuration stays as currently seen
    # by the simulator; its truth is not a control target or success reading.
    copy = SimpleNamespace(model=sim.model, data=mujoco.MjData(sim.model),
                           arm_indices=sim.arm_indices, forbidden_contact=sim.forbidden_contact)
    copy.data.qpos[:] = sim.data.qpos
    ik = {a: RobotVertexIK(copy, a, vertices[a]) for a in _SIDES}
    points = {a: _point(boxes[a], profile, a, angles[a]) for a in _SIDES}
    contact = {a: ik[a].solve(points[a], profile[a][1])[0] for a in _SIDES}
    pre = {a: ik[a].solve(points[a]+[0., 0., .020], contact[a])[0] for a in _SIDES}
    paths = {}
    for a in _SIDES:
        paths[a] = JointPathPlanner(copy, a, clearance=.006).plan(pre[a], max_seconds=4.)
        copy.data.qpos[copy.arm_indices[a][:5]] = pre[a]
    previous = pre
    for u in np.linspace(.05, 1., 20):
        # Preflight the complete free transit and a 5 mm elevated standoff.
        # The remaining contact descent is replanned against the physically
        # responding panels after every <=0.5 mm joint-driven CAD substep.
        goals = {a: ik[a].solve(points[a]+[0., 0., .020-.015*u], previous[a])[0] for a in _SIDES}
        _paired_edge(copy, goals)
        for a in _SIDES:
            copy.data.qpos[copy.arm_indices[a][:5]] = goals[a]
        previous = goals
    return paths


def probe_shorts_against_passive_majors(sim, controller, *, capture=False, target_degrees=10.,
                                      contact_policy='measured_v2', approach_policy='elevated_v2',
                                      stroke_step_degrees=.25):
    """Opt-in paired short attempt, with freely moving partial major panels."""
    if not math.isfinite(target_degrees) or not 0 <= target_degrees <= 10:
        raise ValueError('Bounded short probe target must be 0 to 10 degrees')
    if contact_policy not in _CONTACT_POLICIES:
        raise ValueError('Unknown bounded short contact policy')
    if approach_policy not in _APPROACH_POLICIES:
        raise ValueError('Unknown bounded short approach policy')
    if (isinstance(stroke_step_degrees, bool) or not isinstance(stroke_step_degrees, (float, int))
            or stroke_step_degrees not in _STROKE_STEPS_DEGREES):
        raise ValueError('Declared short stroke increment must be 0.25, 0.5 or 1 degree')
    step_tag = {.25: '0p25', .5: '0p5', 1.: '1'}[float(stroke_step_degrees)]
    c = controller
    if getattr(c, 'partial_short_probe', None) is not None:
        raise ValueError('Partial short probe already attempted; faults cannot be reset')
    prior = getattr(c, 'partial_major_release', None)
    if (not isinstance(prior, dict) or prior.get('fault')
            or prior.get('stage') != 'both partial major angles passively retained for five seconds'
            or prior.get('both_hands_parked') is not True
            or prior.get('both_majors_passively_retained') is not True
            or prior.get('expected_near_degrees') != 40.
            or prior.get('expected_far_degrees') != 35. or not prior.get('checks')):
        raise ValueError('Verified both-hands-parked passive near40/far35 stage required')
    report = dict(simulation_only=True, hardware_commands=0, full_task_complete=False,
                  all_flaps_closed=False, bounded_component_only=True, joint_driven_physics=True,
                  stage='require fresh four-flap entry', fault=None, target_short_degrees=target_degrees,
                  majors_remain_free=True, target_source='fresh RGB-D short angles and carton pose',
                  contact_radius_m=.14, normal_offset_m=.0015, candidates=[], checks=[])
    report.update(trajectory_policy=f'source_coherent_0p5mm_{step_tag}deg_v2', baseline_commit='ecbbb00',
                  target_source='current raw source-camera carton pose paired with its own RGB-D short angle',
                  max_contact_point_step_m=.0005, max_contact_approach_commands=80,
                  contact_approach_endpoint_tolerance_m=.00075,
                  contact_approach_observed_advance_transition_degrees=1.,
                  preflight_scope='free transit and 5 mm elevated standoff; each subsequent contact substep checked at execution',
                  contact_commands=[])
    report['contact_policy'] = contact_policy
    report['approach_policy'] = approach_policy
    report['contact_increment_reference'] = 'measured encoder FK'
    if contact_policy == 'setpoint_feedback_v3':
        report.update(trajectory_policy=f'source_coherent_setpoint_feedback_0p5mm_{step_tag}deg_v3',
                      contact_increment_reference='current actuator-setpoint FK',
                      contact_feedback='setpoint FK plus capped sensed-target-minus-actual-FK error',
                      max_contact_point_step_m=None, max_commanded_setpoint_increment_m=.0005,
                      original_physical_gates_unchanged=True)
    if contact_policy == 'tangent_deadband_v4':
        report.update(trajectory_policy=f'source_coherent_tangent_deadband_0p5mm_{step_tag}deg_v4',
                      contact_increment_reference='current actuator-setpoint FK',
                      contact_feedback='setpoint FK plus capped measured fold-direction lead only; '
                                       'radial and hinge errors corrected only beyond explicit deadbands',
                      tangent_deadbands_m=dict(_TANGENT_DEADBANDS_M),
                      fold_direction_retreat_allowed=False,
                      fold_frame_source='current source-camera carton registration and its own measured short angle',
                      max_contact_point_step_m=None, max_commanded_setpoint_increment_m=.0005,
                      original_physical_gates_unchanged=True)
    if contact_policy == 'jaw_surface_v5':
        report.update(trajectory_policy=f'source_coherent_jaw_surface_0p5mm_{step_tag}deg_v5',
                      contact_increment_reference='current actuator-setpoint FK',
                      contact_feedback='fold lead = encoder-FK jaw hull clearance to the sensed outer panel '
                                       'plane plus a fixed press, capped; radial and hinge errors corrected '
                                       'only beyond explicit deadbands',
                      jaw_press_m=_JAW_PRESS_M, tangent_deadbands_m=dict(_TANGENT_DEADBANDS_M),
                      fold_direction_retreat_allowed=False,
                      sensed_panel_source='current source-camera carton registration and its own measured short angle',
                      jaw_surface_source='permitted jaw collision meshes at encoder FK; no carton state',
                      max_contact_point_step_m=None, max_commanded_setpoint_increment_m=.0005,
                      original_physical_gates_unchanged=True)
    report['stroke_step_degrees'] = float(stroke_step_degrees)
    c.partial_short_probe = report

    def contact_targets(sources, profile, degrees):
        return {a: _point(np.asarray(sources[a]['world_from_box']), profile, a, degrees[a]) for a in _SIDES}
    try:
        reading = c.sense('Register both shorts and passive majors before bounded paired probe')
        guards = {f: ContactProgressGuard(f, reading) for f in _FLAPS}
        if reading['seq'] <= prior['checks'][-1]['seq']:
            raise ValueError('New four-flap observation after passive release required')
        for f, target in (('long_near', 40.), ('long_far', 35.)):
            _verify_target_angle(reading, f, target)
        angles = {a: float(reading['angles']['short_'+a]['degrees']) for a in _SIDES}
        if not all(-25 <= degrees <= 0 for degrees in angles.values()):
            raise ValueError('Fresh outward short angles in -25 to 0 degrees required')
        for a in _SIDES:
            park = np.array([-.20 if a == 'left' else .20, -.18, .30])
            if np.linalg.norm(sim.actual_control_position(a)-park) > .035:
                raise ValueError(a+' hand is not within original park tracking bound')
        report['visual_checks'] = {f: g.checks for f, g in guards.items()}
        coherent = {a: _contact_reading(reading, a) for a in _SIDES}
        contact_guards = {a: ContactProgressGuard('short_'+a, coherent[a][0]) for a in _SIDES}
        report['source_contact_visual_checks'] = {a: g.checks for a, g in contact_guards.items()}
        report['initial_contact_sources'] = {a: coherent[a][1] for a in _SIDES}
        major_commands = {f: reading['angles'][f]['degrees'] for f in ('long_near', 'long_far')}
        major_limits = {f: np.degrees(sim.model.joint(f+'_hinge').range) for f in major_commands}
        chosen = None
        normal_waypoints = None
        if approach_policy == 'whole_jaw_normal_v1':
            name, profile = 'whole_jaw_central_left_front_right', dict(along=0., per_side=_WHOLE_JAW)
            vertices = {a: _validated_jaw_vertex(sim, a, _WHOLE_JAW[a]['vertex']) for a in _SIDES}
            boxes = {a: np.asarray(coherent[a][0]['world_from_box']) for a in _SIDES}
            entry_targets = contact_targets({a: coherent[a][1] for a in _SIDES}, profile, angles)
            paths, normal_waypoints = _whole_jaw_preflight(sim, boxes, angles, profile, vertices, entry_targets)
            chosen = (name, profile, vertices, paths)
            report['candidates'].append(dict(profile=name, free_and_complete_normal_path_clear=True,
                                             static_preflight_only=True))
            report.update(preflight_scope='free transit and complete paired outside-normal exact joint path; runtime checks remain required',
                target_source='exact normal route bound to entry source-camera geometry; fresh geometry monitored after each command; bounded stroke uses current source-camera targets',
                contact_surface_scope='original permitted fixed and moving jaw meshes; proximal left jaw surface is not a distal fingertip',
                normal_approach_travel_m=.050, normal_approach_point_increment_m=.0005,
                normal_approach_waypoint_count=len(normal_waypoints)-1,
                normal_approach_sources={a: coherent[a][1] for a in _SIDES},
                normal_approach_entry_targets_world={a:entry_targets[a].tolist() for a in _SIDES},
                normal_approach_joint_waypoints=[{a:q.tolist() for a,q in row.items()} for row in normal_waypoints],
                normal_approach_increment_reference='current actuator-setpoint FK; not a physical-motion bound',
                normal_approach_fk_increment_allowance_m=.000001,
                normal_approach_fresh_endpoint_drift_limit_m=.035,
                normal_approach_fresh_box_guards='existing 15 mm translation and 8 degree rotation limits remain active',
                bounded_stroke_increment_reference=report['contact_increment_reference'],
                contact_increment_reference='stage-specific: exact normal route uses setpoint FK; bounded stroke uses its selected contact policy',
                max_contact_point_step_scope='bounded stroke only; exact normal waypoint increments checked separately',
                any_short_advance_transition_scope='one short advancing does not establish contact or progress of the other short',
                normal_waypoint_budget='derived from the finite 50 mm planned path with checked joint-edge subdivision',
                max_contact_approach_commands=len(normal_waypoints)-1,
                normal_offset_m=None, normal_offset_by_arm_m={a:_WHOLE_JAW[a]['offset'] for a in _SIDES})
        for name, profile in (() if chosen is not None else _PROFILES.items()):
            vertices = {}
            for a in _SIDES:
                actual_vertices = mesh_contact_vertices(sim.model, sim.data, a)
                vertices[a] = min(actual_vertices, key=lambda v: np.linalg.norm(np.asarray(v['local'])-profile[a][0]))
                if np.linalg.norm(np.asarray(vertices[a]['local'])-profile[a][0]) > 1e-6:
                    raise ValueError('Declared short contact vertex absent from original CAD')
            try:
                boxes = {a: np.asarray(coherent[a][0]['world_from_box']) for a in _SIDES}
                paths = _preflight(sim, boxes, angles, profile, vertices)
                chosen = (name, profile, vertices, paths)
                report['candidates'].append(dict(profile=name, free_and_elevated_standoff_clear=True))
                break
            except ValueError as exc:
                report['candidates'].append(dict(profile=name, free_and_elevated_standoff_clear=False, refusal=str(exc)))
        if chosen is None:
            raise ValueError('Neither declared paired short approach passes original clearance gates')
        name, profile, vertices, paths = chosen
        report.update(contact_profile=name, contact_along_m=profile['along'], actual_cad_vertices=vertices)
        if normal_waypoints is not None:
            report.update(contact_along_m=None, contact_along_by_arm_m={a:_WHOLE_JAW[a]['along'] for a in _SIDES})
        ik = {a: RobotVertexIK(sim, a, vertices[a]) for a in _SIDES}

        def observe(label, commands, *, require_partial_majors=False):
            current = c.sense(label)
            for f, guard in guards.items():
                guard.check(current, commands[f[6:]] if f.startswith('short_') else major_commands[f])
            for a in _SIDES:
                source_reading, _ = _contact_reading(current, a)
                contact_guards[a].check(source_reading, commands[a])
            observed = {a: current['angles']['short_'+a]['degrees'] for a in _SIDES}
            if not all(-25 <= value <= target_degrees+5 for value in observed.values()):
                raise ValueError('Observed short angle left bounded probe range')
            for f, limits in major_limits.items():
                if not limits[0] <= current['angles'][f]['degrees'] <= limits[1]:
                    raise ValueError('Observed passive major left original hinge range')
            if require_partial_majors:
                _verify_target_angle(current, 'long_near', 40.)
                _verify_target_angle(current, 'long_far', 35.)
            report['checks'].append(dict(time=float(sim.data.time), seq=current['seq'],
                                         visual_angles=current['angles']))
            return current, observed

        def command(goals, points, label, row):
            row['execution_status'] = 'checking actual-to-goal path'
            _paired_edge(sim, goals)
            duration = max(.25, max(float(np.max(np.abs(goals[a]-sim.data.qpos[sim.arm_indices[a][:5]])))
                                    for a in _SIDES)/.55)
            event = sim.move({}, duration, label, capture=capture, joint_targets=goals)
            row['execution_status'] = 'motion returned; checking original runtime gates'
            row['event_time'] = float(sim.data.time)
            for a in _SIDES:
                actual = ik[a].point(sim.data.qpos[sim.arm_indices[a][:5]]).copy()
                setpoint_q = _actuator_setpoint(sim, a)
                setpoint = ik[a].point(setpoint_q).copy()
                detail = row['robot_substeps'][a]
                detail.update(postmotion_actual_point_world=actual.tolist(),
                    postmotion_actuator_setpoint_radians=setpoint_q.tolist(),
                    postmotion_setpoint_fk_world=setpoint.tolist(),
                    postmotion_actual_minus_setpoint_world=(actual-setpoint).tolist(),
                    observed_fk_displacement_world=(actual-np.asarray(detail['actual_point_world'])).tolist())
            if event.get('step_error') or event['bad_penetration_mm'] > 1.:
                raise ValueError(event.get('step_error') or 'Forbidden collision during paired short probe')
            if event['max_joint_tracking_error_radians'] > .08:
                raise ValueError('Paired short joint tracking exceeds 0.08 rad')
            for a in _SIDES:
                if np.linalg.norm(ik[a].point(sim.data.qpos[sim.arm_indices[a][:5]])-points[a]) > .035:
                    raise ValueError('Paired short CAD tracking exceeds 35 mm')
            row['execution_status'] = 'completed within original runtime gates'

        with _PanelStepAudit(sim, report) as panel_audit:
            report['stage'] = 'checked free paired short approach'
            for a in _SIDES:
                current_path = JointPathPlanner(sim, a, clearance=.006).plan(paths[a][-1])
                label = ('Reach outside '+a+' short face' if normal_waypoints is not None
                         else 'Reach above '+a+' outward short edge')
                execute_path(sim, a, current_path, label, capture=capture)
                reading, angles = observe('Check all four flaps after short free transit', angles,
                                          require_partial_majors=True)
            report['stage'] = 'checked paired short contact approach'
            approach_angles = dict(angles)
            if normal_waypoints is not None:
                report['stage'] = 'checked exact whole-jaw normal approach'
                for index, goals in enumerate(normal_waypoints[1:], 1):
                    if any(angles[a]-approach_angles[a] >= 1. for a in _SIDES):
                        report['contact_approach_transition'] = 'fresh short advance reached 1 degree on exact normal path; switch to bounded stroke'
                        break
                    sources = {a:_contact_reading(reading, a)[1] for a in _SIDES}
                    targets = contact_targets(sources, profile, angles)
                    drifts = _entry_route_endpoint_drifts(targets, report['normal_approach_entry_targets_world'])
                    points, details = _exact_waypoint_details(sim, ik, goals, targets)
                    for a in _SIDES:
                        details[a].update(entry_route_target_world=report['normal_approach_entry_targets_world'][a],
                                          fresh_endpoint_drift_m=drifts[a])
                    row = dict(stage='normal_approach', seq=reading['seq'], sources=sources,
                        planned_source_sequences={a:coherent[a][1]['seq'] for a in _SIDES},
                        normal_waypoint=index, command_degrees=dict(angles), robot_substeps=details)
                    report['contact_commands'].append(row)
                    command(goals, points, 'Follow exact checked whole-jaw normal joint approach', row)
                    reading, angles = observe('Check fresh shorts and free majors after exact normal waypoint', angles)
                else:
                    report['contact_approach_transition'] = 'complete exact normal joint path; actual contact and folding progress not asserted'
            for _ in range(0 if normal_waypoints is not None else 80):
                sources = {a: _contact_reading(reading, a)[1] for a in _SIDES}
                targets = contact_targets(sources, profile, angles)
                frames = ({a: _fold_frame(np.asarray(sources[a]['world_from_box']), a, angles[a]) for a in _SIDES}
                          if contact_policy == 'tangent_deadband_v4' else None)
                panels = (_sensed_panels(sources, profile, angles)
                          if contact_policy == 'jaw_surface_v5' else None)
                goals, points, details = _bounded_contact_goals(sim, ik, targets, contact_policy=contact_policy,
                                                                frames=frames, panels=panels)
                if all(row['requested_distance_m'] <= .00075 for row in details.values()):
                    report['contact_approach_transition'] = 'CAD points near fresh sensed targets; contact not asserted'
                    break
                if any(angles[a]-approach_angles[a] >= 1. for a in _SIDES):
                    # A small pose bias can leave a sensed endpoint inside a
                    # responding panel. Do not chase it through further folds:
                    # switch to the bounded angle probe after visible response.
                    report['contact_approach_transition'] = 'fresh short advance reached 1 degree; switch to bounded angle commands'
                    break
                row = dict(stage='approach', seq=reading['seq'],
                    sources=sources, command_degrees=dict(angles), robot_substeps=details)
                report['contact_commands'].append(row)
                command(goals, points, 'Approach both outward shorts using actual CAD fingers', row)
                reading, angles = observe('Check both shorts and free majors during contact approach', angles)
            else:
                if normal_waypoints is None:
                    raise ValueError('Short contact approach exceeded 80 conservative CAD substeps')
            for a in _SIDES:
                guards['short_'+a].begin_stroke(reading)
                contact_guards[a].begin_stroke(_contact_reading(reading, a)[0])
            stroke = _ShortStroke(angles, target_degrees, step_degrees=stroke_step_degrees)
            report['measured_stroke'] = dict(max_advance_degrees=stroke.step_degrees, max_commands=stroke.max_commands,
                                             stall_window_commands=stroke.stall_window,
                                             minimum_window_progress_degrees=2.,
                                             attempted_advance_window_degrees=12.,
                                             observed_degrees=stroke.history)
            report['stage'] = 'bounded simultaneous short-fold probe'
            while (commands := stroke.next_angles()) is not None:
                sources = {a: _contact_reading(reading, a)[1] for a in _SIDES}
                targets = contact_targets(sources, profile, commands)
                frames = ({a: _fold_frame(np.asarray(sources[a]['world_from_box']), a, angles[a]) for a in _SIDES}
                          if contact_policy == 'tangent_deadband_v4' else None)
                panels = (_sensed_panels(sources, profile, angles)
                          if contact_policy == 'jaw_surface_v5' else None)
                goals, points, details = _bounded_contact_goals(sim, ik, targets, contact_policy=contact_policy,
                                                                frames=frames, panels=panels)
                row = dict(stage='stroke', seq=reading['seq'],
                    sources=sources, command_degrees=commands, robot_substeps=details)
                report['contact_commands'].append(row)
                command(goals, points, 'Probe simultaneous short folding toward at most +10 degrees', row)
                reading, angles = observe('Observe whether free majors move during bounded short probe', commands)
                stroke.observe(angles)
            c.port.move_arms({}, .5, 'Hold bounded short probe for fresh verification', None)
            reading, angles = observe('Verify bounded paired short component', {a: target_degrees for a in _SIDES})
            for a in _SIDES:
                _verify_target_angle(reading, 'short_'+a, target_degrees)
            if not panel_audit.coverage_complete():
                raise ValueError('Complete per-step panel contact coverage required')
            report.update(stage='bounded paired short target visually held', bounded_target_verified=True,
                          visual_angles=reading['angles'], command_count=stroke.commands,
                          independent_final_angles=sim.truth_angles(), motion=dict(sim.motion_stats))
        return report
    except Exception as exc:
        report['fault'] = str(exc)
        raise
