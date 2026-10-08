"""Independent G4 assembly evidence capture and scoring.

The rigid guide primitive is implemented. The complete G4 workflow is an explicit
contract, not a success inferred from a subset: deformable paper feeding, rolling,
and sheet placement are not yet implemented and therefore cannot pass. Human
preparation, software tests, simulation, and physical validation remain separate.

Capture a row at reset, then for every step call mj_step, capture_contacts,
mj_forward, capture_assembly_state. The two contact sets are deliberate: one is
the solver load which acted during the step, the other is coherent with the
recorded resulting pose. Replaying each step detects unreported object resets.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path
from typing import Mapping
import zlib

import mujoco
import numpy as np

VERSION = 'g4-assembly-contract-v1'
STATE = mujoco.mjtState.mjSTATE_INTEGRATION


def capture_contacts(model, data):
    """Copy raw solver contact geometry and local wrench; no success booleans."""
    out = []
    for index in range(data.ncon):
        c = data.contact[index]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        out.append(dict(geom1=int(c.geom1), geom2=int(c.geom2),
                        position_m=c.pos.tolist(), frame=c.frame.reshape(3, 3).tolist(),
                        distance_m=float(c.dist), wrench=force.tolist()))
    return out


def capture_assembly_state(model, data, phase, step_index, *, step_contacts=None,
                           mutation_events=()):
    """Capture ground truth only after mj_forward; never use this for control."""
    state = np.zeros(mujoco.mj_stateSize(model, STATE))
    mujoco.mj_getState(model, data, state, STATE)
    return dict(schema=1, phase=str(phase), step_index=int(step_index),
                time_s=float(data.time), integration_state=state.tolist(),
                qpos=data.qpos.tolist(), qvel=data.qvel.tolist(), act=data.act.tolist(),
                ctrl=data.ctrl.tolist(), qacc_warmstart=data.qacc_warmstart.tolist(),
                eq_active=data.eq_active.tolist(), qfrc_applied=data.qfrc_applied.tolist(),
                xfrc_applied=data.xfrc_applied.tolist(), contacts=capture_contacts(model, data),
                step_contacts=[] if step_contacts is None else step_contacts,
                mutation_events=list(mutation_events))


@dataclass(frozen=True)
class AssemblyThresholds:
    """Provisional simulation gates, not measured hardware safety limits."""
    peak_contact_force_n: float = 8.
    peak_body_pair_force_n: float = 8.
    peak_penetration_m: float = .0002
    initial_penetration_m: float = .000001
    loaded_jaw_n: float = .005
    opposition_cosine: float = -.5
    acquisition_hold_s: float = .100
    carried_hold_s: float = .100
    released_hold_s: float = .500
    loaded_fraction: float = .95
    maximum_unloaded_gap_s: float = .010
    maximum_rigid_slip_m: float = .005
    minimum_lift_m: float = .020
    target_position_m: float = .0015
    target_rotation_rad: float = .035
    released_speed_m_s: float = .002
    released_angular_speed_rad_s: float = .02
    support_transfer_s: float = .040
    unintended_arm_contact_n: float = .005
    guide_removal_m: float = .028
    guide_lateral_extraction_m: float = .001
    state_replay_atol: float = 1e-8


RIGID_PHASES = ('approach_above', 'approach', 'acquire', 'transfer', 'hold',
                'place', 'release', 'withdraw', 'released_hold')
STAGE_ORDER = ('trough', 'holder', 'guide', 'carrier0', 'carrier1', 'carrier2',
               'carrier3', 'strip0', 'strip1', 'strip2', 'strip3', 'guide_remove',
               'fold3', 'fold2', 'fold1', 'fold0', 'sheet')
REQUIRED_BODIES = ('trough', 'holder', 'guide', 'carrier0', 'carrier1', 'carrier2',
                   'carrier3', 'pusher', 'roller', 'roller_wheel', 'strip0',
                   'strip1', 'strip2', 'strip3', 'sheet')


def full_assembly_contract():
    """Fixed task requirements. A caller cannot replace these with stage flags."""
    return dict(stage_order=list(STAGE_ORDER), required_bodies=list(REQUIRED_BODIES),
        completed_evaluators=['guide_rigid_transfer'],
        required_measurements={
            'trough_holder': 'Joint-driven loaded-jaw transfer, target pose, contact-supported free release and hold; initialized poses earn no credit.',
            'guide': 'Both opposed loaded jaws before lift, whole-body retention until measured upward seat support, target pose, free release and hold.',
            'carriers': 'Four distinct empty source carriers; source-slot alignment, guide capture, jaw release before gravity descent and independent seat support; no forced insertion.',
            'paper': 'Four deformable 46x42 mm prototype strips: measured node grid, passive bending/stretching model, carrier-supported tails, insertion depth, pusher contact and full withdrawal. A rigid paper token cannot qualify.',
            'guide_remove': 'Measured guide rise >=28 mm with <=1 mm lateral travel until clear, then park and release; all carrier and paper nodes remain seated.',
            'folds': 'Order 3,2,1,0 by source slot X; passive wheel contact moves each paper flap toward +X; measured wheel rotation, paper angle and contact footprint; lift clear between strips. No paper pose setters.',
            'sheet': 'Deformable plain 146x76 mm rectangle without notches, measured contact with all four folded strips, free robot release and hands-off retention of every assembled part.',
            'final': 'Complete per-step physics and applied-step contacts, no object actuation/weld/teleport, no missing stages, and final hands-off hold.'},
        paper_dimensions_status='Prototype assumptions, not physical cutting specifications',
        water_test='Separate human-supervised physical test; simulation cannot certify wicking',
        physical_success=False)


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _name(model, kind, index):
    return mujoco.mj_id2name(model, kind, index) or f'unnamed_{index}'


def _descendants(model, body):
    result = {body}
    for i in range(body + 1, model.nbody):
        if int(model.body_parentid[i]) in result:
            result.add(i)
    return result


def audit_model(model):
    """Inspect actual compiled transmissions/topology, not manifest assertions."""
    failures = []
    present = [name for name in REQUIRED_BODIES if _body_id(model, name) >= 0]
    missing = [name for name in REQUIRED_BODIES if name not in present]
    robot_joints = {f'{side}_{joint}' for side in ('left', 'right') for joint in
                    ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')}
    object_bodies = set()
    for name in present:
        object_bodies |= _descendants(model, _body_id(model, name))
    transmissions = []
    for a in range(model.nu):
        kind = int(model.actuator_trntype[a]); jid = int(model.actuator_trnid[a, 0])
        joint_name = _name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) if kind in (0, 1) else None
        allowed = (kind in (0, 1) and joint_name in robot_joints and bool(model.jnt_limited[jid])
                   and int(model.jnt_bodyid[jid]) not in object_bodies)
        transmissions.append(dict(actuator=_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a),
                                  joint=joint_name, allowed=allowed))
        if not allowed:
            failures.append('non_robot_or_unlimited_actuator')
    for name in present:
        if name == 'roller_wheel' or name.startswith('strip') or name == 'sheet':
            continue
        bid = _body_id(model, name)
        joints = np.flatnonzero(model.jnt_bodyid == bid)
        if len(joints) != 1 or int(model.jnt_type[joints[0]]) != int(mujoco.mjtJoint.mjJNT_FREE):
            failures.append(f'{name}_not_free')
        if model.body_mocapid[bid] >= 0:
            failures.append(f'{name}_mocap_driven')
    if model.neq:
        # No assembly equality attachment is approved. A future paper model must
        # implement and review passive internal constraints before extending this.
        failures.append('unreviewed_equality_constraint')
    if model.nmocap:
        failures.append('mocap_bodies_not_allowed')
    hinge = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'roller_wheel_hinge')
    if 'roller' in present and (hinge < 0 or int(model.jnt_type[hinge]) != int(mujoco.mjtJoint.mjJNT_HINGE)
                               or not np.allclose(model.jnt_axis[hinge], [0, 1, 0], atol=1e-10)):
        failures.append('passive_roller_hinge_missing_or_wrong_axis')
    return dict(passed=not failures, failure_reasons=sorted(set(failures)),
                present_bodies=present, missing_bodies=missing, actuator_transmissions=transmissions,
                object_body_ids=sorted(object_bodies),
                source_geometry_verified=False,
                source_geometry_boundary='Compiled dynamics audit does not certify CAD fidelity; exact source/collision geometry audit is a separate required qualification.')


def _finite_vector(value, shape):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError('wrong shape or nonfinite value')
    return array


def _validate_contacts(model, rows):
    if not rows:
        return 0., 0., 0.
    if any(type(c[k]) is not int or not 0 <= c[k] < model.ngeom
           for c in rows for k in ('geom1', 'geom2')):
        raise ValueError('invalid contact geom')
    count = len(rows)
    frames = _finite_vector([c['frame'] for c in rows], (count, 3, 3))
    _finite_vector([c['position_m'] for c in rows], (count, 3))
    wrenches = _finite_vector([c['wrench'] for c in rows], (count, 6))
    distances = _finite_vector([c['distance_m'] for c in rows], (count,))
    if np.any(wrenches[:, 0] < -1e-8):
        raise ValueError('invalid contact load')
    if not np.allclose(frames @ frames.transpose(0, 2, 1), np.eye(3), atol=1e-6):
        raise ValueError('invalid contact frame')
    magnitudes = np.linalg.norm(wrenches[:, :3], axis=1)
    body_pairs = np.sort(model.geom_bodyid[np.asarray([[c['geom1'], c['geom2']] for c in rows])], axis=1)
    pair_load = np.bincount(body_pairs[:, 0]*model.nbody+body_pairs[:, 1], weights=magnitudes).max()
    return float(magnitudes.max()), max(0., float(-distances.min())), float(pair_load)


def _contact_equal(left, right, atol):
    if len(left) != len(right):
        return False
    if not left:
        return True
    if any((a['geom1'], a['geom2']) != (b['geom1'], b['geom2']) for a, b in zip(left, right)):
        return False
    return all(np.allclose([a[key] for a in left], [b[key] for b in right], atol=atol, rtol=1e-7)
               for key in ('position_m', 'frame', 'distance_m', 'wrench'))


def validate_trace(model, rows, *, thresholds=AssemblyThresholds()):
    """Stream and replay every sample; retain only guide/arm contacts for scoring.

    All raw contacts still undergo finite, load, and replay checks. Keeping the
    thousands of parked carrier/table contacts per step is unnecessary in the
    derived guide score; the caller's original JSONL remains the raw evidence.
    """
    failures, states = [], []
    peak_force = peak_pair_force = peak_penetration = 0.
    count = 0; phase_runs = []
    expected = mujoco.mj_stateSize(model, STATE)
    previous = None
    data, replay = mujoco.MjData(model), mujoco.MjData(model)
    fields = {'qpos': (model.nq,), 'qvel': (model.nv,), 'act': (model.na,),
              'ctrl': (model.nu,), 'qacc_warmstart': (model.nv,),
              'eq_active': (model.neq,), 'qfrc_applied': (model.nv,),
              'xfrc_applied': (model.nbody, 6)}
    guide = _body_id(model, 'guide')
    guide_bodies = _descendants(model, guide) if guide >= 0 else set()
    relevant_geoms = set()
    for g, body in enumerate(model.geom_bodyid):
        if int(body) in guide_bodies or _name(model, mujoco.mjtObj.mjOBJ_BODY, int(body)).startswith(('left_', 'right_')):
            relevant_geoms.add(g)
    for i, row in enumerate(rows):
        count += 1
        try:
            if 'contact_codec' in row:
                from planter.g4_contact_codec import decode_contacts
                row = decode_contacts(row)
            if row['schema'] != 1 or type(row['step_index']) is not int or row['step_index'] != i:
                raise ValueError('index/schema')
            if not isinstance(row['phase'], str) or not row['phase']:
                raise ValueError('phase')
            if not phase_runs or phase_runs[-1] != row['phase']:
                phase_runs.append(row['phase'])
            state = _finite_vector(row['integration_state'], (expected,))
            mujoco.mj_setState(model, data, state, STATE)
            if not np.isfinite(row['time_s']) or abs(data.time-row['time_s']) > 1e-10:
                raise ValueError('time/state mismatch')
            if i == 0 and abs(data.time) > 1e-10:
                raise ValueError('trace must include initial reset')
            for key, shape in fields.items():
                array = _finite_vector(row[key], shape)
                if not np.allclose(array, getattr(data, key), atol=1e-12, rtol=0):
                    raise ValueError('state field mismatch: '+key)
            if np.any(data.qfrc_applied) or np.any(data.xfrc_applied):
                failures.append(f'external_applied_force_at_{i}')
            if row['mutation_events'] != []:
                failures.append(f'runtime_mutation_at_{i}')
            for j in range(model.njnt):
                typ = int(model.jnt_type[j]); adr = int(model.jnt_qposadr[j])
                if typ in (int(mujoco.mjtJoint.mjJNT_FREE), int(mujoco.mjtJoint.mjJNT_BALL)):
                    q = data.qpos[adr+3:adr+7] if typ == 0 else data.qpos[adr:adr+4]
                    if abs(np.linalg.norm(q)-1) > 1e-6:
                        raise ValueError('nonunit quaternion')
                elif model.jnt_limited[j] and not model.jnt_range[j, 0]-.001 <= data.qpos[adr] <= model.jnt_range[j, 1]+.001:
                    failures.append(f'joint_limit_at_{i}')
            for a in range(model.nu):
                if model.actuator_ctrllimited[a] and not model.actuator_ctrlrange[a, 0] <= data.ctrl[a] <= model.actuator_ctrlrange[a, 1]:
                    failures.append(f'control_limit_at_{i}')
            for key in ('contacts', 'step_contacts'):
                force, penetration, pair_force = _validate_contacts(model, row[key])
                peak_force = max(peak_force, force)
                peak_pair_force = max(peak_pair_force, pair_force)
                peak_penetration = max(peak_penetration, penetration)
                if i == 0 and penetration > thresholds.initial_penetration_m:
                    failures.append('initial_intersection')
            if i == 0 and row['step_contacts']:
                raise ValueError('initial sample has no applied step')
            mujoco.mj_forward(model, data)
            if not _contact_equal(row['contacts'], capture_contacts(model, data), 1e-7):
                failures.append(f'contact_state_mismatch_at_{i}')
            if previous is not None:
                if abs(data.time-previous['time_s']-model.opt.timestep) > 1e-9:
                    raise ValueError('missing or repeated timestep')
                mujoco.mj_setState(model, replay, np.asarray(previous['integration_state']), STATE)
                replay.ctrl[:] = row['ctrl']
                mujoco.mj_step(model, replay)
                if not _contact_equal(row['step_contacts'], capture_contacts(model, replay), 1e-6):
                    failures.append(f'applied_contact_replay_mismatch_at_{i}')
                for key in ('qpos', 'qvel', 'act'):
                    if not np.allclose(getattr(replay, key), row[key], atol=thresholds.state_replay_atol, rtol=1e-7):
                        failures.append(f'physics_replay_mismatch_at_{i}')
                        break
            compact = {key: [c for c in row[key] if c['geom1'] in relevant_geoms or c['geom2'] in relevant_geoms]
                       for key in ('contacts', 'step_contacts')}
            states.append(dict(time_s=float(data.time), phase=row['phase'],
                               xpos=data.xpos.copy(), xmat=data.xmat.copy().reshape(-1, 3, 3),
                               site_xpos=data.site_xpos.copy(), site_xmat=data.site_xmat.copy().reshape(-1, 3, 3),
                               qpos=data.qpos.copy(), qvel=data.qvel.copy(), **compact))
            previous = row
        except (KeyError, ValueError, TypeError, IndexError, OverflowError, zlib.error) as error:
            failures.append(f'invalid_sample_{i}: {error}')
            break
    if count == 0:
        failures.append('empty_trace')
    if peak_force > thresholds.peak_contact_force_n:
        failures.append('contact_force_limit')
    if peak_pair_force > thresholds.peak_body_pair_force_n:
        failures.append('body_pair_contact_force_limit')
    if peak_penetration > thresholds.peak_penetration_m:
        failures.append('contact_penetration_limit')
    return dict(passed=not failures, failure_reasons=sorted(set(failures)), states=states,
                phase_runs=phase_runs,
                metrics=dict(samples=count, replayed_transitions=max(0, len(states)-1),
                             peak_contact_force_n=peak_force, peak_body_pair_force_n=peak_pair_force,
                             peak_penetration_m=peak_penetration))


SOURCE_HASHES = {
    'trough': '5af456157c9492d5d08c91c981f051d8f2ce7de5e931b36cc9bebcdc5302c5f8',
    'holder': 'e4bd4281d031ce14d04ae58e3d353f7d9806f1aff131266eeb13181577c5aebf',
    'guide': '80310786665aabcbdc2d6cdd89fddcba7ebcfac279c7c2299f3d96cd336375aa',
    'carrier': '9ec76dbfad2515a849a0e036553c736daa9be0547ad887097e4ab543b8f98d72',
    'roller_wheel': '798f404c1d23fa19e0f484a4ddedf8607b9c22919b0601d9fc74ecd8fb80a296',
    'roller_handle': '1e733e9b37573bbe007f041932a0311b5e9684102711ccd2999456fc75fdf8be',
}


def audit_source_visuals(model, source_paths):
    """Bind supplied files to known G4 sources and the actual compiled meshes.

    This catches a source-hash manifest attached to an unrelated scene. It does
    not certify a convex collision decomposition's local surface normals.
    """
    import trimesh
    from scipy.spatial import cKDTree
    result = {}
    for part, path in source_paths.items():
        try:
            if part not in SOURCE_HASHES:
                raise ValueError('unrecognized source role')
            digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            if digest != SOURCE_HASHES[part]:
                raise ValueError('not the audited G4 source')
            mesh = trimesh.load(path, force='mesh')
            expected = np.asarray(mesh.vertices) * .001
            names = [f'carrier{i}_visual' for i in range(4)] if part == 'carrier' else [part+'_visual']
            distances = []
            for name in names:
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                if gid < 0 or int(model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_MESH):
                    raise ValueError('compiled source visual absent: '+name)
                mid = int(model.geom_dataid[gid]); first = int(model.mesh_vertadr[mid]); count = int(model.mesh_vertnum[mid])
                owner = name.removesuffix('_visual')
                if owner == 'roller_handle':
                    owner = 'roller'
                if int(model.geom_bodyid[gid]) != _body_id(model, owner):
                    raise ValueError('source visual attached to wrong body')
                rotation = np.zeros(9); mujoco.mju_quat2Mat(rotation, model.geom_quat[gid])
                # einsum avoids Apple Accelerate's spurious mixed-precision,
                # strided-matmul warnings on compiled float32 mesh buffers.
                actual = np.einsum('ij,kj->ik', np.asarray(model.mesh_vert[first:first+count], dtype=float),
                                   rotation.reshape(3, 3)) + model.geom_pos[gid]
                distance = max(cKDTree(actual).query(expected)[0].max(), cKDTree(expected).query(actual)[0].max())
                distances.append(float(distance))
            if max(distances) > 1e-7:
                raise ValueError('source-to-body compiled transform mismatch')
            result[part] = dict(passed=True, sha256=digest, compiled_vertex_error_m=max(distances))
        except (OSError, ValueError, TypeError, IndexError) as error:
            result[part] = dict(passed=False, reason=str(error))
    return result


def _runs(mask, dt):
    longest = length = 0
    for value in mask:
        length = length+1 if value else 0
        longest = max(longest, length)
    return longest*dt


def _contact_force_on(model, contact, bodies):
    b1, b2 = (int(model.geom_bodyid[contact[k]]) for k in ('geom1', 'geom2'))
    if (b1 in bodies) == (b2 in bodies):
        return None, None, None
    on_second = b2 in bodies
    frame = np.asarray(contact['frame']); local = np.asarray(contact['wrench'])
    force = frame.T @ local[:3] * (1 if on_second else -1)
    normal = frame[0] * (1 if on_second else -1)
    other_geom = contact['geom1'] if on_second else contact['geom2']
    return force, normal, other_geom


def _guide_sample(model, state, bodies, thresholds):
    jaws = {side: {'fixed': [], 'moving': []} for side in ('left', 'right')}
    support = 0.; arm_load = 0.
    for contact in state['contacts']:
        force, normal, other = _contact_force_on(model, contact, bodies)
        if force is None:
            continue
        name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
        other_body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[other]))
        if other_body == 'holder':
            support += max(0., float(force[2]))
        if contact['wrench'][0] <= thresholds.loaded_jaw_n:
            continue
        for side in jaws:
            if name.startswith(side+'_'):
                if 'moving_jaw' in name:
                    jaws[side]['moving'].append(normal)
                elif 'wrist_roll_follower' in name:
                    jaws[side]['fixed'].append(normal)
    # Check applied-step loads as well as the refreshed final state. Arm-base
    # contacts are not hidden by excluding only the end-effector geoms.
    for contact in state['contacts'] + state['step_contacts']:
        if np.linalg.norm(contact['wrench'][:3]) <= thresholds.unintended_arm_contact_n:
            continue
        ids = [int(model.geom_bodyid[contact[k]]) for k in ('geom1', 'geom2')]
        names = [_name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in ids]
        sides = [next((s for s in jaws if n.startswith(s+'_')), None) for n in names]
        unintended = any(sides) and not any(b in bodies for b in ids) and not (sides[0] and sides[0] == sides[1])
        if unintended:
            arm_load = max(arm_load, float(np.linalg.norm(contact['wrench'][:3])))
    loaded = {}
    for side, groups in jaws.items():
        loaded[side] = any(np.dot(a, b) <= thresholds.opposition_cosine
                           for a in groups['fixed'] for b in groups['moving'])
    return dict(loaded=loaded, support_n=support, unintended_arm_load_n=arm_load)


def score_guide_transfer(model, states, *, thresholds=AssemblyThresholds()):
    """Measured bimanual transfer from loose staging through supported release."""
    prefix = 'guide_'
    selected = [s for s in states if s['phase'] in [prefix+p for p in RIGID_PHASES]]
    failures = []
    bid = _body_id(model, 'guide')
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'guide_seat_target')
    if bid < 0 or target < 0 or model.site_bodyid[target] != 0:
        return dict(passed=False, failure_reasons=['guide_or_fixed_target_missing'])
    if not selected:
        return dict(passed=False, failure_reasons=['guide_transfer_not_recorded'])
    observed = []
    for s in selected:
        phase = s['phase'][len(prefix):]
        if not observed or observed[-1] != phase:
            observed.append(phase)
    if observed != list(RIGID_PHASES):
        return dict(passed=False, failure_reasons=['guide_phase_order_or_completion'], observed_phases=observed)
    dt = float(model.opt.timestep)
    bodies = _descendants(model, bid)
    metrics = [_guide_sample(model, s, bodies, thresholds) for s in selected]
    indices = {p: [i for i, s in enumerate(selected) if s['phase'] == prefix+p] for p in RIGID_PHASES}
    first = selected[0]
    if np.linalg.norm(first['xpos'][bid]-first['site_xpos'][target]) <= thresholds.target_position_m:
        failures.append('guide_initialized_at_target_not_robot_placed')
    for i in indices['approach_above']:
        if any(metrics[i]['loaded'].values()):
            failures.append('guide_initialized_in_loaded_grasp')
    acquire = indices['acquire']
    count = int(np.ceil(thresholds.acquisition_hold_s/dt))
    acquisition = acquire[-count:]
    if len(acquisition) < count:
        failures.append('acquisition_hold_too_short')
    loaded = [all(m['loaded'].values()) for m in metrics]
    pregrasp = [loaded[i] for i in acquisition]
    if not pregrasp or np.mean(pregrasp) < thresholds.loaded_fraction or _runs([not x for x in pregrasp], dt) > thresholds.maximum_unloaded_gap_s+1e-10:
        failures.append('bimanual_opposed_grasp_not_established_before_transfer')
    first_carry, last_place = indices['transfer'][0], indices['place'][-1]
    anchor = next((i for i in reversed(acquisition) if loaded[i]), acquire[-1])
    # Seat support must be upward, close to the stationary target, and sustained
    # before intentional release. Side brushing cannot terminate retention.
    streak = 0; setdown = None
    for i in indices['place']:
        s = selected[i]
        close = np.linalg.norm(s['xpos'][bid]-s['site_xpos'][target]) <= thresholds.target_position_m
        supported = close and metrics[i]['support_n'] > thresholds.loaded_jaw_n
        streak = streak+1 if supported else 0
        if streak*dt >= thresholds.support_transfer_s-1e-10:
            setdown = i-streak+1
            break
    if setdown is None:
        failures.append('supported_setdown_not_observed_before_release')
    end = last_place if setdown is None else setdown
    carry_indices = list(range(anchor, end+1))
    retained = [loaded[i] for i in carry_indices]
    if not retained or np.mean(retained) < thresholds.loaded_fraction or _runs([not x for x in retained], dt) > thresholds.maximum_unloaded_gap_s+1e-10:
        failures.append('opposed_grasp_lost_before_setdown')
    vertices = np.array([[x, y, z] for x in (-.0749, .0749) for y in (-.0448, .0448) for z in (0., .05)])
    drift = 0.
    for side in ('left', 'right'):
        grip = _body_id(model, side+'_gripper_link')
        if grip < 0:
            failures.append(side+'_gripper_frame_missing')
            continue
        def relative(s):
            world = vertices @ s['xmat'][bid].T+s['xpos'][bid]
            return (world-s['xpos'][grip]) @ s['xmat'][grip]
        reference = relative(selected[anchor])
        for i in carry_indices:
            drift = max(drift, float(np.linalg.norm(relative(selected[i])-reference, axis=1).max()))
    if drift > thresholds.maximum_rigid_slip_m:
        failures.append('whole_guide_grasp_drift')
    hold = indices['hold']
    lift = min(float(selected[i]['xpos'][bid, 2]-selected[anchor]['xpos'][bid, 2]) for i in hold)
    if len(hold)*dt < thresholds.carried_hold_s-1e-10 or lift < thresholds.minimum_lift_m:
        failures.append('carried_hold_or_lift_incomplete')
    released = indices['released_hold']
    if len(released)*dt < thresholds.released_hold_s-1e-10:
        failures.append('released_hold_too_short')
    max_pos = max_rotation = max_speed = max_angular = 0.
    jid = next((int(j) for j in np.flatnonzero(model.jnt_bodyid == bid) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE), None)
    if jid is None:
        failures.append('guide_not_free')
    for i in released:
        s = selected[i]
        max_pos = max(max_pos, float(np.linalg.norm(s['xpos'][bid]-s['site_xpos'][target])))
        cosine = (np.trace(s['xmat'][bid].T @ s['site_xmat'][target])-1)/2
        max_rotation = max(max_rotation, float(np.arccos(np.clip(cosine, -1, 1))))
        if any(metrics[i]['loaded'].values()):
            failures.append('guide_not_released')
        # Also reject unilateral robot support, not only an opposed pinch.
        for c in s['contacts']:
            f, _, other = _contact_force_on(model, c, bodies)
            if f is not None and np.linalg.norm(f) > thresholds.loaded_jaw_n:
                name = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[other]))
                if name.startswith(('left_', 'right_')):
                    failures.append('guide_not_released')
        if metrics[i]['support_n'] <= thresholds.loaded_jaw_n:
            failures.append('guide_support_lost_after_release')
        if jid is not None:
            adr = int(model.jnt_dofadr[jid]); velocity = s['qvel'][adr:adr+6]
            max_speed = max(max_speed, float(np.linalg.norm(velocity[:3])))
            max_angular = max(max_angular, float(np.linalg.norm(velocity[3:])))
    if max_pos > thresholds.target_position_m or max_rotation > thresholds.target_rotation_rad:
        failures.append('guide_released_pose_outside_target')
    if max_speed > thresholds.released_speed_m_s or max_angular > thresholds.released_angular_speed_rad_s:
        failures.append('guide_unsettled_after_release')
    loads = [m['unintended_arm_load_n'] for m in metrics]
    if max(loads) > thresholds.unintended_arm_contact_n:
        failures.append('unintended_loaded_arm_environment_contact')
    return dict(passed=not failures, failure_reasons=sorted(set(failures)),
                thresholds=asdict(thresholds),
                acquisition_loaded_fraction=float(np.mean(pregrasp)) if pregrasp else 0.,
                carry_loaded_fraction=float(np.mean(retained)) if retained else 0.,
                carry_longest_unloaded_gap_s=_runs([not x for x in retained], dt),
                maximum_rigid_drift_m=drift, carried_minimum_lift_m=lift,
                setdown_time_s=None if setdown is None else selected[setdown]['time_s'],
                released_pose_error_m=max_pos, released_rotation_error_rad=max_rotation,
                released_max_speed_m_s=max_speed, released_max_angular_speed_rad_s=max_angular,
                unintended_arm_peak_n=max(loads),
                unintended_arm_loaded_duration_s=sum(x > thresholds.unintended_arm_contact_n for x in loads)*dt,
                unintended_arm_impulse_upper_bound_ns=sum(loads)*dt)


def score_assembly_episode(rows, *, model, spec: Mapping | None = None):
    """Return honest full-task incompleteness and separately measured primitives.

    spec accepts only source_paths, a mapping from SOURCE_HASHES roles to local
    files. It cannot assert stage results, alter thresholds, or replace order.
    """
    spec = {} if spec is None else dict(spec)
    thresholds = AssemblyThresholds()
    model_audit = audit_model(model)
    trace = validate_trace(model, rows, thresholds=thresholds)
    states = trace.pop('states')
    source_audit = audit_source_visuals(model, spec.get('source_paths', {}))
    failures = list(model_audit['failure_reasons']) + list(trace['failure_reasons'])
    if set(spec)-{'source_paths'}:
        failures.append('unsupported_caller_scoring_override')
    if any(not r['passed'] for r in source_audit.values()):
        failures.append('source_visual_audit_failed')
    guide = score_guide_transfer(model, states, thresholds=thresholds)
    guide['dynamics_passed'] = guide['passed']
    guide['passed'] = guide['passed'] and model_audit['passed'] and trace['passed'] and source_audit.get('guide', {}).get('passed', False)
    guide['source_visual_verified'] = source_audit.get('guide', {}).get('passed', False)
    if not guide['source_visual_verified']:
        guide['failure_reasons'].append('guide_source_visual_not_verified')
    observed = []
    unknown = []
    for phase in trace['phase_runs']:
        if phase in ('initial', 'initial_settle'):
            continue
        matches = [s for s in STAGE_ORDER if phase == s or phase.startswith(s+'_')]
        stage = max(matches, key=len) if matches else None
        if stage is None:
            unknown.append(phase)
        elif not observed or observed[-1] != stage:
            observed.append(stage)
    if unknown:
        failures.append('unknown_phase_names')
    ranks = [STAGE_ORDER.index(s) for s in observed]
    if any(b <= a for a, b in zip(ranks, ranks[1:])):
        failures.append('assembly_stage_order_violation')
    missing_stages = [s for s in STAGE_ORDER if s not in observed]
    incomplete = [f'missing_stage:{s}' for s in missing_stages]
    incomplete += [f'missing_body:{s}' for s in model_audit['missing_bodies']]
    incomplete += ['deformable_paper_feed_fold_sheet_evaluators_not_implemented',
                   'full_source_collision_surface_qualification_not_implemented',
                   'rigid_trough_holder_carrier_extraction_evaluators_not_implemented']
    source = Path(__file__)
    return dict(schema=1, scorer_version=VERSION,
                source=dict(path=str(source.resolve()), sha256=hashlib.sha256(source.read_bytes()).hexdigest()),
                full_assembly_success=False, physical_success=False, policy_export_eligible=False,
                status='failed' if failures else 'incomplete', failure_reasons=sorted(set(failures)),
                incomplete_reasons=incomplete, contract=full_assembly_contract(),
                observed_stage_order=observed, model_audit=model_audit, trace_audit=trace,
                source_visual_audit=source_audit, primitive_results={'guide_transfer': guide},
                thresholds=asdict(thresholds),
                assumptions=['All thresholds provisional simulation limits, not physical safety calibration',
                             'Ground-truth scoring is independent of the controller; it does not certify visual perception',
                             'Human-prepared initial placements do not complete robot placement stages'])
