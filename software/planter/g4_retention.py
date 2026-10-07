"""Separate G4 clean-grasp audit; historical scorer-v3 outcomes stay unchanged.

This is privileged simulation evaluation, not deployable contact sensing. The
contact/chatter limits below are explicit analysis assumptions, not physical
safety limits. A clean pickup establishes the grip before lifting and transfers
support to the rest before opening. A successful late capture or drop-release
can therefore pass scorer v3 while failing this stricter, separately named audit.
"""
from __future__ import annotations

import math
import hashlib
from pathlib import Path
import numpy as np


DEFAULT_THRESHOLDS = dict(
    loaded_contact_n=.005,
    established_close_seconds=.100,
    minimum_loaded_fraction=.95,
    maximum_unloaded_gap_seconds=.010,
    maximum_rigid_drift_mm=5.,
    sustained_setdown_seconds=.040,
    maximum_setdown_rest_clearance_mm=1.,
)
OBJECT_GEOMS = {'pusher_blade', 'pusher_crossbar', 'pusher_handle'}
# Exact G4 pusher collision blocks, already audited against its source STL.
_BLOCKS = [([-.0304, 0, .004], [.0075, .024, .0039]),
           ([-.035, 0, .004], [.001, .047, .004]),
           ([-.052, 0, .004], [.016, .012, .004])]
OBJECT_VERTICES_M = np.asarray([
    np.asarray(p) + np.asarray(s) * [x, y, z]
    for p, s in _BLOCKS for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)
])


def _rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _longest_false(mask):
    longest = current = 0
    for value in mask:
        current = 0 if value else current + 1
        longest = max(longest, current)
    return longest


def _contact_stats(mask, timestep):
    return dict(samples=len(mask), observed_seconds=len(mask)*timestep,
                loaded_fraction=float(np.mean(mask)) if mask else 0.,
                longest_unloaded_gap_seconds=_longest_false(mask)*timestep)


def audit_retention(rows, *, timestep_s, thresholds=None):
    """Audit established grip, aerial retention, setdown, and arm collisions.

    Requires the raw contact normals and gripper frames introduced with scorer
    v2, plus arm_environment_contacts. It complements scorer v3; passing this
    helper alone does not establish sequence completion, policy qualification,
    full planter success, or physical success.
    """
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        unknown = set(thresholds) - set(limits)
        if unknown:
            raise ValueError(f'Unknown retention thresholds: {sorted(unknown)}')
        limits.update(thresholds)
    result = dict(schema_version=1, audit='G4_clean_grasp_and_controlled_setdown',
                  passed=False, failure_reasons=[], thresholds=limits,
                  requires_scorer_v3_success=True, evidence_complete=False,
                  source=dict(path=str(Path(__file__).resolve()),
                              sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
                  privileged_simulator_evidence=True, physical_success=False)
    reasons = result['failure_reasons']
    if (not np.isfinite(timestep_s) or timestep_s <= 0 or not rows or
        not all(np.isfinite(v) and v > 0 for v in limits.values()) or
        limits['minimum_loaded_fraction'] > 1):
        reasons.append('invalid_audit_inputs')
        return result
    if timestep_s > limits['maximum_unloaded_gap_seconds']:
        reasons.append('sampling_too_coarse_for_contact_gap_audit')
        return result
    checked = []
    try:
        for row in rows:
            time = float(row['time_s'])
            phase = row['phase']
            pose = np.asarray(row['object_pose'], dtype=float)
            grasp = np.asarray(row['grasp_world_m'], dtype=float)
            frame = np.asarray(row['gripper_world_rotation'], dtype=float)
            rest_clearance = float(row['rest_clearance_mm'])
            if (not isinstance(phase, str) or pose.shape != (7,) or grasp.shape != (3,) or
                frame.shape != (3, 3) or not np.isfinite(np.r_[time, rest_clearance, pose, grasp, frame.ravel()]).all() or
                not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-5) or
                not np.allclose(frame.T@frame, np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(frame), 1, atol=1e-5)):
                raise ValueError
            jaws = [np.zeros(3), np.zeros(3)]
            counts = [0, 0]
            support = 0.
            for contact in row['contacts']:
                names = contact['geoms']
                force = float(contact['normal_force_n'])
                normal = np.asarray(contact['normal_world_geom1_to_geom2'], dtype=float)
                if (len(names) != 2 or not all(isinstance(n, str) for n in names) or
                    normal.shape != (3,) or not np.isfinite(np.r_[force, normal]).all() or force < 0 or
                    not np.isclose(np.linalg.norm(normal), 1, atol=1e-5) or
                    sum(n in OBJECT_GEOMS for n in names) != 1):
                    raise ValueError
                other = names[1] if names[0] in OBJECT_GEOMS else names[0]
                sign = -1 if names[0] in OBJECT_GEOMS else 1
                if other == 'staging_rest':
                    support += sign*force*normal[2]
                if force > limits['loaded_contact_n']:
                    for i, name in enumerate(('moving_jaw', 'wrist_roll_follower')):
                        if name in other:
                            jaws[i] += sign*force*normal
                            counts[i] += 1
            opposed = bool(all(counts) and np.dot(jaws[0], jaws[1]) < 0)
            environment = []
            for contact in row['arm_environment_contacts']:
                names = contact['geoms']
                force = float(contact['normal_force_n'])
                if (len(names) != 2 or not all(isinstance(n, str) for n in names) or
                    not np.isfinite(force) or force < 0):
                    raise ValueError
                if force > limits['loaded_contact_n']:
                    environment.append(dict(geoms=names, normal_force_n=force))
            vertices = (OBJECT_VERTICES_M@_rotation(pose[3:]).T + pose[:3] - grasp)@frame
            checked.append(dict(time=time, phase=phase, opposed=opposed, vertices=vertices,
                                support=(support > limits['loaded_contact_n'] and
                                         abs(rest_clearance) <= limits['maximum_setdown_rest_clearance_mm']),
                                upward_rest_normal_force_n=support, rest_clearance_mm=rest_clearance,
                                environment=environment))
    except (KeyError, ValueError, TypeError, IndexError):
        reasons.append('missing_or_malformed_retention_evidence')
        return result
    if np.any(np.abs(np.diff([r['time'] for r in checked])-timestep_s) > max(1e-9, timestep_s*1e-5)):
        reasons.append('missing_or_irregular_retention_samples')
        return result
    result['evidence_complete'] = True
    phase_indices = {phase: [i for i, r in enumerate(checked) if r['phase'] == phase]
                     for phase in ('close', 'lift', 'hold', 'lower', 'release')}
    if any(not indices for indices in phase_indices.values()):
        reasons.append('required_retention_phases_missing')
        return result
    ordered = list(phase_indices.values())
    if (any(indices != list(range(indices[0], indices[-1]+1)) for indices in ordered) or
        any(a[-1]+1 != b[0] for a, b in zip(ordered, ordered[1:]))):
        reasons.append('retention_phases_not_contiguous_and_ordered')
        return result
    close, lift, hold, lower, release = ordered
    window_samples = math.ceil(limits['established_close_seconds']/timestep_s - 1e-9)
    window = close[-window_samples:]
    mask = [checked[i]['opposed'] for i in window]
    pregrasp = _contact_stats(mask, timestep_s)
    established = (len(window) >= window_samples and
                   pregrasp['loaded_fraction'] >= limits['minimum_loaded_fraction'] and
                   pregrasp['longest_unloaded_gap_seconds'] <= limits['maximum_unloaded_gap_seconds']+1e-9)
    result['pre_lift_grasp'] = dict(established=bool(established), **pregrasp)
    if not established:
        reasons.append('opposed_loaded_grip_not_established_before_lift')

    # Intentional transfer is supported lower-phase contact persisting for at
    # least 40ms. An isolated impact does not end the retention requirement.
    required_support_samples = math.ceil(limits['sustained_setdown_seconds']/timestep_s - 1e-9)
    setdown = None
    run_start = None
    for i in lower:
        if checked[i]['support']:
            if run_start is None:
                run_start = i
            if i-run_start+1 >= required_support_samples:
                setdown = run_start
                break
        else:
            run_start = None
    result['setdown'] = dict(observed_before_release=setdown is not None,
                            time_s=checked[setdown]['time'] if setdown is not None else None,
                            upward_rest_normal_force_n=checked[setdown]['upward_rest_normal_force_n'] if setdown is not None else None,
                            rest_clearance_mm=checked[setdown]['rest_clearance_mm'] if setdown is not None else None)
    if setdown is None:
        reasons.append('sustained_setdown_not_observed_before_release')
    stop = setdown if setdown is not None else release[0]
    # Include the final close window so a contact gap crossing close -> lift
    # cannot be split into two individually admissible pieces.
    retained = [checked[i]['opposed'] for i in range(window[0], stop)]
    retention = _contact_stats(retained, timestep_s)
    if (retention['loaded_fraction'] < limits['minimum_loaded_fraction'] or
        retention['longest_unloaded_gap_seconds'] > limits['maximum_unloaded_gap_seconds']+1e-9):
        reasons.append('opposed_loaded_grip_lost_before_setdown')
    # Anchor to the first loaded sample of the terminal established window;
    # if none exists, still provide diagnostic displacement from close end.
    reference = next((i for i in window if checked[i]['opposed']), close[-1])
    drift = [float(np.linalg.norm(checked[i]['vertices']-checked[reference]['vertices'], axis=1).max()*1000)
             for i in range(reference, stop)]
    retention.update(reference_time_s=checked[reference]['time'],
                     coverage_start_time_s=checked[window[0]]['time'],
                     reference_is_established_grasp=bool(established),
                     maximum_rigid_relative_drift_mm=max(drift, default=0.))
    result['retention_until_setdown'] = retention
    if established and retention['maximum_rigid_relative_drift_mm'] > limits['maximum_rigid_drift_mm']:
        reasons.append('rigid_grasp_drift_exceeds_limit_before_setdown')

    # All manipulation phases are checked. Object/rest support is intentional;
    # arm/rest support is not declared as part of this clean pickup controller.
    environment_rows = [r for r in checked if r['phase'] != 'settle' and r['environment']]
    result['arm_environment_contacts'] = dict(
        provisional_loaded_contact_threshold_n=limits['loaded_contact_n'],
        loaded_samples=len(environment_rows), loaded_duration_seconds=len(environment_rows)*timestep_s,
        maximum_point_normal_force_n=max((c['normal_force_n'] for r in environment_rows for c in r['environment']), default=0.),
        maximum_summed_normal_force_n=max((sum(c['normal_force_n'] for c in r['environment']) for r in environment_rows), default=0.),
        integrated_summed_normal_impulse_n_s=sum(c['normal_force_n'] for r in environment_rows for c in r['environment'])*timestep_s,
        phases=sorted({r['phase'] for r in environment_rows}))
    if environment_rows:
        reasons.append('unintended_loaded_arm_environment_contact')
    result['passed'] = not reasons
    return result
