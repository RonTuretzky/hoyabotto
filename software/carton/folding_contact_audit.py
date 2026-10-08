"""Independent applied-contact scoring for the bare-claw simulation.

Call sample_applied_contacts immediately after the real mj_step, before any
mj_forward/mj_fwdPosition, render refresh, state mutation or next step. MuJoCo's
one-pass integrators leave the contact/constraint arrays for the solve that
advanced that step; these are not contact forces reconstructed at final qpos.
No world state, solver option, existing penetration gate or motor limit changes.

This audit detects loaded forbidden pairs and loaded non-jaw robot/flap pairs.
It follows the existing moving_jaw / wrist_roll_follower geometry convention;
a named jaw mesh is not proof that contact is on the intended broad fingertip
face. Grasp/retention and task-completion evaluators remain independently needed.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

SCHEMA = 1
SOURCE = 'actual_mj_step_solver_contact_arrays'
# A numerical zero floor, NOT a permitted load or a changed actuator limit.
FORCE_ZERO_N = 1e-6


def contact_classes(a, b, forbidden_contact):
    """Reuse the caller's current forbidden policy; add only non-jaw flap audit."""
    classes = []
    if forbidden_contact(a, b):
        classes.append('forbidden')
    for panel, robot in ((a, b), (b, a)):
        if panel.endswith('_cardboard') and robot.startswith(('left_', 'right_')):
            jaw = any(part in robot for part in ('moving_jaw', 'wrist_roll_follower'))
            classes.append('jaw_flap' if jaw else 'non_jaw_flap')
    return classes


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Contact audit requires finite numeric evidence')
    return float(value)


def _validate_inputs(inputs):
    if not isinstance(inputs, dict) or set(inputs) != {'ctrl', 'qfrc_applied_nonzero', 'xfrc_applied_nonzero'}:
        raise ValueError('Missing applied control/force input evidence')
    if not all(isinstance(value, list) for value in inputs.values()):
        raise ValueError('Applied control/force evidence must contain arrays')
    for value in inputs['ctrl']:
        _number(value)
    for name in ('qfrc_applied_nonzero', 'xfrc_applied_nonzero'):
        indices = set()
        for row in inputs[name]:
            if (not isinstance(row, list) or len(row) != 2 or type(row[0]) is not int
                    or row[0] < 0 or row[0] in indices):
                raise ValueError('Invalid sparse applied-force evidence')
            indices.add(row[0])
            if name == 'qfrc_applied_nonzero':
                _number(row[1])
            else:
                if not isinstance(row[1], list) or len(row[1]) != 6:
                    raise ValueError('Applied body force requires six components')
                for value in row[1]:
                    _number(value)


def _loaded_rows(contacts, forbidden_contact):
    loaded = []
    for contact in contacts:
        a, b = contact['geom1'], contact['geom2']
        if not isinstance(a, str) or not isinstance(b, str):
            raise ValueError('Contact audit requires named geometry')
        distance = _number(contact['distance_m'])
        wrench = contact['wrench_contact_N_Nm']
        if not isinstance(wrench, list) or len(wrench) != 6:
            raise ValueError('Contact audit needs the six solver wrench components')
        wrench = [_number(value) for value in wrench]
        resultant = float(np.linalg.norm(wrench[:3]))
        for category in contact_classes(a, b, forbidden_contact):
            if category != 'jaw_flap' and resultant > FORCE_ZERO_N:
                loaded.append({'category': category, 'pair': [a, b],
                    'distance_m': distance, 'penetration_mm': max(0., -distance*1000),
                    'normal_force_N': wrench[0], 'resultant_force_N': resultant,
                    'wrench_contact_N_Nm': wrench})
    return loaded


def sample_applied_contacts(model, data, *, step_started_at, forbidden_contact):
    """Read, never recompute, the just-completed step's applied contact forces.

    step_started_at must be captured directly before that real mj_step. This
    function cannot detect a caller secretly refreshing solver caches; immediate
    placement is part of the contract. It deliberately offers no qpos replay API.
    RK4/discrete integrators are refused because their force-stage semantics do
    not match this one-solve-per-step record.
    """
    started, ended = _number(step_started_at), _number(float(data.time))
    dt = _number(float(model.opt.timestep))
    if dt <= 0 or not math.isclose(ended-started, dt, rel_tol=1e-8, abs_tol=1e-10):
        raise ValueError('Contact sample must immediately follow exactly one mj_step')
    if model.opt.integrator not in (mujoco.mjtIntegrator.mjINT_EULER,
            mujoco.mjtIntegrator.mjINT_IMPLICIT, mujoco.mjtIntegrator.mjINT_IMPLICITFAST):
        raise ValueError('Applied contact audit requires a one-pass integrator')
    contacts = []
    for index, contact in enumerate(data.contact):
        a, b = model.geom(contact.geom1).name, model.geom(contact.geom2).name
        classes = contact_classes(a, b, forbidden_contact)
        if not classes:
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)
        contacts.append({'contact_id': index, 'geom1': a, 'geom2': b,
            'classes': classes, 'distance_m': float(contact.dist),
            'constraint_address': int(contact.efc_address),
            'position_m': contact.pos.tolist(), 'contact_frame': contact.frame.tolist(),
            'wrench_contact_N_Nm': wrench.tolist()})
    loaded = _loaded_rows(contacts, forbidden_contact)
    return {'schema': SCHEMA, 'source': SOURCE, 'step_started_at': started,
        'step_ended_at': ended, 'timestep_s': dt,
        'integrator': mujoco.mjtIntegrator(model.opt.integrator).name,
        'total_contact_count': int(data.ncon), 'contacts': contacts,
        'applied_inputs': {'ctrl': data.ctrl.tolist(),
            'qfrc_applied_nonzero': [[int(i), float(data.qfrc_applied[i])]
                                   for i in np.flatnonzero(data.qfrc_applied)],
            'xfrc_applied_nonzero': [[int(i), data.xfrc_applied[i].tolist()]
                                   for i in np.flatnonzero(np.any(data.xfrc_applied != 0, axis=1))]},
        'loaded_unintended_contacts': loaded,
        'refusal_reason': None if not loaded else
            'Loaded '+loaded[0]['category']+' contact: '+' / '.join(loaded[0]['pair'])}


def score_applied_contacts(samples, *, expected_start_time, expected_end_time, forbidden_contact):
    """Independently recompute force refusals and complete interval coverage.

    Does not trust cached sample classes, loaded flags or refusal_reason. A list
    of positions/events, no steps, gaps, replayed steps or missing wrench/input
    evidence is incomplete, never an absence-of-loaded-contact pass.
    """
    start, end = _number(expected_start_time), _number(expected_end_time)
    if end <= start:
        raise ValueError('Declare a positive expected motion interval')
    errors, violations, pairs = [], [], {}
    cursor, timestep, observed = start, None, 0
    for index, sample in enumerate(samples):
        try:
            if sample.get('schema') != SCHEMA or sample.get('source') != SOURCE:
                raise ValueError('Missing applied solver-contact source evidence')
            a, b, dt = (_number(sample[k]) for k in ('step_started_at', 'step_ended_at', 'timestep_s'))
            if (dt <= 0 or not math.isclose(b-a, dt, rel_tol=1e-8, abs_tol=1e-10)
                    or not math.isclose(a, cursor, rel_tol=0, abs_tol=1e-9)):
                raise ValueError('Gap, replay or wrong step interval in applied contact evidence')
            if timestep is not None and not math.isclose(dt, timestep, rel_tol=0, abs_tol=1e-12):
                raise ValueError('Timestep changed inside contact audit interval')
            if sample.get('integrator') not in ('mjINT_EULER', 'mjINT_IMPLICIT', 'mjINT_IMPLICITFAST'):
                raise ValueError('Missing one-pass integrator evidence')
            _validate_inputs(sample['applied_inputs'])
            if not isinstance(sample['contacts'], list):
                raise ValueError('Missing per-contact evidence')
            loaded = _loaded_rows(sample['contacts'], forbidden_contact)
            for row in loaded:
                key = (row['category'], *sorted(row['pair']))
                aggregate = pairs.setdefault(key, {'category': key[0], 'pair': list(key[1:]),
                    'loaded_contact_samples': 0, 'max_resultant_force_N': 0.,
                    'max_penetration_mm': 0., 'integrated_resultant_load_Ns': 0.,
                    'first_step_started_at': a, 'last_step_ended_at': b})
                aggregate['loaded_contact_samples'] += 1
                aggregate['max_resultant_force_N'] = max(aggregate['max_resultant_force_N'], row['resultant_force_N'])
                aggregate['max_penetration_mm'] = max(aggregate['max_penetration_mm'], row['penetration_mm'])
                aggregate['integrated_resultant_load_Ns'] += row['resultant_force_N']*dt
                aggregate['last_step_ended_at'] = b
            if loaded:
                violations.append({'step_index': index, 'step_started_at': a, 'step_ended_at': b,
                                   'loaded_unintended_contacts': loaded})
            cursor, timestep, observed = b, dt, observed+1
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            errors.append(f'step {index}: {exc}')
    if not observed or not math.isclose(cursor, end, rel_tol=0, abs_tol=1e-9):
        errors.append('Applied contact samples do not cover the complete requested interval')
    return {'schema': SCHEMA, 'status': ('CONTACT_AUDIT_INCOMPLETE' if errors else
            'LOADED_UNINTENDED_CONTACT' if violations else 'NO_LOADED_UNINTENDED_CONTACT_OBSERVED'),
        'passed': not errors and not violations, 'force_numerical_zero_N': FORCE_ZERO_N,
        'observed_steps': observed, 'expected_start_time': start, 'expected_end_time': end,
        'coverage_complete': not errors, 'errors': errors, 'violating_steps': len(violations),
        'violations': violations, 'loaded_pairs': list(pairs.values()),
        'existing_penetration_and_task_gates_still_required': True,
        'physical_validation': False}
