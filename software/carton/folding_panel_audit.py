"""Read-only scoring of actual-step panel contact evidence, without simulation.

Provenance depends on the recorder being called after the applied solver step.
Hashes bind the saved bytes and source motion interval; they do not turn a
position replay or a simulated component into physical hardware validation.
"""
import hashlib
import json
import math


PANEL_SCHEMA = 1
PANEL_SOURCE = 'actual_mj_step_solver_contact_arrays'
PANEL_LOG_NAME = 'partial-short-panel-contact-steps.jsonl.gz'
PANELS = {name+'_cardboard' for name in ('short_left', 'short_right', 'long_near', 'long_far')}
INTEGRATORS = {'mjINT_EULER', 'mjINT_IMPLICIT', 'mjINT_IMPLICITFAST'}


def event_digest(events):
    """Bind the exact motion-event records as saved in the source folding report."""
    encoded = json.dumps(events, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Finite numeric panel-contact evidence required')
    return float(value)


def _vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'Panel contact evidence requires {length} finite components')
    return [finite_number(v) for v in value]


def _same(a, b):
    return math.isclose(a, b, rel_tol=0., abs_tol=1e-9)


class ReplayedPanelEvidence(ValueError):
    pass


def score_panel_steps(samples, *, expected_start_time, expected_end_time,
                      expected_timestep, recording_id):
    """Independently recompute complete coverage and the 1 mm panel criterion."""
    start, end, dt = map(finite_number, (expected_start_time, expected_end_time, expected_timestep))
    if start < 0 or end <= start or dt <= 0:
        raise ValueError('Positive executed panel-contact interval and timestep required')
    expected_steps = round((end-start)/dt)
    if expected_steps < 1 or not _same(expected_steps*dt, end-start):
        raise ValueError('Panel-contact interval must contain an exact number of original steps')
    cursor, observed, maximum = start, 0, 0.
    violations, errors, replay = [], [], False
    try:
        for index, sample in enumerate(samples):
            if (not isinstance(sample, dict) or type(sample.get('schema')) is not int
                    or sample.get('schema') != PANEL_SCHEMA
                    or sample.get('source') != PANEL_SOURCE
                    or sample.get('recording_id') != recording_id
                    or sample.get('integrator') not in INTEGRATORS):
                raise ValueError('Missing bound actual-solver panel-contact source evidence')
            step_index = sample.get('step_index')
            if type(step_index) is not int or step_index < 0:
                raise ValueError('Explicit integer panel-contact step index required')
            if step_index < index:
                raise ReplayedPanelEvidence('Replayed or out-of-order panel-contact step index')
            if step_index != index:
                raise ValueError('Missing panel-contact step index')
            a, b, step = map(finite_number, (sample.get('step_started_at'),
                sample.get('step_ended_at'), sample.get('timestep_s')))
            if a < cursor-1e-9:
                raise ReplayedPanelEvidence('Replayed or overlapping panel-contact interval')
            if (b <= a or not _same(step, dt) or not _same(b-a, dt)
                    or not _same(a, cursor) or b > end+1e-9):
                raise ValueError('Gap, changed timestep or interval outside source motion in panel evidence')
            contacts, total = sample.get('contacts'), sample.get('total_contact_count')
            if (not isinstance(contacts, list) or type(total) is not int or total < 0
                    or total < len(contacts)):
                raise ValueError('Actual panel-contact records and total contact count required')
            ids = set()
            for contact in contacts:
                if not isinstance(contact, dict):
                    raise ValueError('Panel contact must be an object')
                cid = contact.get('contact_id')
                if type(cid) is not int or not 0 <= cid < total or cid in ids:
                    raise ValueError('Unique solver contact index required within each step')
                ids.add(cid)
                pair = (contact.get('geom1'), contact.get('geom2'))
                if pair[0] not in PANELS or pair[1] not in PANELS or pair[0] == pair[1]:
                    raise ValueError('Two distinct original cardboard panels required')
                distance = finite_number(contact.get('distance_m'))
                _vector(contact.get('position_m'), 3)
                _vector(contact.get('contact_frame'), 9)
                _vector(contact.get('wrench_contact_N_Nm'), 6)
                penetration = max(0., -distance*1000.)
                maximum = max(maximum, penetration)
                if penetration > 1.:
                    violations.append(dict(step_index=index, step_started_at=a, step_ended_at=b,
                                           pair=list(pair), penetration_mm=penetration))
            cursor, observed = b, index+1
        if observed != expected_steps or not _same(cursor, end):
            raise ValueError('Panel contact log does not cover every step of the declared source interval')
    except (ValueError, KeyError, TypeError) as exc:
        errors.append(str(exc))
        replay = isinstance(exc, ReplayedPanelEvidence)
    complete = not errors and observed == expected_steps and _same(cursor, end)
    return dict(passed=complete and not violations, coverage_complete=complete,
                replay_detected=replay, expected_steps=expected_steps, observed_steps=observed,
                expected_start_time=start, expected_end_time=end, timestep_s=dt,
                max_panel_panel_penetration_mm=maximum, violations=violations, errors=errors)
