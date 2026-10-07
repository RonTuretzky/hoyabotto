"""Saved panel evidence must be complete, bound to its run, and independently scored."""
import gzip
import hashlib
import json
from types import SimpleNamespace

import mujoco
import pytest

from carton.folding_panel_audit import (
    PANEL_SCHEMA, PANEL_SOURCE, PANEL_LOG_NAME, event_digest,
)
from carton.folding_partial_short_probe import _PanelStepAudit
from tools.score_partial_short_panels import score_run


def sample(index):
    return dict(schema=PANEL_SCHEMA, source=PANEL_SOURCE, recording_id='a'*32,
                step_index=index, step_started_at=.5+index*.002,
                step_ended_at=.5+(index+1)*.002, timestep_s=.002,
                integrator='mjINT_IMPLICITFAST', total_contact_count=1, contacts=[dict(
                    contact_id=0, geom1='short_left_cardboard', geom2='long_near_cardboard',
                    distance_m=-.0003, position_m=[0., 0., 0.],
                    contact_frame=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                    wrench_contact_N_Nm=[.4, 0., 0., 0., 0., 0.])])


def write_run(run, samples=None):
    samples = [sample(0), sample(1)] if samples is None else samples
    events = [dict(time=.5, duration_s=.5, label='earlier prefix'),
              dict(time=.504, duration_s=.004, label='bounded probe')]
    path = run/PANEL_LOG_NAME
    with gzip.open(path, 'wt') as stream:
        for row in samples:
            stream.write(json.dumps(row)+'\n')
    binding = dict(schema=PANEL_SCHEMA, source=PANEL_SOURCE, recording_id='a'*32,
                   format='gzip_jsonl', path=PANEL_LOG_NAME, source_report='folding.json',
                   expected_start_time=.5, expected_end_time=.504, timestep_s=.002,
                   source_event_start_index=1, source_event_count=1,
                   source_events_sha256=event_digest(events[1:]),
                   sha256=hashlib.sha256(path.read_bytes()).hexdigest(), compressed_bytes=path.stat().st_size,
                   # Deliberately optimistic cached values are never authoritative.
                   max_panel_panel_penetration_mm=0., complete_coverage=True, steps=2)
    (run/'result.json').write_text(json.dumps(dict(partial_short_probe=dict(panel_panel_audit=binding))))
    (run/'folding.json').write_text(json.dumps(dict(final_time_s=.504, events=events)))
    return binding


def test_complete_panel_evidence_passes_without_writing_sources(tmp_path):
    write_run(tmp_path)
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    result = score_run(tmp_path)
    assert result['status'] == 'PANEL_CONTACT_ONLY_CLEAR'
    assert result['panel_score']['observed_steps'] == 2
    assert result['panel_score']['max_panel_panel_penetration_mm'] == pytest.approx(.3)
    assert not result['task_success_evaluated']
    assert not result['physical_validation']
    assert before == {p.name: p.read_bytes() for p in tmp_path.iterdir()}


def test_independent_penetration_score_overrides_optimistic_cached_result(tmp_path):
    rows = [sample(0), sample(1)]
    rows[1]['contacts'][0]['distance_m'] = -.0011
    write_run(tmp_path, rows)
    result = score_run(tmp_path)
    assert result['status'] == 'PANEL_PENETRATION_EXCEEDED'
    assert result['panel_score']['coverage_complete']
    assert result['panel_score']['max_panel_panel_penetration_mm'] == pytest.approx(1.1)
    assert not result['panel_contact_only_passed']


def test_changed_compressed_bytes_do_not_retain_authorization(tmp_path):
    write_run(tmp_path)
    with (tmp_path/PANEL_LOG_NAME).open('ab') as stream:
        stream.write(b'changed')
    assert score_run(tmp_path)['status'] == 'PANEL_EVIDENCE_HASH_MISMATCH'


@pytest.mark.parametrize('rows', [[], [sample(0)], [sample(1)]])
def test_missing_steps_refused_even_when_rehashed_and_cached_complete(tmp_path, rows):
    write_run(tmp_path, rows)
    result = score_run(tmp_path)
    assert result['status'] == 'PANEL_EVIDENCE_INCOMPLETE'
    assert not result['panel_contact_only_passed']


@pytest.mark.parametrize('changed', ['index', 'timestamp', 'out_of_order'])
def test_replayed_or_out_of_order_steps_are_refused(tmp_path, changed):
    rows = [sample(0), sample(1)]
    if changed == 'index':
        rows[1]['step_index'] = 0
    elif changed == 'timestamp':
        rows[1]['step_started_at'] = .5
        rows[1]['step_ended_at'] = .502
    else:
        rows = [sample(0), sample(1), sample(0)]
    write_run(tmp_path, rows)
    assert score_run(tmp_path)['status'] == 'PANEL_EVIDENCE_REPLAY'


@pytest.mark.parametrize('change', [
    lambda row: row['contacts'][0].pop('wrench_contact_N_Nm'),
    lambda row: row['contacts'][0].update(wrench_contact_N_Nm=[0., 0., 0.]),
    lambda row: row['contacts'][0].update(wrench_contact_N_Nm=[float('nan')]*6),
    lambda row: row['contacts'][0].update(distance_m=float('inf')),
    lambda row: row['contacts'][0].update(distance_m=True),
    lambda row: row['contacts'][0].update(position_m=[0., 0., float('nan')]),
    lambda row: row['contacts'][0].update(contact_frame=[0.]*8),
    lambda row: row.update(source='qpos_replay'),
    lambda row: row.update(recording_id='b'*32),
    lambda row: row.update(integrator='mjINT_RK4'),
    lambda row: row.update(timestep_s=.003),
    lambda row: row.pop('total_contact_count'),
])
def test_invalid_or_unbound_contact_records_refused(tmp_path, change):
    rows = [sample(0), sample(1)]
    change(rows[1])
    write_run(tmp_path, rows)
    result = score_run(tmp_path)
    assert result['status'] == 'PANEL_EVIDENCE_INCOMPLETE'
    assert not result['panel_contact_only_passed']


def test_old_position_only_frames_are_not_panel_contact_evidence(tmp_path):
    write_run(tmp_path, [dict(time=.502, qpos=[0.]*10), dict(time=.504, qpos=[0.]*10)])
    assert score_run(tmp_path)['status'] == 'PANEL_EVIDENCE_INCOMPLETE'


def test_source_motion_interval_is_independently_bound(tmp_path):
    write_run(tmp_path)
    path = tmp_path/'folding.json'
    report = json.loads(path.read_text())
    report['events'][1]['duration_s'] = .002
    path.write_text(json.dumps(report))
    result = score_run(tmp_path)
    assert result['status'] == 'PANEL_EVIDENCE_INCOMPLETE'
    assert 'motion-event' in result['errors'][0]


def test_missing_bound_log_is_no_data(tmp_path):
    write_run(tmp_path)
    (tmp_path/PANEL_LOG_NAME).unlink()
    assert score_run(tmp_path)['status'] == 'PANEL_EVIDENCE_NO_DATA'


def test_no_contact_steps_are_evidence_when_every_step_is_present(tmp_path):
    rows = [sample(0), sample(1)]
    for row in rows:
        row.update(contacts=[], total_contact_count=0)
    write_run(tmp_path, rows)
    assert score_run(tmp_path)['status'] == 'PANEL_CONTACT_ONLY_CLEAR'


@pytest.mark.parametrize('overlap, status', [(.0003, 'PANEL_CONTACT_ONLY_CLEAR'),
                                           (.0015, 'PANEL_PENETRATION_EXCEEDED')])
def test_recorder_binds_actual_solver_step_and_source_event_for_scorer(tmp_path, overlap, status):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="0.002"/>
      <worldbody><geom name="long_near_cardboard" type="box" size=".1 .1 .01"/>
      <body pos="0 0 {0.02-overlap}"><freejoint/>
        <geom name="short_left_cardboard" type="box" size=".1 .1 .01" mass=".02"/>
      </body></worldbody></mujoco>''')
    sim = SimpleNamespace(model=model, data=mujoco.MjData(model), out=tmp_path,
                          events=[], step_diagnostic=lambda: None)
    report = {}
    with _PanelStepAudit(sim, report):
        mujoco.mj_step(model, sim.data)
        sim.step_diagnostic()
        sim.events.append(dict(time=float(sim.data.time), duration_s=.002, label='actual test step'))
    (tmp_path/'result.json').write_text(json.dumps(dict(partial_short_probe=report)))
    (tmp_path/'folding.json').write_text(json.dumps(dict(final_time_s=float(sim.data.time), events=sim.events)))
    result = score_run(tmp_path)
    assert result['status'] == status
    assert result['hash_verified']
    assert result['panel_score']['coverage_complete']
