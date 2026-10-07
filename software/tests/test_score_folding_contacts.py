"""Saved-file scorer tests only; no MuJoCo dynamics or robot operations."""
import copy
import gzip
import hashlib
import json

import pytest

from carton.folding_contact_audit import SCHEMA, SOURCE
from carton.folding_sim import FoldingSimulation
from tools.score_folding_contacts import LOG_NAME, main, score_run


def sample(index):
    return {'schema': SCHEMA, 'source': SOURCE, 'step_started_at': index*.002,
            'step_ended_at': (index+1)*.002, 'timestep_s': .002,
            'integrator': 'mjINT_IMPLICITFAST', 'total_contact_count': 0, 'contacts': [],
            'applied_inputs': {'ctrl': [], 'qfrc_applied_nonzero': [], 'xfrc_applied_nonzero': []},
            'loaded_unintended_contacts': [], 'refusal_reason': None}


@pytest.fixture
def run(tmp_path, monkeypatch):
    def forbidden_operation(*args, **kwargs):
        raise AssertionError('The saved-file scorer must not instantiate or advance a simulation')
    monkeypatch.setattr(FoldingSimulation, '__init__', forbidden_operation)
    import mujoco
    monkeypatch.setattr(mujoco, 'mj_step', forbidden_operation)
    monkeypatch.setattr(mujoco, 'mj_forward', forbidden_operation)
    records = [sample(i) for i in range(3)]
    report = {'final_time_s': .006, 'events': [{'time': .002, 'duration_s': .002},
               {'time': .006, 'duration_s': .004}],
              'success': True, 'full_task_complete': True,  # Never used by the contact-only scorer.
              'applied_contact_evidence': {'path': LOG_NAME, 'format': 'gzip_jsonl',
                  'expected_start_time': 0., 'expected_end_time': .006, 'sha256': ''}}

    def save(rows=records):
        (tmp_path/LOG_NAME).write_bytes(gzip.compress(b''.join(json.dumps(r).encode()+b'\n' for r in rows)))
        report['applied_contact_evidence']['sha256'] = hashlib.sha256((tmp_path/LOG_NAME).read_bytes()).hexdigest()
        (tmp_path/'folding.json').write_text(json.dumps(report))
    save()
    return tmp_path, records, report, save


def test_verified_complete_contact_only_score_never_evaluates_task_success(run):
    path, *_ = run
    result = score_run(path)
    assert result['status'] == 'CONTACT_ONLY_CLEAR' and result['contact_only_passed']
    assert result['hash_verified'] and result['recorded_steps'] == 3
    assert result['expected_interval_s'] == [0., .006]
    assert result['contact_score']['coverage_complete']
    assert result['scope'] == 'simulation_applied_contacts_only'
    assert result['task_success_evaluated'] is False and result['physical_validation'] is False
    assert 'success' not in result and 'full_task_complete' not in result


def test_modified_compressed_bytes_fail_before_scoring(run):
    path, records, _, _ = run
    records[0]['contacts'] = []
    # A second gzip member changes the bound bytes, even if the data still parses.
    with (path/LOG_NAME).open('ab') as stream:
        stream.write(gzip.compress(json.dumps(records[0]).encode()+b'\n'))
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_HASH_MISMATCH'
    assert not result['contact_only_passed'] and result['contact_score'] is None


@pytest.mark.parametrize('missing', [0, 1, 2])
def test_rehashed_log_with_missing_start_middle_or_tail_never_passes_coverage(run, missing):
    path, records, _, save = run
    records.pop(missing)
    save(records)  # Hash is valid; physical-step coverage is not.
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_INCOMPLETE'
    assert result['hash_verified'] and not result['contact_only_passed']
    assert not result['contact_score']['coverage_complete']


def test_rehashed_replayed_step_is_distinguished_from_incomplete_coverage(run):
    path, records, _, save = run
    records.insert(1, copy.deepcopy(records[0]))
    save(records)
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_REPLAY'
    assert not result['contact_only_passed'] and not result['contact_score']['coverage_complete']


@pytest.mark.parametrize('missing', ['binding', 'log', 'empty'])
def test_missing_historical_or_empty_evidence_is_no_data(run, missing):
    path, _, report, save = run
    if missing == 'binding':
        report.pop('applied_contact_evidence')
        (path/'folding.json').write_text(json.dumps(report))
    elif missing == 'log':
        (path/LOG_NAME).unlink()
    else:
        save([])
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_NO_DATA'
    assert not result['contact_only_passed']


def test_log_claim_cannot_shorten_the_expected_report_interval(run):
    path, _, report, _ = run
    report['applied_contact_evidence']['expected_end_time'] = .004
    (path/'folding.json').write_text(json.dumps(report))
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_INCOMPLETE'
    assert any('disagree' in e for e in result['errors'])


def test_loaded_contact_is_rescored_with_current_policy_despite_cached_pass_fields(run):
    path, records, _, save = run
    records[1]['contacts'] = [{'geom1': 'long_near_cardboard', 'geom2': 'left_wrist_housing',
        'distance_m': -.000001, 'wrench_contact_N_Nm': [2., 0., 0., 0., 0., 0.], 'classes': ['jaw_flap']}]
    save(records)
    result = score_run(path)
    assert result['status'] == 'LOADED_UNINTENDED_CONTACT' and not result['contact_only_passed']
    assert result['contact_score']['loaded_pairs'][0]['category'] == 'non_jaw_flap'


def test_truncated_gzip_is_incomplete_even_with_updated_hash(run):
    path, _, report, _ = run
    log = path/LOG_NAME
    log.write_bytes(log.read_bytes()[:-5])
    report['applied_contact_evidence']['sha256'] = hashlib.sha256(log.read_bytes()).hexdigest()
    (path/'folding.json').write_text(json.dumps(report))
    result = score_run(path)
    assert result['status'] == 'CONTACT_EVIDENCE_INCOMPLETE' and not result['contact_only_passed']


def test_cli_writes_only_derived_contact_score_and_nonzero_for_missing_data(run, capsys):
    path, _, report, _ = run
    before = (path/'folding.json').read_bytes()
    assert main(['--run', str(path)]) == 0
    result = json.loads((path/'contact-only-score.json').read_text())
    assert result['contact_only_passed'] and result['task_success_evaluated'] is False
    assert (path/'folding.json').read_bytes() == before
    report.pop('applied_contact_evidence')
    (path/'folding.json').write_text(json.dumps(report))
    assert main(['--run', str(path)]) == 2
    assert json.loads(capsys.readouterr().out.splitlines()[-1])['status'] == 'CONTACT_EVIDENCE_NO_DATA'


@pytest.mark.parametrize('name', ['folding.json', LOG_NAME])
def test_cli_cannot_overwrite_source_evidence(run, name):
    path, *_ = run
    before = (path/name).read_bytes()
    with pytest.raises(SystemExit) as exc:
        main(['--run', str(path), '--output', str(path/name)])
    assert exc.value.code == 2 and (path/name).read_bytes() == before


def test_streamed_per_motion_gzip_members_are_scored_as_one_continuous_run(run):
    path, records, report, _ = run
    log = path/LOG_NAME
    log.write_bytes(b''.join(gzip.compress(json.dumps(row).encode()+b'\n') for row in records))
    report['applied_contact_evidence']['sha256'] = hashlib.sha256(log.read_bytes()).hexdigest()
    (path/'folding.json').write_text(json.dumps(report))
    result = score_run(path)
    assert result['contact_only_passed'] and result['recorded_steps'] == 3
