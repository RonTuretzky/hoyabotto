"""Offline contact-only scoring of a hash-bound folding run's applied-step log.

Usage: python -m tools.score_folding_contacts --run /path/to/run

No simulator instance, model loading, dynamics, rendering, device or network
operation is performed. The current FoldingSimulation forbidden-contact policy
is used directly; this does not rerun or certify the complete folding task.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import inspect
import json
import math
from pathlib import Path
import re

from carton.folding_contact_audit import score_applied_contacts
from carton.folding_sim import FoldingSimulation
from carton.servo.common import atomic_json

LOG_NAME = 'applied-contact-steps.jsonl.gz'


def _json(text):
    return json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Finite numeric run times are required')
    return float(value)


def _same(a, b):
    return math.isclose(a, b, rel_tol=0., abs_tol=1e-8)


def _forbidden(a, b):
    # This policy does not read self. Never instantiate a simulator to score logs.
    return FoldingSimulation.forbidden_contact(None, a, b)


def _run_interval(report, binding):
    if binding.get('format') != 'gzip_jsonl' or binding.get('path') != LOG_NAME:
        raise ValueError('Expected the fixed applied-contact-steps.jsonl.gz gzip_jsonl artifact')
    if _number(binding.get('expected_start_time')) != 0.:
        raise ValueError('Applied contact coverage must start at simulation time zero')
    final = _number(report.get('final_time_s'))
    if final <= 0 or not _same(final, _number(binding.get('expected_end_time'))):
        raise ValueError('Final report time and evidence interval disagree')
    events = report.get('events')
    if not isinstance(events, list) or not events:
        raise ValueError('Report has no motion events establishing the full run interval')
    cursor = 0.
    for index, event in enumerate(events):
        ended, duration = _number(event['time']), _number(event['duration_s'])
        if duration <= 0 or ended <= cursor or not _same(ended-duration, cursor):
            raise ValueError(f'Event {index} has a gap, replay or inconsistent duration')
        cursor = ended
    if not _same(cursor, final):
        raise ValueError('Final motion event time differs from final report time')
    return final


def score_run(run):
    """Stream and independently score saved evidence; do not modify run files."""
    run = Path(run).resolve()
    output = {'schema': 1, 'scope': 'simulation_applied_contacts_only',
        'status': 'CONTACT_EVIDENCE_NO_DATA', 'contact_only_passed': False,
        'task_success_evaluated': False, 'physical_validation': False,
        'run': str(run), 'errors': [], 'contact_score': None,
        'policy': 'current FoldingSimulation.forbidden_contact + non-jaw flap load audit',
        'policy_source_sha256': hashlib.sha256(inspect.getsource(FoldingSimulation.forbidden_contact).encode()).hexdigest()}
    report_path = run / 'folding.json'
    if not report_path.is_file():
        output['errors'].append('Missing folding.json; no run interval or log hash is available')
        return output
    try:
        report_bytes = report_path.read_bytes()
        output['folding_report_sha256'] = hashlib.sha256(report_bytes).hexdigest()
        report = _json(report_bytes)
        if not isinstance(report, dict):
            raise ValueError('folding.json must contain an object')
        binding = report.get('applied_contact_evidence')
        if not isinstance(binding, dict):
            output['errors'].append('No applied_contact_evidence binding; historical qpos/event traces cannot establish applied loads')
            return output
        final = _run_interval(report, binding)
        expected_sha = binding.get('sha256')
        if not isinstance(expected_sha, str) or not re.fullmatch('[0-9a-f]{64}', expected_sha):
            raise ValueError('Applied contact evidence needs an exact SHA256 byte identity')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        output.update(status='CONTACT_EVIDENCE_INCOMPLETE', errors=[str(exc)])
        return output
    path = run / LOG_NAME
    if not path.is_file():
        output['errors'].append('Bound applied contact log is missing')
        return output
    if path.resolve().parent != run:
        output.update(status='CONTACT_EVIDENCE_INCOMPLETE', errors=['Applied contact artifact resolves outside the run directory'])
        return output
    output['expected_interval_s'] = [0., final]
    observed = {'lines': 0, 'replay': False, 'last_start': None, 'last_end': None}
    try:
        # Use the same open file for hash verification and decompression. Rehash
        # after scoring to refuse a concurrent rewrite of the evidence bytes.
        with path.open('rb') as raw:
            actual_sha = hashlib.file_digest(raw, 'sha256').hexdigest()
            output['log_sha256'] = actual_sha
            if actual_sha != expected_sha:
                output.update(status='CONTACT_EVIDENCE_HASH_MISMATCH', errors=['Compressed contact log hash differs from folding.json'])
                return output
            output['hash_verified'] = True
            raw.seek(0)
            with gzip.GzipFile(fileobj=raw, mode='rb') as compressed:
                def samples():
                    for line_no, line in enumerate(compressed, 1):
                        if not line.strip():
                            raise ValueError(f'Contact log line {line_no} is empty')
                        sample = _json(line)
                        if not isinstance(sample, dict):
                            raise ValueError(f'Contact log line {line_no} is not an object')
                        observed['lines'] += 1
                        a, b = sample.get('step_started_at'), sample.get('step_ended_at')
                        # The core scorer validates every interval. This only
                        # distinguishes replay/overlap from missing coverage.
                        try:
                            a, b = _number(a), _number(b)
                            if (observed['last_start'] is not None and
                                    (a <= observed['last_start'] or a < observed['last_end']-1e-9)):
                                observed['replay'] = True
                            observed['last_start'], observed['last_end'] = a, b
                        except ValueError:
                            pass
                        yield sample
                score = score_applied_contacts(samples(), expected_start_time=0.,
                    expected_end_time=final, forbidden_contact=_forbidden)
            raw.seek(0)
            if hashlib.file_digest(raw, 'sha256').hexdigest() != expected_sha:
                output.update(status='CONTACT_EVIDENCE_HASH_MISMATCH', hash_verified=False,
                              errors=['Compressed contact log changed while it was being scored'])
                return output
        if report_path.read_bytes() != report_bytes:
            output.update(status='CONTACT_EVIDENCE_INCOMPLETE', errors=['folding.json changed while the log was being scored'])
            return output
        output['contact_score'] = score
        output['recorded_steps'] = observed['lines']
        output['errors'] = list(score['errors'])
        if not observed['lines']:
            output['status'] = 'CONTACT_EVIDENCE_NO_DATA'
        elif observed['replay']:
            output['status'] = 'CONTACT_EVIDENCE_REPLAY'
        elif not score['coverage_complete']:
            output['status'] = 'CONTACT_EVIDENCE_INCOMPLETE'
        elif not score['passed']:
            output['status'] = 'LOADED_UNINTENDED_CONTACT'
        else:
            output.update(status='CONTACT_ONLY_CLEAR', contact_only_passed=True)
    except (OSError, EOFError, ValueError, KeyError, TypeError) as exc:
        output.update(status='CONTACT_EVIDENCE_INCOMPLETE', errors=[f'Cannot score complete gzip JSONL: {exc}'])
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Saved folding run directory')
    parser.add_argument('--output', type=Path, help='Derived score path; default RUN/contact-only-score.json')
    args = parser.parse_args(argv)
    destination = args.output or args.run / 'contact-only-score.json'
    if destination.resolve() in {(args.run/'folding.json').resolve(), (args.run/LOG_NAME).resolve()}:
        parser.error('The derived contact score cannot overwrite its source report or log')
    result = score_run(args.run)
    atomic_json(destination, result)
    print(json.dumps({'status': result['status'], 'contact_only_passed': result['contact_only_passed'],
                      'task_success_evaluated': False, 'output': str(destination)}, sort_keys=True))
    return 0 if result['contact_only_passed'] else 1 if result['status'] == 'LOADED_UNINTENDED_CONTACT' else 2


if __name__ == '__main__':
    raise SystemExit(main())
