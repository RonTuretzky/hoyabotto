"""Read-only independent scoring of the bounded short probe's panel log.

Usage: python -m tools.score_partial_short_panels --run /path/to/run

Prints derived JSON to stdout. No source files are changed, and no simulator,
model, rendering, hardware or network operation is invoked.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re

from carton.folding_panel_audit import (
    PANEL_SCHEMA, PANEL_SOURCE, PANEL_LOG_NAME, event_digest, finite_number, score_panel_steps,
)


def _json(text):
    return json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def _hash(value):
    if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{64}', value):
        raise ValueError('Exact SHA256 byte identity required for panel evidence')
    return value


def _same(a, b):
    return math.isclose(a, b, rel_tol=0., abs_tol=1e-9)


def _source_interval(binding, folding):
    if (type(binding.get('schema')) is not int or binding.get('schema') != PANEL_SCHEMA
            or binding.get('source') != PANEL_SOURCE
            or binding.get('format') != 'gzip_jsonl' or binding.get('path') != PANEL_LOG_NAME
            or binding.get('source_report') != 'folding.json'):
        raise ValueError('Bound actual-step panel evidence and original folding report required')
    recording = binding.get('recording_id')
    if not isinstance(recording, str) or not re.fullmatch('[0-9a-f]{32}', recording):
        raise ValueError('Unique panel recording identity required')
    first, count = binding.get('source_event_start_index'), binding.get('source_event_count')
    if type(first) is not int or type(count) is not int or first < 0 or count < 1:
        raise ValueError('Positive exact source motion-event interval required')
    start, end, dt = map(finite_number, (binding.get('expected_start_time'),
                         binding.get('expected_end_time'), binding.get('timestep_s')))
    if start < 0 or end <= start or dt <= 0:
        raise ValueError('Positive executed panel interval and timestep required')
    events = folding.get('events')
    if not isinstance(events, list) or first+count > len(events):
        raise ValueError('Bound source motion events are absent')
    selected = events[first:first+count]
    if event_digest(selected) != _hash(binding.get('source_events_sha256')):
        raise ValueError('Source motion-event bytes differ from panel interval binding')
    previous = 0. if first == 0 else finite_number(events[first-1]['time'])
    if not _same(previous, start):
        raise ValueError('Panel interval does not begin at its bound source motion boundary')
    cursor = start
    for event in selected:
        ended, duration = map(finite_number, (event['time'], event['duration_s']))
        if duration <= 0 or ended <= cursor or not _same(ended-duration, cursor):
            raise ValueError('Gap or replay in the source motion events bound to the panel log')
        cursor = ended
    if not _same(cursor, end) or finite_number(folding.get('final_time_s')) < end-1e-9:
        raise ValueError('Panel log end differs from the bound executed source interval')
    return start, end, dt, recording


def score_run(run):
    """Verify byte/interval bindings and stream a saved log without writing it."""
    run = Path(run).resolve()
    output = dict(schema=1, scope='bounded_short_probe_panel_contacts_only',
                  status='PANEL_EVIDENCE_NO_DATA', panel_contact_only_passed=False,
                  task_success_evaluated=False, physical_validation=False,
                  run=str(run), errors=[], panel_score=None, hash_verified=False)
    result_path, folding_path = run/'result.json', run/'folding.json'
    if not result_path.is_file() or not folding_path.is_file():
        output['errors'].append('Missing result.json or folding.json source binding')
        return output
    try:
        result_bytes, folding_bytes = result_path.read_bytes(), folding_path.read_bytes()
        result, folding = _json(result_bytes), _json(folding_bytes)
        if not isinstance(result, dict) or not isinstance(folding, dict):
            raise ValueError('Saved result and folding reports must contain objects')
        probe = result.get('partial_short_probe')
        binding = probe.get('panel_panel_audit') if isinstance(probe, dict) else None
        if not isinstance(binding, dict):
            output['errors'].append('No bound panel evidence; positions or cached outcomes cannot establish contact clearance')
            return output
        start, end, dt, recording = _source_interval(binding, folding)
        expected_hash = _hash(binding.get('sha256'))
        expected_bytes = binding.get('compressed_bytes')
        if type(expected_bytes) is not int or expected_bytes < 1:
            raise ValueError('Exact positive compressed byte count required')
        output.update(expected_interval_s=[start, end], recording_id=recording,
                      result_report_sha256=hashlib.sha256(result_bytes).hexdigest(),
                      folding_report_sha256=hashlib.sha256(folding_bytes).hexdigest())
        path = run/PANEL_LOG_NAME
        if not path.is_file():
            output['errors'].append('Bound actual-step panel contact log is missing')
            return output
        if path.resolve().parent != run:
            raise ValueError('Panel contact artifact resolves outside its source run')
        # The same descriptor is hashed, decompressed, and rehashed. Report
        # bytes are also rechecked so a concurrent rewrite cannot yield a pass.
        with path.open('rb') as raw:
            actual_hash = hashlib.file_digest(raw, 'sha256').hexdigest()
            actual_bytes = os.fstat(raw.fileno()).st_size
            output['log_sha256'] = actual_hash
            if actual_hash != expected_hash or actual_bytes != expected_bytes:
                output.update(status='PANEL_EVIDENCE_HASH_MISMATCH',
                              errors=['Compressed panel log bytes differ from the saved binding'])
                return output
            output['hash_verified'] = True
            raw.seek(0)
            with gzip.GzipFile(fileobj=raw, mode='rb') as compressed:
                def samples():
                    for line_number, line in enumerate(compressed, 1):
                        if not line.strip():
                            raise ValueError(f'Empty panel contact line {line_number}')
                        yield _json(line)
                score = score_panel_steps(samples(), expected_start_time=start, expected_end_time=end,
                                          expected_timestep=dt, recording_id=recording)
            raw.seek(0)
            if hashlib.file_digest(raw, 'sha256').hexdigest() != expected_hash:
                output.update(status='PANEL_EVIDENCE_HASH_MISMATCH', hash_verified=False,
                              errors=['Panel contact log changed while being scored'])
                return output
        if result_path.read_bytes() != result_bytes or folding_path.read_bytes() != folding_bytes:
            raise ValueError('Source reports changed while panel evidence was being scored')
        output.update(panel_score=score, errors=score['errors'])
        if score['replay_detected']:
            output['status'] = 'PANEL_EVIDENCE_REPLAY'
        elif not score['coverage_complete']:
            output['status'] = 'PANEL_EVIDENCE_INCOMPLETE'
        elif not score['passed']:
            output['status'] = 'PANEL_PENETRATION_EXCEEDED'
        else:
            output.update(status='PANEL_CONTACT_ONLY_CLEAR', panel_contact_only_passed=True)
    except (OSError, EOFError, ValueError, KeyError, TypeError) as exc:
        output.update(status='PANEL_EVIDENCE_INCOMPLETE', errors=[str(exc)])
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args(argv)
    result = score_run(args.run)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['panel_contact_only_passed'] else 1 if result['status'] == 'PANEL_PENETRATION_EXCEEDED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
