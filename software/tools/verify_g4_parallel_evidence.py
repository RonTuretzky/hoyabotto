"""Apply the current scorer to saved traces and check evidence integrity.

Agreement with saved outcomes is not re-execution of the original scorer.
GIF decoding checks readability, not timeline completeness. Complete-sequence
claims come from the per-step physics evidence and phase checks.
This is an evidence integrity audit, not a new experiment or policy promotion.
Historical scorer-v1 traces are reported separately and are never retroactively
called qualified. Carrier diagnostics have a separate CAD/contact audit.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
from pathlib import Path

from PIL import Image

from tools.train_g4_pusher import score_episode


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def verify(root):
    episodes = []
    snapshots = []
    snapshot_manifests = []
    invocation_audits = {}
    retention_sources = {}
    for manifest_path in sorted(root.rglob('launch-manifest.json')):
        if {'source', 'source-snapshot'} & set(manifest_path.relative_to(root).parts):
            continue
        manifest = json.loads(manifest_path.read_text())
        source_root = Path(manifest['executed_source_root'])
        mismatches = [name for name, digest in manifest['hashes'].items()
                      if not (source_root / name).is_file() or sha(source_root / name) != digest]
        snapshots.append(dict(path=str(manifest_path.relative_to(root)),
                              files_checked=len(manifest['hashes']), mismatches=mismatches))
        snapshot_manifests.append((source_root, manifest['hashes'], not mismatches))
    for path in sorted(root.rglob('result.json')):
        if {'source', 'source-snapshot'} & set(path.relative_to(root).parts):
            continue
        saved = json.loads(path.read_text())
        if 'target_source' not in saved or 'object_reset_offset_m' not in saved:
            continue
        directory = path.parent
        records = [json.loads(line) for line in (directory / 'physics.jsonl').read_text().splitlines()]
        invocation = json.loads((directory.parent / 'invocation.json').read_text())
        if directory.parent not in invocation_audits:
            hashes = invocation.get('source_hashes', {})
            entry = Path(invocation.get('argv', [''])[0])
            bindings = [(base, frozen, valid) for base, frozen, valid in snapshot_manifests
                        if entry.is_relative_to(base)]
            errors = []
            for source, digest in hashes.items():
                source = Path(source)
                if not source.is_file() or sha(source) != digest:
                    errors.append(f'Hash mismatch: {source}')
                if len(bindings) == 1:
                    base, frozen, _ = bindings[0]
                    if not source.is_relative_to(base) or frozen.get(str(source.relative_to(base))) != digest:
                        errors.append(f'Source outside executed snapshot or digest differs: {source}')
            asset_manifest = directory.parent / 'assets.json'
            assets = json.loads(asset_manifest.read_text()).get('files', []) if asset_manifest.is_file() else []
            asset_errors = [item['path'] for item in assets
                            if not Path(item['path']).is_file() or sha(Path(item['path'])) != item['sha256']]
            invocation_audits[directory.parent] = dict(
                source_files_checked=len(hashes), source_hash_errors=errors,
                source_snapshot_bound=bool(hashes) and len(bindings) == 1 and bindings[0][2] and not errors,
                asset_files_checked=len(assets), asset_hash_errors=asset_errors,
                asset_hashes_match=bool(assets) and not asset_errors)
        timestep = saved.get('timestep_s', invocation.get('geometry', {}).get('timestep',
                    invocation.get('solver', {}).get('timestep', .002)))
        strict = saved.get('scorer_version', 1) >= 2
        rescored = score_episode(records, saved['completed'], timestep_s=timestep,
                                 require_contact_geometry=strict)
        retention = None
        retention_matches = None
        expected_success = rescored['success']
        expected_reasons = list(rescored['failure_reasons'])
        if 'clean_pickup_audit' in saved:
            from planter.g4_retention import audit_retention
            helper = Path(inspect.getfile(audit_retention)).resolve()
            retention_sources[str(helper)] = sha(helper)
            retention = audit_retention(records, timestep_s=timestep,
                                        thresholds=saved['clean_pickup_audit'].get('thresholds'))
            retention_matches = (retention['passed'] == saved['clean_pickup_audit']['passed'] and
                                 retention['failure_reasons'] == saved['clean_pickup_audit']['failure_reasons'])
            if saved.get('require_clean_pickup') and not retention['passed']:
                expected_success = False
                expected_reasons += ['clean_pickup:' + reason for reason in retention['failure_reasons']]
        matches = expected_success == saved['success'] and expected_reasons == saved['failure_reasons']
        artifact_hashes = {}
        missing = []
        required = ['result.json', 'physics.jsonl', 'commands.json', 'timeline.gif']
        if strict:
            required.append('replay-reset.npz')
        if saved['success']:
            required += ['observation-rgb.png', 'observation-depth.npz', 'rgbd-observation.json']
        for name in required:
            artifact = directory / name
            if artifact.exists():
                artifact_hashes[name] = sha(artifact)
            else:
                missing.append(name)
        gif_frames = 0
        gif_error = None
        try:
            with Image.open(directory / 'timeline.gif') as gif:
                for i in range(gif.n_frames):
                    gif.seek(i)
                    gif.load()
                    gif_frames += 1
        except (OSError, EOFError) as exc:
            gif_error = str(exc)
        episodes.append(dict(path=str(directory.relative_to(root)), original_scorer_version=saved.get('scorer_version', 1),
                             strict=strict, original_success=saved['success'], rescored=rescored,
                             clean_pickup_rescored=retention,
                             saved_clean_pickup_outcome_matches_current_audit=retention_matches,
                             saved_outcome_matches_current_scorer=matches,
                             original_scorer_reexecuted=False,
                             **invocation_audits[directory.parent],
                             physics_samples=len(records), timestep_s=timestep,
                             gif_frames_decoded=gif_frames, gif_error=gif_error,
                             missing_required_evidence=missing, artifact_sha256=artifact_hashes))
    strict_rows = [row for row in episodes if row['strict']]
    scorer_path = Path(inspect.getfile(score_episode)).resolve()
    return dict(audit_only=True, scorer_source=str(scorer_path), scorer_sha256=sha(scorer_path),
                retention_auditor_sources=retention_sources,
                verifier_sha256=sha(Path(__file__)),
                episodes=episodes, source_snapshots=snapshots, strict_episodes=len(strict_rows),
                strict_outcomes_agree_with_current_scorer=sum(row['saved_outcome_matches_current_scorer'] for row in strict_rows),
                gif_check_scope='All saved GIF frames decode; does not establish timeline completeness',
                strict_evidence_integrity_passed=bool(strict_rows) and bool(snapshots) and all(not s['mismatches'] for s in snapshots) and all(
                    row['source_snapshot_bound'] and row['asset_hashes_match'] and not row['missing_required_evidence'] and not row['gif_error']
                    and row['gif_frames_decoded'] > 0 for row in strict_rows),
                historical_episodes=len(episodes)-len(strict_rows),
                policy_qualified=False, neural_policy_trained=False,
                full_planter_success=False, physical_success=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Use a new verification path; preserve earlier audits')
    result = verify(args.root.resolve())
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'episodes'}))
