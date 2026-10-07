"""Run bounded G4 simulation jobs from one frozen source tree.

Input JSON: {"jobs": [{"id": "case-name", "script":
"tools/diagnose_g4_staging.py", "args": ["--simulation-root", "..."]}]}.
Each worker receives its own --out directory and working directory. This tool
does not interpret process completion as task success or export any policy.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time


SCRIPTS = {'tools/diagnose_g4_staging.py', 'tools/diagnose_g4_carrier.py',
           'tools/train_g4_pusher.py', 'tools/diagnose_g4_station.py',
           'tools/audit_g4_collision_assets.py', 'tools/diagnose_g4_roller.py',
           'tools/diagnose_g4_registration.py', 'tools/diagnose_g4_pusher_transfer.py'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def validate_jobs(value):
    jobs = value.get('jobs')
    if not isinstance(jobs, list) or not jobs:
        raise ValueError('jobs must be a nonempty array')
    seen = set()
    for job in jobs:
        identity = job.get('id', '')
        if not isinstance(identity, str) or not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,79}', identity) or identity in seen:
            raise ValueError('Use unique short lowercase job IDs')
        seen.add(identity)
        if job.get('script') not in SCRIPTS:
            raise ValueError('Only the declared G4 simulation entry points are supported')
        args = job.get('args', [])
        if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
            raise ValueError('Every job args must be an array of strings')
        if any(arg == '--out' or arg.startswith('--out=') for arg in args):
            raise ValueError('The runner assigns --out; do not supply one')
    return jobs


def freeze(software, destination):
    destination.mkdir()
    hashes = {}
    for package in ('farm', 'carton', 'planter', 'tools', 'tests'):
        for path in sorted((software / package).rglob('*')):
            if not path.is_file() or path.suffix not in ('.py', '.json'):
                continue
            if package == 'tests' and path.suffix != '.py':
                continue
            if {'.venv', '.private', '__pycache__'} & set(path.parts):
                continue
            relative = path.relative_to(software)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
            hashes[str(relative)] = digest(target)
    for path in sorted(software.glob('requirements*.txt')):
        target = destination / path.name
        target.write_bytes(path.read_bytes())
        hashes[path.name] = digest(target)
    changed = [name for name, value in hashes.items()
               if not (software / name).is_file() or digest(software / name) != value]
    if changed:
        raise ValueError(f'Sources changed while freezing; use a new output directory and retry: {changed}')
    return hashes


def run_job(job, root, snapshot, timeout):
    work = root / job['id']
    work.mkdir()
    output = work / 'run'
    command = [sys.executable, '-B', str(snapshot / job['script']),
               *job.get('args', []), '--out', str(output)]
    environment = dict(os.environ, PYTHONPATH=str(snapshot), PYTHONDONTWRITEBYTECODE='1',
                       PYTHONUNBUFFERED='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                       MKL_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    row = dict(id=job['id'], command=command, cwd=str(work), output=str(output),
               started_utc=datetime.now(timezone.utc).isoformat(),
               hardware_commands=False, task_success_inferred=False)
    started = time.monotonic()
    save(work / 'worker.json', row)
    print(json.dumps({'started': job['id']}), flush=True)
    with (work / 'stdout.log').open('w') as log:
        try:
            result = subprocess.run(command, cwd=work, env=environment, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=timeout, check=False)
            row['exit_code'] = result.returncode
        except subprocess.TimeoutExpired:
            row.update(exit_code=None, error=f'Worker exceeded {timeout:g} seconds')
    row.update(wall_seconds=time.monotonic() - started,
               finished_utc=datetime.now(timezone.utc).isoformat(),
               result_file_present=(output / 'result.json').is_file())
    if row['result_file_present']:
        row['result_file_sha256'] = digest(output / 'result.json')
    save(work / 'worker.json', row)
    print(json.dumps({key: row.get(key) for key in ('id', 'exit_code', 'error', 'wall_seconds')}), flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--timeout', type=float, default=1800)
    args = parser.parse_args()
    if not 1 <= args.workers <= 3 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Use 1..3 workers and a positive finite timeout')
    specification = json.loads(args.jobs.read_text())
    jobs = validate_jobs(specification)
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=False)
    save(root / 'jobs.json', specification)
    software = Path(__file__).resolve().parents[1]
    snapshot = root / 'source'
    hashes = freeze(software, snapshot)
    versions = {}
    for package in ('mujoco', 'numpy', 'scipy', 'trimesh', 'manifold3d', 'Pillow',
                    'opencv-python', 'opencv-contrib-python', 'pupil-apriltags'):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            pass
    save(root / 'launch-manifest.json', dict(canonical_source_root=str(software),
         executed_source_root=str(snapshot), hashes=hashes, jobs=jobs,
         workers=args.workers, thread_limits=1, timeout_seconds=args.timeout,
         simulation_only=True, hardware_commands=False, python=sys.executable,
         python_version=sys.version, dependency_versions=versions))
    report = dict(workers=args.workers, jobs_total=len(jobs), results=[],
                  simulation_only=True, task_success_inferred=False, hardware_commands=False)
    save(root / 'batch.json', report)
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_job, job, root, snapshot, args.timeout): job for job in jobs}
        for future in as_completed(futures):
            job = futures[future]
            try:
                row = future.result()
            except Exception as exc:
                row = dict(id=job['id'], exit_code=None, error=str(exc), task_success_inferred=False)
            report['results'].append(row)
            report['results'].sort(key=lambda row: row['id'])
            save(root / 'batch.json', report)
    report['wall_seconds'] = time.monotonic() - started
    report['worker_overlap_factor'] = sum(row.get('wall_seconds', 0) for row in report['results']) / report['wall_seconds']
    save(root / 'batch.json', report)
    print(json.dumps({key: report[key] for key in ('jobs_total', 'wall_seconds', 'worker_overlap_factor')}))


if __name__ == '__main__':
    main()
