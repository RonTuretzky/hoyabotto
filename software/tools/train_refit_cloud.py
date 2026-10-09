"""One bounded, simulation-only cloud job: audited demos, images, ACT training.

Run only inside a portable package made by package_refit_cloud.py. The provider
timeout is the spending limit. No hardware client or robot endpoint is used.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import time


def run(argv, cwd, timeout):
    print(json.dumps({'stage_command': argv, 'at': time.time()}), flush=True)
    subprocess.run(argv, cwd=cwd, check=True, timeout=timeout)


def require_collection(summary, minimum=128, fraction=.8):
    rows = summary['results']
    from tools.record_fold_demos import demo_succeeded
    valid = sum(bool(r.get('success')) and demo_succeeded(r) for r in rows)
    if len(rows) != summary['episodes'] or valid != summary['successes']:
        raise RuntimeError('Collection summary disagrees with audited trial results')
    if valid < minimum or valid / max(1, len(rows)) < fraction:
        raise RuntimeError(f'Demonstration gate failed: {valid}/{len(rows)}')
    return valid


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset-repo', required=True)
    ap.add_argument('--model-repo', required=True)
    ap.add_argument('--episodes', type=int, default=320)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--offset-x', nargs=2, type=float, default=[-.005,.005])
    ap.add_argument('--yaw', nargs=2, type=float, default=[-1,1])
    ap.add_argument('--stiffness', nargs=2, type=float, default=[.015,.022])
    args = ap.parse_args()
    software = Path(__file__).resolve().parents[1]
    bundle = software.parent
    sys.path.insert(0, str(software))
    from tools.package_refit_cloud import relocate
    from huggingface_hub import HfApi
    api = HfApi()
    for repo, kind in [(args.dataset_repo, 'dataset'), (args.model_repo, 'model')]:
        info = api.repo_info(repo, repo_type=kind)
        if not info.private:
            raise RuntimeError(f'{kind} destination must be private')
    os.environ.update(PYTHONPATH=str(software), OMP_NUM_THREADS='1',
                      OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    root = bundle / 'simulation'
    relocate(root)
    work = bundle / 'work'
    work.mkdir(exist_ok=False)
    demos, dataset = work/'demos', work/'dataset'
    status = {'hardware_commands': False, 'physical_registration_verified': False,
              'dataset_repo': args.dataset_repo, 'model_repo': args.model_repo,
              'collection': vars(args), 'started': time.time()}

    def publish_status(stage):
        status.update(stage=stage, updated=time.time())
        p = work/'status.json'
        p.write_text(json.dumps(status, indent=2))
        api.upload_file(path_or_fileobj=p, path_in_repo='refit/status.json', repo_id=args.model_repo)
        print(json.dumps(status), flush=True)

    try:
        publish_status('recording')
        run([sys.executable, 'tools/record_refit_fold_demos.py',
             '--upstream', str(root/'upstream/assets/robots/xlerobot/xlerobot.xml'),
             '--along', '-.04', '--radius', '.125', '--axis-sign', '-1',
             '--pinch-normal-tilt-degrees', '15', '--pre-height', '.035',
             '--teacher-position', '-.5', '-.7', '.75', '--clearance', '.002', '--',
             '--simulation-root', str(root), '--out', str(demos), '--episodes', str(args.episodes),
             '--seed0', '10000', '--workers', str(args.workers),
             '--offset-x', *map(str,args.offset_x), '--yaw', *map(str,args.yaw),
             '--stiffness', *map(str,args.stiffness)], software, 1800)
        summary = json.loads((demos/'summary.json').read_text())
        status['valid_demos'] = require_collection(summary)
        api.upload_file(path_or_fileobj=demos/'summary.json', path_in_repo='refit/collection.json',
                        repo_id=args.model_repo)
        publish_status('rendering')
        run([sys.executable, 'tools/fold_demos_to_lerobot.py', '--batches', str(demos),
             '--out', str(dataset), '--workers', '6', '--cameras',
             'front=front', 'left_wrist=left_wrist', 'right_wrist=right_wrist'], software, 3600)
        conversion = json.loads((dataset/'conversion.json').read_text())
        info = json.loads((dataset/'meta/info.json').read_text())
        frames = sum(r['frames'] for r in conversion['episodes'])
        if len(conversion['episodes']) < 128 or info['total_frames'] != frames:
            raise RuntimeError('Dataset frame or episode count failed verification')
        status.update(training_episodes=len(conversion['episodes']), frames=frames)
        # Persist original qpos, scenes, complete contact streams and holdouts.
        evidence = work/'simulation-evidence.tar.gz'
        with tarfile.open(evidence, 'w:gz') as archive:
            for p in sorted(demos.rglob('*')):
                if p.is_file() and (p.suffix in ('.json', '.npz', '.xml') or p.name.endswith('.jsonl.gz')):
                    archive.add(p, arcname=str(p.relative_to(work)))
        status['evidence_sha256'] = hashlib.file_digest(evidence.open('rb'), 'sha256').hexdigest()
        api.upload_file(path_or_fileobj=evidence, path_in_repo='refit/simulation-evidence.tar.gz',
                        repo_id=args.model_repo)
        publish_status('uploading_dataset')
        api.upload_folder(repo_id=args.dataset_repo, repo_type='dataset', folder_path=dataset,
                          ignore_patterns=['images/**'])
        # Train from the already-local images/parquet, avoiding a second download.
        publish_status('training')
        run(['lerobot-train', f'--dataset.repo_id={args.dataset_repo}', f'--dataset.root={dataset}',
             '--policy.type=act', '--policy.device=cuda', '--policy.chunk_size=100',
             '--policy.n_action_steps=100', '--policy.optimizer_lr=3e-5', '--policy.private=true',
             f'--policy.repo_id={args.model_repo}', '--batch_size=32', '--steps=25000',
             '--save_freq=5000', '--save_checkpoint_to_hub=true', '--log_freq=100',
             '--num_workers=10', '--env_eval_freq=0', '--wandb.enable=false',
             '--job_name=dcm_refit', f'--output_dir={work / "train"}'], software, 7200)
        publish_status('training_completed_pending_evaluation')
    except BaseException as exc:
        status['failure'] = f'{type(exc).__name__}: {exc}'[:1000]
        publish_status('failed')
        raise


if __name__ == '__main__':
    main()
