"""Bounded multi-GPU simulation collection, sharded rendering and distributed ACT."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import time


def merge_shards(roots, destination, repo_id, minimum=128):
    """Merge in shard order and verify metadata, indices and holdout separation."""
    from lerobot.datasets.aggregate import aggregate_datasets
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    conversions = [json.loads((p/'conversion.json').read_text()) for p in roots]
    holdouts = [json.loads((p/'holdout.json').read_text()) for p in roots]
    if any(h != holdouts[0] for h in holdouts):
        raise RuntimeError('Shard holdouts disagree')
    episodes = [e for c in conversions for e in c['episodes']]
    seeds = [e['seed'] for e in episodes]
    held = [e['seed'] for e in holdouts[0]]
    if len(set(seeds)) != len(seeds) or len(set(held)) != len(held) or set(seeds) & set(held):
        raise RuntimeError('Duplicate seeds or training/holdout leakage')
    if len(episodes) < minimum or not held:
        raise RuntimeError('Insufficient training episodes or no holdouts')
    for c in conversions:
        if any(c[k] != conversions[0][k] for k in ('task', 'cameras', 'joints', 'fps')):
            raise RuntimeError('Shard feature schema mismatch')
        if c.get('camera_contract') != conversions[0].get('camera_contract'):
            raise RuntimeError('Shard camera contract mismatch')
    aggregate_datasets([f'local/refit_shard_{i}' for i in range(len(roots))], repo_id,
                       roots=roots, aggr_root=destination, concatenate_data=False)
    ds = LeRobotDataset(repo_id, root=destination)
    frames = sum(e['frames'] for e in episodes)
    if ds.num_frames != frames or ds.num_episodes != len(episodes):
        raise RuntimeError('Merged frame or episode count mismatch')
    offset = 0
    for i, ep in enumerate(episodes):
        for index in (offset, offset + ep['frames'] - 1):
            row = ds[index]
            if int(row['episode_index']) != i or int(row['index']) != index:
                raise RuntimeError('Merged dataset index mismatch')
            for cam in conversions[0]['cameras']:
                if row[f'observation.images.{cam}'].ndim != 3:
                    raise RuntimeError('Unreadable merged camera image')
        offset += ep['frames']
    conversion = dict(conversions[0], episodes=episodes, num_shards=len(roots))
    (destination/'conversion.json').write_text(json.dumps(conversion, indent=2))
    (destination/'holdout.json').write_text(json.dumps(holdouts[0], indent=2))
    if conversion.get('camera_contract'):
        from carton.refit_camera_contract import CONTRACT_FILE, load_contract, save_contract, provenance
        contract = load_contract(roots[0] / CONTRACT_FILE)
        if provenance(contract) != conversion['camera_contract']:
            raise RuntimeError('Dataset camera implementation differs from rendered shards')
        for root in roots[1:]:
            if load_contract(root / CONTRACT_FILE) != contract:
                raise RuntimeError('Shard camera contract mismatch')
        save_contract(destination, contract)
    return dict(training_episodes=len(episodes), frames=frames, holdouts=len(held))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset-repo', required=True)
    ap.add_argument('--model-repo', required=True)
    ap.add_argument('--gpus', type=int, choices=(4, 8), default=4)
    ap.add_argument('--max-minutes', type=float, default=355,
                    help='Operational timeout; leave five minutes before provider timeout')
    args = ap.parse_args()
    gpu_count = args.gpus
    batch_per_gpu = 32 // gpu_count
    collection_workers = gpu_count * 8
    software = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(software))
    from huggingface_hub import HfApi
    from tools.package_refit_cloud import relocate
    from tools.train_refit_cloud import require_collection
    api = HfApi()
    for repo, kind in [(args.dataset_repo, 'dataset'), (args.model_repo, 'model')]:
        if not api.repo_info(repo, repo_type=kind).private:
            raise RuntimeError('Cloud destinations must be private')
    os.environ.update(PYTHONPATH=str(software), OMP_NUM_THREADS='1',
                      OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', FOLD_EGL_DEVICE_COUNT=str(gpu_count))
    bundle = software.parent
    root = bundle/'simulation'
    relocate(root)
    work = bundle/'work'
    work.mkdir(exist_ok=False)
    demos, dataset = work/'demos', work/'dataset'
    started = float(os.environ.get('REFIT_JOB_STARTED', time.time()))
    deadline = started + args.max_minutes*60
    from carton.refit_camera_contract import load_contract, provenance, save_contract, CONTRACT_FILE, METADATA_FILE
    camera_contract = load_contract()
    save_contract(work, camera_contract)
    # Root metadata also accompanies final model uploads; checkpoints copy sidecars before upload.
    for name in (CONTRACT_FILE, METADATA_FILE):
        api.upload_file(path_or_fileobj=work/name, path_in_repo=name, repo_id=args.model_repo)
    status = dict(hardware_commands=False, physical_registration_verified=False,
                  dataset_repo=args.dataset_repo, model_repo=args.model_repo,
                  gpus=gpu_count, global_batch_size=32, batch_per_gpu=batch_per_gpu,
                  collection_workers=collection_workers, started=started,
                  camera_contract=provenance(camera_contract))

    def publish(stage):
        status.update(stage=stage, updated=time.time(), seconds_left=max(0,deadline-time.time()))
        path = work/'status.json'
        path.write_text(json.dumps(status, indent=2))
        print(json.dumps(status), flush=True)
        api.upload_file(path_or_fileobj=path, path_in_repo='refit/status.json', repo_id=args.model_repo)

    def processes(commands, stage, timeout, progress=None):
        """Monitor whole subprocess groups; clean up every sibling after failure."""
        children, logs = [], []
        stage_started = time.time()
        stage_deadline = min(deadline, stage_started + timeout)
        try:
            for i, (cmd, env) in enumerate(commands):
                log = (work/f'{stage}-{i}.log').open('w')
                logs.append(log)
                print(json.dumps(dict(command=cmd,stage=stage,gpu=env.get('MUJOCO_EGL_DEVICE_ID'))),flush=True)
                children.append(subprocess.Popen(cmd,cwd=software,env=env,stdout=log,
                                stderr=subprocess.STDOUT,start_new_session=True))
            while True:
                codes = [p.poll() for p in children]
                for i, code in enumerate(codes):
                    if code not in (None, 0):
                        print((work/f'{stage}-{i}.log').read_text()[-12000:], flush=True)
                        raise RuntimeError(f'{stage} process {i} exited {code}')
                if all(code == 0 for code in codes):
                    break
                if time.time() > stage_deadline:
                    raise TimeoutError(f'{stage} exceeded bounded time allowance')
                if progress:
                    progress(time.time()-stage_started)
                publish(stage)
                time.sleep(20)
        finally:
            for p in children:
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGTERM)
            for p in children:
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait()
            for log in logs:
                log.close()
            for i in range(len(logs)):
                api.upload_file(path_or_fileobj=work/f'{stage}-{i}.log',
                                path_in_repo=f'refit/logs/{stage}-{i}.log', repo_id=args.model_repo)

    def one(cmd, stage, timeout, progress=None):
        processes([(cmd,dict(os.environ))],stage,timeout,progress)

    try:
        publish('gpu_preflight')
        processes([( [sys.executable,'tools/refit_gpu_preflight.py','--out',str(work/f'gpu-{i}.json')],
                     dict(os.environ,MUJOCO_EGL_DEVICE_ID=str(i))) for i in range(gpu_count)],'gpu_preflight',180)
        status['gpu_preflight'] = [json.loads((work/f'gpu-{i}.json').read_text()) for i in range(gpu_count)]
        publish('recording')
        def recording_progress(elapsed):
            rows = []
            for path in demos.glob('trial-*/demo.json'):
                try:
                    rows.append(json.loads(path.read_text()))
                except json.JSONDecodeError:
                    continue  # A worker can be in the middle of writing its status.
            done = len(rows)
            status['recording_progress'] = dict(completed=done, total=320, elapsed=elapsed,
                                                successes=sum(bool(r.get('success')) for r in rows))
            if done:
                status['recording_progress']['estimated_seconds_left'] = elapsed * (320-done)/done

        one([sys.executable,'tools/record_refit_fold_demos.py',
             '--upstream',str(root/'upstream/assets/robots/xlerobot/xlerobot.xml'),
             '--along','-.04','--radius','.125','--axis-sign','-1',
             '--pinch-normal-tilt-degrees','15','--pre-height','.035',
             '--teacher-position','-.5','-.7','.75','--clearance','.002','--',
             '--simulation-root',str(root),'--out',str(demos),'--episodes','320',
             '--seed0','10000','--workers',str(collection_workers),'--offset-x','-.005','.005',
             '--yaw','-1','1','--stiffness','.015','.022'],'recording',7200,recording_progress)
        summary = json.loads((demos/'summary.json').read_text())
        status['valid_demos'] = require_collection(summary)
        api.upload_file(path_or_fileobj=demos/'summary.json',path_in_repo='refit/collection.json',repo_id=args.model_repo)
        # Save the expensive simulation evidence before any image conversion.
        publish('saving_demonstrations')
        evidence = work/'simulation-evidence.tar.gz'
        with tarfile.open(evidence,'w:gz',compresslevel=1) as archive:
            for p in sorted(demos.rglob('*')):
                if p.is_file() and (p.suffix.lower() in ('.json','.npz','.xml','.obj','.stl') or p.name.endswith('.jsonl.gz')):
                    archive.add(p,arcname=str(p.relative_to(work)))
        with evidence.open('rb') as f:
            status['evidence_sha256'] = hashlib.file_digest(f,'sha256').hexdigest()
        api.upload_file(path_or_fileobj=evidence,path_in_repo='refit/simulation-evidence.tar.gz',repo_id=args.model_repo)
        roots = [work/f'shard-{i}' for i in range(gpu_count)]

        def render_progress(elapsed):
            rows = [json.loads((p/'conversion-progress.json').read_text()) for p in roots
                    if (p/'conversion-progress.json').exists()]
            done = sum(r['completed'] for r in rows)
            total = sum(r['total'] for r in rows)
            status['render_progress'] = dict(completed=done,total=total,shards_reporting=len(rows),elapsed=elapsed)
            if done and len(rows)==gpu_count:
                estimate = elapsed * (total-done)/done
                status['render_progress']['estimated_seconds_left'] = estimate
                if elapsed > 300 and estimate > deadline-time.time()-900:
                    raise RuntimeError('Rendering forecast leaves insufficient time before operational timeout')

        processes([([sys.executable,'tools/fold_demos_to_lerobot.py','--batches',str(demos),
                    '--out',str(p),'--workers','3','--num-shards',str(gpu_count),'--shard-index',str(i),
                    '--image-writer-threads','8','--cameras','front=front','left_wrist=left_wrist',
                    'right_wrist=right_wrist'],dict(os.environ,MUJOCO_EGL_DEVICE_ID=str(i)))
                    for i,p in enumerate(roots)],'rendering',7200,render_progress)
        publish('merging_dataset')
        status.update(merge_shards(roots,dataset,args.dataset_repo))
        publish('uploading_dataset')
        api.upload_folder(repo_id=args.dataset_repo,repo_type='dataset',folder_path=dataset,ignore_patterns=['images/**'])
        if deadline-time.time() < 900:
            raise RuntimeError('Dataset preserved; insufficient time before operational timeout to start training')
        publish('training')

        def train_progress(elapsed):
            path = work/'training-0.log'
            if path.exists():
                lines = path.read_text(errors='replace').splitlines()
                status['training_log_tail'] = lines[-5:]
                print('\n'.join(lines[-5:]),flush=True)
            # Expose actual per-GPU utilization and memory while all ranks train.
            proc = subprocess.run(['nvidia-smi','--query-gpu=index,utilization.gpu,memory.used',
                                   '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=10)
            status['gpu_utilization'] = proc.stdout.strip().splitlines()

        one(['accelerate','launch','--multi_gpu',f'--num_processes={gpu_count}','--num_machines=1',
             '--mixed_precision=bf16','--num_cpu_threads_per_process=1','tools/train_refit_ddp.py',
             f'--dataset.repo_id={args.dataset_repo}',f'--dataset.root={dataset}',
             '--policy.type=act','--policy.device=cuda','--policy.use_amp=true','--policy.chunk_size=100',
             '--policy.n_action_steps=100','--policy.optimizer_lr=3e-5','--policy.private=true',
             f'--policy.repo_id={args.model_repo}',f'--batch_size={batch_per_gpu}','--steps=25000',
             '--save_freq=1000','--save_checkpoint_to_hub=true','--log_freq=100',
             '--num_workers=8','--env_eval_freq=0','--wandb.enable=false',f'--job_name=dcm_refit_h200x{gpu_count}',
             f'--output_dir={work/"train"}'],'training',max(1,deadline-time.time()),train_progress)
        publish('training_completed_pending_evaluation')
    except BaseException as exc:
        status['failure'] = f'{type(exc).__name__}: {exc}'[:1000]
        publish('failed')
        raise


if __name__ == '__main__':
    main()
