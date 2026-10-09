"""Bounded multi-GPU simulation collection, sharded rendering and distributed ACT."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
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


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset-repo', required=True)
    ap.add_argument('--model-repo', required=True)
    ap.add_argument('--gpus', type=int, choices=(4, 8), default=4)
    ap.add_argument('--max-minutes', type=float, default=355,
                    help='Operational timeout; leave five minutes before provider timeout')
    ap.add_argument('--episodes', type=int, default=320)
    ap.add_argument('--steps', type=int, default=25000)
    # Robot conditions (docs/carton-fold-policy-station-gap.md; run 5 on 9 Oct): servo speed, policy tick rate,
    # camera delays, measured head camera pose, per-episode visual jitter. None by default: the 9 Oct recipe.
    ap.add_argument('--arm-cap-ticks-s', type=float, help='teacher speed cap, arm joints (robot ~80-90 ticks/s)')
    ap.add_argument('--jaw-cap-ticks-s', type=float, help='teacher speed cap, jaws (10 ticks per policy tick)')
    ap.add_argument('--sample-dt', type=float, default=.1, help='recording period; the dataset resamples to --fps')
    ap.add_argument('--max-time', type=float, default=75., help='teacher time limit per demonstration (s)')
    ap.add_argument('--fps', type=int, default=10, help='policy tick rate in the dataset')
    ap.add_argument('--camera-lag', nargs='*', default=[], metavar='KEY=SECONDS')
    ap.add_argument('--visual-jitter', type=float, default=0.)
    ap.add_argument('--front-camera-pose', type=Path, help='measured head camera pose JSON (tools/front_camera_from_registration.py)')
    for name in ('base-spacing', 'base-height', 'base-to-table-edge', 'carton-inset', 'along', 'radius',
                 'pinch-normal-tilt-degrees', 'pre-height', 'clearance', 'prepare-near-degrees', 'start-jitter-m',
                 'left-press-along', 'near-release-lift'):
        ap.add_argument('--' + name, type=float, help='forwarded to record_refit_fold_demos')
    ap.add_argument('--teacher-position', type=float, nargs=3)
    ap.add_argument('--axis-sign', type=int, choices=(-1, 1))
    ap.add_argument('--full-grip-orientation', action='store_true')
    args = ap.parse_args(argv)
    if args.front_camera_pose is not None and not args.front_camera_pose.is_file():
        raise SystemExit(f'Missing {args.front_camera_pose}')
    for name, lo, hi in [('base_spacing', .1, .5), ('base_height', -.05, .3),
                         ('base_to_table_edge', 0, .5), ('carton_inset', 0, .1), ('start_jitter_m', 0, .03),
                         ('left_press_along', -.12, .12), ('near_release_lift', .04, .10)]:
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or not lo <= value <= hi):
            ap.error(f'--{name.replace("_", "-")} must be finite and within {lo}..{hi}')
    for name in ('arm_cap_ticks_s', 'jaw_cap_ticks_s', 'sample_dt', 'max_time', 'max_minutes'):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            ap.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.teacher_position is not None and not all(math.isfinite(v) for v in args.teacher_position):
        ap.error('--teacher-position must be finite')
    if args.fps <= 0 or args.episodes <= 0 or args.steps <= 0:
        ap.error('fps, episodes and steps must be positive')
    return args


def station_arguments(args):
    flags = ['--along', '-.04', '--radius', '.125', '--axis-sign', '-1',
             '--pinch-normal-tilt-degrees', '15', '--pre-height', '.035',
             '--teacher-position', '-.5', '-.7', '.75', '--clearance', '.002']
    for name in ('base-spacing', 'base-height', 'base-to-table-edge', 'carton-inset', 'along', 'radius', 'axis-sign',
                 'pinch-normal-tilt-degrees', 'pre-height', 'clearance', 'prepare-near-degrees', 'start-jitter-m',
                 'front-camera-pose', 'left-press-along', 'near-release-lift'):
        value = getattr(args, name.replace('-', '_'))
        if value is not None:
            flag = '--' + name
            if flag in flags:
                flags[flags.index(flag) + 1] = str(value)
            else:
                flags += [flag, str(value)]
    if args.teacher_position is not None:
        start = flags.index('--teacher-position')
        flags[start+1:start+4] = [str(v) for v in args.teacher_position]
    if args.full_grip_orientation:
        flags.append('--full-grip-orientation')
    return flags


def robot_conditions(args):
    """The exact collection flags and timing saved alongside the dataset/model."""
    return dict(arm_cap_ticks_s=args.arm_cap_ticks_s, jaw_cap_ticks_s=args.jaw_cap_ticks_s,
                sample_dt=args.sample_dt, max_time=args.max_time, fps=args.fps,
                camera_lag=list(args.camera_lag), visual_jitter=args.visual_jitter,
                station=station_arguments(args),
                front_camera_pose=json.loads(args.front_camera_pose.read_text()) if args.front_camera_pose else None)


def main(argv=None):
    args = parse_args(argv)
    conditions = robot_conditions(args)
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
                  collection_workers=collection_workers, started=started, episodes=args.episodes, steps=args.steps,
                  robot_conditions=conditions, camera_contract=provenance(camera_contract))
    (work/'robot-conditions.json').write_text(json.dumps(conditions, indent=2))
    api.upload_file(path_or_fileobj=work/'robot-conditions.json', path_in_repo='refit/robot-conditions.json',
                    repo_id=args.model_repo)

    next_status_upload = 0.

    def best_effort_upload(**kwargs):
        nonlocal next_status_upload
        if time.time() < next_status_upload:
            return
        try:
            api.upload_file(**kwargs)
            next_status_upload = time.time() + 300
        except Exception as exc:
            from tools.refit_hub_reporting import retry_delay
            next_status_upload = time.time() + retry_delay(exc, time.time())
            print(json.dumps(dict(reporting_deferred=str(exc)[:1200], retry_after_epoch=next_status_upload)), flush=True)

    def publish(stage):
        status.update(stage=stage, updated=time.time(), seconds_left=max(0,deadline-time.time()))
        path = work/'status.json'
        path.write_text(json.dumps(status, indent=2))
        print(json.dumps(status), flush=True)
        best_effort_upload(path_or_fileobj=path, path_in_repo='refit/status.json', repo_id=args.model_repo)

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
                best_effort_upload(path_or_fileobj=work/f'{stage}-{i}.log',
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
            status['recording_progress'] = dict(completed=done, total=args.episodes, elapsed=elapsed,
                                                successes=sum(bool(r.get('success')) for r in rows))
            if done:
                status['recording_progress']['estimated_seconds_left'] = elapsed * (args.episodes-done)/done

        station_args = conditions['station']
        recorder_args = ['--sample-dt', str(args.sample_dt), '--max-time', str(args.max_time)]
        if args.arm_cap_ticks_s:
            recorder_args += ['--arm-cap-ticks-s', str(args.arm_cap_ticks_s)]
        if args.jaw_cap_ticks_s:
            recorder_args += ['--jaw-cap-ticks-s', str(args.jaw_cap_ticks_s)]
        one([sys.executable,'tools/record_refit_fold_demos.py',
             '--upstream',str(root/'upstream/assets/robots/xlerobot/xlerobot.xml'), *station_args, '--',
             '--simulation-root',str(root),'--out',str(demos),'--episodes',str(args.episodes),
             '--seed0','10000','--workers',str(collection_workers),'--offset-x','-.005','.005',
             '--yaw','-1','1','--stiffness','.015','.022', *recorder_args],'recording',7200,recording_progress)
        summary = json.loads((demos/'summary.json').read_text())
        status['valid_demos'] = require_collection(summary)
        api.upload_file(path_or_fileobj=demos/'summary.json',path_in_repo='refit/collection.json',repo_id=args.model_repo)
        # Save the expensive simulation evidence before any image conversion: scenes, states and outcomes.
        # The per-physics-step contact streams (tens of GB for slow demos) stay in the job; their independent
        # scores are already in each demo.json. Private Hub storage is limited.
        publish('saving_demonstrations')
        evidence = work/'simulation-evidence.tar.gz'
        with tarfile.open(evidence,'w:gz',compresslevel=1) as archive:
            for p in sorted(demos.rglob('*')):
                if p.is_file() and p.suffix.lower() in ('.json','.npz','.xml','.obj','.stl'):
                    archive.add(p,arcname=str(p.relative_to(work)))
        status['evidence_bytes'] = evidence.stat().st_size
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

        converter_args = ['--fps', str(args.fps), '--visual-jitter', str(args.visual_jitter)]
        if args.camera_lag:
            converter_args += ['--camera-lag', *args.camera_lag]
        processes([([sys.executable,'tools/fold_demos_to_lerobot.py','--batches',str(demos),
                    '--out',str(p),'--workers','3','--num-shards',str(gpu_count),'--shard-index',str(i),
                    '--image-writer-threads','8','--cameras','front=front','left_wrist=left_wrist',
                    'right_wrist=right_wrist', *converter_args],dict(os.environ,MUJOCO_EGL_DEVICE_ID=str(i)))
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
                import re
                for line in reversed(lines):
                    match = re.search(r'(\d+)/%d \[([^]]+)\]' % args.steps, line)
                    if match:
                        status['training_step'] = int(match[1])
                        status['training_progress_text'] = match[0]
                        break
                print('\n'.join(lines[-5:]),flush=True)
            # Expose actual per-GPU utilization and memory while all ranks train.
            proc = subprocess.run(['nvidia-smi','--query-gpu=index,utilization.gpu,memory.used',
                                   '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=10)
            status['gpu_utilization'] = proc.stdout.strip().splitlines()

        one(['accelerate','launch','--multi_gpu',f'--num_processes={gpu_count}','--num_machines=1',
             '--mixed_precision=bf16','--num_cpu_threads_per_process=1','tools/train_refit_ddp.py',
             f'--dataset.repo_id={args.dataset_repo}',f'--dataset.root={dataset}',
             '--policy.type=act','--policy.device=cuda','--policy.use_amp=true','--policy.chunk_size=100',
             '--policy.n_action_steps=100','--policy.optimizer_lr=3e-5','--policy.private=true','--policy.push_to_hub=false',
             f'--policy.repo_id={args.model_repo}',f'--batch_size={batch_per_gpu}',f'--steps={args.steps}',
             # Milestones only: every 1k-step checkpoint is ~1.4 GB on the Hub and counts against private storage.
             '--save_freq=5000','--save_checkpoint_to_hub=true','--log_freq=100',
             '--num_workers=8','--env_eval_freq=0','--wandb.enable=false',f'--job_name=dcm_refit_h200x{gpu_count}',
             f'--output_dir={work/"train"}'],'training',max(1,deadline-time.time()),train_progress)
        publish('training_completed_pending_evaluation')
    except BaseException as exc:
        status['failure'] = f'{type(exc).__name__}: {exc}'[:1000]
        publish('failed')
        raise


if __name__ == '__main__':
    main()
