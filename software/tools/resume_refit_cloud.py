"""Resume the frozen eight-H200 refit from a verified Hub checkpoint; no recollection."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import sys
import time

# Script execution adds tools/, not the repository root, to sys.path. Setting
# PYTHONPATH later only affects children, so establish both before local imports.
SOFTWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOFTWARE))
from tools.refit_hub_reporting import parse_training_progress


def read_progress(path, resume_step=0):
    if not path.exists():
        return {}
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size-65536))
        tail = stream.read().decode(errors='replace')
    result = {'training_log_tail': tail.splitlines()[-5:]}
    result.update(parse_training_progress(tail, resume_step=resume_step))
    metrics = [line for line in tail.splitlines() if 'updt_s:' in line and 'step:' in line]
    if metrics:
        result['training_metrics'] = metrics[-1]
    return result


def verify_resume(dataset, checkpoint, expected_step, expected_contract):
    from carton.refit_camera_contract import load_contract, provenance, CONTRACT_FILE, METADATA_FILE
    conversions = json.loads((dataset/'conversion.json').read_text())
    holdout = json.loads((dataset/'holdout.json').read_text())
    train_seeds = [r['seed'] for r in conversions['episodes']]
    held_seeds = [r['seed'] for r in holdout]
    assert len(train_seeds) >= 128 and held_seeds
    assert len(train_seeds) == len(set(train_seeds))
    assert len(held_seeds) == len(set(held_seeds))
    assert not set(train_seeds).intersection(held_seeds)
    assert load_contract(dataset/CONTRACT_FILE) == expected_contract
    assert load_contract(checkpoint/'pretrained_model'/CONTRACT_FILE) == expected_contract
    assert conversions['camera_contract'] == provenance(expected_contract)
    assert json.loads((checkpoint/'pretrained_model'/METADATA_FILE).read_text()) == provenance(expected_contract)
    state = json.loads((checkpoint/'training_state/training_step.json').read_text())
    assert state == dict(step=expected_step, num_processes=8, batch_size=4), state
    for name in ('optimizer_state.safetensors', 'optimizer_param_groups.json', 'rng_state.safetensors'):
        assert (checkpoint/'training_state'/name).stat().st_size > 0
    return dict(training_episodes=len(train_seeds), holdouts=len(held_seeds),
                frames=sum(r['frames'] for r in conversions['episodes']))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model-repo')
    ap.add_argument('--dataset-repo')
    ap.add_argument('--model-revision')
    ap.add_argument('--dataset-revision')
    ap.add_argument('--step', type=int, default=3000)
    ap.add_argument('--preflight-only', action='store_true',
                    help='Validate runtime imports and frozen camera contract without network access')
    args = ap.parse_args()
    from huggingface_hub import HfApi, hf_hub_download, snapshot_download
    from carton.refit_camera_contract import load_contract, provenance
    contract = load_contract()
    if args.preflight_only:
        from tools.refit_hub_reporting import DeferredUploads
        print(json.dumps(dict(runtime_imports_ok=True, camera_contract=provenance(contract))), flush=True)
        return
    if not all((args.model_repo, args.dataset_repo, args.model_revision, args.dataset_revision)):
        ap.error('model/dataset repos and pinned revisions are required for recovery')
    api = HfApi()
    software = SOFTWARE
    os.chdir(software)
    work = software.parent/'work'
    work.mkdir(exist_ok=False)
    os.environ.update(PYTHONPATH=str(software), FOLD_EGL_DEVICE_COUNT='8')
    started = float(os.environ['REFIT_JOB_STARTED'])
    deadline = started+355*60
    os.environ['REFIT_UPLOAD_DEADLINE'] = str(deadline)
    status = json.loads(Path(hf_hub_download(args.model_repo, 'refit/status.json', revision=args.model_revision)).read_text())
    for key in ('recording_progress', 'render_progress', 'training_log_tail', 'gpu_utilization', 'gpu_preflight', 'failure'):
        status.pop(key, None)
    status.update(job_id=os.environ['JOB_ID'], started=started, resume_step=args.step,
                  training_step=args.step, camera_contract=provenance(contract),
                  upload_not_before=float(os.environ.get('REFIT_UPLOAD_NOT_BEFORE', '0')))

    def publish(stage):
        status.update(stage=stage, updated=time.time(), seconds_left=max(0, deadline-time.time()))
        (work/'status.json').write_text(json.dumps(status, indent=2))
        # Frequent updates use provider stdout, not repository commits.
        print(json.dumps(status), flush=True)

    publish('restoring_checkpoint_and_dataset')
    assert api.repo_info(args.model_repo).private
    assert api.repo_info(args.dataset_repo, repo_type='dataset').private
    dataset = work/'dataset'
    with ThreadPoolExecutor(max_workers=2) as pool:
        data_future = pool.submit(snapshot_download, args.dataset_repo, repo_type='dataset',
                                  revision=args.dataset_revision, local_dir=dataset, max_workers=8)
        model_future = pool.submit(snapshot_download, args.model_repo, revision=args.model_revision,
                                   allow_patterns=[f'checkpoints/{args.step:06d}/**'], local_dir=work/'restored', max_workers=8)
        while not (data_future.done() and model_future.done()):
            publish('restoring_checkpoint_and_dataset')
            time.sleep(20)
        data_future.result(); model_future.result()
    checkpoint = work/'restored/checkpoints'/f'{args.step:06d}'
    status.update(verify_resume(dataset, checkpoint, args.step, contract))
    # Dataset contents and optimizer/RNG state are reused without changing the recipe.
    config = json.loads((checkpoint/'pretrained_model/train_config.json').read_text())
    assert config['steps'] == 25000 and config['batch_size'] == 4
    assert config['policy']['optimizer_lr'] == 3e-5
    publish('gpu_preflight')
    def preflight(i):
        out = work/f'gpu-{i}.json'
        subprocess.run([sys.executable, 'tools/refit_gpu_preflight.py', '--out', str(out)],
                       env=dict(os.environ, MUJOCO_EGL_DEVICE_ID=str(i)), check=True,
                       stdout=subprocess.DEVNULL, timeout=180)
        return json.loads(out.read_text())
    with ThreadPoolExecutor(max_workers=8) as pool:
        status['gpu_preflight'] = list(pool.map(preflight, range(8)))
    command = ['accelerate', 'launch', '--multi_gpu', '--num_processes=8', '--num_machines=1',
               '--mixed_precision=bf16', '--num_cpu_threads_per_process=1', 'tools/train_refit_ddp.py',
               f'--config_path={checkpoint}/pretrained_model/train_config.json', '--resume=true',
               f'--dataset.root={dataset}', f'--dataset.revision={args.dataset_revision}',
               f'--output_dir={work}/train', '--policy.push_to_hub=false', '--save_checkpoint_to_hub=true',
               '--job_name=dcm_refit_h200x8_resume']
    log_path = work/'training-0.log'
    publish('training')
    proc = None
    try:
        with log_path.open('w') as log:
            proc = subprocess.Popen(command, cwd=software, env=os.environ, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            while proc.poll() is None:
                if time.time() > deadline:
                    raise TimeoutError('Recovery job operational deadline reached')
                try:
                    status.update(read_progress(log_path, args.step))
                    gpu = subprocess.run(['nvidia-smi', '--query-gpu=index,utilization.gpu,memory.used',
                                          '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=10)
                    status['gpu_utilization'] = gpu.stdout.strip().splitlines()
                except Exception as exc:
                    print(json.dumps(dict(telemetry_warning=str(exc)[:500])), flush=True)
                publish('training')
                time.sleep(20)
            if proc.returncode:
                print(log_path.read_text(errors='replace')[-18000:], flush=True)
                raise RuntimeError(f'Trainer exited {proc.returncode}')
        status.update(read_progress(log_path, args.step))
        publish('training_completed_pending_evaluation')
        # One final artifact commit after the trainer has flushed all required checkpoints.
        try:
            api.upload_folder(repo_id=args.model_repo, folder_path=work, path_in_repo=f'refit/recovery-{os.environ["JOB_ID"]}',
                              allow_patterns=['status.json', 'training-0.log'])
        except Exception as exc:
            print(json.dumps(dict(final_telemetry_warning=str(exc)[:1200])), flush=True)
    except BaseException as exc:
        status['failure'] = f'{type(exc).__name__}: {exc}'[:1200]
        publish('failed')
        raise
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


if __name__ == '__main__':
    main()
