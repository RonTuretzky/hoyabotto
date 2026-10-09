from concurrent.futures import Future
from types import SimpleNamespace

from tools.refit_hub_reporting import DeferredUploads, retry_delay


class ImmediateExecutor:
    def submit(self, callback):
        f = Future()
        try:
            f.set_result(callback())
        except Exception as exc:
            f.set_exception(exc)
        return f

    def shutdown(self, wait=True):
        pass


def failure(status=429, headers=None):
    exc = RuntimeError('Hub unavailable')
    exc.response = SimpleNamespace(status_code=status, headers=headers or {})
    return exc


def test_rate_limit_does_not_escape_and_respects_server_cooldown():
    now, calls = [100.], []
    q = DeferredUploads(clock=lambda: now[0], executor=ImmediateExecutor())
    def upload():
        calls.append(now[0])
        if len(calls) == 1:
            raise failure(headers={'Retry-After': '3600'})
    q.enqueue('checkpoint', upload)
    q.tick(); q.tick()  # The upload error is observed without escaping into training.
    assert q.next_attempt == 3700
    now[0] = 3699; q.tick()
    assert calls == [100.]
    now[0] = 3700; q.tick(); q.tick()
    assert calls == [100., 3700.]
    assert 'checkpoint' in q.completed


def test_blocked_uploads_coalesce_but_retain_evaluation_milestones():
    now, calls = [0.], []
    q = DeferredUploads(blocked_until=3600, clock=lambda: now[0], executor=ImmediateExecutor())
    q.enqueue('latest', lambda: calls.append(1000))
    q.enqueue('milestone-5000', lambda: calls.append(5000))
    q.enqueue('latest', lambda: calls.append(6000))
    q.tick()
    assert not calls
    now[0] = 3600; q.tick(); q.tick()
    now[0] = 3660; q.tick(); q.tick()
    assert calls == [6000, 5000]


def test_retry_after_http_date_and_safe_fallbacks():
    assert retry_delay(failure(headers={'retry-after':'Thu, 01 Jan 1970 01:00:00 GMT'}), 60) == 3540
    assert retry_delay(failure(), 100) == 3600
    assert retry_delay(failure(status=503), 100) == 300


def test_training_progress_uses_exact_step_instead_of_rounded_metric(tmp_path):
    from tools.resume_refit_cloud import read_progress
    p = tmp_path/'train.log'
    p.write_text('INFO step:3K updt_s:0.29 data_s:0.01\nTraining: 14%|x| 3554/25000 [17:57<1:45:41, 3.38step/s]\n')
    progress = read_progress(p)
    assert progress['training_step'] == 3554
    assert progress['training_metrics'].startswith('INFO step:3K')


def test_newer_checkpoint_survives_failure_of_previous_upload():
    now, calls = [0.], []
    q = DeferredUploads(clock=lambda: now[0], executor=ImmediateExecutor())
    def old():
        raise failure(headers={'Retry-After': '60'})
    q.enqueue('latest', old); q.tick()
    q.enqueue('latest', lambda: calls.append('new'))
    q.tick()
    now[0] = 60; q.tick(); q.tick()
    assert calls == ['new']


def test_recovery_entrypoint_without_pythonpath(tmp_path):
    import json
    import os
    from pathlib import Path
    import subprocess
    import sys
    script = Path(__file__).resolve().parents[1]/'tools/resume_refit_cloud.py'
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    result = subprocess.run([sys.executable, str(script), '--preflight-only'],
                            cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['runtime_imports_ok']


def test_resumed_progress_counts_saved_steps():
    from tools.refit_hub_reporting import parse_training_progress
    p = parse_training_progress('Training: 554/22000 [02:44<1:45:41, 3.38step/s]', resume_step=3000)
    assert p['training_step'] == 3554
    assert abs(p['training_seconds_per_step']-1/3.38) < 1e-10
    assert parse_training_progress('13/96 [00:01<00:03, 20step/s]', resume_step=3000) == {}


def test_progress_seconds_per_step_and_completed_run():
    from tools.refit_hub_reporting import parse_training_progress
    p = parse_training_progress('22000/22000 [1:50:00<00:00, 0.30s/step]', resume_step=3000)
    assert p['training_step'] == 25000
    assert p['training_seconds_per_step'] == .3
