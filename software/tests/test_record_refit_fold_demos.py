import pytest
from tools import record_refit_fold_demos as refit


def test_station_wrapper_restores_recorder_configuration_between_runs(tmp_path, monkeypatch):
    original_args = refit.recorder.BASE_ARGS
    original_trial = refit.recorder.TRIAL
    calls = []
    def record(argv):
        calls.append((list(refit.recorder.BASE_ARGS), refit.recorder.TRIAL, argv))
    monkeypatch.setattr(refit.recorder, 'main', record)
    base = ['--upstream', str(tmp_path/'robot.xml')]
    refit.main(base + ['--base-spacing', '.3104', '--carton-inset', '0', '--full-grip-orientation',
                       '--prepare-near-degrees', '-25', '--', '--episodes', '2'])
    assert '--normal-only' not in calls[0][0]
    assert calls[0][0][calls[0][0].index('--prepare-near-degrees')+1] == '-25.0'
    assert "'base_spacing': 0.3104" in calls[0][1]
    assert "'carton_inset': 0.0" in calls[0][1]
    assert calls[0][2] == ['--episodes', '2']
    refit.main(base + ['--', '--episodes', '1'])
    assert '--normal-only' in calls[1][0]
    assert calls[1][0][calls[1][0].index('--prepare-near-degrees')+1] == '-15.0'
    assert calls[1][1].count('def refit_init') == 1
    assert refit.recorder.BASE_ARGS is original_args
    assert refit.recorder.TRIAL is original_trial


def test_wrapper_restores_configuration_after_recording_failure(tmp_path, monkeypatch):
    original_args, original_trial = refit.recorder.BASE_ARGS, refit.recorder.TRIAL
    def fail(argv):
        raise RuntimeError('recording failed')
    monkeypatch.setattr(refit.recorder, 'main', fail)
    with pytest.raises(RuntimeError, match='recording failed'):
        refit.main(['--upstream', str(tmp_path/'robot.xml'), '--full-grip-orientation', '--'])
    assert refit.recorder.BASE_ARGS is original_args
    assert refit.recorder.TRIAL is original_trial
