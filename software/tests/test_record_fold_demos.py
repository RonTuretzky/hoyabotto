import hashlib
import json
import sys

import pytest

from tools.record_fold_demos import demo_succeeded, run_trial
from tools.run_claw_sweep import snapshot_sources


def test_collision_assets_are_frozen_with_python_sources(tmp_path):
    source = tmp_path / 'software'
    asset = source / 'carton/assets/wrist-camera/hull.obj'
    asset.parent.mkdir(parents=True)
    asset.write_bytes(b'v 0 0 0\n')
    snapshot = tmp_path / 'snapshot'
    hashes = snapshot_sources(source, snapshot)
    asset.write_bytes(b'changed')
    relative = asset.relative_to(source)
    assert (snapshot / relative).read_bytes() == b'v 0 0 0\n'
    assert hashes[str(relative)] == hashlib.sha256(b'v 0 0 0\n').hexdigest()


def valid_demo():
    return dict(stop_reason='task flaps folded and held', exit_code=0,
                controller_error=None, contact_audit=dict(steps=3, first_refusal=None,
                                                        independent_score=dict(passed=True)))


@pytest.mark.parametrize('change', [
    {'controller_error': 'Fresh table tag not observed'},
    {'exit_code': 2},
    {'contact_audit': {}},
    {'contact_audit': dict(steps=3, first_refusal={'pair': ['right_wrist_camera', 'carton']},
                         independent_score={'passed': True})},
    {'contact_audit': dict(steps=3, first_refusal=None, independent_score={'passed': False})},
])
def test_folded_angles_cannot_override_failed_evidence(change):
    demo = valid_demo()
    assert demo_succeeded(demo)
    demo.update(change)
    assert not demo_succeeded(demo)


def test_caught_controller_error_is_preserved(tmp_path, monkeypatch):
    import tools.record_fold_demos as recorder
    # A successful subprocess can still report a caught controller failure.
    def fake_run(cmd, **kwargs):
        cfg = json.loads(cmd[-1])
        from pathlib import Path
        Path(cfg['status']).write_text(json.dumps(valid_demo()))
        result = Path(cfg['status']).parent / 'run/result.json'
        result.parent.mkdir()
        result.write_text(json.dumps({'error': 'Required grasp was lost'}))
        from types import SimpleNamespace
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(recorder.subprocess, 'run', fake_run)
    result = run_trial(sys.executable, tmp_path, tmp_path, tmp_path/'trial',
                       0, 0., 0., .018, 'both-shorts', 3., 75.)
    assert result['controller_error'] == 'Required grasp was lost'
    assert not result['success']
