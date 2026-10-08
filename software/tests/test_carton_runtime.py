import json
import runpy
from pathlib import Path
import pytest

RUNTIME=Path(__file__).resolve().parents[1]/'scripts/carton_robot/carton_runtime.py'

def test_registered_camera_identity_is_not_guessed_from_manifest(tmp_path,monkeypatch):
    monkeypatch.setenv('CARTON_SESSION_DIR',str(tmp_path))
    monkeypatch.delenv('CARTON_HEAD_ID',raising=False)
    (tmp_path/'config.json').write_text(json.dumps({'camera':{'identities':{'head':'registered-head'}}}))
    runtime=runpy.run_path(str(RUNTIME))
    assert runtime['camera_identity']('head')=='registered-head'
    with pytest.raises(RuntimeError,match='Registered camera'):runtime['camera_identity']('right_wrist')

def test_deployment_paths_and_identity_are_explicit(tmp_path,monkeypatch):
    monkeypatch.setenv('CARTON_WORKSPACE_ROOT',str(tmp_path))
    monkeypatch.setenv('CARTON_FRAMES_DIR',str(tmp_path/'frames'))
    monkeypatch.setenv('CARTON_HEAD_ID','expected-head')
    runtime=runpy.run_path(str(RUNTIME))
    assert runtime['ROOT']==tmp_path
    assert runtime['FRAMES']==tmp_path/'frames'
    assert runtime['camera_identity']('head')=='expected-head'
