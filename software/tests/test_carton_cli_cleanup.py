"""Failure paths must not leave a connected robot or continue to another pose."""
from types import SimpleNamespace

import pytest

from carton import cli


def fake_system(events):
    return SimpleNamespace(
        profile=SimpleNamespace(viewer_port=8765, raw={}),
        disconnect=lambda: events.append("disconnect"),
        keyframes=SimpleNamespace(names=lambda: set()),
        cameras={}, backends=SimpleNamespace(vision=None), store=None,
    )


@pytest.mark.parametrize("problem", ["robot: bad reply", "camera head: no frame"])
def test_partial_connection_is_closed(monkeypatch, problem):
    import farm.system
    import farm.config
    events = []
    system = fake_system(events)
    system.connect = lambda: [problem]
    monkeypatch.setattr(farm.config, "load_profile", lambda _: SimpleNamespace(simulated=False))
    monkeypatch.setattr(farm.system, "System", lambda _: system)
    with pytest.raises(SystemExit, match="required devices"):
        cli._system(SimpleNamespace(profile="unused"))
    assert events == ["disconnect"]


def test_connection_interruption_is_closed(monkeypatch):
    import farm.system
    import farm.config
    events = []
    system = fake_system(events)
    def interrupted():
        raise KeyboardInterrupt
    system.connect = interrupted
    monkeypatch.setattr(farm.config, "load_profile", lambda _: SimpleNamespace(simulated=False))
    monkeypatch.setattr(farm.system, "System", lambda _: system)
    with pytest.raises(KeyboardInterrupt):
        cli._system(SimpleNamespace(profile="unused"))
    assert events == ["disconnect"]


def test_single_pose_has_viewer_and_releases_on_failure(monkeypatch):
    import farm.viewer.app
    events = []
    monkeypatch.setattr(cli, "_system", lambda _: fake_system(events))
    monkeypatch.setattr(cli, "_start_viewer", lambda *_: events.append("viewer"))
    def broken(*_):
        events.append("pose")
        raise RuntimeError("model unavailable")
    monkeypatch.setattr(cli, "_teach_one", broken)
    with pytest.raises(RuntimeError, match="model unavailable"):
        cli.cmd_teach(SimpleNamespace(name="look_box"))
    assert events == ["viewer", "pose", "disconnect"]


def test_batch_stops_at_first_unsuccessful_pose(monkeypatch):
    import farm.viewer.app
    events = []
    monkeypatch.setattr(cli, "_system", lambda _: fake_system(events))
    monkeypatch.setattr(cli, "_start_viewer", lambda *_: events.append("viewer"))
    def failed(_, kf):
        events.append(kf.name)
        return False
    monkeypatch.setattr(cli, "_teach_one", failed)
    with pytest.raises(SystemExit) as result:
        cli.cmd_teach_all(SimpleNamespace(force=False))
    assert result.value.code == 1
    assert events == ["viewer", "look_box", "disconnect"]


@pytest.mark.parametrize("command", [cli.cmd_teach, cli.cmd_teach_all, cli.cmd_once, cli.cmd_run])
def test_viewer_failure_releases_without_motion(monkeypatch, command):
    import farm.viewer.app
    events = []
    monkeypatch.setattr(cli, "_system", lambda _: fake_system(events))
    def no_viewer(*_):
        raise RuntimeError("port occupied")
    def no_motion(*_):
        pytest.fail("motion attempted without a viewer")
    monkeypatch.setattr(farm.viewer.app, "serve_in_thread", no_viewer)
    monkeypatch.setattr(cli, "_start_viewer", no_viewer)
    monkeypatch.setattr(cli, "_teach_one", no_motion)
    monkeypatch.setattr(cli, "_close_one", no_motion)
    with pytest.raises(RuntimeError, match="port occupied"):
        command(SimpleNamespace(name="look_box", force=False, record=False))
    assert events == ["disconnect"]


def test_judgement_failure_closes_connections(monkeypatch):
    from carton import perception
    events = []
    monkeypatch.setattr(cli, "_system", lambda _: fake_system(events))
    def broken(*_):
        raise RuntimeError("bad judgement")
    monkeypatch.setattr(perception, "judge", broken)
    with pytest.raises(RuntimeError, match="bad judgement"):
        cli.cmd_check(SimpleNamespace())
    assert events == ["disconnect"]


def test_viewer_bind_failure_is_reported(monkeypatch):
    import farm.viewer.app as viewer
    import uvicorn
    server = SimpleNamespace(started=False, should_exit=False, run=lambda: None)
    monkeypatch.setattr(viewer, "make_app", lambda _: object())
    monkeypatch.setattr(uvicorn, "Config", lambda *_, **__: None)
    monkeypatch.setattr(uvicorn, "Server", lambda _: server)
    with pytest.raises(RuntimeError, match="STOP viewer failed"):
        viewer.serve_in_thread(None, 8765)
    assert server.should_exit
