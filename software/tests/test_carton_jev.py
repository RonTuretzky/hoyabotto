import io
import json
import threading
import time

import httpx
import pytest

from carton.jev import CartonAdvisor, ShadowObserver, stream_advice
from farm.evidence.store import EvidenceStore
from farm.llm.decisions import DecisionsClient
from test_decisions import response_for


def snapshot(**overrides):
    return {"observation_id": "frame-1", "observed_at": time.time(), "stage": "VERIFY",
            "step": "pick_paddle", "attempt": 1, "health_ok": True, "stop_requested": False,
            "missing_keyframes": [], "held": {"right": None},
            "judgement": {"box_present": True, "obstruction": False, "paddle_held": False,
                          "notes": "Claw closed beside the paddle; pickup did not succeed."}, **overrides}


def client(handler=None):
    return DecisionsClient(api_key="test", transport=httpx.MockTransport(handler or (
        lambda request: httpx.Response(200, json=response_for(request, "request_plan")))))


@pytest.mark.parametrize("changes,expected", [
    ({"stop_requested": True}, "stop"),
    ({"health_ok": False}, "stop"),
    ({"health_ok": None}, "unknown"),
    ({"observed_at": 0}, "refresh_view"),
    ({"observed_at": time.time() + 10000}, "refresh_view"),
    ({"judgement": {"box_present": True, "obstruction": "unknown"}}, "refresh_view"),
    ({"judgement": {"box_present": True, "obstruction": True}}, "stop"),
    ({"missing_keyframes": ["paddle_grasp"]}, "request_plan"),
])
def test_deterministic_conditions_do_not_call_model(changes, expected):
    def forbidden(_):
        pytest.fail("A hard condition must not call Jev")
    with client(forbidden) as backend:
        out = CartonAdvisor(backend).advise(snapshot(**changes))
        assert out["action"] == expected and out["source"] == "rules"
        assert out["motion_authorized"] is False


def test_real_decision_interface_preserves_observation_identity_and_no_authority():
    with client() as backend:
        out = CartonAdvisor(backend).advise(snapshot())
    assert out["action"] == "request_plan" and out["observation_id"] == "frame-1"
    assert out["source"] == "jev" and out["cost_usd"] == 0.0001
    assert out["motion_authorized"] is False and out["mode"] == "shadow"


def test_observation_expiring_during_inference_cannot_continue():
    now = [time.time()]
    def handler(request):
        now[0] += 10
        return httpx.Response(200, json=response_for(request, "continue_plan"))
    with client(handler) as backend:
        out = CartonAdvisor(backend, clock=lambda: now[0]).advise(snapshot(observed_at=now[0]))
    assert out["action"] == "refresh_view" and out["proposed_action"] == "continue_plan" and out["stale"]


@pytest.mark.parametrize("changes", [{"observed_at": float("nan")}, {"observed_at": True},
                                      {"observation_id": ""}, {"judgement": None}])
def test_invalid_snapshot_rejected(changes):
    with client() as backend, pytest.raises(ValueError):
        CartonAdvisor(backend).advise(snapshot(**changes))


def test_shadow_never_waits_or_queues_and_superseded_reply_is_only_logged(tmp_path):
    started, release = threading.Event(), threading.Event()
    def handler(request):
        started.set()
        assert release.wait(2)
        return httpx.Response(200, json=response_for(request, "continue_plan"))
    store, state = EvidenceStore(tmp_path), {}
    with client(handler) as backend:
        worker = ShadowObserver(CartonAdvisor(backend), store, state, lambda: False)
        try:
            assert worker.submit(snapshot(), "cycle")
            assert started.wait(1)
            assert worker.submit(snapshot(observation_id="frame-2"), "cycle") is False
            release.set()
            worker._future.result(timeout=2)
            assert "jev_advice" not in state
            rows = store.query("SELECT honoured FROM decisions")
            assert len(rows) == 1 and rows[0]["honoured"] == 0
            assert json.loads(store.query("SELECT payload_json FROM events")[0]["payload_json"])["superseded"]
        finally:
            release.set()
            worker.close()


def test_shadow_budget_blocks_dispatch(tmp_path):
    store = EvidenceStore(tmp_path)
    with client() as backend:
        worker = ShadowObserver(CartonAdvisor(backend), store, {}, lambda: True)
        assert worker.submit(snapshot(), "cycle") is False
        assert backend._client is None
        worker.close()


def test_stream_reuses_connection_and_reports_budget_without_devices():
    calls = []
    def handler(request):
        calls.append(request)
        return httpx.Response(200, json=response_for(request, "request_plan", cost=0.01))
    source = io.StringIO("\n".join(json.dumps(snapshot(observation_id=f"f-{i}")) for i in range(3)))
    output = io.StringIO()
    with client(handler) as backend:
        rc = stream_advice(source, output, CartonAdvisor(backend), max_cost_usd=0.02)
    records = [json.loads(line) for line in output.getvalue().splitlines()]
    assert rc == 1 and len(calls) == 2 and len(records) == 3
    assert records[1]["session_cost_usd"] == 0.02 and "error" in records[2]
    assert all(r["motion_authorized"] is False for r in records)


def test_cli_never_constructs_robot_system(monkeypatch, capsys):
    from carton import cli
    monkeypatch.setattr(cli, "_system", lambda *_: pytest.fail("Device construction is forbidden"))
    monkeypatch.setattr("sys.stdin", io.StringIO(json.dumps(snapshot(observed_at=0)) + "\n"))
    cli.main(["jev-advice"])
    out = json.loads(capsys.readouterr().out)
    assert out["action"] == "refresh_view" and out["motion_authorized"] is False


def test_carton_cycle_shadow_preserves_verified_actions(tmp_path):
    from carton.cycle import CartonCycle
    from carton.geometry import Box
    from carton.plan import STEPS
    from test_carton import _sys
    s = _sys(tmp_path)
    s.profile.llm.jev_carton_shadow = True
    s.profile.llm.backend = "openrouter"  # only Jev: vision is still SimCartonVision
    s.backends._jev = client()
    cycle = CartonCycle(s, Box())
    try:
        out = cycle.run()
        if cycle._jev_shadow._future:
            cycle._jev_shadow._future.result(timeout=2)
        assert out.result == "CLOSED" and out.steps_done == [st.name for st in STEPS]
        actions = s.store.query("SELECT skill, result FROM actions ORDER BY intent_t")
        assert [a["skill"] for a in actions] == [st.name for st in STEPS]
        assert all(a["result"] == "VERIFIED" for a in actions)
        assert all(d["honoured"] == 0 for d in s.store.query("SELECT honoured FROM decisions WHERE kind='carton_jev_shadow'"))
    finally:
        s.disconnect()
