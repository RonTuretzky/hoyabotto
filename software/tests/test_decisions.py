import json
import time

import httpx
import pytest

from farm.config import LLMCfg
from farm.llm.backends import Backends, LLMError
from farm.llm.decisions import DecisionsClient, TYPESAFE_URL, TYPESAFE_MODEL, URL
from farm.llm.jev import EVIDENCE_QUALITY, Jev, question


def response_for(request, choice=None, cost=0.0001):
    body = json.loads(request.content)
    answers = {}
    for name, q in body["questions"].items():
        selected = choice or next(iter(q["criteria"]))
        answers[name] = {"type": "choice", "choice": selected,
                         "probabilities": {k: float(k == selected) for k in q["criteria"]}, "confidence": 1.0}
    return {"answers": answers, "model": "typesafe/jev-1.13-20260917", "id": "test-request",
            "usage": {"cost": cost, "input_tokens": 123, "output_tokens": 10}}


def test_native_endpoint_pools_and_batches_without_images_or_chat():
    calls = []

    def handler(request):
        calls.append(request)
        assert str(request.url) == URL
        assert request.headers["Authorization"] == "Bearer test-key"
        data = json.loads(request.content)
        assert "messages" not in data and "images" not in data
        assert all(q["type"] == "choice" for q in data["questions"].values())
        return httpx.Response(200, json=response_for(request))

    with DecisionsClient(api_key="test-key", transport=httpx.MockTransport(handler)) as backend:
        jev = Jev(backend)
        route, pour = jev.care_decisions({"judgement": {"paper_edge": "dry"}})
        client = backend._client
        jev.evidence_quality({"judgement": {}})
        assert backend._client is client
        assert len(calls) == 2 and len(json.loads(calls[0].content)["questions"]) == 2
        assert route.meta.cost_usd + pour.meta.cost_usd == 0.0001
        assert route.meta.model.endswith("20260917") and route.meta.tokens["request_id"] == "test-request"
    assert client.is_closed


def test_direct_typesafe_endpoint_and_token_cost(monkeypatch):
    monkeypatch.setenv("TYPESAFE_API_KEY", "direct-test-key")
    def handler(request):
        assert str(request.url) == TYPESAFE_URL
        assert request.headers["Authorization"] == "Bearer direct-test-key"
        assert json.loads(request.content)["model"] == TYPESAFE_MODEL
        data = response_for(request)
        data["model"] = TYPESAFE_MODEL
        del data["usage"]["cost"]
        return httpx.Response(200, json=data)
    with DecisionsClient(provider="typesafe", transport=httpx.MockTransport(handler)) as backend:
        answer = Jev(backend).evidence_quality({})
        assert answer.error is None and answer.meta.backend == "typesafe-decisions"
        assert answer.meta.tokens["cost_estimated"] is True
        assert answer.meta.cost_usd == 123 * .042 / 1_000_000
    backends = Backends(LLMCfg())
    assert backends.jev.provider == "typesafe" and backends.jev.model == TYPESAFE_MODEL
    backends.close()


def test_direct_transport_does_not_send_key_to_openrouter_on_failure():
    urls = []
    def handler(request):
        urls.append(str(request.url))
        return httpx.Response(401, json={"error": "secret"})
    with DecisionsClient(provider="typesafe", api_key="test", transport=httpx.MockTransport(handler)) as backend:
        assert Jev(backend).evidence_quality({}).error
        assert Jev(backend).evidence_quality({}).error
    assert urls == [TYPESAFE_URL]


@pytest.mark.parametrize("mutation", [
    lambda a: a.update(choice="move_motor"),
    lambda a: a["probabilities"].pop("unknown"),
    lambda a: a["probabilities"].update(usable=float("nan")),
    lambda a: a["probabilities"].update(usable=True),
    lambda a: a["probabilities"].update(usable=2),
    lambda a: a.update(confidence=float("inf")),
    lambda a: a.update(type="text"),
    lambda a: a.update(choice="unknown"),
])
def test_malformed_choice_never_becomes_permission(mutation):
    def handler(request):
        body = response_for(request)
        mutation(body["answers"]["decision"])
        # NaN cannot be passed to httpx's strict json encoder.
        return httpx.Response(200, content=json.dumps(body))

    with DecisionsClient(api_key="test", transport=httpx.MockTransport(handler)) as backend:
        answer = Jev(backend).evidence_quality({})
    assert answer.choice == "unknown" and answer.confidence == 0


def test_quota_error_opens_circuit_and_redacts_provider_body():
    calls = []

    def handler(request):
        calls.append(request)
        return httpx.Response(403, json={"error": {"message": "Key limit exceeded (daily limit). secret-test-key account-link"}})

    with DecisionsClient(api_key="secret-test-key", transport=httpx.MockTransport(handler)) as backend:
        for _ in range(3):
            c = Jev(backend).evidence_quality({})
            assert c.choice == "unknown" and "daily limit" in c.why
            assert "secret-test-key" not in c.why and "account-link" not in c.why
    assert len(calls) == 1


def test_timeout_returns_unknown_without_retry_and_then_cools_down():
    calls = []

    def handler(request):
        calls.append(request)
        raise httpx.ReadTimeout("sensitive-provider-detail", request=request)

    with DecisionsClient(api_key="test", transport=httpx.MockTransport(handler)) as backend:
        for _ in range(2):
            c = Jev(backend).evidence_quality({})
            assert c.choice == "unknown" and "sensitive" not in c.why
    assert len(calls) == 1


def test_late_success_does_not_become_an_action():
    def handler(request):
        time.sleep(0.01)
        return httpx.Response(200, json=response_for(request))

    with DecisionsClient(api_key="test", timeout_s=0.001, transport=httpx.MockTransport(handler)) as backend:
        c = Jev(backend).evidence_quality({})
        assert c.choice == "unknown" and "deadline" in c.why
        assert c.meta.cost_usd == 0.0001  # a late reply can still be billable


def test_missing_key_and_non_json_state_fail_without_network():
    backend = DecisionsClient(api_key="")
    assert Jev(backend).evidence_quality({}).choice == "unknown"
    assert backend._client is None
    for state in ({"pixels": b"binary"}, {"x": float("nan")}, {"text": "x" * 65000}):
        with pytest.raises(LLMError):
            backend.decide(state, {"quality": question("Quality?", EVIDENCE_QUALITY)})
    backend.close()


def test_backends_cache_native_client_and_migrate_old_router(monkeypatch):
    monkeypatch.setenv("OPENROUTER_API_KEY", "test")
    b = Backends(LLMCfg(jev_model="typesafe/jev-router"))
    assert b.jev is b.jev and b.jev.model == "typesafe/jev-1.13"
    assert b.jev.timeout_s == 2.0
    b.close()


def test_old_chat_decisions_cannot_promote_native_jev(tmp_path):
    from farm.config import AuthorityCfg
    from farm.cycle.authority import Authority
    from farm.evidence.store import EvidenceStore
    from farm.llm.jev import SCHEMA_VERSION
    store = EvidenceStore(tmp_path)
    cfg = AuthorityCfg(jev="approve", jev_shadow_cycles=2)
    store.event(None, "authority_level", {"level": "approve", "who": "old", "why": "old wrapper"})
    for _ in range(3):
        store.decision("old", "jev_route", "q", "", "routine", {}, "farm-jev-1", note="agree:old")
    authority = Authority(cfg, store)
    assert authority.level == "shadow"
    authority.consider_promotion()
    assert authority.level == "shadow"
    authority.set_level("route", "operator", "native test")
    assert Authority(cfg, store).level == "route"
    record = json.loads(store.query("SELECT payload_json FROM events ORDER BY t DESC LIMIT 1")[0]["payload_json"])
    assert record["decision_schema"] == SCHEMA_VERSION
