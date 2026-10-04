"""Native, text-only OpenRouter Decisions transport. No chat or motor interface.

One pooled client per Backends instance; no automatic retries. Provider failures
become typed errors, and quota/auth errors open a circuit for this process.
https://openrouter.ai/docs/guides/community/jev
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any

import httpx

from .backends import LLMError, Meta

DEFAULT_MODEL = "typesafe/jev-1.13"
URL = "https://openrouter.ai/api/alpha/decisions"
MAX_PACKET_BYTES = 64_000


class DecisionError(LLMError):
    def __init__(self, message: str, meta: Meta | None = None):
        super().__init__(message)
        self.meta = meta


def _probability(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 1


def validate_choice(answer: Any, criteria: dict[str, str]) -> dict[str, Any]:
    """Reject malformed distributions instead of inventing/normalizing confidence."""
    if not isinstance(answer, dict) or answer.get("type") != "choice":
        raise LLMError("Jev returned an invalid choice answer")
    choice, probs = answer.get("choice"), answer.get("probabilities")
    if not isinstance(choice, str) or choice not in criteria or not isinstance(probs, dict) or set(probs) != set(criteria):
        raise LLMError("Jev returned an unrequested choice or incomplete probabilities")
    if not all(_probability(p) for p in probs.values()) or not math.isclose(sum(probs.values()), 1.0, abs_tol=0.02):
        raise LLMError("Jev returned invalid probabilities")
    if probs[choice] < max(probs.values()) or not _probability(answer.get("confidence")):
        raise LLMError("Jev returned an inconsistent choice or confidence")
    return {"type": "choice", "choice": choice, "probabilities": dict(probs), "confidence": answer["confidence"]}


class DecisionsClient:
    name = "openrouter-decisions"

    def __init__(self, model: str = DEFAULT_MODEL, timeout_s: float = 2.0, api_key: str | None = None,
                 transport=None, cooldown_s: float = 30.0):
        if model == "typesafe/jev-router":
            raise LLMError("jev-router is a chat router; configure typesafe/jev-1.13 for Decisions")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Jev timeout must be finite and positive")
        self.model, self.timeout_s = model, timeout_s
        self._key = api_key if api_key is not None else os.environ.get("OPENROUTER_API_KEY", "")
        self._transport, self._client = transport, None
        self._lock = threading.Lock()
        self._blocked_until = 0.0
        self._blocked_reason = ""
        self._cooldown_s = cooldown_s
        self._closed = False

    def decide(self, state: dict[str, Any], questions: dict[str, dict[str, Any]]) -> tuple[dict, Meta]:
        if not isinstance(state, dict) or not isinstance(questions, dict) or not questions:
            raise LLMError("Jev needs a state object and named questions")
        for question in questions.values():
            criteria = question.get("criteria") if isinstance(question, dict) else None
            if (not isinstance(question, dict) or question.get("type") != "choice" or
                    not isinstance(question.get("instructions"), str) or not question["instructions"].strip() or
                    not isinstance(criteria, dict) or len(criteria) < 2 or
                    not all(isinstance(k, str) and isinstance(v, str) and v.strip() for k, v in criteria.items())):
                raise LLMError("Jev questions need Choice instructions and described options")
        try:
            encoded = json.dumps({"model": self.model, "state": state, "questions": questions}, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise LLMError("Jev state must be finite JSON; images and binary arrays are not supported") from exc
        if len(encoded.encode()) > MAX_PACKET_BYTES:
            raise LLMError("Jev evidence packet exceeds 64000 bytes; send compact text observations")
        with self._lock:
            if self._closed:
                raise LLMError("Jev client is closed")
            if time.monotonic() < self._blocked_until:
                raise LLMError(self._blocked_reason)
            if not self._key:
                raise LLMError("OPENROUTER_API_KEY is not configured")
            if self._client is None:
                self._client = httpx.Client(timeout=self.timeout_s, transport=self._transport, follow_redirects=False,
                                            headers={"Authorization": f"Bearer {self._key}", "Content-Type": "application/json"})
            started = time.monotonic()
            try:
                response = self._client.post(URL, content=encoded)
            except httpx.RequestError as exc:
                self._block("Jev transport unavailable", permanent=False)
                raise LLMError("Jev transport unavailable") from exc
            latency_ms = (time.monotonic() - started) * 1000
            if response.status_code != 200:
                reason = {401: "Jev authentication rejected", 402: "Jev credit limit reached",
                          403: "Jev access or key limit rejected", 429: "Jev rate limit reached"}.get(
                              response.status_code, f"Jev HTTP {response.status_code}")
                # Never propagate provider bodies: they may contain account links or echoed credentials.
                if response.status_code == 403:
                    try:
                        if "daily limit" in str(response.json().get("error", {}).get("message", "")).lower():
                            reason = "Jev key daily limit exceeded"
                    except (ValueError, AttributeError):
                        pass
                self._block(reason, permanent=response.status_code in (401, 402, 403))
                raise LLMError(reason)
            meta = Meta(self.name, self.model, latency_ms)
            try:
                data = response.json()
                usage = data.get("usage") or {}
                cost = usage.get("cost", 0.0)
                if type(cost) not in (int, float) or not math.isfinite(cost) or cost < 0:
                    raise LLMError("Jev returned invalid usage")
                served_model = data.get("model")
                if not isinstance(served_model, str) or not served_model:
                    raise LLMError("Jev did not identify the served model")
                meta = Meta(self.name, served_model, latency_ms, float(cost), tokens={
                    "input_tokens": usage.get("input_tokens"), "output_tokens": usage.get("output_tokens"),
                    "request_id": data.get("id"), "cost_reported": "cost" in usage})
                answers = data["answers"]
                if set(answers) != set(questions):
                    raise LLMError("Jev returned missing or unexpected answers")
                valid = {name: validate_choice(answers[name], q["criteria"]) for name, q in questions.items()}
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                raise DecisionError("Jev returned an invalid Decisions response", meta) from exc
            except LLMError as exc:
                raise DecisionError(str(exc), meta) from exc
            if latency_ms > self.timeout_s * 1000:
                raise DecisionError("Jev decision arrived after its deadline", meta)
            return valid, meta

    def _block(self, reason: str, permanent: bool) -> None:
        self._blocked_reason = reason
        self._blocked_until = math.inf if permanent else time.monotonic() + self._cooldown_s

    def close(self) -> None:
        with self._lock:
            self._closed = True
            if self._client is not None:
                self._client.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
