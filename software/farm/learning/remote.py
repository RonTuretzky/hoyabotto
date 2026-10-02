"""Client side of the policy server: looks like a local policy to PolicySkill.

One HTTP call fetches a whole action chunk; the client plays `replan_steps`
actions from it, then asks again with a fresh observation. Any failure
(timeout, refused connection, bad status, wrong shape) raises, and PolicySkill
turns that into "stop, hold, report". Nothing is retried and no stale action
is replayed after an error.
"""
from __future__ import annotations

import os
from typing import Any

import numpy as np

from .server import TOKEN_HEADER, encode_image


class RemotePolicyError(RuntimeError):
    pass


class RemotePolicy:
    def __init__(self, url: str, token: str | None = None, timeout_s: float = 2.0, replan_steps: int | None = None, client=None, jpeg_quality: int = 85):
        import httpx
        self.url = url.rstrip("/")
        self.timeout_s = timeout_s
        self.jpeg_quality = jpeg_quality
        token = token if token is not None else os.environ.get("FARM_POLICY_TOKEN")
        self._headers = {TOKEN_HEADER: token} if token else {}
        self._client = client or httpx.Client(timeout=timeout_s)
        self._queue: list[np.ndarray] = []
        self._spec: dict[str, Any] | None = None
        self.replan_steps = replan_steps
        self.calls = 0
        self.last_ms: float | None = None

    def _request(self, method: str, path: str, **kw) -> dict[str, Any]:
        try:
            r = self._client.request(method, self.url + path, headers=self._headers, **kw)
        except Exception as e:  # noqa: BLE001 - timeout, refused, DNS: all mean "no policy right now"
            raise RemotePolicyError(f"policy server unreachable at {self.url}: {type(e).__name__}: {e}") from e
        if r.status_code != 200:
            raise RemotePolicyError(f"policy server answered {r.status_code}: {r.text[:200]}")
        return r.json()

    def input_spec(self) -> dict[str, Any]:
        if self._spec is None:
            self._spec = self._request("GET", "/spec")
        return self._spec

    def reset(self) -> None:
        self._queue = []
        self._request("POST", "/reset")

    def act(self, state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray:
        """Next action (A,). Fetches a new chunk only when the local queue is empty."""
        if not self._queue:
            body = {"state": [float(x) for x in np.asarray(state).reshape(-1)],
                    "images": {k: encode_image(v, self.jpeg_quality) for k, v in images.items()}}
            out = self._request("POST", "/act", json=body)
            chunk = np.asarray(out.get("actions"), dtype=np.float32)
            if chunk.ndim != 2 or chunk.shape[0] == 0 or not np.all(np.isfinite(chunk)):
                raise RemotePolicyError(f"policy server returned an unusable chunk of shape {chunk.shape}")
            n = self.replan_steps or self.input_spec().get("n_action_steps") or chunk.shape[0]
            self._queue = [chunk[i] for i in range(min(int(n), chunk.shape[0]))]
            self.calls += 1
            self.last_ms = out.get("ms")
        return self._queue.pop(0)
