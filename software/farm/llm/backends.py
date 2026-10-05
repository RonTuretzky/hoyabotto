"""LLM backends. Two real ones:

- ClaudeCLI: the `claude` CLI on the user's subscription, headless (`-p --output-format json`).
  Images are passed as file paths the CLI reads with its Read tool.
- OpenRouter: chat/completions with image data URLs, for Astra and vision.
- DecisionsClient: pooled text-only native Jev API, resolved lazily by Backends.

Both expose complete_json(prompt, images) -> (dict, Meta). Cost is tracked so
the orchestrator can refuse to spend past the daily budget. No backend ever
gets to write servo commands: they return typed JSON that the caller validates.
"""
from __future__ import annotations

import base64
import json
import logging
import os
import re
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class Meta:
    backend: str
    model: str
    latency_ms: float
    cost_usd: float = 0.0
    raw: str = ""
    tokens: dict[str, Any] = field(default_factory=dict)


class LLMError(RuntimeError):
    pass


def extract_json(text: str) -> dict[str, Any]:
    """Pull the first JSON object out of a model reply (code fences tolerated)."""
    if not text:
        raise LLMError("empty reply")
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.S)
    cand = m.group(1) if m else None
    if cand is None:
        start = text.find("{")
        if start < 0:
            raise LLMError(f"no JSON in reply: {text[:200]!r}")
        depth = 0
        for i, ch in enumerate(text[start:], start):
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    cand = text[start:i + 1]
                    break
        if cand is None:
            raise LLMError(f"unterminated JSON: {text[:200]!r}")
    try:
        return json.loads(cand)
    except json.JSONDecodeError as e:
        raise LLMError(f"bad JSON ({e}): {cand[:200]!r}") from e


def _to_jpeg_bytes(img: np.ndarray | bytes | Path) -> bytes:
    if isinstance(img, (bytes, bytearray)):
        return bytes(img)
    if isinstance(img, Path):
        return img.read_bytes()
    import cv2
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise LLMError("jpeg encode failed")
    return buf.tobytes()


class ClaudeCLI:
    """Headless Claude Code on the subscription. Requires `claude` on PATH and a logged-in CLI."""
    name = "claude-cli"

    def __init__(self, model: str = "", timeout_s: float = 120, max_turns: int = 4):
        self.model = model
        self.timeout_s = timeout_s
        self.max_turns = max_turns
        self.bin = shutil.which("claude")
        if not self.bin:
            raise LLMError("claude CLI not found on PATH")

    def complete_json(self, prompt: str, images: list[tuple[str, Any]] | None = None, system: str = "") -> tuple[dict[str, Any], Meta]:
        images = images or []
        tmp = tempfile.mkdtemp(prefix="farm-llm-")
        try:
            paths = []
            for label, img in images:
                p = Path(tmp) / f"{re.sub(r'[^a-z0-9_]+', '_', label.lower())}.jpg"
                p.write_bytes(_to_jpeg_bytes(img)); paths.append((label, p))
            parts = []
            if system:
                parts.append(system)
            if paths:
                parts.append("First, Read each of these images (they are camera frames):")
                parts += [f"- {label}: {p}" for label, p in paths]
            parts.append(prompt)
            parts.append("Reply with a single JSON object and nothing else.")
            cmd = [self.bin, "-p", "\n\n".join(parts), "--output-format", "json", "--allowedTools", "Read",
                   "--max-turns", str(self.max_turns)]
            if self.model:
                cmd += ["--model", self.model]
            env = {k: v for k, v in os.environ.items() if k != "CLAUDECODE"}  # allow nesting from inside Claude Code
            t0 = time.time()
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=self.timeout_s, cwd=tmp, env=env)
            lat = (time.time() - t0) * 1000
            if out.returncode != 0:
                raise LLMError(f"claude exit {out.returncode}: {out.stderr[-400:]}")
            d = json.loads(out.stdout)
            if d.get("is_error") or d.get("subtype") not in (None, "success"):
                raise LLMError(f"claude error: {d.get('subtype')} {str(d.get('result'))[:200]}")
            text = d.get("result") or ""
            meta = Meta(self.name, self.model or "cli-default", lat, float(d.get("total_cost_usd") or 0.0), text, {"usage": d.get("usage", {})})
            return extract_json(text), meta
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class OpenRouter:
    name = "openrouter"
    url = "https://openrouter.ai/api/v1/chat/completions"

    def __init__(self, model: str, timeout_s: float = 120, api_key: str | None = None, max_tokens: int = 1200):
        self.model = model
        self.timeout_s = timeout_s
        self.key = api_key or os.environ.get("OPENROUTER_API_KEY", "")
        self.max_tokens = max_tokens
        if not self.key:
            raise LLMError("OPENROUTER_API_KEY not set (software/.env)")

    def complete_json(self, prompt: str, images: list[tuple[str, Any]] | None = None, system: str = "", model: str | None = None) -> tuple[dict[str, Any], Meta]:
        import httpx
        content: list[dict[str, Any]] = []
        for label, img in (images or []):
            b64 = base64.b64encode(_to_jpeg_bytes(img)).decode()
            content.append({"type": "text", "text": f"Image: {label}"})
            content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})
        content.append({"type": "text", "text": prompt + "\n\nReply with a single JSON object and nothing else."})
        msgs = ([{"role": "system", "content": system}] if system else []) + [{"role": "user", "content": content}]
        body = {"model": model or self.model, "messages": msgs, "max_tokens": self.max_tokens, "usage": {"include": True}}
        t0 = time.time()
        r = httpx.post(self.url, headers={"Authorization": f"Bearer {self.key}", "Content-Type": "application/json",
                                          "HTTP-Referer": "https://github.com/RonTuretzky/xlerobot-farm", "X-Title": "xlerobot-farm"},
                       json=body, timeout=self.timeout_s)
        lat = (time.time() - t0) * 1000
        if r.status_code != 200:
            raise LLMError(f"openrouter {r.status_code}: {r.text[:300]}")
        d = r.json()
        if "error" in d:
            raise LLMError(f"openrouter: {d['error']}")
        msg = d["choices"][0]["message"]
        text = msg.get("content") or ""
        usage = d.get("usage", {})
        meta = Meta(self.name, d.get("model", body["model"]), lat, float(usage.get("cost") or 0.0), text, usage)
        if d["choices"][0].get("finish_reason") == "length" and not text.strip().endswith("}"):
            raise LLMError("reply truncated (raise max_tokens)")
        return extract_json(text), meta


class NoLLM:
    """Placeholder that refuses; used by the simulator profile so tests never hit the network."""
    name = "none"
    model = "none"

    def complete_json(self, *a, **k):
        raise LLMError("no LLM backend configured (llm.backend=none)")


class ScriptedLLM:
    """Deterministic backend for tests: returns queued replies in order."""
    name = "scripted"
    model = "scripted"

    def __init__(self, replies: list[dict[str, Any]] | None = None):
        self.replies = list(replies or [])
        self.calls: list[dict[str, Any]] = []

    def complete_json(self, prompt: str, images=None, system: str = "", model: str | None = None):
        self.calls.append({"prompt": prompt, "n_images": len(images or []), "model": model})
        if not self.replies:
            raise LLMError("scripted backend exhausted")
        return self.replies.pop(0), Meta(self.name, self.model, 1.0, 0.0, "")


class Backends:
    """Resolves the profile into vision / jev / astra backends with fallback."""

    def __init__(self, cfg, store=None):
        self.cfg = cfg
        self.store = store
        self._vision = None
        self._router = None
        self._jev = None

    def _openrouter(self, model: str):
        return OpenRouter(model, timeout_s=self.cfg.timeout_s)

    @property
    def vision(self):
        if self._vision is None:
            if self.cfg.backend in ("none", "sim"):
                self._vision = NoLLM()
            elif self.cfg.backend == "claude-cli":
                try:
                    self._vision = ClaudeCLI(self.cfg.claude_cli_model, self.cfg.timeout_s)
                except LLMError as e:
                    log.warning("claude-cli unavailable (%s); using openrouter vision", e)
                    self._vision = self._openrouter(self.cfg.vision_model)
            else:
                self._vision = self._openrouter(self.cfg.vision_model)
        return self._vision

    @vision.setter
    def vision(self, v):
        self._vision = v

    @property
    def jev(self):
        if self.cfg.backend in ("none", "sim"):
            return NoLLM()
        if self._jev is None:
            from .decisions import DecisionsClient, DEFAULT_MODEL
            model = self.cfg.jev_model
            if model == "typesafe/jev-router":
                log.warning("Migrating legacy jev-router setting to native %s", DEFAULT_MODEL)
                model = DEFAULT_MODEL
            provider = self.cfg.jev_provider
            if provider == "auto":
                provider = "typesafe" if os.environ.get("TYPESAFE_API_KEY") else "openrouter"
            self._jev = DecisionsClient(model, self.cfg.jev_timeout_s, provider=provider)
        return self._jev

    def close(self):
        if self._jev is not None:
            self._jev.close()

    @property
    def astra(self):
        if self.cfg.backend in ("none", "sim"):
            return NoLLM()
        return self._openrouter(self.cfg.astra_model)

    def over_budget(self) -> bool:
        if self.store is None:
            return False
        return self.store.daily_cost() >= self.cfg.max_cost_usd_per_day
