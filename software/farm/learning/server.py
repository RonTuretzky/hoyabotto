"""Policy server: load a checkpoint once on the training machine and answer "what next?" over HTTP.

Runs on the computer with the GPU (the M4 Max, or a rented box). The robot
laptop keeps every safety decision: this process never opens a serial port,
holds no calibration, and only ever returns numbers. If it is slow, wrong or
switched off, the policy skill on the robot stops and holds.

    farm policy-server --checkpoint data-train/act_farm --port 8766

    GET  /spec    what the policy expects (cameras, state and action size, chunk length)
    POST /reset   clear the policy's action queue (start of an episode)
    POST /act     {"state": [...], "images": {"<key>": "<base64 JPEG>"}, "task": null}
                  -> {"actions": [[...], ...], "ms": 41.2}     one whole chunk, unnormalized

If FARM_POLICY_TOKEN is set, every request must carry it in the X-Farm-Token header.
"""
from __future__ import annotations

import base64
import os
import threading
import time
from typing import Any

import numpy as np
from pydantic import BaseModel

TOKEN_HEADER = "X-Farm-Token"


class ActRequest(BaseModel):
    state: list[float]
    images: dict[str, str] = {}
    task: str | None = None


def encode_image(rgb: np.ndarray, quality: int = 85) -> str:
    """HxWx3 uint8 RGB -> base64 JPEG."""
    import cv2
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise ValueError("could not encode image")
    return base64.b64encode(buf.tobytes()).decode("ascii")


def decode_image(b64: str) -> np.ndarray:
    """base64 JPEG -> HxWx3 uint8 RGB."""
    import cv2
    arr = cv2.imdecode(np.frombuffer(base64.b64decode(b64), dtype=np.uint8), cv2.IMREAD_COLOR)
    if arr is None:
        raise ValueError("could not decode image")
    return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)


def make_app(runner, token: str | None = None, checkpoint: str = ""):
    """`runner` needs input_spec(), reset() and act_chunk(state, images, task) -> (T, A) array."""
    from fastapi import FastAPI, Header, HTTPException

    app = FastAPI(title="farm policy server")
    lock = threading.Lock()          # one forward pass at a time; the accelerator is not re-entrant
    stats = {"calls": 0, "last_ms": None, "started": time.time()}

    def check(tok: str | None) -> None:
        if token and tok != token:
            raise HTTPException(status_code=401, detail="bad or missing token")

    @app.get("/spec")
    def spec(x_farm_token: str | None = Header(default=None)) -> dict[str, Any]:
        check(x_farm_token)
        s = dict(runner.input_spec())
        s["cameras"] = {k: list(v) for k, v in s.get("cameras", {}).items()}
        return {**s, "checkpoint": checkpoint, **stats}

    @app.post("/reset")
    def reset(x_farm_token: str | None = Header(default=None)) -> dict[str, bool]:
        check(x_farm_token)
        with lock:
            runner.reset()
        return {"ok": True}

    @app.post("/act")
    def act(req: ActRequest, x_farm_token: str | None = Header(default=None)) -> dict[str, Any]:
        check(x_farm_token)
        try:
            images = {k: decode_image(v) for k, v in req.images.items()}
            state = np.asarray(req.state, dtype=np.float32)
            t0 = time.perf_counter()
            with lock:
                chunk = np.asarray(runner.act_chunk(state, images, req.task), dtype=np.float32)
            ms = (time.perf_counter() - t0) * 1000
        except (KeyError, ValueError) as e:        # wrong cameras / state size: the caller's fault
            raise HTTPException(status_code=422, detail=str(e))
        if chunk.ndim == 1:
            chunk = chunk.reshape(1, -1)
        if not np.all(np.isfinite(chunk)):
            raise HTTPException(status_code=500, detail="policy produced non-finite actions")
        stats["calls"] += 1
        stats["last_ms"] = round(ms, 1)
        return {"actions": chunk.tolist(), "ms": round(ms, 1)}

    return app


def serve(checkpoint: str, device: str = "mps", host: str = "0.0.0.0", port: int = 8766) -> None:
    import uvicorn

    from .infer import PolicyRunner
    runner = PolicyRunner(checkpoint, device=device)
    token = os.environ.get("FARM_POLICY_TOKEN") or None
    spec = runner.input_spec()
    print("policy:", spec)
    # The first forward pass on a fresh accelerator takes seconds; pay that here, not on the robot's first request.
    t0 = time.perf_counter()
    runner.act_chunk(np.zeros(spec["state_dim"] or 0, dtype=np.float32), {k: np.zeros((*hw, 3), dtype=np.uint8) for k, hw in spec["cameras"].items()})
    runner.reset()
    print(f"warm-up pass took {time.perf_counter() - t0:.1f} s")
    print(f"serving on http://{host}:{port}  (token {'required' if token else 'not set: anyone on this network can query it'})")
    uvicorn.run(make_app(runner, token=token, checkpoint=runner.path), host=host, port=port, log_level="warning")
