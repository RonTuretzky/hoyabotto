"""Run a trained LeRobot policy checkpoint on raw robot observations (lerobot 0.6.1).

The inference contract, verified against the installed 0.6.1 sources
(`lerobot/scripts/lerobot_eval.py`, `lerobot/policies/factory.py`,
`lerobot/processor/factory.py`):

    cfg    = PreTrainedConfig.from_pretrained(<pretrained_model dir or Hub repo>)
    cfg.device = "mps"
    policy = get_policy_class(cfg.type).from_pretrained(<same path>, config=cfg).eval()
    pre, post = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=<same path>,
        preprocessor_overrides={"device_processor": {"device": "mps"}})
    batch  = pre({"observation.state": (D,) float32,
                  "observation.images.<cam>": (3,H,W) float32 in [0,1], ...})
    action = post(policy.select_action(batch))       # (1, A) float32 on cpu

The preprocessor pipeline saved with an ACT/SmolVLA checkpoint is
`rename_observations -> add_batch_dim -> to_device -> normalize`; the
postprocessor is `unnormalize -> to_cpu`. There is NO resize step, so the
caller must deliver images at the shape recorded in
`cfg.input_features[<cam>].shape == (3, H, W)`; `PolicyRunner.act` does that
resize itself (bilinear, antialiased) and the uint8 -> float [0,1] conversion,
then lets the policy's own normalizer apply the training statistics.

`select_action` keeps an internal queue of `n_action_steps` actions and only
runs the network when the queue is empty; call `reset()` at every episode
start. `act_chunk` always runs the network and returns the whole chunk.

Checkpoint paths accepted: a training output dir (`.../act_so101_pour`), a
checkpoint dir (`.../checkpoints/last` or `.../checkpoints/005000`), the
`pretrained_model` dir itself, or a Hub repo id such as
`lissajous/xlerobot-act-local-grasp-v1`.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)

OBS_STATE = "observation.state"
IMAGE_PREFIX = "observation.images."


def resolve_pretrained_dir(path_or_repo: str | Path) -> str:
    """Map any of the accepted checkpoint locations to the dir holding config.json (or pass a Hub id through)."""
    p = Path(path_or_repo)
    if not p.exists():
        return str(path_or_repo)  # assume Hub repo id; from_pretrained handles download
    for cand in (p, p / "pretrained_model", p / "checkpoints" / "last" / "pretrained_model"):
        if (cand / "config.json").is_file():
            return str(cand.resolve())
    ckpts = sorted((p / "checkpoints").glob("*/pretrained_model/config.json")) if (p / "checkpoints").is_dir() else []
    if ckpts:
        return str(ckpts[-1].parent.resolve())
    raise FileNotFoundError(f"no config.json found under {p}")


def pick_device(requested: str = "mps") -> str:
    import torch

    if requested == "mps" and not torch.backends.mps.is_available():
        log.warning("mps requested but unavailable; falling back to cpu")
        return "cpu"
    if requested == "cuda" and not torch.cuda.is_available():
        log.warning("cuda requested but unavailable; falling back to cpu")
        return "cpu"
    return requested


class PolicyRunner:
    """Load a checkpoint once; turn (state, images) into a joint-position action."""

    def __init__(self, checkpoint_dir_or_repo: str | Path, device: str = "mps"):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        self.path = resolve_pretrained_dir(checkpoint_dir_or_repo)
        self.device = pick_device(device)
        t0 = time.perf_counter()
        cfg = PreTrainedConfig.from_pretrained(self.path)
        cfg.device = self.device
        cfg.pretrained_path = self.path
        self.config = cfg
        policy_cls = get_policy_class(cfg.type)
        self.policy = policy_cls.from_pretrained(self.path, config=cfg)
        self.policy.to(self.device)
        self.policy.eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=cfg,
            pretrained_path=self.path,
            preprocessor_overrides={"device_processor": {"device": self.device}},
        )
        self._torch = torch
        self.load_s = time.perf_counter() - t0
        spec = self.input_spec()
        log.info("loaded %s policy from %s on %s in %.1fs; cameras=%s state_dim=%d action_dim=%d",
                 cfg.type, self.path, self.device, self.load_s, spec["cameras"], spec["state_dim"], spec["action_dim"])

    # ------------------------------------------------------------------ spec
    def input_spec(self) -> dict[str, Any]:
        """Camera keys with the (H, W) each expects, state dim, action dim, and the chunk geometry."""
        cams: dict[str, tuple[int, int]] = {}
        state_dim = None
        for key, ft in self.config.input_features.items():
            shape = tuple(int(s) for s in ft.shape)
            if key.startswith(IMAGE_PREFIX) or key == "observation.image":
                cams[key] = (shape[-2], shape[-1])  # stored as (3, H, W)
            elif key == OBS_STATE:
                state_dim = shape[0]
        action_dim = int(self.config.output_features["action"].shape[0])
        return {
            "policy_type": self.config.type,
            "device": self.device,
            "cameras": cams,
            "camera_names": [k[len(IMAGE_PREFIX):] if k.startswith(IMAGE_PREFIX) else k for k in cams],
            "state_dim": state_dim,
            "action_dim": action_dim,
            "chunk_size": getattr(self.config, "chunk_size", None),
            "n_action_steps": getattr(self.config, "n_action_steps", None),
            "normalization": {k: str(v) for k, v in self.config.normalization_mapping.items()},
        }

    # ------------------------------------------------------------------ control
    def reset(self) -> None:
        """Clear the action queue and any processor state; call at every episode start."""
        self.policy.reset()
        for proc in (self.preprocessor, self.postprocessor):
            if hasattr(proc, "reset"):
                proc.reset()

    def _image_to_tensor(self, key: str, img: np.ndarray, hw: tuple[int, int]):
        torch = self._torch
        arr = np.asarray(img)
        if arr.ndim != 3 or arr.shape[-1] != 3:
            raise ValueError(f"{key}: expected HxWx3 RGB, got {arr.shape}")
        t = torch.from_numpy(np.ascontiguousarray(arr))
        if t.dtype == torch.uint8:
            t = t.to(torch.float32) / 255.0
        else:
            t = t.to(torch.float32)
            if float(t.max()) > 1.0:  # float image still in 0..255
                t = t / 255.0
        t = t.permute(2, 0, 1)  # (3, H, W)
        if tuple(t.shape[-2:]) != tuple(hw):
            t = torch.nn.functional.interpolate(t.unsqueeze(0), size=hw, mode="bilinear", align_corners=False, antialias=True).squeeze(0)
        return t.clamp_(0.0, 1.0)

    def build_batch(self, state: np.ndarray, images: dict[str, np.ndarray], task: str | None = None) -> dict[str, Any]:
        """Unbatched tensors keyed the LeRobot way; the saved preprocessor adds the batch dim, moves to device, normalizes."""
        torch = self._torch
        spec = self.input_spec()
        st = np.asarray(state, dtype=np.float32).reshape(-1)
        if spec["state_dim"] is not None and st.shape[0] != spec["state_dim"]:
            raise ValueError(f"state has {st.shape[0]} dims, policy expects {spec['state_dim']}")
        batch: dict[str, Any] = {OBS_STATE: torch.from_numpy(st)}
        for key, hw in spec["cameras"].items():
            name = key[len(IMAGE_PREFIX):] if key.startswith(IMAGE_PREFIX) else key
            img = images.get(name, images.get(key))
            if img is None:
                raise KeyError(f"missing camera '{name}' (policy expects {list(spec['cameras'])}, got {list(images)})")
            batch[key] = self._image_to_tensor(key, img, hw)
        if task is not None:
            batch["task"] = task
        return batch

    def act(self, state: np.ndarray, images: dict[str, np.ndarray], task: str | None = None) -> np.ndarray:
        """One action (A,) float32; runs the network only when the policy's action queue is empty."""
        torch = self._torch
        batch = self.preprocessor(self.build_batch(state, images, task))
        with torch.inference_mode():
            action = self.policy.select_action(batch)
        action = self.postprocessor(action)
        return action.detach().cpu().numpy().reshape(-1).astype(np.float32)

    def act_chunk(self, state: np.ndarray, images: dict[str, np.ndarray], task: str | None = None) -> np.ndarray:
        """Always run the network; return the full predicted chunk (T, A) float32, unnormalized."""
        torch = self._torch
        batch = self.preprocessor(self.build_batch(state, images, task))
        with torch.inference_mode():
            chunk = self.policy.predict_action_chunk(batch)  # (1, T, A) normalized
        chunk = self.postprocessor(chunk)
        arr = chunk.detach().cpu().numpy().astype(np.float32)
        return arr.reshape(-1, arr.shape[-1]) if arr.ndim == 3 else arr

    def timed_act(self, state: np.ndarray, images: dict[str, np.ndarray], task: str | None = None) -> tuple[np.ndarray, float]:
        """`act` plus wall-clock seconds, synchronizing the accelerator so the number is honest."""
        torch = self._torch
        t0 = time.perf_counter()
        out = self.act(state, images, task)
        if self.device == "mps":
            torch.mps.synchronize()
        elif self.device == "cuda":
            torch.cuda.synchronize()
        return out, time.perf_counter() - t0


def _smoke(argv: list[str] | None = None) -> int:
    """`python -m farm.learning.infer <checkpoint> [--device mps]`: load, print spec, run one random observation."""
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("checkpoint")
    ap.add_argument("--device", default="mps")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    runner = PolicyRunner(a.checkpoint, device=a.device)
    spec = runner.input_spec()
    print("input_spec:", spec)
    rng = np.random.default_rng(0)
    imgs = {name: rng.integers(0, 255, size=(480, 640, 3), dtype=np.uint8) for name in spec["camera_names"]}
    state = rng.standard_normal(spec["state_dim"]).astype(np.float32)
    runner.reset()
    act, dt = runner.timed_act(state, imgs)
    print(f"action shape {act.shape} first-call {dt*1000:.1f} ms:", np.round(act, 2))
    act2, dt2 = runner.timed_act(state, imgs)
    print(f"queued call {dt2*1000:.2f} ms")
    chunk = runner.act_chunk(state, imgs)
    print("chunk shape", chunk.shape)
    return 0


if __name__ == "__main__":
    raise SystemExit(_smoke())
