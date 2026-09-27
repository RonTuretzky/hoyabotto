"""Offline check of a trained policy against recorded episodes (lerobot 0.6.1).

    python -m farm.learning.evaluate --checkpoint data-train/act_so101_pour \
        --repo-id SurajCreation/so101_pour_v1 --root data-train/datasets/SurajCreation__so101_pour_v1 \
        --episodes 3 --stride 5 --device mps --out data-train/act_so101_pour/eval.json

For each sampled frame of the last N episodes the policy sees the recorded state and camera
images exactly as `PolicyRunner.act` would on the robot (HxWx3 uint8 RGB), predicts a fresh
action chunk, and the FIRST action of that chunk is compared with the action the operator
actually sent at that frame. Reported:

  * mean absolute error per action dimension (dataset units, here degrees / gripper %)
  * the same MAE for the trivial predictor "action = current state", as a floor to beat
  * per-call latency of the full network (predict_action_chunk) and of `select_action`,
    which only runs the network once per `n_action_steps` calls

Honesty note: the episodes were part of the training set unless the run used
`--dataset.episodes`, and an offline MAE says nothing about closed-loop success on a robot.
It proves the checkpoint loads, consumes real observations, and tracks the demonstrations;
it is not a success rate.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import statistics
import time
from pathlib import Path

import numpy as np

from farm.learning.infer import IMAGE_PREFIX, OBS_STATE, PolicyRunner

log = logging.getLogger(__name__)


def load_dataset(repo_id: str, root: str | None):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if root:
        os.environ.setdefault("HF_HUB_OFFLINE", "1")  # local tree: never touch the Hub
    return LeRobotDataset(repo_id, root=root)


def frame_to_obs(item: dict, camera_keys: list[str]) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
    """Dataset item (CHW float [0,1] images) -> robot-style obs (HxWx3 uint8) + recorded action."""
    state = item[OBS_STATE].numpy().astype(np.float32)
    images = {}
    for key in camera_keys:
        img = item[key]
        if img.dtype != np.uint8 and img.dtype.__class__.__name__ != "uint8":
            arr = (img.permute(1, 2, 0).numpy() * 255.0).round().clip(0, 255).astype(np.uint8)
        else:
            arr = img.permute(1, 2, 0).numpy()
        images[key[len(IMAGE_PREFIX):]] = arr
    return state, images, item["action"].numpy().astype(np.float32)


def evaluate(checkpoint: str, repo_id: str, root: str | None, episodes: int = 3, stride: int = 5, device: str = "mps",
             max_frames: int | None = None) -> dict:
    ds = load_dataset(repo_id, root)
    meta = ds.meta
    runner = PolicyRunner(checkpoint, device=device)
    spec = runner.input_spec()
    action_names = meta.features["action"].get("names") or [f"a{i}" for i in range(spec["action_dim"])]
    if isinstance(action_names, dict):
        action_names = list(action_names.values())[0]
    cam_keys = [k for k in meta.camera_keys if k in spec["cameras"]]
    missing = [k for k in spec["cameras"] if k not in meta.camera_keys]
    if missing:
        raise KeyError(f"policy expects cameras {missing} that the dataset does not have ({meta.camera_keys})")

    ep_ids = list(range(meta.total_episodes - episodes, meta.total_episodes))
    abs_err, abs_err_state, chunk_ms, select_ms = [], [], [], []
    n_frames = 0
    t_all = time.perf_counter()
    for ep in ep_ids:
        row = meta.episodes[ep]
        start, end = int(row["dataset_from_index"]), int(row["dataset_to_index"])
        runner.reset()
        for idx in range(start, end, stride):
            item = ds[idx]
            state, images, action = frame_to_obs(item, cam_keys)
            # full network every frame -> first action of a fresh chunk
            t0 = time.perf_counter()
            chunk = runner.act_chunk(state, images)
            runner._torch.mps.synchronize() if runner.device == "mps" else None
            chunk_ms.append((time.perf_counter() - t0) * 1000)
            pred = chunk[0]
            # queue-backed select_action as the robot loop would call it
            _, dt = runner.timed_act(state, images)
            select_ms.append(dt * 1000)
            abs_err.append(np.abs(pred - action))
            abs_err_state.append(np.abs(state[: len(action)] - action))
            n_frames += 1
            if max_frames and n_frames >= max_frames:
                break
        if max_frames and n_frames >= max_frames:
            break
    abs_err = np.stack(abs_err)
    abs_err_state = np.stack(abs_err_state)
    mae = abs_err.mean(axis=0)
    mae_state = abs_err_state.mean(axis=0)
    result = {
        "checkpoint": runner.path,
        "policy_type": spec["policy_type"],
        "device": runner.device,
        "repo_id": repo_id,
        "root": root,
        "episodes": ep_ids,
        "stride": stride,
        "n_frames": int(n_frames),
        "held_out": False,
        "note": "episodes were in the training set; offline MAE is not a success rate",
        "action_names": list(action_names),
        "mae_per_dim": {n: round(float(v), 3) for n, v in zip(action_names, mae)},
        "mae_mean": round(float(mae.mean()), 3),
        "baseline_state_equals_action_mae_per_dim": {n: round(float(v), 3) for n, v in zip(action_names, mae_state)},
        "baseline_state_equals_action_mae_mean": round(float(mae_state.mean()), 3),
        "mae_p90_per_dim": {n: round(float(v), 3) for n, v in zip(action_names, np.percentile(abs_err, 90, axis=0))},
        "latency_ms": {
            "predict_action_chunk_mean": round(statistics.fmean(chunk_ms), 2),
            "predict_action_chunk_p50": round(statistics.median(chunk_ms), 2),
            "predict_action_chunk_max": round(max(chunk_ms), 2),
            "select_action_mean": round(statistics.fmean(select_ms), 2),
            "select_action_p50": round(statistics.median(select_ms), 2),
            "select_action_max": round(max(select_ms), 2),
            "model_load_s": round(runner.load_s, 2),
        },
        "input_spec": {k: (v if not isinstance(v, dict) else {kk: list(vv) if isinstance(vv, tuple) else vv for kk, vv in v.items()}) for k, v in spec.items()},
        "wall_s": round(time.perf_counter() - t_all, 1),
    }
    return result


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--root", help="local dataset dir (skips the Hub)")
    ap.add_argument("--episodes", type=int, default=3, help="evaluate the LAST N episodes")
    ap.add_argument("--stride", type=int, default=5, help="use every k-th frame")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--out", help="write JSON here (default <checkpoint>/eval.json when it is a dir)")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    res = evaluate(a.checkpoint, a.repo_id, a.root, a.episodes, a.stride, a.device, a.max_frames)
    out = Path(a.out) if a.out else (Path(a.checkpoint) / "eval.json" if Path(a.checkpoint).is_dir() else Path("eval.json"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=2))
    print(f"{'dim':18} {'MAE':>8} {'p90':>8} {'state=action MAE':>18}")
    for n in res["action_names"]:
        print(f"{n:18} {res['mae_per_dim'][n]:8.3f} {res['mae_p90_per_dim'][n]:8.3f} {res['baseline_state_equals_action_mae_per_dim'][n]:18.3f}")
    print(f"{'mean':18} {res['mae_mean']:8.3f} {'':8} {res['baseline_state_equals_action_mae_mean']:18.3f}")
    print("frames", res["n_frames"], "episodes", res["episodes"], "latency", res["latency_ms"])
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
