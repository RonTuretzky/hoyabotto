"""Thin wrapper around `lerobot-train` (lerobot 0.6.1) for ACT / SmolVLA on our own or a Hub dataset.

Exact command this wrapper reproduced for the reference run (2026-09-27, Apple Silicon, mps):

    cd software && . .venv/bin/activate
    export HF_LEROBOT_HOME=$PWD/data-train/lerobot_home WANDB_MODE=disabled
    lerobot-train \
      --dataset.repo_id=SurajCreation/so101_pour_v1 \
      --dataset.root=data-train/datasets/SurajCreation__so101_pour_v1 \
      --policy.type=act --policy.device=mps --policy.push_to_hub=false \
      --output_dir=data-train/act_so101_pour --job_name=act_so101_pour \
      --batch_size=8 --steps=5000 --log_freq=100 --save_freq=1000 \
      --env_eval_freq=0 --num_workers=2 --wandb.enable=false

Equivalent through this module:

    python -m farm.learning.train --repo-id SurajCreation/so101_pour_v1 \
        --root data-train/datasets/SurajCreation__so101_pour_v1 \
        --policy act --steps 5000 --device mps --output-dir data-train/act_so101_pour

SmolVLA fine-tune from the public base (needs `pip install 'lerobot[smolvla]'`):

    python -m farm.learning.train --repo-id <ours> --root data-train/datasets/<ours> \
        --policy smolvla --pretrained lerobot/smolvla_base --steps 20000 --device mps \
        --output-dir data-train/smolvla_ours

Facts about 0.6.1 that shaped the flags (all checked against the installed sources):
  * `--dataset.root=<dir>` points at a concrete local LeRobotDataset tree; when the tree already
    holds meta/info.json the loader never contacts the Hub, which also sidesteps the
    "dataset must be tagged with a codebase version" error some Hub datasets raise.
    Hub datasets without the tag: `huggingface_hub.snapshot_download(repo, repo_type="dataset",
    local_dir=<root>)` first, then train with `--dataset.root`.
  * `--policy.type=act` builds a fresh ACT from the dataset features; a pretrained policy is
    loaded with `--policy.path=<repo or dir>` INSTEAD of `--policy.type` (draccus accepts one).
  * `--policy.push_to_hub=false` is required, the default is True and would need a repo_id.
  * `--env_eval_freq=0` disables sim evaluation (no gym env here); `--eval_steps` stays 0.
  * wandb is off via `--wandb.enable=false` (plus WANDB_MODE=disabled in the env for safety).
  * `--output_dir` must not exist unless `--resume=true`; checkpoints land in
    `<output_dir>/checkpoints/<step>/pretrained_model` with a `checkpoints/last` symlink.
  * `--rename_map='{"observation.images.cam1": "observation.images.wrist"}'` maps our camera
    keys onto a pretrained policy's expected keys.
  * MPS: works for ACT out of the box. If an op is unsupported set
    PYTORCH_ENABLE_MPS_FALLBACK=1 (this wrapper does so by default).
  * The log line every `log_freq` steps looks like
    `step:100 smpl:800 ep:1 epch:0.03 loss:10.325 grdn:214.653 lr:1.0e-05 updt_s:0.912 data_s:0.035 ...`;
    `step:` is rounded to K past 1000, so `parse_loss_log` numbers rows by `log_freq`.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

SOFTWARE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_DATA_TRAIN = SOFTWARE_DIR / "data-train"
SMOLVLA_BASE = "lerobot/smolvla_base"

_LOSS_RE = re.compile(r"step:(?P<step>[0-9.]+[KMB]?) .*?loss:(?P<loss>[0-9.]+).*?updt_s:(?P<updt>[0-9.]+) data_s:(?P<data>[0-9.]+)")
_SUB_RE = re.compile(r"(l1_loss|kld_loss|grdn|lr):([0-9.e+-]+)")


def build_command(
    repo_id: str,
    policy: str = "act",
    steps: int = 5000,
    device: str = "mps",
    output_dir: str | Path = DEFAULT_DATA_TRAIN / "run",
    root: str | Path | None = None,
    pretrained: str | None = None,
    batch_size: int = 8,
    save_freq: int = 1000,
    log_freq: int = 100,
    num_workers: int = 2,
    job_name: str | None = None,
    rename_map: dict[str, str] | None = None,
    resume: bool = False,
    extra: list[str] | None = None,
) -> list[str]:
    """Return the argv for lerobot-train. `pretrained` swaps `--policy.type` for `--policy.path`."""
    if policy not in ("act", "smolvla"):
        raise ValueError("policy must be 'act' or 'smolvla'")
    if policy == "smolvla" and pretrained is None:
        pretrained = SMOLVLA_BASE  # SmolVLA from scratch is not what anyone wants on 35 episodes
    cmd = ["lerobot-train", f"--dataset.repo_id={repo_id}"]
    if root is not None:
        cmd.append(f"--dataset.root={root}")
    if pretrained:
        cmd.append(f"--policy.path={pretrained}")
    else:
        cmd.append(f"--policy.type={policy}")
    cmd += [
        f"--policy.device={device}",
        "--policy.push_to_hub=false",
        f"--output_dir={output_dir}",
        f"--job_name={job_name or Path(output_dir).name}",
        f"--batch_size={batch_size}",
        f"--steps={steps}",
        f"--log_freq={log_freq}",
        f"--save_freq={save_freq}",
        "--env_eval_freq=0",
        f"--num_workers={num_workers}",
        "--wandb.enable=false",
    ]
    if rename_map:
        cmd.append(f"--rename_map={json.dumps(rename_map)}")
    if resume:
        cmd += ["--resume=true", f"--config_path={Path(output_dir) / 'checkpoints' / 'last' / 'pretrained_model' / 'train_config.json'}"]
    if extra:
        cmd += list(extra)
    return cmd


def training_env(data_train: Path = DEFAULT_DATA_TRAIN) -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault("HF_LEROBOT_HOME", str(data_train / "lerobot_home"))
    env["WANDB_MODE"] = "disabled"
    env.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
    env.setdefault("TOKENIZERS_PARALLELISM", "false")
    return env


def parse_loss_log(log_path: Path, csv_path: Path, log_freq: int = 100) -> list[dict]:
    """Extract the per-`log_freq` metrics lines from a lerobot-train log into loss.csv."""
    rows = []
    for line in Path(log_path).read_text(errors="replace").splitlines():
        m = _LOSS_RE.search(line)
        if not m:
            continue
        subs = dict(_SUB_RE.findall(line))
        n = len(rows) + 1
        rows.append({
            "step": n * log_freq,
            "step_logged": m.group("step"),
            "loss": float(m.group("loss")),
            "l1_loss": float(subs["l1_loss"]) if "l1_loss" in subs else "",
            "kld_loss": float(subs["kld_loss"]) if "kld_loss" in subs else "",
            "grad_norm": float(subs["grdn"]) if "grdn" in subs else "",
            "lr": float(subs["lr"]) if "lr" in subs else "",
            "update_s": float(m.group("updt")),
            "data_s": float(m.group("data")),
            "steps_per_s": round(1.0 / max(float(m.group("updt")) + float(m.group("data")), 1e-9), 3),
        })
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        with csv_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    return rows


def run(cmd: list[str], log_path: Path, env: dict[str, str] | None = None, cwd: Path = SOFTWARE_DIR) -> int:
    """Run lerobot-train, teeing stdout/stderr to `log_path`; returns the exit code."""
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print("$", " ".join(shlex.quote(c) for c in cmd), flush=True)
    t0 = time.time()
    with log_path.open("w") as log, subprocess.Popen(
        cmd, cwd=str(cwd), env=env or training_env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    ) as p:
        assert p.stdout is not None
        for line in p.stdout:
            log.write(line)
            if "loss:" in line or "Checkpoint" in line or "Error" in line or "End of training" in line:
                sys.stdout.write(line if line.endswith("\n") else line + "\n")
                sys.stdout.flush()
        rc = p.wait()
    print(f"lerobot-train exited {rc} after {(time.time()-t0)/60:.1f} min; log at {log_path}", flush=True)
    return rc


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Train ACT or SmolVLA with lerobot-train on a LeRobotDataset.")
    ap.add_argument("--repo-id", required=True, help="dataset repo id (also the name under --root)")
    ap.add_argument("--root", help="local LeRobotDataset dir (our recorder output or a snapshot_download)")
    ap.add_argument("--policy", choices=["act", "smolvla"], default="act")
    ap.add_argument("--pretrained", help=f"policy path/repo to fine-tune; smolvla defaults to {SMOLVLA_BASE}")
    ap.add_argument("--steps", type=int, default=5000)
    ap.add_argument("--device", default="mps", choices=["mps", "cpu", "cuda"])
    ap.add_argument("--output-dir", default=str(DEFAULT_DATA_TRAIN / "run"))
    ap.add_argument("--batch-size", type=int, default=8)
    ap.add_argument("--save-freq", type=int, default=1000)
    ap.add_argument("--log-freq", type=int, default=100)
    ap.add_argument("--num-workers", type=int, default=2)
    ap.add_argument("--rename-map", help='JSON, e.g. {"observation.images.cam1":"observation.images.wrist"}')
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print the command and exit")
    ap.add_argument("--parse-log", help="skip training; parse this existing log into <output-dir>/loss.csv")
    ap.add_argument("extra", nargs="*", help="passed through to lerobot-train (after --)")
    a = ap.parse_args(argv)

    out = Path(a.output_dir)
    if a.parse_log:
        rows = parse_loss_log(Path(a.parse_log), out / "loss.csv", a.log_freq)
        print(f"wrote {len(rows)} rows to {out / 'loss.csv'}")
        return 0
    cmd = build_command(
        a.repo_id, a.policy, a.steps, a.device, out, a.root, a.pretrained, a.batch_size, a.save_freq, a.log_freq,
        a.num_workers, None, json.loads(a.rename_map) if a.rename_map else None, a.resume, a.extra,
    )
    if a.dry_run:
        print(" ".join(shlex.quote(c) for c in cmd))
        return 0
    # lerobot-train refuses an existing output_dir, so the live log sits beside it and is moved in afterwards.
    side_log = out.parent / f"{out.name}_train.log"
    rc = run(cmd, side_log)
    if out.is_dir():
        final_log = out / "train.log"
        side_log.replace(final_log)
        rows = parse_loss_log(final_log, out / "loss.csv", a.log_freq)
        print(f"loss.csv: {len(rows)} rows; last loss {rows[-1]['loss'] if rows else 'n/a'}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
