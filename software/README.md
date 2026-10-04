# farm — the XLeRobot plant-care program

## R3.2 planter assembly preparation

See [the connected-Mac handoff](docs/r32-handoff.md) first, then
[training instructions](docs/r32-training.md). The current open-front planter
parts and offline training tools are included; no R3.2 policy has been trained
and the live robot adapter remains to be integrated.


A Python program on the Mac that turns the XLeRobot kit (two SO-101 arms, head, three cameras) into a supervised plant caretaker for paper-grown cress in fixed trays. It stands on Hugging Face LeRobot 0.6 and XLeRobot's own robot class and IK solver (vendored under `farm/vendor/`, Apache-2.0).

Nobody drives the robot. Motion is taught by a vision model (the LLM-servo loop), decisions are made by rules → Jev → a person only when needed, and every action is written down before, during and after it happens.

```
farm/
  status.py            one status vocabulary: OK · STALE · INVALID · NOT_APPLICABLE · UNKNOWN
  config.py            profiles/*.yaml → dataclasses; data/overrides.yaml overlay (Astra apply-safe)
  adapters/            robot (LeRobot), cameras (OpenCV), ESP32 light (serial JSON), human (web); sim fakes
  skills/              arm model + IK, keyframes, the eight skills, LLM-servo (self-teaching)
  perception/          OpenCV frame quality / green fraction; VLM typed judgements (each has UNKNOWN)
  cycle/               state machine, care-cycle runner, authority ladder (Jev earns approval rights)
  evidence/            SQLite + images by hash; INTENT → ATTEMPT → RESULT; crash ⇒ UNKNOWN
  llm/                 backends (Claude CLI on the subscription; OpenRouter for Jev/Astra/vision), Jev, Astra
  viewer/              local web page: state, frames, questions, reconciliation, authority, proposals
  safety/              joint clamps, step limits, temperature/load, watchdog, pour bounds (code, not config)
firmware/esp32_light/  BH1750 → JSON lines at 5 Hz
profiles/              paper-tray-v0.yaml (real), sim.yaml (fakes)
parts/                 printable nest plate, tag tiles, light paddle, bottle rest (AnkerMake M5C)
assembly/            R2a planter-assembly contract (disabled): frames, stage machine, evidence, dataset schema, split
  tests/                 109 tests: cycle paths, UNKNOWN delivery, authority ladder, head servo, e-stop, Telegram, recorder, bus probe, motors-only self-test, calibration report, auto-calibration wrapper, remote policy, soak, MCP tools, carton task
```

## Setup (MacBook)

```sh
cd software
uv venv --python 3.12 .venv && . .venv/bin/activate
uv pip install -e .            # lerobot[feetech], opencv, pyserial, fastapi, httpx …
cp .env.example .env           # OPENROUTER_API_KEY=… (Jev/Astra). Claude vision uses the logged-in `claude` CLI.
python -m pytest -q            # 109 passed
farm sim --auto-answer         # whole program on fakes; viewer at http://localhost:8765
```

macOS asks for camera permission the first time a Terminal process opens a camera; allow it.

## Backends

| Role | Default | Fallback |
|---|---|---|
| Vision judgements + LLM-servo | `claude` CLI on your subscription (`llm.backend: claude-cli`) | OpenRouter `anthropic/claude-sonnet-5` |
| Jev (typed choices) | OpenRouter `typesafe/jev-router` | — |
| Astra (daily proposals) | OpenRouter `openai/gpt-6-astra` | — |

`llm.max_cost_usd_per_day` caps OpenRouter spend; when it is hit, rules refuse to pour.

## Japan, day 1 (robot assembled, wall-powered, base parked)

1. `farm devices --probe` — pings each motor bus (IDs 1–8 = bus 1, 9–10 = bus 2) and snapshots every camera into `data/devices/`; put the ports and camera indices in `profiles/paper-tray-v0.yaml`.
2. `farm calibrate` — LeRobot's one-time range-of-motion calibration (support the arms; this is setup, not operation). Then `farm robot-test --move --ask` — motors only, no cameras or models: reads every joint and nudges each one in turn so you can confirm the named joint is the one that moves.
3. `farm check` — connects everything, asks the vision model which camera is which, prints statuses.
4. Put the bottle in its rest, the paddle in its rest, the cress tray in its nest. `farm teach-all` — the LLM-servo learns every keyframe the profile needs (bottle rest, paddle rest, look/pour/measure per tray) and saves them to `data/keyframes.yaml`. Re-run any single one with `farm teach --arm right --goal "…" --save pour_B`.
5. Empty-bottle rehearsal: `farm once --tray B` with an empty bottle; authorize from the viewer.
6. Cup test: `farm cup-test --tilt 25 --seconds 1.5 --who you` pours into a kitchen measure and records the millilitres you read.
7. `farm run --every 3600 [--record]` — cycles every tray hourly, rests the arms torque-off between cycles, reconnects on USB drops; the viewer is at `http://<mac-ip>:8765` from your phone (big red STOP holds every motor). `--record` writes the robot's own runs to `data/dataset` as a LeRobotDataset.
   Questions also go to Telegram when `TELEGRAM_BOT_TOKEN` / `TELEGRAM_CHAT_IDS` are in `.env` (first answer wins).
8. `farm review` once a day — Astra proposes one change; accept/reject in the viewer (or `authority.astra: apply-safe` for bounded numeric changes).

## Learning (v2)

- `farm run --record` / `farm once --record` write the robot's own runs to `data/dataset` (LeRobotDataset v3, `.pos` joint names, three cameras).
- `python -m farm.learning.train --policy act|smolvla --dataset-root data/dataset --steps N --device mps` wraps `lerobot-train`; `python -m farm.learning.evaluate` reports per-joint MAE on held-out episodes; `farm policy-test --checkpoint DIR [--real]` runs a checkpoint as a skill under the same clamps as everything else.
- First run (2026-09-27, pipeline check): ACT on `SurajCreation/so101_pour_v1` (35 SO-101 bottle-pour episodes, wrist + overhead), 5000 steps on Mac MPS in ~70 min, loss 3.28 → 0.42; offline first-action MAE 5.5 (normalized ±100 units) on the last 3 episodes vs 2.0 for a hold-still baseline — i.e. a motion prior, **not** a deployable skill; inference 77 ms per 100-action chunk on MPS. The checkpoint drove the simulator through `farm policy-test` under the clamps.
- Zero-shot plumbing candidate on the real robot: `lissajous/xlerobot-act-local-grasp-v1` (XLeRobot right arm, 6-dim state, one wrist camera — the same interface as the `policy:` profile defaults; a glue-stick grasp, so only for testing the wrapper). Public datasets without a Hub version tag must be `snapshot_download`ed and loaded with `--dataset.root` (documented in `farm/learning/train.py`).
- `docs/research-training.md`: survey of 57 datasets / 16 checkpoints on the Hub, literature, footage, and the recommendation (ACT locally, SmolVLA on a rented GPU; same-arm SO-101 pour data as a prior; 60–100 of our own episodes).

## Added from the community review (2026-10-02)

| Command | What it does |
|---|---|
| `farm calibration-report` | Reads the saved calibration and flags a wrapped reading, a short sweep, or arms that disagree. No motion. |
| `farm robot-test [--move --ask]` | Motors only: reads every joint; with `--move` nudges each one and asks whether the named part moved. |
| `farm soak [--keyframe pour_B] [--minutes 20]` | Holds a pose, logs every servo's temperature and load to `data/soak/`, stops at the temperature ceiling. |
| `farm policy-server --checkpoint DIR` | Parked. On a second machine: serves a checkpoint over HTTP; `farm policy-test --server URL` uses it. See `docs/gpu-server.md`. |
| `farm calibrate --auto --arm left` (then `right`), `farm calibrate --head` | Automatic calibration: each arm finds its own limits (LeRobot PR #3282, vendored in `farm/vendor/autocal`); the head is a two-joint hands-on step. **Never run on this robot.** Staged modes: `--motor gripper`, `--unfold-only`. Procedure in the `farm-bringup` skill. |
| `farm r2a [--checkpoint DIR]` | R2a planter-assembly readiness: parts hashes, station transform, grip thresholds, dataset/checkpoint schema. Files only. Contract in `farm/assembly/`, plan in `docs/r2a-assembly.md`. Execution disabled. |
| `farm mcp` | Stdio MCP server: state, camera frames and three skill-level actions (stop, go_rest, go_keyframe). No raw joint access. It holds the robot's serial ports while it runs. |

To give Claude Code on the robot laptop the MCP tools: `claude mcp add farm -- "$PWD/.venv/bin/farm" mcp -p paper-tray-v0`, run from `software/`. Remove it or stop it before running other `farm` commands, since only one process can hold the motor boards.

The review itself is in `docs/community-projects.md`.

## What can never happen

- Water moves without rules passing **and** one of: a named person in the viewer, or Jev at `approve` level with p ≥ 0.85 after earning it.
- A pour whose result is UNKNOWN is retried. It blocks every cycle until a person reconciles it.
- A model relaxes a safety limit, drives the wheels, or writes a joint command directly.
- A disabled device is read as a value. NOT_APPLICABLE ≠ dry.

## Authority ladder (`authority.jev`)

shadow → route → approve. Starts at shadow; promotes itself after `jev_shadow_cycles` agreeing cycles; demotes on a false approval (spill at reconciliation). Set from the viewer at any time.

## Firmware

`farm light-monitor` streams the ESP32 readings once the board is flashed.

`firmware/esp32_light/esp32_light.ino` — Arduino IDE, board "ESP32 Dev Module", 115200 baud. BH1750 on 3V3/GND/SDA=GPIO21/SCL=GPIO22 (verify the delivered ELEGOO board's labels). `farm check` shows the live lux stream.
