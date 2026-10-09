# Fold policy from the chat Mac: handoff — 9 October 2026

For whoever connects the learned short-flap fold policy to the chat. "Chat Mac" is the laptop that runs the
pilot chat (`chat_server.py`) and talks to the robot Mac over mTLS. This page has four parts: what exists, what this
branch added, what the simulation says about the robot path, and the ordered list of what is still needed.
**Nothing here has moved a motor.** The safety rules in `CLAUDE.md` apply throughout: STOP and the 12 V switch within
reach, nobody in the arms' sweep, never relax limits.

## 1. Where things stand

| Item | State |
|---|---|
| Policy | ACT, trained only in simulation at the robot-model station (bases 220 mm apart); folds both short flaps in **63/63** unseen simulated starts. Hub `RonTuretzky/act_carton_both_shorts_220_bs32_lr3e5_chunk100`, `checkpoints/015000` |
| Inputs / outputs | `front` (head OAK), `left_wrist`, `right_wrist` at 240×320 (4:3), 12 joints in URDF radians; 12 absolute joint targets at 10 Hz, temporal ensembling 0.01 |
| Robot runner | `carton/fold_policy_runner.py`: reads the owner and cameras through the robot API, converts ticks to radians with the joint maps, runs the policy, clamps and sends. Dry-run unless `--execute` |
| Chat tools | **New:** `carton/fold_policy_chat.py` + `tools/install_fold_policy_chat.py` (section 2.4). Not installed yet |
| Owner (robot Mac) | Deployed owner is `paddle-success-v1`. It **cannot run the policy as is**: jaw closures cannot be streamed, a refused streamed target releases every motor, and held joints fault at 96 ticks of drift (section 3) |
| Owner stream mode | **New, not deployed:** always on once installed (no flag), unused until a client streams; see section 2.5 |
| Joint maps | Arms: owner-accepted twin mapping (zero tick 2047, sign +1). Jaws: **corrected on this branch** (section 2.2) |
| Head camera | Pose unknown (the twin's head mapping is off ~15° tilt / 17° pan). The OAK now streams **16:9** 1080p; the policy was trained on a **4:3** head camera (section 4, step B) |

The chat Mac side, as found on 9 October (read only):

- Pilot: `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/` (not in git). Started with
  `../.venv/bin/python start.py`; the chat listens on 127.0.0.1:1241 and on the LAN at 8443.
- Its Python (`gemma-xlerobot/.venv`, 3.12) has torch 2.11, lerobot 0.6.1, mujoco 3.14 and opencv, so it can run the
  policy. Its `farm` package comes from `gemma-xlerobot/farm-live` (a worktree detached at an older commit,
  `dbf55b8`, without the fold runner). The chat tools therefore run the runner from this checkout's `software/`
  with `PYTHONPATH` set; nothing in farm-live is changed.
- Tools reach the chat through wrapper classes around the pilot's `Robot`:
  `chat=Chat(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config)))))`.
- **Supervisor mode is on** (`.private/orchestration.json` `enabled: true`). In that mode the model returns JSON
  decisions; `move` decisions may only name the tools in `MOVE_TOOLS`, and the supervisor prompt lists their
  argument names but not their descriptions.
- The chat's STOP (`#stop` buttons, `POST /api/stop`, a dropped client stream, SIGTERM) calls `robot_stop` through
  the wrapper chain.

## 2. What this branch adds (review these)

### 2.1 The runner reads the demo model's cameras

`CAMERA_KEYS` was hard-wired to the first policy's simulated `top`/`front`; the demo checkpoint reads `front`,
`left_wrist` and `right_wrist`, so every real run would have refused.

- The runner now takes the camera keys from the camera source and checks them against the checkpoint.
- On the robot, the camera mapping defaults to `front=oak left_wrist=left_wrist right_wrist=right_wrist`.
- Preflight warns `camera_aspect: …` when a camera's aspect ratio differs from the checkpoint's by more than 2%. The
  policy squeezes every frame to 240×320, so a 16:9 frame is a different field of view, not a resize.
- The simulation fakes render the scene's own cameras (the 220 mm scenes have them) and publish wrist frames through
  the real `robot_get_cameras` code path.

### 2.2 Jaw map corrected

The simulation's closed jaw is URDF −10°, its joint limit, where the pads meet.

- **Measured on the left jaw (9 Oct, STATUS.md):** empty closes stop at **1355–1359**, which is saved range_min
  (1273) + 82. The twin (`farm/sim/sim_robot.py`, `GRIPPER_CLOSED_OFFSET_TICKS`) uses the same offset.
- **Old maps (8 Oct):** they put −10° at range_min itself, about 7° too far closed. Two effects:
  - The right jaw reads constant in training, and the runner aborts beyond 5° of that, so every run would have
    aborted.
  - Closures would have been commanded into the pads.
- **New gripper zero (URDF 0°):** `range_min + 82 + 113.8`, which is **1468.8** on the left and **1464.8** on the
  right. The right meeting point is **assumed** to be the same (not measured).
- The training's closed jaw (−9.7°) now lands at tick ≈1358.
- Test: `test_released_joint_maps_put_the_closed_jaw_where_the_pads_meet`.

### 2.3 Runner additions

- `--gripper-mode stream`: a closing jaw moves at most 10 ticks per tick. It is refused unless the transport streams
  jaw closures (the owner's stream mode, or the simulation's `sim-direct`).
- `--parent-pid`: the runner stops (halts, holding) if its parent exits, e.g. the chat server is killed. SIGTERM now
  stops it the same way as SIGINT.

### 2.4 Chat tools: `carton/fold_policy_chat.py`

`FoldPolicyRobot` wraps the pilot's robot chain the same way `CalibrationRobot` does. It uses the standard library
only and is loaded by path, so the chat's older `farm` is not shadowed. It adds four tools:

| Tool | Does | Motors |
|---|---|---|
| `robot_get_fold_policy_status` | Configuration, blockers, running job, last dry run and run | none |
| `robot_fold_policy_dry_run {max_steps ≤ cap}` | Runs the runner **without** `--execute`: reads the owner and the three cameras at 10 Hz, runs the policy, logs what it would send | none |
| `robot_fold_policy_run {operator, max_steps}` | Runs the runner **with** `--execute`, `--operator` | streams targets; ends **holding** |
| `robot_fold_policy_start_pose {execute?, operator?}` | Runs `tools/move_to_start_pose.py`: plans a collision-checked path to the training start pose; with `execute` moves the enabled arms there leg by leg ([auto-start-pose.md](auto-start-pose.md)). Own gates: `start_pose_execute_enabled`, `operators`, `start_pose_blockers`, `start_pose_scene` | none (dry run) / blocking moves; ends **holding** |

Gates for `robot_fold_policy_run`. They come from the owner's config, never from the tool arguments; all must hold:

- `execute_enabled: true` in `<pilot>/.private/fold-policy.json`;
- no remaining `blockers` listed there;
- the operator is listed in `operators`;
- `max_steps` is within `execute_max_steps` (default **10**, i.e. 1 s);
- a **clean** dry run (no abort, no warnings) of the **same** checkpoint weights, joint-map files and camera mapping
  finished within `dry_run_valid_s` (default 15 min).

While a job runs:

- It is a subprocess of the chat server using the configured Python, with `--parent-pid`.
- The tool call blocks until the job ends.
- The shared local motion lock (`.private/tag-calibration.lock`, the same file the tag calibration takes) is held.
- Every other motion tool through the wrapper is refused with `ok: false`; reads pass.
- **STOP:** `robot_stop` first SIGINTs the runner (it halts, holds, and releases nothing), then goes to the robot as
  always, and the owner releases every motor.
- Runs and logs go to `runs_dir` (default `gemma-xlerobot/fold-policy-runs/`). The last dry run and run are kept in
  `.private/fold-policy-state.json`.

The installer is `tools/install_fold_policy_chat.py`. It patches three lines of `chat_server.py`:

- the loader;
- `chat=Chat(FoldPolicyRobot(...))`;
- `MOVE_TOOLS` gains the dry-run, run and start-pose tools, so supervisor `move` decisions can use them (an
  install from before the start-pose tool is upgraded in place by running the installer again).

It also backs up the original to `.private/fold-policy-backups/` and writes the config with **execution disabled**
and two default blockers. `--uninstall` reverses the patch. It never restarts the chat. The patch was checked against
a copy of the live `chat_server.py` and reverses exactly.

Tests: `tests/test_fold_policy_chat.py` (8). The runner is a stand-in subprocess. The tests cover:

- every gate;
- the clean or unclean dry-run rule;
- identity changes;
- staleness;
- refusing motion and passing reads while a job runs;
- the shared lock;
- STOP interrupting the job;
- the installer's idempotent and reversible patch.

### 2.5 Owner stream mode (robot Mac)

The full rules are in `docs/commissioning/2026-10-07-paddle-success/qwen-bridge/STREAM-MODE.md`. There is **no flag**. It is on whenever the
owner runs the pickup profile, which is how the owner is deployed. It does nothing until a client sends one of its two
ops, and both tools are hidden from the chat model, so every existing tool behaves as before. What it adds:

- **`robot_stream_joint_targets {positions}`**: one call per tick names **both arms, jaws included**. The owner keeps
  one stream executor across commands. Its rules:
  - each goal ramps at most 40 ticks per owner loop;
  - out-of-range targets, or targets more than 96 ticks from present, reject the whole set;
  - joints within 2 ticks are **skipped**, not refused;
  - jaws change at most 10 ticks per command, closing too;
  - a closing jaw with load ≥ 250 or 40 ticks behind freezes where it is (`jaw_contact`) until it is opened;
  - arm contact (load ≥ 350, stalled, 50 ticks behind) is counted across commands and **holds** everything at
    present (`contact_halt`);
  - with no command for 0.5 s, the stream ends holding.
  - A refusal answers `accepted: false` and **never sends STOP**.
- **`robot_hold_here`**: every enabled joint holds at its present position, with no torque ramp-down and no release.
- **Unchanged:** the 96-tick drift fault, the load faults (800 arm, 500 jaw), the watchdogs and the camera gate. All
  still release everything.
- Both tools are hidden from the chat model (`PILOT_HIDDEN`); only the runner uses them.

The runner's side:

- `--transport api-stream` (`StreamOwnerTransport`) makes one `robot_stream_joint_targets` call per tick.
- `--gripper-mode stream` limits jaws to ±10 ticks per tick.
- It checks `stream_mode` in `robot_get_capabilities` at preflight.
- It halts with `robot_hold_here`.

The simulation fakes run the real owner with `stream=True` (`--transport sim-owner-stream`).

Tests: `docs/commissioning/2026-10-07-paddle-success/qwen-bridge/test_stream_targets.py`, added to the redeploy test list. All 24 redeploy test files pass. Runner tests
cover the stream transport against the real owner code: one call per tick, a jaw closure while the arms move, no
STOP, and a refusal when the owner lacks stream mode.

**Never run on hardware.** STREAM-MODE.md lists the free-air checks to do first.

## 3. What the simulation says about the robot path (9 October)

These runs use the demo checkpoint at 15k. The runner and all its bounds were used: 40-tick step, 3-tick rule, race
guard, and the ±5° training envelope. The policy ran at 10 Hz with 30 ms of virtual inference.

| Path | Trial | Result |
|---|---|---|
| `sim-owner`: deployed owner semantics, jaw closures not streamable | b02 trial-010 | **Fail.** At 39 s the owner faulted with `left_arm_elbow_flex: uncommanded holding drift` and **released all motors**; shorts at 1.5° / 12° |
| `sim-direct`: runner bounds only, jaw closures dropped (today's owner cannot stream them) | b02 trial-010 | **Fail.** Left short 95°, right short 25° (401 closures dropped) |
| `sim-direct` with `--gripper-mode stream` (closures ≤ 10 ticks per tick) | b02 trial-010 | **Both shorts held** at 59.9 s (94° / 92°), carton moved 6.7 mm |
| same | b01 trials 020 / 040 / 060 | **All three held** at 56.9 / 57.1 / 59.7 s (shorts 92–94°), carton moved 1.1 / 2.1 / 1.1 mm |
| same, plus 100 ms of inference delay (no owner) | b02-010, b01-040 | **Both held** (carton ≤ 4.5 mm): the policy tolerates latency |
| `sim-owner-stream`: the real owner code in stream mode, **50 ms owner loop** | the same four | **1/4.** No owner fault in any run, but three pushed the carton 64–73 mm and left a short flap open or pushed out |
| `sim-owner-stream`, **33 ms owner loop** | the same four | **4/4 held**, carton ≤ 3.9 mm, no owner fault |
| `sim-owner-stream`, **20 ms owner loop** | b02-010, b01-040 | **Both held**, carton ≤ 3.6 mm |
| `sim-direct` stream, **AprilTags removed from the box** (all 12 box/flap tags and the table tags invisible) | the same four | **2/4.** b01-060 pushed the left flap outward (−101°) and slid the carton 107 mm; b02-010 never folded the right flap |
| same, only the gripper tags kept | the same four | **2/4**: the right flap was not folded in b01-060 and b02-010 |

Reading:

- The tick conversion, the clamps and 10 Hz are compatible with a successful fold.
- What breaks it is the owner's current semantics: no streamed jaw closures, and the 96-tick holding-drift fault.
- With stream mode, the deciding number is the **owner loop period**. The fold succeeds at ≤ 33 ms and degrades at
  50 ms. The cause is not added latency: 100 ms of delay without the owner is fine. More likely it is the owner's
  40-ticks-per-loop goal ramp and its status rows being one loop old.
- The real owner polls every motor, then sleeps 20 ms. Its period has never been measured; the contract tests assume
  50 ms. It must be measured before any motion (step G). If it is above 33 ms, shorten the owner's sleep or the poll
  while streaming; that is an owner change.
- **The policy depends on the AprilTags it saw in training.** Without them it folds 2/4, against 4/4 with them. Put the 12 box
  tags on the real carton exactly as the setup slides show (steps 4b–4g; `tools/make_fold_box_tags.py` makes the
  print sheet and diagrams), or retrain on tag-free or randomised renders to drop the dependence.
- The simulated actuator loads in the `sim-owner` run reached the model's ceiling on the left shoulder lift and
  elbow. On the robot, `Present_Load` above 800 is a fault that releases everything. Real loads during the pushes are
  unknown.

Reproduce (in a Python with lerobot and mujoco; trials are under `…/Hackatuson/output/fold-demos/`):

```sh
cd software
PYTHONPATH=. python -m carton.fold_policy_runner --checkpoint <015000>/pretrained_model \
  --transport sim-direct --sim-trial <fold-demos>/batch-220-01/trial-020 --out /tmp/fold-run \
  --execute --gripper-mode stream --policy-latency-s 0.03 --max-steps 750
```

## 4. What is still needed, in order

Each step names who does it. "Agent" means a coding session on the chat Mac, with read-only robot calls only.

**A. Review and merge this branch (owner).**
- Read sections 2.1–2.5.
- Run `PYTHONPATH=. python -m pytest -q tests/test_fold_policy_runner.py tests/test_fold_policy_chat.py` and the
  qwen-bridge tests (each file in `TESTS` of `docs/commissioning/2026-10-07-paddle-success/qwen-bridge/redeploy_robot_server.py`, run as its own process with the pilot venv's python).

**B. Head camera aspect and pose (owner + agent).**
- The OAK's wide path now streams the full sensor at 1080p (16:9, commit `dfc3249`). The simulated head camera is
  4:3 (OAK-D Lite full sensor, 54° vertical FOV, `carton/xlerobot_cameras.py`). Pick one:
  1. **Retrain for the real stream.** Measure the OAK's intrinsics at the streamed size
     (`tools/oak_intrinsics.py`, from Terminal) and the head pose from the floor tags (`tools/camera_pose_from_tag.py`).
     Re-render the demonstrations with those (`tools/restage_fold_scenes.py`, `tools/fold_demos_to_lerobot.py`) and
     retrain on HF Jobs: about 1 h, about $2.5. **Preferred:** it also fixes the head pose, which must be measured
     anyway.
  2. **Crop the stream.** Give the runner a 4:3 head stream (a centre crop of the full field). This is only valid if
     the crop's field of view matches the simulation's, which is not known.
- Until then, every dry run reports `camera_aspect: front …`, and the chat refuses to execute.

**C. The rest of the station measurements (owner).** Slides `docs/carton-fold-policy-setup-slides.html`, steps 1–6,
and handoff `carton-fold-policy-handoff-2026-10-08.md` §3: arm spacing 220 mm, base height 12 cm above the carton,
setback, carton placement, wrist-camera calibration, sleeves. Feed them into the retrain in B.

**D. Right jaw meeting point (owner, 2 minutes).** Do an empty close of the right gripper to a target below its
meeting point (e.g. 1330) and read where it settles (`robot_get_state`). If it isn't about 1351 (range_min + 82),
change `profiles/fold-joint-maps/right-joint-map.json` `gripper.model_zero_tick` to `settle + 113.8`, then rebind.

**E. Owner stream mode on the robot Mac (owner).**
1. Approve the change in 2.5.
2. Install it. This is one owner restart, which **releases every motor**, so the arms must be resting or supported
   first; the deploy refuses while motors are holding.
   - Remotely: `python robot_admin.py deploy <branch>` from the chat Mac (`POST /admin/deploy`). It runs all the
     redeploy tests on the robot Mac first, checks that the fresh owner advertises `stream`, and rolls back if the
     new version does not come up. The API is back within seconds.
   - Locally on the robot Mac: `./restart-robot-server.sh`.
3. Validate in free air, STOP in hand: a jaw close and open stream, a hold-here, the 0.5 s timeout hold, and a
   contact yield against a hand.

**F. Switch the chat to the stream transport (owner, after E).** The code is done (2.5). In
`.private/fold-policy.json`, set `"transport": "api-stream"` and `"gripper_mode": "stream"`. Then remove the
stream-mode blocker, run a fresh dry run (its preflight checks `stream_mode`), and record a jaw-close free-air run
in STATUS.md.

**G. Owner loop, link and inference timing (agent, read-only).**
- `python tools/measure_robot_link.py --pilot-root "$PILOT"`. It reports:
  - the round trip;
  - **`owner_loop_period_estimate_s`**: the spread of one poll's per-motor `captured_at` stamps, plus the owner's
    20 ms sleep;
  - `fold_policy_stream_ok`, which is true at ≤ 33 ms (section 3).
- If the loop is slower, the owner loop must be made faster before running the policy.
- The dry run reports `effective_hz` and per-tick latency. If CPU inference is too slow, set `"device": "mps"` in
  the chat config.

**H. Install the chat tools (owner approves; agent runs it).** Run it from a stable checkout of `main`, not a
temporary worktree: the loader line stores this file's absolute path.

```sh
cd <repo>/software
python tools/bind_fold_joint_maps.py --calibration <live calibration copy> --out <pilot>/../fold-bound-maps
python tools/install_fold_policy_chat.py --pilot <pilot> \
  --checkpoint <…>/act_carton_both_shorts_220_bs32_lr3e5_chunk100/checkpoints/015000/pretrained_model \
  --joint-map left=<pilot>/../fold-bound-maps/left-joint-map.json \
  --joint-map right=<pilot>/../fold-bound-maps/right-joint-map.json
```

Then restart the chat server: stop it, then `../.venv/bin/python start.py`. Restarting drops any open chat stream,
which sends STOP, so do it with the arms released.

**I. Dry runs from the chat (owner present; no motor command).**
1. Pose the released, supported arms at the training start pose (handoff 2026-10-08 §2a: ticks within about ±30).
2. Ask the chat to "dry-run the fold policy for 5 seconds".
3. Check the result:
   - `clean`;
   - no `start_outside_training`;
   - `effective_hz` about 10;
   - clamp counts, and `ticks.jsonl` in the run directory.
4. Repeat until clean.

**J. First motion (owner at STOP).**
1. Edit `.private/fold-policy.json`: `execute_enabled: true`, `operators: ["<name>"]`, keep `execute_max_steps: 10`,
   and clear `blockers` only for what is actually closed.
2. Enable both arms; they hold at the start pose.
3. No carton.
4. Ask the chat to run 10 steps.
5. Expect at most 40 ticks of motion per tick. It ends holding.
6. Support the arms, then STOP.
7. Raise `execute_max_steps` only after reading `ticks.jsonl`.

**K. Carton (owner).**
- Only after B–F and a retrain for the measured station.
- Use an empty carton.
- First enough steps for one short flap, then both. A full fold is about 600 steps (60 s).

Stop and press STOP if any of these happen:
- an arm presses on the table, the cart or the other arm;
- the carton slides more than a few centimetres or lifts;
- a joint stalls under load;
- anything moves unexpectedly.

Record every run in `software/STATUS.md` with its run directory.

## 5. Files

| What | Where |
|---|---|
| This handoff | `docs/carton-fold-policy-chat-mac-handoff.md` |
| Chat tools, installer, tests | `carton/fold_policy_chat.py`, `tools/install_fold_policy_chat.py`, `tests/test_fold_policy_chat.py` |
| Runner, fakes, tests | `carton/fold_policy_runner.py`, `carton/fold_policy_fakes.py`, `tests/test_fold_policy_runner.py` |
| Owner stream mode | `docs/commissioning/2026-10-07-paddle-success/qwen-bridge/stream_joint_executor.py`, `STREAM-MODE.md`, `test_stream_targets.py`; changes in `gemma_hardware_owner.py`, `gemma_direct_client.py`, `gemma_robot_tools.py`, `redeploy_robot_server.py` |
| Joint maps | `profiles/fold-joint-maps/` (+ `tools/bind_fold_joint_maps.py`) |
| Earlier handoffs | `carton-fold-policy-handoff-2026-10-08.md` (station, hand poses), `carton-fold-policy-robot.md` (owner contract, 8 Oct), `carton-fold-policy.md` (training results) |
| Setup slides | `docs/carton-fold-policy-setup-slides.html` |
