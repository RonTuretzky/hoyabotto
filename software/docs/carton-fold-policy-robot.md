# Fold policy on the robot: control path, adapter, blockers and commissioning

**Status, 8 October 2026: software only. No robot command, owner API call, LAN or relay connection or camera was
used to write this.** The adapter was tested against the deployed robot-server code running on fake servos and
against the MuJoCo carton simulation. It has never moved a real motor.

**The policy was trained only in simulation, on simulated cameras.** Its `top` and `front` images are renders of
a simulated overhead camera (0.85 m above the table) and a front camera. No real camera on the robot has been
shown to see the station that way. The simulated station also differs from the real one
([sim handoff, caveat 1](carton-sim-training-handoff-2026-10-08.md)). Do not expect the checkpoint to fold a
real carton. On the robot, this adapter is a commissioning and data-collection harness, not a task skill.

- Code: `carton/fold_policy_runner.py` (runner, joint maps, real transports, CLI) and
  `carton/fold_policy_fakes.py` (fake owner, plants, simulated cameras).
- Tests: `tests/test_fold_policy_runner.py`.

## 1. The policy

- Built by `tools/record_fold_demos.py`, `tools/fold_demos_to_lerobot.py` and `tools/eval_fold_policy.py`.
  It is a LeRobot ACT checkpoint, loaded with `farm/learning/infer.py` `PolicyRunner`.
- **Input:** `observation.state` is 12 floats in radians, in the URDF `so101_new_calib` convention. The order is
  left `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`, then the same six for the right
  arm. `observation.images.top` and `.front` are 240×320 RGB.
- **Output:** 12 absolute joint targets in the same units, at 10 Hz. Use ACT temporal ensembling (coefficient
  0.01, `n_action_steps` 1), as `enable_temporal_ensembling` sets it.
- **Simulation results** (held-out starts, `output/fold-evals/*/result.json`): checkpoint 015000 with temporal
  ensembling held both short flaps in 12 of 16 episodes; without ensembling, 1 of 16. Another session is still
  evaluating 020000 and 025000 (`last`).
- **Two properties of the data matter on a robot:**
  - The right gripper is constant in training: state std 3.8e-5 rad at −9.7°. Normalization divides by
    std + 1e-8 (lerobot `normalize_processor.py`), so a one-tick encoder difference (0.0015 rad) becomes about
    40 standard deviations.
  - Demonstrated joint speeds reach about 6° per 0.1 s at the shoulders and 19° per 0.1 s at the wrist roll
    (trial-000).

## 2. The real control path today

The robot Mac runs one **sole hardware owner** and an **HTTPS tool API**. Every motion client goes through the
API. Nothing else may open the servo ports. `./restart-robot-server.sh` →
`qwen-bridge/redeploy_robot_server.py:30` starts the owner as
`--both-arms --paddle-profile --wheels --allow-missing-bus`, so the profile is `paddle-success-v1` for both arms.
All `qwen-bridge/` paths are under `docs/commissioning/2026-10-07-paddle-success/qwen-bridge/`.

```
chat Mac / any client --mTLS POST /call--> gemma_robot_tools.dispatch (robot Mac)
    --> DirectJointClient (gemma_direct_client.py) --command.json / status.json--> HardwareOwner (gemma_hardware_owner.py)
        --> PaddleJointExecutor (paddle_joint_executor.py) --> Feetech bus writes (Goal_Position, ticks)
```

Not the real path:

- **`farm` skills and `PolicySkill`** (`farm/skills/runner.py:87`, `farm/skills/policy.py`). They drive
  `LeRobotXLeRobot`, which opens the ports itself, so it would be a second owner.
- **`farm/safety/rules.clamp_targets`** (`rules.py:30-39`). It clamps LeRobot normalized units (−100..100);
  `step_deg_max` (`farm/config.py:73`) is applied in those units, not in ticks.
- **`carton/servo/bimanual_owner.py`** (protocol v2). It is offline-tested only:
  `capabilities()['hardware_deployment_verified']` is False (`bimanual_owner.py:142`).
- **`farm policy-server` / `farm/learning/remote.py`.** They replay whole chunks and do not ensemble. They can host
  inference, but they do not command anything.

The in-repo precedent for an API motion client is the AprilTag mover, `carton/servo/gemma.py`. Its docstring
(lines 6–13) records the same owner contract this adapter relies on.

### Sending targets

| What | Today's behaviour | Where |
|---|---|---|
| Tool | `robot_move_joint_targets {arm, positions, duration_s ≤ 25, wait, replace}`. Takes **one arm per call**. `robot_move_motor_targets` accepts both arms but always waits for settling. | `gemma_robot_tools.py:202`, `:190`, `:84-125` |
| Units | Integer raw encoder ticks, 4096 per turn. The API says "use 4095 ticks/rev" for degrees. | `gemma_robot_tools.py:490`; `direct_joint_executor.py:33` |
| Range | Targets must be 40 ticks inside the saved `range_min..range_max`. Enable is allowed within 4 ticks of the range. | `paddle_joint_executor.py:21,58`; `gemma_robot_tools.py:73-82`; `gemma_hardware_owner.py:186` |
| 3-tick rule | Each joint in a move must travel 3..341 ticks from its **current encoder reading**. If every joint is within 2 ticks, the API answers `no_op` and sends nothing. | `paddle_joint_executor.py:60`; `gemma_robot_tools.py:92-94`; `carton/servo/gemma.py:25` |
| Ramp | The goal moves at most 40 ticks per write. The first write is immediate; later writes come every max(0.4 s, duration/steps). Owner servo settings: Goal_Velocity 100 (jaw 200), Acceleration 5, Torque_Limit 800 (elbow 400, jaw 500), P 32 on lift and elbow. | `paddle_joint_executor.py:25,68,177-185`; `gemma_hardware_owner.py:193-195` |
| Following | \|present − goal\| ≤ 96 ticks for the moving executor and for every held joint. Otherwise **fault: release all**. | `paddle_joint_executor.py:110`; `gemma_hardware_owner.py:136` |
| Load | Above 800 (arm) or 500 (jaw) on an enabled motor: fault, release all. Load ≥ 600 for 2 samples on an arm joint lagging ≥ 20 ticks: `contact_halt` (that joint's goal is pulled back, nothing is released). | `gemma_hardware_owner.py:128`; `paddle_joint_executor.py:31-32,128-144` |
| Gripper | A **closing** gripper runs alone and blocking: 10 ticks per ≥ 1.5 s, never in a path, never with `wait=false` or `replace`. | `paddle_joint_executor.py:62-63,68`; `gemma_robot_tools.py:99-100` |
| Both arms | One owner command may name joints of both arms, but each named arm needs all six of its joints enabled. | `gemma_hardware_owner.py:309-312`; `gemma_direct_client.py:159-161` |
| Replace / halt | `replace=true` halts the running motion and starts the new one from the held goals. `robot_halt_motion` holds the last goals and releases nothing. There is no "hold where you are now" command. | `gemma_hardware_owner.py:275-283,296-299`; `paddle_joint_executor.py:86-89` |
| Watchdogs | The owner loop polls all motors, then sleeps 20 ms (`:389`). Its period is not measured; the contract test assumes about 50 ms. Executor tick gap ≤ 1.0 s. Lease: deadline + 5 s during a command, 120 s after enable or completion; on expiry, release all. The phone feed must be fresh (< 10 s), with a 20 s pause, then release. | `gemma_hardware_owner.py:122,137,149,202,317,389`; `paddle_joint_executor.py:107`; `paddle_camera_gate.py` |
| STOP | `robot_stop` → `DirectJointClient.stop` → owner `release_all`: torque is ramped off over 2 s (`soft_release_s=2.0`, `:364`), then released. `stop_count` +1. **No latch:** motors stay off until an explicit `robot_set_motor_enable`. Clients detect a STOP or fault by `stop_count`. | `gemma_direct_client.py:73-98,256,271`; `gemma_hardware_owner.py:230-249,260` |
| **Failure after dispatch → STOP** | If a command fails after it was written (owner rejection, stale status, timeout), **the client itself sends STOP, which releases all motors.** | `gemma_direct_client.py:268,272,328-330` |

### Reading joints and cameras

- **Joints:** `robot_get_execution` returns the owner's `status.json`, including all 16 rows
  (`gemma_robot_tools.py:244-252`):
  - per motor: `Present_Position` (int ticks), `Present_Load`, `Present_Velocity`, `Moving`, `Status`,
    `Torque_Enable` and `captured_at`. `captured_at` is on the robot clock, stamped at each poll
    (`gemma_hardware_owner.py:126`);
  - for the owner: `time`, `status_age_s`, `started`, `phase`, `ok`, `stop_count`, `last_stop`,
    `enabled_motors`, `goals` (enabled motors only), `ranges` (saved), `execution_profile`, `closure_outcome`
    and `lease_remaining`.

  `robot_get_state` serves the same rows (`:255-305`).
- **Cameras:** `robot_get_cameras` returns `oak` (head OAK-D RGB, rectified), `phone` (receipt time only, capture
  delay unknown) and `left_wrist`/`right_wrist` (640×480), as base64 JPEG. Frames older than 1 s are refused
  (`gemma_robot_tools.py:383-447`, `:434`).

### Calibration and kinematics files

- **Saved calibration.** The live file is on the robot Mac:
  `/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json`
  (`gemma_robot_tools.py:50`). Both arms were recalibrated on 7 October (STATUS.md). The repo snapshot
  `calibration/farm_xlerobot/farm_xlerobot.json` (5 October) is therefore stale.
- **Right arm:** `qwen-bridge/right-arm-kinematics.json`. Mapping `feetech_degrees_v1`, `mapping_validated: false`,
  no `model_zero_tick`/`model_sign`, `gripper_from_tool: null`.
- **Left arm:** `outputs/Standard-Reach-Candidate.json` exists only on the robot Mac (`gemma_reach_planner.py:18`).
  Its contents cannot be determined from this repo.
- **Requirements:** `CalibratedArm` (`farm/kinematics/lerobot.py:87-119`) and `JointUnits`
  (`farm/kinematics/units.py:49-60`) need a measured model zero tick and sign for each of the five arm joints.
- **No gripper (jaw) tick↔angle mapping exists anywhere.**

## 3. What the adapter does

`FoldPolicyRunner` runs one tick every 0.1 s. Every step is recorded in `<run>/ticks.jsonl`:

1. Run the operator stop hook. Read the owner with **one** `robot_get_execution` call, then check:
   - the owner is healthy, with the same `started` and **the same `stop_count`** as at start (no retry after a STOP
     or fault);
   - motor `Status` is 0 and, when executing, all 12 arm motors are enabled;
   - the joint watchdog: the oldest arm row is ≤ 0.5 s old on the robot's clock (profile `watchdog_s`), and rows
     must advance.
2. Fetch both camera frames. Check age ≤ 1 s and that a new frame arrived (at most 2 repeats).
3. Convert ticks to radians per arm with the **measured** joint map (`load_arm_map`, using `JointUnits`).
   - It refuses if any of the five joints' zero/sign, the gripper zero/sign, the evidence references or the
     calibration digest is missing.
   - It refuses the deployed `feetech_degrees_v1` candidate.
   - It refuses if the owner's ranges differ from the map's calibration.
4. Condition the state. A dimension that was constant in training (the right gripper) is fed the training value
   if the measurement is within 5° of it; otherwise the run aborts. Both the substitution and the deviation are
   logged.
5. Run the policy (temporal ensembling, one inference per tick). The action must be 12 finite values.
6. Plan targets, recording every bound applied:
   - clip to the training action range ± 5°;
   - convert to ticks and clamp to the commandable range (saved range − 40);
   - clamp to ±40 ticks of the **measured** position (the owner's STEP);
   - drop joints under 3 ticks (owner rule);
   - hold the grippers (default `gripper_mode='hold'`). In `follow` mode an opening may be streamed, never a
     closure, and never on an arm the operator declared `holding`;
   - **race guard:** do not command a target inside [measured, held goal] ± 3 ticks. The joint will pass through
     it before the owner reads the command, the owner would refuse (3-tick rule), and the client would answer
     with STOP.
7. **Dry-run (default):** log what would be sent. **`--execute`:** send. `ApiOwnerTransport` makes two
   `robot_move_joint_targets` calls (left, then right; `wait=false`, `replace=true`, `duration_s` 0.4).
   `DirectClientOwnerTransport` makes one both-arm `DirectJointClient.execute` on the robot Mac.
8. **Runner-side contact rule.** The owner's contact guard counts per motion, and a streamed motion lives about
   one owner poll. Its `contact_halt` is overwritten by the next command's start. So the runner applies the same
   rule across ticks: load ≥ 600 on an arm joint whose held goal or last target is ≥ 20 ticks from where it is,
   for 2 ticks, aborts the run.
9. **Tick watchdog:** a tick longer than 0.3 s aborts the run.

On any abort, or at the end, the runner sends `robot_halt_motion`, which holds every joint and **releases
nothing**. It never calls `robot_stop` or disables torque: STOP stays with the operator. Owner faults still
release on their own.

Refusals before motion (with `--execute`):

- owner profile not `paddle-success-v1`;
- arms not enabled by the operator;
- start state outside the training range + 10°;
- warm-up inference failed.

Limits can only be tightened. `SafetyConfig.validate` refuses a step over 40 ticks, a minimum travel under
3 ticks, a margin under 40 ticks, watchdog ages above the owner's, and a looser contact rule.

## 4. Results in simulation (no robot)

- **Fast tests** (`tests/test_fold_policy_runner.py`, 22 tests, about 6 s): the real `gemma_robot_tools.dispatch`
  → `DirectJointClient` → `HardwareOwner` / `PaddleJointExecutor` stack runs over a fake bus on a virtual clock
  (the recipe of `qwen-bridge/test_tag_registration_contract.py`). They cover:
  - refusals for unmeasured or stale maps;
  - limits that cannot be relaxed;
  - every clamp, and the race guard;
  - dry-run sends nothing (no bus writes);
  - execute stays within 40 ticks per tick and ends holding with all motors enabled;
  - the stop hook halts without releasing;
  - an operator STOP between ticks ends the run with no re-enable;
  - stale telemetry, stale or frozen frames, non-finite actions and contact all abort;
  - the both-arm direct client;
  - `ApiCameras` through the real `robot_get_cameras` (JPEG, receipt time for the phone);
  - the MuJoCo station behind the owner.
- **Recorded demonstration replayed as the "policy"** (trial-000, 75 s, owner loop 50 ms, this adapter's bounds):

  | transport | outcome |
  |---|---|
  | `sim-direct` (runner bounds only) | both shorts held: 91.5° / 92.2° at 55 s; carton moved 14.8 mm |
  | `sim-owner` (deployed owner) | at 52 s the owner faulted with `left_arm_shoulder_lift: uncommanded holding drift` and released all motors, with the right short at 45°; carton moved 92.9 mm |

  The tick conversions and clamps preserve the demonstrated fold. The owner's semantics do not, as described in
  §5.
- **Trained checkpoint, closed loop.** Checkpoint `last` = 025000 on CPU, temporal ensembling 0.01, with a fixed
  30 ms of virtual inference time per tick. 75 s per episode on held-out starts. Success means both shorts stay at
  80° or more for 3 s. Two of these rows are diagnostics that bypass a bound the deployed owner imposes; they are
  marked.

  | Transport and bounds | trial-000 | trial-020 | trial-030 |
  |---|---|---|---|
  | `sim-owner`, defaults (`gripper_mode=follow`) | no: shorts 17° / 5°, no owner fault, 10.0 Hz | – | – |
  | `sim-direct`, defaults (`follow`: openings only) | no: 17° / 4° | no: 9° / 1° | no: 12° / 6° |
  | `sim-direct`, defaults, **gripper closures streamed too** (diagnostic; the owner refuses this) | **yes**, at 58.5 s | **yes** | no: 8° / 7° |
  | `sim-direct`, no step, race or 3-tick bounds, closures streamed (diagnostic) | yes, at 56.8 s | – | – |

  Read with care: these are one episode per cell. They do show two things:
  - The joint conversion, the 40-tick step, the 3-tick rule and the race guard can coexist with a successful
    closed-loop fold (2 of 3 seeds).
  - **Dropping the policy's gripper closures** (the deployed owner cannot stream them) made every run fail.

  Reproduce with:
  ```sh
  FOLD_POLICY_E2E=1 PYTHONPATH=. python -m pytest -q -s tests/test_fold_policy_runner.py -k checkpoint
  ```
  or with the CLI in §6. Logs from these runs are not in the repo.

## 5. Blockers before any physical run

Items 1–3 are refused by the runner itself. The rest are measurements and decisions for the owner.

1. **Per-arm URDF zero and sign for all five positioning joints, both arms.** Neither arm has them.
   The right arm has only the unvalidated `feetech_degrees_v1` midpoint candidate; the left arm's file is not in
   the repo.
2. **Gripper mapping (zero tick and sign) for both jaws**, plus a check that the URDF jaw angle is linear in ticks.
   It does not exist.
3. **Calibration identity.** Joint maps must be measured against the live 7 October calibration, and its digest
   recorded. Whether the calibration survives a 12 V power cycle is unconfirmed (STATUS.md).
4. **Camera identity and pose.** No robot camera corresponds to the simulated `overhead` (0.85 m, straight down)
   or `front` camera. The head OAK looks closely down at a small patch of the table (sim handoff, caveat 1). The
   phone carries only receipt time.
   - Until the station is rebuilt in simulation from measured geometry and camera poses
     (`carton-real-station-measurements.md`) and the policy is retrained or fine-tuned, `--camera top=… front=…`
     only feeds out-of-distribution images.
   - The mapping is deliberately not defaulted.
5. **Station geometry and start pose.** Training starts parked with shoulder lifts at −100° (the URDF limit) and
   wrist flex at 95°. These start states must lie inside the real commandable ranges and be reachable. The
   carton must sit where the simulation put it.
6. **Timing.** With per-arm API calls each tick, the loop waits for at least 2 owner loops (acceptance), plus
   4 round trips and camera transfer.
   - At a 50 ms owner period the send alone takes about 0.1 s: the loop ran at 9.6 Hz in simulation with zero
     network delay.
   - The relay's 150–250 ms round trip (STATUS.md) makes 10 Hz impossible. LAN round trip, owner loop period and
     OAK latency are unmeasured; measure them read-only with `tools/measure_robot_link.py`.
   - Inference must also fit: on this loaded development Mac a cold CPU inference took 0.27 s.
7. **Speed.** The owner sets Goal_Velocity 100. If that means about 100 ticks/s, as the pilot's
   40-ticks-per-0.4-s ramp suggests (unverified), joints move at most about 9°/s, against demonstrated peaks of
   about 59°/s. The adapter's 40-tick per-tick bound already caps them at 35°/s.
8. **Owner semantics that do not fit a 10 Hz policy** (not changed here: owner changes need their own review and
   hardware validation):
   - A refused streamed target (the 3-tick rule racing a moving joint, a stale status) makes `DirectJointClient`
     send **STOP, releasing every motor and dropping both arms**. The race guard only reduces the odds.
   - The owner's `contact_halt` is ineffective for streamed commands, and there is no command to hold at the
     present position. The runner can only halt at the last goals.
   - **Gripper closure cannot be streamed.** The demonstrations open and close the left claw (open-claw
     transfer). With the default `hold` the policy's gripper motion is dropped; with `follow` only openings run.
     In simulation this alone took the closed-loop fold from 2/3 to 0/3 (§4). This is the main owner-side gap.
   - The **96-tick holding-drift fault** ends the replayed fold (§4): a held arm pushed back by the carton releases
     everything.
   - Real `Present_Load` during pushes is unknown. Above 800, the owner releases all motors.
9. **Right gripper must read within 5° of −9.7°** in the policy convention, or the run aborts (training constant).
10. **Physical material and contact.** Crease stiffness, friction and carton mass are simulation assumptions.
    The simulation's applied-contact checks do not certify the real station.

## 6. Supervised commissioning procedure (for the owner)

Do these in order and stop at the first failure. Record each step in `software/STATUS.md`. Keep the pilot's STOP
(`robot_stop`) and the 12 V switch in reach whenever any motor is enabled. Nobody may be in the arms' sweep.

0. **Offline**, on the chat Mac:
   ```sh
   cd software
   PYTHONPATH=. python -m pytest -q tests/test_fold_policy_runner.py
   ```
   Optionally run the simulation end to end:
   ```sh
   PYTHONPATH=. python -m carton.fold_policy_runner \
     --checkpoint <ckpt> --transport sim-owner --sim-trial <fold-demos trial> \
     --out <run> --execute --policy-latency-s 0.03
   ```
   Read `<run>/summary.json`.
1. **Close blockers 1–3, arms released, torque off.** For each joint, put the arm at the URDF zero pose with a
   jig or protractor and read `robot_get_state` ticks: that gives the zero. Move the joint by hand in the URDF
   positive direction and note whether the ticks increase: that gives the sign. Do the same for the jaw (closed
   and open angles). Write `left-joint-map.json` and `right-joint-map.json` in the format of `load_arm_map`'s
   docstring, with `calibration_sha256` of the live calibration file and an `evidence` reference (photos, readings).
   Check them with:
   ```sh
   python -c "from carton.fold_policy_runner import load_arm_maps; load_arm_maps({...})"
   ```
2. **Link timing (read-only):**
   ```sh
   tools/measure_robot_link.py --pilot-root "$PILOT"
   ```
   If the median round trip plus the owner loop does not fit 4 calls inside 0.1 s, do not continue at 10 Hz.
   Either run the `direct-client` transport on the robot Mac, or plan an owner-side change.
3. **Dry run, arms released and supported, no carton.**
   ```sh
   python -m carton.fold_policy_runner --checkpoint <ckpt> --transport api \
     --pilot-root "$PILOT" --joint-map left=… --joint-map right=… \
     --camera top=<cam> --camera front=<cam> --out <run> --max-steps 100
   ```
   No `--execute`. Nothing is sent. Check in the log:
   - state radians against hand-measured angles;
   - the `start_outside_training` and `degenerate_state` warnings;
   - the action magnitudes;
   - the clamp counts;
   - `latency_s` and `effective_hz`.
4. **Dry run at the training start pose.** Pose the released arms by hand at the training start: the first state
   of a held-out demonstration (`demo.npz` `qpos[0]`). Trial-000 has shoulder lifts at −100°, wrist flex 95°,
   elbows about 31°, pans −31° (left) and +31° (right), and wrist rolls 0° (left) and 86° (right). Repeat step 3
   until preflight reports no start-state warning.
5. **Motors, free air, STOP in hand.**
   - Enable each arm with `robot_set_motor_enable` (six joints per call; it holds in place).
   - Run with `--execute --operator <name> --max-steps 10`, with no carton and both claws open in free space.
     Expect at most 40 ticks of motion per tick.
   - The run ends holding (halt). Support the arms by hand, then `robot_stop` to release them.
   - Increase `--max-steps` only after reviewing `ticks.jsonl`.
6. **Carton.** Only after blockers 4–8 are closed and the policy was retrained or validated for the measured
   station and real cameras. Start with an empty carton, a person at STOP, and `--max-steps` covering one short
   flap.
