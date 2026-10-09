# Sole-owner pickup profile

Start it on the robot Mac with `./restart-robot-server.sh` from the repo root: it stops the API and the previous owner (refusing if motors are holding), installs this folder's files, and starts the owner with `--right-arm-only --paddle-profile` plus a fresh API. `restart_gemma_owner_released.py --right-arm-only --paddle-profile` still works once the previous owner has exited. Motors remain released at startup. Runtime profile is paddle-success-v1. Existing API tool names are unchanged. Fetch fresh readiness/state after restart; never replay a stale owner session.

The profile mirrors the observed successful encoder pilot: 3..341 ticks per joint per command, saved-range margin 40, 40-tick steps no faster than every 0.4 seconds, tracking envelope 96 ticks. Motor velocity 100 (gripper 200), acceleration 5; torque ceilings 800 (elbow 400, gripper 500), capped by previous settings; shoulder-lift and elbow P gain 32. Saved settings restore on release. Three fresh quiet samples are required, Moving=0, velocity magnitude below 3, position change at most 3, endpoint tolerance 57 (gripper 30).

Decreasing right-gripper targets use 10-tick contact steps, stationary for 0.3 seconds, maximum 1.5 seconds per step. Resistance at least 40 ticks stops further closure and returns stationary_closure_unverified, grasp_verified=false. Camera evidence is still required to establish pickup. Supply, load, travel, readbacks, stale telemetry, watchdog, STOP and session ownership checks remain. There is no temperature check under this profile (temperature handling was removed on 2026-10-05); the 70 °C check applies only to `legacy-direct`. The profile does not establish camera transforms or add autonomous camera interpretation.

Client completion checks and deadlines use the named profile; the legacy executor remains unchanged. New fake tests cover 25-tick lag acceptance, 96-tick rejection, ramp, margins, segment bound, quiet settling, contact stop and client endpoint acceptance. Existing owner and legacy executor tests pass.

Installed on the robot Mac with motors released, zero startup motor writes, all 16 responding, stop_latched=false and motion_ready=true. No physical movement test was performed in this update; prior physical pickup evidence does not validate the new integration. Qwen must inspect fresh cameras/readiness and correct its approach using current encoder positions.

Pickup-profile lease correction: enable and completed movement grant a monitored 120-second idle hold, matching the successful pilot. Each accepted move reserves its bounded executor deadline plus five seconds. The client requires a still-positive current lease rather than requiring the entire new move to fit inside the previous idle lease. After an expired lease or a STOP, motion is refused until an explicit enable, and a changed session is refused; no automatic rearming. Fake-client checks verify acceptance with one second remaining and rejection with zero remaining before dispatch.


Additional pilot corrections now implemented: camera freshness is checked before activation and before each new target; a stale phone feed pauses target advancement while servo health and STOP remain active, resumes only on a new fresh sequence, and times out after 20 seconds. The next ramp starts from the last commanded held waypoint and rejects a changed owner position outside the 96-tick envelope. Pickup commands require all six right-arm joints enabled, enforce a 20-segment session budget, and preserve a 120-second monitored hold.


The pickup watchdog accepts up to a one-second telemetry gap; legacy profiles retain the 200 ms watchdog. This is deliberately bounded and does not turn stale telemetry into a ten-minute wait. The motion tool keeps any move of <=341 ticks per joint as one segment, splits longer moves into <=280-tick segments (each later piece stays within 341 of a previous piece that settled up to 57 ticks off), and returns a no-op for residuals within two ticks.

## Simultaneous joints and settling (2026-10-07)

Not yet run on hardware; fake-hardware tests only (`test_paddle_joint_executor.py`, `test_paddle_segments.py`, `test_paddle_owner.py`, `test_paddle_client.py`, `test_paddle_stop_recovery.py`).

- **Several joints per command.** `PaddleJointExecutor` accepts any set of right-arm joints. The longest ramp moves in full 40-tick steps and the shorter ones are spread over the same number of steps, so all joints arrive together. Each joint keeps its own 3..341-tick, 40-tick-margin and 96-tick following checks. A closing gripper (contact mode) is still commanded alone; `robot_move_joint_targets` runs it after the other joints of the same call. An opening gripper can move with the arm. The 20-segment budget counts one per command, so combining joints uses fewer segments.
- **Settling under load.** Previously a joint that came to rest short of target, such as 58 ticks short under gravity with zero stable samples, ran into the deadline. The owner then faulted, released all motors and latched STOP. Now, once the ramp has reached target and every joint has been still for three fresh samples (velocity below 3, change at most 3), each unsettled joint gets a goal correction that overdrives its held goal by the measured residual. A correction moves at most 40 ticks per write, at most 57 ticks beyond target, and never more than 90 ticks from the present position. It also stays 40 ticks inside the saved range. Each joint gets at most 3 corrections, with 5 extra seconds on the deadline. While a joint is overdriven, the firmware `Moving` flag is ignored, because it compares against the overdriven goal; physical stillness is still required. Endpoint tolerance stays 57 (gripper 30).
- **`settled_short`.** If a joint is still at rest outside tolerance once corrections are spent, the move ends `closure_outcome: settled_short`, `endpoint_reached: false`, with `settle_residual_ticks`. The motors keep holding and there is no STOP. The client returns `completed: false` and does not send STOP. The tool stops any remaining segments. A joint that never comes to rest still faults at the deadline. Tolerances, load, travel, step, watchdog and following limits are unchanged.
- **No STOP latch.** Any fault or `robot_stop` still releases every motor at once and cancels the move in progress, which is never resumed. The owner then returns to phase `idle` with `operator_armed: true` and `stop_latched: false` (kept for compatibility). No owner restart is needed. Nothing re-enables by itself: motors stay released until an explicit `robot_set_motor_enable`, which repeats every health, range, camera and voltage check. A camera pause ends with the release, so that enable needs a fresh phone feed. Each stop or fault is recorded as `last_stop` (`time`, `reason`, `command_id`, `released`, `release_errors`) and counted in `stop_count`; `error` holds the latest reason. A client waiting on a move reports `Owner stopped: <reason>` as soon as the stop is recorded instead of waiting for its deadline, and a segmented `robot_move_joint_targets` call does not continue past a STOP. If a torque-off release fails, `ok` stays false and readiness blocks on `OWNER_NOT_HEALTHY`: that is a hardware problem, not a latch. The owner refuses enable until a later `robot_stop` confirms the release. `./restart-robot-server.sh` is no longer needed to clear a STOP; it is still how new code is deployed. The 20-segment budget is per owner session and a STOP does not reset it.
- **Hold lease.** Re-sending `robot_set_motor_enable` for the six joints while holding renews the 120 s hold (camera must be fresh). Readiness now reports this as `explicit_enable_renews_idle_lease: true` under this profile.

The procedure Qwen receives is `paddle-procedure.json`, served by `robot_get_handoff` as `physical_pickup_procedure`.

## Base drive (2026-10-07)

Not yet run on hardware; `test_wheel_pulse.py` covers it on fake hardware. `wheel_pulse_executor.py` ports the hardware-validated `drive-pulse.py` into the sole owner. Start the owner with `--wheels`; `./restart-robot-server.sh` does this by default, and `--no-wheels` turns it off.

- `robot_move_base(linear_m_s, angular_rad_s, duration_s)` runs one pulse:
  1. Check both wheels are released, status 0 and 10–14 V, and that the phone feed is fresh.
  2. Save Operating_Mode/Acceleration/Torque_Limit/Lock, then switch to velocity mode (Lock 0, mode 1, acceleration 10) and turn torque on.
  3. Drive at the commanded wheel speeds, checking every sample.
  4. Command zero velocity and require the wheels to settle within 0.9 s.
  5. Turn torque off and require 5 still samples (not rolling).
  6. Restore the saved settings.
- The wheels never stay powered and never join the arm's enabled/hold set. `robot_set_motor_enable` still cannot power them in the right-arm scope.
- Limits: each wheel at most 0.02 m/s, the validated 261 ticks/s, so |linear| ≤ 0.02 m/s or |angular| ≤ 0.16 rad/s. Duration is at most 3 s per call, up from the prototype's 1 s. Kinematics use the vendor wheel radius 0.05 m and wheelbase 0.25 m; the left wheel is mirrored, so forward is left −, right +, matching the prototype.
- Checks every sample: Status 0, |load| ≤ 500, |velocity| ≤ 400, 10–14 V, telemetry gap ≤ 0.3 s. A stale phone feed (≥ 10 s) brakes the pulse early (`stopped_early`) instead of faulting.
- Any other failure, `robot_stop` or owner exit runs the abort path first: Goal_Velocity 0 and torque off on both wheels, then restore. The arm is released after that.
- Settle thresholds are new, because the prototype's `wheel_stop_check` module is not in the repo. Settled means |velocity| ≤ 5 and ≤ 3 ticks of movement over 2 samples; released means ≤ 5 ticks over 5 samples.
- Allowed while the arm is released or holding; refused while an arm move runs, and an arm move is refused while the base drives. The prototype only drove with the arm released.
- Risk to know: velocity mode keeps spinning until told to stop. If the owner process is killed hard (SIGKILL or power loss to the Mac) mid-pulse, the wheels keep turning at ≤ 0.02 m/s until the 12 V supply is cut. A normal exit, SIGTERM or fault stops them.
- The result gives wheel encoder deltas and estimated travel. Slip and real cart motion are unverified, so check the cameras after each pulse.

## Usability fixes from the first API use (2026-10-07)

- **Enable near a limit.** Enable used to require every joint to sit 40 ticks inside its saved range. After a release, the shoulder sagged near its limit, so the arm could never be enabled to drive itself back. Now a joint only needs to be 4 ticks inside its range to enable and start a move. Targets and corrected goals still stay 40 ticks inside, so a move from the edge can only head inward. A joint reading beyond its range still has to be moved by hand.
- **Wheel release check.** The first API base pulse drove and braked correctly but then faulted with "Wheels rolling after release". The after-release check required every Present_Velocity reading to be ≤ 5, and released Feetech servos report spurious velocity while stationary (STATUS.md, right elbow "velocity50 despite stable position"). The check now judges rolling by encoder position only: more than 5 ticks over 5 released samples. If it fires, the error carries the measured position change and velocity readings.

## Continuous, monitored motion; no move limit (2026-10-07)

Fake-hardware tests only (`test_continuous_motion.py`).

- **No per-session move limit.** The 20-segment budget is gone. Moves are still counted in `pickup_motion_segments_used`, for information.
- **Waypoint paths.** A command can carry `waypoints` instead of `positions`. The ramp passes through intermediate waypoints without settling, at the same 40-tick / ≥0.4 s cadence, and settles only at the last one, with corrections and `settled_short` as before. Each leg is ≤ 341 ticks per joint, with up to 24 waypoints, a duration of at most 60 s and a deadline of at most 80 s. A path cannot close the gripper. `robot_move_joint_targets` sends a move longer than 341 ticks as one continuous path instead of separate stop-and-go segments. `robot_move_path` takes explicit waypoints and fills joints that don't change.
- **Non-blocking moves.** With `wait=false` the move returns as soon as the owner accepts it. `robot_get_motion` reports live progress: phase, waypoint, per-joint current/goal/target and following error, elapsed time, outcome.
- **Halt.** `robot_halt_motion` (owner op `halt`) stops advancing and holds the last commanded goals, at most one 40-tick step ahead of the arm. The outcome is `halted`, nothing is released, and the 120 s hold renews. On the base it brakes the pulse early.
- **Change course.** A move with `replace=true` while an arm motion runs halts it and starts the new one from the held goals, with no release in between. Without `replace`, a second move is refused while one runs.
- All the existing per-tick checks still run during monitored motion: following envelope, health, watchdog, camera gate, STOP.

## Contact guard (2026-10-07)

There is no self-collision model: per-joint calibration ranges cannot stop the arm reaching into the mast, head, base or other arm through a combination of joints. As a stopgap, `test_contact_guard.py` covers this:
- **Contact halt.** If an arm joint (not the gripper) shows |Present_Load| ≥ 600 for two fresh samples while lagging ≥ 20 ticks behind its command, the motion ends `contact_halt`. That joint's goal is set to its present position, so it stops pushing; the others hold. Nothing is released, and the 120 s hold renews. The result names the joint and its load. 600 is a first guess (75% of the 800 release limit) to tune from logs.
- **Trade-off.** If the high load was gravity rather than contact, the joint sags by its following error after the back-off.
- **Corrections.** A joint at ≥ 600 load gets no settle correction. A joint that did not move after a correction gets no further ones, and is reported in `possible_contact_joints`.
- The 800-load and 96-tick release limits are unchanged and still catch anything faster.

## Soft release on STOP and faults (2026-10-07)

`test_soft_release.py` covers this. STOP, and owner faults where the bus still answers, release in four steps:
1. Every enabled joint's goal is set to its present position, so motion stops at once.
2. Each joint's Torque_Limit is lowered to 0 in 10 steps over 2 s, so a gravity-loaded arm settles instead of dropping.
3. Torque is turned off.
4. Saved settings are restored.

Exceptions:
- Communication faults release immediately, as before, and so does any failure while easing; `last_release_mode` records which happened.
- Wheels still stop and release immediately.
- A closed gripper also eases open, so a held object is still let go.
- The client waits up to 6 s for a STOP to be confirmed.
- The 12 V switch remains the hard stop.

## Automatic calibration tool (2026-10-07)

`robot_auto_calibrate(arm, velocity=300|200, user_confirmed_clearance)` runs LeRobot PR #3282 for one arm through the pinned runner `software/scripts/carton_robot/upstream_pr3282_calibration.py --execute --install` (the unchanged upstream sweep, then validation; see `software/docs/auto-calibration.md`). It runs as a detached job (`calibration_job.py`, started via `remote_admin.start_calibration`):

1. Precheck: no motor enabled, no motion running, and a phone frame younger than 10 s.
2. Stop the hardware owner so the runner can open the servo ports.
3. Run the sweep.
4. Handle the result:
   - If it validated, it is installed.
   - If it ran but was not installed, the previous offsets, limits and position mode are written back into the six servos with torque off and read back. Without this, a failed run leaves the servos disagreeing with the file, which is how the left-arm mismatch arose.
   - If torque release could not be verified, stop and ask for 12 V off; the server is not restarted.
5. Restart the robot server.

`robot_stop` sends SIGINT to a running sweep; the upstream routine makes the motors limp. `robot_get_calibration_job` reports the phase, log and outcome. The tool refuses unless `user_confirmed_clearance` is true. Its description tells the pilot to show the clearance checklist and get an explicit yes. One job runs at a time, whether deploy or calibration. `test_calibration_job.py` covers it; it has not been run on hardware through the API.

## Both arms and calibration restore (2026-10-07)

- **Both arms.** The restart script now starts the owner with `--both-arms --paddle-profile --wheels`; `--right-arm-only` gives the old scope. A pickup command needs all six joints of the arm it moves enabled, but not the other arm. The pickup profile settings and guards apply to each arm.
- **Mismatched arm.** If an arm's saved calibration does not match its servos, that arm stays read-only (`scope_reduced` in status) and the other arm works. A single-arm scope with a mismatch, or a mismatch in every scoped arm, still refuses startup.
- **Restore.** `robot_restore_calibration(arm)` is a no-motion job. It stops the owner, writes the saved calibration file's homing offset, limits and position mode into that arm's six servos with torque off, reads them back, then restarts. This fixes the left arm's 6 October mismatch (four servos left holding the rejected candidate). `test_both_arms.py` covers this.

## Head scope (2026-10-09, OAK on the head)

The OAK-D Lite now sits on the two-servo head (slot cradle on the stock tilt link); the old USB head camera is gone.
The restart script starts the owner with `--head` as well (`--no-head` leaves the head read-only).

- **Enable/release.** `robot_set_motor_enable` accepts `head_motor_1` (pan) and `head_motor_2` (tilt), alone or
  together. They hold where they are, like arm joints. They are not part of any arm's six-joint rule, and an arm move never needs them.
- **Moves.** Only `robot_move_head {positions, duration_s}` (owner op `head_move`, `head_joint_executor.py`):
  - at most 200 ticks per joint per move;
  - `duration_s` at least 1 s per 100 ticks of the longest travel (default: that minimum, at least 1 s);
  - targets inside the saved range minus 40 ticks;
  - no head move while another motion runs;
  - head names in `direct_joint` targets are refused.
- **Guards.** These are the arm guards: the 40-tick ramp, the 96-tick following error (fault and release), the
  contact halt (load ≥ 350, stalled, ≥ 50 ticks behind), the 1 s watchdog, the phone-feed gate and the 120 s idle lease.
  The head's torque limit and load fault level are 500 (arm joints: 800).
- **Calibration mismatch.** A head calibration mismatch, or a head missing from the answering buses, leaves the head
  read-only (`scope_reduced.head`); the arms keep working.
- **Capabilities.** `robot_get_capabilities` reports `head_supported`, `head_motors` and `head_move_limits`.
- **Tests.** `test_head_scope.py`.
