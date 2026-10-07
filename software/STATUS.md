## Left bus stuck "port busy"; owner now recovers it — 7 October 2026, 20:09–20:12

At 20:09:33 the left bus (`/dev/cu.usbmodem5B790186401`) stopped answering. From then on every owner
poll failed instantly on left shoulder_pan with `Coherent servo read communication failure: -1` (Feetech SDK
COMM_PORT_BUSY, about 20 µs, nothing sent on the wire). The owner soft-released and retried about 35 times per
second (3,300 stops), and the telemetry of all 16 motors stayed frozen. Power-cycling the left arm did not help.
Every motor was already released and no bus was missing. A remote restart at the same commit (d8a4432,
job 20261007-201121-bc01a0) brought all 16 motors back, 12 of them commandable, with fresh telemetry and no errors.

Cause: the SDK sets `port.is_using` at the start of a transaction and clears it only on a normal return. A serial
exception in between (a USB glitch) leaves it set, so every later transaction returns -1 forever. The original
exception had already scrolled out of the log tail that `/admin/logs` returns.

Fix: in `gemma_hardware_owner.py`, `recover_ports` runs after a -1 failure. It clears the stale flag (and reopens
the port if its buffer cannot be reset), releases any motor still powered, and records `port_recoveries` and
`port_recovery_count` in status. `strict_servo_replies.guard_replies` also clears the flag when a write or read
raises. The owner loads that module from the utility checkout, so on the robot the owner-level recovery is what
applies.

## Tag registration binding robustness (software only) — 7 October 2026

No hardware touched. `robot_get_registered_tags` no longer refuses after an OAK publisher restart or a cart move:

- A registration is bound to the camera id and its geometry hash (resolution, projection, intrinsics, distortion), not the OAK `stream_id`. After a restart in the same mode, the first read must pass the gripper-consistency check (tag 2 versus the arm model, 4 mm / 2°), and the event is recorded. Switching `--wide` on or off changes the projection and is refused by name.
- If table tag 1 moved (cart move, base pulse, tag bumped) while the head ticks are unchanged (3 ticks) and the gripper-consistency check passes, the new tag-1 pixels become the reference (`table_anchor_re_anchored`). A head move is still refused, as are model, tag-geometry, mount and arm-calibration changes; each refusal names the cause and the fix.
- Re-anchor and stream events go in `tag-registration-binding.json` next to `tag-registration.json`, which stays unchanged. A new registration deletes the binding file.
- Wide (`--wide`) tag poses now undistort corners with a converged iteration before IPPE. OpenCV's built-in 5-iteration undistortion left up to about 1 mm error near strongly distorted image corners. The OAK manifest now records `distortion_model`, and anything other than Perspective is refused.

## Read-only paddle target tool (software only) — 7 October 2026

- Added `robot_get_paddle_target` to the chat-side calibration wrapper (`farm/perception/paddle_target.py`), next to `robot_get_registered_tags`. It returns the paddle tag 3 pose in the right arm base, with uncertainty, frame IDs and age. It refuses with the registered read's own reason when no passing registration exists, and refuses frames that are too old.
- The grasp point, approach direction and jaw tool poses stay withheld until the owner records `paddle_grasp` in `.private/apriltag-geometry.json`. That means the tag-3-to-handle offset (printed tag frame, mm) and `gripper_from_jaw_contact`, each with a tolerance and a source. Both are **unmeasured**. The section is excluded from the geometry hash, so recording it does not invalidate a registration.
- A `robot_plan_reach` pre-grasp proposal is included only when the planner is configured and uses the same jaw offset and calibration. It is labelled "proposal, not executed, not collision checked". The rule that planner output is not sent to the motor owner is unchanged; that decision still belongs to the owner. The pilot's reach procedure (small segments with existing tools, re-detect tag 3 and check cameras after each) is in [docs/gemma-automatic-calibration.md](docs/gemma-automatic-calibration.md#paddle-target-read-only). No motion limits changed.
- Tested on rendered MuJoCo tag images through the production detector (`tests/test_paddle_target.py`): grasp point within 1 mm of scene truth. No hardware was touched; no physical registration had been fitted as of the last hardware record.

## Tag-registration mover checked against today's robot server — 7 October 2026 (no hardware)

`qwen-bridge/test_tag_registration_contract.py` runs CalibrationRobot → `run_calibration(..., 'registration')` through TagRobot and the real `gemma_robot_tools` dispatch, DirectJointClient and HardwareOwner (`--both-arms --paddle-profile --wheels`, 2 s soft release) on a fake bus, with rendered OAK frames. It is in the restart script's test list. The first run against the 6 October mover failed; the adapter (`carton/servo/gemma.py`, `tag_calibration.py`) now:
- enables all six right-arm motors (the owner refuses moves otherwise; the jaw only holds);
- refuses steps under 3 ticks before dispatch (the server answers ≤2 ticks with a no-op), sends `duration_s` 0.4;
- reads status as one `robot_get_execution` (it carries the owner's 16 rows) and keeps two round trips between a frame and the next command (at 150 ms the old pattern exceeded the 1.0 s frame age on the first step);
- detects STOP/faults by `stop_count` (no latch), names `closure_outcome` on `completed: false`, and confirms a soft release from fresh reads after `robot_stop`.

Limits are unchanged. Registration makes ~530 relay calls: ~125 s at 150 ms RTT, ~178 s at 250 ms; slower links exceed the 180 s budget (`limits.max_seconds` may go to 300). Before a physical run, measure the link read-only with `tools/measure_robot_link.py --pilot-root "$PILOT"` (RTT, robot−chat clock offset, projection). Unverified on hardware: holding Present_Load (<500 required on all 16 rows), Moving/velocity noise while holding, ≤3-tick drift of held joints during a step, endpoint within 5 ticks after a 16-tick step, and OAK capture latency versus the encoder bracket.

## Right arm recalibrated via robot_auto_calibrate — 7 October 2026, 19:50

Job 20261007-195000-fd5abb validated and installed; the owner's servo-versus-file check shows no mismatches. The 18:19 attempt had failed validation only because left/right pan travel differed by 48.9°, against the old 191° left pan; after the left recalibration (238.5°) the right run validated.

| Joint | Old (homing / range) | New (homing / range) | Travel |
|---|---|---|---|
| Pan | −57 / 679..3415 | −51 / 673..3421 | 241.5° |
| Lift | 785 / 826..3268 | 785 / 824..3270 | 215.0° |
| Elbow | 634 / 932..3162 | 634 / 932..3162 (unchanged) | 196.0° |
| Wrist flex | −770 / 929..3165 | −777 / 924..3170 | 197.4° |
| Roll | 790 / 121..3973 | 798 / 130..3964 | 337.0° |
| Gripper | 329 / 1270..2824 | 329 / 1269..2825 | 136.8° |

Homing shifts are at most 8 ticks (under 1°), so the recorded paddle sequence is effectively unchanged. Any future tag registration must use this calibration. Persistence across a 12 V power cycle is unconfirmed, as for the left arm.

## Left arm recalibrated via robot_auto_calibrate — 7 October 2026, 19:43–19:46

Job 20261007-194349-22d282 ran pinned PR #3282 at velocity 200, started by the owner from the chat. Evidence is in `work/calibration-runs/left-20261007-194349` on the robot Mac.
- **Outcome.** The routine completed, release was verified, the range was validated with no problems, and the result was installed into the saved file. The job's server restart then loaded it, and the owner's servo-versus-file check passed (no mismatches).
- **New left values** (homing / range, travel):

  | Joint | Homing | Range | Travel |
  |---|---|---|---|
  | Pan | −49 | 690..3404 | 238.5° |
  | Lift | 969 | 847..3247 | 210.9° |
  | Elbow | 109 | 944..3150 | 193.9° |
  | Wrist flex | −819 | 901..3193 | 201.4° |
  | Roll | 1319 | 111..3983 | 340.3° |
  | Gripper | 119 | 1273..2821 | 136.1° |

  These are consistent with the previous verified run, and pan now matches the right arm (about 240°).
- **Not yet confirmed.** Whether the values persist across a left-side 12 V power cycle. Earlier today left homing offsets read 0 after a bus/power event that interrupted a restore. Confirm by power-cycling the left side and checking for mismatches at the next owner start.

## Outage and right-arm calibration attempt — 7 October 2026, 18:19–18:35

- **18:19, right-arm auto-calibration.** The pilot ran `robot_auto_calibrate` on the RIGHT arm at velocity 200, stopping the owner. The runner exited 2: the candidate was retained but not validated. Whether the job's register restore completed was not read before the API went down. Check `/admin/job?id=20261007-181916-761f48` and the right arm's calibration mismatches before any right-arm motion. The right arm's calibration was meant to stay frozen (pickup sequence, tag registration).
- **18:19–18:29, left bus failures.** The left-arm bus (`/dev/cu.usbmodem5B790186401`) repeatedly failed servo reads (−6), and the owner exited.
- **~18:33, server down.** A remote restart could not start the owner (the left bus did not answer) and aborted before starting the API, leaving the robot unreachable remotely. Fixed in code:
  - The restart always starts the API even if the owner fails.
  - With `--allow-missing-bus` (the default), the owner starts on the buses that answer, e.g. right arm and wheels, and reports `missing_buses`.
- **Left wrist camera.** It fails with AVFoundation "Cannot Use USB2.0_CAM1 … stop any other actions using" it: another app holds it. The Codex app's video-capture service is a candidate.

## Cameras kept alive by the restart script — 7 October 2026, 18:10

- **OAK.** It went stale because `farm.oak_camera stream` exits after `--seconds` and nothing restarted it. The camera step now restarts it when stale: a 24 h stream, `--usb2`, into the API's OAK folder, using a Python with depthai found under the Codex workspaces and remembered in `work/oak-python`.
- **Left wrist.** It froze again mid-session, then streamed again after the publisher was restarted. Treat it as intermittent USB until the cable is reseated.
- **Tools.** `robot_restart_cameras` lets the pilot restart stale streams with no motors involved; `/admin/processes` lists the robot services remotely.

## Left arm calibration restored; both arms movable — 7 October 2026, 17:45

Done remotely from the chat Mac: `/admin/deploy` to 2e887dc, then the `robot_restore_calibration(left)` tool. No motion.
- **Before.** The owner, now started `--both-arms --paddle-profile --wheels`, demoted the left arm to read-only for the 4 known mismatches (shoulder lift, wrist flex, wrist roll, gripper still holding the rejected 6 Oct upstream-300 candidate).
- **Restore.** It wrote the saved file's validated velocity-200 values into all six left servos with torque off and read them back. All six match: pan −22/959..3135, lift 975/855..3239, elbow 107/942..3152, wrist flex −826/898..3196, roll 1318/115..3979, gripper 121/1275..2819, all in position mode 0.
- **After the automatic restart.** There are no calibration mismatches and 12 arm motors are commandable. Left joints rest folded inside their ranges, with elbow 3128, gripper 1302 and lift 918 near their limits; enabling there is allowed and moves can only head inward.
- **Not yet tested.** No left-arm motion has been run since. Validate with small moves while watching. Its last full-sweep travel (191/210/194/202/340/136°) is the reference.
- **Wrist cameras.** Both streamed after the restart; the left wrist has frozen before.

## Automatic calibration as a robot tool — 7 October 2026

Software only; not yet run through the API. `robot_auto_calibrate` wraps the pinned PR #3282 runner as a background job: owner stops, sweep, validate/install, otherwise restore the previous servo registers, then the server restarts. STOP interrupts the sweep. It is the intended way to fix the left-arm mismatch from the chat. Prior runs on this robot hit the cart and produced short ranges, so the clearance checklist and someone watching are mandatory. Record each run's result and evidence folder here.

## Wrist cameras through the API — 7 October 2026, 16:59

Deployed remotely with `/admin/deploy` (cameras-only, commit e7716dc); no motors involved.
- **Cause of "no publisher output".** The restart script pointed at the wrong source path for `capture-single`, so the build failed. Fixed in e7716dc.
- **Camera IDs.** They are unchanged: right `0x12200005a39230`, left `0x12140005a39230`, head USB `0x12400005a39230`. The Mac also lists its built-in camera.
- **Right wrist.** Streams 640×480 through `robot_get_cameras` (identity verified). The frame shows the white claw jaw, the cart basket and the floor.
- **Left wrist.** It started, then stopped delivering frames within about 5 s (stale 18–28 s on later reads). This is the same freeze as `docs/Left-Wrist-Camera-Diagnosis.md`: reseat its USB cable or connect it directly to the Mac, then run `--cameras-only` again.

## Remote administration of the robot server — 7 October 2026

After the next manual `./restart-robot-server.sh` (motors released), the robot API exposes `/admin/logs`, `/admin/deploy` and `/admin/job` behind the pinned client certificate. The chat Mac can then read the restart, owner, API and camera logs, and deploy a pushed branch or commit with a restart, without anyone at the robot. A restart that fails to come up rolls back to the previous files. Remote restarts still refuse while motors are holding.

## Contact guard — 7 October 2026

Software only; fake-hardware tests pass. There is no self-collision model, so joint limits alone let the arm reach into the robot's own parts. Now, if an arm joint shows load ≥ 600 while lagging ≥ 20 ticks behind its command, the motion ends `contact_halt`: that joint stops pushing and holds, and nothing is released. Settle corrections never push a loaded or non-moving joint. 600 is a first guess; record real contact and false-halt loads here to tune it.

## Wrist cameras configure themselves — 7 October 2026

`./restart-robot-server.sh` now sets up the wrist cameras itself, with no separate commands; `--cameras-only` does just this step. In order, it:
1. Builds `work/capture-single` if it is missing.
2. Lists the cameras the Mac sees.
3. Keeps configured wrist IDs that are still present.
4. Re-matches a wrist whose ID moved. The same hub port path on another USB bus counts as the same camera. Anything else is assigned and marked `identity_verified: false`, and the chat flags it on each image.
5. Saves the result to `work/wrist-cameras.json`, which the API reads.
6. Stops stale capture processes holding a wrist camera, starts the streams, and restarts only the API if the IDs changed.

Earlier "no publisher output" was not a camera-permission problem: the script was run from Terminal. A stale pinned ID is the likely cause, and the next run will print which.

## Continuous monitored motion and chat sessions — 7 October 2026

Software only; fake-hardware tests pass.
- **Robot API.** The 20-move limit is gone. Arm moves can run continuously through waypoints, start without waiting (`wait=false`), be watched with `robot_get_motion` and cameras, be halted in place with `robot_halt_motion`, or be re-aimed mid-motion with `replace=true`. Wrist cameras: run `./restart-robot-server.sh --cameras-only` from Terminal on the robot Mac. The earlier "no publisher output" most likely came from a restart run without Terminal's camera permission.
- **Chat Mac.** The pilot UI at `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot`, which is not in this repo, is now one persistent session:
  - Full history is kept; only old images and old bulky tool outputs are trimmed.
  - Each message allows 60 steps; "continue" resumes.
  - Controller refusals go back to the model instead of ending the request, pausing after 3 in a row.
  - A **New session** button archives the conversation to `.private/sessions/`.
  - Backups of the previous `chat_server.py`/`chat.html` are in `.private/backup-*`.

## First API use of the new owner — 7 October 2026

The new owner was deployed and reached from the Qwen chat. Two of its rules blocked the first real requests, and both are fixed on `main`:
- **Arm enable refused.** It returned "right_arm_shoulder_lift: current position outside saved travel margin": the shoulder rested within 40 ticks of its limit, so the arm could not be enabled. Enable now only needs each joint inside its saved range; moves from the edge can only head inward.
- **Base pulse faulted.** A "move the wheels back" pulse drove and braked, then faulted with "Wheels rolling after release". The cause was very likely spurious released-servo velocity readings, not real rolling, but this is unconfirmed. The check now uses encoder position only and reports the measured values.

To pick up the fixes, release the arm and rerun `./restart-robot-server.sh` from an updated `main`. If "rolling after release" appears again, record the numbers in its message here.

## Full-scope owner startup failed — 7 October 2026

Reported by the robot-Mac session. A restart was attempted with a full-scope owner (all 16 motors commandable), and it failed:
- Startup hit the existing left-arm saved-versus-hardware calibration mismatch. That is shoulder lift, wrist flex, wrist roll and gripper; the servos still hold the unvalidated 6 October upstream-300 candidate, while the file holds the validated velocity-200 calibration.
- The owner then hit telemetry communication faults during startup.
- The process ended stopped and latched.

Final safe state:
- All 16 motors released, `Torque_Enable = 0` on every motor.
- Zero motor writes after the release.

The right-only pickup owner (`--right-arm-only --paddle-profile`) remains the known-working profile. Full-scope startup stays blocked until the left-arm calibration is deliberately resolved: restore those four joints' offsets/limits from the validated file with torque off, as was already done for pan and elbow, or recalibrate.

This was not the new server on `main`. `./restart-robot-server.sh` always starts the owner with `--right-arm-only --paddle-profile --wheels`, and the owner on `main` (25c57bc) has no STOP latch, so a full-scope start ending "latched" ran the previously installed owner code. The new owner (simultaneous joints, settle corrections, no latch, base drive, wrist cameras) is still undeployed. To deploy it: release the arm, then from a checkout of `main` in Terminal run `./restart-robot-server.sh --dry-run`, then `./restart-robot-server.sh`. Check that the summary shows `execution_profile: paddle-success-v1` and `base_drive_supported: true`. The cause of the startup telemetry faults is not established.

## Qwen owner: no STOP latch, base drive — 7 October 2026

Software only; not yet run on the robot. Fake-hardware tests pass.
- **No STOP latch.** A STOP or owner fault still releases every motor, but a restart is no longer needed afterwards. Motors stay off until an explicit `robot_set_motor_enable`. If turning torque off fails, enable stays blocked until a later STOP confirms the release.
- **Base drive.** `robot_move_base` runs guarded wheel pulses ported from the validated `drive-pulse.py`:
  - each wheel at most 0.02 m/s, at most 3 s per call
  - the wheels are released and their settings restored after every pulse
  - a stale phone feed brakes the pulse early
  - STOP or a fault stops the wheels first

  The restart script starts the owner with `--wheels`. If the owner process is killed hard mid-pulse, the wheels keep turning until 12 V is cut. Record the first API base pulse here, with measured travel against the encoder estimate.

## Pickup owner: simultaneous joints, settle corrections, wrist cameras — 7 October 2026

Software only; not yet run on the robot. The code was changed in [qwen-bridge](docs/commissioning/2026-10-07-paddle-success/qwen-bridge/PADDLE-PROFILE.md) and tested on fake hardware:

- One pickup command can now move several right-arm joints together. A closing gripper still runs alone.
- A joint that comes to rest short under load gets at most 3 goal corrections (at most 57 ticks of overdrive). If it is still short, the move ends `settled_short` with the motors holding. Before, the owner faulted at the deadline, released everything and latched STOP. This is what happened when a joint ended 58 ticks short with zero stable samples. A joint that never comes to rest still faults.
- `robot_get_cameras` can return `left_wrist`/`right_wrist` from native capture manifests.
- `robot_get_handoff` now serves `physical_pickup_procedure`.

The paddle files that commit d364773 dropped from the repo are restored. On the robot Mac, run `./restart-robot-server.sh --dry-run` from the repo root in Terminal, then run it without the flag. It replaces the API and owner (all motors released, STOP clear) and starts the wrist publishers. Record the first physical multi-joint move and any `settled_short` here.

## Verified physical right-arm paddle pickup — 7 October 2026

The right claw picked up the white paddle, followed through two bounded lifts, held it visibly above the table, then lowered/released it and withdrew. The operator explicitly confirmed the lift. Cleanup verified all six right-arm torque releases and restored the original RAM controller settings. This is one successful observed trial, not general autonomy or repeatability.

[Exact successful process, settings, failed attempts, telemetry, and Qwen integration requirements](docs/commissioning/2026-10-07-paddle-success/README.md). The success used camera-guided encoder commands with temporary shoulder/elbow P32, a measured-resistance jaw close, current right-arm calibration, and an earlier 5.43 cm forward cart reposition. No saved calibration or travel limits were rewritten. Approximately 40° wrist movement was separately observed; the requested 40 cm cart travel remains incomplete.

The Qwen tool server now has a motion-capable right-only owner, all16 released and zero recovery motor writes. Normal full-scope startup refuses four left-arm saved-versus-hardware calibration mismatches; they remain read-only. [Actual server commands, relay update, verification and integration gaps](docs/commissioning/2026-10-07-paddle-success/QWEN-SERVER.md). Do not treat a live API or this historical pose sequence as motion readiness.

## Bare-claw parallel simulation progress — 7 October 2026

Offline only; no physical camera or robot accessed. Three isolated workers now
search frozen controller variants and noise seeds concurrently while recording
all state samples and every applied contact step. The latest fixed profile
folds the near major to about 40°, folds the far major to 35°, withdraws the
right hand, and observes five seconds of passive far-flap retention in 3/3 seeds.
All three full intervals pass independent applied-contact scoring. A subsequent
extension also parks the left hand and retains both partial major folds for five
seconds in 3/3 seeds, with independently clear applied contacts. Both shorts
remain open. **Full four-flap closure,
tape application and hands-clear retention are incomplete.**

The first bounded short-regrasp extension passes the same partial release in
3/3 seeds but completes the +10° short target in 0/3. Two runs refuse predicted
left wrist collisions, and one loses fresh table registration. One advances
both shorts several degrees before the refusal. Executed robot and panel
contact intervals independently pass; this does not certify the unexecuted
stroke. The explicit additional-view open-short observer passes 15/15 frozen
pixel replays and refuses 3/3 current-surface occlusion negatives. See
[the observer scope](docs/carton-open-short-vision.md).

The source-coherent 0.5 mm / 0.25° contact variant also completes 0/3 short
probes with the original front camera and 0/3 with the hypothetical left/back
view. The new stops are missing primary carton registration or cross-view
angle disagreement; all six executed contact intervals pass both audits.
No full fold is inferred from the improved geometry or camera components.

The explicit primary-carton-absence fallback now accepts fresh secondary
geometry after verified startup, current primary anchors and visible housing/FK
checks. Its next full-prefix batch still completes 0/3 short probes: fresh
far-angle disagreement, an 80-command approach limit, and missing left-short
geometry. Five partial observations use the fallback; all three original
partial-release prefixes remain exactly identical. Executed robot and panel
contacts pass their scoped audits. Seed 2 shifts the carton 10.79 mm and reports
9.68 mm of XY table-edge overhang; the legacy clearance field is not a vertical
penetration measurement. Broader software regression: 808 passed, two optional
skips. No full closure, taping, or robot execution is established.

The explicit setpoint-feedback V3 variant removes repeated measured-position
servo-offset accumulation. Its three full-prefix trials finish in 44.50 wall
seconds and preserve the exact partial-release prefix, but all stop on the
unchanged predicted left-wrist/short penetration gate before reaching the short
target. Executed robot/panel contacts pass their scoped audits. Actual refusal
RGB-D is now saved and byte-verified for both views in all three runs. Current
regression: 866 passed, two optional skips. The next geometric task is a clear
claw approach/orientation; full folding and retention remain incomplete.

The fixed station camera loses far-plane visibility around 39°; successful
partial release at 35° does not establish continued visibility or closure.
Actual station/material/camera calibration and deployment remain unverified.
Read [the current carton handoff](docs/carton-bare-claws-handoff-2026-10-07.md)
for reproducible commands, preserved failures, robot/Gemma software boundaries
and the distinction from published historical hardware calibration below.

## Left retry05: short shoulder and elbow ranges — 6 October 2026

User confirmed stepped out of frame. Fresh feed showed clear sweep; guarded left retry05 ran atvelocity100 with outward swivel retained. Measured shoulder47.1 degrees (raw endpoints3685..125 wrapping), elbow57.5 degrees (1784..2439); routine rejected shoulder below120 and elbow below115. All six left releases verified. No definite cart strike identified in sampled fresh phone frames; actual cause remains unproven. No completed calibration saved. Added load/voltage/current fields from the existing coherent15-byte sample to limit-wait trace for later diagnosis; no new writes, additional motor reads or threshold changes. Evidence at current task outputs/auto-calibration-left-retry5-2026-10-06.

## Left calibration interrupted for person in sweep — 6 October 2026

After user confirmed clear, left guarded retry04 began from outward pan1759. Wrist unfolding reversed successfully; shoulder and elbow unfold succeeded. A fresh phone frame showed a person's head immediately below/beside the claw while it was moving. Sent Ctrl-C; vendor returned130; guarded launcher aborted and verified all six left-arm torque releases. No completed calibration saved. Backups/evidence in current task outputs/auto-calibration-left-clearance-2026-10-06. Require everyone outside the arm sweep before resuming powered motion.

## Further camera-guided swivel steps — 6 October 2026

User widened phone framing. Left bounded pan steps continued:1918 original to1759 current (13.97 degrees decreasing overall). Step08 refused before motion on transient released elbow velocity50/Moving1; after10s settling step09 passed. Steps10 and11 moved23 and24 ticks respectively but settled11/10 ticks short of their targets at the3s deadline. Both verified torque release and prior settings restoration. Step10 load max80, status0, voltage119; no confirmed physical contact. Subsequent read-only pan: torque0, position1759, goal1749, CW/CCW dead zones1, P16. Do not interpret endpoint misses as successful positioning or widen checks/raise torque to force movement. All12 arms checked torque0/status0 after step10. Per-step evidence in current task outputs/left-swivel-clearance-step-07..11.json. No new arm calibration saved; both-arm goal incomplete.

## Camera-guided clearance investigation — 6 October 2026

After the phone was moved to the opposite side, a task-local bounded shoulder-pan helper ran six small left swivel steps. Each command <=34 encoder ticks (2.99 degrees), torque limit300, position speed100, acceleration5, strict coherent fault/load/travel checks and3s deadline. Other joints remained released. Every step reached its tolerance and verified selected-joint torque release and prior speed/acceleration/torque restoration. Left pan moved from1918 to1859 overall (5.19 degrees decreasing), after an initial increasing-direction probe and reversal. Current pan homing0; no calibration offsets/limits changed by this helper. Camera showed decreasing pan moving the hanging forearm outward. Further rotation stopped because the elbow reached the camera's right edge; requested wider framing. No new full calibration run and no completion claim. Per-step evidence: current task `outputs/left-swivel-clearance-step-01.json` through`06.json`.

## Folded left-arm retry and diagnostic — 6 October 2026

After user fully folded both arms, strict preflight passed all12 arm status/torque checks. Fresh phone frames inspected, start held until hands cleared. Left guarded velocity100 retry again rejected elbow41.1 degrees (limits2806..3274; minimum115). Shoulder measured179.8 degrees. All six left motors verified released; no completed calibration saved. Evidence at `/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/outputs/auto-calibration-left-folded-2026-10-06/`.

Subsequent read-only elbow settings: both torque0, velocity mode1, homing0, min0/max4095, torque limit800, P16. Thus narrow stored encoder limits do not explain the short sweep. No bypass or widened limits applied. Await supported power-off manual elbow freedom/clearance check before additional powered sweeps. Both elbows currently remain in velocity mode from incomplete calibration; ordinary position operation requires deliberate restoration or successful calibration, not blind torque enable.

## Left-arm retry after USB/12 V reset — 6 October 2026

User reset left USB and12V. Fresh strict precheck: all12 arm motors status0, torque0. Fresh phone-camera frame was inspected from `/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera/` (not the stale older task stream). Guarded left retry at velocity100 unfolded successfully but rejected elbow travel41.5 degrees against115..225 degrees; shoulder measured139.3 degrees. Routine returned1, no completed calibration saved. All six left-arm torque releases verified. Evidence/backups in `/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/outputs/auto-calibration-left-retry-2026-10-06/`. Neither arm newly calibrated; partial homing/limit changes may remain. Do not accept narrower obstructed travel or expand safety limits. Need elbow mechanical/clearance inspection before further powered sweeps.

## Right-arm automatic calibration attempt — 6 October 2026, follow-up

User requested the right arm while left wrist retained fault 8. Guarded right-arm velocity100 calibration unfolded wrist, shoulder and elbow, but rejected elbow range 3538 ticks /310.96 degrees against the existing225-degree maximum. Second elbow leg tracked600 ticks of negative travel; this does not establish a full mechanical range. No completed calibration saved. Cleanup verified all six right-arm motors torque off. Backups/evidence: `/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/outputs/auto-calibration-right-2026-10-06/`.

Before this run, an independent read confirmed all12 arm motors torque0 (including faulted left wrist). User reported unplugging/replugging left arm during the right run. Subsequent left-arm precheck received no packet from ID1 (communication-6); no left retry started. Both arms remain incompletely calibrated with possible partial EEPROM changes. Do not use saved calibration as evidence of matching hardware. Need restored left communication and physical inspection of right elbow/clearance before more sweeps.

## Automatic calibration attempt — 6 October 2026, this chat

The user requested automatic calibration and confirmed all clearance conditions. Read-only preflight received replies from all 16 servos. The guarded left-arm PR3282 launcher ran at velocity 100 with a 45-second leg timeout. Wrist-flex unfolding stopped short in both directions, then the launcher aborted on `SDK unexpected servo fault 8` from motor ID 4. No completed calibration was saved. Partial homing/limit changes may exist; the old saved calibration must not be treated as matching hardware.

Cleanup verified torque off for left shoulder pan/lift, elbow, wrist roll and gripper. Wrist-flex release could not be confirmed because its reply retained fault 8. The user was instructed to switch 12 V off immediately. Right-arm calibration was not started; head and wheels were not commanded. Preserve register backups and inspect the wrist before another attempt; do not bypass the fault. Evidence is in `/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/outputs/auto-calibration-left-2026-10-06/`.

## Handoff stop condition — 6 October 2026

After updating main and reconciling local changes, the requested read-only `farm servo-protection -p paper-tray-v0` check returned no status packets for the servos while `lsof /dev/cu.usbmodem*` was empty. Hardware work stopped at that read failure. No protection-register writes or power-cycle verification were performed. The earlier successful table below is dated evidence, not a fresh post-update success; do not report “ALL DONE” or temperature sensing removed. Calibration remains paused, motors were last verified released, and the cause of the silent read is not established.

The read-only CLI now skips calibration handshake writes and disconnects without changing torque. A failed second-bus connection closes the first successfully opened bus.

## Square-carton proposal and observed spring-back — 2026-10-06

The new [square-carton audit](docs/carton-square-minor-progress.md) keeps the
empty free box, resistant hinges, original limits and robot/table distance.
With fresh hinge-tag recovery, claws fold both short flaps while the box moves
only 0.033 mm horizontally. Releasing the left hand reopens its flap from
91.19 to 57.87 degrees after five seconds. The paddle reaches 62.68 degrees
and stops at the unchanged IK bound. This is a separate setup proposal, with
new floor-tag mounts and rendered 1280 x 720 sensing; none is physically
calibrated. All 105 focused folding tests pass. Both complete four-flap
sequences, tape application and arms-clear retention remain incomplete.
No hardware was accessed.

## Major-flap transfer and tape component — 2026-10-06

The new near-major press attempts fail with the same empty free carton and
resistant hinges: claws move the carton 63.94 mm before a penetration stop;
the paddle trial moves it 78.14 mm before losing fresh registration. Neither
has a completed major fold. The [support audit](docs/carton-raised-minor-support.md)
records those failures without weakening friction, resistance or motor limits.

An isolated [passive tape component](docs/carton-tape-material-audit.md) passes
small-load and finite-overload controls without a weld or hidden actuator.
Full-strip peel trials complete with Newton/implicitfast at smaller timesteps,
but their release timing is not converged. It is not integrated into robot
folding and cannot certify physical retention. All 97 folding tests pass.
Both complete carton sequences, tape placement and five-second arms-clear
retention remain unfinished. No hardware was accessed.

## Prepared minor support — 2026-10-06

The [raised-minor audit](docs/carton-raised-minor-support.md) adds an isolated
three-second test in which a passive paddle supports both short flaps around
75°, with 0.243 mm blade slip and 0.0064 mm box movement. This starts from an
explicitly prepared pose: it does not establish a folding approach or handoff.
A full approach reached the short flap but the forearm pushed the near flap,
the unbolted carton slid and the bracing grasp failed. Bare-claw bridge searches
have not found a clear candidate. Neither method has completed all four folds,
taping or arms-clear retention. Material and physical station values remain
unmeasured. No robot or camera hardware was used.

## Carton contact validation — 2026-10-06

The [contact audit](docs/carton-contact-clearance-audit.md) adds rejection of
invalid initial arm/flap overlaps and limits planned and executed intended
contacts to 1 mm penetration. Full original-layout replays pass these checks
at every 2 ms step: peaks are 0.369 mm for claws and 0.125 mm for the paddle.
Their scene XML and recorded states exactly match the preceding full GIFs.
All 92 folding tests pass; physical coefficients and motor limits are unchanged.

Neither variant completes the carton. Bridge-pose searches and a separate
lower-table proposal have not produced a clear hand-transfer path. A rotated
carton trial reached a grip but then lost its brace and slid over 10 cm. The
successful stage remains two short flaps held; release produces spring-back.
Full closure, taping and hands-off retention remain open. No hardware was used.

## Paddle end-contact progress — 2026-10-06

The [new matched audit](docs/carton-paddle-end-contact-audit.md) reproduces both
short flaps held by a left claw and a free paddle in the right jaws. An explicit
60 mm grip position and −71° initial grip rotation allow blade-end contact.
Maximum box translation is 4.96 mm, versus 12.81 mm in the bare-claw run.
The same empty free carton, resistant hinges, table position, forces and limits
apply. No weld, clamp, guide or tape was added; grip pickup is untested.

Both full GIFs include the release test: the left flap springs back to about
58° in both. Neither completes all four folds or hands-off retention. Planning
a jaw to hold the minor while the other folds the major has not produced a
validated dynamic hand transfer. Full closure and tape application remain open.

Fresh rendered paddle tags now register its blade-end TCP without moving the
original slip reference. Tracking verification measures the actual free tool,
not just its virtual gripper target. All 88 folding tests pass. Material values,
external camera and physical tag mounts remain unmeasured assumptions. No
hardware was accessed. This updates the old paddle result immediately below.

## Resistant carton hand transfers — 2026-10-06

The [new transfer audit](docs/carton-minor-transfer-audit.md) reproduces both
short flaps held by the bare claws: 92.44° left and 91.92° right, with maximum
free-box movement 12.81 mm. Releasing the left hand makes that flap reopen to
58.40° after withdrawal and a two-second wait. The camera independently sees
the spring-back. This is a partial stage, not a full folding success.

The matched paddle attempt still stops at 15 mm tool slip before completing
the first short flap. Its gripper contacts the near flap before the blade
contacts cardboard. Alternate approaches have not fixed that failure. Trials
transferring a claw to the near major flap exposed interference from the other
forearm and large box motion; no major-flap transfer is validated.

Both complete attempt GIFs include failures. The free carton, empty load,
resistant hinges, original motors and all physical coefficients are unchanged.
The new 6 mm path clearance affects a copied planning model only. Proposed
paddle tags now provide fresh offline RGB-D tool poses without changing the
grip-slip reference. Physical tags/mounts remain unverified. 80 focused tests
pass. All four folds, final release/retention and explicit tape application
remain unfinished; no hardware was accessed.

## Corrected carton physics and bracing — 2026-10-06

The [new audit](docs/carton-braced-folding-audit.md) supersedes the folding
success claims below. An all-inward starting pose interpenetrated neighboring
flap panels by about 11 mm. The initializer now rejects that pose. The default
starts separated panels without changing spring rests, friction or resistance.
Historical recorded passes must not be treated as validated folding results.

The matched corrected trials use an empty 272 g free carton and resistant
hinges. Neither completes all folds or hands-off retention. Bare claws brace
the box and fold one short flap, then stop at an unreachable grasp transition
(45.764 s; maximum box movement 12.19 mm). The right-hand paddle slips during
contact (32.366 s; box movement 26.45 mm). Full attempt GIFs include both failures.

A declared solver sensitivity check distinguishes soft-contact creep from
physical grip failure: the NoSlip preset holds the stationary paddle for 30 s
at both 2 ms and 1 ms timesteps; zero friction still fails immediately. Both
folding variants use the same preset. Regression tests confirm that the box
still slides and the flaps spring open, with unchanged physical coefficients
and motor limits. 65 focused tests pass. Parameters remain unmeasured.

The proposed external camera and 45 mm carton side tags 21/22 improve rendered
tracking; neither is verified on the real station. This remains an offline
RGB-D experiment with independent simulator contact checks and obstacle
snapshots. No hardware was accessed or calibration changed. Regrasp planning,
all remaining folds, explicit retention and tape application remain unfinished.

## Historical paddle versus claws comparison — 2026-10-06

[Four matched offline cases](docs/carton-paddle-comparison.md) compare bare
claws with a left claw and an actual-CAD free-body paddle in the right jaws.
Bare claws complete the loaded weak-crease reference, including withdrawal
and five seconds without hand contact. The paddle variant slips in that case;
neither method closes the empty resistant box. An ideal rigid tool attachment
also fails and is labeled as a separate diagnostic. No general tool winner or
physical validation is claimed. The paddle planner still has face-angle/reach
limitations, and neither strategy implements deliberate box bracing.

Existing paddle grasp work supplied a verified simulated initial grip pose;
the comparison does not simulate pickup at this station. Material, friction,
tool mass and station dimensions remain assumptions. Full-duration paired
GIFs include whole-robot and close views, with explicit failure timestamps.
102 focused tests pass. No hardware was accessed or safety limits relaxed.

## Historical unbolted carton and springback update — 2026-10-06

The [new free-box and material tests](docs/carton-free-box-springback.md) expose
limits of the previous passing GIF. The box was already a free body, but the
passing assumptions included 960 g contents, friction 0.7 and weak creases.
The loaded weak-crease reference still passes withdrawal and a five-second
hands-off hold. Eight empty/lower-friction/stronger-crease sensitivity variants
fail; the empty weak-crease box moves about 35 mm horizontally in recorded
frames. Separate explicitly preset-closed material tests show stronger creases
springing open without any hand contact. Those are not robot-folding successes.

Material/load/friction parameters and per-flap resistance are configurable.
Zero contents mass removes both weight and support. Overall success now also
requires retention during withdrawal and after hands release; held closure is
reported separately. All material values remain unmeasured. Stock fingertips
are still used, and no brace-and-paddle controller has been validated. RGB plus
aligned depth remains required for the folding controller; the material-only
probe does not use perception. 94 focused tests pass. No hardware was accessed.

## Historical two-hand carton folding simulation — 2026-10-06

The [corrected rear-cart simulation](docs/carton-diagonal-folding.md) closes all
four flaps with both arm bases **150 mm behind the table edge** and **60 mm above
the tabletop**. A fixed three-tray cart has collision geometry and 35 mm front
clearance. The box is rotated 30 degrees and translated so its nearest bottom
corner stays 10 mm onto the table. These dimensions are assumed, not measured
from the photos. Stock SO101 fingertip geometry is used; the white attachments
visible in the photos are still unmodeled.

Both real arm models contact two flaps each. All four passive flaps stay within
5 degrees of horizontal for every 2 ms step of a two-second hold. Final angles:
91.843, 91.843, 90.315 and 90.302 degrees. Forbidden robot penetration: 0 mm.
Nominal and +/-5 mm sideways box trials pass. Six negative controls reject a
success claim. A heavier depth-noise/dropout trial fails; reliability is not
established. The filled-box model uses contents to support the short flaps;
empty-carton closure, taping and hands-free retention are not demonstrated.

The strategy uses observed tags/depth, not a learned Molmo policy. Table tags
1 and 20 jointly register a stationary camera; fresh anchor geometry and carton
tag 10 are required during folding. A [supplemental ID 20 A4 sheet](docs/assets/bimanual-table-anchor-20-a4.pdf)
is included. Run `tools/simulate_bimanual_folding.py --strategy diagonal` with
the explicit dimensions in the guide. No hardware calls or calibration changes
were made. [Independent results and known failures](docs/evidence/bimanual-folding-rear-cart.json).

The original success put bases over the tabletop. It remains explicitly
historical under `--reference-layout` and is not evidence for the photographed
station. The [station audit](docs/carton-folding-station-audit.md) retains that
correction and the straight-box reach failures.

## Servo protection readback — 6 October 2026

Read-only verification at 2026-10-06 13:30:02 JST: all 16 servos responded and all torque-enable readings were zero. The servos' own EEPROM protection was still at the factory values below; `farm servo-protection --write` has not been run yet (see `ROBOT_HANDOFF_2026-10-06-servo-protection.md`).

| Servo | Temperature limit (°C) | Unloading mask | LED alarm mask |
| --- | ---: | ---: | ---: |
| left_arm_shoulder_pan | 70 | 44 | 47 |
| left_arm_shoulder_lift | 70 | 44 | 47 |
| left_arm_elbow_flex | 70 | 44 | 47 |
| left_arm_wrist_flex | 70 | 44 | 47 |
| left_arm_wrist_roll | 70 | 44 | 47 |
| left_arm_gripper | 70 | 44 | 47 |
| head_motor_1 | 70 | 44 | 47 |
| head_motor_2 | 70 | 44 | 47 |
| right_arm_shoulder_pan | 70 | 44 | 47 |
| right_arm_shoulder_lift | 70 | 44 | 47 |
| right_arm_elbow_flex | 70 | 44 | 47 |
| right_arm_wrist_flex | 70 | 44 | 47 |
| right_arm_wrist_roll | 70 | 44 | 47 |
| right_arm_gripper | 70 | 44 | 47 |
| base_left_wheel | 70 | 44 | 47 |
| base_right_wheel | 70 | 44 | 47 |

The calibration goal is paused at the user’s request. Both wheel actuators were verified with a bounded pulse, followed by zero velocity, torque release, and settings restoration; calibrated odometry was not established. Arm calibration remains incomplete: failed attempts changed some arm homing/limit registers, while the saved calibration file remains the previous version. Do not claim that saved arm calibration currently matches hardware. Complete guarded calibration or deliberately restore the recorded register backups before normal arm operation. The head was not calibrated or moved.

Legacy task-local `work/carton_session.py`, `work/temperature_confirmation.py`, and `work/left_claw_prepare.py` were renamed with `.retired-2026-10-06`; state and logs were retained. The guarded PR #3282 launcher now lives in `scripts/carton_robot/`, with strict packet validation, bounded limit seeking, and verified-release cleanup; its temperature cutoff was removed on 2026-10-06 like everything else that read servo temperature.

The earlier session notes below are historical; they do not supersede this readback or establish current arm calibration.

## Current verified state — latest hardware session

- Power connection restored (5 October 2026, supersedes earlier power-feed blocker): user exchanged both battery-side USB-C power connections after exchanging the barrel plugs. Fresh readback at1791165058: all16 respond, torque0, status0,11.9–12.2V; saved arm/head calibration matches. Both power leads therefore work in this arrangement; a negotiation/contact issue is plausible, but no failed component has been established. Keep the working connections. Right elbow remains outside saved range (3186 vs3092 maximum); user asked for a supported manual bend before controller startup. Supplied head/right-wrist capture publisher restarted as solePID64122, both coherent5.02fps streams passed20s audit. No new motion, grasp, or fold.

- Current carton hardware diagnosis (5 October 2026): the communication failure follows the original right-side 12 V power feed. After exchanging the two round power inputs, right-arm IDs 1–6 and wheel IDs 9/10 reply normally through the USB hub (11.8–12.0 V, 26–28 C, status 0, torque 0), while left/head IDs 1–8 no longer reply on the suspect feed. This isolates the failing path to that power lead, its supply output, or their connection; the individual component is not yet established. Direct USB and a read-only host-baud scan had not restored right-side replies. User is checking the suspect lead at its battery USB-C end. No motion was commanded. Right elbow is currently 3186, beyond its saved maximum 3092; calibration readback matches. Head video remains live; right-wrist video interrupted during USB changes is being restored. No current local visual model, paddle grasp, or carton fold is validated. Evidence: task work/carton-swapped-feeds-1791164281.json and outputs/Carton-Utility-Commissioning.json.

- Carton utility commissioning (4 October 2026): switched from chat-by-chat joint nudges to isolated worktree `work/carton-visual-controller`, pinned commit `0e17b22e5a00cfbfda09bcdf6cbd91abf3d93a46`. Live environment/calibration and dirty source preserved. Utility tests:58 passed,8 explicit optional solver skips; rendered simulation ALIGNED_ONLY in17 corrections, max2.0003px, no robot/contact physics. All16 pinned URDF/model assets SHA-verified. Isolated Python3.12 environment excludes system packages, but pinned dependency installation failed on package-server network/DNS errors, so actual solver tests did not execute. Supplied Swift camera publisher compiled and is the sole head/right-wrist publisher.20s audit PASSED:5.0226fps each, maximum gap0.19974s, pair skew0.23297s. No utility motor commands. All16 motors released at read-only preflight; right elbow3198 exceeds saved3092 by106ticks (~9.32deg). No further automatic recovery under this direction. Prepare/seed ran, inspect correctly refused absent a healthy active owner. Live feature experiment remains unvalidated: wood anchor failed, printed anchor tracked, then paddle reposition invalidated target seed. Head view crops the claw. User asked to support/reposition right elbow and tilt head camera down; reseed afterward. Evidence under isolated ignored `software/data-carton/commission-20261004-mac-01/` and `agent-validation-20261004-01/`; user summary in task outputs/Carton-Utility-Commissioning.json. No paddle grasp or flap fold achieved.

- Carton approach in progress (4 October 2026): no flap folded and no verified paddle pickup yet. Head aimed down to include the box and paddle; head raw tilt2476/pan2084 at latest read-only preflight. Task-local `work/carton_session.py` provides parked, single-arm raw encoder steps with STOP, calibrated travel checks, low speed/torque, motor health checks and automatic release. Runtime state is `work/carton-session/status.json`; inspect its timestamp and phase before connecting another process. Do not treat an older preflight as current torque state.
- Wired camera streaming works at640x480/5fps input using exact AVFoundation identities: head0x12400005a39230, right wrist0x12200005a39230, left wrist0x12140005a39230. Right identity was additionally confirmed by a wrist-roll step and matching image rotation. Task streamer currently opens head plus the working wrist, yielding roughly2.5fps per saved image; left wrist is not continuously active. Explicit format must be reapplied after session.startRunning(), which otherwise resets it. These facts supersede older camera-failure and high-aim notes below. Phone images are supplemental.
- User requested more tolerance for camera upload gaps. Stationary carton-arm controller now allows10s age before a new movement,15s while moving,30s holding; it uses both wired head and active wrist timestamps. Hardware load/travel/STOP checks remain active. Seven task-local controller tests passed. No wheel behavior changed.
- Cartesian carton integration remains unvalidated: default real adapter uses range-normalized motor positions, whereas ArmModel treats them as degrees. Automatic calibration also does not establish the model's physical joint zeros. Separately, vendored FK used the wrong lower-link angle; changed theta1+theta2-pi to theta1-theta2, with six roundtrip/workspace regression tests and22 relevant tests passing. This mathematical correction does not validate physical Cartesian control. Current hardware approach uses observed raw joint steps rather than that model or the ACT checkpoint.

- Latest head-camera port change SUCCESS (4 October 2026): exact USB identity0x12400005a39230 delivers a fresh native AVFoundation image, visually confirmed as head overview of open carton (task outputs/Carton-Scene/head-new-port.jpg). Keep this port. Prior hub-port failure0x12130005a39230 does not apply to this working connection. Camera numerical mapping is still unverified; view remains aimed high. No motors accessed during camera check.

- Head-camera hub retest after direct-adapter success (4 October 2026): user moved head back to hub. USB identity0x12130005a39230 is detected, but exact-ID native AVFoundation capture again received no frame within8s. Same capture method succeeded on direct adapter identity0x11100005a39230. Current hub connection is NOT a usable head feed. Preferred continuation: return head to verified direct adapter while wrists stay on hub. Hub/port/power/bandwidth cause is not individually isolated. No motor commands sent.

- Head camera recovered via the user's USB-A to USB-C adapter (4 October 2026). Exact USB identity0x11100005a39230 returned a fresh native AVFoundation image; visually confirmed head overview of open carton. Earlier head-no-frame notes are superseded for this connection. Image: task outputs/Carton-Scene/head-adapter.jpg. Current view is aimed high and crops the near box/work area; aim downward before carton work. Camera indices remain unverified after reconnections; do not reuse old numeric mapping blindly. No motor ports accessed during this camera test.

- Carton preparation (4 October 2026): fast-forwarded to origin/main 6bac20e while preserving all pre-existing local control/calibration edits; backups are in task work/carton-integration-backup. Downloaded published carton-act-8000 (191775973 bytes), archive and all internal SHA256 checks passed. Synthetic PolicyRunner inference on MPS passed with action(12,), chunk(100,12), front/top inputs. This did not access motors and is NOT physical task validation. Base suite:187 passed,3 cache-permission failures; all3 pass when HF_DATASETS_CACHE is placed in task work/test-datasets-cache. Carton CLI now disconnects on connect/check/teach/viewer failures, individual teaching starts STOP viewer, batch teaching ends on first failed pose, and viewer startup must be confirmed.22 targeted carton/cleanup tests passed. No carton movement or real poses have been recorded. Missing physical integration: head camera stream and station measurements/clear views; ACT front/top mapping and robot-specific task competence remain unvalidated.
- Current camera diagnosis: three external USB cameras now detected. Both wrist scenes captured; camera index ordering changed and must be reverified before profiles are used. Newly reconnected expected head USB identity0x12130005a39230 yields no frames with exact-ID native AVFoundation capture at640x480 or native preset, despite permission authorized. Same exact-ID code succeeds on wrist0x12140005a39230, so camera connection/stream path needs isolation (direct-to-Mac USB recommended). Do not treat OS detection as a working head image. OpenCV numbered captures returned duplicate wrist views; no new camera mapping saved. Phone last-frame age became stale during prep; refresh before physical work. No motor ports were opened in carton preparation. Evidence: task outputs/Carton-Camera-Diagnosis.json, Carton-Model-Verification.json and Carton-Scene/.

- Latest spin attempt (4 October 2026): stopped; full physical rotation NOT VERIFIED. Clearance advance ~29.76cm encoder estimate, then a nominal full encoder turn plus visual-alignment sections (15574/15582 total wheel ticks). No communication or stop faults during these sections; final independent check all16 torque-off, status0. The camera heading still differs from the start, so configured turn geometry/traction is unverified. Extra visual travel allowance paused once and was preserved before bounded continuation. Automatic review then REJECTED more motion pending independent heading/cause verification. Do not resume spin helpers without addressing that rejection. Evidence: outputs/Robot-Spin-Result.json and starting/stopped heading photos. Previous50cm result remains historical; no long loop started.

- Latest50cm drive COMPLETE (2026-10-04): one user-requested outward/return trip, arms/head released throughout; .02m/s in <=1.5s segments, stops verified after each, camera inspected between short batches. Outbound6515/6521 cumulativeticks (~49.97/50.02cm). Final residual2/-5ticks (~0.15/-0.38mm encoder-derived; floor distance not independently measured), all16 independently verified torque-off. Required two diagnostic pauses: (1) zero-target wheels were stationary but one-count velocity quantization prevented two consecutive simultaneouszero samples; now require5stable-position samples over>=.18s with actual zero-speed observation on each wheel, then separate5sample stationary check after release; speed400 and.9s stop deadline unchanged. (2) one left-wheel return command registered only2ticks vs right338; normal voltage/status. Isolated acknowledged left-wheel command moved326ticks and corrected mismatch. Starts now verify torque-enable and each acknowledged/read-back velocity command within.15s before continuing pulse. Original missed-movement root cause is UNCONFIRMED; subsequent paired return segments passed.17 targeted tests pass. No long loop was started. Evidence: task outputs/Robot-50cm-Drive-Result.json and before/turnaround/returned images; work/fifty-cm-drive.json, left-wheel-return-recovery.json. Earlier partial notes are historical.

- Short trip COMPLETE: outbound988/990ticks (~7.6cm), returned with remaining23/6ticks (1.8/0.5mm encoder-derived), all16 torque-off verified. The following partial-return notes are historical. User subsequently requested a new single50cm outward/return test; task work/drive_fifty_cm.py is in progress with lower.02m/s commands, maximum1.5s pulses, stop/release checks after every pulse, common encoder reference and cumulative unwrapped distance tracking. No30-minute loop. State: work/fifty-cm-drive.json.

- Latest short-drive state supersedes the preceding partial note: outbound reached988/990ticks (~7.6cm), then two backward pulses passed with stops. Current velocity-mode readings3181/447 relative origin3700/4032; remaining return519/511ticks (~4cm). All16 independently verifiedoff. Person directly beside base in latest fresh frame, so final return pulses withheld. Continue from work/short-drive-resume.json only after area clears and current position passes the unchanged guard. Outputs/Robot-Current-Drive-Check.json records latest partial outcome.

- Latest short-drive continuation (2026-10-04): requested single ~8cm forward/return is INCOMPLETE. Initial 1s/.03m/s pulse stopped on speed sample left-450/right350 (unchanged400 cap). Two subsequent 1s/.02m/s pulses passed stop/release checks, with all16 independently torque-off after each. Current velocity-mode encoder origin3700/4032, last2949/690, outbound offsets751/754ticks (~5.76cm). No backward movement in this trip yet. Do not restart a whole outbound trip or use old-room coordinates. Latest camera shows a person beside/crouching at the cart, so next pulse held. Diagnostic fixed an encoder-reference mismatch: Operating_Mode0 subtracts saved Homing_Offset85; mode1 does not. Verified twice by torque-off mode changes with zero speed and no physical movement; comparison now uses a common mode1 reference. All fault/camera checks retained, speed reduced.3 new coordinate tests and7 existing stop tests pass. Task files: work/resume_short_drive.py, work/wheel_coordinates.py, work/short-drive-resume.json, work/wheel-mode-offset-verification.json; output Robot-Current-Drive-Check.json.

- Latest result (2026-10-04): ALL16 motors passed today's basic movement checks across the recorded attempts (14 arm/head +2 wheels). Right elbow was recovered autonomously from raw3198 to an in-range holding pose at low speed100, acceleration10 and torque cap400, within a12-degree capture envelope; no manual repositioning was needed. A normal sub-six-degree inward nudge and return then passed with vendor-documented default P=32: start94.833,target88.833,reached90.526,end95.024 normalized. Raw load218, no servo fault or rejected reply. Working elbow P32/Torque_Limit400/Goal_Velocity100 saved in profile and verified on hardware with torque off. Acceleration was restored to its prior setting after the test. Calibration offsets/limits unchanged; all16 motors independently verified torque-off and settings matched saved files afterward. Task evidence: work/right-elbow-recovery-p32-passed.json, work/final-calibration-verification.json, outputs/Robot-Readiness-Check.json.42 targeted helper tests pass. Earlier15/16 notes below are superseded.
- Residual behavior: released right elbow can sag beyond its saved range (raw3175 then3198 observed, saved max3092). Normal adapter startup MUST retain its range check. Task-local work/recover_right_elbow.py implements the specifically tested, single-motor, speed/torque-limited recovery without widening calibration: target stays at least24 ticks inside the range, capture travel<=12degrees, timeout3s, other15 motors verifiedoff, health/camera checks, release and setting restoration. It refuses starts outside that envelope. The current camera threshold is5s by explicit user instruction. The larger30-minute routine remains unstarted; these are basic bounded tests, not full operational/collision validation.

- Latest 2026-10-04 outcome: 15/16 motors passed today's basic tests. All13 arm/head joints except right elbow passed small move/return checks across attempts. Both wheels then passed one1s forward pulse and one1s backward pulse at nominal0.03m/s, with independent stop/release checks; final encoder offsets-2/+3 ticks. All16 motors independently verified torque-off afterward. Right elbow remains unfinished: undertravel (1.914 normalized units for requested5, minimum2.0), then sag beyond saved max prevented retry. It needs supported repositioning before its final small test. No sustained load/servo fault or reply-validation rejection was reported. Evidence: task outputs/Robot-Readiness-Check.json and work/new-setting-wheel-summary.json.
- User explicitly requested removing the strict two-second camera stops. The task-local bounded tests now use a five-second camera age threshold, reject absent/future/older frames, and allow up to ten seconds for a valid frame before beginning a new joint. Wheel pulses remain bounded to1s in today's check; independent zero-speed/stable-encoder/release checks are unchanged.29 targeted software tests passed (7 camera,9 packet-validation,7 wheel-stop). The right wrist/claw then passed on hardware. This does not authorize or launch the previously rejected30-minute loop; none was started. Earlier timestamped notes below describe superseded intermediate states.

- New-location movement checks (2026-10-04): 10 of 14 arm/head joints passed across attempts: both head axes, all six left-arm joints, and both right shoulders. Left elbow passed a 5-normalized-unit inward nudge after under-travelling the smaller 3-unit request. Right elbow moved1.914 units against a5-unit target, slightly below the standard minimum2.0, and returned; load144 at hold, status0, no rejected packets. After release it sagged to raw3175 beyond the saved max3092, so its planned5.5-degree retry was blocked before enable. Right wrist/claw tests were then isolated with other motors off, but camera upload age2.562s stopped before their first movement. No wheel motion has been attempted at this location. Camera resumed, with10 new frames over a5s diagnostic window and maximum age1.395s. All16 motors independently verified torque-off after the latest camera stop. Latest evidence: task work/new-setting-first-movement-attempt.json, new-setting-second-movement-attempt.json, new-setting-movement-test.json, new-setting-final-release.json. Remaining: reposition right elbow with motors released, retest it, finish right wrist/claw, then a bounded wheel check from the new location. Prior-day passed checks are historical, not today's completion.

- New-location preflight (2026-10-04): new phone supplies fresh portrait frames showing the whole base and nearby floor; table edge obscures part of the arms. Read-only check with strict reply validation active: all 16 motors responded, torque0/status0/load0, 27–30 C, raw voltage118–121; no communication errors or rejected packets in 96 register reads. Right elbow is outside saved range (3174 versus 1002..3092); all other arm/head joints within saved ranges. Shoulders and left elbow are near endpoints. No motors enabled or moved in this session yet. Current wheel readings3617/3945 are a new-location pose, not the old interrupted-demo origin; do not return to yesterday's coordinates. Evidence: task `work/new-setting-readonly-preflight.json`.

- Most recent immediate-demo retry is INCOMPLETE: only the first ~2.6 cm forward pulse ran and passed stop/release checks. Setup for the second forward pulse raised `IndexError: list index out of range`, before enabling wheels or moving arms. All 16 motors were subsequently independently verified torque-off, with calibration unchanged. Original wheel readings: left 276/right 3052; after first pulse: left 4030/right 3392 (4096-tick wrap). Preserve this origin: do not start another complete forward sequence and accumulate displacement. Evidence: task `work/dramatic-single-cycle/`. The phone then explicitly stopped sharing, so no further motion was attempted. The 30-minute loop remains unstarted.
- Offline investigation reproduced a Feetech SDK bug: a write acknowledgement can be accepted as a read response; a two-byte read then raises IndexError, and a one-byte read can incorrectly use the checksum as data. This is a plausible cause of the latest error, not confirmed by an original traceback or packet capture. Task-local `work/strict_servo_replies.py` now rejects incorrect reply lengths and preserves servo fault flags; integrated only into the demo bus instances, without changing motor limits or the installed SDK. Detailed traceback/event capture was added. Nine new offline regression tests and 13 existing diagnostic tests passed (22 total). This new receive guard has NOT been verified on hardware. Before resuming: restore phone sharing, inspect a fresh image, perform read-only communication checks, verify no cart repositioning, then use the recorded origin for a bounded recovery/completion. Do not blindly rerun the full demo.

- Latest larger-demo outcome: user accepted triple the small travel as fallback. One bounded cycle passed: roughly 8 cm forward, -65-normalized-unit shoulder sweep (~68 degrees), return to rest, then backward; final encoder errors +4/+1 ticks. All 16 motors independently verified released afterward; calibration unchanged. The 30-minute loop was rejected by automatic approval review for cumulative physical-damage risk given the incomplete route view. No loop or automation was launched. Do not bypass this rejection by splitting the rejected long run into repeated trials. Saved details: task outputs/Robot-Larger-Demo-Result.json and work/dramatic-single-cycle/. The older pending note below predates the accepted shorter-distance fallback.

- New larger-demo request is pending: user wants arms up to halfway along the neck, no lower than the present pose, and one-to-two cart lengths forward/back for 30 minutes. The timed loop has NOT started. Current phone view does not include the complete floor route or tether; user was asked to move the phone farther back. A +20-unit shoulder trial passed; a +40 trial stopped early, followed by eight normal samples with status0/torque0. A subsequent -10-unit direction trial passed. No longer base motion was attempted. Details: task `work/dramatic-demo-request.json`.

- Both arms completed automatic calibration at velocity 200. The head completed manual calibration over comfortable cable-clear working travel. The full calibration report says `LOOKS COMPLETE`, without range problems or notes.
- All 14 arm/head joints have passed small movement-and-return checks across the logged attempts. The right claw initially fell short; it passed with position gain P=32, torque cap 500 (50%), and position speed 200, and passed again after those settings were saved in the profile. Other joints retain their prior position settings. These are basic movement checks, not full task/collision validation or exact-position metrology.
- All 16 motors were independently verified torque-off after the latest demo. Every saved calibration offset/limit and the right claw's saved position settings match hardware. The user explicitly authorized a one-off wheel test; the normal profile remains `wheels: false`.
- A local smartphone camera is connected and supplies fresh 720x1280 images at approximately 4 fps. No microphone or continuous video archive. Latest image/status: parent task `work/phone_camera/latest.jpg` and `latest.json`; always check image freshness before relying on it. Local viewer/QR/restart instructions are in parent task outputs. The phone view supplements motor checks; it is not a real-time safety monitor.
- Current calibration backup: parent task `work/calibration-head-and-arms.json`; verification: `work/final-calibration-verification.json`; user-facing backup: `outputs/robot-calibration-verified.json`. Movement summary: `work/movement-test-summary.json`. Older snapshots are historical.
- Latest repository test suite: 93 passed. Changes are local and uncommitted; no push was performed.
- Two consecutive complete demos passed: short forward drive, both shoulder lifts +8 normalized units and return, then backward drive. Each drive used a 1.0 s command at nominal 0.03 m/s; encoder-derived wheel travel was approximately 2.6 cm each way (ground distance not independently measured). Final wheel offsets from each cycle's start were +4/-5 and +2/-4 ticks. All four drives passed stop checks; both arm cycles returned within the 2-normalized-unit tolerance. Fresh phone frames were inspected between stages.
- Earlier stop-check failures were diagnosed: soft deceleration takes about 0.4 s at the demo speed, and the released encoder can jitter by one tick with a quantized +/-50 speed report. The task-local demo acknowledges/readbacks zero targets, requires zero-speed samples before release, then verifies five stationary samples (encoder span <=2 ticks, speed <=one 50-unit bin) after verified torque-off. This does not change global load limits. Seven meaningful stationary-check tests passed, including rejection of drift, continuous slow rolling, and large speed readings. Calibration and right-claw tuning remain unchanged.
- Demo evidence: parent task `work/forward-arms-back-results.json`, per-stage `work/cycle1-*` and `work/cycle2-*`, and `outputs/Robot-Movement-Demo-Results.json`. Routine components: `work/robot_demo.py` and `work/wheel_stop_check.py`. Normal profile remains `wheels: false`; only explicitly authorized isolated tests drove the wheels. Stops are host software controlled; no device watchdog was configured.
- Full farm tasks, taught poses, long-distance navigation, and the missing light sensor are not validated. Earlier camera/cart contact remains a clearance consideration for larger movements.
- The chronological sections below contain superseded states; use this summary for current state.

# Where the build stands

## Bare-gripper bimanual carton exploration — 2026-10-06

- Began the requested two-hand folding approach, initially scoped to closing
  the existing carton's four top flaps. Added an offline reach screen reusing
  `Box/Stance` and a strict left-six/right-six action-data boundary. The active
  `remote robot control` chat's Molmo paddle experiment was inspected, not edited.
- Verified actual checkpoint normalizers: local Molmo SO101 is 6-D, Molmo YAM
  is 14-D; our two SO101 arms require 12 channels. ABC publishes a specific
  `folding_paper_box` 14-D YAM checkpoint; its public metadata was inspected,
  but neither its 8.063 GB weights nor gated demonstrations were downloaded.
- Fresh `box_closing3` metadata confirms the existing 50-episode, 12-channel
  bimanual SO100 data as an embodiment-near baseline. Joint units and station
  transfer remain unverified; matching dimensions do not authorize execution.
- At the saved 60 mm shoulder setback, bare-gripper far-flap contact misses
  the existing reach allowance by 29 mm. A 20 mm assumed setback passes only
  the spherical reach screen; both arms, bracing, orientations and collisions
  still need a contact simulation and measured station registration.
- 22 focused bimanual/carton tests passed. No new learned box policy, dual-arm
  contact simulation or physical folding result is claimed. No motor commands
  or model training were issued by this exploration.
- [Research, reuse decisions and experiment plan](docs/carton-bimanual-molmoact2.md),
  [source and feasibility evidence](docs/evidence/carton-bimanual-exploration.json).

## Calibrated tags consumed by Gemma and simulated grasp — 2026-10-06

- Added `robot_get_registered_tags` to the installed local Gemma chat (24 tools).
  Automatic registration saves a passing fit for this read-only tool; each read
  rechecks stream, intrinsics, tag mount, robot model, motor configuration/raw
  ranges, table/head stability, and current gripper agreement. A missing or
  changed registration refuses coordinates. Cartesian motion stays disabled.
- Installed the pinned LeRobot/Placo dependencies in the local pilot; the
  detector-only environment previously lacked these registration dependencies.
  Calibration now releases motors before the offline FK/fitting stage.
- Full rendered end-to-end run passed: production adapters, eight fitting and
  three held-out poses, actual LeRobot FK/OpenCV solve, registered tag 3, CAD
  handle offset, then contact-physics lift/hold/release. A moved paddle produced
  a new target and successful grasp; an open-jaw control correctly failed to lift.
- Held-out fit RMS was 0.637 mm, but independent handle error was about 7 mm and
  camera-origin error 8.60 mm. Rendering was 1920×1440; these results do not
  establish the live 640×360 camera's accuracy. Station placement, mass, friction
  and the simulated tag mount remain assumptions, including compliant contacts.
- 646 repository tests passed, 8 skipped; 26 pilot tests passed. Actual Gemma
  inference used the new read-only tool and correctly reported that a physical
  registration is not installed. No physical motor commands were issued.
- Latest read-only readiness is blocked by stale/stopped owner status and an
  unavailable authorized motion interface; only table tag 1 was detected.
  A subsequent fresh image shows the lower edge of gripper tag 2 outside the
  image. This differs from the earlier frame with about 8 px clearance.
  Physical calibration/use remain
  unvalidated; this work did not restart or change the remote motor owner.
- [End-to-end method and limits](docs/gemma-calibration-simulation.md),
  [compact evidence](docs/evidence/gemma-calibration-e2e.json),
  [installation and execution](docs/gemma-automatic-calibration.md).

## Automatic Gemma tag calibration integration — 2026-10-06

- Connected the existing `Experiment.calibrate` to authenticated Gemma motor
  commands and same-frame OAK tag observations. Automatic registration mode
  collects eight fitting and three held-out poses for the existing LeRobot FK
  assembly/OpenCV fitter. Normal completion verifies release; faults stop and
  retain evidence without retrying or resetting the owner.
- Installed and reloaded the idle local Gemma chat. Its live 23-tool catalog
  includes `robot_calibration_status` and `robot_calibrate_tags`. Actual Gemma
  inference used the read-only status tool and correctly reported the right
  elbow's saved-range blocker. No physical movement was requested by this work.
- Fresh frames detected tags 1 and 2; tag 2's black square was fully visible,
  about 8 px from the bottom edge. The previous clipping claim was too strong.
  Detection can vary between frames; missing tag 3 does not prevent hand-eye
  collection when tags 1/2 are observable.
- 628 repository tests passed, 8 skipped; 26 existing pilot tests passed. The
  30 new integration tests cover measured completion, held-out fitting, STOP,
  session changes, wrong-arm binding, stale/changed camera data and no-motion
  acknowledgements. These use a simulated API, not physical/collision testing.
- Physical registration remains unvalidated. The right elbow is outside its
  saved range and the right-arm candidate geometry configuration is absent.
  Other motion clients must remain idle for the supervised calibration run.
- [Setup and execution guide](docs/gemma-automatic-calibration.md),
  [verification evidence](docs/evidence/gemma-automatic-calibration.json).

## Right-arm AprilTag binding correction — 2026-10-06

- The user confirmed tag 2 on the right fixed gripper housing. The earlier
  left-encoder captures and their candidate FK are invalid for registration.
  Camera-only metric observations remain evidence of detection, not robot
  registration. There are zero accepted independent right-arm poses.
- Capture now requires an explicit arm and confirmed matching fixed mount.
  Dataset assembly and fitting preserve and check this binding; old unbound
  samples and mixed arm/tag identities are rejected.
- Targeted geometry, capture, registration and Gemma adapter tests: 88 passed.
  [Correction and fresh read-only evidence](docs/evidence/gemma-right-arm-binding.json).
- A corrected read-only capture could not detect tag 2 at the lower image edge.
  Right elbow readback was 3155 against recorded 1002..3092, and right-arm
  geometry configuration was absent. The owner repair chat is auditing this
  range discrepancy; no motor commands were issued by the binding correction.
- The existing carton controller already implements automatic joint probing
  and independent visual-model checks. Its command-file transport and seeded
  camera observer needed adapters for Gemma's current authenticated owner
  and OAK tags at this checkpoint. The integration above now supplies them;
  physical pose collection remains unvalidated.
  See the [automatic workflow](docs/gemma-tag-geometry.md#automatic-move-and-observe-workflow).

## Metric AprilTags and registration utilities — 2026-10-06

- The live Gemma chat now reads camera-relative tag centres in millimetres,
  through its existing OAK camera and authenticated transport. Actual Gemma
  inference read IDs 1/2/3 and reported approximately 378 mm between the gripper
  and paddle tag centres, correctly stating that these data cannot yet command
  a grasp. A prior stale-frame probe was rejected. No motor commands were issued.
- Added stationary camera/encoder sampling, candidate FK through existing
  LeRobot/Placo and fixed-camera OpenCV hand-eye fitting with independent
  validation poses. The initially captured pose and candidate FK were later
  invalidated by the right-arm mounting confirmation above. No robot-frame transform
  was invented or activated. Printed sizes remain user-confirmed 60/40/40 mm,
  without a separate ruler measurement.
- Metric rendered-camera tests retained both failures: worst error 6.72 mm at
  640×480 and 5.05 mm at 1280×960. The 1920×1440 run passed the unchanged 5 mm
  target at 4.73 mm maximum across 138 tag positions. This does not validate the
  live 640×360 camera's physical accuracy. All three runs detected all three
  tags in 46/46 nominal frames and passed the existing negative controls.
- Repository tests: 583 passed, 8 skipped. Existing pilot tests: 26 passed;
  browser-session recovery checks passed. The local chat was restarted idle
  with saved history and inactive prior goals; its 21-tool catalog includes
  metric observations. This local adapter installation restarted neither the
  camera process nor the remote owner.
- Physical pickup remains unvalidated: the initial remote owner latched after an idle
  `COMM_RX_CORRUPT` read. Source and saved-row timestamps point to right gripper
  ID 6 but do not prove USB contention or a particular packet failure. Added
  bounded transaction/reply diagnostics to the existing telemetry helper, with
  unchanged failure handling and no retries. These hunks were then staged in
  the actual remote helpers; 23 remote hardware-independent tests passed.
- A subsequent direct user request in the Gemma chat initiated controller
  recovery. That chat replaced the released owner to load diagnostics. The
  follow-up readback showed idle, motion-ready and all 16 motors released.
  Its communication stability check is ongoing; this is not a verified servo
  fault fix or a completed physical grasp. The initial stopped-state evidence
  and later recovery snapshot are kept separately in the evidence file.
- [Guide and next steps](docs/gemma-tag-geometry.md),
  [measured evidence](docs/evidence/gemma-tag-geometry.json).

## Gemma AprilTag simulation — 2026-10-06

- Passed the production `TagRobot`/`TagObserver` on MuJoCo-rendered RGB using
  the existing SO-101 meshes/joints and paddle CAD. All three tags were detected
  in 46/46 nominal frames across 24 degrees of measured shoulder-pan travel.
  Maximum tag-centre error was 0.274 px; maximum gripper-to-paddle displacement
  error was 0.455 px against independently projected simulator positions.
- Occlusion, duplicate IDs, undersized tags, stale frames and hash mismatches
  were rejected; tags were reacquired after faults cleared. Simulated STOP
  prevented a subsequent movement request.
- Local Gemma called observe, simulated +12-degree pan, observe and STOP,
  correctly reporting the changed pixel displacement. No hardware tools were
  exposed, and no physical camera or robot was contacted.
- The camera pose and tag mounts are illustrative, and rendered lighting is
  idealized. This validates tracking/data flow and model tool use, not a
  calibrated physical grasp or autonomous carton folding. Reproducer and
  limitations: [simulation guide](docs/gemma-apriltag-simulation.md).
  Results: [simulation evidence](docs/evidence/gemma-apriltag-simulation.json).

## Gemma AprilTag observations — 2026-10-06

- The Gemma chat exposes `robot_get_tags` through the shared detector and its
  existing authenticated `robot_get_cameras` transport. Detection runs on the
  Gemma Mac; no additional camera stream or motor owner was opened.
- After the user repositioned the tags, OAK detected IDs 1 (table), 2 (gripper)
  and 3 (paddle) together in three advancing frames. The tag-3 centre was about
  78.6 px right and 89.9 px above tag 2. The earlier same-view visibility gap is
  resolved for this stationary setup; tag-based 2D displacement is available.
  The phone view contained no valid tags in this check and remains receipt-only.
- Real Gemma inference invoked the tool and accurately reported missing views
  and tags. The live chat advertised 19 tools after installation, retaining
  existing direct-control tools and STOP. This check issued no motor writes.
- The integration provides 2D measurements and annotated images. No metric
  pose, camera-to-arm calibration, grasp or physical motion was validated.
  See [setup](docs/gemma-apriltags.md) and
  [initial evidence](docs/evidence/gemma-apriltags.json) and
  [same-view follow-up](docs/evidence/gemma-apriltags-covisible.json).

## Servo temperature sensing removed — 2026-10-05

- At the owner's direction the software no longer reads or acts on servo temperature anywhere: the `farm` safety
  rules, `farm robot-test`, `farm mcp`, the simulator, and the carton motor owner and its clients.
  `farm soak` and the profile key `servo_temp_max_c` are gone.
- Load, step, travel, watchdog, lease and STOP checks are unchanged. A nonzero servo `Status` byte still stops
  the carton owner, and that byte is where the servo's own firmware would report overheating.
- 2026-10-06: the servo-side protection is next. `farm servo-protection -p paper-tray-v0` reads every servo's
  EEPROM temperature limit (default 70 °C) and unload/alarm masks; `--write` sets the limit to 200 °C (one byte;
  Feetech documents 0..100; if a servo will not keep 200 the tool writes 100 instead and says so) and clears
  the temperature bit in both masks, so the firmware neither unloads nor flags on heat. Not yet run on the robot: this Mac has no motor boards. Run it from the robot Mac following
  `ROBOT_HANDOFF_2026-10-06-servo-protection.md` and record the before/after table here.
- The 2026-10-05 session-archive copies of the motor owner and claw script, which enforced 55 °C, were deleted
  on 2026-10-06. A robot-Mac working copy of the carton owner outside this repository (`work/`) keeps its old
  checks until the packaged scripts in `software/scripts/carton_robot/` replace it.

## OAK-D Lite test — 2026-10-05

- User has an OAK-D Lite connected to the development Mac, separate from the
  robot-control Mac. Added a camera-only DepthAI utility and
  [setup guide](docs/oak-d-lite.md). It does not connect motors or integrate
  depth observations into the farm controller.
- DepthAI 3.10.0 failed at boot in both USB modes, including after the user
  unplugged/reconnected directly. Switched the isolated Python 3.12 environment
  to DepthAI 2.33.0.0 (OpenCV 5.0.0.93, NumPy 2.5.3); v2 boots and streams.
  Four depth evidence/storage tests pass. A v2 device-info-only probe reported
  a crash during shutdown; the final 30-second capture exited cleanly.
- Live capture: 448 synchronized RGB/depth pairs in 30.032 s over USB 2,
  640×360 output, maximum timestamp skew 1.265 ms. Last depth frame had
  30.89% valid pixels; centre patch had no valid depth. Camera was facing the
  room/ceiling, not the planter. RGB image inspected; raw 16-bit depth saved.
- Local evidence on development Mac:
  `/Users/wk/conductor/workspaces/research/minsk/.context/oak-captures/20261005T005902.949416Z/`.
  Camera imagery stays local and is not committed. USB 3 remains unverified.
- Next: aim at stationary trough/carrier, compare measurements with a ruler,
  check fin/rim depth coverage, then calibrate camera-to-robot coordinates.
  No claim of planter accuracy, robot integration, or successful assembly.

## Earlier robot handoff

Written 2026-10-02 for whoever picks this up on the robot's laptop (person or agent). Read this first, then `SETUP.md` (installing) and `README.md` (what the program is). `docs/community-projects.md` reviews the 43 projects on the XLeRobot community page against this plan.

## The robot

- **Kit:** WowRobo XLeRobot 0.4.0 two-wheel Combo: two SO-101 follower arms (12 V STS3215 servos), head with two servos, two drive wheels, three bare USB camera boards, two motor control boards, IKEA cart.
- **Assembly:** built to the end of the official 0.4.0 video (https://www.youtube.com/watch?v=4bXCFw57T60). The older 0.3.0 video and docs are for the three-wheel version and do not match this kit.
- **Orientation:** as in the video. The top base sits against the drive-wheel side of the top tray; arms and head camera face outward over that edge; motor boards and cables are at the back (caster side). The official 0.3.0 render shows the opposite (arms facing inward); the 0.4.0 arm bases can be re-clocked either way. Outward is what the farm needs: trays sit on a separate surface in front of the arms.
- **Base:** parked. The profile has `wheels: false`; nothing in the farm program drives the wheels.

## What is done and verified

| Item | State |
|---|---|
| Loose servo IDs | Set and read back on the real servos: head 7 and 8, wheels 9 and 10 (`farm set-motor-id`) |
| Arm servos | One arm probed: IDs 1-6 all answer (STS3215). The other arm was not probed on its own. |
| Motor power | A USB-C-to-12 V cable gave about 12.5 V on the bus |
| Software | Fresh clone installs and passes 109 tests; simulator runs end to end |
| LLM backends | Claude CLI and OpenRouter (Jev, Astra) both answered from the first laptop |

## What is not done

| Item | State |
|---|---|
| Full-robot probe | Verified on robot laptop: all 16 STS3215 servos answer; bus 1 IDs 1–8, bus 2 IDs 1–6, 9, 10. Read-only health/position reads succeeded for every servo; 29–31°C, raw voltage 118–121, torque off, load/current zero. Powered movement not tested. |
| Profile | Motor ports saved: port1 `/dev/cu.usbmodem5B790186401`, port2 `/dev/cu.usbmodem5B790182091`. Camera indices still unset. |
| Calibration | Not done. Nothing can move until it is |
| Taught poses | None (`data/keyframes.yaml` does not exist) |
| Cameras | macOS now detects all three external `USB2.0_CAM1` cameras plus the built-in MacBook camera after the owner powered the head camera. Newly appeared device: `0x11130005a39230`, likely head based on that action; live frames and identities still unverified. Run camera probe from Terminal. |
| Light sensor | ESP32 + BH1750 not wired or flashed |
| Printed parts | Cress planter, nests, tag tiles, bottle rest, paddle: print status unknown |
| Head naming | Pan = 7, tilt = 8 is assumed, not checked on the hardware. RoboCrew's XLeRobot driver uses the same mapping (yaw 7, pitch 8). `farm robot-test --move --ask --only head` checks it |
| Wheel sides | Left = 9, right = 10 is assumed, not checked (unused while parked) |

## Hardware facts worth keeping

- **Motor boards (serial → macOS port):** `5B790182091` → `/dev/cu.usbmodem5B790182091`; `5B790186401` → `/dev/cu.usbmodem5B790186401`. Which one ended up on the left arm is decided by the probe, not by the name.
- **Left vs right:** the arms are identical. The left arm is the one whose board also carries the head servos (7, 8); the right arm's board carries the wheel servos (9, 10). Left and right are from the robot's own point of view.
- **3-pin wires are servo wires and carry 12 V.** They go board → servo → servo. Never plug one into a camera.
- **Cameras:** each has a 4-pin USB port and its own cable to the hub. They do not connect to the arms.
- **Wiring to the laptop:** both motor boards (USB-C) and all three cameras go into one USB hub; the hub's main cable goes to the laptop. Each motor board gets its own 12 V feed (USB-C-to-12 V cable from the battery). Switch 12 V off before moving any servo wire.

## Next steps, in order

Run from Terminal (camera permission is per app), inside `software/` with the environment active.

1. `farm devices --probe`: confirm both boards and their IDs, and look at the camera snapshots in `data/devices/`.
2. Edit `profiles/paper-tray-v0.yaml`: `port1` = the board with IDs 1-8, `port2` = the board with 9 and 10. For each camera add `index_or_path: <n>` using the snapshot that shows the matching view.
3. Calibrate, one of two ways, then check the result:
   - By hand (known to work): `farm calibrate`. Support both arms and follow the prompts.
   - Automatic (decided 2026-10-03 to try it; **never run on this robot**): `farm calibrate --auto --arm left`, then `--arm right`, then `farm calibrate --head`. Go up in stages first (`--motor gripper`, `--motor wrist_roll`, `--unfold-only`); the `farm-bringup` skill, Step 4B, has the procedure and what to do when a stage goes wrong. Write down what happened here.

   Then `farm calibration-report`: reads the saved file and flags a wrapped reading, a short sweep, or arms that disagree, before anything moves.
4. `farm robot-test`, then `farm robot-test --move --ask`: motors only (no cameras, no models, no trays). The first reads every joint and load. The second nudges one joint at a time and asks whether the named part moved; this catches swapped left/right boards and swapped head motors. Start with the arms folded; the motors go limp when it ends.
5. `farm check`: connects everything and has the vision model confirm which camera is which.
6. Put the bottle, paddle and trays in their fixed places, then `farm teach-all`.
7. `farm once --tray B` with an empty bottle; authorize from the viewer (http://localhost:8765).
8. `farm cup-test --tilt 25 --seconds 1.5 --who <name>`.
9. `farm run --every 3600 --record`.

## Testing the policy we already trained

Decided 2026-10-03: test the existing checkpoint on the robot before doing anything with a second computer. The two-computer policy server is parked until then.

The checkpoint is ACT, trained for 5,000 steps on someone else's SO-101 pouring data (`SurajCreation/so101_pour_v1`, one arm, a wrist camera and an overhead camera). Offline it predicts worse than holding still, so expect pour-like motion of the right arm, not a working pour. What the test proves is the path: checkpoint → camera frames → clamped joint steps on the real robot.

1. Copy `act_so101_pour_checkpoint.zip` (191 MB, in the first laptop's Downloads folder) to this laptop, for example by AirDrop, and unpack it inside `software/data-train/` so that `software/data-train/act_so101_pour/checkpoints/005000/pretrained_model/config.json` exists.
2. On the simulator first: `farm policy-test --checkpoint data-train/act_so101_pour --steps 40`.
3. On the robot, after calibration and `farm robot-test` pass, with the right wrist and head cameras working, nothing in the right arm's reach, and the viewer's STOP button open on a phone: `farm policy-test --checkpoint data-train/act_so101_pour --real --steps 60`. Every step is clamped to the profile's limit; it ends after 60 steps with `max_steps reached`, which is the normal ending.
4. Record here what the arm did.

The policy's `wrist` view is mapped to the right wrist camera and its `overhead` view to the head camera. Our head camera does not look from where the training camera did, which is one more reason not to expect a real pour.

Stop at any point with the red STOP button in the viewer.

## Built after the community review, not yet run on the robot

`farm calibration-report`, `farm robot-test`, `farm calibrate --auto` and `--head`, `farm mcp`, and `farm policy-server` with `farm policy-test --server`. All pass on the simulator, except that the limit-seeking motion inside `farm calibrate --auto` (LeRobot PR #3282, vendored) cannot be simulated and is untested here. The README has a table of them; `docs/community-projects.md` is the review they came from.

Decided 2026-10-03: teaching by hand was removed (no human operation, no exceptions); the two-computer policy server (`docs/gpu-server.md`) is parked until the existing checkpoint has been tried on the robot.

## R2a planter assembly (decided 2026-10-03; contract only, nothing physical)

The design chat handed over a modified cress planter ("R2a": the original holder with four insets fused in and
two grip fins, an optional paper frame, a grip coupon) for the robot to assemble. The full handoff is
`docs/r2a-handoff.md`; the reconciliation, station plan and what was built are in `docs/r2a-assembly.md`.

- Code: `farm/assembly/` (frames and units, stage machine with required evidence, fail-closed dataset schema,
  episode metadata and held-out split), `profiles/r2a-assembly-v0.yaml` with `execution_enabled: false`, 28 tests.
  `farm r2a` prints what blocks execution. The watering profile and skills are untouched.
- Parts: `parts/r2a/` holds the release STLs and records; hashes checked by the tests.
- Physical state when written: the full plate was printing on the M5C after one failed start (nozzle traced
  past the sheet edge; cause unresolved). Completion, part quality, fit, grips, fixture, station transform:
  **all unknown**. Nothing may run until `farm r2a` shows no blockers, and then only supervised and dry.
- Open hardware questions carried over: whether cameras were in the kit; the trough fixture (`nest_cress.stl`
  assumes a plain outline; the real trough has an offset refill bay).

### R2a continuation, October 3 (development Mac)

- User reports calibration currently running on the **other Mac connected to the robot**. It was not
  restarted or touched from this session. Completion, calibration report, camera identities and motor-test
  results have not yet been read back here. Local missing-device reports describe this development Mac only.
- Print identity is known: `cress_R2a_full_prototype_plate.gcode`, carrier + frame + coupon; original trough
  reused. The 15:19 JST observation is historical, not a new printer check. Print completion/inspection unknown.
- Added offline discrete supervisor rehearsals for both variants and fault paths (`farm r2a --simulate`),
  measured-point station fitting (`farm r2a-station`), and file-only laptop readiness/keyframe checks.
  These are not physics/vision tests or hardware assembly. Synthetic traces cannot qualify as training data.
- Fixed acceptance of non-affirmative OK readings, stale evidence/jaw readings, invalid grip thresholds,
  fractional paper counts, malformed enable flags, non-finite deadlines and extra placement attempts.
- [Connected-Mac continuation](docs/r2a-connected-mac.md) gives the measured inputs, isolated pose names,
  commands and remaining live executor/observer/recording integration. Execution is still disabled.

## Second task: carton closing (2026-10-03)

Separate scope, same robot: `software/carton/`, command `carton`, profile `carton-v0`. Measurements, reach analysis, plan and status in `docs/carton.md`. Built and tested on the simulator only. Uses the printed flap paddle and an ordered tape dispenser; actual dispenser setup/pickup remains unverified.

### Carton handoff, October 4 (development Mac)

- Start at [docs/carton-connected-mac.md](docs/carton-connected-mac.md) on the robot Mac. It covers
  preserving local calibration, the release download/checksums, local profile, station and remaining integration.
- ACT completed 8,000 steps on October 3 at 18:23 JST, final loss 0.293. Two duplicate same-seed
  processes wrote the directory; both ended. Final inference files were subsequently verified on MPS.
- [Release](https://github.com/RonTuretzky/xlerobot-farm/releases/tag/carton-act-8000-2026-10-04)
  contains the inference checkpoint; `git pull` alone does not download weights. Optimizer state excluded.
- Real recorded-frame check: 27 frames, episodes 47–49, **not held out**. MAE 3.888 versus a
  hold-current-state baseline of 1.831. Synthetic inference outputs `(12,)` and `(100, 12)`.
  [Evidence](docs/evidence/carton-act-8000-check.json). Neither is a physical success rate.
- 167 tests passed before this handoff. Print source meshes are tracked under `parts/carton/`.
- Real policy profile now names missing `policy_front`/`policy_top` views explicitly instead of
  feeding the head image twice. Learned control remains disabled and is not wired into carton cycles.
- Physical setup, print fit, teaching sequence/object state, observation-only ACT integration and
  hardware validation remain for the connected Mac. Teaching CLI changes are described below.
- Existing hardware calibration may have progressed independently on that Mac. Read it back; do not
  overwrite it or restart calibration based on this development-Mac status.

### Direct dispenser pickup follow-up, October 4 (development Mac)

- [Tape test instructions](docs/carton-tape.md): `carton tape-test --plan` is hardware-free;
  `carton tape-test -p profiles/carton-local.yaml --teach` physically teaches and executes pickup,
  lift-clear, adhesive-down placement, release and retract on a closed carton. **No flip/turnover.**
- Owner identified the ordered dispenser as [LUKDOF M1000, ASIN B0DQY77P16](https://www.amazon.co.jp/dp/B0DQY77P16).
  Listing checked October 4: front outlet, 20–999 mm cuts, 7–50 mm tape width, manual/automatic modes.
  Initial cut target is 80 mm; actual tape width, adhesive orientation, outlet clearance and release
  still need physical checks. Printed tape rest/folded tab are not required by the controller.
  First trial uses manual mode with automatic refill disabled and the mechanism stopped; verify removal
  does not trigger another feed/cut. No machine control/interlock or autonomous replenishment is implemented.
- Added staged head/wrist checks, locked gripper during transport, strict finite left-only poses,
  calibration/profile-bound teaching bundles and no retry/release on uncertain tape state.
- The full carton cycle uses the same tape sequence. Existing tape keyframes require re-teaching.
- Tape trial/individual teaching verifies the STOP viewer belongs to its session; ordinary batch
  teaching stops on the first failure. Live tape trials require attended Terminal shutdown.
- Validation: 199 tests passed, including 32 tape-controller tests; `carton tape-test --plan`
  lists six direct-pickup poses and opens no hardware. No training job or physical motion was run.
- These are controller/simulator checks only. Physical tape grip, visual face recognition, clearance,
  adhesion and release remain unverified. Put a visible mark on the strip's nonsticky backing for teaching.

## Rules that were decided

- Nobody drives the robot by hand. Poses are taught by the vision model (`farm teach`, `farm teach-all`); people only answer questions in the viewer or over Telegram.
- Right arm holds the bottle, left arm holds the light paddle.
- Trays, bottle rest and paddle rest stay in fixed positions; everything taught is relative to that layout.
- Water only moves when the rules pass and either a named person or Jev at `approve` level authorizes it. A pour with an unknown result blocks further cycles until someone reconciles it.

## Not on this laptop

These stayed on the first laptop: the trained ACT test checkpoint (copy it over as described under "Testing the policy we already trained"), the 3D-print files and print handoffs, and the site build output. The OpenRouter key must be typed into `.env` again; make a fresh one, since the old one was pasted into a chat.

## Robot laptop software verification

- Python 3.12 environment installed; 42 tests passed and simulator completed.
- Arduino CLI and ESP32 core 3.3.12 installed; Rosetta installed; light firmware compiled successfully. No ESP32 serial port detected.
- OpenRouter key authentication passed earlier on this laptop.
- Claude login needs attention: prior real request failed with expired OAuth session; latest auth check reports logged out.
- Calibration remains undone (confirmed by owner); no powered movement performed during the full-robot diagnostic.

### Camera probe result

Terminal captured valid 640×480 images from indices 0, 1, 2. Index 0 appears to be the head; indices 1 and 2 show wrist/gripper views, with left/right assignment pending a lens-cover check. Index 1 appears blurry. Index 3 was detected but rejected 640×480 (reported 1920 width); likely built-in camera, not yet conclusively mapped. Out-of-bounds warnings for 4–59 are enumeration noise. No powered movement performed.

Camera identity confirmed by owner covering the RIGHT wrist lens: index 2 went black, index 1 remained a wrist view. Saved mapping: head 0, left wrist 1, right wrist 2. Left wrist remains visibly blurry; focus needs checking before vision-guided motion.

### Calibration recovered on robot laptop

Both arms and head were manually ranged. Final right-bus save failed with a missing acknowledgement on ID 1; readback confirmed the minimum limit had been written. Subsequent read-only check found all 16 servos responding, voltage raw 119–122, torque off. Recovered the captured ranges using verified stored homing offsets, wrote only missing arm limits with retries, and verified all limits by readback. Saved and reloaded `~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json`. Existing wheel offsets/limits preserved without wheel writes. No powered movement test performed yet.

### Powered-test preflight

Corrected parked adapter to prime/read back hold targets before enabling arm/head torque, omit wheel commands, verify calibration instead of rewriting it, and clamp joint targets without the vendored action-key mismatch. Self-test now stops on a reported failure or wrong-part confirmation, and refuses nudges after health problems. New adapter and tool tests pass (10). Hardware preflight stopped before torque enable: left shoulder lift was 3250, above recorded maximum 3240. All 14 arm/head torque registers confirmed zero afterward. User must reposition shoulder slightly inside measured travel before retry. Powered movement remains untested. Changes are local, not pushed.

### First powered head test passed

After owner repositioned both elbows, all arm/head joints were inside calibrated ranges. Ran head-only robot-test at delta 3: pan moved 2.6 normalized units and returned; tilt moved 1.3 and returned; tool reported ALL OK. Initial loads zero. All 14 arm/head torque registers explicitly verified off afterward. Wheels untouched. Arm movement and visual confirmation of head axis identities remain untested.

### Automatic calibration preflight, 2026-10-03

Pulled upstream bb30bb3, preserving the local parked-adapter fixes. No automatic calibration motion has run. Read-only preflight found all 16 servos responding, torque off, 27–30 C, raw voltage 118–121. Both elbows are slightly beyond yesterday's recorded maxima, so do not run the normal powered adapter test without resolving that.

Found and locally corrected automatic-tool startup defects: single-motor mode previously configured/enabled all six arm motors; initialization enabled torque without refreshing Goal_Position after offset changes and could continue after verification failures; the range-measurement OperatingMode import was invalid. Single-motor mode now constructs a one-motor bus; initialization verifies torque-off, configuration and fresh raw holding targets before any enable, and fails closed. Range seeking now sends stop commands on every exit and rejects timeout/communication errors as calibration results. Fifteen focused tests passed. These software checks do not validate mechanical-stop seeking on this cart.

Installed the new mcp dependency. Saved the prior calibration under the task's work/before-auto-calibration.json. Physical clearance confirmation is pending. The unreviewed remainder of the upstream motion workflow (including midpoint transitions/unfolding) still requires review before powering any calibration stage. No hardware offsets or limits were changed during this preflight.

### First automatic gripper stage completed

Owner confirmed physical setup and requested start. Ran `farm calibrate --auto --arm left --motor gripper` using the isolated single-motor bus. Exit 0: raw endpoints CW 2965 / CCW 1390; measured sweep 1575 counts (138.4 degrees). Tool backed off and returned toward center. New left gripper EEPROM: Homing_Offset 130, Min_Position_Limit 1260, Max_Position_Limit 2834, Operating_Mode 0. After exit, read-only verification confirmed Torque_Enable=0 on all 16 motors. Left gripper position 2027. No wheel commands sent.

IMPORTANT: single-motor mode intentionally does NOT update the saved calibration file. Left gripper hardware now differs from yesterday's saved file; the parked adapter will reject that mismatch. Do not run robot-test or normal motion until calibration is reconciled. Original calibration backup remains `work/before-auto-calibration.json` in the parent task; post-stage readings are `work/after-auto-gripper.json`. Waiting for the owner's observation of jaw movement/contact before proceeding to wrist roll. Full automatic arm calibration has not run.

### Automatic left-arm run stopped on fault; calibration restored

Owner confirmed the gripper opened/closed fully, full physical clearance, and requested continuous staged calibration. Left wrist-roll stage exited 0: endpoints 117/1424, chosen travel 2789 counts (245.1 degrees), offset 771 and limits 653..3441. Unfold-only stage exited 0: wrist_flex and elbow_flex reverse, shoulder_lift forward.

Full left-arm stage completed shoulder_lift (2394 counts, 210.4 degrees) and elbow_flex (2224 counts, 195.5 degrees), then failed during simultaneous wrist_roll/gripper/wrist_flex limit seeking: `Could not confirm stop command for ['wrist_roll', 'gripper']`. Exited 1 after cleanup; saved calibration file unchanged. Read-only checks then found ALL 16 Torque_Enable registers zero, Status 0, voltage raw 119–121. Full calibration did not complete; right arm was not run.

Restored and read-back verified yesterday's backed-up Homing_Offset and Min/Max_Position_Limit for all six LEFT arm motors, plus Operating_Mode=0, with torque verified off throughout. No Goal_Position commands were sent during recovery. Head/right arm/wheels not written. This supersedes the earlier partial gripper-mismatch warning: offsets and limits now match the saved file again. Saved diagnostics: parent task work/after-auto-left-failure.json. Do not retry the full automatic sweep until the wrist/gripper communication fault is understood. Head remains manually calibrated from yesterday, not automatically recalibrated.

### Requested retry reproduced lost feedback

Preserved chained exception diagnostics; 15 focused tests passed. Retried full LEFT automatic calibration once at user request. It again failed during simultaneous wrist_roll/gripper/wrist_flex seeking. Wrist-roll Present_Velocity and Present_Position reads returned no status packet; all five position retries failed. Stop writes to IDs 5, 6 and 4 also received no status packet after four attempts each. Fault occurred before Ctrl-C was delivered at the exit prompt. Shoulder/elbow measured travel also changed materially between full attempts (210.4/195.5 degrees first, 160.1/147.8 second), so these partial measurements must not be accepted as repeatable calibration.

Afterward all 16 motors responded and Torque_Enable=0 was verified; raw voltage 118–121. Restored and readback verified previous saved left-arm offsets, limits and position mode again without motion. No new calibration file saved, no right-arm sweep performed. Repeated failure is correlated with simultaneous distal-joint movement; cause (power, wiring, servo/driver behavior) not established. Do not bypass lost-feedback stop or repeatedly rerun unchanged sweeps.

### Communication diagnosis, stationary test

Without torque or configuration writes, read six feedback registers for every left-bus motor (six arm plus two head), 30 cycles: 1440 reads, zero failures. All firmware 3.10, baud register 0, return delay 0, response level 1. Idle voltage raw 118–120, all Status registers 0. This excludes an always-present communication failure but does not measure voltage during the missing-packet interval.

Leading hypotheses: power/current delivery or voltage drop under simultaneous motor load; a distal servo-chain cable/connector affected by arm position; motor-noise/bus or driver timing remains possible. No cause proven. The two full runs lost responses at the simultaneous distal-joint stage while earlier individual tests passed. Upstream PR discussion https://github.com/huggingface/lerobot/pull/3282 reports a different XLeRobot left arm improved with velocity-limit=100, and the author notes insufficient supply current can cause stalls. This supports a lower-speed/sequential diagnostic after checking power delivery, not bypassing missing-feedback stops. No further movement performed during diagnosis.

### Reduced-velocity and isolated-wrist trials

User requested reduced velocity first, then one motor at a time. First velocity=100 start failed before any torque enable with missing Lock read response from ID5. Restored setup settling delays (50 ms torque-off, 10 ms configuration, 50 ms limits) and added bounded configuration read retries; fail-closed verification retained. Focused tests passed. This pre-movement failure weakens a purely load-only diagnosis: root cause remains unproven.

Next full-left velocity=100 run reached the shoulder/elbow measurements but produced invalid travel: shoulder 357.8 degrees, elbow 0 degrees. Interrupted and disabled torque; file not saved. Restored previous left-arm offsets/limits/mode and verified torque off. Added per-joint range plausibility validation before applying measured calibration or derived targets; 17 focused tests passed.

Then ran isolated left wrist_flex at velocity=100. No communication errors; stage exit 0, endpoints 2363/1108, chosen span 1255 counts (110.3 degrees), offset -312, limits 1420..2674. Candidate report flags mismatch versus right wrist_flex's manually recorded 199 degrees (left previous manual also 199). Therefore did NOT accept/save this as complete calibration. Restored original left wrist_flex offset -840 and limits 930..3197 with readback, then verified ALL 16 motors torque off.

Current conclusion: one-motor testing at low velocity communicated successfully once, but complete/repeatable travel was not established. Full auto-calibration remains incomplete. No right-arm auto-calibration attempted; prior calibration file unchanged and arm settings restored. Do not confuse successful stage exit with a valid calibration.

### Requested both-claw movement

Preflight left gripper position3533, saved limits2029..3476, so left torque was not enabled. Right gripper position2070, limits2046..3518, load0, torque0. Attempted isolated right-gripper small open-and-return (15 normalized points, increments capped3); other joints excluded from adapter groups. The test aborted before completing. Right torque readback0 after cleanup. Subsequent five torque-off reads returned load0, voltage121, status0, position2074 (start2070). No complete open/close cycle verified. Left claw remains beyond saved range.

### Interactive left-wrist observation

User requested another auto-calibration with physical-part callouts. Ran isolated left wrist_flex at velocity100. First endpoint2365 was detected; reverse sweep timed out after20 seconds. Exited1, file unchanged. After exit: torque0, raw position296, velocity0, load0, Status0. Restored original wrist offset/limits/position mode and verified torque0. User observed it looked fine while moving and said it was visibly at its stopping point. This establishes a possible end-stop detection/timing discrepancy, not confirmed inability to reach the stop. We did not log every in-motion sample, so cannot determine which stability/velocity/Moving condition prevented detection or whether it reached the stop near the timeout.

### Traced wrist retry isolated speed/timeout issue; velocity200 succeeded

At user request, instrumented read/write calls without changing stall detection. Isolated wrist_flex at velocity100 timed out during first sweep. Trace showed uninterrupted position progression2326→2348 in final0.29s, speed100, Moving1, Status0, and no communication exceptions: it was STILL MOVING when the20s deadline expired. Earlier user-observed stop may have occurred close to the deadline; this trace does not prove the same timing on the earlier unlogged attempt.

Retried isolated wrist_flex at velocity200 (one-fifth original1000), unchanged20s timeout and all fault guards. Exit0, no communication errors. CW2371 / CCW75, travel2296counts=201.8deg (manual prior199.2deg). Candidate offset-824, limits899..3195; readback verified, torque0, temp36C, Status0, position2039. Candidate snapshot saved in parent task work/successful-left-wrist-200.json and trace in work/wrist-calibration-trace-200.jsonl. Restored original wrist calibration again to keep saved file/hardware consistent; full calibration file unchanged. This successfully validates a plausible wrist-only sweep at200; does not resolve simultaneous-joint communication failure or validate remaining arm joints. Awaiting user's visual observation before further interactive diagnosis.

### Both automatic arm runs completed at velocity200; final validation still blocked

Owner confirmed isolated left wrist moved freely, then requested all remaining calibration. Full LEFT run at velocity200 completed exit0 and saved. Ranges degrees: shoulder_pan159.3, shoulder_lift210.9, elbow162.4, wrist_flex200.4, wrist_roll339.3, gripper136.1. First report flagged only old head pan. Left calibration was explicitly readback verified, torque0.

RIGHT gripper and wrist-roll staged runs at velocity200 exited0; unfolding stage exited0. Full RIGHT run at velocity200 exited0 and saved. Right ranges degrees: shoulder_pan206.0, shoulder_lift211.3, elbow183.7, wrist_flex201.1, wrist_roll335.6, gripper132.7. No lost-feedback errors reported during these full successful runs.

FINAL REPORT IS NOT CLEAR: left vs right shoulder_pan travel159 vs206 degrees exceeds30-degree mismatch threshold; old head pan0..4073 touches encoder edge. Other reported arm ranges have no flags. Do not call the whole robot fully validated, and do not start general powered tests/farm motion yet. Need inspect left bottom sideways shoulder swivel for its shorter travel, and redo/verify head pan manually with cable clearance. The auto tool does not support head calibration.

Readback verification of both full-arm results and all16 torque registers recorded in parent task work/after-both-auto-arms.json; current saved-calibration snapshot work/calibration-after-both-auto-arms.json. IMPORTANT: earlier work/before-auto-calibration.json is now an OLD manual backup. Do not blindly restore it over these completed results. Wheels untouched; head unchanged.

### Confirmed camera/cart contact during left retry — no more powered sweeps until clearance fixed

User moved a possibly tight wire and requested another full LEFT velocity200 calibration. It completed and persisted before the user's contact report/our stop arrived. User saw the camera bump the cart with the arm extended downward and rotating. Treat this as observed physical obstruction, not an actual joint end stop. Run had no communication error; elbow improved195deg, shoulder_pan stayed158deg versus right206, and wrist_flex shrank144deg versus right201. Final report flagged both mismatches plus old head pan. Do NOT accept this contact-affected run as valid.

Preserved its file in parent task work/calibration-with-camera-contact.json. Restored ALL LEFT entries/hardware offsets and limits from work/calibration-after-both-auto-arms.json (the preceding full200 run), preserving successful RIGHT calibration and unchanged head/wheels. All16 torque-off and hardware/file calibration equality explicitly verified. This restores the prior state only: left swivel159-vs206 mismatch and head pan still unresolved; restored calibration is NOT a clearance guarantee. Camera mount/placement and its cable must clear the cart across the calibration poses before another powered sweep. Prefer checking the identified affected joints with the user watching instead of repeated whole-arm motion.

### Left retry after repositioning passed all arm checks

Initially both serial boards reappeared but servos did not reply. After owner confirmed power lights, read-only preflight found both buses responding normally (left119/right121 raw V, torque0). Owner explicitly requested left calibration again after repositioning. Full LEFT velocity200 run exited0 and saved. Left raw limits/offsets: shoulder_pan959..3135/-22; shoulder_lift855..3239/975; elbow942..3152/107; wrist_flex898..3196/-826; wrist_roll115..3979/1318; gripper1275..2819/121. Travel:191,210,194,202,340,136 degrees respectively. Right previous automatic results preserved.

Final report: NO ARM FLAGS OR LEFT/RIGHT MISMATCHES. Only old head_motor_1 range0..4073 remains flagged as wrapped. Explicit readback verified ALL16 Torque_Enable=0 and calibration file offsets/limits equal hardware. Snapshot parent task work/calibration-arms-passed.json; verification work/arms-calibration-verification.json. This is now the newest calibration backup. Do not restore older manual/partial/camera-contact snapshots over these results. User has not yet provided visual clearance confirmation for this latest full run; motor data alone cannot rule out all contact. Head pan still needs supervised manual calibration; no whole-robot powered self-test until report is clear.

## Manual head calibration follow-up

- Corrected the head wrapper to write and read back measured head limits, reject implausible ranges, save atomically, and restore prior hardware settings on interruption or failure. Head motors remain torque-off; only head IDs 7 and 8 are selected. All 11 wrapper tests passed, including rollback for interruption, invalid ranges, readback mismatch, and failed file save.
- First recording was cancelled after the user corrected the forward reference. Second recording was cancelled after the user reported possibly turning too far left. Both cancellations restored and verified the previous head settings with torque off; the calibration file and successful arms were unchanged. No completed new head calibration yet.

### Completed manual head calibration

The user reset the head straight ahead and level, then manually swept left/right (up to approximately 90 degrees each way) and comfortable up/down travel. The routine recorded pan 1019..3151 and tilt 1932..2665, validated the ranges, wrote and read back the head settings, and atomically saved the merged calibration. The full calibration report is clear. An independent read-only check verified all 16 motors torque-off, all hardware offsets and limits matching the saved file, and every non-head calibration entry unchanged. No new powered motion test was performed.

### Movement test attempt after final calibration

The read-only position/health preflight stopped before any motor enable: right elbow raw position 3193 is beyond its saved maximum 3092 (minimum 1002), about 8.9 degrees beyond. Other arm/head positions were within saved limits; no servo-status faults were reported. Torque-off commands succeeded. The movement-and-return stage has not run. The user needs to reposition the right elbow gently inward while torque is off; do not widen the calibration to suppress the check.

The next repositioning check read the right elbow at the same 3193; it remains 101 ticks beyond the saved maximum. No joints were enabled or nudged. Torque-off commands succeeded again.

### Partial movement test after elbow repositioning

The user correctly repositioned the right elbow (normalized 84.7), and all position/health preflight checks passed. A 3-unit head tilt request moved only 1.09 normalized units (0.35 degrees), below the test's movement threshold. After a torque-off verification, the head tilt request was increased to 6 units (1.93 degrees); both head motions passed movement-and-return checks, as did left shoulder pan/lift. The left elbow started at normalized 98.64, requested 95.64, and reached only 98.37 (about 0.26 degrees of movement), failing the movement threshold. The test stopped and sent torque-off commands. No communication or load threshold failure occurred in this attempt. The left elbow needs inspection/repositioning away from its end pose before another attempt. Detailed results are in the parent task's `work/movement-test-left-elbow-small-step.json`.

### Movement test resumed: right claw remains unresolved

After the user repositioned the left elbow, its reading was normalized 85.6 and the nudge passed. Left wrist flex/roll and gripper passed; right shoulder pan/lift, elbow, and wrist flex/roll also passed. The final right-gripper nudge aborted. All motors were released and independently read back torque-off. Saved calibration still matches hardware. Ten stationary samples of that claw returned fault status 0, load 0, and 11.9–12.1 V. All 13 other arm/head small movement-and-return checks have now passed across the logged attempts, but visual identity confirmation and the right claw check remain incomplete. Evidence: parent task `work/movement-test-summary.json`.

### Final isolated claw attempts

Two right-claw movement attempts failed to reach the position target within the three-second leg timeout. The second attempt used explicit position speed 200 and a bounded 5.3-degree request: commanded raw goal 1430 verified, actual reached 1393 from 1370. Torque-off was verified at exit and the original position speed was restored. Right claw positioning remains unresolved; 13 other checks passed. A phone camera setup is now the active user request.

### Right claw passed and settings persisted

The isolated standard movement test also failed at P=16: raw goal 1460 from 1393, actual 1413, return command 1393. A bounded diagnostic used P=32 (the vendored driver documents this as the servo default), lowered Torque_Limit to 500, and explicit position speed 200. The commanded step was 4.5 normalized units / 5.97 physical degrees, within the profile's 6-degree step limit. The claw moved 3.245 normalized units and returned within the standard test tolerance. These settings were added as a right-claw-only profile override, with validation and hardware readback before enabling torque. A second check loaded the saved profile and passed: moved 3.046 units and returned within tolerance. Final independent readback verified all 16 torque-off, every calibration entry matching hardware, and all three saved claw settings matching hardware. All 14 basic arm/head movement checks have now passed. The repository suite passed 93 tests.


### 2026-10-06 JST: reproduce prior velocity-200 calibration, incomplete

User authorized both-arm full calibration and camera-monitored retry after confirming clearance. Left prior-method-01 aborted on missing ID1 telemetry before movement; subsequent read-only precheck showed all12 arm motors status0/torque0. Left prior-method-02 initialized and unfolded but endpoint settling timed out after1s with elbow position2098–2099 and alternating velocity/Moving telemetry. All6 left releases verified. Extended settling observation window to3s while retaining three consecutive low-velocity/Moving0/stable-position samples; focused suite29 passed, including delayed chatter acceptance and persistent movement rejection. Left prior-method-03 at velocity200,20s passed settling but measured shoulder31.6deg (invalid120–230 bounds), elbow157.5deg. Shoulder first leg load reached800 with position132–139; resistance source unproven. No calibration saved; all6 releases verified. Partial homing/mode/EEPROM changes may remain; do not assume saved calibration matches hardware. Right-arm sweep awaits complete camera coverage. Logs in local task work/auto-calibration-left-prior-method-01..03 and outputs/auto-calibration-left-prior-method-2026-10-06. No new reusable default or remote publication validated.


### 2026-10-06 JST: unchanged upstream PR3282, default velocity1000

Owner explicitly requested exact PR code and selected default1000. Downloaded unchanged workflow/mixin/defaults from PR head1c8e185e3694def2469d7d312c78dab217e336da; hashes in task outputs/pr3282-original/provenance.json. Loader only attaches upstream mixin to installed driver, holds exclusive ownership, redirects staged result path. No local range minimum or settling/limit-seeking overrides. Full LEFT run initialized/unfolded, measured shoulder213.4deg and elbow196.7deg, applied/read back their partial calibration, then upstream aborted during simultaneous wrist/claw seeking: wrist_flex ID4 Present_Position had no status packet after5 attempts. Exit1, no complete result saved. Independent postcheck confirmed all12 arm motors torque0/status0. Partial hardware calibration remains; saved XLeRobot file has not been merged or verified against new partial settings. Right not run: camera coverage still obscured. Task outputs/pr3282-original/left-run-result.json records exact failure. No calibration completion or preset-success claim.


### 2026-10-06 JST: owner-requested unchanged PR1000 retry

Fresh precheck all12 torque0/status0; owner confirmed people clear. Repeated full LEFT original PR1c8e185 at1000/20s, no local range/settling overrides. Unfold succeeded; shoulder161.8deg and elbow151.0deg partial ranges written. Same distal stage again lost wrist_flex ID4 Present_Velocity feedback then failed5 Present_Position reads. Ctrl-C sent when fresh frame showed new person at cart; failure had already reached exit prompt. Process terminal exit1. Independent all12 torque0/status0 check passed. No full result saved; partial hardware settings remain, do not run normal control based on old calibration file. Task outputs/pr3282-original/left-retry-result.json. Need diagnose motion-dependent distal-chain feedback loss before repeating full powered sweep.


### 2026-10-06 JST: upstream velocity300 left completed; orientation unresolved

Owner requested300. Original PR full LEFT exit0, all6 saved into task-local candidate and EEPROM; independent offsets/limits readback matched all6, mode0 torque0. Saved spans:pan87.19deg, shoulder162.07, elbow148.36, wrist202.32, roll340.14, grip137.29. No local minimum/settling checks altered upstream run. Candidate not merged into live XLeRobot file: pan drastically shorter than prior191deg, working-side orientation unresolved. Started RIGHT300 after fresh wider camera showed both arms; owner then reported calibration behind cart rather than front and camera moved/cut off active right arm. Sent Ctrl-C; upstream returned130 during shoulder/elbow phase. All12 torque0/status0 independently verified afterward. Right partial settings may remain; do not treat old saved calibration as hardware-matched. Need establish intended cart front and starting yaw/clearance before next powered sweep. Outputs/pr3282-original/left-300-candidate.json and left-300-readback.json preserve completed but unvalidated left candidate.


### 2026-10-06 JST: front-facing right300 retry, failed cleanup — power-off required

Owner confirmed white-table front, repositioned/folded both arms and requested start. Left all6 servo reads silent before motion; right all6 status0/torque0. Proceeded RIGHT upstream300 independently. Unfold stage completed; shoulder/elbow phase lost shoulder_lift ID2 feedback, all5 Present_Position attempts failed. At exit/disconnect Torque_Enable0 write to ID1 received no status packet after6 tries. Process terminal exit1, no new right result saved. Postcheck both buses/all12 arm motors silent. Torque release NOT verified. Immediately instructed owner to switch BOTH12V supplies OFF; confirmation pending. Do not resume motion until physical power-off confirmed and cause/readback established. Failure outputs/pr3282-original/right-300-failed-stop.json; current-bus-status.json.


### 2026-10-06 JST: both reseated, right300 load-dependent bus loss repeats

After owner reseated supplies/first-servo leads, all12 Status0/Torque0 preflight passed. Original PR RIGHT300 started. First shoulder/elbow stops at1613/3797 then shoulder_lift ID2 Torque_Enable1 failed3 attempts with no packet. Upstream disconnected with ID1 Torque_Enable0 write failing6 attempts. Terminal exit1, no result saved. Independent postcheck LEFT all6 Status0/Torque0, RIGHT all6 silent. Thus failure isolated to loaded right bus this time; power or chain connection remains hypothesis, not established. RIGHT torque release NOT verified. Immediately told owner to switch RIGHT12V OFF; confirmation pending. Outputs/pr3282-original/right-300-after-reseat.json. No further powered motion until power-off and fault cause resolved.


### 2026-10-06 JST: left-only300 with right12V off

Owner explicitly confirmed right12V OFF and requested left alone. Fresh left all6 Status0/Torque0, camera clear. Original upstream PR full LEFT300/20s measured shoulder212.0deg and elbow194.8deg, wrote partial offsets/limits, then lost wrist_roll ID5 Present_Position feedback after5 attempts during simultaneous distal-joint stage. Terminal exit1. Fresh independent left all6 Status0/Torque0 verified. No new complete result saved; prior left30087deg-pan candidate remains task-local, not a valid current hardware snapshot. Right remains owner-confirmed unpowered. Evidence outputs/pr3282-original/left-only-300-retry.json. Both-arm calibration remains incomplete; current saved XLeRobot calibration not assumed equal partial hardware settings.


### 2026-10-06 JST: left-only300 next retry completed, candidate unvalidated

Owner requested retry with right12V remaining confirmed OFF. Original PR1c8e185 full LEFT300/20s exit0, no communication error. All6 saved candidate and EEPROM. Independent all6 homing/min/max equality, Operating_Mode0 and Torque_Enable0 verified. Saved travel degrees:pan87.36, shoulder161.19, elbow149.94, wrist202.32, roll340.49, grip138.16. Pan again materially shorter than prior191; shoulder/elbow shorter than immediately preceding failed212/195 run. Thus upstream completion does not prove full unobstructed mechanical calibration. Candidate preserved task outputs/pr3282-original/left-only-300-completed-candidate.json; readback left-300-readback.json; result left-only-300-completed-result.json. Live XLeRobot calibration file NOT replaced; do not assume hardware matches old live calibration. Right remains off; both-arm validation and reusable success preset unfinished.


### 2026-10-06 JST: resting-pose request and upstream fold discrepancy

Owner requested left hand resting pose after completed but unvalidated87deg pan candidate. Left all6 read Status0/Torque0; elbow mode0/homing-150/limits1194..2900. Bounded low-torque elbow positive34tick step (2.99deg), torque300, speed100,acc5,failed target1243 from1209: final1217 after3s. Max recorded load128, no fault; no physical cause established. Torque release and prior settings restoration verified. No further powered movement performed; asked owner to support/poweroff/manualfold. Evidence outputs/left-rest-step-01.json. Independent code inspection confirms original PR _fold_arm waits for Moving0 but prints reached target without tolerance check. Prior pre-pan pose reported elbow1413 versus1194 target (19.25deg error), wrist1328 versus896 (37.97deg error). Thus nominal folded clearance pose was not established even when upstream exits0. Do not call candidate fully calibrated or replace live file. Diagnosis outputs/pr3282-original/fold-clearance-diagnosis.json. Left still extended in latest camera; await supported resting pose before further full sweeps.


### 2026-10-06 JST: owner-requested larger resting movement succeeded to compact pose

Small elbow moves using temporary torque300 did not reliably settle/retain position; all releases/settings restorations verified. Owner explicitly requested larger movements. Used existing Torque_Limit800 (unchanged), speed100,acc5 with gradual40tick position target increments (3.52deg each, below profile6deg limit), coherent status/load<=800/voltage/tracking<=96tick checks. Three monitored nominal30deg segments moved elbow1273→1581→1894→2220 (83.23deg net). Camera showed forearm retracting upwards; stopped progression before hand approached mast.15s hold deadline expired and cleanup verified all6 torque0/settingsrestored. Independent all6 mode0/status0/torque0; elbow2220 remained with torque released. Thus compact intermediate rest achieved; not identical to original fully folded starting pose and not a valid calibration. No offsets/limits changed by resting helper; right12V remains owner-confirmed off. Evidence outputs/left-rest-normal-session.json and Left-Arm-Resting-Pose.json. Full left calibration still incomplete: prior87deg pan candidate unvalidated.


### 2026-10-06 JST: starting-joint restoration and separate right calibration chat

Compared shortened candidate with last verified Oct5 snapshot. Restored LEFT pan(-22,959..3135) and elbow(107,942..3152) from that snapshot, with torque initially verified off and fresh hold goals written in the new coordinate frame. Pan unexpectedly read Torque_Enable1 after EEPROM restoration; no deliberate torque-enable command was issued. Immediately explicitly disabled ALL6 left motors; independent all6 Torque_Enable0/Status0 verified (outputs/left-release-after-restoration.json). Physical encoder reference remained stable. Restoration settings/backup: outputs/pr3282-original/starting-joints-restoration.json. Left remains mixed old/candidate settings, not newly validated, and normal control must not assume old live file matches hardware. Owner explicitly requested RIGHT calibration in another chat. Created Calibrate XLeRobot right arm, thread01a11043-3c2d-75d0-9ba0-ce89d1db0fa0, with upstream300/20s, full source/physical state, exclusive ServoOwnership and failure-release instructions. Parent will HOLD powered work while that chat uses robot; never concurrent powered arm sweeps.


### 2026-10-06 JST: separate right upstream300 run interrupted for camera coverage

Fresh both-arm preflight all12 Status0/Torque0; exclusive ServoOwnership held through unchanged PR1c8e185 RIGHT300/20s and cleanup/readback. Unfold completed; shoulder/elbow first stops1614/3790. Right claw left phone image boundary during outward movement, preventing clearance monitoring; sent Ctrl-C before completing sweep. Upstream130, no complete calibration saved or merged. Independent all12 Status0/Torque0 verified. Right shoulder/elbow remain Operating_Mode1, homing0/limits0..4095; partial settings must not be treated as matching live calibration. No communication failure observed in this attempt and no definite contact established. Await wider fresh camera coverage and people-clear confirmation before further powered movement. Evidence: /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/run.log, preflight.json, release-readback.json, run-result.json.


### 2026-10-06 JST: right300 repeat loses feedback — RIGHT12V off required

Owner widened phone camera. Waited for left-front-pan-positioning ownership to release, then acquired exclusive lock; all12 Status0/Torque0 fresh preflight passed. Unchanged original PR1c8e185 RIGHT300/20s unfolded, then shoulder_lift ID2 Present_Position failed all5 reads during simultaneous shoulder/elbow limit seeking. Upstream disconnect ID1 Torque_Enable0 failed6 attempts; independent emergency release/readback attempted every right motor, evidence below. Torque release UNVERIFIED; immediately instructed owner RIGHT12V OFF, confirmation pending. Process exited1; no complete candidate saved or merged. This repeats motion-dependent right-bus loss; actual physical cause remains unproven. Do not repeat powered calibration until diagnosed. Evidence /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/run.log, run-result.json, emergency-release.json.


### 2026-10-06 JST: left pan aligned; right chat owns motion then fails feedback

After separate right run130 completed and all12 torque0/status0 readback, parent slowly aligned LEFT pan from2888 to2052 using restored validated offset-22/limits959..3135; gradual40tick steps, speed100,acc5, existing torque800, bounded telemetry/tracking. Target2047, achieved2052 (0.44deg error); physical total73.48deg.15s hold deadline released motor; all6 left torque0 and settings restored. Evidence outputs/left-front-pan-positioning.json. Prepared elbow-fold trajectory toward3072 within restored validated limits942..3152, but execution refused before connection because right chat resumed and held ServoOwnership; NO elbow motion from this attempted setup. Right second run then terminal failed: bus silent and emergency torque-off unverified. Parent holds additional LEFT motion pending owner-confirmed RIGHT12V OFF in right chat. Full-left calibration remains incomplete; left mixed settings described above.


### 2026-10-06 JST: right restart restores stationary replies

Owner reported restarting right arm. Exclusive read-only check at17:17:59 JST received all6 right motor replies: Status0, Torque_Enable0. Shoulder/elbow remain Operating_Mode1 from interrupted calibration; other4 mode0. No movement or register writes. Communication recovered at rest, but movement-dependent failure cause remains unproven; no further calibration retry authorized by this status question. Evidence: /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/right-online-after-restart.json.


### 2026-10-06 JST: right stationary diagnostic after restart

User requested autonomous completion. Exclusive read-only 30-sample coherent audit: all6 replied throughout, voltage120..121 raw, status0, initial torque0. Shoulder/elbow mode1; their homing0 combined with restored stored limits makes current settings mixed. Released elbow reports Moving1/velocity50 intermittently despite stable2503..2504 position and current/load0; not evidence of mechanical travel. No powered reposition or sweep attempted: repeated prior complete right-bus losses and unacknowledged torque releases remain unresolved by reboot. Calibration incomplete. Diagnostic evidence /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/stationary-diagnostic.json and diagnostic-summary.json.

Diagnostic correction after aggregating all30 samples: gripper reported112raw once (11.2V) while proximal pan/shoulder stayed121; wrist119..120. Thus complete voltage envelope11.2..12.1V, not only final-sample12.0..12.1V. Coherent packet evidence retained; isolated low distal report does not establish cause.


### 2026-10-06 JST: right link settings verified without motion

All6 right firmware3.10, Baud_Rate0, Return_Delay_Time0, Response_Status_Level1, Status0/Torque0. No wrong/inconsistent stationary link settings identified. macOS log query for serial-number/usbmodem events in failure window returned no matching entries; does not exclude USB issue. No physical cause proven; no powered retries or reposition. Power/chain integrity under load remains required before unchanged full routine. Evidence current task outputs/right-link-settings.json and failure-diagnosis.json.


### 2026-10-06 JST: right full-calibration goal blocked after diagnostic audit

Same load-dependent whole-right-bus loss remained unresolved across3 autonomous goal turns. Fresh17:20:54JST all6 status0/torque0; camera fresh, no owner process running, no physical remediation evidence. Full calibration not saved, travel/EEPROM match not validated. Marked goal BLOCKED pending physical power/chain inspection with12Voff and supported load validation; no blind powered retry. Audit current task outputs/blocked-audit.json.


### 2026-10-06 JST: replacement right12V cable — passed shoulder/elbow, coverage stop

Owner replaced12V cable and requested retry. Fresh all12 Status0/Torque0; exclusive lock. Unchanged original PR1c8e185 RIGHT300/20s completed shoulder/elbow calibration without communication loss. Applied/readback partial right shoulder homing785 limits829..3265 (214.10deg), elbow632 limits935..3159 (195.47deg). Fresh camera showed claw beyond left edge during raised/outward pose; Ctrl-C before distal seek. Upstream130; independent all12 Status0/Torque0, right all6 mode0 verified. No completed candidate saved or merged. Cable change correlates with improved loaded communication in this attempt, not proven root cause/resolution. Need additional left margin camera coverage for full right sweep. Evidence current right task outputs/cable-replacement-retry/run.log, release-readback.json.


### 2026-10-06 JST: wider-view right300 fails distal feedback despite new12V cable

Owner fixed camera left margin. Exclusive original PR1c8e185 RIGHT300/20s full retry preflight all12 Status0/Torque0. Shoulder/elbow this time127.6/90.5deg (far shorter than immediately preceding214/195); partial homing1278/35 limits1321..2773 /1532..2562 applied. Distal simultaneous wrist_roll/gripper/wrist_flex first stops653/3129/2427. Wrist_roll ID5 Present_Position then failed5 reads, all right feedback lost. Upstream ID1 torque-off write failed6 tries; independent release attempts ALL6 returned SDK-6/no packet. Left all6 Status0/Torque0 verified. Immediately instructed RIGHT12V OFF; confirmation pending. No complete candidate saved/merged. Replacement cable did not eliminate failure; physical cause unproven. Evidence right task outputs/wider-view-retry/run.log, release-readback.json, run-result.json. No blind repeated sweep.


### 2026-10-06 JST: tracked repeatable upstream calibration tooling

Added pinned unchanged PR3282 sources1c8e185 and hash manifest, plus scripts/carton_robot/upstream_pr3282_calibration.py: default300/20s, plan-only default, explicit execution/clearance prompt, both-port ownership, selectable physically confirmed other-arm-poweroff preflight, unique evidence directories, original candidate staging, emergency release of every selected motor and readback verification. Post-run quality rejects incomplete/short/wrapped/wide candidates separately from upstream sweeps; optional --install uses atomic selected-arm merge, preserves other joints, and refuses externally changed live files. Updated procedure/skill to distinguish prior success from current unresolved failures and unsupported fold-success messages. Focused verification57 tests passed; source-loader and plan-only smoke passed; no motors opened or moved for this code update. No new calibration validity claim: current physical goal remains blocked pending RIGHT12V OFF confirmation.


### 2026-10-06 JST: post-new-cable right goal blocked again

Three consecutive goal turns retained same feedback/release blocker after wider-view run failed. Fresh right read-only check all6 silent; last independent emergency release attempts all6 unacknowledged. RIGHT12Voff confirmation still absent. Marked goal BLOCKED; full calibration incomplete, no result merged. Physical diagnosis/remediation required before powered calibration. Evidence current right task outputs/wider-view-retry/blocked-audit.json.


### 2026-10-06 JST: battery-wire rearrangement restores all16 replies

Owner reported rearranging battery wires, suspected battery path. Fresh exclusive read-only all16 check: every arm/head/wheel servo Status0/Torque_Enable0, voltage120..122raw (12.0..12.2V). No motor movement or register writes. Right IDs2..6 remain Operating_Mode1 with homing0 and prior EEPROM limits; right elbow3774 beyond stored max3092 in current mixed frame. Thus normal powered position control is not ready, calibration remains incomplete, no live file merge. Changed power arrangement correlates with recovered stationary replies; failure cause/resolution under load not yet proven. Evidence /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/all-servo-check.json.


### 2026-10-06 JST: power recovered, camera service restarted before calibration

All16 stationary replies recovered after battery-wire rearrangement. Autonomous calibration resume checked phone freshness: saved frame was ~36min old; old server97755 missing and head snapshot stale. Restarted existing phone server (no secret printed); now listening/waiting, seq0/no received frame. Camera tunnel process absent. No motor movement started. Tracked unchanged-upstream300/20s plan/provenance check passed. Need restored fresh complete phone view before sweep; no inference of clearance from old image. Evidence current right task outputs/camera-recovery.json.


### 2026-10-06 JST: right goal blocked by absent live clearance feed

After restored all16 idle replies,3 consecutive autonomous goal turns had no fresh complete camera feed. Restarted server session17395 confirmed running; seq0/received_atnull. Old phone frame ~36min stale, wired head snapshot stale. Local camera QR regenerated from current config without printing secrets; external tunnel creation rejected by automatic approval review, not bypassed. Goal BLOCKED pending phone sharing via local QR. No powered motor operations started after battery wiring change. Evidence current right task outputs/camera-blocked-audit.json.


### 2026-10-06 JST: battery rewiring right300 retry still loses whole bus

Owner restored fresh Cloudflare phone feed, explicitly confirmed clearance/goahead. Tracked unchanged PR1c8e185 RIGHT300/20s with exclusive ownership, staged output and backed-up live file. Shoulder214.6deg/elbow196.0deg measured/applied; fold readings843 vs826,3140 vs3162 supported near-target position. During simultaneous distal seek first CW stops roll653/grip3151/wrist2382, wrist_roll ID5 position failed5 retries. Upstream ID1 torque-off failed6 tries; outer cleanup Goal_Velocity0 and Torque_Enable0 failed4 tries EACH for ALL6, readback failedALL6. Exit1; release UNVERIFIED, no complete candidate and no install. Immediately instructed RIGHT12V OFF; confirmation pending. Battery rewiring restored idle communication but did not resolve load-dependent failure. No blind repeat. Evidence /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/battery-rewired-retry/result.json and release-readback.json.


### 2026-10-06 JST: wall-powered right300 full routine completes, pan range unresolved

Owner switched right to wall supply after reporting0V at failure on battery. Fresh all16 Status0/Torque0; right124..125rawV. Bounded pan34tick roundtrip passed (2085→2115→2089), torque300/speed100/acc5, all6 release and prior settings restoration verified. Then tracked unchanged PR1c8e185 full RIGHT300/20s completed exit0 without communication failure. Saved staged candidate, independent all6 hardware offsets/min/max equality, mode0, and all16 Status0/Torque0 verified. Travel: pan240.47, shoulder214.63, elbow196.00, wrist196.52, roll338.55, grip136.58deg. Validator refuses installation because pan differs49.2deg from saved left191.25; previous verified right206.02 also34.45deg narrower. No definite cart contact identified in sampled fresh camera views, but wider pan endpoints require independent quality resolution. Candidate NOT merged; do not claim fully calibrated. Live file does not match new right EEPROM. Wall supply completed this run where battery failed; exact battery/cable shutdown mechanism unproven. Evidence /Users/teachera/Documents/Codex/2026-10-06/xlerobot-right-arm-calibration/outputs/wall-powered-full01/validation-audit.json, candidate/robots/so_follower/right.json, independent-all16-readback.json.


### 2026-10-06 JST: owner accepted/installed wall-powered right candidate; left rested

Owner explicitly chose to assume wider right calibration is better and requested remote update. Fresh right all6 homing/min/max, mode0/status0/torque0 verified; backed up live file, atomically merged only right6, reread file, verified every other entry unchanged. Right pan240.47deg remains owner-accepted by assumption, not independently full-range validated; prior206.02/left191.25 mismatch not suppressed in validator. Right installed calibration now matches hardware. Versioned accepted candidate in docs/evidence/right-arm-calibration-2026-10-06.json.

Owner requested LEFT resting pose. Initial helper refused before motion because EEPROM torque1000 differed from expected800; all6 released. Used temporary LOWER torque800, speed100/acc5,40tick goal increments within restored elbow942..3152. Camera-monitored segments1964→2296→2636→2970 formed compact resting pose. First interactive hold15s expired safely; later segments release immediately. All6 left release/settings restoration verified, followed by independent all16 Status0/Torque0; right calibration still equals hardware. No left calibration registers/head/wheel movement. Left calibration remains mixed, not newly validated. Evidence docs/evidence/left-arm-resting-pose-2026-10-06.json and current right task outputs/final-all16-verification.json, Left-Resting-Pose.jpg.


### 2026-10-06 JST: drive/reach request — head camera verified, no movement yet

Owner requested forward cart movement toward white table and arm extension near tagged paddle, plus head-camera verification. Explicit request authorizes a bounded wheel test despite parked profile; tracked wheels flag left unchanged. Fresh exclusive all16 read-only preflight: Status0/Torque0/mode0; all RIGHT6 installed offsets/min/max match hardware, both wheels/head match saved file; LEFT shoulder/wrist/roll/gripper remain mismatched/mixed. Selected right arm for prospective reach, left parked. Native exact-ID AVFoundation capture via Terminal: previous head identity0x12130005a39230 absent; current externalIDs0x12400005a39230,0x12200005a39230,0x12140005a39230 all returned real frames.0x1240 elevated forward wide room/table/paddle view identifies head; other captures show wrist/claw closeups. Do not infer OpenCV numbered mapping from native enumeration; no profilecamera mapping edited. Camera evidence current task outputs/Head-Camera-Verified.jpg/json and Camera-Devices-20261006.txt. Floor camera shows power leads; cart now wall-powered/tethered. Asked owner to confirm cable slack, no lead under wheels, clear5cm forward path. NO cart or arm motion in this request yet; dependent drive/reach awaits physical clearance answer.


### 2026-10-06 JST:20cm forward demo and paddle handoff readiness

Owner confirmed cable slack/no leads under wheels and requested20cm pulses. One logical20cm advance at.02m/s,11 motor pulses <=1s, strict coherent fault/load<=500/velocity<=400/voltage checks, acknowledgement/readback of both commands, zero targets, <=.9s powered-stop confirmation using five stable encoder samples and actualzero observation per wheel, separate stationary-after-release check, mode/acceleration/torque/lock restoration and post-restore release checks. Encoders estimate left19.8957cm/right20.0875cm (mean19.9916cm); floor travel not independently measured. Fresh phone frames inspected between pulses; no definite contact identified. Evidence outputs/Table-Drive-20cm-Result.json and Table-Drive-Pulse01..11. No other wheel loop or turn.

Head camera identity changed: exactID0x12400005a39230 native AVFoundation through Terminal captures valid1920x1080 forward table views; all3 USBcameras capture. Original headID absent. No OpenCV index remap inferred. Right-arm installed calibration freshly matched hardware. Right guided extension used only5 positioning motors, jaw left off/closed; exact settings verified/temporary lower torques (elbow400/P32, others<=800), speed100/acc5,40tick increments, per-joint saved limits, load/status/voltage/tracking checks, live-phone freshness. Initial pre-motion SDK-7 wrong-ID reply aborted/released; subsequent released precheck passed. Controlled shoulder/elbow/pan moves unfolded arm but did NOT establish paddle pregrasp. Timed holds and final stale phone feed ended sessions with all6 right releases/settings restoration verified. Current right pose partly extended, not identical to starting fold. No jaw closure, contact or pickup performed.

Owner referenced15b5b36 handoff. Fetchedorigin/main without changing working tree; read document/inventory; all37 files hash-match. Native fresh-capture tag audit detects1table and3paddle (Hamming0/margins55.86/46.19), no2fixedgripperhousing in3USBviews. No metric pose or current registration/jawhandle transform verified. Local robot-tools TCP1241 reachable, recorded Gemma gateway192.168.100.142:1240 times out; DevMac pilotpath unavailable here; old ownerstatus stopped~7h old. No new owner or registration motion started. Phone feed stale; requested restart/wider co-visible1/2/3 view and current Gemma connectivity. Independent finalall16 Status0/Torque0 verified. Evidence outputs/Paddle-Handoff-Readiness-Summary.json, source audit, tag audit, owner readiness and original camera images. Approach near paddle remains INCOMPLETE pending observation/registration readiness; do not claim picked up or use simulated offsets.


### 2026-10-06 JST: phone feed restored, tags1/2/3 co-visible

After server/tunnel disappearance, restarted existing camera server on8876/8877; new camera HTTPS quick tunnel uses local HTTPS8877 with existing CA verification, authenticated status passed and unauthenticated status401; QR decoded roundtrip matches configured link. Owner requested killing Gemma tunnel; revalidated PID2077 TCP127.0.0.1:1241 and terminated it, camera tunnel separate. Owner restarted phone upload: freshseq93/frame age0.106s. Detects1table,2gripperhousing,3paddle; tag1 one corner x=-1.39 clipped, Hamming1, margin48;2/3 Hamming0/margins32/46. Requested slight wider framing. Both arms fresh Status0/Torque0; no new motor motion. No matching registration/contact transforms verified and no pickup claimed. Evidence outputs/Phone-Camera-Recovery.json, Phone-Readjusted-Tag-Check.json and Phone-Camera-Readjusted-Readiness.json.


### 2026-10-07 JST: paddle commissioning archived; pickup incomplete

See docs/commissioning/2026-10-07-paddle/README.md and its evidence/scripts archive for the 20cm encoder-estimated drive, bounded non-contact reach/return, failed camera registration, provisional depth/CAD offsets, and restored 7 October connectivity. No verified paddle pregrasp/closure/lift/place. Requested40deg arm and40cm cart operations remain incomplete. All16 last read-only responses Status0/Torque0; no present-state guarantee. First phone QR was corrected to the raw URL-encoded bearer fragment; secrets, QR images and footage were not committed. Model zero/sign binding and current camera-to-arm registration remain absent. Archived prototypes are opt-in, workstation-specific experiments, not production controllers.
