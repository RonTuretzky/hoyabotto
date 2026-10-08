# Physical paddle pickup: successful commissioning procedure

Verified on 7 October 2026, robot-local right arm. This is the first observed successful pickup in this session, not a repeatability or autonomous-policy claim. The operator explicitly confirmed the lift. The paddle was subsequently lowered onto the table, released, and the arm withdrawn. All six right-arm torque releases and original controller-setting restoration were read back.

The original [AprilTag/CAD handoff](../../paddle-apriltag-pickup-handoff.md), introduced at commit `15b5b36`, supplied the scene/tool context. Actual execution here was camera-guided encoder control. It did not use the unaccepted Cartesian registration, simulated initial grasp, or exploratory CAD fit as a physical target.

## Exact successful sequence

The table records actual starting encoders, commanded absolute targets, and measured settled endpoints. Commands are relative to fresh measured positions; the differences between consecutive starts and previous endpoints are real drift/settling, not transcription errors. Units are raw encoder ticks. Approximate joint degrees use 4096 ticks per revolution.

| Step | Interactive command | Actual start | Commanded target | Measured endpoint |
| --- | --- | ---: | ---: | ---: |
| 1 | `elbow_flex 100` | 2798 | 2898 | 2862 |
| 2 | `shoulder_lift -341` | 2781 | 2440 | 2453 |
| 3 | `shoulder_lift -130` | 2453 | 2323 | 2336 |
| 4 | `shoulder_pan 40` | 2122 | 2162 | 2160 |
| 5 | `shoulder_pan 40` | 2160 | 2200 | 2197 |
| 6 | `elbow_flex -114` | 2901 | 2787 | 2796 |
| 7 | `CLOSE` | 1592 | 1310 | 1350 |
| 8 | `shoulder_lift -60` | 2336 | 2276 | 2296 |
| 9 | `shoulder_lift -60` | 2296 | 2236 | 2257 |
| 10 | `shoulder_lift 79` | 2257 | 2336 | 2336 |
| 11 | `gripper 260` | 1349 | 1609 | 1586 |
| 12 | `elbow_flex 114` | 2791 | 2905 | 2903 |
| 13 | `shoulder_lift 341` | 2339 | 2680 | 2681 |

Step 7 stopped closing at target 1310, actual 1350, raw load 328. Empty-jaw baseline was target 1310, actual 1331, load 176. The resistance report itself returned `grasp_verified: false`: only the subsequent visual lift/hold and operator confirmation established the physical grasp. Steps 8 and 9 lifted the paddle while it stayed between the jaws. Step 10 returned the shoulder to the pre-lift measured position. Step 11 opened the jaws. Steps 12–13 withdrew/parked the empty arm. `STOP` then released and restored settings.

The successful held target before closure was pan 2200, shoulder 2323, elbow 2787, wrist flex 1579, wrist roll 1242, jaw 1592. The held lift targets were shoulder 2276 then 2236, with the other targets fixed and jaw closing target 1310. These are station-specific observations, not universal pickup keyframes.

## Preconditions and physical layout

- Right arm on bus B, servo IDs 1–6; left/head on bus A; wheels 9–10 on bus B. Left/right mean robot perspective.
- Both buses are exclusively locked for the entire pilot session. Do not launch this prototype while the canonical tool owner owns the ports.
- The right arm's hardware homing/limits matched the saved calibration. Full-file SHA256 and six right-arm entries are in `success.json`. Calibration and firmware protection were not rewritten.
- White paddle lies on the white/wood table with its handle projecting beyond the near edge. Table tag 1 is 60 mm; fixed gripper tag 2 and paddle tag 3 are 40 mm, using the operator-confirmed printed defaults. Tag size was not independently ruler-verified.
- Cameras: head-mounted OAK, native right wrist USB camera, and a phone side view showing the handle and both fingers. The new phone uploaded 540×960 JPEG frames. Receipt age was checked from `received_at`; cached `live: true` was never used as freshness proof.
- Before the successful reach, the empty/released cart moved 5.4303 cm forward toward the paddle, in three 1-second guarded pulses at nominal 0.02 m/s. Every pulse verified wheel stopping, release, and setting restoration. This is not a 40 cm drive.
- Through the API (owner profile `paddle-success-v1`): enable all six right-arm joints in one `robot_set_motor_enable` call; every other command is refused until all six are enabled. Each joint must be inside its saved range (4-tick margin) at enable; a joint resting near a limit can be enabled and driven back inward, since every target stays 40 ticks inside. The phone feed (`work/phone_camera/latest.json`) must have `received_at` under 10 s old with a `seq`, or enable and each new target are refused; after 20 s stale while holding, the owner releases torque.
- The owner allows 20 motion segments per session, including `settled_short` moves; a STOP does not reset the count, only an owner restart does, and a restart needs the arm released. The 13 steps below are 13 segments; steps 12 and 13 can be one simultaneous call.
- Approximate start pose before step 1: pan 2122, lift 2781, elbow 2798, gripper 1592 (measured starts), wrist flex 1579 and wrist roll 1242 (held targets). Re-measure before every move.
- Earlier, an approximately 39.375° wrist-roll movement was recorded in two segments; the wrist was later returned to the original width-grip orientation near 1242 ticks. The thin-edge orientation near 2251 ticks did not produce a grasp.

## Applied controller settings and guards

The archive contains the exact successful pilot source with only a default-off execution gate prepended. It is host-specific commissioning code, not a production skill or a client-side Qwen driver.

- Initial all six right-arm torque/status checks, position mode, hardware-versus-file homing/min/max match, and current-position goals before torque enable.
- Temporary positioning-joint torque ceiling 800, elbow ceiling 400, jaw ceiling 500. Original limits were 1000 and were restored. No saved travel limits were widened.
- Temporary shoulder and elbow P coefficient 32; original 16 restored. Jaw P was already 32. Integral/damping and firmware protection settings were retained. The local pilot neither read nor acted on temperature.
- Position speed 100; jaw speed 200; acceleration 5. Ordinary motion targets remain inside saved ranges with 40-tick margins and at most 341 ticks per segment, ramped in at most 40-tick increments.
- Coherent motor status must remain zero; voltage raw 100–140; load ceilings 800 for positioning joints and 500 for jaw; all held joints must remain within saved ranges and within 96 ticks of held goals.
- Position settling: three fresh stationary samples, velocity below 3, movement flag zero, sample-to-sample change at most 3, endpoint error at most 57 ticks; jaw endpoint tolerance 30 after measuring normal 18–26 tick stationary errors.
- `CLOSE`: target saved minimum plus 40; at most 341 ticks closing travel; advance by 10 ticks, check stability for 0.3 seconds. Stop advancing if stationary closing error reaches 40 ticks. Each step has a 1.5-second stationary deadline. Report resistance as unverified, never as object identity or grasp success. Load and following guards still apply.
- Fresh phone data required before starting. If receipt age exceeds 10 seconds after a hold is established, pause target advancement, retain the last issued waypoint, and poll motor health for at most 20 seconds. Resume only on a new sequence with receipt age below 2 seconds. A motor fault or persistent outage ends the session. This pause logic passed mock tests; no pause occurred in the successful pickup (`camera_pauses: []`).
- Every interactive wait has a 120-second monitored hold deadline. Through the API a finished move renews it, and so does re-sending `robot_set_motor_enable` for the six joints while holding. Never leave a payload held while doing unrelated work. A timeout/fault releases torque, so a gravity-loaded arm can fold; that is not a commanded reset.
- Cleanup verifies all six releases, restores backed-up settings, and checks all six off again. Wheels were controlled only in separate empty/released sessions.

## What failed and what changed

1. Partial closes at targets around 1419/1373 did not secure the handle. Image overlap alone was not accepted as a grasp.
2. Exploratory camera registration/CAD fits remained unvalidated; they were not converted into motor targets.
3. The phone view initially cut off the paddle. Reframing, then switching phones, gave a closer side view. Both phones still showed intermittent upload gaps; changing the phone alone did not eliminate them.
4. Immediate stale-camera aborts and strict jaw endpoint checks caused all-arm torque release and passive folding. Local corrections after an empty miss should retain a healthy approach pose. Stale-feed holding now has a bounded monitored pause; persistent failure still releases.
5. A 20-tick jaw tolerance falsely rejected ordinary stationary errors up to 26 ticks. An initial contact detector also falsely stopped at 22 ticks. Empty-jaw close/open testing led to 30-tick ordinary settling and a separate 40-tick stationary resistance stop.
6. Shoulder P=16/I=0 left a 77-tick stationary error in a gravity-loaded pose. Temporary P=32 reduced measured errors to 11–17 ticks without raising the pilot torque ceiling. The original P was restored afterward.
7. A 90° thin-edge grip pushed the handle aside and did not hold it. Returning to the original wrist orientation, centring the handle with two +40-tick pan corrections, then extending elbow −114 made the final width grip possible.
8. A proposed 40 cm backward drive was cancelled after the operator questioned its relevance; only a 1.79 cm guarded reverse pulse completed. Do not present that as the requested 40 cm motion. The later forward repositioning reduced the actual reach gap.
9. After the success, an API pickup move faulted at the settle deadline: a joint rested 58 ticks short of its target (tolerance 57) with zero stable samples, so the owner released all motors and latched STOP. The owner now applies up to 3 bounded goal corrections to a joint at rest short of target. If the joint is still short, the move returns `completed: false`, `closure_outcome: settled_short`, `endpoint_reached: false` with motors holding and no STOP. Re-plan from `readbacks`. A joint that never comes to rest still faults and releases all motors. The owner no longer latches STOP: after a STOP or fault it returns to idle with motors released until an explicit `robot_set_motor_enable`, and no restart is needed. See [qwen-bridge/PADDLE-PROFILE.md](qwen-bridge/PADDLE-PROFILE.md).
10. Qwen called `robot_get_registered_tags` and failed on a missing registration file; tags are not part of this procedure ([TAG-TOOL-FAILURE-HANDOFF.md](TAG-TOOL-FAILURE-HANDOFF.md)).

## Evidence and reproducibility

- `success.json`: sequence, endpoints, calibration hash/right-arm entries, result, release checks, frame hashes, and limitations.
- `telemetry.json.gz`: all ordered coherent samples and exact cleanup result. It has no per-sample timestamps; it is not a calibrated time-series training episode.
- `guarded-paddle-pilot.py`: source used for the successful trial, default-off archive gate.
- `drive-pulse.py`: guarded forward/backward pulse prototype; the owner's `qwen-bridge/wheel_pulse_executor.py` is ported from it.
- `jaw-close-open-check.json.gz`, `camera-pause-checks.json`: measured jaw baseline and offline pause guard verification.
- `qwen-bridge/`: the API, sole owner and pickup profile as deployed; `qwen-bridge/paddle-procedure.json` is the procedure served to Qwen as `physical_pickup_procedure`; `tool-schemas.json` lists the API tools; `server-recovery.json` is the 2026-10-07 recovery record.
- `controller-provenance.json`: original/archived source hashes and private frame locations. Original camera frames remain on the robot Mac; their hashes are portable, but the raw people-containing scene frames are not published.

To reproduce, first inspect fresh state, confirm the calibration identity and hardware match, and re-establish the actual table/cart/paddle layout from fresh cameras. Do not blindly replay this sequence after power-off/manual arm movement or cart repositioning. Keep a closed-loop observation after every segment. Only mark success when the paddle follows the jaws, visibly clears the table, remains held, and can be placed/released without a passive drop.

## Qwen integration contract

Use the existing `GET /health`, `GET /tools`, `POST /call` service and existing sole hardware owner through `DirectJointClient`/`DirectJointExecutor`. Qwen/Cerebras stays on the chat Mac; local Gemma interprets images there. Do not give a chat-side process direct serial access and do not start a second owner.

Tool-call envelope remains `{"name":"tool_name","arguments":{},"request_id":"unique-id"}`. Discover actual schemas, current blockers and units before generating actions. Read `robot_get_state`, `robot_list_motors`, `robot_get_capabilities`, `robot_get_cameras` (request `right_wrist` explicitly for the wrist view), `robot_get_depth`, `robot_get_readiness`, `robot_get_execution` and `robot_get_handoff`. Keep `robot_stop` independent and accessible. It releases all motors and cancels any move in progress, which is never resumed; there is no STOP latch, and motors stay released until an explicit `robot_set_motor_enable`. Timestamp/image projection semantics must survive Gemma interpretation and Qwen planning.

The pilot's close/hold/settings are integrated as owner profile `paddle-success-v1` ([qwen-bridge/PADDLE-PROFILE.md](qwen-bridge/PADDLE-PROFILE.md)), started with `--right-arm-only --paddle-profile` by `./restart-robot-server.sh` from the repo root. That integration has not yet been physically re-validated. The generic `legacy-direct` owner keeps its own protections. Do not run the archived pilot beside the owner. `robot_get_handoff` serves the API procedure as `physical_pickup_procedure`, built from `qwen-bridge/paddle-procedure.json`. Current service exposes a motion-capable right-only owner; four mismatched left-arm entries remain read-only. See [server recovery and chat Mac connection](QWEN-SERVER.md).

Suggested Qwen state machine: `inspect → verify station/calibration → establish empty guarded hold → visually align handle inside both jaws → bounded close until measured resistance → verify small lift and stable hold → place → open → withdraw → release`. On an empty miss, locally correct a healthy held pose; do not reset the entire arm as a reflex. Resistance, simulation, a plausible image description, and tool-call success alone are not verified pickup.

Cartesian FK/reach tools remain read-only proposals until the missing model-zero/sign binding, camera-to-arm transform, and physical jaw/handle offsets are validated. They were not established by this one successful visual encoder trial. Base driving is available through `robot_move_base` as guarded pulses. It is the owner-side port of `drive-pulse.py` and has not yet run on hardware through the API (see [qwen-bridge/PADDLE-PROFILE.md](qwen-bridge/PADDLE-PROFILE.md)). The requested 40 cm cart travel is outstanding and was not attempted during server recovery.

Large telemetry is losslessly compressed. Decode with `gzip -dc telemetry.json.gz > telemetry.json` and `gzip -dc jaw-close-open-check.json.gz > jaw-close-open-check.json`. Do not infer time-series timestamps from sample indices.
