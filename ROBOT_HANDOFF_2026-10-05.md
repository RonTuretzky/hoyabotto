# Robot / carton pickup handoff

Prepared 2026-10-05, approximately 16:01 JST (Asia/Tokyo). The user has removed this agent from the task and explicitly requested this handoff and its upload. This document records established results and remaining gaps; it is not a claim that the carton task succeeded.

## Outcome and shutdown

**The paddle has not been securely grasped or lifted. The carton has not been folded.** A physical reach using the geometric planner has not been commissioned or executed.

At handoff, the recurring `robot-progress-to-github` automation was PAUSED. All three helpers were interrupted; they had already completed their latest work. Task-owned head/OAK capture processes, both depth viewer servers and the phone-camera server were terminated. A process check confirmed no matching capture, viewer, phone server or carton motor-owner process remained. No new motor commands or calibration writes were issued during handoff.

Last verified actuator state: the isolated left-claw test verified torque-off and release of all left-board motors. The other board was not commanded during that test and had been released at the preceding all-sixteen check. This is the last verified state, not a fresh physical battery-power check. Software release does not disconnect the motor battery.

## User intent and operational history

The requested end task is to pick up a white printed paddle and fold the carton using the carton handoff/branch. The user requested autonomy, faster parallel helpers, larger justified movements and fewer repeated approvals. Standing authorization for ordinary scoped robot motion was given repeatedly. The user also requested GitHub calibration export, code/progress publication and a twenty-minute progress report.

The initial pickup work used the robot's RIGHT arm (user's LEFT when facing it). Repeated powered right-claw temperature anomalies interrupted that work. Later work selected the robot's LEFT arm (user's RIGHT when facing it), which the user placed near the paddle. The arm switch was not communicated clearly enough at the time. The active candidate at handoff is LEFT; historical right-arm targets and image response measurements must not be reused for it.

The user repeatedly requested ignoring all temperature readings, removing sensing, adding a disabling flag/environment variable, and raising the cutoff to 100 C. **None of those requested changes was implemented.** Existing temperature protections remain. Isolated sudden readings may be faulty telemetry, but actual motor temperature and the cause of repeated powered anomalies were not independently established. A later verbal description of a physically damaged/melting sensor was not independently verified. Do not describe it as a confirmed hardware inspection result.

## Verified physical work

### Calibration and basic motion

Both arms were calibrated through the integrated automatic arm tool after earlier communication faults and invalid range measurements. Lower speed initially caused timeout/false end-stop issues; a faster setting subsequently completed plausible arm ranges. A camera contacting the cart caused an obstructed left-arm run; those results were not accepted. After clearance/power reconnection, the left run passed and its shorter-travel warnings cleared. Head calibration was recorded manually with the motors released, a forward/level reference and gentle usable yaw/pitch sweeps; new limits were saved and checked against hardware.

Earlier basic movement-and-return checks eventually passed for all sixteen motors across separate tests, including both drive wheels. This does not mean every joint passed in one uninterrupted full-robot run. Right elbow sag after release was a recurring issue. Bounded isolated recovery moved it back inside its saved range; a documented stronger position response enabled its follow-up check. A similar positioning-response adjustment resolved a right-claw short-movement test earlier. Working settings were saved and retested at that time.

### Driving and turning

Earlier short forward / arm lift-lower / backward sequences completed twice. A larger single demo also completed with approximately eight centimetres each way and a higher arm sweep. A requested thirty-minute loop did not start; automatic approval review rejected it under the then-current visibility and temperature evidence.

A later approximately fifty-centimetre forward/back test completed with two diagnostic pauses. One stop check was corrected to recognize stable positions despite one-count speed noise. A left wheel failed to move in one return segment; it passed an isolated test, corrected the small turn and finished the return. The exact cause was not confirmed. Per-wheel command receipt verification was added. Wheel-control mode changes were found to change reported angle without physical motion; comparisons were corrected. These results are largely encoder-based distance estimates, not externally calibrated floor metrology.

A staged spin ran, but turning geometry and visual heading disagreed. Saved intermediate images were consistent with one nearly completed revolution; a precise complete 360-degree spin was not confirmed. A continuous-turn controller was prepared but its preparation is not proof that it ran. No stock automatic dance/calibration-certification routine was established. Matcha exploration was abandoned when the user returned to carton work.

### Paddle attempts

Historical right-arm probes and shoulder/elbow recovery brought an open claw near the handle. Wrist and head views initially disagreed about alignment. Partial closure met resistance, but a lift check showed the paddle stayed on the table. Thus no grip was established. Subsequent right-claw tests encountered abrupt high readings followed by normal stationary values. The thermal release/confirmation path had bounded retry budgets; do not silently reset exhausted recovery budgets to keep retrying.

Latest isolated LEFT-claw test: requested raw position 1342 to 1410 (+68 ticks, about six degrees). Actual final position was 1393 (+51 ticks, about 4.5 degrees). The endpoint check failed because it settled 17 ticks short. Temperature remained 37 C; voltage and status were normal. The test released torque and verified all left-board motors off. This establishes real claw movement, not approach, grip or lift.

Prior to that test, read-only preflight found all sixteen responding and released, with saved calibration matching hardware. Left-arm readings were pan 1936, lift 3222, elbow 967, wrist flex 2089, roll 2125, grip 1342. Lift and elbow were close to their respective saved endpoints. These are historical readings, not future targets; refresh before any new action. The final camera check showed a human hand beside the left claw/paddle, so further movement was not started from that image.

## Cameras

The user explicitly selected **two cameras concurrently: head plus OAK depth**. Right-wrist capture was disabled. The left-wrist capture fault was not resolved, but is not a prerequisite for the selected mode. Camera streams are now stopped as part of handoff; restarting them is successor work.

Registered identities:

- Head: `0x12400005a39230`.
- Right wrist: `0x12200005a39230`.
- Left wrist, previously called auxiliary: `0x12140005a39230`.
- OAK device: `1944301091DA1C2E00`; logical camera ID `oak-1944301091DA1C2E00`.

Head last operated at 640x480, about five frames per second. OAK used DepthAI 2.33.0.0, USB2 mode, 640x360 RGB with aligned CAM_A axial depth in millimetres, zero invalid depth, fixed calibrated focus 79 and 15 fps. OAK intrinsics were recorded in each manifest. **No OAK-to-arm extrinsic was registered.** RGB/depth bytes and source timestamps are validated using immutable manifests and hashes; reception time must not be substituted for capture time.

Left-wrist diagnosis: it briefly delivered frames, then stalled. With right-wrist capture off, head continued while left delivered zero callbacks for roughly seventy seconds; timestamp rejection was zero. Advertised lower-resolution capture attempts encountered intermittent device disappearance and a macOS camera-device error. Shared USB bandwidth is only a hypothesis. Replugging that camera directly into the Mac was suggested, but the user elected two-camera operation instead. Avoid repeated unchanged capture retries or requiring one camera to show every feature when two views can be combined.

The local OAK preview used `http://127.0.0.1:61687/`; it is stopped now. Its old phone preview and local authorization tokens must not be published or reused. Old pixel targets/plane fits are stale after manual arm or camera repositioning. Some white handle ROIs had depth holes/mixed foreground-background distances and occlusion; those estimates were not accepted as registered target geometry.

## Local software and verification

### Canonical controller

The isolated utility checkout is on `codex/carton-controller-handoff`, latest committed local change `e8d369c` (guarded carton trajectories, depth perception and measured arm units). Its canonical path comprises:

- `software/carton/servo/continuous.py`: guarded continuous trajectory execution.
- `continuous_owner_binding.py`: profile/config/calibration/identity binding.
- `program.py`, `depth.py`, `vision.py`, `trajectory.py`: observations, depth and paths.
- `software/scripts/carton_robot/carton_session.py`: packaged sole motor owner.
- `software/farm/oak_camera.py`: capture and aligned metric depth manifests.

The packaged owner primes present goals before enabling torque; verifies writes; limits selected-arm speeds/torque; maintains STOP, deadlines, telemetry, corridor and camera freshness checks; and releases on failure. Its powered high-gripper-temperature branch releases without automatic reenabling. Commissioned trajectory support does not imply a physical profile exists.

At handoff, newer explicit head/OAK support is **uncommitted locally** in eight existing files: CLI, common/config, continuous, continuous binding, depth, program, vision, and packaged owner. New files `software/docs/carton-head-oak.md` and `software/tests/test_carton_head_oak.py` are also uncommitted. The helper reported **126 focused tests passed**, live-Python compilation passed and whitespace checks clean. Tests cover left-arm preparation, real configured owner-reader paths, synthetic two-camera pickup, STOP, depth loss, moved camera registration and dropped-object rejection. No hardware writes were issued by the helper.

Head/OAK mode is explicit in both station config and continuous profile. It keeps independent camera identities, validates aligned depth, and replaces wrist-relative pixel retention with an OAK-optical-frame metric paddle-minus-tool displacement. It creates no fictional wrist camera or extrinsic. Standalone bounded raw probes are supported using `camera_pair: head_oak`, robot source and explicit manifests/identities, without requiring a fictional continuous commissioning profile.

**Deployment gap:** the older live work-directory owner remains different from the updated packaged owner. The final turn inspected this difference but was interrupted before actual-runner integration. Do not assume the old runner was upgraded or launch both owners. Confirm import paths, session configuration and sole serial ownership before deployment.

Unused experimental files `owner_trajectory.py`, `trajectory_binding.py`, `trajectory_transport.py` and their tests remain untracked. They were excluded from the prepared publication payload and are not the canonical integration. Avoid creating competing execution paths.

### Reach planner and units

The scale-conversion gap was addressed locally using `software/farm/kinematics/analytical_reference.py`, `units.py` and arm-model integration. It converts explicit model degrees to raw encoder ticks to driver-normalized values and reverses that mapping for feedback. It requires a measured reference tick, actual model angle and direction sign for all five positioning joints, bound to exact calibration and model hashes. Templates intentionally contain null fields and fail validation until measured.

**The physical reference is still missing.** Saved motor limits/midpoints/homing values do not by themselves identify the model's geometrical zero. Analytical-model and URDF conventions are not interchangeable. A convenient rest pose or a camera pixel is not a zero reference. Physical Cartesian execution retains commissioning gates.

An isolated pinned solver environment was installed without replacing the live robot environment. Three actual pinned URDF FK/IK/FK numerical pose checks passed; maximum position error was about 0.276 mm and orientation error below 0.008 degrees. All 22 upstream geometry/perception tests passed, without skips. Earlier reference tests passed 13 cases and the combined pre-head/OAK suite passed 109. These are software validations, not reach, collision or grasp evidence. The model emitted neutral self-collision warnings at adjacent links; neutral is not certified clear.

Known paddle STL dimensions: overall 210x40x6 mm, blade about 150 mm and handle 60 mm. Earlier depth plane fits were not registered into the arm frame and became stale when the setup moved. Do not use them as the target transform.

## Exact remaining prerequisites

1. With motor power off or verified release and forearm supported, establish an independently measurable arm pose. Record simultaneous encoder ticks and geometry for all five positioning joints; measure direction signs with isolated bounded probes. A partial analytical example is upper-arm axis vertically up and forearm axis horizontally forward, yielding shoulder lift approximately -13.96796 degrees and elbow flex 16.17545 degrees in that analytical convention. This supplies neither raw ticks nor swivel/wrist/roll references and is not a motor command.
2. Independently establish swivel forward direction, tool pitch/twist convention, arm-base/station relationship and camera-to-arm registration. A calibrated rigid marker target with known pose relative to the arm base is one concrete route. Existing motor calibration alone cannot determine these transforms.
3. Restart the selected cameras and refresh views; verify hands/objects clear of the intended motion. Record visible tool, paddle, jaw aperture open/empty/contact baselines and independent fixed anchors. Head/OAK grasp observation also needs usable depth table/tool/paddle/bottom features and a measured optical-frame up direction.
4. Measure or validate collision-safe joint corridors, approach/park/lift waypoints and rate limits for this left arm and current table. No complete continuous commissioning profile or physical Cartesian pickup path exists yet.
5. Deploy the checked packaged sole owner with correct identities/configuration. Validate noncontact approach first, then partial closure/contact and a small observed lift. Confirm that the paddle actually rises and stays in the claw before any carton action.
6. Only after a real grip/lift, commission the carton-fold sequence against the fixture and flap geometry. No learned box-folding model, completed recipe or measured physical fold poses were installed by this session.

## Workspace and evidence for the successor

Workspace root: `/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20`.

- Live repo/Python: `xlerobot-farm/software`, `.venv/bin/python` beneath it.
- Utility checkout: `work/carton-visual-controller`; source under its `software/`.
- Isolated solver environment: `work/geometry-check`; verification utility `work/check_geometry_solver.py`.
- Legacy owner: `work/carton_session.py`; shared session state/config: `work/carton-session/`.
- Latest bounded left-claw utility: `work/left_claw_prepare.py`; actual result `work/left-claw-prepare-result.json`.
- Read-only preflight utility: `work/carton_preflight.py`; latest pre-test snapshot `work/left-ready-preflight.json`.
- Coherent telemetry and spike helper: `work/coherent_servo_telemetry.py`, `work/temperature_confirmation.py`.
- Camera manifest folder: `work/robot-camera-stream/`; capture binary `work/capture-single`; OAK environment `work/.venv-oak`; viewer `work/depth-viewer/server.py`; phone server `work/phone_camera/server.py`.
- Camera/model reports: `outputs/Left-Wrist-Camera-Diagnosis.md`, `outputs/Paddle-Depth-Check.md`, `outputs/Reach-Solver-Verification.json`, `outputs/Arm-Reference-Guide.md`, and both blank arm-reference templates.
- Old ROI/plane review: `work/oak-depth-review/` (historical, not live geometry).
- Publication inventory: `outputs/Code-Publication-Manifest.md`.

The latest isolated opening used the left board; it retained strict SDK replies, hashed fresh head/OAK images, raw speed/acceleration limits, a low torque cap, travel/deadline and health checks, then restored prior speed/acceleration/torque/lock settings. It never commanded shoulder, elbow, head or wheels. Inspect this utility before reusing it rather than claiming it is a full controller.

## GitHub state and publication limits

Repository: `RonTuretzky/xlerobot-farm`.

Saved calibration was exported with user approval to branch `codex/robot-calibration-snapshot`, path `software/calibration/farm_xlerobot/farm_xlerobot.json`, with a README. The authenticated connector was used; shell Git push authentication was unavailable. The local export is `work/calibration-export/software/calibration/farm_xlerobot/farm_xlerobot.json`. Actual live calibration was in the LeRobot user cache, not a new assumed live-repo calibration path.

Progress reports were published only to `ROBOT_BRINGUP_STATUS.md` on that branch. Latest completed report commit before this handoff: `d2f34ca51c50c0b24926a991f9b6926aabcef29e`, about 15:56 JST. It records actual left-claw movement and software tests, with no grip claim. The older local `outputs/Robot-Progress-Update.md` has stale elbow/camera statements; prefer this handoff and raw results over that older summary. The twenty-minute reporting automation is now PAUSED as explicitly requested by ending this agent's work.

A 45-file controller source publication was prepared for a proposed new branch `codex/carton-reference-owner-integration`. Automatic approval review rejected that broad public-source payload; exact-payload approval remained unresolved. **No source branch, tree or PR was created from that payload.** Local head/OAK additions also postdate it. This handoff upload is a separate explicit user request; it does not assert approval to publish the source tree.

Remote references inspected included utility baseline `0e17b22`, local-program update `1f333093` and diagnosis branch `ernest/arm-diagnosis` at `6bd41a4`. The diagnosis branch supplied guidance, not the missing physical reference. Do not treat the source changes as merged into remote main.

## Stop here

This agent's work ends after uploading and verifying this document. Background reports and task-owned capture/viewer processes have been stopped. No further robot actions, diagnosis, source edits or recurring updates are authorized for this agent by this handoff request.
