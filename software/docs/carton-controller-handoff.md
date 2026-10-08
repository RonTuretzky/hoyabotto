# Robot-Mac controller source handoff

This source exports the currently commissioned owner and diagnostic clients. It does not certify grasping or folding. All hardware stays released during source handoff.

## Entry points and deployment

Use the robot environment's Python from software/, with local configuration supplied separately:

```sh
python scripts/carton_robot/carton_session.py --help
python scripts/carton_robot/carton_command.py --help
python scripts/carton_robot/carton_visual_step.py --help
python scripts/carton_robot/carton_preflight.py
```

The last command performs read-only motor access; the others above are help only. Owner launch (`--arm right --recover-right-elbow`) enables the selected arm, so it must not be used while the gripper fault remains unresolved.

Deployment variables: CARTON_WORKSPACE_ROOT (state/artifacts), CARTON_LIVE_SOFTWARE (existing farm/profile implementation), CARTON_UTILITY_SOFTWARE (this controller), CARTON_PROFILE (default paper-tray-v0), CARTON_SESSION_DIR, CARTON_FRAMES_DIR, CARTON_CALIBRATION. Without overrides, workspace is the repository root and both software paths are this software directory. Use the existing environment's Python; no new robot environment is required. Local profile supplies both motor ports. No machine ports, identities, profile snapshots, calibration or secret environment files accompany this commit.

Camera expected identities must be independently registered: CARTON_HEAD_ID/CARTON_RIGHT_WRIST_ID, or camera.identities in the local session config. Never infer an expected identity from a newly arriving frame. Body/camera movement invalidates old image seeds/local reach models; re-register and remeasure before approach.

Session config must exist locally before owner startup:

```json
{"camera":{"source":"robot","start_max_age_s":10,"motion_max_age_s":15,"hold_max_age_s":30,"identities":{"head":"REGISTERED_ID","right_wrist":"REGISTERED_ID"}}}
```

These values are camera grace ceilings, not target registration. Limits must be positive, ordered and at most30seconds. Head/right-wrist immutable camera manifests remain independently freshness/identity validated during elbow recovery.

## Exact file protocol

Single motor owner, single command producer. Clients atomically replace command.json under CARTON_SESSION_DIR:

```json
{"id":123456789,"op":"move","delta_ticks":{"right_arm_wrist_flex":-48}}
{"id":123456790,"op":"hold"}
{"id":123456791,"op":"stop"}
```

IDs are unique increasing integers. One arm joint per move, raw encoder ticks, abs(delta)<=68 (about6physicaldegrees), within saved range with4tick margin. Wheels/head/other arm remain released. Move is refused if another move is pending or current starting camera frame is too old.

status.json contains arm/phase/ok/started/time, coherent raw rows, goals, accepted/completed IDs, lease_remaining. Match owner started identity and your completed ID; stale files are not acknowledgment. Owner move completion requires three samples within24ticks of the goal, deadline4seconds. Default SessionTransport precision remains5ticks; camera-reviewed endpoints permit11ticks, or24 only for right elbow with correct-direction displacement≥max(12,halfrequested), command-envelope containment and stable feedback. Other-joint drift remains≤5ticks. Measured endpoint acceptance is not object-success evidence.

Lease starts at180seconds. Accepted move or processed hold resets it to180; there is no automatic/background renewal. This exported owner adds last_hold_accepted ID; command client waits for that exact hold acknowledgment. A hold does not extend an existing move deadline. Expiry stops/releases the selected arm. STOP requires released=true and empty release_errors; the client waits for verified release. Neither a written STOP nor phase alone proves physical release.

Explicit visual checkpoint creates another caller-reviewed local series from stable measured pose without releasing or renewing a lease. It has no automated motion loop, permits at most3new phases, and has not been exercised on hardware. Each local series has5steps/240ticks total and128tick per-joint origin envelope. Gripper contact diagnostics are separate bounded moves; do not concatenate these as a folding trajectory.

## Gripper semantics

Partial jaw motion requires12–68ticks requested, actual movement≥12ticks, three stable samples and matching completion, abs(load)<=250, other-joint drift≤5, saved-range and command-envelope checks. It reports PARTIAL_CLOSURE_ONLY/PARTIAL_OPEN_ONLY, never a verified grasp. Up to24tick endpoint shortfall with load≥50 is possible_contact only. There is no empty-gripper reference proving contact force; friction can look identical. Verification requires the object following a measured lift in independent views.

Selected-arm load limit500; gripper diagnostic cap250. The packaged owner exposes `automatic_gripper_reenable:false` and increments `gripper_release_generation` for claw releases. A program must invalidate grip evidence whenever that generation changes.

## Continuous trajectory protocol

Canonical protocol is now the published `carton.servo.continuous` interface at upstream commit6d3b543. The packaged motor owner integrates `ContinuousOwnerBinding`; it does not run a second command writer. Earlier experimental `trajectory_binding`/`TrajectoryTransport` modules remain offline regression artifacts and their commission-file command format is not accepted by this owner.

Load `continuous.profile_file` and `continuous.config_file` independently at startup before motor connection. A missing commissioned profile leaves only existing small-probe control available. The config binds actual calibration bytes, selected arm and registered camera IDs; no physical corridor, rate or poses are invented. The local binding additionally caps velocity100ticks/s, acceleration200ticks/s², sample increment68ticks and whole trajectory plus endpoint settling30seconds, with actual saved motor range margins enforced by the hardware callback.

Canonical trajectory command contains `session_started`, owner-published `profile_sha256`, both registered `camera_streams`, and raw six-joint `waypoints`. Client updates `trajectory-vision.json` during execution. Owner binds these to current immutable manifests, independently configured identities, profile/config/calibration bytes, real per-motor oldest telemetry timestamps, health, STOP and lease. The continuous executor samples without intermediate endpoint waits and checks final stable encoder settling. See [carton-continuous-protocol.md](carton-continuous-protocol.md) for exact schema.

Normal terminal STOP records matching completed command ID only after verified release, empty errors and fresh status. Owner retains no silent claw reenable and release generation. Runtime metrics record command count, per-tick updates, loop rate, check latency and separately verified release latency. Synthetic tests do not certify real hardware throughput or grasp success. No commissioned live trajectory has run.

## Physical evidence and remaining prerequisites

Verified: all16communicate and release; autonomous rightelbow recovery within12degree cap; camera-only two-view capture; bounded approach joint motions; partial closures. NOT verified: metric task geometry, usable local Jacobian, paddle grip, paddle lift, any flap fold. Head view showed jaws below/left of handle while wrist projection overlapped it; paddle stayed on table in lift check.

Latest powered claw reported150/94/137°C with fault0 while holding, followed by41°C within~0.11s of release. After two confirmations, repeated reading stop released all6selected motors. Subsequent independent read-only all16check confirmed release. Exact sensor/electrical/firmware cause unproven. Forty paired off-state reads at40–41°C do not establish powered health.

Depth camera existing utility is on origin/main: farm/oak_camera.py, docs/oak-d-lite.md, requirements-oak.txt. OAK-D Lite/DepthAI2.33, synchronized CAM_A640x360RGB and aligned uint16 axial millimetre depth; zero means missing. Camera-to-robot transform is uncalibrated. Depth camera-frame points alone cannot command the robot.

## Validation

131focused owner/telemetry/receive validation/recovery/controller tests passed before the source-only hold-ack fix; its command and owner regression tests passed separately. CLI import/help smoke tests access no hardware. Software tests are not physical grasp or folding evidence.

New local integration validation:111focused trajectory/client/binding/program/depth and owner regression tests passed. The continuous454tick test uses simulated telemetry only.
