# Paired trajectory owner library, protocol v2

`carton.servo.bimanual_owner.BimanualTrajectoryOwner` is an offline-tested
library for later integration **inside the existing sole motor owner**. It
opens no serial port, camera, endpoint, file, thread or service. It never
enables motors, changes firmware/register limits, clears STOP, or recovers a
pose. No hardware launcher currently invokes it. The existing single-arm
`carton.servo.continuous` protocol v1 is unchanged.

This library is not a deployed or commissioned bimanual robot controller.
Tests use synthetic state and fake write/release callbacks. No physical
trajectory, collision clearance, calibration, force, camera accuracy or STOP
latency has been established by these tests.

## Local profile and binding authority

The sole owner must independently load and verify a measured profile before
enabling both arms. Construct the library with that profile and its separately
verified current bindings; never copy either from an incoming command.
Construction performs local validation only and calls no supplied callback.

Profile fields:

| Field | Contract |
| --- | --- |
| `protocol`, `schema`, `units` | `carton_bimanual_trajectory`, `2`, `encoder_ticks` |
| `joints` | Exactly all twelve canonical left/right arm joints, including both jaws; no head/wheels |
| `bindings` | Exact match to the independent owner binding described below |
| `commissioning_evidence` | `status: COMMISSIONED`, nonempty `reference`, and `sha256` matching the binding's commissioning digest |
| `corridor` | Twelve measured clear `[low, high]` intervals inside actual saved ranges with at least four ticks margin |
| `velocity`, `acceleration` | Twelve measured limits, positive and no greater than 100 ticks/s and 200 ticks/s² |
| `load_raw` | Twelve positive raw-load ceilings; at most 500 for positioning motors and 250 for jaws |
| `following_ticks`, `start_ticks`, `settle_ticks` | Respectively 1–24, 1–5 and 1–5 |
| `max_sample_ticks` | Positive and no greater than 68 |
| `vision_age_s`, `max_camera_skew_s` | At most 1 s and 0.3 s; use the stricter commissioned task/depth constraints |
| `max_tick_gap_s`, `telemetry_age_s` | At most 0.2 s, with telemetry age no greater than tick gap |
| `max_telemetry_skew_s` | At most 0.1 s across all twelve actual per-motor capture timestamps |
| `max_dispatch_skew_s` | At most 0.05 s and no greater than tick gap; bounds the complete first-write-start to second-write-finish span |
| `settle_timeout_s` | 0.1–5 s; path plus settling must remain within 30 s |

These are ceilings, not suggested physical measurements. Do not fill a profile
with these maxima just to make it accepted. The evidence record must actually
commission the loaded mechanism, both-arm swept volume, rates/following,
dual-bus capture/write timing and STOP behavior. `COMMISSIONED` and a hash are
binding declarations; this library cannot prove the underlying record is true.

Independent `bindings` contains:

- `calibration_sha256`, `registration_sha256`, `kinematics_sha256`: each an
  exact `{left: <sha256>, right: <sha256>}` map. Both camera-to-arm registrations
  and measured zero/sign/TCP mappings must be independently validated.
- `station_sha256` and `commissioning_sha256`: SHA256 identities of the measured
  station and commissioning evidence.
- `ranges`: actual saved `[low, high]` raw encoder intervals for all twelve
  joints, independently matched against hardware. File hash agreement alone
  does not prove the present homing/limit registers match.
- `camera_ids` and `camera_streams`: matching maps for two to four independently
  registered cameras. IDs must be distinct. Stream restart invalidates this
  binding, consistent with registration provenance.

Use lowercase 64-character hexadecimal SHA256 strings. The library deep-copies
profile/bindings and checks their identities during operation. The containing
owner must reread/revalidate current file/register/registration identities and
provide the resulting current binding in each context. It must not blindly
echo the constructor's cached dictionary after configuration changes.

## Owner callbacks

```python
owner = BimanualTrajectoryOwner(
    local_profile, independent_bindings,
    write_arm=write_arm_raw_ticks,
    stop_arm=stop_and_verify_arm_release,
    guard=existing_owner_guard,
)
```

All methods and serial callbacks run in the **same sole-owner thread**. The UI
STOP path sets the owner's independent STOP event; the guard and `stop_requested`
consume it. Do not call serial release/write callbacks concurrently from a
second thread or create one owner per arm. Other clients must be excluded by
the actual owner's exclusive lease/serial ownership policy.

`guard()` must raise on owner STOP, lost ownership, communication faults,
inactive head/wheel drift or enable state, and any stricter existing safety
condition. It remains authoritative. The containing owner retains all actual
torque, speed, hardware-range, strict packet validation and safety policy.
No thermal-policy change is introduced by this library.

`write_arm(arm, goals)` synchronously writes exactly that arm's six integer raw
goals through the existing verified writer. It returns:

```python
{
    "arm": arm,
    "written": goals,             # exact six names and dispatched values
    "started_at": actual_start,   # same monotonic clock as the coordinator
    "finished_at": actual_finish,
}
```

Record real dispatch timestamps around the actual serial operations. They must
fall inside the callback's independently measured invocation/return interval.
An acknowledgment proves dispatched setpoints only, never measured motor
completion. A partial write must raise even if some goals reached hardware.
The coordinator immediately latches STOP for both arms on partial writes,
missing/mismatched acknowledgments or excessive dispatch skew. It does not
retry or roll back motor commands. Serial writes cannot be physically atomic;
both-side prevalidation and whole-operation STOP handle this unavoidable gap.
The existing owner/transport must enforce bounded I/O timeouts and an independent
watchdog. A synchronous library cannot interrupt a callback that never returns;
measured watchdog/STOP behavior and the hardware owner's independent fail-safe
remain prerequisites.

`stop_arm(arm)` must attempt release and obtain fresh uncached torque readback
for all six motors. Return `arm`, `released: true`, `release_errors: []`,
`cached: false`, `captured_at` using the wall capture clock, and `torque_enable`
mapping exactly those six names to zero. Capture must occur after that release
callback was requested. Both callbacks are attempted even if the first raises.
Both proofs must still be fresh at paired confirmation. Missing/old/nonzero
proof leaves `released:false` and records errors; requesting STOP alone cannot
be reported as release. The latch has no reset method.

## Command and owner-observed context

A command contains `protocol`, `schema`, `op: bimanual_trajectory`, a strictly
increasing integer `id`, `session_started`, owner-published `profile_sha256`
and `bindings_sha256`, `scene_revision`, `scene_sha256`, the two current
`gripper_release_generation` counters, and `waypoints`.

Every waypoint contains increasing `time_s` (the first is zero) and all twelve
raw `positions`. There are 2–100 waypoints. Existing `JointTrajectory` checks
the whole corridor and analytically retimes interpolation to the measured
velocity/acceleration limits. One time parameter drives both arms; there is no
independent left/right endpoint wait or model inference in the owner loop.

The separately supplied `context` must come from trusted owner-side readers
and the observation-derived scene validator, never directly from the command:

```python
{
    "owner_started": existing_owner_session,
    "bindings": independently_verified_current_bindings,
    "gripper_release_generation": {"left": left_generation, "right": right_generation},
    "frames": {
        camera_name: {"camera_id": registered_id, "stream_id": registered_stream,
                      "seq": capture_sequence, "captured_at": real_capture_wall_time,
                      "sha256": immutable_image_hash},
        # Include every independently registered camera.
    },
    "scene": {
        "revision": planning_revision,
        "sha256": planning_scene_hash,
        "trajectory_sha256": independently_collision_checked_proposal_digest,
        "valid": True,
        "collision_checked": True,
        "observation_only": True,
        "frame_sequences": current_validated_camera_sequences,
    },
}
```

The scene hash/revision identify the paired plan's checked observation-derived
scene and allowed motion/contact envelope. At each tick the local scene
validator must affirm, from the referenced fresh frames, that the current
geometry remains within that checked envelope. Merely seeing camera traffic
cannot establish `valid:true`. Changed carton/flap pose outside the envelope,
unknown required surfaces, lost registration or unmodeled obstacles must
invalidate it. Simulator truth snapshots cannot be marked `observation_only`.
The owner library verifies provenance/timing/identity; it does not generate
collision geometry or certify that an upstream scene validator is correct.

`trajectory_sha256` must be supplied by the independent owner-side collision
validator after checking the exact proposal. Use `trajectory_digest(command)`
to identify the canonical `profile_sha256`, `bindings_sha256`, `scene_revision`,
`scene_sha256` and complete `waypoints` object that was checked. Profile identity
also binds the rates used for interpolation/retiming. Changed timing or even a
one-tick in-range endpoint invalidates that certificate before either write.
The receiving owner must compare with the saved validated certificate; simply
hashing each incoming command and echoing that hash as a certificate is not
collision validation and violates this contract.

Changing scene hash/revision while a trajectory runs stops it. A new command
requires a strictly newer scene revision and a new frame from every registered
camera. Replaying a previous scene or advancing only its revision refuses.
Same-sequence changed image bytes/capture time, sequence rollback, future or
stale timestamps, and changed streams also refuse. Scene validation must name
the current frame sequences exactly. Never timestamp an old observation with
the processing time.

`rows` contains exactly twelve coherent raw telemetry records with integer
`Present_Position`, integer zero `Status`, integer one `Torque_Enable`, and
finite `Present_Load`. `telemetry_at` is a twelve-name map of **actual per-motor
capture times**, not one refreshed status timestamp. All joints are checked
before either write. The oldest age and overall capture skew are enforced.
Reusing a timestamp for changed telemetry or rolling it back refuses.

## Owner loop and completion

```python
accepted = owner.start(
    command, coherent_rows, per_motor_capture_times, trusted_context,
    session_started=current_owner_identity, lease_remaining=actual_lease_remaining,
)
# start validates but does not dispatch a goal or enable anything.

while owner.active:
    # Existing owner acquires coherent rows and fresh local context each cycle.
    update = owner.tick(
        coherent_rows, per_motor_capture_times, trusted_context,
        lease_remaining=actual_lease_remaining,
        stop_requested=independent_stop_event,
    )
    # Persist inputs, update, owner.metrics and any owner.stop_result.
```

Start assumes the existing owner already owns and holds exactly the two arms;
constructor preflight must precede enabling them. Any start or tick refusal
latches both-arm STOP. Do not use start as an exploratory client probe against
some other client's active owner. External safety checks remain necessary even
when no trajectory is active.

Each tick validates both telemetry sets, vision/scene/bindings, lease, following
error and **all twelve rounded targets** before the first arm write. It rechecks
the guard and freshness/lease before the second arm and after dispatch. The
shared monotonic lease deadline may shorten but cannot be extended by later
tick arguments. The frame/telemetry clocks use wall capture time; dispatch and
trajectory timing use monotonic time. Keep both clock bases correct.

Completion requires both arms at the final endpoint for three distinct
all-twelve telemetry samples. Repeated readbacks cannot supply extra settling
votes. Return `completed:<exact command ID>`, `phase:holding` only then. This
means measured encoder completion, not flap closure or physical success. Keep
normal independent owner supervision while holding; no automatic jaw release
or re-enable occurs between stages. Explicit terminal STOP has separate
release proofs and invalidates prior grip evidence through the outer owner's
release-generation counters.

`metrics` records per-joint capture times, oldest telemetry age and maximum
skew, last scene/frame identities, both real dispatch intervals, dispatch
onset skew and full span, write attempts/paired successes, tick gaps and
trajectory duration/travel. `last_dispatch` includes partial/failure evidence.
Persist each update to retain a complete trace; the in-memory metrics keep
the latest event and maxima. `stop_result` keeps both release proofs/errors
and measured callback latency.

## Offline verification and remaining integration

```sh
.venv/bin/python -m pytest tests/test_carton_bimanual_owner.py tests/test_carton_continuous.py tests/test_carton_continuous_owner_binding.py tests/test_carton_trajectory.py -q
```

The paired fake-owner tests cover invalid right-arm plans before any write,
asymmetric health faults, missing/stale/skewed telemetry, camera/scene replay,
changed registration, expired/shortened lease, independent STOP, partial writes,
bad acknowledgments/timestamps, dispatch delay, jaw release-generation changes,
stable paired completion and failed/stale release proofs.

Before any deployment, integrate and validate the callbacks in the actual sole
owner without a second command writer. Independently commission both arms,
real camera/registration/station geometry, paired collision envelopes, physical
forces, loaded rates, dual-bus timing and STOP/vision-loss behavior. The motor
calibration/register mismatch and missing physical registration recorded in
STATUS remain separate blockers. The library's profile acceptance or test
suite does not resolve them, and no simulation result authorizes that work.
