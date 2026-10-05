# Local carton execution and OAK depth observations

The program runs locally with **zero LLM calls** during motion. It retains one
motor owner and one command-writer lock across approach, jaw closure, lift,
optional folds, placement and supported parking. The default CLI template is
now a **continuous pickup cycle**; four flap trajectories are not required to
test picking up the paddle.

The complete client/owner protocol and pickup sequence pass hardware-free tests.
**The robot Mac still needs the owner integration, measured station recipe,
working camera streams and healthy actuators before this can run physically.**
No real grasp, lift or fold has been verified by these tests. We do not ship
invented physical poses, speeds, collision corridors or image targets.

## Prepare and run

From `software/`, using the robot environment's Python:

```sh
python -m carton.servo program-template --config /path/to/experiment.json --out /path/to/recipe.json
python -m carton.servo program-check --recipe /path/to/recipe.json
python -m carton.servo program-run --recipe /path/to/recipe.json --out /path/to/new-run
python -m carton.servo program-run --recipe /path/to/recipe.json --out /path/to/new-run --execute
```

Default template options are `--task pickup --motion continuous`. Use `--task
fold` for four flap paths. `--motion alignment` explicitly selects the earlier
local-Jacobian commissioning controller and its small-step envelope. The
continuous client **never falls back to small-step commands**.

Static checks collect missing measurements before accessing the owner. Passing
static checks is not live readiness. Actual start encoders, scene, owner profile,
camera identity/freshness, lease and motor health are checked again at dispatch.
Recipes bind to camera seeds and saved calibration. Moving the body, station or
camera requires new measurements.

## Continuous movement

A recipe's `motion` contains `mode: continuous`, a commissioned `profile`, and
`segments`. The owner loads and validates its own copy of that profile at startup;
commands only reference its digest. See [the complete owner integration
contract](carton-continuous-protocol.md). The profile supplies all six selected
arm joints, a measured clear joint corridor, velocity/acceleration limits and
watchdog tolerances. Merely taking the full servo range is not collision
commissioning.

Each segment contains timed absolute encoder `waypoints`, `start_features`,
`end_features`, and `feature_bounds` (one [minimum, maximum] per configured image
measurement). Every waypoint names all six joints. Measured image endpoints must
match that stage's targets. Profile retiming obeys velocity and acceleration
limits throughout the interpolation, not just at waypoints. Consecutive segments
must meet; jaw-only stages hold the positioning joints.

Pickup uses these six segments in order: `approach`, `close_gripper`, `lift`,
`place`, `open_gripper`, `park`. Folding adds `fold_<name>` and `retract_<name>`
for `short_left`, `short_right`, `far_long`, `near_long`. A segment can exceed
68 encoder ticks: the protocol test includes a single 456-tick (about 40-degree)
approach. Small internal setpoint samples are one continuous motion, without
waypoint acknowledgement or settling pauses. Settling occurs only at the end of
an entire segment.

Both cameras continuously check tracked visual bounds, stream identity and
freshness while the owner updates goals independently. Wrist-relative paddle
retention is checked throughout the lift and every later movement until deliberate
placement. A separate timestamped vision file lets the owner stop if the client
blocks or disappears. Full evidence images are encoded at stage boundaries;
per-frame measurements remain in `trace.jsonl` without image-write latency in
the active movement loop. `progress.json` reports the current stage.

## Grasp, placement and parking

Jaw closure uses measured open, empty-closed and desired grip encoder positions.
A stall or load spike is not grasp success. A lift requires upward tool/paddle
co-motion in the head view, paddle-bottom table clearance, stable wrist-relative
geometry and nonempty jaw aperture on three independent frame pairs. Optional
OAK depth adds metric lift, slip and clearance checks.

Pickup requires an explicit `finish` with `mode: place_and_park`: measured
`place_targets`, `park_targets`, all six `park_positions`, and a
`supported_park_evidence` reference establishing a mechanically supported,
unloaded rest pose. `placement_seconds` is 0.5–5 seconds,
`max_placement_drift_px` and `max_table_gap_px` are at most 3 pixels.
The program verifies stable table placement before opening, after opening,
and after retreating. It verifies park encoders, requests the owner's STOP,
and requires an explicit matching all-motor release acknowledgement.
**Do not substitute an unsupported raised-arm pose for the measured park.**

A successful pickup returns `PADDLE_PICKUP_CYCLE_PASSED` with distinct grasp,
replacement and release evidence. The legacy `physical_task_completed` field
continues to refer to the entire carton task and remains false: pickup does not
establish folding or sealing. Four-flap runs require independent flap-shape and
retracted-tool gates over multiple frames. All flaps are rechecked together for
late springback, including after terminal placement when configured. Folding
without a `finish` retains a healthy supervised session and is not unattended.

The owner must publish `gripper_release_generation` and
`automatic_gripper_reenable: false`. Any unplanned jaw release invalidates the
grasp. The program never resets a fault budget or silently releases/regrips.
Motor thermal/load faults, encoder lag, stale vision, stale telemetry, tick
watchdog expiry and STOP interrupt a trajectory. Existing strict telemetry
validation and the owner's torque/release policy remain authoritative.

## OAK-D Lite: reuse the verified camera handoff

Use the existing isolated environment pinned by `requirements-oak.txt`
(DepthAI 2.33.0.0, tested with USB 2). Run one camera producer on the **robot Mac**:

```sh
python -m farm.oak_camera list
python -m farm.oak_camera stream --usb2 --seconds 300 --output /path/to/oak-stream
```

The producer adds a continuously updated `oak.json` manifest and immutable RGB
JPEG/16-bit depth PNG pairs. It retains the last 90 pairs. Each pair includes
hashes, SDK-derived capture times accounting for queue/USB delay, local host,
device and stream identities, sequence, intrinsics and aligned axial millimetre
units. Zero remains missing depth. This is based on the existing calibrated
[Luxonis aligned-depth pipeline](https://github.com/luxonis/depthai-python/blob/v2.33.0.0/examples/StereoDepth/rgb_depth_aligned.py).

To require depth in the program, add a `depth` object to the recipe:

- `manifest`, `camera_id`, `reference`, `regions`: OAK-specific manifest,
  registered identity, current RGB reference image and existing patch/AprilTag
  seed format. **Head/wrist camera pixels are not OAK pixels.**
- `tool`, `paddle`, `bottom`: names of independently tracked OAK features;
  bottom must identify the relevant lowest part of the paddle.
- `table_features`: at least six well-spread, stationary table features with
  `anchor: true` in their region definitions.
- `up_hint_camera`: measured unit direction away from the table in OAK optical
  coordinates; `min_lift_mm`, `min_clearance_mm`: measured thresholds of at
  least 10 mm; `max_slip_mm`: no more than 5 mm.

The depth observer uses only this camera's intrinsics and aligned pixels.
It rejects stale frames, cross-host timestamp assumptions, restarts, corrupted
files, missing pixels and patches spanning substantially different depths.
Table points define a plane; camera/table movement relative to the initial
registration invalidates it. Depth lift checks require actual motion away from
the table, tool/paddle co-motion and paddle-bottom clearance. Merely sliding the
object or overlapping it in a 2D projection does not pass.

Depth is **camera-frame observation**, not an arm target or collision-free path.
There is no guessed camera-to-robot transform. Physical accuracy and useful depth
coverage on the actual paddle must still be measured. Moving the robot body,
camera or table requires new reference images and registration; previous images
or image Jacobians must not be silently reused.

## Verification boundary

Tests exercise the actual command/status file transport plus synthetic image
features, thermal faults, absent grasp, lost retention, springback, stale models,
missing commissioning data and strict depth manifests. They open no motor or
camera devices and do not simulate carton contact mechanics. Hardware speed,
grasp reliability and folding success require separate physical evidence.
