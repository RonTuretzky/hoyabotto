# Local carton execution and OAK depth observations

`carton.servo.program` executes an already commissioned sequence in one local
process: approach, exact-position jaw closure, test lift, four flap paths,
retraction and image checks. It makes **zero LLM calls**. It retains one motor
owner and one command-writer lock across stages. `progress.json` reports its
current stage; `trace.jsonl` and `result.json` preserve measurements and failures.

This is software for station commissioning, **not a completed carton-folding
deployment**. No real grasp, lift or fold has been verified. No station recipe
is shipped with invented targets. The current positioning adapter still uses
the existing bounded alignment controller and its local model; its command
granularity is not yet a continuous trajectory implementation. A recipe outside
that measured neighborhood is rejected before sending a partial sequence.
Supporting this station's larger motions requires a commissioned trajectory
controller, not increasing the local Jacobian's trust radius arbitrarily.

## Prepare everything before energizing the arm

From `software/`, using the robot environment's Python:

```sh
python -m carton.servo program-template --config /path/to/experiment.json --out /path/to/recipe.json
python -m carton.servo program-check --recipe /path/to/recipe.json
python -m carton.servo program-run --recipe /path/to/recipe.json --out /path/to/new-run
# Only a commissioned recipe against a healthy existing owner can execute:
python -m carton.servo program-run --recipe /path/to/recipe.json --out /path/to/new-run --execute
```

The template contains missing measurements deliberately. Static checking lists
all missing recipe fields at once. Passing static checks does not establish live
readiness; model age/session, camera identity/freshness, actual start position,
lease and motor health are checked again against the running owner.

Targets are image-feature vectors in the experiment's measurement order.
The model must describe positioning joints relative to the fixed station, not
relative to the paddle or a flap that will move on contact. There must be enough
independent visual measurements to identify every selected joint. Each flap
requires path targets, retract targets, and independent outcome/clearance checks.
The head camera needs seeded `short_left`, `short_right`, `far_long`, `near_long`
features and stationary references. Constraint fields are:

```json
{"role":"flap", "camera":"head", "a":"short_left", "b":"station_anchor",
 "direction":[1,0], "interval_px":[100,104]}
```

These numbers illustrate the format, **not this station's calibration**. A second
constraint with role `tool_clearance` must compare the tool with that flap after
retraction. Both checks run for at least three fresh observations over the
commissioned interval. All four flaps are checked again together at the end to
detect an earlier flap reopening during a later fold.

Jaw closure uses measured open, empty-closed and desired grip encoder positions.
It does not infer a successful grasp from a load spike or stalled jaw. Endpoint
failure stops the program. A lift needs head-view upward tool/paddle co-motion,
visible paddle-bottom clearance, stable wrist-relative geometry, and nonempty
jaw aperture on three fresh frame pairs. Wrist-relative retention is monitored
during every later alignment correction, not just between stages.

There is no automatic re-energization, fault retry or lease reset in this client.
The owner must publish an integer `gripper_release_generation` (incremented on
every jaw torque release) and `automatic_gripper_reenable: false` while this
program executes. Any generation change invalidates all grasp evidence. Older
owners without these fields are rejected before motion. The exported owner's
temperature-confirmation path can release and re-enable the claw; that behavior
must not silently continue inside a grasp sequence. The owner integration for
this contract remains required before deployment.
Accepted-command lease behavior remains with the motor owner. A fault requests
the owner's STOP/release policy; the client never substitutes a new torque
policy. Successful checks retain the owner's healthy session and its existing
lease/health supervision. This version does not implement paddle parking, a
verified unloaded rest pose or tape sealing; `physical_task_completed` therefore
remains false even when `CARTON_VISUAL_CHECKS_PASSED` is returned. The program is
not suitable for unattended operation until those terminal behaviors and the
actual station recipe are commissioned.

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
