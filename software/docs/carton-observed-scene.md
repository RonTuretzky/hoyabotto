# Observation-derived folding collision scene

`carton/folding_observed_scene.py` constructs a **geometric planning snapshot**
from calibrated RGB-D estimates, named robot encoders, and a declared measured
robot/station/carton model. It accepts no simulator or existing `MjData` object.
It does not command hardware, step dynamics, certify a physical calibration, or
establish that a planned contact is safe to execute.

The existing simulator-based `JointPathPlanner` can consume this snapshot
through `ObservedScene.planner()`. No shared planner, controller, PixelPort, or
hardware-owner code is changed by this component.

## API

```python
from carton.folding_observed_scene import ObservedSceneBuilder

builder = ObservedSceneBuilder(measured_robot, measured_calibration)
scene = builder.build(
    rgbd_estimates,
    named_encoder_packet,
    now_s=current_monotonic_time,
    required_flaps=("short_left", "short_right", "long_far", "long_near"),
    unknown_flaps="refuse",
)
planner = scene.planner(
    "left",
    now_s=current_monotonic_time,
    clock_id=measured_calibration.clock_id,
    calibration_id=measured_calibration.calibration_id,
)
path_proposal = planner.plan(goal_joint_angles)
```

Use the wrapper to preserve freshness and mandatory uncertainty margins. Calling
`JointPathPlanner(scene, ...)` directly bypasses that wrapper. The exposed
MuJoCo model and data are intended for trusted in-process geometric code, not
as an immutable security boundary.

The builder accepts **observation-derived estimates**, not raw image arrays.
The upstream RGB-D estimator must provide the provenance and bounded errors
below. A source label, a verified flag, or an empty obstacle list is an explicit
upstream assertion; this module cannot prove that a caller actually obtained
it from images or a physical survey. The schema intentionally rejects raw
PixelPort output rather than inventing missing metadata.

## Declared geometry

`RobotCollisionModel` requires flattened robot-only MJCF, its exact SHA-256,
model and measurement IDs, two named arm roots, an asset root, a SHA-256 map
for every external mesh/texture, and a bound on collision geometry error in
metres. Asset bytes are verified and frozen at builder creation. Later file
changes cannot alter a cached scene. The model must explicitly use radians
and contain exactly the six named rotational SO101 joints on each arm.

No template free bodies, equality constraints, unobserved scene roots, or
non-adjacent/cross-arm/environment contact exclusions are admitted. Explicit
contact pairs are refused because their settings can override collision
margins. Actuator, sensor, keyframe, and extension sections are removed from
the geometric copy. Compiled collision detection must be enabled; both arms
must have active collision geometry and mutually compatible collision masks.
Generated obstacles use explicit masks that interact with every supported
active robot mask, even if the XML default is visual-only. Only nonnegative
31-bit masks are supported.

`SceneCalibration` requires identities for the calibration, measurement,
world frame, clock, camera, robot, carton, and station inventory. It also
requires both measured robot-base rigid transforms, their position/rotation
error bounds, anchor IDs, a nonempty explicit station-box inventory, and a
`MeasuredCarton`. `measurements_verified=True` is required but is not itself
evidence that measurement occurred.

Each `StationBox` is an oriented box with a proper `world_from_box` transform,
three half sizes in metres, and a point-position error bound. Its dimensions
must conservatively cover the relevant fixture or obstacle. Unknown loose
objects cannot be silently omitted from this inventory.

`MeasuredCarton` specifies length, width, wall height, flap length, thickness,
major-hinge height offset, dimension-error bound, all four hinge intervals,
and unique flap tag IDs. Its frame is exactly
`bottom_center_z_up_x_length_y_far`: the carton bottom centre is the origin,
positive x follows length, positive y points toward the far wall, and positive
z is up. Flaps are upright at zero angle and fold inward with positive angles.
The carton can have a fully observed rigid world rotation and translation.

`contents_height_m` is mandatory: `None` explicitly declares an empty carton;
a numeric value creates an inflated full-footprint box from the bottom to
that height. This is a conservative contents envelope, not a hidden default
payload state. Loose contents extending beyond that envelope invalidate the
declaration.

## Observation and encoder envelope

All timestamps and expiries must use the declared common monotonic clock.
Sequence numbers are positive integers and must increase on successful builds.
A failed build does not consume a sequence. Default maximum age is 150 ms,
RGB/depth or RGB/encoder skew is at most 20 ms, and future timestamps are
refused.

The RGB-D packet requires these fields:

| Fields | Required meaning |
| --- | --- |
| `seq`, `timestamp_s`, `depth_timestamp_s` | Frame identity and separate RGB/depth capture times |
| `uncertainty_valid_until_s` | Time through which the stated geometric bounds cover both measurement error and possible object motion |
| `clock_id`, `calibration_id`, `world_frame_id`, `sensor_id`, `carton_id`, `inventory_id` | Exact matches to the measured declaration |
| `source`, `length_unit`, `angle_unit` | `calibrated_rgbd`, `m`, `deg` |
| `world_from_camera`, `world_from_box` | Finite proper 4×4 rigid transforms in the declared frame |
| `box_registration` | Current `source_seq`, `identity_verified=True`, `ambiguous=False`, `position_error_m`, `rotation_error_deg` |
| `anchor_ids`, `anchor_fit_rms_mm` | Current declared anchor observations and fit RMS at most 6 mm |
| `unmodelled_obstacles` | Explicitly `[]`; missing, unverified, or nonempty inventory refuses planning |
| `tags`, `angles` | Current decoded IDs and named flap rows |

Each flap row requires `degrees`, `observed_seq`, its exact flap `identity`,
`unambiguous=True`, `error_bound_deg`, and an accepted `method`. The entire
angle interval must lie inside the measured hinge limits. Default caps are
12 mm carton translation error, 3° carton rotation error, and 5° flap-angle
error. These are acceptance ceilings, not automatically assigned errors.

Accepted methods are:

- `apriltag_aligned_depth_plane`: the flap's declared tag ID must be decoded
  in this same frame.
- `aligned_depth_hinge_consistent_plane`: major flaps only, with at least 60
  supporting pixels, at least six spatial patches, hinge-axis error at most
  4°, and absolute hinge-plane offset at most 6 mm.

The legacy cardboard-plane method is not sufficiently identity-specific for
this boundary. Prior angles cannot supply a missing measurement.

The encoder packet requires its own sequence, capture time, bound expiry,
clock/calibration IDs, matching robot/model IDs, `angle_unit="rad"`, and
`position_frame="model_joint_coordinates"`. Both `joint_positions` and
`error_bounds_rad` must contain exactly all twelve named joints:
`left_` and `right_` versions of `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`, and `gripper`. Initial uncertainty intervals must
fit inside the measured joint limits. The default encoder-error ceiling is
0.020 rad. Raw ticks or normalized percentages require a separately measured
mapping; they are not accepted as joint angles.

## Missing flaps and conservative occupancy

Every required flap must have a fresh unambiguous supported estimate. Missing,
stale, ambiguous, or out-of-range required flaps always refuse the scene.
Removing a flap from `required_flaps` alone does not permit it to disappear.

With the default `unknown_flaps="refuse"`, any unobserved flap refuses the
scene. An explicit `unknown_flaps="swept"` allows an optional unobserved flap
to become an inflated box covering its complete 360° hinge sweep. It has no
hinge state and cannot be listed as an allowed contact face. There is no
restoration from a default or simulated angle. This deliberately may block an
approach that would be possible if the flap were known to be open.

The current observer's visibility limits remain relevant: the narrow legacy
short-flap estimate is not admitted, and current tag-angle gates do not cover
all strongly outward-open states. If opening a short flap outward removes its
valid observation, the next stage must either obtain another calibrated view
or accept the conservative swept obstacle. If that swept volume blocks the
stage, this backend refuses it. These gates are not widened here.

## Error bounds and planning limits

Carton boxes are expanded by the sum of translation error,
`2 * radius * sin(rotation_error / 2)`, and six independent dimension-error
bounds. The radius covers all carton/flap points, including declared contents.
Each observed flap adds `2 * (flap_length + half_thickness) * sin(angle_error / 2)`.
Station boxes add their own point-error bounds.

Robot clearance uses an all-pose reach bound from the declared kinematic
chain and collision geometry. Each arm contributes its base-position error,
geometry error, and reach times the sum of six encoder angular errors and
base-rotation error. Both arms' bounds are summed to cover relative arm/arm
uncertainty. Another 0.1 mm preserves the bound despite the existing planner's
signed-distance tolerance. The wrapper requires this clearance and refuses
combined clearance above the existing planner's 20 mm capacity.

Only freshly observed flap faces may be nominated as allowed contacts. The
existing planner's nominal 1 mm penetration gate remains unchanged. Its
intentional-contact exception does not enforce the robot clearance margin
against that face; it is not a physical contact-force or penetration guarantee
under encoder/tracking error. Leave `allowed_flaps=()` for conservative
free-space path proposals. A future contact executor needs separate bounded
tracking, contact-force, and live-observation checks.

Scene expiry is the earliest capture-age deadline or supplied error-bound
expiry. `scene.planner()` checks it when the planner is created. Planning can
outlast that expiry, and the existing planner samples joint-space edges rather
than proving continuous swept-volume clearance. The returned path is therefore
only a proposal. A future execution adapter must reacquire/revalidate current
observations and encoders, account for path tracking and between-sample motion,
and stop on expiry or invalid bounds. No such executor or readiness claim is
included here. Gripper and other-arm poses are held at the captured encoder
state during a one-arm planning call.

## Verification, 2026-10-07

Run from `software`:

```sh
PYTHONPATH=. .venv/bin/python -m pytest tests/test_folding_observed_scene.py tests/test_folding_paths.py -q
```

91 component and existing-planner tests pass. Negative cases include stale,
future, unsynchronized, mismatched-unit or identity data; missing error bounds;
wrong flap tags; ambiguous required flaps; invalid transforms; unverified
models/assets; missing encoders; suppressed collisions; and expiry. Tests
also verify that a full missing-flap sweep blocks space a guessed angle would
leave clear, that both arms' uncertainty is reserved, and that later external
mesh-file mutation cannot alter a cached scene.

The hidden-state invariance test changes a separate simulator data object's
carton pose and every flap angle while keeping the exposed observation/model/
encoder inputs fixed. Rebuilt qpos, scene hash, and planned path are identical.
A world-frame transformation test moves all declared objects together and
preserves path results.

Local artifacts are in
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/observed-scene-2026-10-07/`.
`junit.xml` records the tests. The robot-template compile diagnostic uses the
actual SO101 robot CAD and 469 verified asset files, producing 1,084 geoms and
23 qpos values, with zero actuators and zero equality constraints. It uses a
**synthetic** observation/calibration envelope and recorded robot encoder
values only; it does not validate physical RGB-D accuracy, survey completeness,
whole-robot execution, or carton closure. No recorded carton/flap state supplies
the constructed obstacle poses. The earlier compile report is preserved;
the final report and source hashes identify the version with collision-filter
validation.

The final CAD diagnostic uses the test fixture's explicit bounds: 0.5 mm base
position, 0.1° base rotation, 0.2 mm geometry error, and 0.001 rad per encoder.
They produce 10.544 mm global robot clearance. The initial left configuration
is **refused** because a wrist/jaw pair has 9.881 mm nominal separation. This
global bound deliberately overestimates uncertainty even for self-contact
pairs whose base errors cancel. The earlier compile report used a different
synthetic uncertainty envelope and its accepted start is not comparable to
this refusal. No bounds were reduced to obtain a path. Tighter measured
bounds or a separately validated pair-specific uncertainty planner would be
needed to resolve this conservatism.

Physical measurement, a truthful timestamped observation adapter, robust
visibility for each required flap, and a bounded execution/revalidation adapter
remain prerequisites for hardware use.
