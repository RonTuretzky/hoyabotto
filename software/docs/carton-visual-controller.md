# Carton: measured visual control without demonstration training

This package is ready for **software review and bounded hardware commissioning**.
No physical grasp, flap fold or tape placement is certified by this handoff.
Read the [upstream comparison](carton-upstream-review.md) for the reuse decision
and [original station specification](carton-connected-mac.md#3-set-up-the-fixed-station)
for the measured carton. Keep carton-only scope: no conveyor, pushing or cycle-time work.

## What it does

`python -m carton.servo` provides:

- A macOS camera publisher with immutable, hashed frames and capture timestamps.
- Read-only camera timing checks and annotated target/tool tracking.
- Reuse of the farm's AprilTag detector for identified tool/target/anchor features.
- LeRobot/Placo FK and checked reach proposals using the pinned upstream SO-101 model.
- Bounded single-joint probes that measure a local encoder-to-image Jacobian,
  plus independent half-sized probes withheld from fitting.
- A numerical correction loop with no LLM request between movements.
- Refusal on stale frames, camera movement, uncertain tracking, bad motor health,
  command conflicts, unexpected joint motion, model disagreement or exhausted budgets.
- JSONL observations/actions, annotated images, numeric error and explicit outcomes.
- Saved visual alignment goals, a grasp-evidence checker and a crease-arc planner.

It does **not** open a motor port, change calibration, extend a supervision lease
in the background, command wheels/head/gripper, plan collision-free paths,
automatically close a gripper, or execute an entire fold. Four image coordinates
do not establish full gripper orientation or contact geometry. Use it initially
for a nearby, collision-clear approach to the paddle handle, then establish the
separate grasp and contact primitives on the robot.

Development-Mac validation on October 4 after upstream integration: **265 tests
passed in 82.19 seconds**, including 44 controller cases and 22 upstream integration
cases, with no skips. The Swift publisher compiled before this refactor and is
unchanged. The rendered-image
experiment measured a four-joint model, passed independent probes (maximum
0.864px prediction error), then reached the target with 15 corrections and
three final observations. All four feature errors were within 2.001px.
See [sanitized synthetic evidence](evidence/carton-servo-synthetic-check.json).
The test contains no carton/contact physics and opens no devices.

The subsequent upstream integration was also exercised with the **actual**
LeRobot/Placo solver and pupil-apriltags detector. Two nonzero URDF reach cases
converged in 2 and 3 solver iterations, with position residuals of 0.275 mm and
0.131 mm. These are model-consistency errors, not measured robot accuracy. See
[the reproducible solver evidence](evidence/carton-upstream-kinematics-check.json).
The original environment, without Placo, also passed 58 controller/integration
tests with 8 explicit solver skips. The wheel includes the shared adapters and
model manifest; a fresh `model-fetch` verified all 16 upstream files. Full-suite
validation used an environment excluding global site packages after a Homebrew
SciPy/native-library conflict was found in the initial test environment.

## Transfer without disturbing the existing session

Source branch: `codex/carton-visual-controller` in the existing repository.
On the robot Mac, inspect the current checkout and preserve all `work/` scripts,
calibration files, profiles and environment. Fetch this branch into a **separate
worktree**; do not reset or replace the live checkout. Launch this package with
the existing environment's Python and `PYTHONPATH` pointing to this worktree's
`software/`. A fresh install of the entire LeRobot/training stack is unnecessary.
The visual loop needs NumPy/OpenCV and uses the existing pupil-apriltags dependency
when tag features are selected. Optional geometric proposals need the isolated
kinematics dependencies below; they do not require replacing the live environment.

First reproduce, without cameras or motors:

```sh
python -m pytest -q tests/test_carton_servo.py
python -m carton.servo simulate --out data-carton/servo-synthetic-first
swiftc carton/servo/capture.swift -o data-carton/carton-capture
```

Use a new output directory on every run. Synthetic results say
`SYNTHETIC_RENDERED_PIXELS`, `robot_connected: false` and
`physical_task_completed: false`. They cannot be saved as physical skills.

## Device ownership and camera commissioning

The existing robot-local `work/carton_session.py` remains the sole motor owner.
Its STOP viewer remains reachable. The new client writes its existing
`command.json` protocol and reads `status.json`; it never calls the serial SDK.
Confirm this contract against the current owner before powered tests:

- `arm`, `phase`, `ok`, `started`, `time`, `lease_remaining`, and all six selected
  arm `rows` with raw `Present_Position`, `Present_Load`, `Present_Temperature`, `Status`.
- One-joint `{id, op: "move", delta_ticks: {joint: integer}}` commands; an exact
  `completed` ID; `{id, op: "stop"}` retains the owner's established stop policy.
- Saved travel ranges and existing speed, torque, temperature, load and lease checks.

Stop the old manual command producer before giving this client command ownership.
The new clients share a file lock and detect foreign command writes, but the old
manual tool does not cooperate with that lock: **there must be only one command
producer**. Never run an upstream standalone motor script beside the owner.

Prepare the camera publisher while the arms are supported and the powered session
is stopped. Replace the previous camera process, rather than adding another one.
Run from the robot Mac's Terminal that already has camera permission. Obtain the
actual IDs, identify the views from images, then supply those exact IDs:

```sh
data-carton/carton-capture --list
data-carton/carton-capture /absolute/path/to/existing/frame-directory \
  'head=EXACT_HEAD_DEVICE_ID' 'right_wrist=EXACT_RIGHT_WRIST_DEVICE_ID'
```

The placeholders above must be replaced from that Mac's device inventory. The
publisher requires a supported 640×480/5fps format and writes `head.jpg` and
`right_wrist.jpg` for viewer compatibility. Control reads the immutable image
named in each atomic JSON manifest. `received_at` remains available for the old
owner; `captured_at` comes from the camera sample timestamp. Confirm the existing
owner/viewer uses these filenames and freshness keys before restarting it.

In a second Terminal:

```sh
python -m carton.servo camera-check --frames /absolute/path/to/frame-directory \
  --arm right --seconds 20 > data-carton/camera-audit.json
```

Pass requires distinct devices, advancing sequences/timestamps, matching image
hashes, fresh frames, at least 4 delivered fps per camera, gaps no longer than
0.5s and pair skew no greater than 0.3s. This is a delivered-timing check, not a
USB root-cause diagnosis. If it fails, inspect the active device format and USB
topology; eliminate duplicate readers and reduce load at its source. Do not just
increase tolerated frame age. Native AVFoundation sample timing and USB behavior
still need validation on the connected Mac; local compilation cannot prove them.

## Seed the experiment before enabling movement

Use the actual calibration file already loaded by the motor owner:

```sh
python -m carton.servo prepare \
  --session /absolute/path/to/work/carton-session \
  --frames /absolute/path/to/frame-directory \
  --calibration /absolute/path/to/this-robots-calibration.json \
  --arm right --out data-carton/paddle-approach-01
```

This reads files and saves two reference images and `experiment.json`; it does
not command or connect to motors. The configuration is deliberately incomplete.
Astra should open the reference images, select textured regions for `tool` and
`target`, and select a fixed table/background `anchor` in the head image. These
are visual annotations, not hand-guided motion demonstrations. In a pickup
experiment `tool` means the gripper, and `target` means the paddle handle.

Use `seed --config ... --camera head --region 'tool:x,y,w,h'
--region 'target:x,y,w,h' --region 'anchor:x,y,w,h'`, substituting actual pixel
rectangles. Repeat for `right_wrist` with tool and target. `--point name:x,y`
selects a reference point within a region. See `seed --help` for syntax.
If the plastic or cardboard has no distinctive texture, use a better visible
feature or a fixed visual marker; never accept an arbitrary tracker match.

To reuse the farm's existing AprilTag perception, put distinct **tag36h11** IDs
on the gripper/tool, target and fixed head-view background, then use:

```sh
python -m carton.servo seed --config data-carton/paddle-approach-01/experiment.json \
  --camera head --tag tool:2 --tag target:3 --tag anchor:1
python -m carton.servo seed --config data-carton/paddle-approach-01/experiment.json \
  --camera right_wrist --tag tool:2 --tag target:3
```

IDs above are examples; identify the mounted markers in both actual reference
images. Features can mix patches and tags. A tag point defaults to its centre;
`--point` can select a point inside its quadrilateral. Missing/duplicate IDs,
weak/corrected decoding, a moving head anchor, a resolution change or more than
100 pixels of displacement from the seed refuse. Anchor corner checks also catch
rotation. This is 2D tracking; tags do not automatically establish camera intrinsics,
depth, robot registration or a grasp pose. Mount tags outside contact/occlusion areas.

The default feature order is head target-minus-tool x/y followed by wrist
target-minus-tool x/y. **The default `[0,0,0,0]` is not an automatically safe grasp
goal.** Set `--target` to the image offsets of the intended collision-clear
approach. Keep the selected points visible and outside the occlusion/contact
zone. A wrist-mounted camera needs a gripper feature actually visible in its
image, or an explicitly redesigned measurement definition; do not fabricate one.

After the ROI work is complete, establish the existing guarded holding session.
Review clearance for the entire ±32-tick probe and return envelope, including
the paddle, wires and table. Keep the fixed camera/head and station unchanged.

```sh
python -m carton.servo inspect --config data-carton/paddle-approach-01/experiment.json \
  --out data-carton/paddle-inspect-01
```

Inspect the labelled images in this run. `inspect` sends no motor command. Confirm
target identity and physical clearance from the actual scene; a passing tracker
is a numerical measurement, not a semantic or collision check.

## Measure, validate, then approach

The following calibration command **moves the selected arm's positioning joints**.
It probes each selected joint ±32 encoder ticks (about ±2.8°), returns to the
measured origin, then checks independent +16-tick probes. Other joints hold.
It uses the existing session, and fails if the owner/lease/telemetry is not ready.

```sh
python -m carton.servo calibrate --config data-carton/paddle-approach-01/experiment.json \
  --out data-carton/paddle-model-01 --execute
```

Default four-joint calibration has 24 bounded moves including returns. No model
is published unless stationary jitter, return error, rank/conditioning, measured
encoder displacement, fit residual and all holdout checks pass. Backlash or
contact may invalidate the experiment; preserve the failed trace.

Review `model.json`, the holdout errors and images. Then request a read-only
proposal using a fresh observation:

```sh
python -m carton.servo align --config data-carton/paddle-approach-01/experiment.json \
  --model data-carton/paddle-model-01/model.json --out data-carton/paddle-shadow-01
```

Only after that proposal and the observed clearance are sensible, run the same
command with a fresh output directory and `--execute`. It commands one joint
at a time, default maximum 16 ticks per correction, default 96 ticks from the
local origin, at most 80 iterations and 180 seconds. Each action must settle
within 5 ticks in two telemetry samples, then produce a fresh camera pair.
The older owner's looser completion threshold is insufficient by itself.

`ALIGNED_ONLY` requires all feature errors within 3 pixels in three distinct
observations. It does not close the gripper or imply a grasp. A model expires
after 15 minutes and is invalid after a camera/owner restart, seed/measurement
change, motor calibration change or scene/model disagreement. Do not reset a
budget repeatedly to force the same failing approach.

On a failed moving experiment, the client asks the existing owner to STOP using
its current release behavior. There is no blind return or automatic recovery
motion. Successful commands do not create a permanent holding lease: finish the
session using its established support/stop procedure.

## Evidence and the next physical milestones

```sh
python -m carton.servo report data-carton/paddle-align-01
python -m carton.servo save-skill --config data-carton/paddle-approach-01/experiment.json \
  --run data-carton/paddle-align-01 --name paddle-approach \
  --out data-carton/skills/paddle-approach-v1.json
```

The saved artifact is an achieved **visual goal**, not an executable joint
trajectory. It requires local recalibration before reuse and never marks a fold
or grasp complete. Keep private camera images and calibration artifacts under
ignored `data-carton/`; commit only source and sanitized evidence summaries.

| Milestone | Required physical evidence | Available now |
|---|---|---|
| Local approach | Held-out probe validation, actual error reduction, stable final target | Controller and evidence capture |
| Paddle grasp | Measured jaw closing direction/empty-closure baseline, bounded closure, short clear lift, independent tool/object co-motion | `verify-grasp --evidence FILE --out FILE` numerical checker; closure/lift execution must be commissioned on the robot |
| One flap | Measured crease/tool geometry, controlled contact, flap-angle progress, successful release without reopening | `hinge-plan --spec FILE --out FILE` geometric point arc only; it produces no motor commands |
| Four flaps | Successful single-flap primitive plus reach/collision checks and retention of previous folds | Unvalidated |
| Tape | Actual dispenser presentation, grip, orientation, release and adhesion on the closed box | Existing staged requirements in `carton-tape.md`; physical skill unvalidated |

`verify-grasp` consumes `before`/`after` head-view tracked point dictionaries,
`gripper_ticks`, measured `empty_closed_ticks`, and `min_aperture_ticks`. Both
gripper and object must move at least 8 pixels together with at most 3 pixels of
relative slip, and the jaw must stop short of empty closure. Use a stationary
head anchor and a separately reviewed vertical lift: sliding an object on the
table can also produce co-motion and must not be labelled a lift. This checker
is supporting evidence, not a tactile sensor or a standalone grasp guarantee.

`hinge-plan` accepts `hinge`, `axis`, `contact` in a common measured 3D frame
(metres), plus `start_deg`, `end_deg`, optional `step_deg` (default 5°). It keeps
the contact point on a circular crease arc. It does not establish paddle
orientation, forces, springback or a camera/robot transform. For full approach
and fold geometry, use the integrated LeRobot proposals below after measuring
model zero/sign and station registration. Do not extrapolate this local image
model across the fold.

## Reuse of existing perception and kinematics

The shared implementation is in `farm/kinematics/`; the carton package supplies
configuration, provenance and observation adapters. It calls LeRobot's
`RobotKinematics`, with Placo enforcing the upstream joint limits. It does not
connect through LeRobot's motor driver or use the legacy planar Cartesian helper.

For the optional geometric tools, use Python 3.12 and a separate environment.
The pins match LeRobot 0.6.1's kinematics requirements; do not install a newer
Placo into the currently running motor owner's environment.

```sh
uv venv .venv-carton-geometry --python 3.12
uv pip install --python .venv-carton-geometry/bin/python -e '.[dev,carton-kinematics]' \
  -c constraints-carton.txt -c constraints-carton-kinematics.txt
.venv-carton-geometry/bin/python -m carton.servo model-fetch \
  --out data-carton/models/so101
.venv-carton-geometry/bin/python -m carton.servo kinematics-check \
  --model-dir data-carton/models/so101 --out data-carton/kinematics-check.json
.venv-carton-geometry/bin/python -m pytest -q tests/test_carton_upstream.py
```

Only `model-fetch` downloads files. The unchanged upstream URDF, referenced
meshes, README and license are pinned in `farm/kinematics/so101-assets.json` and
checked before every model load. Conflicting local files are preserved and
refused. The model is ignored data, not a new vendored solver. Set
`CARTON_MODEL_DIR` for tests if it lives elsewhere. Real solver tests explicitly
skip when the optional assets/dependency are absent; skips are not solver validation.

Create a draft bound to this robot's existing motor calibration:

```sh
.venv-carton-geometry/bin/python -m carton.servo kinematics-template \
  --config data-carton/paddle-approach-01/experiment.json \
  --model-dir data-carton/models/so101 --out data-carton/paddle-kinematics.json
```

The draft deliberately has unknown values. Measure these before using it:

- `joints`: each of the five arm joints needs a `model_zero_tick` and a
  `model_sign` of +1 or -1 relative to **this new-calibration URDF**. Saved travel
  endpoints alone do not determine a geometric zero. The shared adapter uses
  4096 encoder ticks per revolution; it never equates normalized values to degrees.
- `gripper_from_tool`: a right-handed 4×4 transform in metres mapping tool
  coordinates into `gripper_frame_link`. Identity explicitly selects the URDF
  frame origin, which is not automatically the grasp/contact point.
- `workspace_bounds_m`: measured `[minimum_xyz, maximum_xyz]` limits in the
  arm-base frame. Bounds constrain proposed endpoints; they are not collision checks.
- `base_from_station`: a right-handed 4×4 transform mapping station coordinates
  into this arm's base frame. It may remain null for arm-base-only requests.

Check model predictions against independent measured tool positions at several
separated poses before relying on this mapping. The code checks file binding,
units, transforms, ranges and numerical consistency; it cannot certify physical
calibration from a supplied JSON file. Record those physical residuals separately.

Use the same session and image observations as the visual loop:

```sh
.venv-carton-geometry/bin/python -m carton.servo inspect \
  --config data-carton/paddle-approach-01/experiment.json \
  --kinematics data-carton/paddle-kinematics.json --out data-carton/geometric-inspect-01
.venv-carton-geometry/bin/python -m carton.servo plan-reach \
  --config data-carton/paddle-approach-01/experiment.json \
  --kinematics data-carton/paddle-kinematics.json \
  --request data-carton/reach-request.json --out data-carton/reach-proposal-01
```

`reach-request.json` requires `frame` (`arm_base` or `station`), `units` (`metres`),
`orientation` (`constrained` or explicitly `position_only`) and `tool_poses` (1–100
4×4 transforms). Start by checking the currently observed tool pose from
`geometric-inspect-01/result.json` before specifying independently measured targets.
An offset tool requires constrained orientation. A five-joint arm cannot realize
every six-dimensional pose; infeasible requests refuse instead of clamping.

Each proposal checks upstream FK residuals and then checks them again after
rounding to encoder ticks and applying the full tool offset. Results include the
starting encoders, fresh camera/session identity, configuration fingerprint,
model revision and actual LeRobot/Placo versions. Changed calibration, a different
arm, mismatched ranges or a stale/moving starting state refuse.

`KINEMATIC_PROPOSAL_ONLY` has no execution switch. Waypoint endpoints do not prove
clearance of intervening motion, reach of the other arm, contact forces or flap
progress. Placo reports adjacent mesh intersections at the upstream model's
neutral pose; this model has not been configured as a validated collision model.
The proposals explicitly report `collision_checked: false`. The existing bounded
visual loop remains the only added powered behavior. Do not replay proposal ticks
as a folding sequence or pipe them directly into the motor owner.

## Changes to older paths

The legacy Cartesian helper mixed geometric degrees with normalized motor
positions. Physical `LLMServo` teaching, Cartesian moves and the unmeasured
rest fallback now refuse. Its planar inverse math is also corrected and rejects
unreachable targets; simulator seeds were adjusted to remain reachable. The
existing measured joint-keyframe interfaces remain, but their existence does not
validate any stored pose. Do not follow the older `teach-all`/tape-teaching recipe
as a physical workaround.

This is ready to measure whether the new local loop works on the actual Mac.
It is not a prevalidated autonomous carton-folding system. Report the first
physical gate's measurements and failure reason before expanding its scope.
