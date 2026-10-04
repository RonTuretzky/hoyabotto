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

Development-Mac validation on October 4: **243 tests passed**, including 44
controller-specific cases; the Swift publisher compiled. The rendered-image
experiment measured a four-joint model, passed independent probes (maximum
0.864px prediction error), then reached the target with 15 corrections and
three final observations. All four feature errors were within 2.001px.
See [sanitized synthetic evidence](evidence/carton-servo-synthetic-check.json).
The test contains no carton/contact physics and opens no devices.

## Transfer without disturbing the existing session

Source branch: `codex/carton-visual-controller` in the existing repository.
On the robot Mac, inspect the current checkout and preserve all `work/` scripts,
calibration files, profiles and environment. Fetch this branch into a **separate
worktree**; do not reset or replace the live checkout. Launch this package with
the existing environment's Python and `PYTHONPATH` pointing to this worktree's
`software/`. A fresh install of the entire LeRobot/training stack is unnecessary.
The new runtime needs NumPy and OpenCV, already used by this project.

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
and fold trajectories, reuse LeRobot's URDF-based kinematics after measuring
model zero/sign and station registration. Do not extrapolate this local image
model across the fold.

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
