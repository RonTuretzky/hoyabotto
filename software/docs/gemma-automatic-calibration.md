# Automatic calibration through Gemma's existing robot connection

The integration uses the existing authenticated `Robot` client, `TagRobot`,
`Experiment.calibrate`, stationary encoder/tag sampler and OpenCV hand-eye
fitter. It creates no camera stream or serial owner. It changes no calibration
registers, controller limits or STOP state.

## What is available

- `robot_calibration_status`: read-only owner readiness, tags 1/2, tag-2 mounting
  and image-border clearance, current arm geometry, and proposed registration grid.
- `robot_calibrate_tags(mode="local_model")`: hold the selected arm's five
  positioning joints, probe the configured joints in both directions, return
  after each probe, validate with independent smaller movements, and release.
  This produces a local pixel-motion model, not robot-frame coordinates.
- `robot_calibrate_tags(mode="registration")`: automatically collect eight
  training and three held-out poses, return to the starting joint positions,
  release the motors, then assemble candidate FK through the existing LeRobot
  model and fit camera/base and tag/gripper transforms. A rejected fit returns
  no transforms. A passing fit is saved as `.private/tag-registration.json`
  for read-only coordinate estimates. It still has `motion_ready:false`.
- `robot_get_registered_tags`: use the saved fit to return fresh tag centres
  and unambiguous poses in the selected arm's base. Rechecks the camera stream,
  intrinsics, tag sizes/mount, model, motor configuration and raw ranges, fixed
  table/head, and observed gripper pose against fresh encoder FK. A changed or
  missing binding refuses coordinates. Ambiguous object orientation produces
  only its centre, so an offset handle target cannot be inferred from it.
  This read-only tool enables neither Cartesian control nor physical grasping.
- `robot_get_paddle_target`: read-only guidance built on the same registered
  read. It returns the paddle tag 3 pose in the arm base and, once the owner has
  measured the offsets, the handle grasp point, approach direction, jaw tool
  poses and a labelled planner proposal. See
  [Paddle target (read-only)](#paddle-target-read-only).

Registration may also start from an arm the operator has just positioned and left
holding (`robot_calibrate_tags(mode="registration", held_start=true)`, CLI
`registration --execute --from-held`): exactly the six motors of the selected arm
must be enabled and the owner `holding`; no enable is sent, and the run still
releases or STOPs at the end. This is for arms that sag out of the camera view
when released (the right arm, 2026-10-10). Position with the pilot's ordinary
move tool and hand over within the owner's idle-hold lease.

Both execution modes are explicit tool calls for the current user-requested
calibration. Merely constructing the adapter, opening the chat, reading status,
or restarting the local chat cannot start or resume a calibration.

The default registration grid uses exactly two configured positioning axes,
96 ticks to either side of the observed start and 16-tick observed transitions
(`limits.trust_ticks` / `limits.step_ticks`; the real owner's controller stops a
few ticks short of each goal, so the 2026-10-10 configuration used trust 80, step
32, `settle_ticks` 16 (the cap), `frame_age_s` 2.0 and `max_path_ticks` 2000; see
commissioning/2026-10-10-apriltag-calibration/README.md).
Its planned total joint travel including return is 1248 ticks, under the
existing 1500-tick budget. It is a local grid, not a collision-checked trajectory.
The operator must supervise a cleared workspace. All grid endpoints must fit
inside the owner's commandable ranges before any motor is enabled. The fitter
also requires nonparallel rotational excitation, at least 25 mm positional
span, and independent validation poses. An unsuitable pair/view can therefore
fail; the code does not expand its limits or continue after that failure.

## Install on the existing Gemma Mac

First install the existing tag adapter and metric dependencies as described in
[the geometry guide](gemma-tag-geometry.md). Do not change the working DepthAI
camera environment. Registration mode also requires the existing LeRobot/Placo
FK dependencies and verified model assets; it checks those before enabling.
The complete pinned local-pilot dependency set is:

```sh
uv pip install --python /path/to/gemma/.venv/bin/python \
  -r /path/to/xlerobot-farm/software/requirements-gemma-calibration.txt
```

This retains OpenCV 4.11 for the hand-eye solver and adds LeRobot/Placo to the
pilot environment. Having the detector installed alone is insufficient.

Create `.private/tag-calibration.json` next to the pilot's `robot.json`:

```json
{
  "schema": 1,
  "arm": "right",
  "camera": "oak",
  "joints": ["shoulder_pan", "wrist_flex"],
  "model_directory": "/absolute/path/to/verified-so101-model"
}
```

The two joints above are an example for the confirmed right gripper. Their
use requires a usable starting pose, observable tag motion, and workspace
clearance. Tag 2's geometry file must also confirm `right` and
`fixed_gripper_housing`. Local configuration, not model arguments, selects
arm, joints, camera, model directory and any stricter experiment limits.

```sh
cd /path/to/xlerobot-farm/software
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/install_gemma_calibration.py \
  --pilot /path/to/gemma/pilot

# Read-only: no enable, move, STOP or owner restart.
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/calibrate_gemma_tags.py status \
  --pilot-root /path/to/gemma/pilot
```

The installer adds two source hooks to the already tag-enabled chat and
preserves a content-addressed backup. It does not restart anything. Reload the
**idle local chat** using its existing launcher to expose the three tools;
preserve history and leave old goals inactive. No camera or remote-owner
restart is needed. The CLI works without reloading the running chat.

When the operator is supervising and the status/current scene permit the
requested run, the equivalent explicit CLI actions are:

```sh
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/calibrate_gemma_tags.py local_model \
  --pilot-root /path/to/gemma/pilot --execute
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/calibrate_gemma_tags.py registration \
  --pilot-root /path/to/gemma/pilot --execute

# Read-only after a passing registration; prints JSON without image payloads.
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/calibrate_gemma_tags.py registered \
  --pilot-root /path/to/gemma/pilot
```

## Paddle target (read-only)

`robot_get_paddle_target` answers "where is the paddle handle in the right
arm's base frame?" It is information for the pilot. It sends nothing to the
motors. The repository rule still applies: reach-planner output is not sent to
the motor owner. The owner has not decided to change that rule.

Each call:

1. Reads `apriltag-geometry.json` (beside `tag-calibration.json`). A malformed
   `paddle_grasp` section refuses before any robot call.
2. Performs the same fresh registered read as `robot_get_registered_tags`. A
   missing registration, or any change that read refuses on, returns
   `ok:false` with that reason. Report "registration unavailable" and carry on
   camera-guided. It is not a prerequisite for `paddle-success-v1`.
3. Takes tag 3 from that fresh frame. If tag 3 is missing it refuses. If its
   orientation is ambiguous it returns only the tag centre.
4. Returns `paddle_tag` (centre in mm, `arm_base_from_tag` in metres).
   `frame_id` is `right_arm_base`. `freshness` carries the camera ID, stream,
   sequence, frame hash, capture time and age. `registration_sha256` and
   `grasp_geometry_sha256` identify the inputs.
5. Returns the grasp only when the offsets are measured. With `tag_to_handle`
   measured you get `handle_point_arm_base_mm`, `approach_direction_arm_base`,
   `jaw_closing_axis_arm_base` and a pre-grasp point `pregrasp_standoff_mm` back
   along the approach. With `jaw_contact` measured too you also get
   `grasp_tool_pose_arm_base` and `pregrasp_tool_pose_arm_base`. These tool poses
   follow the convention +z = approach, +y = jaw closing axis.
6. With tool poses available, asks `robot_plan_reach` (read-only) for the
   pre-grasp. Ticks come back only if the planner's `gripper_from_tool` matches
   `jaw_contact` within 1 mm and 1 degree, and its motor calibration matches the
   registration. They are labelled `"proposal, not executed, not collision
   checked"` with `executed:false` and `collision_checked:false`. Otherwise
   `reach_proposal.reason` says why. Today the right arm has no planner
   configuration, so expect the planner's missing-configuration list.
7. Refuses the whole answer if the frame is more than 6 s old when the answer is
   assembled. The registered read itself already refuses frames older than 3 s.

`uncertainty` lists its terms: registration train/validation residuals, the
live gripper-tag disagreement, the tag 3 position standard deviation from 0.5 px
corner noise (`tag3_depth_std_mm` is the camera-z, monocular depth term; OAK
stereo depth is not fused), the orientation error multiplied by the offset
length, and the declared offset tolerances. `combined_rss_mm` adds them in
quadrature. `conservative_bound_mm` adds the worst cases. Print size,
intrinsics, paper warp, joint backlash and poses outside the sampled workspace
are not included.

### What the owner measures

Fill `paddle_grasp` in `.private/apriltag-geometry.json`. For each block, set
`"status": "measured"` and record `tolerance_mm` (0-20) and a `source` (how,
when, who).

- `tag_to_handle.handle_center_mm`: the grasp centre on the handle, measured
  from the centre of tag 3's black square, in the **printed tag frame**. Hold
  the paddle so tag 3 looks like the kit image. +x points to the tag's right
  edge, +y to its top edge and +z out of the printed face (millimetres). For
  example, a handle 75 mm to the tag's left and 6 mm below its face is
  `[-75, 0, -6]`.
- `tag_to_handle.approach_direction`: the unit vector the jaw travels along to
  reach the handle, in the same frame. For a top-down grasp of a paddle lying
  flat, use `[0, 0, -1]`.
- `tag_to_handle.jaw_closing_axis`: the unit vector across the handle along
  which the jaws close, perpendicular to the approach. For a handle running
  along the tag's x axis, use `[0, 1, 0]`.
- `jaw_contact.gripper_from_jaw_contact`: a 4x4 transform in metres from the
  right gripper model frame (the FK target frame) to the jaw contact frame.
  The contact frame has +z out of the jaws and +y along the closing axis. It
  must equal the planner configuration's `gripper_from_tool` (step 12 of the
  registration slides) or the proposal is withheld.
- `pregrasp_standoff_mm` (20-150, default 60): the back-off distance along the
  approach.

The decoded tag frame that `robot_get_tags` returns is the printed frame
rotated 180 degrees about +y. The code applies this conversion, and
`tests/test_paddle_target.py` checks it against rendered detections. The 2026-10-06
rendered `[105, 0, 4.45]` offset belongs to the simulator mount and must not be
copied.

### Reach procedure for the pilot

The target is guidance only. Use it like this:

1. Call `robot_get_paddle_target {}`. If it refuses, do not reach by tags.
   Continue with the camera-guided procedure.
2. Never send `reach_proposal.joint_targets_ticks` as one move. Use them, and
   `joint_change_ticks`, only to see which joints change, in which direction,
   and by how much.
3. Move in small segments with the existing motion tools: `robot_move_path`
   with `wait=false` plus `robot_get_motion`, or `robot_move_joint_targets`.
   Change each joint by a small part of the remaining difference, for example
   a quarter of it and not more than about 100 ticks. These numbers are
   suggestions, not limits. The owner's existing limits, segmentation, contact
   guard, watchdog and STOP are unchanged and remain authoritative.
4. After every segment, look at the cameras (`robot_get_cameras` with `oak` and
   `right_wrist`). Once the arm has settled, call `robot_get_paddle_target`
   again to re-detect tag 3 and re-read the target. The read refuses while
   joints move, and it needs tags 1 and 2 in view as well. If the paddle moved,
   the arm is near contact, or a read refuses, use `robot_halt_motion` (or
   `robot_stop`) and stop moving toward the target.
5. Stop at the pre-grasp. Do the final alignment, closing, lift, hold, place,
   open, withdraw and release with the camera-guided `paddle-success-v1` steps.

## Evidence and interruption

Each run gets a new `tag-calibration-runs/` directory in the pilot, containing
initial observation, per-frame annotated images and encoder brackets, command
trace and either `result.json` or `failure.json`. Registration also saves
`plan.json`, eleven pose records and the assembled dataset. Earlier evidence
is never overwritten. `calibration_commands_sent` counts movement requests;
`commanded_path_ticks` counts requested travel, not measured motor writes.
The fitter's zero-write count refers only to the mathematical solve.

The adapter checks fresh all-motor readbacks, current-owner identity, controller
phase, unchanged ranges, requested/completed command identity, measured endpoint
error, uncommanded drift, camera stream/sequence/time and fixed table-tag corners.
It enables all six motors of the selected arm in one call, because today's
pickup-profile owner (`paddle-success-v1`) refuses any move otherwise; the jaw
is held where it is and never commanded. It never enables the head, other arm or
wheels. Those other encoders must remain stationary. Each step moves one joint
3..16 ticks (the owner treats a target within 2 ticks as a no-op) with
`duration_s` 0.4.

Successful motor responses must confirm completion from the bound owner;
acceptance alone is insufficient. A response with `completed: false` and a
`closure_outcome` (`settled_short`, `halted`, `contact_halt`) is a refusal. The
owner accepts endpoints within 57 ticks; the adapter still requires its own 5.
Enable/move leaves the owner `holding` and release leaves it `idle`. Normal
completion releases the held motors and verifies fresh all-sixteen torque-zero
readback. Failure after an enable attempt requests independent STOP; the owner
eases torque off over about 2 s, and the adapter keeps reading state until
torque-zero is confirmed (`cleanup.release_confirmed`). The owner has no STOP
latch, so a STOP or owner fault shows up as a changed `stop_count` and an idle,
released owner, which aborts the run. A failed preflight sends no STOP to
another client's owner. There is no automatic retry, reset, restart or blind
return movement following a missing tag or refusal.

## Link timing

Owner rows must be 0..0.75 s old and camera frames 0..1.0 s old on the chat
Mac's clock. One status read uses `robot_get_execution`, which carries the
owner's own 16 telemetry rows, and only two relay round trips separate a camera
frame from the next command. A registration makes about 530 relay calls: about
125 s at a 150 ms round trip against the default 180 s budget. Measure the link
before a run (read-only; no motor, camera or state change):

```bash
cd software && PYTHONPATH=. python tools/measure_robot_link.py --pilot-root /path/to/gemma/pilot
```

It reports round trip, robot-minus-chat clock offset (within half a round trip)
and the projected registration time and freshness margins. A chat clock behind
the robot makes owner timestamps look like the future and is refused. Fix the
clocks (`sudo sntp -sS time.apple.com` on both Macs) or the link; do not relax
the freshness limits. `limits.max_seconds` (at most 300) may be raised in
`tag-calibration.json` for a slower but in-limit link.
`qwen-bridge/test_tag_registration_contract.py` runs the whole registration
against the real robot-server code with a fake bus, in the restart script's
test step.

The installed wrapper and CLI share a local motion lock. Independent STOP and
read tools bypass it. Owner command/write counters also detect outside activity,
but this is not an atomic lease over arbitrary clients on another computer.
Keep those clients idle during calibration; observed outside activity aborts the
run. Owner supervision and all existing hardware enforcement remain authoritative.

## Verification boundary

Unit tests use an API-level simulated owner and analytic camera/encoder data. They
exercise the existing probing algorithm, automatic sample collection, actual
OpenCV fit, release and fault handling. This is not a dynamics/collision simulation
or proof that physical motor movement, hand-eye calibration or grasping works.

The separate [rendered end-to-end validation](gemma-calibration-simulation.md)
uses actual camera images rendered from the existing SO101/paddle meshes,
the production Gemma adapters, eleven automatic poses, LeRobot FK, OpenCV
fitting and `robot_get_registered_tags`. It then derives a handle target from
tag 3 plus its declared simulated mounting offset and executes a dynamics
grasp, lift, two-second hold and release. Independent object truth is used
only for scoring. This passed for two paddle placements; an open-jaw control
correctly failed to lift. These results do not establish physical accuracy.

The live read-only check on 2026-10-06 detected table tag 1 and gripper tag 2.
Tag 2's black square was fully visible with approximately 8 px bottom clearance;
the earlier claim that it was clipped was too strong. Detection can change
between frames. The same check reported the right elbow outside its saved range.
No physical calibration movement was executed by this integration work.
