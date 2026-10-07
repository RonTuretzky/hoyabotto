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

Both execution modes are explicit tool calls for the current user-requested
calibration. Merely constructing the adapter, opening the chat, reading status,
or restarting the local chat cannot start or resume a calibration.

The default registration grid uses exactly two configured positioning axes,
96 ticks to either side of the observed start and 16-tick observed transitions.
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
