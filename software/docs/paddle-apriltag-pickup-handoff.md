# AprilTag paddle pickup: robot commissioning handoff

Prepared 2026-10-06. Goal: use the **right arm** to locate the tagged paddle,
approach its handle, close the jaw, make a small observed lift, place it back,
and release. This is a supervised commissioning test, not a carton-folding run.

## What is ready, and what is not

The repository contains AprilTag observation, metric pose estimation, automatic
camera/arm registration, registered observations, bounded visual-servo building
blocks, and a grasp-evidence checker. The authenticated Gemma adapters reuse the
existing motor owner and camera feed. A rendered end-to-end registration and
pickup passed in simulation; **no physical tag-guided pickup is verified**.

There is **no single production `pickup --execute` command**. In particular,
`GemmaTransport` deliberately excludes jaw control, and the `carton.servo`
CLI's `align` command uses the older file-session transport. Do not assume
that command controls the current HTTPS owner. The robot operator/agent must
inspect the current direct-joint API and commission approach, jaw closure and
lift through that owner. If the required motion primitive or validated joint
mapping is missing, report that specific implementation gap before contact.

Source baseline on GitHub main:
`d46e6ed14073c6e3c934feb5a988f842613844c7`.
The companion [inventory](evidence/paddle-apriltag-handoff.json) records the
required source files and hashes, tested from a clean clone of that baseline.
The new handoff commit should be fetched as well.

## Current hardware evidence

- [Right-arm calibration](evidence/right-arm-calibration-2026-10-06.json): the
  accepted calibration was installed on the robot and all six entries matched
  hardware readback. Non-right entries were preserved. The wider shoulder-pan
  travel was accepted by the owner **by assumption**, not independently checked
  over its full range. Do not sweep that full range for this pickup.
- [Left resting pose](evidence/left-arm-resting-pose-2026-10-06.json): the left
  arm was placed in a compact resting pose. Its calibration remains mixed;
  leave it out of this right-arm test.
- All sixteen motors were verified released at the end of that calibration
  session. This is historical evidence, not a live torque-state assertion.
- At handoff preparation, `robot main thread` on Neooooo.local was executing a
  separately requested cart repositioning. Wait for that work to finish. Read
  fresh owner state and images; do not interrupt it or open another motor owner.
  After repositioning, recollect or revalidate camera/arm registration and the
  table anchor against the new scene. Do not reuse earlier local pixel models
  or table-relative plans unchanged.

Motor calibration and camera/arm registration are different. New motor
calibration also requires a matching arm-geometry binding, including current
calibration bytes, raw ranges, joint zeros/signs and model identity. Do not
make an old registration appear current by changing only its hash.

## Where to run it

The existing architecture runs the tag adapters on the **Gemma Mac**, using
the robot Mac's authenticated owner and published images. A repo checkout on
the robot Mac does not itself install or authenticate a Gemma client.

Known locations, to verify before use:

| Item | Last known location |
| --- | --- |
| Robot checkout | `/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm` on Neooooo.local |
| Existing Gemma pilot | `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot` on the development Mac |
| Pilot environment | `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/.venv` on the development Mac; verify the actual interpreter |
| Required model assets | Host-local verified SO101 directory selected in `tag-calibration.json`; retrieve with `python -m carton.servo model-fetch --out /new/model/directory` if absent |

Fetch current main and inspect local changes before updating either working
checkout. Use a separate clean checkout if another task is editing it; never
reset that task's files. The model fetcher checks the upstream revision and
file hashes in `farm/kinematics/so101-assets.json`.

Already installed runtime prerequisites are **not all stored in this repo**:
the Gemma pilot's `chat_server.py`/gateway, the running HTTPS motor owner, its
authentication and camera services. Reuse the existing installation. The
private `robot.json`, camera configuration, `apriltag-geometry.json`,
`tag-calibration.json`, arm-geometry binding and any accepted
`tag-registration.json` must be inspected on their owning host. Do not publish
credentials or copy another Mac's absolute paths into configuration.

For missing adapter dependencies, use the **complete metric/calibration set**
in the existing pilot environment:

```sh
uv pip install --python /actual/gemma/python \
  -r /actual/repo/software/requirements-gemma-calibration.txt
```

Do not install `requirements-carton-vision.txt` afterward: its OpenCV 5 pin is
for detector-only use; this path requires the tested OpenCV 4.11 hand-eye solver.
Leave the working DepthAI camera environment unchanged. If hooks are missing,
follow [tag installation](gemma-apriltags.md) and
[calibration installation](gemma-automatic-calibration.md), using the complete
dependency set above. The CLI below can reuse the pilot without restarting its
chat; installers must not restart the motor owner or camera.

## First test: read-only readiness

From the selected checkout's `software` directory, with actual host paths:

```sh
PYTHONPATH=. /actual/gemma/python tools/calibrate_gemma_tags.py status \
  --pilot-root /actual/gemma/pilot
PYTHONPATH=. /actual/gemma/python tools/calibrate_gemma_tags.py registered \
  --pilot-root /actual/gemma/pilot
```

The second command may correctly refuse if there is no current registration.
Neither command enables motors. Save fresh state and annotated images through
the existing tools, including:

```json
{"name":"robot_get_tags","arguments":{"cameras":["oak"],"tag_ids":[1,2,3]}}
```

Confirm IDs **1 = table, 2 = right fixed gripper housing, 3 = paddle**, full
black-square visibility, current capture timing, correct resolution/intrinsics,
and no stale image reuse. Tag size means the measured black square excluding
the white border. The 60/40/40 mm print-kit sizes were user-confirmed defaults,
not an independent measurement of the actual scaled print. Record provenance
and resolve this before treating estimated millimetres as measured distances.
RGB AprilTag pose does not require OAK aligned depth; this integration reports
`depth_used:false`. Missing or ambiguous observations are not grasp targets.

## Registration and approach commissioning

Use an existing registration only if its bindings and independent current-scene
checks pass. Otherwise use [automatic registration](gemma-automatic-calibration.md)
after the robot task has exclusive use of the owner, the operator is present,
and the proposed local poses are visibly clear. The status result exposes the
grid. Calibration uses two chosen positioning joints; it is not a collision
planner. Do not relax limits to force a rejected run to pass.

The explicit registration command, **only during that supervised phase**, is:

```sh
PYTHONPATH=. /actual/gemma/python tools/calibrate_gemma_tags.py registration \
  --pilot-root /actual/gemma/pilot --execute
```

It collects eight fit and three held-out poses, returns and releases. A passing
fit remains `motion_ready:false`: it permits checked coordinate estimates,
not automatic grasp execution. Re-read `registered` afterward. A local pixel
model is a separate optional commissioning tool, not a substitute for height,
orientation, clearance or contact geometry.

Before approach, establish the physical tag-2-to-jaw contact transform and
tag-3-to-handle transform, grasp orientation, reachable approach and table
clearance. Reuse the [paddle CAD](../parts/carton/paddle_flap.stl) where it matches
the actual part, and validate the actual tag placement. The simulated
`[105, 0, 4.45]` mm offset is specific to the rendered mount and must not be
copied as a physical measurement. An ambiguous paddle orientation cannot be
combined with a handle offset. Do not equate tag centres with contact points.

Use current owner catalog schemas, existing bounds and fresh encoder readback
for short, observed right-arm steps. Observe after each step. Do not replay
simulation joint trajectories or use wheels/head/left-arm motion to make this
test pass. Do not widen controller limits or reintroduce temperature checks.

## Physical pickup acceptance

1. Reach a collision-clear pregrasp, then the measured handle grasp pose with
   the jaw open. Save current images and encoder positions.
2. Close in bounded increments using the owner's existing jaw command. A jaw
   that stops short is possible contact, not proof of a held paddle.
3. Make the smallest useful supervised lift, with its clearance and endpoint
   chosen from the actual scene. Observe the paddle and gripper independently.
   Require paddle clearance above the table and a stable relative pose during
   a short hold. If the tag is occluded, obtain independent visible evidence;
   do not substitute a commanded lift for an observed one.
4. Lower, place, open, withdraw, and verify the owner's release/readbacks.
   On failure use the existing owner's stop/recovery procedure; support/place
   a held object before ordinary torque release. Do not leave a suspended load
   based on a software success flag.

`python -m carton.servo verify-grasp --evidence ... --out ...` evaluates recorded
2D co-motion and jaw aperture. It cannot distinguish a slide from a vertical
lift and cannot establish table clearance. Its result is supporting evidence,
not the sole physical pass criterion. New evidence must contain actual tracked
points and measured empty-jaw/aperture information, never planned stage labels.

Save a new local run directory with source SHA, calibration/model identities,
owner session, intrinsics, tag sizes and offsets, timestamped images, tag
observations, encoder/command/readback records, lift/hold/place evidence and
final release status. Report the last completed stage and exact failure. Keep
private endpoint/authentication data out of any published summary.

## Verification of this handoff

A fresh clone of the source baseline passed **160 tests** across tag detection,
Gemma adapters, geometry, sampling, registration and registered observations:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python -m pytest -p no:cacheprovider -q \
  tests/test_gemma_tags.py tests/test_carton_tags.py \
  tests/test_gemma_calibration.py tests/test_registered_tags.py \
  tests/test_tag_geometry.py tests/test_tag_sampling.py tests/test_tag_registration.py
```

These are software/fixture tests. No live camera query, calibration motion,
gripper closure or robot pickup was performed while preparing this handoff.
Earlier [rendered pickup evidence](gemma-calibration-simulation.md) remains
separate; its external simulation directory is not required for this physical
commissioning handoff and is not bundled by it.

## Prompt to give the robot/Gemma task

> Read `software/docs/paddle-apriltag-pickup-handoff.md` on current GitHub main
> in RonTuretzky/xlerobot-farm, including its source inventory. Prepare one
> supervised right-arm AprilTag paddle pickup test. First finish any existing
> authorized cart movement and obtain fresh owner state and images. Reuse the
> existing authenticated Gemma pilot, motor owner and camera; do not start
> competing hardware clients. Check the newly installed right-arm calibration,
> current tag visibility, camera/arm registration and actual jaw/handle offsets.
> Run the documented read-only status checks first and report their evidence.
> The handoff does not authorize new movement: use the current robot task's
> explicit operator authorization before calibration or pickup. If authorized
> and the documented prerequisites pass, commission approach, bounded closure,
> a small observed lift/hold, then placement/release through current owner tools.
> A passing registration or simulator run is not physical grasp proof. Report
> completed stages, object-following and table-clearance evidence, any exact
> missing primitive or geometry, and final release state. Do not proceed to
> carton folding, alter servo calibration/limits or replay old trajectories.
