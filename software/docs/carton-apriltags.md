# AprilTags for carton tracking

This integrates printed markers into the existing carton visual controller. It
reuses `farm.perception.tags` (pupil-apriltags) and `TagTracker`; there is no second
detector, motor owner, learned policy or DepthAI dependency. The OAK-D Lite is
optional: the existing head and wrist RGB cameras can read these markers.

The software and rendered print sheet are tested. **Physical mounting, visibility
on the robot, motor calibration and a successful grasp remain unverified.** A tag
check can run with the motor session stopped, including while the other Mac is
diagnosing an elbow/power problem. It neither reads the motor session nor sends
commands. Do not run it alongside a moving experiment: hold the scene stationary
using the station's established support/stop procedure first.

## 1. Print and mount

The [ready-to-print A4 sheet](assets/carton-apriltags.pdf) contains the default
markers below. It was rendered and decoded with the actual shared detector;
the [software validation record](evidence/carton-apriltag-check.json) distinguishes
that result from the still-pending paper/mount/camera test.

From this checkout's `software/`, using its existing vision environment:

```sh
python -m carton.servo tag-kit --out data-carton/tags-01
```

Open `data-carton/tags-01/print.html` and print at **100% / Actual Size**, with
headers/footers and Fit to Page disabled. Individual SVGs retain physical units;
PNGs are detector fixtures and have no guaranteed printer scale. The HTML and
optional PDF have a 100 mm ruler: measure it on paper before cutting.

| ID / role | Black square | Including white margin | Mount |
|---|---:|---:|---|
| 1 / anchor | 60 mm | 75 mm | Fixed table/backboard visible to the head camera |
| 2 / tool | 40 mm | 50 mm | Rigid gripper body; visible from both cameras |
| 3 / target | 40 mm | 50 mm | Paddle away from the grip/contact area; visible from both cameras |

These sizes are starting points, not verified fit. Use `--tool-mm`, `--target-mm`
and `--anchor-mm` for 20–80 mm black squares. Preserve the white margin, use flat
matte backing, avoid shiny tape on the pattern, and keep unused copies out of
view. Never put the same ID on two distinct objects. Do not span a moving jaw,
joint or carton fold. A tag that disappears behind the hand requires a different
mount or camera view; a depth camera does not make a hidden marker visible.

The default placement assumes one gripper marker and one paddle marker can each
be seen in both views. If that is impossible, a different face may have its own
distinct ID and be configured separately with `seed --tag`. It is a different
tracking point: review the new visual goal and recalibrate. Do not duplicate an
ID on multiple faces or treat different tag centers as the same physical point.

Optional vector PDF export (reportlab is a print-only dependency):

```sh
python tools/carton_tags_pdf.py data-carton/tags-01/kit.json data-carton/tags-01/print.pdf
```

The A4 PDF supports the default sizes or smaller. Print larger sizes from the
individual SVGs. Tag patterns come from OpenCV's `DICT_APRILTAG_36h11`, and
`tag-kit` must successfully decode every generated marker with the controller's
actual detector before writing the kit. See [OpenCV marker generation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
and the [AprilRobotics reference](https://github.com/AprilRobotics/apriltag).

## 2. Check cameras and markers without motors

Use the **existing** coherent head/wrist publisher and its actual frame directory.
Do not open another camera reader. The publisher setup is in
[the controller guide](carton-visual-controller.md#device-ownership-and-camera-commissioning).

```sh
python -m carton.servo tag-check --frames /absolute/path/to/existing/frame-directory \
  --arm right --seconds 20 --out data-carton/tag-visibility-01
```

This requires IDs 1, 2, 3 in the head view and 2, 3 in the wrist view. It uses the
first valid detection of each marker as its temporary reference and reports
`UNSEEDED_TAG_VISIBILITY`. It does **not** seed a controller or establish a grasp.

Pass means every checked pair satisfies:

- The actual shared detector returns unique IDs, hamming 0, decision margin >=30.
- Every required marker's shortest edge is >=24 pixels (the default). The old
  8 pixel tracker minimum is a hard floor, not a commissioning target.
- Marker corners stay inside the existing 100 pixel local envelope; fixed head
  anchor corners stay within 2 pixels of their reference.
- Distinct camera identities, unchanged stream identities, valid image hashes,
  advancing sequences/timestamps, frame age <=1 s and pair skew <=0.3 s.
- At least 4 checked pairs per second and no capture gap greater than 0.5 s.

Each pair produces untouched decoded-pixel PNGs, separate labelled JPEGs and a
JSONL trace with IDs, corners, margins, sizes, timestamps, stream/sequence IDs,
file hashes and explicit failure reasons. `result.json` summarizes the run.
Failure images are retained, including a missing marker; no stale point replaces
a lost detection. Detection failures may recover during this read-only audit,
but **any failed pair makes the whole audit fail**. Camera contract failures end
the audit immediately. Exit code 2 means failed/refused; 0 means passed.

Fix the actual image problem and rerun in a fresh output directory. For small
markers, increase printed size or move/reorient the camera; for blur/glare or
occlusion, correct lighting/exposure/mounting. An anchor failure can mean camera
movement or noisy detection. Do not lower thresholds to disguise it. Run the
existing `camera-check` separately when delivery timing fails.

## 3. Seed a fresh controller experiment

After mounting is stable, run `prepare` from the controller guide with the real
session, frame directory and saved motor calibration paths. Use a **new** output
directory (here `data-carton/paddle-tags-01`); do not overwrite the old patch
experiment or its model. `prepare` reads files/images and does not open motors.

```sh
python -m carton.servo seed --config data-carton/paddle-tags-01/experiment.json \
  --camera head --tag anchor:1 --tag tool:2 --tag target:3
python -m carton.servo seed --config data-carton/paddle-tags-01/experiment.json \
  --camera right_wrist --tag tool:2 --tag target:3
python -m carton.servo tag-check --config data-carton/paddle-tags-01/experiment.json \
  --seconds 20 --out data-carton/tag-tracking-01
```

The seeded check uses the experiment's exact references, IDs and points, reporting
`SEEDED_TAG_TRACKING`; it still does not require a running motor session. All
regions must be AprilTags for this command. Existing mixed patch/tag controller
configs remain supported by the normal `inspect` command.

New tag seeds store `min_edge_px: 24` in the experiment, so the same minimum is
enforced during actual tracking, not just in the audit. `seed --min-tag-edge-px`
can deliberately choose another 8–200 pixel minimum; `tag-check --min-edge-px`
only tightens the stored requirement and never rewrites the experiment.

**Set and review a new visual approach goal.** `prepare`'s `[0,0,0,0]` is a
placeholder. Gripper/paddle tag centers are offset from their physical contact
points; zero image error is not automatically a usable grasp. The existing
`seed --point` only accepts a point inside the tag quadrilateral. Use the desired
head/wrist target-minus-tool offsets with `seed --target` after reviewing the
actual geometry. Do not copy offsets or a Jacobian from the old patch experiment.
Changes to references, IDs or tag size constraints invalidate that model binding.

Only after motor health, holding stability and clearance are independently
established should the robot operator use the existing `inspect`, bounded
`calibrate`, shadow `align`, and then explicit `align --execute` sequence. Tags
solve visual identity; they do not fix the separate elbow startup issue, establish
collision clearance, or demonstrate a fold. This integration remains 2D tracking;
metric 3D pose needs camera calibration, measured tag size and robot/contact
registration, which are outside this change.

## Dependencies and transferring the change

The carton utility environment already includes NumPy, OpenCV and pupil-apriltags.
No reinstall is necessary if the import/tests pass. If missing, make an isolated
vision environment using `requirements-carton-vision.txt`; do not upgrade the
live robot environment or the working DepthAI 2.33 OAK environment. `reportlab`
is only needed to regenerate the optional PDF, not to track tags.

Use the existing `codex/carton-visual-controller` checkout. On the other Mac,
preserve uncommitted controller/reacquisition changes before applying this
commit. This change adds commissioning modules and touches tag seeding and tag
edge validation; it does not replace `vision.py`, `controller.py` or motor code.

```sh
python -m pytest -q tests/test_carton_tags.py tests/test_carton_servo.py tests/test_carton_upstream.py
```

Tests use the real pupil detector and synthetic camera manifests. They cover
marker identity, perspective movement, missing/wrong/duplicate tags, minimum
size, anchor movement, raw failure-frame retention, restart/stale/hash/sequence
failures, and no motor transport construction. These tests are not physical
commissioning. Keep captures under ignored `data-carton/`; commit only sanitized
evidence. A fold/grasp remains `physical_task_completed: false` in tag reports.
