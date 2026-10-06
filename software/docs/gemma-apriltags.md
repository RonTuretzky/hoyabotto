# AprilTags in Gemma's robot tools

`farm.perception.gemma_tags.TagRobot` adds `robot_get_tags` to the existing Gemma
pilot's dynamic catalog. It reuses `robot_get_cameras`, authenticated robot
transport and `farm.perception.tags.detect_tags`. It does not open a camera,
construct a motor owner, change movement tools or infer a grasp.

The initial deployment runs detection on the **Gemma Mac**, using the robot
Mac's existing OAK/phone snapshots. This integrates with the current server
without interrupting its hardware owner. `TagObserver.observe` can also be
called on the robot server with the same camera response. If a future server
advertises native `robot_get_tags`, the client prefers it without duplication.

## What Gemma receives

```json
{"name":"robot_get_tags","arguments":{"cameras":["oak","phone"],"tag_ids":[1,2,3]}}
```

The response contains structured `result.observations`, plus top-level `images`
in the format already displayed by the chat and passed to Gemma's vision input.
Annotations use high-contrast outlines and text labels. `include_images:false`
omits annotations when only measurements are needed.

- IDs, centres and four corners in **each image's pixels**, dimensions and quality.
- Marker-kit roles: 1 table anchor, 2 gripper, 3 paddle. This is a configured
  identity mapping; it must match the actual physical mounting.
- Frame hash, sequence, stream, capture/receipt timestamp and observation ID.
- Explicit missing IDs and per-camera `UNKNOWN` errors. No remembered position
  is returned in place of a missing detection.
- `gripper_to_paddle_px` only when IDs 2 and 3 are accepted **in the same frame**.
  Positive x is image-right; positive y is image-down. These are not motor axes.
- `pose_3d:null`, `metric_pose_available:false`, `depth_used:false` and
  `physical_task_completed:false`.

OAK RGB capture time is preferred over depth time. Phone frames retain
`captured_at:null` and `receipt_only_capture_delay_unknown`. Age is calculated on
the observer's wall clock; synchronize both Macs' clocks. The adapter reports
`clock_synchronization_verified:false`: it cannot establish clock agreement
from images. It rejects ages over 2 seconds or future timestamps over 0.25
seconds, including detector latency, and never treats recent phone receipt as
verified capture timing.

Pixels must match their SHA-256 and accompanying metadata. Duplicate IDs/views,
regressed sequences, altered contents under the same sequence, stale frames and
detector failures are unknown. Accepted tags require hamming 0, decision margin
>=30 and shortest edge >=24 px. These are not calibrated probabilities or
millimetre error bounds.

## Using the observations while piloting

Gemma can read tags, choose a movement through its current direct-joint tools,
inspect measured encoder results and request another tag observation. Each
tag call performs one camera-tool read. The existing owner handles motor
timing and STOP. Missing tags do not impose a new restriction on unrelated
manual joint control; they prevent establishing tag-based alignment in that view.

This observation tool has no fixed-head assumption or 100 px seeded envelope.
Moving the head changes pixel coordinates; do not interpret that change as
object motion without accounting for the camera. The carton controller's
seeded tracker, goal and Jacobian remain specific to a local experiment and
must be re-established when its setup changes.

A tag centre is not the jaw contact point. Metric camera-relative pose needs
measured tag size and matching intrinsics; robot-relative reaching additionally
needs camera/arm registration, tag/contact offsets and task validation. This
adapter does not fuse OAK depth or combine pixels across views.

## Install into the existing pilot

Use the **pilot virtual environment**, not the working DepthAI environment:

```sh
uv pip install --python /path/to/gemma/.venv/bin/python \
  -r /path/to/xlerobot-farm/software/requirements-carton-vision.txt
/path/to/gemma/.venv/bin/python /path/to/xlerobot-farm/software/tools/install_gemma_tags.py \
  --pilot /path/to/gemma/pilot
```

The installer preserves existing chat/history/control code. It adds an import
and wraps `Robot` in `TagRobot`, saves the previous source under the pilot's
`.private/tag-adapter-backups/`, and adds the repository software directory to
that environment with a `.pth` file. It refuses an unfamiliar source layout.
Keep the checkout at that path; rerun after pilot updates that remove the hooks.
No TLS secrets are copied.

Restart the **idle local chat server** with its existing launcher. No robot owner
restart is needed. The chat's `/api/tools` should contain `robot_get_tags` beside
the current server tools. A disconnected robot supplies no camera-backed tools.

Suggested first Gemma request:

> Use robot_get_tags and report visible and missing tags for each camera.
> Only inspect; do not move or enable motors.

## Verification

```sh
cd software
python -m pytest -q tests/test_gemma_tags.py tests/test_carton_tags.py
```

Tests exercise the real detector on generated markers in actual camera-tool
envelopes. They cover discovery/dispatch, native-tool precedence, quality,
hash/timing/identity rejection, missing/duplicate tags, annotations, receipt-only
timing and argument rejection before upstream calls. Run the existing pilot's
`test_chat` and `test_gateway` after installation.

Deployment evidence is in `docs/evidence/gemma-apriltags.json`, separate from
physical motion or grasp validation. For print dimensions and placement see
[the tag guide](carton-apriltags.md).

For the rendered-camera motion test, negative controls and actual Gemma tool
use in MuJoCo, see [simulator validation](gemma-apriltag-simulation.md).
