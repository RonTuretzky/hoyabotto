# Metric AprilTags and camera-to-arm commissioning

The Gemma adapter now estimates tag centres in camera millimetres using the
existing authenticated RGB snapshots, matching intrinsics and explicit printed
black-square widths. It also supplies read-only capture, candidate FK assembly
and fixed-camera hand-eye fitting. None of these utilities enables motors,
resets STOP, installs a robot transform or claims a completed grasp.

## Current physical result

On 2026-10-06 the OAK view contained IDs 1/2/3. A stationary frame was bracketed
by fresh unchanged left-arm and head encoder reads and saved as a calibration
sample. The source dimensions are user-confirmed print-kit sizes, 60/40/40 mm;
no independent ruler measurement has been supplied. The live camera is 640×360.
Camera-to-arm registration, jaw contact offset, workspace clearance and a
physical paddle grasp remain unvalidated. Repeating the same stationary pose
does not supply the missing independent calibration poses.

During the initial capture, the owner was latched after an idle coherent servo read returned `-7`
(`COMM_RX_CORRUPT`). Fault-row timestamps and source order point to the right
gripper ID 6 reading address 56, length 15. That identity is inferred, because
the original error omitted the transaction. It does not establish USB
contention. Partial-packet timeout, checksum failure and strict length rejection
all produce this code. The current repository helper now preserves the exact
motor/port/ID, request, return code and bounded reply-validation events in its
exception; it adds no retry or relaxed validation. The diagnostic hunks were
subsequently staged in the actual remote helpers and passed 23 local tests there.
Following a separate direct user request, the Gemma chat replaced the released
owner to load diagnostics; a follow-up readback showed idle and all 16 motors
released, with stability testing ongoing. This does not establish that the
communication fault is fixed. Do not resume an old goal to collect calibration
data; refresh the current owner state and use the recovery chat's evidence.

## Install into the existing Gemma pilot

Keep the working DepthAI environment unchanged. Use the local pilot environment:

```sh
uv pip install --python /path/to/gemma/.venv/bin/python \
  -r /path/to/xlerobot-farm/software/requirements-tag-geometry.txt
```

OpenCV 4.11.0.86 supplies both IPPE and `calibrateHandEye`; the examined 5.0.0.93
wheel did not expose the latter. For the optional FK assembly step use the
existing LeRobot/Placo kinematics environment and
`constraints-carton-kinematics.txt`. No custom IK solver is introduced.

Create `.private/apriltag-geometry.json` beside the pilot's existing `robot.json`:

```json
{
  "schema": 1,
  "family": "tag36h11",
  "camera_ids": ["the-camera-id-from-robot_get_cameras"],
  "tags": {
    "1": {"black_square_mm": 60, "source": "Record how this physical width was established"},
    "2": {"black_square_mm": 40, "source": "Record how this physical width was established"},
    "3": {"black_square_mm": 40, "source": "Record how this physical width was established"}
  }
}
```

These example widths are not evidence that a particular print has those sizes.
Measure the black outer square, excluding the white border. Width error scales
the recovered distance. Configuration is local; model tool arguments cannot
change it. Restart the idle local chat with its existing launcher. The remote
camera and hardware owner need no restart for this adapter update.

`robot_get_tags` remains the same tool. Each `pose_3d` includes camera optical
axes (+x right, +y down, +z forward), centres in mm, a rigid transform in metres
only when orientation is unambiguous, reprojection error and a local position
standard-deviation estimate assuming 0.5-pixel corner noise. `coarse_position_only`
flags sensitivity exceeding 2 mm on any axis. This estimate excludes print
size, camera calibration, paper warp and mounting errors; it is not an accuracy
guarantee. `gripper_to_paddle_mm` is a tag-centre vector, not a jaw motion target.

The intrinsics must be bound to the same camera ID, stream, sequence, image hash,
dimensions and projection as the pixels. Rectified RGB uses zero effective
distortion even if metadata retains original factory coefficients. Raw RGB uses
its supplied matching distortion. Unknown/mismatched calibration leaves 2D
detection available and makes metric data unavailable. OAK depth is not fused.

## Capture and fit

These commands only read the existing owner's tools or saved files:

```sh
cd /path/to/xlerobot-farm/software
PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/commission_tag_geometry.py capture \
  --pilot-root /path/to/gemma/pilot \
  --geometry /path/to/gemma/pilot/.private/apriltag-geometry.json \
  --camera oak --arm left --split train --out /path/to/new-capture-01

PYTHONPATH=. /path/to/kinematics/.venv/bin/python tools/commission_tag_geometry.py assemble \
  --captures /path/to/new-capture-01 /path/to/new-capture-02 \
  --model-directory /path/to/verified-so101-model --out /path/to/dataset.json

PYTHONPATH=. /path/to/geometry/.venv/bin/python tools/commission_tag_geometry.py fit \
  --dataset /path/to/dataset.json --out /path/to/registration.json
```

`observe` saves observations without attempting an encoder bracket. `capture`
reads fresh encoders, one camera observation, fresh encoders again and the
existing arm-geometry status. It requires unchanged joint/head positions within
three ticks, stationary fault-free telemetry, a capture timestamp between the
encoder reads within three seconds, tags 1/2 and unambiguous gripper-tag pose.
Both Macs' clocks must agree. Endpoint checks assume no other actuator writer.

Keep the camera/head, table tag and gripper-tag mounting fixed. Collect at least
eight spatially and rotationally diverse training poses and three independently
chosen held-out poses with `--split validation`. Rotate around at least two axes;
a pan sweep alone cannot identify both transforms. This tool does not move the
arm to obtain those poses. Only resume pose collection through the sole owner
after its fault has been diagnosed and its existing readiness checks pass.

Assembly reuses the verified SO-101 assets, upstream LeRobot FK and upstream
Feetech DEGREES normalization. Range midpoints are only a mapping candidate,
not a physical measurement of URDF zero. Model/calibration identities and ranges
must match the capture. This is an offline dataset, never a motor calibration
rewrite. The fitter solves `B_G @ G_T = B_C @ C_T`; `B_C` maps camera to arm base
and `G_T` maps tag to model gripper. It rejects reused frames, changed hashes,
head/table movement, anchor corner shifts over 2 px and degenerate pose sets.

Training and held-out sets must each meet <=2 mm position RMS, <=4 mm maximum
position residual and <=2 degrees maximum orientation residual. Failed fits
return no transforms. Accepted fits still have `motion_ready:false`: establish
the physical jaw contact offset and workspace/path clearance before using
camera-derived grasp targets. Revalidate after any mount, camera, head or motor
calibration change. A good fit only supports the sampled configuration range.

## Verification and limitations

```sh
python -m pytest -q tests/test_tag_geometry.py tests/test_tag_sampling.py \
  tests/test_tag_registration.py tests/test_gemma_tags.py

PYTHONPATH=. /path/to/gemma/.venv/bin/python tools/verify_gemma_tag_geometry.py \
  --pilot-root /path/to/gemma/pilot --out /path/to/new-gemma-probe.json
```

The live Gemma probe exposes only `robot_get_tags`, permits one OAK observation,
and preserves the user's chat history. It saves the actual model response and
tool result. A stale frame fails the probe, with no motor command or automatic
action retry.

The simulator's `--metric --resolution 640|1280|1920` compares 138 rendered tag
positions against independent MuJoCo coordinates. The October 6 runs measured
maximum errors of 6.72, 5.05 and 4.73 mm respectively; only 1920×1440 met the
unchanged 5 mm acceptance target. All three detected tags in 46/46 nominal
frames and passed the existing negative controls. This does not validate the
current 640×360 physical view, tag prints or robot alignment. Improve pixel
coverage with a suitable higher-resolution stream, closer view or larger tag,
then measure real accuracy before depending on millimetre-scale clearance.

See [evidence](evidence/gemma-tag-geometry.json) and the
[rendered-camera guide](gemma-apriltag-simulation.md). Methods reuse OpenCV's
[square-marker pose solver](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)
and [hand-eye calibration](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
