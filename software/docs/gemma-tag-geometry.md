# Metric AprilTags and camera-to-arm commissioning

The Gemma adapter now estimates tag centres in camera millimetres using the
existing authenticated RGB snapshots, matching intrinsics and explicit printed
black-square widths. It also supplies read-only capture, candidate FK assembly
and fixed-camera hand-eye fitting. None of these utilities enables motors,
resets STOP, installs a robot transform or claims a completed grasp.

## Current physical result

The user confirmed on 2026-10-06 that tag 2 is attached to the **right arm's
fixed gripper housing**. Earlier captures paired it with left-arm encoders.
Those captures and their assembled FK datasets are invalid for registration;
there are currently **zero accepted independent right-arm calibration poses**.
The earlier camera-only observations of IDs 1/2/3 remain valid as observations.
The source dimensions are user-confirmed print-kit sizes, 60/40/40 mm;
no independent ruler measurement has been supplied. The live camera is 640×360.
Camera-to-arm registration, jaw contact offset, workspace clearance and a
physical paddle grasp remain unvalidated. Repeating the same stationary pose
does not supply the missing independent calibration poses. A corrected right-arm
capture was rejected because tag 2 was no longer detected at the lower image
boundary. Fresh right-elbow readback was 3155 ticks against the recorded
1002..3092 range; the provenance/recovery audit is separate from hand-eye fitting.
The right-arm geometry configuration is also absent. These are concrete starting
conditions to resolve, not reasons to require manual demonstration of every pose.

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
    "2": {
      "black_square_mm": 40,
      "source": "Record how this physical width was established",
      "mount": {
        "arm": "right",
        "body": "fixed_gripper_housing",
        "source": "User confirmed right fixed gripper housing on 2026-10-06"
      }
    },
    "3": {"black_square_mm": 40, "source": "Record how this physical width was established"}
  }
}
```

These example widths are not evidence that a particular print has those sizes.
Measure the black outer square, excluding the white border. Width error scales
the recovered distance. Configuration is local; model tool arguments cannot
change it. Restart the idle local chat with its existing launcher. The remote
camera and hardware owner need no restart for this adapter update.

The explicit mount is mandatory for registration samples. It is included in
the configuration fingerprint and carried into each sample and dataset. A
left-arm capture of this right-mounted marker, a moving-jaw marker or an old
sample without mount provenance is rejected. Do not relabel old left-arm
samples as right-arm samples: their actual encoder readings are different.

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
  --camera oak --arm right --split train --out /path/to/new-capture-01

PYTHONPATH=. /path/to/kinematics/.venv/bin/python tools/commission_tag_geometry.py assemble \
  --captures /path/to/new-capture-01 /path/to/new-capture-02 \
  --model-directory /path/to/verified-so101-model --out /path/to/dataset.json

PYTHONPATH=. /path/to/geometry/.venv/bin/python tools/commission_tag_geometry.py fit \
  --dataset /path/to/dataset.json --out /path/to/registration.json
```

`observe` saves observations without attempting an encoder bracket. `capture`
reads fresh encoders, one camera observation, fresh encoders again and the
existing arm-geometry status. `--arm` has no default. It requires unchanged joint/head positions within
three ticks, stationary fault-free telemetry, a capture timestamp between the
encoder reads within three seconds, tags 1/2 and unambiguous gripper-tag pose.
Both Macs' clocks must agree. Endpoint checks assume no other actuator writer.

Keep the camera/head, table tag and gripper-tag mounting fixed. Collect at least
eight spatially and rotationally diverse training poses and three independently
chosen held-out poses with `--split validation`. Rotate around at least two axes;
a pan sweep alone cannot identify both transforms. This tool does not move the
arm to obtain those poses. Only resume pose collection through the sole owner
after its fault has been diagnosed and its existing readiness checks pass.

## Automatic move-and-observe workflow

The pose list should be collected by the robot once the starting scene and
controller are usable. No teleoperation training dataset is required for this
geometric calibration. Reuse the following existing implementations:

1. `carton/servo/controller.py:Experiment.calibrate` already takes stationary
   observations, probes selected joints in both directions, checks actual
   encoder response, returns to the measured starting pose and validates a
   local image-motion model with separate half-size movements. This model is
   useful for centering and local alignment; it does not establish 3D registration.
2. `stationary_sample`, `assemble_dataset` and `fit_registration` already collect
   synchronized encoder/tag evidence and solve both the camera-to-base and
   tag-to-gripper transforms. Collect diverse orientations about at least two
   axes plus position changes, retaining at least eight fitting and three
   independent validation poses. Start with small observable movements and
   expand only within the established workspace; a pan-only sweep is insufficient.
3. The original `Experiment` transport uses a command/status-file owner and its
   observer uses seeded camera views. They are **not yet adapters for the current
   authenticated Gemma owner and single OAK tag observation**. Implement that
   bridge through the existing sole owner, preserving session identity,
   freshness, measured settling, travel bounds and STOP handling. Do not start
   the old serial owner or run the old CLI against the current hardware setup.

For this installation, first restore full tag-2 visibility, finish the right
elbow range audit, and bind the right-arm candidate kinematics. Then collect
the motion samples automatically and evaluate the existing independent-fit
thresholds. A camera adjustment before collection is fine; it invalidates
previous camera registration and must remain fixed during collection.

This is an established method: [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
combines a robot model, tracking and MoveIt-driven sample movements to estimate
hand-eye transforms. Its ROS/MoveIt driver is not a drop-in replacement for our
owner. Our fitter reuses OpenCV's solver instead of introducing a second robot
control stack.

The fitted tag-to-gripper transform removes the need to measure tag 2's exact
mount offset. It does **not** identify the physical jaw contact point or which
part of the paddle is graspable. Use the actual gripper/paddle CAD for initial
contact geometry, then verify the physical fit under observation. An ideal CAD
site or rendered tag mount is a candidate, not a measured hardware offset.
If contact features are occluded, a specific physical check may still be needed;
that is separate from manually teaching all calibration poses.

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
