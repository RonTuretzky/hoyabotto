# Existing work and the carton controller decision

Reviewed October 4, 2026. This is a targeted source review, not a claim to have
tested every community project. We screened the official software guide and
its community catalogue (9 papers and 34 projects), then inspected the code
and documentation most relevant to control without teleoperation demonstrations.
Downloaded source snapshots remain outside this repository; nothing from those
projects was connected to our robot or executed against hardware.

## What already exists

| Existing work | Evidence inspected | Decision for this carton task |
|---|---|---|
| [XLeRobot vision following](https://github.com/Vector-Wangel/XLeRobot/blob/749abc837d5d771f26aeff961009c290e574b024/software/examples/3_so100_yolo_ee_follow.py) | YOLO target centre, manually tuned `K_pan`/`K_y`, planar IK, a local P-control loop, and manually edited joint calibration coefficients. Other object-control variants are present. | Reuse the architecture of local numerical feedback. Our measured image Jacobian replaces guessed per-camera gains. Image centring is not a verified grasp. |
| [Official LLM/RoboCrew guide](https://xlerobot.readthedocs.io/en/latest/software/getting_started/LLM_agent.html#activate-arm-manipulation) | The agent invokes an arm-manipulation tool backed by a previously trained policy; the guide explicitly requires training that policy. | An LLM orchestrator does not supply the missing physical skill. Let Astra select targets and interpret failures; keep individual corrections local. |
| [LeRobot kinematics](https://github.com/huggingface/lerobot/blob/8c920c4270460851cedd2737657584586d3dc66f/src/lerobot/model/kinematics.py) and [processors](https://github.com/huggingface/lerobot/blob/8c920c4270460851cedd2737657584586d3dc66f/src/lerobot/robots/so_follower/robot_kinematic_processor.py) | `RobotKinematics` uses the robot URDF with Placo; its public joint inputs and outputs are degrees. Processor steps support Cartesian bounds and joint conversion. | **Integrated:** the shared adapter calls LeRobot 0.6.1 for FK and checked IK proposals. Physical use still requires measured model zeros and station/tool geometry. No new general IK engine and no normalized positions treated as degrees. |
| [so101-control](https://github.com/vi-vi-3482/so101-control/tree/7461e9ed741bdc607818b3f819a8ce8e75f869f5) | `control.py` wraps LeRobot kinematics and interpolated movement; its CLI selects `use_degrees=True` and `connect(calibrate=False)`. Dry-run tests and macOS/Linux support are documented. | A useful reference for programmatic motion without demonstrations. Its standalone connection cannot run beside our active motor owner. It does not supply camera registration, contact handling or carton folding. |
| [XLeRobot-Pro](https://github.com/Minko82/xlerobot-pro/tree/522f8364ac9761fc88053b97178cf1373b759b40/cube-vision) | `ik_solver/ik_so101.py` uses Pinocchio/Pink and a reduced five-joint model; `pincer_transform/conventions.py` has explicit arm/head offsets. [Website grasp instructions](https://minko82.github.io/xlerobot-pro-website/software.html) describe RealSense geometry and warn that demo bus maps differ from full-robot maps. | Geometry-based autonomous grasping is a real precedent. Do not transplant another build's offsets, motor IDs or startup movements. The inspected tree uses `cube-vision/`, while website instructions also name newer paths absent from that snapshot; pin and inspect code. |
| [XLeRobot Home Service Demo](https://github.com/xujiayuxian-png/xlerobot_home_service_demo/blob/7bcc5a4391f2b458901c4c348c9f3f760bbee733/docs/en/grasping.md) | Separate ACT, centroid and GPD backends share calibration and MoveIt pregrasp. Geometric top grasp uses segmented RGB-D geometry. [Calibration](https://github.com/xujiayuxian-png/xlerobot_home_service_demo/blob/7bcc5a4391f2b458901c4c348c9f3f760bbee733/docs/en/calibration.md) distinguishes saved samples, fitted drafts, held-out validation and active runtime. | Strongest architectural reference: calibrate, validate a hover/approach, then close and verify. Its ROS 2/MoveIt/RGB-D/GPU stack and demonstrated right-arm task are not an immediate replacement for our Mac/RGB/carton setup. |

The official [ACT guide](https://github.com/Vector-Wangel/XLeRobot/blob/749abc837d5d771f26aeff961009c290e574b024/docs/en/source/software/getting_started/VLA_ACT.md)
uses recorded demonstrations and tells builders to check dropped frames and
bandwidth. It is an alternative if demonstrations become acceptable; it is not
evidence that another station's checkpoint can operate our four-flap carton.
The [community catalogue](https://xlerobot.readthedocs.io/en/latest/relatedworks/index.html)
also lists towel folding, teleoperation, simulation and perception projects.
We did not find a demonstrated, directly reusable no-demonstration controller
for our measured carton, paddle and masking-tape setup in the reviewed sources.
That is a bounded search result, not proof that none exists anywhere.

## Why retain this small utility

The reuse is now implemented:

- **LeRobot 0.6.1 `RobotKinematics` + Placo 0.9.15:** shared FK/IK adapter with
  five explicitly ordered joints, URDF limits and independently checked residuals.
  The wrapper repeats upstream solver steps; it does not implement another IK engine.
- **TheRobotStudio's [SO-101 model](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101):**
  unchanged `so101_new_calib.urdf`, meshes, README and Apache license, fetched at
  an exact revision and checked against packaged SHA-256 hashes.
- **This repository's `farm.perception.tags`:** its existing pupil-apriltags
  detector now supplies carton tool/target/anchor tracks. Duplicate IDs and bad
  measurements refuse; the carton adapter additionally rejects corrected or weak
  decodes. Existing texture-patch configurations still work.
- **Shared encoder conversions and existing motor session:** one unit adapter
  handles raw ticks, normalized positions and measured model degrees. The motor
  owner and guarded command protocol stay the device interface.

`inspect --kinematics` adds a model-derived tool pose to the existing observation
record. `plan-reach` binds proposals to fresh images, measured encoders, the arm,
calibration, tool geometry and upstream model revision. It cannot execute a path.
The [commissioning guide](carton-visual-controller.md#reuse-of-existing-perception-and-kinematics)
contains installation and use instructions. Numerical evidence is saved in
[the upstream solver check](evidence/carton-upstream-kinematics-check.json).

The immediate gap is turning fresh observations and bounded encoder commands
into repeatable local corrections on the already connected Mac. The current
motor owner, calibration files, STOP controls and hardware limits remain the
device layer. The shared AprilTag detector and OpenCV supply tracking. The new code adds identification,
independent validation, image/command synchronization, travel budgets and
evidence records. There is no new servo driver, model training or LLM call per
correction.

LeRobot's [OpenCV camera implementation](https://github.com/huggingface/lerobot/blob/8c920c4270460851cedd2737657584586d3dc66f/src/lerobot/cameras/opencv/camera_opencv.py)
already validates FPS and FOURCC and offers a background reader. The Mac adapter
here has a specific additional purpose: atomically bind image bytes, device ID,
capture PTS and sequence for the existing file-based session. Prefer an existing
publisher if it can provide the same verified contract. Do not open both camera
owners. Requesting 5 fps or receiving BGRA does not establish USB wire format or
prove that bandwidth contention caused the observed failures.

## Scope corrected after the review

1. **Commissioning and local alignment:** the implemented utility measures a
   small joint-to-image model and corrects a nearby visual target. Its result is
   explicitly `ALIGNED_ONLY`. It is not a general motion planner.
2. **Geometric approach and folding:** the LeRobot URDF solver is integrated for
   inspection and proposals. Physical use still needs measured and independently
   validated model-zero/sign, station registration, tool dimensions and carton
   crease geometry. A small local Jacobian cannot be extrapolated across a fold.
3. **Contact skills:** separately validate jaw closure, a short test lift,
   object co-motion, flap rotation and release. The included grasp checker
   evaluates evidence; it does not command closure or invent a lift direction.
4. **Repeatability:** save achieved visual targets with calibration provenance;
   remeasure the local model before reuse. Do not replay an old joint sequence
   solely because its name says `paddle_grip` or `fold_done`.

This review supports the no-demonstration direction **as an engineering
experiment**, with calibration and physical validation. It does not establish
that Astra can independently finish all contact manipulation, or that local
alignment will be enough for folding and taping. If the first bounded physical
identification fails, fix observability, backlash, mounting or the fixture;
do not spend another long session guessing small motions in chat.
