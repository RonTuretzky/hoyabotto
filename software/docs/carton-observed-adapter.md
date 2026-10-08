# Offline PixelPort adapter

`carton/folding_observed_adapter.py` connects the existing PixelPort RGB-D
output to `ObservedSceneBuilder` without providing simulator carton qpos,
hidden flap angles, contact state, or grasp-success state. It supplies a
declared **offline** metadata envelope. It does not establish hardware
readiness or a measured physical calibration.

There is currently no authoritative measured physical station profile for this
adapter. Camera mounts, rear robot spacing, tag mounts, model errors, timing,
and uncertainty horizons remain explicit simulation assumptions. A small
AprilTag square-fit, plane-fit, or anchor-fit residual measures internal fit
consistency; it is not a camera-transform or carton-pose accuracy bound. The
adapter never substitutes those residuals for the declared transform errors.
Marker disagreement can increase an assumed bound or trigger refusal, but
cannot validate or reduce it.

## Inputs and call sequence

Construct the robot/station `ObservedSceneBuilder` first. Every calibration,
measurement, world-frame, clock, sensor, robot, carton, inventory, and robot-model
ID must begin with `offline:`. The builder's declaration-verification flag
then refers to checking this offline source/model declaration, never to a
physical survey. The adapter labels its packets, audit and scene metadata
`simulation_only=True` and `physical_calibration_verified=False`.

`OfflineObservationAssumptions` requires an offline run ID and assumption ID,
the PixelPort camera name, carton translation/rotation error bounds, explicit
error bounds for all four flaps and all twelve robot encoders, an uncertainty
horizon, and an explicit policy for carton registration from paired short-flap
tags. These numbers must cover possible motion through that horizon in the
declared experiment. They are not inferred confidence intervals.

`OfflineSensorCapture` holds:

- Matching run and clock IDs and the PixelPort observation sequence.
- Simulated RGB and depth capture times.
- The **pixel observer's** calibrated `world_from_camera` transform copied at
  that sequence, never a rendered camera body's world pose.
- A named encoder sequence and capture time, plus exactly twelve named robot
  joint coordinates in model radians. A full simulator qpos array is not an
  accepted encoder packet.

Copy this capture together with the PixelPort reading and the matching
`port.observer.history[-1]` row, before any later motion or observation:

```python
adapter = OfflinePixelSceneAdapter(builder, explicit_offline_assumptions)
result = adapter.build(
    reading,
    matching_observer_history_row,
    captured_camera_and_named_robot_encoders,
    now_s=declared_simulation_clock_now,
    required_flaps=("long_near", "long_far"),
    unknown_flaps="swept",
    inventory_complete=True,
    unmodelled_obstacles=[],
)
scene = result.scene
audit = result.packets.audit
```

All four final policy arguments are explicit. `inventory_complete=True` is a
closed simulated-inventory assertion, not an image-based unknown-object
detector. Any observed paddle or declared unmodeled obstacle refuses this
bare-claw adapter. Station geometry must include the complete declared table,
cart and fixture inventory. Static geometry or model export alone does not
establish the free carton's pose.

`adapter.adapt(...)` returns `OfflineScenePackets(observation, encoders, audit)`
without building a model or consuming the builder's sequence. Its output is
inspectable before a failed scene build, but only `build()` applies all
freshness, joint-limit, uncertainty-capacity and required-flap gates.

The adapter accepts current wall-tag registration and, only when explicitly
enabled, paired-short-hinge tag registration. PixelPort sequence, observer
sequence, camera capture sequence, detected tag set, and aligned-depth tag
quality must agree. A stale or ambiguous row is not promoted to a fresh
identity. Current PixelPort task dimensions and tag mapping must match the
declared carton. The carton-marker registry is hashed at adapter creation;
changing it later refuses subsequent adaptation.

Tag-derived flap angles and major-flap hinge-consistent plane estimates retain
their current acceptance gates. Legacy unlabelled cardboard-plane estimates
are omitted with an audit reason. A missing required flap refuses; an optional
missing flap uses the builder's full hinge-sweep occupancy only when explicitly
requested. There is no default angle, prior-angle fill, commanded-angle fill,
or truth-angle replacement. Known privileged mechanics probes are refused.

Use `scene.planner()` as described in `carton-observed-scene.md` so the mandatory
uncertainty margin and creation-time expiry check are retained. Planning and
execution limits in that document still apply. This adapter adds no executor,
online revalidation, hardware link, or physical contact guarantee.

## Static robot template export

`export_offline_robot_model(scene_xml, root_body_names=..., asset_root=...,
model_id="offline:...", measurement_id="offline:...", geometry_error_m=...)`
is explicitly an offline setup helper. It extracts only the two named robot
root trees and static model declarations, hashes external asset bytes, and
returns a `RobotCollisionModel`. Scene objects, keyframes, actuators, sensor
data and object qpos are not exported. Explicit offline base transforms are
supplied separately by the calibration. The builder subsequently validates
joint identities, collision filters and frozen assets.

Changing a nonrobot body's source XML pose does not change the exported robot
template. This helper must not be presented as a way to obtain a measured
physical station profile from a simulation scene.

## Component evidence, 2026-10-07

```sh
PYTHONPATH=. .venv/bin/python -m pytest tests/test_folding_observed_adapter.py tests/test_folding_observed_scene.py tests/test_folding_paths.py -q
```

144 tests pass: 53 adapter tests and 91 scene-builder/existing-planner tests.
Coverage includes stale or mismatched capture records, missing depth quality,
wrong identities, unsupported or ambiguous flap rows, explicit swept-volume
policy, known privileged input, no extra object coordinates in encoders,
fixed error assumptions despite tiny fit residuals, static template export,
and invariance of packets/scene/path under unrelated hidden diagnostic fields.

The rendered PixelPort component experiment is saved at:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/observed-adapter-2026-10-07/`

Its `replay-pixelport-component.py` restores archived qpos only to produce RGB-D
images. PixelPort then runs its normal AprilTag/depth estimators. Only those
outputs, the matching pixel-derived camera transform, and named robot encoder
coordinates enter the adapter. There is no dynamics step or robot action.
The complete source-declared static inventory contains 30 station obstacles;
the robot export hashes 469 assets. The source calibration and bounds are
explicit synthetic assumptions, and no independent camera-accuracy measurement
is made. The renderer reports limited depth precision because
`ARB_clip_control` is unavailable; this experiment is not a sensor-accuracy
validation.

| Archived frame | Result |
| --- | --- |
| 0, all flaps required | Refuses `short_right`: only a legacy unlabelled cardboard estimate is present. |
| 0, majors required and optional sweeps | Builds with a conservative right-short sweep. Left path start then refuses: 10.544 mm global robot uncertainty exceeds a 9.881 mm wrist/jaw separation. |
| 527 and 529, either policy | Refuses the left wrist encoder interval. Recorded positions are 1.6581387 and 1.6581618 rad; the declared upper joint limit is 1.65806 rad, before adding the assumed ±0.001 rad encoder error. |

No missing angle was filled, joint value clipped, or bound reduced to make
these cases pass. `pixelport-component.json` contains observations, tag-quality
evidence, refusal reasons and source hashes; `junit.xml` records the tests.
The experiment demonstrates the adapter boundary and conservative refusal,
not a usable closure path or physical execution.
