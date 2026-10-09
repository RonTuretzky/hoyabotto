# Current tag angles and OAK calibration contract, 9 October 2026

Read-only capture: 21:19:38–21:19:46 JST. Six spaced samples from each of four existing viewpoints; no head/arm sweep, enable, release, restart, fold or training command. Raw camera bytes, frame hashes, accepted-observer results and unfiltered detector diagnostics are preserved separately. All 24 frame sequences and saved hashes are distinct. No physical task completion is claimed.

## Which angles are usable for tag identification (step 2)?

| View | What the image shows | Same-source-byte result over six frames | Assessment |
|---|---|---|---|
| OAK 640×360 | Untagged inside faces of the carton; external markers hidden | No decoded candidates; 0/6 accepted | Current angle cannot identify the outside tagged panels |
| Phone 540×960 | Nearly frontal exterior right wall and short flap, underexposed relative to room | Candidates 12 and 22 in 6/6, hamming 0; 0/6 accepted | Best current angle; improve illumination/exposure before using it |
| Left wrist 640×480 | Pads and very oblique/blurred markers at left edge | No decoded candidates; 0/6 accepted | Useful for pad context, currently unsuitable for tag identification |
| Right wrist 640×480 | Pads and nearby untagged cardboard | No decoded candidates; 0/6 accepted | No usable tag geometry in this pose |

Original phone-frame scores: ID 12 decision margin 15.61–16.39, shortest edge 42.79–43.09 px; ID 22 margin 13.41–14.25, edge 55.21–55.53 px. These differ from the prior annotated-image diagnostic because these are the original capture bytes. The base detector drops margin <20; the observer requires margin >=30, hamming 0, edge >=24 px. Do not lower gates. Tags already exceed the pixel-size minimum. Visible darkness suggests lighting/exposure is a useful next test, not a proven sole cause.

Keep the carton present for tag tests. Best next optical trial: illuminate the tagged exterior from the camera side with diffuse light, keep the phone nearly frontal to IDs 12/22, preserve visible white borders and avoid glare. The phone exposure should favor the tags rather than the bright room. Compare multiple new raw frames before calling the angle usable. A tag moved or duplicated on another face needs a new measured layout; do not silently reinterpret its pose.

OAK sequences 111608/111621/111634/111647/111659/111672; phone 445832/445837/445843/445848/445852/445858; left 79115/79121/79128/79134/79140/79147; right 79134/79140/79147/79153/79159/79166. Observer ages: OAK 0.372–0.501s, left 0.590–0.738s, right 0.621–0.769s on cross-host clocks whose synchronization is not verified. Phone 0.297–0.596s is receipt age only; capture latency is unknown.

All 16 positions and torque flags stayed unchanged across fresh, noncached owner brackets (read ages 44ms and 37ms). Both head motors were ALREADY enabled; twelve arm joints and both wheels were off. Do not describe this capture as all motors released. Head ticks are 1919/2622, versus 2094/2621 in the earlier clear-table baseline. That baseline cannot be reused as an unchanged-camera calibration. No inference about who moved the head is made.

## Turning tags into reliable claw targets (step 3)

1. Fix the camera projection contract: camera identity, sensor mode/ROI, focus, source dimensions, K, complete distortion vector, rectification, crop and resize. Keep raw/full-view images for the pilot; create a separately named policy input. Fix the known distortion-model enum handling and verify RGB/depth registration with a measured target at several image locations/distances before using depth as robot coordinates.
2. Fix the head at a measured pose initially. Establish OAK-to-arm-base position AND rotation from a rigid calibration target whose location is independently measured relative to the two arm bases. Alternatively use rigid gripper tags across several distinct poses only after verifying joint zeros/signs, actual base placement and tag-to-link mounting. Existing box-only poses do not establish arm registration. Do not use assumed carton placement to prove the camera transform that will then measure that same placement.
3. Measure actual printed black-square sizes and tag-to-hinge layout, plus gripper-tag-to-white-pad contact offsets. Calibrate wrist lenses and their camera-to-gripper transforms. CAD camera collision geometry is useful but is not a physical lens calibration.
4. Save the result with camera/stream/projection and joint-map fingerprints, source frames/ticks, uncertainty and holdout checks. Test at separate target locations near the working region; choose error limits against available grasp/collision clearance. Existing `xlerobot-fold-station-measurement/1` can hold `cameras.front.position_m`, `rotation_cv` and intrinsics in the `arm_base` frame (+x robot right, +y toward table, +z up, metres, origin midway between SO101 base mounting origins). Keep D and preprocessing provenance alongside it: `camera_entry()` itself does not serialize the distortion vector.
5. Install the validated mapping into the carton observer/guarded planner. It currently returns only per-image pixels (`depth_used:false`, `metric_pose_available:false`), and its custom `CartonTagRobot` observer deliberately has no metric geometry. A legacy paddle geometry file alone does not connect this path. Validate observations and targets without powered motion before a supervised approach.

The saved clear-table fit constrains tilt/roll/lens height, but not heading or horizontal translation. Persist a baseline only after projection and registration checks, and invalidate on head/camera/cart/table changes. This avoids treating a vertical carton panel as a new tabletop. Repeatedly removing the carton is not the durable fix.

## What the current frozen training bundle expects

Inspected local `training/cloud-release-v5` source, including `record_refit_fold_demos.py`, `folding_refit_preview.py`, `folding_station_measured.py` and `fold_policy_runner.py`. This is direct source evidence; a live training-agent reply has not yet been received.

- Policy cameras: `front`=OAK, `left_wrist`, `right_wrist`; RGB plus joint state. Phone, depth and tag-pose values are not direct inputs to this ACT policy. Teacher fiducials are a separate mechanism; synthetic table-marker group 4 is hidden from policy images.
- OAK: 640×360 raw frame; factory K fx 504.894104, fy 504.973846, cx 314.753784, cy 192.592651; complete 14-element factory D. Rectify using the same K, then largest principal-point-centered 4:3 crop. Calculated float bounds x91.543986..537.963582, y25.185303..360; crop 446.419596×334.814697. Resize to 320×240; nominal K' fx 361.915370, fy 361.972530, cx 160, cy 120. Sim uses square pixels/fovy 36.682459°, so its nominal fx equalsfy 361.972530; tiny source fx/fy difference is approximated.
- Crop rounding/interpolation is not fixed by a production preprocessing implementation yet. `oak-policy-candidate.png` demonstrates an OFFLINE linear-remap proposal only. It does not validate physical extrinsics or RGB/depth alignment and is not installed.
- Head transform remains model-derived at tilt 35.145°, pan −5.132°, from the approved simulation configuration. Current head ticks 1919/2622 differ from the previous evidence pose; no verified tick-to-optical-angle transform permits treating these as the trained view. The old 58°/pan 0 comparison helper refers to a legacy profile, not the frozen refit recorder.
- Expected simulation station: base-line setback 180mm, base spacing 220mm, tabletop 700mm, CAD mounting plane 729.1mm, carton near-wall inset 10mm. These are configured assumptions except the user-confirmed desk; no new physical mounting height/spacing/setback/carton registration measurement was obtained.
- Wrist 90°VFOV and CAD-derived camera-to-gripper mounting remain assumptions. The source acknowledges this.
- The physical policy runner still passes source frames to its policy and only warns on aspect mismatch; it does not implement the promised OAK undistort/crop preprocessing. A bigger or wider raw stream must be mapped into the agreed virtual camera or the demonstrations re-rendered. Do not stretch the whole 16:9 frame to4:3 or equate a full-sensor switch with matching training.

## Training-thread coordination

Conductor native UI attachment failed with `Sky Computer Use native pipe startup failed`. No chat message or reply is claimed. The current evidence and a concrete request were deposited in the server-evidence directory that the training agent previously said it would read. Awaiting its confirmation of the calibration contract, exact projection sampling, pose tolerances and whether corrected measured cameras require re-rendering.
