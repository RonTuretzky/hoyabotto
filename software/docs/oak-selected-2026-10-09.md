# Selected OAK camera and policy projection

The tested capture is 1040×780 at requested 10 fps, derived from IMX214 13MP ISP 1/4
with a centered horizontal crop of 6 pixels on each side. Focus is 79. Factory lens
calibration was expanded from its 3840×2160 centered sensor ROI to 4208×3120 at runtime;
EEPROM was not modified. The exact source K/D and configuration hash are in
`software/config/oak-policy-camera-20261009.json`.

The virtual policy camera is 320×240 RGB, square-pixel focal length
289.70562748477136, principal point (160,120), 45° vertical FOV, identity rectification
and zero output distortion. Call `carton.oak_policy_camera.preprocess` with the exact
immutable source frame and its manifest. OpenCV BGR callers must set
`input_color_order="BGR"`. Image hashes/frame identity and freshness remain the caller's
responsibility. The helper rejects changed camera, focus, configuration, source K/D,
size or invalid/folded sampling maps. It performs one OpenCV INTER_LINEAR remap and
returns RGB. Pixel centers are integer coordinates; no subsequent crop/aspect resize.

Render this pinhole directly in simulation; do not distort or undistort the simulator
image using the source lens. Validate pixel-center conventions against actual rendered
points. Dataset, checkpoint, preview, evaluation and physical inference must bind the
same contract and helper hashes. Changing the crop or camera pose invalidates the match.
The canceled v5 recipe (36.682° VFOV) is not compatible.

The 60-second 10 fps trial delivered 585 pair intervals in 60.3036 seconds (9.7009/s).
603 RGB device sequence intervals passed: 18 were unpublished (2.99%); the cause is
not distinguished among synchronization, USB and host queues. Maximum sampled RGB/depth
skew was 32.923 ms. Train/evaluate with observed holds/delays where relevant; do not
weaken freshness gates. End-to-end capture-to-action latency remains unmeasured.

A largest centered 52.2° rectified crop crosses a pole in the factory rational distortion
model near normalized radius 0.718. The selected 45° view has valid smooth sampling
throughout. Additional useful rectified coverage requires a full-sensor physical lens
calibration. Natural-feature cross-mode agreement supports the source ROI transformation,
not camera-to-arm or RGB/depth registration. No camera pose has been physically registered.
The observed head ticks were 2085/2623, not the former 1919/2622.

Training owner: las-vegas-v1/.context/chat-mac. Final evidence and reply are in
las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/oak-selected-20261009/.
The training owner acknowledged canceling the old job and must confirm consumption,
actual simulator projection/sampling tests, and new job state after restart. This handoff
contains no physical-motion authorization. Three RGB images and 12 joint values remain
the ACT input; tags and depth are separate guarded observer/planner evidence.

The optional local `oak-profile.json` in the stream output directory pins device,
13MP ISP 1/4, 10 fps and exact configuration hash across normal stream restarts.
Explicit `--full-sensor` trials or `--ignore-profile` can override it for commissioning.
Missing or changed calibration against a saved profile must fail before pipeline start.
No claim of physical calibration, tag readiness or verified folding follows from these tests.
