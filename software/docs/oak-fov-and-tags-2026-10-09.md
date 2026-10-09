# OAK coverage and current AprilTags, 9 October 2026

The live OAK is not using the full sensor field. The AprilTag pipeline is installed and responds, but the current observations do not support claiming that tag-guided folding is commissioned. This investigation changed no runtime settings and sent no motion, enable, restart or camera-revival command.

## Camera evidence and documented sensor behavior

- Live manifest: DepthAI 2.33.0.0, USB2 (`UsbSpeed.HIGH`), ColorCamera 1080p preview 640x360, manual lens position 79, raw RGB with factory distortion, mono stereo 640x400, requested alignment CAM_A, depth output 640x360. RGB/depth sequence 105145 had a 19.4 ms timestamp separation. Synchronization within that pair does not establish pixel registration.
- [Luxonis IMX214 sensor documentation](https://docs.luxonis.com/hardware/sensors/IMX214) explicitly describes `THE_1080_P` as cropping to 4K followed by binning. `THE_4_K` and `THE_12_MP` are also cropped; `THE_13_MP` is full resolution, 4208x3120. This establishes the documented mode behavior, not a live measurement of this unit's sensor ROI.
- The selected 16:9 1080p picture is proportionally reduced to the 16:9 640x360 preview. That extra resize cannot restore the omitted sensor area. A bigger frontend image cannot restore omitted field or captured detail either.
- [OAK-D Lite specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite) list nominal RGB HFOV/VFOV of 69/54 degrees. Earlier 64.7/39.2-degree estimates came from a pinhole calculation with runtime K; they are not a physical lens/FOV measurement.
- The earlier frontend check found source JPEG and MJPEG frames at 640x360, preserved by the server and displayed with `object-fit:contain`. The 1100-pixel-wide lightbox enlarged their content proportionally to 1100x618.75 CSS pixels. This explains softness without CSS stretching.

## Proposed candidate, not installed or hardware-validated

Use `THE_13_MP` and preserve its 263:195 aspect ratio through the complete output path. A sensible first candidate is ISP scale 1/4, yielding 1052x780, at 5 fps, then test 10 fps if stable. An initial lower-bandwidth candidate is 526x390 (1/8). These are calculated output sizes; they are not verified successful modes on this station. 640x480 is only approximately the native ratio and should not be called exactly uncropped.

Prefer a full-field ISP-derived output for the commissioning comparison; avoid assuming a preview or video output is full field merely because the sensor is set to 13MP. The pinned [DepthAI v2.33 ColorCamera source](https://github.com/luxonis/depthai-core/blob/v2.33.0/src/pipeline/node/ColorCamera.cpp) defines 13MP as 4208x3120 and limits automatic video dimensions. Inspect the actual ISP/video/preview dimensions, crop and field landmarks. [Luxonis resolution guidance](https://docs.luxonis.com/software/depthai/resolution-techniques) distinguishes cropping, letterboxing and stretching. Preserve proportions or letterbox explicitly; do not stretch a full-field image into 16:9 to make it look wide.

Prior ISP-based streaming crashed on USB2 with X_LINK_ERROR. Higher source resolution is not a demonstrated repair. A controlled camera-owner maintenance test should compare fixed-scene boundaries with the current mode, record exact configuration and stream IDs, verify stability, latency, dropped frames and restart behavior, and provide a rollback. No such switch was performed here.

Changing the RGB projection requires matching calibration metadata, overlays and depth handling. Derive intrinsics from the actual source calibration dimensions and every applied crop/scale/padding transform. The [v2.33 calibration implementation](https://github.com/luxonis/depthai-core/blob/v2.33.0/src/device/CalibrationHandler.cpp) scales/crops K; it does not certify an arbitrary stream's physical alignment. Keep raw RGB distortion distinct from rectified/depth projection, correct the existing enum parsing mismatch, and validate registration against known targets at several distances and image locations. Resizing depth to the RGB dimensions or setting CAM_A alignment alone is insufficient. Higher RGB resolution also cannot create additional stereo information from the 640x400 mono input. More field at unchanged output dimensions can make tags smaller; improved tag detection is not automatic.

## Fresh read-only tag and depth observations

Capture at approximately 21:08:44 JST, evidence under `.context/oak-fov-research/live/`:

- Three `/api/carton-tags` reads succeeded, but every view reported `NO_VALID_TAGS`. OAK sequences 105142, 105143 and 105144 were distinct. Phone sequences 443243/443244 and wrist sequences 75829/75830 (left), 75848/75849 (right) include repeated frames in this short burst; these are not twelve independent exposures.
- OAK/wrist reported capture ages were about 0.42-0.65 seconds. Cross-host clock synchronization remains unverified. The phone has only server receipt timestamps, not verified capture age.
- Offline raw decoding of the saved, annotated/re-encoded phone JPEG found IDs 12 and 22 with hamming 0, margins 18.73 and 17.00, and shortest edges 43.29 and 55.77 pixels. These edges exceed the size threshold; decode quality remains the issue in this diagnostic. This used a derived image, not original capture bytes. It suggests weak contrast/decode quality; it does not establish a lighting root cause or live acceptance. The base detector drops margins below 20; the carton observer additionally requires margin at least 30, hamming 0 and shortest edge at least 24 pixels. No thresholds were lowered.
- The tagged carton is back on the table. The OAK shows large carton panels; phone shows the outside tagged panel; wrists show pads and nearby cardboard. Printed markers being visually present is not proof of reliable accepted detection. Earlier phone IDs 11/28 were accepted in a different view, as recorded in the prior handoff.
- Tag result reports `depth_used:false`, `metric_pose_available:false`, `coordinate_system:per_image_pixels`. The carton observer supplies identity and pixel geometry, not registered robot targets or grasp verification.
- Real stereo depth is available. Current scene sequence 105147 fails the table fit: dominant plane approximately 76% of sampled points, normal 88.8 degrees from expected up. Runtime falls back to `camera_pose_source:model`, with `mapping_validated:false`, `robot_frame_calibrated:false`, no loaded joint map. The earlier clear-table fit is saved evidence, not a runtime fallback calibration.
- The factory distortion coefficients are still skipped because `CameraModel.Perspective` does not match the parser's accepted `Perspective`. Correcting this alone would not validate RGB/depth correspondence or camera-to-arm geometry.
- Bracketing owner readbacks contained all 16 motors released, zero position deltas and no torque-flag changes. Motor commands sent: zero.
- Head ticks 2094/2621 match the saved clear-table baseline. This does not prove unchanged physical camera/cart/table geometry. Removing the carton is unnecessary for tag diagnosis and would hide the very markers being checked. A durable table-calibration workflow still needs projection correction/validation, explicit baseline installation, freshness/geometry invalidation, and independent registration evidence; repeatedly clearing the table is not that software fix.

## Updated pilot prompt

[carton-pilot-v8b-tags-v1.txt](prompts/carton-pilot-v8b-tags-v1.txt) retains the historical pinch/hinge-arc/hold strategy while explicitly requesting tags and requiring current evidence for pinch, path and final retained fold. It removes reliance on blind legacy scans, fixed height correction, tick-only grasp verdicts, continuing after contact, and depth-only success. This is a new, unbenchmarked prompt; the historical 7/8 Flash metric must not be attributed to it. It was saved only, not installed or executed.

## Requested independent research

Delegation was attempted through Shape Rotator's Codex research backend; it failed parsing an empty final response. A direct read-only Codex subprocess then reported the account usage limit. There is no completed independent subagent finding. The primary agent checked the official documentation and pinned source directly. Failure receipts and fetched source files are in `.context/oak-fov-research/`.
