# Real camera references for the simulated robot

Captured read-only from the real robot server on 2026-10-08 (`robot_get_cameras`, `robot_get_depth`,
`robot_get_clip` through the chat Mac's `chat_server.Robot`), so `farm/sim/sim_cameras.py` can match the look
and the result shapes. Pixels of the depth PNG are not kept; everything else is the tool output minus
`data_base64`.

- `real_left_wrist.jpg`, `real_right_wrist.jpg` (640x480) and `real_oak.jpg` (640x360): one frame each.
- `real_*_image_record.json`: the image record fields that came with each JPEG.
- `real_oak_depth_manifest.json`: the `robot_get_depth` result (manifest with intrinsics, projection,
  distortion) and its depth image record.
- `real_clip_result.json`: a `robot_get_clip` result for the right wrist (1 s, 2 fps, 320 px).

## `real_oak_depth_frame/`

Two complete `robot_get_depth` captures with pixels, taken read-only on 2026-10-08 evening (robot idle and
released, head at pan 1930 / tilt 2609 ticks) for the table-plane camera self-calibration in
`farm/perception/depth_scene.py`: `depth.png` / `depth_2.png` (640x360 uint16 mm), `robot_get_depth*.json`
(manifest, intrinsics and factory distortion; `data_base64` removed), `robot_get_state*.json` (the encoder
state at capture, for the model camera pose) and `oak_rgb*.jpg` (the RGB frame at the same moment). In both the
OAK is looking at the robot's own black-sleeved arm a few centimetres in front of the lens (29 % and 42 % valid
depth, median 0.5 m), so they are the fallback cases: no table plane qualifies and the tool keeps the model
pose with `camera_pose_source: model`.
