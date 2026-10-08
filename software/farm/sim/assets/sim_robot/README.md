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
