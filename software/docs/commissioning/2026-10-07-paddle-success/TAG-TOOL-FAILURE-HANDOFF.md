# Tag-tool failure handoff — 2026-10-07

## What failed

The Qwen-side request to `robot_get_registered_tags` failed because the calibration adapter tried to read:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/tag-registration.json`

That path belongs to the other Mac. No validated `tag-registration.json` was found on the robot Mac or in this repository. The error is a missing calibration artifact, not a servo, camera, or paddle-grasp failure.

## What the successful pickup used

The verified paddle pickup did not depend on AprilTag registration. It used fresh camera frames for visual confirmation, live right-arm encoder positions, the bounded joint sequence recorded in `Qwen-Paddle-Success-Handoff.json`, measured gripper closure, and camera confirmation that the paddle remained between the fingers during two lifts. The pickup completed and the paddle was later placed and released.

Tags may have been visible in camera images, but no camera-to-arm hand-eye transform was fitted or consumed by that procedure. Tag visibility is not the same as registration.

## Correct Qwen behavior

Qwen should not call `robot_get_registered_tags` as a prerequisite for replaying the verified pickup sequence. It should query fresh `robot_get_state`, `robot_get_readiness`, `robot_get_execution`, and camera results, then use the scoped `paddle-success-v1` controller. The controller requires all six right-arm joints to be explicitly enabled and keeps the existing motion, camera, watchdog, STOP, and release protections.

`robot_get_registered_tags` is a separate read-only capability. It should report `registration unavailable` and continue when no registration file exists, unless a tag-based pose is specifically required.

## To enable tag-based estimates later

Run the configured tag-registration calibration with the selected arm, OAK stream, fixed table anchor, gripper tag, and current motor calibration. Only a passing hand-eye fit should create `tag-registration.json`; do not synthesize it from the paddle pickup record.

## Current live state after latch reset

The sole owner was restarted with `--right-arm-only --paddle-profile`. Startup verified all 16 servos, zero motor writes, all torque disabled, and `stop_latched=false`. No movement was performed during this handoff.
