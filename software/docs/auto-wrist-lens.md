# Automatic wrist-camera lens calibration from the box tags

The fold policy was trained on simulated wrist cameras with an assumed 90° vertical field of view. To find the real
lens, the robot now photographs the tagged carton itself, and the twelve box tags serve as the calibration target. No
printed checkerboard is needed. The output is the same intrinsics JSON as `tools/calibrate_camera_checkerboard.py`,
with `"method": "box_tags"` added. The checkerboard tool is still there as a fallback.

## What it does

1. **Plans the poses** (`tools/capture_wrist_calibration.py`, simulation only). Starting from the arm's current
   position, it searches the 220 mm training scene for about 12 poses of one arm. In each pose the wrist camera sees
   at least two box tags clearly: they face the camera, they fit inside a field of view narrower than the assumed
   one, and nothing hides them. At each pose and along the straight joint path from the start pose, no part of the
   arm comes within 3 cm of the carton, its flaps, the table, the cart or the other arm. Every target also stays at
   least 65 ticks inside the saved range: the owner's 40-tick margin plus 25 ticks. The plan goes to `plan.json`.
2. **Captures the frames** (only with `--execute --operator NAME`). For each pose, it moves the arm there with
   `robot_move_joint_targets` (one arm, `wait=true`), waits 1 s, checks that the arm is still where it was sent,
   and takes a new frame with `robot_get_cameras`. The frame must have been captured after the arm settled. Then
   it moves back to the start pose. It never commands the gripper and never calls `robot_stop`. It stops sending
   commands as soon as anything goes wrong: a refused or unfinished move, an owner fault, STOP or restart
   (`stop_count` changed), a motor status bit, a joint off target, a stale frame, or Ctrl-C. When it stops, the arm
   holds wherever the owner left it.
3. **Solves for the lens** (`tools/calibrate_wrist_from_tags.py`, offline). It detects the tags and keeps only clean
   decodes (hamming 0, margin of at least 30) in frames that show at least 2 tags. It starts with `cv2.calibrateCamera`
   on the tags' nominal 3D layout (`carton/box_tag_geometry.py`) and an initial guess of a 90° vertical field of view.
   Then a bundle adjustment fits the lens, the carton pose in each frame and each tag's position on the carton. That
   last part absorbs hand-placed tags, bulging walls and leaning flaps. The result is refused (non-zero exit, no file
   written) if fewer than 8 frames are usable or the RMS reprojection error is above 1.0 px.

In simulation the full chain recovers the field of view to within 0.7°: 80° came out as 80.1° (right wrist) and
70° as 70.7° (left wrist), at 0.1 px RMS. That chain is the deployed owner code over the MuJoCo fold scene, with
frames sent through the real camera tool. See `tests/test_wrist_lens_from_tags.py`.

## Commands

```bash
cd software
# 1. plan only (default): reads the arm position through the API, moves nothing
python tools/capture_wrist_calibration.py --pilot-root <pilot> --arm right --out runs/wrist-lens-right
# 2. supervised capture + calibration (operator holds STOP)
python tools/capture_wrist_calibration.py --pilot-root <pilot> --arm right --out runs/wrist-lens-right \
    --execute --operator NAME
#    -> runs/wrist-lens-right/frames/*.png, capture.json, right_wrist-intrinsics.json
# re-solve saved frames at any time (no robot)
python tools/calibrate_wrist_from_tags.py --images runs/wrist-lens-right/frames --out right_wrist-640x480.json
```

Repeat with `--arm left`. Joint maps default to `profiles/fold-joint-maps/`. To use other maps, pass
`--joint-map left=PATH --joint-map right=PATH`. If you run it without `--pilot-root`, it plans offline from the
scene's start pose and contacts no robot.

## What the owner does

- Put the carton in its spot from the setup slides, with all twelve tags on and the flaps upright. Do not touch it
  during the run: every frame must show the same carton.
- Enable that arm's six motors (`robot_set_motor_enable`), clear the space around the station and keep the STOP
  button in your hand.
- Read the dry-run's list of poses, then approve the run by starting it with `--execute --operator <your name>`.
- Afterwards, check that `rms_reprojection_px` is at most 1.0 and that `vertical_fov_degrees` makes sense. If a
  tag's `tag_adjustments` value is above 10 mm, that tag is probably misplaced. If the flaps were not upright, re-solve
  with `--rigid-only` (walls and floor tags only).

## Limits

- At the 220 mm station the wrist cameras mostly see the two floor tags (24, 25) from above, plus the near wall and
  the near flap now and then. Tags on the side walls and side flaps hardly show up in the planned views, because
  of the arm's reach and the upright flaps. The solver works with that set, but more tags in view would make the
  distortion estimate stronger.
- The plan comes from the training scene. If the real station differs from it (carton spot, base height), the real
  views differ too. The capture checks only that each frame is fresh, not which tags it shows. The solver then
  skips frames with fewer than 2 tags and refuses if fewer than 8 remain.
- The lens model is pinhole + k1, k2, p1, p2 (k3 fixed at 0). Pixel coordinates use the OpenCV convention: pixel
  centres sit on integer coordinates. The 0.5 px offset of the AprilTag detector is removed.
