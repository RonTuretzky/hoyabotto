# Automatic head-camera pointing and pose measurement

`tools/auto_head_pose.py` points the head OAK at the fold policy's training view (tilt 58° down, pan 0) and measures
where the camera actually is, through the robot API. `carton/head_pose.py` holds the solver.

## What it replaces

In the station set-up (`carton-fold-policy-setup-slides.html`) it replaces these manual steps:

| Manual step | Now |
|---|---|
| 5: set head pan 0° / tilt 58° with the head controls, note the ticks | the tool moves the head in small steps and records the ticks |
| 6a: print and tape floor tags 40–43 at measured spots | **not needed**: the robot's own gripper tags (left 4, right 2) are the reference |
| 6b: check 58° with a phone inclinometer, save a still | **not needed**: the tilt is measured from the image; every frame used is saved |
| 6c: run `camera_pose_from_tag.py` by hand | done on every frame; the result is written as `cameras.front` |

The gripper tags' 3D corners come from the arm encoder ticks, the joint maps (`profiles/fold-joint-maps/`) and the
SO-101 kinematics of the fold training scene. All corners of all visible tags go into one `solvePnP`. With
`--box-tags` and the carton in its taped spot, the carton's wall and floor tags are added too.

`tools/oak_intrinsics.py` is only needed when the OAK stream manifest carries no intrinsics. Today's stream carries
them, and the tool reads them with the frame.

## Before running

- The owner runs with `--head` (`robot_get_capabilities.head_supported` is true), so `robot_move_head` works. The
  phone feed is fresh, because the owner gates motion on it.
- **Take the carton away** unless you use `--box-tags` (see below).
- Put both arms in the measurement pose and keep them still: grippers raised in front of the robot, tags facing the
  head. Print its encoder targets for your usual arm tools (this tool never moves an arm):

  ```bash
  cd software
  PYTHONPATH=. python tools/auto_head_pose.py --print-arm-pose
  ```

  In URDF degrees (shoulder_pan … wrist_roll), that is left `-1, 34, -4, -76, -14` and right `1, 34, -4, -76, 20`:
  tag centres about 0.20 m above the base plane and 0.28 m forward. In the training scene, both tags stay in view
  for head tilts from about 33° to 60° on the 640×360 stream (46° to 64° at 640×480). Any other still pose works if both tags show and face
  the head.

## Commands

Dry run (the default). It is read-only: it reads arm and head ticks plus a few OAK frames, then reports the measured
pose against the training pose. Nothing moves, and nothing is enabled.

```bash
PYTHONPATH=. python tools/auto_head_pose.py --pilot-root <pilot> --out runs/head-pose-dry
```

Closed loop (moves the head only):

```bash
PYTHONPATH=. python tools/auto_head_pose.py --pilot-root <pilot> --out runs/head-pose-1 \
    --execute --operator <name of the person holding STOP> [--enable-head] [--box-tags]
```

Each iteration does four things:

1. Reads the encoders, then takes 3 fresh frames and averages their tag corners.
2. Reads the encoders again. If any arm or head joint moved by more than 2 ticks, it aborts.
3. Solves the pose.
4. If |tilt − 58| > 1° or |pan| > 1°, it sends one `robot_move_head` toward the target.

Limits on each move:

- **Step:** at most 60 ticks per joint per command (about 5°; `--max-step-ticks`, never more than the owner's 200).
- **Range:** the target stays inside the saved range minus the owner's 40-tick margin.
- **Duration:** 1.5 × the owner's minimum duration.

Tick direction starts from the twin's convention (more `head_motor_2` ticks = tilt down, more `head_motor_1` ticks =
pan left). From each move the tool measures how many degrees one tick turns the camera. If the direction turns out
reversed, it uses the measured direction and warns.

The loop stops in any of these cases:

- both angles are within 1° (`within_tolerance`);
- after 12 moves (`max_iterations`);
- when the next step would leave the commandable range (`head_range_limit`, with the tick the target would need).

It aborts on any of these, and sends nothing after that:

- a refused or failed call;
- an owner STOP or fault;
- arms or head moving during a frame;
- fewer than 2 usable tags, or a reprojection RMS above 1 px;
- a head response that is implausible against the measurements.

It never calls `robot_stop` and never releases a motor: the head holds where it stopped. `--enable-head` enables the
two head motors first. They hold where they are, so nothing moves.

Re-render the demonstrations with the measured camera (step 7):

```bash
PYTHONPATH=. python tools/restage_fold_scenes.py --measurement runs/head-pose-1/station-measurement.json \
    --batches <fold-demos>/batch-220-01 --out <fold-demos>/restaged-head-1
```

## Output

`<out>/auto-head-pose.json` contains:

- every measurement: pose, tilt/pan/roll, RMS, tags used, head and arm ticks, the frames with their sha256, seq and
  capture time, and the spread between single frames;
- every command with the owner's answer;
- the gain estimates;
- the intrinsics binding and stream report;
- warnings;
- `camera_entry`: `cameras.front` in the measurement-file format of `carton/folding_station_measured.py`, with
  position, rotation, intrinsics, head angles and head ticks.

`<out>/station-measurement.json` is `--station` (default `profiles/fold-station-xlerobot-220.json`) with
`cameras.front` replaced. It is the file `tools/restage_fold_scenes.py` reads. Only the camera is measured; the
station numbers stay those of the profile. `--update FILE` writes the entry into an existing measurement file
instead. The frames are under `<out>/frames/`.

## Owner's role

- Approve the run and give your name for `--operator`.
- **Keep STOP in reach** the whole time. The tool never stops anything itself.
- Put the arms in the measurement pose and keep them still, with both gripper tags visible to the head.
- Watch the head. Each move is a few degrees and takes about a second.

## What to expect today (9 Oct facts)

- **The training tilt is probably out of reach.** The OAK looked level at released `head_motor_2` = 2211, and the
  tilt range is 1972..2625 commandable. With 4096 ticks per turn, 58° down would need about 2871 ticks, and the
  head stops near 36°. The loop then ends with `head_range_limit` and still writes the measured pose. Do not relax
  the range. Re-render the demonstrations at the measured pose (step 7), or change the head mounting or calibration
  (owner's decision).
- **The stream is not the training camera.** It is 640×360 (16:9) with fy ≈ 505, a 39° vertical FOV. The training
  camera is 4:3 with 54° vertical. The tool warns about this, and the measurement entry carries the real
  intrinsics. Restaging then uses the largest 4:3 crop (39° vertical).
- The lens offset in the slot cradle (`farm/sim/xlerobot_twin.py` `HEAD_OPTICAL_OFFSET_M`, a design value) is used
  only to compare the measured position with the model. The measured pose itself needs no head model.

## Accuracy and limits

Measured on simulated frames (`tests/test_auto_head_pose.py`), with the arms in the measurement pose:

- **MuJoCo renders of the training scene** (640×480 at 54°, 640×360 at 39°; tilts 33–64°, pans −3° to 5°): within
  0.15° tilt, 0.3° pan and 1.1 mm.
- **Synthetic frames:** within 0.5° and 3 mm.
- **Closed loop:** the fake head's mapping was deliberately wrong: zero below the range, +5 % scale on tilt, and pan
  reversed and −7 % scale. The loop goes from tilt 49° / pan 4° to within 1° in 3 moves.

Limits:

- **Corner noise matters.** At 0.5 px of corner noise, one frame's 1-sigma is about 0.7° tilt and 1.1° pan (two
  40 mm tags at ~65 px). The tool averages 3 frames (`--frames`) and reports the spread it actually saw.
- **The answer is relative to the arms as the joint maps place them.** The maps are owner-accepted, not measured. A
  zero error shared by both arms tilts the answer by about the same angle, with a perfect fit: shoulder_lift +2°
  gives about 2° of tilt. A wrong joint on one arm usually shows up as reprojection error and is refused, but not
  always (wrist_flex +8° on one arm passed at 0.4 px RMS). For an independent check, run once with the carton in
  its taped spot and `--box-tags` (the arms must not hide the carton tags). The gripper-only and box-only solutions
  are then compared, and the tool warns when they differ by more than 1.5° or 15 mm.
- **Box tags assume the nominal carton spot:** 10 mm from the table edge, centred and square. The station profile
  gives the spacing, height and setback.
- **This is not the pilot's table-plane check.** The pilot's head agent measures the head sign and tilt zero from the
  table plane (`farm/perception/depth_scene.py` `fit_table_plane`). That method needs no arms but gives no pan.
  This tool does not repeat it.
- **Not run on the robot yet.**
