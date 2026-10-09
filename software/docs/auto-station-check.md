# Automated station check for the fold policy

One head-camera (OAK) frame of the tagged carton is enough to check the station the fold policy was trained
on. The check prints what to change, for example:

```
Do this, then re-run the check:
  1. Lower the table 7 mm (table top is 113 mm below the arm bases; want 120).
  2. Turn the carton 3 degrees clockwise (seen from above), about its centre.
  3. Move the carton 12 mm to the robot's left (its centre belongs 10 mm left of the midpoint between the arms).
```

When everything is within tolerance it prints `OK` and exits 0.

It is read-only. It never moves a motor. On the robot it requests one frame with `robot_get_cameras` (or
`robot_get_depth` with `--with-depth`) through the chat pilot's client, which refuses any tool that is not a
read.

Code: `carton/station_check.py` (the check), `tools/check_station.py` (the command),
`tests/test_station_check.py` (synthetic frames from the training scene).

## What it replaces

These are the tape-measure checks in steps 2-4 of `docs/carton-fold-policy-setup-slides.html`:

| Slide step | Tape check | Automated check |
|---|---|---|
| 2 | Table top 120 mm below both arm bases, left and right within 3 mm | `table height` (the carton bottom's height below the arm bases) and `table level` (the carton's tilt, given as mm across its length and its depth) |
| 3 | Carton centre 10 mm left of the arms' midpoint, near wall 10 mm in from the table edge, square to within 4° | `carton left/right`, `carton distance`, `carton rotation` |
| 4 | Empty training carton, 379 x 283 x 108 mm, flaps up, tags in place | The tags must fit the training carton's geometry; otherwise the result is `MISMATCH`. Flap tags far from their upright place give "Stand the ... flap straight up". |

Some checks stay manual:
- Step 1, the arm spacing.
- Step 3's cart-to-table setback. The table edge is not visible. The check measures the carton against the
  arm bases, so a carton at the right place relative to the arms is correct even if the table edge is not at
  150 mm. Moving the carton toward the robot only makes sense while it still sits on the table.

## Commands

From a saved frame (no robot contact at all):

```
cd software
PYTHONPATH=. python tools/check_station.py --image front.jpg --camera-json head-pose.json \
    [--intrinsics oak-640x360.json] [--depth depth.png] [--json report.json]
```

From the robot (one OAK frame, read-only):

```
cd software
PYTHONPATH=. python tools/check_station.py --pilot-root "$PILOT" --camera-json head-pose.json \
    [--with-depth] [--save-frame front.jpg] [--json report.json]
```

Exit status:
- 0: OK.
- 1: adjustments needed.
- 2: cannot check (fewer than 2 box tags on rigid faces visible), or the box does not match the training
  carton.

### Head-camera pose (`--camera-json`)

The head-pose tool writes the camera's pose in the `arm_base` frame. The frame's origin is midway between the
two SO-101 base origins on their mounting plane (the bottom of the bases). +x points to the robot's right, +y
horizontally toward the table, +z up. Units are metres. Schema `xlerobot-head-camera-pose/1`; the `schema` key
is optional:

```json
{"schema": "xlerobot-head-camera-pose/1", "frame": "arm_base",
 "position_m": [0.002, 0.0507, 0.4167],
 "rotation_cv": [[1, 0, 0], [0, -0.848, 0.530], [0, -0.530, -0.848]],
 "fx": 504.9, "fy": 505.0, "cx": 314.8, "cy": 192.6, "width": 640, "height": 360,
 "dist": []}
```

- `rotation_cv` columns are the optical axes (image right, image down, forward) expressed in `arm_base`.
- The intrinsics are optional, and so are `width`/`height`. Intrinsics given for another resolution are scaled
  if the aspect ratio is the same. If it differs (4:3 against 16:9 is a different sensor crop), the check
  refuses them.
- `dist` holds the OpenCV distortion coefficients. Leave it empty for the OAK's rectified stream.

Also accepted:
- A camera entry from `tools/camera_pose_from_tag.py` (intrinsics nested under `"intrinsics"`).
- A station-measurement file whose `cameras.front` is such an entry.

Without `--camera-json`, the check uses the `front` camera of `profiles/fold-station-xlerobot-220.json`. That
camera is the XLeRobot model at head tilt 58°, not a measurement, so the report is marked
`[UNMEASURED FALLBACK]`. Do not trust those numbers on the two-servo head. A 1° error in the camera's tilt moves
the answers by about 8 mm.

Intrinsics are taken from the first of these that is available:
1. The OAK manifest bound to the fetched frame (`--pilot-root`; it handles the 640x360 stream, fx ≈ 505).
2. `--intrinsics`.
3. The camera JSON.
4. The model's 54° vertical field of view. This only applies to 4:3 frames; for anything else the check
   refuses.

## Targets and tolerances

| Quantity | Target | Recommended tolerance | Policy training range |
|---|---|---|---|
| Table top below the arm bases | 120 mm | ±3 mm | 120 mm only |
| Table level (carton tilt) | 0° | 1° | level only |
| Carton rotation (yaw, + = counter-clockwise from above) | 0° | ±2° | -4..+4° |
| Carton centre x | -10 mm (robot's left) | ±3 mm | -25..+5 mm |
| Nearest bottom corner of the carton, y | 160 mm (table edge 150 + 10) | ±3 mm | 160 mm only |

Out-of-range values get a warning in addition to the instruction.

## Owner's part

1. Put the tagged training carton (all four flaps standing up) on the table.
2. Set the head to the policy pose.
3. Run the check.
4. Do the instructions in the order printed: the table first (height, level), then turn the carton, then slide
   it.
5. Re-run. Repeat until it prints `OK`.

If it says `MISMATCH`, the tags do not sit where they would on the 379 x 283 x 108 mm carton. Possible causes:
- A different box.
- A misplaced tag.
- A dented wall.

Fix the box or the tags (slides 4c-4g). The measurements are not trustworthy until the result is no longer a
mismatch.

If it says floor tags 25/24 are not visible, the near long flap is probably leaning in. Stand it up.

If only the floor tags are visible, the near wall is off the bottom of the image. This happens with the
640x360 stream at the model head pose. Aim the head a little lower.

With a single tagged face in view the check still measures, but it cannot cross-check the box, so it never
reports `OK` in that state. With only the two floor tags (80 mm apart) it does not judge the table level.

## How it works

1. Detect tag36h11 tags with `carton/servo/features.tags_from_bgr`. Keep decodes with hamming distance 0 and
   decision margin ≥ 30. At least 2 rigid-face tags are needed.
2. Fit the carton pose. The model is the eight tags on rigid faces (near wall 26/10/27, left wall 21/28, right
   wall 22, inside floor 25/24). Each tag's corners come from the printed layout in
   `tools/make_fold_box_tags.py`, with the training scene's 2.1 mm wall and 4.1 mm floor stand-off. The fit
   seeds from the nominal placement, SQPnP and every tag's two IPPE solutions, then refines with
   Levenberg-Marquardt on all corners.
3. Check the geometry for each tag:
   - the residual against the joint fit (limit 3 mm in the tag's plane);
   - the leave-one-out misfit against the fit of the other tags (limit 6 mm; needs 3 or more tags).

   On rendered training frames both stay below 0.6 mm. Exceeding either limit gives `MISMATCH`.
4. Move the pose into `arm_base` with the head-camera pose. The table gap is minus the z of the carton's bottom
   centre. The tilt comes from the carton's up axis. The near-wall distance is the smallest y of the four bottom
   corners, which is the same rule the training placement used.
5. Optional second height estimate: the OAK depth table plane (`farm.perception.depth_scene.fit_table_plane`).
   It is reported but not used for the instructions. Its 10 mm inlier band also takes in the carton's floor,
   so expect it about 3-5 mm high.

## Accuracy (rendered training scene, known camera)

The test frames are rendered at 640x480 (4:3, 54°), 640x360 (16:9, fx 505) and 1280x960. Errors were applied
in combination: carton x -15..+12 mm, yaw -4..+3°, table -7..+7 mm, near wall -3..+8 mm. Over 18 such frames
the worst recovered errors were:
- 0.5 mm in table gap;
- 0.1 mm in carton x;
- 0.2 mm in near-wall distance;
- 0.15° in yaw;
- 0.2° in tilt (0.5° from the floor tags alone, which is why the level check is skipped then).

Example: a carton +12 mm right, 3° counter-clockwise, on a table 7 mm too high reads +12.0 mm, 3.0° and
-7.1 mm. The near-wall tags alone, or the floor tags alone, give the position, yaw and height within 1 mm.

## Limitations

- **Camera pose.** The answers are only as good as the head-camera pose. An error in the camera's pitch shows
  up directly as height and distance errors. Run the head-pose tool after every head move.
- **Table edge.** The table edge is not seen. The check places the carton relative to the arm bases, as the
  policy does.
- **Tag placement.** Placement errors of a few mm that stay under the mismatch limits shift the result by about
  the same amount. The model assumes the training scene's tag stand-off. A real carton whose 379 x 283 is the
  inside size, or thicker walls, adds about 1-2 mm.
- **Oblique near wall.** At the head tilt of 58° the near wall is seen obliquely. Carton tilts that turn it
  further away lose its tags. The end-wall tags (21/28/22) are usually hidden by the arms and flaps.
- **No real frames yet.** The check has only been tested on renders. Lens distortion, blur and lighting on the
  real OAK are untested; the first real run should be compared once with a tape.
