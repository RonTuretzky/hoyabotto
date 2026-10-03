# Carton closing

A second task for the same robot, separate from the farm: close a filled shipping carton the way the packing-line video shows. Fold the two short end flaps in, fold the two long flaps over so they meet at the centre, tape the seam. No conveyor, no pushing, no next station for now.

Code: `software/carton/` (`carton` command). Shares the farm's devices, skill runner, safety clamps, evidence store, viewer, STOP button and vision-model teaching.

## The box (measured 2026-10-03)

Hachiyo "B" carton, 子持めかぶ 塩分30%カット, marked 40 g × 3+1 × 12.

| | |
|---|---|
| Outer length × width × height | 37.9 × 28.3 × 10.8 cm |
| Flaps | 14 cm, all four |
| Long flaps | 2 × 14 = 28.0 vs 28.3 wide: they meet at the centre seam with a 3 mm gap |
| Short flaps | 2 × 14 = 28 vs 37.9 long: a 9.9 cm opening stays between them (covered by the long flaps) |
| Contents | 24 × 40 g ≈ 0.96 kg; the robot never lifts the box |
| Seal | masking tape for now; a pre-cut strip (about 8 cm) with its tab end free in a printed tape rest |

Photos of the empty carton with flaps standing: `.context/carton-photos/` in the research workspace.

## Reach: why the right arm holds a paddle

The cart stands along one long side. The SO-101 arm reaches about 32 cm from the shoulder with bare fingers (25 cm of arm, 10 cm of gripper, 3 cm margin). The far long flap stands 28 cm plus the cart's setback away: bare fingers reach it only with the shoulders within 3 cm of the near rim. A 15 cm printed paddle held in the right gripper makes the far flap reachable from 18 cm back, and its flat blade presses the tape. `carton geometry` prints the table; with the profile's stance (6 cm setback, 15 cm above the rim) every target is within reach, the tightest being the tape laid by the left arm (6 cm margin).

Two numbers in the profile must be measured on the cart: shoulder spacing (`spacing_m`, assumed 30 cm) and shoulder height above the box rim (`height_m`, depends on table height).

## The plan

Left arm, bare fingers; right arm holding the paddle for the whole job. Order as in the video.

| Step | Arm | Keyframes | Verified by |
|---|---|---|---|
| pick_paddle | right | above rest → grip → carry | paddle_held |
| fold_short_left | left | touch mid-height outside face → done (flat) → rest | short_left folded |
| fold_short_right | right | touch → done → carry | short_right folded |
| fold_long_far | right | touch in line with the shoulder → done (edge at the seam) → carry | long_far folded |
| fold_long_near | left | touch → done → rest | long_near folded |
| tape | left | above tape rest → pinch tab → over seam → down → release | tape_on_seam |
| press | right | near end → far end → rest | tape_pressed |

Every keyframe is taught by the vision model (`carton teach-all`), as in the farm: nobody positions the arms by hand. After each step the head camera is judged (`carton/perception.py`, every field has an unknown). True moves on; false retries once then asks a person; unknown asks a person. STOP wins at any time. Results are written INTENT → ATTEMPT → RESULT like the farm.

## Printed parts

`parts/out/paddle_flap.stl` (210 × 40 × 6 mm: 60 mm handle with the light paddle's grip grooves, 150 mm blade) and `parts/out/tape_rest.stl` (90 × 40 × 18 mm block with a slot the tape tab hangs over). One plate: `~/Downloads/xlerobot-farm-parts/carton/plate_carton_paddle_tape_rest.3mf`.

## Learning

Carton closing is a well-covered task on the Hub. Same arms and joint names as ours, bimanual:

| Dataset | Episodes | Cameras | Licence |
|---|---|---|---|
| yoshikokulala/box_closing3 | 50 | front, top | Apache-2.0 |
| yoshikokulala/box_closing4 | 20 | front, top | Apache-2.0 |
| masato-ka/donuts-shop-close-box-dataset-v0 | 30 | front, back | Apache-2.0 |
| andrejarden/bimanual-close-box | 10 | front, left_wrist, right_wrist | Apache-2.0 |

`carton train` downloads box_closing3 and trains ACT on it (12-joint state and action, matching our two arms). The policy's `front`/`top` views map to our head camera in `profiles/carton-v0.yaml`; a model trained on someone else's box, table and camera placement is a motion prior, not a finished skill. The keyframe path is what closes boxes first; the robot's own recorded runs (`carton run --record`) are what a policy is fine-tuned on later.

## Status

Built and tested on the simulator (11 carton tests; 109 in the suite). Nothing has run on the real robot. Open items: print the paddle and tape rest; measure shoulder spacing and height on the cart; teach the 20 keyframes with a real carton on a table at the chosen height; the download of the training data was slow on the first attempt.

## Commands

```sh
carton geometry [--setback 0.06 --height 0.15]    # reach check, no hardware
carton check                                       # connect and judge the box once
carton teach-all                                   # vision model teaches the 20 keyframes
carton once [--record]                             # close one carton (viewer at :8765)
carton run [--record]                              # close, ask for the next box, repeat
carton sim [--auto-answer] [--faults flap_stuck=fold_long_far judge_unknown no_box]
carton train [--steps 8000]
```
