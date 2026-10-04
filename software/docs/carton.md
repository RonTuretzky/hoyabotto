# Carton closing

**Robot laptop: start with [the October 4 handoff](carton-connected-mac.md)** for the trained
checkpoint download, checksums, station setup and remaining physical-integration work.

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
| Seal | dispenser-cut masking tape; exposed end for direct pickup, adhesive down; actual strip length/width to be recorded |

Photos of the empty carton with flaps standing: `.context/carton-photos/` in the research workspace.

## Reach: why the right arm holds a paddle

The cart stands along one long side. The simplified reach calculation uses about 32 cm from the shoulder with bare fingers (25 cm of arm, 10 cm of gripper, 3 cm margin), plus 15 cm for the right-hand paddle. `carton geometry` prints shoulder-to-target distances. The current profile starts at 6 cm setback and shoulders level with the rim. This calculation does not check joint limits, collision, contact forces or the tool pickup positions; its pass is not physical validation.

The upstream robot model (simulation/mujoco/xlerobot.xml, URDF) places the arm bases 30 cm apart and shoulders about 82 cm above the floor. Confirm both on the assembled robot. Set `height_m` to measured shoulder height minus tabletop height minus 10.8 cm, in metres, for every station. For the model estimate and a 70 cm table this is 0.012 m; at 80 cm it is -0.088 m.

## The plan

Left arm, bare fingers; right arm holding the paddle for the whole job. Order as in the video.

| Step | Arm | Keyframes | Verified by |
|---|---|---|---|
| pick_paddle | right | above rest → grip → carry | paddle_held |
| fold_short_left | left | touch mid-height outside face → done (flat) → rest | short_left folded |
| fold_short_right | right | touch → done → carry | short_right folded |
| fold_long_far | right | touch in line with the shoulder → done (edge at the seam) → carry | long_far folded |
| fold_long_near | left | touch → done → rest | long_near folded |
| tape | left | approach dispenser → pinch exposed end → lift clear → over seam → supported placement → release → retract | staged grip/clearance/orientation checks, then tape_on_seam |
| press | right | near end → far end → rest | tape_pressed |

Ordinary keyframes are taught by the vision model; tape poses require the whole
[`carton tape-test --teach` sequence](carton-tape.md), preserving adhesive-down orientation without any flip/turnover.
Nobody positions the arms by hand. Tape uses both head and left-wrist checks between phases and
stops on false/unknown without an automatic retry or release. Other steps use the existing head-camera
judgement/retry behavior. STOP ends motion; results use INTENT → ATTEMPT → RESULT.

## Printed parts

The owner has ordered a dispenser; its exact model and pickup geometry are pending. The printed tape rest is now optional legacy hardware. No tape flip/turnover will be taught.

Tracked meshes: [parts/carton](../parts/carton/README.md), `paddle_flap.stl` (210 × 40 × 6 mm: 60 mm handle, 150 mm blade) and `tape_rest.stl` (90 × 40 × 18 mm block). Slice on the intended printer; fit/grip and physical execution are unverified. Development-Mac print preparations under `~/Downloads/` do not transfer with Git.

## Learning

Carton closing is a well-covered task on the Hub. Same arms and joint names as ours, bimanual:

| Dataset | Episodes | Cameras | Licence |
|---|---|---|---|
| yoshikokulala/box_closing3 | 50 | front, top | Apache-2.0 |
| yoshikokulala/box_closing4 | 20 | front, top | Apache-2.0 |
| masato-ka/donuts-shop-close-box-dataset-v0 | 30 | front, back | Apache-2.0 |
| andrejarden/bimanual-close-box | 10 | front, left_wrist, right_wrist | Apache-2.0 |

`carton train` downloads box_closing3 and trains ACT (12-joint state and action). The October 3 run completed 8,000 steps, final loss 0.293. The saved model loads; see the handoff for evaluation limits. Real-profile `front`/`top` inputs now name unconfigured `policy_front`/`policy_top` views instead of duplicating `head`. Joint order, normalization and camera setup require validation. The current carton cycle runs taught keyframes, not this checkpoint. Robot-specific recordings can support later fine-tuning; neither path has closed a real box here.

## Status

The full suite passed 167 tests on October 4. Training and saved-model inference checks are complete; physical carton execution is unverified. See the connected-Mac handoff for station setup, calibration readback, print/grip checks and specific teaching/ACT integration blockers before movement.

## Commands

```sh
carton geometry [--setback 0.06 --height 0.15]    # reach check, no hardware
carton check                                       # connect and judge the box once
carton teach-all                                   # ordinary poses; tape uses separate complete trial
carton tape-test --plan                            # show tape stages without hardware
carton tape-test -p profiles/carton-local.yaml --teach  # LIVE: learn direct dispenser pickup/place on a closed box
carton tape-test -p profiles/carton-local.yaml      # LIVE: repeat with a fresh strip
carton once [--record]                             # close one carton (viewer at :8765)
carton run [--record]                              # close, ask for the next box, repeat
carton sim [--auto-answer] [--faults flap_stuck=fold_long_far judge_unknown no_box]
carton train [--steps 8000]
```
