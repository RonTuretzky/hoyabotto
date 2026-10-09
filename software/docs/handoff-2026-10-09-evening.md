# Handoff — 9 October 2026, evening

> **SUPERSEDED for station setup, camera geometry and next actions:**
> [Final DCM desk station-refit handoff](handoff-2026-10-09-station-refit.md).
> Preserve this page as the smoke-test history. Do not follow its 45 cm cart move, infer calibrated
> camera-to-arm translation from box-only tags, or rely on its 15.4 mm camera-clearance replay
> (incorrect camera-box orientation). The new preview uses colliding camera CAD and the confirmed
> 500 × 480 × 700 mm rectangular desk. Neither a wedge nor training has been started in this refit.
> The 775 mm legacy frame origin is not the mounting plane; demo reach is not the full reach envelope.

State of the carton fold policy after today's first real-robot smoke test, with absolute paths. Branch
`RonTuretzky/fold-box-tags` at `f6b0196`, pushed, **not merged to main**. Nothing below has folded a real carton.

Repo checkout used for everything here:
`/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/`
(the pilot session's scratch checkout of the same branch: `/tmp/foldtags/software/`).

## 1. What the smoke test proved (pilot session seville-v2-34, owner at STOP)

The whole chain runs end to end on the real robot:

| Step | Result |
|---|---|
| Joint maps bound to the live calibration | OK (`/tmp/bound-maps/`; calibration identical to the 8 Oct snapshot) |
| Both arms to the training start pose (`tools/move_to_start_pose.py --execute`) | reached, 11 legs; right wrist_flex stops 40 ticks short at its mechanical stop (see §3) |
| Policy dry run, 28 ticks (`carton.fold_policy_runner --transport api-stream`) | reads 3 cameras + joints, infers, clamps, logs; no refusal |
| Stream mode on the deployed owner | live (main `d55e081`) |

Two findings from the dry run (`/tmp/fold-dry/`):

- **The policy is off-distribution on today's view.** Its first raw target was shoulder_lift **+73°** from the
  start pose; on a correct observation (same checkpoint, 220 mm simulation) the first target is a 6° nudge and the
  demos hold the lift still for the first second. The 40-tick step clamp is all that makes it look sane. Cause:
  head at 35° (trained 58°), 16:9 stream (trained 4:3), cart ~45 cm too far from the carton. **Do not execute
  until the first raw tick looks like the simulation's (lift ≈ −107°).**
- **Timing:** 0.19–0.31 s per tick on the LAN (two round trips + inference) against the 0.3 s tick limit, which
  by design can only be tightened. Run at **`--hz 5`**: temporal ensembling, one inference per tick, so the fold
  executes at half wall-clock speed, coherently. Do not drop cameras; the checkpoint requires all three.

## 2. The real blockers and their fixes

| Blocker | Measured | Fix | State |
|---|---|---|---|
| Cart too far from the table | lens 0.73–0.79 m from the near wall (tag sizes); training ≈ 0.30 m | move the cart in ~45 cm: pan axes 111 mm from the table edge (setup step 3) | **owner, not done** |
| Head cannot tilt to 58° | 35.1° at the servo's commandable max (tags) / 32° (table plane) | **print the 23° wedge cradle** (§4): servo max → 58.6°, keeps the trained view, **no retrain** | STL ready, not printed |
| 16:9 OAK stream vs 4:3 training | 39° vertical FOV vs 54° | after the wedge + cart: either crop to the trained FOV in the runner, or re-film the demos from the measured camera (`tools/refit_fold_policy.py`, ~3 h, ~$2.40) | decide after re-measuring |
| Camera pose unknown relative to the arms | dry run used box tags only | re-run with the gripper tags up: `tools/auto_head_pose.py --box-tags` with arms at `--print-arm-pose` | after the two above |

Order: cart → wedge → head pose → station check → dry run → compare first tick with sim → execute.

## 3. Code changes today (all on the branch)

- `software/carton/fold_start_pose.py`: a joint whose target is within 8 ticks of its commandable limit and that
  stops short **toward** that limit by ≤ 60 ticks is accepted and reported (`stopped_short_at_mechanical_stop`),
  never resent. Everything else aborts as before; tolerances stay un-relaxable. Tests:
  `software/tests/test_move_to_start_pose.py` (23 pass). The pilot's local `ARM_TOLERANCE_TICKS` 30→45 edit in
  `/tmp/foldtags` must be reverted.
- `software/carton/station_check.py`: reads `tools/auto_head_pose.py` output directly (`8be2931`).
- Full suite: 112 passed, 2 opt-in skipped.

## 4. The wedge cradle (print this)

- **STL (print orientation, standing on its end wall):**
  `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/parts/head/oak_d_lite_wedge23_cradle.stl`
- README with the why, fitting steps and the servo→angle table:
  `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/parts/head/README-wedge.md`
- CAD source / report / STEP: `make_oak_wedge_cradle.py`, `geometry-report-wedge.json`,
  `oak_d_lite_wedge23_cradle.step` in the same folder; previews in `parts/head/preview/wedge_*.{png,jpg}`.
- Verified in CAD: one watertight solid, 50.5 cm³ (~63 g PLA); zero intersection with the tilt link, the camera
  body, a 20 mm USB-C plug and the four M3 shanks. Same tongue, same four M3 × 10 screws, same snap-in box as the
  stock cradle. Wedge direction verified three ways (the other sign swings the box into the link).
- Print: 0.2 mm layers, 4 walls, 30 % infill, PLA/PETG, standing on its −X end wall (95.5 mm tall; 1.4 cm² of
  overhang on the plate edges, light support or none). Not physically tested: **watch the first head motion with
  a hand on STOP.** The camera can no longer look level or up (4°–58.6° down range), which the fold never needs.

## 5. Measurement artefacts from today (scratch, not in git)

- Head pose dry run: `/tmp/head-pose-dry/auto-head-pose.json` (+ `frames/`, `station-measurement.json`)
- Station check: `/tmp/station-check/report.json`, camera `/tmp/station-check/head-camera.json`
- Policy dry run: `/tmp/fold-dry/preflight.json`, `ticks.jsonl`, `summary.json`
- Bound maps: `/tmp/bound-maps/{left,right}-joint-map.json`; live calibration `/tmp/live-cal.json`

## 6. Slides and docs

- Setup deck (single file, everything embedded, 12 MB):
  `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/docs/carton-fold-policy-setup-slides-standalone.html`
  (source `carton-fold-policy-setup-slides.html`; rebuild with `python3 software/tools/bundle_slides.py <source>`)
- Box tags, print at 100 %: `.../software/docs/img/fold-policy-setup/fold-box-tags.pdf`
- Chat-Mac integration handoff: `.../software/docs/carton-fold-policy-chat-mac-handoff.md`
- Automation docs: `auto-head-pose.md`, `auto-station-check.md`, `auto-wrist-lens.md`, `auto-start-pose.md`,
  `auto-refit.md` in `.../software/docs/`

## 7. Open decisions for the owner

1. Print the wedge (recommended; avoids a retrain) vs retrain for 35° (`refit_fold_policy.py --launch`, ~$2.40,
   ~3 h; only after the cart is moved, since it films from the measured camera).
2. Merge `RonTuretzky/fold-box-tags` to main (everything above; tests green).

## Wrist camera body: not in the simulation (measured 9 Oct)

The simulated arms carry **no camera geometry**: the gripper links are the bare SO-101 meshes. The real arms carry
the wrist camera module on each fixed jaw: **36 × 43 × 67 mm**, extending **65 mm along the jaw** beyond the pad
and 13 mm sideways (upstream model `Left_Arm_Camera`/`Right_Arm_Camera` meshes in the `Fixed_Jaw` frame). The
policy trained with claws that are physically smaller than the real ones, and the fold is contact (the claws press
the flaps: closest gripper-to-flap approach in the demos is 0 mm).

Checked by replaying 6 demos (816 sampled steps) with a module-sized box attached to each simulated gripper at the
model's position: the module **never enters** the carton or a flap; its **nearest approach to any carton surface is
15.4 mm**. So the unmodelled body clears the trained motion, with a small margin (the carton spot tolerance alone is
±3 mm). It is still a gap: if the policy ever pushes with the jaw's outer face, or the real module sits further out
than the model's, the box would meet the flap first. The 9 Oct real run never reached the carton, so this has not
been observed on hardware. The wrist-lens tool (`tools/capture_wrist_calibration.py`) plans its poses with 3 cm
clearance for the same reason.

**CORRECTION, 9 Oct evening (owner's observation + phone recording `pilot/recordings/20261009-173405-manual-phone.mp4`,
run 5):** on the real robot the RIGHT wrist camera module **did contact and push the carton**. The claw descended onto
the box's right rim at ~31 s and from ~54 s the module/jaw bore on the rim and right flap; the carton shifted over the
following minute; the right_wrist stream shows the lens against the rim edge. The converged "claw at the rim" pose
was the module resting on the carton, not a hover. The replay above (0 intrusion) checked the TRAINED motion; the
robot was running an off-distribution pose the demos never produce, and that pose reaches the rim with the module.
**The module must be in the collision model and the policy's approach from now on.** The policy is not to be run
again until it is.
