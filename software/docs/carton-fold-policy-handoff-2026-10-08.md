# Carton fold policy: handoff for the robot — 8 October 2026, late

For the person or session taking the learned short-flap fold to the real robot. The step-by-step slides are
`docs/carton-fold-policy-setup-slides.html` (open in a browser; arrows to move). This page is the checklist and the
numbers. **Nothing below has moved a motor yet.** Safety rules from `CLAUDE.md` apply throughout: STOP in reach,
12 V switch in reach, nobody in the arms' sweep, never relax limits.

## Where things stand

| Item | State |
|---|---|
| Policy | ACT, trained only in simulation, folds both short flaps and holds them: **63/63 unseen starts** (carton moves ≤6 mm) |
| Checkpoint | Hub (private) `RonTuretzky/act_carton_both_shorts_220_bs32_lr3e5_chunk100`, `checkpoints/015000`; locally `/Users/wk/Documents/ChatGPT/Hackatuson/output/fold-train/act_carton_both_shorts_220_bs32_lr3e5_chunk100/checkpoints/015000`. Runner-up: `…_a100_bs16_50k` `045000` (59/63) |
| Inputs | head OAK as `front` (4:3, 240×320 after resize), `left_wrist`, `right_wrist`, 12 joints in SO-101 URDF radians |
| Run it | temporal ensembling 0.01 (re-plan every 0.1 s over 10 s action chunks), 10 Hz |
| Station it learned | robot-model layout: arm bases 220 mm apart, 120 mm above the carton surface, base line 150 mm from the table edge |
| Arm joint maps | **accepted by the owner, not measured**: `profiles/fold-joint-maps/` |
| Head camera pose | **unknown**: the twin's head mapping was off by ~15° tilt and ~17° pan on 8 Oct; measure it with tags |
| Robot runner | `carton/fold_policy_runner.py`, dry-run unless `--execute`; tests `tests/test_fold_policy_runner.py` |
| Demo video (simulation) | `.context/demo-videos/DEMO-BEST-chunk100-15k-seed3010.mp4` in the las-vegas-v1 workspace |

## 1. Joint maps (done, accepted)

- Arms use the digital twin's mapping: **zero = middle of each saved range (tick 2047 for every arm joint today), sign +1**.
  The owner accepted it on 8 Oct after a live overlay that agreed roughly. It is recorded as accepted, not measured.
- Jaw: saved `range_min` = closed = URDF −10°, 360/4096° per tick (twin convention, assumed).
- The maps are tied to the calibration read live on 8 Oct (`calibration-2026-10-08.json`). On the robot Mac, bind them
  to the live file. The tool refuses if any arm motor's homing offset or range differs, meaning the arms were recalibrated:

```sh
cd software
python tools/bind_fold_joint_maps.py --calibration <live saved calibration .json> --out bound-maps/
python -c "from carton.fold_policy_runner import load_arm_maps; \
  load_arm_maps({'left':'bound-maps/left-joint-map.json','right':'bound-maps/right-joint-map.json'})"
```

## 2. Hand poses

All hand posing is done with the arms **released** (torque off) and supported. No motor is enabled for any of these.

### 2a. Training start pose (needed for every run)

The policy only ever started from this pose. Pose both arms by hand until the live ticks are within about ±30 ticks
(±3°) of the table, reading them with `robot_get_state` (read-only). The runner warns `start_outside_training`
until they are.

| Joint | Angle (URDF) | Left arm tick | Right arm tick |
|---|---|---|---|
| shoulder_pan | ∓31° (left −31.3, right +31.1) | **1691** | **2401** |
| shoulder_lift | −100° (folded down, near its lower stop) | **909** | **909** |
| elbow_flex | +30.7° | **2396** | **2396** |
| wrist_flex | +95° (bent up) | **3128** | **3128** |
| wrist_roll | left 0.3°, right 85.9° | **2050** | **3024** |
| gripper | closed | **≈1276** (closed) | **≈1272** (closed) |

Picture: `docs/img/fold-policy-setup/training-start-pose.png`. Shoulders folded low, wrists bent up, jaws closed,
both claws parked either side of the carton's near wall; the right wrist is rolled about 86°.

### 2b. Optional: confirm the arm maps (3 overlay poses)

Skippable, because the owner accepted the maps. If you want the evidence later, do it after step 3, sleeves off:
move the released arms by hand into three different poses with both grippers in the head camera's view, e.g. elbows
bent, wrists turned, jaws open. At each one, the agent reads the ticks and one frame (read-only) and overlays the twin
(`/Users/wk/Documents/ChatGPT/Hackatuson/output/robot-readonly/twin_overlay.py`).

## 3. Station and cameras (owner, slides steps 1–6)

1. **Arm spacing.** Measure the two shoulder-pan axes; the policy assumes base origins 220 mm apart.
2. **Height.** Arm-base undersides **12 cm above the carton surface**. With the 70 cm table: measure the floor →
   base underside. About 82 cm means the table is right; about 77.5 cm means it is about 4.5 cm too high.
3. **Setback and carton.** 111 mm from each pan axis to the table edge, cart square; carton near wall 10 mm from the
   edge, centre 10 mm to the robot's left, square within 4°; empty, flaps up, short flaps facing the arms.
4. **Head camera.**
   - Set it to roughly 58° down, pan 0; the twin's head angles are wrong, so use the inclinometer.
   - Read the lens with `python tools/oak_intrinsics.py --width 640 --height 480 --out oak-640x480.json` (Terminal, camera only).
   - Lay the four tags from `docs/img/fold-policy-setup/fold-pose-tags-40-43.pdf` and save one 640×480 still.
   - Solve the pose with `tools/camera_pose_from_tag.py` (slide 6c).
5. **Wrist cameras.** Checkerboard calibration with `tools/calibrate_camera_checkerboard.py`; the simulation assumed 90°.
6. **Sleeves.** The real arms wear black sleeves; the simulated arms are bare. Take them off for runs, or the simulation
   is recoloured to match before retraining.

Send the measurements and files. The agent re-renders the demonstrations with the measured cameras and retrains on the
cloud GPU (about 1–2 h; around $2–3). It re-records the demonstrations only if the layout can't match.

## 4. Robot software (agent, then owner review)

- **Gripper streaming.** The owner refuses jaw closures during streamed motion, and the policy needs them (simulation:
  0/3 folds without, 2/3 with). This needs a server change, reviewed and deployed by the owner, then a free-air jaw test.
- **Link timing.** `tools/measure_robot_link.py --pilot-root "$PILOT"` (read-only). Four calls must fit in 0.1 s,
  otherwise run the runner on the robot Mac (`--transport direct-client`).
- **Joint speed.** The demonstrations move up to ~59°/s; the owner may cap joints near 9°/s. Confirm the cap, then
  slow the demonstrations (retrain) or raise the cap with the owner's approval.

## 5. Runs (owner holds STOP)

1. **Dry run, nothing sent.** Arms released at the training start pose (2a), no carton:
   ```sh
   python -m carton.fold_policy_runner --checkpoint <ckpt> --transport api --pilot-root "$PILOT" \
     --joint-map left=bound-maps/left-joint-map.json --joint-map right=bound-maps/right-joint-map.json \
     --camera front=<head OAK> --camera left_wrist=<cam> --camera right_wrist=<cam> --out <run> --max-steps 100
   ```
   Check the joint angles against the table, no start-pose warning, small sensible actions, and about 10 Hz.
2. **Free air, 10 steps.** Enable the arms (they hold), add `--execute --operator <name> --max-steps 10`, no
   carton. The run ends holding; support the arms, then `robot_stop`.
3. **Empty carton.** Only after sections 3–4 are closed and the policy has been retrained for the measured station:
   first enough steps for one short flap, then both.

Stop and press STOP if an arm presses on the table, cart or the other arm, the carton slides more than a few
centimetres or lifts, a joint stalls against a load, or anything moves unexpectedly. Record every run in
`software/STATUS.md` with its run directory.

## Files

| Item | Location |
|---|---|
| Slides | `docs/carton-fold-policy-setup-slides.html` |
| Results | `docs/carton-fold-policy.md` |
| Station and cameras | `docs/carton-fold-policy-station-gap.md` |
| Robot path and runner | `docs/carton-fold-policy-robot.md` |
| Joint maps | `profiles/fold-joint-maps/` (+ `tools/bind_fold_joint_maps.py`) |
| Printables | tags `docs/img/fold-policy-setup/fold-pose-tags-40-43.pdf`, checkerboard `fold-checkerboard-9x6-25mm.pdf` |
| Read-only robot scripts and the 8 Oct frames | `/Users/wk/Documents/ChatGPT/Hackatuson/output/robot-readonly/` |
| Training pipeline | `tools/record_fold_demos.py`, `tools/record_measured_fold_demos.py`, `tools/fold_demos_to_lerobot.py`, `tools/eval_fold_policy.py`, cloud training with `lerobot-train --job.target=a100-large` |
