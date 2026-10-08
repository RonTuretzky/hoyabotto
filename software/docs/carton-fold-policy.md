# Learned short-flap fold policy (ACT, simulation) — 8 October 2026

An ACT network (LeRobot 0.6.1) folds **both short flaps** of the simulated carton with the two claws and
holds them, using only two camera images and the 12 joint angles. It was trained by imitation on
demonstrations from the scripted vision-in-the-loop controller
([carton-sim-training-handoff-2026-10-08.md](carton-sim-training-handoff-2026-10-08.md)).
**Simulation only. Nothing has run on the robot.** The station is the original simulated one (bases 300 mm
apart, overhead `top` camera), which the robot model does not match; see "Next" below.

## Result

Closed loop in MuJoCo from 32 held-out starts (carton offset x −25…+5 mm, yaw ±4°, crease stiffness
0.012–0.030 N·m/rad, never seen in training). Success = both short flaps ≥ 80° for 3 s, scored from the
physics, not from commands.

| checkpoint | execution | success | carton slide (median / max) | robot–flap penetration (max) |
|---|---|---|---|---|
| 5k | chunks of 10 | 0/10 | — | — |
| 10k | chunks of 10 | 4/10 | — | 0.34 mm |
| 15k | chunks of 10 | 0/10 | — | — |
| 15k | temporal ensembling 0.01 | 12/16 | — / 40 mm | — |
| **25k** | **temporal ensembling 0.01** | **30/32** | 11 / 24 mm | 0.20 mm |
| 30k | temporal ensembling 0.01 | 29/32 | 13 / 27 mm | 0.34 mm |

Run it with temporal ensembling (re-plan every 0.1 s, average overlapping chunks); executing 10-step chunks
open loop fails. Replaying the recorded commands through the same harness reproduces 10/10 demonstrations,
so the harness is not the source of the difference. The scripted demonstrations slide the carton 1–3 mm;
the policy slides it more but stays within the table and never presses the robot into anything but flaps.

## Pipeline

| step | tool | notes |
|---|---|---|
| record | `tools/record_fold_demos.py` | runs the unchanged controller per trial; samples qpos/qvel and the 12 commanded targets every 0.1 s; stops 3 s after both shorts are folded (~55 s sim, ~16 s wall). 304/320 succeed at the ranges above; positive x or larger yaw makes the left pregrasp unreachable |
| dataset | `tools/fold_demos_to_lerobot.py` | renders `top` (overhead) and `front` at 240×320; state = joints, action = next commanded target; seed % 10 held out. 272 episodes, 150,360 frames |
| train | `python -m farm.learning.train --policy act --batch-size 16 --steps 30000 -- --policy.chunk_size=30 --policy.n_action_steps=10` | M4 Max MPS, 174 min, final loss 0.037 |
| evaluate | `tools/eval_fold_policy.py --temporal-ensemble .01` | `--replay` checks the harness; `--videos N` writes presentation MP4s |

Artifacts (local, not in git): demos `…/Hackatuson/output/fold-demos/batch-01`, dataset
`…/fold-datasets/both-shorts-v1`, checkpoints `…/fold-train/act-both-shorts-v1/checkpoints/025000`,
evaluations and videos `…/fold-evals/act-v1-0*`.

## Limits and next

- Simulated station and cameras: the robot model (`upstream/assets/robots/xlerobot/xlerobot.xml`) puts the
  arm bases 220 mm apart and has head and wrist cameras, no overhead camera. The policy above cannot transfer
  as is. Next: rebuild the sim to the robot model, record at 220 mm, train on head + wrist cameras
  (branch `RonTuretzky/fold-policy-station-gap`).
- Robot execution path: branch `RonTuretzky/fold-policy-robot-adapter` (`docs/carton-fold-policy-robot.md`)
  has a dry-run-by-default runner and its blockers: per-arm joint zero/sign and jaw mappings are unmeasured,
  the owner cannot stream gripper closures (the policy needs them: 0/3 without, 2/3 with, in simulation),
  and 10 Hz over the current link is unverified.
- Material values (crease stiffness, friction, panel rigidity) are simulation assumptions.
