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

## Robot-model station (220 mm, robot cameras) — 8 October 2026, evening

Rebuilt to the XLeRobot model (`upstream/assets/robots/xlerobot/xlerobot.xml`): arm bases 220 mm apart, policy
cameras are the robot's own head camera (`front`, tilt 58°) and the two wrist cameras; the overhead camera is not a
policy input. Station tooling and camera poses: branch `RonTuretzky/fold-policy-station-gap`
(`docs/carton-fold-policy-station-gap.md` §7). The scripted demonstrator succeeds 318/320 there; 287 episodes
(162,574 frames, 3 cameras at 240×320) form `both-shorts-220-v1` (private Hub dataset
`RonTuretzky/carton_both_shorts_220_sim`).

Trained on Hugging Face Jobs (A100 80 GB, `lerobot-train --job.target=a100-large`; 0.069 s/step at batch 16,
about 5× this Mac). Closed loop, temporal ensembling 0.01, held-out starts:

| run | checkpoint | success | carton slide (median / max) | robot–flap penetration |
|---|---|---|---|---|
| batch 32, 15k steps | 5k | 0/31 | 17 / — mm | — |
| batch 32, 15k steps | 15k | 0/16 | 18 / 106 mm | 0.47 mm |
| batch 16, 25k steps | 20k | 8/16 | 14 / 62 mm | 0.60 mm |
| **batch 16, 25k steps** | **25k** | **24/31** | 15 / 82 mm | 1.02 mm |

Further cloud runs (same dataset, scored on the same 31 starts; full log in
`…/fold-evals/scoreboard.jsonl`):

| run | 30k | 35k | 40k | 45k | 50k |
|---|---|---|---|---|---|
| batch 16, 50k steps | 20/31 | 27/31 | 28/31 | **28/31** | 8/31 |

| run | 15k | 20k | 25k |
|---|---|---|---|
| batch 32, learning rate 3e-5 | 27/31 | 6/31 | 25/31 |
| batch 16, seed 2 | — | — | 1/31 |

**Demo model: batch 16, 50k-step run, checkpoint 45k** (`RonTuretzky/act_carton_both_shorts_220_a100_bs16_50k`,
`checkpoints/045000`). 28/31 on the first held-out set (carton slide median 10 mm, max 27 mm; flap penetration
max 0.59 mm) and **31/32** on 32 further held-out starts from a second demo batch (max slide 23 mm, penetration
0.13 mm): **59/63 overall**. Checkpoints swing widely (constant learning rate); the failing ones stall with the right
arm parked while the left braces, so pick by score, never by "last". Two seeds of the same recipe gave 24/31 and 1/31.

All seven 25k failures fold the right short but not the left. Batch 32 at the same learning rate is undertrained.
Cloud cost for these two runs: 58 min, about $2.43. Checkpoints load into lerobot 0.6.1 after dropping the newer
trainer's `dtype: null` config field.

## Limits and next

- The first policy (300 mm, overhead camera) cannot transfer; the 220 mm robot-camera policy is the one to
  carry forward. Its wrist-camera field of view (90°) is assumed and the head tilt chosen in simulation; measure
  both on the robot and re-render if they differ.
- Robot execution path: branch `RonTuretzky/fold-policy-robot-adapter` (`docs/carton-fold-policy-robot.md`)
  has a dry-run-by-default runner and its blockers: per-arm joint zero/sign and jaw mappings are unmeasured,
  the owner cannot stream gripper closures (the policy needs them: 0/3 without, 2/3 with, in simulation),
  and 10 Hz over the current link is unverified.
- Material values (crease stiffness, friction, panel rigidity) are simulation assumptions.
