# Two-arm carton folding, without the paddle

Exploration started 2026-10-06. **Update:** the earlier tag/depth folding passes are unvalidated after an initial-panel intersection was found; the [corrected bracing audit](carton-braced-folding-audit.md) records incomplete attempts; this page records the initial research and compatibility checks. Initial scope is **closing the four top flaps of
the already-formed carton**, using both grippers. Erecting a flat blank is a
different contact task. Tape, lifting the box and conveyor handling are outside
this first experiment. The existing paddle workflow remains available.

## What we can reuse

| Source | Verified evidence | Application here |
| --- | --- | --- |
| Existing local MolmoAct2 experiment | `output/molmoact2-mac-sim/mac_policy.py` loads the pinned SO100/101 checkpoint on MPS and requires 6 state/action channels. One selected adapted paddle trial succeeded; the three completed factorial follow-ups inspected here failed. | Reuse model loading, image preprocessing, adaptation and logging. Paddle success is not a box-folding policy. Keep its active experiment separate. |
| [MolmoAct2](https://github.com/allenai/molmoact2) | SO100/101 and BimanualYAM checkpoints are separately released. Actual normalizers contain 6 and 14 action channels respectively; the YAM profile expects top/left/right images. | Train an explicit 12-channel dual-SO101 policy. The model's padded 32-channel capacity is not a trained dual-arm mapping. Two independent single-arm calls do not learn coordination. |
| [ABC box-folding checkpoint](https://github.com/amazon-far/abc/blob/d0832d12651d1b260a652861a14648dc5f3660c7/deploy/README.md#dagger-checkpoints) | The released `folding_paper_box` metadata specifies a 14-action YAM policy, 75,000 training steps and an 8.063 GB checkpoint. Metadata was fetched; weights were not downloaded or tested. | Closest task-specific learned reference. Its arm geometry, initialization and camera setup do not transfer directly to XLeRobot. Use demonstrations/task decomposition or retargeted reachable tool poses, never drop two joint channels. |
| [ABC project](https://abc-robot-dataset.github.io/) | Reports box-folding **mean progress**, rising from 24% after task fine-tuning to 85% following corrective-intervention training. This is not an 85% full-success claim. | Evidence that two-hand folding is feasible, and that fine contact corrections matter. This does not establish a demonstration-free transfer to our station. |
| [Existing box-closing data](https://huggingface.co/datasets/yoshikokulala/box_closing3) | Fresh metadata: 50 episodes, `bi_so100_follower`, 12 left-six/right-six outputs, distinct front/top RGB views. Our released ACT checkpoint already uses this layout. | Closest embodiment baseline; compare ACT against adapted Molmo on the same box task. The source's position units/zeros and physical scene still need auditing. |
| Existing farm code | `carton.geometry.Box/Stance`, verified SO101 URDF/LeRobot FK, AprilTag registration and the existing single motor owner. | Reuse geometry and registration. Extend coordinated control through that owner after validation; do not start independent serial owners for the two arms. |

The [ABC data card](https://huggingface.co/datasets/XDOF/ABC-130k) describes
two 6-DoF YAM arms, three cameras, joint/end-effector data and optional subtask
annotations. The task metadata request returned HTTP 401 in this session, so
we have not inspected the gated box trajectories. The published code/checkpoint
metadata remain useful. The released simulation catalogue inspected here does
not establish an existing cardboard-folding environment.

## First experiment

Use one policy observation containing both arms and output one coordinated
12-value action at each step: five positioning joints plus gripper per arm.
Preserve robot-left/robot-right, explicit units, calibration hashes and paired
timestamps throughout. Our new offline boundary rejects 6-D/14-D chunks, swapped
arm order, missing units and accidental reshaping.

The proposed contact sequence is:

1. Left folds its short flap; right braces the carton.
2. Right folds its short flap; left retains the first flap.
3. Right folds the far long flap; left keeps the short flaps from springing up.
4. Left folds the near long flap; right retains the far flap.
5. Both press and hold; score closure, then test release retention separately.

This sequence specifies the task and evaluation milestones, not saved motor
poses. It needs a coupled policy: one hand's useful action depends on the other
hand's contact and the flap state. Successful closure under hand pressure is
distinct from an untaped box remaining shut after both hands withdraw.

## Reach result that changes the setup

The saved carton is 379×283×108 mm with 140 mm flaps. Reusing the saved 60 mm
shoulder setback and zero shoulder-height offset above the rim, a half-height
contact on the far flap is about **350 mm** from its shoulder. The existing
arm/fingertip approximation allows **321 mm** after its 30 mm margin. Bare
grippers therefore fail this coarse screen by **29 mm**.

At a 20 mm assumed setback, that screen has about 10 mm remaining margin.
This suggests evaluating a closer *station layout* first. It is not an
instruction to drive the cart closer: these are saved/assumed dimensions, and
the table, cart and both arm swept volumes must fit. Passing a spherical reach
test does not prove orientation reachability or collision clearance.

## Implementation and next checks

The initial exploration implemented `carton.bimanual`, `tools/explore_bimanual_carton.py`, boundary
tests and an evidence report. These perform **offline geometry and model-schema
checks only**. They do not fold a simulated or physical carton, run a learned
box policy, or change the active Molmo experiment.

```sh
cd /path/to/xlerobot-farm/software
PYTHONPATH=. .venv/bin/python tools/explore_bimanual_carton.py \
  --profile profiles/carton-v0.yaml --out /new/path/bimanual-reach.json
.venv/bin/python -m pytest -q tests/test_carton_bimanual.py
```

Original follow-up plan (simulation items now have results in the linked guide):

1. **Dual-arm contact scene:** import both actual SO101 models; use a common
   table frame, measured base separation, carton hinges, flap thickness and
   springback. Test bracing and full swept-volume clearance. Avoid direct
   simulator flap actuators, attachments or hidden object poses in the policy.
2. **Comparable baselines:** run the existing bimanual ACT and a geometric
   contact teacher separately. Keep teacher-driven folding, policy-driven
   folding and simulated grasp results distinct.
3. **Molmo adaptation:** reuse the local loader but replace its six-channel
   boundary with an explicitly trained 12-channel normalization/data profile.
   Use existing bimanual demonstrations and validated simulation/recovery data
   first, consistent with the preference to avoid manual teleoperation. Test
   new carton poses and seeds, with stuck-flap and one-arm-disabled controls.
4. **Physical observability:** register both bases and fingertips; validate
   distinct camera views. The current tag 2 tracks only the right gripper.
   Fiducial corrections for both hands need a separate left-gripper identity
   (for example tag 4) and observable carton/hinge geometry. A marker on the
   paddle does not observe the carton flaps.
5. **Owner integration:** validate synchronized two-arm setpoints, fresh feedback,
   cross-arm collision checks and existing STOP behavior before supervised
   physical folding. No controller limits or existing calibration are relaxed.

The [research evidence](evidence/carton-bimanual-exploration.json) records source
revisions, checkpoint metadata and the reach/compatibility results. No physical
motor commands or additional model training were issued in this exploration.
