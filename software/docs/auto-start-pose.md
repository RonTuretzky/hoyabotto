# Automated move to the fold policy's training start pose

**Status, 9 October 2026: software only.** Tested against the deployed owner code on fake servos (`FakeOwner`) and in
the MuJoCo fold training scene. It has **never moved a real motor**. The rules in `CLAUDE.md` apply: STOP and the
12 V switch within reach, nobody in the arms' sweep, never relax limits.

This replaces setup step 8b / handoff §2a, where the owner poses the released arms by hand until every joint reads
within about ±30 ticks of the start-pose table. The tool moves the enabled arms there itself.

- Code: `carton/fold_start_pose.py` (start pose, collision check, planner, mover) and `tools/move_to_start_pose.py` (CLI).
- Chat tool: `robot_fold_policy_start_pose` in `carton/fold_policy_chat.py`.
- Tests: `tests/test_move_to_start_pose.py`, plus the start-pose cases in `tests/test_fold_policy_chat.py`.

## 1. The target pose

The policy only ever started from one pose. All 640 recorded demonstrations (`demo.npz` `qpos[0]`) and frame 0 of all
287 episodes of the training dataset `both-shorts-220-v1` hold the same 12 angles. The tool converts them to ticks
through the arms' joint maps (`ArmMap.rad_to_ticks`, the runner's own code), so the ticks follow the maps if the maps
change. With `profiles/fold-joint-maps/`:

| Joint | URDF angle | Left tick | Right tick |
|---|---|---|---|
| shoulder_pan | −31.27° / +31.08° | 1691 | 2401 |
| shoulder_lift | −100.00° | 909 | 909 |
| elbow_flex | +30.71° | 2396 | 2396 |
| wrist_flex | +95.00° | 3128 | 3128 |
| wrist_roll | +0.33° / +85.94° | 2051 | 3025 |
| gripper | −9.74° (pads meet) | 1358 | 1354 |

The two roll values differ by one tick from the handoff table, which truncated them instead of rounding.

The pose source can be chosen:

- built in (the default);
- `--demo <trial dir>`: that demonstration's `qpos[0]`;
- `--dataset <LeRobot dir>`: frame 0 of every episode, which must all agree.

`--checkpoint <pretrained_model>` also refuses if the pose lies outside the checkpoint's own training state range. The
tool prints the pose in radians, degrees and ticks, then a per-joint table of current against target ticks.

## 2. What it does

```
robot_get_execution (read) -> start pose in ticks -> plan -> MuJoCo clearance check of every leg
  -> per leg: re-read the owner, re-check the leg from the measured position,
     robot_move_joint_targets(arm, positions, duration_s, wait=true, replace=false)
  -> final read: residual per joint, "at training start pose"
```

**Plan.** Only one arm moves at a time; the other holds still. Each checked configuration is therefore one the robot
actually passes through. The phases run in this order:

1. **raise**: shoulder_lift, elbow_flex and wrist_flex go to the clearance pose, with pan and roll unchanged. By
   default the clearance pose is the start pose's own lift, elbow and wrist values. The start pose is the high tuck:
   the jaws sit about 0.35 m above the table, well above the carton rim.
2. **pan_roll**: shoulder_pan and wrist_roll go to the start pose.
3. **descend**: from the clearance pose to the start pose. With the default clearance pose this phase is empty.
4. **jaw**: each gripper moves **alone and blocking**, last. The owner closes a jaw 10 ticks per 1.5 s, so the plan
   prints the expected time.

For each phase and arm, several joint orderings are tried: all together, wrist first, lift first, one joint at a time,
and so on. Both arm orders are tried too. The first ordering that is clear everywhere is used. If none is clear, the
tool **refuses** and prints what blocked each attempt.

**Owner limits.** The tool uses the owner's limits and only ever tightens them:

- every target lies 40 ticks inside the saved range;
- a leg moves each joint at most 280 ticks (the owner's own segment size) and drops joints under 3 ticks;
- legs run at 60 ticks/s by default (about 5°/s). `--speed-ticks-s` can only lower that; the maximum is 100, which is
  the owner's ramp of 40 ticks per 0.4 s;
- `duration_s` is at most 25 s.

**Collision check.** The check uses the MuJoCo fold training scene, with the carton in its nominal spot and its flaps
up. A configuration is clear when no moving arm geometry comes within 15 mm of:

- the station: table, cart, mounts, head neck;
- the carton;
- the other arm.

`--clearance-mm` can raise the 15 mm but not lower it below 10. Arm bases are fixed and skipped. Pairs inside one arm
are not checked. Each leg is sampled every 4 ticks along the straight line in tick space, which is how the owner ramps
the joints of one move together. The present configuration must itself be clear.

**Checks before execution.** `--execute --operator NAME` refuses unless all of the following hold:

- the owner is `paddle-success-v1` and healthy;
- the owner's ranges match the joint maps' calibration;
- all 12 arm motors are enabled and holding;
- the owner is idle or holding;
- the scene is loaded;
- the whole plan is clear.

**During execution.** Before each leg the tool:

- runs the stop hook (SIGINT or SIGTERM, or the parent process exiting);
- re-reads the owner, which must show the same `started`, the same `stop_count`, healthy, holding, all enabled and
  status 0;
- checks that the arms are within ±30 ticks of where the plan expects;
- re-checks the leg from the measured position.

An arm leg must end `completed`. `settled_short` or `contact_halt` aborts the run, holding, with no retry. The tool
never calls `robot_stop`, `robot_halt_motion` or `robot_set_motor_enable`, and it never resends a move.

**Jaws.** A jaw that stops short (`settled_short`, `stationary_closure_unverified` or `contact_halt`) is **reported and
never resent**. The right jaw stalls mid-travel in free air today, and a resent open trips the owner's no-progress
guard. An owner fault aborts the run. `--no-jaws` leaves the jaws where they are.

**End.** The tool prints `at training start pose` and the residual per joint. Arm joints must be within ±30 ticks;
jaws within 30 ticks of the pads-meet point. The arms are left **holding**.

The owner's idle lease releases them after about 120 s unless the fold run, or an explicit enable, comes next.

**Exit codes.**

| Code | Meaning |
|---|---|
| 0 | Dry run done, or at the start pose |
| 2 | Arms at the start pose, but a jaw stopped short |
| 1 | Refused or aborted |

## 3. Commands

On the chat Mac (`$PILOT` = the pilot checkout; bind the joint maps to the live calibration first, handoff §1):

```sh
cd software
# dry run: reads the owner, prints current vs target ticks and the checked plan; sends nothing
python tools/move_to_start_pose.py --pilot-root "$PILOT" \
  --joint-map left=bound-maps/left-joint-map.json --joint-map right=bound-maps/right-joint-map.json \
  --scene /Users/wk/Documents/ChatGPT/Hackatuson/output/fold-demos/batch-220-01/trial-020 \
  --checkpoint <ckpt>/pretrained_model --out runs/start-pose-dry
# execute: the operator has enabled both arms (they hold) and is at STOP
python tools/move_to_start_pose.py ...same... --execute --operator Ron --out runs/start-pose
```

To see it run in simulation, use the deployed owner code over the MuJoCo fold scene from a random clear start. No robot
is involved:

```sh
python tools/move_to_start_pose.py --sim <fold-demos trial dir> --sim-start random:3 --execute --operator sim
```

For each run, the simulation prints:

- owner faults;
- `stop_count`;
- robot–carton and other penetration;
- how far the carton moved.

In the tests these are none, 0, 0 mm and 0 mm, with under 3 mm of carton settling.

From the chat, `robot_fold_policy_start_pose {}` is a dry run. `{execute: true, operator}` executes, and is gated by
`.private/fold-policy.json`:

- `start_pose_execute_enabled: true`;
- the operator is listed in `operators`;
- no `start_pose_blockers`;
- `start_pose_scene` is set.

These settings are separate from the fold run's `execute_enabled` and `blockers`. Re-run the installer once so that
`MOVE_TOOLS` lists the new tool.

## 4. Tests (no robot)

`tests/test_move_to_start_pose.py`, run with the deployed owner over a kinematic plant (fast) and the MuJoCo fold scene
when it is present. The tests check that:

- the built-in pose equals the demos' `qpos[0]`, the dataset's frame 0 and the handoff ticks through the released
  maps, and lies inside the checkpoint's training range;
- limits cannot be relaxed;
- a dry run makes no motion call and no bus write;
- execution reaches the pose from random starts: 4 on the kinematic rig, 3 in MuJoCo, and 1 with the robot's real
  maps and saved ranges. In every case:
  - each joint ends within ±30 ticks;
  - no owner fault, `stop_count` 0, ends holding;
  - one arm per call, jaws alone and last;
  - in MuJoCo, no robot–carton or robot–station penetration;
- released arms are refused;
- a colliding path is refused with nothing sent, and so is a start already inside the carton;
- an operator STOP injected after leg 2 aborts the run with no further move and no re-enable;
- the stop hook ends the run before the next leg, releasing nothing;
- a stalled jaw is reported and sent once, and a jaw fault aborts the run;
- the CLI simulation mode works.

## 5. Limitations

- **The check is against the training scene, not the measured station.** Arm spacing, base height, table and carton
  placement on the real station may differ (`carton-fold-policy-station-gap.md`). The owner's contact guard is still
  active, and the operator keeps STOP.
- **The joint maps are owner-accepted, not measured.** If a map is wrong, the checked path and the real path differ
  by that error.
- **Path model.** The owner spreads shorter joints over the same number of steps with rounding, so the real path
  deviates slightly from the straight line. The 15 mm clearance covers this; it is not modelled exactly.
- **Self-collision inside one arm is not checked.** The joint ranges are trusted for that.
- **The arms must start clear of the carton as modelled, with its flaps up.** After a fold, with the flaps held
  folded, the tool refuses. Lift the carton out and use `--carton absent` (the operator's statement), or move by hand.
- **Settling under load.** If gravity holds a joint more than 30 ticks off, the run ends "outside the arrival
  tolerance", holding. There is no extra overdrive beyond the owner's own bounded corrections.
- **SIGINT stops before the next leg; it does not interrupt the leg in progress.** Use `robot_stop` to stop
  immediately; it releases every motor.
- **Plain blocking moves only.** Stream mode is deployed (main `d55e081`), but this tool does not use it.
