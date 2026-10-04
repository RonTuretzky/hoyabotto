# R3.2 planter training handoff

Prepared 2026-10-04. Start here on the Mac connected to the robot.

**Status: offline preparation implemented and tested. No R3.2 model training,
physical recording or robot motion was performed by this preparation session.**
The robot is attached to a different Mac. Its latest calibration and print status
have not been read back here; missing development-Mac files say nothing about it.

## What this revision replaces

R3.2 uses a new deep reservoir, an open-front basket, rectangular watering cloth,
a separate grow pad, and an optional narrow four-tab ring. It supersedes the old
R2a assembly design for this task. Do not run R2a carrier poses, watering keyframes,
old G-code or a pouring/box-closing checkpoint as an R3.2 assembly policy.

The printable parts and editable source are in [parts/r32](../parts/r32/).
The basket has a 92 mm open front for an assumed 88 mm cloth. The reservoir is
45 mm high; its integral rests seat the basket at Z=35 mm. The illustration assumes
2 mm watering cloth and a 3 mm grow pad. These are nominal material assumptions.
Actual printed fit, leakage, mat handling and capillary performance remain untested.
`plate.stl` is the combined alternative to the three individual STLs; print one set.

Direct robot pickup from the build plate is an accepted project assumption.
Printer-plate extraction, flexing, support removal equipment, cleaning and return
mechanisms are outside the task. The proposed basket preparation stand, material
feeders and wetting/filling station are not completed hardware designs.

## Delivered software

| File | Purpose |
|---|---|
| `profiles/r32-assembly-v0.json` | Disabled hardware configuration, part hashes, proposed interface and training defaults |
| `farm/assembly/r32.py` | Dry stage ordering, evidence checks, held-object rules and offline failure rehearsal |
| `farm/assembly/r32_data.py` | Recording API, clock/image/action validation, episode provenance and immutable session splits |
| `farm/learning/r32_train.py` | Stage-specific ACT job preparation and optimizer launch after real dataset checks |
| `farm/learning/r32_evaluate.py` | Explicit validation/test selection, frozen-data checks and offline evaluation |
| `tests/test_r32_preparation.py` | Contract, recorder, split, normalization and CLI configuration checks |

[Training instructions](r32-training.md) contain the commands and recorder API.
The supervisor emits intentions only; it is not a motor controller. The recorder
accepts supplied observations/commands; it does not acquire them from the robot.
A live R3.2 observer/executor adapter has not been connected yet.

## Resume without disturbing the robot Mac

1. Let any active calibration finish. Inspect its checkout and preserve its local
   ports, camera mappings, calibration, taught poses and recordings.
2. Bring in the remote changes. If its branch and worktree allow a fast-forward:

   ```sh
   git status --short
   git pull --ff-only origin main
   cd software
   .venv/bin/farm calibration-report
   .venv/bin/python -m farm.assembly.r32
   .venv/bin/python -m pytest -q tests/test_r32_preparation.py
   ```

   If local changes or divergence prevent the update, reconcile them without
   discarding robot-specific work. Do not apply the old transfer patch after
   pulling the same changes. Do not restart calibration merely because this
   handoff lists it as unverified.
3. Follow the repository's existing bring-up procedure for any hardware checks
   not yet evidenced. Preserve wheels-off, motion limits, watchdogs and STOP.
4. Verify the actual R3.2 prints and dock. Measure camera identities, fixture
   frames, gripper thresholds and usable approach/release paths. Never convert
   a Blender pose or plate coordinate directly into a commanded robot pose.

## First engineering and collection milestone

Start with **dry reservoir pickup and placement** (`PLACE_RESERVOIR`). The intended
motion is a side approach to a low fin, pinch, lift, level transfer, stable dock
seating, release and retreat. Prove the real grasp and clearances first.

Connect a separate R3.2 adapter to the existing bounded visual-teaching controller:

1. Supply timestamped observations to the supervisor. A model claiming “done” is
   not an observed grasp, supported part or released jaw. Missing/stale/unknown
   evidence must stop the sequence.
2. Send only validated, bounded actions through the existing runtime. Record the
   action actually sent after clamping. Include commanded gripper state and real
   gripper readback; generic arm settling alone is insufficient.
3. After a failed or uncertain grasp, hold/stop. Do not release, park, drop torque
   or retry blindly with an unknown held object.
4. Wire `r32_data.Recorder` to the real joint/camera/action stream using measured
   runtime limits. Keep the existing robot-driven visual-teaching approach;
   manual teleoperation is not introduced by this handoff.
5. Declare session splits before collection. Preserve failed attempts with their
   outcomes. Rehearsal traces and Blender frames are never physical demonstrations.

After rigid handling works, progress through basket-on-stand, cloth, grow pad,
tail inspection/guidance, optional ring, loaded-basket transfer and dry verification.
The ring may need a controlled release or a different tool because fingers cannot
be assumed to fit beside the seated rim. That motion is not validated. Wetting
and reservoir filling are a separate fixed-station integration.

## Training and evaluation boundary

The initial interface is proposed as the right arm's six joints, right-wrist and
head cameras at 10 Hz. Confirm it before recording. Keep R3.2 data separate from
R2a, pouring and box-closing data.

The launcher requires eligible physical-source records in distinct train, val and
test sessions. It creates a training-only dataset and recomputes normalization
statistics using training episodes only. Timing is retained for audit but excluded
from policy inputs. Evaluation verifies dataset hashes and the checkpoint's saved
training selection before reporting held-out prediction errors.

Without recordings the launcher returns `WAITING_FOR_R3.2_RECORDINGS`, exit 2,
before starting an optimizer. `execution_enabled: false` is mandatory for this
offline preparation profile; flipping it does not create a live executor.

Offline error or rehearsal completion is not physical assembly success. The
connected-Mac milestones are measured setup, integrated observation/control,
real task recordings, offline training, and supervised hardware validation.

## Verification available at handoff

- 29 R3.2 tests passed after the final clock-precision and CLI parsing changes.
- 17 nominal/fault rehearsal cases had their expected complete/stopped outcomes.
- The full suite passed 226 tests before the last two R3.2-specific test additions;
  subsequent final changes were covered by the 29-test R3.2 run.
- LeRobot recorder round-trip, training-only statistics, exact episode selection,
  wrong revisions, synthetic source labels, changed splits and changed payloads
  were checked using isolated test fixtures.
- Installed versions tested: LeRobot 0.6.1, PyTorch 2.11.0, NumPy 2.2.6, pytest 9.1.1.
- No real R3.2 dataset, trained weights, hardware calibration, water trial or crop
  result is included or claimed.
