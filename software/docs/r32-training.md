# R3.2 training preparation

This package is prepared offline. No R3.2 model has been trained, and no robot
commands have run. The R3.2 parts are pinned by SHA-256 in
`profiles/r32-assembly-v0.json`; they match the open-front basket, deeper
reservoir and four-tab retainer shown in the deck.

## Prepared now

- Ordered dry assembly supervisor with fresh evidence checks, held-object state,
  deadlines and failure termination. It emits intent records, not motor commands.
- Offline nominal and fault rehearsals for both retainer variants. Their output
  always has `training_eligible: false` and `physical_success: false`.
- Physical-recording API with one six-joint arm, wrist/head images, actual commanded
  actions and timestamps; malformed, stale, future or missing observations fail.
  Relative per-episode clocks preserve subsecond precision; timing is recorded
  for audit and explicitly excluded from the policy inputs.
- Immutable session assignments and per-episode source, revision, mesh, calibration,
  fixture, tool, camera and outcome metadata. Failures remain recorded but are
  excluded from the initial success-only behavioral-cloning selection.
- Per-stage ACT training launcher and explicit validation/test evaluator. They do
  not connect to hardware. Training first creates a separate dataset containing
  only eligible training episodes, with normalization statistics recomputed from
  those episodes. Validation and test sessions are excluded from optimizer inputs.
- Output/provenance hashes, no silent resume, no Hub publication or W&B reporting.

These checks validate software contracts and file consistency. Metadata is an
attestation by the collector, not automatic proof that a physical event happened.

## Use now: no robot needed

Run from `software/` using the existing environment:

```sh
.venv/bin/python -m farm.assembly.r32
.venv/bin/python -m farm.assembly.r32 --rehearse --out data/r32/rehearsals/no-ring.json
.venv/bin/python -m farm.assembly.r32 --rehearse --with-retainer --out data/r32/rehearsals/with-ring.json
.venv/bin/python -m farm.assembly.r32 --rehearse --fault double_cloth --out data/r32/rehearsals/double-cloth.json
.venv/bin/python -m pytest -q tests/test_r32_preparation.py
```

The fault case intentionally exits 2. Output files are exclusive: choose another
name instead of overwriting an attempt. These rehearsals test ordering and checks,
not reachability, grasp physics, cloth deformation or water transfer.

## First collection task

Start with `PLACE_RESERVOIR`, dry: side-fin grasp, lift, level transfer, stable
seating, verified release. Then prepare basket handling, cloth placement, grow pad,
tail guidance, optional ring, loaded basket transfer and final dry verification.
Wetting/filling is a separate fixed-station integration, not part of this initial
arm policy. The optional ring's release gap is not a measured motion.

Keep the existing robot-driven visual-teaching approach; no manual teleoperation
is introduced here. The R2a executor contract and watering commands have different
parts/stages and must not be used as an R3.2 executor.

## What waits for the connected Mac

Preserve that machine's calibration and local profile. Once its active calibration
finishes, inspect its calibration report, camera identities and timestamps,
measured fixture frames, printed-part fit, actual gripper thresholds, and usable
approach/release paths. Keep the base parked and existing hardware limits intact.
None of those measurements is inferred from a Blender pose or STL coordinate.

A live R3.2 observer/executor adapter still needs to be connected to the existing
bounded visual-teaching controller and STOP/watchdog handling. The preparation
here supplies the state/evidence contract and recorder API, not that hardware
integration. The adapter must retain torque/hold behavior for an uncertain held
object and must never invent success readings or pad missing commands.

## Recording API and session split

Declare the split before each physical collection session. All episodes from the
same setup session use the same split. Reserve distinct validation and test
sessions before fitting a model; do not split adjacent frames or episodes from a
single setup across train and test.

```sh
.venv/bin/python -m farm.assembly.r32_data --session YOUR_SESSION_ID --split train
```

Use `--split val` and `--split test` for later distinct sessions. IDs are append-only;
a declared session cannot be reassigned. No physical sessions are pre-created.
The profile's proposed right-arm/head+wrist interface must be checked on the robot
Mac before collection. If it changes, use a fresh compatible dataset.

The future hardware adapter calls:

```python
from farm.assembly.r32 import load_profile
from farm.assembly.r32_data import Recorder

profile = load_profile()
# Supply limits read from the real runtime profile, not arbitrary permissive values.
rec = Recorder(profile, dataset_root, sessions_file,
               step_max=runtime_limits.step_deg_max,
               watchdog_s=runtime_limits.watchdog_s)
rec.start(measured_episode_metadata)  # see docs/r32-examples/episode.example.json
# RGB uint8 images of exact configured size; Reading timestamps use the same clock.
rec.tick(joint_reading, actual_clamped_action_sent, camera_readings,
         observation_time, action_time)
# End only after real observation checks. Record failed/intervened attempts too.
rec.end(outcome, verified_checks=observed_checks,
        interventions=interventions, safety_events=safety_events)
rec.close()
```

The example metadata intentionally has empty fields and cannot be used as real
training provenance. Fill measured IDs and replace `mesh_sha256` with the profile's
hash mapping. `Recorder.start` sets the actual start time. Do not feed rehearsal
traces, Blender renders or synthetic images into this physical recording path.

## Prepare and launch after recordings exist

```sh
# Prints the exact optimizer command and selected indices; does not train.
.venv/bin/python -m farm.learning.r32_train --stage PLACE_RESERVOIR --output data-train/r32-reservoir-v0
# When dataset validation succeeds, start the offline optimizer.
.venv/bin/python -m farm.learning.r32_train --stage PLACE_RESERVOIR --output data-train/r32-reservoir-v0 --launch
```

With no R3.2 recordings, this reports `WAITING_FOR_R3.2_RECORDINGS` and exits 2.
Defaults: ACT, MPS, 5,000 steps, batch size 8. These are starting experiment settings,
not a promised data requirement or success threshold. Choose a new output name
for each experiment; no checkpoints or training subsets are overwritten.

The frozen selection is `data-train/r32-reservoir-v0.selection.json`; the optimizer
log sits beside it. The training-only dataset is in
`data-train/r32-reservoir-v0.datasets/train`. It reindexes original episode IDs;
the manifest records that mapping and the copied dataset hashes.

```sh
.venv/bin/python -m farm.learning.r32_evaluate --selection data-train/r32-reservoir-v0.selection.json --checkpoint data-train/r32-reservoir-v0 --split val --out data-train/r32-reservoir-v0.val.json
# Use the reserved test sessions after model selection is finished.
.venv/bin/python -m farm.learning.r32_evaluate --selection data-train/r32-reservoir-v0.selection.json --checkpoint data-train/r32-reservoir-v0 --split test --out data-train/r32-reservoir-v0.test.json
```

Evaluation verifies the frozen dataset, split and checkpoint training selection.
It compares predictions with actual recorded commands and a hold-still baseline.
Offline error does not establish physical assembly success. Supervised hardware
validation is still required before autonomous use.

## Transfer

The code and [connected-Mac handoff](r32-handoff.md) are also committed to the
project repository. Prefer a clean fast-forward update that preserves the robot
Mac’s local work. Do not apply the transfer patch after pulling those same changes.

The preparation ZIP includes a patch against the recorded repository base and the
changed/new files. On the other checkout, inspect local changes first and run
`git apply --check r32-training-preparation.patch` before applying the patch.
Preserve robot-specific profiles, calibration and recordings. No robot command,
calibration reset, automatic deployment or file overwrite is part of installation.
