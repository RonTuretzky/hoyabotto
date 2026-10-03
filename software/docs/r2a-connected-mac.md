# R2a continuation on the robot's Mac

October 3, 2026. The user reports calibration running on the other Mac. Let that process finish before
updating the checkout or starting another robot command. Do not restart calibration just because the
development Mac has no calibration file.

The active print is `cress_R2a_full_prototype_plate.gcode`: fused carrier, optional retaining frame, grip
coupon. It reuses the printed original trough. Completion and usable parts have not been confirmed here.

## After the current calibration finishes

1. Save its result. Run `farm calibration-report` on that Mac; it reads files only. Resolve any flagged
   range before motions. `farm r2a` prints the calibration id needed for station measurement provenance.
2. With no robot process running, bring this repository's changes over without discarding local profile
   or calibration work. Inspect `git status` first. Use `git pull --ff-only` if the checkout permits it;
   otherwise reconcile changes while preserving the local ports and camera mapping.
3. From `software/` with the existing environment active, run `python -m pytest -q` and `farm r2a`.
   The latter lists file-level blockers without opening any device. The local calibration file remains
   on this Mac; do not replace it with a development or simulator file.
4. Continue the repository's `farm-bringup` skill at the post-calibration checks: motors-only readback,
   supervised joint identity checks, then camera identity/freshness verification. Preserve the wheels-off
   setting and existing motion, watchdog, temperature and load limits.

## Physical setup for the assembly milestone

- Inspect the completed carrier, coupon and frame after supports/brim are removed. Confirm actual fit on
  the existing trough, intact pockets, fin strength and clearance. Record print instance and modifications.
- Fix the empty trough against sliding/rotation with its refill bay identified. The existing generic nest
  remains an unverified candidate. A fixture does not have to be printed, but must be measured and tested.
- Define fixed source locations for the wick-loaded carrier, one real top sheet, and optional frame.
  Paper pickup and separation must be tested independently; plastic pockets do not replace paper wicks.
- Measure empty/holding jaw thresholds on the coupon, actual carrier, frame and paper over a catch area.
  The profile needs actual measurements, date and operator. The bottle thresholds are unrelated.
- Keep the setup dry for R2a-D0/D1. Water transfer and growing performance are separate later checks.

## Measure the station instead of typing a guessed transform

Copy `profiles/r2a-station-measurements.example.yaml` to `data/r2a/measurements.yaml`. Fill in at least four
paired landmarks with broad two-dimensional spread. Coordinates on the fixture are **assembly-frame
millimetres**; corresponding calibrated robot/TCP coordinates are **metres**. Use a fifth independent
landmark, at least 10 mm from every fitted point, for the check. Preserve the measurement method and the
calibration id reported on this Mac. No rendered poses, STL plate offsets or slicer coordinates belong here.

```sh
farm r2a-station --measurements data/r2a/measurements.yaml --output data/r2a/station.yaml
farm r2a
```

The fit refuses inconsistent measurements, nearly collinear/tightly clustered points, errors above 2 mm,
and overwriting an earlier station file. A small residual proves internal consistency only, not reachability,
clearance or TCP accuracy. Re-measure after moving the fixture or changing robot calibration.

## Remaining software integration, before any live assembly

The contract and discrete rehearsal exist; a live R2a executor and observer do not yet exist. Build them
against measured hardware, using the rehearsal's checks as acceptance requirements:

1. Teach poses with the existing vision-servo approach, without hand teleoperation. Store only R2a poses
   in `data/r2a/keyframes.yaml`: `r2a_<carrier|paper|retainer>_<approach|grasp|lift|transfer|seat|retreat>`
   and `r2a_safe`. `farm r2a` lists which are absent; never substitute watering keyframes.
2. Wire bounded motions through `SkillRunner` and live STOP/watchdog/health checks. Verify the actual
   gripper reaches the requested close/open state: the current generic `move_joints` excludes grippers
   from its settling test, so its success result alone is insufficient for a grasp/release.
3. Implement timestamped object observations from the wrist/head views plus sensor evidence. Model `done`
   is not a verified grasp. Check held state after lift and transfer; for paper determine sheet count
   before transfer; require support before release and clear jaws before retreat. UNKNOWN stops and holds.
4. On failure, record the attempt and stop/hold. No automatic release, parking, torque-off or insertion retry.
   Do not disconnect a real held-part run through a cleanup path that drops torque. A supervised recovery
   needs the actual held state and a validated path.
5. Connect `R2aRecorder`/`StrictTick` to real commanded actions, observations and camera timestamps. Preserve
   failed episodes with their outcomes. Declare session splits before training. Synthetic rehearsal traces
   are not demonstrations. The old pouring checkpoint is incompatible with the R2a camera contract.

Before live use, test the hardware integration with fake adapters, then validate each primitive under
supervision on this robot. Keep `execution_enabled: false` until those prerequisites are evidenced.

## Offline checks available now

```sh
farm r2a --simulate
farm r2a --simulate --variant R2a_with_frame
farm r2a --simulate --fault double_paper
```

The last command intentionally exits 2 with `SIMULATED_STOPPED`. Traces go to `data/r2a/rehearsals/`.
They exercise supervisor logic only: no camera model, collision model, contact physics or robot motion.

Done means actual carrier/frame handling for R2a-D0, and verified placement of the prepared carrier,
real top sheet and declared optional frame for R2a-D1. Passing tests or finishing calibration alone is not
either milestone.
