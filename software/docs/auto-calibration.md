# Repeating automatic arm calibration

Use the tracked pinned-upstream runner from `software/`:

```sh
.venv/bin/python scripts/carton_robot/upstream_pr3282_calibration.py --arm left
```

That command only prints a plan and checks source hashes. It does not open motors.
To run and install a result **only if validation passes**:

```sh
.venv/bin/python scripts/carton_robot/upstream_pr3282_calibration.py \
  --arm left --execute --clearance-confirmed --install
```

Use `--arm right` for the other arm, sequentially. The default is velocity **300** and
20 seconds per limit-seeking leg. Set `--velocity 200 --timeout 20` to reproduce the
previously successful slower setting, or `--velocity 1000` for the upstream default.
These options change the upstream arguments, not its algorithm. Velocity 300 has
completed full left routines; it has also had lost-feedback failures. It is not a
promise that calibration will succeed with any starting pose or power setup.

The routine is the unchanged PR #3282 source at
`1c8e185e3694def2469d7d312c78dab217e336da`, stored in
`farm/vendor/autocal/upstream_pr3282/`. `provenance.json` and pinned hashes in the runner
verify all three original files. Only driver compatibility, ownership, output paths,
logging, preflight and final cleanup/readback are supplied around the routine. Local
minimum-travel and settling overrides are **not** applied during its sweeps.
The existing `farm calibrate --auto` command uses the separately modified vendored
workflow; it is not this unchanged upstream runner.

## Before motion

Fold and support the arms, establish the intended working front (white-table side on
this robot), clear both sweeps and the tray rim, give camera/servo leads slack, and
keep the 12 V switch reachable. Watch the complete arm from its base to its claw,
including outward/downward travel. If an agent monitors the phone stream, compare
`latest.json`'s `received_at` with current time before and during motion; `live: true`
alone is insufficient. The runner itself does not perform visual collision detection.

The script prompts for `yes` before opening motors. Preflight requires every arm motor
to reply with status 0 and torque off. If the other arm is deliberately unpowered,
physically confirm **its 12 V is OFF** and add `--other-arm-powered-off`. A silent bus
is never inferred to be off. Both serial ports remain exclusively locked throughout
calibration and readback; another chat must wait, even when it controls the other arm.
Override changed port names with `--port-left` and `--port-right`.

## Evidence and installation

Every attempt gets a new directory under `data/calibration_runs/`, or an explicitly
new `--backup-dir`. It contains the prior live file, preflight, `run.log`, staged
upstream candidate, release/readback and `result.json`. The result distinguishes:

- `routine_completed`: the upstream routine returned 0 and produced its candidate;
- `release_verified`: every selected motor replied with torque off;
- `full_range_validated`: six correctly identified joints, position mode/status 0,
  candidate-to-hardware equality, no post-run short/wrapped/too-wide range flags,
  and no excessive travel mismatch against saved opposite-arm ranges;
- `installed`: `--install` was requested and the validated arm was atomically merged.

The range plausibility thresholds are local report heuristics from
`farm/tools/calibration_report.py`, not requirements from the upstream PR. They are
applied **after** it finishes. Physical contact can still resemble a joint stop, so
an acceptable report does not replace observation. Previous verified left travel
was approximately 191/210/194/202/340/136 degrees for pan/shoulder/elbow/wrist/roll/claw.
The repeated 87-degree pan candidate in October 2026 remains unvalidated.

Without `--install`, a candidate stays staged. Installation preserves other-arm,
head and wheel entries and refuses to overwrite a file changed during the run.
Exit 0 means the candidate passed post-run checks; exit 2 means it was retained but
not validated; exit 1 means execution, cleanup or installation failed.

## Failure and starting-pose traps

On missing feedback, stop and verify release. If torque-off cannot be read back,
switch the selected arm's **12 V OFF** immediately; reconnecting USB alone is not a
verified stop. Diagnose repeated bus loss instead of automatically looping powered
retries. An incomplete run may leave offsets, limits and velocity modes changed;
the older live calibration must not be assumed to match hardware. There is no blind
restoration or automatic powered retry.

The PR infers unfold/fold direction from its initial motion. Its fold helper prints
“reached target” after motion stops without checking positional error. In a recorded
short-pan run, elbow and wrist were 19 and 38 degrees short of fold targets. Verify
the actual clearance pose before the pan sweep; the robot has no cart collision model.
Keep restoration and resting-pose movement deliberate and separately verified.

No head, wheel or temperature commands are part of this runner. No hardware was moved
when testing this tracked update; fake-bus tests cover provenance, failure cleanup,
validation, selected-arm installation and preservation of the remaining calibration.
