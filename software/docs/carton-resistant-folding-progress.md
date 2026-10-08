# Empty resistant carton: controller work remains incomplete

Latest work is recorded in the [bracing and physics audit](carton-braced-folding-audit.md).
The ordinary all-inward initialization discussed below was subsequently found
to intersect neighboring flap panels and has been replaced. The upright-short
diagnostic below is distinct and now passes its initial poses explicitly to
the validator, so its report matches its actual initial state.

Neither bare claws nor the held paddle has completed folding the empty,
resistant, freely moving carton. No robot hardware was accessed.

The earlier claw GIF reported success with 960 g of contents, table friction 0.7
and hinge stiffness 0.008 Nm/rad. Its intersecting initial panels invalidate
that success claim independently of the material mismatch. The empty resistant benchmark uses a 272 g carton,
friction 0.35, stiffness 0.018 Nm/rad, hinge friction 0.004 Nm and damping
0.008 Nms/rad. These values remain unmeasured assumptions. The carton has a
six-DOF free joint with no weld, guide or clamp. Both original controllers
fail that benchmark, as recorded in `carton-paddle-comparison.md`.

## What this iteration established

The old approach can contact raised flaps with fingers and forearms before
reaching its intended contact point. Planning whole-arm transit paths avoids
some of those collisions; it does not solve contact manipulation or retention.
The new planner samples joint-space edges and checks the held paddle's sweep
in a separate planning copy. It never moves the physical simulated tool by
resetting its pose. Joint execution retains the original actuator limits and
runtime collision checks. The obstacle snapshot comes from simulator state:
this is not a hardware-calibrated motion planner.

A near-flap-first diagnostic starts both short flaps **upright at 0 degrees**.
The ordinary benchmark starts them 5.73 degrees inward. Those inward short
flaps can obstruct the near panel's corners; the upright diagnostic isolates
that interaction and must not be presented as solving the original start.
All material, station, joint, force and perception gates are unchanged.

| Diagnostic | Observed progress | Stop / failure |
|---|---|---|
| Bare claws | Near flap reaches 89.98 degrees while held | During the second fold it rotates to 124.34 degrees into the empty box. Table-tag registration is lost at 30.954 s. Right short flap is only 42.44 degrees. Maximum box motion 27.44 mm. |
| Left claw and right paddle | Left hand reaches 71.55 degrees on the near flap | Parked paddle grip drifts beyond 15 mm at 15.308 s, with 7.6 degrees rotation. No right-hand fold is attempted. Maximum box motion 11.34 mm. |

The first case shows why one fingertip pressing from above is not reliable
retention over an empty cavity. Updating its target from the visually tracked
box pose did not fix the hold. Alternative side grasps contacted the adjacent
short flap instead of establishing the intended near-flap pinch; those trials
are failures, not grasp validation. Static posture searches also failed to
find collision-free alternatives at the selected near-flap contact points.

The new near-first runner always reports full-task success as false, even
if its two-stage diagnostic finishes. Four-flap closure, withdrawal and
hands-free retention have not been established. Tape application has not
been simulated. No hidden flap constraints, material softening or contents
were added to produce a successful animation.

## Reproduce

From `software`, run each tool with a different new output directory:

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_resistant_folding.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/review/near-first-claws \
  --tool claws --normal tilt --short-along -.11 --end-angle 85 \
  --track-hold --video

PYTHONPATH=. .venv/bin/python tools/diagnose_resistant_folding.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/review/near-first-paddle \
  --tool paddle --normal tilt --short-along -.11 --end-angle 85 \
  --track-hold --video

PYTHONPATH=. .venv/bin/python tools/render_folding_comparison.py \
  --comparison /absolute/path/to/review --case near-first
```

The comparison renderer labels this a partial diagnostic, includes the whole
recorded timeline and holds a stopped run's final state with its stop reason.
The evidence JSON includes source hashes and the unchanged material values.
Fifty focused tests pass, including six new tests for tool sweep collisions,
intermediate path collisions, invalid joint inputs and original joint limits.

Next work is a verified pinch or support transition that retains a flap while
the other arm moves, a grip that retains the paddle throughout the operation,
and a contact path that keeps required tags visible. Final retention must then
be demonstrated after release with an explicit closure mechanism.

[Recorded results](evidence/carton-resistant-folding-progress.json).
