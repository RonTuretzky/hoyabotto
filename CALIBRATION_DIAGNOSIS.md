# Why the arms are not moving: diagnosis

2026-10-05, Claude. Evidence: Ron's branch `codex/robot-calibration-snapshot` (calibration export) and `codex/carton-local-program` (latest carton work), read without checking them out. Nothing in the repo was changed.

## 1. The calibration is fine

`farm calibration-report` on the pushed `farm_xlerobot.json` (SHA-256 matches its README) prints **LOOKS COMPLETE**:

| Joint | Left | Right |
|---|---|---|
| Shoulder pan | 191° | 206° |
| Shoulder lift | 210° | 211° |
| Elbow | 194° | 184° |
| Wrist flex | 202° | 201° |
| Wrist roll | 340° | 336° |
| Gripper | 136° | 133° |

The profile id `farm_xlerobot` matches the file name, so the farm program finds it.

The head, calibrated by hand, is narrow: pan 187° (expected 240°) and **tilt 64°** (expected 85°). It is not flagged, but it limits how far the head can look down. Our OAK mount analysis assumed −40°…+80° of tilt; the real range is about 64° in total.

My earlier hypotheses (wrist roll with no stop, velocity versus timeout) are ruled out for this robot: the wrist rolls stopped at about 340°.

## 2. Why nothing moves

On `codex/carton-local-program`, `docs/carton.md` now says that legacy physical Cartesian/LLM teaching "now refuses because its geometric and normalized motor units were mixed".

`docs/carton-local-program.md` says the replacement "still needs the owner integration, measured station recipe, working camera streams and healthy actuators before this can run physically. No real grasp, lift or fold has been verified."

So the old way to move the arms has been switched off, and the new way needs four things that do not exist yet. The branch adds about 7,100 lines of gates and tests in 12 commits since Oct 4, but no physical motion.

## 3. The unit mix, and what Ron's branch already fixes

On `main`, `farm/skills/arm.py` sends IK angles in **degrees** as LeRobot **RANGE_M100_100** values (`use_degrees: False`). With this calibration one unit is 0.92–1.06° (span/200), so the scale error is up to about 8% per joint, plus an offset. Joint-space commands (keyframes, `robot-test`) are not affected.

There is a second bug on `main`. The vendored `SO101Kinematics` IK and FK disagree: IK → FK misses by up to about 6 cm. `SkillRunner` re-estimates the Cartesian pose with FK on every joint read.

**Both are already handled on `codex/carton-local-program`** (commit `1d76688`, Oct 4):
- IK was corrected to match FK, and it now raises on unreachable targets instead of clamping. The test `test_real_inverse_forward_regressions` asserts an exact round trip.
- `carton/servo` adds `JointUnits` (ticks ↔ normalized ↔ model degrees) with a **measured model zero tick and sign per joint**.
- Legacy Cartesian moves stay refused on the real robot until those are measured.

Correction (Claude, later the same day): an earlier version of this note proposed a new unit fix. I prototyped one, fixing FK rather than IK. Applied on top of `codex/carton-local-program` it broke 8 of that branch's tests, because the two fixes cancel. It was not pushed. The branch's approach is the one to keep.

## 4. Proposed order (for Ron to decide)

1. **First physical motion, today:** on the robot Mac, run `farm calibration-report`, then `farm robot-test`, then `farm robot-test --move --ask`. This is joint-space only and is not affected by either bug. It also covers the "healthy actuators" prerequisite.
2. **Measure the per-joint model zero tick and sign** that `JointUnits` requires, once, with the arm in the model's zero pose. This is what unblocks the Cartesian path on the branch.
3. **Merge** `codex/carton-local-program` into `main` when ready. Note that it already conflicts with `main` in `.gitignore` and `farm/oak_camera.py`, because the OAK commit exists on both with different hashes.
4. **Then one measured carton milestone:** pick up and put down the paddle once, with the smallest recipe, before adding more gates.
5. If Ron's assistant runs at a high reasoning setting, each step is slow. Routine bring-up commands do not need it.
