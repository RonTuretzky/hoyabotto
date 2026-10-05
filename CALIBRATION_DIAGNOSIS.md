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

## 3. The unit mix, measured

On `main`, `farm/skills/arm.py` sends IK angles in **degrees** as LeRobot **RANGE_M100_100** values (`use_degrees: False`). The comment says this follows upstream's teleop examples. With this calibration, one normalized unit is 0.92–1.06° (span/200), so the scale error is **up to about 8% per joint**, plus an offset wherever the calibrated mid-range differs from the IK zero pose. It is real, but it is small, and it applies only to Cartesian moves.

Joint-space commands (keyframes, `robot-test`) are not affected.

## 4. Proposed order (for Ron to decide)

1. **First physical motion, today:** on the robot Mac, run `farm calibration-report`, then `farm robot-test`, then `farm robot-test --move --ask`. This is joint-space only: small nudges per joint, not touched by the unit issue. It also covers the "healthy actuators" prerequisite.
2. **Fix the unit mix at its source** instead of disabling motion: either run the arm in `use_degrees: True` (LeRobot DEGREES mode) or convert degrees to normalized units through each joint's calibrated span. Measure the IK-zero offset once. Add a unit test, which is cheap with the calibration file now in the repo.
3. **Then one measured carton milestone:** pick up and put down the paddle once, with the smallest recipe, before adding more gates.
4. If Ron's assistant runs at a high reasoning setting, each step is slow. Routine bring-up commands do not need it.
