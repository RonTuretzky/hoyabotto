# Carton four-flap simulation: handoff for training — 8 October 2026

For a thread that will train a policy from these simulations. Everything here
is **simulation only**; nothing has run on the robot.

## What exists

A scripted, vision-in-the-loop controller that folds an empty 379 × 283 ×
108 mm carton with the two bare SO101 claws in MuJoCo: both shorts first, then
the far and near majors, ending with all four flaps closed and **held by both
jaws** (no tape, no hands-free retention). It is an expert/demonstrator, not a
learned policy; no neural network has been trained on it.

- Branch `RonTuretzky/carton-handoff-pick-up`, head `ddc802c` (pushed; not on
  `main`). Sequence and rules: `docs/carton-four-flap-shorts-first.md`.
- Stage code: `carton/folding_majors_over_shorts.py` (after the existing
  open-claw short hold in `carton/folding_retention.py`). Runner:
  `tools/diagnose_short_flap_brace.py`; parallel sweeps: `tools/run_claw_sweep.py`.
- Sequence: left claw braces the left short → right claw folds the right
  short → left presses the left short → right claw opens across both shorts
  (open-claw hold) → left drags the far major's top edge to 34° (regrip/hook
  if it slips) → right claw releases the shorts → **left keeps the far major**
  and advances it to 70° → right pushes the near major to 88° → left closes the
  far major to 88°. The left arm must not let go of the far flap: with 2×
  stiffer creases the release variant closes 0/20.

## Success rates (latest code, `ddc802c`)

| batch (under `/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/shorts-first/`) | crease stiffness | four flaps closed and held |
|---|---|---|
| `hold04-k018` | 0.018 N·m/rad (assumed) | **20/30** |
| `hold04-k036` | 0.036 (2×) | **10/20** |
| `four-flap-close-28` (release variant, older code) | 0.018 | 24/30 |

Successful trials are labelled by `result.json → majors_over_shorts →
four_flaps_closed_and_held: true`. Failed trials keep `error` and the stage
reached. All successes pass the independent applied-contact audit
(`tools/score_folding_contacts.py --run <trial>/run` → `CONTACT_ONLY_CLEAR`).
An experiment with 0.5° drag leads (`hold05-*`) scored 0 and was reverted.

## Running it

Environment (no venv in Conductor worktrees):

```sh
cd <worktree>/software
PY=/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/.venv/bin/python
PYTHONPATH=. $PY tools/run_claw_sweep.py \
  --simulation-root /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot \
  --out /Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/shorts-first/<new-batch> \
  --workers 4 --seeds 0 1 2 3 \
  --near-targets -15 --support-heights .113 --close-majors-after-open-claw \
  --majors-far-target 34 --base-height .12 --far-open-degrees 1.0 \
  --extra-wall-markers --pinch-clearance -.0045 [--hinge-stiffness .018] [--hinge-friction .004] [--video]
```

- About 70–150 s wall time per trial; at most 4 workers (handoff rule).
  Background shell jobs here are killed after 1–2 h, so split large sweeps.
- Different seeds change camera noise and therefore registration and contact
  choices; `--carton-offset-x` shifts the carton.
- `--release-far` runs the older let-go variant for comparison.
- Crease rest angle (to model pre-folded cartons that settle partway closed)
  exists as `CartonMaterial.hinge_rest_degrees` but is not yet a CLI option.

## Data per trial (`<batch>/trial-NNN/run/`)

- `folding-frames.json`: list of `{time, label, qpos}` at roughly 7 states/s
  (about 1,000 per run). `qpos` has 23 values:
  - `[0:6]` left arm: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper (radians);
  - `[6:12]` right arm, same order;
  - `[12:19]` carton free joint (xyz, quaternion wxyz);
  - `[19:23]` flap hinges: short_left, short_right, long_far, long_near (radians; 0 = upright, π/2 = closed).
- `folding.json → events`: one per commanded motion (`label`, duration, joint
  or Cartesian targets `targets_m`, tracking errors, contacts, penetration,
  flap extrema, carton motion). Commanded joint targets are not stored
  separately; derive actions from consecutive `qpos` or from the stage
  report's command lists.
- `result.json`: configuration, stage, error, every perception `readings`
  entry (carton pose from markers, flap angles with their method, tags), and
  `majors_over_shorts` (per-stage command lists such as `far_pin_commands`,
  `far_hold_commands`, `near_commands`, `far_commands`, contact choices,
  angle sources).
- `applied-contact-steps.jsonl.gz`: per-solver-step contacts and wrenches.
- `scene.xml`: the exact MuJoCo scene, so any state can be re-rendered from any
  camera. Images are not stored unless `--video`; render frames with
  `tools/render_run_gif.py RUN OUT.gif 4 12`, or adapt it for training images.

## Caveats before training anything for the robot

1. **The simulated station does not match the real one.** Live check on 8
   October (read-only, over the LAN bridge): the real OAK-D looks closely down
   at a small patch of a small folding table, and the arm bases sit on the
   cart well above the tabletop. The simulation assumes a calibrated overhead
   camera at about 0.85 m, bases 120 mm above and 150 mm behind the table
   edge, a large table, and extra printed carton markers (IDs 26–28). A policy
   trained on these scenes will not transfer until the scene is rebuilt from
   measured station geometry and camera pose
   (`docs/carton-real-station-measurements.md`).
2. **Material values are assumptions**: crease stiffness, friction, rigid
   3 mm panels, empty 272 g carton, table friction 0.35. The owner plans to
   pre-fold cartons, which lowers stiffness and adds crease memory; measure
   it with a luggage scale and set `--hinge-stiffness`, `--hinge-friction`
   (and the rest angle once exposed).
3. **The demonstrator uses privileged planning.** Collision checks and contact
   searches run on copies of the simulator state (true flap and carton poses);
   the controller's targets come from vision, but its safety pre-checks do not.
   For imitation learning, prefer observations a robot has: camera images,
   joint encoders, and the perception readings in `result.json`.
4. **Real execution path does not exist.** The robot API (direct-joint owner,
   12 arm joints, mTLS) accepts joint-tick targets; there is no adapter from
   this controller or from a policy to it yet, and no collision certificate.

## Pre-folded (soft) creases

Pre-folding should make creases softer and friction-dominated. Modelled as
stiffness 0.006 N·m/rad (a third of the assumption, friction unchanged), both
variants fail **40/40 at the first stage** (`prefold-release`, `prefold-hold`):
the right short folds past flat and sags to about 101° into the empty carton,
because the flap's own weight (about 0.016 N·m) beats the soft spring plus
friction (about 0.013 N·m), and the original fold step accepts only 80–101°.
A real empty, pre-folded carton will likely do the same: harmless for closing
(the majors still go on top) but the short-flap stages and their closure
bands assume flat shorts and need adapting for that case.
`--hinge-rest-degrees` is now a CLI option, but it pulls all four flaps
toward that angle from the start, so it models a different presentation.

## Remaining failures (latest code)

Penetration over 1 mm during the far-edge drag (5/30, 6/20 at 2×), the near
flap brushing the right forearm while the shorts are held (2/30), shorts
pressed past 110° in the final hold (2/30), and occasional lost far-flap
observations. See `docs/carton-four-flap-shorts-first.md` for causes and the
fixes that worked.
