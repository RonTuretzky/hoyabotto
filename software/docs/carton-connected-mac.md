# Carton task: handoff to the Mac connected to the robot

**For the current no-demonstration route, start with
[measured visual control](carton-visual-controller.md) and its
[upstream comparison](carton-upstream-review.md).** The checkpoint/teaching
workflow below is historical context. Legacy physical Cartesian/LLM teaching
now refuses because its geometric and normalized motor units were mixed.
The new route first commissions local alignment; grasping/folding remain unvalidated.

Updated October 4, 2026. Start here for **box closing**, separate from watering and R2a planter assembly.
The immediate goal is to install the published checkpoint, verify it without motors, preserve the
robot's existing calibration, and establish the real station. Physical carton execution is unvalidated.

## What is ready, and what is not

| Item | Verified on development Mac |
|---|---|
| Source | `carton/`, profiles, simulator, vision teaching and keyframe cycle are in this repo |
| Tests | 167 passed on October 4; component/simulator coverage, not physical tests |
| ACT training | 8,000 steps, batch 8, MPS; completed October 3 at 18:23 JST; final loss 0.293 |
| Saved inference files | Loaded on MPS; action shape `(12,)`, chunk shape `(100, 12)` |
| Recorded-image check | 27 frames from episodes 47–49: MAE 3.888, hold-current-state baseline 1.831 |
| Generalization | **Not established.** Those episodes were in the training dataset; no held-out evaluation |
| Robot, calibration, cameras | Must read back on this Mac; development-Mac checks cannot confirm these |
| Physical closing/taping | **Not demonstrated**, including paddle grip and tape pickup |

Two same-seed training processes wrote the same output directory. Both exited successfully. We tested
the final saved inference files and fixed their identity with hashes; this does not reconstruct an
isolated training run. The release excludes optimizer state and must not be treated as a resume bundle.
Full measured results and dependency versions: [verification JSON](evidence/carton-act-8000-check.json).

## 1. Update without losing this Mac's work

From the existing `xlerobot-farm` checkout:

```sh
git status --short --branch
git fetch origin
git log --oneline HEAD..origin/main
```

Inspect local changes first, especially calibration/profile edits from bring-up. Do not reset, clean,
overwrite, or stash an active robot session automatically. If on a clean `main`, use:

```sh
git pull --ff-only origin main
cd software
. .venv/bin/activate
uv pip install -e '.[dev]' -c constraints-carton.txt
python -m pytest -q
```

If the checkout or environment does not exist, follow [SETUP.md](../SETUP.md), using the constraints
above for the software installation. Reuse the existing Claude login and local secrets. Do not copy
the development Mac's `.env`, calibration, databases, or simulated poses. Carton work does not require
the ESP32/light sensor. The separate GPU server is still deferred; this Mac runs local inference.

## 2. Download and verify the checkpoint (no robot connection)

The model is too large for normal Git. It is attached to the existing repository's
[carton ACT release](https://github.com/RonTuretzky/xlerobot-farm/releases/tag/carton-act-8000-2026-10-04).
From `software/`:

```sh
mkdir -p data-train
curl -fL --retry 3 \
  https://github.com/RonTuretzky/xlerobot-farm/releases/download/carton-act-8000-2026-10-04/carton-act-8000.zip \
  -o data-train/carton-act-8000.zip
cd data-train
shasum -a 256 -c ../docs/evidence/carton-release-SHA256SUMS
unzip -n carton-act-8000.zip
cd carton-act-8000
shasum -a 256 -c SHA256SUMS
cd ../..
python -m farm.learning.infer data-train/carton-act-8000/pretrained_model --device mps
```

Do not continue past a checksum failure. `unzip -n` preserves existing files; if verification fails
for an existing extraction, preserve it and extract into a fresh directory instead of overwriting it.
Expected: state/action dimensions 12, cameras `front` and `top`, action `(12,)`, chunk `(100, 12)`.
This is a synthetic inference smoke check; it neither opens motor ports nor closes a box.
CPU inference is available with `--device cpu`; do not assume the same speed on another Mac.

Model provenance: [yoshikokulala/box_closing3](https://huggingface.co/datasets/yoshikokulala/box_closing3),
50 episodes, dataset robot type `bi_so100_follower`, front/top RGB at 640×480. Dataset metadata declares
Apache-2.0. It is other people's robot/station data, not demonstrations of our carton, paddle or tape.

## 3. Set up the fixed station

Left/right below always mean the robot's perspective.

```text
                    FAR LONG FLAP
                 +-------------------+
                 |                   |
      28.3 cm    |       CARTON      |
                 |                   |
                 +-------------------+
                       37.9 cm
                    NEAR LONG FLAP

        tape dispenser                    paddle pickup
           LEFT ARM                  RIGHT ARM
                         ROBOT
```

- Box: 37.9 × 28.3 × 10.8 cm, 14 cm flaps, contents about 0.96 kg. No lifting, conveyor or pushing.
- Face the long side; centre the box between the shoulders. Mark the outline and use low stops
  against the bottom walls so it does not slide. Keep all flaps free and the cart chocked.
- Start with a rigid table around 70 cm high. The model estimates shoulder height at 82 cm;
  a 70 cm table puts the rim at 80.8 cm. **Measure the assembled robot**, do not assume those match.
- The profile starts at 30 cm shoulder spacing and 6 cm setback. Setback is the horizontal distance
  from the shoulder joint centres to the near box wall, not from the cart front. Check table/cart clearance.
- Set `carton.stance.height_m` to **actual shoulder height minus actual tabletop height minus 0.108**,
  all in metres. For the estimated 82/70 cm heights this is `0.012`, not `0.15`.
- Right arm holds the printed paddle; left arm uses its fingers. Fix the paddle pickup and dispenser
  within their respective arms' reach, outside flap travel. Tool pickup positions are not in the reach checker.
- The owner ordered the [LUKDOF M1000, ASIN B0DQY77P16](https://www.amazon.co.jp/dp/B0DQY77P16).
  Its listing specifies 20–999 mm cuts and 7–50 mm tape width. Start at **80 mm**, measure the actual
  cut and record tape width. Face the front outlet toward the left gripper, with the unit outside flap travel.
  Present a fully cut strip at a repeatable exposed-end pickup point, **adhesive down**; verify that
  orientation and jaw clearance on the delivered unit. No flip/turnover training or folded tab is planned;
  the printed tape rest is optional. Test grip, release and closure strength as in [carton-tape.md](carton-tape.md).
  Use manual mode with automatic refill disabled and verify strip removal does not restart the mechanism.
  Keep the dispenser stopped during initial pickup trials. This software does not
  control the dispenser or replenish strips; unattended replenishment still needs model-specific integration.
- Use even lighting; the head camera must see all four flaps, and wrist cameras must see their grippers.
  Keep cables clear. Start with an empty carton and isolated movements before filled/taped cycles.

Print source meshes are now in [parts/carton](../parts/carton/README.md), available through `git pull`.
Printing, strength, fit, grip and tape release remain physical checks. Do not copy unfinished G-code
from another session. The geometry command is only a shoulder-to-target distance calculation,
not an IK, joint-limit, collision, force or contact check.

## 4. Preserve calibration; identify devices

Only one process may own the motor boards. If calibration or another robot command is running,
let that session finish and record its result before connecting anything else.

Back up this Mac's existing calibration and taught data locally before changes. Use the current
working calibration file and `robot.id`/`calibration_dir` from its verified robot profile. Do not
overwrite them with empty files from a fresh checkout, or rerun auto-calibration just to start cartons.

From the terminal app with macOS camera permission:

```sh
farm devices --probe
```

Read the camera snapshots under `data/devices/`. Verify port1 has left-arm IDs 1–6 plus head 7/8;
port2 has right-arm IDs 1–6 plus wheel 9/10. Keep wheels disabled.

Create a local profile (do this once; preserve it on later updates):

```sh
cp -n profiles/carton-v0.yaml profiles/carton-local.yaml
```

Fill its `robot.port1`, `robot.port2`, verified `robot.id`/`calibration_dir`, and each camera's
`index_or_path` from observed device identities. Retain `simulated: false`, `wheels: false`,
`light.enabled: false`, `policy.enabled: false`, and `data_dir: data-carton`.
Set the measured stance from section 3. This local profile is gitignored.

```sh
farm calibration-report -p profiles/carton-local.yaml
carton geometry -p profiles/carton-local.yaml
```

These two commands read files/calculate only. A reach pass does not authorize an unchecked movement.
If calibration is absent or flagged, resolve bring-up using `STATUS.md` and the existing `/farm-bringup`
workflow. No hand teaching or teleoperation: preserve the user's preference. Head calibration may
still need a physical setup step; record it as a remaining requirement if incomplete.

Next, with the arms supported at a stable rest pose and power cutoff accessible:

```sh
farm robot-test -p profiles/carton-local.yaml
```

Even without `--move`, connection configures motors and can toggle torque; disconnect makes arms limp.
It is not electrically read-only. Only after a valid report and clear workspace, the existing
`farm robot-test -p profiles/carton-local.yaml --move --ask` performs small joint identity checks.
The operator confirms which joint moved; this is not hand teaching.

## 5. Start carton integration in small steps

With validated device/calibration results, run:

```sh
carton check -p profiles/carton-local.yaml
```

It connects/configures motors and captures the head camera for a model judgement; it does not command
a folding trajectory. Start at rest; disconnect can release torque. Confirm that the real box and
flap states are recognized, rather than treating a simulator result as visual validation.

For direct dispenser pickup and placement, follow [carton-tape.md](carton-tape.md).
The October 4 follow-up resolves these teaching CLI gaps:

- Individual ordinary `carton teach` now starts/verifies its STOP viewer and disconnects on exceptions.
- `carton teach-all` stops on the first failure. Tape poses require the separate complete
  `carton tape-test --teach` sequence. Do not blindly batch-teach the full plan as the first test.
- The paddle-grip teaching goal positions an open gripper; later carry poses assume it is held.
  Verify actual pickup and maintain the required object/flap state between teaching steps. Teaching
  is a physical sequence, not independent arbitrary poses. Direct dispenser pickup/placement/release has a guarded
  software sequence now, but still needs physical testing with a marked strip and an already-closed box.

Then teach and verify one pose/step at a time with STOP and physical power cutoff available. Preserve
the existing step/temperature/load limits. Reset the carton as needed; never use simulator keyframes
as real poses. Once all prerequisites and actual observations pass, the single-cycle command is:

```sh
carton once -p profiles/carton-local.yaml --record
```

It runs **keyframes**, not ACT. Do not start the repeating
`carton run` until single-cycle behavior has been checked. Record failures as well as successes.

## 6. ACT integration is a separate gate

The checkpoint has **12 outputs** (left six, right six) and expects two distinct fixed camera views.
Confirm the order against the metadata, plus calibration conventions, normalized units and joint signs.
The source profile previously duplicated `head` into both policy inputs. It now names unconfigured
`policy_front` and `policy_top` inputs so the missing views are explicit. Supply and validate actual
views or collect data/retrain for the installed cameras; do not substitute one image twice.

`carton once`/`run` do not call the checkpoint. `farm policy-test` is not a ready carton test:
its default simulator uses the six-joint pouring profile, and the physical route does not enforce
`policy.enabled`/`shadow` as a motion gate or start the viewer. Do not run it with `--real` as a shortcut.
The next agent must implement and test a carton-specific no-motion observation/shadow path, validate
two-arm mapping and STOP/disconnect behavior, then consider bounded physical trials. Current replay
error is worse than the hold-current-state baseline, so loss reduction is not evidence of task mastery.

## 7. Leave a concrete continuation record

Update `software/STATUS.md` with the code commit, checkpoint hash, device identities, calibration-report
result, measured table/shoulder/setback values, camera identity screenshots, actual print/grip checks,
poses tested, observed results, and next blocker. Keep private photos, `.env`, calibration and runtime
databases local; share only the required sanitized evidence. Commit and push source/docs on the user's
behalf without co-author attribution. Never call a command launch, model claim or mock cycle a real closure.
