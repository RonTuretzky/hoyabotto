# Bare-claw carton folding handoff — 7 October 2026

## Current outcome

**Incomplete. No full four-flap closure, tape application, or five-second
hands-clear retention has passed. No robot or physical camera was accessed.**
Keep this distinction when handing the work to Gemma or the robot Mac.

### Latest update — all four flaps closed and held (shorts first)

**The two bare claws now close all four flaps and hold them in 14 of 20
simulated seeds (70%)**: shorts 89.7–93°, majors 86.6–90.4°, carton motion
≤ 9.7 mm, all 14 contact audits `CONTACT_ONLY_CLEAR`. It needs new station
assumptions (arm bases 120 mm above the table, far flap presented about 1°
inward, three extra printed carton markers). No tape, hands-clear retention
or hardware. Sequence, rules, failures and assumptions:
`carton-four-flap-shorts-first.md`.

### Earlier update — jaw-surface controller and major-pixel exclusion

**Both shorts now fold from about −13° to just past upright (+1.0° to +1.5°)
in two of three seeds**, then stop where the shorts meet the partially closed
majors. This is still **0/3 bounded +10° probes and 0/3 carton folds**, but it
is the furthest the paired short stroke has gone. All nine new runs score
`CONTACT_ONLY_CLEAR` (robot) and the eight that reached the probe score
`PANEL_CONTACT_ONLY_CLEAR`. No hardware was accessed.

Two root causes were found and fixed:

1. **The right claw never touched its short.** Replaying the recorded stalled
   V4 strokes against the exact collision meshes shows the right jaw hovering
   1.3 mm off the right short for the whole stroke (the left was in contact).
   The stroke target reused the right arm's 3.5 mm approach standoff, and a
   0.25° angle lead is only 0.6 mm at the tip, so a target-tracking controller
   could never close the gap. The new opt-in
   `--short-contact-policy jaw_surface_v5` instead measures the clearance from
   the permitted jaw collision hull (at encoder FK) to the *sensed* outer face
   of the short (current camera registration, measured angle, 3 mm cardboard),
   and leads by that clearance plus a 0.5 mm press, capped at 0.5 mm per
   command, never retreating; radial/hinge corrections keep the V4 deadbands.
   It uses only robot CAD, encoders and the camera, so it has a direct
   physical counterpart. Against the recorded strokes its clearance matches
   the true jaw/panel distance within about ±1 mm (camera registration noise).
2. **Major-flap pixels made a visible short look ambiguous or wrong.** Near the
   carton corners, the inward-leaning majors fall in the short sector and form
   a second hinge-aligned "plane" at +25–30°. With 43% of the true short's
   support it tripped the 35% ambiguity refusal ("short_left missing from both
   views"); once the short was edge-on to the extra camera, the phantom was
   *reported as the short* (+29°), which caused batch 10's view disagreement.
   Every phantom inlier lies on `long_near`/`long_far` in the replayed scenes.
   `depth_open_short_flap_angles` now accepts `majors=` (same-frame measured
   major angles) and drops pixels within 4 mm of those panels before fitting;
   both the extra view and the primary open-short path pass them. This also
   removes synthetic false positives (a +8.6° "short" made of major fragments).
   Measured majors are never used as priors.

| batch | settings (all whole-jaw approach, left/back view) | result |
|---|---|---|
| `paired-short-jaw-surface-11` | V5 | right short now moves in 3/3 (to −11.1/−9.7/−9.9°); left lost at −7.5 to −8.4° (phantom ambiguity) |
| `paired-short-jaw-surface-majorx-12` | V5 + major exclusion | left to −3.7/−3.7/−3.8°, right to −7.3/−5.2/−5.1°; left then edge-on to the extra camera |
| `paired-short-jaw-surface-majorx-primary-13` | V5 + exclusion + `--observe-primary-open-shorts` | seed 0 refuses left entry (1.93 mm predicted penetration); seeds 1/2 reach +0.97/+1.46° and +1.08/+1.34°, then the next left path is refused for 1.24/1.44 mm predicted penetration |

**Why it stops at +1°:** in the last second of seeds 1/2 the shorts are
pushing on the majors (short_left/long_far up to 2.4 N, short_right/long_far
2.7 N). With the majors resting at about 38°/34°, the shorts cannot pass
upright without moving them, the jaw presses harder, and the unchanged 1 mm
penetration gate refuses. This is the ordering conflict noted below ("rigid
shorts intersect [majors] during their middle rotation"), now reached by an
executed, vision-controlled stroke rather than a static scan. Further short
progress in this order needs the majors moved out of the way, not a different
short controller.

Reproduce batch 13 (from `software`):

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/new/batch --workers 3 --seeds 0 1 2 \
  --open-short-angles -15 --near-pre-out .03 --near-hold-degrees 40 \
  --far-after-near --far-hold-degrees 35 --far-contact-profile central \
  --far-startup-lift .0005 --release-far-after --release-near-after-far \
  --probe-shorts-after-release --short-view-camera front_left_back \
  --allow-primary-carton-absence --short-approach-policy whole_jaw_normal_v1 \
  --short-contact-policy jaw_surface_v5 --observe-primary-open-shorts
```

An outer-face correction from the fitted plane offset was also tried
offline. It reduced the left clearance bias from +1.8 to +0.7 mm but
increased the right bias from +0.5 to +2.0 mm (the orange right arm passes the
cardboard colour mask and skews that plane), so it was not kept.

### Earlier update — tangent-deadband controller and stroke-step variants

Four more three-seed full-prefix batches, all from the original open box with
the `whole_jaw_normal_v1` approach and the left/back view, under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/major-first`.
**All four are 0/3 short probes and 0/3 carton folds.** Every partial-major
prefix still completes and releases, and every run scores `CONTACT_ONLY_CLEAR`.
No hardware was accessed.

The new opt-in `--short-contact-policy tangent_deadband_v4` keeps the V3
setpoint reference but resolves the sensed-target error in each short panel's
fold frame (fold normal, radial, hinge axis), all taken from the current camera
registration and measured angle. It spends the 0.5 mm increment only on fold-direction
lead, never retreats along the fold, and corrects radial and hinge error only
beyond 3 mm. Recorded V3 strokes jitter about 1 mm (1σ) in both off-axis
directions. The new `--short-stroke-step-degrees {0.25,0.5,1}` declares the
measured-angle advance per stroke command (the default 0.25 is unchanged).

| batch | policy, step | faults | right short, first → last |
|---|---|---|---|
| `paired-short-tangent-deadband-07` | V4, 0.25° | 3× left short missing from both views | −15.35→−15.36, −14.59→−9.73, −14.12→−14.06 |
| `paired-short-tangent-deadband-halfdeg-08` | V4, 0.5° | 2× left missing, 1× right stall | −15.35→−15.36, −14.59→−10.35, −14.12→−14.08 |
| `paired-short-setpoint-halfdeg-control-09` | V3, 0.5° | 1× left missing, 2× right stall | −15.35→−12.01, −14.59→−12.70, −14.12→−13.76 |
| `paired-short-tangent-deadband-primary-open-10` | V4, 0.25°, `--observe-primary-open-shorts` | 1× left whole-jaw endpoint refused (1.93 mm), 2× views disagree on short_left | −14.46→−14.75, −13.60→−13.62 |

What the fold-frame summary of the recorded commands shows:

- **V4 stalls the right short by construction.** In 6 of the 8 V4 strokes with commands, the right sensed
  target sits 0.3–1.6 mm *behind* the actual jaw vertex along the fold
  direction (mean fold gap −0.30 to −1.56 mm). With no lead and no retreat,
  V4 commands nothing useful and the right flap does not move. The seed-1
  prefix is the exception in both V4 batches (right advances ~4–5°).
- **In 5 of those 6 stalled strokes the applied-contact log has no right
  jaw/short contact at all** (the sixth touches at most 0.15 N). The CAD vertex
  is reported past the sensed panel while not touching it, so the
  right-side sensed target does not coincide with the physical panel surface.
  Whether this is camera-registration bias on the right short or a contact
  vertex that is not the jaw point nearest the panel is not yet established.
  That is the next thing to diagnose, ahead of any further controller variant.
- **V3 at 0.5° moved the right short further than V3 at 0.25°** (batch 09
  versus 06: 3.3°/1.9°/0.4° against 0.9°/0.4°/0.0°). Three seeds per arm,
  so this is a hint, not an established effect.
- **The left short leaves both views near −7° to −8°** in 6 of 12 runs.
  With strict primary-pixel open-short evidence (batch 10) the left short is
  pushed furthest yet (to −3.6° and −4.6°), but then the two fresh views
  disagree on it and the probe refuses. Seed 0 of that batch refused its left
  entry for a 1.93 mm predicted wrist/short penetration.

Summarize any run or batch in fold-frame terms (read-only, no physics replay):

```sh
PYTHONPATH=. .venv/bin/python -m tools.summarize_short_stroke_frames \
  --run /absolute/batch-or-run [...] --out /absolute/summary.json
```

### Earlier update — whole-jaw normal approach and parallel run

The latest full-prefix experiment is
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/major-first/paired-short-whole-jaw-normal-06`.
It used three isolated workers, the original free/unbolted carton, the same
hinge stiffness/friction and loose-box dynamics, the V3 setpoint-feedback
controller, the hypothetical left/back RGB-D view, and the new opt-in
`whole_jaw_normal_v1` approach. It did not access hardware or train model
weights. All three original partial-major prefixes completed and the two major
flaps remained passively retained; none completed the short-fold target:

- seed 0: right short stalled after 148 approach/stroke commands; final
  shorts −8.81° / −14.13° and maximum horizontal carton motion 1.265 mm;
- seed 1: right short stalled after 148 commands; final shorts −8.60° /
  −14.09° and maximum horizontal motion 0.205 mm;
- seed 2: fresh left-short geometry disappeared; final shorts −9.41° /
  −13.83° and maximum horizontal motion 0.032 mm.

The run therefore remains **0/3 complete short probes and 0/3 full carton
folds**. It is useful evidence: changing the bare-claw entry posture removed
the previous left-wrist collision refusal, but it did not produce reliable
right-short progress or a complete vision-controlled stroke. The detailed
worker outputs, frozen source snapshot, RGB-D refusal records, contact logs,
and panel logs are all in that absolute directory. They are intentionally not
copied into Git because they are large generated artifacts.

The new static approach is documented in
`docs/carton-whole-jaw-short-approach.md`. It uses the actual SO101 jaw mesh,
checks the selected vertex against its declared rigid body, follows a reversed
joint path that was statically checked through the full 50 mm approach, and
subdivides planned endpoints to at most 0.5 mm. This is a planning and
collision result, not a force or contact-success result. The source-CAD audit
of the previous refusals is at
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/cad-hull-audit-20261007/`;
it confirms the original STL itself intersects those proposed poses, so the
1 mm collision gate was retained.

The observer now also has an explicit, default-off `observe_tagged_shorts`
phase. It reuses current AprilTag 11/12 poses only when same-frame aligned
depth quality, identity, angle range, and cross-view agreement pass. Rejected
or stale tag-depth records refuse; the existing strict hinge-plane bounds and
all carton/arm failure latches remain. This makes a later tagged transition
possible, but no dynamic trial has yet established continuous short visibility
through 90°.

Parallel controller search is now reproducible with isolated worker directories
and frozen source snapshots. The measured same-host benchmark was 95.35 s
sequential versus 53.37 s with three workers (1.79× observed speedup), with all
2,465 timestamped states identical. This is a measured local result, not a
guaranteed speedup on every machine. The latest three-worker run above took
66.81 s with a 2.93 worker-overlap factor; overlap is diagnostic, not a speedup
claim.

The strongest executed components are:

- Both shorts folded and supported by one open right claw while the left
  withdraws. The earlier fixed profile passed 8/9 camera-noise seeds; keep its
  one failure rather than combining different profiles into a claimed 9/9.
- Both shorts physically pushed outward and released, then the near major
  folded and held at about 91°. Three fixed-profile seeds pass this partial
  stage. Every executed step now has independently scored applied-contact
  evidence; none showed loaded forbidden or non-jaw flap contact.
- Near-major holds at 40°, 55° and 70° also pass from the complete original
  open-box prefix. These are clearance holds, not closed flaps.
- With the near held at 40°, a revised central far contact reached 38.21° in
  seed 2 with 0.0459 mm maximum horizontal carton movement, then stopped when
  the far-angle observation disappeared. Seeds 0/1 refused their approaches at
  the unchanged 1 mm penetration limit. This is not a robust far-fold pass.
- A subsequent 0.5 mm carton-up startup lift clears the approach. The frozen
  35° far target then physically releases and parks the right hand and retains
  the far flap for five seconds in all three seeds. Final far angles are
  34.93°, 35.42° and 34.88°, with maximum horizontal box movement of
  1.265, 0.203 and 0.032 mm. The left still supports the near flap at about 40°,
  and both shorts remain open. Each full run independently scores
  `CONTACT_ONLY_CLEAR`; this is a useful released partial fold, not closure.
- Extending that sequence to lift and park the left hand also passes 3/3 seeds.
  Both large flaps remain near 40°/35° for five seconds with both hands parked;
  all three applied-contact intervals independently pass. This frees both hands
  for a future regrasp, but the shorts remain outward and full closure is still
  incomplete. The batch is `both-partial-majors-passive-release-01`.

Full experimental details and failures are in `carton-near-transfer.md` and
`evidence/carton-near-transfer-20261007.json`. Newer partial-far batches are under
`output/bimanual-fold-sim/major-first` in the Hackatuson workspace. Do not treat
static reachability or a held flap as completion.
The 45-trial partial-major and short-regrasp inventory is
`evidence/carton-partial-major-search-20261007.json`; the released-far batch is
`partial-far-passive-release-02`. Every trial starts from the original open box.

## Scope and physical model

Bare SO101 claw collision meshes, unchanged rear-cart/table spacing, empty
272 g free carton, 379 × 283 × 108 mm walls and 140 mm flaps. The box is neither
bolted down nor welded. Hinges retain stiffness 0.018 Nm/rad, dry friction
0.004 Nm and damping 0.008 Nm s/rad. Table friction is 0.35. These material
values, station dimensions, camera poses, marker placement and sensor-noise
bounds remain assumptions, not physical measurements.

Original gates remain: 8 mm IK residual, 6 mm transit clearance, 1 mm robot/flap
penetration, 35 mm actual Cartesian tracking, 0.08 rad joint tracking, original
joint ranges and actuator limits. Contact folding stops for more than 15 mm
carton translation or 8° rotation. Loaded forbidden or non-jaw robot/flap
contacts now stop independently of penetration. No executing carton/flap state
is assigned after initialization. Static copied-state scans are explicitly
labeled and never counted as executed folds.

Rigid flap panels do not model cardboard bending or plastic crease history.
There is no measured physical station profile available here. Do not infer
camera-transform accuracy from a small calibration fit residual, or treat this
synthetic noise model as the distribution of the user's OAK-D Lite.

## Parallel search and replay

From `software`, use the repository `.venv/bin/python` with `PYTHONPATH=.`.
Each worker has an isolated output directory, frozen Python sources, and one
numerical-library thread. A maximum of four concurrent workers is permitted.
These are controller/physics experiments, not neural-network weight training.

Reproduce the near-held component:

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/new/near-hold-batch --workers 3 --seeds 0 1 2 \
  --near-targets -15 --open-short-angles -15 --near-pre-out .03
```

Reproduce the released partial far fold (still not full box folding):

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/new/far-release-batch --workers 3 --seeds 0 1 2 \
  --open-short-angles -15 --near-pre-out .03 --near-hold-degrees 40 \
  --far-after-near --far-hold-degrees 35 --far-contact-profile central \
  --far-startup-lift .0005 --release-far-after
```

Append `--release-near-after-far` to verify the both-hands-parked partial hold.
This optional stage requires exactly the near40/far35 released-far profile;
missing or stale observations, drift, and failed attempts cannot be reset away.

Appending `--probe-shorts-after-release` attempts a bounded two-hand short-flap
regrasp and fold toward at most +10°. This explicitly adds the hypothetical
front RGB-D camera and open-short hinge-plane observer described in
`carton-open-short-vision.md`; it is not a physical camera configuration.
The first full-prefix batch, `paired-short-passive-majors-01`, passed the partial
major release in all three seeds, but **0/3 completed the +10° probe**. Seeds
0/2 refused a predicted left wrist/short collision; seed 1 lost fresh table
registration after free transit. Seed 2 physically advanced the left short
from −12.90° to −9.25° and the right from −13.82° to −11.60° before refusing.
Static approach clearance therefore does not establish a safe complete stroke.

The new independent panel-contact log covers every executed step of this probe,
in addition to the existing full-run robot contact log. Both audits pass all
three executed intervals; predicted colliding commands were not executed. This
does not turn a refused probe into a fold pass. Verify panel evidence with:

```sh
PYTHONPATH=. .venv/bin/python -m tools.score_partial_short_panels --run /absolute/path/to/run
```

The scorer checks the compressed byte hash, unique recording identity, exact
source-event interval, every original solver step and the unchanged 1 mm
panel penetration criterion. Missing, position-only, replayed or incomplete
logs cannot pass. Panel-to-panel contact forces are recorded; the criterion is
penetration, not proof of measured cardboard deformation or retention strength.

The subsequent `source_coherent_0p5mm_0p25deg_v2` policy pairs every short angle
with that same camera's current carton registration. Actual CAD-point commands
are capped at 0.5 mm, with 0.25° measured-angle advances and the same attempted
advance budget. Free transit/standoff preflight no longer implies that a future
contact stroke is clear. Each contact substep is checked again before execution.

Both newer three-seed batches still finish **0/3 short probes**:

- `paired-short-coherent-front-02`: one current-view far-angle disagreement,
  two missing primary-camera carton registrations.
- `paired-short-coherent-left-view-03`: three missing primary-camera carton
  registrations. This opt-in mount is selected with
  `--short-view-camera front_left_back`; read `carton-additional-view-placement.md`.

All six executed robot/panel contact intervals independently pass. The full
original open-box prefixes match the baseline timestamped states exactly;
`evidence/carton-short-prefix-replay-20261007.json` records that comparison.
The offset view passed all 57 recorded moving-pose observations before this
new dynamic batch, but that component did not cover its new primary-camera
marker-detection failures. Subsequent pixel diagnosis found both arm occlusion
of wall marker 10 and an unobstructed but very oblique, narrow wall marker 21.
Missing identity is therefore not synonymous with occlusion.

The opt-in `--allow-primary-carton-absence` now permits fresh additional-view
geometry only after normal primary startup, current primary anchor registration
and visible housing/FK checks. No missing pose is copied from a prior. Both
current poses, when present, must still pass the existing contradiction gates.
Read `carton-primary-carton-fallback.md` for the exact packet contract and
negative tests. The default still requires both carton registrations.

The full-prefix `paired-short-fresh-view-fallback-04` batch uses that option
with `--short-view-camera front_left_back`. All three partial-release prefixes
remain exactly identical to baseline, and the fallback accepts five observations
across the batch. Nevertheless **0/3 reaches the +10° short target**:

- Seed 0 stops after 34 contact commands on fresh far-angle disagreement.
- Seed 1 reaches the 80-command approach limit; the left short has moved
  outward to −19.37°, so more approach commands are not established as useful.
- Seed 2 executes 130 contact commands and reaches −6.23° / +1.20° shorts,
  then loses a fresh left-short observation from both cameras. Maximum
  horizontal carton displacement is 10.79 mm. Its reported negative bottom
  corner clearance is an XY table-edge inset, not vertical penetration;
  carton/table validity requires its own audit.

All three executed robot and panel contact intervals independently pass their
respective scoped checks. These do not certify table support or full-task
success. The batch takes 62.85 wall seconds; worker overlap is 2.50, not a
controlled speedup ratio. The broader software regression passes 808 tests
with two optional skips. The missing-fold and physical-validation boundaries
remain unchanged.

The seed-1 approach trace identifies a controller problem, not an inadequate
iteration budget. During its first nine commands, the left CAD point is asked
to descend about 0.498 mm from measured FK but moves about 1.467 mm; a roughly
0.969 mm steady servo following offset is repeatedly added by resetting the
command from measured position. Across 80 commands the flap opens outward
about 6.72°. No panel-to-panel contacts occur in that interval.

The explicit `--short-contact-policy setpoint_feedback_v3` variant adds the
bounded sensed-target-minus-actual-FK correction to current actuator-setpoint
FK, then solves from the current actuator command. Its 0.5 mm limit describes
the commanded setpoint increment, not guaranteed physical motion. Actual-to-goal
collision planning and original actuator, joint, IK and runtime tracking gates
remain unchanged. `measured_v2` remains the default and the approach geometry
is unchanged. Reproduce with the same short-probe command and this extra flag;
all trials still start from the original open box. Constant-offset component
tests demonstrate convergence and holding, not successful folding.

The full-prefix `paired-short-setpoint-feedback-05` batch completes in 44.50
wall seconds with three workers. All three original partial-release prefixes
again match baseline exactly. The following contact approach executes 35, 32
and 35 commands, respectively; each next proposed command is refused for left
wrist/short penetration of 1.0205, 1.0600 and 1.0210 mm. Those refused commands
are recorded separately from completed commands. **0/3 short probes pass.**
Maximum returned CAD endpoint displacement is approximately 0.5004 mm, while
measured-to-goal FK distances can exceed 1 mm; this illustrates the distinction
between the new command-increment bound and physical tracking. All executed
robot and panel intervals pass their respective independent checks, without
certifying carton/table contact or full success. The next mechanical issue is
a clear claw approach/orientation, not a longer iteration budget.

Both exact camera caches are present and hash-verified in all three refusals.
These images precede geometry refusals and are not camera failures. A read-only
check using the existing strict open-short estimator on primary pixels finds
both shorts in all three, with maximum 0.275° disagreement against the current
additional view; no production estimator change is implied. Current broad
regression: **866 passed, two optional skips**. An independent V3 code audit
finds no blocking variant-specific issue, but nonlinear dynamics, continuous
setpoint-path bounds and physical portability remain unproven.

Refused runs now retain exact exposed RGB-D arrays when current, with explicit
unavailable entries otherwise. See `carton-primary-carton-fallback.md` for the
cache provenance and shared failure-latch corrections. This is diagnostic
evidence; no hardware adapter or physical readiness is enabled.

Omit `--video` during searches: all timestamped qpos states are still recorded,
while presentation rendering/compression is skipped. Perception still renders
its RGB-D inputs. The trace-only check exactly matched all 563 state/time pairs
of a video-recorded run. Three contact-audited near holds completed in 28.42 wall
seconds with a 2.90 worker-overlap factor; that is not a controlled speedup ratio.
For the later three-seed released-far profile, an exact-source sequential
comparison took 95.35 seconds versus 53.37 seconds with three workers: an
observed 1.79× speedup. All 2,465 timestamped state records matched exactly.
This is one same-host comparison under variable background load, not a general
speed guarantee. See `evidence/carton-parallel-benchmark-20261007.json`.

Score applied contact evidence independently:

```sh
PYTHONPATH=. .venv/bin/python tools/score_folding_contacts.py --run /absolute/path/to/run
```

Render the complete chosen attempt afterward:

```sh
PYTHONPATH=. .venv/bin/python tools/render_claw_retention.py \
  --run /absolute/path/to/run --out /absolute/new/review-directory
```

`folding.json` binds the compressed `applied-contact-steps.jsonl.gz` byte hash
and full simulation interval. The scorer checks coverage, replay and applied
wrenches. Old position-only traces cannot establish executed forces. Partial
results always retain `full_task_complete:false`; stop conditions and failed
trials must remain in the handoff.

## Why the remaining sequence is hard

The original outer-edge hook mainly loaded the far flap's top edge and slid the
box. A more central fixed-finger contact gives useful folding torque. Its
approach must still handle visual pose error and maintain a fresh far-angle
observation.
The startup lift passed approach in three 0.5/1/1.5 mm variants on seed 0, all
reaching about 39° before the station view lost the far plane. The fixed camera
views the far flap nearly edge-on there; the existing spatial-support gate
correctly refuses the sparse plane. Moving to the smaller 35° target gives the
released component above. An additional rendered view is being evaluated
explicitly as an extra calibrated-camera assumption, not a camera already
commissioned on the physical robot. With this explicit second-view mode,
the far45 target passes 3/3 full-prefix trials (independent final angles
43.44°, 45.64° and 44.45°) and all three applied-contact audits pass. A matched
single-camera far45 control passes only seed0; seeds1/2 stop at missing far
observations. The extra view supplies one and two otherwise missing frames
respectively. Seed0's entire 727-state trace is identical in both modes; it did
not need the second view. Neither mode has folded the shorts or completed the
carton. Enable the hypothesis with `--additional-far-view` and read
`carton-additional-view.md` before interpreting these results.

Folding the majors first is not a complete ordering solution: rigid shorts
intersect them during their middle rotation. Static scans found that much larger
major overfold angles only avoid this by intersecting the box bottom. Opening
the near flap far outward instead runs into the shoulder/base in this station.
These scans reject particular routes; they are not a proof that every bare-claw
policy is impossible. The missing piece is a physically executed sequence of
support, release and regrasp that preserves all four flap states.

## Software prepared for the robot/Gemma path

- Per-arm tag binding: right housing tag 2 and left housing tag 4, checked across
  sampling, registration, calibrated FK and Gemma tools.
- `carton/servo/bimanual_owner.py`: paired 12-joint owner component with one
  trajectory clock, lease, fault latch and both-arm stop. Exact trajectory,
  profile, bindings and scene identities are bound to the collision certificate.
  The component is not installed in a running sole owner. Existing single-arm
  trajectory v1 remains separate.
- `carton/servo/folding_readiness.py`: optional read-only Gemma status and paired
  IK proposal tools. All outcomes retain `motion_ready:false`,
  `execution_available:false`, `motor_writes:0`. No local manifest was installed,
  no pilot restarted, and no physical commissioning claim is made.
- `carton/folding_observed_scene.py`: robot-only collision snapshot from explicit
  observations, encoders, calibrated transforms and bounded uncertainty. Unknown
  flaps refuse or occupy their full conservative sweep; no hidden object state
  fills them in.
- `carton/folding_observed_adapter.py`: explicitly offline PixelPort/encoder
  adapter and robot-only model export. Its real replay component checks retain
  missing-short, uncertainty-clearance and wrist-limit refusals. It has not
  supplied a complete executable folding path.
- Passive tape coupon: a converged component exists at 25 microsecond discrete
  steps. The normal 2 ms folding step is unsuitable for that strip. Coupon
  retention is neither robotic tape application nor proof it holds all creases.
- A separate 180 mm tape model now has finite passive axial extension; its
  contact-free material tests do not prove adhesion. See
  `carton-extensible-tape.md`. The early-short tape-retention idea remains
  unproven, and the actual masking-tape product/strength is not established.

Read `carton-folding-readiness.md`, `carton-bimanual-owner-protocol.md`,
`carton-observed-scene.md`, `carton-observed-adapter.md`,
`carton-observed-scene-cli.md`,
`carton-applied-contact-audit.md` and `carton-tape-convergence-2026-10-07.md` for
contracts and component limits.

## Outstanding physical prerequisites

Use the existing authorized sole-owner path on the robot Mac; do not run these
simulation tools against hardware. Current robot state was not refreshed by this
work. Main was reconciled with the published commissioning archive at
`e5fda65803f28e6e975bc4857a5b5a31c4ea0297`: it records an installed right-arm
calibration matching hardware at that session (its wider pan range was accepted
by owner assumption), a left arm with mixed calibration, and a later
encoder-estimated 20 cm cart advance. Thus old image-derived station placement
and earlier registrations must not be assumed to match the physical setup.
See `paddle-apriltag-pickup-handoff.md` and
`commissioning/2026-10-07-paddle/README.md`; these are published historical
records, not a fresh live status check.

Main also includes the newer published right-arm pickup archive at `294f419`
and server-readiness clarification at `3f69297`. That archive records an
operator-confirmed paddle lift, placement, release and cleanup using bounded
camera-guided encoder commands, plus an earlier 5.43 cm cart reposition. The
reported server scope is right-arm only: four left-arm calibration mismatches
still prevent full-scope startup. This is neither a two-arm carton test nor a
validated Cartesian mapping. Read
`commissioning/2026-10-07-paddle-success/README.md` and `QWEN-SERVER.md` in that
directory. This task did not refresh the live service or change its protections.

Actual tag widths/mount transforms,
both-arm zero/sign/tool calibration,
independent registration validation, current homing/limit-register readbacks,
station geometry and camera/depth accuracy still need verified measurements.
The visible white jaw inserts also need a measured collision/contact model if
present on the physical claws.

The deployed owner still needs deliberate paired-component wiring, serial I/O
timeouts, independent watchdog, both-arm stop/release commissioning and an
observed-scene certificate for each actual proposed path. No complete adaptive
four-flap policy is installed. A software test, catalog entry, profile assertion,
small registration residual or simulation clip does not remove these remaining
requirements. Do not restore obsolete temperature checks removed by the owner;
retain the repository's current guards and sole-writer ownership.

Other chat work in `software/planter`, G4 tools/tests and its reports is separate
and must not be overwritten or included accidentally in carton commits.
