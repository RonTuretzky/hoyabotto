# G4 planter: first training experiments

This work uses **G4**, the shared guide, four separate gravity carriers and
seam roller. It does not use R3.x, snap fits, a G3 paper liner, or Blender poses
as robot commands. The original trough and growing holder remain the intended
assembly, and the paper must provide the water path.

**Current result:** the rigid pusher pickup, retained lift/hold, supported
setdown and release primitive passes its declared synthetic bench benchmark.
This includes nine placement cases, six fresh confirmations, two nine-case
solver comparisons, and eight correctly rejected fault controls. Carrier
insertion remains unqualified; the complete planter, physical
robot and paper/water stages are unvalidated. See the continuation section for
source-bound evidence and the historical results it supersedes.

The first executable curriculum stage is **G4 paper-pusher pickup, two-second
hold, lowering, release, withdrawal and one-second settled observation**.
It is an isolated one-arm bench experiment. The full planter, second arm,
cart, paper, guide insertion, roller assembly and wicking are **not modeled
by this first-stage simulator**. Hashing their source assets is inventory,
not proof that their dynamics have been implemented.

## What runs

The active objective remains the **complete G4 assembly sequence**, not a
pusher-only or passive-drop benchmark. The new full-station work is described
under "Full assembly continuation" below. Partial stages do not qualify the
whole task, and a prepared initial assembly does not earn robot placement credit.

- `planter/g4_sim.py`: unchanged SO101 imported joint frames/limits and jaw
  collision parts; only six robot actuators. The pusher is a free body. No
  object actuator, weld, mocap driver or object-pose action exists.
- `tools/train_g4_pusher.py`: parameter search over visual target offsets and
  jaw closure, plus a complete-sequence evaluator. This is a small scripted
  controller search, **not neural-policy training or full assembly training**.
- `tools/audit_g4_assets.py`: Boolean comparison of the three pusher collision
  blocks against the actual G4 print-oriented STL. Difference is less than
  0.001 mm³; the blocks preserve the real crossbar and handle shape.
- `planter/g4_curriculum.json`: dependencies and completion requirements for
  every later G4 stage, including manual paper tests and supervised watering.
- `tools/render_g4_report.py`: a local report linked to every actual GIF and
  machine-readable score, including failed attempts.

The G4 source remains in
`/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4`.
The experiments do not edit those CAD files or the active carton work.

## Observation and control boundary

The production `TagObserver` detects rendered tag36h11 markers. The production
`CalibrationRobot` uses quantized simulated encoder readings, LeRobot SO101
forward kinematics, eight fitting poses and three independent validation
poses. Each simulator receives a fresh camera-stream identity and its own
registration. Registration is never silently carried between instances.

An 18 mm black-square tag, ID31, is hypothetically attached to the pusher.
Its 22.5 mm backing fits the 24 mm grip width. This mount has not been tested
on the physical tool. RGB square-pose ambiguity was observed in the first
run. The subsequent controller therefore reuses `carton.folding_vision.depth_tag_pose`
to fit the actual detected corners against rendered, aligned optical-Z depth
with 0.8 mm Gaussian noise and 25% dropout. Missing depth is a failure.
RGB/depth are captured at the same simulation state and saved together.

The fitted base-from-camera transform and declared CAD tag-to-grip offset
provide the target. The decoded marker axes include the same 180° Y rotation
audited in the carton handoff. Simulator object pose is not passed to target
construction or IK. Joint actuators perform all motion after trial reset.
This first controller acquires its target before the sequence; it is not a
continuous visual-servo or deployable hardware controller.

Ground truth enters the independent evaluator and diagnostic contact stops.
Those stops are explicitly **privileged simulation diagnostics**, not claims
of physical force sensing. This distinction applies even when a trial passes.

## Scoring and stopping

Every physics step is recorded during task episodes. The historical v3 scorer
requires the following, but the continuation audit below found that these
conditions are **insufficient for retained pickup** because they do not check
grasp establishment before lift and retention throughout lift:

1. Ordered execution of approach, close, lift, hold, lower, release and withdrawal.
2. At least two seconds of continuous hold with loaded contacts on both jaws,
   at least 20 mm clearance above the staging rest, and no more than 5 mm slip.
3. At least one second after withdrawal with the tool settled freely on the
   staging rest, no loaded jaw contacts, and no more than 2 mm drift.
4. Finite, increasing-time observations and no collision or contact-stop failure.

The simulator aborts above 1 mm contact penetration, 8 N summed jaw/tool normal
contact force, or 8 mm final gripper tracking error. The original joint limits,
2.94 Nm arm actuator limits and 0.5 Nm jaw limit remain. **8 N is an experimental
simulation abort value, not a validated robot insertion-force threshold.**
It must not be copied into physical robot execution.

Missing/occluded tool tags, stale frames, missing depth, corrupted registration,
open jaws, zero grip friction and disabled motion are separate controls.
The final registration-corruption control actually changes the fitted transform
and checks the production current-gripper consistency rejection. The earlier
`baseline-03` control with that name instead injected a bad target offset;
keep that earlier evidence distinct.

An open-jaw failure during approach is not proof that a successful grasp would
depend on closing. Controls are meaningful when compared with the **same
passing candidate**, not just because every variant failed. The report keeps
failure reasons and does not treat all-control failure as task success.

## Assumptions and unmodeled effects

All current station dimensions are hypothetical. The tabletop is 60 mm below
the arm-base origin, with its near edge 240 mm forward. The camera is at
(0.54, -0.42, 0.46) m in that frame. The initial arm pose is planned, not a
captured physical encoder pose. Two loose staging-rest heights, 20 and 40 mm,
are explicitly compared. There is no physical fixture or station acceptance.

Pusher mass is assumed to be 12 g; friction is 0.8. The centroid comes from
the uniform-density CAD, while inertia remains an approximation. The actual
jaw meshes use the existing approximately 1 mm concavity decomposition;
other arm collision meshes retain their upstream convex approximations.
The collision-block/STL equality audit applies only to the G4 pusher.

MuJoCo contacts are compliant. Solver sensitivity must accompany a favorable
result; NoSlip must not be treated as evidence of a real grasp. See the
[MuJoCo collision modeling documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#collision-detection)
and [slip guidance](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip).
This Mac renderer reported limited depth precision because `ARB_clip_control`
is unavailable; that limitation is additional to the injected sensor noise.

No material model yet covers paper buckling, tear, permanent creasing,
paper-to-paper contact or fluid transport. No data from those stages is
eligible for successful demonstrations. The purchased material's reported
poor wicking remains unresolved.

## Parallel audit, 2026-10-07

The follow-up uses three agents with separate file ownership: staging,
independent scoring, and hollow carrier/guide collision geometry. Simulation
workers run as separate processes with separate working directories and
single-thread numerical libraries. A source tree is copied and hashed **before
launch**, and the worker executes that copy. Copying live files after import
would not prove which code was loaded. This follows the carton chat's
`tools/run_claw_sweep.py` process-isolation pattern.

Scorer v2 rejects interrupted or reentered phases, gaps in physics samples,
nonfinite state/contact fields, and unsupported release. It recomputes loaded
jaw contacts from raw contact data, verifies opposing force directions, and
measures slip in the gripper frame. A release requires actual loaded staging
support and no other loaded contact. Old traces lacking contact normals and
gripper rotation can receive an explicitly limited retrospective audit, but
cannot become qualified evidence. Original scores and traces remain intact.
Scorer v3 additionally tracks the full rigid tool's vertices, preventing
rotation about an apparently stationary grip point from hiding slip or an
unsettled release. Every evidence set retains its executed scorer hash.

The old three validation placements informed the staging redesign. They are
therefore **development regression cases** for that redesign. A wider grid
used to select geometry is also development data. Fresh confirmation poses and
noise seeds must be declared after freezing the fixture and controller; solver
repeats do not increase the count of independent placements. The normal
training CLI no longer exports a policy from the reused regression set.

`tools/diagnose_g4_staging.py` compares explicit rest hypotheses through the
same rendered perception, original robot joints, contact gates and complete
sequence. It never exports a policy. `tools/verify_g4_parallel_evidence.py`
independently re-scores the raw traces, verifies source snapshots, hashes
episode artifacts and decodes complete GIFs. `tools/render_g4_parallel_report.py`
keeps the saved scorer versions and passive carrier diagnostics separate.

Parallel evidence is saved under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-parallel`.

### Fixture development and fresh confirmation

The original rest starts at X395mm while the nominal pusher centroid is near
X398.6mm. A -4mm placement shift therefore puts the centroid beyond its support.
Moving the assumed rest edge to X393mm fixes all four known placements through
the full sequence under scorer v3. The original scene defaults remain intact.
All eight fault controls still fail with the same controller and unchanged
1mm penetration, 8N contact-force and 1.5mm IK tolerances.

The broader results prevent qualification:

| Fixture / split | Complete sequences | Main failures |
| --- | --- | --- |
| X393mm, four known regressions | 4/4 | None in this fixed set |
| X393mm, nine-point ±8mm development grid | 3/9 | Negative-X tipping; positive-X lift contact-force stops |
| X389mm, same development grid | 4/9 | Withdrawal IK failures and lift contact-force stops |
| Frozen X393mm, four fresh confirmation poses | 1/4 | One tip/tag loss; two lift contact-force stops |

Fresh confirmation was declared before running: offsets (-6.1,+4.7),
(-2.7,-6.2), (+5.4,+6.5), (+6.7,-4.1)mm, with noise seeds701–704.
Only (-2.7,-6.2)mm completed. The controller and fixture were not retuned
after that result. The fixture repair is a bounded improvement, not a reliable
pickup skill across the tested distribution. No policy was exported.

The same four known regressions also pass at 1ms and with NoSlip disabled.
Across passing episodes, full-tool hold drift is at most 0.498 mm and released
full-tool drift at most 0.0034 mm. These solver repetitions do not rescue the
failed fresh confirmation. Forty-six recorded pusher GIFs cover exploratory,
regression, development, confirmation, fault and solver episodes; they are
not forty-six independent placement trials.

The independent scorer audit rejected eight deliberately corrupted synthetic
traces that the original scorer accepted. The nine historical passing episodes
among twenty audited legacy episodes still pass the checks their logs support;
missing normals and gripper frames prevent strict certification. These counts
describe stored episodes, not additional independent placements.

### Passive carrier diagnostic

`planter/g4_carrier.py` and `tools/diagnose_g4_carrier.py` implement one empty
carrier released under gravity above the first shared-guide funnel. The guide
and original holder are fixed. Only their X = [-61,-29] mm corridor has collision
geometry; their full visual meshes must not imply full collision coverage.
No robot, grasp/release, perception, trough, paper or water is included.

The actual source STLs are partitioned into convex pieces instead of using
whole-mesh convex hulls that fill holes. The final audit has 305 guide,
412 carrier and 101 holder pieces; all 25 local channel, collar-gap, thin-rim
and descent probes pass. Added collision volume is at most 0.217 mm³ per part;
missing volume is below 0.00171 mm³. The first rejected geometry audit is retained:
an invalid empty Boolean union initially escaped a global volume check but
failed the local rim probes. The code now rejects invalid/empty unions directly.

The initial parallel audit's authoritative batch is `carrier-06`: **0 of 12 diagnostic trials pass**.
Eleven stop at the unchanged 2 N contact diagnostic; disabled gravity fails to
descend and seat. The set covers offsets, yaw, friction 0/0.4/2, timestep 0.1/0.05 ms,
solver 100/200 iterations and softer contact. The nominal carrier starts with
no intersections and reaches the holder, but does not complete the required
two-second sequence and final 0.5-second supported settle before the abort.

Nominal peak summed normal force is 60.23 N across 32 contacts, with 7.65 N maximum
single contact and 26.66 N net upward contact force. Penetration is 0.00949 mm.
These are rigid simulator impact measurements, **not measured hardware force
or proof of a physical jam**. Segmented/opposing contacts affect the sum; the
signed net force was independently checked against MuJoCo generalized force.
Reducing timestep removed the coarse penetration error but did not establish
acceptable contact behavior. The 2 N gate is provisional and is not a validated
physical stopping threshold. No gate was raised to obtain a pass.

The independent carrier scorer checks every timestep, finite/unit state,
initial unloaded release, actual descent, complete body containment, raw
loaded holder contact, upward support, and continuous final settling. Fifteen
carrier tests and the combined 45 G4 software tests pass. Those tests establish
software behavior; no carrier trial or complete assembly has succeeded.

Next work should investigate contact compliance and the intended release
height against a measured manual carrier drop, then establish a passing
passive baseline before adding a robot grasp and visual insertion controller.
Paper feeding and wicking remain blocked on the manual material tests.

## Continuation and whole-motion audit, 2026-10-07

The new evidence root is
`/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-continuation`.
`tools/run_g4_batch.py` freezes the source tree before launching bounded,
isolated worker processes. The launch manifest records source hashes,
dependency versions, explicit commands and worker limits. Process completion
never establishes a successful task.

The X389mm rest with an 8mm declared target shift toward the handle tip and a
25mm horizontal withdrawal passes the saved v3 scorer on the nine development
placements, six fresh confirmation placements, and both full-grid solver
repeats. All eight matched fault controls fail. **These are saved scorer
outcomes, not qualified retained-grasp results.** The independent audit
`scoring-new-results-whole-sequence-01.json` found no loaded jaw contact at the
end of closing. Opposed contact begins approximately 0.36s into lift after
11–12mm of tool motion relative to the gripper. Some development cases also
load the fixed-jaw housing against the rest by about 3N while remaining below
the old penetration-only collision threshold. More forceful closure and a
straight Cartesian lift do not by themselves resolve all acquisition losses.

The controller was therefore corrected against explicit pre-lift grasp
establishment, retention throughout the lift and hold, rigid-body slip from
that established grip, and unintended loaded arm/fixture contact. Original
v3 results remain immutable. Later stricter audits are separate evidence and
must not be used to rewrite the original score. No demonstrations or policy
are promoted from these v3 passes.

`planter/g4_retention.py` implements the separate independent audit. It
requires at least 95% opposed loaded contact in the last 100ms of closing,
no unloaded gap longer than 10ms through lift/hold/lowering before sustained
setdown, and at most 5mm rigid relative displacement from the established
grasp. Setdown requires 40ms of upward rest contact within 1mm of its surface;
a lateral brush cannot terminate the retention requirement. Arm/environment
contacts above 0.005N are rejected, with peak force, duration and impulse
reported. These are explicit simulation analysis thresholds, not calibrated
physical stopping limits. Twenty tests exercise missing evidence, late
acquisition, loss across phase boundaries, false support and collisions.
Demonstration eligibility now requires this audit to pass even when a command
retains legacy v3 success reporting.

The saved MuJoCo step logs have an explicit timing detail: contact, body and
site transforms describe the step's pre-integration configuration, while
`encoder_radians` is read after integration. The independent mesh-contact
audit therefore reconstructs jaw geometry from the preceding encoder sample
and checks it against the logged gripper transform. Retention compares body
and gripper geometry from the same configuration. Using the same row's
post-integration joints for source-surface comparisons would introduce a
motion-dependent alignment error.

The first independently verified clean development pickup is
`pusher-seating07/x8-z5-jaw098/run/placement-00`: declared correction
(-8,-4,+5)mm, jaw command -0.098rad, Cartesian path segments of 5mm, and return
height zero relative to the pickup target. Opposed contact is present in all
recorded samples from the final close window through supported setdown. Whole
tool drift is 1.957mm, peak jaw/tool force 4.554N, maximum penetration 0.046mm,
and no loaded arm/fixture contacts occur. Both X=±8mm development placements
also pass. The moving jaw contacts the actual handle about 6mm inside its tip.
The final unchanged candidate also passes all nine ±8mm development-grid
placements, six predeclared fresh confirmations, all nine cases at 1ms, and
all nine cases with NoSlip disabled. The matched nominal passes and all eight
fault controls fail. The independent reviewer reproduces all 34 positive
outcomes and eight fault outcomes from raw evidence; the original three
selection positives bring the stored selected-controller count to 37.
Solver repeats and reused placements are not additional independent samples.
The largest rigid drift is 2.890mm, peak jaw/tool force 5.924N, maximum
penetration 0.168mm and minimum hold clearance 77.471mm. No loaded
arm/environment contact occurs. Two grid cases have isolated 2ms contact
gaps, within the explicitly declared 10ms/95% analysis limits; do not describe
all episodes as having loaded contact at every timestep. The earlier two
controllers each fail all three paired clean-grasp cases under the same
source and seeds.

The final parameter/evidence summary is
`pusher-clean-final-summary-v2.json`; independent final rescoring is
`scoring-clean-completed-matrix-01.json`. The latest frozen run passes all
74 G4 software tests (`integration-tests-02`), including the explicit holder
cover option. The earlier 73-test snapshot is preserved. Software tests do
not substitute for the physics trials.
This is a yaw-fixed, assumed-fixture benchmark of one rigid tool, not a
certified physical controller or a neural-policy export. In particular, its
1920×1440 rectified rendering and 0.8mm depth-noise assumption have not been
validated against the provisional 640×360 OAK camera measurement profile.

An additional original-jaw CAD audit checks 123 loaded contacts across 16
sampled close, peak-force, hold and setdown states. The contact endpoints are
within 0.000647mm of the actual SO101 jaw source surfaces, with no clear
off-material or inward-source support at those samples. Source-face normal
approximations reach 2.845 degrees, so this is not exact normal fidelity or a
physical gripper validation. The scope and original-source transforms are
recorded in `scoring-pusher-jaw-source-audit-02.json`.

Carrier development found conditional passes when released only 1–1.5mm above
the seat with a particular unmeasured contact compliance. This is an
**already-guided seating diagnostic**: it omits the preceding robot-guided
descent through the funnel and grasp release. It is not evidence for full
funnel capture or four-carrier insertion. A frozen 21-case sensitivity matrix
at 1.25mm release height and 0.9ms contact time constant passes only 9/21 with
the original contact mode; small pose, friction and solver changes still
cause failures under the unchanged 2N and 0.1mm gates.

A minimal two-mesh reproducer in `carrier-ccd-audit-04/minimal` isolates a
native multi-contact inconsistency: one recorded pose reports 0.136mm contact
penetration while `mj_geomDistance` reports about 0.00056mm for the same pair.
Disabling only multi-contact makes those depths agree in that reproducer.
The new single-contact matrix explicitly records its solver flags and retains
the original geometry, compliance and gates. This is a fresh experiment,
not permission to silently reinterpret failed traces. The reproducer does
not establish that the same problem affects other carton or robot contacts.

The single-contact matrix (`carrier-single-05`) passes 13/21 cases: six
sensitivity cases fail and the two intentional negative controls fail as
required. The nominal, half-timestep and 200-iteration cases pass. A fresh,
frozen four-case confirmation (`carrier-confirm-06`) passes only 3/4. The
failed pose at (-0.075,-0.075)mm, -0.04 degrees yaw and 1.4mm release height
reaches 4.267N summed contact force with 0.0521mm penetration. Thus even these
four samples in a very small prealignment range do not qualify robustness.
The current pusher registration results do not establish anything approaching
75-micrometre placement accuracy for a carrier. A robot insertion stage must
demonstrate capture from its actual observed uncertainty, not assume this
privileged prealignment.

The failed fresh case is a transient modeled impact, not evidence of sustained
wedging: a separately labelled diagnostic continuation observes only two
0.1ms samples above the gate. More seriously, independent surface probes show
the strongest contact normal points **into** the original holder surface,
while other contact endpoints lie about 27–37 micrometres outside the source
mesh. These are decomposition/convex-hull artifacts despite the global volume
and opening audits. The original gate failure remains; neither conditional
passes nor this spike establish physical insertion resistance. A tighter,
merged convex cover is being tested with source-surface checks before another
matched dynamics trial. There is no blanket contact deletion or force-limit
increase.

The subsequent opt-in `--holder-cover continuous` builds 82 overlapping wall
sections constrained by actual vertical and chamfer planes across the full
plate thickness. This removes the second artificial horizontal seam while
preserving the source: extra/missing volume is about 0.000012/0.000003mm³,
and all 25 aperture/rim probes pass. The previously failing near-seat pose
then completes under the original gates (1.122N, 0.0545mm penetration).
This is a conditional development pass. In the fixed `carrier-cover-controls-11`
matrix, nominal reaches 1.807N while halving the timestep reaches 2.180N and
fails. A full 56mm funnel-height release reaches 15.223N and fails. Height and
strong-friction cases also fail, so the corrected model does not establish
robust gravity insertion. The previous partition model's half-timestep pass
must not be carried forward as evidence for this replacement.

The final fixed 26-case batch completes with **17/23 guided development and
sensitivity cases passing**, two correctly rejected intentional negatives,
and a failed full-height release. The 17 passes include all four reused small
placement cases; these are development cases after the geometry changes,
not fresh confirmation. All 26 GIFs decode, raw record counts agree, source
hashes match and geometry/probe audits pass. `carrier-cover-controls-11`
contains `SUMMARY.txt`, `result.json`, `verification.json` and exact frozen
source reproduction commands in `REPRODUCE.txt`. No additional sweep is
running after this batch. Further insertion work needs a defensible contact
approximation and measured printed-material response, followed by an actual
joint-controlled grasp, lowering and release over the visual positioning
uncertainty. Conditional near-seat drops do not provide that trajectory.

Small contact-normal discrepancies remain explicitly bounded rather than
called physical defects: the final nominal's sampled source-cone residual
corresponds to about 1.75 degrees on a 0.027N contact. An earlier corrected
case has a 45-degree corner ambiguity whose endpoint is 2.94 micrometres
from an adjacent chamfer. Exact source-normal agreement is not established;
the task's independent force/timestep failures already prevent qualification.
No physical insertion-force threshold or real printed-material response has
been measured.

The diagnostic gate sums normal contact forces across collision pieces; it
is not an axial robot-force measurement. The corrected nominal's peak sum is
1.807N while its signed net magnitude is 0.765N; strong friction produces a
17.649N sum but only 0.725N net. The full-height impact has 15.223N summed and
6.269N net at about 1.049m/s. These raw quantities are preserved in
`carrier-cover-controls-11/selected-peak-force-summary.json`. No reported sum
should be interpreted as measured physical insertion resistance or proof of
a jam, and the gate has not been changed to obtain a pass.

`registration-bias-audit-02.json` separately isolates the current RGB
calibration error. At reconstructed quantized encoder poses, the kinematic
model agrees with the simulator's matching gripper frame within 0.056mm.
Replacing only the observed tag poses with simulator truth for a privileged
diagnostic fit reduces camera error from 8.861mm to 0.231mm. This supports the
inference that image pose error and calibration conditioning dominate this
example; it does not correct or qualify the visual controller. The first
version of this diagnostic compared different gripper frames and is explicitly
superseded. No ground-truth poses are supplied to the actual pickup controller.

Paper buckling, feeding, guide withdrawal, roller folding and wicking remain
unmodeled and unqualified until the requested manual tests and actual material
measurements are available.

## Historical first-run results, 2026-10-07

The nominal pusher sequence finally passed after using a 40 mm staging rest,
a -0.08 rad jaw target, an 8 mm raised release target and a 12 mm vertical
disengagement before withdrawing horizontally. Without that disengagement,
the lower jaw dragged the released tool off the rest. Those failed attempts
remain saved; no scoring threshold was relaxed to obtain the passing run.

| Frozen controller / solver | Nominal complete sequence | Independent placements | Hold slip, nominal |
| --- | --- | --- | --- |
| 2 ms, NoSlip 3 | Pass | 2 of 3 pass | 0.0374 mm |
| 1 ms, NoSlip 3 | Pass | Same 2 of 3 pass | 0.0372 mm |
| 2 ms, NoSlip 0 | Pass | Same 2 of 3 pass | 0.0670 mm |

Nominal minimum tool clearance above the rest was 72.4–72.6 mm. In the
2 ms run, the largest pusher contact penetration was 0.0773 mm and maximum
summed jaw/tool normal force 0.664 N. All eight fault controls failed as
required with the same candidate parameters. The zero-friction trial stopped
on penetration before completing a hold; it is not a measured slip threshold.

The failed independent placement is (-4, +6) mm relative to staging. The
tool tips off the front of the rest during the pre-motion settling period
and its tag disappears. This was checked in the rendered RGB and the
independent object-pose log; it is not merely a detector error. The two
passing placements were (+4, +3) and (+3, -5) mm, each with independent depth
noise. Repeated solver trials are not additional independent placement cases.

**The stage is still unqualified and no policy was exported.** The next
rigid-task improvement is a wider stable staging support with verified jaw
clearance, followed by the same held-out placements. Do not train through
the tipped-tool condition or silently narrow the evaluation distribution.

At that point, 11 focused tests passed. These verify contracts and independent
scoring, not physical performance. The fitted calibration's held-out position
RMS was approximately 0.63 mm while its independent camera-origin error was
8.86 mm. Report both: a small fitting residual did not remove absolute bias.

## Reproduce

From `software`, use the existing simulation environment or install the pinned
`requirements-planter-sim.txt`. Choose a **new** output directory for every run.

```sh
PYTHONPATH=. .venv/bin/python tools/train_g4_pusher.py \
  --simulation-root /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot \
  --cad /Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4 \
  --model-directory /Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-carton-controller/software/data-carton/models/so101 \
  --out /absolute/path/to/new-run \
  --rest-height .040 --release-height .008 --single-candidate --vertical-withdrawal
```

`--fine-grip` searches two visual offsets and three jaw angles. Without a
search flag the initial coarser target-offset search is used. `--single-candidate`
freezes the (0, -4, +4) mm target correction and -0.08 rad jaw target, allowing
the same controller to be compared at `--timestep .001` and `--noslip 0`.
`--probe` stops the search after its first candidate, still evaluates the three
held-out placements if that candidate passes, and skips the negative controls.
It cannot qualify or export a policy.

Each output contains asset hashes, exact argv, declared assumptions, scene
XML, calibration captures and residuals, target observations, raw RGB/depth,
joint commands, per-physics-step contact logs, independent outcomes and full
GIF timelines ending at the real failure/success state. Runs from
`release-1ms-07` onward also snapshot the imported repository Python sources.
Earlier exploratory runs retain the two G4 source hashes; most also retain
their G4 source files, but do not contain that complete imported-source snapshot.

```sh
PYTHONPATH=. .venv/bin/python -m pytest tests/test_g4_training.py -q
PYTHONPATH=. .venv/bin/python tools/render_g4_report.py \
  /Users/wk/Documents/ChatGPT/Hackatuson/output/g4-training
```

Reproduce the repaired-rest regression/fault set and the current passive
carrier matrix into fresh directories:

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_g4_staging.py \
  --simulation-root /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot \
  --cad /Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4 \
  --model-directory /Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-carton-controller/software/data-carton/models/so101 \
  --out /absolute/path/to/new-staging-run \
  --rest-near-edge .393 --suite legacy --controls --evidence-role regression

PYTHONPATH=. .venv/bin/python tools/diagnose_g4_carrier.py \
  --cad /Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4 \
  --holder /Users/wk/Downloads/xlerobot-farm-parts/cressmaster-holder.stl \
  --out /absolute/path/to/new-carrier-run

PYTHONPATH=. .venv/bin/python -m pytest \
  tests/test_g4_training.py tests/test_g4_staging.py tests/test_g4_carrier.py -q
```

When other agents are editing dependencies, execute a frozen source copy as
recorded in the `staging-frozen-*/launch-manifest.json` files, rather than the
live checkout. Each final evidence batch records the exact executed sources.

Software-test success, any simulated skill success, and physical success must
remain separate. A `policy.json`, if qualification eventually produces one,
is simulation-only and still requires solver-sensitivity review; it is never
a hardware release or a trained full-planter policy.

## Next physical evidence needed

Measure the actual station and G4 grips. Manually test empty-carrier gravity
descent, paper offsets/yaw, rim catching, pusher withdrawal, guide withdrawal
of at least 28 mm, and dry/damp roller folding. Record force/resistance and
misses as well as successes. Confirm the purchased M5 axle permits free wheel
rotation. Measure actual paper stock before accepting the 46×42 mm strips
or 146×76 mm sheet as cutting specifications. Verify the water path through
all four paper tails and the full growing sheet before adding seeds.

These are required evidence inputs for later training, not assumed passes.

## Full assembly continuation

New evidence lives in
`/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next`.
The previous goal turn made progress by completing and auditing the pusher and
carrier experiments. It did not achieve full assembly. This continuation adds
missing model/control capabilities rather than promoting those partial results.

`planter/g4_station.py` imports both actual SO101 arm hierarchies, joint limits,
inertias and twelve joint actuators. The original trough, original holder,
shared guide, four carriers, pusher and assembled passive roller have free
rigid bodies. New collision assets cover all four channels, rather than the
previous diagnostic's single corridor. The station's dimensions and staged
part poses are hypotheses. Its first guide experiment initializes the holder
in the trough; it must not claim robot trough or holder placement from that
initial state. A complete future episode must begin with those parts staged
and perform their transfers too.

`planter/g4_roller.py` preserves the actual G4 wheel and fork meshes, chamfers,
axle bores and end gaps. Collision union extra/missing volume is below
0.001mm³ for each part, and the local aperture tests pass. A free handle/root
contains the approximate purchased smooth M5 shaft; the wheel rotates on a
passive hinge. The ideal hinge models radial constraint and axial retention,
so wheel/handle collision is explicitly excluded for that bearing pair.
There is no wheel actuator, weld to the robot, or object-pose action.
Wheel10g, handle14g, shaft10.79g, axle length70mm, damping2e-6Nm·s and friction
loss1e-5Nm are unmeasured assumptions. Actual washer/retainer geometry, bearing
play and preload have not been validated. This follows MuJoCo's documented
[passive joint model](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint),
not a claim that a purchased axle behaves identically.

`tools/diagnose_g4_roller.py` tests this mechanism on a narrow rigid coupon,
with an explicitly constrained X/Z handle and a single initial-velocity reset.
It is not a robot controller or a deformable paper test. The initial0.4mm
drop (`roller-01`) causes0.120–0.126mm penetration and fails the declared
0.1mm gate. This failure is retained. The near-contact20µm reset (`roller-02`)
passes the same gate at both0.5ms and0.25ms timesteps: respectively32.45mm and
31.95mm travel,4.024rad and3.960rad wheel rotation, and0.0291mm/0.0305mm peak
penetration. The stationary, zero-friction and jammed-bearing controls all
reject. The passive free-spin test decays with no applied torque.

`roller-independent-audit-02.json` reconstructs all six outcomes from raw
state/contact traces, verifies the frozen source and asset hashes, decodes
the six complete GIFs and checks that no actuators, mocap, equalities or
applied forces drive the bench. Sampled loaded endpoints agree with the wheel
source within0.000001mm; this sampled proximity result is not an all-contact
normal-fidelity proof. These tests qualify only the declared mechanism bench.
No paper fold, robot roller grip or complete-assembly success follows from it.

`tools/audit_g4_station_vision.py` renders a saved full-scene state through
the production `TagObserver`, saving raw/annotated RGB and aligned depth with
0.8mm noise and25% dropout. In `vision-initial01`, one camera sees the left
arm tag and another sees the right arm tag, but none sees the guide tag.
The table marker is partly covered by the staged pusher. Missing and stale
images are rejected. These findings require camera/marker/staging corrections
before targets can come from vision. They do not provide the required8/3
hand-eye calibration or a deployable multi-arm visual controller.

The table marker was moved to an explicitly clear location in `vision-initial03`.
A separate side-camera hypothesis (`vision-camera07`) reads both guide41 and
table1. The frame-bound RGB/depth adapter in `planter/g4_station_vision.py` now
also exercises production metric pose estimation with the declared marker
sizes. `vision-metric08` obtains unambiguous guide/table poses, but that camera
cannot see a gripper tag in the tested poses. It cannot yet support hand-eye
registration. Stale RGB and mismatched frame metadata are rejected. Depth has
the same camera, sequence, RGB hash and capture timestamp; the local renderer
reports limited depth precision because ARB_clip_control is unavailable.
Pose-reset visibility searches are explicitly privileged diagnostics and are
not executed robot trajectories or stationary calibration samples.

The preferred complete collision cover is `collision-angular-07`: 147 geometry
probes pass and sampled loaded resting contacts agree with the source surfaces.
Tiny erroneous exact-touch contacts remain documented; this is not an all-time
collision-fidelity qualification. The full independent contract checks original
arm limits, permitted actuators, source meshes, every physics transition and
contact evidence. It refuses missing stages instead of accepting caller flags.

The guide experiment first stopped on a real jaw/top-edge collision, before
closing (`contact02`). After correcting approach clearance, `contact03/inset400`
independently passes acquisition and lift: all 3,001 transitions replay, all four
jaw groups remain loaded throughout lift, actual rise is59.899mm and maximum
whole-grasp slip is0.173mm. This is only a partial development primitive.
The extended `contact04` stops during transfer at8.160N body-pair load when
both wrist-flex commands reach their original upper limits. Source-surface
checks support real contact preload rather than a false collision. Its failure
is retained. Base setback and lift-height hypotheses are being tested without
changing limits or force gates. Guide placement, release, all-four carrier
insertion and complete assembly remain unverified.

With bases20mm nearer and lift reduced to40mm, `contact06` completes transfer
and hold, then stops during placement. Independent source checks show that
some16.99N placement load comes from artificial normals at internal convex
partition cuts. The force is therefore not a validated physical insertion
measurement. Original-CAD Boolean checks separately show a real≈0.1mm lateral
clearance on each side: the recorded pose, with−0.159mm lateral error and tilt,
has0.18649mm³ source overlap. Both issues must be preserved. Repair the collision
representation before changing the insertion controller; do not force the fit.

The higher side-camera hypothesis in `vision-metric12` sees table1 and fixed
left-gripper4 with unambiguous RGB poses at all eight fitting and three held-out
planned poses, with no initial intersections above0.1mm. This remains a pose-reset
visibility check. `tools/diagnose_g4_registration.py` separately executes those
poses through joint actuators and records actual simulated encoder brackets,
raw RGB/depth and the full physics trace for the production sampler and fitter.
It cannot install a registration or issue assembly targets; fit residuals and
independent absolute error remain separate from physics replay and task success.

`registration13` completes the actual joint-actuated8/3 capture and production
fit: held-out translation RMS1.271mm, maximum1.787mm, rotation maximum0.094°.
Independent camera-transform error is9.551mm/0.356°, and the saved RGBD guide
grip targets are wrong by up to9.76mm. These targets are not qualified for
contact control. All four saved-observation controls reject: wrong camera,
stale depth, missing depth and a25mm-corrupted registration. A separate
eight-training-pose pixel reprojection refinement (`pixel-refinement15`) lowers
held-out pixel RMS from1.192px to0.278px and camera error to4.027mm, still too
large for this fit. It does not change the production fitter or install any
registration; simulator truth is evaluated only after optimization. The11
samples, full2,826-transition trace and complete GIF are retained for independent
replay. No hardware or complete planter success follows from these fit results.

The completed independent registration audit replays every transition/contact
stream and exactly re-renders all11 RGB PNGs and seeded noisy depth arrays.
Recorded encoder ticks and velocities reproduce from qpos/qvel in every4ms
bracket; production detector, sampler, dataset and fit reproduce exactly.
It independently confirms the9.551mm absolute error. Reproducibility passes;
contact-target accuracy remains insufficient.

`registration18` increases the grid radius to the existing136tick commissioning
limit and changes the initial wrist angle to−0.45rad to retain tag visibility.
Original joint/force limits and `Limits` validation remain intact. All11actual
samples and the production fit pass; initial independent camera error is6.147mm.
Independent replay now verifies all 2,858 transitions and exact reproduction
of all 11 RGB/depth/encoder captures, the dataset and fit. Saved-pixel refinement19 lowers camera
error to2.652mm/0.107° and held-out pixel RMS to0.228px. These are development
diagnostics, with no corrected or installed controller registration. A previous
192tick visibility-only search is explicitly ineligible for the production
commissioning envelope and was never executed as a calibration trajectory.

The source-equivalent `collision-interface-11` cover removes artificial cuts
in the raised rim and guide flange. Exact-state independent audit changes the
failed-pose resultant guide/holder force from16.993N to4.794N (normal force from
14.816N to4.132N), with unchanged mechanical settings. `contact08` reruns the
unchanged full controller against this cover, but still stops: right-jaw/guide
resultant8.691N, guide/holder4.794N. Its36loaded source endpoints qualify at that
state. A separate18-pose audit finds remaining lower-plate radial-cut artifacts
at±0.5° pitch; it limits wider-angle model validity without invalidating the
separately checked contact08 stop. A new continuous seating-face decomposition
and one unchanged-target slow-descent experiment are being evaluated separately.

`software-tests-contact03.json` records124 passing focused tests against that
exact frozen source snapshot. Software verification does not qualify any missing
assembly stage. The current full-scope ledger and evidence links are in
`g4-assembly-next/progress.json` and `g4-assembly-next/index.html`.

Paper feeding, buckling, guide removal with paper, folding retention, sheet
placement and the water path remain required. The independent full-episode
scorer must reject missing stages, including trough/holder initialization
that substitutes for robot placement. Manual paper/material evidence is still
required before paper-stage training; no such evidence has been supplied.

The slower `contact09` trajectory maintains established opposed grasp throughout
7.548 s of carry and recorded placement, with maximum whole-grasp drift 0.251 mm
and no unintended arm/environment loads. It reaches only 0.086 mm above nominal
seat, but stops before release at 27.27 N. Independent original-surface tests
show artificial contact normals at that state, so the load is not validated
physical insertion resistance. `collision-seating-16` preserves the source
unions and all 147 probes and fixes the sampled tilt cases, yet independent
replay of actual contact09 still rejects it: 67 outward probes remain inside
original material. It must not be promoted to a trajectory. Isolated source
triangle/solid collision alternatives are being assessed with unchanged gates.

The pusher fixture can be moved into the full station by the exact transform
from the proven bench arm base to the right station arm base. The new
`tools/audit_g4_pusher_station_transfer.py` records that transform, hashes all
source assets, and tests 380 sampled states over every old episode phase.
`pusher-transfer20` has zero unexpected intersections above 1 micrometre and
0.618 mm minimum arm/table clearance. It explicitly resets recorded object
poses and is therefore only a privileged geometry diagnostic, not dynamics,
perception or swept-path evidence. Original source-mesh inertia remains in the
full station; it differs from the old bench approximation.

`tools/diagnose_g4_pusher_transfer.py` now runs a separate contact experiment:
only recorded right-joint actuator controls are replayed after initialization;
every object remains free and the left arm holds its initial command. The full
8 N and 0.2 mm gates remain. `pusher-transfer21` completes 9,112 steps over
18.224 s from frozen sources, with maximum load 3.624 N and penetration 0.156 mm.
All input hashes remain unchanged. Complete native replay, grasp retention,
supported setdown, release and withdrawal still need independent scoring;
that audit is running separately. This transferred-command diagnostic makes no new visual-control claim
and gives no paper-feeding or full-assembly credit.

Whole-source contact feasibility checks reject two further representations.
A rigid 2D triangle shell loses original-solid containment: a small sphere
entirely inside the guide receives no contact, and crossing a surface from
inside reverses its contact direction. Source-conforming 3D tetrahedra preserve
the original boundary and volume exactly and fix that containment witness, but
independent replay still finds two invalid source normals at the contact09
pose. Neither representation is promoted. A separate native whole-mesh SDF
experiment is now checking distance/gradient accuracy and contact coverage.
The installed MuJoCo 3.14 compiler accepts a source-mesh SDF without a custom
plugin; this also appears in its [official example](https://github.com/google-deepmind/mujoco/blob/main/model/plugin/sdf/cow.xml).
Octree approximation and collision-search sensitivity remain unqualified.

The exact frozen `pusher-transfer21/source` snapshot passes 127 focused G4
software tests in 3.29 s, and every frozen hash remains unchanged. The artifact
`software-tests-pusher21.json` records this separately from task outcomes.
