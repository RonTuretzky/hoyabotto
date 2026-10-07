# Bare-claw carton folding handoff — 7 October 2026

## Current outcome

**Incomplete. No full four-flap closure, tape application, or five-second
hands-clear retention has passed. No robot or physical camera was accessed.**
Keep this distinction when handing the work to Gemma or the robot Mac.

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

Full experimental details and failures are in `carton-near-transfer.md` and
`evidence/carton-near-transfer-20261007.json`. Newer partial-far batches are under
`output/bimanual-fold-sim/major-first` in the Hackatuson workspace. Do not treat
static reachability or a held flap as completion.
The 21-trial partial-major inventory is
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
commissioned on the physical robot.

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
records, not a fresh live status check. Actual tag widths/mount transforms,
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
