# Final DCM desk station refit — 9 October 2026

Scene review was approved. **Cloud retraining is authorized and submitted. The corrected H200 job is RUNNING and recording audited demonstrations; optimizer training has not yet been observed.**
The existing server thread owns hardware. This checkout owns simulation geometry and candidate previews.
The user authorized up to **$50 total** and asked for the fastest training. No further spending approval
is needed within that cap. Static clearance and IK are not motion validation.

## Retraining handoff (latest)

Approved station: 180 mm base-line setback, 220 mm arm spacing, 500 × 480 × 700 mm DCM desk,
729.1 mm CAD mounting plane, centered carton with 10 mm near-wall inset. Physical camera registration
remains unverified. The earlier preview observations below remain applicable limitations.

All evidence is under the outer workspace's `.context/station-refit-2026-10-09/training/`:

- `pilot-qualified-03`: **15/16** nominal dynamic folds, fresh seeds 9100–9115.
- `pilot-portable-varied-v2`: **15/16** dynamic folds using relocated, portable assets,
  seeds 9200–9215, lateral offset ±5 mm, yaw ±1°, hinge stiffness 0.015–0.022.
- Each successful demonstration has a complete, independently scored applied-contact stream.
  Nominal rejection: near-flap clearance not retained. Varied rejection: non-jaw flap contact.
  Neither rejected episode enters training. Contact/tracking/grasp gates were retained.
- `dataset-smoke`: one complete three-camera LeRobot episode, 578 frames; two holdouts identified.
- **120 targeted tests passed** (119 combined plus the added policy-render integration test).
  The latter confirms synthetic teacher markers and collision hulls are hidden from policy images.
- `cloud-release-v2.tar.gz` and `cloud-release-v2/manifest.json`: frozen, credential-free source/assets.
- `launch-cloud.py`: one-job submission with durable deduplication and source checksum.
- `budget-and-launch.json`: authorization, authentication blocker, qualifications, and spending ledger.

Teacher recipe: along −0.04 m, radius 0.125 m, normal tilt 15°, axis sign −1, preheight 0.035 m,
clearance +0.002 m. Moving the open pinch outward avoids the fixed jaw snagging the flap's top edge.
Synthetic teacher camera: [−0.5, −0.7, 0.75], looking at [0, 0, 0.15]. The teacher table marker
is group 4 and is removed from policy observations. Policy cameras retain the approved scene transforms.
The head view uses the principal-point-centered 4:3 crop at 320×240. The physical runner does not yet
implement this matching preprocessing; do not deploy or move hardware from this training handoff.

Active job: Hugging Face H200 **$5/hour**, three-hour timeout **$15 compute cap**.
The ledger conservatively reserves **$30 across both attempts**, leaving **$20 unallocated**
within the $50 authorization. Actual billed charges are not yet available; the first failed attempt ran for four seconds. Official pricing checked at https://huggingface.co/docs/hub/main/en/jobs-pricing.
320 trials, 12 recorder workers; collection must retain ≥80% audited success and ≥128 valid episodes.
Every tenth seed is held out. Six render workers, three cameras, ACT batch 32, chunk/action steps 100,
lr 3e-5, 25,000 updates, checkpoints every 5,000. Private dataset/model destinations:

- `RonTuretzky/carton_dcm_refit_20261009_v1`
- `RonTuretzky/act_carton_dcm_refit_20261009_v1`

The repositories are now private and created. The user supplied a write/Jobs-capable credential;
it is passed through process stdin and the provider's secret field, never stored in source or local files.

- First job: `6ac8c46ffee2c90070177627`, confirmed ERROR before recording: the image uses uv and has no pip module.
- Active job: [`6ac8c522095c57808930534d`](https://huggingface.co/jobs/RonTuretzky/6ac8c522095c57808930534d),
  started 10:43:38 UTC. H200 and LeRobot 0.6.1 confirmed in provider logs.
- Corrective launch uses `uv pip install --python <container-python>` and preserves the first attempt in the budget ledger.
- Each attempt has a three-hour/$15 cap. The ledger conservatively reserves the full first cap until billed usage is known.
- Total authorized budget is now $50; do not count that as a request to spend the entire amount.
- The evidence archive now includes baked OBJ/STL scene meshes for later held-out evaluation.

Record the returned job ID immediately (launcher does this), inspect live logs, and distinguish
recording/rendering from actual optimizer steps. Do not blindly resubmit an uncertain launch.
No automatic paid retries. Training completion still requires held-out closed-loop evaluation before
choosing a checkpoint; a completed training job is not physical readiness.

## Audited checkpoint evaluation

Use `tools/eval_refit_fold_policy.py` for this new scene. It checks loaded forbidden contacts
at every physics step, including camera-to-flap contacts; the historical evaluator only sampled
penetration at policy-tick boundaries. Policy images hide collision hulls and teacher-only markers.
Malformed/nonfinite actions cannot advance physics. Complete contact streams are independently rescored
and hashed, and an audit failure prevents a success result.

Three evaluator regression tests pass (seven combined with cloud pipeline checks). A full recorded-command smoke replay (`eval-harness-smoke`, seed 9100)
folded both short flaps at 57.7 s with 28,850 audited steps, complete coverage, no loaded forbidden
contact, 0.33 mm maximum flap penetration and 1.4 mm carton translation. This verifies the evaluation
harness, **not a trained-policy score**. The evaluator was added after active cloud bundle v2;
evaluation runs locally against downloaded checkpoints and restored holdouts.

`training/watch-cloud.py` is a read-only Hub watcher with local evaluation, running separately
from the paid job. It never submits or retries a cloud job. Its outputs are `live-cloud-status.json`,
`watch-cloud.log` and `cloud-evaluation/`. It verifies the evidence SHA-256, rewrites cloud asset paths,
loads every holdout scene, and checks training/holdout seed separation before evaluation. An offline
cloud-style restore smoke loaded successfully. It screens 5k on four starts and evaluates 15k/20k/25k
on every held-out start with temporal ensembling 0.01, four concurrent local workers. Results go into
`scoreboard.json`; `best-checkpoint.json` remains explicitly simulation-only and not physical-ready.

To resume monitoring if that local process stops (no paid launch):

```sh
/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/.venv/bin/python -u \
  /Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/training/watch-cloud.py
```

Do not start two watchers simultaneously. Completed evaluation shards are reused; failed/incomplete
shards are preserved under an `-incomplete-` suffix before a local retry.

## Locations

- Implementation checkout: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/chat-mac/software/`
  on `RonTuretzky/fold-box-tags` (baseline `c4ae0dc`).
- Outer workspace: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/`.
- Standalone review page: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/dcm-preview-v4/scene-review.html`.
- Chat preview: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/dcm-preview-v4/preview.jpg`.
- Full candidate report: same directory, `report.json`. Includes all IK attempts, start state and projection metadata.
- Each `setback-150`, `setback-180`, `setback-200` directory has `scene.xml`, baked cart assets,
  overall/work/top/side views, a camera-body detail and head/wrist views. `artifact-hashes.json` hashes the bundle.
- Server evidence and limitations: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/final-table/reply.md`.
- Final capture: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/final-table/capture-20261009-183423/`.
- Upstream model: `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/xlerobot/xlerobot.xml`.

## Confirmed inputs and remaining assumptions

User confirmed **DCM-F5040H folding desk, high type, 500 mm wide × 480 mm deep × 700 mm high**,
label capacity **5 kg**. The phone image shows the carton on the brown rectangular support; the round white
surface in front is a separate table. The complete tabletop and its 500-versus-480 axis orientation are not visible.
Established carton dimensions: **379 × 283 × 108 mm**, with **140 mm flaps**.

Still assumed: **220 mm base spacing**, **729.1 mm CAD mounting plane above floor** (29.1 mm above the table),
**10 mm carton inset**, centered carton, no yaw, 500 mm table edge across the robot, **32 mm tabletop thickness**,
schematic desk legs. Cart setback and camera-to-arm transform have no new physical measurement.
Arm fabric and cables are omitted. Camera-module mass is uncalibrated; existing gripper inertia is retained.
The preview uses an assumed clear park pose, not the captured arm pose or a validated route to that pose.

## Camera and contact corrections

`carton/wrist_camera_geometry.py` adds exact registered upstream camera CAD visuals and a separate convex
collision hull for each of two parts per wrist. Assets and provenance are in `carton/assets/wrist-camera/`;
`tools/export_wrist_camera_geometry.py` regenerates them from the upstream model and checked-in registration.
Registration is a fixed-jaw/servo CAD fit, approximately **1.7 mm RMS**, not a physical mount calibration.
The upstream **1 mm left/right mount difference** is preserved.

Right camera bounds in `gripper_link`, mm: X [−14.7033, 21.2850], Y [22.2428, 89.4975],
Z [−29.0968, 13.6865]. Envelope approximately **36 × 67 × 43 mm X/Y/Z**; **Y is the long axis**.
The old module box had the axes wrong. Its claimed **15.4 mm nearest clearance is invalid**, including for
the old trained trajectory; the trained-versus-live pose difference alone did not explain that bad claim.

The default scene now includes camera collisions. `wrist_camera=False` is for explicit historical reproduction.
Camera contacts are forbidden against all objects, including flap cardboard. Arm contact with the floor and
schematic desk legs is forbidden too. Safety thresholds were not relaxed.

Historical head tilt **35.145°**, pan **−5.132°**, roll **1.178°** came from **box tags [10,26,27] with zero
gripper tags**. Translation was conditional on the assumed station, not a camera-to-arm calibration.
Withdraw the previous **45 cm cart correction** and **114 mm measured setback**; the recovered **113.65 mm
was vertical**. **775 mm legacy Base origin and 729.1 mm mounting plane are different frames.** A demo's
175 mm forward excursion is not a global reach-envelope result.

## Fresh capture

Server capture: **18:34:23 JST**, all 16 motors released before/after, zero encoder deltas. Head ticks 2078/2580.
OAK RGB/depth share publisher sequence **13598**, reported timestamps differ by **0.72 ms**.
RGB/depth spatial registration and robot-frame transform are still unverified. Reads were sequential;
wrist exposures preceded the first state snapshot by about 0.25–0.30 s, clocks are unverified, and phone
time is server receipt only. Do not describe these as synchronized hardware exposures.

OAK raw image: 640×360, K = [[504.894104,0,314.753784],[0,504.973846,192.592651],[0,0,1]],
with 14 factory distortion coefficients in `cameras.json`. The review rectifies the real frame with the full
vector and renders the corresponding pinhole projection. CAD head position and historical box-relative
angles remain conditional. The real/sim framing mismatch is visibly unresolved. Wrist 90° VFOV is assumed.

## Results

| Base-line setback | Pan-axis setback | Park camera/carton gap | Sparse terminal poses passing |
|---|---|---|---|
| 150 mm baseline | 111.2 mm | 39.9 mm | 1/8 |
| 180 mm (+30) | 141.2 mm | 62.5 mm | 2/8 |
| 200 mm (+50) | 161.2 mm | 80.4 mm | 2/8 |

All three static park configurations have no forbidden penetration. CAD cart/table separation was checked
independently because fixed-world geoms do not generate normal contact entries. The minimum static cart/table
gaps are 82.5 / 112.5 / 132.5 mm. The centered carton has 60.5 mm side support margins, 10 mm near inset,
and 187 mm rear margin.

Eight endpoint probes per case: left/right short flap at 0/30/60/85°, nine deterministic IK starts each.
The opposite arm stays parked. Gates: position error ≤8 mm, face-normal error ≤10°, camera gap ≥1 mm,
no forbidden penetration >0.1 mm. Failures include camera/carton contact, arm self-contact and orientation
miss. **None validates a full fold.** These sparse probes neither validate transitions nor prove a station
impossible. Extra setback improves the parked gap only in the tested posture.

## Reproduce and verify

Use the nested implementation checkout as the working directory. `--out` must be a new directory.

```sh
PYTHONPATH=. /Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/.venv/bin/python tools/preview_fold_station_refit.py \
  --simulation-root /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot \
  --upstream /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/xlerobot/xlerobot.xml \
  --camera-metadata /Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/final-table/capture-20261009-183423/cameras.json \
  --out /absolute/path/to/new-preview

PYTHONPATH=. /Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/.venv/bin/python tools/build_fold_refit_review.py \
  --preview /absolute/path/to/new-preview \
  --capture /Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/final-table/capture-20261009-183423
```

**124 targeted tests passed** across camera geometry, pinhole projection, station geometry/measurement,
cart collisions, contact audit/scoring and capture. A real MuJoCo contact test proves the camera housing
applies force and displaces a free body; disabling its collision counterpart yields no displacement/force.
The projection test places an off-axis point within 1 px of the supplied OAK K. These are software checks,
not hardware certification. Renderers are explicitly closed to avoid macOS teardown crashes.

Rendered images inspected; standalone page script passes `node --check`. Browser tooling blocks `file:`
URLs, so interactive UI behavior/local-storage persistence was not browser-verified. View/setback controls,
per-view local notes and a copy-all-notes control are included for user review.

## Next work and ownership

Present the local scene, accept corrections, then have the server owner establish physical mounting height,
pan-axis spacing/setback, carton offset/yaw and usable camera registration. Obtain camera-safe approaches
and full dynamic fold validation with the new collision bodies before generating a replacement dataset.
Review approval alone is not physical calibration or execution clearance.

The server separately reported a finish-only live Flash readiness review; no fold was dispatched in that
handoff. That model opinion and the old recipe are not scene registration or collision certification.
Preserve sole-owner hardware control and the existing resistance/telemetry/STOP gates.
