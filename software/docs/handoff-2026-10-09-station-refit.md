# Final DCM desk station refit — 9 October 2026

The final OAK contract is integrated and cloud job **6ac8ea01fee2c90070178b79** is RUNNING:
https://huggingface.co/jobs/RonTuretzky/6ac8ea01fee2c90070178b79

The camera wait is finished. User requested stopping the four-H200 job and switching to eight H200s.
Previous v6 job 6ac8e7ae095c57808930634a is confirmed CANCELED after about 7.6 minutes; no demonstration
archive or checkpoint had been uploaded. Collection restarts. Use live status below for current progress.
All eight NVIDIA H200 EGL and camera projection checks passed (maximum axis error 0.1268 px).
Provider and pipeline readback confirm demonstration recording with 64 workers. Optimizer steps are not yet confirmed.
No physical commands were issued. The server thread retains hardware ownership.

## Current training handoff

Evidence root: outer workspace `.context/station-refit-2026-10-09/training/`.

- `cloud-release-v7.tar.gz`: credential-free frozen source/assets, SHA-256 `6241f957793115af901d1ddff21ce9d2f658e613415e3aecd6834fa3a1a418e4`.
- `eight-h200-verification.json`, `eight-h200-tests.log`: **36 passed**; eight-way merge/holdout checks. Only parallel-runner and DDP docstring differ from frozen v6 source; camera/assets unchanged.
- `camera-integration-verification.json`, `camera-integration-tests.log`: **70 passed, 2 skipped** and one successful audited frozen-bundle teacher fold (seed 10001). Earlier station qualification: 15/16 nominal and 15/16 varied portable folds.
- `launch-eight-h200-v7.py`, `launch-v7-receipt.json`, `budget-and-launch.json`: deduplicated submission and source revision. Old launcher versions are historical and must not be reused.
- `watch-cloud-v7.py`, `watch-cloud-v7.log`, `watch-cloud-v7.pid`: active read-only monitor; no cloud retries. `live-cloud-status.json` is authoritative polling output; `cloud-evaluation-v7/` will hold downloaded checkpoints and local evaluation.
- Private model `RonTuretzky/act_carton_dcm_refit_20261009_v3`; dataset `RonTuretzky/carton_dcm_refit_20261009_v3`.

User removed the previous spending ceiling and requested speed. Eight H200s cost approximately **$40/hour total**.
Six-hour provider watchdog (~$240 maximum new compute) and 355-minute internal deadline; not an ETA.
All prior jobs were terminal before launch; v5 `6ac8daa0fee2c9007017836a` remains canceled. Actual billing unknown.

Pipeline: 320 trials, seed 10000, 64 recorder workers; independent contact scoring, minimum 80% success
and 128 valid demos. Every tenth seed is held out before eight-way sharding. Three render workers per GPU,
eight image writer threads per shard, native dataset aggregation and index/image/holdout validation.
Eight Accelerate ranks train ACT in bf16, batch 4 each (global 32), lr 3e-5, chunk 100, 25k steps,
checkpoint upload each 1k. All GPUs must verify actual NVIDIA EGL and correct pixel projection first.

## Final camera integration

Contract: `config/oak-policy-camera-20261009.json`, digest
`8cda81a5a74bca6fd373ac9cfec51ef2665fdf4a9bbf79fea1864953008da869`.
Shared helper is copied unchanged from server evidence. Capture 1040×780, 10 FPS requested, focus 79,
full 14-term factory distortion. Virtual policy view 320×240 RGB, fx=fy=289.70562748477136,
cx=160, cy=120, 45° vertical FOV, identity rectification, zero output distortion.

`carton/refit_camera_contract.py` corrects MuJoCo's half-pixel convention with explicit GL frusta;
actual off-axis raster error is at most 0.128 px. The same renderer serves demos, evaluation,
fake-robot inference and `folding_refit_preview.oak_policy_render`. Historical raw `oak_render` stays separate.
The API camera adapter verifies frame/manifest identity and source projection before the exact shared remap.
No second aspect crop or resize. Source mismatch and stale-frame guards remain in effect.
Dataset, trial and checkpoint sidecars carry contract/helper/renderer hashes. Checkpoint saving writes
sidecars before upload; local evaluation checks checkpoint and holdout contracts.

The 180 mm setback, 220 mm base spacing, 500×480×700 mm desk, 729.1 mm CAD mounting plane,
10 mm inset and model head pose remain explicit simulation assumptions. Physical camera-to-arm,
wrist and station registration are unverified. OAK delivered 9.700913 pairs/s with approximately 2.99%
unpublished RGB sequences; wrists reported 5 FPS. Simulation uses ideal 10 Hz observations;
latency/drop robustness and powered transfer remain unverified. No physical readiness is implied.

## Evaluation

Use `tools/eval_refit_fold_policy.py`: every physics step gets contact checks and independent audit scoring.
The watcher restores immutable simulation evidence, verifies its hash and holdout separation, then screens
5k on four starts and evaluates 10k/15k/20k/25k on all holdouts using temporal ensembling 0.01.
On terminal status it also evaluates the last available checkpoint. Results land in `scoreboard.json`;
`best-checkpoint.json` remains provisional while training/evaluation continues and explicitly simulation-only.
Do not run a second watcher. The watcher downloads/evaluates; it never submits another paid job.

The following scene review details are historical physical-model evidence; old camera capture values below
are superseded by the final contract above.

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

Server acknowledgement is now saved at outer `.context/station-refit-2026-10-09/server-evidence/training-reply.md`,
with a pointer in `oak-calibration-request-20261009-211938/training-reply.md`. The server status initially
reported a full-sensor 1040×780 RGB trial at 5 FPS in progress; subsequent readback reports that
60-second trial passed and 10 FPS is being tested before the final contract. Head ticks are
2085/2623; the earlier capture pose is stale. Do not freeze the old 640×360 contract. Reconcile the
selected projection, pose and delivered image cadence with the existing 10 Hz action timeline.

Await the server thread's corrected OAK configuration. Compare its stream identity, resolution,
rotation, intrinsics/distortion, crop and pose provenance against the hardcoded settings in
`tools/record_refit_fold_demos.py` and the physical input path in `carton/fold_policy_runner.py`.
The latest user clarification is that this checkout may have the wrong configuration; do not infer
the intended replacement from historical captures. Keep cloud training and the watcher stopped.

Present the corrected local scene, accept corrections, then have the server owner establish physical mounting height,
pan-axis spacing/setback, carton offset/yaw and usable camera registration. Obtain camera-safe approaches
and full dynamic fold validation with the new collision bodies before generating a replacement dataset.
Review approval alone is not physical calibration or execution clearance.

The server separately reported a finish-only live Flash readiness review; no fold was dispatched in that
handoff. That model opinion and the old recipe are not scene registration or collision certification.
Preserve sole-owner hardware control and the existing resistance/telemetry/STOP gates.
