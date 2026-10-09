# Final DCM desk station refit — 9 October 2026

Recovery job **6ac8f71a095c57808930689e (v9)** is RUNNING on eight H200s:
https://huggingface.co/jobs/RonTuretzky/6ac8f71a095c57808930689e

The 261 training episodes and full step-3,000 optimizer/RNG checkpoint were restored. All eight
new-node EGL/camera preflights passed (max axis error 0.1268 px). Fresh loss/step logs confirm
optimizer advancement to at least step 3,340 at 3.3–3.6 steps/sec (~105–110 minutes left). Camera/assets and the 25k-step recipe are unchanged. The camera wait
is finished. No hardware commands were issued; the server thread retains hardware ownership.

## Failure and recovery

- v7 `6ac8ea01fee2c90070178b79` failed at 14:00:42 UTC after 2,386 compute seconds. Progress
  commits every 20 seconds exhausted HF's 128 repository commits/hour limit. The unhandled
  HTTP 429 terminated the supervisor and trainer. Approximately $26.51 compute at $40/hour;
  actual billing unknown. It reached step 3,554, with full checkpoints saved through step 3,000.
- v8 `6ac8f626fee2c9007017915f` stopped after 19 seconds because its script import path omitted
  the software root. No optimizer steps ran. Approximately $0.21 compute; actual billing unknown.
- v9 explicitly establishes the project import path. The exact embedded runner passed an
  isolated launch preflight without inherited PYTHONPATH. **44 targeted checks passed** (42 before launch),
  including nonfatal HTTP 429 handling, Retry-After, checkpoint coalescing and script launch.
- Frequent progress is stdout-only. Checkpoint uploads are asynchronous, rate-limited and
  retried without propagating HTTP errors into training. Five-thousand-step milestones are
  retained; intermediate checkpoints coalesce while delivery is blocked. Final success requires
  checkpoint delivery before the deadline. Repository writes stay deferred until
  **2026-10-09 15:02:43 UTC / October 10 00:02:43 JST**, honoring the existing cooldown.
- No recollection or image rendering repeats: 291/320 teacher demonstrations passed,
  261 train / 30 held out, 146,905 training frames. Dataset revision
  `290ef16faaa27e0b667fa8a8c68d441ed756ac8d`; recovery model revision
  `9f26fbbd76a1aacbe517eb32acd34b29f455eee7`. Full optimizer/RNG/camera metadata validated locally.
- Prior measured speed was 3.3–3.4 steps/sec: about 110 minutes from 3k to 25k, plus startup,
  final uploads and evaluation. v9 fresh optimizer rate agrees; evaluation and final delivery are additional.
- Terminal v7 step-3k evaluation completed: **0/30 full-fold successes**. This is an early,
  incomplete-training checkpoint and is not a usable physical policy.

## Current training handoff

Evidence root: `/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/training/`.

- `cloud-bootstrap-v9.py`, `recovery-v9-plan.json`: SHA-256-verified patches applied over frozen
  v7 source without writing to the rate-limited repository. `cloud-release-v7.tar.gz` SHA-256
  `6241f957793115af901d1ddff21ce9d2f658e613415e3aecd6834fa3a1a418e4`; simulation/camera assets unchanged.
- `launch-recovery-v9.py`, `launch-v9-receipt.json`, `budget-and-launch.json`: deduplicated launch
  and spend ledger. Historical launchers must not be reused. No automatic paid retries.
- `watch-cloud-v9.py`, `.log`, `.pid`: read-only provider/checkpoint watcher, PID 3550.
  **`live-cloud-status-v9.json`** is the current status; `live-cloud-status.json` belongs to old v7.
  `cloud-evaluation-v9/` holds forthcoming evaluation. Both old v7 and v8 watchers have exited.
- `notify-eta-v9.py`, `.log`, `.pid`: ETA notifier PID 3553. Stage changes, first measured ETA,
  >=10-minute revisions and 15-minute periodic updates. Requires this Mac awake. Initial macOS
  notification command succeeded. `eta-current-v9.json` and `eta-updates-v9.jsonl` hold the estimates
  and delivery ledger. Stale snapshots are flagged; it does not submit jobs or control hardware.
  The cloud v9 runner emits resumed tqdm with total 22k; the watcher maps this to absolute 25k
  progress. The checked-in parser includes the same correction for subsequent launches. Do not
  restart the paid job for this local monitoring change; frozen v9 payload/hash remain unchanged.
- `recovery-v9-tests.log`, `bootstrap-preflight-v9.log`, `resume-verification/`: recovery evidence.
- Private model `RonTuretzky/act_carton_dcm_refit_20261009_v3`; dataset `RonTuretzky/carton_dcm_refit_20261009_v3`.

User withdrew the previous spending ceiling and requested eight H200s for speed. Current rate
**$40.00002/hour total**. Six-hour provider watchdog (~$240 maximum new compute) and 355-minute
internal deadline are operational limits, not ETAs. All prior training jobs were terminal at v9 launch.

Recipe unchanged: eight Accelerate ranks, bf16, batch 4 each (global 32), lr 3e-5, chunk 100,
25k total steps. Full checkpoints every 1k; uploads defer safely through the cooldown. Native
resume preserves the optimizer, normalization, RNG and sample ordering.

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

The final selected OAK contract has been consumed and tested. No further OAK trial is pending.
Current server acknowledgement: outer `.context/station-refit-2026-10-09/server-evidence/training-reply.md`.
Monitor v9 optimizer progress and evaluate uploaded milestones on the immutable held-out scenes.
Do not launch duplicate watchers or automatic paid retries. Completion requires delivered checkpoints
and a separate simulation result; physical registration and powered transfer remain unverified.

The server retains sole hardware ownership. Preserve resistance, telemetry and STOP gates. No physical
motion is authorized by a passing simulated teacher, a learned-policy score or this training recovery.
