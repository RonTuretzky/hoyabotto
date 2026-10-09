# Carton pilot: tags, benchmark selection and final station evidence

Updated 9 October 2026, 18:37 JST. No physical folding run or training occurred during this work.

## Best completed LLM pilot benchmark

**DeepSeek Flash with the v8b recipe is the practical choice for speed; it did not demonstrate higher reliability than Opus on matched seeds.** Latest completed folding confirmation was matrix 38, reported at 16:23 JST.

| Pilot | Right flap folded, any method | Passed pinch/push metric | Median wall time |
|---|---:|---:|---:|
| DeepSeek Flash, seeds 12–19 | 8/8 | 7/8 | 1.065 min |
| Claude Opus 5.5, seeds 12–15 | 4/4 | 3/4 | 2.205 min |

On the same seeds 12–15, both passed 3/4; median times were Flash 1.03 min and Opus 2.205 min. Both failed seed 14 because about 17° of the fold came from non-pad pushing, above the metric's 10° allowance. Successful runs also logged illegal-contact time: this is not a zero-contact or zero-push certification. Flash's matrix-37-allhard stress batch was 0/4. Small samples do not establish a general reliability ranking.

The recipe is `pilot/bench/prompt-fold-real.txt` (v8b); its simulation counterpart is `prompt-fold-best.txt` (r9). It scans for a pinch, carries the flap in a continuous hinge-centered arc to 110°, holds 10 seconds, releases/parks, then checks depth heights. Qwen 3.8 27B is the configured eyes model, although most latest runs did not call the eyes.

Matrices 13–35 used the wrong carton dimensions (77 cm rim, 16 cm flaps). Matrices 36–38 use HACHIYO 379 × 283 × 108 mm, 140 mm flaps, approximately 81 cm rim. Earlier results use different recipes, scenes and metrics and should not be pooled. These LLM tests are also separate from the ACT policy's training/evaluation results.

All of those folding benchmarks predate the corrected wrist-camera collision geometry and final rectangular table. This work's new tag simulations are perception tests, not a new folding benchmark. The legacy real recipe still contains old station coordinates and a fixed 3 cm model-height correction; do not treat them as measurements of the final setup.

Raw benchmark sources live under `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/bench/results/`; a compact result extract with source hashes is in [evidence/carton-pilot-tags-2026-10-09.json](evidence/carton-pilot-tags-2026-10-09.json). Historical experiment notes are `/tmp/sim_loop_log.md`.

## Live status and the user's live-run request

At the user's subsequent request to use the best model live, the existing chat API selected **DeepSeek Flash** and verified the selection with a model call and status readback. Qwen remains the eyes model. No restart was performed for this switch or the final-table capture.

A separate read-only Flash readiness review used fresh motor state and carton-tag observations, with a finish-only decision schema and no motion dispatch. Phone accepted IDs 11 (`left_short_flap`) and 28 (`left_wall_near`); no accepted right-flap tag was present. Those are print-plan roles; physical mounting still needs verification. OAK and both wrists returned no valid tags. Motor snapshots before/after showed all 16 motors released and zero tick changes.

The readiness review was blocked by unresolved robot/table registration and wrist-camera body clearance. The selected recipe's depth-based fold check also lacks verified RGB/depth and robot-frame registration. The server can accept authorized joint commands, but its readiness flag does not certify these task-specific assumptions. **No live fold was launched.** A tag is not a pinch point, a missing tag is not proof of a folded flap, and same head ticks do not establish a calibrated camera-to-arm transform.

Local receipts: `.context/flash-live-selection.json` and `.context/flash-live-preflight/` in the seville-v2 workspace. They distinguish model selection and live sensing from physical execution. No new physical-run video exists to review.

## AprilTags now available to the pilot

- `farm/perception/carton_tags.py` adds `robot_get_carton_tags`, separately from the legacy paddle kit. It returns accepted/rejected IDs, print-plan roles, pixel centers/corners, quality, frame identity/hash/sequence, timestamps, freshness and missing IDs.
- IDs: 10/26/27 near wall; 21/28 left wall; 22 right wall; 25/24 floor; 11 left short flap; 12 right short flap; 13 far long flap; 14 near long flap.
- Camera collection explicitly uses `revive:false` where the catalog supports it. The carton observer supplies no metric pose, grasp verdict, robot transform or motor commands.
- The pilot supports `sense` with `what:["tags"]` and GET `/api/carton-tags`. The simulation client uses the same wrapper. Legacy paddle commissioning stays hidden.
- `gemma_tags.py` retains legacy defaults and metadata, while permitting a separate role map/tool schema. Gripper-to-paddle displacement is only produced for those legacy roles.

The integration is installed in the existing pilot and `farm-live/software` runtime, not merely staged. During that earlier installation the chat process alone was restarted after checking idle/released state; the hardware owner was not restarted. Subsequent final-table work made no restarts.

## Verification

- 60 tag tests passed across `test_gemma_tags.py`, `test_carton_pilot_tags.py` and the existing `test_carton_tags.py`.
- 109 focused orchestration/backend tests passed.
- A broader 165-test combination had 7 failures; the same 7 gripper-range/refusal failures reproduced on the untouched pilot baseline (145 tests). No hardware safety limits were changed to clear them. Baseline log: `.context/carton-tag-baseline-tests.log`.
- The MuJoCo perception fixture rendered 54 frames in 18 cases: yaw −8/0/+8°, right flap 0/45/100°, tags off/on, three synthetic cameras. At least one accepted tag appeared in all 9 tagged cases; untagged false detections were 0/9. This does not mean every panel was identified.
- Four actual Flash sensing trials (yaw 0, flap 0/100°, tags off/on) each requested tags and finished. They identified the right flap only in the tagged upright detail view and did not claim a completed fold. Earlier narration mislabeled a floor role and margin units; deterministic guidance was tightened before the final four trials. The final responses still contain speculative wording, so do not treat model prose as a calibrated observation.

Fixture evidence: `.context/carton-tag-sim/`, including `results.json`, `flash-sensing-results.json`, per-case sensor text/images/XML and `comparison.png`. It deliberately models idealized visibility, not the rebuilt station or camera-body contacts. The 35° and 58° views also differ in distance, so the result is not an isolated camera-angle experiment. Training remained held for the simulation thread's scene preview.

Reproduce the deterministic fixture with the installed MuJoCo environment:

```sh
PYTHONPATH=software python software/tools/simulate_carton_pilot_tags.py \
  --pilot /path/to/staged/pilot --out .context/carton-tag-sim
```

## Final-table handoff

Full evidence and hash manifests are at:

`/Users/wk/conductor/workspaces/xlerobot-farm/las-vegas-v1/.context/station-refit-2026-10-09/server-evidence/`

**Use `final-table/reply.md` and `final-table/manifest.json` for the current configuration.** The 18:15 round-table images are superseded by `final-table/capture-20261009-183423/` (18:34:23 JST). The user confirmed DCM-F5040H, rectangular 500 × 480 mm, top 700 mm above floor, label rating 5 kg. The partial images do not independently verify the 500-versus-480 axis orientation or exact carton offsets.

All four feeds were fresh. Sequential API captures were bracketed by owner state reads; exposures were not hardware synchronized. Wrist exposures precede the first bracket read. Phone time is server receipt only. OAK RGB and depth both have sequence 13598 in this bundle, but matching publisher sequence is not spatial-registration validation. All 16 motors remained released with zero tick deltas; head 2078/2580.

Physical mounting-plane height, pan spacing and setback remain unknown. Model-only values are mounting plane 729.1 mm above floor (29.1 mm above the final tabletop), pan spacing 220 mm and nominal setback 111.2 mm. The older 775 mm value used another origin. The historical claim of measured 114 mm setback has no recovered provenance; a report's 113.65 mm is a vertical gap, not setback.

Historical head solve: 35.145° tilt, −5.132° pan, 1.178° roll, box IDs [10,26,27], **no gripper tags**. Its translation to the bases is untrusted. It does not justify the earlier 45 cm cart-movement suggestion. Current OAK shows no visible external tags for a fresh solve.

The wrist-camera registered CAD bounds in `gripper_link` are x [−14.7,21.3], y [22.2,89.5], z [−29.1,13.7] mm; long axis **Y**, not Z. The other thread owns transformed meshes/colliders and `folding_sim.py`; this thread did not edit them. Run-5 contact remains historical user/video evidence, with tick logs and recording references in the parent handoff; it was not independently re-reviewed here.

## Installation and rollback

Stage a copy first, then run `software/tools/install_carton_pilot_tags.py --pilot /path/to/pilot`. It validates source hooks/AST, adds sensing/schema/HTTP/benchmark hooks, and writes content-addressed backups under the pilot's `.private/carton-tag-backups/`. Repeated installation is idempotent. It does not copy farm runtime modules, restart anything, or access hardware.

The runtime must import matching `gemma_tags.py` and `carton_tags.py`. Actual install hashes are in `.context/carton-tag-deploy.json`; baseline pilot copies are in `.context/carton-tag-baseline/`. To roll back, compare current files against the receipt first so later edits are preserved, restore the corresponding backed-up sources and `gemma_tags-original.py`, and remove only the added carton module if unused. A chat-only restart would then be needed at an authorized idle point. Do not restart the hardware owner or restore unrelated/private configuration files.
