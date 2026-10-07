# G4 planter simulation handoff

Date: 2026-10-07 (Asia/Tokyo)

Objective: make the simulated SO101 robot assemble the G4 shared-funnel/seam-roller cress planter through every stage, with contact physics, rendered perception, independent scoring, failure tests, and reproducible evidence.

## Current verdict

The full objective is **not complete**. There is no physical success and no hardware command. The simulation does not yet execute a complete robot assembly: the trough and holder are initialized in place, paper and water are not modeled, and the guide has not reached release and hands-off retention.

The strongest new result is the full-station pusher primitive. A recorded joint-controlled pusher sequence completes pickup, lift, hold, supported setdown, release, withdrawal, and a released hold in the complete rigid station. The independent replay and mechanical scorer pass that primitive, but it is still not paper feeding and it does not earn full-assembly credit.

The guide remains blocked by contact-model fidelity near seating. Several collision representations have been tested and rejected with unchanged force and penetration gates. No force threshold was relaxed and no artificial success flag was accepted.

The parallel agents have reached the account usage limit and no simulation process is currently running. All completed outputs are saved locally below.

## Canonical repository and state

- Repository: `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm`
- Working software directory: `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software`
- Branch: `main`
- Remote: `https://github.com/RonTuretzky/xlerobot-farm.git`
- The working tree contains other carton changes and uncommitted G4 source files. This handoff commit must not be treated as a claim that every dirty file is committed or that every output artifact is in Git.
- Main local report: `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/index.html`
- Main status ledger: `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/progress.json`

## Requirement-by-requirement status

| Requirement | Status | Evidence boundary |
|---|---|---|
| Human preparation, printed fits, station measurements | Unverified | Layout, masses, friction, compliance, tag mounts, paper sizes, and fixture positions are hypotheses. No physical fit or watering result is recorded. |
| Robot places the trough | Not executed | The full station includes a free trough, but the diagnostic starts with it fixture-initialized. |
| Robot seats the original holder | Not executed | The holder starts human-prepared in the trough. This earns no robot placement credit. |
| Robot places the shared guide | Incomplete | Pickup, transfer, and continuous grasp are demonstrated. Near-seat contact remains invalid; release and hands-off retention are not reached. |
| Four empty carriers descend through the guide | Not executed | Four source carrier bodies and corridors exist, but there is no validated robot insertion trajectory. |
| Four paper strips are fed and the pusher withdraws | Incomplete | The rigid pusher primitive passes as a tool-handling experiment. Paper thickness, buckling, ledge catching, insertion depth, paper release, and the full paper stage are absent. |
| Guide lifts at least 28 mm and parks while retaining carriers/paper | Not executed | No strips or validated carrier/paper retention state exists. |
| Seam roller folds four flaps rightmost-first toward +X | Mechanism only | Passive roller coupon tests pass nominal and half-timestep rolling and reject controls. There is no robot roller grasp or deformable paper folding result. |
| Rectangular growing sheet is placed | Not implemented | No growing-sheet body or paper-to-paper retention score exists. |
| Watering test | Unverified | Water and wicking are outside the current simulation; purchased capillary performance remains unverified. |
| Complete sequence and independent score | Not complete | The scorer rejects missing stages and initialized placement. `full_planter_success` and `physical_success` remain false in all current evidence. |

## Verified simulation evidence

### Guide handling

- `contact03/inset400` independently verifies guide acquisition and lift: 3,001 transitions replay, both arms maintain opposed jaw loads, guide rise is about 59.9 mm, and maximum whole-grasp drift is about 0.173 mm. This is only a pickup/lift primitive.
- `contact08` with the interface-11 cover retains the guide through transfer and hold, then stops during placement at an 8.691 N jaw/guide load. Its first-gate loaded source surfaces are independently checked.
- The slower `contact09` trajectory retains opposed grasp throughout 7.548 s of carry and placement, with maximum whole-grasp drift about 0.251 mm and no unintended arm/environment load. It stops before release at a guide/holder load of about 27.27 N, at roughly 0.086 mm above the nominal seat. Independent source checks find artificial internal-cut normals in that state, so that force is not accepted as physical insertion resistance.
- Candidate16 preserves source unions and 147 functional probes and fixes synthetic pitch cases, but exact replay of the real contact09 failure still finds source-inside outward probes and invalid normals. It is rejected for trajectory promotion.

Primary guide artifacts:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-station-contact03-inset400-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-station-contact08-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-station-contact09-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-station-contact09-stop-source-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-seating16-contact09-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/collision-seating-16/SUMMARY.txt`

### Pusher handling in the complete station

- `pusher-transfer20` applies the exact rigid transform from the proven isolated bench to the full station and samples 380 recorded states. It finds zero unexpected penetration above 1 micrometre and a minimum arm/table clearance of 0.618 mm. It is a privileged static geometry audit only.
- `pusher-transfer21` replays only recorded right-arm joint controls after initialization. The pusher remains a free rigid body and the full station uses its source-mesh inertia. The run completes 9,112 physics steps over 18.224 s, with peak point/body-pair load 1.198/3.624 N and maximum penetration 0.156 mm.
- Independent native replay verifies all 9,112 transitions and the contact streams. The dedicated mechanical scorer reports: 100% opposed loaded grasp during acquisition/carry, 2.817 mm maximum rigid grasp drift, 77.462 mm minimum hold clearance, supported setdown at 12.556 s, 1.1 s released hold, approximately zero released drift/speed, and zero unintended arm resultant. This is a successful simulated rigid pusher-handling primitive, not paper feeding or complete assembly.
- The generic full-assembly source visual audit does not know a pusher role; the separate source audit does. It verifies the compiled `feeder_print_PROTOTYPE.stl` transform and exact original SO101 jaw hashes. Keep both facts in scope.

Primary pusher artifacts:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/pusher-transfer20/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/pusher-transfer21/recorded-controls/run/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-pusher-transfer21-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-pusher-transfer21-native-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-pusher-transfer21-source-01.json`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/planter/g4_station_pusher_score.py`

### Perception and registration

- Registration18 uses actual simulated joint actuation with eight fitting and three held-out poses. Its complete 2,858-step replay and all 11 RGB/depth/encoder captures reproduce exactly; production fit residuals pass the existing contract.
- Its independent absolute camera error is 6.147 mm / 0.429 degrees, so it is not contact-target qualified and was not installed.
- Pixel-only refinement is explicitly a development diagnostic. At 1280 width it reaches 2.652 mm / 0.107 degrees after fitting pixels, but it is not the production fitter and uses no simulator truth during optimization.
- Resolution sensitivity does not solve the problem: 1920 and 2560 renders still have approximately 6.96–7.04 mm production absolute translation error. The 1280 baseline comparison had an expected stream-ID/hash difference; numerical fit and all RGB/depth captures remain exact.

Primary perception artifacts:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration18/left-grid136/run/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-registration18-physics-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/scoring-registration18-captures-01.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/pixel-refinement19/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration-resolution22/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration-resolution22/baseline-identity-review.json`

### Collision-model investigations

The original CAD boundary and contact-normal fidelity are separate gates. The following alternatives are retained as rejected or exploratory evidence:

- Interface-11 removes earlier rim/flange partition artifacts but does not make the whole seating neighborhood valid.
- A 2D rigid-flex shell loses filled-solid containment and can reverse surface crossing normals.
- Conforming 3D tetrahedral rigid-flex restores volume containment but still has invalid internal element-face normals at the actual contact09 state.
- Native MuJoCo mesh SDF has octree approximation error and search-setting sensitivity at the G4 feature scale. Do not interpret its contact `dist` as exact physical penetration without an independent CAD distance check.
- An output-only exact source-triangle SDF plugin passes analytic controls and substantially improves distance error, but its gradient handling still needs source-specific repair and independent actual-state qualification. It has not been promoted into the station.

Collision artifacts:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/collision-interface-11/SUMMARY.txt`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/collision-seating-16/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/rigid-flex2d-01/result.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/rigid-flex3d-feasibility-01/assessment.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/rigid-flex3d-witnesses-01/occupancy-witnesses.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-native-sdf-01`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-exact-sdf-01`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-exact-sdf-audit-02` (field audit)
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-exact-sdf-pairing-01`, `-02` (collider search comparison)
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-exact-sdf-settle-01` (hold-settle per collider)

### Roller and software checks

- The passive roller mechanism passes near-contact nominal and half-timestep rolling and rejects no-friction, locked-bearing, and disabled-motion controls. No robot roller grasp is validated.
- The latest frozen pusher-transfer source snapshot passes 127 focused G4 tests in 3.29 s; frozen hashes remain unchanged. This is software evidence only.
- The staged repository source currently passes 135 G4-focused tests in 6.97 s (`software/.venv/bin/python -m pytest -q software/tests/test_g4*.py`). This verifies code behavior only; it does not change any simulation or physical-success result.

Artifacts:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/roller-independent-audit-02.json`
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/software-tests-pusher21.json`

## Canonical source files

The G4 implementation lives under:

- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/planter/`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tools/diagnose_g4_station.py`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tools/audit_g4_pusher_station_transfer.py`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tools/diagnose_g4_pusher_transfer.py`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tools/audit_g4_registration_resolution.py`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tools/run_g4_batch.py`
- `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/docs/g4-planter-training.md`

G4 CAD and source parts:

- `/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4/`
- `/Users/wk/Downloads/xlerobot-farm-parts/cressmaster-trough.stl`
- `/Users/wk/Downloads/xlerobot-farm-parts/cressmaster-holder.stl`

## Next actions

1. (Continued 2026-10-07, see the end of `software/docs/g4-planter-training.md`: field qualified, gradient fixed in source mode, exhaustive opt-in mesh/exact-SDF collider matches independent overlap to 2e-7 mm; recorded stop states do not transfer between colliders, so the rerun below must start from the initial state.) Finish the exact source-triangle SDF contact audit. Repair the documented skinny-triangle gradient issue, then independently check occupancy, signed distance, source normals, contact search sensitivity, and the exact contact09 state. Keep the original 8 N and 0.2 mm gates. Only after that should one identical guide trajectory be rerun.
2. If guide seating becomes source-qualified, add and independently score guide release/withdrawal and hands-off retention. Do not infer placement from a commanded pose.
3. Implement trough and holder robot placement instead of initialized poses, with the same original SO101 actuator limits and full contact audit.
4. Obtain the required manual paper evidence: carrier fit, off-centre insertion, buckling, ledge catching, guide removal, and roller folding. The paper stage must model paper contact, bending/crease approximation, friction, damping, and release.
5. Add four paper strips, guide extraction with retained paper/carriers, flap folding, and the plain rectangular growing sheet. Keep the independent scorer stage-complete and fail closed when any body/stage is missing.
6. Resolve perception with measured camera/tag/joint offsets. The current registration is a simulated calibration diagnostic, not an installed deployable visual controller.
7. Run failure and sensitivity cases: missing/occluded tags, stale frames, wrong registration, open jaws, zero friction, disabled motion, stronger springback, timestep and solver changes. Compare claws and paddle under the same conditions.
8. Preserve all raw traces, source hashes, commands, contacts, failure reasons, and complete timelines. Avoid new multi-gigabyte runs while only about 11 GiB disk space remains.

## Large local artifacts

These are intentionally kept out of the source commit. Reuse them by absolute path rather than copying them:

- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/station` — about 9.2 GiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/pusher-transfer21` — about 2.7 GiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/pusher-transfer21/recorded-controls/run/physics.jsonl.gz` — about 2.83 GB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/station/contact09/gentle-seat/run/physics.jsonl.gz` — about 1.86 GB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/collision` — about 1.3 GiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-native-sdf-01` — about 778 MiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration18` — about 874 MiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration13` — about 864 MiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-assembly-next/registration-resolution22` — about 237 MiB
- `/Users/wk/Documents/ChatGPT/Hackatuson/output/g4-exact-sdf-01` — about 106 MiB

Do not delete these while the handoff is being resumed; their manifests and hashes are part of the reproducibility record.
