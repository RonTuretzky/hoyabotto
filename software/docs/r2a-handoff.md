# Cress retrofit R2a — robot assembly training handoff

Prepared 2026-10-03, Asia/Tokyo. For the robot implementation/training chat.

This document: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/HANDOFF-ROBOT-TRAINING.md`

## 1. Objective and selected design

Train the user's XLeRobot to assemble the **modified original cress planter, revision R2a**, reusing the already printed original trough. Do not implement the older R0 stack-planter or R1 workcell concept by mistake.

The longer-term user goal is a robot that fully constructs a planter. The present R2a prototype is a deliberately smaller milestone: **a person loads four paper wicks and fixes the empty trough in place; the robot places the prepared carrier, a separate real top sheet, and optionally the retaining frame.** This is not yet construction from loose consumables, automatic sowing, watering, or harvesting. Describe results with that boundary intact.

The user chose to print the whole carrier and test its strength, rather than wait for a coupon-only trial. That is permission for a prototype experiment, not evidence of strength, fit, water transfer, or robot success.

The current plate contains:

1. One modified carrier: the original holder and four plastic inset/wick-pocket pieces fused into one assembly, with two new robot grip fins.
2. One optional loose paper-retaining frame with its own grip fins.
3. One grip coupon for preliminary jaw/fin testing, not an assembly component.

The original trough is reused. No additional planter plate is currently planned for this milestone. A validated locating fixture or paper pickup aid may still require a later print; neither should be represented as already available and proven.

**The fused plastic pockets are not absorbent wicks. Four real paper wicks are still required.** The separate top paper is also still required for this design. The retaining frame is a new experimental aid to keep that top paper down after placement; the original planter did not require it. Test with and without it rather than treating it as an original requirement.

## 2. Read these first

1. Current design contract: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/design.json`
2. Current print preparation and retry record: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/plate-preparation.json`
3. CAD source: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_retrofit.scad`
4. Mesh checks and source hashes: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/out/geometry-report.json`
5. Visual guide: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/index.html` — R2a is slides **39–47**. Browser entry: `file:///Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/index.html#slide-39`.
6. Robot repository instructions: `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/CLAUDE.md`
7. Robot status, setup, and command reference:
   - `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/STATUS.md`
   - `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/SETUP.md`
   - `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/README.md`

For actual hardware bring-up, first follow `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/.claude/skills/farm-bringup/SKILL.md`. This handoff does not bypass that workflow or authorize unattended movement.

There are older “not sliced” / “no print job” strings in the generated slideshow and geometry/render reports. Those describe the earlier CAD-generation stage. The later print record supersedes those status strings; it does not supersede the remaining physical-validation requirements. Likewise, a later hardware result in the robot chat should supersede an older repository status note when supported by actual evidence.

## 3. State at handoff: printing is not physical validation

At approximately **15:19 JST on 2026-10-03**, a read-only check of eufyMake Studio showed Squareship printing `cress_R2a_full_prototype_plate.gcode`, about **03:58:46 remaining**, **50 mm/s**, **1.97 g** reported extrusion, nozzle **230°C**, bed approximately **64°C / 65°C target**, and layer counter **0/275**. This is a timestamped startup/progress observation, not a promise of the state when this handoff is read.

The preceding attempt failed. The user specifically reported the nozzle drawing the print outline **past the outer edge of the sheet**, not merely loose plastic or poor adhesion. Studio then reported “Print Failed.” The user washed and realigned the removable sheet and reported starting auto-leveling; successful leveling was not independently observed. A supervised retry was subsequently started at the user's request with the same G-code.

The read-only G-code audit found planned extruding XY moves inside the nominal 220 × 220 mm build area: X **12.827–207.173 mm**, Y **5.987–213.965 mm**. The file uses the M5C profile, homes, and uses absolute positioning; no explicit XY-origin offset was found. That does **not** prove the printer's physical homing/origin was correct. The off-edge failure's root cause remains unresolved, and the user has not yet confirmed that the retry's outline stayed on the sheet.

The M5C has no camera. Do not claim first-layer adhesion, successful completion, usable parts, or resolved positioning from software telemetry. Before physical training, obtain a completed-print/inspection result and check the actual parts. If a print is still running, do not interrupt it just to begin training work. If another print is needed, require a fresh clear-bed confirmation before launch.

No robot was moved, no policy was trained, and no assembly was physically validated while writing this handoff. Offline implementation and tests can proceed while waiting for the parts.

## 4. Exact assets and provenance

All paths below are absolute local paths on this Mac. If work moves to another computer, copy the needed directories and record the new root; do not assume these paths exist remotely. Do not copy secret environment files with the assets.

### Current editable CAD and printable meshes

| Purpose | Full path |
| --- | --- |
| Design decisions and nominal assembly/grasp frames | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/design.json` |
| OpenSCAD source | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_retrofit.scad` |
| Mesh export and geometry-check builder | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/build.py` |
| Geometry report | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/out/geometry-report.json` |
| R2a carrier | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/out/carrier.stl` |
| Optional R2a retaining frame | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/out/retainer.stl` |
| Carrier-fin grip coupon | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/out/grip_coupon.stl` |
| Original holder used by the derivative | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/source/cressmaster-holder.stl` |
| Original unchanged trough | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/source/cressmaster-trough.stl` |
| Original inset used four times | `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/source/cressmaster-inset.stl` |

All STL dimensions are **millimetres**; print at **100% scale**. Preserve this release before rebuilding: the builder overwrites exported meshes and the report. A design change needs a new revision and new hashes, not a silent replacement beneath an existing training dataset.

The local provenance record identifies Macce's “Self-watering Cress or Microgreens Planter,” Printables model 434505, CC BY-SA 4.0: https://www.printables.com/model/434505-self-watering-cress-or-microgreens-planter . This attribution/license is from the existing local records, not a fresh upstream verification in this handoff. Preserve source attribution and derivative-change notes when sharing the CAD.

### Actual full prototype print job

- Project: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_R2a_full_prototype_plate.3mf`
- Sliced toolpath: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_R2a_full_prototype_plate.gcode`
- Settings, part locations, verification notes, retry history: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/plate-preparation.json`

This release uses M5C 0.4 mm, PLA+ Basic White, Normal, 0.2 mm layers / 0.14 mm first layer, three walls, 10% grid infill, automatic brim, and **supports ON, build-plate-only grid**. Estimated full plate: **4 h 3 m 14 s, 105.61 g**. Do not inherit the old original-planter “supports off” instructions for this fused carrier. Support removal from the wick features is an untested physical risk.

Earlier carrier-only review files are not the active full plate:

- `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_R2a_carrier_supports_REVIEW.3mf`
- `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/cress_R2a_carrier_supports_REVIEW.gcode`

Release SHA-256 values, rechecked against the files while preparing this handoff:

| Artifact | SHA-256 |
| --- | --- |
| carrier.stl | `438fd4cdaa9ca21353d6958f6854e96f7fa7dffeac08d432e9c352c9838348f2` |
| retainer.stl | `e276ba3f244630971ae02e79efd09929df6dd4bda424b1558e2e915ac100de4c` |
| grip_coupon.stl | `33d8bb116c3120c6c6e8c8ee06866d9a6051125afb754888a7da82d73c00eb3f` |
| Full plate 3MF | `c95d92b74958a7ee2fed9e820f562e6e02d40eaaec0a3cb9207ddcc4409bcdcb` |
| Full plate G-code | `ee76470bdab8628a0141b8ebe5705b80dcee0c74c1461ce35642cd3b70733ade` |

### Visual assembly references

- HTML slideshow: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/index.html`
- R2a Blender scene: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit.blend`
- R2a render script: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/render_cress_retrofit.py`
- R2a slide source: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress_retrofit_slides.py`
- Overall slideshow builder: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/build_slideshow.py`
- Render report: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/render-report.json`

Step images:

1. Original: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_01_original.png`
2. Exploded retrofit: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_02_exploded.png`
3. Underside and integrated pockets: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_03_underside.png`
4. Human preparation: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_04_human_setup.png`
5. Carrier placement: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_05_carrier.png`
6. Separate top paper: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_06_paper.png`
7. Frame placement: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_07_frame.png`
8. Completed design view: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_08_complete.png`
9. Illustrative print layout: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/renders/cress_r2_09_print.png`

These are explanatory renders, not demonstrations, calibrated trajectories, verified seed counts, paper-fold templates, or proof of robot reach. The actual 3MF defines the print layout. Use labels and dimensions, not color alone, in further training/debug visualizations: the user is color-blind.

### Original assembly history and older designs — reference only

- Original assembly guide: `/Users/wk/Downloads/xlerobot-farm-parts/cress-assembly-guide/index.html`
- Original guide Blender scene: `/Users/wk/Downloads/xlerobot-farm-parts/cress-assembly-guide/cress-assembly-guide.blend`
- Original guide generator: `/Users/wk/Downloads/xlerobot-farm-parts/cress-assembly-guide/build_guide_v2.py`
- Original download/print handoff: `/Users/wk/conductor/workspaces/research/minsk/.context/handoff-print-cress.md`
- Original parts/fixture notes: `/Users/wk/Downloads/xlerobot-farm-parts/README.md`
- Existing nest candidate: `/Users/wk/Downloads/xlerobot-farm-parts/nest_cress.stl`
- Older robot-planter handoff: `/Users/wk/conductor/workspaces/research/minsk/.context/handoff-robot-planter-assembly.md`
- Older R0 concept: `/Users/wk/conductor/workspaces/research/minsk/.context/robot-planter-v0/README.md`
- Older R0 contract: `/Users/wk/conductor/workspaces/research/minsk/.context/robot-planter-v0/assembly-contract.json`
- Older R1 workcell contract: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/design-contract.json`
- Older workcell scene including illustrative robot: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/robot-farm-design.blend`
- Its renderer and pose report: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/render_design.py` and `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/pose-report.json`
- SO-101 visual asset: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/assets/so101/so101.urdf`
- Visual asset provenance: `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/assets/so101/UPSTREAM_README.md`

Do not follow the original guide's separate-plastic-inset assembly sequence for R2a: those pieces are already fused into the carrier. Do not substitute R0 or R1 object geometry, bottle designs, robot poses, or task contracts for this revision. Rendered SO-101 joint values are not commands for the actual XLeRobot.

## 5. Geometry, coordinate frames, and grasp candidates

### What changed

The CAD unions the original holder with four copies of the original inset at X = **−45, −15, +15, +45 mm**, using connecting collars, then adds two grip fins. The original inset's own source position is X = −45 mm; the source applies the necessary translations for the other three copies. Do not import four copies at a shared origin.

Nominal exported dimensions:

| Part | Overall X × Y × Z, mm | Intended grasp feature |
| --- | --- | --- |
| Carrier | 177.977 × 83 × 43.5 | Two 24 × 6 mm section fins |
| Retaining frame | 148 × 66 × 18 | Two 4 × 20 mm section end fins |
| Grip coupon | 36 × 24 × 25 | 24 × 6 mm fin, carrier-grip trial only |

The frame ring is 2.4 mm thick and has a 140 × 58 mm opening. The starting top-sheet template is **146 × 64 mm**, with a nominal 1 mm stack allowance. Actual medium type, thickness, moisture condition, and required sheet count are unresolved; measure and record them. The render's paper thickness/folds are schematic. No seed count or water dose has been validated for R2a.

### Assembly datum versus exported print coordinates

Define assembly frame A using the original source XY coordinates and original holder deck top at **Z = 0**. Keep the original trough in that source frame. All three transforms below have identity rotation:

| Mesh coordinates | Translation to assembly frame A, mm |
| --- | --- |
| Original trough STL | (0, 0, 0) |
| Exported carrier STL | (0, 0, −20.5) |
| Exported retainer STL | (0, 0, +1.0) |

Equations:

```text
p_A_mm = p_carrier_STL_mm + [0, 0, -20.5]
p_A_mm = p_retainer_STL_mm + [0, 0,  1.0]
p_A_mm = p_trough_source_mm
```

The CAD exporter raises the carrier by +20.5 mm so its feet sit at print Z = 0. The design JSON's `carrier_export_translation` describes the inverse, export-to-assembly placement above. Do not apply +20.5 again during assembly import.

The renderer adds a common +23.5 mm world shift and additional exploded-view offsets. Neither is a robot calibration. Slicer XY locations are also unrelated to the assembly station.

For robot-base coordinates in metres, measure a rigid transform `T_robot_from_A` and then use:

```text
p_robot_m = T_robot_from_A * homogeneous(0.001 * p_A_mm)
```

No measured robot-to-fixture transform, table plane, tool-center-point transform, camera extrinsics, safe approach height, grasp orientation, or collision-free trajectory is supplied. Do not invent them from a render. Define source-station frames for the loose carrier, paper, and frame separately from their assembled destinations.

### Nominal grasp candidates, not executable poses

- Carrier: assembly-frame points **(0, −36.5, 12)** and **(0, +36.5, 12) mm**; equivalent exported-STL Z is 32.5 mm. Fin thickness is 6 mm along Y; candidate jaw closure is across that thickness. Fins extend X = −12…12 mm and assembly Z approximately −1.5…23 mm. Choose the accessible side after real collision/reach checks.
- Frame: exported-STL points **(−72, 0, 10)** and **(+72, 0, 10) mm**; assembled Z is 11 mm. Fin thickness is 4 mm along X and length 20 mm along Y. Candidate jaw closure is across X. Frame placement and gripper release must clear the carrier fins and paper.
- The coupon tests a 6 mm fin only. It does not validate the thinner frame fin, carrier center-of-mass torque, or the strength of the actual printed carrier fin root.

There is only about **0.5 mm nominal lateral gap** between the frame's Y edge and each carrier fin's inner face. Bench-test printed fit and real-paper seating before treating this as an easy drop-in action. Chamfers, tolerances, and compliant insertion behavior are not established.

The geometry report records manifold/consistent, single-component output meshes and essentially zero static intersection volume for the intended seated assemblies. These checks are not proof of printability, dimensional tolerance, gripper clearance, swept-path clearance, reachability, strength, or water function.

## 6. Physical setup and qualification gates

Before enabling an assembly policy:

1. Confirm the print completed; remove supports/brim and inspect for cracks, warped seating surfaces, blocked wick pockets, and damaged fins. Preserve a photo and identify the particular printed instance.
2. Dry-fit the actual carrier to the actual original trough by hand. Check level seating and refill-bay orientation. Do not sand/modify a part without recording the change for the dataset.
3. Test the coupon grip, then actual carrier and frame grips over a protected catch area under supervision. Determine grip closure/force, slip and deflection, lift stability, and release clearance. Do not reuse the bottle's grip threshold.
4. Load four real folded paper wicks by hand. Verify that each can contact the separate top medium and reach the intended water region without support debris obstructing it. A separate bench capillary test is needed before calling the planter functional.
5. Secure the empty trough in a measured, non-sliding fixture. The older `nest_cress.stl` is only a candidate: its notes assume a plain rectangular trough, while the actual trough has an offset refill feature. Verify the full outline and fit before using it. Do not rely on its name as compatibility evidence.
6. Establish repeatable source stations for the prepared carrier, one declared top-sheet stack, and the frame. A workable paper-pick presentation fixture is not yet designed or tested. If real paper cannot be picked reliably, record that blocker; do not quietly swap in a rigid printed sheet and call the task solved.
7. Resolve robot hardware/calibration status with the robot chat, bind cameras by real device identity, measure fixture and tool frames, and validate workspace/clearance with the wheels disabled.
8. Start dry: no water or seeds in robot assembly trials. Keep other arm and cables clear. Wet functional tests are a separate milestone, away from electronics.

The two similar carrier fins and frame end fins can create orientation ambiguity. Use a measured station constraint and visible orientation cue tied to the trough/refill feature; these are not mechanically keyed parts. Existing tray tag IDs 1 and 2 are already used in the farm profile—allocate any new tags explicitly rather than reusing them accidentally.

## 7. Proposed assembly state machine and evidence

Implement these as explicit bounded stages with a postcondition before advancing. They are a proposed contract, not existing working robot skills.

| Stage | Action | Required evidence to advance |
| --- | --- | --- |
| PREFLIGHT | Check calibration, fresh cameras, empty secured trough, correct part revision, four preloaded wicks, paper and frame present, empty gripper | Logged setup, visible parts/station, valid safety checks; no stale frame or unknown held object |
| PLACE_PREPARED_CARRIER | Approach the validated fin, grasp, lift level, transfer above trough, lower along verified path, release, retreat | Carrier stays in gripper during transfer; trough did not move; carrier seated level and remains after release |
| VERIFY_CARRIER | Inspect seat and wick state before covering them | No tilted seating, caught/broken wick pocket, displaced wick, or retained gripper contact; uncertain hidden wick state requires inspection |
| PLACE_REAL_TOP_PAPER | Pick one specified sheet/stack from its presentation station and lay it onto the carrier | Correct declared quantity, orientation and coverage; no dropped/folded-off sheet, double pickup, or sheet still stuck to jaws |
| PLACE_RETAINER | If the run's declared variant includes it, grasp frame fin, align, lower gently, release, retreat | Frame rests evenly on the intended paper stack without trapping/moving the carrier or paper; jaws release cleanly |
| VERIFY_DRY_ASSEMBLY | Inspect final arrangement and retreat to a validated safe pose | Carrier seated, paper covers target area, frame state matches variant, gripper empty, no fixture displacement or damage |

Declare the variant before each episode: `R2a_no_frame` or `R2a_with_frame`. Do not silently omit a failed frame stage from a nominally complete episode. A rigid-part practice run without real paper must be labeled separately and is not a completed planter assembly.

Do not define success as “joint target reached,” low prediction error, or a rendered pose match. Use object-state evidence from cameras plus human validation of the initial verification method. A joint-position-only callback cannot see whether a sheet was dropped.

Failure handling must be explicit:

- Lost camera, stale observation, unexpected load, fixture movement, tilt, slip, or suspected collision: stop progression and preserve evidence.
- Unknown held-object state: do not automatically open the gripper or execute a generic park path. First determine a safe recovery with supervision; do not drop a held assembly onto the work area.
- Misaligned carrier/frame: do not push harder or run blind insertion retries. Recover only through a validated path with bounded attempts; otherwise request physical reset.
- Failed or double paper pickup: mark the episode failed/intervened and reset the declared sheet stack. Do not contaminate successful demonstrations with hidden human correction.
- A human intervention ends the autonomous-success claim for that episode, even if assembly is eventually completed.

Keep all existing hardware limit/watchdog/emergency-stop protections. No new high-force insertion behavior is proposed.

## 8. Robot repository and current implementation facts

Canonical checkout: `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm`

Software root: `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software`

`/Users/wk/conductor/workspaces/research/cheap-plant-robots` currently resolves to `/Users/wk/conductor/workspaces/research/minsk`; it is an alias, not a separate implementation to keep in sync.

Read-only inspection found repository HEAD `bb30bb3f26b34575f351ed65f7f948c516eed614` and no working-tree changes at that moment. Recheck before editing; the other robot chat can advance this checkout. This handoff does not require checking out that commit or overwriting newer work.

The repository identifies a **WowRobo XLeRobot 0.4.0 two-wheel kit**, two SO-101 follower arms with 12 V STS3215 servos, a two-servo head, three USB cameras, two controller boards, and an IKEA-cart mounting arrangement. Treat wiring, camera readiness and calibration statements in STATUS as recorded history, not a live hardware inventory. The profile still has blank serial ports and placeholder camera matches. This handoff did not probe hardware.

The checked-in status decision removes hand-teaching/teleoperation from the default workflow. Do not introduce a leader arm or ask the user to teleoperate demonstrations as if that were already agreed. Start from a bounded autonomous/vision-verified scripted baseline and collect verified rollouts from it; if that cannot produce demonstrations, report the blocker or seek a changed plan.

### Relevant implementation paths

| Area | Full path |
| --- | --- |
| Hardware profile to inspect, not blindly enable | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/profiles/paper-tray-v0.yaml` |
| Simulation profile | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/profiles/sim.yaml` |
| Runtime and recorder wiring | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/system.py` |
| Episode recording | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/learning/recorder.py` |
| Training entry points | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/learning/train.py` |
| Offline evaluation | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/learning/evaluate.py` |
| Inference adapter | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/learning/infer.py` |
| Learned-policy skill | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/skills/policy.py` |
| Bounded skill runner and grip state | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/skills/runner.py` |
| Existing keyframe path | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/skills/keyframes.py` |
| Existing visual-servo path | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/skills/llm_servo.py` |
| Fiducials | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/perception/tags.py` |
| Robot adapter and command units | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/adapters/robot_lerobot.py` |
| Camera adapter | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/adapters/camera_opencv.py` |
| Safety rules | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/safety/rules.py` |
| Existing farm state machine, not yet assembly-specific | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/cycle/machine.py` |
| Evidence storage | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/evidence/store.py` |
| CLI | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/cli.py` |
| Viewer / stop controls | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/farm/viewer/app.py` |
| Training background | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/docs/research-training.md` |
| Recorder tests | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tests/test_recorder.py` |
| Policy integration tests | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tests/test_remote_policy.py` |
| Cycle tests | `/Users/wk/conductor/workspaces/research/minsk/.context/xlerobot-farm/software/tests/test_cycle.py` |

Do not repurpose the active watering profile in place. Add an isolated, initially disabled R2a assembly configuration and tests, respecting current repo instructions. Existing tray refill targets, bottle grips, and pouring checkpoints are not R2a assembly parameters.

### Integration issues to resolve before collecting training data

These are code/configuration observations at the above HEAD, not changes implemented by this handoff:

1. **Recorder/policy dimension mismatch.** The system recorder is wired for 14 positions: six left-arm, six right-arm, two head joints. The selected policy profile lists only six right-arm state joints. Explicitly choose the controlled arm and ordered state/action vectors, then make recording, training and inference agree. Do not append R2a recordings to an incompatible existing dataset.
2. **Missing-state padding.** The system recorder uses fallback values for missing positions/actions, including zero or current state. The recorder's own key checks cannot detect values already filled upstream. Fail closed on unavailable/stale controlled joints; log actual observations and the command actually sent after safety clamping.
3. **Camera-key mismatch risk.** The profile maps `observation.images.wrist` to `right_wrist` and `observation.images.front` to `head`. Some prior checkpoint notes refer to an overhead input. Read the actual checkpoint feature specification; do not assume `front`, `head`, and `overhead` are interchangeable. Bind real devices, verify views and synchronization, and reject missing frames rather than pad them.
4. **Timing is not automatically uniform.** Recording is attached to runtime ticks/moves and rate-limited. Store observation, image, and action timestamps, actual intervals and dropped-frame information. A nominal 10 Hz setting is not evidence of synchronized 10 Hz demonstrations.
5. **Task/stage conditioning is not implemented by a task label alone.** The current policy call consumes state and images; the skill's goal text is not automatically passed as a learned conditioning input. Choose separate stage policies or implement and test explicit conditioning. Keep the high-level state machine responsible for preconditions and outcomes.
6. **Success checking needs object observations.** The current policy completion callback receives joint state. Add visual/object-state verification for assembly rather than declaring success from joints. Add real per-stage deadlines; the current policy loop's documented time bound is not a substitute for its implemented max-step/stall controls.
7. **Grip thresholds are task-specific.** Existing runner thresholds describe bottle/paddle usage. Measure a 6 mm carrier fin, 4 mm frame fin and deformable paper independently; record closure units and a reliable empty/held/released test.
8. **Offline evaluation is not independent assembly evidence.** The existing evaluator reports `held_out: false` and first-action error on selected episodes. Establish a genuinely excluded validation/test split by recording session/part/setup before training. Do not report that existing metric as held-out physical success.

The profile currently keeps policy execution disabled, shadow mode true, wheels disabled, and limits including 6-degree maximum skill steps, relative-target bound 8, a 0.5 s watchdog, servo temperature limit 55°C and load limit 800. Preserve or tighten the effective safety limits; do not increase them to force a fit or make a rollout pass. Confirm the adapter's actual calibrated command units before issuing any joint target—render radians, CAD millimetres and runtime joint values are different quantities.

The existing learned pouring checkpoint is not an assembly policy. The repository also records the remote policy-server approach as parked pending better checkpoint evidence; this task does not require enabling it.

## 9. Training and evaluation plan

### A. Offline contract and tests first

Create an R2a-specific stage controller, object/fixture definitions, explicit controlled-joint and camera schema, and success/failure event logging. Leave hardware execution disabled. Test frame/unit conversions, wrong-revision rejection, missing/stale sensor data, malformed/non-finite actions, limits, deadline expiry, stage transitions, unknown held object, paper double-pick, and intervention accounting. Mesh collision checks and simulated tests should be labeled as such, not robot trials.

### B. Qualify the station and bounded baseline

After successful print inspection and hardware bring-up, measure the actual fixture/TCP/cameras and validate approach, grasp, lift, transfer, seat, release and retreat separately at conservative speed under supervision. Start with the 6 mm coupon, then the actual carrier, then frame. Do not skip real paper: it is likely the most difficult manipulation step and needs its own presentation/pick validation.

Use the existing bounded motion/safety infrastructure. A vision model may propose a stage or assess evidence, but unconstrained text must not become arbitrary motor commands. A failed physical prerequisite is a design/setup issue to fix, not something to hide with more imitation learning.

### C. Collect demonstrations only after a baseline works

Use a new dedicated R2a dataset. Start with a small interface-validation pilot, inspect every recorded field and replay offline, then expand data across measured allowable placement variations. No fixed demonstration count is claimed sufficient. Keep failed rollouts for analysis with explicit labels; don't mix corrected trajectories into unqualified successful demonstrations.

Minimum episode metadata:

- Revision and mesh hashes; actual printed instance, material/profile, support cleanup and any manual modifications.
- Variant with/without frame; stage name, goal, start/end time, success/failure reason, retry count and human interventions.
- Human-prepared four-wick scope; real top-medium type, cut dimensions, thickness/stack count, moisture condition; original trough instance and orientation.
- Robot/calibration identity; tool/jaw geometry; joint ordering and units; commanded and measured state; safety-clamped actions, temperatures/load/stop events where available.
- Fixture/source-station transforms with units and calibration date; camera identities/intrinsics/extrinsics and image timestamps.
- Pre-grasp, post-lift, seated, released and final verification images; confidence/unknown states rather than fabricated certainty.
- Dataset schema version, code commit, train/validation/test allocation and any later relabeling.

Do not train on Blender frames and describe them as real successful demonstrations. Synthetic data can be a separately labeled perception aid if evaluated against real observations.

### D. Learn only the skill that needs learning

First compare the bounded baseline against any learned candidate. Reuse the repository's ACT path only after schema/action/camera checks pass. Begin with one stage at a time; rigid carrier placement, flexible paper placement and frame seating have different failure modes. Keep outcome verification outside the policy. Run offline and shadow checks before supervised hardware evaluation.

Do not enable a learned candidate merely because loss decreases or it beats a hold-still model. Require physical trial evidence under the same stated starting conditions as the baseline, with untouched test sessions and all interventions counted.

### E. Report milestones honestly

| Milestone | What counts | What does not count |
| --- | --- | --- |
| R2a-D0 rigid handling | Verified carrier/frame grasps and placements on real printed parts | A rendered motion or coupon grip alone |
| R2a-D1 dry assembly | Prepared carrier + real paper + declared frame variant assembled from fixed source stations, without mid-episode intervention | Human wick preparation being described as automated; silently omitting failed paper/frame steps |
| R2a-W1 functional planter | Separate bench evidence of seating, capillary transfer and appropriate water/media behavior | Dry assembly alone or water merely touching plastic |
| Future full construction | Robot prepares/loads all required consumables, with watering/seeding added and validated if included in the goal | Renaming the current human-prepared scope “fully autonomous” |

Suggested initial engineering gates, to be agreed before testing rather than presented as established performance: at least 18/20 successful trials for each placement stage and 9/10 full dry assemblies within a declared narrow setup envelope, with no collision/limit violations and all interventions counted as failures. These are prototype gates, not proof of general reliability. Report trial counts, setup range and every failure; any safety fault blocks promotion regardless of success percentage.

## 10. Requested first deliverables from the robot chat

1. Reconcile this handoff with the latest actual robot status and report what is still missing: finished/inspected parts, fixture fit, camera identities, calibration, grip tests or paper presentation.
2. Implement the disabled R2a stage/dataset contract and tests in the existing robot repository, without overwriting concurrent work or enabling the old pouring policy.
3. Resolve the recorder/action/camera-schema mismatches before saving demonstrations. Define a separate dataset and explicit held-out split.
4. Produce a measured station/grasp plan with diagrams/photos and actual transforms, distinguishing nominal CAD points from tested robot poses.
5. Build and evaluate the bounded baseline; collect validated data and train only after the prerequisites pass. Keep the longer-term goal of loose-consumable construction visible, but do not expand to water/seed handling silently.
6. Hand back code/config paths, commit/schema identifiers, test results, physical trial evidence, failures/interventions, and the next blocked milestone. Do not claim training complete from an artifact, launch screen, simulation, or offline score.

No hardware actions, printer changes, training runs, commits, or cross-chat messages were performed as part of creating this document. The user's request here was the handoff itself.
