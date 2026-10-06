## Handoff stop condition — 6 October 2026

After updating main and reconciling local changes, the requested read-only `farm servo-protection -p paper-tray-v0` check returned no status packets for the servos while `lsof /dev/cu.usbmodem*` was empty. Hardware work stopped at that read failure. No protection-register writes or power-cycle verification were performed. The earlier successful table below is dated evidence, not a fresh post-update success; do not report “ALL DONE” or temperature sensing removed. Calibration remains paused, motors were last verified released, and the cause of the silent read is not established.

The read-only CLI now skips calibration handshake writes and disconnects without changing torque. A failed second-bus connection closes the first successfully opened bus.

## Paddle end-contact progress — 2026-10-06

The [new matched audit](docs/carton-paddle-end-contact-audit.md) reproduces both
short flaps held by a left claw and a free paddle in the right jaws. An explicit
60 mm grip position and −71° initial grip rotation allow blade-end contact.
Maximum box translation is 4.96 mm, versus 12.81 mm in the bare-claw run.
The same empty free carton, resistant hinges, table position, forces and limits
apply. No weld, clamp, guide or tape was added; grip pickup is untested.

Both full GIFs include the release test: the left flap springs back to about
58° in both. Neither completes all four folds or hands-off retention. Planning
a jaw to hold the minor while the other folds the major has not produced a
validated dynamic hand transfer. Full closure and tape application remain open.

Fresh rendered paddle tags now register its blade-end TCP without moving the
original slip reference. Tracking verification measures the actual free tool,
not just its virtual gripper target. All 88 folding tests pass. Material values,
external camera and physical tag mounts remain unmeasured assumptions. No
hardware was accessed. This updates the old paddle result immediately below.

## Resistant carton hand transfers — 2026-10-06

The [new transfer audit](docs/carton-minor-transfer-audit.md) reproduces both
short flaps held by the bare claws: 92.44° left and 91.92° right, with maximum
free-box movement 12.81 mm. Releasing the left hand makes that flap reopen to
58.40° after withdrawal and a two-second wait. The camera independently sees
the spring-back. This is a partial stage, not a full folding success.

The matched paddle attempt still stops at 15 mm tool slip before completing
the first short flap. Its gripper contacts the near flap before the blade
contacts cardboard. Alternate approaches have not fixed that failure. Trials
transferring a claw to the near major flap exposed interference from the other
forearm and large box motion; no major-flap transfer is validated.

Both complete attempt GIFs include failures. The free carton, empty load,
resistant hinges, original motors and all physical coefficients are unchanged.
The new 6 mm path clearance affects a copied planning model only. Proposed
paddle tags now provide fresh offline RGB-D tool poses without changing the
grip-slip reference. Physical tags/mounts remain unverified. 80 focused tests
pass. All four folds, final release/retention and explicit tape application
remain unfinished; no hardware was accessed.

## Corrected carton physics and bracing — 2026-10-06

The [new audit](docs/carton-braced-folding-audit.md) supersedes the folding
success claims below. An all-inward starting pose interpenetrated neighboring
flap panels by about 11 mm. The initializer now rejects that pose. The default
starts separated panels without changing spring rests, friction or resistance.
Historical recorded passes must not be treated as validated folding results.

The matched corrected trials use an empty 272 g free carton and resistant
hinges. Neither completes all folds or hands-off retention. Bare claws brace
the box and fold one short flap, then stop at an unreachable grasp transition
(45.764 s; maximum box movement 12.19 mm). The right-hand paddle slips during
contact (32.366 s; box movement 26.45 mm). Full attempt GIFs include both failures.

A declared solver sensitivity check distinguishes soft-contact creep from
physical grip failure: the NoSlip preset holds the stationary paddle for 30 s
at both 2 ms and 1 ms timesteps; zero friction still fails immediately. Both
folding variants use the same preset. Regression tests confirm that the box
still slides and the flaps spring open, with unchanged physical coefficients
and motor limits. 65 focused tests pass. Parameters remain unmeasured.

The proposed external camera and 45 mm carton side tags 21/22 improve rendered
tracking; neither is verified on the real station. This remains an offline
RGB-D experiment with independent simulator contact checks and obstacle
snapshots. No hardware was accessed or calibration changed. Regrasp planning,
all remaining folds, explicit retention and tape application remain unfinished.

## Historical paddle versus claws comparison — 2026-10-06

[Four matched offline cases](docs/carton-paddle-comparison.md) compare bare
claws with a left claw and an actual-CAD free-body paddle in the right jaws.
Bare claws complete the loaded weak-crease reference, including withdrawal
and five seconds without hand contact. The paddle variant slips in that case;
neither method closes the empty resistant box. An ideal rigid tool attachment
also fails and is labeled as a separate diagnostic. No general tool winner or
physical validation is claimed. The paddle planner still has face-angle/reach
limitations, and neither strategy implements deliberate box bracing.

Existing paddle grasp work supplied a verified simulated initial grip pose;
the comparison does not simulate pickup at this station. Material, friction,
tool mass and station dimensions remain assumptions. Full-duration paired
GIFs include whole-robot and close views, with explicit failure timestamps.
102 focused tests pass. No hardware was accessed or safety limits relaxed.

## Historical unbolted carton and springback update — 2026-10-06

The [new free-box and material tests](docs/carton-free-box-springback.md) expose
limits of the previous passing GIF. The box was already a free body, but the
passing assumptions included 960 g contents, friction 0.7 and weak creases.
The loaded weak-crease reference still passes withdrawal and a five-second
hands-off hold. Eight empty/lower-friction/stronger-crease sensitivity variants
fail; the empty weak-crease box moves about 35 mm horizontally in recorded
frames. Separate explicitly preset-closed material tests show stronger creases
springing open without any hand contact. Those are not robot-folding successes.

Material/load/friction parameters and per-flap resistance are configurable.
Zero contents mass removes both weight and support. Overall success now also
requires retention during withdrawal and after hands release; held closure is
reported separately. All material values remain unmeasured. Stock fingertips
are still used, and no brace-and-paddle controller has been validated. RGB plus
aligned depth remains required for the folding controller; the material-only
probe does not use perception. 94 focused tests pass. No hardware was accessed.

## Historical two-hand carton folding simulation — 2026-10-06

The [corrected rear-cart simulation](docs/carton-diagonal-folding.md) closes all
four flaps with both arm bases **150 mm behind the table edge** and **60 mm above
the tabletop**. A fixed three-tray cart has collision geometry and 35 mm front
clearance. The box is rotated 30 degrees and translated so its nearest bottom
corner stays 10 mm onto the table. These dimensions are assumed, not measured
from the photos. Stock SO101 fingertip geometry is used; the white attachments
visible in the photos are still unmodeled.

Both real arm models contact two flaps each. All four passive flaps stay within
5 degrees of horizontal for every 2 ms step of a two-second hold. Final angles:
91.843, 91.843, 90.315 and 90.302 degrees. Forbidden robot penetration: 0 mm.
Nominal and +/-5 mm sideways box trials pass. Six negative controls reject a
success claim. A heavier depth-noise/dropout trial fails; reliability is not
established. The filled-box model uses contents to support the short flaps;
empty-carton closure, taping and hands-free retention are not demonstrated.

The strategy uses observed tags/depth, not a learned Molmo policy. Table tags
1 and 20 jointly register a stationary camera; fresh anchor geometry and carton
tag 10 are required during folding. A [supplemental ID 20 A4 sheet](docs/assets/bimanual-table-anchor-20-a4.pdf)
is included. Run `tools/simulate_bimanual_folding.py --strategy diagonal` with
the explicit dimensions in the guide. No hardware calls or calibration changes
were made. [Independent results and known failures](docs/evidence/bimanual-folding-rear-cart.json).

The original success put bases over the tabletop. It remains explicitly
historical under `--reference-layout` and is not evidence for the photographed
station. The [station audit](docs/carton-folding-station-audit.md) retains that
correction and the straight-box reach failures.

## Servo protection readback — 6 October 2026

Read-only verification at 2026-10-06 13:30:02 JST: all 16 servos responded and all torque-enable readings were zero. The servos' own EEPROM protection was still at the factory values below; `farm servo-protection --write` has not been run yet (see `ROBOT_HANDOFF_2026-10-06-servo-protection.md`).

| Servo | Temperature limit (°C) | Unloading mask | LED alarm mask |
| --- | ---: | ---: | ---: |
| left_arm_shoulder_pan | 70 | 44 | 47 |
| left_arm_shoulder_lift | 70 | 44 | 47 |
| left_arm_elbow_flex | 70 | 44 | 47 |
| left_arm_wrist_flex | 70 | 44 | 47 |
| left_arm_wrist_roll | 70 | 44 | 47 |
| left_arm_gripper | 70 | 44 | 47 |
| head_motor_1 | 70 | 44 | 47 |
| head_motor_2 | 70 | 44 | 47 |
| right_arm_shoulder_pan | 70 | 44 | 47 |
| right_arm_shoulder_lift | 70 | 44 | 47 |
| right_arm_elbow_flex | 70 | 44 | 47 |
| right_arm_wrist_flex | 70 | 44 | 47 |
| right_arm_wrist_roll | 70 | 44 | 47 |
| right_arm_gripper | 70 | 44 | 47 |
| base_left_wheel | 70 | 44 | 47 |
| base_right_wheel | 70 | 44 | 47 |

The calibration goal is paused at the user’s request. Both wheel actuators were verified with a bounded pulse, followed by zero velocity, torque release, and settings restoration; calibrated odometry was not established. Arm calibration remains incomplete: failed attempts changed some arm homing/limit registers, while the saved calibration file remains the previous version. Do not claim that saved arm calibration currently matches hardware. Complete guarded calibration or deliberately restore the recorded register backups before normal arm operation. The head was not calibrated or moved.

Legacy task-local `work/carton_session.py`, `work/temperature_confirmation.py`, and `work/left_claw_prepare.py` were renamed with `.retired-2026-10-06`; state and logs were retained. The guarded PR #3282 launcher now lives in `scripts/carton_robot/`, with strict packet validation, bounded limit seeking, and verified-release cleanup; its temperature cutoff was removed on 2026-10-06 like everything else that read servo temperature.

The earlier session notes below are historical; they do not supersede this readback or establish current arm calibration.

## Current verified state — latest hardware session

- Power connection restored (5 October 2026, supersedes earlier power-feed blocker): user exchanged both battery-side USB-C power connections after exchanging the barrel plugs. Fresh readback at1791165058: all16 respond, torque0, status0,11.9–12.2V; saved arm/head calibration matches. Both power leads therefore work in this arrangement; a negotiation/contact issue is plausible, but no failed component has been established. Keep the working connections. Right elbow remains outside saved range (3186 vs3092 maximum); user asked for a supported manual bend before controller startup. Supplied head/right-wrist capture publisher restarted as solePID64122, both coherent5.02fps streams passed20s audit. No new motion, grasp, or fold.

- Current carton hardware diagnosis (5 October 2026): the communication failure follows the original right-side 12 V power feed. After exchanging the two round power inputs, right-arm IDs 1–6 and wheel IDs 9/10 reply normally through the USB hub (11.8–12.0 V, 26–28 C, status 0, torque 0), while left/head IDs 1–8 no longer reply on the suspect feed. This isolates the failing path to that power lead, its supply output, or their connection; the individual component is not yet established. Direct USB and a read-only host-baud scan had not restored right-side replies. User is checking the suspect lead at its battery USB-C end. No motion was commanded. Right elbow is currently 3186, beyond its saved maximum 3092; calibration readback matches. Head video remains live; right-wrist video interrupted during USB changes is being restored. No current local visual model, paddle grasp, or carton fold is validated. Evidence: task work/carton-swapped-feeds-1791164281.json and outputs/Carton-Utility-Commissioning.json.

- Carton utility commissioning (4 October 2026): switched from chat-by-chat joint nudges to isolated worktree `work/carton-visual-controller`, pinned commit `0e17b22e5a00cfbfda09bcdf6cbd91abf3d93a46`. Live environment/calibration and dirty source preserved. Utility tests:58 passed,8 explicit optional solver skips; rendered simulation ALIGNED_ONLY in17 corrections, max2.0003px, no robot/contact physics. All16 pinned URDF/model assets SHA-verified. Isolated Python3.12 environment excludes system packages, but pinned dependency installation failed on package-server network/DNS errors, so actual solver tests did not execute. Supplied Swift camera publisher compiled and is the sole head/right-wrist publisher.20s audit PASSED:5.0226fps each, maximum gap0.19974s, pair skew0.23297s. No utility motor commands. All16 motors released at read-only preflight; right elbow3198 exceeds saved3092 by106ticks (~9.32deg). No further automatic recovery under this direction. Prepare/seed ran, inspect correctly refused absent a healthy active owner. Live feature experiment remains unvalidated: wood anchor failed, printed anchor tracked, then paddle reposition invalidated target seed. Head view crops the claw. User asked to support/reposition right elbow and tilt head camera down; reseed afterward. Evidence under isolated ignored `software/data-carton/commission-20261004-mac-01/` and `agent-validation-20261004-01/`; user summary in task outputs/Carton-Utility-Commissioning.json. No paddle grasp or flap fold achieved.

- Carton approach in progress (4 October 2026): no flap folded and no verified paddle pickup yet. Head aimed down to include the box and paddle; head raw tilt2476/pan2084 at latest read-only preflight. Task-local `work/carton_session.py` provides parked, single-arm raw encoder steps with STOP, calibrated travel checks, low speed/torque, motor health checks and automatic release. Runtime state is `work/carton-session/status.json`; inspect its timestamp and phase before connecting another process. Do not treat an older preflight as current torque state.
- Wired camera streaming works at640x480/5fps input using exact AVFoundation identities: head0x12400005a39230, right wrist0x12200005a39230, left wrist0x12140005a39230. Right identity was additionally confirmed by a wrist-roll step and matching image rotation. Task streamer currently opens head plus the working wrist, yielding roughly2.5fps per saved image; left wrist is not continuously active. Explicit format must be reapplied after session.startRunning(), which otherwise resets it. These facts supersede older camera-failure and high-aim notes below. Phone images are supplemental.
- User requested more tolerance for camera upload gaps. Stationary carton-arm controller now allows10s age before a new movement,15s while moving,30s holding; it uses both wired head and active wrist timestamps. Hardware load/travel/STOP checks remain active. Seven task-local controller tests passed. No wheel behavior changed.
- Cartesian carton integration remains unvalidated: default real adapter uses range-normalized motor positions, whereas ArmModel treats them as degrees. Automatic calibration also does not establish the model's physical joint zeros. Separately, vendored FK used the wrong lower-link angle; changed theta1+theta2-pi to theta1-theta2, with six roundtrip/workspace regression tests and22 relevant tests passing. This mathematical correction does not validate physical Cartesian control. Current hardware approach uses observed raw joint steps rather than that model or the ACT checkpoint.

- Latest head-camera port change SUCCESS (4 October 2026): exact USB identity0x12400005a39230 delivers a fresh native AVFoundation image, visually confirmed as head overview of open carton (task outputs/Carton-Scene/head-new-port.jpg). Keep this port. Prior hub-port failure0x12130005a39230 does not apply to this working connection. Camera numerical mapping is still unverified; view remains aimed high. No motors accessed during camera check.

- Head-camera hub retest after direct-adapter success (4 October 2026): user moved head back to hub. USB identity0x12130005a39230 is detected, but exact-ID native AVFoundation capture again received no frame within8s. Same capture method succeeded on direct adapter identity0x11100005a39230. Current hub connection is NOT a usable head feed. Preferred continuation: return head to verified direct adapter while wrists stay on hub. Hub/port/power/bandwidth cause is not individually isolated. No motor commands sent.

- Head camera recovered via the user's USB-A to USB-C adapter (4 October 2026). Exact USB identity0x11100005a39230 returned a fresh native AVFoundation image; visually confirmed head overview of open carton. Earlier head-no-frame notes are superseded for this connection. Image: task outputs/Carton-Scene/head-adapter.jpg. Current view is aimed high and crops the near box/work area; aim downward before carton work. Camera indices remain unverified after reconnections; do not reuse old numeric mapping blindly. No motor ports accessed during this camera test.

- Carton preparation (4 October 2026): fast-forwarded to origin/main 6bac20e while preserving all pre-existing local control/calibration edits; backups are in task work/carton-integration-backup. Downloaded published carton-act-8000 (191775973 bytes), archive and all internal SHA256 checks passed. Synthetic PolicyRunner inference on MPS passed with action(12,), chunk(100,12), front/top inputs. This did not access motors and is NOT physical task validation. Base suite:187 passed,3 cache-permission failures; all3 pass when HF_DATASETS_CACHE is placed in task work/test-datasets-cache. Carton CLI now disconnects on connect/check/teach/viewer failures, individual teaching starts STOP viewer, batch teaching ends on first failed pose, and viewer startup must be confirmed.22 targeted carton/cleanup tests passed. No carton movement or real poses have been recorded. Missing physical integration: head camera stream and station measurements/clear views; ACT front/top mapping and robot-specific task competence remain unvalidated.
- Current camera diagnosis: three external USB cameras now detected. Both wrist scenes captured; camera index ordering changed and must be reverified before profiles are used. Newly reconnected expected head USB identity0x12130005a39230 yields no frames with exact-ID native AVFoundation capture at640x480 or native preset, despite permission authorized. Same exact-ID code succeeds on wrist0x12140005a39230, so camera connection/stream path needs isolation (direct-to-Mac USB recommended). Do not treat OS detection as a working head image. OpenCV numbered captures returned duplicate wrist views; no new camera mapping saved. Phone last-frame age became stale during prep; refresh before physical work. No motor ports were opened in carton preparation. Evidence: task outputs/Carton-Camera-Diagnosis.json, Carton-Model-Verification.json and Carton-Scene/.

- Latest spin attempt (4 October 2026): stopped; full physical rotation NOT VERIFIED. Clearance advance ~29.76cm encoder estimate, then a nominal full encoder turn plus visual-alignment sections (15574/15582 total wheel ticks). No communication or stop faults during these sections; final independent check all16 torque-off, status0. The camera heading still differs from the start, so configured turn geometry/traction is unverified. Extra visual travel allowance paused once and was preserved before bounded continuation. Automatic review then REJECTED more motion pending independent heading/cause verification. Do not resume spin helpers without addressing that rejection. Evidence: outputs/Robot-Spin-Result.json and starting/stopped heading photos. Previous50cm result remains historical; no long loop started.

- Latest50cm drive COMPLETE (2026-10-04): one user-requested outward/return trip, arms/head released throughout; .02m/s in <=1.5s segments, stops verified after each, camera inspected between short batches. Outbound6515/6521 cumulativeticks (~49.97/50.02cm). Final residual2/-5ticks (~0.15/-0.38mm encoder-derived; floor distance not independently measured), all16 independently verified torque-off. Required two diagnostic pauses: (1) zero-target wheels were stationary but one-count velocity quantization prevented two consecutive simultaneouszero samples; now require5stable-position samples over>=.18s with actual zero-speed observation on each wheel, then separate5sample stationary check after release; speed400 and.9s stop deadline unchanged. (2) one left-wheel return command registered only2ticks vs right338; normal voltage/status. Isolated acknowledged left-wheel command moved326ticks and corrected mismatch. Starts now verify torque-enable and each acknowledged/read-back velocity command within.15s before continuing pulse. Original missed-movement root cause is UNCONFIRMED; subsequent paired return segments passed.17 targeted tests pass. No long loop was started. Evidence: task outputs/Robot-50cm-Drive-Result.json and before/turnaround/returned images; work/fifty-cm-drive.json, left-wheel-return-recovery.json. Earlier partial notes are historical.

- Short trip COMPLETE: outbound988/990ticks (~7.6cm), returned with remaining23/6ticks (1.8/0.5mm encoder-derived), all16 torque-off verified. The following partial-return notes are historical. User subsequently requested a new single50cm outward/return test; task work/drive_fifty_cm.py is in progress with lower.02m/s commands, maximum1.5s pulses, stop/release checks after every pulse, common encoder reference and cumulative unwrapped distance tracking. No30-minute loop. State: work/fifty-cm-drive.json.

- Latest short-drive state supersedes the preceding partial note: outbound reached988/990ticks (~7.6cm), then two backward pulses passed with stops. Current velocity-mode readings3181/447 relative origin3700/4032; remaining return519/511ticks (~4cm). All16 independently verifiedoff. Person directly beside base in latest fresh frame, so final return pulses withheld. Continue from work/short-drive-resume.json only after area clears and current position passes the unchanged guard. Outputs/Robot-Current-Drive-Check.json records latest partial outcome.

- Latest short-drive continuation (2026-10-04): requested single ~8cm forward/return is INCOMPLETE. Initial 1s/.03m/s pulse stopped on speed sample left-450/right350 (unchanged400 cap). Two subsequent 1s/.02m/s pulses passed stop/release checks, with all16 independently torque-off after each. Current velocity-mode encoder origin3700/4032, last2949/690, outbound offsets751/754ticks (~5.76cm). No backward movement in this trip yet. Do not restart a whole outbound trip or use old-room coordinates. Latest camera shows a person beside/crouching at the cart, so next pulse held. Diagnostic fixed an encoder-reference mismatch: Operating_Mode0 subtracts saved Homing_Offset85; mode1 does not. Verified twice by torque-off mode changes with zero speed and no physical movement; comparison now uses a common mode1 reference. All fault/camera checks retained, speed reduced.3 new coordinate tests and7 existing stop tests pass. Task files: work/resume_short_drive.py, work/wheel_coordinates.py, work/short-drive-resume.json, work/wheel-mode-offset-verification.json; output Robot-Current-Drive-Check.json.

- Latest result (2026-10-04): ALL16 motors passed today's basic movement checks across the recorded attempts (14 arm/head +2 wheels). Right elbow was recovered autonomously from raw3198 to an in-range holding pose at low speed100, acceleration10 and torque cap400, within a12-degree capture envelope; no manual repositioning was needed. A normal sub-six-degree inward nudge and return then passed with vendor-documented default P=32: start94.833,target88.833,reached90.526,end95.024 normalized. Raw load218, no servo fault or rejected reply. Working elbow P32/Torque_Limit400/Goal_Velocity100 saved in profile and verified on hardware with torque off. Acceleration was restored to its prior setting after the test. Calibration offsets/limits unchanged; all16 motors independently verified torque-off and settings matched saved files afterward. Task evidence: work/right-elbow-recovery-p32-passed.json, work/final-calibration-verification.json, outputs/Robot-Readiness-Check.json.42 targeted helper tests pass. Earlier15/16 notes below are superseded.
- Residual behavior: released right elbow can sag beyond its saved range (raw3175 then3198 observed, saved max3092). Normal adapter startup MUST retain its range check. Task-local work/recover_right_elbow.py implements the specifically tested, single-motor, speed/torque-limited recovery without widening calibration: target stays at least24 ticks inside the range, capture travel<=12degrees, timeout3s, other15 motors verifiedoff, health/camera checks, release and setting restoration. It refuses starts outside that envelope. The current camera threshold is5s by explicit user instruction. The larger30-minute routine remains unstarted; these are basic bounded tests, not full operational/collision validation.

- Latest 2026-10-04 outcome: 15/16 motors passed today's basic tests. All13 arm/head joints except right elbow passed small move/return checks across attempts. Both wheels then passed one1s forward pulse and one1s backward pulse at nominal0.03m/s, with independent stop/release checks; final encoder offsets-2/+3 ticks. All16 motors independently verified torque-off afterward. Right elbow remains unfinished: undertravel (1.914 normalized units for requested5, minimum2.0), then sag beyond saved max prevented retry. It needs supported repositioning before its final small test. No sustained load/servo fault or reply-validation rejection was reported. Evidence: task outputs/Robot-Readiness-Check.json and work/new-setting-wheel-summary.json.
- User explicitly requested removing the strict two-second camera stops. The task-local bounded tests now use a five-second camera age threshold, reject absent/future/older frames, and allow up to ten seconds for a valid frame before beginning a new joint. Wheel pulses remain bounded to1s in today's check; independent zero-speed/stable-encoder/release checks are unchanged.29 targeted software tests passed (7 camera,9 packet-validation,7 wheel-stop). The right wrist/claw then passed on hardware. This does not authorize or launch the previously rejected30-minute loop; none was started. Earlier timestamped notes below describe superseded intermediate states.

- New-location movement checks (2026-10-04): 10 of 14 arm/head joints passed across attempts: both head axes, all six left-arm joints, and both right shoulders. Left elbow passed a 5-normalized-unit inward nudge after under-travelling the smaller 3-unit request. Right elbow moved1.914 units against a5-unit target, slightly below the standard minimum2.0, and returned; load144 at hold, status0, no rejected packets. After release it sagged to raw3175 beyond the saved max3092, so its planned5.5-degree retry was blocked before enable. Right wrist/claw tests were then isolated with other motors off, but camera upload age2.562s stopped before their first movement. No wheel motion has been attempted at this location. Camera resumed, with10 new frames over a5s diagnostic window and maximum age1.395s. All16 motors independently verified torque-off after the latest camera stop. Latest evidence: task work/new-setting-first-movement-attempt.json, new-setting-second-movement-attempt.json, new-setting-movement-test.json, new-setting-final-release.json. Remaining: reposition right elbow with motors released, retest it, finish right wrist/claw, then a bounded wheel check from the new location. Prior-day passed checks are historical, not today's completion.

- New-location preflight (2026-10-04): new phone supplies fresh portrait frames showing the whole base and nearby floor; table edge obscures part of the arms. Read-only check with strict reply validation active: all 16 motors responded, torque0/status0/load0, 27–30 C, raw voltage118–121; no communication errors or rejected packets in 96 register reads. Right elbow is outside saved range (3174 versus 1002..3092); all other arm/head joints within saved ranges. Shoulders and left elbow are near endpoints. No motors enabled or moved in this session yet. Current wheel readings3617/3945 are a new-location pose, not the old interrupted-demo origin; do not return to yesterday's coordinates. Evidence: task `work/new-setting-readonly-preflight.json`.

- Most recent immediate-demo retry is INCOMPLETE: only the first ~2.6 cm forward pulse ran and passed stop/release checks. Setup for the second forward pulse raised `IndexError: list index out of range`, before enabling wheels or moving arms. All 16 motors were subsequently independently verified torque-off, with calibration unchanged. Original wheel readings: left 276/right 3052; after first pulse: left 4030/right 3392 (4096-tick wrap). Preserve this origin: do not start another complete forward sequence and accumulate displacement. Evidence: task `work/dramatic-single-cycle/`. The phone then explicitly stopped sharing, so no further motion was attempted. The 30-minute loop remains unstarted.
- Offline investigation reproduced a Feetech SDK bug: a write acknowledgement can be accepted as a read response; a two-byte read then raises IndexError, and a one-byte read can incorrectly use the checksum as data. This is a plausible cause of the latest error, not confirmed by an original traceback or packet capture. Task-local `work/strict_servo_replies.py` now rejects incorrect reply lengths and preserves servo fault flags; integrated only into the demo bus instances, without changing motor limits or the installed SDK. Detailed traceback/event capture was added. Nine new offline regression tests and 13 existing diagnostic tests passed (22 total). This new receive guard has NOT been verified on hardware. Before resuming: restore phone sharing, inspect a fresh image, perform read-only communication checks, verify no cart repositioning, then use the recorded origin for a bounded recovery/completion. Do not blindly rerun the full demo.

- Latest larger-demo outcome: user accepted triple the small travel as fallback. One bounded cycle passed: roughly 8 cm forward, -65-normalized-unit shoulder sweep (~68 degrees), return to rest, then backward; final encoder errors +4/+1 ticks. All 16 motors independently verified released afterward; calibration unchanged. The 30-minute loop was rejected by automatic approval review for cumulative physical-damage risk given the incomplete route view. No loop or automation was launched. Do not bypass this rejection by splitting the rejected long run into repeated trials. Saved details: task outputs/Robot-Larger-Demo-Result.json and work/dramatic-single-cycle/. The older pending note below predates the accepted shorter-distance fallback.

- New larger-demo request is pending: user wants arms up to halfway along the neck, no lower than the present pose, and one-to-two cart lengths forward/back for 30 minutes. The timed loop has NOT started. Current phone view does not include the complete floor route or tether; user was asked to move the phone farther back. A +20-unit shoulder trial passed; a +40 trial stopped early, followed by eight normal samples with status0/torque0. A subsequent -10-unit direction trial passed. No longer base motion was attempted. Details: task `work/dramatic-demo-request.json`.

- Both arms completed automatic calibration at velocity 200. The head completed manual calibration over comfortable cable-clear working travel. The full calibration report says `LOOKS COMPLETE`, without range problems or notes.
- All 14 arm/head joints have passed small movement-and-return checks across the logged attempts. The right claw initially fell short; it passed with position gain P=32, torque cap 500 (50%), and position speed 200, and passed again after those settings were saved in the profile. Other joints retain their prior position settings. These are basic movement checks, not full task/collision validation or exact-position metrology.
- All 16 motors were independently verified torque-off after the latest demo. Every saved calibration offset/limit and the right claw's saved position settings match hardware. The user explicitly authorized a one-off wheel test; the normal profile remains `wheels: false`.
- A local smartphone camera is connected and supplies fresh 720x1280 images at approximately 4 fps. No microphone or continuous video archive. Latest image/status: parent task `work/phone_camera/latest.jpg` and `latest.json`; always check image freshness before relying on it. Local viewer/QR/restart instructions are in parent task outputs. The phone view supplements motor checks; it is not a real-time safety monitor.
- Current calibration backup: parent task `work/calibration-head-and-arms.json`; verification: `work/final-calibration-verification.json`; user-facing backup: `outputs/robot-calibration-verified.json`. Movement summary: `work/movement-test-summary.json`. Older snapshots are historical.
- Latest repository test suite: 93 passed. Changes are local and uncommitted; no push was performed.
- Two consecutive complete demos passed: short forward drive, both shoulder lifts +8 normalized units and return, then backward drive. Each drive used a 1.0 s command at nominal 0.03 m/s; encoder-derived wheel travel was approximately 2.6 cm each way (ground distance not independently measured). Final wheel offsets from each cycle's start were +4/-5 and +2/-4 ticks. All four drives passed stop checks; both arm cycles returned within the 2-normalized-unit tolerance. Fresh phone frames were inspected between stages.
- Earlier stop-check failures were diagnosed: soft deceleration takes about 0.4 s at the demo speed, and the released encoder can jitter by one tick with a quantized +/-50 speed report. The task-local demo acknowledges/readbacks zero targets, requires zero-speed samples before release, then verifies five stationary samples (encoder span <=2 ticks, speed <=one 50-unit bin) after verified torque-off. This does not change global load limits. Seven meaningful stationary-check tests passed, including rejection of drift, continuous slow rolling, and large speed readings. Calibration and right-claw tuning remain unchanged.
- Demo evidence: parent task `work/forward-arms-back-results.json`, per-stage `work/cycle1-*` and `work/cycle2-*`, and `outputs/Robot-Movement-Demo-Results.json`. Routine components: `work/robot_demo.py` and `work/wheel_stop_check.py`. Normal profile remains `wheels: false`; only explicitly authorized isolated tests drove the wheels. Stops are host software controlled; no device watchdog was configured.
- Full farm tasks, taught poses, long-distance navigation, and the missing light sensor are not validated. Earlier camera/cart contact remains a clearance consideration for larger movements.
- The chronological sections below contain superseded states; use this summary for current state.

# Where the build stands

## Bare-gripper bimanual carton exploration — 2026-10-06

- Began the requested two-hand folding approach, initially scoped to closing
  the existing carton's four top flaps. Added an offline reach screen reusing
  `Box/Stance` and a strict left-six/right-six action-data boundary. The active
  `remote robot control` chat's Molmo paddle experiment was inspected, not edited.
- Verified actual checkpoint normalizers: local Molmo SO101 is 6-D, Molmo YAM
  is 14-D; our two SO101 arms require 12 channels. ABC publishes a specific
  `folding_paper_box` 14-D YAM checkpoint; its public metadata was inspected,
  but neither its 8.063 GB weights nor gated demonstrations were downloaded.
- Fresh `box_closing3` metadata confirms the existing 50-episode, 12-channel
  bimanual SO100 data as an embodiment-near baseline. Joint units and station
  transfer remain unverified; matching dimensions do not authorize execution.
- At the saved 60 mm shoulder setback, bare-gripper far-flap contact misses
  the existing reach allowance by 29 mm. A 20 mm assumed setback passes only
  the spherical reach screen; both arms, bracing, orientations and collisions
  still need a contact simulation and measured station registration.
- 22 focused bimanual/carton tests passed. No new learned box policy, dual-arm
  contact simulation or physical folding result is claimed. No motor commands
  or model training were issued by this exploration.
- [Research, reuse decisions and experiment plan](docs/carton-bimanual-molmoact2.md),
  [source and feasibility evidence](docs/evidence/carton-bimanual-exploration.json).

## Calibrated tags consumed by Gemma and simulated grasp — 2026-10-06

- Added `robot_get_registered_tags` to the installed local Gemma chat (24 tools).
  Automatic registration saves a passing fit for this read-only tool; each read
  rechecks stream, intrinsics, tag mount, robot model, motor configuration/raw
  ranges, table/head stability, and current gripper agreement. A missing or
  changed registration refuses coordinates. Cartesian motion stays disabled.
- Installed the pinned LeRobot/Placo dependencies in the local pilot; the
  detector-only environment previously lacked these registration dependencies.
  Calibration now releases motors before the offline FK/fitting stage.
- Full rendered end-to-end run passed: production adapters, eight fitting and
  three held-out poses, actual LeRobot FK/OpenCV solve, registered tag 3, CAD
  handle offset, then contact-physics lift/hold/release. A moved paddle produced
  a new target and successful grasp; an open-jaw control correctly failed to lift.
- Held-out fit RMS was 0.637 mm, but independent handle error was about 7 mm and
  camera-origin error 8.60 mm. Rendering was 1920×1440; these results do not
  establish the live 640×360 camera's accuracy. Station placement, mass, friction
  and the simulated tag mount remain assumptions, including compliant contacts.
- 646 repository tests passed, 8 skipped; 26 pilot tests passed. Actual Gemma
  inference used the new read-only tool and correctly reported that a physical
  registration is not installed. No physical motor commands were issued.
- Latest read-only readiness is blocked by stale/stopped owner status and an
  unavailable authorized motion interface; only table tag 1 was detected.
  A subsequent fresh image shows the lower edge of gripper tag 2 outside the
  image. This differs from the earlier frame with about 8 px clearance.
  Physical calibration/use remain
  unvalidated; this work did not restart or change the remote motor owner.
- [End-to-end method and limits](docs/gemma-calibration-simulation.md),
  [compact evidence](docs/evidence/gemma-calibration-e2e.json),
  [installation and execution](docs/gemma-automatic-calibration.md).

## Automatic Gemma tag calibration integration — 2026-10-06

- Connected the existing `Experiment.calibrate` to authenticated Gemma motor
  commands and same-frame OAK tag observations. Automatic registration mode
  collects eight fitting and three held-out poses for the existing LeRobot FK
  assembly/OpenCV fitter. Normal completion verifies release; faults stop and
  retain evidence without retrying or resetting the owner.
- Installed and reloaded the idle local Gemma chat. Its live 23-tool catalog
  includes `robot_calibration_status` and `robot_calibrate_tags`. Actual Gemma
  inference used the read-only status tool and correctly reported the right
  elbow's saved-range blocker. No physical movement was requested by this work.
- Fresh frames detected tags 1 and 2; tag 2's black square was fully visible,
  about 8 px from the bottom edge. The previous clipping claim was too strong.
  Detection can vary between frames; missing tag 3 does not prevent hand-eye
  collection when tags 1/2 are observable.
- 628 repository tests passed, 8 skipped; 26 existing pilot tests passed. The
  30 new integration tests cover measured completion, held-out fitting, STOP,
  session changes, wrong-arm binding, stale/changed camera data and no-motion
  acknowledgements. These use a simulated API, not physical/collision testing.
- Physical registration remains unvalidated. The right elbow is outside its
  saved range and the right-arm candidate geometry configuration is absent.
  Other motion clients must remain idle for the supervised calibration run.
- [Setup and execution guide](docs/gemma-automatic-calibration.md),
  [verification evidence](docs/evidence/gemma-automatic-calibration.json).

## Right-arm AprilTag binding correction — 2026-10-06

- The user confirmed tag 2 on the right fixed gripper housing. The earlier
  left-encoder captures and their candidate FK are invalid for registration.
  Camera-only metric observations remain evidence of detection, not robot
  registration. There are zero accepted independent right-arm poses.
- Capture now requires an explicit arm and confirmed matching fixed mount.
  Dataset assembly and fitting preserve and check this binding; old unbound
  samples and mixed arm/tag identities are rejected.
- Targeted geometry, capture, registration and Gemma adapter tests: 88 passed.
  [Correction and fresh read-only evidence](docs/evidence/gemma-right-arm-binding.json).
- A corrected read-only capture could not detect tag 2 at the lower image edge.
  Right elbow readback was 3155 against recorded 1002..3092, and right-arm
  geometry configuration was absent. The owner repair chat is auditing this
  range discrepancy; no motor commands were issued by the binding correction.
- The existing carton controller already implements automatic joint probing
  and independent visual-model checks. Its command-file transport and seeded
  camera observer needed adapters for Gemma's current authenticated owner
  and OAK tags at this checkpoint. The integration above now supplies them;
  physical pose collection remains unvalidated.
  See the [automatic workflow](docs/gemma-tag-geometry.md#automatic-move-and-observe-workflow).

## Metric AprilTags and registration utilities — 2026-10-06

- The live Gemma chat now reads camera-relative tag centres in millimetres,
  through its existing OAK camera and authenticated transport. Actual Gemma
  inference read IDs 1/2/3 and reported approximately 378 mm between the gripper
  and paddle tag centres, correctly stating that these data cannot yet command
  a grasp. A prior stale-frame probe was rejected. No motor commands were issued.
- Added stationary camera/encoder sampling, candidate FK through existing
  LeRobot/Placo and fixed-camera OpenCV hand-eye fitting with independent
  validation poses. The initially captured pose and candidate FK were later
  invalidated by the right-arm mounting confirmation above. No robot-frame transform
  was invented or activated. Printed sizes remain user-confirmed 60/40/40 mm,
  without a separate ruler measurement.
- Metric rendered-camera tests retained both failures: worst error 6.72 mm at
  640×480 and 5.05 mm at 1280×960. The 1920×1440 run passed the unchanged 5 mm
  target at 4.73 mm maximum across 138 tag positions. This does not validate the
  live 640×360 camera's physical accuracy. All three runs detected all three
  tags in 46/46 nominal frames and passed the existing negative controls.
- Repository tests: 583 passed, 8 skipped. Existing pilot tests: 26 passed;
  browser-session recovery checks passed. The local chat was restarted idle
  with saved history and inactive prior goals; its 21-tool catalog includes
  metric observations. This local adapter installation restarted neither the
  camera process nor the remote owner.
- Physical pickup remains unvalidated: the initial remote owner latched after an idle
  `COMM_RX_CORRUPT` read. Source and saved-row timestamps point to right gripper
  ID 6 but do not prove USB contention or a particular packet failure. Added
  bounded transaction/reply diagnostics to the existing telemetry helper, with
  unchanged failure handling and no retries. These hunks were then staged in
  the actual remote helpers; 23 remote hardware-independent tests passed.
- A subsequent direct user request in the Gemma chat initiated controller
  recovery. That chat replaced the released owner to load diagnostics. The
  follow-up readback showed idle, motion-ready and all 16 motors released.
  Its communication stability check is ongoing; this is not a verified servo
  fault fix or a completed physical grasp. The initial stopped-state evidence
  and later recovery snapshot are kept separately in the evidence file.
- [Guide and next steps](docs/gemma-tag-geometry.md),
  [measured evidence](docs/evidence/gemma-tag-geometry.json).

## Gemma AprilTag simulation — 2026-10-06

- Passed the production `TagRobot`/`TagObserver` on MuJoCo-rendered RGB using
  the existing SO-101 meshes/joints and paddle CAD. All three tags were detected
  in 46/46 nominal frames across 24 degrees of measured shoulder-pan travel.
  Maximum tag-centre error was 0.274 px; maximum gripper-to-paddle displacement
  error was 0.455 px against independently projected simulator positions.
- Occlusion, duplicate IDs, undersized tags, stale frames and hash mismatches
  were rejected; tags were reacquired after faults cleared. Simulated STOP
  prevented a subsequent movement request.
- Local Gemma called observe, simulated +12-degree pan, observe and STOP,
  correctly reporting the changed pixel displacement. No hardware tools were
  exposed, and no physical camera or robot was contacted.
- The camera pose and tag mounts are illustrative, and rendered lighting is
  idealized. This validates tracking/data flow and model tool use, not a
  calibrated physical grasp or autonomous carton folding. Reproducer and
  limitations: [simulation guide](docs/gemma-apriltag-simulation.md).
  Results: [simulation evidence](docs/evidence/gemma-apriltag-simulation.json).

## Gemma AprilTag observations — 2026-10-06

- The Gemma chat exposes `robot_get_tags` through the shared detector and its
  existing authenticated `robot_get_cameras` transport. Detection runs on the
  Gemma Mac; no additional camera stream or motor owner was opened.
- After the user repositioned the tags, OAK detected IDs 1 (table), 2 (gripper)
  and 3 (paddle) together in three advancing frames. The tag-3 centre was about
  78.6 px right and 89.9 px above tag 2. The earlier same-view visibility gap is
  resolved for this stationary setup; tag-based 2D displacement is available.
  The phone view contained no valid tags in this check and remains receipt-only.
- Real Gemma inference invoked the tool and accurately reported missing views
  and tags. The live chat advertised 19 tools after installation, retaining
  existing direct-control tools and STOP. This check issued no motor writes.
- The integration provides 2D measurements and annotated images. No metric
  pose, camera-to-arm calibration, grasp or physical motion was validated.
  See [setup](docs/gemma-apriltags.md) and
  [initial evidence](docs/evidence/gemma-apriltags.json) and
  [same-view follow-up](docs/evidence/gemma-apriltags-covisible.json).

## Servo temperature sensing removed — 2026-10-05

- At the owner's direction the software no longer reads or acts on servo temperature anywhere: the `farm` safety
  rules, `farm robot-test`, `farm mcp`, the simulator, and the carton motor owner and its clients.
  `farm soak` and the profile key `servo_temp_max_c` are gone.
- Load, step, travel, watchdog, lease and STOP checks are unchanged. A nonzero servo `Status` byte still stops
  the carton owner, and that byte is where the servo's own firmware would report overheating.
- 2026-10-06: the servo-side protection is next. `farm servo-protection -p paper-tray-v0` reads every servo's
  EEPROM temperature limit (default 70 °C) and unload/alarm masks; `--write` sets the limit to 200 °C (one byte;
  Feetech documents 0..100; if a servo will not keep 200 the tool writes 100 instead and says so) and clears
  the temperature bit in both masks, so the firmware neither unloads nor flags on heat. Not yet run on the robot: this Mac has no motor boards. Run it from the robot Mac following
  `ROBOT_HANDOFF_2026-10-06-servo-protection.md` and record the before/after table here.
- The 2026-10-05 session-archive copies of the motor owner and claw script, which enforced 55 °C, were deleted
  on 2026-10-06. A robot-Mac working copy of the carton owner outside this repository (`work/`) keeps its old
  checks until the packaged scripts in `software/scripts/carton_robot/` replace it.

## OAK-D Lite test — 2026-10-05

- User has an OAK-D Lite connected to the development Mac, separate from the
  robot-control Mac. Added a camera-only DepthAI utility and
  [setup guide](docs/oak-d-lite.md). It does not connect motors or integrate
  depth observations into the farm controller.
- DepthAI 3.10.0 failed at boot in both USB modes, including after the user
  unplugged/reconnected directly. Switched the isolated Python 3.12 environment
  to DepthAI 2.33.0.0 (OpenCV 5.0.0.93, NumPy 2.5.3); v2 boots and streams.
  Four depth evidence/storage tests pass. A v2 device-info-only probe reported
  a crash during shutdown; the final 30-second capture exited cleanly.
- Live capture: 448 synchronized RGB/depth pairs in 30.032 s over USB 2,
  640×360 output, maximum timestamp skew 1.265 ms. Last depth frame had
  30.89% valid pixels; centre patch had no valid depth. Camera was facing the
  room/ceiling, not the planter. RGB image inspected; raw 16-bit depth saved.
- Local evidence on development Mac:
  `/Users/wk/conductor/workspaces/research/minsk/.context/oak-captures/20261005T005902.949416Z/`.
  Camera imagery stays local and is not committed. USB 3 remains unverified.
- Next: aim at stationary trough/carrier, compare measurements with a ruler,
  check fin/rim depth coverage, then calibrate camera-to-robot coordinates.
  No claim of planter accuracy, robot integration, or successful assembly.

## Earlier robot handoff

Written 2026-10-02 for whoever picks this up on the robot's laptop (person or agent). Read this first, then `SETUP.md` (installing) and `README.md` (what the program is). `docs/community-projects.md` reviews the 43 projects on the XLeRobot community page against this plan.

## The robot

- **Kit:** WowRobo XLeRobot 0.4.0 two-wheel Combo: two SO-101 follower arms (12 V STS3215 servos), head with two servos, two drive wheels, three bare USB camera boards, two motor control boards, IKEA cart.
- **Assembly:** built to the end of the official 0.4.0 video (https://www.youtube.com/watch?v=4bXCFw57T60). The older 0.3.0 video and docs are for the three-wheel version and do not match this kit.
- **Orientation:** as in the video. The top base sits against the drive-wheel side of the top tray; arms and head camera face outward over that edge; motor boards and cables are at the back (caster side). The official 0.3.0 render shows the opposite (arms facing inward); the 0.4.0 arm bases can be re-clocked either way. Outward is what the farm needs: trays sit on a separate surface in front of the arms.
- **Base:** parked. The profile has `wheels: false`; nothing in the farm program drives the wheels.

## What is done and verified

| Item | State |
|---|---|
| Loose servo IDs | Set and read back on the real servos: head 7 and 8, wheels 9 and 10 (`farm set-motor-id`) |
| Arm servos | One arm probed: IDs 1-6 all answer (STS3215). The other arm was not probed on its own. |
| Motor power | A USB-C-to-12 V cable gave about 12.5 V on the bus |
| Software | Fresh clone installs and passes 109 tests; simulator runs end to end |
| LLM backends | Claude CLI and OpenRouter (Jev, Astra) both answered from the first laptop |

## What is not done

| Item | State |
|---|---|
| Full-robot probe | Verified on robot laptop: all 16 STS3215 servos answer; bus 1 IDs 1–8, bus 2 IDs 1–6, 9, 10. Read-only health/position reads succeeded for every servo; 29–31°C, raw voltage 118–121, torque off, load/current zero. Powered movement not tested. |
| Profile | Motor ports saved: port1 `/dev/cu.usbmodem5B790186401`, port2 `/dev/cu.usbmodem5B790182091`. Camera indices still unset. |
| Calibration | Not done. Nothing can move until it is |
| Taught poses | None (`data/keyframes.yaml` does not exist) |
| Cameras | macOS now detects all three external `USB2.0_CAM1` cameras plus the built-in MacBook camera after the owner powered the head camera. Newly appeared device: `0x11130005a39230`, likely head based on that action; live frames and identities still unverified. Run camera probe from Terminal. |
| Light sensor | ESP32 + BH1750 not wired or flashed |
| Printed parts | Cress planter, nests, tag tiles, bottle rest, paddle: print status unknown |
| Head naming | Pan = 7, tilt = 8 is assumed, not checked on the hardware. RoboCrew's XLeRobot driver uses the same mapping (yaw 7, pitch 8). `farm robot-test --move --ask --only head` checks it |
| Wheel sides | Left = 9, right = 10 is assumed, not checked (unused while parked) |

## Hardware facts worth keeping

- **Motor boards (serial → macOS port):** `5B790182091` → `/dev/cu.usbmodem5B790182091`; `5B790186401` → `/dev/cu.usbmodem5B790186401`. Which one ended up on the left arm is decided by the probe, not by the name.
- **Left vs right:** the arms are identical. The left arm is the one whose board also carries the head servos (7, 8); the right arm's board carries the wheel servos (9, 10). Left and right are from the robot's own point of view.
- **3-pin wires are servo wires and carry 12 V.** They go board → servo → servo. Never plug one into a camera.
- **Cameras:** each has a 4-pin USB port and its own cable to the hub. They do not connect to the arms.
- **Wiring to the laptop:** both motor boards (USB-C) and all three cameras go into one USB hub; the hub's main cable goes to the laptop. Each motor board gets its own 12 V feed (USB-C-to-12 V cable from the battery). Switch 12 V off before moving any servo wire.

## Next steps, in order

Run from Terminal (camera permission is per app), inside `software/` with the environment active.

1. `farm devices --probe`: confirm both boards and their IDs, and look at the camera snapshots in `data/devices/`.
2. Edit `profiles/paper-tray-v0.yaml`: `port1` = the board with IDs 1-8, `port2` = the board with 9 and 10. For each camera add `index_or_path: <n>` using the snapshot that shows the matching view.
3. Calibrate, one of two ways, then check the result:
   - By hand (known to work): `farm calibrate`. Support both arms and follow the prompts.
   - Automatic (decided 2026-10-03 to try it; **never run on this robot**): `farm calibrate --auto --arm left`, then `--arm right`, then `farm calibrate --head`. Go up in stages first (`--motor gripper`, `--motor wrist_roll`, `--unfold-only`); the `farm-bringup` skill, Step 4B, has the procedure and what to do when a stage goes wrong. Write down what happened here.

   Then `farm calibration-report`: reads the saved file and flags a wrapped reading, a short sweep, or arms that disagree, before anything moves.
4. `farm robot-test`, then `farm robot-test --move --ask`: motors only (no cameras, no models, no trays). The first reads every joint and load. The second nudges one joint at a time and asks whether the named part moved; this catches swapped left/right boards and swapped head motors. Start with the arms folded; the motors go limp when it ends.
5. `farm check`: connects everything and has the vision model confirm which camera is which.
6. Put the bottle, paddle and trays in their fixed places, then `farm teach-all`.
7. `farm once --tray B` with an empty bottle; authorize from the viewer (http://localhost:8765).
8. `farm cup-test --tilt 25 --seconds 1.5 --who <name>`.
9. `farm run --every 3600 --record`.

## Testing the policy we already trained

Decided 2026-10-03: test the existing checkpoint on the robot before doing anything with a second computer. The two-computer policy server is parked until then.

The checkpoint is ACT, trained for 5,000 steps on someone else's SO-101 pouring data (`SurajCreation/so101_pour_v1`, one arm, a wrist camera and an overhead camera). Offline it predicts worse than holding still, so expect pour-like motion of the right arm, not a working pour. What the test proves is the path: checkpoint → camera frames → clamped joint steps on the real robot.

1. Copy `act_so101_pour_checkpoint.zip` (191 MB, in the first laptop's Downloads folder) to this laptop, for example by AirDrop, and unpack it inside `software/data-train/` so that `software/data-train/act_so101_pour/checkpoints/005000/pretrained_model/config.json` exists.
2. On the simulator first: `farm policy-test --checkpoint data-train/act_so101_pour --steps 40`.
3. On the robot, after calibration and `farm robot-test` pass, with the right wrist and head cameras working, nothing in the right arm's reach, and the viewer's STOP button open on a phone: `farm policy-test --checkpoint data-train/act_so101_pour --real --steps 60`. Every step is clamped to the profile's limit; it ends after 60 steps with `max_steps reached`, which is the normal ending.
4. Record here what the arm did.

The policy's `wrist` view is mapped to the right wrist camera and its `overhead` view to the head camera. Our head camera does not look from where the training camera did, which is one more reason not to expect a real pour.

Stop at any point with the red STOP button in the viewer.

## Built after the community review, not yet run on the robot

`farm calibration-report`, `farm robot-test`, `farm calibrate --auto` and `--head`, `farm mcp`, and `farm policy-server` with `farm policy-test --server`. All pass on the simulator, except that the limit-seeking motion inside `farm calibrate --auto` (LeRobot PR #3282, vendored) cannot be simulated and is untested here. The README has a table of them; `docs/community-projects.md` is the review they came from.

Decided 2026-10-03: teaching by hand was removed (no human operation, no exceptions); the two-computer policy server (`docs/gpu-server.md`) is parked until the existing checkpoint has been tried on the robot.

## R2a planter assembly (decided 2026-10-03; contract only, nothing physical)

The design chat handed over a modified cress planter ("R2a": the original holder with four insets fused in and
two grip fins, an optional paper frame, a grip coupon) for the robot to assemble. The full handoff is
`docs/r2a-handoff.md`; the reconciliation, station plan and what was built are in `docs/r2a-assembly.md`.

- Code: `farm/assembly/` (frames and units, stage machine with required evidence, fail-closed dataset schema,
  episode metadata and held-out split), `profiles/r2a-assembly-v0.yaml` with `execution_enabled: false`, 28 tests.
  `farm r2a` prints what blocks execution. The watering profile and skills are untouched.
- Parts: `parts/r2a/` holds the release STLs and records; hashes checked by the tests.
- Physical state when written: the full plate was printing on the M5C after one failed start (nozzle traced
  past the sheet edge; cause unresolved). Completion, part quality, fit, grips, fixture, station transform:
  **all unknown**. Nothing may run until `farm r2a` shows no blockers, and then only supervised and dry.
- Open hardware questions carried over: whether cameras were in the kit; the trough fixture (`nest_cress.stl`
  assumes a plain outline; the real trough has an offset refill bay).

### R2a continuation, October 3 (development Mac)

- User reports calibration currently running on the **other Mac connected to the robot**. It was not
  restarted or touched from this session. Completion, calibration report, camera identities and motor-test
  results have not yet been read back here. Local missing-device reports describe this development Mac only.
- Print identity is known: `cress_R2a_full_prototype_plate.gcode`, carrier + frame + coupon; original trough
  reused. The 15:19 JST observation is historical, not a new printer check. Print completion/inspection unknown.
- Added offline discrete supervisor rehearsals for both variants and fault paths (`farm r2a --simulate`),
  measured-point station fitting (`farm r2a-station`), and file-only laptop readiness/keyframe checks.
  These are not physics/vision tests or hardware assembly. Synthetic traces cannot qualify as training data.
- Fixed acceptance of non-affirmative OK readings, stale evidence/jaw readings, invalid grip thresholds,
  fractional paper counts, malformed enable flags, non-finite deadlines and extra placement attempts.
- [Connected-Mac continuation](docs/r2a-connected-mac.md) gives the measured inputs, isolated pose names,
  commands and remaining live executor/observer/recording integration. Execution is still disabled.

## Second task: carton closing (2026-10-03)

Separate scope, same robot: `software/carton/`, command `carton`, profile `carton-v0`. Measurements, reach analysis, plan and status in `docs/carton.md`. Built and tested on the simulator only. Uses the printed flap paddle and an ordered tape dispenser; actual dispenser setup/pickup remains unverified.

### Carton handoff, October 4 (development Mac)

- Start at [docs/carton-connected-mac.md](docs/carton-connected-mac.md) on the robot Mac. It covers
  preserving local calibration, the release download/checksums, local profile, station and remaining integration.
- ACT completed 8,000 steps on October 3 at 18:23 JST, final loss 0.293. Two duplicate same-seed
  processes wrote the directory; both ended. Final inference files were subsequently verified on MPS.
- [Release](https://github.com/RonTuretzky/xlerobot-farm/releases/tag/carton-act-8000-2026-10-04)
  contains the inference checkpoint; `git pull` alone does not download weights. Optimizer state excluded.
- Real recorded-frame check: 27 frames, episodes 47–49, **not held out**. MAE 3.888 versus a
  hold-current-state baseline of 1.831. Synthetic inference outputs `(12,)` and `(100, 12)`.
  [Evidence](docs/evidence/carton-act-8000-check.json). Neither is a physical success rate.
- 167 tests passed before this handoff. Print source meshes are tracked under `parts/carton/`.
- Real policy profile now names missing `policy_front`/`policy_top` views explicitly instead of
  feeding the head image twice. Learned control remains disabled and is not wired into carton cycles.
- Physical setup, print fit, teaching sequence/object state, observation-only ACT integration and
  hardware validation remain for the connected Mac. Teaching CLI changes are described below.
- Existing hardware calibration may have progressed independently on that Mac. Read it back; do not
  overwrite it or restart calibration based on this development-Mac status.

### Direct dispenser pickup follow-up, October 4 (development Mac)

- [Tape test instructions](docs/carton-tape.md): `carton tape-test --plan` is hardware-free;
  `carton tape-test -p profiles/carton-local.yaml --teach` physically teaches and executes pickup,
  lift-clear, adhesive-down placement, release and retract on a closed carton. **No flip/turnover.**
- Owner identified the ordered dispenser as [LUKDOF M1000, ASIN B0DQY77P16](https://www.amazon.co.jp/dp/B0DQY77P16).
  Listing checked October 4: front outlet, 20–999 mm cuts, 7–50 mm tape width, manual/automatic modes.
  Initial cut target is 80 mm; actual tape width, adhesive orientation, outlet clearance and release
  still need physical checks. Printed tape rest/folded tab are not required by the controller.
  First trial uses manual mode with automatic refill disabled and the mechanism stopped; verify removal
  does not trigger another feed/cut. No machine control/interlock or autonomous replenishment is implemented.
- Added staged head/wrist checks, locked gripper during transport, strict finite left-only poses,
  calibration/profile-bound teaching bundles and no retry/release on uncertain tape state.
- The full carton cycle uses the same tape sequence. Existing tape keyframes require re-teaching.
- Tape trial/individual teaching verifies the STOP viewer belongs to its session; ordinary batch
  teaching stops on the first failure. Live tape trials require attended Terminal shutdown.
- Validation: 199 tests passed, including 32 tape-controller tests; `carton tape-test --plan`
  lists six direct-pickup poses and opens no hardware. No training job or physical motion was run.
- These are controller/simulator checks only. Physical tape grip, visual face recognition, clearance,
  adhesion and release remain unverified. Put a visible mark on the strip's nonsticky backing for teaching.

## Rules that were decided

- Nobody drives the robot by hand. Poses are taught by the vision model (`farm teach`, `farm teach-all`); people only answer questions in the viewer or over Telegram.
- Right arm holds the bottle, left arm holds the light paddle.
- Trays, bottle rest and paddle rest stay in fixed positions; everything taught is relative to that layout.
- Water only moves when the rules pass and either a named person or Jev at `approve` level authorizes it. A pour with an unknown result blocks further cycles until someone reconciles it.

## Not on this laptop

These stayed on the first laptop: the trained ACT test checkpoint (copy it over as described under "Testing the policy we already trained"), the 3D-print files and print handoffs, and the site build output. The OpenRouter key must be typed into `.env` again; make a fresh one, since the old one was pasted into a chat.

## Robot laptop software verification

- Python 3.12 environment installed; 42 tests passed and simulator completed.
- Arduino CLI and ESP32 core 3.3.12 installed; Rosetta installed; light firmware compiled successfully. No ESP32 serial port detected.
- OpenRouter key authentication passed earlier on this laptop.
- Claude login needs attention: prior real request failed with expired OAuth session; latest auth check reports logged out.
- Calibration remains undone (confirmed by owner); no powered movement performed during the full-robot diagnostic.

### Camera probe result

Terminal captured valid 640×480 images from indices 0, 1, 2. Index 0 appears to be the head; indices 1 and 2 show wrist/gripper views, with left/right assignment pending a lens-cover check. Index 1 appears blurry. Index 3 was detected but rejected 640×480 (reported 1920 width); likely built-in camera, not yet conclusively mapped. Out-of-bounds warnings for 4–59 are enumeration noise. No powered movement performed.

Camera identity confirmed by owner covering the RIGHT wrist lens: index 2 went black, index 1 remained a wrist view. Saved mapping: head 0, left wrist 1, right wrist 2. Left wrist remains visibly blurry; focus needs checking before vision-guided motion.

### Calibration recovered on robot laptop

Both arms and head were manually ranged. Final right-bus save failed with a missing acknowledgement on ID 1; readback confirmed the minimum limit had been written. Subsequent read-only check found all 16 servos responding, voltage raw 119–122, torque off. Recovered the captured ranges using verified stored homing offsets, wrote only missing arm limits with retries, and verified all limits by readback. Saved and reloaded `~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json`. Existing wheel offsets/limits preserved without wheel writes. No powered movement test performed yet.

### Powered-test preflight

Corrected parked adapter to prime/read back hold targets before enabling arm/head torque, omit wheel commands, verify calibration instead of rewriting it, and clamp joint targets without the vendored action-key mismatch. Self-test now stops on a reported failure or wrong-part confirmation, and refuses nudges after health problems. New adapter and tool tests pass (10). Hardware preflight stopped before torque enable: left shoulder lift was 3250, above recorded maximum 3240. All 14 arm/head torque registers confirmed zero afterward. User must reposition shoulder slightly inside measured travel before retry. Powered movement remains untested. Changes are local, not pushed.

### First powered head test passed

After owner repositioned both elbows, all arm/head joints were inside calibrated ranges. Ran head-only robot-test at delta 3: pan moved 2.6 normalized units and returned; tilt moved 1.3 and returned; tool reported ALL OK. Initial loads zero. All 14 arm/head torque registers explicitly verified off afterward. Wheels untouched. Arm movement and visual confirmation of head axis identities remain untested.

### Automatic calibration preflight, 2026-10-03

Pulled upstream bb30bb3, preserving the local parked-adapter fixes. No automatic calibration motion has run. Read-only preflight found all 16 servos responding, torque off, 27–30 C, raw voltage 118–121. Both elbows are slightly beyond yesterday's recorded maxima, so do not run the normal powered adapter test without resolving that.

Found and locally corrected automatic-tool startup defects: single-motor mode previously configured/enabled all six arm motors; initialization enabled torque without refreshing Goal_Position after offset changes and could continue after verification failures; the range-measurement OperatingMode import was invalid. Single-motor mode now constructs a one-motor bus; initialization verifies torque-off, configuration and fresh raw holding targets before any enable, and fails closed. Range seeking now sends stop commands on every exit and rejects timeout/communication errors as calibration results. Fifteen focused tests passed. These software checks do not validate mechanical-stop seeking on this cart.

Installed the new mcp dependency. Saved the prior calibration under the task's work/before-auto-calibration.json. Physical clearance confirmation is pending. The unreviewed remainder of the upstream motion workflow (including midpoint transitions/unfolding) still requires review before powering any calibration stage. No hardware offsets or limits were changed during this preflight.

### First automatic gripper stage completed

Owner confirmed physical setup and requested start. Ran `farm calibrate --auto --arm left --motor gripper` using the isolated single-motor bus. Exit 0: raw endpoints CW 2965 / CCW 1390; measured sweep 1575 counts (138.4 degrees). Tool backed off and returned toward center. New left gripper EEPROM: Homing_Offset 130, Min_Position_Limit 1260, Max_Position_Limit 2834, Operating_Mode 0. After exit, read-only verification confirmed Torque_Enable=0 on all 16 motors. Left gripper position 2027. No wheel commands sent.

IMPORTANT: single-motor mode intentionally does NOT update the saved calibration file. Left gripper hardware now differs from yesterday's saved file; the parked adapter will reject that mismatch. Do not run robot-test or normal motion until calibration is reconciled. Original calibration backup remains `work/before-auto-calibration.json` in the parent task; post-stage readings are `work/after-auto-gripper.json`. Waiting for the owner's observation of jaw movement/contact before proceeding to wrist roll. Full automatic arm calibration has not run.

### Automatic left-arm run stopped on fault; calibration restored

Owner confirmed the gripper opened/closed fully, full physical clearance, and requested continuous staged calibration. Left wrist-roll stage exited 0: endpoints 117/1424, chosen travel 2789 counts (245.1 degrees), offset 771 and limits 653..3441. Unfold-only stage exited 0: wrist_flex and elbow_flex reverse, shoulder_lift forward.

Full left-arm stage completed shoulder_lift (2394 counts, 210.4 degrees) and elbow_flex (2224 counts, 195.5 degrees), then failed during simultaneous wrist_roll/gripper/wrist_flex limit seeking: `Could not confirm stop command for ['wrist_roll', 'gripper']`. Exited 1 after cleanup; saved calibration file unchanged. Read-only checks then found ALL 16 Torque_Enable registers zero, Status 0, voltage raw 119–121. Full calibration did not complete; right arm was not run.

Restored and read-back verified yesterday's backed-up Homing_Offset and Min/Max_Position_Limit for all six LEFT arm motors, plus Operating_Mode=0, with torque verified off throughout. No Goal_Position commands were sent during recovery. Head/right arm/wheels not written. This supersedes the earlier partial gripper-mismatch warning: offsets and limits now match the saved file again. Saved diagnostics: parent task work/after-auto-left-failure.json. Do not retry the full automatic sweep until the wrist/gripper communication fault is understood. Head remains manually calibrated from yesterday, not automatically recalibrated.

### Requested retry reproduced lost feedback

Preserved chained exception diagnostics; 15 focused tests passed. Retried full LEFT automatic calibration once at user request. It again failed during simultaneous wrist_roll/gripper/wrist_flex seeking. Wrist-roll Present_Velocity and Present_Position reads returned no status packet; all five position retries failed. Stop writes to IDs 5, 6 and 4 also received no status packet after four attempts each. Fault occurred before Ctrl-C was delivered at the exit prompt. Shoulder/elbow measured travel also changed materially between full attempts (210.4/195.5 degrees first, 160.1/147.8 second), so these partial measurements must not be accepted as repeatable calibration.

Afterward all 16 motors responded and Torque_Enable=0 was verified; raw voltage 118–121. Restored and readback verified previous saved left-arm offsets, limits and position mode again without motion. No new calibration file saved, no right-arm sweep performed. Repeated failure is correlated with simultaneous distal-joint movement; cause (power, wiring, servo/driver behavior) not established. Do not bypass lost-feedback stop or repeatedly rerun unchanged sweeps.

### Communication diagnosis, stationary test

Without torque or configuration writes, read six feedback registers for every left-bus motor (six arm plus two head), 30 cycles: 1440 reads, zero failures. All firmware 3.10, baud register 0, return delay 0, response level 1. Idle voltage raw 118–120, all Status registers 0. This excludes an always-present communication failure but does not measure voltage during the missing-packet interval.

Leading hypotheses: power/current delivery or voltage drop under simultaneous motor load; a distal servo-chain cable/connector affected by arm position; motor-noise/bus or driver timing remains possible. No cause proven. The two full runs lost responses at the simultaneous distal-joint stage while earlier individual tests passed. Upstream PR discussion https://github.com/huggingface/lerobot/pull/3282 reports a different XLeRobot left arm improved with velocity-limit=100, and the author notes insufficient supply current can cause stalls. This supports a lower-speed/sequential diagnostic after checking power delivery, not bypassing missing-feedback stops. No further movement performed during diagnosis.

### Reduced-velocity and isolated-wrist trials

User requested reduced velocity first, then one motor at a time. First velocity=100 start failed before any torque enable with missing Lock read response from ID5. Restored setup settling delays (50 ms torque-off, 10 ms configuration, 50 ms limits) and added bounded configuration read retries; fail-closed verification retained. Focused tests passed. This pre-movement failure weakens a purely load-only diagnosis: root cause remains unproven.

Next full-left velocity=100 run reached the shoulder/elbow measurements but produced invalid travel: shoulder 357.8 degrees, elbow 0 degrees. Interrupted and disabled torque; file not saved. Restored previous left-arm offsets/limits/mode and verified torque off. Added per-joint range plausibility validation before applying measured calibration or derived targets; 17 focused tests passed.

Then ran isolated left wrist_flex at velocity=100. No communication errors; stage exit 0, endpoints 2363/1108, chosen span 1255 counts (110.3 degrees), offset -312, limits 1420..2674. Candidate report flags mismatch versus right wrist_flex's manually recorded 199 degrees (left previous manual also 199). Therefore did NOT accept/save this as complete calibration. Restored original left wrist_flex offset -840 and limits 930..3197 with readback, then verified ALL 16 motors torque off.

Current conclusion: one-motor testing at low velocity communicated successfully once, but complete/repeatable travel was not established. Full auto-calibration remains incomplete. No right-arm auto-calibration attempted; prior calibration file unchanged and arm settings restored. Do not confuse successful stage exit with a valid calibration.

### Requested both-claw movement

Preflight left gripper position3533, saved limits2029..3476, so left torque was not enabled. Right gripper position2070, limits2046..3518, load0, torque0. Attempted isolated right-gripper small open-and-return (15 normalized points, increments capped3); other joints excluded from adapter groups. The test aborted before completing. Right torque readback0 after cleanup. Subsequent five torque-off reads returned load0, voltage121, status0, position2074 (start2070). No complete open/close cycle verified. Left claw remains beyond saved range.

### Interactive left-wrist observation

User requested another auto-calibration with physical-part callouts. Ran isolated left wrist_flex at velocity100. First endpoint2365 was detected; reverse sweep timed out after20 seconds. Exited1, file unchanged. After exit: torque0, raw position296, velocity0, load0, Status0. Restored original wrist offset/limits/position mode and verified torque0. User observed it looked fine while moving and said it was visibly at its stopping point. This establishes a possible end-stop detection/timing discrepancy, not confirmed inability to reach the stop. We did not log every in-motion sample, so cannot determine which stability/velocity/Moving condition prevented detection or whether it reached the stop near the timeout.

### Traced wrist retry isolated speed/timeout issue; velocity200 succeeded

At user request, instrumented read/write calls without changing stall detection. Isolated wrist_flex at velocity100 timed out during first sweep. Trace showed uninterrupted position progression2326→2348 in final0.29s, speed100, Moving1, Status0, and no communication exceptions: it was STILL MOVING when the20s deadline expired. Earlier user-observed stop may have occurred close to the deadline; this trace does not prove the same timing on the earlier unlogged attempt.

Retried isolated wrist_flex at velocity200 (one-fifth original1000), unchanged20s timeout and all fault guards. Exit0, no communication errors. CW2371 / CCW75, travel2296counts=201.8deg (manual prior199.2deg). Candidate offset-824, limits899..3195; readback verified, torque0, temp36C, Status0, position2039. Candidate snapshot saved in parent task work/successful-left-wrist-200.json and trace in work/wrist-calibration-trace-200.jsonl. Restored original wrist calibration again to keep saved file/hardware consistent; full calibration file unchanged. This successfully validates a plausible wrist-only sweep at200; does not resolve simultaneous-joint communication failure or validate remaining arm joints. Awaiting user's visual observation before further interactive diagnosis.

### Both automatic arm runs completed at velocity200; final validation still blocked

Owner confirmed isolated left wrist moved freely, then requested all remaining calibration. Full LEFT run at velocity200 completed exit0 and saved. Ranges degrees: shoulder_pan159.3, shoulder_lift210.9, elbow162.4, wrist_flex200.4, wrist_roll339.3, gripper136.1. First report flagged only old head pan. Left calibration was explicitly readback verified, torque0.

RIGHT gripper and wrist-roll staged runs at velocity200 exited0; unfolding stage exited0. Full RIGHT run at velocity200 exited0 and saved. Right ranges degrees: shoulder_pan206.0, shoulder_lift211.3, elbow183.7, wrist_flex201.1, wrist_roll335.6, gripper132.7. No lost-feedback errors reported during these full successful runs.

FINAL REPORT IS NOT CLEAR: left vs right shoulder_pan travel159 vs206 degrees exceeds30-degree mismatch threshold; old head pan0..4073 touches encoder edge. Other reported arm ranges have no flags. Do not call the whole robot fully validated, and do not start general powered tests/farm motion yet. Need inspect left bottom sideways shoulder swivel for its shorter travel, and redo/verify head pan manually with cable clearance. The auto tool does not support head calibration.

Readback verification of both full-arm results and all16 torque registers recorded in parent task work/after-both-auto-arms.json; current saved-calibration snapshot work/calibration-after-both-auto-arms.json. IMPORTANT: earlier work/before-auto-calibration.json is now an OLD manual backup. Do not blindly restore it over these completed results. Wheels untouched; head unchanged.

### Confirmed camera/cart contact during left retry — no more powered sweeps until clearance fixed

User moved a possibly tight wire and requested another full LEFT velocity200 calibration. It completed and persisted before the user's contact report/our stop arrived. User saw the camera bump the cart with the arm extended downward and rotating. Treat this as observed physical obstruction, not an actual joint end stop. Run had no communication error; elbow improved195deg, shoulder_pan stayed158deg versus right206, and wrist_flex shrank144deg versus right201. Final report flagged both mismatches plus old head pan. Do NOT accept this contact-affected run as valid.

Preserved its file in parent task work/calibration-with-camera-contact.json. Restored ALL LEFT entries/hardware offsets and limits from work/calibration-after-both-auto-arms.json (the preceding full200 run), preserving successful RIGHT calibration and unchanged head/wheels. All16 torque-off and hardware/file calibration equality explicitly verified. This restores the prior state only: left swivel159-vs206 mismatch and head pan still unresolved; restored calibration is NOT a clearance guarantee. Camera mount/placement and its cable must clear the cart across the calibration poses before another powered sweep. Prefer checking the identified affected joints with the user watching instead of repeated whole-arm motion.

### Left retry after repositioning passed all arm checks

Initially both serial boards reappeared but servos did not reply. After owner confirmed power lights, read-only preflight found both buses responding normally (left119/right121 raw V, torque0). Owner explicitly requested left calibration again after repositioning. Full LEFT velocity200 run exited0 and saved. Left raw limits/offsets: shoulder_pan959..3135/-22; shoulder_lift855..3239/975; elbow942..3152/107; wrist_flex898..3196/-826; wrist_roll115..3979/1318; gripper1275..2819/121. Travel:191,210,194,202,340,136 degrees respectively. Right previous automatic results preserved.

Final report: NO ARM FLAGS OR LEFT/RIGHT MISMATCHES. Only old head_motor_1 range0..4073 remains flagged as wrapped. Explicit readback verified ALL16 Torque_Enable=0 and calibration file offsets/limits equal hardware. Snapshot parent task work/calibration-arms-passed.json; verification work/arms-calibration-verification.json. This is now the newest calibration backup. Do not restore older manual/partial/camera-contact snapshots over these results. User has not yet provided visual clearance confirmation for this latest full run; motor data alone cannot rule out all contact. Head pan still needs supervised manual calibration; no whole-robot powered self-test until report is clear.

## Manual head calibration follow-up

- Corrected the head wrapper to write and read back measured head limits, reject implausible ranges, save atomically, and restore prior hardware settings on interruption or failure. Head motors remain torque-off; only head IDs 7 and 8 are selected. All 11 wrapper tests passed, including rollback for interruption, invalid ranges, readback mismatch, and failed file save.
- First recording was cancelled after the user corrected the forward reference. Second recording was cancelled after the user reported possibly turning too far left. Both cancellations restored and verified the previous head settings with torque off; the calibration file and successful arms were unchanged. No completed new head calibration yet.

### Completed manual head calibration

The user reset the head straight ahead and level, then manually swept left/right (up to approximately 90 degrees each way) and comfortable up/down travel. The routine recorded pan 1019..3151 and tilt 1932..2665, validated the ranges, wrote and read back the head settings, and atomically saved the merged calibration. The full calibration report is clear. An independent read-only check verified all 16 motors torque-off, all hardware offsets and limits matching the saved file, and every non-head calibration entry unchanged. No new powered motion test was performed.

### Movement test attempt after final calibration

The read-only position/health preflight stopped before any motor enable: right elbow raw position 3193 is beyond its saved maximum 3092 (minimum 1002), about 8.9 degrees beyond. Other arm/head positions were within saved limits; no servo-status faults were reported. Torque-off commands succeeded. The movement-and-return stage has not run. The user needs to reposition the right elbow gently inward while torque is off; do not widen the calibration to suppress the check.

The next repositioning check read the right elbow at the same 3193; it remains 101 ticks beyond the saved maximum. No joints were enabled or nudged. Torque-off commands succeeded again.

### Partial movement test after elbow repositioning

The user correctly repositioned the right elbow (normalized 84.7), and all position/health preflight checks passed. A 3-unit head tilt request moved only 1.09 normalized units (0.35 degrees), below the test's movement threshold. After a torque-off verification, the head tilt request was increased to 6 units (1.93 degrees); both head motions passed movement-and-return checks, as did left shoulder pan/lift. The left elbow started at normalized 98.64, requested 95.64, and reached only 98.37 (about 0.26 degrees of movement), failing the movement threshold. The test stopped and sent torque-off commands. No communication or load threshold failure occurred in this attempt. The left elbow needs inspection/repositioning away from its end pose before another attempt. Detailed results are in the parent task's `work/movement-test-left-elbow-small-step.json`.

### Movement test resumed: right claw remains unresolved

After the user repositioned the left elbow, its reading was normalized 85.6 and the nudge passed. Left wrist flex/roll and gripper passed; right shoulder pan/lift, elbow, and wrist flex/roll also passed. The final right-gripper nudge aborted. All motors were released and independently read back torque-off. Saved calibration still matches hardware. Ten stationary samples of that claw returned fault status 0, load 0, and 11.9–12.1 V. All 13 other arm/head small movement-and-return checks have now passed across the logged attempts, but visual identity confirmation and the right claw check remain incomplete. Evidence: parent task `work/movement-test-summary.json`.

### Final isolated claw attempts

Two right-claw movement attempts failed to reach the position target within the three-second leg timeout. The second attempt used explicit position speed 200 and a bounded 5.3-degree request: commanded raw goal 1430 verified, actual reached 1393 from 1370. Torque-off was verified at exit and the original position speed was restored. Right claw positioning remains unresolved; 13 other checks passed. A phone camera setup is now the active user request.

### Right claw passed and settings persisted

The isolated standard movement test also failed at P=16: raw goal 1460 from 1393, actual 1413, return command 1393. A bounded diagnostic used P=32 (the vendored driver documents this as the servo default), lowered Torque_Limit to 500, and explicit position speed 200. The commanded step was 4.5 normalized units / 5.97 physical degrees, within the profile's 6-degree step limit. The claw moved 3.245 normalized units and returned within the standard test tolerance. These settings were added as a right-claw-only profile override, with validation and hardware readback before enabling torque. A second check loaded the saved profile and passed: moved 3.046 units and returned within tolerance. Final independent readback verified all 16 torque-off, every calibration entry matching hardware, and all three saved claw settings matching hardware. All 14 basic arm/head movement checks have now passed. The repository suite passed 93 tests.
