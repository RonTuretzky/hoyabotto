# Where the build stands

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
- Load, step, travel, watchdog, lease and STOP checks are unchanged, and a nonzero servo `Status` byte still stops
  the carton owner.
- A robot-Mac working copy of the carton owner outside this repository (`work/`) keeps its old checks until the
  packaged scripts in `software/scripts/carton_robot/` replace it.

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
| Full-robot probe | Not run since final wiring. Expect one board with IDs 1-8 and one with 1-6, 9, 10 |
| Profile | `profiles/paper-tray-v0.yaml` has empty `port1`, `port2` and no camera indices |
| Calibration | Not done. Nothing can move until it is |
| Taught poses | None (`data/keyframes.yaml` does not exist) |
| Cameras | Never opened from software. They need 4-pin-to-USB cables; whether the kit included them was an open question |
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
