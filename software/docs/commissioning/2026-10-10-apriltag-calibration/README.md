# AprilTag stationary sampling and short-move diagnosis

The collector is read-only: it accepts only specific read calls, pins all sixteen
encoder positions and owner markers during the dwell, and saves five separately
bracketed OAK tag observations. It neither positions the robot nor installs a fit.
Phone/wrist views are supplemental, not metric registration inputs. Repeated
frames at one pose count as one calibration pose.

Run from the repository with a Python environment containing its dependencies:

```sh
PYTHONPATH=software python software/tools/collect_stationary_tag_pose.py \
  --pilot-root /path/to/pilot --geometry /path/to/apriltag-geometry.json \
  --arm right --out /path/to/new-evidence --samples 5 --interval-s 1
```

`settled-evidence.json` preserves the original live capture and installed recorder
repair evidence. `recorder-short-buffer.patch` is an archival diff for the external
pilot installation; do not blindly apply it again. The installed recorder SHA is
`a176ac0f62c6be5477e5be469d8df762ee18001ed85393b7d2186242065cf6b8`.

## Observed short lift

In `live-lift-20261010-c`, a single right shoulder target of 3200 was requested
from 3216 at normal speed. Readback was 3214, although the owner reported
`endpoint_settled`. The calibration transport rejected the 14-tick residual, then
STOP/release returned the encoder to 3216. Subsequent read-only sampling confirmed
the original pose. This establishes a reporting mismatch, not the mechanical
reason for the short movement and not a validated lift direction.

The owner's pickup controller settles within 57 ticks (about 5 degrees). The
entire requested movement was 16 ticks (1.41 degrees), with only two ticks
(0.176 degrees) observed. The local fix checks small-move residuals only after
the original controller has finished. It reports a missed endpoint as
`settled_short`, preserving the existing command writes, dwell, correction
budget, contact handling and limits. It does not make a small request travel
farther. Tightening the controller's settling tolerance directly would invoke
additional overdrive corrections, which this fix deliberately does not do.

The reporting check covers single non-gripper arm-joint moves of 3–57 ticks,
requiring at most `max(1, min(5, distance // 10))` ticks endpoint error. Paths,
head motion, gripper closure and larger moves keep their existing contracts.
The regression replays 3216→3200 with measured3214 and asserts exactly one write
to3200, zero corrective writes, and `endpoint_reached:false`. Boundary cases test
both directions, no travel, exact tolerance and missed tolerance.

This controller fix is local source, not deployed or physically validated.
No automatic retry, larger target, torque change, or direction assumption follows.

Verification: all31 required bridge test scripts pass, including the recorded
shortfall regression;239 stationary sampling/registration tests also pass.
The live-owner reporting change requires a released-only deployment preserving
the current Joy-Con setup and camera publishers. The default broad restart may
reconfigure cameras, so do not use it merely to install this reporting fix.

## Recording limitation

The same attempt's 120-second recording contains 921 OAK and 610 wrist source
frames with no gaps over one second. The phone contains 295 frames and two such
gaps. Its sidecar records only a total count, so it cannot establish whether those
gaps intersected the approximately 6.19-second enabled interval. Do not count the
encoded video's repeated frames as new camera observations. Future recorder
evidence should retain source timestamps or gap intervals before qualifying only
the enabled interval. The failed historical coverage verdict remains unchanged.

## Remaining calibration work

Before/after stationary tag variation was at most0.252/0.207mm respectively.
That measures observed noise at this pose, not absolute camera/arm accuracy.
Use measured encoder travel plus multiple settled tag frames to distinguish a
real displacement from noise; never enlarge a request simply until visible.

Physical joint zero/direction, camera-to-base and tag-to-gripper transforms remain
unvalidated. The candidate shoulder mapping is outside the pinned URDF range.
The fitter still requires eight fitting poses and three distinct held-out poses.
Neither the stationary collector nor this reporting correction makes the pilot
ready for registered Cartesian motion.

## Result, 02:40 JST: right-arm camera-to-arm registration validated

The automatic registration (`robot_calibrate_tags mode=registration`) ran to all eleven poses on
the real robot and the fit passed. It is installed as the pilot's `.private/tag-registration.json`
and `robot_get_registered_tags` answers live. Evidence in `held-registration/`.

| | train (8 poses) | held-out (3 poses) |
|---|---|---|
| position RMS | 1.47 mm | 0.84 mm |
| position max | 1.86 mm | 1.06 mm |
| orientation max | 0.29 deg | 0.31 deg |

Live re-read while the arm was held (`live-registered-read.json`): gripper tag 2 at
(385, 21, 33) mm in the right arm base, repeatable to 0.3 mm over three reads, observed-versus-FK
gripper consistency 0.57 mm / 0.35 deg. Table tag 1 at (819, 301, 71) mm (the desk was moved about
a metre away for this). Frames: SO-101 URDF right arm base (+x forward, +y the arm's left, +z up),
camera `CAM_A_optical`. Rotation excitation singular values 0.79 / 0.76 / 0.05 rad, position extent
11 x 93 x 35 mm: good about two axes, as the two-axis grid implies.

### How it was made to work (each run refused on a different thing)

1. **The released arm sags out of the OAK view.** Lifting the hand with the pilot's own
   `robot_move_joint_targets` (elbow -300 ticks, confirmed direction: fewer ticks lifts the hand) put
   tag 2 in view; releasing let the forearm fall back past the start (elbow 1335 -> 1881). So the
   registration now accepts a **held start** (`GemmaTransport(held_start=True)`,
   `run_calibration(held_start=True)`, CLI `calibrate_gemma_tags.py registration --execute --from-held`,
   tool argument `held_start: true`): exactly the six motors of the arm must already be holding; no
   enable is sent; the run still releases (or STOPs) at the end. `held_registration.py` is the driver:
   pan 2000, wrist_flex 1780, shoulder_lift 3100, elbow 1335 (shoulder_lift must be below 3184 or the
   candidate degrees exceed the URDF's +/-100 and FK refuses), then the registration.
2. **Camera stamps trail the movement cutoff.** OAK frames arrive 0.37-0.51 s after capture, encoder
   rows 0.13-0.16 s; a frame inside the encoder bracket but before the local-clock movement cutoff
   was refused at once ("camera frame did not advance"). The observer now keeps reading inside its
   deadline instead; `frame_age_s` is 2.0 in the config. The same wait was added to
   `read_registered_tags`, which single-read a pre-bracket frame every time.
3. **The owner's position controller stops short.** Every command landed 3-7 ticks short on the pan and
   11-14 on the wrist (`steps.json`), and 13-17 on the loaded elbow; the owner's own endpoint tolerance
   is 57. The Limits cap on `settle_ticks` went from 5 to 16 (config: settle 16, step 32, trust 80,
   max_path_ticks 2000; requested travel is about 1.5x the planned grid because of the shortfall), and
   `_move_to` accepts a catch-up step that ends inside the tolerance (wrong-way motion still refuses).
4. **Release confirmation on a sagging arm.** After the eleventh pose the normal release read the
   shoulder as Moving=1 (gravity) and refused; `finish()` now requires fresh torque-zero on all sixteen
   motors, not settled telemetry. That run's eleven poses were fitted offline with the same
   `assemble_dataset`/`fit_registration` (`registration-result.json`, `release-refusal.json`).

Also fixed: `test_tag_registration_contract.py` expected the pre-f789d13 owner wording for an
obstructed step (now `settled_short`). 269 tests pass across the tag suites.

### What this does and does not give

- Gives: fresh tag centres and poses in the right arm base frame while head, cart, desk tag and
  gripper tag stay fixed; a measured OAK camera pose (`base_from_camera`) for the simulator.
- Does not give: the jaw contact offset from tag 2 (`gripper_from_tool`), workspace bounds,
  collision clearance, or Cartesian motion. `robot_get_arm_pose` still says
  NEEDS_GEOMETRIC_CONFIGURATION for those. Moving the head or the OAK invalidates the fit.
