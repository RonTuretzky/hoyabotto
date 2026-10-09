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
