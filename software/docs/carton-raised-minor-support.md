# Raised minor-flap support — 6 October 2026

**Neither variant has completed the carton.** A new paddle pose can support
both short flaps partway folded in an isolated prepared-state test. The
robot has not reached that state through a successful folding approach.
The bare-claw searches have not produced an equivalent clear pose.

## What the prepared test establishes

The right hand grips the paddle at 122.77 mm from its handle base, with an
initial grip rotation of 22.55°. The paddle spans the two raised minor edges.
The left hand is parked. Both short flaps are explicitly initialized at 75°;
the near major is initialized at −37.66°. These initializations are reported
as preparation, not robot actions or observed folding success.

The same empty 272 g carton, six-degree-of-freedom carton joint, 0.35 table
friction, 0.018 Nm/rad hinge springs, 0.004 Nm hinge friction and original
robot limits apply. The tool is passive, held only by jaw contact. There
are twelve robot actuators and no equality constraints, contents or tape.

After three seconds:

| Quantity | Result |
|---|---:|
| Left/right short angles | 74.48° / 76.03° |
| Maximum box translation | 0.0064 mm |
| Maximum tool blade offset | 0.243 mm |
| Maximum tool rotation in grip | 4.30° |
| Maximum robot/flap penetration | 0.171 mm |
| Paddle normal force on left short, samples after 0.3 s | 0.041–0.137 N |
| Paddle normal force on right short, samples after 0.3 s | 0.070–0.140 N |

Angles and penetration are checked at every 2 ms physics step; the listed
contact forces are sampled every 0.1 s. These are simulated measurements,
not calibrated physical measurements. This is a support mechanism test,
not a fold, handoff, pickup, tape or hands-off-retention pass.

## What failed during the approach

The new grip covers the old tool tag at 135 mm. Moving the two declared
simulated tool tags to 180 mm restored fresh camera registration. That is
an explicitly proposed marker mounting change, not a hardware change.

Keeping the paddle horizontal throughout the approach exceeded the IK
gate: 16.98 mm error for the central approach and 18.26 mm for a closer
front-edge approach. Rotating the paddle into the crossbar orientation
made the early targets reachable. Adding 10 mm of initial outward
clearance cleared the planning margin at the first waypoint.

During that approach, however, the right forearm contacts the near flap
from about a 26° commanded short-flap angle. The near flap is simultaneously
held by the left claw. The box then slides and the brace is lost. With
explicit opposing-face verification, the trial stops at 37.598 s with
43.44 mm maximum box movement; the right short is only 34.90° folded.
An earlier version without that brace check continued to 78.24 mm box
movement before its joint-tracking stop. It is not a successful handoff.

Moving the brace nearer the left corner still failed, this time at the
20° tool-rotation slip gate with 65.90 mm box movement. Trying to open the
near flap farther outward first lost the pinch around −36°, before the
right short was folded. These failures are contact/clearance problems;
increasing friction or weakening the crease would conceal them.
A separate attempt to preserve only the jaw face normal during opening
lost the pinch sooner, around −20°, and also did not reach a short fold.

Finite static searches also tested 75° short flaps with the right claw,
65° short flaps, the left claw, and unequal short-flap angles. None of the
sampled bare-claw poses passed the collision check. This is not a proof of
global infeasibility, particularly with unmodeled physical finger attachments.

## Separate 45-degree layout trial

Rotating the carton to 45° while keeping its nearest bottom corner 10 mm
inside the same table edge did not change the robot/table spacing. The
initial outside carton markers were occluded. A proposed floor marker at
the carton centre was also obscured by the upright near flap; moving that
declared marker 80 mm toward the far short wall made registration possible.
This marker/layout proposal is not the original 30° comparison.

The paddle folded the first short flap, but the original left-hand retreat
ended only 3.39 mm from the other upper arm, inside the planner's 6 mm
clearance margin. A different retreat, 60 mm left, 50 mm backward and 60 mm
upward in the world frame, passed that check. Both shorts were then held at
92.77° / 93.27°, with 5.42 mm maximum carton movement. Both major flaps
remained open. This is another partial hold, not complete closure or release.

The corresponding claw approach missed by 29.73 mm. Moving its proposed
contact closer to the corner made IK reachable but left finger/panel
intersections above the existing 1 mm bound. A static grid found 39 reachable
contact candidates and no collision-qualified candidate. Searches for a
crossbar pose using the existing 60 mm / −71° paddle grip also found no
pose-qualified sample at 65° or 75° in the 30° layout, or at 75° in this
45° layout. None of these finite searches proves global impossibility.

## Reproduction and remaining work

A further attempt keeps the right short held while the left releases its
near-flap pinch, withdraws, and presses the near major. It is selected with
`--press-near-after-short` in `tools/diagnose_braced_folding.py`. The first
bare-claw approach missed the 6 mm planning clearance by 0.44 mm. Moving its
approach farther out and upward cleared planning but the carton moved 63.94 mm
and rotated 19.70°; execution stopped at 1.227 mm panel penetration. The near
major was still −25.87°, so this was not a successful major fold.

The paddle attempt moved the free carton 78.14 mm and rotated it 19.12° before
fresh carton registration was lost. The near major was still about −5°.
This provides direct evidence that retaining one minor with downward pressure
is insufficient to anchor the carton during this particular major-flap push.
No clamp, friction increase or motor-limit change was introduced. These
finite failures do not rule out a different reachable contact strategy.

An isolated [passive tape component](carton-tape-material-audit.md) now tests
finite holding and peeling. It remains separate from robot folding, with
unmeasured material values and unconverged peel timing.

`tools/diagnose_partial_minor_bridge.py` runs the prepared support test using
the ordinary simulation runtime, collision checks and independent tool-slip
measurement. It records `prepared_pose_only: true`, `full_task_complete: false`
and `folding_approach_executed: false` separately from the support result.

From `software`, after generating the earlier two-short-flap run:

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_partial_minor_bridge.py \
  --simulation-root /path/to/gemma-xlerobot \
  --prepared-run /path/to/tip-paddle \
  --candidates docs/evidence/carton-raised-minor-support.json \
  --out /path/to/new-prepared-support-run
```

The evidence file retains the chosen static candidate, result paths and
hashes. The original complete partial-run GIFs still show both short folds
and their release failure; see the [end-contact audit](carton-paddle-end-contact-audit.md).
The new short support GIF must not replace either complete sequence as proof
of folding. Both major folds, a dynamic handoff, explicit tape application
and five seconds of verified arms-clear retention remain unfinished.
No hardware commands were issued.
