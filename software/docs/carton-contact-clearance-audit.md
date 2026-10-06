# Contact and starting-pose audit — 6 October 2026

The complete carton task remains unfinished. This audit strengthens the
physics checks used to judge future attempts; it does not turn the existing
two-short-flap holds into successful closure.

## Corrections verified in the real simulation model

1. **Reject initial robot/panel intersections.** An earlier zero-yaw trial
   started with wrist geometry 9.9–15.6 mm inside the near flap. Its subsequent
   20.7 mm carton movement was contaminated by that invalid initial state.
   Both arm-only and paddle initializers now reject obstacle/flap overlaps
   beyond 0.1 mm before running dynamics. Intentional paddle/jaw gripping
   contact remains permitted. The original 30° comparison starts clear.
2. **Bound intended contacts during planning.** Naming a flap as an allowed
   contact previously bypassed its penetration depth. Allowed jaw or paddle
   contact now has the same 1 mm maximum penetration bound as other motion
   checks. This prevents a geometrically intersecting static pose from being
   accepted merely because its fingertip positions are correct.
3. **Check all robot/flap contacts during execution.** Every 2 ms step records
   penetration, including planned pressing and incidental forearm contact.
   A value above 1 mm stops the segment and propagates an error through arm,
   gripper and joint-path commands. The tool-slip and existing forbidden-contact
   checks still apply independently.

No material coefficient, collision shape, actuator, joint limit, carton
constraint or physical station dimension changed in these corrections.
All 92 folding tests pass. Tests include actual signed-distance contacts,
an allowed shallow contact, rejection of a deeply intersecting named contact,
initialization checks, and a stop at the first excessive-contact time step.

## Full replay of the existing two-short-flap sequences

| Result | Claws | Paddle end contact |
|---|---:|---:|
| Simulated duration | 67.592 s | 67.994 s |
| Maximum robot/flap penetration, all steps | 0.369 mm | 0.125 mm |
| Maximum box translation | 12.807 mm | 4.956 mm |
| Left short flap after withdrawal + 2 s | 58.396° | 58.298° |
| Full carton closure | **Not completed** | **Not completed** |

The saved scene XML and every recorded state, timestamp and frame label are
identical to the preceding end-contact comparison. Its complete GIFs therefore
still show the newly checked trajectories exactly. Both include spring-back;
the final display pause is not retention evidence. See the
[end-contact audit](carton-paddle-end-contact-audit.md) for the GIFs and commands.

## Retention approaches explored

A static search varied paddle grip angle, grip location, crossbar height and
orientation, and contact position. A separate search varied the opening and
pose of the stock claw to put one fingertip on each short flap. In the original
30° layout, 62 paddle poses and 89 claw poses met their position/orientation
criteria; none of those samples cleared the near-major-flap sweep. Forearm,
hand and flap interference was the recurring issue. A separate 100 mm-lower
table proposal, preserving horizontal distance and physics, yielded 59 paddle
and 89 claw pose candidates, again with no collision-qualified full sweep.
These finite searches are **not a proof that every possible strategy fails**.
They do not authorize a physical station change or validate a moving handoff.

A zero-yaw carton experiment used a different, verified clear starting arm
pose, leaving the carton 10 mm inside the same table edge. The initial camera
observation contained both housing tags, table anchors and a carton marker.
The first grasp then missed its target with the original soft-orientation IK.
An experimental position-constrained solver reached the grasp, but the carton
subsequently moved and its external tags became occluded. A proposed interior
floor tag, used only in that local empty-carton experiment, maintained fresh
registration and exposed the subsequent physical failure: the brace slipped,
with 114.8 mm movement for claws and 107.6 mm for the paddle. Neither trial
reached a short-flap fold, and both partly overhung the table before stopping.
That controller and marker change are not enabled in the released runner.

A subsequent variant rotated the measured gripper pose about the visually
registered hinge and attempted to hold the near flap upright instead. It
reached the position-error stop at 8.2 mm for claws and 8.1 mm for the paddle,
after roughly 46 mm and 45 mm of box movement respectively. It did not reach
either short fold. Those results do not establish a rigid, stable panel grasp
merely because both jaws touched opposite faces.

The remaining work is a valid dynamic hand transfer, both major folds,
explicit tape application and five seconds of hands-off retention. Material
values, proposed camera geometry and physical tag mounting remain unmeasured.
No hardware commands were issued.

Quantitative results, source hashes, replay identity checks and local trial
paths are recorded in [the evidence file](evidence/carton-contact-clearance-audit.json).
