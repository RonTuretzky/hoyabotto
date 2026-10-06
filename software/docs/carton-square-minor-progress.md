# Square-carton proposal: two short folds, then spring-back

Neither tool has completed the four-flap carton task. These are offline
experiments with an empty, unbolted carton and resistant hinges. No physical
robot or camera was used. The complete recorded attempts include their failures;
there is no successful full-closure GIF.

## What this separate setup changes

The carton is square to the robot instead of the earlier 30-degree layout.
Its nearest bottom corner remains 10 mm inside the table. The robot/table
locations, actual imported arm meshes, original joint and force limits,
contact guards and material parameters are unchanged. Both arms start in
validated rearward parked poses. Two proposed 45 mm tags are on the inside
floor: ID 24 at x=80 mm and ID 25 at x=0, both at y=0. Their locations and
the external camera mount have not been measured on hardware.

The paddle is a free body held in the right jaws, initially prepared at
122.77 mm from its handle base and 22.55 degrees of grasp rotation. Pickup is
not tested. Its proposed tags are at x=180 mm to avoid the grip. This differs
from the earlier blade-end comparison and must not be presented as a matched
repeat of that experiment.

The final trials use rendered 1280 x 720 RGB and aligned depth, with 0.8 mm
independent depth noise and 25% missing samples. This is a sensor assumption,
not a verified OAK streaming or accuracy result. A failed paddle pose at
960 x 540 decoded only tags 1, 4 and 20; the same scene at 1280 x 720 decoded
1, 4, 11, 20, 21, 23 and 24. Raising resolution removed that particular
visibility failure; it did not solve the paddle's subsequent reach failure.

## Resistance and freedom are part of the physics

- Empty carton mass: 272 g; contents mass: zero.
- Six-DOF free joint; no weld, clamp, guide or hidden carton actuator.
- Explicit table/cardboard friction: 0.35.
- Crease stiffness: 0.018 Nm/rad, rest angle upright.
- Crease friction: 0.004 Nm; damping: 0.008 Nms/rad.
- Original arm torque limits: 2.94 Nm; jaw limit: 0.5 Nm.
- Intended robot/flap penetration remains limited to 1 mm.

These values are unmeasured assumptions. The panels are rigid and the creases
are elastic/frictional hinges. Plastic crease conditioning, panel bending,
buckling and real cardboard hysteresis are not modeled. Passing this model
would still not establish performance on the user's carton.

## Recorded outcome

| Variant | Result | Maximum horizontal box motion |
|---|---|---|
| Two claws | Both short flaps reach 91.19 and 90.16 degrees while held. After releasing the left hand and observing for five seconds, the left flap reopens to 57.87 degrees. Long flaps remain open. | 0.033 mm |
| Left claw and right paddle | Right short flap reaches 62.68 degrees. Stops when the next IK target misses by 8.44 mm, exceeding the unchanged 8 mm bound. Left short flap remains at 7.26 degrees. | 1.000 mm |

The claw run lasts 49.306 simulated seconds; the paddle run lasts 26.620.
The very small claw translation is a simulated bracing result, not a carton
anchor: a separate physics test applies 2 N and confirms that the free box
slides with low table friction. Maximum 3D displacement includes about 1.10 mm
of initial vertical settling; new logs separate horizontal motion from height.

The left flap opens about 33 degrees from its held pose. The right hand still
holds its flap during this negative control. This is **not** five-second
hands-free retention of a closed carton. Tape has not been applied in either
folding run; the isolated tape coupon is a separate component experiment.

## Fresh tracking through a fold

When rigid wall/floor tags are hidden, the observer can now use both short-flap
tags, IDs 11 and 12. Their measured world transforms and declared mounting
transforms locate the two hinge origins. The hinge-to-hinge direction gives
the carton x axis; their shared hinge axis gives y; the cross product gives z.
The known rim height locates the base. No commanded flap angle, stored carton
pose, level-table assumption or simulator object state enters that estimate.

Both tags must be fresh. A missing pair, hinge spacing error above 12 mm, or
axis disagreement above 8 degrees stops recovery. Physical use requires
measured tag-to-hinge mounts. On the rendered claw frame where wall/floor
tracking previously failed, the fallback has 0.739 mm position error and
0.400 degrees rotation error against an independent simulator evaluation.
That is one synthetic sensor sample, not a physical accuracy specification.

Fifty varied rigid carton poses and independent flap angles test the geometry.
A compiled-scene test checks the actual marker mounts. Missing, nonrigid and
inconsistent observations are rejected. All 105 focused folding tests pass,
including actual passive spring-back and low-friction free-box sliding.

## Planning and failed alternatives

The geometric joint planner now evaluates kinematics and contact geometry
without building force constraints for deeply colliding rejected candidates.
The old constraint construction could exhaust MuJoCo's stack. Contact outputs
match the full position pipeline in a regression test and 21 saved real-CAD
states, with zero numerical difference. Executed physics is unchanged.

A finite audit of eight camera positions and two table-tag layouts for each
tool found no candidate observing all sampled poses under the existing
registration requirements. Adding the centre floor tag alone also failed.
These are limited searches, not proof that another camera arrangement cannot
work. The successful claw continuation used fresh hinge tags instead.

## Reproduce

Use new output directories; the runner refuses to overwrite previous trials.
These commands execute only the offline simulator.

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_short_flap_brace.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/new-claw-run --tool claws \
  --fold-right --press-left --release-left-minor \
  --radius .115 --carton-yaw-degrees 0 --park-back --normal-only \
  --floor-marker-x .08 --center-floor-marker --width 1280 --height 720 --video

PYTHONPATH=. .venv/bin/python tools/diagnose_short_flap_brace.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/new-paddle-run --tool paddle \
  --fold-right --paddle-contact handle --scan-tool --paddle-along-travel 0 \
  --radius .115 --carton-yaw-degrees 0 --park-back --normal-only \
  --floor-marker-x .08 --center-floor-marker --width 1280 --height 720 --video

PYTHONPATH=. .venv/bin/python tools/render_folding_comparison.py \
  --comparison /absolute/path/to/review --case square \
  --claws /absolute/path/to/new-claw-run --paddle /absolute/path/to/new-paddle-run
```

The comparison renderer replays the recorded physical states at their recorded
times. Its final 2.5-second display pause is not extra simulated retention.
The output includes separately labeled full attempts and a side-by-side GIF.
[Parameters, source hashes and results](evidence/carton-square-minor-progress.json).

Remaining work is a dynamic support transfer that preserves the short folds,
both major folds, explicit tape placement and five seconds with both arms
clear. Neither a paddle advantage nor reliable carton closure is established.
