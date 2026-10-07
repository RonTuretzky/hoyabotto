# Measuring the real station and carton for the four-flap sequence

The simulated four-flap closure (`carton-four-flap-shorts-first.md`) only
transfers if the real station and carton match its assumptions. None of these
values has been measured. Measure them, rerun the simulation sweep with the
measured values, and only then consider a supervised hardware attempt through
the existing authorized sole-owner path (`carton-folding-readiness.md`: every
folding tool currently reports `motion_ready:false`).

## 1. Crease resistance (the largest unknown)

The simulation models each fold line as a spring toward upright plus dry
friction: torque = k·angle ± friction. Assumed: k = 0.018 N·m/rad,
friction = 0.004 N·m, damping 0.008 N·m·s/rad, rigid 3 mm panels, 23 g per
flap. Real corrugated creases may be several times stiffer and keep some of
their fold (crease memory), which the model does not represent.

Bench measurement, per flap type (one short, one long), on the actual carton:

1. Fold the flap fully in and out three times (a real operator pre-creases).
2. Hook a luggage or kitchen scale at the flap tip (140 mm from the crease)
   and pull perpendicular to the flap. Record the force while slowly closing
   the flap at 30°, 60° and 90°, and again while letting it open back through
   the same angles.
3. Torque = force × 0.14 m (force in N; grams × 0.0098 = N).
   Stiffness k ≈ slope of the average of closing and opening torques against
   angle (in radians). Friction ≈ half the difference between closing and
   opening torque at the same angle.
4. Release the flap from 90° and note where it stops: that is the angle where
   k·angle equals friction plus the flap's own weight.

Rerun the sweep with the measured values:

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py ... \
  --hinge-stiffness <k> --hinge-friction <friction>
```

The sequence now keeps the far major held from the moment it pins the shorts
until the end (a released far flap with a stiff crease springs open and lets
the shorts rise), so stiffer creases mainly raise the forces the jaws and the
open-claw short hold must supply.

## 2. Station geometry

| quantity | simulation value | how to measure |
|---|---|---|
| arm-base origin height above the tabletop | 120 mm | tape measure from tabletop to the SO101 `base_link` origin (the base mounting plane); raise the table or lower the cart to match |
| arm-base origin line to the near table edge | 150 mm | horizontal distance from the line through both base origins to the table edge |
| base spacing | 300 mm | between the two base origins |
| carton near wall to table edge | 10 mm | carton placed square to the table edge |
| carton | 379 × 283 × 108 mm, 140 mm flaps | measure the actual carton; these are already in `carton/geometry.py` |

The 120 mm base height was chosen because at 60 mm no far-edge contact kept
8 mm clearance from the outward near flap; 100 mm and 140 mm failed in other
stages. If the real station cannot match it, rerun the sweep with
`--base-height` and `--base-to-table-edge` set to the measured values before
anything else.

## 3. Presentation and markers

- **Far flap presented about 1° inward** (nearly upright, not leaning
  outward): the left arm reaches the far top edge only from about +1°. More
  than about 1.6° inward intersects the upright shorts.
- **Near flap** is opened to −15° by the robot; leaning further out made the
  sequence worse (9/20 at −17°).
- **Printed carton markers**: IDs 10 (near wall centre), 21 (left wall,
  y = +40 mm), 22 (right wall) and the three added ones, 26/27 (near wall,
  x = ±120 mm) and 28 (left wall, y = −80 mm), all 45 mm, at half wall height.
  Their measured positions must match `carton/folding_markers.py`.

## 4. Cameras

The station RGB-D camera pose and intrinsics come from the existing
calibration; the simulation adds 0.8 mm depth noise and pixel dropout. The
sequence relies on depth-measured flap edges (registration was 2–4 mm off in
simulation) and a bounded encoder/CAD angle when a flap is edge-on to the
camera. Verify the real camera sees the near flap below about 20° and above
about 25°, and the far flap below about 38° and above about 46°.

## 5. Not covered by the simulation

Taping and hands-clear retention, jaw friction and wear, servo compliance
under load (the simulated servos track positions with gravity sag only), and
the carton's contents. A held four-flap closure in simulation is a plan to
test, not a measured capability.
