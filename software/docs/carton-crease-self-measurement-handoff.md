# Handoff to the robot: measure the carton's crease resistance (and the station) yourself

For the agent running on the robot Mac with the robot API tools. The owner
supervises every motion at the STOP button. Goal: measure, on a **pre-folded**
carton, the numbers the folding simulation currently has to assume, so the
simulation can be rebuilt to match. Nothing here folds the carton.

## Rules (read first)

- Before every motion, tell the owner in one sentence what will move and
  where; wait for "go". Keep `robot_stop` ready; `robot_halt_motion` holds in
  place, `robot_stop` releases everything.
- Move slowly: `duration_s` at least 2 s for any move near the carton,
  touches in small steps (5–10 mm, 1 s each). Watch `robot_get_motion`
  (following error) and `robot_get_state` (`Present_Load`) after each step.
- Never relax limits, never push the carton across the table: if the carton
  visibly moves (phone or OAK frame), stop and report.
- Start with all motors released (`robot_get_state`: `Torque_Enable` 0).
  Enable only the arm you use (`robot_set_motor_enable`), release it when done.
- Plan reaches with `robot_plan_reach` (read-only), move with
  `robot_move_joint_targets` / `robot_move_path`, read the claw-tip position
  with `robot_get_arm_pose` (arm-base metres, live encoders). If
  `robot_get_arm_pose` reports missing geometry for an arm, use the other arm
  or stop and report.

## How a "touch" works

A touch measures a point with the claw tip instead of the camera:

1. Move the closed claw (gripper closed) to about 20 mm above or beside the
   point.
2. Step toward it in 2 mm moves (1 s each). After each step read
   `robot_get_state`. Contact is when the pushing joint's `Present_Load`
   rises clearly above its value at the previous free step (record both), or
   following error appears in `robot_get_motion`.
3. Record `robot_get_arm_pose` (tip x, y, z), then back off 10 mm.

Repeat each touch twice; if the two differ by more than 3 mm, do a third.

## Step 1 — station geometry (about 10 touches)

Record every result in arm-base metres, with which arm measured it:

- **Tabletop height:** touch the bare tabletop at three points in front of the
  robot. Base height above the table = −(mean z of the touches).
- **Table near edge:** touch the top of the near table edge at two points.
- **Carton:** with the carton placed where folding would happen, touch the top
  rim of each wall near its middle (near, far, left, right) and two top
  corners. These give the carton pose and confirm its size (379 × 283 × 108 mm
  expected).

## Step 2 — crease release tests (per flap: one short, one long)

The owner pre-folds the carton (each flap fully in and out three times) and
weighs it on a kitchen scale (record grams). Do one short flap and one long
flap; both of the same type behave alike.

For the chosen flap:

1. **Hinge line:** touch the top of the wall at the flap's crease at both
   ends of the crease (two points). This is the hinge line H.
2. **Closed release:** push the flap inward to about flat (90°) by touching
   its outer face about 25 mm below its free edge and stepping inward. Hold
   2 s, then lift the claw straight up 50 mm slowly and wait 5 s. Then touch
   the flap's **free edge** (from above) and record the tip point T1.
3. **Outward release:** push the flap outward (away from the box) to about
   30° past upright by touching its inner face and stepping outward. Lift
   clear, wait 5 s, touch the free edge, record T2.
4. Angle of a resting flap from upright: in the plane perpendicular to the
   hinge line, θ = atan2(horizontal distance of T inward from H, height of T
   above H). Inward is positive (θ1), outward negative (θ2).

Also report plainly whether, after step 2, the flap **stayed near flat or sagged
past flat into the box** (θ1 ≥ 88°): that is the "sag check" the simulation
needs.

## Step 3 — compute (or just report T1, T2, H and the weight)

With flap length L = 0.14 m, flap mass m ≈ carton mass × (flap area / total
cardboard area) — for 379 × 283 × 108 mm with 140 mm flaps, a short flap is
about 9% and a long flap about 12% of the cardboard (empty carton) — and
g = 9.81, gravity torque at angle θ is
m·g·(L/2)·sin|θ|. A released flap stops where spring minus gravity equals
friction, from either side:

```
k·θ1 − m·g·(L/2)·sin θ1 = f          (closed release, θ1 in radians)
k·|θ2| − m·g·(L/2)·sin|θ2| = f       (outward release)
k = m·g·(L/2)·(sin θ1 − sin|θ2|) / (θ1 − |θ2|)
f = k·θ1 − m·g·(L/2)·sin θ1
```

If the flap stayed flat or sagged in step 2 (θ1 ≥ 88°), report that instead
of k and f: the crease is too soft to reopen against the flap's own weight,
which the simulation must model differently (it then fails at its first
stage). If θ1 and |θ2| are within 5° of each other, the two equations are
degenerate: report the angles only.

Optional cross-check: hold the flap with the claw at 30°, 60° and 90° and record
the pushing joint's `Present_Load`, then the same poses with the flap folded
out of the way; the difference rises with angle if the crease resists.
`Present_Load` is in servo units, so this is relative only.

## What to report

Write `work/carton-crease-measurement.json` on the robot Mac and paste it to
the owner:

```json
{
  "date": "...", "arm_used": "...", "carton_mass_g": 0,
  "station": {"tabletop_z_m": [], "table_edge_points_m": [], "carton_rim_points_m": {}},
  "flaps": {
    "short": {"hinge_points_m": [], "T1_m": [], "T2_m": [], "theta1_deg": 0, "theta2_deg": 0,
              "sagged_past_flat": false, "k_Nm_per_rad": null, "friction_Nm": null},
    "long": {"...": "same fields"}
  },
  "notes": "carton moved? any refused moves, loads at contact"
}
```

Also add a short dated entry to `software/STATUS.md`, as the working rules ask.
These numbers feed the simulation via `--hinge-stiffness`, `--hinge-friction`
and the station options (see `docs/carton-real-station-measurements.md` and
`docs/carton-sim-training-handoff-2026-10-08.md`).
