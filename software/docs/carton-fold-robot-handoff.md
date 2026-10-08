# Carton fold: procedure for the robot agent

Close all four flaps with the two bare claws and hold them. Untested on the
real robot; positions come from the simulation and need checking against the
cameras and `robot_get_arm_pose`.

## Practical details

- Tell the owner what moves before each motion and wait for "go".
  `robot_halt_motion` holds; `robot_stop` releases everything (the flaps
  spring open).
- Near the carton: `duration_s` ≥ 2 s, steps ≤ 5 mm or ≤ 3° of flap rotation.
- Stop if the carton moves more than 15 mm, or if anything other than a jaw
  pushes cardboard.
- While holding flaps, send a command at least every 100 s or the motors
  release. Re-sending `robot_set_motor_enable` (`enabled: true`) renews this.
  If that is refused (a loaded claw), move a joint a few ticks instead.
- A move ending `contact_halt` means the arm stopped pushing and is holding.
  Re-measure, then continue in smaller steps.

## Carton and frame

- Carton 379 × 283 × 108 mm, flaps 140 mm, cardboard 3 mm.
- Carton frame:
  - origin at the centre of the carton bottom, on the table;
  - +x to the robot's right, +y away from the robot, +z up;
  - all positions in mm.
- Hinges: short flaps at x = ±189.5, z = 108; long flaps at y = ±141.5,
  z = 111.5.
- Angles: 0° = upright, 90° = closed, negative = leaning outward.
- Folded shorts leave a 99 mm gap at the centre (x = −49.5 to +49.5).
- First measure the carton's pose in each arm's frame (touch its rim, see
  `carton-crease-self-measurement-handoff.md` step 1), then convert the points
  below. In the simulation, the arm bases were at (∓150, −301.5, 120):
  120 mm above the table, 150 mm behind its edge, carton 10 mm in from the
  edge.
- Before starting: near flap leaning slightly out, far flap upright to
  about 1° inward.

## Procedure

**0. Near flap out to −15° (right claw, closed).** From inside the carton
above the near flap, about (6, −108, 257), lower to z 254 and push outward
toward (5, −155, 223) until the flap is at −15°. Lift to z 317 and park.

**1. Brace the left short (left claw).** Open the claw about 34° (0.6 rad).
Lower it over the left short's top edge at (−173, −100), from z 255 to z 220.
Close it on the flap.

**2. Fold the right short (right claw, closed).** Contact its outer face near
the top at about (234, −108, 245). Push it inward along an arc about its hinge,
about 2° per step, to 90°. Tip path (x, z): (234, 245) → (162, 240) →
(106, 212) → (90, 178) → (88, 150) → (94, 117). Hold it there.

**3. Fold the left short (left claw).** Open, lift out to (−213, −101, 260)
and close the claw. Go to (−255, −101, 216) and push the outer face along the
mirror arc to 90°, ending at about (−93, −99, 114). Hold.

**4. Right claw holds both shorts.** Roll the right wrist while still holding
the right short. Open the claw gradually to about 77° (1.35 rad) while sliding
left. It ends at about (−49, −42, 113), one jaw resting on each short:
- jaws must overlap each short by at least 10 mm (span about ±60 mm);
- jaw underside about 5 mm above the rim (z 113).

Lift the left claw off in four 1.5 mm steps, then clear.

**5. Pin the far flap at 34° (left claw, closed).**
1. Go above its top edge at (−156, 146, 283).
2. Lower onto the edge (z ≈ 263), 2 mm outside the edge's centre line, and
   press 0–3 mm.
3. Drag it toward the robot.
4. If it slips: lift, re-measure the edge, re-grip (up to 4 times).
5. Once past about 4°, place the tip 1–2.5 mm behind the outer face and push.
6. Stop at 34°: tip about (−156, 76, 234).

**6. Right claw lets go.** Lift 15 mm, close the claw in the air, and park at
(200, −180, 300). The far flap now holds the shorts down.

**7. Far flap to 70° (left claw, never let go from here on).** Push on to 70°
(tip about (−157, 27, 166)), so the left forearm is clear of the near flap's
swing. Re-grip if it slips.

**8. Near flap to 88° (right claw, closed).**
1. Go outside the near flap at (−39, −184, 267).
2. Contact its outer face 25 mm below the free edge, at x = +25 to +40
   (between the shorts).
3. Push it closed along its arc to 88°: tip about (31, −30, 121).
4. Hold it there.

If there is no contact, back out and try a nearby spot (up to 3 times).

**9. Far flap to 88° (left claw).** Keep pushing with the same contact,
sliding down the panel at most 5 mm per step, to 88°: tip about
(−157, 34, 129).

**10. Hold.** Done when the shorts are at 85–110° and both long flaps at
85–95°. Keep both claws holding and keep the motors from timing out. The
owner tapes if wanted.

## If something goes wrong

- **A short rises above 110° or drops below 80° (steps 4 to 9):** stop and
  re-press it.
- **The near flap creeps back toward upright and touches the right forearm:**
  push it back out to −15°.
- **A pre-folded short sags past 90° by itself:** that is fine. Don't push it
  further.
- **The camera loses a flap that is edge-on to it:** estimate its angle from
  the claw position and the hinge line. Take only a few steps that way, then
  re-check with the camera.
- **Don't change the order.** Long flaps before the shorts jams.
