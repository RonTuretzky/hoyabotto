# Handoff to the robot agent: how to fold the carton's four flaps with the two bare claws

For the agent controlling the robot through the robot API, with the owner at
the STOP button. Everything here comes from the MuJoCo simulation (about 300
simulated trials, branch `RonTuretzky/carton-handoff-pick-up`). **Nothing has
been done on the real robot yet.** Treat the numbers as a starting point, and
correct them with what the cameras and `robot_get_arm_pose` show.

Result in simulation: all four flaps closed and **held by the two claws** in
20 of 30 trials, and 10 of 20 with creases twice as stiff. Taping is not
solved: at the end both claws are still holding the flaps, and letting go
lets them spring open.

## Rules

- Before every motion, tell the owner in one sentence what moves and where,
  then wait for "go". `robot_halt_motion` stops and holds. `robot_stop`
  releases every motor, so the flaps spring open and a held flap is lost.
- Move slowly near the carton: `duration_s` of at least 2 s, contact steps of
  at most 5 mm or 2–3° of flap rotation, and check the cameras after each step.
- Never relax limits. Stop and report if the carton slides more than 15 mm or
  turns more than about 3°, if any part other than a jaw is pushing cardboard,
  or if an arm touches the table or a carton wall.
- Owner behaviour that matters here:
  - Motors are released if no command arrives for about 120 s. While holding
    flaps, keep commanding: re-sending `robot_set_motor_enable` with
    `enabled: true` for motors that are already enabled renews the timer
    without moving them.
  - A joint with load at 600 or more that lags 20 or more ticks behind its
    command ends the move `contact_halt`. It stops pushing and holds; that is
    not a failure.
  - A holding joint more than 96 ticks off its goal faults and releases
    everything. Press with small overdrive, a few millimetres past contact,
    never a large one.
  - The heartbeat renewal is refused when a gripper's load is over 500 or an
    arm joint's is over 800. Then send a real move of a few ticks: a move to
    the current targets is a no-op and may not count.
- Joint ticks to angles: the only mapping is the unvalidated LeRobot midpoint
  candidate (0° at the middle of each saved range). Check it before relying
  on it in contact.

## The carton and the frame used below

- Carton 379 × 283 × 108 mm, 140 mm flaps, 3 mm cardboard, 272 g empty.
- Carton frame:
  - origin at the centre of the carton's bottom on the tabletop;
  - **+x toward the robot's right**, **+y away from the robot**, +z up;
  - the near wall is at y = −141.5 mm and the far wall at y = +141.5 mm.
- Hinges:
  - short (end) flaps at x = ±189.5 mm, z = 108;
  - long flaps at y = ±141.5 mm, z = 111.5 (the long flaps fold on top of
    the shorts).
- Flap angles are measured from upright: 0° = vertical, 90° = closed (flat),
  negative = leaning outward.
- With the shorts flat, their free edges sit at x = ±49.5 mm, leaving a
  99 mm gap at the centre.
- The simulated station:
  - arm bases at (−150, −301.5, 120) and (+150, −301.5, 120) mm, so the base
    origins are 120 mm above the tabletop and 150 mm behind the table's near
    edge;
  - carton near wall 10 mm in from the table edge, square to it.

  Arm-base frame (x forward, y left, z up) from carton-frame point p, for a
  base at b: x_b = p_y − b_y, y_b = −(p_x − b_x), z_b = p_z − b_z. **The real
  station is different** (8 October camera check: bases well above a small
  table). Measure the real carton pose in each arm's frame first. The touch
  procedure in `docs/carton-crease-self-measurement-handoff.md`, step 1, does
  this. Then convert the points below with the measured pose.

## Sequence

"Closed claw" means the gripper is fully closed; its tip pushes like a
finger. The left claw does the left short and the far flap. The right claw
does the near flap, the right short and the two-short hold.

### 0. Open the near flap a little outward (right claw)

The near long flap must lean outward, out of the way, while the shorts are
done. Close the right claw in the air. Go above the inside face of the near
flap (about (6, −108, 257)), lower to about z 254, then push outward toward
y −155 at about z 223 until the flap is at about **−15°**. Lift straight up
(to about z 317) and park the right arm away.

- Keep the near flap at −15°. At −17° the success rate fell from 20/30 to
  9/20. At −13° it creeps back into the right forearm later.
- If the far flap leans outward, set it to about **+1° inward** (nearly
  upright) by hand before starting. The left claw can only reach its top edge
  from about +1°, and above about +1.6° it hits the upright shorts.

### 1. Left claw braces the left short (pinch)

Open the left claw to about 34° (0.6 rad). Lower it over the left short near
its near end, around (−173, −100), from z 255 to z 220, so the jaws straddle
the short. Close the claw on it (pinch). The left short tilts to about 9° and
the carton is braced.

### 2. Right claw folds the right short to flat

Right claw closed:
1. Approach the right short's **outer face** from outside, near its top, at
   about (238, −108, 245). Make contact at about x 234.
2. Push it inward along an arc about its hinge (189.5, z 108), about 2° of
   flap rotation per step. The tip path in simulation: (234, 245) → (162, 240)
   → (106, 212) → (90, 178) → (88, 150) → (94, 117) (x, z in mm, y ≈ −110).
3. Stop at about **90°**. Hold it there; the right claw keeps holding it
   during step 3.

### 3. Left claw folds the left short to flat

1. Open the pinch and lift the left claw out (to about (−213, −101, 260)).
   The left short springs back to about −9°.
2. Close the claw in the air.
3. Go outside the left short to about (−255, −101, 216), then push its outer
   face inward along the mirror arc to about **90°**. In simulation it ends
   at (−93, −99, 114).

Both claws now hold one short each, flat.

### 4. Right claw takes both shorts (open-claw hold)

1. Turn the right claw (wrist roll) while it still holds the right short.
2. Open it gradually (to about 77°, 1.35 rad) while sliding it across the gap
   toward the left. It ends at about (−49, −42, 113), jaws open across the
   gap, **one jaw resting on each short**.
3. Let the left claw take the load off slowly: lift it in 4 steps of about
   1.5 mm, then fully. In simulation the left short stays at 91° for 5 s.

Details that mattered:
- **Span: the jaws must overlap each short by about 10 mm**, about ±60 mm
  across the gap. With 2.5 mm overlap, a few millimetres of carton motion in
  the next step dropped a short. That was the single largest cause of
  failures.
- Jaw underside about 113 mm above the table, i.e. about 5 mm above the rim.
  At 111 mm a jaw dug into the right short.
- Judge whether the claw is really carrying the shorts over about half a
  second, not one reading: a resting contact chatters.

### 5. Left claw pins the far flap at 34° (from its top edge)

The far flap is nearly upright, and from the robot's side only its **top
edge** is reachable.

1. Close the left claw. Go above the far flap's top edge, around
   (−156, 146, 283).
2. Lower the tip onto the edge (about z 263), 2 mm on the outer side of the
   edge's centre line. Use the flap's own depth pixels to find the edge,
   since registration error is comparable to the 3 mm edge. Press it 0–3 mm.
3. **Drag it inward toward the robot.**
4. If the drag slips: lift, re-measure the edge, re-grip (up to 4 tries).
   Past about 4°, put the tip 1–2.5 mm behind the outer face (a "hook") so it
   pushes the panel instead of relying on friction.
5. Stop at **34°**. In simulation the tip ends at about (−156, 76, 234).

34° matters: the far flap now rests on the shorts and keeps them at about
88°. At 17° the shorts rose to 86° and the closing near flap later jammed on
their corners.

The **main remaining failure** in simulation is this drag pushing more than
1 mm into the cardboard (5 of 30). Go slowly and watch the edge.

### 6. Right claw lets go of the shorts

1. Lift the right claw 15 mm straight up.
2. **Close it in the air above the shorts.** Left open, its moving jaw stood
   in the far flap's later sweep.
3. Park it away (about (200, −180, 300)).

The held far flap alone keeps the shorts down (about 88°).

### 7. Left claw keeps the far flap and takes it to 70°

**Do not let go of the far flap at any point from here to the end.** With
real crease resistance it springs back and the shorts pop up. The let-go
version closed 0 of 20 with stiffer creases; keeping hold closed 10 of 20.

Push it on to **70°** (tip about (−157, 27, 166)). At 70° the left forearm is
out of the near flap's path. Choose the far-edge grip spot so the left
forearm stays clear of where the near flap will swing. If the held contact
slips, re-grip.

### 8. Right claw closes the near flap to 88°

Right claw closed:
1. Go outside the near flap, around (−39, −184, 267), then to its **outer
   face**, about **25 mm below its free edge** (115 mm from the hinge), at
   **x ≈ +25 to +40 mm**. That is right of centre, and it falls in the 99 mm
   gap between the folded shorts, so the claw does not land on them.
2. Push it closed along the arc about the near hinge to **88°**. In
   simulation it ends at about (31, −30, 121).
3. Hold it there.

If there is no clear contact, back out along the same approach and try a
nearby spot (up to 3 tries). Do not retry after an unexpected contact,
carton motion or excess load.

### 9. Left claw closes the far flap to 88°

From the 70° hold, keep pushing with the same contact. As the flap turns
toward the robot, move the contact down the panel by at most 5 mm per
command. Stop at **88°** (tip about (−157, 34, 129)).

### 10. Done: both claws hold

Pass: shorts 85–110° (a closing long flap can press them a little below flat
into the empty carton), long flaps 85–95°. Stop at 88°, not 90°: pushing to
90° pressed the shorts down into the carton.

Keep holding, and keep the 120 s heartbeat alive. Tape is not part of this.
If the owner wants to tape, they tape the seam while the claws hold.

## Watch-outs from the simulation

- **Bands while moving:** shorts must stay at 80–110° from step 4 on.
- **Camera blind spots.** A flap seen edge-on by the camera has no reliable
  angle: the near flap at about 20–25°, the far flap at about 38–46° (for the
  simulated overhead camera; the real ones differ). Across that band:
  - estimate the angle from the claw position (`robot_get_arm_pose`) and the
    hinge line;
  - do at most about 12 commands that way;
  - re-check with the camera as soon as the flap is visible again.
- **The near flap creeps back.** With stiffer creases the outward near flap
  creeps from −15° toward −6° and can touch the right forearm while it holds
  the shorts (steps 4–5). If it does, push it back out first.
- **Pre-folded (soft) creases:** an empty pre-folded carton's shorts will
  probably sag past flat (about 101°) when released, because the flap's
  weight beats the crease. That is fine for the result (85–110° passes);
  just don't push a short further once it is past 90°.
- **Carton slides:** a hard push on the far or near flap can slide the empty
  carton. Push in small steps. If it moves more than about 15 mm, stop. Having
  a person hold the carton, or putting the contents in, helps.
- **Do not reorder.** Long flaps first (or the near flap before the far flap)
  failed in simulation: rigid shorts collide with half-closed long flaps
  during their middle rotation and stall at about +1°.

## What is not known

Crease stiffness and friction, the real station geometry, camera poses, jaw
friction, and servo compliance under load were all assumed. Measuring them
first (`docs/carton-crease-self-measurement-handoff.md`) and reporting back
lets the simulation be rebuilt to match. More detail:
`docs/carton-four-flap-shorts-first.md` and
`docs/carton-real-station-measurements.md`.
