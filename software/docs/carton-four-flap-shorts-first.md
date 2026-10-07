# Four-flap closure, shorts first — 7 October 2026

**Simulation only. The two bare claws close all four flaps and hold them in
24 of 30 seeds (80%).** No tape, no hands-clear retention, no hardware. Both
large flaps are still held by the jaws at the end; releasing them would let
them spring open (hinge rest angle 0°).

## Result

Batch `four-flap-close-28` (seeds 0–29), under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/shorts-first/`:

- **24/30 close and hold all four flaps.** Final independent angles: shorts
  89.7–106.9° (a closing major can press them below flat into the empty
  carton), near 87.9–90.8°, far 87.5–90.7°. Carton motion ≤ 12.8 mm and
  ≤ 2.2°. All 24 applied-contact audits score `CONTACT_ONLY_CLEAR`.
- Failures (6/30):

| seeds | stage | stop |
|---|---|---|
| 4, 13 | far pin (A) | right forearm, holding the shorts, touches the outward near flap |
| 20, 24 | final far push (E) | carton moved over 15 mm |
| 28 | final far push (E) | robot/flap penetration over 1 mm |
| 23 | near push (D) | no clear contact on the near flap |

Changes that took the rate from 1/5 to 24/30, each traced to an executed
failure: a wider open-claw span (below), sampling the open-claw support over
0.5 s (a resting panel's contact chatters, so one instant could read no
load), a left-minor pinch tip 4.5 mm off the panel midplane instead of 2 mm
(the fixed jaw otherwise landed on the short's edge at 120 mm base height),
regrips and hooks on the far edge, closing the right claw in the air, near
contacts in the gap between the shorts, and wider pre-contact searches.
Holding the near flap at −17° instead of −15° drops the rate to 9/20.

## Keep holding the far flap (default since this revision)

A real crease resists more than the assumed hinge, so the far flap cannot be
let go while the right arm closes the near flap (owner's review). The left jaw
now keeps the far flap from the pin to the end: after the right claw leaves
the shorts it pushes the far flap on to 70°, where its forearm clears the near
flap's sweep, holds it there while the right jaw closes the near flap, then
finishes the far flap with the same contact. `--release-far` keeps the older
release-and-recontact sequence for comparison.

| crease stiffness | release far (old) | keep holding (default) |
|---|---|---|
| 0.018 N·m/rad (assumed) | 24/30 (`four-flap-close-28`) | 17/30 (`four-flap-hold-01`) |
| 0.036 N·m/rad (2×) | **0/20** (`stiff2x-release`): in 17 seeds the shorts spring to 50–60° as the left lets go | **12/20** (`stiff2x-hold`) |

The released sequence only worked because the assumed crease friction held a
free far flap at 33°. Keeping hold is the sequence to develop. Its failures at
2× stiffness: the outward near flap creeps back into the right forearm holding
the shorts (3; a stiffer crease creeps toward about 6° from upright instead of
13°), the held far flap slips or leaves the camera's view before 70° (3), and
the first far-edge contact (2).

## Sequence

`--fold-right --press-left --open-claw-transfer --close-majors-after-open-claw`
(module `carton/folding_majors_over_shorts.py`):

0. **Open-claw short hold (existing, widened).** The right claw now spans the
   shorts at ±60 mm (10.5 mm overlap per short) instead of ±52 mm (2.5 mm),
   falling back to ±56/±52 mm and 115/117 mm support heights only if a fully
   planned sweep is refused (nothing moves before a plan succeeds). With
   2.5 mm overlap a few millimetres of carton motion during the far drag
   dropped a short; this was the largest single cause of failures.
1. **A. Pin the far major at 34° (left arm).** The far flap is nearly upright
   and only its top edge is reachable from the near side. The jaw tip presses
   on that edge, located from the far flap's own depth pixels
   (`free_edge_radius_mm`, `midplane_offset_mm`; registration error cancels),
   biased 2 mm outward, and drags it inward. When the drag slips it lifts,
   re-measures and regrips (up to 4 contacts). Past 4° the tip can also hook
   1–2.5 mm behind the outer face, which pushes it instead of relying on
   friction at the edge's inner corner. A hook with a full re-approach is the
   last resort. 34° keeps the shorts near 88°; at 17° they rose to 86° and the
   closing near flap jammed on their corners.
2. **B. Right claw lets go** of the shorts, closes, and parks. The far major
   alone keeps the shorts down. The claw is closed in the air: left open, its
   moving jaw later stood in the closing far flap's sweep.
3. **C. Left claw lets go** of the far major and parks (its forearm otherwise
   lies in the near flap's sweep).
4. **D. Right jaw closes the near major** to 88°, pushing its outer face 25 mm
   below the tip at x = +0.04 m: right of the centre line, but in the 99 mm
   gap between the folded shorts.
5. **E. Left jaw closes the far major** to 88°, starting at the tip (the only
   reachable point on a far face at 34°) and moving down the panel at most
   5 mm per command as it turns toward the robot.

Targets stop at 88° (inside the original 85–95° closure band): pushing to 90°
pressed the shorts below flat into the empty carton.

## Controller rules added

- **Push pre-check excludes only the pushed panel** (it moves with the push);
  all other obstacles keep 6 mm clearance, and the runtime 1 mm robot/flap
  penetration stop still applies to the pushed panel.
- **Bounded contact-kinematics angle** across a camera's edge-on band (near
  about 20–25°, far about 38–46° for the station camera): only after
  agreement with vision within 3°, at most 12 commands, 15 mm / 8° carton
  limits on every fresh registration, reacquisition within 4°.
- **Recontact.** Stalls and pre-execution refusals retreat along the executed
  approach and search a new contact (at most 3). Penetration, forbidden or
  loaded non-jaw contact, carton motion and tracking stops are never retried.
- **Shorts bands.** During motion the shorts must stay within 80–110° (closing
  majors can press them a little below flat in the empty carton); the final
  hold requires shorts 85–110° and both majors 85–95°.

## Station assumptions this needs (none measured)

| assumption | value | why |
|---|---|---|
| arm-base origin height above tabletop | **120 mm** (was 60) | at 60 mm no far-edge contact keeps 8 mm clearance from the outward near flap; 100 mm puts the right forearm against the near flap (0/13) |
| arm-base line to table edge | 150 mm (unchanged) | 130 mm breaks an earlier right-arm reach; 140 mm stalls the right short fold |
| far flap as presented | **about +1° inward** (was −5.7°) | the left arm reaches the far top edge from about +1°; more than ~1.6° inward intersects the upright shorts |
| extra printed carton markers | **IDs 26, 27 (near wall, x = ±120 mm), 28 (left wall, y = −80 mm)**, 45 mm | one reaching arm hid the only visible near- or left-wall marker and registration failed |
| open-claw support height | 113 mm (115/117 fallback) | 111 mm put a moving jaw 1.03 mm into the right short at 120 mm base height |

These are requirements for the real station if this sequence is used, and
they must be measured there, not assumed.

## Reproduce

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot --out /absolute/new/batch \
  --workers 4 --seeds 0 1 2 3 4 5 6 7 8 9 --near-targets -15 --support-heights .113 \
  --close-majors-after-open-claw --majors-far-target 34 --base-height .12 \
  --far-open-degrees 1.0 --extra-wall-markers --pinch-clearance -.0045
```

About 70 s per trial; score contacts with `tools/score_folding_contacts.py --run <trial>/run`.

## Still open

- Remaining failures above (forearm/near-flap clearance during the far pin, carton motion in the final far push).
- Taping and hands-clear retention: the majors spring open when released.
- Physical validation: hinge stiffness, friction, cardboard thickness, camera
  noise and the station dimensions are all assumptions.
