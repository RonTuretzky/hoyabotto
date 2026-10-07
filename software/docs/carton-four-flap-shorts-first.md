# Four-flap closure, shorts first — 7 October 2026

**Simulation only. All four flaps have now been closed and held by the two
bare claws in two executed runs, but the sequence is not reliable yet (1 of 5
seeds with the current code). No tape, no hands-clear retention, no hardware.**

## Result

| run | final flap angles (independent) | short angles during final 2 s hold | carton motion | contact audit |
|---|---|---|---|---|
| `shorts-first/close-27-s0` (seed 0, earlier revision) | shorts 89.76 / 89.76, far 88.81, near 88.30 | 89.75–89.76 | 1.6 mm, 0.2° | `CONTACT_ONLY_CLEAR` |
| `shorts-first/four-flap-close-06/trial-003` (seed 4, current code) | shorts 89.79 / 89.79, far 89.64, near 88.18 | 89.77–89.79 | 1.1 mm, 0.0° | `CONTACT_ONLY_CLEAR` |

Paths are under `/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/`.
Both majors are pushed by jaws that stay in place; releasing them would let the
majors spring open (hinge rest angle 0°). Taping and hands-clear retention are
the next stages and are not attempted here.

The current-code sweep `four-flap-close-06` (seeds 0, 2, 3, 4, 5) passes
**1/5**; seed 1 fails earlier in the existing open-claw transfer. Failures:

| seed | stage | stop |
|---|---|---|
| 0 | near push | right-arm IK misses by 13.9 mm on the preferred right-side near contact |
| 2 | far hook | no pre-contact pose with 6 mm transit clearance |
| 3 | far hook | robot/flap penetration over 1 mm |
| 5 | far hook | carton moved over 15 mm |
| 1 | open-claw transfer (existing) | predicted left/right claw collision |

## Sequence

Run with `--fold-right --press-left --open-claw-transfer --close-majors-after-open-claw`
(the open-claw prefix is the existing 8/9 component; see
`carton-open-claw-retention.md`). Module: `carton/folding_majors_over_shorts.py`.

- **A. Pin the far major (left arm).** The far flap is nearly upright and out of
  reach behind the outward near flap: from the near side the 5-DOF arm can
  neither pinch its edge nor reach behind it (static scans: best pinch pose
  misses by 12–14 mm with a 45° jaw-axis error; hooks collide with the near
  flap). The left jaw tip therefore presses on the far **top edge** and drags it
  inward. The edge is located from the far flap's own depth pixels, not from
  carton registration (registration was 2–4 mm off and slid the tip beside the
  3 mm panel): `depth_major_flap_angles` now reports `free_edge_radius_mm`
  (99th percentile of in-plane pixels, measured in the registered frame so the
  registration offset cancels) and `midplane_offset_mm`. The tip is biased 2 mm
  toward the outer side (on the inner corner a press pushes the flap outward).
  If the friction drag slips after 4°, the jaw lifts off and hooks behind the
  edge instead. The far flap is held at 34°, which bounds the shorts' far
  corners.
- **B. Right claw lets go of the shorts** and parks. The far major alone keeps
  them closed (87–90°).
- **C. Left claw lets go of the far major** and parks. Held at 34° its forearm
  lies in the near flap's sweep; held at 60° the far panel blocks the right
  claw's release. The released far major stays near 33–34°.
- **D. Right jaw closes the near major** from −15° to 88°, pushing its outer
  face 25 mm below the tip (at the tip, 2–4 mm of registration error put the
  jaw on the edge and loaded the hinge axially: 198 N, no rotation).
- **E. Left jaw closes the far major** to 88° while the right jaw holds the near
  one. Contacts start at the tip (the only reachable point on a far face at
  34°) and move down the panel as it turns toward the robot. The right jaw
  keeps to the near flap's right side and the left to the far flap's left side:
  a right wrist near the centre line lies in the closing far flap's sweep.

Targets stop at 88°: pushing a major to 90° pressed the shorts to 98.6° into
the empty carton. 88° is inside the original 85–95° visual closure band.

## Controller rules added

- **Push pre-check excludes only the pushed panel.** A push necessarily moves
  the jaw into the panel's current pose, so the static 1 mm allowance refused
  every push deeper than 1 mm. The pushed panel is removed from a copy of the
  planning model; every other obstacle keeps 6 mm clearance and the runtime
  1 mm robot/flap penetration stop still applies to the pushed panel.
- **Bounded contact-kinematics angle.** The station camera sees the near major
  edge-on near 20–25° and the far near 38–46°. While a jaw is pushing, the flap
  angle follows from encoder FK, the CAD contact point and the fresh carton
  registration. It is used only when the visual angle is missing, after it
  agreed with vision within 3°, for at most 12 commands, with the 15 mm / 8°
  carton-drift limits; the next visual angle must agree within 4°. Each
  command records its angle source.
- **Recontact.** A stalled push, or a command whose every proposal is refused
  before execution, lifts off and searches a new contact (at most 3 times).
  Penetration, forbidden or loaded non-jaw contact, carton motion and tracking
  stops are never retried.
- **Contact search margin.** Contact poses are searched with 10, then 8, then
  6 mm clearance and 40 random IK seeds; clear poses near the far edge are rare.

## Station assumptions this needs (none measured)

| assumption | value | why |
|---|---|---|
| arm-base origin height above tabletop | **120 mm** (was 60) | at 60 mm no far-edge contact keeps 8 mm clearance from the outward near flap; at 120 mm 11–27 of 150 IK seeds do along the whole edge |
| arm-base line to table edge | 150 mm (unchanged) | 130 mm breaks an earlier right-arm reach; 140 mm stalls the right short fold |
| far flap as presented | **+1° inward** (was −5.7°) | the left arm reaches the far top edge from about +1°; more than ~1.6° inward intersects the upright shorts |
| extra printed carton markers | **IDs 26, 27 (near wall, x = ±120 mm), 28 (left wall, y = −80 mm)**, 45 mm | one reaching arm hid the only visible near- or left-wall marker and registration failed |
| open-claw support height | 113 mm (111 also declared) | 111 mm put a moving jaw 1.03 mm into the right short at 120 mm base height |

The base height, table distance and far-flap presentation are physical
requirements for the real station if this sequence is used; they must be
measured, not assumed. A second camera was tried for the far flap but its
carton registration drifted by about 8 mm when the left arm covered its
markers; the bounded contact-kinematics angle is used instead.

## Reproduce

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot --out /absolute/new/batch \
  --workers 4 --seeds 0 2 3 4 5 --near-targets -15 --support-heights .113 \
  --close-majors-after-open-claw --majors-far-target 34 --base-height .12 \
  --far-open-degrees 1.0 --extra-wall-markers
```

Score contacts with `tools/score_folding_contacts.py --run <trial>/run`.

## What is still weak

All remaining failures are in the far major. Contacts on it sit within a few
millimetres of reach and clearance limits, the friction drag slips at
different angles per seed, and hooks can pull the free carton. Ideas not yet
tried: choose contacts by margin rather than first-clear order, hold the near
flap closer to its hinge during stage E, and anchor the carton with the idle
hand during the far pin.
