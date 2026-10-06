# Photo review: the simulated station was too favorable

The successful two-hand fold does **not** validate the photographed station.
The old simulation put the arm bases above the tabletop, very close to the
carton. That placement was not obtained from the actual photos. The user's
distance correction changes what we can conclude from the earlier result.

## What the photographs establish

- The 5 October 19:12 JST phone side view shows the cart behind the table edge
  and a gripper extending across the separation. It does not support a mount
  directly over the carton. The saved measurement packet explicitly has no
  camera-to-arm-base transform. Its metric measurements are camera-relative.
- The newer packet, captured on 6 October at 13:32:34 JST, shows hanging arms
  in the phone view and the tagged paddle at the table edge in the OAK view.
  There is no positioned folding carton in that OAK frame. Phone timestamps
  record server receipt, not sensor capture; their pose is not calibrated.
- Neither packet provides a measured common-frame transform tying the arm
  mounts to the table edge and the folding box. The newer view cannot inherit
  registration from the older, differently arranged scene.

Source paths, timestamps and image SHA-256 hashes are in the
[audit evidence](evidence/bimanual-folding-station-audit.json). Photos remain
local; no room images were published to the repository. The available evidence
does not justify claiming a precise current gap or a photo-calibrated digital
twin. Arm mounts, cart envelope, finger attachments and camera pose still need
registration; changing one assumed distance is insufficient.

## The modeling error

The world frame places the nominal carton bottom at x=y=0 and the tabletop at
z=0. Positive y is away from the robot. For the earlier passing scene:

| Feature | y position |
| --- | ---: |
| Near table edge | -430 mm |
| Arm base line | -181.5 mm |
| Near carton wall | -141.5 mm |

Thus the bases were **248.5 mm past the table edge**, over the tabletop. They
were only **40 mm behind the carton rim**, at a height of **260 mm** above the
table. The cart body was omitted. Describing this only as a 40 mm setback hid
the distinction between base-to-table separation and carton placement.

The simulator now requires three separate dimensions: base origin height above
the tabletop, base origin line to table edge, and table edge to near carton
wall. Base spacing remains explicit, defaulting to an assumed 300 mm. The
origin is the imported SO101 `base_link`, not the shoulder joint or cart front.
Positive base-to-edge distance puts the base behind the edge. The table geometry
and assumed table tag are placed on the appropriate side of that boundary.
Changing the gap keeps the initial arm pose relative to its own base.

## Rechecked reach

The following are **hypothetical sensitivity cases**, not photo-derived
measurements, confidence bounds or physical success rates. The box starts
50 mm onto the table. Numbers are far-flap initial-contact position error from
the controller's orientation-constrained, multistart IK; its limit is 8 mm.

| Base line to table edge | Base 60 mm above table | Base 120 mm above table | Base 260 mm above table |
| --- | ---: | ---: | ---: |
| 50 mm | 4.3 mm | 2.4 mm | 0.5 mm |
| 150 mm | **34.9 mm miss** | **31.3 mm miss** | **52.1 mm miss** |
| 250 mm | **131.0 mm miss** | **128.1 mm miss** | **147.0 mm miss** |

The 50 mm cases pass eight sampled contact-point checks, which says nothing
about intervening paths, forces, camera visibility or cart collisions. Numerical
IK failure at 150 mm rejects this trajectory; it is not a proof that every
possible grasp or wrist orientation fails. At 250 mm, the far contact also lies
outside the conservative **475.6 mm** shoulder-pan-to-fingertip chain-length
bound by over 8 mm. That bound sums all downstream link offsets and ignores
joint limits, so it is deliberately generous. This contact is unreachable even
under that optimistic bound; another strategy would have to change the contact,
box pose, tool or station.

The explicit 60 mm height / 150 mm gap / 50 mm inset full RGB-D trial failed
before folding because both housing tags were not visible for initial
registration. An earlier version also lost the table tag behind the arm.
These are recorded failures, not folds. The separate reach audit needs no
camera detections and exposes the motion limitation even if visibility is fixed.

The historical reference still reproduces its four-flap pass after the geometry
refactor. It remains useful for testing contact control in that declared layout.
It is now selected explicitly with `--reference-layout`, and every result says
physical registration is unverified. The original evidence and failed attempts
are retained. The correction does not claim a newly successful physical setup.

## Reproduce

```sh
cd /path/to/xlerobot-farm/software
# Reach sensitivity only: ten layouts, no vision loop/contact dynamics.
PYTHONPATH=. python tools/audit_folding_reach.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/new-reach-audit

# Explicit hypothetical station; this is a recorded failure case.
PYTHONPATH=. python tools/simulate_bimanual_folding.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/path/to/new-separated-trial \
  --base-height .06 --base-to-table-edge .15 --box-from-table-edge .05
```

The next physical-layout input is a registered arm-base/table/box relationship:
both base origins and orientations, table plane and edge, carton pose and
fingertip geometry. Calibrated imagery can supply this; ruler measurements can
independently check it. Until then, the correct deliverable is an uncertainty
study and a clearly hypothetical control baseline, not a physical fold claim.
