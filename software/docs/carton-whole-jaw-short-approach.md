# Whole-jaw normal approach for the short-flap probe

This explicit offline variant tests a different contact posture after both
partial major flaps have been released near 40° and 35°. It is not a complete
folding controller or a physical robot calibration. The bounded short-fold
target remains at most +10°; 0° means upright and 90° means folded flat.

Use `--short-approach-policy whole_jaw_normal_v1` with
`--short-contact-policy setpoint_feedback_v3`. The default remains
`elevated_v2`, and neither option changes the carton, resistance, actuator
limits, joint limits, camera mounts or clearance gates.

## Why change the approach

The preceding V3 batch removed repeated servo-offset accumulation but all
three next proposed motions still intersected the left short's distal edge.
An independent check against every triangle of the original SO101 STL
reproduced the exact refusals. Original-surface directional overlap depths
were 1.021364, 1.062754 and 1.021826 mm. Corresponding collision-hull depths
were 1.020547, 1.062229 and 1.021005 mm. The source surface was slightly deeper,
not a millimetre clear of the hull. These are finite-edge directional depths,
not general nonconvex minimum translation distances or executed contact forces.

The diagnostic is saved under
`output/bimanual-fold-sim/cad-hull-audit-20261007/` in the Hackatuson workspace.
It checks compiler transforms, exact rejected poses and original triangle
normals, with four analytic tests. No collision geometry or tolerance changes
follow from it; the physical manufactured gripper and inserts remain unmeasured.

## Contact surfaces and joint path

The left hand uses the existing **proximal fixed-jaw part 37**, centrally along
the short flap. The right target is an existing distal fixed-jaw vertex at
−100 mm along the flap; other permitted moving-jaw surfaces can make first
contact. Both target the 140 mm flap radius. Normal offsets are 1.5 mm left
and 3.5 mm right. This is a bare-gripper approach, not fingertip-only contact.
The old distal-only vertex selector is unchanged. The new selector explicitly
checks each declared point against a named permitted original collision mesh
attached to the declared rigid body. It cannot treat a moving-jaw vertex as a
point rigidly attached to the fixed gripper. Independent nearest-triangle checks
put the selected left/right points within 0.916/0.811 µm of the original STL;
that establishes local CAD fidelity, not physical insert dimensions.

Point-only IK can reach the same target with a different wrist posture.
The new method first solves an outward path from the contact endpoint to a
50 mm standoff, then reverses those **exact joint waypoints**. It subdivides
their joint edges until successive commanded CAD endpoints are no more than
0.5 mm apart, with the existing 1 µm numerical tolerance. It does not solve a
different inward path. This bound concerns commanded endpoints; it is not a
continuous or actual-motion bound.

The static search checked all 24 paired combinations across three recorded
entry states, including the complete free transit, paired normal approach,
and a first +0.25° goal with fixed panels. All passed the original geometry
gates. At the preferred endpoint the right hand still had approximately
1.22/1.45 mm clearance in seeds 0/2 and 0.40 mm overlap in seed 1. Static
clearance therefore does not establish actual contact or folding progress.
The candidate manifest and full mesh/source hashes are in
`output/bimanual-fold-sim/far-contact-20261007/paired-whole-jaw-candidate-manifest.json`.

At execution, each actual-to-goal path is checked again against the current
responding plant. Every move keeps the original runtime collision, load,
joint-tracking and CAD-tracking checks, followed by a fresh source-bound
four-flap observation and carton-drift checks. The finite normal approach
records its original source sequence separately from each current observation.
A fresh endpoint more than 35 mm from the planned endpoint invalidates the
route, reusing the existing Cartesian bound. This is an additional refusal
condition, not proof that smaller shifts preserve the whole route. Original
15 mm carton-translation and 8° rotation checks remain active.
A measured inward advance of 1° ends that approach and transitions to the
existing bounded V3 stroke. The two arms need not contact simultaneously;
their actual response must be evaluated from the executed trial.

The normal approach's waypoint count is derived from 50 mm of checked travel.
It is not an enlarged retry budget for the failed vertical endpoint approach.
No flap or carton executing state is reset. No camera failure is filled from
a prior, and no preflight result guarantees the subsequent contact stroke.

## Full-prefix experiment

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/new/whole-jaw-normal-batch --workers 3 --seeds 0 1 2 \
  --open-short-angles -15 --near-pre-out .03 --near-hold-degrees 40 \
  --far-after-near --far-hold-degrees 35 --far-contact-profile central \
  --far-startup-lift .0005 --release-far-after --release-near-after-far \
  --probe-shorts-after-release --short-view-camera front_left_back \
  --allow-primary-carton-absence --short-contact-policy setpoint_feedback_v3 \
  --short-approach-policy whole_jaw_normal_v1
```

Each worker starts from the original open box and freezes all runtime Python
sources. The first batch leaves the new primary-open-short observer off to
isolate the geometric change. Independently score both the robot-contact and
panel-contact logs; neither score establishes table support, complete closure,
taping, hands-clear retention or robot readiness.
