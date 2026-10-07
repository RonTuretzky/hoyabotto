# Near-major transfer experiments — 7 October 2026

Full four-flap closure is still incomplete. Physically opening the short flaps
outward now permits the near major to fold and remain held around 92°. Two
seed-0 trials with different approach offsets passed that partial stage; a
subsequent fixed 30 mm approach reproduced the near hold across seeds 0–2. The
far major, closing the shorts, tape application and hands-clear retention remain
unverified. The improved camera observer also fixes a demonstrated angle error.
All results here use the original SO101 CAD, empty 272 g free carton, resisting
hinges and unchanged robot/table placement, actuator limits and collision gates.
No motor or camera device was accessed.

## Observation correction

`folding_hinge_vision.depth_major_flap_angles` fits an unconstrained cardboard
plane and requires alignment with the measured hinge, distributed area, and
sufficient visible extent. A nearly horizontal major also needs visible pixels
in the gap between the short flaps, so a closed short cannot masquerade as the
major. Missing or ambiguous planes remain unknown. PixelPort removes the old
major estimates before applying this observer; independently decoded flap tags
can still provide identity. Priors never substitute for visible pixels.

Two recorded trajectories were rerendered with fresh AprilTag registration,
0.8 mm depth noise and 25% dropout. Across 23 sampled frames the new observer
returned all 46 major angles; maximum absolute errors were 0.372° in the
occlusion trajectory and 0.324° in the historical closed-flap trajectory. The
historical loaded/weak-crease run is **only a perception fixture**, not a valid
free resistant-carton folding success. Wrong surfaces, competing planes and
hidden majors are covered by negative tests. Real camera images remain untested.

The controller now checks the withdrawal threshold during every contact
approach observation, and regenerates the press sweep from the newly measured
angle. The old implementation checked too late and then commanded a stale angle.

## Physical results

| Experiment | Result |
| --- | --- |
| Mechanics-only angle oracle, release at −10° | Withdrawal now triggered before the forearm jam; a 100 mm lift ended only 1.86 mm from the flap, below the unchanged 6 mm transit clearance. Park refused. |
| Same oracle, release at −5° or 0° | Right support was lost before either threshold. Neither timing passed. |
| New RGB-D observer, 140 mm lift, −10° release, seeds 0–2 | All three short-hold prefixes passed. Withdrawal completed; short flaps reopened to about 53°. The near flap reached only about 2°, then the box slid beyond the 15 mm stop bound. No major fold passed. |
| Near-major-first from the initially open carton, contact x=0 or −80 mm | Same corner jam near 1° and about 20 mm maximum horizontal displacement before stop. |
| Near-major-first, contact x=−140 mm | Approach IK error 22.68 mm; the unchanged 8 mm gate refused it. |

An independent reconstruction of the first major-first run's final geometry
found four near/short corner contacts. The executed trajectory records the
carton sliding while the near angle stalls. Force values recomputed from
position-only replay are not actual executed contact-force measurements and
are not used here. Simply changing the order without clearing the
inward-leaning short flaps did not resolve the geometry. The outward-opening experiments below resolve that initial jam; no artificial initial angle or
post-initialization state reset is counted as preparation.

The three normal-vision trials completed in 50.59 wall seconds. Summed worker
time divided by wall time was 2.97, confirming overlap, not establishing a
controlled serial speedup. These are parallel controller experiments, not
neural-network weight training.

## Physical outward preparation and near-major hold

Rotating the original pinched short outward hit the unchanged 8 mm IK gate.
The revised preparation physically releases that pinch, clears the left claw,
and pushes each short outward from its inside face. Fresh images drive progress
and verify the released angles. A −20° target lost left visibility around −15.6°
and stopped. Targets −10° and −15° were physically opened and released.

With a −15° target, the released shorts settle around −13° and −14.8°. The near
major's 20/25 mm outside approach failed the unchanged 6 mm clearance check;
30/40 mm approaches cleared it and then physically folded the near flap:

| Near approach outward offset | Final near angle | Max horizontal box movement | Result |
| --- | ---: | ---: | --- |
| 30 mm | 91.86° | 0.0202 mm | Two-second held near closure; other three flaps open |
| 40 mm | 92.05° | 0.0202 mm | Two-second held near closure; other three flaps open |

Both use seed 0, so they are **not** two independent noise trials of one frozen
policy. No forbidden penetration occurred; maximum robot/flap penetration was
0.125 mm, below the original 1 mm gate. The 30 mm run executes the full 71.062 s
sequence from the initial open carton without a state reset. Its 357-frame
review GIF covers all recorded motion at 1×, plus a labeled 2.5 s final display
pause. Neither held closure nor preparation counts as hands-clear retention.

Reproduce the partial near hold through the bounded parallel runner:

```sh
PYTHONPATH=. .venv/bin/python tools/run_claw_sweep.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --out /absolute/new/major-first-runs --workers 3 --seeds 0 1 2 \
  --near-targets -15 --open-short-angles -15 --near-pre-out .03 --video
```

The optional `--far-after-near` adds the experimental next stage; it is not a
validated full folding command and does not connect to hardware.

## Follow-up: far major and the remaining layer-order problem

The same 30 mm near approach completed across camera-noise seeds 0–2 before
attempting the far stage: near angles 91.86°, 91.22° and 90.67°, with maximum
horizontal carton movement 0.0202, 0.0181 and 0.0176 mm. This is a three-seed
partial near-hold result, not full closure or retention after release.

An initial far-stage attempt refused a left-arm soft-limit overshoot of
0.0000425 rad. The revised approach commands a bounded interior settle through
the actuators; it never resets joint state. Static CAD path checks found a far
fixed-finger route with a lateral edge approach, but the full dynamic trials
still failed:

| Seed | Far-stage result |
| --- | --- |
| 0 | Intended far contact would penetrate 1.431 mm, above the original 1 mm gate; approach refused |
| 1 | Far reached 13.65°; all usable carton-marker observations were occluded and the run stopped |
| 2 | Far reached 19.61°; carton movement exceeded 15 mm and the run stopped |

A separate geometry sweep also shows that closing both majors first is not a
complete solution: sweeping a short underneath the closed rigid majors creates
up to 6.28 mm panel intersection. That is a static layer-order diagnostic, not a
simulated material-flexing result. The next policy search therefore considers
far-first reach and intermediate major positions rather than assuming the short
flaps can pass through already closed panels. Pushing the near major much
farther outward also collides with the robot shoulder/base in the unchanged
rear-cart placement; those geometries are rejected rather than moved away.

## Faster parallel search with complete replay

Workers freeze their Python sources and write separate trial directories.
Omitting `--video` now retains all timestamped motion states without rendering
presentation images. The trace-only verification reproduced all 563 timestamps
and joint/object states exactly from the seed-0 near-hold run. It finished in
20.57 wall seconds; this is a measured run time, not a controlled speedup claim.
The full 71.062-second physical sequence remains available for later rendering.
The equivalence record is in
[evidence/carton-trace-only-equivalence-20261007.json](evidence/carton-trace-only-equivalence-20261007.json).
Use `--video` only when immediate presentation images are needed. Perception
still renders its RGB-D inputs in every ordinary search run.

## Evidence and boundaries

[Trial summaries and result hashes](evidence/carton-near-transfer-20261007.json)
retain all reported failures. Full local runs are under
`output/bimanual-fold-sim/claws-retention/parallel-near-vision-01` and
`output/bimanual-fold-sim/major-first` in the Hackatuson workspace. The first
major-first attempt had a report-save API error; it has no valid result file
and is excluded from the experiment counts.

`--privileged-near-angle` is an explicit mechanics-only diagnostic. It records
its ground-truth override and always leaves end-to-end perception and full-task
completion false. Ordinary runs use RGB-D. Existing geometric path planning
still copies simulator obstacle state and therefore is not a hardware-ready
perception-only planner.

193 tests passed across the observer/replay, handoff sequencing and tag
commissioning suites. This is software verification, not a substitute for the
failed physical simulation results or real hardware calibration.
