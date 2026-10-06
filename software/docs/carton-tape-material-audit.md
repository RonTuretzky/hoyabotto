# Passive tape component — 6 October 2026

**Neither robot variant has completed the carton.** These are isolated tape
material tests, not robot pickup, placement, four-flap closure or retention.
The tape component is not enabled in the folding scene or controller.

The existing folding trials still use an empty 272 g free carton, table friction
0.35, crease stiffness 0.018 Nm/rad, crease friction 0.004 Nm and damping
0.008 Nm s/rad. None of these unmeasured physical assumptions was weakened.

## What the component represents

`carton/folding_tape.py` creates a free 80 × 24 mm strip with a 100 µm backing
and a separate 20 µm adhesive layer. Sixteen passive segments have bending and
twist stiffness; the total mass is 0.1728 g. There is no pin, weld, equality
constraint or tape actuator. Geometry, elastic modulus, density and adhesion
are assumptions, not measurements of the user's masking tape.

The model uses [MuJoCo's passive contact adhesion](https://mujoco.readthedocs.io/en/stable/computation/#adhesion).
Its 0.020 N parameter is a tensile limit **per contact point**, not a measured
peel force for the whole strip. Contacts can separate. The 20 µm adhesive range
allows a contact to sustain tension across a finite separation. This is not
a calibrated pressure-sensitive adhesive law: there is no pressure aging,
viscoelastic adhesive, axial stretch or in-plane bending. The thin exposed
adhesive edges also participate in the geometric contact model.

`tools/diagnose_tape_adhesion.py` places the strip on a free 50 g cardboard
sample resting on a table. An explicitly logged test instrument pulls one
end upward at a target speed of 20 mm/s, through 120 mm, with force capped at
0.15 N. This instrument is external test equipment, not a robot action.
Both bodies remain free. The diagnostic stops on engine warnings, penetration
exceeding half the backing thickness, or tensile contact on its covered face.

## Tests and limitations

The first coarse trials were rejected. With zero adhesive range the strip did
not sustain a useful bond, and soft contact allowed covered-face interactions.
Adding the adhesive range without resolving the thin strip produced unstable
or excessive penetration. Merely decreasing the timestep under CG/discrete
still failed the penetration guard. These rejected cases are retained in the
evidence inventory; they are not tape successes.

Newton/implicitfast completed the 6.7-second peel experiment at three smaller
timesteps, without engine warnings or covered-face tensile contact:

| Timestep | Peak instrument force | Maximum tape/sample penetration | Last sampled contact |
|---|---:|---:|---:|
| 50 µs | 0.08224 N | 9.54 µm | 4.60 s |
| 25 µs | 0.07927 N | 6.77 µm | 4.55 s |
| 10 µs | 0.08221 N | 2.71 µm | 3.75 s |

All three end fully separated from the sample. Peak load is similar, but the
contact-release time changes substantially at 10 µs. **Peel timing is not
numerically converged**, so these results cannot certify a retention duration
or a physical tape strength. The full-arm 2 ms integrator is not suitable for
this strip without further work.

The zero-adhesion control has zero tensile contact force and a 0.00189 N peak
instrument load. The full-length flipped strip does not initially adhere;
late in the pull it curls and its exposed adhesive contacts the sample again
(first sampled tensile force at 4.20 s). The covered-face tension guard remains
zero. This is not a permanently nonadhesive strip and must not be summarized
as having zero adhesion throughout.

Five short-patch physics tests separately establish that the adhesive face
holds a small load, the backing and zero-adhesion controls lift off, and an
overload breaks the bond at both 50 and 25 µs. The sample stays free, and there
are no actuators or equality constraints. All 97 folding tests pass. These are
component checks; robot application and carton retention remain untested.

## Reproduction

From `software`, with a MuJoCo build supporting `geom.adhesion` (tested: 3.14.0):

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-coupon
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-zero --adhesion 0
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-flipped --flipped
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-finer --dt .00001 --no-video
PYTHONPATH=. .venv/bin/python -m pytest tests/test_folding_tape.py -q
```

[Case inventory, source hashes and local results](evidence/carton-tape-material-audit.json).
No robot, camera or dispenser hardware was used.
