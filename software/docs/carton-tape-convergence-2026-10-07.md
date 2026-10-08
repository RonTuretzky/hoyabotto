# Passive tape coupon convergence — 7 October 2026

The **25 µs discrete/Newton coupon with exact constraint-inertia diagonals**
agrees with the 10 µs reference within the audit tolerances below. This resolves
the earlier coupon timing discrepancy for this numerical configuration.
It does **not** establish robot tape pickup, placement, full-robot performance,
or the ability to retain a carton crease torque of approximately 0.028 Nm.

The free 80 × 24 mm, 16-segment strip and free 50 g cardboard sample retain
their previous geometry, mass, bending/twist stiffness, damping, friction,
20 µm adhesive range and 0.020 N adhesion per contact. No adhesion parameter
was tuned for carton closure. Every case has zero actuators and zero equality
constraints. The only applied load comes from the explicitly reported coupon
instrument. No hardware or complete folding trial was run in this audit.

## Numerical change and evidence

MuJoCo's `discrete` integrator includes position stiffness in its implicit
step, including passive joint stiffness. The `diagexact` flag replaces the
body-based constraint-inertia approximation with its exact diagonal, which
is relevant to the coupled, anisotropic segment chain. These are numerical
choices; they do not alter the declared tape material. See the primary
[integrator documentation](https://mujoco.readthedocs.io/en/latest/computation/index.html#numerical-integration)
and [exact-diagonal documentation](https://mujoco.readthedocs.io/en/latest/XMLreference.html#option-flag-diagexact).

The native finite-tension contact model remains unchanged. Its force limit is
per contact point, not measured peel strength or a bond-area law. A physical
tape calibration still needs measured peel forces, application pressure,
cardboard surface condition and relevant loading direction. See
[native adhesion](https://mujoco.readthedocs.io/en/latest/computation/index.html#adhesion).

The original 6.7 s peel protocol settles for 0.2 s, then raises a compliant
instrument target at 20 mm/s through 120 mm, with force capped at 0.15 N.
The diagnostic now records complete adhesive-contact loss and recontact at
every solver step. A reported sustained separation lasts at least 50 ms.
The contact timestamp is the beginning of its solver step, with resolution
limited by that timestep. Source files are copied and hashed at run start.

| Exact discrete/Newton timestep | Peak load | First sustained separation | Max tape/sample penetration | Total instrument work | Audit result |
|---|---:|---:|---:|---:|---|
| 10 µs | 0.08220918 N | 4.582790 s | 0.0858 µm | 0.75300 mJ | Reference |
| 25 µs | 0.08221044 N | 4.582825 s | 0.0811 µm | 0.75417 mJ | Pass |
| 50 µs | 0.08222333 N | 4.582650 s | 0.1453 µm | 0.70023 mJ | Total work differs by 7.0% |
| 100 µs | 0.08222733 N | 4.023100 s | 4.8795 µm | 0.71530 mJ | Premature separation, then recontact |
| 250 µs | 0.08225358 N | 4.018000 s | 24.6544 µm | 0.72148 mJ | Premature separation, then recontact |
| 500 µs | 0.04032783 N | — | 94.9085 µm | 0.41156 mJ | Stops on penetration guard at 4.0135 s |

At 100 and 250 µs, final contact loss still occurs near 4.583 s. Peak load
and the last sampled contact alone would therefore conceal the earlier
detachment. The 50 µs run agrees on peel timing and peak load but differs in
total work, which includes the free strip's later motion. It is not the
recommended reference configuration.

For 25 versus 10 µs, separation differs by 35 µs, peak force by 0.00154%, and
total work by 0.156%. On the common 0.2–4.5 s interval sampled every 50 ms,
instrument-force RMS difference is 0.247 mN; tape-end position RMS and maximum
differences are 0.0289 and 0.1895 mm. The audit tolerances are 1 ms separation,
1% peak-force and total-work differences, 1 mN force RMS, 0.05 mm position
RMS and 0.5 mm maximum position difference. Both runs complete without engine
warnings or covered-face tension and remain below the unchanged 50 µm
penetration guard. This is agreement for one component protocol and the
listed observables, not proof for arbitrary loading or a different timestep.

Without `diagexact`, discrete/Newton also improves the old implicitfast result:
50/25/10 µs last sampled contact is 4.60/4.55/4.55 s. The earlier implicitfast
sequence was 4.60/4.55/3.75 s. Those baseline cases remain in the inventory;
the finer exact-diagonal comparison is the recommendation.

## Controls and remaining work

Matched exact-diagonal 25 µs controls use the same free sample and strip:

- A 10 mN constant upward coupon load after settling retains all 64 adhesive
  contacts through two seconds of applied load, without a pin or constraint.
- Zero adhesion produces zero tensile contact force and separates.
- A flipped strip does not initially bond, but can curl and present exposed
  adhesive later. The covered-face tension guard stays zero; the experiment
  is not a permanently nonadhesive control.

Fourteen tape tests pass, including finite-load support, zero-adhesion and
backing-face lift-off, overload separation with a free sample, exact-diagonal
checks at three timesteps, and contact-timeline chatter/recontact handling.

The coupon does not bridge independently moving carton flaps. Its 10 mN hold
does not certify retention against a 0.028 Nm crease torque. Robot application,
actual tape strength and a suitable full-scene integration method remain
unverified. The ordinary 2 ms folding configuration is not validated for this
strip. `TapeSpec` therefore still declines a generic folding/retention claim.

## Reproduction and artifacts

From `software`, using the existing MuJoCo 3.14.0 environment:

```sh
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-coupon-25us --dt .000025 --integrator discrete --solver Newton --exact-impedance --no-video
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-reference-10us --dt .000010 --integrator discrete --solver Newton --exact-impedance --no-video
PYTHONPATH=. .venv/bin/python tools/diagnose_tape_adhesion.py --out /path/to/new-hold-25us --dt .000025 --integrator discrete --solver Newton --exact-impedance --mode hold --hold-force .01 --seconds 2.2 --no-video
PYTHONPATH=. .venv/bin/python -m pytest tests/test_folding_tape.py -q
```

[Audit inventory, numerical comparisons and source hashes](evidence/carton-tape-convergence-2026-10-07.json)
cover all 15 new cases. Raw scenes, results, initial source snapshots and the
JUnit report are under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/tape-convergence-2026-10-07/`.
All recorded source snapshots match their hashes. The
[6 October material audit](carton-tape-material-audit.md) and its rejected
cases remain preserved.
