# Finite-extension tape component

The separate 180 × 24 mm tape model now has finite passive axial compliance.
It is a material component, not a successful adhesive bond, tape-placement
policy, retained short fold or complete carton. The existing 80 mm coupon and
all original folding gates remain unchanged.

`carton/folding_extensible_tape.py` reuses the original tape builder with 36
5 mm rigid segments. Its 35 passive axial slides have total compliance
`L/(E*A)`: the assumed 200 MPa, 100 µm backing gives `EA = 480 N` and each
slide stiffness is 93,333.33 N/m. Terminal half-cell compliance is explicitly
lumped into the interfaces. There is no actuator, pinned root, equality or
prescribed extension. The rigid geometry does not stretch; small interface
gaps approximate the backing extension. Adhesive axial stiffness, plasticity
and measured material strength are absent.

Four isolated free-strip mechanical runs use the established discrete/Newton
coupon solver at 25 and 10 µs. Equal and opposite 0.6 N end loads produce the
predicted 225 µm extension. A separate balanced transverse protocol applies
+0.06 N to each end and −0.06 N to each of the two middle segments. Actual
net force is zero and maximum net torque is below 1.28e−11 Nm. These are
explicit zero-gravity instrument tests with no plate, carton or contacts.

Maximum matched timestep differences are 0.1013 µm in extension and 0.2803 µm
in transverse deflection. The transverse center-of-mass drift is 0.568/0.227 µm,
and final signed instrument-work residual is −0.369/−0.0548 µJ at 25/10 µs.
The reports explicitly retain `energy_passivity_validated:false`. Close
agreement in these responses does not certify adhesive strength or generic
numerical convergence. The folding contact audit still refuses the discrete
integrator; no audit was relaxed for this component.

Evidence, raw source snapshots and full limitations are under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/extensible-tape-20261007/`.
`run-02` is the current four-run set. All 24 component/legacy tests pass,
including a separate 0.3 N load regression giving 112.5 µm extension.

Reproduce from `software` into a new output directory:

```sh
PYTHONPATH=. .venv/bin/python -W error -m tools.diagnose_extensible_tape --out /absolute/new/tape-cases
PYTHONPATH=. .venv/bin/python -m pytest -q tests/test_folding_extensible_tape.py tests/test_folding_tape.py
```

A 180 mm strip could geometrically overlap the two short flaps by about 40 mm
each across the 99 mm gap. That static observation does not demonstrate pickup,
pressure, adhesive contact, release or retention against opening loads. A strip
initialized already bonded across folded shorts cannot substitute for executing
those actions. No physical hardware or camera was accessed.
