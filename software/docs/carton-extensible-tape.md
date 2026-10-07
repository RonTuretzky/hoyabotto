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

## Local overlap experiment: retention not established

`tools/diagnose_extensible_tape_bond.py` lets an initially unbonded strip settle
from an 80 µm gap onto an explicitly fixed laboratory substrate. Gravity forms
contact; no executing strip pose, latch, equality or bond is prescribed. An
external instrument then ramps a force at the free tail toward 0.6 N tangential
and 0.06 N opening load. This fixture is not the free carton or robot application.

The 180 mm strip curls around the fixture's lower rim and violates the unchanged
50 µm penetration limit at both timesteps. Those invalid trajectories cannot
establish bond strength. A separate 50 mm strip retains the same 5 mm cells,
material and 40.5 mm overlap, with a short free tail to test only that local
interface. Both timesteps form contact, then lose the interface before reaching
the target, with successive empty returned-contact observations spanning 50 ms.
A zero-adhesion control also
separates. Maximum penetration in the two adhesive local cases is below 0.21 µm.

Current `run-03` evidence bounds the transition using solver-call intervals:
25 µs gives [0.231275, 0.231325] s; 10 µs gives [0.231430, 0.231450] s.
The imposed force is approximately 0.14 N tangential / 0.014 N opening there.
This is a negative outcome for the tested loading protocol, not calibrated
failure strength: slip onset, transient reactions and work do not fully converge.
No stronger tape law was substituted to make it pass.

Raw contact arrays are recorded immediately after each `mj_step` and bound to
that call's start/end interval. Their instantaneous evaluation time within the
discrete integrator has not been certified. Returned-force, moment and impulse
sums are diagnostic; the original robot contact scorer still refuses discrete.
Earlier point-timestamp interpretations in `run-01`/`run-02` are superseded,
while their raw evidence remains preserved.

Evidence and frozen sources are under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/extensible-tape-bond-20261007/`.
All 38 tape/extensible/bond tests pass. Reproduce the local adhesive case at
both timesteps, each in a new directory:

```sh
PYTHONPATH=. .venv/bin/python -m tools.diagnose_extensible_tape_bond --strip-length .050 --dt .000025 --out /absolute/new/local-25us
PYTHONPATH=. .venv/bin/python -m tools.diagnose_extensible_tape_bond --strip-length .050 --dt .000010 --out /absolute/new/local-10us
```

Early-tape carton retention remains unproven. The actual masking-tape product,
width, substrate adhesion and application pressure still need appropriate data.

As an example of product-specific data, the manufacturer's
[Scotch 2328 technical sheet, dated 04/01/2024](https://multimedia.3m.com/mws/media/1809252O/3m-scotch-tape-2328-masking-tape.pdf)
lists 0.135 mm total thickness and adhesion to **steel** of 6.5 N/25 mm under
AFERA 4001. That is not the user's identified tape or a cardboard result, and
does not map directly to MuJoCo's force-per-contact parameter. No model value
was changed from that example. A defensible update needs the actual product,
substrate/loading protocol and a matched numerical coupon; tensile breaking
strength likewise is not an elastic modulus.
