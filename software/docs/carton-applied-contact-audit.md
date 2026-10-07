# Applied contact evidence for bare-claw folding

The simulation now samples each executed MuJoCo contact solve immediately after
`mj_step`, before geometry refresh, rendering or another step. Every sample
records contact identities, signed distance, the six wrench components, applied
controls and external forces. A load on a forbidden robot pair or a non-jaw
robot/flap pair stops the motion even if penetration is less than 1 mm. The
1e-6 N cutoff only separates numerical zero; it is not a permitted load limit.
The existing penetration, reach, joint, actuator and carton-motion gates remain.

Samples stream to `applied-contact-steps.jsonl.gz`. `folding.json` binds its exact
compressed-byte SHA256 and full simulated interval. The independent scorer
checks the hash, all event intervals and every step, recalculating classifications
and loads rather than trusting cached pass flags:

```sh
PYTHONPATH=. .venv/bin/python tools/score_folding_contacts.py --run /absolute/path/to/run
```

`CONTACT_ONLY_CLEAR` means complete applied-load coverage with no forbidden or
non-jaw loaded contacts. It is not a grasp, retention, four-flap, hardware or
material-validation pass. A named jaw geometry can still contact an unsuitable
surface; separate contact-location and task checks remain necessary. Old qpos
replays without applied solver samples return missing/incomplete evidence.
Forces reconstructed from those positions are not executed-force measurements.

Three full-prefix near-hold runs (seeds0–2) independently scored clear across
35,531, 34,009 and 36,938 steps. Final near angles were91.86°,91.22°,90.67°;
all other flaps remained open. Concurrent wall time was28.42seconds, with a
2.90 worker overlap factor. This measures overlap, not controlled serial speedup.
Exact report/log identities are in
[evidence/carton-applied-contact-audit-20261007.json](evidence/carton-applied-contact-audit-20261007.json).

Regression tests include a real0.001mm overlap carrying about2.7006N, which
must stop, and the same overlap with zero separating-contact force, which must
not be misreported as loaded. Runtime integration verifies stopping after the
first2ms step and preserving that step's evidence. No hardware was accessed.
