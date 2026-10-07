# Tagged short-flap observation phase

The normal RGB-D short-flap plane estimator is intentionally limited to
`-40° < angle < 30°`. Near a flat flap, its hinge plane can become sparse or
ambiguous, so that bound must not be widened to make a run continue.

The explicit `AdditionalViewConfiguration(observe_tagged_shorts=True)` phase
uses the current decoded AprilTag poses for IDs 11 and 12 when those IDs have
accepted same-frame aligned-depth quality. It retains the existing PixelPort
tag angle range `-40° < angle < 103°`, the current tag identity and depth
quality checks, the strict plane estimate when available, and the 3° tag/plane
and cross-view contradiction gates. A decoded-but-depth-rejected tag, stale
history, missing current tag angle, or missing current source refuses; no prior
angle fills a missing observation. The option requires `observe_open_shorts=True`
and is default-off.

This phase changes observation coverage only. It does not authorize motion,
relax collision limits, infer cardboard contact, or establish that tags remain
visible through a 90° fold. The current major-first dynamic run did not enable
it; the next experiment should treat it as a separate camera/observation
assumption and score the full executed contact evidence independently.

Focused coverage is in
`tests/test_folding_tagged_short_view.py`, including tag-only packets, mixed
primary/additional sources, rejected depth, stale quality, missing tags,
cross-view contradictions, strict-plane disagreement, and unchanged RNG/render
behavior.
