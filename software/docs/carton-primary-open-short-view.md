# Opt-in current primary short-flap planes

`AdditionalViewConfiguration(..., observe_primary_open_shorts=True)` enables
the existing strict open-short hinge estimator on the station's exact current
exposed RGB-D cache. It requires `observe_open_shorts=True` and is intended for
the paired-short phase after partial-major release. The default is `False`;
the existing major-only and additional-only short paths retain their behavior.

```python
configuration = AdditionalViewConfiguration(
    assumption_id='offline:primary-and-additional-open-shorts',
    clock_id='offline:simulation',
    required_flaps=('long_near', 'long_far', 'short_left', 'short_right'),
    camera='front_left_back',
    observe_open_shorts=True,
    observe_primary_open_shorts=True,
)
```

This is an offline observation variant. The additional calibrated camera is
still a hypothetical, hardware-unverified mount. It does not establish camera
accuracy, safe motion, regrasp success or full closure.

## Freshness and source binding

The wrapper first runs the ordinary primary observation, including its existing
startup housing/FK and fresh anchor/carton checks. It then reads
`primary.last_rgbd_frame` only when camera, sequence, simulation timestamp and
clock identity match the current primary packet. Intrinsics and exposed array
dtypes/values come from that cache. Camera and carton transforms come from that
same current primary observation. The new path has no render, noise generation,
physics step, simulator object pose, contact identity or commanded-angle input.

The wrapper still clears both frame caches at every observation attempt. Missing,
stale or misidentified required primary cache data refuses; no previous frame
is relabeled. When the separately enabled typed primary-carton-absence fallback
returns no carton pose, primary short estimation is skipped entirely. It never
borrows the additional camera's carton registration. All required geometry must
then come from the current registered additional packet as before.

The reused estimator retains every support, hinge identity, competing-plane and
strict **−40° < angle < +30°** check documented in
[the open-short observer](carton-open-short-vision.md). Its own prior history
starts empty and updates only after the full merged observation accepts. Priors
can reject a discontinuity; they never supply a missing plane.

Legacy `aligned_depth_cardboard_plane` short estimates remain untrusted for
fusion. Current identified primary estimates, including decoded short tags,
must agree with a current strict primary plane within the existing 3° comparison
gate when both exist. A valid current tag retains precedence after that check.
A tag can still supply its current identified estimate when the plane is absent.
No decoded identity, support, ambiguity, range or cross-view contradiction gate
is weakened. The existing independent carton-pose comparisons remain mandatory
whenever both current carton poses exist.

New station plane rows retain method
`aligned_depth_open_short_hinge_consistent_plane`, `source_camera='station'`
and the current `observed_seq`. They are stored in the current audit's primary
packet. Thus the contact helper pairs the angle with its own primary carton pose,
even when the additional camera is also registered. The raw PixelPort reading
is not modified. `audit.primary_open_short_observation` retains the original
PixelPort short rows, exact frame metadata/array hashes, all strict plane rows,
tag/plane comparisons and valid bounds. An absent primary carton instead records
`status='not_observed'` and its explicit reason. The phase declaration includes
the opt-in flag and primary data source in its assumptions hash.

Merged observations still use `calibrated_rgbd_additional_view`; the existing
single-camera portable adapter continues to reject that source. Rewrapping a
nonfailed wrapper preserves its entire reading and audit prefix. Shared primary
observation/command failure latches, cache clearing and motion guards remain.

## Exact recorded-array validation, 2026-10-07

Evidence directory:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/primary-open-short-optin-2026-10-07-02/`

`check_exact_cache.py` verifies the result-to-manifest hash, archive hash/size,
every declared array's hash/dtype/shape/size, and every producer source-snapshot
hash for the final frames of `paired-short-setpoint-feedback-05`, seeds 0–2. It
matches each current primary packet by sequence, time and clock, checks current
anchor/carton identity, and verifies that the additional packet's RGB/depth
hashes match its saved arrays. It invokes the actual new primary helper and
unchanged packet fusion on those current inputs. No replay, new render, new
noise or physics runs.

| Seed / sequence | Primary left angle | Primary right angle | Largest short disagreement |
| --- | ---: | ---: | ---: |
| 0 / 259 | −11.855772° | −15.217708° | 0.245034° |
| 1 / 288 | −12.448180° | −14.696485° | 0.227190° |
| 2 / 286 | −12.598196° | −13.968702° | 0.274457° |

All three enriched observations accept all four required angles, with both
shorts sourced from station. Left support is 814–1,023 pixels across 35–42
patches; right support is 8,559–9,745 pixels across 54–57 patches. These are
pixel-estimator results and cross-view comparisons, not measured physical
accuracy. The original three runs remain **0/3** at the +10° short target because
of predicted wrist collisions. This component changes no recorded outcome.

Five negative checks per scene refuse: stale packet time, absent current carton
pose, contradictory current short view, a deliberately derived zero-depth
primary plus absent additional shorts despite priors, and cleared cache.
The deliberately modified negatives are labeled separately from the exact
recorded inputs. Source snapshots and full packet/evidence results are saved.

The first attempt, retained in `primary-open-short-optin-2026-10-07-01`, stopped
on a diagnostic assertion expecting all quality fields to equal an earlier
analysis. That analysis used merged short priors; the new method correctly
starts its own empty primary history. Estimated angles were identical, while
one competing-plane ratio was 2.494% versus 2.319%. The final component records
empty priors explicitly and independently checks all unchanged quality gates.

At freeze, **198 tests pass** across the new primary wrapper tests and existing
additional-view, short-estimator and recording suites. Coverage includes source
coherent contact poses, decoded-tag conflicts, camera/clock/time/sequence errors,
absent current poses, no prior filling, all plane gates, shared failure latches,
unchanged additional RNG/render counts, phase-prefix preservation and retained
wrong-surface/edge-on/competing-plane negatives. No dynamic trial with this
primary opt-in has been run here. Near-horizontal or occluded shorts can still
be ambiguous and must refuse.

The expanded suite including the existing paired-short contact probe and
portable observed adapter passes **283 tests**. `git diff --check` passes.
