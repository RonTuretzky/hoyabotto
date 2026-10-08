# Hypothetical additional-camera observation, 2026-10-07

The single station view loses the far major flap near 38–39 degrees because the panel becomes nearly edge-on. An explicitly enabled second rendered view recovers the angle in the frozen component cases below without changing the major-plane identity or spatial-support gates. This is an offline sensor component result. It does not establish a complete fold, a hands-clear hold, a physical camera installation, or camera accuracy.

## Diagnosed visibility failure

The central far-fold recording is:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/major-first/partial-far-central-01/trial-002/run`

At frame 786 its independently scored far angle is 38.213135 degrees. The station camera's sight line is 89.530187 degrees from the panel normal, only 0.470 degrees from edge-on. Independent segmentation used **only for diagnostic scoring** counts 1,021 visible far-panel pixels; only 163 enter the production hinge ROI. The candidate plane has 155 points, 0.086-degree hinge-axis error, 1.309 mm hinge offset and ample overall span, but **zero** 25 mm patches containing at least eight points. The production observer requires six such patches. Its refusal is appropriate: overall span alone would accept a thin, sparsely supported strip. The carton is still freshly registered from wall marker 21.

The original diagnostic, including images and candidate tracing, is preserved at:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/far-hinge-visibility-2026-10-07-01/`

The existing `front` rendered camera sees the panel more directly. It cannot replace the primary startup camera: its initial image lacks the left housing marker needed by the normal two-arm registration check. It is treated as an **additional hypothetical calibrated camera**, never silently substituted for the primary.

## Frozen view check

Evidence is in:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/far-hinge-visibility-2026-10-07-02/`

`frozen_views.py` records actual station/front RGB-D renders at the initial and final frames of the central recording and the three `partial-far-startup-lift-01/trial-000..002/run` recordings. The stored source XML and frame hashes identify the runs. Recorded full simulator state is restored only to produce diagnostic sensor images. The observer receives rendered RGB-D, intrinsics, declared anchor/marker geometry and prior **pixel** estimates; diagnostic truth is added only after estimation.

For each run, ten depth-noise/dropout seeds were checked with the unchanged major observer. The added front camera independently solves its world transform from fresh visible anchor pixels on every frame (`stationary_camera=False`), then independently registers the carton from its current marker pixels. At the final frames:

| Component result | Station view | Additional front view |
| --- | ---: | ---: |
| Fresh far-major estimates | 1 / 40 | 40 / 40 |
| Maximum front far-angle error against independent scoring truth | — | 0.246 degrees |

Maximum cross-view carton disagreement was 5.131 mm and 1.592 degrees. These are simulated disagreements, **not bounds on physical camera error**. The synthetic profile is 0.8 mm depth standard deviation with 25% dropout. Rendered depth also emitted the local renderer's `ARB_clip_control unavailable` precision warning. Neither noise injection, marker-fit residual nor agreement of the two views substitutes for physical calibration measurements.

## Explicit API and refusal semantics

The new module is `carton/folding_additional_view.py`:

```python
from carton.folding_additional_view import (
    AdditionalViewConfiguration, AdditionalViewPixelPort,
)

# primary is the existing camera='station' PixelPort. It may already have
# completed the primary-only prefix and its normal startup housing checks.
port = AdditionalViewPixelPort(
    primary,
    configuration=AdditionalViewConfiguration(
        assumption_id='offline:front-additional-v1',
        clock_id='offline:simulation-seconds',
        required_flaps=('long_near', 'long_far'),
    ),
    seed=0, noise=.0008, dropout=.25,
)
```

Construction is the explicit opt-in. Only station-primary/front-additional identities and the declared front 48-degree vertical field of view are supported. Both assumption and clock IDs must begin `offline:`. The camera pose is solved from observed anchors; no simulator camera extrinsic or object transform is read. The current carton dimensions and marker mounts remain declared model assumptions.

The wrapper calls the existing primary `observe`, including its startup housing/FK checks. It preserves a deep copy of all pre-existing primary readings and records the activation sequence. It rejects a primary with evidence of a missing/failed startup and checks fresh primary sequence/history at activation. Primary motion and gripper operations retain their existing collision and tracking checks.

The simulation clock must remain **exactly unchanged** across primary observation, additional RGB, additional depth, and completion of fusion. This explicit frozen-state rule does not allow a timing tolerance that could combine different simulated states. Both views must freshly register world anchors and the carton. The additional observer recomputes its camera transform on every call; a missing anchor or carton marker refuses the entire combined observation even if the primary has angles.

For each major, a fresh primary estimate is retained when available. A missing primary major may be supplied by a fresh additional-view hinge estimate. The additional view uses `depth_major_flap_angles` unchanged: at least 60 pixels, six supported patches, 75 mm along-hinge span, 30 mm radial span, 4-degree axis and 6 mm plane-offset limits, plus its existing near-horizontal identity and competing-plane checks. Priors can reject a discontinuity but never supply a missing estimate. Additional-camera AprilTags register the camera and carton; they do not bypass the major's six-patch gate.

If both views report the same major, their angle difference must be at most 3 degrees. This applies to near and far independently, including a common major that is not in the required set. Required majors missing from both views refuse. Optional majors absent from both remain absent. Cross-view carton translation/rotation disagreement must be at most 12 mm/8 degrees, matching the existing rigid-marker comparison limits. Configuration can make these comparisons stricter, not weaker. **These comparison gates are not physical uncertainty or clearance guarantees.** Values are selected without averaging either pose or angle.

A failed observation latches the wrapper: later observations and motion calls refuse until reconstruction. This also prevents a failed primary startup at sequence 1 from being skipped by continuing at sequence 2. Failed readings have an empty `angles` mapping and preserve separate raw evidence and the refusal reason.

## Evidence surfaces and portable-adapter boundary

- `readings`: preserved primary prefix followed by merged accepted/refused readings.
- `observer.history` and `arm_tag_checks`: unchanged primary-camera evidence.
- `additional_view_history`: complete combined-view audits, including independent additional camera/carton transforms, current tag quality, image hashes, per-view simulated capture times, and the raw primary/secondary estimates. Accepted audits include numerical pose and common-angle disagreements; refused audits retain the raw values and explicit reason.
- `declaration` and `assumptions_sha256`: explicit synthetic profile, anchor poses, setup/clock identities, activation point and mandatory `simulation_only=True`, `physical_camera_verified=False` labels.

Additional-sourced rows use method `additional_view_hinge_consistent_plane`, preserve `source_method`, and identify `source_camera='front'`. Entire merged readings use source `calibrated_rgbd_additional_view`. Consequently the current **single-camera** `OfflinePixelSceneAdapter` refuses these packets rather than silently assigning a front-derived angle to a station-only calibration. A future portable multiple-camera schema must explicitly carry both sensor/calibration identities and their uncertainty declarations before that backend can consume this mode. Existing folding controllers may consume the combined angle values in this declared offline experiment.

## Actual PixelPort wrapper component check

Final evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/additional-view-component-2026-10-07-02/`

`check_adapter.py` composes the real PixelPort with the new wrapper. It reuses the 16 hashed, actually rendered RGB-D caches from the frozen check. The sensor replay boundary owns recorded-state restoration and exposes only the simulated clock, declared intrinsics, images, and a separate robot-only FK model populated with the twelve named robot encoders. Source PixelPort angle priors are explicitly recorded and hashed. Independent simulator angle scoring is imported only after observation.

For each of four recordings, the primary startup housing check ran normally, then the wrapper was activated for the frozen final frame using seeds 0, 3 and 9. Results:

- **12 / 12** component observations accepted; all twelve far angles came from the additional front view with at least 51 supported patches.
- Maximum absolute far-angle error: **0.218 degrees**. Maximum near-angle error: **1.852 degrees**, with the primary estimate retained. These are independent simulation scores, not hardware accuracy claims.
- Maximum carton disagreement in these twelve wrapper cases: **3.857 mm / 1.929 degrees**. The random sequence differs from the earlier 40-case view check; the two maxima are not directly comparable.
- **4 / 4** current-front-anchor-hidden negative cases refused after a prior successful observation. The previous camera solution and angle did not fill the gap.
- 469 asset files, 16 cached sensor inputs, source-run files, pixel-prior source, module sources, declarations and output evidence are hashed in the report. No dynamics step or robot action occurred.

The earlier `additional-view-component-2026-10-07-01` evidence is preserved. The `02` report uses the final activation-freshness guards. To reproduce without overwriting either report, copy `check_adapter.py` into a new sibling directory under `bimanual-fold-sim` and run it with the software repository on `PYTHONPATH` and its `.venv/bin/python`.

Validation at freeze: **251 passed, 2 skipped** across the additional-view, hinge-vision, observed-adapter, observed-scene CLI, observed-scene and folding-path tests. The skips are pre-existing optional recorded hinge-replay tests; the explicit wrapper component above ran separately. Negative tests cover wrong/missing identities, unsupported planes, weakened declarations, pose/angle disagreement, missing fresh registration, inconsistent simulation time, stale sequences, prior-only filling, startup bypass, and failed-observation motion. A fixed-pixel test changes independent hidden object state and obtains identical combined output.

## Full-prefix dynamic component and matched control

`major-first/partial-far-additional-view-01` activates the wrapper only after
the physically executed near40 prefix and attempts far45 with a 0.5 mm startup
lift. All three seeds pass the declared partial target: independently scored
far angles are 43.44°, 45.64° and 44.45°. All three complete applied-contact
intervals independently score `CONTACT_ONLY_CLEAR`. The shorts remain open,
both majors remain partial, and this is not full carton closure.

The matched `partial-far-single-view-45-control-01` batch changes only the
additional-view option: seed0 passes, while seeds1/2 stop for missing fresh far
angles. The second view supplies one frame in seed1 and two in seed2. Seed0's
entire 727-state trace matches exactly; it crosses the visibility gap without
requiring the secondary estimate. Seeds1/2 retain identical recorded physical
prefixes until their respective single-view refusals. Evidence and file hashes
are in `evidence/carton-partial-major-search-20261007.json`.

Across the dynamic added-view runs, all 175 combined observations are accepted;
maximum cross-view carton translation disagreement is 4.770 mm. These are
comparison results under synthetic noise, not physical accuracy bounds.
Visibility beyond this partial trajectory remains unverified. No authoritative
measured station profile or physical second-camera calibration has been
established here.
