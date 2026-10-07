# Open-short hinge observation after partial-major release

The missing short-left estimate after the near40/far35 release is a visibility and surface-mixture problem in the legacy sampling stripe. Fresh front-view pixels support both short hinge planes in the three recorded release states, using the same geometric checks as the major observer. The new method is explicitly limited to **−40° < angle < +30°**. It does not identify closed or nearly horizontal shorts.

This is offline component work. There is no hardware action, measured camera-accuracy claim, successful regrasp claim or full-fold claim. The front camera remains an additional hypothetical calibrated view.

## Released-state diagnosis

Source runs:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/major-first/both-partial-majors-passive-release-01/trial-000..002/run`

Bounded replay evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/short-release-visibility-2026-10-07-01/`

`diagnose.py` records initial/final station and front RGB-D for three runs, then evaluates five 0.8 mm depth-noise/25% dropout seeds per view. Both views register fresh anchors and the carton from pixels. Station retains its stationary-camera startup semantics; front solves its camera transform independently each frame. Priors are saved pixel observations. Simulator angles, printed-face normals and segmentation labels are joined **after** pixel decisions, solely for independent scoring.

The latest saved station readings omit short-left in all three runs. Fresh replays sometimes cross the legacy acceptance threshold in trial 001 (2/5 cases), explaining the intermittent estimate. Neither view decodes short tag 11 or 12 at the final frames:

- Station short-left printed-face view angle is 86.9–87.4°, nearly edge-on, with only 98–109 visible tag-geometry pixels in diagnostic scoring.
- Front sees the **unprinted back surfaces**; printed-face view angles are about 117–119°. It cannot provide decoded short-tag identity.
- Broad cardboard remains visible. Trial 000 has 3,070 visible short-left cardboard pixels from station and 48,323 from front.

Trial 000, noise seed 0, has 378 eligible old-stripe pixels: 126 score as short-left, 229 as far-major. Its leading histogram group is 149/378 = 39.4%, below the unchanged 45% requirement. Refusal is correct for that stripe. Lowering the threshold would accept a surface mixture.

The preliminary broad-plane trace reuses the major observer's unconstrained hinge/plane/six-patch checks and 35% competing-plane refusal. Front candidates identify both shorts in 15/15 cases each, at least 56 patches, with independent errors below 0.187°. Station-left candidates pass only 11/15; the remaining cases are ambiguous. This preliminary trace is preserved as a diagnostic proposal, separately from the implemented wrapper evidence below.

## Independent bounded observer

`carton/folding_short_hinge_vision.py` exposes:

```python
depth_open_short_flap_angles(
    rgb, depth, intrinsics, world_from_camera, world_from_box,
    priors=None, *, box=None,
)
```

Inputs are aligned metric RGB-D, pinhole intrinsics, current calibrated camera/carton transforms, declared dimensions and optional prior **observations**. There are no simulator, segmentation, contact, grasp-success or commanded-angle inputs. An unconstrained plane is fitted in known short-hinge coordinates: along y, inward along ±x, and upward from the declared short hinge height.

| Retained check | Limit |
| --- | --- |
| Plane support | At least 60 pixels |
| Spatial support | At least six 25 mm patches with at least eight pixels each |
| Along-hinge / radial span | At least 75 / 30 mm |
| Hinge-axis error | At most 4° |
| Hinge-plane offset | At most 6 mm |
| Competing distinct plane | Refuse at 35% or more of leading support |
| Prior discontinuity | Refuse at 35° or more |
| Supported short angle | Strictly −40° to +30° |

The broad ROI keeps points 45 mm to `flap_length − 2 mm` radially from the hinge and inside the declared panel width. The full histogram seed search and candidate deduplication follow the major estimator. The open-angle restriction is a refusal boundary, not clipping. Priors never fill omissions. Closed/coplanar identity is intentionally unsupported.

Rows use method `aligned_depth_open_short_hinge_consistent_plane`, with valid angle bounds, pixel/patch support, span, axis/offset checks and competing-plane support ratio. Missing/ambiguous panels are omitted. The caller remains responsible for fresh calibration and synchronization.

## Explicit additional-view phase

```python
configuration = AdditionalViewConfiguration(
    assumption_id='offline:open-short-stage',
    clock_id='offline:simulation-seconds',
    required_flaps=('long_near', 'long_far', 'short_left', 'short_right'),
    observe_open_shorts=True,
)
port = AdditionalViewPixelPort(previous_port, configuration=configuration, seed=0)
```

Activation belongs to the paired-short stage **after** both partial majors have physically released in the simulation. Default `observe_open_shorts=False` continues to process majors only. The primary PixelPort, controller, scene, material and collision gates are unchanged.

Both estimators consume the **same** front RGB, depth, intrinsics, independently solved camera pose and carton registration. No separate short capture is made. Exact same-clock, fresh registration, cross-view pose and common-angle disagreement checks remain in force.

Primary short rows from `aligned_depth_cardboard_plane` are explicitly omitted from trusted fusion: the old stripe lacks hinge identity. Such rows cannot fill a required short or override a current identified estimate. A current decoded primary short tag may corroborate or supply an angle inside the open interval. Conflicting identified short estimates beyond 3° refuse. Required shorts missing from both identified views refuse. Front-sourced rows use `additional_view_open_short_hinge_consistent_plane`, retaining source method, camera and valid bounds.

Passing a **nonfailed existing wrapper** preserves its complete merged reading prefix, previous additional-view history and declaration/hash under `previous_phase`. Capture uses its original underlying primary. Rewrapping a failed wrapper refuses and requires a new primary startup; it cannot bypass the failure latch.

Merged source remains `calibrated_rgbd_additional_view`, which the single-camera portable `OfflinePixelSceneAdapter` deliberately rejects. Portable or physical multiview use needs an explicit multiple-sensor calibration/uncertainty contract. Fit residuals and cross-view agreement are not physical error bounds.

## Actual wrapper component and negative cases

Final evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/short-open-component-2026-10-07-01/`

`check_adapter.py` runs real PixelPort startup housing checks, a major-only wrapper phase, and the new short phase on frozen released RGB-D. The existing sensor replay boundary owns full recorded state; separate robot-only FK consumes twelve named robot encoders. No dynamics or robot action runs. Pixel priors are explicit and independent truth scores are added only after observation.

Across three final scenes × five noise seeds:

- **15/15** observations contain all four required fresh angles. Both shorts come from the same front capture with at least **56 supported patches**.
- Maximum independent errors: **0.282° left / 0.266° right**. These are simulation scores, not physical bounds.
- Largest competing-plane ratio: **6.32%**, below the unchanged 35% ambiguity limit.
- Maximum carton disagreement: **2.707 mm / 1.104°**, again not measured camera accuracy.
- Merged readings and prior additional-view histories survive the phase transition exactly.

Three negative cases mask fixed front-image rectangles covering the short surfaces, chosen in image coordinates rather than by segmentation. Anchors 1/20, carton marker 10 and both major estimates remain fresh. **3/3** omit both shorts and refuse the required short-left observation despite previous valid short values. Current registration and historical angles cannot substitute for current flap pixels.

The report hashes 469 assets, 12 cached sensor files, source recordings, pixel-prior sources, module/test sources and declarations. It asserts that sources remain unchanged during execution. To reproduce without overwriting evidence, copy `check_adapter.py` to a fresh sibling output directory and run it from the software repository with `PYTHONPATH=.` and `.venv/bin/python`.

At source freeze, **294 tests passed, 2 skipped** across short/major vision, additional view, observed adapter/scene/CLI and folding paths. Skips are pre-existing optional major recorded-replay tests; the explicit component above ran separately. New negatives cover missing pixels, edge-on support, wrong axes/offsets, competing planes, major-only surfaces, closed/out-of-range shorts, prior filling, legacy omission, identified-tag disagreement and phase-latch bypass. Shared-array tests verify there is no extra capture for shorts.

Actual recorded validation covers approximately −12.3° to −12.8° left and −13.8° to −15.2° right. Analytic pixel tests additionally cover visible −30°, −15°, 0°, +10° and +25° poses. Dynamic regrasp through positive angles remains a separate controller experiment. A hand can still occlude the flap or make its plane ambiguous, in which case the observer refuses rather than supplying an unsupported angle.
