# Explicit fresh secondary geometry after primary carton identity is absent

This is an offline observation component. It does not establish a successful fold, physical camera calibration, or permission to execute on a robot. The hypothetical camera profiles and all observation, collision, material and controller thresholds remain unchanged.

## What failed in the dynamic controls

Both `major-first/paired-short-coherent-front-02` and `major-first/paired-short-coherent-left-view-03` completed **0/3**. The first control failed once on a current cross-view far-angle disagreement and twice on missing primary carton registration. All three left-offset runs failed on missing primary carton registration. These failures are retained; the component below does not relabel them as successes.

Frozen replay diagnosis:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/short-moving-registration-2026-10-07-03/`

`check_final_visibility.py` restores six exact final states only for rendering. It feeds each view's RGB-D, declared intrinsics/anchors/marker geometry and earlier pixel priors to the unchanged estimators. `score_marker_visibility.py` performs separate privileged visibility scoring afterward. No object pose, hinge state or segmentation enters observation.

The station has fresh anchors 1/20 in all six states. In the five registration failures, no carton marker is decoded in any of three fresh synthetic depth-noise seeds. The current configured additional view—original front for two states, declared left-offset for three—independently decodes anchors 1/20 and wall marker 10 and identifies all four hinge planes in **15/15** cases.

The missing primary identity is not purely arm occlusion:

- Station wall 21's code is entirely occupied by its own marker geometry in all six projected code regions. No arm occludes that code. Its viewing angle is 77.2–77.6° from the face normal and its narrow projected altitude is only 9.82–10.14 pixels, about 1.23–1.27 pixels per code cell. It is absent even from the raw detector output in five states. This supports a perspective/resolution decoding limitation.
- Wall 10 is partly covered by left-arm geometry in five states. In the remaining left-offset seed-1 state, its code is visible but its quiet-zone region is crossed by upper-arm geometry. It is also oblique, about 71°, and fails to decode. Projected boundary pixels assigned to neighboring cardboard are recorded separately; these are not claimed to be robot occlusion.
- In the sixth state, station wall 21 decodes with Hamming distance 1 and margin 58.8. The saved station far angle is 31.634° versus front 34.661°, a 3.027° disagreement, so the existing gate correctly refuses. Independent scoring of fresh component seeds finds station carton orientation errors 2.35–2.85°, versus front 0.34–0.56°. The station tag's small depth-fit residual does not imply accurate carton orientation.

The raw report hashes source scenes, frames, results, assets, source files and RGB-D caches. Depth noise is independently seeded at 0.8 mm with 25% dropout; it does not reproduce the dynamic run's complete random-number stream. No dynamics steps run.

## Opt-in contract

The default is unchanged:

```python
AdditionalViewConfiguration(..., allow_primary_carton_absence=False)
```

Explicit `True` uses `PixelPort.observe_with_carton_absence(label)` only after a normal sequence-1 observation has fully succeeded. That startup still requires both housing tags, their encoder/FK agreement, fresh anchors and a carton registration. A missing or failed startup cannot be promoted into a valid prefix. Any failed observation latches the underlying PixelPort and wrapper against further observation or motion until reconstruction.

Every partial capture first runs the existing fresh primary anchor registration and every currently visible housing/FK check. Only **absent carton identity** may return a partial result. Decoded carton/short identity with rejected aligned depth is not absence. Ambiguous carton markers, anchor failure, stale/unsynchronized data, bad FK and other exceptions remain fatal; no exception is caught and converted to missing data.

The exact schema is defined in `carton/folding_observation_status.py`:

| Field | Value |
| --- | --- |
| `packet_schema` | `carton_rgbd_missing_carton_identity/v1` |
| `source` | `calibrated_rgbd_missing_carton_identity` |
| `carton_status` | `missing_carton_identity` |

The partial packet carries current sequence/time, camera identity, anchor-calibrated `world_from_camera`, declared intrinsics/anchor poses with a declaration hash, current tags and observer quality, image hashes, and the current visible housing/FK checks. It contains **no** `world_from_box`, `box_registration` or `angles` fields. PixelPort clears its cached box field; prior angles are not returned as observations.

The wrapper validates this packet against its current primary observer and FK trace before processing the additional view. The additional camera must independently register fresh anchors and carton geometry and supply **every required flap** with the unchanged identity, spatial support, angle-range and ambiguity gates. Missing primary anchors are not supported by this fallback.

On acceptance, all selected geometry comes from the additional view: top-level `camera`, `world_from_box` and `pose_source_camera` identify it; each angle retains `source_camera` and its raw method. The audit preserves the primary partial packet and the additional full packet, records `primary_carton_status`, and marks carton comparison as unavailable rather than fabricating a zero disagreement. Both cameras share one unchanged simulation timestamp.

Whenever both current carton poses exist, the existing 12 mm / 8° carton and 3° common-flap comparison gates still apply. The opt-in never suppresses a contradictory current primary estimate. Source-coherent short-contact consumers can continue matching the angle's source camera to the current raw packet. Existing single-camera portable-scene adapters still reject the multiview source.

## Actual component validation

Evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/primary-carton-absence-component-2026-10-07-01/`

`check_component.py` runs the real station PixelPort, normal startup housing/FK check, and additional-view wrapper on the same cached frames. A separate robot-only FK model receives twelve named robot encoders. Full recorded state is confined to the sensor replay. Nominal optional-camera metadata is checked only as setup provenance; calibration remains solved from pixels.

- All **15/15** cases with genuinely missing current primary carton identity accept fresh additional geometry. The existing short helper's source-coherent packet checks pass for both shorts in every case.
- Maximum independent angular errors across those fallback cases are 0.410° near, 0.381° far, 0.371° left short and 0.346° right short. These synthetic frozen-frame scores are not physical accuracy bounds.
- Removing only the currently decoded additional carton tag 10 from RGB produces **5/5 refusals** while both additional anchors remain valid. The mask derives from detected image corners, not hidden object geometry.
- The exact saved 3.027° contradiction still refuses with the opt-in enabled.
- Fresh noise seeds on that sixth, both-registered state produce one refusal and two ordinary accepted observations; fallback is not used. In the two accepted cases, the retained station far-angle errors are 3.293° and 3.046° against independent truth. A 3° relative comparison gate is not a 3° absolute accuracy guarantee. This limitation remains unresolved, and the gate was not changed.

At source freeze, **411 tests passed and 2 optional recorded replay tests skipped**. Focused negatives cover startup bypass/retry, stale time/sequence/quality/calibration, cached geometry in partial packets, invalid FK, missing/contradictory anchors, decoded-but-rejected carton depth, ambiguous carton markers, both carton registrations unavailable, missing required secondary flaps and current cross-view contradictions. Prior camera, hinge, observed-scene, short-helper and paddle regression tests also pass.

This establishes the opt-in observation path on a bounded frozen component, not dynamic regrasp success or full closure. Camera mounting, station dimensions, actual depth uncertainty and physical execution remain unverified.

## Full-prefix dynamic follow-up

`major-first/paired-short-fresh-view-fallback-04` starts each of three seeds at
the original open carton and uses the left/back additional view. Its frozen
source tree includes `folding_observation_status.py`. The three original
partial-release prefixes reproduce 836, 892 and 905 baseline timestamped states
exactly. The accepted missing-primary-identity fallbacks number 1, 3 and 1.

All three partial major releases pass, but **0/3 completes the +10° short-flap
probe**. Seed 0 stops on a current far-angle contradiction after 34 contact
commands. Seed 1 reaches the 80-command approach bound; its left short moves
outward to −19.37°. Seed 2 progresses through 130 contact commands to short
angles −6.23° and +1.20°, then neither view supplies the required left short.
Its carton moves horizontally 10.79 mm and a bottom corner projects 9.68 mm
past the table edge. That legacy "clearance" statistic is an XY inset, not
vertical penetration.

The independent robot and panel contact scorers pass all executed intervals
within their respective scopes; they do not score table support or establish
task success. All failures remain in the 42-trial inventory and broad regression
passes 808 tests with two optional skips. The current evidence supports the
narrow observation fallback, not a completed folding controller.

## Shared failure state and exact refusal images

A subsequent independent audit found and fixed two observation gaps: a new
wrapper constructed around a failed wrapper's underlying primary could retry,
and a decoded housing tag with rejected depth could be treated as absent.
Observation failures now latch on the shared primary. Rejected housing IDs 2/4
stop both normal and partial observations; genuinely absent housing tags retain
the existing post-startup behavior. Command failures also latch on the shared
session and block direct calls, preexisting wrappers and rewrapping. The driver
already aborted on the first exception; these fixes close retry paths.

Each view now retains one bounded cache of the exact RGB and exposed noisy
depth already passed to its observer. The diagnostic driver writes
`refusal-rgbd/frames.npz` and a byte-hashed manifest on run refusal. Saving adds
no rendering, randomness, physics step or recovery. Sequence, simulation clock,
timestamp and view identities must match; stale or not-yet-captured views are
explicitly unavailable. A run refusal does not relabel an otherwise valid
observation as invalid. The manifest and result histories preserve that distinction.

This replaces the need to guess the original failure pixels from post-step qpos.
An earlier replay did not match the saved image hashes and remains explicitly
a restored-state diagnostic, not an exact replay. Evidence and limitations:
`output/bimanual-fold-sim/short-fallback-dynamic-diagnosis-2026-10-07-02/REPORT.md`
and `output/bimanual-fold-sim/fallback-contract-audit-20261007/README.md`.

The subsequent `paired-short-setpoint-feedback-05` batch verifies this raw
recording path on all three runs: result-to-manifest, archive and per-array
hashes match; both cameras' current sequence/time/clock match; and the exposed
additional arrays match the original observation packet hashes exactly. The
runs stop on predicted wrist geometry, not invalid observations. A separate
read-only reuse of the strict open-short estimator finds both shorts in the
saved primary pixels in all three cases, with maximum 0.275° current-view
disagreement. That analysis does not change the production observer or cure the
claw approach. See `exact-cache-v3-summary.json` beside the diagnosis report.
