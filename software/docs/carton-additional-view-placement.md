# Declared additional-view placement variants

The original front view loses fresh world-anchor registration during the first paired-short regrasp batch. This is a real arm occlusion. Two explicitly declared side-offset camera variants recover full fused observations on the bounded set of recorded poses below. They are hypothetical renderer setups, not physically installed or calibrated cameras.

## Original failure and screened alternatives

Source: `major-first/paired-short-passive-majors-01/trial-001/run`, frame 923, observation sequence 256. Primary station registration remains fresh: anchors 1/20, carton wall 21 and both majors are present. The **additional front** observer fails before carton or flap estimation because it decodes neither world anchor.

Evidence root:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/short-moving-registration-2026-10-07-01/`

`screen.py` replays the failed physical state for RGB-D rendering. No object state enters observation. Independent segmentation scoring afterward shows:

- Anchor 1 has zero visible geometry pixels. Its projected code region contains 1,092 left-wrist, 102 lower-arm and 44 gripper pixels.
- Anchor 20 is partially visible, but its code region is crossed by the right housing/tag geometry and does not decode.
- Original front RGB decodes only carton tag 10 and major tag 14. Registration correctly refuses in all three noise seeds.

Moving the optical center straight back and higher restores anchors, but the modeled camera mast blocks the carton registration marker. Those four candidates correctly refuse carton registration. Side offsets avoid that mast obstruction and recover anchors, carton marker 10 and all four hinge planes in the failed frame.

## Explicit profiles and API

`carton/folding_additional_view_profiles.py` defines only these names:

| Name | Optical center in declared station frame (m) | Look-at point (m) | Vertical FOV |
| --- | --- | --- | --- |
| `front` | Existing scene camera, unchanged | Existing scene camera, unchanged | 48° |
| `front_left_back` | (−0.20, −0.85, 1.05) | (0, 0.10, 0.12) | 48° |
| `front_right_back` | (+0.20, −0.85, 1.05) | (0, 0.10, 0.12) | 48° |

`add_additional_view_camera(root, camera_name)` appends only the requested named camera before XML compilation and returns its declaration. Original `front` is preserved. Calling the helper for `front` is an exact no-op when that camera exists. Unsupported names and duplicate named cameras refuse. No mass, collision geometry, marker, actuator or constraint is added.

Select the profile through the existing configuration field:

```python
configuration = AdditionalViewConfiguration(
    assumption_id='offline:left-offset-short-stage',
    clock_id='offline:simulation',
    camera='front_left_back',
    required_flaps=('long_near', 'long_far', 'short_left', 'short_right'),
    observe_open_shorts=True,
)
port = AdditionalViewPixelPort(previous_port, configuration=configuration)
```

The profile must have been explicitly inserted into the simulation model. The wrapper verifies its nominal position, quaternion, FOV, fixed camera mode and world attachment against the declaration. It repeats that metadata check on observations and includes the declaration in `assumptions_sha256`. These renderer extrinsics are **provenance only**. The observation transform is independently solved from fresh anchor RGB-D on every capture; it is never assigned from the renderer pose. The original front behavior remains the default.

The named source is retained in the raw additional packet, merged row `source_camera`, full audit and configuration. Source-coherent geometric consumers must match this source name to its current packet and use that packet's observed `world_from_box`; they must not silently mix the station carton transform with a secondary-local short angle. Existing synchronization, anchor/carton registration, pose/angle disagreement, plane support and missing-required-angle gates are unchanged.

## Full fusion across recorded moving poses

Evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/short-moving-registration-2026-10-07-02/`

`check_profiles.py` selects up to eight evenly spaced short-stage observation states per run, including both endpoints: eight from trial 000, three from trial 001 and eight from trial 002. It renders the original station/front views and the two declared alternatives, then runs the **actual station PixelPort plus additional-view wrapper**, not just the raw secondary estimator. Original housing startup checks use a separate robot-only FK model fed twelve named robot encoders. Prior measurements come from earlier saved pixel readings. Full recorded state is confined to sensor rendering and independent scoring.

Each of the 19 states is evaluated with noise seeds 0, 1 and 2, using 0.8 mm synthetic depth noise and 25% dropout:

| Profile | Accepted complete fused packets | Minimum left/right short patches | Maximum left/right short error against independent truth |
| --- | ---: | ---: | ---: |
| Original `front` | 54 / 57 | 47 / 55 | 0.282° / 0.266° |
| `front_left_back` | 57 / 57 | 23 / 30 | 0.430° / 0.480° |
| `front_right_back` | 57 / 57 | 34 / 27 | 0.617° / 0.818° |

All three original-front refusals reproduce trial 001's final transit state. Both offset profiles see **both** world anchors in every accepted case. Their largest competing-plane support ratios are 19.47% left-profile and 10.09% right-profile, below the unchanged 35% ambiguity limit.

Maximum cross-view carton disagreement is 5.142 mm / 1.025° for the left profile and 3.811 mm / 1.221° for the right profile. These synthetic comparisons and angular scores are **not physical camera accuracy bounds**. The lower short-angle error makes the left profile a reasonable first offline experiment; the Cartesian contact target and dynamic trajectory still require separate validation.

Independent model scoring places the optical centers at least 0.837 m from conservative bounding spheres of recorded robot collision geometries. This checks only optical-point separation in sampled robot states. The actual camera enclosure, support, cables and a mount's swept-volume interference are unmodeled. It does not prove a physical installation is feasible or calibrated. The longer viewing distance and reduced short-patch support also remain real tradeoffs.

The report hashes source scenes/frames/results, 469 assets, rendered input caches, source files and per-case declarations. Sources are checked unchanged through execution. No dynamics, contacts, robot commands or modified material parameters are introduced. The two variant cameras in the diagnostic XML represent alternative optical views; a normal variant run inserts only its requested additional camera.

Parent integration independently compared compiled models with one extra camera each. All 444 non-camera physical arrays and all five original camera poses/FOV/body/mode values remained identical. Only the requested camera and expected allocation metadata differed. Evidence:

`/Users/wk/Documents/ChatGPT/Hackatuson/output/bimanual-fold-sim/additional-camera-model-audit-20261007-02/comparison.json`

## Validation and limits

At source freeze, **313 tests passed, 2 optional recorded major-replay tests skipped**. Tests cover camera-only XML insertion, original-front preservation, duplicate/unsupported profiles, wrong nominal pose/FOV/attachment, metadata changes between observations, named source identity and independence of pixel registration from nominal renderer extrinsics, alongside the prior vision and scene tests.

This 57-case component is a bounded sample of existing recorded poses, not exhaustive trajectory coverage. Later source-coherent/substep trajectories can reveal new anchor or carton-marker occlusions and must retain refusals. Dynamic folding success, physical mounting, real camera noise, measured station calibration and hardware execution remain unverified.
