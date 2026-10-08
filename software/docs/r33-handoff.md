# R3.3 handoff after the first physical print

Current design: R3.3, 4 October 2026. Offline preparation only; no R3.3 physical trial or model training has run.

The user reported that the R3.2 print finished, the basket perched too high on its three internal rests, and the ring was too light and lacked a robot grip. The user rejected a latch as extra robot difficulty. R3.3 therefore uses gravity retention and a dedicated handle.

## Print changes

- Reservoir: 152 × 94 × 45 mm including grip fins; opening 134 × 88 mm. Rest height is 27 mm instead of 35 mm, placing the basket rim level with the 45 mm reservoir rim. Lowering the rests alone in the old reservoir would cause interference.
- Basket: byte-identical to R3.2. Reuse the existing print.
- Retainer: 9 mm tall sloped body outside the grow pad, four small contact tabs, 34 mm overall height including a rear handle. Handle is 16 mm wide × 6 mm pinch thickness. **Print at 100% infill.** Solid volume 13.967 cm³ estimates 16.76 g at an assumed material density of 1.2 g/cm³. Actual mass and ability to hold the purchased mat flat remain untested. No latch, magnets or added weights.
- `parts/r33/plate.stl` contains the new reservoir and ring ONLY; it is an alternative to printing their separate STLs. Set the retainer object to 100% infill. The basket is supplied separately for completeness.
- Print upright, in millimetres, at 100% scale. Re-slice; previous R3.2 G-code and 3MF do not contain this revision. No R3.3 slicing or physical-print claim is made.

[Current CAD and STLs](../parts/r33/) · [Updated HTML slides](r33-review.html)

## Robot placement

Pick the ring by the top 10 mm of its rear handle. Lower it fully until supported on the pad edges, verify support and seating, then open above the basket rim and retreat vertically. There is no drop placement. The supervisor now refuses ring release without both `supported_before_release` and `ring_seated` evidence. The illustrative fingers are 12 × 4 × 24 mm, with 6 mm closed and 16 mm open spacing; their seated tips clear the rear rim by 13.4 mm. These are CAD proxy dimensions, not measurements of the actual robot.

Preparation on the stand stays at basket-floor Z=35; final seating in the reservoir is Z=27. Cloth underside in the reservoir is at Z=29.4 under the 2 mm cloth and 3 mm pad assumptions. Keep the water below it. The lower basket leaves more cloth at the bottom; confirm length, bend and water contact with the real material.

## Resume on the connected Mac

Preserve local calibration, ports, camera identities, recordings and taught poses. Let active calibration finish. Inspect `git status` and update with `git pull --ff-only origin main` only when the local branch permits it. Do not apply the old transfer patch after pulling.

The existing `farm.assembly.r32`, `farm.learning.r32_train`, `farm.learning.r32_evaluate` entrypoint names and `profiles/r32-assembly-v0.json` filename remain for compatibility. The active profile now pins R3.3 hashes and uses `data/r33`; old R3.2 episodes are incompatible. Hardware execution remains disabled. [Training instructions](r32-training.md) give the commands and recorder contract.

Before collecting training data, verify printed seating, ring strength and weight, real jaw approach/opening, material thickness, leakage, water delivery and measured fixture poses. No live observer/executor adapter has been connected by this preparation. Preserve the robot-driven visual-teaching workflow and existing motion limits; uncertain held objects stop/hold without release.

## Evidence

The current three meshes are watertight single solids. The upright audit finds no downward area beyond 45 degrees excluding bed faces. Nominal assembly and 61 sampled vertical offsets are clear; the defined jaw-opening and vertical-motion envelopes are clear. The ring covers 64 mm², or 1.73%, of the pad in top view. Replacement layout has an 11.7 mm minimum bed-edge margin. See `parts/r33/geometry.json`.

31 offline preparation tests passed, including refusal to release an unseated ring. CAD clearance, a Blender storyboard and offline tests do not establish physical grasp, robot reach, mat retention or successful growing.
