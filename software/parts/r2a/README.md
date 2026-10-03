# parts/r2a — Cress retrofit R2a release (copied 2026-10-03)

Release meshes and records for the modified cress planter the robot is meant to assemble.
Copied from `/Users/wk/Documents/ChatGPT/Hackatuson/robot-farm-design/cress-retrofit/` on the
first laptop so the robot laptop has them; the hashes below are what `farm r2a` and
`farm.assembly.r2a.verify_parts` check. A file that does not match is not R2a.

| File | What | SHA-256 |
|---|---|---|
| `carrier.stl` | Original holder + four insets fused, two 24 × 6 mm grip fins. 177.98 × 83 × 43.5 mm | `438fd4cd…348f2` |
| `retainer.stl` | Optional paper-retaining frame, two 4 × 20 mm end fins. 148 × 66 × 18 mm | `e276ba3f…0de4c` |
| `grip_coupon.stl` | 6 mm fin on a base, for a dry jaw test only. 36 × 24 × 25 mm | `33d8bb11…3eb3f` |
| `source/cressmaster-trough.stl` | Original trough, reused unchanged. 180 × 85 × 25 mm, refill bay on −X | `5af45615…02cf8` |
| `source/cressmaster-holder.stl` | Original holder the carrier derives from | `e4bd4281…5aebf` |
| `source/cressmaster-inset.stl` | Original inset, used four times in the carrier | `c72c55b1…48037` |
| `cress_retrofit.scad` | OpenSCAD source of the derivative | — |
| `design.json` | Design contract: frames, grasp candidates, what is unvalidated | — |
| `geometry-report.json` | Mesh checks (manifold, components, zero intersection) and source hashes | — |
| `plate-preparation.json` | The M5C print job: settings, supports ON, part locations, failed-start history | — |

Full hashes: `farm/assembly/r2a.py` (`MESH_SHA256`, `SOURCE_SHA256`, `PLATE_SHA256`). Millimetres, print at 100 %.

The 3MF and G-code of the full plate (5 MB each) were not copied; they are only needed at the printer.
The print used supports ON (build-plate-only grid). Do not use the original planter's "supports off".

## Provenance

Derivative of Macce, "Self-watering Cress or Microgreens Planter", Printables model 434505,
CC BY-SA 4.0: https://www.printables.com/model/434505-self-watering-cress-or-microgreens-planter
Changes: holder and four insets unioned with collars; two grip fins added; new loose paper frame;
trough unchanged. Share-alike applies to the derivative.

## Frames

Assembly frame A = original source XY, original holder deck top at Z = 0. Translations from each
exported mesh to A (identity rotation): trough (0, 0, 0); carrier (0, 0, −20.5); retainer (0, 0, +1).
Code: `farm/assembly/frames.py`. Nothing here is a robot pose; see `docs/r2a-assembly.md`.
