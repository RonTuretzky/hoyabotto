# OAK-D Lite cradle with a 23° wedge (`oak_d_lite_wedge23_cradle.stl`)

**Print this instead of the stock slot cradle** to let the head camera reach the fold policy's 58° down.

## Why

The fold policy trained with the head camera tilted **58° down**. Measured on 9 October, the stock cradle only
reaches **35.1° down** at the tilt servo's commandable maximum (tick 2580; two independent methods, the gripper
tags and a table-plane fit, agree). The servo cannot go further. This part builds the missing **23°** into the
cradle: at every servo position the camera looks 23° further down than with the stock part, so the servo's top
tick now gives **58.6°**. Nothing in the servo, its calibration or its limits changes.

It keeps the existing 63/63 policy's trained view, which is the only option that avoids a retrain.

| | Stock cradle | Wedge cradle |
|---|---|---|
| Camera at servo max (tick 2580) | 35° down | **58.6° down** |
| Camera at the "level" tick (2211) | 0° | 23° down |
| Camera at servo min (tick 2012) | 19° up | 4° down |

The camera can no longer look level or up. The fold task never needs that.

## What is the same as the stock part

Same tongue in the tilt link's slot, same **four M3 x 10** screws from the stock connector, same snap-in box for
the OAK-D Lite (push in from the front, lenses up, USB-C down, until the fingers click), same vent window, same
USB-C opening. Only the box is rotated 23° about the tilt axis; a solid block joins it to the tongue.

Verified in CAD against the stock tilt link mesh and the camera body (`geometry-report-wedge.json`): one
watertight solid, 50.5 cm³ (about 63 g PLA); **zero** intersection with the tilt link, the camera body, a straight
20 mm USB-C plug, and the four M3 shanks, which land in the pilot holes as before. The wedge direction was
checked three ways (mesh fit of both rotation signs, side-view render, and the optical-axis test): the other
sign swings the box into the link, so it is refused by the build script.

Not checked: the pan bracket (its mesh is misplaced in the design frame; the stock part reports the same false
overlap and fits the real robot). The servo's full sweep with the wedge was not physically tested. **Watch the
first head motion with a hand on STOP.** Nothing has been printed yet.

## Printing

STL is in print orientation: box front face on the bed, tongue standing up, 0.2 mm layers, 4 walls, 30 % infill,
PLA or PETG. Check the four pilot holes with a 2.5 mm drill if the printer closes them. See the report for bed
contact and overhang figures.

## Fitting (12 V off)

1. Unscrew the four M3 screws from the outside of the tilt link's feet, pull the stock cradle out of the slot, and
   unclip the OAK.
2. Slide the wedge cradle's tongue into the slot from the front until the block touches the link; put the four
   M3 x 10 back in through the feet (do not over-tighten).
3. Push the OAK into the box until the fingers click. Plug the USB-C, route the cable with slack for the full tilt.
4. **Re-measure the head pose** (`tools/auto_head_pose.py`, read-only dry run): expect ~58° at the servo max.
   The training also needs the camera at the right *position*, which this does not change; the station setback
   (cart ~45 cm too far on 9 Oct) must still be fixed.

## Files

- `make_oak_wedge_cradle.py` builds it from the stock cradle's script (`make_oak_slot_cradle.py`), `--wedge 23`
  default, sign verified in-script.
- `oak_d_lite_wedge23_cradle.stl` / `.step`, `geometry-report-wedge.json`.
