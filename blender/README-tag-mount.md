# Actual gripper AprilTag placement rendering

`render_carton_gripper_tags.py` imports the actual SO-101 wrist, gripper and
moving-jaw STLs using their pinned URDF assembly transforms. It does not use the
illustrative gripper primitives in `render_handbook.py`. The gripper body STL is
byte-identical to the one in the existing `robot-farm-design` scene assets.

Prepare the upstream model with the existing controller's `model-fetch` command,
then render in a separate process:

```sh
cd software
python -m carton.servo model-fetch --out data-carton/models/so101
cd ..
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
  --python-exit-code 1 --python blender/render_carton_gripper_tags.py -- \
  --out /absolute/path/to/output
```

`--model-dir` overrides the model path; `--tag-kit` overrides the marker JSON.
The checked-in `carton-marker-grids.json` comes from `carton.servo tag-kit`; its
ID 2 pattern was decoded from both final Blender renders using the actual shared
pupil-apriltags detector (hamming 0, margins approximately 67). `--draft` renders
both sides without a backing to inspect the underlying geometry.

Outputs include two labelled PNGs, a self-contained `.blend` with embedded source
license/attribution, and a JSON placement report. The source meshes retain their
units and shape. Material finishes and studio lighting are illustrative. Added
backing/spacers are explicitly named PROPOSED in Blender and are **not** a
manufacturing-ready bracket. Do not export them as a fit-tested printable part.

The code samples the original moving jaw at 23 angles (-10 to 100 degrees) and
checks mesh surface intersections against the new backing/spacers. It excludes
unmodeled installed camera, cables, fasteners and other robot joints, and is not a
continuous or whole-robot collision test. Camera visibility is not established
by a studio rendering. Validate both physical camera views before mounting.
