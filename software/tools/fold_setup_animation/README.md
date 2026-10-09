# Setup animation: arm base, pan axis, 220 mm, 120 mm

Makes `docs/img/fold-policy-setup/setup-words.mp4` and its stills for the setup slides. No robot.

1. `export_scene.py`: in a Python with mujoco. It writes the training station from a 220 mm fold-demo trial: the SO-101
   arms, the table and the tagged carton. It adds the upstream XLeRobot model's cart, mast and head (not its arms) and
   writes everything to `/tmp/foldviz/station.obj` and `station.json`.
   The model's legacy arm bases sit on the top-tray rim, so the cart is placed with the rim at the bottom of the
   SO-101 bases.
2. `build.py`: `Blender -b --python build.py -- --out /tmp/foldviz/frames` renders 600 frames (24 fps, about 10 min).
   It also writes the label anchors.
3. `overlay.py frames out`: draws the captions and labels. Then encode:
   `ffmpeg -framerate 24 -i out/o%04d.png -c:v libx264 -pix_fmt yuv420p -crf 22 setup-words.mp4`.

Paths at the top of each script point at the local fold-demo trial and the upstream model.
