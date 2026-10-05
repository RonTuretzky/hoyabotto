# OAK-D Lite: camera-only depth bring-up

This utility uses DepthAI 2.33 directly, without LeRobot, ROS, or motor access.
It does not yet supply observations to the farm controller or enable assembly.
Run camera commands from Terminal on macOS. Connect the OAK with a USB 3 data
cable, preferably directly to the Mac for the first test. Quit other OAK apps.

From the repository's `software/` directory:

```sh
python3.12 -m venv .venv-oak
source .venv-oak/bin/activate
python -m pip install -r requirements-oak.txt
python -m farm.oak_camera list
python -m farm.oak_camera capture --usb2
python -m farm.oak_camera preview --usb2 --seconds 120
```

Use `--device ID` if several OAKs are attached. In preview, click the RGB image
to measure the median axial distance Z in an 11-by-11 pixel patch. The label
also shows how much of the patch has valid depth. Q exits; S saves a capture.
The depth display is grayscale: black means missing, white means 2 metres or
farther. Raw saved measurements are not clipped to that display range.

Captures go under `data/oak-captures/` in unique timestamped directories:
RGB PNG, 16-bit millimetre depth PNG, NumPy depth array, and JSON metadata.
Zero depth is missing data. The JSON records device identity, USB speed,
synchronized device timestamps, and valid-depth fractions. A capture with no
valid depth exits nonzero. A nonzero valid fraction is not an accuracy test.

If startup reports `X_LINK_DEVICE_NOT_FOUND` after booting, unplug the camera
for 10 seconds and reconnect with a short data cable directly to the Mac.
Retry `python -m farm.oak_camera capture --usb2` to test at USB 2 speed.
This error does not by itself distinguish cable, power, firmware, or host USB
problems. Enumeration alone does not establish that the camera can stream.
On the development Mac, v3.10 failed to boot this camera even after reconnecting,
whereas v2.33 captured RGB/depth with the same connection. Keep the pinned SDK
for this unit. USB 3 streaming remains unverified; omit `--usb2` to test it.

## Tabletop test

Start with a rigid head/table-facing mount roughly 50–80 cm from the parts,
then adjust based on measured coverage. This is a trial position, not a
verified mount or required working distance. Keep both stereo lenses clear.
Use diffuse lighting and a textured, matte background. The Lite has no dot
projector; plain plastic can produce sparse depth even with clear RGB images.

The pipeline uses 640×400 stereo, extended disparity, left-right checking,
and synchronized depth aligned to 640×360 RGB at 15 fps. RGB uses factory
undistortion and the calibrated focus position when available. Luxonis lists about
20 cm minimum depth for 400p with extended disparity; that does not guarantee
precision on small fins at that distance.

With arms stationary, test the empty table, trough, and carrier separately:
save captures, measure camera-to-object distances independently, compare
depth readings, and check repeated captures for missing pixels and variation.
Measure the actual grip fin and rim, not just a large neighbouring surface.
After this passes, calibrate the camera-to-robot transform. Position in the
camera frame alone cannot authorize arm motion or prove successful seating.

## Sources

- [OAK-D Lite specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)
- [DepthAI Python SDK](https://github.com/luxonis/depthai-python)
- [Luxonis aligned-depth example](https://github.com/luxonis/depthai-python/blob/main/examples/StereoDepth/rgb_depth_aligned.py)

## Verification

```sh
python -m unittest discover -s tests -p test_oak_camera.py
```

These tests check missing-data handling, pixel bounds, and lossless metric
depth storage. Live camera and physical accuracy results are recorded separately.
For a longer capture check, add `--capture-seconds 30`; the JSON records frame
count, elapsed time, and maximum RGB/depth timestamp difference.
