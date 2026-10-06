# Gemma AprilTag test in MuJoCo

`tools/simulate_gemma_tags.py` runs the production `TagRobot`/`TagObserver` and
shared pupil-apriltags detector on RGB images rendered by MuJoCo 3.14.0. It uses
the existing photo-informed `PaddleSimulation`, actual SO-101 mesh/joint model,
actual paddle CAD and its existing position actuators.

The script adds tag36h11 patterns as black square geometry on white backings,
rigidly attached to the simulated table, gripper body and paddle. Perspective,
lighting and occlusion occur in the renderer. It does not paint tags onto the
rendered image or feed simulator positions to the detector. Only the verifier
uses the independent pinhole projection of MuJoCo's tag-centre sites.

## Reproduce

Use the existing Gemma simulation environment (MuJoCo, SciPy, Pillow and the
AprilTag dependencies). The scene packet and collision meshes are local assets,
not bundled into the repository:

```sh
cd /path/to/xlerobot-farm/software
PYTHONPATH=. /path/to/gemma-xlerobot/.venv/bin/python tools/simulate_gemma_tags.py \
  --simulation-root /path/to/gemma-xlerobot \
  --out /path/to/output/gemma-apriltag-simulation --gemma
```

`--simulation-root` must contain the existing `paddle_sim.py`, `gemma_mujoco.py`,
`scene-assets/` and `real-scene/measurement.json`. Outputs from this run are
written to `--out`; the prior paddle experiment is retained. Omit `--gemma` for
the deterministic tracking test without model inference. `--initial-only`
produces a setup preview and is not a passing validation run.

With `--gemma`, only local LM Studio at `127.0.0.1:1234` is contacted. The selected
model is `google/gemma-4-e4b`, using the existing simulation inference helper.
No robot client, motor owner, credentials or physical camera is instantiated.
No live Gemma chat is restarted or modified by this test.

## Checks

- Three tags detected on every nominal frame during an approximately 24 degree
  shoulder-pan sweep, with measured travel >=20 degrees.
- Detected tag centres and gripper-to-paddle displacement compared with projected
  simulator ground truth; maximum allowed error 2 pixels.
- Physical occlusion of the simulated paddle tag removes its measurement.
- Duplicate ID3 makes identity ambiguous and invalidates the observation.
- Shrinking the paddle tag below the production size threshold removes its
  accepted measurement. Removing the faults allows reacquisition.
- Stale capture timestamps and bad image hashes are rejected.
- A latched simulated STOP rejects subsequent motion.
- Real Gemma calls `robot_get_tags`, requests +12 degrees through a simulation-only
  motion tool, reads tags again, and stops. The two reported offsets must differ.

The threshold is checked against rendered pixels, not a calibrated real-world
distance. The initial gripper marker placement was partly hidden by the modeled
jaw; the test mount was moved above the fixed housing before the measured run.
Tag mounts and camera position are illustrative and are not registered to the
user's physical setup. Ideal rendered lighting does not reproduce motion blur,
paper curl, glare, print distortion or network delay. This test validates
perception, data flow and Gemma tool use, not physical grasp/contact success.

Results are recorded in [simulation evidence](evidence/gemma-apriltag-simulation.json).
The complete local output includes the scene, raw structured observations,
annotated before/after/failure images and `apriltag-simulation.gif`.

Implementation references: [MuJoCo Python](https://mujoco.readthedocs.io/en/stable/python.html)
and [MJCF geometry](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom).
