# Rendered calibration and use, end to end

The 2026-10-06 run passed the full software path on a MuJoCo-rendered scene:

1. `CalibrationRobot` dispatches automatic registration through `GemmaTransport`.
2. The existing controller moves the simulated arm and detects tags in fresh
   1920×1440 rendered RGB frames, bracketed by quantized simulated encoders.
3. Eight fit and three held-out poses go through the production sampler,
   verified LeRobot SO101 forward kinematics and OpenCV hand-eye fitter.
4. The installed registration feeds `robot_get_registered_tags`, including its
   stream, calibration, mounting, anchor and current-gripper consistency checks.
5. Tag 3's returned pose plus a declared tag-to-handle CAD offset selects the
   handle target. The existing simulator's robot-only IK and contact physics
   execute approach, close, lift, hold, lower, release and withdraw.

No ground-truth paddle position is supplied to the target calculation or IK.
Independent simulated object positions and contact pairs score the result.
This replaces the earlier disconnected detector test and fixed-target grasp.

## Results and limits

| Check | Result |
| --- | --- |
| Held-out registration translation RMS / maximum | 0.637 / 0.885 mm |
| Held-out registration orientation maximum | 0.107 degrees |
| Camera-transform error against independent simulator truth | 8.60 mm / 0.928 degrees |
| Handle estimate, nominal / moved paddle | 6.94 / 6.87 mm error |
| Nominal lift / two-second hold | 64.50 / 64.63 mm minimum paddle-bottom height above table at endpoints |
| Paddle moved +5 mm in base X, +15 mm in base Y | New tag-derived target; lift, hold and release passed |
| Open-jaw negative control | Failed to lift, as expected |
| Calibration arm/table penetration | 0 mm |
| Largest arm/table penetration during use | 0.067 mm |

A low held-out residual does **not** imply the same absolute position accuracy:
both fitted transforms can retain bias. The independent 7 mm handle error and
8.6 mm camera-origin error are recorded, not replaced by the smaller fit error.

The rendered grid's decoded tag axes differ from its MJCF body axes by a
180-degree Y rotation. The declared paddle offset therefore uses
`[105, 0, 4.45]` mm in the decoded tag frame, derived from the CAD handle at
`[30, 0, 3]` mm and tag plane at `[135, 0, 7.45]` mm. The original sign convention
produced a 216 mm error; that diagnostic is retained locally. This offset is
specific to this simulated mount and must not be used as a physical measurement.

Station/camera placement, 30 g mass and friction 0.8 remain assumptions. MuJoCo
uses compliant contacts; the paddle bottom briefly penetrated the table by
about 2.8 mm during closing, so the result does not certify physical contact
forces or clearances. Only one arm has dynamics; the other telemetry rows,
servo load/status, torque-release acknowledgements and watchdog are fixtures.
The live camera previously streamed at 640×360, so this higher-resolution
simulation does not establish its physical pose accuracy. No physical motors
were commanded and no physical grasp or registration is validated here.

## Reproduce

Use the existing `gemma-xlerobot` simulation directory containing
`paddle_sim.py`, its `scene-assets/`, and `real-scene/measurement.json`. Supply
the existing verified SO101 model directory. This reuses those assets rather
than creating a replacement robot model; their provenance is saved in the run.

```sh
cd /path/to/xlerobot-farm/software
uv pip install --python .venv/bin/python -r requirements-gemma-calibration.txt \
  mujoco==3.14.0 scipy==1.18.1 pillow
PYTHONPATH=. .venv/bin/python tools/simulate_tag_calibration.py \
  --simulation-root /absolute/path/to/gemma-xlerobot \
  --model-directory /absolute/path/to/verified-so101-model \
  --out /absolute/path/to/a-new-evidence-directory
```

The script refuses an existing output directory and has no hardware backend.
It saves registration captures, rendered images, accepted fit, registered
observations, per-step contact results, JSON summary and an animated GIF.
Each simulator instance has a unique camera-stream identity so registrations
cannot silently carry over to a different instance.

The compact checked-in [evidence](evidence/gemma-calibration-e2e.json) points
to the retained full local run. The production Gemma installation and live
read-only catalog check are recorded there separately from the simulated use.
