# Registering the analytical arm convention

The legacy SO101 model uses **degrees**; the adapter uses calibrated normalized positions, −100..100 for positioning joints. `AnalyticalReference` bridges degrees → raw encoder ticks → normalized positions, and reverses those conversions before FK. Gripper values remain 0..100. This is a local offline conversion facility; physical `SkillRunner` Cartesian execution still refuses without commissioned frames and a checked workspace/collision path.

From `software/`, create an **unfilled** identity-bound reference:

```sh
python -m farm.kinematics.analytical_reference template --arm right --calibration /absolute/path/to/calibration.json --out /absolute/path/to/reference.json
python -m farm.kinematics.analytical_reference validate --reference /absolute/path/to/reference.json --calibration /absolute/path/to/calibration.json
```

The first command refuses to overwrite an existing file. The second deliberately refuses the blank template. Neither command connects to motors or moves the arm.

Fill a unique `registration_id` and an `evidence` path or description of the actual measured setup. For **each of the five positioning joints**, record `reference_tick`, the independently known `reference_degrees` at that same physical pose, and measured `model_sign` (+1 if increasing raw ticks increases the analytical angle, −1 otherwise). All five values must come from physical measurements; calibration endpoints, homing offsets, range midpoints, current camera pixels and a convenient rest pose do not supply geometric zero angles. The record binds the exact saved calibration bytes and analytical source SHA. A model or calibration change requires a new reviewed registration. URDF and analytical references use different conventions and are not interchangeable.

One partial geometrical example, **not a motor target or a safe-pose claim**: in the analytical arm plane, upper-arm link axis vertically up (`theta1=90°`) and forearm link axis horizontally forward (`theta1+theta2−180°=0°`) imply shoulder lift ≈ −13.96796008° and elbow flex ≈ 16.17545169°. FK then yields `(x,y)=(0.1350,0.1159)` metres relative to the model's shoulder origin. Identify the actual link/joint axes with a full-arm view or independent fixture before using this example. It supplies neither raw ticks nor pan, wrist-flex, wrist-roll references. Pan/roll need independently declared axis/direction conventions; wrist flex also needs a measured tool pitch consistent with `wrist_flex = pitch − lift − elbow`.

Offline use after validation:

```python
reference = AnalyticalReference.load(reference_file, calibration_file)
model = ArmModel("right", reference=reference)
normalized_targets = model.joints_for(pose)
model.sync_from_joints(measured_normalized_joints)
```

No real reference currently exists merely because this code or template exists. Moving into a recorded pose first requires an actual recorded encoder pose with supported travel/clearance; the template cannot provide one. Station reach additionally needs measured arm-base/station and gripper/tool transforms, physical workspace and collision validation, and correctly converted physical rate/step limits. Those remain separate commissioning requirements.
