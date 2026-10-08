# XLeRobot MuJoCo model (vendored, unmodified)

Used by `farm/sim/xlerobot_twin.py` to render the robot's current pose. Rendering only: no physics,
no motors.

- Source: [Vector-Wangel/MuJoCo-GS-Web](https://github.com/Vector-Wangel/MuJoCo-GS-Web), commit
  `0d60421c6cd8b16525695f32d2f8c5ba1329d45d`, folder `assets/robots/xlerobot/` (the XLeRobot
  dual-arm model: IKEA Raskog cart, two SO-ARM arms, two-servo head).
- Licence: MIT, `LICENSE` here is that repository's licence file, copied verbatim (its copyright line
  names Konstantin Gredeskoul, the template it came from). The meshes come from the XLeRobot project
  (Apache-2.0, see `farm/vendor/LICENSE-XLeRobot`) and TheRobotStudio SO-ARM100 (Apache-2.0); both
  permit redistribution with attribution.
- Files: `xlerobot.xml` and `assets/*.stl`, 31 files, about 4.8 MB (1.7 MB compressed).
  `source.json` lists the sha256 of every file; they match the upstream commit byte for byte.

Nothing here is edited. At load time the twin removes the chassis free joint (base and wheels
fixed), drops the wheel tendons/actuators, adds a floor, lights and an offscreen buffer, colours
the left arm orange and the right arm blue, and adds an invisible site `twin_tip_L`/`twin_tip_R` to
each `Fixed_Jaw` body (the claw tip point `claw_positions` reports: where the two jaw tips meet
when closed, `TIP_POS` in `xlerobot_twin.py`).

Model facts the twin relies on (checked by printing the joint list):

- The robot faces model `-x`. The `*_L` arm is on `-y`, the robot's left; `*_R` on `+y`.
  (The model's `left_wheel`/`Right_Arm_Camera` names disagree with this; the wheels are fixed, so
  it does not matter.)
- Arm joints use the old SO-ARM100 menagerie convention (`Rotation`, `Pitch`, `Elbow`,
  `Wrist_Pitch`, `Wrist_Roll`, `Jaw`), not the SO-101 `so101_new_calib` one. The per-joint
  offset/sign in `JOINT_TABLE` converts between them.
- The jaw is the SO-100 design (the robot has SO-101 jaws): closed at the joint's lower limit
  (-21.5 deg), open at the upper (100 deg).

To use another copy, point `XLEROBOT_TWIN_MODEL` at its `xlerobot.xml` (the meshes must sit in the
`assets/` folder next to it). If this folder is missing, copy it back from the upstream commit
above (`assets/robots/xlerobot/` plus `LICENSE`) and check the hashes against `source.json`.
