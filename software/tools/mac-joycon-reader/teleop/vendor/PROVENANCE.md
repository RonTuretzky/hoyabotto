# Reused upstream control code

These are extracted definitions, not rewritten implementations. `manifest.json`
pins each repository commit, original path, source SHA-256, and extracted names.
`sources/*.txt` preserves the corresponding complete original source bytes for
review and offline AST comparisons. Tests compare every imported method with
its pinned source. The original entrypoints are never imported or executed.

- [box2ai-robotics/joycon-robotics](https://github.com/box2ai-robotics/joycon-robotics/tree/3e37ebfdf1db88fe4ecaa7018ae9c8c49efcde0e/hidapi_for_windows), MIT (`joycon-robotics-LICENSE`).
  `windows_attitude.py`: original reader constructor, attitude filter, state getter.
  `windows_controller.py`: complete original `JoyConController`.
- [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot/tree/b017b5e6354bd9f61f4247a920c72622ca0aade0/software), Apache 2.0 (`XLeRobot-LICENSE`).
  `xlerobot_control.py`: arm/head constructors, pose handlers, proportional
  position actions, joint maps and base button mapping from
  `7_xlerobot_2wheels_teleop_joycon.py` (the direct version, not `_smooth`).
  `so101_kinematics.py`: constructor, inverse and forward kinematics.
  No NOTICE files exist in that pinned repository tree.

Changes to module scaffolding: remove physical drivers, connection routines,
entrypoints, calibration threads, plotting, and unused imports. Retain required
math/numpy/threading imports. Route XLeRobot's per-frame `print` calls through
module-local debug logging so they cannot corrupt the HID JSON stream. No
extracted method body is modified. No homing method is included.

## Adapter boundaries

`../hid_reader.py` owns macOS HID transport, SPI calibration, stationary gyro
bias estimation, and reconnection. It normalizes left Y/Z signs, matching the
original Joy-Con wrapper, and feeds calibrated acceleration/rate to the original
Windows filter every two 5 ms IMU samples (100 Hz, its declared `dt=0.01`).
It does not use the original fixed stick center or its Windows Bluetooth setup.

`../upstream.py` emits logical joint values through a destination interface. It calls `JoyConController.get_control()` at 50 Hz, then
`SimpleTeleopArm.handle_joycon_input()` and `p_control_action()`. That Windows
controller advances 3 mm per call; upstream's demo has no fixed control cadence,
so our fixed cadence is an explicit integration choice. The original XLeRobot
pose gains are retained: roll * 45 degrees, pitch * -60 + 10 degrees, lateral
position * 250 degrees/meter. Combined with the Windows controller's pitch gain,
these are not one-to-one human wrist angles. Yaw affects Windows stick directions;
XLeRobot's five-axis arm does not reproduce independent wrist yaw.

The Windows demo is right-hand-only. The adapter mirrors the controls to the
left using L/ZL/click/Capture, maps its open/closed gripper scalars to 90/0 logical
degrees, and maps logical joint degrees to the simulator's radians/zero offsets.
The default arms face +X toward the workbench (left pan -pi/2, right +pi/2);
logical shoulder pan has the opposite sign to the model hinge. Measured
forward/sideways/up tests cover both arms and both directions.
Start captures current simulated pose and wrist center; it never homes. Head
uses the original D-pad handler. Holding Plus selects the original XLeRobot base
button function, preventing its X/B turn buttons from also moving the right
hand. Release Plus brakes. Base interface speeds are 0.1 m/s and 30 degrees/s.

Target bounds follow the model. The adapter writes a bounded position back to
Windows' `set_position` to avoid accumulated unreachable hand travel. For that
readback/start conversion, `position_for_upstream_ik` reverses the *inverse*
kinematics equations: the supplied upstream forward method uses a different
elbow convention and does not round-trip (about 32 degrees of elbow discrepancy
at the initial pose). That method is preserved for comparison but is not used.

`../upstream_simulator.py` is a separate local MuJoCo position-action sink with
an independent 200 ms input watchdog. The physical adapter in `../upstream_hardware.py` uses the existing pinned mTLS
velocity protocol and retains its rate limits. Its opt-in owner mode requires a
matching measured reference and physical rail hold-to-run buttons. It has only
been exercised against fake motors; no remote installation has been performed. Browser focus, neutral start, explicit
Stop, reader freshness, and reconnect gates remain in the bridge. This port is
source reuse with documented adapters, not full Windows/hardware parity.
