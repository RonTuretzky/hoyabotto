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
units, and maps those values to the simulator's radians/zero offsets. That
simulator conversion is geometric and does not describe the hardware driver.
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
an independent 200 ms input watchdog. `../upstream_hardware.py` uses the existing
pinned mTLS velocity protocol and retains its rate limits and rail hold-to-run.
Browser focus, neutral start, explicit Stop, reader freshness, and reconnect
gates remain in the bridge. This is source reuse with documented adapters;
physical performance and full Windows/hardware parity remain unverified.

## Original hardware units (corrected after initial inactive installation)

The pinned XLeRobot example leaves `use_degrees=False`. Its driver therefore
uses `RANGE_M100_100` for arm/head joints and `RANGE_0_100` for grippers. Values
labelled degrees inside its controller are passed directly to that driver;
they are not geometric degrees at the motor boundary. Gripper target 90 means
90 percent of saved travel. The previous measured-angle physical adapter is
an explicit geometric alternative, not native-unit parity.

`native-units-manifest.json` pins complete source snapshots for the original
XLeRobot config/driver and LeRobot 0.6.1 motors bus/Feetech driver. LeRobot's
Apache 2.0 license is bundled as `LeRobot-LICENSE`. The two original conversion
methods and enum are extracted unchanged (AST-equivalent) into the owner-side
`joycon_native_units.py`. No serial/device constructor or driver import is
included. The Mac test environment's installed 0.6.1 motors-bus source matches
that pinned source byte-for-byte.

`NativeReference` applies those methods using exact saved ranges, homing
registers and drive_mode. Both ends bind the same source/calibration hash;
the sole owner also checks saved calibration against hardware. Native mode
needs no invented joint zero, direction, or fully-open/closed geometry.
Geometric references from the simulator or another IK model are not used.

Physical adaptation still bounds targets 40 ticks inside travel, retains the
existing speed limits, and suppresses up to two ticks of normalization rounding
at both target feedback and rate conversion so neutral input cannot creep.
This small deadband is an explicit adapter difference. Initial pose must fit
the original IK branch; an incompatible pose is rejected before claiming control.
The original automatic startup homing remains excluded.

Revision `04dd3ae` was installed inactive with a launch lock on Neooooo. The
native-unit correction postdates that package; it must be installed separately
before using native mode. No physical test has been performed.
