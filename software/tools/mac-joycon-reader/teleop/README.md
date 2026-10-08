# Joy-Con practice and prepared teleoperation

**2026-10-08: the user instructed us not to change the remote robot or activate its motors.**
The code in this branch is prepared locally. No teleoperation code was deployed,
no owner/API was restarted, and no motor activation or movement was requested.
Physical teleoperation has **not** been validated.

## Hand-space and whole-body practice

`run-simulator.command` now defaults to **Cartesian / whole-body** practice.
The original joint-layer mode remains available with `--control-mode joint`.
This new mode is explicitly **simulation-only** in both the bridge and owner;
no physical kinematic calibration or deployment is implied.

| Input | Behavior (in one session) |
| --- | --- |
| Hold upper **L / R** | Enable the matching arm; release stops it |
| Each stick up/down, left/right | Move that gripper forward/back, sideways using coordinated arm joints |
| Hold stick click + stick up/down | Raise/lower that gripper |
| **ZL / ZR** while corresponding L/R is held | Move the gripper; each new press reverses open/close direction |
| Left D-pad + L | Head pan/tilt |
| **Y / A**, both L and R held | Drive forward/back |
| **X / B**, both L and R held | Turn left/right |
| Hold **+** + stick + matching L/R | Wrist pitch/roll when gyro is off |
| **Home**, centered controls, both L and R held | Bounded return of arm/head/gripper joints to this session's starting pose; release L/R or command a move to cancel |
| **−**, Escape, Stop, input loss, page blur | End the local session |

The UI displays live X/Y stick values and held button names for mapping checks.
The raw HID reader uses Nintendo's printed button labels. Verify the displayed
button names when using Apple's controller profile; OS remapping is rejected.

`Stick directions` selects robot-relative axes or directions relative to each
gripper. XYZ + wrist pitch/roll are solved with a damped MuJoCo Jacobian against
live simulated positions. SO101 has five arm axes: arbitrary independent XYZ,
roll, pitch **and** yaw are not all simultaneously achievable. Targets have a
15 mm lead bound, joint-rate limits remain 80 ticks/s (head 60), and unreachable
motion is bounded rather than accumulated. Arm and base rates ramp while held;
releasing hold-to-run buttons commands zero immediately. Wheels retain 2 cm/s
limits. The solver is not a collision-free planner.

### Independent gyro input

The launcher selects the HID reader when both original Nintendo controllers
are visible; otherwise it uses Apple GameController. While practice is stopped,
use **Controller reader → Independent Joy-Cons + gyro** to try the raw backend.
You can also run `run-simulator.command --input-backend hid`.

The new reader uses `hidapi` and only opens Nintendo IDs 057e:2006/2007. It
requests volatile 0x30 reports/IMU streaming, reads factory or user stick/IMU
calibration from SPI, and integrates all three 5 ms IMU samples per packet.
Each controller must remain still for 200 samples before its gyro is ready.
Enable the gyro checkbox while stopped. Each L/R press anchors that controller's
relative wrist orientation; release/repress to reanchor. A stale/missing gyro
or changed gyro session stops practice. Long-held yaw/orientation can drift;
there is no external tracking or absolute heading reference.

The HID output allowlist includes only report mode, IMU enable/sensitivity and
SPI **read** commands. No calibration/firmware writes, pairing changes or rumble
are implemented. Switching readers restarts only the local input process and
requires fresh hold-to-run checks.

Source references:
- [XLeRobot smooth Joy-Con teleop](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/7_xlerobot_2wheels_teleop_joycon_smooth.py)
- [Windows adapter controls](https://github.com/box2ai-robotics/joycon-robotics/blob/master/hidapi_for_windows/README_hidapi.md)
- [Nintendo report format](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md)
- [Stick/IMU calibration layout](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/spi_flash_notes.md)
- [IMU units](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md)

**Verification boundary:** 24 local Python tests passed, including measured
positive/negative XYZ travel for both hands, simultaneous arms/grippers/head/
wheels, wrist fallback, independent synthetic gyro input, gyro loss, bounded
pose return, neutral gating, disconnect/timeout and physical whole-body claim
rejection. All 20 existing fake-hardware suites passed. The browser reader selector and both movement-frame options were exercised.
HID parsing/calibration used protocol fixtures; no real Joy-Con was visible during the initial HID
probe. Independent physical gyro streams, printed button mapping and actual
robot motion are therefore not yet verified. This is not a claim of complete
physical Windows/Mac parity.

## MuJoCo 3D simulator

Double-click `../run-simulator.command` for the live 3D XLeRobot in MuJoCo.
This launcher rejects robot-connection flags. It uses the existing local Python
3.12 environment with MuJoCo 3.14.0 and the existing full XLeRobot mesh model.
No remote service or motor connection is used. Orbit, front, side and top views
are available beside the whole-body Joy-Con controls; rendered frames stay on localhost.

The Joy-Con mapping and guarded owner drive simulated position actuators and
wheel velocity actuators. Register readback comes from MuJoCo joint positions
and velocities after `mj_step`. The base is free to move on the floor through
wheel contacts; it is not repositioned with a scripted animation. The scene
includes a reference workbench and a free solid box.

The model is the existing local copy of
[MuJoCo-GS-Web's XLeRobot](https://github.com/Vector-Wangel/MuJoCo-GS-Web/tree/0d60421c6cd8b16525695f32d2f8c5ba1329d45d/assets/robots/xlerobot).
The local XML SHA-256 is reported in the simulator status. The source asset tree
is read without modification; scene additions are generated in memory.

For another installation, set `JOYCON_SIM_PYTHON` to a Python environment with
`teleop/requirements-simulator.txt` installed, and set `XLEROBOT_MUJOCO_MODEL`
to the full model's `xlerobot.xml` (its `assets` folder must be beside it).
The current defaults point to the already installed simulator under
`/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/`.
The [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)
provides the physics stepping and offscreen rendering.

**Model limits:** synthetic encoder calibration maps 100–4000 ticks to the
model's joint ranges. Disarmed joints hold their pose for practice. Voltage,
load and temperature fields are synthetic, not physical predictions. The
upstream cart geometry/wheel spacing is not a measured replica of the user's
robot. The solid box has no foldable flaps. This setup supports teleoperation
practice, not hardware calibration or validated carton folding.

Run the optional physics tests with the MuJoCo Python environment:

```sh
python -m unittest discover -s teleop -p 'test_mujoco_simulator.py' -v
```

Both integration tests passed: real MuJoCo feedback changed for all 14 position
joints in both directions; the wheeled base translated about 18 mm and turned
about 0.075 rad in the test; input timeout released the virtual session. The
renderer produced distinct orbit/top scene images. No robot network was allowed
in the controller-to-physics test.

## Lightweight local preview

Double-click `../run-teleop.command`. It builds the native reader and opens a
loopback operator screen. The default preview does not load robot credentials,
connect to Neooooo, or send physical robot commands. Real arming is disabled.

Press and release ZL and ZR, center both sticks, select components, then click
**Start practice**. You can move virtual arms, grippers, head joints and a virtual
base with the controls below. Gauges show all 14 position joints; the top-down
base view shows simulated driving. STOP, lost focus, input loss and disconnects
end the local practice session.

The simulator runs the actual prepared mapping, API/mailbox and motor-owner
logic against a memory-only bus in a temporary directory. It never loads robot
credentials or opens sockets to a robot. Its travel ranges and telemetry are
synthetic. It verifies control flow, not physical geometry, loads, collisions or
stopping distance. Browser refresh preserves the local UI session token.

The paired original Joy-Cons expose one Apple GameController profile named
`Nintendo Switch Joy-Con (L/R)`. Independent controller profiles, OS-remapped
profiles, unknown devices and demo frames cannot control the robot. No usable
motion sensor appeared in the observed Apple profile; the original joint-layer
controls use sticks and buttons. The new optional HID backend is described above. Battery telemetry is not relied upon.

## Original joint-layer mapping

| Component group | Hold-to-run | Stick X | Stick Y |
| --- | --- | --- | --- |
| Left arm | ZL | Left stick, selected layer | Left stick, selected layer |
| Right arm | ZR | Right stick, selected layer | Right stick, selected layer |
| Both arms | Respective trigger | Respective stick | Respective stick |
| Head | ZL | Head motor 1 | Head motor 2 |
| Drive | ZL **and** ZR | Right stick turns | Left stick forward/reverse |

Arm layers: **1** shoulder pan/lift, **2** wrist flex/elbow flex, **3** wrist
roll/gripper. Plus (`Button Menu`) cycles layers only with neutral sticks and
released triggers. Minus (`Button Options`), Escape or Stop ends the session.
The UI also selects a layer. Changing component groups requires stopping first.
Physical signs of arm/head motion still need an observed commissioning test.
This is joint-space control, not Cartesian hand tracking or gyro control.

## Prepared robot integration

The canonical sole serial owner gains an opt-in `--teleop` capability. The normal
restart retains its existing options; an operator must explicitly request
`--joycon-teleop` in the deploy script to enable this capability. The API gains
operator-only `/teleop/status`, `/teleop/claim`, `/teleop/input` and
`/teleop/release` routes behind the existing pinned mTLS client identity. They
are not model tools. A normal robot command cannot steal an active manual
session; STOP remains available.

The local bridge also needs the explicit `--connect-robot` option before it can
make any network connection. **Do not run a deployment or connected session
under the current user instruction.** This documentation describes prepared
code, not authorization to operate the robot.

An explicit Arm action requires a healthy, fully released owner, saved
calibration matching hardware, a fresh supervision camera, observed controller
triggers, and neutral controls. It primes each selected joint to its measured
position before enabling torque. Head control gets a separate manual scope;
it does not expand the AI's existing arm scope.

Streaming uses latest-value input, a unique session, increasing sequence,
one-use expiring server permits, and an independent owner watchdog. No packet
is retried and no fault automatically re-arms. Input expiry is 450 ms after
server receipt, with an owner poll-gap bound of 350 ms. These are software
checks, not certified braking-time guarantees; a hung process or failed bus can
prevent software from completing a stop. A physical stop remains necessary.

Arms are limited to 80 encoder ticks/s in the UI (100 enforced by the owner);
head to 60 ticks/s; goals stay inside saved limits. Release of an arm trigger
holds the measured pose. Session loss/fault ends control and invokes the
existing soft release. Ten idle seconds also ends the session. Driving uses
the existing mirrored wheel signs, 2 cm/s per-wheel cap, health checks, braking,
rest observation, and register restoration. Both triggers are required.
The local operator page must remain focused; closing it, leaving it, reader
exit, disconnected/replaced controllers or stale input ends control.

GameController reports connection state and input callbacks, not a timestamp
for every physical Bluetooth packet. Reader sampling time is not proof that a
new radio packet arrived. Bluetooth failure behavior remains a physical test.

## Local verification

From the reader directory:

```sh
swift test
python3 -m unittest discover -s teleop -p 'test_*.py'
```

From `software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge`:

```sh
python3 test_joycon_teleop.py
```

Seven native tests and eleven local mapping/bridge tests passed. With the MuJoCo
environment, the current expanded suite passes all 24 Python tests, including
hand-space/whole-body physics and HID protocol tests.
All 20 files in the
robot deployment's fake-hardware suite passed, including new tests for all 16
fake servos, scope exclusion, neutral gating, dropped/expired/replayed inputs,
wheel braking, camera loss, partial wheel startup failure and retry after a
failed wheel release. The six bridge tests exercise every virtual joint through
the complete local mapping → API → file mailbox → owner path, both driving
wheels, input timeout, focus loss, disconnect, STOP during session startup, and rejection of a competing client without
cancelling the manual session.
They fail if the simulated robot opens a socket. The local HTTP preview was
also started with a nonexistent credential file; real arm requests are rejected.

The practice interface and live MuJoCo scene were opened and visually checked
in Chrome. An earlier native-reader check detected the paired Joy-Cons and the
left trigger check passed; the latest check saw only the right controller.
Wake or reconnect the left controller to restore the combined pair. The new
screen has not yet been fully exercised with physical controller input
across all modes. Synthetic test inputs are distinct from that operator check.
No physical motion, end-to-end live network timing, gripper direction, head
axis direction or physical stopping distance has been verified.
