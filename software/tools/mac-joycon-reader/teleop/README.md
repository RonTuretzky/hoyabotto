# Joy-Con practice and prepared teleoperation

**2026-10-08: the user instructed us not to change the remote robot or activate its motors.**
The code in this branch is prepared locally. No teleoperation code was deployed,
no owner/API was restarted, and no motor activation or movement was requested.
Physical teleoperation has **not** been validated.

## Original controls in the local simulator

`run-simulator.command` defaults to `--control-mode upstream --input-backend hid`.
This **reuses** the Windows `JoyConController` and tilt filter plus XLeRobot's
`SimpleTeleopArm`, `SimpleHeadControl`, base mapping and SO101 inverse kinematics.
[Exact sources and adapter differences](vendor/PROVENANCE.md) include pinned
commits, preserved original files and licenses. No Windows drivers are needed.

1. Pair both original Joy-Cons with this Mac.
2. Launch `run-simulator.command`. Set them down for 2–3 seconds to calibrate.
3. Pick them up comfortably, release buttons, center sticks and click **Start practice**.
   Gyro is on by default. Start sets the current wrist center and keeps the
   simulated robot at its present pose; it does not home.
4. Use the controls below. **Stop**, **−**, Escape, controller loss or leaving
   the page ends the session. Start again explicitly to resume.

| Input | Behavior |
| --- | --- |
| Each stick | Move that hand relative to the controller's orientation |
| Upper L / R | Raise the corresponding hand |
| Click a stick | Lower that hand |
| Tilt each Joy-Con | Bend and roll that wrist while gyro is on |
| Tap ZL / ZR | Toggle that gripper open / closed |
| Left D-pad | Turn and tilt the head |
| X / B | Move the right hand forward / back in the fixed frame |
| Hold + with Y / A | Drive forward / back; release + to brake |
| Hold + with X / B | Turn left / right instead of moving the right hand |
| Capture / Home | Return left / right hand position to where it started |
| −, Escape, Stop | End practice |

This is the Windows movement controller plus the **direct** XLeRobot teleop
example. It does not use the separate `_smooth` example's 2-degree/s arm ramp.
The shoulder buttons raise the arms; they are no longer hold-to-run controls.
The Windows gripper toggle differs from XLeRobot's Linux hold-to-open mapping.
The Plus drive modifier resolves the two originals' conflicting X/B mappings.

The adapter runs movement at 50 Hz (3 mm per upstream call, up to 15 cm/s for a
single stick direction). Multiple simultaneous movement inputs can add together,
as in the original. The original tilt estimator runs at 100 Hz with its original
gains. XLeRobot's pose gains also remain; wrist angle is not one-to-one with your
hand. Heading can drift, and an independent wrist yaw is unavailable on this
five-axis arm. Translation comes from sticks/buttons, not tracking the Joy-Con's
physical position in the room. Start re-centers wrists; Home/Capture resets hand
position only. Model joint bounds limit travel.

The Mac HID adapter keeps factory/user stick and IMU calibration and estimates
resting gyro bias from 200 stable samples. It normalizes the left sensor's Y/Z
axes and feeds the original Windows gravity/gyro filter. It permits only volatile
report/IMU configuration and SPI reads, never pairing, firmware writes or rumble.
Diagnostics show calibration and live input. Missing/old IMU data or a changed
sensor session stops practice. The original filter needs a further half-second
to settle before Start is enabled.

`upstream_simulator.py` consumes original position actions directly in MuJoCo.
It has its own 200 ms command watchdog. The old physical owner's tick-rate
limits remain untouched. This mode is rejected with `--connect-robot` and with
Apple-only input. The local UI requires focus and fresh input.

### Legacy local modes

`run-simulator.command --control-mode cartesian` retains the previous custom
Jacobian solver and shoulder hold-to-run controls. Add `--input-backend auto`
for its optional Apple reader. `--control-mode joint --input-backend apple`
retains the original three joint layers. Their on-screen guides show their own
mappings; neither is the default anymore.

### Verification boundary

Offline tests compare reused method ASTs to the pinned original sources; replay
mirrored left/right sensor data through the original gravity filter; and exercise
Start without homing, wrists, lift/lower, gripper toggles, Home/Capture, head,
drive modifier, invalid targets, reconnect, focus loss and watchdog expiry.
The wrist test requests 30 degrees through the complete controller/IK/MuJoCo
chain and requires the measured joint change to be within 2 degrees after one
simulated second. It does not equate a 30-degree human tilt with a 30-degree
robot response. Tests forbid network sockets. Live operator feel and physical
robot calibration/movement remain separate, unvalidated steps.

On 2026-10-08 the complete current suite passed **38 tests**. The updated local
browser showed a live MuJoCo scene with no renderer error and practice stopped.
Both Joy-Cons were disconnected at handoff, so this build's live tilt/button
feel still requires an operator check after reconnecting. No remote robot was
contacted or activated.

## MuJoCo 3D simulator

Double-click `../run-simulator.command` for the live 3D XLeRobot in MuJoCo.
This launcher rejects robot-connection flags. It uses the existing local Python
3.12 environment with MuJoCo 3.14.0 and the existing full XLeRobot mesh model.
No remote service or motor connection is used. Orbit, front, side and top views
are available beside the whole-body Joy-Con controls; rendered frames stay on localhost.

The default upstream mode drives MuJoCo position actions directly; legacy modes
exercise the guarded register owner. Both use simulated position actuators and
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
controls use sticks and buttons. The default MuJoCo HID backend is described above. Battery telemetry is not relied upon.

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

## Legacy verification records

The counts and controller observations below record earlier builds, before the
upstream port. Run the full command below for the current suite.

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
