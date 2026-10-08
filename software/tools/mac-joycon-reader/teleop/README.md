# Joy-Con practice and prepared teleoperation

**2026-10-08: the user instructed us not to change the remote robot or activate its motors.**
The code in this branch is prepared locally. No teleoperation code was deployed,
no owner/API was restarted, and no motor activation or movement was requested.
Physical teleoperation has **not** been validated.

## Local preview

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
motion sensor appeared in the observed profile; these controls use sticks and
buttons. Battery telemetry is not relied upon.

## Prepared mapping

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

Seven native tests and eleven local mapping/bridge tests passed. All 20 files in the
robot deployment's fake-hardware suite passed, including new tests for all 16
fake servos, scope exclusion, neutral gating, dropped/expired/replayed inputs,
wheel braking, camera loss, partial wheel startup failure and retry after a
failed wheel release. The six bridge tests exercise every virtual joint through
the complete local mapping → API → file mailbox → owner path, both driving
wheels, input timeout, focus loss, disconnect, STOP during session startup, and rejection of a competing client without
cancelling the manual session.
They fail if the simulated robot opens a socket. The local HTTP preview was
also started with a nonexistent credential file; real arm requests are rejected.

The practice interface was opened and visually checked in Chrome. The native
reader again detected the real paired Joy-Cons and the left trigger check passed.
The new screen has not yet been fully exercised with physical controller input
across all modes. Synthetic test inputs are distinct from that operator check.
No physical motion, end-to-end live network timing, gripper direction, head
axis direction or physical stopping distance has been verified.
