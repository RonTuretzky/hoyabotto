# Mac Joy-Con reader

Native macOS input reader for original Nintendo Switch Joy-Cons. Uses Apple's
GameController framework and has no third-party packages. This is the input
stage of manual robot control: it **does not connect to motors**, call an AI
service, calibrate the robot, or move anything. It can run alongside the carton
pilot while controller inputs are being checked.

## Run on the Mac paired with the Joy-Cons

Requires macOS 13 or later and Apple's Command Line Tools or Xcode. If the Swift
compiler is missing, run `xcode-select --install` and complete Apple's installer.

1. In **System Settings → Bluetooth**, connect **Joy-Con (L)** and **Joy-Con (R)**.
   Hold each controller's small SYNC button between SL and SR until lights flash.
2. From the repository's `software` directory:

   ```sh
   ./tools/mac-joycon-reader/run.command
   ```

   The first launch compiles the reader; subsequent builds reuse that output.
   You can also double-click `run.command` in Finder.
3. Press every button, move each stick to its limits and release it, then rotate
   each controller. Check that values change and releases return to neutral.
   The display identifies left/right by the controller name macOS supplies;
   ambiguous identities remain **UNKNOWN**. It shows other recognized gamepads
   too, to help diagnose what macOS is actually exposing.
4. Look for motion update counts increasing and changing quaternion/rotation
   values. “Connected” or “Motion enabled” alone does not establish working
   inputs. Battery and unsupported motion fields are explicitly unavailable.
5. Disconnect a controller in Bluetooth settings. Its held inputs must disappear.
   Reconnect and verify input again. Close the reader window or press Command-Q
   when finished.

The UI labels pressed/released states with text and borders, not color alone.
It does not install a driver or request Accessibility permission.

## Build a normal app

```sh
./tools/mac-joycon-reader/build-app.sh
open './tools/mac-joycon-reader/dist/Joy-Con Reader.app'
```

The app is built for the current Mac's architecture and signed locally for
development, not notarized for public distribution. Build from source on the
other Mac using the same commands. A built app runs without Python or Swift
installed; the compiler is needed only to build it.

## Inspect the stream or UI without hardware

```sh
./tools/mac-joycon-reader/run.command --demo
./tools/mac-joycon-reader/run.command --json --demo --seconds 2
```

Demo frames always say `source: "demo"`; the demo never opens controller APIs.
Do not treat a demo as evidence that the real Joy-Cons work.

## Live stream for a future robot adapter

```sh
./tools/mac-joycon-reader/run.command --json --hz 30 --deadzone 0.08
```

JSON Lines go to stdout, build/errors to stderr. Ctrl-C/SIGTERM or `--seconds N`
ends the reader with a `shutdown` frame. Nothing is saved unless you redirect
stdout. There is no network server.

Frame contract (version 1):

- `schema_version`, `input_only: true`, `source: "live" | "demo"`.
- `session_id` changes each reader run. `sequence` increases for every frame.
- `timestamp` is Unix seconds; `uptime` is seconds from this Mac's monotonic clock.
- `event`: `sample`, `disconnect`, or `shutdown`.
- `controllers` contains every connected profile. On disconnect, that frame also
  contains one tombstone with the old `id`, `connected: false`, empty buttons,
  axes and pads, and no motion. Later frames omit it. On shutdown all profiles
  are tombstones. A reconnect gets a new ID; IDs are not hardware serial numbers.
- `role` is `left`, `right`, `pair`, or `unknown`, inferred from the reported name.
  It is a diagnostic hint, not a validated assignment to a robot arm.
- `buttons` retains Apple's element names, values, pressed states and available
  physical-name mappings. Nintendo's printed A/B/X/Y labels can differ from
  Apple's logical aliases. Do not use those aliases as a robot map unverified.
- `axes` and `pads` contain both `raw` and `filtered` values. Filtering is a
  rescaled per-axis deadzone; ranges are -1 to 1. D-pads are reported alongside
  sticks. The reader preserves macOS orientation rather than guessing how a
  sideways Joy-Con is being held.
- `motion` reports available quaternion attitude, rotation rate (rad/s), and
  acceleration vectors (g). Unsupported optional fields are omitted. Sensor
  activation is restored on normal shutdown/disconnect. No reset is sent.
- `input_event_count` and `last_input_event_uptime` track input-change callbacks;
  motion has its own event count/time. **An unchanged stick held still need not
  generate events. These times are not Bluetooth packet timestamps or a link
  watchdog.** A sampling heartbeat only proves the reader process is alive.

## macOS combined pairs and troubleshooting

macOS may expose a combined pair instead of two individual profiles. The reader
shows what the OS actually supplies; it does not invent two independent motion
streams from one. Independent wrist orientation remains unverified until both
controllers' data is observed. If the OS only exposes one motion stream, a
separate raw-HID backend would be needed for independent gyro control. Buttons
and both sticks can still be inspected in the combined profile.

If the reader is waiting, confirm pairing on **this Mac**, wake the Joy-Cons with
a button press, and try disconnecting/reconnecting them. The reader monitors
controller events while backgrounded. No Linux `joycond`, `apt`, or `make install`
step applies here. Check System Settings → Game Controllers for custom mappings.
No permission bypass or unsigned kernel extension is needed for this backend.

Before connecting these inputs to robot motion, the adapter must verify the
button/axis mapping, explicitly select left/right devices, preserve the current
pose, and take over from the carton pilot through the existing motor owner. It
also needs hold-to-run, bounded increments, wheel-stop on release, and a watchdog
that stops on reader loss/EOF, controller loss, or loss of valid input. This
reader deliberately has no motor-command path or stock startup zero-pose move.

## Validation and sources

```sh
cd tools/mac-joycon-reader
swift test
```

Tests use Apple's writable controller snapshots to exercise the real capture
path, plus connection lifecycle and the JSON contract. They are not physical
controller tests. See `VALIDATION.md` for the checked environment and results.

- [Apple controller discovery and profiles](https://developer.apple.com/documentation/gamecontroller/gccontroller)
- [Apple physical input snapshots and remapping](https://developer.apple.com/documentation/gamecontroller/gcphysicalinputprofile)
- [Apple motion capability checks](https://developer.apple.com/documentation/gamecontroller/gcmotion)
- [Apple background events](https://developer.apple.com/documentation/gamecontroller/gccontroller/shouldmonitorbackgroundevents)
- [XLeRobot upstream two-wheel reference](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/7_xlerobot_2wheels_teleop_joycon_smooth.py)

The upstream robot script is a mapping reference only. Do not launch it alongside
the robot's existing controller: it opens motor buses and commands a zero pose.

## Prepared teleoperation

The [local teleoperation preview](teleop/README.md) adds control mappings for both arms, grippers, head and driving. `run-teleop.command` defaults to local practice with virtual joints and a virtual base, with no robot connection. Robot-side integration remains local and undeployed.

For the full 3D MuJoCo model, use `run-simulator.command`. It opens the same controls with live robot rendering and physics feedback, entirely on this Mac. See [simulator setup and limits](teleop/README.md#mujoco-3d-simulator).
