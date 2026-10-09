# Validation — 2026-10-04

Built and checked on Apple Silicon, macOS 26.6.2, using the installed Xcode Swift
toolchain. Deployment target: macOS 13. No robot libraries or motor ports opened.

## Completed

- `swift test`: **7 tests passed**. Apple's extended and micro controller
  snapshots exercised actual framework capture, button press/release, stick
  values and snapshot immutability. Additional checks cover unknown/combined
  identity, deadzone behavior, disconnect/reconnect lifecycle, shutdown and JSON.
- `./build-app.sh`: release executable and locally signed native app built.
  `codesign --verify --deep --strict` and `plutil -lint` passed.
- One-second, 30 Hz JSON smoke tests: 32 demo frames (2 synthetic profiles),
  33 live frames (**0 physical profiles detected on the build Mac**). Both had
  monotonically increasing sequence numbers, one session ID, explicit source
  labels, and a final shutdown frame with cleared inputs.
- Four malformed CLI cases (zero frequency, NaN duration, out-of-range deadzone,
  unknown option) exited with status 2 and no stdout stream.
- SIGTERM produced a shutdown frame. Closing the output pipe exited with status
  1 rather than hanging or silently reporting success.
- Native demo window inspected visually and via accessibility: distinct left and
  right cards, moving values, explicit DEMO label, pressed/released text.
- Native live window inspected: **Waiting for Joy-Cons**, zero profiles, and
  explicit input-only/no-robot-motion status.

## Not established

- Real Joy-Con button, stick, battery or gyro input on either Mac.
- Independent motion streams for both controllers if macOS combines the pair.
- Installation on the robot-connected Mac. A direct SSH probe timed out.
- End-to-end latency, Bluetooth-loss detection timing, or any robot movement.

The user acceptance procedure is in README.md. Software tests and synthetic
demo input are not physical hardware validation.
