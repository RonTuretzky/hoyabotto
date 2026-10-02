# Where the build stands

Written 2026-10-02 for whoever picks this up on the robot's laptop (person or agent). Read this first, then `SETUP.md` (installing) and `README.md` (what the program is). `docs/community-projects.md` reviews the 43 projects on the XLeRobot community page against this plan.

## The robot

- **Kit:** WowRobo XLeRobot 0.4.0 two-wheel Combo: two SO-101 follower arms (12 V STS3215 servos), head with two servos, two drive wheels, three bare USB camera boards, two motor control boards, IKEA cart.
- **Assembly:** built to the end of the official 0.4.0 video (https://www.youtube.com/watch?v=4bXCFw57T60). The older 0.3.0 video and docs are for the three-wheel version and do not match this kit.
- **Orientation:** as in the video. The top base sits against the drive-wheel side of the top tray; arms and head camera face outward over that edge; motor boards and cables are at the back (caster side). The official 0.3.0 render shows the opposite (arms facing inward); the 0.4.0 arm bases can be re-clocked either way. Outward is what the farm needs: trays sit on a separate surface in front of the arms.
- **Base:** parked. The profile has `wheels: false`; nothing in the farm program drives the wheels.

## What is done and verified

| Item | State |
|---|---|
| Loose servo IDs | Set and read back on the real servos: head 7 and 8, wheels 9 and 10 (`farm set-motor-id`) |
| Arm servos | One arm probed: IDs 1-6 all answer (STS3215). The other arm was not probed on its own. |
| Motor power | A USB-C-to-12 V cable gave about 12.5 V on the bus |
| Software | Fresh clone installs and passes 47 tests; simulator runs end to end |
| LLM backends | Claude CLI and OpenRouter (Jev, Astra) both answered from the first laptop |

## What is not done

| Item | State |
|---|---|
| Full-robot probe | Not run since final wiring. Expect one board with IDs 1-8 and one with 1-6, 9, 10 |
| Profile | `profiles/paper-tray-v0.yaml` has empty `port1`, `port2` and no camera indices |
| Calibration | Not done. Nothing can move until it is |
| Taught poses | None (`data/keyframes.yaml` does not exist) |
| Cameras | Never opened from software. They need 4-pin-to-USB cables; whether the kit included them was an open question |
| Light sensor | ESP32 + BH1750 not wired or flashed |
| Printed parts | Cress planter, nests, tag tiles, bottle rest, paddle: print status unknown |
| Head naming | Pan = 7, tilt = 8 is assumed, not checked on the hardware. RoboCrew's XLeRobot driver uses the same mapping (yaw 7, pitch 8). `farm robot-test --move --ask --only head` checks it |
| Wheel sides | Left = 9, right = 10 is assumed, not checked (unused while parked) |

## Hardware facts worth keeping

- **Motor boards (serial → macOS port):** `5B790182091` → `/dev/cu.usbmodem5B790182091`; `5B790186401` → `/dev/cu.usbmodem5B790186401`. Which one ended up on the left arm is decided by the probe, not by the name.
- **Left vs right:** the arms are identical. The left arm is the one whose board also carries the head servos (7, 8); the right arm's board carries the wheel servos (9, 10). Left and right are from the robot's own point of view.
- **3-pin wires are servo wires and carry 12 V.** They go board → servo → servo. Never plug one into a camera.
- **Cameras:** each has a 4-pin USB port and its own cable to the hub. They do not connect to the arms.
- **Wiring to the laptop:** both motor boards (USB-C) and all three cameras go into one USB hub; the hub's main cable goes to the laptop. Each motor board gets its own 12 V feed (USB-C-to-12 V cable from the battery). Switch 12 V off before moving any servo wire.

## Next steps, in order

Run from Terminal (camera permission is per app), inside `software/` with the environment active.

1. `farm devices --probe`: confirm both boards and their IDs, and look at the camera snapshots in `data/devices/`.
2. Edit `profiles/paper-tray-v0.yaml`: `port1` = the board with IDs 1-8, `port2` = the board with 9 and 10. For each camera add `index_or_path: <n>` using the snapshot that shows the matching view.
3. `farm calibrate`: support both arms and follow the prompts. One-time. Then `farm calibration-report`: reads the saved file and flags a wrapped reading, a short sweep, or arms that disagree, before anything moves.
4. `farm robot-test`, then `farm robot-test --move --ask`: motors only (no cameras, no models, no trays). The first reads every joint, temperature and load. The second nudges one joint at a time and asks whether the named part moved; this catches swapped left/right boards and swapped head motors. Start with the arms folded; the motors go limp when it ends.
5. `farm check`: connects everything and has the vision model confirm which camera is which.
6. Put the bottle, paddle and trays in their fixed places, then `farm teach-all`.
7. `farm once --tray B` with an empty bottle; authorize from the viewer (http://localhost:8765).
8. `farm cup-test --tilt 25 --seconds 1.5 --who <name>`.
9. `farm run --every 3600 --record`.

Stop at any point with the red STOP button in the viewer.

## Rules that were decided

- Nobody drives the robot by hand. Poses are taught by the vision model (`farm teach`, `farm teach-all`); people only answer questions in the viewer or over Telegram.
- Right arm holds the bottle, left arm holds the light paddle.
- Trays, bottle rest and paddle rest stay in fixed positions; everything taught is relative to that layout.
- Water only moves when the rules pass and either a named person or Jev at `approve` level authorizes it. A pour with an unknown result blocks further cycles until someone reconciles it.

## Not on this laptop

These stayed on the first laptop and are not needed to run the robot: the trained ACT test checkpoint (`data-train/`, a motion prior only), the 3D-print files and print handoffs, and the site build output. The OpenRouter key must be typed into `.env` again; make a fresh one, since the old one was pasted into a chat.
