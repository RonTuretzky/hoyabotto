---
name: farm-bringup
description: Bring up the assembled XLeRobot from the laptop it is plugged into — find the motor boards and cameras, fill in the profile, calibrate, and run the motors-only test. Use when the user types /farm-bringup, or asks to "calibrate the robot", "bring up the robot", "check the robot is wired right", or "what do I do next on the robot".
---

# Farm bring-up

Take the user from "robot assembled and plugged in" to "calibrated, every joint confirmed, ready to teach poses". You run the commands that need no hands on the robot; the user does the parts that do. Stop at every step marked **pause**.

Read `software/STATUS.md` first. It says what is already verified and what is only assumed; do not re-derive it and do not contradict it without evidence from this session.

All commands run inside `software/` with the environment active:

```sh
cd software && . .venv/bin/activate
```

If `.venv` does not exist, follow `software/SETUP.md` first and come back.

## This robot (do not use numbers from other XLeRobot guides)

- WowRobo XLeRobot **0.4.0 two-wheel** kit. Guides written for the three-wheel 0.3.0 use different wheel IDs.
- Board A carries the **left arm (IDs 1-6) and head (7, 8)**. Board B carries the **right arm (IDs 1-6) and wheels (9, 10)**. Left and right are from the robot's point of view, looking the way the arms face.
- 3-pin wires are servo wires and carry 12 V. Cameras use separate 4-pin USB cables.
- The base stays parked. Never send wheel commands.

## Step 1 — Pre-flight (you)

```sh
git pull
python -m pytest -q          # expect: all passed
```

Then ask the user to confirm, and wait: both motor boards have 12 V (a light on each), both USB-C data cables and the cameras are plugged into the hub, the hub is plugged into this laptop, the arms are folded at rest.

## Step 2 — Probe (you)

```sh
farm devices --probe
```

Expected: two serial ports, one answering IDs 1-8 ("bus1 (left arm + head)"), one answering 1-6 plus 9 and 10 ("bus2 (right arm + wheels)"), and one snapshot per camera in `data/devices/`.

| What you see | Meaning | What to do |
|---|---|---|
| A port answers only 1-6 | Head or wheel servos are not on that chain | **Pause.** Ask the user to check the 3-pin wire from that board to the head (or down to the wheels), with 12 V off |
| Both ports show 7, 8, or both show 9, 10 | Chains are crossed | **Pause.** Report it; do not guess |
| An ID is missing inside 1-6 | A servo wire inside that arm is loose, or the board has no 12 V | **Pause.** Report which ID |
| No ports listed | Hub or data cable | Ask the user to plug one board straight into the laptop |
| "not authorized to capture video" | macOS camera permission | The user enables the terminal app under System Settings → Privacy & Security → Camera, then rerun |
| Fewer than three cameras | Cable or hub bandwidth | Ask the user to plug the missing camera straight into the laptop |

Never continue past a wrong probe. Calibrating with crossed buses writes a calibration that looks valid and is wrong.

## Step 3 — Fill in the profile (you)

Edit `profiles/paper-tray-v0.yaml`:

- `robot.port1` = the port that answered 1-8. `robot.port2` = the port that answered 9 and 10.
- For each camera, open its snapshot (Read the image) and decide which view it is: head (looks down at the work area from above), left wrist, right wrist (each looks along a gripper). Add `index_or_path: <index>` to that camera's entry. If two views cannot be told apart, **pause** and ask the user to wave a hand in front of one wrist camera, then rerun the probe.

Show the user the three lines you changed.

## Step 4 — Calibrate (the user, in their own Terminal window — pause)

`farm calibrate` waits for the ENTER key while the user's hands are on the robot, so it cannot run through you. Tell the user to open a second Terminal window and run:

```sh
cd xlerobot-farm/software && . .venv/bin/activate
farm calibrate
```

Explain what it will ask before they start. Motors are limp throughout; support each arm so it does not drop.

1. **"Move left arm and head motors to the middle of their range of motion and press ENTER"** — left arm pointing straight ahead, each joint about halfway between its two stops, gripper half open, wrist roll centred; head facing forward and level. Then ENTER.
2. **"Move all left arm and head joints sequentially through their entire ranges of motion… Press ENTER to stop"** — one joint at a time, slowly to one stop, then to the other: base rotation, shoulder, elbow, wrist tilt, wrist roll, gripper, head pan, head tilt. Watch the camera cables while turning the head. Then ENTER.
3. The same two prompts for the **right arm** (no head; the wheels are skipped).

It ends with `calibration saved: <path>`. Ask the user to paste the last lines.

Known trap: if a joint (usually wrist roll) was near the end of its travel at step 1, its reading wraps and the calibration is wrong. The fix is to switch 12 V off, put that joint at mid-travel, switch 12 V on, and run `farm calibrate` again. Do not try to correct it by turning the joint.

## Step 5 — Motors-only test (you, then the user watches)

```sh
farm robot-test
```

Reads every joint, temperature and load; nothing moves. Every row should have a number and the last line should be `ALL OK`.

Then the moving test. Tell the user first: arms folded at rest, hands clear, each joint will move a few degrees and return, and the motors go limp when it finishes. Because it asks a question after each joint, the user runs it in their Terminal window:

```sh
farm robot-test --move --ask
```

For each joint it prints the part that should move and asks whether that was the part that moved.

| Result | Meaning | What to do |
|---|---|---|
| "did not move" | No 12 V on that board, or the joint is against a stop | Report; ask the user to check power and reposition, then rerun with `--only head`, `--only left` or `--only right` |
| The user answers "n" for every left-arm joint and the right arm moved | `port1` and `port2` are swapped | Swap them in the profile, redo Step 4 |
| The head nodded when it should have turned (or the reverse) | Head servos 7 and 8 are in each other's place | **Pause.** Report it and record it in STATUS.md; do not rename anything on your own |
| "did not return" | Joint is loaded or binding | Report which joint; ask the user to look for a snagged cable |

## Step 6 — Whole-system check (you)

```sh
farm check
```

Connects motors, cameras and (if built) the light sensor, and has the vision model confirm which camera is which. Report the `problems:` line verbatim. A missing light sensor is expected until it is built.

## Step 7 — Write it down (you)

Update `software/STATUS.md`: move what was verified from "not done" to "done" (probe result, ports, camera indices, calibration, robot-test result, head mapping), and note anything that failed. Commit the profile and STATUS.md and push:

```sh
git add software/profiles/paper-tray-v0.yaml software/STATUS.md
git commit -m "Bring-up: ports, cameras, calibration, robot-test results"
git push
```

Commit as the user, with no co-author or tool attribution lines. The calibration file itself lives in `~/.cache/huggingface/lerobot/calibration/` and is not in the repo; say so.

## Rules

- Stop, report and ask when a step fails. Do not keep going.
- Report hardware errors verbatim. The user can see the robot; you cannot. Do not speculate about broken parts.
- 12 V off before any servo wire is moved.
- Do not edit limits, safety rules or the wheel setting to get past a failure.
- Do not start `farm teach-all`, `farm once` or `farm run` from this skill.

## End state

Ports and camera indices in the profile, a saved calibration, `farm robot-test --move --ask` passed with every joint confirmed, `farm check` reporting no motor or camera problems, STATUS.md updated and pushed. The next step is placing the bottle, paddle and trays and running `farm teach-all`; that is outside this skill.

---
Calibration trap and stop-report-ask rules adapted from [xlerobot-onboard](https://github.com/ScavieFae/xlerobot-onboard) (MIT), which does the same job for the stock XLeRobot software.
