# XLeRobot community projects, checked against the farm plan

Source: https://xlerobot.readthedocs.io/en/latest/relatedworks/index.html (9 papers and 34 projects, collected by the XLeRobot maintainers on 2026-09-18). Reviewed 2026-10-02.

How it was checked: every entry's summary was read; for the eleven closest ones the README (or the linked doc) was read as well. Nothing here was installed or run on the robot.

The plan they are judged against: a parked two-wheel XLeRobot 0.4.0, run from a laptop over USB, that waters paper-grown cress by pouring from a bottle. Poses are taught by a vision model, not by a person driving the robot. Learned policies come later, from the robot's own recorded runs.

## Worth something to this plan

| Project | What it is | Use for us |
|---|---|---|
| [xlerobot-onboard](https://github.com/ScavieFae/xlerobot-onboard) (MIT) | Installer that pins LeRobot v0.5.1 and an XLeRobot commit, copies the robot classes into LeRobot, and verifies the imports. Ships a Claude Code skill that walks through motor IDs, ports, calibration and first teleop. | The quickest way to run the stock XLeRobot examples (keyboard teleop) as a check that is independent of the farm program. Uses its own environment, so it does not disturb ours (LeRobot 0.6.1). |
| [LeRobot PR #3282](https://github.com/huggingface/lerobot/pull/3282) (MakerMods) | Automatic calibration for SO-101 arms: drives each joint to its mechanical limits at low torque, no hand movement. Open, not merged (last updated 2026-09-29). | Would remove the one step where a person still moves the arms. Not adopted: it was tested on a single free-standing arm, and on the cart an arm sweeping to its limits can reach the neck, the other arm and the tray rim. Revisit if it merges. |
| [RoboCrew](https://github.com/Grigorij-Dudnik/RoboCrew) (MIT, active) and [xlerobot-mcp](https://github.com/windht/xlerobot-mcp) | An LLM agent loop for XLeRobot: camera frame in, tool call out. Tools include saved arm positions, head yaw/pitch, and trained policies wrapped as tools. The MCP server exposes the same controls to any MCP client. | Same shape as our LLM-servo and keyframes, built independently. Its driver maps head yaw to servo 7 and pitch to servo 8, limits yaw to ±120° and pitch to 0-85°: a second source for our head assumption and sensible clamp values. Its arms are moved by saved poses or trained policies, not by free-form LLM joint commands. |
| [Dexbotic XLeRobot integration](https://github.com/dexmal/dexbotic/blob/HEAD/hardware/docs/xlerobot_inference_example.md) (MIT) | Data conversion, training and deployment of a VLA policy, written for the v0.4.0 two-wheel robot: 16-dimension action (two arms, head, wheels), three cameras named head / wrist_left / wrist_right. | The only pipeline here aimed at our exact robot. A candidate for the learning phase once we have our own episodes; needs a GPU server. Our recorder already writes the LeRobot format it converts from. |
| [Kinesthetic recorder](https://github.com/tianrui-li-0/xlerobot-kinesthetic-recorder) | Record a demonstration by moving the follower arms by hand with torque off, then replay it to produce a LeRobot dataset. No leader arms needed. | The fallback if vision-model teaching does not work: it needs no hardware we lack. It is hand guidance, so it breaks the no-human-operation rule and is only a fallback. Tested by its author on Linux/WSL. |
| [XLeRobot-Pro measurement tools](https://github.com/Minko82/xlerobot-pro-data) | Protocols and scripts for brown-out, servo temperature under a held pose, and payload tests. | A model for a soak test before the robot is left running for days: hold the pour pose, log servo temperature, stop at the ceiling. We already refuse to act above 55 °C. No licence on the repo, so ideas only. |
| [Home Service Demo](https://github.com/xujiayuxian-png/xlerobot_home_service_demo) (Apache-2.0) | Fetch-and-deliver on a modified two-wheel XLeRobot: hand-eye calibration tools, ACT and geometric grasping. | Its ACT checkpoint is already listed in our README as a wrapper test. The rest needs a depth camera, lidar, ROS 2 and a second GPU computer, so it is not adoptable. |

## Related, but nothing to take

| Project | Why not |
|---|---|
| AnchorVLA4D (paper) | Evaluates pouring on an XLeRobot with its own demonstrations. The arXiv page links no code, data or weights. |
| Matcha Bot | Liquid handling with a GR00T policy. The linked repo is a generic two-arm LeRobot plugin; no dataset found there. |
| XLeRobot Perception Engine | Head pan/tilt tracking and a head calibration tool. Overlaps what we have; aimed at following people. |
| Laundry Bot, OneRobotAI, Grievous, XS-VLA paper | Policy training for other tasks (towel folding, dusting, handover). Confirms ACT and SmolVLA are the usual choices, which is what we planned. |
| MakerMods XLeRobot, Cutting the Cord (paper) | Redesigned printed parts, power-bank mounts, stiffer arms, onboard computer. Useful only if the robot goes untethered. |

## Not relevant to a parked, model-taught robot

- **Teleoperation (11 projects, 1 paper):** VR and WebXR (XLVR, Pounce, XLeRobot WebXR), Joy-Con (two projects), leader arms over the network (LiveKit Portal, XoQ, Vesalius-Ai), ROS 2 drivers (Jazzy, Humble), mecanum base control, and the low-cost teleoperation paper. All are ways for a person to drive the robot.
- **Simulation (3 projects, 1 paper):** IsaacLab, Isaac Sim on DGX Spark, Gazebo, LeHome. We test on our own simulator and train from real runs.
- **Different hardware (3):** XLeRobot Pinc (other arms and grippers), XLerobot-X1 (Qualcomm computer), Nori Bot (vertical lift).
- **Voice agents and other robot stacks (5):** QWEN-XLeRobot, Maque, XLeRobot-dev, the Clemson project, and BrainBot (teleoperation plus policy serving).
- **Other research papers (3):** MARS-RA and PIPHEN (multi-robot coordination), ElasticFlow (a policy method with qualitative demos).
- **Hackathon training tools:** xle-hack (same entry as the installer listed above).

## What changed in our plan

Built and tested on the simulator; none of it has run on the real robot yet.

1. `farm robot-test`: a motors-only check that needs no cameras or models (from the need xlerobot-onboard addresses).
2. `.claude/skills/farm-bringup`: a bring-up skill for the robot laptop (xlerobot-onboard's skill, rewritten for the two-wheel kit).
3. `farm calibration-report`: flags a wrapped, short or mismatched calibration before anything moves. Head ranges are compared with RoboCrew's limits.
4. `farm policy-server` and the remote policy client: the training Mac serves the model, the robot laptop keeps the clamps (the split Dexbotic uses; see `gpu-server.md`).
5. `farm soak`: hold a pose and log servo temperature to a ceiling (the XLeRobot-Pro protocol).
6. `farm teach --by-hand`: teaching by placing the arm (the kinesthetic recorder's idea). Off unless the profile allows it, because it is human operation.
7. `farm mcp`: an MCP tool surface that stops at the skill layer (xlerobot-mcp exposes raw servo positions; ours does not).
8. Automatic calibration (LeRobot PR #3282): not built. It needs a decision, because limit-seeking motion on the cart is unproven.

The head mapping (pan = 7, tilt = 8) now has a second source; it is still checked on the hardware by `farm robot-test --move --ask --only head`.
