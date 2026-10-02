# xlerobot-farm

Two things live here: the handbook site (`handbook/`, built by `build_site.py`, published to GitHub Pages) and the farm program that runs the robot (`software/`).

If you are on the laptop connected to the robot, read these in order before doing anything:

1. `software/STATUS.md`: where the physical build stands, what is verified, what is assumed, and the next steps.
2. `software/SETUP.md`: installing everything on a blank laptop.
3. `software/README.md`: what the program is and its commands.

To bring the robot up (probe, profile, calibration, motors-only test), use the `farm-bringup` skill in `.claude/skills/`: type `/farm-bringup` or ask to "bring up the robot".

Working rules:

- `farm` commands other than `farm sim` and the tests talk to real motors. Say what a command will move before running it, and keep the viewer's STOP button reachable.
- Switch 12 V off before changing any servo wire. 3-pin wires carry 12 V; cameras use separate 4-pin USB cables.
- Do not drive the wheels (`wheels: false` in the profile) and do not relax limits in `farm/safety/rules.py` or the profile's `limits:` to make something work.
- Camera commands must run from Terminal on macOS (camera permission is per app).
- Secrets go in `software/.env`, typed by the owner. Never commit them or print them.
- Commit as the user, with no co-author or tool attribution lines.
- Record what you learn on the hardware in `software/STATUS.md` and push, so the next session starts from it.
