# Direct tape pickup from the dispenser

Updated October 4, 2026: the owner ordered a dispenser and chose **no tape flip/turnover training**.
The exact purchased model, outlet geometry, tape width and strip length still need to be recorded
on the connected Mac. Do not assume it is the carousel discussed during shopping or assume 80 mm.
**No physical dispenser pickup, adhesion or placement has been demonstrated.** This is a keyframe
controller; it does not train or execute the ACT checkpoint.

## Sequence

Open fingers → approach the exposed end of a fully cut strip → pinch → visually check grip →
lift clear of the dispenser support → check clearance and adhesive-down orientation → carry
above the closed box seam → lower onto the carton → verify support → release → retract →
verify the tape stayed on the seam. Paddle pressing remains a separate step.

There are six tape poses, no turnover poses, and no requirement for a folded pinch tab. The strip
must be presented **adhesive down** and stay that way. If the real outlet presents it otherwise,
stop and adapt the station/pickup support; do not add a flip or assume the dispenser fixes orientation.
The gripper stays closed during transport even if a saved pose or teaching request asks to open it.
Unknown grip/orientation/clearance, an uncut strip, low confidence, stale images, motion failure or
STOP ends the sequence without an automatic retry, release or retreat. Vision is fallible and is
not force/contact sensing or a reliable machine-motion interlock.

## First station setup

1. Preserve robot calibration and follow [carton-connected-mac.md](carton-connected-mac.md).
   Fix the dispenser near the left arm, outside flap travel; keep its cutter/feed mechanism and
   cover outside the gripper path. The right paddle stays clear during this isolated test.
2. Record the purchased model and actual tape/strip dimensions. Present one **fully cut** strip
   with a reachable end and adhesive facing down. The printed tape rest is no longer required.
   Test that the end can be gripped and released without sticking to the jaws; this is not yet proven.
3. For the first trial, stop the dispenser and disable automatic cycling/refill. The robot software
   has no dispenser control or interlock. For a carousel, the disc must remain stopped throughout
   pickup. Automatic replenishment is a later model-specific integration, not implemented here.
4. Put a visible mark on the nonsticky backing of the test strip so the cameras can establish its
   face. Use an already-closed, supported carton with an untaped seam. Verify the chosen strip
   length bridges the seam and, after paddle pressing, holds the flaps shut. Shorter strips are
   accepted as a prototype choice, not established sealing performance.
5. Both head and left-wrist cameras must see the pickup and placement. Keep the physical power
   cutoff reachable. Run from an attended Terminal with camera permission; no detached live job.

## Connected-Mac commands

From `software/` with the environment active, inspect the plan without opening hardware:

```sh
carton tape-test --plan
```

After setting up the actual dispenser, physically teach and execute one direct pickup/placement:

```sh
carton tape-test -p profiles/carton-local.yaml --teach
```

The command starts and verifies its own STOP viewer at `http://localhost:8765` before moving.
Connecting still configures motors and toggles torque: begin with supported arms at stable rest.
The vision model teaches the poses; nobody positions the robot by hand. All six poses are saved
only after the full sequence succeeds, under one run ID tied to profile and calibration. Failure
leaves existing poses unchanged. The controller holds when finished and asks for arms to be
supported before ENTER disconnects torque. Ctrl-C, EOF, process termination and power loss can
also release torque, so remain present.

Present a new strip at the same stopped outlet and reset the closed carton before replaying:

```sh
carton tape-test -p profiles/carton-local.yaml
```

Do not retry until a failed held-strip/box state has been inspected and reset. Moving the robot,
dispenser, box or cameras requires re-teaching even if configuration text did not change. Changing
profile/calibration invalidates the bundle. Old tape-rest/turnover bundles are rejected.

`carton once` uses this same sequence. Tape failure/unknown stops the cycle without a second pickup.
Individual tape-pose teaching is refused; `carton teach-all` directs untaught tape poses to this
isolated trial. Teach and verify the separate paddle pickup/folding/pressing work before a full cycle.

## Evidence

The database records each pose/check and head/wrist image hashes. Record actual outcomes and
remaining blockers in `STATUS.md` on the robot Mac. Simulator faults include `tape_not_clear`,
`tape_wrong_side`, `tape_uncut`, `tape_dropped`, `tape_unknown=lifted` and `tape_missed`.
These exercise controller decisions only; the simulator does not model adhesion, release, flexible
tape, cutter motion or dispenser geometry. Never copy its synthetic poses to real hardware.
