# Emoji show: visitors pick an emoji, the robot performs it

Visitors pick preset emojis on a kiosk page and type their name. The robot performs each emoji's gesture, and
their name is on the big screen while it does. The first and only gesture so far is 👋, a right-arm wave.

It runs on the **chat Mac**, the one that holds the paired client certificate
(`gemma-xlerobot/pilot/.private/robot.json`). It moves the robot only through the existing robot API (`POST /call`
on the sole hardware owner). It opens no serial port, starts no owner and changes no limits. It uses only the
standard library.

```sh
cd software
python3 -m robot_emoji                  # real robot; kiosk on http://localhost:8790/
python3 -m robot_emoji --fake           # no robot: simulated moves at the owner's pace
python3 -m robot_emoji --host 0.0.0.0   # phones on the same network can reach the kiosk
```

| Page | For |
|---|---|
| `/` | Kiosk: pick up to 3 emojis, type a name (at most 24 characters), get a place in line |
| `/screen` | Big display: whose turn it is, the current phase, who is next |
| `/operator` | Arm/pause, remove from the queue, log, and a big **STOP** |

Operator actions are accepted from this Mac or with the token printed at startup (`/operator?token=…`).

## What one performance does

1. **Check.** `robot_get_motion` and `robot_get_state` are read. If anyone else holds a motor, or anything is
   moving, the request waits ("robot in use") and nothing is sent. Every target is checked against the live
   `commandable_ranges`, and none is clamped.
2. **Enable.** All six joints of the arm are enabled in one `robot_set_motor_enable`, as the pickup profile
   requires. They hold where they are.
3. **Raise, then gesture.** `robot_move_path` runs with `wait=true` for each gesture: its raise, then its motion.
4. **Return and release.** The arm goes back to where it rested, clamped into the commandable range, and only
   its six joints are released.

The owner's pace is 40-tick steps every 0.4 s (`farm/safety` and the pickup profile are unchanged). From
today's resting pose, a wave therefore takes about 14 s to raise, 15 s to wave and 14 s to return, plus settling.

Failures:
- **Owner fault or STOP.** The owner has already released everything, so the service sends nothing more.
- **Other failures while the arm is held.** A refused move or a network error leads to a halt (if moving), a
  move home and a release. If that also fails, the service sends `robot_stop`.
- **Afterwards.** Any failure pauses the show. A person looks at the robot, then re-arms it.

**The show starts paused.** Arm it only when the arm has room to rise above the robot and nothing is in the
gripper. STOP on the operator page calls `robot_stop`, which soft-releases every motor. The 12 V switch remains
the hard stop. The phone camera must be fresh: the owner refuses to enable without it, and the visitor sees
"could not do this one".

## Gestures (`robot_emoji/gestures.json`)

Each gesture has an emoji, a label, an arm, a `raise` path (from rest to the start pose) and a `motion` path.
Ticks are raw encoder values with short joint names. Rules, checked when the file loads:
- The gripper is excluded, because a closing gripper cannot run inside a path.
- Each motion leg is at most 341 ticks per joint, so the owner runs it as one piece and keeps the rhythm.
- At most 12 waypoints per path.

To add an emoji, add an entry. The kiosk shows it on the next start.

The wave's poses were chosen in the digital twin (`farm/sim/xlerobot_twin.py`):
- **Raised hand:** pan 2100, lift 2300, elbow 1150, wrist flex 1600. The upper arm leans forward, and the
  forearm and hand point up, to the right of the mast. The twin puts the claw about 1.27 m above the floor.
- **Wave:** the wrist flaps ±170 ticks while the shoulder pan sways between 1990 and 2210 (the hand moves about
  8 cm outward).

The twin's tick-to-angle mapping is unvalidated, so the gesture is marked `verified_on_hardware: false` and the
operator page says so. **For the first run on the robot**, someone watches the whole arm with STOP and the
12 V switch in reach. If the real pose differs from the twin, correct the ticks, then set
`verified_on_hardware: true`.

Tests: `tests/test_robot_emoji.py` (plans, the call sequence against a fake owner, busy/refusal/fault/STOP
handling, the queue and the web routes).
