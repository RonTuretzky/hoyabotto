# Handoff for the robot laptop: turn off the servos' own temperature cutoff

Prepared 2026-10-06 on the development Mac, which has no motor boards. Everything below was tested
only against a fake bus; nothing has been written to a real servo yet. This session exists to do that.

## What is already done (on `main`)

- The software no longer reads or acts on servo temperature anywhere. Load, step, travel, watchdog,
  lease and STOP checks are unchanged. `farm soak`, the profile key `servo_temp_max_c`, the carton
  owner's 55 °C checks and the archived 2026-10-05 scripts that enforced 55 °C are gone.
- New command `farm servo-protection` changes the servo's own firmware protection, which lives in
  the servo's EEPROM and is independent of our software:

  | Register | Factory default | After `--write` |
  |---|---|---|
  | `Max_Temperature_Limit` (13) | 70 °C | 200 °C (`--limit`; one byte, Feetech documents 0..100; falls back to 100 if the servo will not keep 200) |
  | `Unloading_Condition` (19) | 44 = temperature + current + overload | 40: temperature bit cleared |
  | `LED_Alarm_Condition` (20) | 47 | 43: temperature bit cleared |

  Only the temperature bit (bit 2) changes; current and overload protection stay as they are.

## What this session must do

Nothing in this handoff moves a joint. Torque is switched off on each servo while its EEPROM is
written, so the arms must already be resting where they can go limp safely.

### One-liner, from the repository root

```sh
software/.venv/bin/farm servo-protection -p paper-tray-v0 --write --limit 200 --yes
```

It finds the two motor boards itself when the profile's `port1`/`port2` are empty, writes every servo on
both boards, and reads each one back. Leave off `--yes` to get an ENTER prompt first. The numbered steps
below are the same thing done carefully (read first, write, verify after a power cycle).

### 0. Preconditions

1. `cd xlerobot-farm/software && git pull --ff-only && . .venv/bin/activate`
2. Nothing else may hold the motor ports: no `farm mcp`, no carton owner (`carton_session.py`), no
   Gemma robot/hardware server, no `work/` script. Check with `lsof /dev/cu.usbmodem*` (expect nothing).
3. 12 V on, both motor boards plugged in, wheels off or parked (they are not commanded either way).
4. Arms folded at rest on the cart, head wherever it is. Keep a hand near the 12 V switch.
5. Use the profile this laptop already runs with (`paper-tray-v0` unless the ports are filled in
   elsewhere). `farm devices --probe` shows the two boards and which servo IDs answer on each.

### 1. Read first (no writes)

```sh
farm servo-protection -p paper-tray-v0
```

Expected: one row per register for all 16 servos (left arm 1–6 + head 7–8 on board 1, right arm 1–6
+ wheels 9–10 on board 2), `now` 70 / 44 / 47 and `target` 200 / 40 / 43, ending with
"16 motor(s) still have temperature protection". If any servo shows other values, note them; they are
not a reason to stop unless a read fails. A read failure means the port is busy or a servo is not
answering: fix that before writing.

### 2. Write

```sh
farm servo-protection -p paper-tray-v0 --write
```

It prints what it will do and waits for ENTER. Per servo it then: torque off, `Lock` 0, writes the
three registers, `Lock` 1, reads all three back, and prints "written and read back" only if the
read-back matches. A mismatch prints `WRITE FAILED` for that servo and the command continues with
the rest, exiting 1 at the end. `--only head|left|right` restricts it if you want to do one group
first.

If a servo will not keep 200, the tool writes 100 instead and prints "servo kept 100 C, not 200"; that still
means never, for a motor that works, so record it and carry on. If one servo fails otherwise, run the same command again once (it skips servos that are already done). If it
fails twice on the same servo, stop and record the exact line; do not try to write registers by hand.

### 3. Verify

```sh
farm servo-protection -p paper-tray-v0
```

Expected: every row `now` equal to `target`, ending with
"ALL DONE: no servo on these buses unloads or flags on temperature." Then power-cycle 12 V and run
it once more: EEPROM must survive the power cycle.

### 4. Record and push

Paste the final table into `software/STATUS.md` under "Servo temperature sensing removed" and
commit as the user (no co-author lines), push to `main`.

### 5. Retire the old `work/` owner on this laptop

The 2026-10-05 session ran a motor owner from a `work/` directory outside the repository
(`work/carton_session.py`, `work/temperature_confirmation.py`, `work/left_claw_prepare.py`). Those
copies still enforce 55 °C. With the user's go-ahead, move them aside so they cannot be launched by
habit, for example `mv work/carton_session.py work/carton_session.py.retired-2026-10-06`, and use
only `software/scripts/carton_robot/` (see `software/docs/carton-controller-handoff.md` for the
environment variables it needs). Do not delete `work/carton-session/` state or any logs.

## Stop if

- a read fails after the ports are free: the bus or a servo is not answering, which is a wiring or
  power question, not a software one;
- a servo reads back something other than the target twice;
- anything moves. Nothing in these commands sends a goal position; if a joint moves, switch 12 V off
  and record what happened.

## Open questions the session can answer

- Whether the firmware still raises a temperature fault flag in its status byte with the mask
  cleared. With the limit at 200 °C (or 100 °C) it should never trigger, but the datasheet does not say which
  of the two settings governs the flag. If a carton session ever stops with "motor fault" while
  the servo is hot to the touch, that is the answer, and it belongs in STATUS.md.
