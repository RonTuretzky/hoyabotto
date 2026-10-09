# Stream mode (owner `--stream`)

Stream mode is an opt-in mode of the sole hardware owner. It is meant for a 10 Hz learned-policy client: the client
sends one set of arm targets per step and gets an acknowledgement back.

**It is off by default and has never run on hardware.** When it is off, the two new owner ops are rejected exactly
like an unknown op (`Unsupported hardware command`), `capabilities` does not contain `stream`, and nothing else in the
owner changes.

It closes these gaps in the normal motion path (see `software/docs/carton-fold-policy-robot.md` §5 item 8):

- A gripper closure can now be streamed.
- A refused streamed target no longer makes the client send STOP.
- `contact_halt` now works for streamed commands, because the counters persist across commands.
- There is a "hold where you are now" command.
- One call can name both arms.

Code:

- `stream_joint_executor.py`: the executor.
- `gemma_hardware_owner.py`: `stream_command`, `hold_here` and the `--stream` flag.
- `gemma_direct_client.py`: `DirectJointClient.stream` and `DirectJointClient.hold_here`.
- `gemma_robot_tools.py`: the two API tools.
- `test_stream_targets.py`: the tests.

## Owner ops

### `stream_targets` `{targets: {motor: int ticks}, command_id?}`

The owner keeps one persistent `StreamJointExecutor`. Each command only updates its targets, so the contact counters,
the jaw guard and the jaw blocks carry over from one command to the next. The optional `command_id` is the caller's
own tag; it is echoed back as `client_command_id`.

| Rule | Value |
|---|---|
| Motors | The 12 arm motors only (`left_arm_*` / `right_arm_*`, grippers included). Each arm you name needs all six of its joints enabled. The streamed set is all six joints of every named arm. |
| Range | A target must lie inside the saved range minus `MARGIN` = 40 ticks. A target outside it **rejects the whole command**: nothing is clamped and nothing changes. |
| Envelope | A target more than `ENVELOPE` = 96 ticks from the present position rejects the whole command. For a jaw the check uses the 10-tick-limited target. |
| Ramp | Each owner loop moves every goal at most `STEP` = 40 ticks toward its target. |
| Skip | A joint whose target is within 2 ticks of its present position or of its held goal is skipped: its goal stays where it is, and it is listed in `skipped_joints`. If every joint is skipped, the command is accepted as a no-op (`no_op: true`). A no-op does not start a stream; the owner treats it as a heartbeat, like `hold`. |
| Jaws | A jaw may open or close, but its goal changes by at most 10 ticks per command. The limited target is reported in `jaw_limited`. There is no "did not become stationary" fault. |
| Jaw guard | A closing jaw that reads \|Present_Load\| ≥ 250, or lags its goal by ≥ 40 ticks, is frozen at its present position (clamped to range ± 40). It is reported as `jaw_contact: {name: {load, position}}`. Later closing targets for that jaw are ignored (`jaw_ignored_closing`) until an opening target arrives. A release clears the block, and so does a normal motion that moves that jaw. |
| Arm contact | This is the paddle executor's `contact_halt` rule: load ≥ `CONTACT_HALT_LOAD` (350) for 2 samples, stalled (velocity < 3 and ≤ 3 ticks of movement) for 2 samples, and lagging ≥ `CONTACT_PUSH_TICKS` (50) ticks. The counts carry across commands. On contact, every streamed joint is held at its present position (clamped to range ± 4), and the stream ends with `stream_phase: 'contact_halt'`, `closure_outcome: 'contact_halt'`, `stream_contact_joints` and `contact`. Nothing is released. New `stream_targets` are then refused until a `hold_here`, a `halt`, a started normal motion, or a release. |
| Timeout | If no `stream_targets` arrives for 0.5 s, the stream ends holding its current goals (`stream_phase: 'holding'`, `closure_outcome: 'stream_timeout'`). Nothing is released. The next command starts a new stream. |
| Replace | `stream_targets` is rejected while a non-stream motion runs: halt it first. A normal motion command sent while streaming ends the stream first, holding its goals, as `replace=true` does today. A `base_pulse` is still refused while a stream runs. |
| Unchanged | The 96-tick \|present − goal\| fault (while streaming and while holding afterwards). Load > 800 (arm) or > 500 (jaw). Status, voltage and range faults. The 1.0 s telemetry and tick watchdog. The phone-feed camera gate. Any of these still releases everything. While a stream runs, the lease is renewed to 5.5 s per command; after it ends, the usual 120 s. |

The owner state carries `stream_ack` (the id of the last processed stream command), `stream_result` (`no_op`,
`skipped_joints`, `jaw_limited`, `jaw_ignored_closing`, `jaw_contact`), `jaw_contact`, `stream_phase` and
`stream_limits`. `stream_phase` is one of `streaming`, `holding`, `halted`, `contact_halt` or `released`. While a
stream runs, `phase` stays `moving`.

### `hold_here` `{}`

Sets every enabled joint's goal to its freshly read present position, clamped to range ± 4 (the first loop of
`soften()`). There is no torque ramp-down and no release. It ends any running motion or stream, which shows as
`closure_outcome: 'halted'`. A base pulse is told to stop early. It also acknowledges a stream `contact_halt`, and
afterwards `stream_phase` is `holding`.

## Client and API

`DirectJointClient.stream(targets, command_id=None)` and `DirectJointClient.hold_here()` return
`{'accepted': False, 'reason': ...}` and **never send STOP** in any of these cases:

- `STREAM_MODE_DISABLED`
- a stale status
- the owner is not healthy
- a STOP is pending
- the owner rejects the command
- no acknowledgement arrives within 1 s

The existing ops keep their STOP-on-failure behaviour. An actual owner fault is still visible: the client returns
`owner_stopped: True`, and `stop_count` / `last_stop` increase.

Two API tools are added. Both are listed among the motion tools and hidden from the pilot (`PILOT_HIDDEN`), so the
chat model is never offered them. Both answer `accepted: false, reason: STREAM_MODE_DISABLED…` unless the owner
advertises `stream`; `robot_get_capabilities` reports this as `stream_mode`.

- `robot_stream_joint_targets {positions: {canonical arm motor: integer ticks}}`. Both arms and grippers are allowed.
  The schema bounds are range ± 40. There is no wait: one owner op, one acknowledgement. It returns `accepted`,
  `skipped_joints`, `jaw_contact`, `jaw_limited`, `jaw_ignored_closing`, `no_op`, `phase` (the stream phase),
  `owner_phase` and `stop_count`.
- `robot_hold_here {}`.

`software/farm/sim/sim_robot.py` lists both tools in `MOTION_TOOLS`. They are not simulated, so they are not in
`IMPLEMENTED_TOOLS`.

## Enabling it

1. Get the owner's approval. This changes how the arms are driven.
2. On the robot Mac, with the arms supported and released, run `./restart-robot-server.sh --stream`. That runs
   `redeploy_robot_server.py --stream`, which adds `--stream` to the owner command line and checks that the fresh
   owner advertises `stream`. Leave out `--stream` to turn the mode off again.
3. A remote `/admin/deploy` restart does not pass `--stream`, so it starts the owner with stream mode off.
   `--api-only` does not restart the owner and does not change the mode.

## What was tested (fake bus, virtual clock; no hardware)

`test_stream_targets.py` covers:

- Off by default: both ops are rejected like an unknown op, and `capabilities` lacks `stream`.
- A two-arm stream with a ≤ 40-tick-per-loop ramp.
- Range and envelope rejection with nothing changed.
- The 2-tick skip and the all-skipped no-op.
- Jaw closure in ≤ 10-tick steps.
- The jaw guard (load 250 and 40-tick lag variants): freeze at present, closing ignored until an opening.
- Arm contact across successive commands: `contact_halt` holding at present, nothing released, latched until
  `hold_here`.
- The 0.5 s timeout: holding, nothing released.
- `hold_here`: goals at present and clamped, with no torque change and no release.
- Replace and halt rules.
- The 96-tick, load-800 and jaw-500 faults still releasing everything, and the holding-drift fault after a stream.
- The client against a real owner loop: refusals return `accepted: false` with no STOP written, and a fault is visible
  via `stop_count`.
- The API tools: schema, `PILOT_HIDDEN`, and refusal without `stream`.

All of `redeploy_robot_server.TESTS` still pass.

## Validate on hardware first (supervised, 12 V switch in reach)

1. **Free-air jaw stream.** With nothing between the jaws, stream a jaw close and reopen at 10 Hz in 10-tick steps.
   Check that the jaw follows smoothly and that no `jaw_contact` appears in free air (load stays < 250, lag < 40).
2. **Jaw contact.** Close on a soft object and check that the jaw freezes, `jaw_contact` is reported, and further
   closing is ignored until an opening.
3. **Arm contact yield.** Stream one joint slowly into a padded block. Check for `contact_halt` holding at the present
   position, with no release. Note the real `Present_Load` and lag.
4. **Timeout hold.** Stop the client mid-stream. Within about 0.5 s the stream should end holding, nothing released.
   Also check that a held arm under load stays inside the 96-tick holding-drift fault.
5. **Timing.** Measure the owner loop period and the stream acknowledgement round trip. The ramp assumes 40 ticks per
   loop; the loop period on hardware is unmeasured.
