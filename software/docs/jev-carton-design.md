# Jev for this carton task

Use Astra for visual interpretation, geometry and changing the plan; use Jev to
choose among already eligible carton actions; use local code for movement,
limits, stopping and objective completion checks. Jev does not accept images.
Giving it access to every servo does not give it the missing visual geometry.

## Where the actual task is

The connected Mac's `Set up XLeRobot farm Mac` chat was inspected on 2026-10-04.
It is operating `work/carton_session.py` and `work/carton_command.py`, separately
from this repository's ordinary `CartonCycle`. The latest observed work is
right-claw alignment below the paddle handle, with shoulder-pan/lift adjustments
and fresh head/right-wrist images. No verified paddle grasp or folded flap was
observed in that transcript slice. This is a timestamped observation, not a live
status feed.

The saved carton profile describes a 379 × 283 × 108 mm box, 140 mm flaps and a
150 mm paddle in the right gripper. The existing full program includes taping;
the active remote request is folding. `fold_only` and `fold_and_tape` are separate
goals. A tape operation must not be added merely because it exists in the code.

The immediate bottleneck is establishing the claw-to-handle relationship and
reusable movements. Replacing the model used to reason about each shell command
cannot by itself solve that. The existing legacy IK also mixes angle and
normalized-position conventions; the new motion path uses raw encoder ticks.

## Division of work

| Component | Owns | Called when |
| --- | --- | --- |
| Astra / existing approved vision model | Interpret camera views; estimate claw, handle, rim and flap geometry; propose a phase plan and recovery | Initial setup, ambiguous view/geometry, changed registration, failed grasp, repeated failed fold |
| Local perception | Timestamp frames, track previously identified features, detect visibility loss and scene changes | Continuously, independently of inference |
| Jev | Select one useful primitive from the controller's current eligible set; choose observe, verify or replan when needed | At ambiguous action boundaries with fresh structured evidence |
| Motor owner | Read actual joints and health; establish eligibility; execute bounded motion; stop; enforce lease and budgets | Before and during every movement |
| Deterministic carton supervisor | Known stage transitions, required verification, STOP, faults, missing primitives, completion conditions | Every observation; these cases do not need Jev |

The local feature tracker is a required next integration component, **not an
implemented capability claimed by this branch**. Until it exists, useful scene
updates still require the current vision workflow. A new image timestamp must
never make an old Astra interpretation look fresh.

## Exact carton loop

1. **Register station.** Inspect camera identity, calibration and measured box
   geometry. If repositioning is needed, use a commissioned short wheel pulse
   with arms stowed, no contact and clear floor. Then invalidate all old station
   poses and register again. Wheels are available, but driving while the paddle
   or left arm contacts the carton is ineligible.
2. **Acquire paddle.** Astra identifies handle and jaws and proposes a local
   alignment strategy. Commission the correction's direction and bounded range.
   Local tracking updates the error; Jev can choose `look_handle`,
   `align_handle`, or a verified grasp primitive. If the mapping is unknown,
   request planning instead of asking Jev to guess a joint angle.
3. **Verify grasp.** Read jaw position/load and inspect fresh wrist/head evidence.
   A gripper-close acknowledgement is not a grasp. Lift only when the pickup
   primitive's own postconditions are established.
4. **Fold short flaps.** Execute one taught approach/contact/sweep/retract
   primitive. Inspect whether the flap stayed down. Use a commissioned left-arm
   stabilizer if required; if the flap springs back repeatedly, ask Astra for a
   different retention strategy. Do not repeat the same nudge indefinitely.
5. **Fold long flaps.** Choose only primitives matching the observed short-flap
   and held-tool state. Verify each flap after retraction. Known next stages are
   simple code transitions; Jev is useful only where there is a real choice.
6. **Finish or tape.** Folding completes only after all four flap outcomes are
   observed. For `fold_and_tape`, additionally verify the tape placement and
   release. A model's `finish` answer cannot satisfy these conditions.

Each motor command requires a new observation sequence. A completed command
sets a pending verification record. Its successor must carry fresh, post-motion
evidence matching that primitive. Failure requests planning, not a blind retry.

## Plan and evidence contract

An Astra proposal should contain: goal scope, phase, station revision,
calibration identity, camera frame IDs/capture times, measured feature locations
and uncertainty, proposed primitive name, expected starting pose, clearance
conditions, bounded action parameters, expected visual result, and failure
recovery. The local controller validates/commissions the primitive; Astra cannot
set its own proposal's `validated` flag.

The perception producer gives the supervisor `sequence`, `observed_at`,
`camera_ok`, `box_visible`, `health_ok`, `stop_requested`, `obstruction`,
`geometry_known`, `station_revision`, `calibration_id`, `stage`, held-tool and
alignment observations, `consecutive_failures`, and outcome verification.
Raw images, credentials, shell commands and free-form executable code are not
Jev inputs. Primitive callbacks are installed locally and cannot be supplied
by a model response.

## What the live evaluation showed

On 2026-10-04, the direct TypeSafe endpoint served pinned `jev-1.13.0` using the
user-provided key held only in process memory. One connection pool, no retries,
text only, no robot attached. We ran 24 hand-labelled synthetic carton scenarios
twice, including the observed setup gap, occlusion, grasp, both arms, spring-back,
driving, stale registration, tape scope, false completion and STOP injection.

| Measurement | Result |
| --- | --- |
| Native requests | 48; no API errors |
| Workflow route matches | 48 / 48 |
| Full route + candidate matches | 46 / 48 |
| Median HTTP latency | 202.645 ms |
| 95th percentile HTTP latency | 466.75 ms |
| Motion examples meeting both configured thresholds | 8 / 14 |
| Estimated total API cost | $0.001402464 |

The two candidate mismatches were the base-reposition case: route `execute`,
candidate `unknown`, both below motion thresholds. Alignment and grasp choices
also fell below the thresholds despite selecting the labelled candidate. The
controller would stop/escalate all these cases. Do not lower thresholds simply
to improve the acceptance rate: neither probability nor confidence measures
collision clearance or grasp quality.

The raw model's motor-overheat route was correct but only about 0.62 probability
on the first repeat. That is another reason faults and STOP belong to code,
which blocks them before inference. Successful routing is not a safety proof.

These descriptions explicitly supply important facts. They test routing and API
behavior, **not real perception, physical success, or generalization**. They are
development examples, not a held-out evaluation. Repeating them does not create
48 independent task situations. No physical folding rate, complete-loop speedup
or Astra comparison was measured. Sources and all case definitions are checked
in; detailed replies and timings are in
[`evidence/jev-carton-2026-10-04.json`](evidence/jev-carton-2026-10-04.json).

A separate live transport-to-simulator check executed **36/36** requested bounded
actions: both directions for all 14 position servos and all eight wheel actions.
Both simulated wheel velocities were zero afterwards in every case. Each request
offered one locally eligible movement plus hold/stop/unknown; this verifies the
API-to-executor interface, not selection among competing spatial actions. See
[`evidence/jev-actuation-sim-2026-10-04.json`](evidence/jev-actuation-sim-2026-10-04.json).

Reproduce with `TYPESAFE_API_KEY` supplied securely in the process environment:

```sh
python -m carton.jev_eval --repeats 2 --out carton-evaluation.json
```

TypeSafe does not return a dollar charge. Cost is estimated from the documented
1.13 price of $0.042 per million input tokens, with free output, as checked on
2026-10-04. It is not a billing receipt.

## Implementation and integration boundary

This branch provides:

- Direct TypeSafe and OpenRouter native transports, pinned model, pooling,
  bounded response time, no hidden retries, usage accounting and sanitized errors.
- `carton.supervisor.CartonSupervisor`: batched route + candidate selection,
  deterministic gates, registered executable callbacks and outcome verification.
- `farm.control.jev.MotionController`: 28 position nudges across 14 servos
  (both arms/grippers and head), plus eight wheel actions including independent
  wheels, forward/backward and turning. Targets, speeds and durations are set
  locally, not generated by Jev.
- `farm.control.feetech.ConnectedFeetechActuator`: real raw-register writes and
  acknowledgements inside an **already connected sole motor owner**. It compares
  saved/device calibration, preserves operating modes/torque state, and never
  invokes the legacy connection/configuration path or normalized IK.
- `python -m farm.control.cli --catalog` and `--simulate --goal ...` for the
  complete action interface without opening hardware.

Use the broad nudge catalogue during commissioning only with locally checked
clearance and direction mappings. Production carton operation should expose
phase-specific primitives, not all 36 actions on every inference call.

Install the supervisor and connected actuator **inside** the remote motor-owning
process with its shared I/O lock, STOP callback and deadman. Register only the
primitives that process has actually validated. Do not start a second serial
client beside the current carton holder. Extend that holder to own both buses
before activating both arms, head or wheels; the current one-arm session cannot
be treated as a whole-robot owner.

This branch does not install itself into `work/carton_session.py`, enable wheel
torque/modes, commission trajectories or move the connected robot. The old
parked cycle's default profile remains parked. Simulated register tests prove
command mapping and failure handling, not motor commissioning.

Wheel duration is a local command window followed by acknowledged zero goals;
serial latency and mechanical deceleration add to physical stopping time.
Python `finally` cannot guarantee a stop after process death, OS failure or lost
serial communication. The owner must retain its independent deadman and physical
stop; this is not a hardware safety controller. Readback timestamps cover the
start of each serial sweep and stale sweeps are refused.

## What should happen next on the carton

First finish establishing one repeatable paddle pickup: correct camera geometry,
known joint-direction mapping, bounded approach, observed grasp and safe lift.
Then integrate that primitive and its local tracker into the persistent owner
and run Jev at its action boundaries. Keep Astra for setup and failures. Extend
the same pattern to a short flap and left-arm retention before full closure.

Measure capture age, perception time, Jev time, movement time, verification time,
interventions and verified completion separately. Compare existing per-move
reasoning, fixed primitives, and the hybrid on the same station and task scope.
The fixed-primitive baseline matters: a known sequence may be fastest without
any model at those boundaries. Only a physical, outcome-verified comparison can
establish the end-to-end speedup.

API references: [TypeSafe API](https://docs.typesafe.ai/api),
[models, modalities and price](https://docs.typesafe.ai/models),
[confidence](https://docs.typesafe.ai/confidence).
