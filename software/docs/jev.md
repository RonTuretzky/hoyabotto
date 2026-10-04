# Native Jev integration

Jev now uses `POST https://openrouter.ai/api/alpha/decisions` with
`typesafe/jev-1.13`. It receives compact text evidence and typed Choice questions.
It does not receive camera images, generate explanations, or write motor commands.
Vision and planning remain separate backends.

The previous `typesafe/jev-router` setting called a chat router and extracted
generated JSON. Existing profiles with that value are migrated in memory to the
native model with a warning. Update the saved profile when convenient.

## Persistent interface for the connected Mac

From `software/`, with `OPENROUTER_API_KEY` already available in the environment
or the existing untracked `.env`:

```sh
python -m carton.cli jev-advice -p carton-v0 --input observations.jsonl
```

Omit `--input` to read one JSON observation per line from stdin. The process
keeps one HTTP connection pool and returns one JSON recommendation per line,
flushing each response. It never creates `System` or opens serial ports/cameras,
so it can run alongside the existing motor-owning process. The robot-side
script can keep it open and send snapshots through stdin.

Each input is an observation captured by the current controller:

```json
{"observation_id":"camera-session:123","observed_at":1791099000.0,"stage":"VERIFY","step":"pick_paddle","attempt":1,"health_ok":true,"stop_requested":false,"missing_keyframes":[],"held":{"right":null},"judgement":{"box_present":true,"obstruction":false,"paddle_held":false,"notes":"Claw closed beside the paddle; pickup did not succeed."}}
```

Replace the example ID and timestamp with the real observation identity and
capture time. **Do not stamp old images with the current time.** Preserve frame
timestamps when vision processing is slow. Invalid, stale, future-dated or
incomplete observations cannot produce a `continue_plan` recommendation.
Robot health and STOP status must come from the local controller, not the model.

Outputs always include `mode: "shadow"`, `motion_authorized: false`, and the
input observation ID/time. The action is one of:

| Action | Meaning |
|---|---|
| `continue_plan` | Suggest continuing through the existing controller's checks |
| `refresh_view` | Acquire useful camera evidence |
| `request_plan` | Establish missing poses or revise a failing strategy |
| `stop` | Request the controller's existing stop behavior |
| `unknown` | No usable decision; do not infer permission |

`source` distinguishes deterministic rules from Jev. Native decisions include
the served model, probability distribution, confidence, latency, cost and request
ID. `error` identifies unavailable inference. A confident recommendation is not
physical validation or authorization. The existing motor-owning process remains
responsible for action eligibility and applying safety checks.

The default `--max-cost-usd 0.05` is a session dispatch cap on reported usage;
one request may overshoot it. It does not change provider key/account limits.
Exit status is nonzero if an input is invalid, the session budget is exhausted,
or inference is unavailable. The stream keeps responding to subsequent lines.

## Carton-cycle comparison

For an already commissioned station, add `--jev-shadow` to the existing
`carton once` or `carton run` command. Those commands retain their normal hardware
behavior. The flag only adds background comparison; it is **not** required for
the motor-free `jev-advice` interface.

Configuration is also available in the existing profile:

```yaml
llm:
  jev_model: typesafe/jev-1.13
  jev_timeout_s: 2.0
  jev_observation_max_age_s: 5.0
  jev_carton_shadow: false
```

At camera-judgement boundaries, the carton cycle snapshots text evidence and
submits at most one advisory request in the background. There is no backlog.
Newer observations supersede older results; all recorded decisions have
`honoured: false`. A model suggestion never changes the cycle, replaces an
outcome check, permits a tape retry, or claims successful folding. The viewer's
state includes `jev_advice`; SQLite stores `carton_jev_shadow` decisions/events.
Comparison is off by default and respects the existing daily LLM budget.

The native client has its own short timeout, keeps one connection pool, and
does not retry automatically. Auth, credit and key-limit errors block further
API calls for that client. After fixing the key or its limit, restart the advice
process. Transient errors use a 30-second cooldown. Provider response bodies and
credentials are not written to logs. Malformed probabilities are rejected, not
normalized into apparent confidence. Late successful replies retain their
reported cost but cannot authorize progress.

The plant-care cycle's existing review and pour questions now share one native
request. Its safety and authorization checks remain in force. Stored authority
from the old chat wrapper resets to shadow, and old agreements cannot promote
the native model. Carton advice does not use that plant-care promotion ladder.

## Verification and remaining work

Tests cover native wire format, pooling, batching/cost accounting, malformed
answers, quotas, timeouts, stale evidence, bounded background work, and the
carton simulator's unchanged action/evidence sequence. These use mock transport;
they do not demonstrate physical folding or provider inference performance.

The first live evaluation on October 4, 2026 authenticated successfully but
inference returned HTTP 403, `Key limit exceeded (daily limit)`. No successful
Jev latency or accuracy measurement is claimed. No credential is included in
this branch.

Next, connect the motor-free stream to the robot's current observation producer,
compare recommendations with labelled outcomes, and measure perception, API,
actuator and verification times separately. Enabling Jev to select executable
primitives still requires a station-specific candidate/validation interface.
Missing pickup/folding poses and the angle-unit mismatch are separate work.
This integration does not accelerate the outer Codex chat loop by itself.

Sources: [OpenRouter Jev guide](https://openrouter.ai/docs/guides/community/jev),
[Decisions API](https://openrouter.ai/docs/api/api-reference/alphadecisions/submit-a-decisions-request),
[TypeSafe confidence](https://docs.typesafe.ai/confidence).
