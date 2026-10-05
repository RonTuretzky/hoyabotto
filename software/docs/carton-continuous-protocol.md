# Continuous owner protocol v1

This is the integration contract for `carton.servo.continuous`, exercised by
`tests/test_carton_continuous.py`. It does not open a motor port or compete with
the existing session's sole writer. Integrate the executor into that owner
locally; publishing the owner's private implementation is unnecessary.

## Owner startup

Load the commissioned recipe/profile from disk before enabling motion. Call
`validate_profile(profile, config)` and construct
`TrajectoryExecutor(profile, write_goals, stop, clock=monotonic, wall=time.time)`.
`write_goals` is the owner's existing synchronous **raw encoder tick** write
for the selected arm. It must never use normalized LeRobot position units.
`stop` is the existing STOP/release policy. Publish `executor.capabilities()`
in status, alongside the existing arm/session/health/row/lease fields.

Require one command writer and the existing lease/STOP/strict coherent telemetry
checks. Keep `automatic_gripper_reenable: false`; increment integer
`gripper_release_generation` whenever jaw torque is released. Do not run the
old release-and-reenable temperature path invisibly during a grasp.

Profile fields:

| Field | Meaning |
| --- | --- |
| `schema`, `units`, `arm` | `1`, `encoder_ticks`, selected arm |
| `joints` | Exactly all six selected-arm joint names, including gripper |
| `fingerprint` | `binding(config)` for this measured station/camera/calibration |
| `commissioning_evidence` | Reference to physical corridor/rate validation |
| `corridor` | Per-joint measured clear interval inside saved range margins |
| `velocity`, `acceleration` | Positive per-joint ticks/s and ticks/s², measured for the loaded mechanism |
| `following_ticks` | Measured following tolerance, at most 24 ticks |
| `start_ticks`, `settle_ticks` | At most 5 ticks |
| `vision_age_s` | Measured camera watchdog age, at most 1 second |
| `max_tick_gap_s` | Measured owner loop watchdog, at most 0.2 seconds |
| `settle_timeout_s` | Endpoint timeout, at most 5 seconds |

A profile is a robot-local commissioned constraint, not a client override. The
executor copies it at startup and checks its SHA256 in every command. Do not
load new profile limits from incoming command payloads. The measured corridor
must account for the whole swept volume, table, tool and payload; raw servo
ranges alone do not establish clearance.

## Command and ongoing execution

Atomic `command.json`:

```json
{"id":123, "op":"trajectory", "session_started":900,
 "profile_sha256":"<owner-published digest>",
 "camera_streams":{"head":"<stream>","right_wrist":"<stream>"},
 "waypoints":[{"time_s":0,"positions":{"<all six joints>":0}},
              {"time_s":2,"positions":{"<all six joints>":0}}]}
```

The numbers above describe a schema, not a robot pose. The client atomically
publishes `trajectory-vision.json` before dispatch and refreshes it throughout:
`command_id`, `session_started`, `ok`, `streams`, `sequences`, and
`captured_at` (per-camera capture timestamps, not processing time).

On receiving a trajectory while holding, call:

```python
accepted = executor.start(command, coherent_rows, vision,
                          session_started=session_started,
                          lease_remaining=lease_remaining)
```

Merge the returned `accepted` and `phase` into status. A refused start sends no
goals. Then call on **every** motor-owner cycle:

```python
update = executor.tick(coherent_rows, vision,
                       telemetry_at=actual_telemetry_capture_time,
                       stop_requested=existing_stop_or_expired_lease)
```

Merge the returned fields and publish fresh status. Tick uses a monotonic clock
for trajectory time; camera/status timestamps use the same host's wall clock.
Supply actual coherent telemetry capture time, never relabel old rows with now.
Do not put sleeps, image encoding or LLM requests in this loop. The executor
writes coordinated setpoints each tick, checking health, encoder following,
vision age/identity and loop timing **before** that write. It settles only the
final endpoint for three distinct telemetry samples. Match `completed` exactly
to the command id. Returned `trajectory_metrics` include goal-write count,
retimed duration, elapsed duration, travel and maximum tick gap.

An exception in tick invokes the STOP callback, deactivates execution, and must
be handled as a failed command by the containing owner. Never retry, clear STOP,
or reset fault budgets automatically. The enclosing owner must retain its
independent lease and hardware safety checks even if the client disappears.
Once the executor finishes, continue normal health/lease supervision while
holding; do not release between program stages.

Normal terminal STOP must publish the matching `completed` id, `released: true`,
`release_errors: []`, fresh `time` and `phase: stopped` or `released` **only after
verified release of the owner's motors**. A request alone is not confirmation.
The client refuses a successful pickup-cycle result without this readback.

## Deployment verification

Run the focused suite in the same environment before touching hardware:

```sh
python -m pytest tests/test_carton_continuous.py tests/test_carton_servo.py tests/test_carton_program.py tests/test_carton_depth.py tests/test_carton_tags.py tests/test_carton_trajectory.py tests/test_oak_camera.py -q
```

Then verify the owner's actual adapter with its transport fake, followed by live
read-only owner/camera health. Refresh moved-station seeds and commission the
profile and six pickup paths. Test live STOP/vision interruption at an unloaded
clear pose, measure loop gap and stop latency, and only then run pickup.
Record deployed commit/profile digest, command count, real elapsed time,
gripper health, synchronized grasp/lift evidence and final release.

The fixture's 40-degree command and sub-30-second complete cycle are **synthetic
software checks**. They do not establish real motor speed, collision clearance,
contact physics, temperature validity, OAK availability or a physical grasp.
