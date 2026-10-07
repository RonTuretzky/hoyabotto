# Qwen robot tool server: recovery and integration

## Current verified service

Robot API: `https://127.0.0.1:1241`, end-to-end mutual TLS, exact paired client certificate pinned. Cloudflare TCP relay: `organisation-fish-jvc-shopper.trycloudflare.com` as of this recovery. Quick-tunnel hostnames change whenever the relay restarts, so read the current one from `restart_relay.py` output. Chat Mac local proxy remains `https://127.0.0.1:1242`; this is a different computer and port from the robot API. The browser UI remains `http://127.0.0.1:1241/` on the chat Mac.

One canonical `HardwareOwner` holds both servo ports. It starts all16 released, skips the existing historical command file, and supports motion only for six right-arm joints. Fresh client readiness reports `motion_ready: true`, no right-arm blockers, all16 torque zero, and zero recovery motor writes. No setup movement test ran. The default full-scope owner still refuses saved-versus-hardware calibration mismatches. The right-only mode does not rewrite or ignore a mismatch on a commandable motor; it marks all other motors read-only and refuses their activation before a write.

All20 existing tool schemas are in `tool-schemas.json`. State, capabilities, motor inventory, readiness, execution and handoff local dispatch passed. Camera dispatch returned fresh OAK and phone frames; depth returned a fresh paired PNG. The API uses the current 640×360 OAK distorted raw stream explicitly as fallback. It does not relabel raw data as rectified or validated RGB/depth registration. Phone `received_at` means server receipt, with unknown camera capture delay.

Actual positive authenticated HTTP health/tools/call from the other Mac is **not yet verified**. Its current configured relay hostname is stale. The robot's old local test client certificate is expired and unpaired; it was not substituted for the paired chat Mac identity. Missing and unpaired client certificates are rejected. Current robot/paired-client public certificates are valid through November5 and were not changed during recovery. Private keys were not copied or published.

## Robot-side startup/restart commands

**Preferred: `./restart-robot-server.sh` from the repo root, in Terminal.** It runs the fake-hardware tests, refuses if motors are holding, and stops the API and then the sole owner (which releases every motor on exit). It installs the repo's `qwen-bridge` files into `work/`, keeping the old copies in `work/backups/`, and starts a fresh owner with `--right-arm-only --paddle-profile` and then a fresh API. It also starts `work/capture-single` for any wrist camera without a fresh stream, writing to `work/wrist-camera-stream/`. Use `--dry-run` first to see what would change. The new owner starts with all 16 motors released. It is no longer needed to recover from a STOP or fault: the owner has no STOP latch. `robot_stop` or any fault releases every motor and cancels the move in progress; the owner returns to idle and records `last_stop` and `stop_count`. Motors stay released until an explicit `robot_set_motor_enable`, which repeats every health, range, camera and voltage check. If a release fails, readiness blocks on `OWNER_NOT_HEALTHY` (a hardware problem): call `robot_stop` again and inspect the motor before enabling. The script remains the way to deploy new code. Afterwards `robot_get_execution` must show `execution_profile: paddle-success-v1`. The relay, phone and OAK services are left running.

The individual scripts below remain. Run them from `/Users/teachera/Documents/Codex/2026-10-05/m` using `/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python`.

- `python work/restart_gemma_owner_released.py --right-arm-only --paddle-profile` starts the existing sole owner only when no existing owner is running. It refuses pre-enabled motors and mismatches within its right-arm scope; it never calibrates. Log: `work/gemma-hardware-owner.log`; process record: `work/gemma-hardware-owner-process.json`; live atomic status/commands: `work/gemma-hardware-session/`.
- `python work/reload_gemma_api.py` verifies the recorded API process identity, replaces only that API, and starts it detached with passive fallback and the current OAK source. Log: `work/qwen-server-recovery/api.log`; record: `work/gemma-robot-tools-process.json`.
- `python work/qwen-server-recovery/restart_relay.py` reuses the live relay when present or starts `work/bin/cloudflared tunnel --url tcp://127.0.0.1:1241 --no-autoupdate`. Log: `work/qwen-server-recovery/relay.log`; record: `work/gemma-hardware-relay-process.json`. A newly started quick tunnel can change the hostname.

Before replacing an owner, independently inspect its actual process identity, fresh all16 torque-zero status, and serial-port ownership, then terminate only that owner and wait for the ports to close. Never run a commissioning prototype or another serial reader beside it. API/relay restarts must not restart/arm motors or replay historical commands. Do not run the owner restart command blindly if a current owner is active.

The camera services were reused, not restarted: phone HTTP/HTTPS8876/8877 in `work/phone_camera/`; current OAK stream in `/Users/teachera/Documents/Codex/2026-10-06/users-teachera-documents-codex-2026-10/work/oak-live-stream/`. Camera upload tokens and URLs remain private. Freshness must be checked for every action, not inferred from this dated verification.

## Exact chat Mac change

In `/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json`, replace only the relay hostname `enquiries-sections-gsm-passage.trycloudflare.com` with `organisation-fish-jvc-shopper.trycloudflare.com`. Preserve the existing API URL `https://127.0.0.1:1242`, client certificate/key paths, server trust certificate and all other configuration.

Restart the existing relay client, targeting the new hostname. The underlying command is:

`cloudflared access tcp --hostname organisation-fish-jvc-shopper.trycloudflare.com --url 127.0.0.1:1242`

Stop only its prior proxy listener before binding1242; do not create two proxies on that port. Restart/reload the chat application's existing robot client if it caches the config. Do not change or print the Cerebras key; model remains `qwen-3.8-27b`, with local Gemma for camera interpretation.

Using the chat Mac's existing paired credentials, verify `GET /health`, `GET /tools`, then read-only `POST /call` requests for `robot_get_state`, `robot_get_capabilities`, `robot_list_motors`, `robot_get_cameras`, `robot_get_depth` and `robot_get_handoff`. Envelope is `{"name":"tool_name","arguments":{},"request_id":"unique-id"}`. Every attempt needs a unique request ID; do not blindly retry an uncertain movement or use historical requests as new actions. Preserve returned image formats, timestamps and controller error details.

Expected current health: active sole owner, motion-ready right scope, six supported right joints, all16 released. Activation is explicit; none occurs during health/discovery. Left/head/wheel activation is refused in this scope. `robot_move_base` accurately reports unsupported. FK/reach tools report missing actual model binding rather than manufacturing a Cartesian target.

## Remaining blockers and Qwen pickup integration

Four left hardware entries differ from the file: shoulder lift, wrist flex, wrist roll and gripper. Actual/expected values are in `server-recovery.json`. Do not copy either set blindly or recalibrate as part of connection setup. Full left/head/base motion requires a deliberate separately reviewed scope/session.

The [successful physical pickup process](README.md) and `success.json` are also served by `robot_get_handoff` as `physical_pickup_handoff`, alongside the original historical handoff. This gives Qwen the exact13 segments, held targets, settings, baseline/contact distinction and success criteria after the relay connects. Publication/availability is not evidence that the Qwen chat has fetched it.

The generic existing direct owner keeps its original conservative torque, following-error, interpolation/watchdog, jaw endpoint and health checks. Its 70°C software check applies only to that `legacy-direct` profile; the pickup profile has no temperature check (temperature handling was removed on 2026-10-05). Do not assume the two controller profiles are equivalent. The verified close-until-resistance and shoulder P32 profile are integrated as owner profile `paddle-success-v1` ([qwen-bridge/PADDLE-PROFILE.md](qwen-bridge/PADDLE-PROFILE.md)), not yet physically re-validated. Attempt the sequence with fresh visual feedback. Do not globally loosen direct-joint checks, execute the archived standalone pilot alongside the owner, or treat the tick sequence as a universally safe keyframe.

No message was injected into the remote browser chat: its loopback UI and paired private key are on the other Mac, and the previously configured LAN gateway did not respond. The tool handoff is installed and ready to be fetched once that Mac updates its relay. The requested40cm cart travel remains outstanding; server recovery did not drive the base.
