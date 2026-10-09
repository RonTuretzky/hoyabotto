# Emoji show: visitors pick an emoji, the robot performs it

Visitors pick preset emojis on a kiosk page and type their name. The robot performs each emoji's gesture, and
their name is on the big screen while it does. The presets are **Quick wave** and **Full wave**, both 👋 on the right arm.

It runs on the **chat Mac**, the one that holds the paired client certificate
(`gemma-xlerobot/pilot/.private/robot.json`). It moves the robot only through the existing robot API (`POST /call`
on the sole hardware owner). It opens no serial port, starts no owner and changes no limits. It reuses the existing paired mTLS client and its installed Python environment.

```sh
cd software
source /Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/.venv/bin/activate
python -m robot_emoji                  # real robot; kiosk on http://localhost:8790/
python3 -m robot_emoji --fake           # no robot: simulated moves at the owner's pace
python3 -m robot_emoji --host 0.0.0.0   # phones on the same network can reach the kiosk
```

| Page | For |
|---|---|
| `/` | Kiosk: pick up to 3 emojis, type a name (at most 24 characters), get a place in line |
| `/screen` | Big display: whose turn it is, the current phase, who is next |
| `/operator` | Arm/pause, remove from the queue, log, and a big **STOP** |

Operator actions are accepted from this Mac or with the token printed at startup (`/operator?token=…`).

## hoyabotto.com integration

Visitors use **https://hoyabotto.com/wave.html**; the big display uses
**https://hoyabotto.com/screen.html**. The homepage has a small Wave link. GitHub Pages continues serving
the presentation and all static pages from the existing repository's `gh-pages` branch.

The visitor pages call a separate HTTPS endpoint that tunnels to the **public-only listener on 8791**.
Both listeners share the same queue and performer. Operator controls stay at
**http://127.0.0.1:8790/operator** on this Mac. The public listener always rejects operator actions,
including from a tunnel's loopback connection and even with a valid operator token. It never returns
operator logs or robot connection metadata. CORS allows the Hoya Botto site origins; public submissions
are limited to one per visitor IP per 30 seconds, and the queue remains capped at 30.

```sh
cd software
python3 -m robot_emoji --public-port 8791
# In a separate terminal (expose ONLY 8791, never the operator port):
cloudflared tunnel --url http://127.0.0.1:8791
```

Export pages into a checkout of the **existing** `gh-pages` branch, using the HTTPS origin printed by
cloudflared (or a named tunnel's stable HTTPS origin):

```sh
python3 -m robot_emoji.publish_site /path/to/gh-pages-checkout --api-url https://YOUR-VISITOR-ENDPOINT
```

The exporter copies only `wave.html`, `screen.html`, `emoji-api.js`, and `emoji-config.js`, then adds the
homepage link once. Commit those assets and the homepage on `gh-pages` and push to publish. The config
contains only the public URL, no certificate or operator token. Never copy software or runtime logs to Pages.
The presentation's slide contents stay intact; its file hash changes because the link is appended.

The presentation itself also polls the same public state. The active participant's name, emoji and phase
appear in a fixed overlay **on every slide at hoyabotto.com**, including after slide changes. The overlay
hides when idle or offline and briefly says thanks after completion. All 14 supplied slides retain their
content and navigation. The visitor link opens a new tab so it does not replace the presentation.

### Fixed cloud endpoint and automatic startup

The fixed API is **https://hoya-botto-show.ronturetzky.workers.dev**. GitHub Pages still hosts the slides.
A small Cloudflare Worker forwards only visitor queue/status routes; a Durable Object persists the current
connection origin. Authenticated registration updates that origin whenever the Mac's internal Quick Tunnel
changes. The website URL and config stay the same across Mac restarts; no manual publishing is needed.
The cloud registration secret is stored as a Worker secret and in a private local token file. It cannot arm
the robot, and is never included in the website.

The live runtime is installed outside the Conductor workspace, in:

- `~/Library/Application Support/HoyaBotto/robot_emoji/` — copied service and supervisor
- `~/Library/Application Support/HoyaBotto/queue.json` — persistent waiting requests and outcomes
- `~/Library/LaunchAgents/com.hoyabotto.emoji-show.plist` — automatic startup at **login after restart**

The supervisor restarts the show/tunnel after a process failure and reconnects the fixed cloud endpoint.
The show always starts paused. Requests waiting in line survive restart; an interrupted performance is
marked failed instead of being replayed. Names remain public, and a visitor still needs the robot Mac and
the chat-Mac service online for a physical performance. The paired hardware client is reused directly from
`chat_server.Robot` in its existing venv, with LAN discovery disabled for this runtime. The existing internet
relay is used with the pinned server certificate and client certificate. No SSH or hardware-server restart
is required. The server thread continues to own the remote relay setup and the sole hardware owner.

To update/reinstall the copied runtime while the show is paused, from `software/` with the existing venv:

```sh
python -m robot_emoji.install_runtime --api-url https://hoya-botto-show.ronturetzky.workers.dev
```

A restart of this launch agent is a useful recovery test; it is not a literal Mac reboot test. Keep the
Mac awake while the robot show is running. Startup requires the user's normal login/unlock.

The earlier manual Quick Tunnel commands above remain useful for standalone development. For this
installed runtime, the supervisor manages and registers its internal tunnel automatically.

Test the public form with the show paused, verify the ticket on the display, then remove the test request
locally. Do not arm the real robot just to smoke-test website integration. A successful queue submission
does not validate a physical wave.

## What one performance does

1. **Check.** `robot_get_motion` and `robot_get_state` are read. If anyone else holds a motor, or anything is
   moving, the request waits ("robot in use") and nothing is sent. Every target is checked against the live
   `commandable_ranges`, and none is clamped.
2. **Enable.** All six joints of the arm are enabled in one `robot_set_motor_enable`, as the pickup profile
   requires. They hold where they are.
3. **Raise, then gesture.** `robot_move_path` runs with `wait=true` for each gesture: its raise, then its motion.
4. **Return and release.** The arm goes back to where it rested, clamped into the commandable range, and only
   its six joints are released.

The owner's maximum permitted pace is 40-tick steps every 0.4 s (100 ticks/s, about 9 degrees/s).
Both presets request that pace. Full wave was observed completing in about 61 s. Quick wave uses a
halfway raise pose and one smaller sway: its plan is 9.6 s raise, 4.4 s wave, 9.6 s return at the
October 9 resting pose (23.6 s movement, roughly 30 s including settling/network time). The Quick
wave duration is an estimate until its supervised hardware test. Firmware maximum speed was not
set; reaching it would require changing the robot's guarded speed profile.

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

The Full wave's poses were chosen in the digital twin (`farm/sim/xlerobot_twin.py`):
- **Raised hand:** pan 2100, lift 2300, elbow 1150, wrist flex 1600. The upper arm leans forward, and the
  forearm and hand point up, to the right of the mast. The twin puts the claw about 1.27 m above the floor.
- **Wave:** the wrist flaps ±170 ticks while the shoulder pan sways between 1990 and 2210 (the hand moves about
  8 cm outward).

Full wave completed on the real robot on October 9: raise, wave and return all reported
`completed: true`, `endpoint_settled`, followed by release. The operator reported it worked perfectly.
Its `verified_on_hardware` flag is true; the general twin mapping is still unvalidated.

Quick wave is now the default `wave` preset. Its starting pose is halfway along that successful
raising route, with shoulder pan ±55 ticks and wrist flex ±85 ticks for one sway. It stays marked
`verified_on_hardware: false` until its own supervised test. The original full-height sequence is
preserved unchanged as `full_wave`. Watch the whole right arm on the first Quick wave with STOP
and the 12 V switch in reach.

Tests: `tests/test_robot_emoji.py` (plans, the call sequence against a fake owner, busy/refusal/fault/STOP
handling, the queue and the web routes).
