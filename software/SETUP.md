# Setting up a new laptop to run the robot

Everything a blank laptop needs to run the XLeRobot farm program, in order. Written for an agent or a person; about 30 minutes plus downloads (~2 GB).

Assumes an Apple-silicon Mac (M1 or newer). Linux differences are at the end. Intel Macs will not work (PyTorch no longer ships builds for them); Windows is untested.
No Raspberry Pi is used: the laptop talks to the robot's two motor boards, the cameras and the light sensor over USB.

| # | Install | Why | Who |
|---|---|---|---|
| 1 | Homebrew, git, uv | package and Python tooling | agent |
| 2 | This repo + Python 3.12 environment | the robot software (LeRobot, OpenCV, viewer) | agent |
| 3 | Claude Code CLI | vision and LLM teaching on the Claude subscription | agent installs, **owner logs in** |
| 4 | `software/.env` | OpenRouter key for Jev and Astra; optional Telegram | **owner** |
| 5 | Camera permission for the terminal app | cameras cannot open without it | owner clicks Allow |
| 6 | arduino-cli + ESP32 board package | flashing the light sensor (once) | agent |
| 7 | Robot data from the old laptop | calibration and taught poses, if any exist | owner copies |

## 1. Base tools

```sh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# follow the two "Next steps" lines Homebrew prints, then open a new terminal
brew install git uv
```

Homebrew asks for the Mac's password and installs Apple's command-line tools if they are missing. Python itself is not installed separately: uv downloads 3.12 in step 2.

## 2. The farm program

```sh
git clone https://github.com/RonTuretzky/xlerobot-farm.git
cd xlerobot-farm/software
uv venv --python 3.12 .venv && . .venv/bin/activate
uv pip install -e ".[dev]"        # ~1.2 GB: lerobot, torch, opencv, fastapi
python -m pytest -q               # expect: 42 passed (1-2 minutes)
farm sim --auto-answer            # whole program on fake devices; viewer at http://localhost:8765
```

The repo is public, so no GitHub login is needed to clone. In every new terminal, run `cd xlerobot-farm/software && . .venv/bin/activate` before using `farm`.

## 3. Claude Code CLI

```sh
curl -fsSL https://claude.ai/install.sh | bash
claude            # the account owner completes /login once, then exits
claude -p "reply with the word ok" --output-format json     # check
```

Quickstart, if the installer changes: https://docs.claude.com/en/docs/claude-code/quickstart

## 4. Secrets (the account owner types these)

```sh
cp .env.example .env
# edit .env:
#   OPENROUTER_API_KEY=...        required (Jev, Astra)
#   TELEGRAM_BOT_TOKEN=...        optional (questions to your phone)
#   TELEGRAM_CHAT_IDS=...         optional
farm review                       # check: prints an Astra proposal
```

Make a fresh key at https://openrouter.ai/settings/keys with a small credit limit rather than copying the old one. `.env` is never committed.

## 5. Camera permission

Plug in a camera, then from the terminal app you will actually use:

```sh
farm devices --probe
```

Click **Allow** on the camera prompt. If no prompt appears: System Settings → Privacy & Security → Camera → enable the terminal app. Permission is per app, so run camera commands from that same app.

## 6. USB devices

**Motor boards:** no driver. On a Mac they appear as `/dev/cu.usbmodem5B790182091` and `/dev/cu.usbmodem5B790186401` (the name comes from the board's serial number, so it is the same on any Mac). `farm devices --probe` lists them with the servo IDs on each.

**Light sensor (ESP32 + BH1750), flash once:**

```sh
brew install arduino-cli
URL=https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index --additional-urls $URL
arduino-cli core install esp32:esp32 --additional-urls $URL
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_light
arduino-cli board list                                   # find the ESP32's port
arduino-cli upload -p /dev/cu.XXXX --fqbn esp32:esp32:esp32 firmware/esp32_light
farm light-monitor                                       # check: lux readings
```

The firmware uses only the built-in `Wire` library. If the ESP32 shows no port when plugged in, install the driver for the chip printed next to its USB socket: [CP210x](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers) or [CH34x](https://github.com/WCHSoftGroup/ch34xser_macos). If the sensor was already flashed from another laptop, skip this section entirely.

## 7. Bring over robot data (only if it exists)

A fresh clone has the code and profiles but not what was learned on the robot. If the robot was already calibrated or taught from another laptop, copy these across:

| On the old laptop | What |
|---|---|
| `software/data/` | taught keyframes, evidence database, overrides, recordings |
| `~/.cache/huggingface/lerobot/calibration/` | arm calibration |
| edits to `software/profiles/paper-tray-v0.yaml` | ports and camera indices (commit and push them instead, if possible) |

If none of that exists yet, there is nothing to copy: run `farm calibrate` on the new laptop.

## Not needed

Raspberry Pi image, ROS, the upstream XLeRobot repo (the files used are vendored in `farm/vendor/`), the Windows Feetech FD program (`farm set-motor-id` replaces it), a 3D-printer slicer (only for printing parts).

## Final check

```sh
python -m pytest -q                                          # 42 passed
claude -p "reply with the word ok" --output-format json      # JSON with "ok"
farm review                                                  # Astra proposal
farm devices --probe                                         # motor boards + servo IDs, camera snapshots
farm light-monitor                                           # lux readings (after flashing)
```

For long runs keep the laptop awake and on mains power: `caffeinate -dims farm run --every 3600`.

## Linux differences

- Step 1: install git with the package manager and uv with `curl -LsSf https://astral.sh/uv/install.sh | sh`.
- Motor boards appear as `/dev/ttyACM0`, `/dev/ttyACM1`; add yourself to the serial group once (`sudo usermod -aG dialout $USER`, then log out and in).
- No camera permission prompt. `caffeinate` does not exist; use `systemd-inhibit farm run --every 3600`.
- Install arduino-cli from https://arduino.github.io/arduino-cli/latest/installation/.

## Stop and ask instead of proceeding if

- Anything asks for an API key, account login or payment (the owner does those)
- A driver install asks to change security settings or reboot into recovery
- `uv pip install` or the tests fail (do not switch Python versions or delete things to work around it)
