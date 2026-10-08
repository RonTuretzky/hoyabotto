#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
swift build -c release
# Reuse the installed simulator environment; this launcher cannot select robot transport.
sim_python="${JOYCON_SIM_PYTHON:-/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/.venv/bin/python}"
if [[ ! -x "$sim_python" ]]; then
  echo "Set JOYCON_SIM_PYTHON to a Python with mujoco, numpy, and pillow installed." >&2
  exit 1
fi
for argument in "$@"; do
  if [[ "$argument" == --connect-robot* || "$argument" == --simulator* ]]; then
    echo "This launcher is for local MuJoCo simulation only." >&2
    exit 2
  fi
done
exec "$sim_python" teleop/bridge.py --simulator mujoco "$@"
