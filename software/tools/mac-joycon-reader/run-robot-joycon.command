#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ -e MOTOR_CONTROL_DISABLED ]]; then
  echo "Motor control is disabled for this installation. Local simulation remains available."
  exit 2
fi
explicit_connection=false
for argument in "$@"; do
  if [[ "$argument" == --connect-robot ]]; then explicit_connection=true; fi
done
if [[ "$explicit_connection" != true ]]; then
  echo "Robot connection is opt-in. Use run-simulator.command for local practice."
  echo "After commissioning: $0 --connect-robot --upstream-reference /path/to/measured-reference.json --config /path/to/robot.json"
  exit 2
fi
joycon_python="${JOYCON_SIM_PYTHON:-/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/.venv/bin/python}"
exec "$joycon_python" teleop/bridge.py --control-mode upstream --input-backend hid "$@"
