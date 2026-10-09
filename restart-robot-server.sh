#!/usr/bin/env bash
# One-time, operator-authorized Joy-Con admin bootstrap. Never restarts the motor owner.
set -euo pipefail
cd "$(dirname "$0")"
PY="${XLEROBOT_PYTHON:-/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python}"
[ -x "$PY" ] || PY=python3
if [[ "${1:-}" == "--cameras-only" || "${1:-}" == "--dry-run" ]]; then
  exec "$PY" software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/oak_commissioning.py "$@"
fi
exec "$PY" software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/bootstrap_joycon_admin.py "$@"
