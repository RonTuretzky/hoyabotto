#!/usr/bin/env bash
# Robot Mac only: take down the running Qwen robot server (API + sole hardware owner) and bring
# up the repo's qwen-bridge version with --right-arm-only --paddle-profile. Motors start released.
#   ./restart-robot-server.sh --dry-run          tests + show what would change; stops nothing
#   ./restart-robot-server.sh                    restart (refuses if motors are holding)
#   ./restart-robot-server.sh --release-holding  also stop an owner that is holding (arm drops)
# Run from Terminal: the API serves camera frames and macOS camera permission is per app.
set -euo pipefail
cd "$(dirname "$0")"
PY="${XLEROBOT_PYTHON:-/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software/.venv/bin/python}"
[ -x "$PY" ] || PY=python3
exec "$PY" software/docs/commissioning/2026-10-07-paddle-success/qwen-bridge/redeploy_robot_server.py "$@"
