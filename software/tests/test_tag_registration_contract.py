"""Runs the qwen-bridge mover/owner contract (fake hardware) under this interpreter.

The contract itself lives beside the robot server so the restart script's test step runs it too:
docs/commissioning/2026-10-07-paddle-success/qwen-bridge/test_tag_registration_contract.py
"""
from pathlib import Path
import subprocess
import sys

BRIDGE = Path(__file__).resolve().parents[1]/'docs/commissioning/2026-10-07-paddle-success/qwen-bridge'


def test_registration_mover_against_todays_robot_server():
    run = subprocess.run([sys.executable, 'test_tag_registration_contract.py'], cwd=BRIDGE,
                         capture_output=True, text=True, timeout=300)
    assert run.returncode == 0, run.stdout[-4000:]+run.stderr[-4000:]
    assert "'hardware_access': False" in run.stdout
