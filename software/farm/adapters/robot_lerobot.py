"""Real robot adapter: XLerobot2Wheels over two Feetech buses, no prompts, no wheels.

Differences from the upstream class's own connect():
- calibration is restored from the saved file without an input() prompt; if it is
  missing we refuse to connect (run `farm calibrate` once, with the arms supported).
- cameras are handled by CameraAdapters, not by the robot object.
- wheels stay at zero velocity; a watchdog stop is available at any time.
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode

from ..config import RobotCfg
from ..status import Reading, Status, invalid
from ..vendor.config_xlerobot_2wheels import XLerobot2WheelsConfig
from ..vendor.xlerobot_2wheels import XLerobot2Wheels
from .base import ARM_JOINTS, HEAD_JOINTS

log = logging.getLogger(__name__)


class LeRobotXLeRobot:
    name = "robot"

    def __init__(self, cfg: RobotCfg):
        if not cfg.port1 or not cfg.port2:
            raise ValueError("robot.port1 and robot.port2 must be set in the profile (see `farm devices`)")
        self.cfg = cfg
        rc = XLerobot2WheelsConfig(id=cfg.id, port1=cfg.port1, port2=cfg.port2, max_relative_target=cfg.max_relative_target, cameras={})
        if cfg.calibration_dir:
            rc.calibration_dir = Path(cfg.calibration_dir)
        self.robot = XLerobot2Wheels(rc)
        self._connected = False
        self.calibration_id = "uncalibrated"

    # ---- lifecycle ---------------------------------------------------------
    def connect(self) -> None:
        r = self.robot
        fpath = r.calibration_fpath
        if not fpath.is_file():
            raise RuntimeError(f"No calibration at {fpath}. Run `farm calibrate --profile ...` once with the arms supported.")
        r.bus1.connect(); r.bus2.connect()
        r.bus1.calibration = {k: v for k, v in r.calibration.items() if k in r.bus1.motors}
        r.bus2.calibration = {k: v for k, v in r.calibration.items() if k in r.bus2.motors}
        r.bus1.write_calibration(r.bus1.calibration)
        r.bus2.write_calibration(r.bus2.calibration)
        r.configure()
        self.calibration_id = f"{fpath.name}:{int(fpath.stat().st_mtime)}"
        self._connected = True
        self.stop()
        log.info("robot connected; calibration %s", self.calibration_id)

    def disconnect(self) -> None:
        if self._connected:
            try:
                self.stop()
            finally:
                self.robot.disconnect()
                self._connected = False

    # ---- reads -------------------------------------------------------------
    def joints(self) -> Reading[dict[str, float]]:
        if not self._connected:
            return invalid(self.name, "not connected")
        try:
            r = self.robot
            pos = {}
            pos.update(r.bus1.sync_read("Present_Position", r.left_arm_motors + r.head_motors))
            pos.update(r.bus2.sync_read("Present_Position", r.right_arm_motors))
            return Reading(pos, Status.OK, source=self.name)
        except Exception as e:  # serial hiccup: never a fake number
            return invalid(self.name, f"read failed: {e}")

    def health(self) -> Reading[dict[str, dict[str, float]]]:
        if not self._connected:
            return invalid(self.name, "not connected")
        try:
            r = self.robot
            out: dict[str, dict[str, float]] = {}
            for bus, motors in ((r.bus1, r.left_arm_motors + r.head_motors), (r.bus2, r.right_arm_motors)):
                load = bus.sync_read("Present_Load", motors, normalize=False)
                for m in motors:
                    out[m] = {"load": float(load[m])}
            return Reading(out, Status.OK, source=self.name)
        except Exception as e:
            return invalid(self.name, f"health read failed: {e}")

    # ---- writes ------------------------------------------------------------
    def move_to(self, targets: dict[str, float], max_step: float | None = None) -> Reading[dict[str, float]]:
        if not self._connected:
            return invalid(self.name, "not connected")
        action = {f"{k}.pos": float(v) for k, v in targets.items()}
        try:
            if max_step is not None:
                old = self.robot.config.max_relative_target
                self.robot.config.max_relative_target = max_step
                try:
                    sent = self.robot.send_action(action)
                finally:
                    self.robot.config.max_relative_target = old
            else:
                sent = self.robot.send_action(action)
            return Reading({k[:-4]: v for k, v in sent.items() if k.endswith(".pos")}, Status.OK, source=self.name)
        except Exception as e:
            return invalid(self.name, f"send failed: {e}")

    def stop(self) -> None:
        """Hold position: goal := present for both arms and head; wheels := 0."""
        r = self.robot
        try:
            r.stop_base()
        except Exception as e:
            log.warning("stop_base failed: %s", e)
        for bus, motors in ((r.bus1, r.left_arm_motors + r.head_motors), (r.bus2, r.right_arm_motors)):
            try:
                present = bus.sync_read("Present_Position", motors)
                bus.sync_write("Goal_Position", present)
            except Exception as e:
                log.warning("hold failed on %s: %s", motors[0], e)

    def torque_off(self, motors: list[str] | None = None) -> None:
        r = self.robot
        for bus in (r.bus1, r.bus2):
            sel = [m for m in bus.motors if motors is None or m in motors]
            if sel:
                bus.disable_torque(sel)

    def torque_on(self, motors: list[str] | None = None) -> None:
        r = self.robot
        for bus in (r.bus1, r.bus2):
            sel = [m for m in bus.motors if motors is None or m in motors]
            if sel:
                bus.enable_torque(sel)

    # ---- one-time calibration (interactive by necessity: LeRobot's procedure) -----
    def calibrate_interactive(self) -> Path:
        """Runs LeRobot's range-of-motion calibration. This is setup, not operation."""
        r = self.robot
        r.bus1.connect(); r.bus2.connect()
        r.calibrate()
        r.bus1.disconnect(); r.bus2.disconnect()
        return r.calibration_fpath


def find_serial_ports() -> list[dict[str, Any]]:
    from serial.tools import list_ports
    return [{"device": p.device, "description": p.description, "vid": p.vid, "pid": p.pid, "serial": p.serial_number} for p in list_ports.comports()]
