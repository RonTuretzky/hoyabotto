"""Real robot adapter: XLerobot2Wheels over two Feetech buses, no prompts, no wheels.

Differences from the upstream class's own connect():
- calibration is restored from the saved file without an input() prompt; if it is
  missing we refuse to connect (run `farm calibrate` once, with the arms supported).
- cameras are handled by CameraAdapters, not by the robot object.
- wheels are never commanded; arm/head hold targets are verified before torque.
"""
from __future__ import annotations

import math
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
        settings = self.cfg.position_settings
        allowed_motors = set(r.left_arm_motors + r.head_motors + r.right_arm_motors)
        bounds = {"P_Coefficient": (1, 32), "Torque_Limit": (1, 1000), "Goal_Velocity": (1, 1000)}
        for motor, overrides in settings.items():
            if motor not in allowed_motors:
                raise ValueError(f"Position settings must name an arm/head joint: {motor}")
            for field, value in overrides.items():
                if field not in bounds or type(value) is not int or not bounds[field][0] <= value <= bounds[field][1]:
                    raise ValueError(f"Invalid position setting for {motor}: {field}={value}")
        try:
            for bus, motors in self._groups():
                bus.connect()
                bus.calibration = {m: r.calibration[m] for m in motors}
                bus.disable_torque(motors)
                # Calibration must already match hardware; connection never rewrites it.
                for m in motors:
                    c = bus.calibration[m]
                    for field, value in (("Homing_Offset", c.homing_offset),
                                         ("Min_Position_Limit", c.range_min),
                                         ("Max_Position_Limit", c.range_max)):
                        if bus.read(field, m, normalize=False, num_retry=3) != value:
                            raise RuntimeError(f"Calibration mismatch: {m} {field}")
                    bus.write("Operating_Mode", m, OperatingMode.POSITION.value, num_retry=3)
                    for field, value in (("P_Coefficient", 16), ("I_Coefficient", 0), ("D_Coefficient", 43)):
                        bus.write(field, m, value, num_retry=3)
                    for field, value in settings.get(m, {}).items():
                        bus.write(field, m, value, normalize=False, num_retry=3)
                        if bus.read(field, m, normalize=False, num_retry=3) != value:
                            raise RuntimeError(f"Position setting readback mismatch: {m} {field}")
                self._prime_hold(bus, motors)
            # Both buses are fully prepared before any torque is enabled.
            for bus, motors in self._groups():
                self._prime_hold(bus, motors)
                bus.enable_torque(motors)
            self._connected = True
            self.calibration_id = f"{fpath.name}:{int(fpath.stat().st_mtime)}"
        except BaseException:
            self.disconnect()
            raise

    def _groups(self):
        r = self.robot
        return ((r.bus1, r.left_arm_motors + r.head_motors), (r.bus2, r.right_arm_motors))

    @staticmethod
    def _prime_hold(bus, motors):
        present = bus.sync_read("Present_Position", motors, normalize=False, num_retry=3)
        for m, pos in present.items():
            c = bus.calibration[m]
            if not c.range_min <= pos <= c.range_max:
                raise RuntimeError(f"{m} is outside its calibrated range; reposition before enabling torque")
        bus.sync_write("Goal_Position", present, normalize=False, num_retry=3)
        for m, pos in present.items():
            if bus.read("Goal_Position", m, normalize=False, num_retry=3) != pos:
                raise RuntimeError(f"Hold target readback mismatch: {m}")

    def disconnect(self) -> None:
        errors = []
        for bus, motors in self._groups():
            if bus.is_connected:
                try:
                    bus.disable_torque(motors, num_retry=3)
                except Exception as e:
                    errors.append(e)
                finally:
                    bus.disconnect(disable_torque=False)
        self._connected = False
        if errors:
            raise RuntimeError(f"Could not release all motor torque: {errors}")

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
        try:
            allowed = {m for _, motors in self._groups() for m in motors}
            if not set(targets) <= allowed:
                raise ValueError("Only arm and head joints may be commanded")
            step = min(self.cfg.max_relative_target, max_step if max_step is not None else self.cfg.max_relative_target)
            if not math.isfinite(step) or step <= 0:
                raise ValueError("Invalid movement step")
            sent = {}
            pending = []
            for bus, motors in self._groups():
                selected = [m for m in motors if m in targets]
                if not selected:
                    continue
                present = bus.sync_read("Present_Position", selected, num_retry=3)
                goals = {}
                for m in selected:
                    target = float(targets[m])
                    if not math.isfinite(target):
                        raise ValueError("Non-finite target")
                    lo = 0 if m.endswith("gripper") else -100
                    target = max(lo, min(100, target))
                    goals[m] = max(present[m] - step, min(present[m] + step, target))
                pending.append((bus, goals))
            for bus, goals in pending:
                bus.sync_write("Goal_Position", goals, num_retry=3)
                sent.update(goals)
            return Reading(sent, Status.OK, source=self.name)
        except Exception as e:
            return invalid(self.name, f"send failed: {e}")

    def stop(self) -> None:
        """Hold arms/head where they are; never command wheels."""
        for bus, motors in self._groups():
            if bus.is_connected:
                present = bus.sync_read("Present_Position", motors, normalize=False, num_retry=3)
                bus.sync_write("Goal_Position", present, normalize=False, num_retry=3)

    def torque_off(self, motors: list[str] | None = None) -> None:
        for bus, allowed in self._groups():
            selected = [m for m in allowed if motors is None or m in motors]
            if selected:
                bus.disable_torque(selected, num_retry=3)

    def torque_on(self, motors: list[str] | None = None) -> None:
        for bus, allowed in self._groups():
            selected = [m for m in allowed if motors is None or m in motors]
            if selected:
                self._prime_hold(bus, selected)
                bus.enable_torque(selected, num_retry=3)

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
