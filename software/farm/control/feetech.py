"""Raw-tick executor INSIDE an existing, exclusive motor-owning session.

Does not connect serial ports, configure motors, change calibration or enable
torque. The session must supply its own shared I/O lock and STOP/deadman handling.
Never instantiate a second legacy LeRobot connection alongside the carton holder.
"""
from __future__ import annotations

import hashlib
import json
import time

from .jev import JOINTS, MOTORS, WHEELS, MotionRefused, action_catalog


class ConnectedFeetechActuator:
    def __init__(self, robot, owner_lock):
        self.robot, self.lock = robot, owner_lock
        self.buses = {m: b for b in (robot.bus1, robot.bus2) for m in b.motors}
        if set(self.buses) != set(MOTORS):
            raise MotionRefused("expected 14 position servos and two wheel servos")
        self.bounds = {}
        identity = {}
        with self.lock:
            for m in JOINTS:
                c = robot.calibration.get(m)
                if c is None or not 0 <= c.range_min < c.range_max <= 4095 or c.range_max - c.range_min < 16:
                    raise MotionRefused(f"missing, wrapped or invalid calibration: {m}")
                # Compare EEPROM to the saved file. This path NEVER rewrites it.
                for field, expected in (("Homing_Offset", c.homing_offset), ("Min_Position_Limit", c.range_min),
                                        ("Max_Position_Limit", c.range_max)):
                    if self._read(m, field) != expected:
                        raise MotionRefused(f"device and saved calibration differ: {m}")
                self.bounds[m] = [c.range_min + 4, c.range_max - 4]
                identity[m] = [c.id, c.homing_offset, c.range_min, c.range_max]
        self.calibration_id = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()

    def _read(self, motor, field):
        value = self.buses[motor].read(field, motor, normalize=False, num_retry=0)
        if type(value) is not int:
            raise MotionRefused(f"invalid raw register reply: {motor}/{field}")
        return value

    def _write(self, motor, field, value):
        self.buses[motor].write(field, motor, int(value), normalize=False, num_retry=0)
        if self._read(motor, field) != int(value):
            raise MotionRefused(f"motor write not acknowledged: {motor}/{field}")

    def snapshot(self):
        # Timestamp the START of the sample, so a slow serial sweep is not fresh.
        started = time.time()
        with self.lock:
            positions = {m: self._read(m, "Present_Position") for m in JOINTS}
            velocities = {m: self._read(m, "Present_Velocity") for m in WHEELS}
            health = {m: {key: self._read(m, register) for key, register in (
                ("temperature", "Present_Temperature"), ("load", "Present_Load"), ("status", "Status"),
                ("torque_enabled", "Torque_Enable"), ("mode", "Operating_Mode"))} for m in MOTORS}
        return {"observed_at": started, "positions": positions, "bounds": dict(self.bounds),
                "wheel_velocities": velocities, "health": health, "calibration_id": self.calibration_id}

    def _check(self, selected, limits):
        for m in selected:
            if self._read(m, "Torque_Enable") != 1 or self._read(m, "Operating_Mode") != (1 if m in WHEELS else 0):
                raise MotionRefused("selected motor disabled or in wrong mode")
            if self._read(m, "Status") != 0 or not -20 <= self._read(m, "Present_Temperature") <= limits.max_temperature or abs(self._read(m, "Present_Load")) > limits.max_load:
                raise MotionRefused("motor health changed during motion")
        for m in JOINTS:
            if not self.bounds[m][0] <= self._read(m, "Present_Position") <= self.bounds[m][1]:
                raise MotionRefused("joint left calibrated travel")

    def execute(self, action, before, limits, guard):
        # This is also checked here so direct callers cannot inject larger targets.
        if action != action_catalog(limits).get(action.name):
            raise MotionRefused("action is not in the bounded catalogue")
        selected = action.wheels or action.joints
        if before.get("calibration_id") != self.calibration_id:
            raise MotionRefused("calibration changed")
        old_speeds = {}
        try:
            with self.lock:
                guard()
                self._check(selected, limits)
                for m in JOINTS:
                    if abs(self._read(m, "Present_Position") - before["positions"][m]) > limits.drift_ticks:
                        raise MotionRefused("pose changed before write")
                for m, delta in action.joints.items():
                    target = round(before["positions"][m]) + delta
                    if not self.bounds[m][0] <= target <= self.bounds[m][1]:
                        raise MotionRefused("target outside calibrated travel")
                    old_speeds[m] = self._read(m, "Goal_Velocity")
                    self._write(m, "Goal_Velocity", limits.joint_speed)
                    guard()
                    self._write(m, "Goal_Position", target)
                # Start the pulse clock BEFORE the first wheel write.
                until = time.monotonic() + (limits.pulse_s if action.wheels else limits.move_timeout_s)
                for m, speed in action.wheels.items():
                    guard()
                    if time.monotonic() >= until:
                        raise MotionRefused("wheel command deadline expired")
                    self._write(m, "Goal_Velocity", speed)
            reached = not action.joints
            while time.monotonic() < until:
                guard()
                with self.lock:
                    self._check(selected, limits)
                    if action.joints and all(abs(self._read(m, "Present_Position") - (round(before["positions"][m]) + d)) <= 8 for m, d in action.joints.items()):
                        reached = True
                        break
                time.sleep(0.01)
            if not reached:
                raise MotionRefused("joint did not reach target; no automatic retry")
        finally:
            self.stop()
            # Position is held before restoring the owner's speed settings.
            with self.lock:
                for m, speed in old_speeds.items():
                    self._write(m, "Goal_Velocity", speed)
        return self.snapshot()

    def stop(self):
        errors = []
        with self.lock:
            # Attempt BOTH wheels and every active joint even if one bus fails.
            for m in WHEELS:
                try:
                    if self._read(m, "Operating_Mode") == 1:
                        self._write(m, "Goal_Velocity", 0)
                    elif self._read(m, "Torque_Enable") != 0:
                        raise MotionRefused("wheel enabled in an unexpected operating mode")
                except Exception as exc:
                    errors.append(type(exc).__name__)
            for m in JOINTS:
                try:
                    if self._read(m, "Torque_Enable") == 1:
                        self._write(m, "Goal_Position", self._read(m, "Present_Position"))
                except Exception as exc:
                    errors.append(type(exc).__name__)
        if errors:
            raise MotionRefused("STOP could not be confirmed on all motors; use the physical stop")
