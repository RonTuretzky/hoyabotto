"""Conversions shared by visual commissioning and LeRobot URDF kinematics.

Raw STS3215 ticks, LeRobot normalized positions and URDF degrees are separate
units. A recorded physical model zero/sign is required for the last mapping.
"""
from dataclasses import dataclass
import math


def finite(value, name="value"):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}: expected a finite number")
    return float(value)


@dataclass(frozen=True)
class JointUnits:
    range_min: int
    range_max: int
    # Must be measured when converting into a geometric model. Calibration
    # range endpoints alone do NOT establish a URDF/kinematic zero angle.
    model_zero_tick: float | None = None
    model_sign: int = 1
    gripper: bool = False

    def __post_init__(self):
        if (type(self.range_min) is not int or type(self.range_max) is not int
                or not 0 <= self.range_min < self.range_max <= 4095 or type(self.model_sign) is not int
                or self.model_sign not in (-1, 1)):
            raise ValueError("Invalid joint-unit calibration")
        if self.model_zero_tick is not None:
            if not 0 <= finite(self.model_zero_tick, "model zero") <= 4095:
                raise ValueError("Model zero must be a measured encoder tick in 0..4095")

    def normalized_to_ticks(self, value):
        lo, span = (0, 100) if self.gripper else (-100, 200)
        value = finite(value)
        if not lo <= value <= lo + span:
            raise ValueError("Normalized position outside calibration")
        return self.range_min + (value-lo)/span*(self.range_max-self.range_min)

    def ticks_to_normalized(self, ticks):
        ticks = finite(ticks)
        if not self.range_min <= ticks <= self.range_max:
            raise ValueError("Encoder outside calibration")
        lo, span = (0, 100) if self.gripper else (-100, 200)
        return lo + (ticks-self.range_min)/(self.range_max-self.range_min)*span

    def ticks_to_model_degrees(self, ticks):
        if self.model_zero_tick is None:
            raise ValueError("Measure the model zero/sign before using URDF kinematics")
        self.ticks_to_normalized(ticks)
        return self.model_sign*(ticks-self.model_zero_tick)*360/4096

    def model_degrees_to_ticks(self, degrees):
        if self.model_zero_tick is None:
            raise ValueError("Measure the model zero/sign before using URDF kinematics")
        ticks = self.model_zero_tick + self.model_sign*finite(degrees)*4096/360
        self.ticks_to_normalized(ticks)
        return ticks
