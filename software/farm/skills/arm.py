"""Per-arm Cartesian model on top of XLeRobot's analytical IK.

The abstract simulator retains its original degree-like positions. An explicitly
registered AnalyticalReference converts every modeled positioning joint through
raw encoder ticks into actual normalized adapter units, and reverses that mapping
before FK. SkillRunner still refuses physical Cartesian execution until frame,
workspace and collision commissioning exists. x is forward and y is up, metres.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ..adapters.base import ARM_JOINTS, arm_joint
from ..vendor.so101_kinematics import SO101Kinematics
from ..kinematics.analytical_reference import AnalyticalReference

REST_XY = (0.1629, 0.1131)  # upstream "zero" reach point


@dataclass
class ArmPose:
    x: float = REST_XY[0]
    y: float = REST_XY[1]
    pitch: float = 0.0
    pan: float = 0.0
    roll: float = 0.0
    gripper: float = 0.0

    def copy(self) -> "ArmPose":
        return ArmPose(self.x, self.y, self.pitch, self.pan, self.roll, self.gripper)


@dataclass
class ArmModel:
    arm: str
    kin: SO101Kinematics = field(default_factory=SO101Kinematics)
    pose: ArmPose = field(default_factory=ArmPose)
    x_range: tuple[float, float] = (0.05, 0.25)
    y_range: tuple[float, float] = (-0.05, 0.25)
    reference: AnalyticalReference | None = None

    def __post_init__(self):
        if self.reference is not None and self.reference.arm!=self.arm:
            raise ValueError('Analytical reference belongs to another arm')

    def joints_for(self, pose: ArmPose) -> dict[str, float]:
        x = max(self.x_range[0], min(self.x_range[1], pose.x))
        y = max(self.y_range[0], min(self.y_range[1], pose.y))
        lift, elbow = self.kin.inverse_kinematics(x, y)
        wrist = -lift - elbow + pose.pitch
        result = {
            arm_joint(self.arm, "shoulder_pan"): float(pose.pan),
            arm_joint(self.arm, "shoulder_lift"): float(lift),
            arm_joint(self.arm, "elbow_flex"): float(elbow),
            arm_joint(self.arm, "wrist_flex"): float(wrist),
            arm_joint(self.arm, "wrist_roll"): float(pose.roll),
            arm_joint(self.arm, "gripper"): float(max(0.0, min(100.0, pose.gripper))),
        }
        if self.reference is not None:
            modeled={name:result[arm_joint(self.arm,name)] for name in self.reference.units}
            result.update(self.reference.normalized_from_degrees(modeled))
        return result

    def joints(self) -> dict[str, float]:
        return self.joints_for(self.pose)

    def apply_delta(self, dx_mm: float = 0, dy_mm: float = 0, dpitch: float = 0, dpan: float = 0, droll: float = 0, gripper: float | None = None,
                    max_mm: float = 15.0, max_deg: float = 6.0) -> ArmPose:
        c = lambda v, m: max(-m, min(m, float(v)))  # noqa: E731
        p = self.pose.copy()
        p.x += c(dx_mm, max_mm) / 1000.0
        p.y += c(dy_mm, max_mm) / 1000.0
        p.pitch += c(dpitch, max_deg)
        p.pan += c(dpan, max_deg)
        p.roll += c(droll, max_deg * 2)
        if gripper is not None:
            p.gripper = max(0.0, min(100.0, float(gripper)))
        p.x = max(self.x_range[0], min(self.x_range[1], p.x))
        p.y = max(self.y_range[0], min(self.y_range[1], p.y))
        self.pose = p
        return p

    def sync_from_joints(self, joints: dict[str, float]) -> None:
        """Update the Cartesian estimate from measured joints (FK on lift/elbow)."""
        if self.reference is not None:
            modeled=self.reference.degrees_from_normalized(joints)
            joints={**joints,**{arm_joint(self.arm,name):value for name,value in modeled.items()}}
        lift = joints.get(arm_joint(self.arm, "shoulder_lift"))
        elbow = joints.get(arm_joint(self.arm, "elbow_flex"))
        if lift is None or elbow is None:
            return
        x, y = self.kin.forward_kinematics(lift, elbow)
        self.pose.x, self.pose.y = float(x), float(y)
        self.pose.pan = float(joints.get(arm_joint(self.arm, "shoulder_pan"), self.pose.pan))
        self.pose.roll = float(joints.get(arm_joint(self.arm, "wrist_roll"), self.pose.roll))
        self.pose.gripper = float(joints.get(arm_joint(self.arm, "gripper"), self.pose.gripper))
        wrist = joints.get(arm_joint(self.arm, "wrist_flex"))
        if wrist is not None:
            self.pose.pitch = float(wrist + lift + elbow)


def arm_joint_names(arm: str) -> list[str]:
    return [arm_joint(arm, j) for j in ARM_JOINTS]
