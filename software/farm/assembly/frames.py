"""Frames and units for the R2a assembly.

Three kinds of numbers appear in this project and they are never the same thing:

  * CAD / STL millimetres   -- the exported meshes (print orientation, Z = 0 on the bed)
  * assembly frame A, mm    -- original source XY, original holder deck top at Z = 0
  * robot frame, metres     -- whatever the arm is calibrated in, after a MEASURED rigid transform

Render offsets (the +23.5 mm world shift, exploded-view spacing) and slicer plate XY are
explanatory only and never enter this module. Joint values of the runtime are a fourth
quantity (LeRobot-normalized -100..100) and are not positions at all.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

MM_PER_M = 1000.0

# Translation from each exported mesh's coordinates into assembly frame A (mm). Identity rotation.
# The exporter raised the carrier by +20.5 mm so its feet sit at print Z = 0; this undoes that.
EXPORT_TO_ASSEMBLY_MM: dict[str, tuple[float, float, float]] = {
    "trough": (0.0, 0.0, 0.0),
    "carrier": (0.0, 0.0, -20.5),
    "retainer": (0.0, 0.0, 1.0),
}
RENDER_WORLD_SHIFT_MM = (0.0, 0.0, 23.5)   # renderer only; recorded so nobody applies it by mistake

# Nominal mesh extents in their own export frames (mm), from the release STLs.
EXPORT_BOUNDS_MM: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {
    "trough": ((-102.5, -42.5, -23.5), (77.5, 42.5, 1.5)),
    "carrier": ((-101.48, -41.5, 0.0), (76.5, 41.5, 43.5)),
    "retainer": ((-74.0, -33.0, 0.0), (74.0, 33.0, 18.0)),
    "grip_coupon": ((-18.0, -12.0, 0.0), (18.0, 12.0, 25.0)),
}

# Grasp candidates. Nominal geometry from the CAD; NOT calibrated robot poses.
CARRIER_FIN_CENTRES_A_MM = ((0.0, -36.5, 12.0), (0.0, 36.5, 12.0))
CARRIER_FIN_SECTION_MM = (24.0, 6.0)          # length along X, thickness along Y; jaws close across Y
CARRIER_FIN_X_RANGE_MM = (-12.0, 12.0)
CARRIER_FIN_Z_RANGE_A_MM = (-1.5, 23.0)
RETAINER_FIN_CENTRES_EXPORT_MM = ((-72.0, 0.0, 10.0), (72.0, 0.0, 10.0))
RETAINER_FIN_SECTION_MM = (4.0, 20.0)         # thickness along X, length along Y; jaws close across X
RETAINER_RING_HALF_Y_MM = 33.0
CARRIER_FIN_INNER_FACE_Y_MM = 36.5 - 3.0      # 33.5


class TransformNotMeasured(RuntimeError):
    """Raised when a robot-frame quantity is requested but T_robot_from_A was never measured."""


def _vec(p) -> np.ndarray:
    a = np.asarray(p, dtype=float).reshape(-1)
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        raise ValueError(f"expected a finite 3-vector, got {p!r}")
    return a


def stl_to_assembly_mm(part: str, p_stl_mm) -> np.ndarray:
    """p_A = p_STL + EXPORT_TO_ASSEMBLY_MM[part]."""
    if part not in EXPORT_TO_ASSEMBLY_MM:
        raise KeyError(f"no export translation for part {part!r}; known: {sorted(EXPORT_TO_ASSEMBLY_MM)}")
    return _vec(p_stl_mm) + np.asarray(EXPORT_TO_ASSEMBLY_MM[part])


def assembly_to_stl_mm(part: str, p_A_mm) -> np.ndarray:
    if part not in EXPORT_TO_ASSEMBLY_MM:
        raise KeyError(part)
    return _vec(p_A_mm) - np.asarray(EXPORT_TO_ASSEMBLY_MM[part])


@dataclass
class GraspCandidate:
    part: str
    point_A_mm: tuple[float, float, float]
    closure_axis: str            # "X" or "Y": the axis the jaws close across
    thickness_mm: float
    length_mm: float
    status: str = "nominal_cad"  # becomes "tested" only after a real grip on the printed part

    def as_dict(self) -> dict[str, Any]:
        return {"part": self.part, "point_A_mm": list(self.point_A_mm), "closure_axis": self.closure_axis,
                "thickness_mm": self.thickness_mm, "length_mm": self.length_mm, "status": self.status}


def grasp_candidates() -> list[GraspCandidate]:
    out = [GraspCandidate("carrier", tuple(float(v) for v in c), "Y", CARRIER_FIN_SECTION_MM[1], CARRIER_FIN_SECTION_MM[0]) for c in CARRIER_FIN_CENTRES_A_MM]
    for c in RETAINER_FIN_CENTRES_EXPORT_MM:
        pa = stl_to_assembly_mm("retainer", c)
        out.append(GraspCandidate("retainer", tuple(float(v) for v in pa), "X", RETAINER_FIN_SECTION_MM[0], RETAINER_FIN_SECTION_MM[1]))
    return out


def nominal_frame_to_fin_gap_mm() -> float:
    """Lateral gap between the retainer ring's Y edge and the carrier fin's inner face. 0.5 mm nominal."""
    return CARRIER_FIN_INNER_FACE_Y_MM - RETAINER_RING_HALF_Y_MM


@dataclass
class RobotFromAssembly:
    """T_robot_from_A: rigid transform taking assembly-frame points (metres) to robot-frame points (metres).

    `measured` is False until a person records how it was measured. An unmeasured transform
    refuses to produce robot coordinates; it exists only so the file format is fixed in advance.
    """
    matrix: np.ndarray = field(default_factory=lambda: np.eye(4))
    measured: bool = False
    method: str = ""
    measured_on: str = ""          # ISO date
    by: str = ""
    residual_mm: float | None = None
    calibration_id: str = ""      # which robot calibration this transform belongs to
    notes: str = ""

    def __post_init__(self) -> None:
        m = np.asarray(self.matrix, dtype=float)
        if m.shape != (4, 4) or not np.all(np.isfinite(m)):
            raise ValueError("matrix must be a finite 4x4")
        R = m[:3, :3]
        if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
            raise ValueError("rotation block is not a proper rotation")
        if not np.allclose(m[3], [0, 0, 0, 1]):
            raise ValueError("last row must be [0 0 0 1]")
        self.matrix = m

    def apply(self, p_A_mm) -> np.ndarray:
        """p_robot_m = T * homogeneous(0.001 * p_A_mm). Raises unless measured."""
        if not self.measured:
            raise TransformNotMeasured("T_robot_from_A has not been measured; a render or a guess is not a transform")
        p = _vec(p_A_mm) / MM_PER_M
        return (self.matrix @ np.append(p, 1.0))[:3]

    # ---- persistence ------------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {"matrix": self.matrix.tolist(), "measured": bool(self.measured), "method": self.method, "measured_on": self.measured_on,
                "by": self.by, "residual_mm": self.residual_mm, "calibration_id": self.calibration_id, "notes": self.notes, "units": "metres"}

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "RobotFromAssembly":
        d = dict(d or {})
        d.pop("units", None)
        d["matrix"] = np.asarray(d.get("matrix", np.eye(4)), dtype=float)
        if d.get("measured") and not (d.get("method") and d.get("measured_on") and d.get("by")):
            raise ValueError("a measured transform must say how (method), when (measured_on) and by whom (by)")
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump({**self.to_dict(), "written": time.strftime("%Y-%m-%dT%H:%M:%S")}, sort_keys=False))

    @classmethod
    def load(cls, path: Path) -> "RobotFromAssembly":
        path = Path(path)
        if not path.exists():
            return cls()   # unmeasured
        return cls.from_dict(yaml.safe_load(path.read_text()) or {})
