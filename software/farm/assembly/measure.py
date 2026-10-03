"""Fit a station transform from measured correspondences; no hardware or guessed poses."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .frames import RobotFromAssembly


def fit_station(assembly_mm, robot_m, *, method: str, measured_on: str, by: str,
                calibration_id: str, max_error_mm: float = 2.0) -> tuple[RobotFromAssembly, dict]:
    """Proper rigid least-squares fit; reject degenerate geometry and large point errors.

    At least four paired points with two-dimensional spread are required. One should
    be an independent check point, not used for the fit (handled by fit_station_file).
    The residual is a consistency check, not proof of TCP calibration or collision clearance.
    """
    a, b = np.asarray(assembly_mm, dtype=float), np.asarray(robot_m, dtype=float)
    if a.ndim != 2 or a.shape[1:] != (3,) or len(a) < 4 or b.shape != a.shape:
        raise ValueError("need at least four paired Nx3 assembly_mm / robot_m points")
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("measurement points must be finite")
    if not all(isinstance(s, str) and s.strip() for s in (method, measured_on, by, calibration_id)):
        raise ValueError("method, measured_on, by and calibration_id are required")
    if not np.isfinite(max_error_mm) or not 0 < max_error_mm <= 2:
        raise ValueError("max_error_mm must be positive and at most 2 mm")
    a = a / 1000.0
    ac, bc = a - a.mean(axis=0), b - b.mean(axis=0)
    for centered in (ac, bc):
        singular = np.linalg.svd(centered, compute_uv=False)
        if singular[1] < 0.01 or singular[1] / singular[0] < 0.1:
            raise ValueError("points too close or nearly collinear; spread them over the fixture")
    u, _, vt = np.linalg.svd(ac.T @ bc)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    t = b.mean(axis=0) - r @ a.mean(axis=0)
    errors = np.linalg.norm((r @ a.T).T + t - b, axis=1) * 1000
    if errors.max() > max_error_mm:
        raise ValueError(f"point error {errors.max():.3f} mm exceeds {max_error_mm} mm; check units/correspondences/TCP")
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    rms = float(np.sqrt(np.mean(errors**2)))
    station = RobotFromAssembly(m, True, method, measured_on, by, rms, calibration_id,
                                "Least-squares point fit; reach, fixture stability and TCP require physical validation.")
    return station, {"rms_mm": rms, "max_error_mm": float(errors.max()), "point_errors_mm": errors.tolist()}


def fit_station_file(source: Path, destination: Path) -> dict:
    """Read actual measurements; require an independent check point; refuse overwrite."""
    data = yaml.safe_load(Path(source).read_text())
    if not isinstance(data, dict) or data.get("kind") != "r2a-station-measurements":
        raise ValueError("expected kind: r2a-station-measurements")
    station, report = fit_station(data["assembly_mm"], data["robot_m"],
        **{key: data[key] for key in ("method", "measured_on", "by", "calibration_id")})
    check_a = np.asarray(data["check_assembly_mm"], dtype=float)
    check_b = np.asarray(data["check_robot_m"], dtype=float)
    if check_a.shape != (3,) or check_b.shape != (3,) or not np.isfinite(check_b).all():
        raise ValueError("independent check point must be two finite 3-vectors")
    if np.min(np.linalg.norm(np.asarray(data["assembly_mm"]) - check_a, axis=1)) < 10:
        raise ValueError("independent check point must be at least 10 mm from every fitted point")
    error = float(np.linalg.norm(station.apply(check_a) - check_b) * 1000)
    if error > 2:
        raise ValueError(f"independent check error {error:.3f} mm exceeds 2 mm")
    report["check_error_mm"] = error
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x") as f:
        yaml.safe_dump({**station.to_dict(), "fit_report": report,
                        "measurement_source": str(Path(source).resolve())}, f, sort_keys=False)
    return report
