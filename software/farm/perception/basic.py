"""OpenCV perception: what can be judged without a model, and when to refuse.

Outputs are Readings. Anything this file cannot see is UNKNOWN, never CLEAR.
"""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from ..status import Reading, Status, unknown

EXTRACTOR_VERSION = "basic-1"


def frame_quality(frame: Reading[np.ndarray]) -> Reading[dict[str, Any]]:
    """Brightness, sharpness and dark-fraction. INVALID/STALE frames propagate."""
    if frame.status is not Status.OK or frame.value is None:
        return Reading(None, frame.status, frame.t, source="perception.quality", note=frame.note)
    img = frame.value
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    mean = float(gray.mean())
    lap = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    dark_frac = float((gray < 25).mean())
    q = {"mean": mean, "sharpness": lap, "dark_fraction": dark_frac, "h": int(img.shape[0]), "w": int(img.shape[1])}
    status = Status.OK
    note = ""
    if mean < 20:
        status, note = Status.UNKNOWN, "frame too dark"
    elif dark_frac > 0.45:
        status, note = Status.UNKNOWN, f"{dark_frac:.0%} of the frame is black: likely occluded"
    elif lap < 5:
        status, note = Status.UNKNOWN, "frame too blurry"
    return Reading(q, status, frame.t, source="perception.quality", note=note, meta={"extractor": EXTRACTOR_VERSION})


def green_fraction(frame: Reading[np.ndarray], roi: tuple[int, int, int, int] | None = None) -> Reading[float]:
    """Fraction of green-ish pixels (HSV). A trend for people, never a decision input."""
    if frame.status is not Status.OK or frame.value is None:
        return Reading(None, frame.status, frame.t, source="perception.green", note=frame.note)
    img = frame.value
    if roi:
        y0, y1, x0, x1 = roi
        img = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (35, 60, 40), (90, 255, 255))
    return Reading(float(mask.mean() / 255.0), Status.OK, frame.t, source="perception.green", meta={"extractor": EXTRACTOR_VERSION})


def frame_diff(a: Reading[np.ndarray], b: Reading[np.ndarray]) -> Reading[float]:
    """Mean absolute difference between two frames (spill/lighting change hint)."""
    if a.status is not Status.OK or b.status is not Status.OK:
        return unknown("perception.diff", "needs two OK frames")
    ga = cv2.cvtColor(a.value, cv2.COLOR_RGB2GRAY).astype(np.int16)
    gb = cv2.cvtColor(b.value, cv2.COLOR_RGB2GRAY).astype(np.int16)
    if ga.shape != gb.shape:
        return unknown("perception.diff", "frame sizes differ")
    return Reading(float(np.abs(ga - gb).mean()), Status.OK, b.t, source="perception.diff")
