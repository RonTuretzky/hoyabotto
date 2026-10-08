"""AprilTag (36h11) detection: the optional, vision-based confirmation that the
tray in front of the camera is the tray the nest says it is.

A missing detector or an unreadable frame yields UNKNOWN, never a match.
"""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from ..status import Reading, Status, unknown

EXTRACTOR_VERSION = "apriltag-2"
_detector = None


def _get_detector():
    global _detector
    if _detector is None:
        from pupil_apriltags import Detector
        _detector = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=True)
    return _detector


def detect_tags(frame: Reading[np.ndarray]) -> Reading[dict[int, dict[str, Any]]]:
    """All 36h11 tags: {id: {center, corners, margin, hamming}}; UNKNOWN on ambiguous identity or bad frames."""
    if frame.status is not Status.OK or frame.value is None:
        return Reading(None, frame.status, frame.t, source="perception.tags", note=frame.note)
    try:
        det = _get_detector()
    except Exception as e:  # noqa: BLE001
        return unknown("perception.tags", f"detector unavailable: {e}")
    try:
        gray = cv2.cvtColor(frame.value, cv2.COLOR_RGB2GRAY)
        detections = det.detect(gray)
    except Exception as e:  # noqa: BLE001
        return unknown("perception.tags", f"detection failed: {e}")
    found, seen = {}, set()
    for d in detections:
        tag_id = int(d.tag_id)
        if tag_id in seen:
            return unknown("perception.tags", f"duplicate tag ID {tag_id}: object identity is ambiguous")
        seen.add(tag_id)
        if not np.isfinite(d.decision_margin) or not np.isfinite(d.corners).all() or not np.isfinite(d.center).all():
            return unknown("perception.tags", "nonfinite tag measurement")
        if d.decision_margin < 20:   # weak decode: do not trust
            continue
        found[tag_id] = {"center": [float(d.center[0]), float(d.center[1])], "corners": d.corners.astype(float).tolist(),
                         "margin": float(d.decision_margin), "hamming": int(d.hamming)}
    return Reading(found, Status.OK, frame.t, source="perception.tags", meta={"extractor": EXTRACTOR_VERSION})


def confirm_tray(frame: Reading[np.ndarray], expected_id: int | None) -> Reading[bool]:
    """True if the expected tag is visible; False if a *different* tray tag is visible instead;
    UNKNOWN if no tag can be read (tags are optional: an absent tag never blocks on its own)."""
    if expected_id is None:
        return Reading(None, Status.NOT_APPLICABLE, source="perception.tray_tag", note="no tag configured for this tray")
    tags = detect_tags(frame)
    if tags.status is not Status.OK:
        return Reading(None, tags.status, frame.t, source="perception.tray_tag", note=tags.note)
    if expected_id in tags.value:
        return Reading(True, Status.OK, frame.t, source="perception.tray_tag", meta={"seen": sorted(tags.value)})
    if tags.value:
        return Reading(False, Status.OK, frame.t, source="perception.tray_tag", note=f"saw tag(s) {sorted(tags.value)}, expected {expected_id}", meta={"seen": sorted(tags.value)})
    return unknown("perception.tray_tag", "no tag readable in frame")


def render_tag(grid: list[str], cell_px: int = 40, border_cells: int = 1) -> np.ndarray:
    """Render a tag from its 8x8 grid ('1' = black) into an RGB image with a white quiet zone (for tests and printing)."""
    n = len(grid)
    size = (n + 2 * border_cells) * cell_px
    img = np.full((size, size, 3), 255, np.uint8)
    for r, row in enumerate(grid):
        for c, ch in enumerate(row):
            if ch == "1":
                y0 = (r + border_cells) * cell_px; x0 = (c + border_cells) * cell_px
                img[y0:y0 + cell_px, x0:x0 + cell_px] = 0
    return img
