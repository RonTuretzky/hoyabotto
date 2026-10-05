"""Shared AprilTag perception and the existing reviewed-patch tracker."""
from __future__ import annotations

import cv2
import numpy as np

from farm.perception.tags import detect_tags
from farm.status import Reading, Status
from .common import Refused, finite, vector


class FeatureFailure(Refused):
    def __init__(self, name, spec, reason):
        self.feature, self.reason = name, reason
        self.recoverable = (spec.get("type", "patch") == "patch" and not spec.get("anchor") and reason in {
            "Patch appearance lost or multiple similar targets are ambiguous",
            "Tracker lost its patch", "Tracker backward verification failed",
            "Tracker lost too many consistent image features",
            "Tracked patch does not move as one rigid region",
        })
        super().__init__(f"{name}: {reason}")


def tags_from_bgr(image):
    # The farm's common camera/perception contract is RGB; OpenCV JPEGs are BGR.
    result = detect_tags(Reading(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), Status.OK))
    if result.status is not Status.OK:
        raise Refused(f"AprilTag perception is not trustworthy: {result.note}")
    return result.value


class TagTracker:
    def __init__(self, reference, spec, detections):
        self.spec, self.shape = spec, reference.shape
        self.tag_id = spec["tag_id"]
        if type(self.tag_id) is not int or not 0 <= self.tag_id < 587:
            raise Refused("Expected one tag36h11 ID from 0 to 586")
        self.radius = finite(spec.get("max_displacement_px", 100))
        if not 1 <= self.radius <= 100:
            raise Refused("Tag tracking is limited to 100 pixels around the reviewed seed")
        self.reference_corners, centre = self._measurement(detections)
        self.point = vector(spec.get("point", centre), 2)
        if cv2.pointPolygonTest(self.reference_corners.astype(np.float32), tuple(self.point), False) < 0:
            raise Refused("Tracked tag point must be inside its reference quadrilateral")

    def _measurement(self, detections):
        found = detections.get(self.tag_id)
        if found is None or found["margin"] < 30 or found["hamming"] != 0:
            raise Refused(f"tag36h11 ID {self.tag_id}: missing, weak or corrected decode")
        corners = np.asarray(found["corners"], float)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise Refused("Invalid tag quadrilateral")
        if min(np.linalg.norm(corners-np.roll(corners, 1, axis=0), axis=1)) < 8:
            raise Refused("Tag is too small for precise alignment")
        return corners, vector(found["center"], 2)

    def locate(self, image, detections):
        if image.shape != self.shape:
            raise Refused("Camera resolution changed")
        corners, _ = self._measurement(detections)
        if np.max(np.linalg.norm(corners-self.reference_corners, axis=1)) > self.radius:
            raise Refused("Tag left its reviewed local image envelope")
        if self.spec.get("anchor") and np.max(np.linalg.norm(corners-self.reference_corners, axis=1)) > 2:
            raise Refused("Fixed head-camera/background anchor moved")
        transform = cv2.getPerspectiveTransform(self.reference_corners.astype(np.float32), corners.astype(np.float32))
        point = cv2.perspectiveTransform(self.point.reshape(1, 1, 2), transform)[0, 0]
        if not np.isfinite(point).all() or not (0 <= point[0] < image.shape[1] and 0 <= point[1] < image.shape[0]):
            raise Refused("Invalid projected tag point")
        return point


class FeatureTracks:
    def __init__(self, reference, regions):
        from .vision import RegionTracker  # compatibility with existing patch configs
        kinds = {r.get("type", "patch") for r in regions.values()}
        if not kinds <= {"patch", "apriltag"}:
            raise Refused("Region type must be patch or apriltag; no implicit fallback")
        self.has_tags = "apriltag" in kinds
        detections = tags_from_bgr(reference) if self.has_tags else None
        ids = [r["tag_id"] for r in regions.values() if r.get("type") == "apriltag"]
        if len(ids) != len(set(ids)):
            raise Refused("Each named object/anchor needs a distinct tag ID in a camera")
        self.trackers = {name: TagTracker(reference, spec, detections) if spec.get("type") == "apriltag"
                         else RegionTracker(reference, spec) for name, spec in regions.items()}

    def locate(self, image, anchors_only=False):
        detections = tags_from_bgr(image) if self.has_tags else None
        points = {}
        for name, tracker in self.trackers.items():
            if anchors_only and not tracker.spec.get("anchor"):
                continue
            try:
                points[name] = (tracker.locate(image, detections) if isinstance(tracker, TagTracker)
                                else tracker.locate(image))
            except Refused as exc:
                raise FeatureFailure(name, tracker.spec, str(exc)) from exc
        return points
