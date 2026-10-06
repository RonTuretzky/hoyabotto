"""AprilTag observations for the existing Gemma robot-tool interface.

The same adapter works next to a robot camera provider or on the Gemma Mac.
It consumes robot_get_cameras responses; it never opens cameras or motors.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
import math
import threading
import time

import cv2
import numpy as np

from farm.perception.tags import detect_tags
from farm.status import Reading, Status

TOOL_NAME = "robot_get_tags"
DEFAULT_ROLES = {1: "table_anchor", 2: "gripper", 3: "paddle"}
_DETECT_LOCK = threading.Lock()
MAX_IMAGE_BYTES = 4 * 1024 * 1024


def tool_schema(cameras):
    return {"type": "function", "function": {
        "name": TOOL_NAME,
        "description": (
            "Read AprilTag36h11 measurements from existing robot camera snapshots. "
            "Returns pixel centers/corners, quality, missing IDs, frame identity/timing, "
            "gripper-to-paddle pixel displacement and annotated images. The marker-kit "
            "mapping is 1=table anchor, 2=gripper, 3=paddle; verify physical mounting. "
            "Phone receipt time is not capture time. Missing/stale tags are unknown. "
            "Pixels are not millimetres, joint directions, jaw contact or grasp proof. "
            "Use fresh observations and measured motor results between movements. "
            "This tool reads only; it neither authorizes nor performs motor commands."
        ),
        "parameters": {"type": "object", "properties": {
            "cameras": {"type": "array", "items": {"type": "string", "enum": list(cameras)},
                        "minItems": 1, "maxItems": len(cameras), "uniqueItems": True},
            "tag_ids": {"type": "array", "items": {"type": "integer", "minimum": 0, "maximum": 586},
                        "minItems": 1, "maxItems": 32, "uniqueItems": True,
                        "description": "Expected IDs; defaults to the printed kit's 1, 2, 3."},
            "include_images": {"type": "boolean", "default": True},
        }, "additionalProperties": False},
    }}


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Missing or invalid frame timestamp")
    return float(value)


def _view_name(image, metadata, requested):
    identity = image.get("camera_id")
    for name, meta in metadata.items():
        if isinstance(meta, dict) and meta.get("camera_id") == identity and identity:
            return name
    if isinstance(identity, str):
        if identity.startswith("oak-") and "oak" in requested:
            return "oak"
        if identity == "phone_overview" and "phone" in requested:
            return "phone"
        if identity in requested:
            return identity
    raise ValueError("Camera image has no matching requested identity")


def _decode(image, meta):
    encoded = image.get("data_base64", image.get("base64"))
    if image.get("mime_type", image.get("mime")) not in ("image/jpeg", "image/png"):
        raise ValueError("Expected an RGB JPEG or PNG")
    if not isinstance(encoded, str) or len(encoded) > MAX_IMAGE_BYTES * 4 // 3 + 4:
        raise ValueError("Missing or oversized camera pixels")
    raw = base64.b64decode(encoded, validate=True)
    if not raw or len(raw) > MAX_IMAGE_BYTES:
        raise ValueError("Missing or oversized camera pixels")
    digest = hashlib.sha256(raw).hexdigest()
    if image.get("sha256") != digest:
        raise ValueError("Camera image hash mismatch or missing hash")
    for key in ("sha256", "seq", "stream_id", "camera_id"):
        if key in meta and key in image and meta[key] != image[key]:
            raise ValueError(f"Camera metadata mismatch: {key}")
    bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or bgr.shape[0] * bgr.shape[1] > 8_000_000:
        raise ValueError("Invalid or oversized decoded camera image")
    for key, actual in (("width", bgr.shape[1]), ("height", bgr.shape[0])):
        if key in meta and meta[key] != actual:
            raise ValueError(f"Camera metadata mismatch: {key}")
    if type(image.get("seq")) is not int or image["seq"] < 0:
        raise ValueError("Missing or invalid frame sequence")
    return bgr, digest


class TagObserver:
    """Per-view fresh measurements; no seeded displacement or fixed-head assumption."""

    def __init__(self, *, clock=time.time, max_age_s=2.0, detector=detect_tags):
        self.clock = clock
        self.max_age_s = max_age_s
        self.detector = detector
        self._previous = {}

    def _measure(self, image, meta, ids):
        bgr, digest = _decode(image, meta)
        # OAK's manifest captured_at can refer to depth; prefer the RGB timestamp.
        captured = meta.get("rgb_captured_at", image.get("captured_at"))
        received = image.get("received_at")
        receipt_only = captured is None
        stamp = _number(received if receipt_only else captured)
        if meta.get("live") is False or image.get("fresh") is False:
            raise ValueError("Camera provider reports an unavailable/stale frame")
        self._check_age(stamp)
        camera_id, seq, stream = image["camera_id"], image["seq"], image.get("stream_id")
        previous = self._previous.get(camera_id)
        if previous and previous[0] == stream:
            if seq < previous[1] or stamp < previous[2]:
                raise ValueError("Frame sequence or timestamp went backwards")
            if seq == previous[1] and (digest != previous[3] or stamp != previous[2]):
                raise ValueError("Frame contents or timestamp changed without a new sequence")
        with _DETECT_LOCK:
            detection = self.detector(Reading(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), Status.OK, t=stamp))
        if detection.status is not Status.OK:
            raise ValueError(detection.note or "AprilTag detection unavailable")
        tags, accepted = [], {}
        for tag_id, found in sorted(detection.value.items()):
            corners = np.asarray(found["corners"], dtype=float)
            edge = float(np.min(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
            quality_ok = found["hamming"] == 0 and found["margin"] >= 30 and edge >= 24
            row = {"tag_id": tag_id, "role": DEFAULT_ROLES.get(tag_id, "unassigned"),
                   "center_px": found["center"], "corners_px": found["corners"],
                   "decision_margin": found["margin"], "hamming": found["hamming"],
                   "shortest_edge_px": edge, "status": "DETECTED" if quality_ok else "REJECTED"}
            if not quality_ok:
                row["reason"] = "Need hamming=0, decision margin>=30 and shortest edge>=24px"
            else:
                accepted[tag_id] = row
            tags.append(row)
        age = self._check_age(stamp)  # Include detector latency in the age decision.
        frame = {"camera_id": camera_id, "seq": seq, "stream_id": stream, "sha256": digest,
                 "captured_at": captured, "received_at": received,
                 "timestamp_basis": "receipt_only_capture_delay_unknown" if receipt_only else "capture",
                 "age_s_on_observer_clock": age, "clock_basis": "cross_host_wall_clocks_must_be_synchronized",
                 "capture_age_within_limit": not receipt_only, "clock_synchronization_verified": False,
                 "new_since_last_call": previous is None or previous[:2] != (stream, seq)}
        self._previous[camera_id] = (stream, seq, stamp, digest)
        relative = None
        if 2 in accepted and 3 in accepted:
            relative = {"from_tag": 2, "to_tag": 3, "units": "pixels",
                        "dx": accepted[3]["center_px"][0] - accepted[2]["center_px"][0],
                        "dy": accepted[3]["center_px"][1] - accepted[2]["center_px"][1],
                        "axes": "+x=image right, +y=image down", "contact_offset_calibrated": False}
        return {"status": "OBSERVED" if accepted else "NO_VALID_TAGS", "frame": frame,
                "image_size_px": [bgr.shape[1], bgr.shape[0]], "tags": tags,
                "expected_ids": ids, "missing_ids": [i for i in ids if i not in accepted],
                "gripper_to_paddle_px": relative, "pose_3d": None}, bgr

    def _check_age(self, stamp):
        age = self.clock() - stamp
        if not -0.25 <= age <= self.max_age_s:
            raise ValueError("Stale/future frame or unsynchronized host clocks")
        return max(0.0, age)

    def observe(self, payload, cameras, ids=(1, 2, 3), include_images=True):
        """Consume the existing camera-tool envelope; images stay outside text results."""
        if not isinstance(payload, dict) or payload.get("ok") is not True:
            return {"ok": False, "error": "Robot camera tool failed; no tag observations", "motor_writes": 0}
        result = payload.get("result", {})
        if not isinstance(result, dict) or not isinstance(result.get("cameras", {}), dict) or not isinstance(payload.get("images", []), list):
            return {"ok": False, "error": "Malformed robot camera response", "motor_writes": 0}
        metadata = result.get("cameras", {})
        observations, images, seen = {}, [], set()
        if len(payload.get("images", [])) > 8:
            return {"ok": False, "error": "Too many camera images in one response", "motor_writes": 0}
        for image in payload.get("images", []):
            name = "unidentified"
            try:
                if not isinstance(image, dict):
                    raise ValueError("Malformed camera image entry")
                name = _view_name(image, metadata, cameras)
                if name not in cameras:
                    continue
                if name in seen:
                    raise ValueError("Duplicate camera view in one response")
                seen.add(name)
                row, bgr = self._measure(image, metadata.get(name, {}), list(ids))
                observations[name] = row
                if include_images:
                    annotated = self._annotate(image, row, bgr)
                    annotated["view"] = name
                    images.append(annotated)
            except (ValueError, TypeError, KeyError, cv2.error) as exc:
                observations[name] = {"status": "UNKNOWN", "reason": str(exc), "tags": [], "pose_3d": None}
                images = [i for i in images if i.get("view") != name]
        for name in cameras:
            observations.setdefault(name, {"status": "UNKNOWN", "reason": "Requested camera image missing",
                                          "tags": [], "pose_3d": None})
        # An earlier view can expire while a later view is being decoded.
        for name, row in list(observations.items()):
            if row["status"] == "UNKNOWN":
                continue
            frame = row["frame"]
            try:
                stamp = frame["captured_at"] if frame["captured_at"] is not None else frame["received_at"]
                frame["age_s_on_observer_clock"] = self._check_age(stamp)
            except ValueError as exc:
                observations[name] = {"status": "UNKNOWN", "reason": str(exc), "tags": [], "pose_3d": None}
                images = [i for i in images if i["view"] != name]
        available = any(row["status"] != "UNKNOWN" for row in observations.values())
        compact = {"schema": 1, "family": "tag36h11", "observations": observations,
                   "role_mapping": {str(k): v for k, v in DEFAULT_ROLES.items()},
                   "role_mapping_source": "printed_carton_kit_verify_physical_mounting",
                   "coordinate_system": "per_image_pixels", "depth_used": False,
                   "metric_pose_available": False, "physical_task_completed": False}
        compact["observation_id"] = hashlib.sha256(json.dumps(compact, sort_keys=True).encode()).hexdigest()[:24]
        out = {"ok": available, "result": compact, "images": images, "motor_writes": 0}
        if not available:
            out["error"] = "No usable camera observations; inspect per-camera reasons"
        return out

    @staticmethod
    def _annotate(source, row, bgr):
        overlay = bgr.copy()
        for tag in row["tags"]:
            corners = np.round(tag["corners_px"]).astype(np.int32)
            cv2.polylines(overlay, [corners], True, (0, 0, 0), 4)
            cv2.polylines(overlay, [corners], True, (255, 255, 255), 2)
            x, y = corners.min(axis=0)
            label = f'{tag["tag_id"]} {tag["role"]} {tag["status"]}'
            pos = (max(2, min(int(x), overlay.shape[1] - 200)), max(18, int(y) - 7))
            for color, weight in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                cv2.putText(overlay, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .45, color, weight)
        ok, buffer = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            raise ValueError("Could not encode annotated image")
        return {"camera_id": row["frame"]["camera_id"], "view": source.get("view"),
                "captured_at": row["frame"]["captured_at"], "received_at": row["frame"]["received_at"],
                "timestamp_basis": row["frame"]["timestamp_basis"], "seq": row["frame"]["seq"],
                "source_sha256": row["frame"]["sha256"], "mime_type": "image/jpeg",
                "data_base64": base64.b64encode(buffer).decode()}


class TagRobot:
    """Decorate Gemma's existing Robot client; motor calls pass through unchanged.

    If the server already provides robot_get_tags, prefer its native implementation.
    """

    def __init__(self, robot, *, observer=None):
        self.robot = robot
        self.observer = observer or TagObserver()
        self.last_catalog = None
        self._cameras = None

    def get(self, path):
        return self.robot.get(path)

    def catalog(self):
        catalog = copy.deepcopy(self.robot.catalog())
        functions = {t["function"]["name"]: t["function"] for t in catalog["tools"]}
        self._cameras = None
        if TOOL_NAME not in functions and "robot_get_cameras" in functions:
            camera_schema = functions["robot_get_cameras"].get("parameters", {}).get("properties", {}).get("cameras", {})
            names = camera_schema.get("items", {}).get("enum", [])
            if names and all(isinstance(n, str) for n in names):
                self._cameras = names
                catalog["tools"].append(tool_schema(names))
                catalog.setdefault("metadata", {})["apriltags"] = {
                    "execution": "local_detector_on_authenticated_robot_camera_snapshots",
                    "tool": TOOL_NAME, "motor_access": False}
        self.last_catalog = catalog
        return catalog

    def call(self, name, args, request_id=None):
        if name != TOOL_NAME:
            return self._forward(name, args, request_id)
        if self.last_catalog is None:
            self.catalog()
        if self._cameras is None:
            return self._forward(name, args, request_id)
        if not isinstance(args, dict) or set(args) - {"cameras", "tag_ids", "include_images"}:
            raise ValueError("Unknown AprilTag tool arguments")
        cameras = args.get("cameras", list(self._cameras))
        ids = args.get("tag_ids", [1, 2, 3])
        if not isinstance(cameras, list) or not cameras or any(not isinstance(c, str) or c not in self._cameras for c in cameras) or len(cameras) != len(set(cameras)):
            raise ValueError("Choose distinct available camera names")
        if not isinstance(ids, list) or not 1 <= len(ids) <= 32 or any(type(i) is not int or not 0 <= i <= 586 for i in ids) or len(ids) != len(set(ids)):
            raise ValueError("Choose distinct tag36h11 IDs from 0 to 586")
        include = args.get("include_images", True)
        if type(include) is not bool:
            raise ValueError("include_images must be boolean")
        payload = self._forward("robot_get_cameras", {"cameras": cameras}, request_id)
        return self.observer.observe(payload, cameras, ids, include)

    def _forward(self, name, args, request_id):
        if request_id is None:
            return self.robot.call(name, args)
        return self.robot.call(name, args, request_id=request_id)
