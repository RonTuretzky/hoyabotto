"""Digital-twin views for the chat's robot tools: a MODEL of the robot posed from live encoder readings.

TwinRobot decorates the chat's robot client the same way TagRobot and CalibrationRobot do. It adds one
read-only tool, robot_get_twin_view, and passes every other call to the inner robot unchanged. It reads
the owner's status (robot_get_state with fresh=False, no serial access) and, on request, the phone
camera; it never calls a motion or enable tool. Rendering is done by farm.sim.xlerobot_twin, imported
lazily so the chat starts even when the renderer or its dependencies are missing.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import importlib
import json
import math
from pathlib import Path
import threading
import time
import urllib.error

import cv2
import numpy as np

TOOL_NAME = "robot_get_twin_view"
VIEWS = ("front", "left", "right", "top")  # left/right: side views from the robot's left and right
RENDERER_MODULE = "farm.sim.xlerobot_twin"
JOINT_MAP_NAME = "twin-joint-map.json"
CANDIDATE_MAPPING = "feetech_degrees_v1"
MAX_STATE_AGE_S = 2.0
MAX_FUTURE_SKEW_S = 0.25  # same allowance as the AprilTag observer
LOCK_TIMEOUT_S = 20.0
LABEL_HEIGHT = 46
# The chat's own transport errors: it retries read-only tools on these while the robot server restarts.
_NETWORK_ERRORS = (urllib.error.URLError, ConnectionError, TimeoutError)

DESCRIPTION = (
    "Third-person views of a MODEL of the robot posed from the live servo encoder readings (not a camera). "
    "Use it to see how the arms are placed. The tick-to-angle mapping is the unvalidated candidate "
    "feetech_degrees_v1 unless mapping_validated is true, so a joint can appear mirrored or offset; trust the "
    "real cameras for contact and clearance. Read-only: reads the owner's last encoder status (no serial access) "
    "and, with compare_with_phone, one phone frame; it sends nothing to the motors. Refuses when the encoder "
    "reading is older than 2 s. compare_with_phone returns one side-by-side image (phone left, twin front right, "
    "each labelled with its timestamp) for checking the model against reality."
)


def tool_schema():
    return {"type": "function", "function": {
        "name": TOOL_NAME,
        "description": DESCRIPTION,
        "parameters": {"type": "object", "properties": {
            "views": {"type": "array", "items": {"type": "string", "enum": list(VIEWS)},
                      "minItems": 1, "maxItems": len(VIEWS), "uniqueItems": True, "default": list(VIEWS),
                      "description": "Model camera angles to render; defaults to all three."},
            "compare_with_phone": {"type": "boolean", "default": False,
                                   "description": "Also fetch the phone camera frame and return it side by side "
                                                  "with the twin's front view (phone left, twin right)."},
        }, "additionalProperties": False},
    }}


class Refusal(Exception):
    """A reason to answer ok False; never raised into the chat."""


def _fail(message, **extra):
    return {"ok": False, "result": {"error": message, "motor_writes": 0, **extra}}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _unwrap(payload, tool):
    """{'ok': True, 'result': {...}} -> the innermost result that carries the data (the chat nests up to 3 deep)."""
    if not isinstance(payload, dict) or payload.get("ok") is not True or not isinstance(payload.get("result"), dict):
        detail = payload.get("result", payload) if isinstance(payload, dict) else payload
        if isinstance(detail, dict):
            detail = detail.get("error", detail.get("reason", detail.get("blocker", detail)))
        raise Refusal(f"{tool} refused: {str(detail)[:400]}")
    result = payload["result"]
    for _ in range(3):
        if any(k in result for k in ("motors", "live_rows", "cameras")) or not isinstance(result.get("result"), dict):
            break
        result = result["result"]
    return result


def _clock_label(stamp):
    if not _finite(stamp):
        return "unknown"
    centis = round(stamp * 100)
    return time.strftime("%H:%M:%S", time.localtime(centis // 100)) + f".{centis % 100:02d}"


def load_joint_map(path):
    """Optional owner-measured overrides; an absent file means none. A malformed file is refused, not ignored."""
    if path is None:
        return None, None
    path = Path(path)
    if not path.is_file():
        return None, None
    raw = path.read_bytes()
    try:
        data = json.loads(raw)
    except ValueError as exc:
        raise Refusal(f"Twin joint map {path} is not valid JSON: {exc}") from None
    if not isinstance(data, dict) or not isinstance(data.get("joints"), dict):
        raise Refusal(f"Twin joint map {path} needs a 'joints' object")
    if type(data.get("validated", False)) is not bool:
        raise Refusal(f"Twin joint map {path}: 'validated' must be true or false")
    for motor, joint in data["joints"].items():
        if (not isinstance(joint, dict) or type(joint.get("zero_tick")) is not int
                or type(joint.get("sign")) is not int or joint["sign"] not in (1, -1)):
            raise Refusal(f"Twin joint map {path}: joint {motor!r} needs integer zero_tick and sign 1 or -1")
    return data, hashlib.sha256(raw).hexdigest()


def _config_path(robot):
    """The chat's robot config path, found through any wrappers (TagRobot keeps it on its inner .robot)."""
    seen = 0
    while robot is not None and seen < 8:
        config = vars(robot).get("config") if hasattr(robot, "__dict__") else None
        if config:
            return Path(config)
        robot, seen = vars(robot).get("robot") if hasattr(robot, "__dict__") else None, seen + 1
    return None


def _render_function():
    try:
        module = importlib.import_module(RENDERER_MODULE)
    except Exception as exc:  # ImportError, or a dependency (mujoco, model files) failing at import time
        raise Refusal(f"Twin renderer {RENDERER_MODULE} is unavailable ({type(exc).__name__}: {exc}). "
                      "No twin view; use the real cameras.") from None
    render = getattr(module, "render_twin", None)
    if not callable(render):
        raise Refusal(f"Twin renderer {RENDERER_MODULE} has no render_twin function. No twin view.")
    return render


def _decode(data, what):
    if not isinstance(data, (bytes, bytearray)) or not data:
        raise Refusal(f"{what} has no image bytes")
    bgr = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or bgr.shape[0] * bgr.shape[1] > 8_000_000:
        raise Refusal(f"{what} could not be decoded")
    return bgr


def _labelled(bgr, title, subtitle):
    bar = np.zeros((LABEL_HEIGHT, bgr.shape[1], 3), np.uint8)
    for text, y, scale in ((title, 19, .55), (subtitle, 39, .48)):
        cv2.putText(bar, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, bgr])


def compose_side_by_side(phone_bgr, twin_bgr, phone_label, twin_label):
    """Phone frame left, twin front right, scaled to the twin's height, each with a two-line label bar."""
    height = twin_bgr.shape[0]
    width = max(1, min(round(phone_bgr.shape[1] * height / phone_bgr.shape[0]), 2 * height))
    phone = cv2.resize(phone_bgr, (width, height), interpolation=cv2.INTER_AREA)
    gap = np.full((height + LABEL_HEIGHT, 4, 3), 255, np.uint8)
    canvas = np.hstack([_labelled(phone, *phone_label), gap, _labelled(twin_bgr, *twin_label)])
    ok, buffer = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise Refusal("Could not encode the comparison image")
    return buffer.tobytes(), canvas.shape


class TwinRobot:
    """Decorate the chat's robot client with robot_get_twin_view; every other call passes through unchanged."""

    def __init__(self, robot, *, joint_map_path=None, clock=time.time, size=(640, 480),
                 max_age_s=MAX_STATE_AGE_S, renderer=None):
        self.robot = robot
        if joint_map_path is None:
            config = _config_path(robot)
            joint_map_path = config.with_name(JOINT_MAP_NAME) if config else None
        self.joint_map_path = Path(joint_map_path) if joint_map_path else None
        self.clock = clock
        self.size = tuple(size)
        self.max_age_s = max_age_s
        self._renderer = renderer  # tests may inject; otherwise imported lazily on first use
        self._lock = threading.Lock()
        self.last_catalog = None
        self._available = False
        self._native = False
        self._phone = False

    def __getattr__(self, name):
        # Attributes the chat or outer wrappers read (config, link, lan_state, ...) come from the inner client.
        if name.startswith("__") or name == "robot":
            raise AttributeError(name)
        return getattr(self.robot, name)

    def get(self, path):
        return self.robot.get(path)

    def catalog(self):
        catalog = copy.deepcopy(self.robot.catalog())
        functions = {t["function"]["name"]: t["function"] for t in catalog["tools"]}
        self._native = TOOL_NAME in functions
        self._available = not self._native and "robot_get_state" in functions
        cameras = (functions.get("robot_get_cameras", {}).get("parameters", {}).get("properties", {})
                   .get("cameras", {}).get("items", {}).get("enum", []))
        self._phone = isinstance(cameras, list) and "phone" in cameras
        if self._available:
            catalog["tools"].append(tool_schema())
            catalog.setdefault("metadata", {})["twin_view"] = {
                "tool": TOOL_NAME, "execution": "local_model_render_from_owner_encoder_status",
                "motor_access": False, "synthetic_images": True, "phone_compare_available": self._phone}
        self.last_catalog = catalog
        return catalog

    def call(self, name, args, request_id=None):
        if name != TOOL_NAME:
            return self._forward(name, args, request_id)
        if self.last_catalog is None:
            self.catalog()
        if self._native:
            return self._forward(name, args, request_id)
        if not self._available:
            return _fail("The robot server does not publish robot_get_state, so the twin cannot be posed.")
        if not self._lock.acquire(timeout=LOCK_TIMEOUT_S):
            return _fail("Another twin render is still running; try again.")
        try:
            return self._twin_view(args, request_id)
        except _NETWORK_ERRORS:
            raise  # the chat retries read-only tools on these while the robot server restarts
        except Refusal as exc:
            return _fail(str(exc))
        except Exception as exc:  # never raise a renderer or decoding fault into the chat
            return _fail(f"Twin view failed ({type(exc).__name__}: {str(exc)[:300]}). No twin view; use the real cameras.")
        finally:
            self._lock.release()

    # -- the tool -----------------------------------------------------------------------------------

    @staticmethod
    def _arguments(args):
        if not isinstance(args, dict) or set(args) - {"views", "compare_with_phone"}:
            raise Refusal("Unknown twin view arguments; allowed: views, compare_with_phone")
        views = args.get("views", list(VIEWS))
        if (not isinstance(views, list) or not 1 <= len(views) <= len(VIEWS) or len(set(map(str, views))) != len(views)
                or any(not isinstance(v, str) or v not in VIEWS for v in views)):
            raise Refusal("views must be distinct names from front, left, right, top")
        compare = args.get("compare_with_phone", False)
        if type(compare) is not bool:
            raise Refusal("compare_with_phone must be true or false")
        return views, compare

    def _check_age(self, stamp, what):
        age = self.clock() - stamp
        if not -MAX_FUTURE_SKEW_S <= age <= self.max_age_s:
            raise Refusal(f"{what} is {age:.2f} s old on this machine's clock (allowed {-MAX_FUTURE_SKEW_S} to "
                          f"{self.max_age_s} s): the owner may not be polling the servos, or the two Macs' clocks "
                          "disagree. No twin view; use the real cameras.")
        return max(0.0, age)

    def _read_state(self, request_id):
        state = _unwrap(self._forward("robot_get_state", {"fresh": False}, self._sub_id(request_id, "state")),
                        "robot_get_state")
        stamp = state.get("time", state.get("owner_time"))
        if not _finite(stamp):
            raise Refusal("robot_get_state returned no encoder reading time")
        if isinstance(state.get("motors"), list):
            rows = {r.get("name"): r for r in state["motors"] if isinstance(r, dict)}
        elif isinstance(state.get("live_rows"), dict):
            rows = {n: r for n, r in state["live_rows"].items() if isinstance(r, dict)}
        else:
            raise Refusal("robot_get_state returned no motor rows")
        positions, stale = {}, []
        for name, row in rows.items():
            value = row.get("Present_Position")
            if not isinstance(name, str) or not _finite(value) or value != int(value):
                continue
            positions[name] = int(value)
            captured = row.get("captured_at")
            if _finite(captured) and stamp - captured > self.max_age_s:
                stale.append(name)
        if not positions:
            raise Refusal("robot_get_state returned no encoder positions")
        ranges = {}
        for name, span in (state.get("raw_calibration_ranges") or {}).items():
            if isinstance(span, dict) and type(span.get("min_ticks")) is int and type(span.get("max_ticks")) is int:
                ranges[name] = (span["min_ticks"], span["max_ticks"])
        if not ranges:
            raise Refusal("robot_get_state returned no raw_calibration_ranges")
        return state, float(stamp), positions, ranges, sorted(stale)

    def _read_phone(self, request_id):
        result = self._forward("robot_get_cameras", {"cameras": ["phone"], "revive": False},
                               self._sub_id(request_id, "phone"))
        _unwrap(result, "robot_get_cameras")
        images = [i for i in result.get("images", []) if isinstance(i, dict)]
        image = next((i for i in images if i.get("camera_id") == "phone_overview" or i.get("camera_name") == "phone"),
                     images[0] if len(images) == 1 else None)
        if image is None:
            errors = (result.get("result") or {}).get("camera_errors", {})
            raise Refusal(f"Phone camera returned no image: {str(errors.get('phone', errors))[:300]}")
        if image.get("mime_type", image.get("mime")) not in ("image/jpeg", "image/png"):
            raise Refusal("Phone image is not a JPEG or PNG")
        encoded = image.get("data_base64", image.get("base64"))
        if not isinstance(encoded, str):
            raise Refusal("Phone image has no pixels")
        captured, received = image.get("captured_at"), image.get("received_at")
        stamp = captured if _finite(captured) else received
        if not _finite(stamp):
            raise Refusal("Phone image has no timestamp")
        self._check_age(stamp, "The phone frame")
        bgr = _decode(base64.b64decode(encoded, validate=True), "Phone image")
        basis = "captured" if _finite(captured) else "received"
        return bgr, stamp, basis, image

    def _twin_view(self, args, request_id):
        views, compare = self._arguments(args)
        state, stamp, positions, ranges, stale = self._read_state(request_id)
        self._check_age(stamp, "The encoder reading")
        joint_map, joint_map_sha = load_joint_map(self.joint_map_path)

        phone = compare_error = None
        if compare:
            if not self._phone:
                compare_error = "The robot server does not offer the phone camera"
            else:
                try:
                    phone = self._read_phone(request_id)
                except Refusal as exc:
                    compare_error = str(exc)
                except (ValueError, cv2.error) as exc:
                    compare_error = f"Phone image unusable: {exc}"
        render_views = list(views) + (["front"] if phone and "front" not in views else [])

        render = self._renderer or _render_function()
        started = time.perf_counter()
        try:
            rendered = render(positions, ranges, views=tuple(render_views), size=self.size, joint_map=joint_map)
        except Exception as exc:
            raise Refusal(f"Twin renderer failed ({type(exc).__name__}: {str(exc)[:300]}). "
                          "No twin view; use the real cameras.") from None
        render_s = time.perf_counter() - started
        if not isinstance(rendered, dict) or not isinstance(rendered.get("images"), list):
            raise Refusal("Twin renderer returned no images")
        by_view = {i.get("view"): i for i in rendered["images"] if isinstance(i, dict)}
        missing = [v for v in render_views if v not in by_view or not isinstance(by_view[v].get("data"), (bytes, bytearray))]
        if missing:
            raise Refusal(f"Twin renderer returned no image for {', '.join(missing)}")

        validated = rendered.get("mapping_validated") is True
        mapping = rendered.get("mapping", CANDIDATE_MAPPING)
        images = []
        compare_info = None
        if phone:
            phone_bgr, phone_stamp, basis, phone_image = phone
            now = self.clock()
            twin_bgr = _decode(by_view["front"]["data"], "Twin front render")
            data, shape = compose_side_by_side(
                phone_bgr, twin_bgr,
                ("PHONE CAMERA (real)", f"{basis} {_clock_label(phone_stamp)}  age {max(0.0, now - phone_stamp):.2f} s"),
                ("TWIN MODEL front (not a camera)",
                 f"encoders {_clock_label(stamp)}  age {max(0.0, now - stamp):.2f} s"
                 + ("" if validated else "  mapping UNVALIDATED")))
            images.append({"camera_id": "twin-compare-phone", "camera_name": "phone_vs_twin_front",
                           "view": "compare_front", "mime_type": "image/jpeg",
                           "data_base64": base64.b64encode(data).decode(),
                           "captured_at": stamp, "received_at": phone_stamp if basis == "received" else None,
                           "synthetic": "right_panel_only", "width": shape[1], "height": shape[0]})
            compare_info = {"ok": True, "layout": "phone left, twin front right",
                            "phone_camera_id": phone_image.get("camera_id"), "phone_time": phone_stamp,
                            "phone_timestamp_basis": basis, "phone_minus_encoder_s": round(phone_stamp - stamp, 3)}
        elif compare:
            compare_info = {"ok": False, "error": compare_error}
        for view in views:
            if phone and view == "front":
                continue  # already the right half of the comparison image
            images.append({"camera_id": f"twin-{view}", "camera_name": f"twin_{view}", "view": view,
                           "mime_type": by_view[view].get("mime_type", "image/jpeg"),
                           "data_base64": base64.b64encode(bytes(by_view[view]["data"])).decode(),
                           "captured_at": stamp, "received_at": None, "synthetic": True})

        note = ("Rendered MODEL, not a camera: posed from encoder readings at state_time. "
                + ("The tick-to-angle mapping is marked validated. " if validated else
                   f"The tick-to-angle mapping {mapping} is an UNVALIDATED candidate, so a joint can appear mirrored "
                   "or offset. ")
                + "Trust the real cameras for contact and clearance.")
        if stale:
            note += f" Rows older than {self.max_age_s} s at state_time: {', '.join(stale)}; those joints may lag."
        result = {"views": list(views), "angles_deg": rendered.get("angles_deg", {}),
                  "unmapped": rendered.get("unmapped", []), "mapping": mapping, "mapping_validated": validated,
                  "model": rendered.get("model"), "state_time": stamp,
                  "state_age_s": round(max(0.0, self.clock() - stamp), 3), "render_s": round(render_s, 3),
                  "state_source": state.get("source"), "state_cached": state.get("cached"),
                  "stale_motors": stale, "motors_posed": len(positions),
                  "joint_map": {"path": str(self.joint_map_path) if self.joint_map_path else None,
                                "loaded": joint_map is not None, "sha256": joint_map_sha,
                                "validated": bool(joint_map and joint_map.get("validated") is True)},
                  "synthetic_images": True, "motor_writes": 0, "note": note}
        if compare_info is not None:
            result["compare_with_phone"] = compare_info
        return {"ok": True, "result": result, "images": images}

    # -- plumbing -----------------------------------------------------------------------------------

    @staticmethod
    def _sub_id(request_id, suffix):
        # The robot server caches by request_id, so each inner read needs its own id.
        return None if request_id is None else f"{str(request_id)[:100]}:twin-{suffix}"

    def _forward(self, name, args, request_id):
        if request_id is None:
            return self.robot.call(name, args)
        return self.robot.call(name, args, request_id=request_id)
