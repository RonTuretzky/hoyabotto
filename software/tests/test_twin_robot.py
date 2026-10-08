"""robot_get_twin_view, robot_get_claw_positions and robot_get_scene_points: chat-side wrappers around the twin
renderer, with a fake robot (encoder state, phone frame, synthetic 16-bit depth PNG) and a fake renderer."""
import base64
import copy
import json
import sys
import threading
import time
import types
import urllib.error

import cv2
import jsonschema
import numpy as np
import pytest

from farm.perception import twin_robot
from farm.perception.twin_robot import CLAW_TOOL_NAME, SCENE_TOOL_NAME, TOOL_NAME, TwinRobot

NOW = 1000.0
MOTORS = {"right_arm_shoulder_pan": 2048, "right_arm_elbow_flex": 1500, "left_arm_gripper": 1393,
          "head_motor_1": 1623}
RANGES = {n: {"min_ticks": 900, "max_ticks": 3100} for n in MOTORS}
FRAME = "Origin on the floor below the shoulder midpoint; +forward front, +left left, +up; metres."
# Synthetic OAK depth: 640x360 mm, background 1.5 m, a 100x80 px box at 0.5 m (pixels x 100..199, y 130..209), one hole.
INTRINSICS = [[505.0, 0.0, 315.0], [0.0, 505.0, 193.0], [0.0, 0.0, 1.0]]
DEPTH_BOX = (100, 130, 200, 210)
# Camera 1.181 m up, 0.037 m ahead, level and forward: optical x = robot right, y = down, z = forward.
CAMERA = {"position_m": [0.037, -0.002, 1.181], "rotation": [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
          "site": "head_camera_link/twin_head_optical", "frame": FRAME, "head_sign_note": "tilt + = down",
          "head_angles_deg": {"pan": 0.0, "tilt": None}}


def fake_claws(positions_ticks, joint_map):
    arm = {"forward_m": 0.358612, "left_m": 0.15561, "up_m": 1.00377, "reach_m": 0.37504, "shoulder_up_m": 0.894,
           "shoulder_left_m": 0.1552, "tip_site": "Fixed_Jaw/twin_tip_L"}
    return {"left_arm": arm, "right_arm": dict(arm, left_m=-0.15481, tip_site="Fixed_Jaw_2/twin_tip_R"),
            "frame": FRAME, "mapping": "feetech_degrees_v1+joint_map" if joint_map else "feetech_degrees_v1",
            "mapping_validated": bool(joint_map and joint_map.get("validated")),
            "unmapped": [n for n in positions_ticks if n.startswith("head")], "model": "xlerobot-test"}


def depth_png(box_mm=500, background_mm=1500):
    depth = np.full((360, 640), background_mm, np.uint16)
    x0, y0, x1, y1 = DEPTH_BOX
    depth[y0:y1, x0:x1] = box_mm
    depth[40:100, 500:560] = 0
    ok, data = cv2.imencode(".png", depth)
    assert ok
    return data.tobytes()


def jpeg(width, height, color):
    image = np.zeros((height, width, 3), np.uint8)
    image[:] = color
    ok, data = cv2.imencode(".jpg", image)
    assert ok
    return data.tobytes()


def tool(name, properties=None):
    return {"type": "function", "function": {"name": name, "description": name,
            "parameters": {"type": "object", "properties": properties or {}, "additionalProperties": False}}}


class FakeRobot:
    def __init__(self, config=None, state_time=NOW - .3, phone_time=NOW - .2, depth_time=NOW - .4, extra_tools=(),
                 depth_tool=True):
        self.config = config
        self.link = "lan"
        self.calls = []
        self.request_ids = []
        self.state_time = state_time
        self.phone_time = phone_time
        self.depth_time = depth_time
        self.depth_png = depth_png()
        self.depth_manifest_extra = {}
        self.depth_error = None
        self.phone_error = None
        self.network_error = False
        self.tools = [tool("robot_get_state", {"fresh": {"type": "boolean"}}),
                      tool("robot_get_cameras", {"cameras": {"type": "array", "items": {
                          "type": "string", "enum": ["oak", "phone", "left_wrist", "right_wrist"]}},
                          "revive": {"type": "boolean"}}),
                      tool("robot_set_motor_enable"), tool("robot_move_motor_targets"), tool("robot_stop"),
                      *([tool("robot_get_depth")] if depth_tool else []), *extra_tools]
        self.last_catalog = None

    def get(self, path):
        return {"path": path}

    def catalog(self):
        self.last_catalog = {"tools": copy.deepcopy(self.tools), "metadata": {"stop_tool": "robot_stop"}}
        return self.last_catalog

    def call(self, name, args, request_id=None):
        self.calls.append((name, copy.deepcopy(args)))
        self.request_ids.append(request_id)
        if self.network_error:
            raise urllib.error.URLError("relay restarting")
        if name == "robot_get_state":
            return {"ok": True, "result": {
                "source": "canonical_hardware_owner", "time": self.state_time, "cached": False,
                "motors": [{"name": n, "Present_Position": p, "captured_at": self.state_time - .05,
                            "Torque_Enable": 0} for n, p in MOTORS.items()],
                "raw_calibration_ranges": copy.deepcopy(RANGES),
                "commandable_ranges": {n: [950, 3050] for n in MOTORS}}}
        if name == "robot_get_cameras":
            if self.phone_error:
                return {"ok": True, "result": {"cameras": {}, "camera_errors": {"phone": {"error": self.phone_error}}},
                        "images": []}
            data = jpeg(320, 240, (0, 0, 200))
            return {"ok": True, "result": {"cameras": {"phone": {"seq": 4}}, "camera_errors": {}},
                    "images": [{"camera_id": "phone_overview", "mime_type": "image/jpeg", "captured_at": None,
                                "received_at": self.phone_time, "seq": 4,
                                "data_base64": base64.b64encode(data).decode()}]}
        if name == "robot_get_depth":
            if self.depth_error:
                return {"ok": False, "result": {"error": self.depth_error}}
            manifest = {"schema": 1, "camera_id": "oak-18443010", "stream_id": "s1", "seq": 77,
                        "captured_at": self.depth_time - .01, "rgb_captured_at": self.depth_time - .01,
                        "depth_captured_at": self.depth_time, "width": 640, "height": 360,
                        "intrinsics": INTRINSICS, "alignment": "CAM_A RGB", "depth_units": "mm", "invalid_depth": 0,
                        "projection": "camera_pinhole_with_factory_distortion", "distortion_model": "Perspective",
                        "distortion_coefficients": [0.0] * 14, "coordinate_frame": "CAM_A_optical",
                        "robot_frame_calibrated": False, **self.depth_manifest_extra}
            return {"ok": True, "result": {"manifest": manifest, "source": "stream", "reason": "unverified"},
                    "images": [{"camera_id": "oak-18443010:depth", "mime_type": "image/png", "captured_at": self.depth_time,
                                "received_at": None, "seq": 77, "data_base64": base64.b64encode(self.depth_png).decode()}]}
        return {"ok": True, "result": {"forwarded": name}}


class FakeRenderer:
    def __init__(self, fail=None, delay=0.0):
        self.calls = []
        self.fail = fail
        self.delay = delay
        self.active = 0
        self.max_active = 0
        self.guard = threading.Lock()

    def render_twin(self, positions_ticks, ranges, *, views=("front", "left", "right", "top"), size=(640, 480), joint_map=None):
        with self.guard:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        try:
            time.sleep(self.delay)
            self.calls.append({"positions": dict(positions_ticks), "ranges": dict(ranges), "views": tuple(views),
                               "size": size, "joint_map": copy.deepcopy(joint_map)})
            if self.fail:
                raise self.fail
            colors = {"front": (0, 200, 0), "left": (0, 0, 200), "right": (200, 0, 0), "top": (200, 200, 200)}
            return {"images": [{"view": v, "mime_type": "image/jpeg", "data": jpeg(size[0], size[1], colors[v])}
                               for v in views],
                    "angles_deg": {n: 0.0 for n in positions_ticks if n.startswith(("left_arm", "right_arm"))},
                    "unmapped": [n for n in positions_ticks if n.startswith("head")],
                    "mapping": "feetech_degrees_v1+joint_map" if joint_map else "feetech_degrees_v1",
                    "mapping_validated": bool(joint_map and joint_map.get("validated")),
                    "model": "xlerobot-test",
                    "claws": fake_claws(positions_ticks, joint_map)}
        finally:
            with self.guard:
                self.active -= 1

    def claw_positions(self, positions_ticks, ranges, *, joint_map=None):
        self.calls.append({"positions": dict(positions_ticks), "ranges": dict(ranges), "views": None,
                           "joint_map": copy.deepcopy(joint_map)})
        if self.fail:
            raise self.fail
        return fake_claws(positions_ticks, joint_map)

    def camera_pose(self, positions_ticks, ranges, *, joint_map=None, camera="oak"):
        self.calls.append({"positions": dict(positions_ticks), "ranges": dict(ranges), "views": "camera",
                           "joint_map": copy.deepcopy(joint_map), "camera": camera})
        if self.fail:
            raise self.fail
        return dict(copy.deepcopy(CAMERA), camera=camera, model="xlerobot-test",
                    mapping="feetech_degrees_v1+joint_map" if joint_map else "feetech_degrees_v1",
                    mapping_validated=bool(joint_map and joint_map.get("validated")),
                    unmapped=[n for n in positions_ticks if n.startswith("head")])


@pytest.fixture
def renderer(monkeypatch):
    fake = FakeRenderer()
    module = types.ModuleType(twin_robot.RENDERER_MODULE)
    module.render_twin = fake.render_twin
    module.claw_positions = fake.claw_positions
    module.camera_pose = fake.camera_pose
    module.VIEWS = ("front", "left", "right", "top")
    monkeypatch.setitem(sys.modules, twin_robot.RENDERER_MODULE, module)
    return fake


def make(robot=None, **kwargs):
    robot = robot or FakeRobot()
    kwargs.setdefault("clock", lambda: NOW)
    return robot, TwinRobot(robot, **kwargs)


def decode(image):
    data = base64.b64decode(image["data_base64"], validate=True)
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


def schema_of(catalog):
    return next(t["function"] for t in catalog["tools"] if t["function"]["name"] == TOOL_NAME)


def test_catalog_adds_the_tool_once_with_the_published_schema():
    robot, twin = make()
    first = twin.catalog()
    twin.catalog()
    catalog = twin.catalog()
    assert [t["function"]["name"] for t in catalog["tools"]].count(TOOL_NAME) == 1
    assert twin.last_catalog is catalog and first is not catalog
    assert len(robot.tools) == 6  # the inner catalog is not mutated
    function = schema_of(catalog)
    assert function["description"].startswith(
        "Third-person views of a MODEL of the robot posed from the live servo encoder readings (not a camera). "
        "Use it to see how the arms are placed. The tick-to-angle mapping is the unvalidated candidate "
        "feetech_degrees_v1 unless mapping_validated is true, so a joint can appear mirrored or offset; trust the "
        "real cameras for contact and clearance.")
    params = function["parameters"]
    assert params["additionalProperties"] is False
    assert params["properties"]["views"]["items"]["enum"] == ["front", "left", "right", "top"]
    assert params["properties"]["views"]["uniqueItems"] is True
    assert params["properties"]["views"]["default"] == ["front", "left", "right", "top"]
    assert params["properties"]["compare_with_phone"] == {
        "type": "boolean", "default": False, "description": params["properties"]["compare_with_phone"]["description"]}
    assert catalog["metadata"]["twin_view"]["motor_access"] is False
    assert catalog["metadata"]["stop_tool"] == "robot_stop"


def test_schema_rejects_unknown_views_and_arguments(renderer):
    robot, twin = make()
    params = schema_of(twin.catalog())["parameters"]
    jsonschema.validate({}, params)
    jsonschema.validate({"views": ["top", "front"], "compare_with_phone": True}, params)
    for bad in ({"views": ["back"]}, {"views": ["front", "front"]}, {"views": []},
                {"compare_with_phone": "yes"}, {"zoom": 2}):
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(bad, params)
        answer = twin.call(TOOL_NAME, bad)
        assert answer["ok"] is False and answer["result"]["error"]
    assert robot.calls == [] and renderer.calls == []


def test_other_tools_and_attributes_pass_through():
    robot, twin = make(FakeRobot(config="/tmp/x/robot.json"))
    assert twin.call("robot_move_motor_targets", {"positions": {"a": 1}}, request_id="r1") == {
        "ok": True, "result": {"forwarded": "robot_move_motor_targets"}}
    assert twin.call("robot_stop", {}) == {"ok": True, "result": {"forwarded": "robot_stop"}}
    assert robot.calls == [("robot_move_motor_targets", {"positions": {"a": 1}}), ("robot_stop", {})]
    assert robot.request_ids == ["r1", None]
    assert twin.get("/tools") == {"path": "/tools"}
    assert twin.config == "/tmp/x/robot.json" and twin.link == "lan"
    assert twin.last_catalog is None  # its own, not the inner client's


def test_server_native_tool_is_preferred():
    native = tool(TOOL_NAME)
    robot, twin = make(FakeRobot(extra_tools=[native]))
    catalog = twin.catalog()
    assert [t["function"]["name"] for t in catalog["tools"]].count(TOOL_NAME) == 1
    assert twin.call(TOOL_NAME, {}) == {"ok": True, "result": {"forwarded": TOOL_NAME}}


def test_default_call_reads_state_only_and_returns_four_labelled_images(renderer):
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is True
    assert robot.calls == [("robot_get_state", {"fresh": False})]
    call = renderer.calls[0]
    assert call["positions"] == MOTORS
    assert call["ranges"] == {n: (900, 3100) for n in MOTORS}
    assert call["views"] == ("front", "left", "right", "top") and call["size"] == (640, 480) and call["joint_map"] is None
    assert [i["camera_id"] for i in answer["images"]] == ["twin-front", "twin-left", "twin-right", "twin-top"]
    for image in answer["images"]:
        assert image["mime_type"] == "image/jpeg" and image["synthetic"] is True
        assert image["captured_at"] == NOW - .3 and image["received_at"] is None
        assert image["camera_name"] == "twin_" + image["view"]
        assert decode(image).shape == (480, 640, 3)
    result = answer["result"]
    assert result["views"] == ["front", "left", "right", "top"]
    assert result["mapping"] == "feetech_degrees_v1" and result["mapping_validated"] is False
    assert result["unmapped"] == ["head_motor_1"] and result["model"] == "xlerobot-test"
    assert set(result["angles_deg"]) == {n for n in MOTORS if "arm" in n}
    assert result["state_time"] == NOW - .3 and result["state_age_s"] == pytest.approx(.3)
    assert result["render_s"] >= 0 and result["motor_writes"] == 0
    assert "UNVALIDATED" in result["note"] and "not a camera" in result["note"]
    assert "compare_with_phone" not in result
    assert result["claws"]["left_arm"]["forward_m"] == 0.359 and result["claws"]["right_arm"]["left_m"] == -0.155
    assert result["claws"]["frame"] == FRAME
    json.dumps(answer["result"], allow_nan=False)


# ---------------------------------------------------------------- robot_get_claw_positions

def test_catalog_adds_the_claw_tool_with_no_parameters():
    robot, twin = make()
    catalog = twin.catalog()
    names = [t["function"]["name"] for t in catalog["tools"]]
    assert names.count(CLAW_TOOL_NAME) == 1 and names.index(CLAW_TOOL_NAME) > names.index(TOOL_NAME)
    assert len(robot.tools) == 6
    function = next(t["function"] for t in catalog["tools"] if t["function"]["name"] == CLAW_TOOL_NAME)
    assert function["parameters"] == {"type": "object", "properties": {}, "additionalProperties": False}
    assert "forward_m" in function["description"] and "older than 2 s" in function["description"]
    assert catalog["metadata"]["claw_positions"] == {
        "tool": CLAW_TOOL_NAME, "execution": "local_model_kinematics_from_owner_encoder_status",
        "motor_access": False, "synthetic_images": False}
    jsonschema.validate({}, function["parameters"])
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({"arm": "left"}, function["parameters"])


def test_claw_positions_reads_state_only_and_returns_rounded_metres(renderer):
    robot, twin = make()
    answer = twin.call(CLAW_TOOL_NAME, {}, request_id="q1")
    assert answer["ok"] is True and "images" not in answer
    assert robot.calls == [("robot_get_state", {"fresh": False})] and robot.request_ids == ["q1:twin-state"]
    assert renderer.calls == [{"positions": MOTORS, "ranges": {n: (900, 3100) for n in MOTORS}, "views": None,
                               "joint_map": None}]
    result = answer["result"]
    assert set(result) == {"left_arm", "right_arm", "frame", "mapping", "mapping_validated", "unmapped", "model",
                           "state_time", "state_age_s", "compute_s", "state_source", "state_cached", "stale_motors",
                           "motors_posed", "joint_map", "motor_writes", "note"}
    assert result["left_arm"] == {"forward_m": 0.359, "left_m": 0.156, "up_m": 1.004, "reach_m": 0.375,
                                  "shoulder_up_m": 0.894, "shoulder_left_m": 0.155, "tip_site": "Fixed_Jaw/twin_tip_L"}
    assert result["right_arm"]["left_m"] == -0.155 and result["right_arm"]["tip_site"] == "Fixed_Jaw_2/twin_tip_R"
    assert result["frame"] == FRAME and result["model"] == "xlerobot-test"
    assert result["mapping"] == "feetech_degrees_v1" and result["mapping_validated"] is False
    assert result["unmapped"] == ["head_motor_1"]
    assert result["state_time"] == NOW - .3 and result["state_age_s"] == pytest.approx(.3)
    assert result["compute_s"] >= 0 and result["motor_writes"] == 0 and result["motors_posed"] == 4
    assert result["note"] == "model estimate from encoder readings with the unvalidated candidate mapping; not measured"
    assert result["joint_map"] == {"path": None, "loaded": False, "sha256": None, "validated": False}
    json.dumps(answer, allow_nan=False)


def test_claw_positions_refuses_arguments(renderer):
    robot, twin = make()
    answer = twin.call(CLAW_TOOL_NAME, {"arm": "left"})
    assert answer["ok"] is False and "no arguments" in answer["result"]["error"]
    assert robot.calls == [] and renderer.calls == []


@pytest.mark.parametrize("state_time,ok", [(NOW - 2.5, False), (NOW + .5, False), (NOW + .1, True), (NOW - 1.9, True)])
def test_claw_positions_freshness(renderer, state_time, ok):
    robot, twin = make(FakeRobot(state_time=state_time))
    answer = twin.call(CLAW_TOOL_NAME, {})
    assert answer["ok"] is ok
    if not ok:
        assert "encoder reading" in answer["result"]["error"] and renderer.calls == []
    assert robot.calls == [("robot_get_state", {"fresh": False})]


def test_claw_positions_use_the_joint_map(renderer, tmp_path):
    config = tmp_path / "robot.json"
    joint_map = {"validated": True, "joints": {"right_arm_shoulder_pan": {"zero_tick": 2047, "sign": -1}}}
    (tmp_path / "twin-joint-map.json").write_text(json.dumps(joint_map))
    robot, twin = make(FakeRobot(config=config))
    result = twin.call(CLAW_TOOL_NAME, {})["result"]
    assert renderer.calls[0]["joint_map"] == joint_map
    assert result["mapping"] == "feetech_degrees_v1+joint_map" and result["mapping_validated"] is True
    assert result["joint_map"]["loaded"] is True and result["joint_map"]["validated"] is True
    assert result["note"] == "model estimate from encoder readings with the validated joint map; not measured"


def test_claw_positions_failures_are_reported_not_raised(renderer, monkeypatch):
    renderer.fail = RuntimeError("model lacks the shoulder-pan joints")
    robot, twin = make()
    answer = twin.call(CLAW_TOOL_NAME, {})
    assert answer["ok"] is False and "shoulder-pan" in answer["result"]["error"]
    monkeypatch.setitem(sys.modules, twin_robot.RENDERER_MODULE, None)
    answer = twin.call(CLAW_TOOL_NAME, {})
    assert answer["ok"] is False and "unavailable" in answer["result"]["error"]


def test_server_native_claw_tool_is_preferred_while_twin_view_is_added():
    robot, twin = make(FakeRobot(extra_tools=[tool(CLAW_TOOL_NAME)]))
    names = [t["function"]["name"] for t in twin.catalog()["tools"]]
    assert names.count(CLAW_TOOL_NAME) == 1 and names.count(TOOL_NAME) == 1
    assert twin.call(CLAW_TOOL_NAME, {}) == {"ok": True, "result": {"forwarded": CLAW_TOOL_NAME}}


def test_claw_tool_stacks_under_the_chat_wrappers(renderer, tmp_path):
    from farm.perception.gemma_calibration import CalibrationRobot
    from farm.perception.gemma_tags import TagRobot
    robot = FakeRobot(config=tmp_path / "robot.json")
    chat = CalibrationRobot(TagRobot(TwinRobot(robot, clock=lambda: NOW)))
    names = [t["function"]["name"] for t in chat.catalog()["tools"]]
    assert names.count(CLAW_TOOL_NAME) == 1
    answer = chat.call(CLAW_TOOL_NAME, {})
    assert answer["ok"] is True and answer["result"]["left_arm"]["up_m"] == 1.004
    assert robot.calls == [("robot_get_state", {"fresh": False})]


def test_selected_views_only(renderer):
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {"views": ["top"]})
    assert renderer.calls[0]["views"] == ("top",)
    assert [i["camera_id"] for i in answer["images"]] == ["twin-top"]


def test_compare_with_phone_composes_one_side_by_side_jpeg(renderer):
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {"compare_with_phone": True})
    assert answer["ok"] is True
    assert robot.calls == [("robot_get_state", {"fresh": False}),
                           ("robot_get_cameras", {"cameras": ["phone"], "revive": False})]
    assert [i["camera_id"] for i in answer["images"]] == ["twin-compare-phone", "twin-left", "twin-right", "twin-top"]
    composite = answer["images"][0]
    pixels = decode(composite)
    # phone 320x240 scaled to the twin's 480 height (640 wide), 4 px gap, twin 640 wide, 46 px label bars
    assert pixels.shape == (480 + 46, 640 + 4 + 640, 3)
    assert (composite["width"], composite["height"]) == (1284, 526)
    phone_pixel, twin_pixel = pixels[300, 300].astype(int), pixels[300, 1000].astype(int)
    assert phone_pixel[2] > 150 and phone_pixel[1] < 60  # red phone frame on the left (BGR)
    assert twin_pixel[1] > 150 and twin_pixel[2] < 60    # green twin front on the right
    assert pixels[:40, :600].max() > 200                  # label text drawn in the bar
    assert composite["captured_at"] == NOW - .3 and composite["received_at"] == NOW - .2
    compare = answer["result"]["compare_with_phone"]
    assert compare["ok"] is True and compare["phone_timestamp_basis"] == "received"
    assert compare["phone_minus_encoder_s"] == pytest.approx(.1)


def test_compare_renders_front_even_when_not_requested(renderer):
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {"views": ["top"], "compare_with_phone": True})
    assert renderer.calls[0]["views"] == ("top", "front")
    assert [i["camera_id"] for i in answer["images"]] == ["twin-compare-phone", "twin-top"]


def test_compare_without_a_phone_frame_still_returns_the_twin(renderer):
    robot = FakeRobot()
    robot.phone_error = "phone image is stale"
    robot, twin = make(robot)
    answer = twin.call(TOOL_NAME, {"compare_with_phone": True, "views": ["right"]})
    assert answer["ok"] is True
    assert answer["result"]["compare_with_phone"]["ok"] is False
    assert "stale" in answer["result"]["compare_with_phone"]["error"]
    assert [i["camera_id"] for i in answer["images"]] == ["twin-right"]
    assert renderer.calls[0]["views"] == ("right",)


def test_stale_phone_frame_is_not_compared(renderer):
    robot, twin = make(FakeRobot(phone_time=NOW - 5))
    answer = twin.call(TOOL_NAME, {"compare_with_phone": True})
    assert answer["result"]["compare_with_phone"]["ok"] is False
    assert "phone frame" in answer["result"]["compare_with_phone"]["error"]
    assert [i["camera_id"] for i in answer["images"]] == ["twin-front", "twin-left", "twin-right", "twin-top"]


def test_joint_map_next_to_the_robot_config_is_loaded_and_passed(renderer, tmp_path):
    config = tmp_path / "robot.json"
    joint_map = {"validated": True, "joints": {"right_arm_shoulder_pan": {"zero_tick": 2047, "sign": -1}}}
    (tmp_path / "twin-joint-map.json").write_text(json.dumps(joint_map))
    robot, twin = make(FakeRobot(config=config))
    assert twin.joint_map_path == tmp_path / "twin-joint-map.json"
    answer = twin.call(TOOL_NAME, {"views": ["front"]})
    assert renderer.calls[0]["joint_map"] == joint_map
    result = answer["result"]
    assert result["joint_map"]["loaded"] is True and result["joint_map"]["validated"] is True
    assert len(result["joint_map"]["sha256"]) == 64
    assert result["mapping"] == "feetech_degrees_v1+joint_map" and result["mapping_validated"] is True
    assert "UNVALIDATED" not in result["note"]


def test_explicit_joint_map_path_and_absent_file(renderer, tmp_path):
    robot, twin = make(joint_map_path=tmp_path / "missing.json")
    answer = twin.call(TOOL_NAME, {"views": ["front"]})
    assert answer["ok"] is True and renderer.calls[0]["joint_map"] is None
    assert answer["result"]["joint_map"]["loaded"] is False


@pytest.mark.parametrize("content", ["{not json", json.dumps({"joints": {"a": {"zero_tick": 1.5, "sign": 1}}}),
                                     json.dumps({"joints": {"a": {"zero_tick": 1, "sign": 2}}}),
                                     json.dumps({"validated": "yes", "joints": {}})])
def test_malformed_joint_map_is_refused(renderer, tmp_path, content):
    path = tmp_path / "twin-joint-map.json"
    path.write_text(content)
    robot, twin = make(joint_map_path=path)
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is False and "joint map" in answer["result"]["error"].lower()
    assert renderer.calls == []


@pytest.mark.parametrize("state_time,ok", [(NOW - 2.5, False), (NOW + .5, False), (NOW + .1, True), (NOW - 1.9, True)])
def test_encoder_state_freshness(renderer, state_time, ok):
    robot, twin = make(FakeRobot(state_time=state_time))
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is ok
    if not ok:
        assert "encoder reading" in answer["result"]["error"]
        assert "images" not in answer and renderer.calls == []
        assert robot.calls == [("robot_get_state", {"fresh": False})]


def test_refused_state_read(renderer):
    robot = FakeRobot()
    robot.call = lambda name, args, request_id=None: {"ok": False, "result": {"error": "HARDWARE_OWNER_UNAVAILABLE"}}
    robot, twin = make(robot)
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is False and "HARDWARE_OWNER_UNAVAILABLE" in answer["result"]["error"]


def test_renderer_import_error_is_reported_not_raised(monkeypatch):
    monkeypatch.setitem(sys.modules, twin_robot.RENDERER_MODULE, None)  # import raises ImportError
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is False
    assert "renderer" in answer["result"]["error"] and "unavailable" in answer["result"]["error"]
    assert robot.calls == [("robot_get_state", {"fresh": False})]


def test_renderer_failure_is_reported_not_raised(renderer):
    renderer.fail = RuntimeError("mujoco GL context lost")
    robot, twin = make()
    answer = twin.call(TOOL_NAME, {})
    assert answer["ok"] is False and "mujoco GL context lost" in answer["result"]["error"]


def test_network_errors_propagate_so_the_chat_retries(renderer):
    robot = FakeRobot()
    robot.network_error = True
    robot, twin = make(robot)
    with pytest.raises(urllib.error.URLError):
        twin.call(TOOL_NAME, {})
    robot.network_error = False
    assert twin.call(TOOL_NAME, {})["ok"] is True  # the lock was released


def test_inner_reads_get_their_own_request_ids(renderer):
    robot, twin = make()
    twin.call(TOOL_NAME, {"compare_with_phone": True}, request_id="abc")
    assert robot.request_ids == ["abc:twin-state", "abc:twin-phone"]


def test_one_render_at_a_time(monkeypatch):
    fake = FakeRenderer(delay=.05)
    robot, twin = make(clock=time.time, renderer=fake.render_twin)
    robot.state_time = time.time() + 1  # refreshed below for each call
    original = robot.call

    def fresh_state(name, args, request_id=None):
        robot.state_time = time.time()
        return original(name, args, request_id)
    robot.call = fresh_state
    results = []
    threads = [threading.Thread(target=lambda: results.append(twin.call(TOOL_NAME, {"views": ["front"]})))
               for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert [r["ok"] for r in results] == [True] * 4
    assert fake.max_active == 1


def test_stacks_under_the_chat_wrappers(renderer, tmp_path):
    from farm.perception.gemma_calibration import CalibrationRobot
    from farm.perception.gemma_tags import TagRobot
    config = tmp_path / "robot.json"
    robot = FakeRobot(config=config)
    chat = CalibrationRobot(TagRobot(TwinRobot(robot, clock=lambda: NOW)))
    names = [t["function"]["name"] for t in chat.catalog()["tools"]]
    assert names.count(TOOL_NAME) == 1 and "robot_get_tags" in names
    answer = chat.call(TOOL_NAME, {"views": ["right"]})
    assert answer["ok"] is True and [i["camera_id"] for i in answer["images"]] == ["twin-right"]
    assert robot.calls == [("robot_get_state", {"fresh": False})]


# ---------------------------------------------------------------- robot_get_scene_points

def scene_schema(catalog):
    return next(t["function"] for t in catalog["tools"] if t["function"]["name"] == SCENE_TOOL_NAME)


def test_catalog_adds_the_scene_tool_with_the_published_schema():
    robot, twin = make()
    catalog = twin.catalog()
    names = [t["function"]["name"] for t in catalog["tools"]]
    assert names.count(SCENE_TOOL_NAME) == 1 and names.index(SCENE_TOOL_NAME) > names.index(CLAW_TOOL_NAME)
    assert len(robot.tools) == 6
    function = scene_schema(catalog)
    assert function["description"].startswith(
        "Numeric 3D positions from the head depth camera, in the robot frame (forward/left/up from the base): a 5x3 "
        "grid of region distances, the nearest object's position and size, optional points at given pixels, and each "
        "claw's offset to the nearest object. Use it for distances instead of guessing from images. The stereo depth "
        "is blind closer than about 25 cm and on textureless or blown-out areas.")
    params = function["parameters"]
    assert params["additionalProperties"] is False and set(params["properties"]) == {"pixels", "with_claws"}
    assert params["properties"]["pixels"]["maxItems"] == 10 and params["properties"]["pixels"]["default"] == []
    assert params["properties"]["with_claws"]["type"] == "boolean" and params["properties"]["with_claws"]["default"] is True
    assert catalog["metadata"]["scene_points"] == {
        "tool": SCENE_TOOL_NAME, "execution": "local_depth_backprojection_with_model_camera_pose",
        "motor_access": False, "synthetic_images": False, "depth_tool": "robot_get_depth"}
    jsonschema.validate({}, params)
    jsonschema.validate({"pixels": [[320, 180], [0, 0]], "with_claws": False}, params)
    for bad in ({"pixels": [[1, 2, 3]]}, {"pixels": [[-1, 2]]}, {"pixels": [[1.5, 2]]}, {"pixels": [[1, 2]] * 11},
                {"with_claws": "yes"}, {"arm": "left"}):
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(bad, params)


def test_scene_tool_is_not_offered_without_robot_get_depth(renderer):
    robot, twin = make(FakeRobot(depth_tool=False))
    names = [t["function"]["name"] for t in twin.catalog()["tools"]]
    assert SCENE_TOOL_NAME not in names and CLAW_TOOL_NAME in names
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "robot_get_depth" in answer["result"]["error"]
    assert robot.calls == [] and renderer.calls == []


def test_scene_points_reads_state_and_depth_once_and_returns_robot_frame_numbers(renderer):
    robot, twin = make()
    answer = twin.call(SCENE_TOOL_NAME, {"pixels": [[150, 175], [530, 70]]}, request_id="s1")
    assert answer["ok"] is True and "images" not in answer
    assert robot.calls == [("robot_get_state", {"fresh": False}), ("robot_get_depth", {})]
    assert robot.request_ids == ["s1:twin-state", "s1:twin-depth"]
    assert [c["views"] for c in renderer.calls] == ["camera", None]   # camera pose, then claws
    assert renderer.calls[0]["positions"] == MOTORS and renderer.calls[0]["camera"] == "oak"
    assert renderer.calls[0]["ranges"] == {n: (900, 3100) for n in MOTORS} and renderer.calls[0]["joint_map"] is None
    result = answer["result"]
    assert set(result) == {"image", "valid_fraction", "invalid_fraction", "centre_invalid_fraction", "min_valid_distance_m",
                           "max_valid_distance_m", "grid", "nearest", "query", "undistorted", "undistortion",
                           "distance_definition", "frame", "notes", "camera", "claws", "claw_to_nearest_m",
                           "depth_camera_id", "depth_seq", "depth_captured_at", "depth_age_s", "depth_minus_state_s",
                           "robot_frame_calibrated", "state_time", "state_age_s", "compute_s", "state_source",
                           "state_cached", "stale_motors", "motors_posed", "mapping", "mapping_validated", "unmapped",
                           "model", "joint_map", "motor_writes", "note"}
    assert result["image"] == {"width": 640, "height": 360}
    assert result["depth_seq"] == 77 and result["depth_captured_at"] == NOW - .4 and result["depth_age_s"] == pytest.approx(.4)
    assert result["depth_camera_id"] == "oak-18443010:depth" and result["depth_minus_state_s"] == pytest.approx(-.1)
    assert result["state_time"] == NOW - .3 and result["state_age_s"] == pytest.approx(.3)
    assert result["undistorted"] is False and "carries the factory distortion" in result["undistortion"]
    assert result["robot_frame_calibrated"] is False and result["motor_writes"] == 0
    assert result["mapping"] == "feetech_degrees_v1" and result["mapping_validated"] is False
    assert result["unmapped"] == ["head_motor_1"] and result["model"] == "xlerobot-test"
    assert result["note"] == ("robot-frame positions use the model camera pose and the unvalidated candidate head/arm "
                              "mapping; depth itself is measured")
    assert result["camera"]["position_m"] == [0.037, -0.002, 1.181]
    assert result["camera"]["rotation"] == CAMERA["rotation"] and result["camera"]["head_sign_note"] == "tilt + = down"
    assert result["camera"]["site"] == "head_camera_link/twin_head_optical"
    # the box: 100x80 px at 0.5 m, centre pixel (149.5, 169.5) -> 0.537 m ahead, 0.164 m left, 0.023 m above the lens
    near = result["nearest"]
    assert near["pixel_bbox"] == [100, 130, 199, 209] and near["pixel_count"] == 8000
    assert near["centre_m"] == pytest.approx([0.537, 0.162, 1.204], abs=0.002)
    assert near["extent_m"]["width"] == pytest.approx(0.098, abs=0.002) and near["extent_m"]["height"] == pytest.approx(0.078, abs=0.002)
    assert 0.51 <= near["median_distance_m"] <= 0.53
    assert len(result["grid"]["regions"]) == 15
    centre = next(r for r in result["grid"]["regions"] if r["column"] == "centre" and r["row"] == "middle")
    assert centre["median_point_m"][0] == pytest.approx(1.537, abs=0.002) and centre["valid_fraction"] == 1.0
    assert result["invalid_fraction"] == pytest.approx(3600 / (640 * 360), abs=1e-3)
    q = result["query"]
    assert q[0]["pixel"] == [150, 175] and q[0]["point_m"] == pytest.approx([0.537, 0.161, 1.199], abs=0.002)
    assert q[1]["point_m"] is None and "invalid depth" in q[1]["reason"]
    # claws: fake tips at forward 0.359, left +-0.155, up 1.004 -> vector to the box centre
    assert result["claws"]["left_arm"]["forward_m"] == 0.359 and result["claws"]["right_arm"]["left_m"] == -0.155
    assert set(result["claws"]) == {"left_arm", "right_arm"}
    left = result["claw_to_nearest_m"]["left_arm"]
    assert left["forward_m"] == pytest.approx(0.537 - 0.359, abs=0.002)
    assert left["left_m"] == pytest.approx(0.162 - 0.156, abs=0.002)
    assert left["up_m"] == pytest.approx(1.204 - 1.004, abs=0.002)
    assert left["distance_m"] == pytest.approx((left["forward_m"] ** 2 + left["left_m"] ** 2 + left["up_m"] ** 2) ** .5, abs=0.002)
    right = result["claw_to_nearest_m"]["right_arm"]
    assert right["left_m"] == pytest.approx(0.162 + 0.155, abs=0.002) and right["distance_m"] > left["distance_m"]
    assert all(isinstance(v, float) and round(v, 3) == v for v in left.values())
    json.dumps(answer, allow_nan=False)


def test_scene_points_without_claws_skips_kinematics(renderer):
    robot, twin = make()
    answer = twin.call(SCENE_TOOL_NAME, {"with_claws": False})
    assert answer["ok"] is True
    assert [c["views"] for c in renderer.calls] == ["camera"]
    assert answer["result"]["claws"] is None and answer["result"]["claw_to_nearest_m"] is None
    assert answer["result"]["query"] == [] and answer["result"]["nearest"]["pixel_count"] == 8000


def test_scene_points_with_no_valid_depth_reports_null_nearest(renderer):
    robot = FakeRobot()
    robot.depth_png = cv2.imencode(".png", np.zeros((360, 640), np.uint16))[1].tobytes()
    robot, twin = make(robot)
    result = twin.call(SCENE_TOOL_NAME, {"pixels": [[320, 180]]})["result"]
    assert result["nearest"] is None and result["invalid_fraction"] == 1.0
    assert result["claw_to_nearest_m"] == {"left_arm": None, "right_arm": None}
    assert result["query"][0]["point_m"] is None
    assert any("no valid depth anywhere" in n for n in result["notes"])


def test_scene_points_rectified_manifest_passes_no_distortion(renderer):
    robot = FakeRobot()
    robot.depth_manifest_extra = {"projection": "rectified_pinhole"}
    robot, twin = make(robot)
    result = twin.call(SCENE_TOOL_NAME, {})["result"]
    assert result["undistorted"] is False and "rectified_pinhole" in result["undistortion"]


def test_scene_points_distorted_manifest_undistorts_with_cv2(renderer):
    robot = FakeRobot()
    robot.depth_manifest_extra = {"distortion_coefficients": [-0.3, 0.1] + [0.0] * 12}
    robot, twin = make(robot)
    result = twin.call(SCENE_TOOL_NAME, {"pixels": [[315, 193]]})["result"]
    assert result["undistorted"] is True and "undistortPoints" in result["undistortion"]
    assert result["query"][0]["point_m"] == pytest.approx([1.537, -0.002, 1.181], abs=0.001)  # principal point unchanged


@pytest.mark.parametrize("bad", [{"pixels": [[1, 2, 3]]}, {"pixels": [[-1, 2]]}, {"pixels": [[1.5, 2]]},
                                 {"pixels": [[1, 2]] * 11}, {"pixels": "320,180"}, {"with_claws": "yes"}, {"arm": "left"}])
def test_scene_points_refuses_bad_arguments(renderer, bad):
    robot, twin = make()
    answer = twin.call(SCENE_TOOL_NAME, bad)
    assert answer["ok"] is False and answer["result"]["error"]
    assert robot.calls == [] and renderer.calls == []


@pytest.mark.parametrize("state_time,ok", [(NOW - 2.5, False), (NOW + .5, False), (NOW + .1, True), (NOW - 1.9, True)])
def test_scene_points_encoder_freshness(renderer, state_time, ok):
    robot, twin = make(FakeRobot(state_time=state_time))
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is ok
    if not ok:
        assert "encoder reading" in answer["result"]["error"] and "No scene points" in answer["result"]["error"]
        assert robot.calls == [("robot_get_state", {"fresh": False})] and renderer.calls == []


@pytest.mark.parametrize("depth_time,ok", [(NOW - 2.5, False), (NOW + .5, False), (NOW + .1, True), (NOW - 1.9, True)])
def test_scene_points_depth_freshness(renderer, depth_time, ok):
    robot, twin = make(FakeRobot(depth_time=depth_time))
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is ok
    assert robot.calls == [("robot_get_state", {"fresh": False}), ("robot_get_depth", {})]
    if not ok:
        assert "depth frame" in answer["result"]["error"] and renderer.calls == []


def test_scene_points_refused_or_broken_depth_is_reported_not_raised(renderer):
    robot = FakeRobot()
    robot.depth_error = "OAK_UNAVAILABLE"
    robot, twin = make(robot)
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "OAK_UNAVAILABLE" in answer["result"]["error"]
    robot.depth_error = None
    robot.depth_png = jpeg(640, 360, (0, 0, 0))  # not a 16-bit PNG
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "PNG" in answer["result"]["error"]
    robot.depth_png = depth_png()
    robot.depth_manifest_extra = {"intrinsics": None}
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "intrinsics" in answer["result"]["error"]
    robot.depth_manifest_extra = {"width": 320}
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "320" in answer["result"]["error"]
    assert [c["views"] for c in renderer.calls] == ["camera"]  # only the intrinsics case reached the twin
    robot.depth_manifest_extra = {}
    assert twin.call(SCENE_TOOL_NAME, {})["ok"] is True  # the lock was released each time


def test_scene_points_twin_failures_are_reported_not_raised(renderer, monkeypatch):
    renderer.fail = RuntimeError("model lacks the head_camera_link body")
    robot, twin = make()
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "head_camera_link" in answer["result"]["error"]
    monkeypatch.setitem(sys.modules, twin_robot.RENDERER_MODULE, None)
    answer = twin.call(SCENE_TOOL_NAME, {})
    assert answer["ok"] is False and "unavailable" in answer["result"]["error"]


def test_scene_points_use_the_joint_map(renderer, tmp_path):
    config = tmp_path / "robot.json"
    joint_map = {"validated": True, "joints": {"head_motor_2": {"zero_tick": 2047, "sign": -1}}}
    (tmp_path / "twin-joint-map.json").write_text(json.dumps(joint_map))
    robot, twin = make(FakeRobot(config=config))
    result = twin.call(SCENE_TOOL_NAME, {})["result"]
    assert all(c["joint_map"] == joint_map for c in renderer.calls) and len(renderer.calls) == 2
    assert result["mapping"] == "feetech_degrees_v1+joint_map" and result["mapping_validated"] is True
    assert result["joint_map"]["loaded"] is True and result["joint_map"]["validated"] is True
    assert result["note"].startswith("robot-frame positions use the model camera pose and the validated joint map")


def test_scene_points_network_errors_propagate(renderer):
    robot = FakeRobot()
    robot.network_error = True
    robot, twin = make(robot)
    with pytest.raises(urllib.error.URLError):
        twin.call(SCENE_TOOL_NAME, {})
    robot.network_error = False
    assert twin.call(SCENE_TOOL_NAME, {})["ok"] is True


def test_server_native_scene_tool_is_preferred():
    robot, twin = make(FakeRobot(extra_tools=[tool(SCENE_TOOL_NAME)]))
    names = [t["function"]["name"] for t in twin.catalog()["tools"]]
    assert names.count(SCENE_TOOL_NAME) == 1
    assert twin.call(SCENE_TOOL_NAME, {}) == {"ok": True, "result": {"forwarded": SCENE_TOOL_NAME}}


def test_scene_tool_stacks_under_the_chat_wrappers(renderer, tmp_path):
    from farm.perception.gemma_calibration import CalibrationRobot
    from farm.perception.gemma_tags import TagRobot
    robot = FakeRobot(config=tmp_path / "robot.json")
    chat = CalibrationRobot(TagRobot(TwinRobot(robot, clock=lambda: NOW)))
    names = [t["function"]["name"] for t in chat.catalog()["tools"]]
    assert names.count(SCENE_TOOL_NAME) == 1
    answer = chat.call(SCENE_TOOL_NAME, {"pixels": [[150, 175]]})
    assert answer["ok"] is True and answer["result"]["nearest"]["pixel_count"] == 8000
    assert robot.calls == [("robot_get_state", {"fresh": False}), ("robot_get_depth", {})]
