"""robot_get_twin_view: chat-side wrapper around the twin renderer, with a fake robot and a fake renderer."""
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
from farm.perception.twin_robot import TOOL_NAME, TwinRobot

NOW = 1000.0
MOTORS = {"right_arm_shoulder_pan": 2048, "right_arm_elbow_flex": 1500, "left_arm_gripper": 1393,
          "head_motor_1": 1623}
RANGES = {n: {"min_ticks": 900, "max_ticks": 3100} for n in MOTORS}


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
    def __init__(self, config=None, state_time=NOW - .3, phone_time=NOW - .2, extra_tools=()):
        self.config = config
        self.link = "lan"
        self.calls = []
        self.request_ids = []
        self.state_time = state_time
        self.phone_time = phone_time
        self.phone_error = None
        self.network_error = False
        self.tools = [tool("robot_get_state", {"fresh": {"type": "boolean"}}),
                      tool("robot_get_cameras", {"cameras": {"type": "array", "items": {
                          "type": "string", "enum": ["oak", "phone", "left_wrist", "right_wrist"]}},
                          "revive": {"type": "boolean"}}),
                      tool("robot_set_motor_enable"), tool("robot_move_motor_targets"), tool("robot_stop"),
                      *extra_tools]
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
                    "model": "xlerobot-test"}
        finally:
            with self.guard:
                self.active -= 1


@pytest.fixture
def renderer(monkeypatch):
    fake = FakeRenderer()
    module = types.ModuleType(twin_robot.RENDERER_MODULE)
    module.render_twin = fake.render_twin
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
    assert len(robot.tools) == 5  # the inner catalog is not mutated
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
    json.dumps(answer["result"], allow_nan=False)


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
