"""MuJoCo rendered-camera validation of the production Gemma AprilTag adapter.

Reuses an existing photo-informed SO-101 PaddleSimulation checkout. No robot
client, serial transport, hardware configuration or live cameras are imported.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from carton.servo.tag_kit import marker_grid
from farm.perception.gemma_tags import TagObserver, TagRobot

WIDTH, HEIGHT = 640, 480


def words(values):
    return " ".join(str(float(v)) for v in values)


def add_marker(parent, name, tag_id, size, pos, xyaxes=None):
    """Physical square cells on a white backing; perspective/occlusion in MuJoCo."""
    attrs = {"name": name, "pos": words(pos)}
    if xyaxes is not None:
        attrs["xyaxes"] = words(xyaxes)
    body = ET.SubElement(parent, "body", **attrs)
    cell = size / 8
    common = {"type": "box", "contype": "0", "conaffinity": "0", "mass": "0"}
    ET.SubElement(body, "geom", name=name+"_paper", pos="0 0 0",
                  size=words([cell*5, cell*5, .00015]), rgba="1 1 1 1", **common)
    for row, cells in enumerate(marker_grid(tag_id)):
        for col, value in enumerate(cells):
            if value == 0:
                ET.SubElement(body, "geom", name=f"{name}_cell_{row}_{col}",
                              pos=words([(col-3.5)*cell, (3.5-row)*cell, .0003]),
                              size=words([cell/2, cell/2, .00015]), rgba="0 0 0 1", **common)
    ET.SubElement(body, "site", name=name+"_center", pos="0 0 .00045", size=".001", rgba="0 0 0 0")
    return body


def camera_payload(rgb, seq, stream_id, captured_at, fovy_degrees, camera_id="sim"):
    """robot_get_cameras-shaped envelope for one rendered rectified pinhole frame."""
    height, width = rgb.shape[:2]
    ok, encoded = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError("Failed to encode simulated image")
    digest = hashlib.sha256(encoded).hexdigest()
    frame = {"camera_id": camera_id, "captured_at": captured_at, "seq": seq,
             "stream_id": stream_id, "sha256": digest, "mime_type": "image/png",
             "projection": "rectified_pinhole", "data_base64": base64.b64encode(encoded).decode()}
    focal = height / (2 * math.tan(math.radians(fovy_degrees) / 2))
    return {"ok": True, "result": {"cameras": {camera_id: {"camera_id": camera_id, "seq": seq,
            "stream_id": stream_id, "sha256": digest, "width": width, "height": height,
            "projection": "rectified_pinhole", "coordinate_frame": "sim_camera_optical",
            "intrinsics": [[focal, 0, width/2], [0, focal, height/2], [0, 0, 1]]}}},
            "images": [frame], "simulation_only": True}


# Decoded AprilTag axes (tag_geometry.square_points: +x right, +y up as printed,
# +z out of the printed face) relative to an add_marker() body frame.
MARKER_FROM_DECODED = np.diag([-1., 1., -1., 1.])


def _pose(xpos, xmat):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = np.asarray(xmat).reshape(3, 3), xpos
    return out


class MarkerScene:
    """Minimal rendered tag scene that needs no PaddleSimulation checkout.

    Table tag 1, a rigid gripper body carrying tag 2 and a paddle body carrying
    tag 3 plus a ``handle`` site. The camera image goes through the production
    detector/IPPE path. Ground-truth poses are for test evaluation only.
    """

    def __init__(self, width=1280, height=960, *, fovy=45., paddle_pos=(.26, -.04, .012),
                 paddle_yaw_degrees=25., handle_in_paddle=(-.075, .002, -.0058),
                 gripper_pos=(.17, -.15, .11), base_pos=(.08, -.04, 0.), base_yaw_degrees=15.,
                 camera_pos=(.47, -.33, .36), lookat=(.23, -.06, .03)):
        self.width, self.height, self.fovy = width, height, fovy
        root = ET.Element("mujoco", model="marker_scene")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
        ET.SubElement(visual, "headlight", ambient=".5 .5 .5", diffuse=".5 .5 .5", specular="0 0 0")
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(world, "geom", name="table", type="box", size="1 1 .01", pos="0 0 -.01",
                      rgba=".55 .52 .48 1", contype="0", conaffinity="0")
        add_marker(world, "tag1", 1, .060, [.33, .09, .0005])
        yaw = math.radians(paddle_yaw_degrees)
        paddle = ET.SubElement(world, "body", name="paddle", pos=words(paddle_pos),
                               xyaxes=words([math.cos(yaw), math.sin(yaw), 0, -math.sin(yaw), math.cos(yaw), 0]))
        ET.SubElement(paddle, "geom", name="paddle_board", type="box", size=".035 .03 .0055",
                      pos="0 0 -.0058", rgba=".72 .58 .40 1", contype="0", conaffinity="0")
        ET.SubElement(paddle, "geom", name="paddle_handle", type="capsule", size=".006",
                      fromto=words(np.add(handle_in_paddle, [.04, 0, 0]).tolist() + np.add(handle_in_paddle, [-.04, 0, 0]).tolist()),
                      rgba=".35 .25 .15 1", contype="0", conaffinity="0")
        add_marker(paddle, "tag3", 3, .040, [0, 0, .0001])
        ET.SubElement(paddle, "site", name="handle", pos=words(handle_in_paddle), size=".002", rgba="0 0 0 0")
        camera_pos, lookat = np.asarray(camera_pos, float), np.asarray(lookat, float)
        # Gripper housing faces the camera at a 25 degree slant so tag 2's pose is unambiguous.
        toward = camera_pos - np.asarray(gripper_pos)
        toward /= np.linalg.norm(toward)
        normal = toward + np.array([.35, 0, -.25])
        normal /= np.linalg.norm(normal)
        x_axis = np.cross([0, 0, 1], normal)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis)
        gripper = ET.SubElement(world, "body", name="gripper", pos=words(gripper_pos),
                                xyaxes=words(np.r_[x_axis, y_axis]))
        ET.SubElement(gripper, "geom", name="housing", type="box", size=".03 .03 .01", pos="0 0 -.0105",
                      rgba=".2 .2 .22 1", contype="0", conaffinity="0")
        add_marker(gripper, "tag2", 2, .040, [0, 0, .0001])
        b = math.radians(base_yaw_degrees)
        ET.SubElement(world, "body", name="arm_base", pos=words(base_pos),
                      xyaxes=words([math.cos(b), math.sin(b), 0, -math.sin(b), math.cos(b), 0]))
        back = camera_pos - lookat
        back /= np.linalg.norm(back)
        right = np.cross([0, 0, 1], back)
        right /= np.linalg.norm(right)
        up = np.cross(back, right)
        ET.SubElement(world, "camera", name="tag_camera", pos=words(camera_pos),
                      xyaxes=words(np.r_[right, up]), fovy=str(fovy))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.renderer = None
        self.stream_id = "marker-scene-" + uuid.uuid4().hex
        self.seq = 0

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.renderer.update_scene(self.data, camera="tag_camera")
        return self.renderer.render().copy()

    def payload(self, captured_at):
        self.seq += 1
        return camera_payload(self.render(), self.seq, self.stream_id, captured_at, self.fovy)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    # Ground truth below is for evaluation only.
    def world_from(self, name):
        if name == "camera_optical":
            camera = self.model.camera("tag_camera").id
            return _pose(self.data.cam_xpos[camera], self.data.cam_xmat[camera]) @ np.diag([1., -1., -1., 1.])
        if name == "handle":
            site = self.data.site("handle")
            return _pose(site.xpos, site.xmat)
        if name.startswith("tag"):
            body = self.data.body(name)
            return _pose(body.xpos, body.xmat) @ MARKER_FROM_DECODED
        body = self.data.body(name)
        return _pose(body.xpos, body.xmat)

    def base_from(self, name):
        return np.linalg.inv(self.world_from("arm_base")) @ self.world_from(name)

    def handle_in_decoded_tag3(self):
        return (np.linalg.inv(self.world_from("tag3")) @ self.world_from("handle"))[:3, 3]


class CameraSimulation:
    def __init__(self, source, out, metric=False, width=WIDTH, height=HEIGHT):
        self.out = out
        self.width, self.height = width, height
        out.mkdir(parents=True, exist_ok=True)
        sys.path.insert(0, str(source))
        module = importlib.import_module("paddle_sim")
        module.OUT = out / "base-scene"
        module.OUT.mkdir(exist_ok=True)
        module.build_model()
        tree = ET.parse(module.OUT / "scene.xml")
        root = tree.getroot()
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        global_options = visual.find("global")
        if global_options is None:
            global_options = ET.SubElement(visual, "global")
        global_options.set("offwidth", str(width))
        global_options.set("offheight", str(height))
        world = root.find("worldbody")
        gripper = root.find(".//body[@name='gripper_link']")
        paddle = root.find(".//body[@name='paddle']")
        add_marker(world, "tag1", 1, .060, [.60, .03, module.TABLE_Z+.001])
        add_marker(gripper, "tag2", 2, .040, [.045, 0, .008], [0, 1, 0, 0, 0, 1])
        target = add_marker(paddle, "tag3", 3, .040, [.135, 0, .007])
        ET.SubElement(target, "geom", name="occluder", type="box", size=".026 .026 .001",
                      pos="0 0 .002", rgba=".4 .4 .4 0", contype="0", conaffinity="0", mass="0")
        duplicate = add_marker(world, "duplicate3", 3, .040, [.52, .06, module.TABLE_Z+.002])
        for geom in duplicate.findall("geom"):
            geom.set("rgba", geom.get("rgba").rsplit(" ", 1)[0] + " 0")
        position = np.array([.54, -.42, .46])
        lookat = np.array([.35, .005, -.015])
        back = position-lookat
        back /= np.linalg.norm(back)
        right = np.cross([0, 0, 1], back)
        right /= np.linalg.norm(right)
        up = np.cross(back, right)
        ET.SubElement(world, "camera", name="tag_camera", pos=words(position),
                      xyaxes=words(np.r_[right, up]), fovy="40")
        tree.write(out / "tag-scene.xml", encoding="unicode")
        model = mujoco.MjModel.from_xml_path(str(out / "tag-scene.xml"))
        original_builder = module.build_model
        module.build_model = lambda: model
        try:
            self.sim = module.PaddleSimulation(render=False)
        finally:
            module.build_model = original_builder
        self.model, self.data = self.sim.model, self.sim.data
        self.camera = self.model.camera("tag_camera").id
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.seq, self.calls, self.frames, self.samples = 0, [], [], []
        self.stream_id = 'mujoco-tags-' + uuid.uuid4().hex
        self.stopped = False
        self.initial_pan = float(self.data.qpos[0])
        geometry = {"schema": 1, "family": "tag36h11", "camera_ids": ["sim"],
                    "tags": {str(i): {"black_square_mm": size, "source": "Exact simulator geometry"}
                             for i, size in ((1, 60), (2, 40), (3, 40))}} if metric else None
        self.tagged = TagRobot(self, observer=TagObserver(geometry=geometry))
        self.tagged.catalog()

    def catalog(self):
        return {"tools": [{"type": "function", "function": {
            "name": "robot_get_cameras", "parameters": {"type": "object", "properties": {
                "cameras": {"type": "array", "items": {"type": "string", "enum": ["sim"]}}}}}}],
            "metadata": {"simulation_only": True}}

    def render(self):
        self.renderer.update_scene(self.data, camera="tag_camera", scene_option=self.option)
        return self.renderer.render().copy()

    def call(self, name, args, request_id=None):
        if name != "robot_get_cameras":
            raise ValueError("Simulation backend only supplies rendered camera images")
        rgb = self.render()
        self.seq += 1
        self.calls.append(name)
        return camera_payload(rgb, self.seq, self.stream_id, time.time(),
                              self.model.cam_fovy[self.camera])

    def ground_truth_metric(self):
        rotation = self.data.cam_xmat[self.camera].reshape(3, 3)
        origin = self.data.cam_xpos[self.camera]
        return {i: ((self.data.site(f"tag{i}_center").xpos-origin) @ rotation) * [1000, -1000, -1000]
                for i in (1, 2, 3)}

    def ground_truth(self):
        rotation = self.data.cam_xmat[self.camera].reshape(3, 3)
        origin = self.data.cam_xpos[self.camera]
        focal = self.height / (2 * math.tan(math.radians(self.model.cam_fovy[self.camera]) / 2))
        pixels = {}
        for tag_id in [1, 2, 3]:
            relative = (self.data.site(f"tag{tag_id}_center").xpos-origin) @ rotation
            pixels[tag_id] = [self.width/2 + focal*relative[0]/-relative[2],
                              self.height/2 - focal*relative[1]/-relative[2]]
        return pixels

    def observe(self, label, record=True):
        result = self.tagged.call("robot_get_tags", {"cameras": ["sim"]})
        row = result.get("result", {}).get("observations", {}).get("sim", {})
        truth = self.ground_truth()
        detected = {t["tag_id"]: t for t in row.get("tags", []) if t["status"] == "DETECTED"}
        errors = {str(i): float(np.linalg.norm(np.array(t["center_px"])-truth[i]))
                  for i, t in detected.items() if i in truth}
        pair_error = None
        if row.get("gripper_to_paddle_px"):
            delta = np.array(truth[3])-truth[2]
            measured = row["gripper_to_paddle_px"]
            pair_error = float(np.linalg.norm(delta-[measured["dx"], measured["dy"]]))
        sample = {"label": label, "sim_time_s": float(self.data.time), "joint_pan_degrees": math.degrees(self.data.qpos[0]),
                  "detected_ids": sorted(detected), "center_error_px": errors, "relative_error_px": pair_error,
                  "observation": {k: v for k, v in result.items() if k != "images"}}
        metric_truth = self.ground_truth_metric()
        sample["metric_center_error_mm"] = {str(t["tag_id"]): float(np.linalg.norm(
            np.array(t["center_camera_mm"]) - metric_truth[t["tag_id"]]))
            for t in (row.get("pose_3d") or {}).get("tags", []) if t.get("center_camera_mm") is not None}
        if record:
            self.samples.append(sample)
            rgb = self.render()
            if result.get("images"):
                rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(result["images"][0]["data_base64"]), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            picture = Image.fromarray(rgb)
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, self.width, 42), fill="white")
            draw.text((9, 7), "SIMULATION ONLY | Actual SO-101 / paddle meshes | Production tag detector", fill="black")
            draw.text((9, 24), label, fill="black")
            self.frames.append(picture)
        return sample

    def move(self, delta_degrees, seconds=1.0):
        if self.stopped:
            return {"ok": False, "reason": "Simulation STOP latched", "simulation_only": True}
        if type(delta_degrees) not in (int, float) or not math.isfinite(delta_degrees) or abs(delta_degrees) > 12:
            raise ValueError("Simulation test permits shoulder-pan offsets of at most 12 degrees")
        start = self.data.ctrl.copy()
        target = start.copy()
        target[0] = self.initial_pan + math.radians(delta_degrees)
        lo, hi = self.model.jnt_range[0]
        if not lo <= target[0] <= hi:
            raise ValueError("Outside simulator joint range")
        steps = round(seconds/self.model.opt.timestep)
        for i in range(steps):
            t = min(1., (i+1)/(steps*.8))
            self.data.ctrl[:] = start+(target-start)*(t*t*(3-2*t))
            mujoco.mj_step(self.model, self.data)
            if i % 100 == 0:
                self.observe(f"Shoulder pan toward {delta_degrees:+g} deg offset")
        return {"ok": True, "simulation_only": True, "measured_pan_degrees": math.degrees(self.data.qpos[0])}

    def faults(self):
        out = {}
        index = self.model.geom("occluder").id
        self.model.geom_rgba[index, 3] = 1
        hidden = self.observe("Negative control: paddle tag physically occluded")
        self.frames[-1].save(self.out / "occluded.png")
        row = hidden["observation"].get("result", {}).get("observations", {}).get("sim", {})
        out["occlusion_rejected"] = 3 not in hidden["detected_ids"] and row.get("gripper_to_paddle_px") is None
        self.model.geom_rgba[index, 3] = 0
        duplicate_body = self.model.body("duplicate3").id
        ids = np.flatnonzero(self.model.geom_bodyid == duplicate_body)
        self.model.geom_rgba[ids, 3] = 1
        duplicated = self.observe("Negative control: duplicate ID3")
        self.frames[-1].save(self.out / "duplicate.png")
        out["duplicate_identity_rejected"] = not duplicated["observation"]["ok"]
        self.model.geom_rgba[ids, 3] = 0
        target_body = self.model.body("tag3").id
        target_geoms = np.flatnonzero(self.model.geom_bodyid == target_body)
        original_positions = self.model.geom_pos[target_geoms].copy()
        original_sizes = self.model.geom_size[target_geoms].copy()
        # Keep this negative control comparably small in pixels at each resolution.
        tiny_scale = .4 * WIDTH / self.width
        self.model.geom_pos[target_geoms, :2] *= tiny_scale
        self.model.geom_size[target_geoms, :2] *= tiny_scale
        mujoco.mj_forward(self.model, self.data)
        tiny = self.observe("Negative control: paddle marker too small")
        self.frames[-1].save(self.out / "undersized.png")
        tiny_row = tiny["observation"].get("result", {}).get("observations", {}).get("sim", {})
        out["undersized_tag_rejected"] = 3 not in tiny["detected_ids"] and tiny_row.get("gripper_to_paddle_px") is None
        self.model.geom_pos[target_geoms] = original_positions
        self.model.geom_size[target_geoms] = original_sizes
        mujoco.mj_forward(self.model, self.data)
        stale = self.call("robot_get_cameras", {})
        stale["images"][0]["captured_at"] -= 10
        result = TagObserver().observe(stale, ["sim"])
        out["stale_frame_rejected"] = not result["ok"]
        corrupt = self.call("robot_get_cameras", {})
        corrupt["images"][0]["sha256"] = "bad"
        out["hash_mismatch_rejected"] = not TagObserver().observe(corrupt, ["sim"])["ok"]
        recovered = self.observe("Tags reacquired after occlusion removed")
        out["reacquired"] = recovered["detected_ids"] == [1, 2, 3]
        return out

    def save(self, report):
        (self.out / "result.json").write_text(json.dumps(report, indent=2)+"\n")
        if self.frames:
            self.frames[0].save(self.out / "before.png")
            self.frames[-1].save(self.out / "after.png")
            frames = [self.frames[0]]*8+self.frames+[self.frames[-1]]*12
            frames[0].save(self.out / "apriltag-simulation.gif", save_all=True,
                           append_images=frames[1:], duration=100, loop=0)


def run_gemma(sim):
    """Actual local Gemma inference; the only motion tool changes MuJoCo state."""
    import jsonschema
    from gemma_mujoco import request, tool
    tags = next(t for t in sim.tagged.catalog()["tools"] if t["function"]["name"] == "robot_get_tags")
    moves = tool("sim_move_pan", "SIMULATION ONLY. Move shoulder pan to an offset from its initial angle, in degrees. Never contacts physical hardware.",
                 {"delta_degrees": {"type": "number", "minimum": -12, "maximum": 12}})
    stop = tool("sim_stop", "Latch simulation STOP; no further simulated movement.")
    tools = [tags, moves, stop]
    schemas = {t["function"]["name"]: t["function"]["parameters"] for t in tools}
    messages = [{"role": "system", "content":
        "You are testing an AprilTag integration in a MuJoCo SIMULATION ONLY. No physical robot is connected. "
        "Use exactly one tool per turn and inspect its result. Pixel positions are per-image, not millimetres. "
        "If any tool returns ok=false, stop the simulation and report the failure. Do not claim a grasp or real robot success."},
        {"role": "user", "content":
         "Read robot_get_tags using camera sim. Then use sim_move_pan to move the simulated shoulder to +12 degrees offset. "
         "Read robot_get_tags again to verify the gripper-to-paddle pixel displacement changed. "
         "Finally call sim_stop and briefly report the before/after offsets. This is a tracking test, not a grasp."}]
    trace, observations = [], []
    outcome = {"model": "google/gemma-4-e4b", "hardware_tools_exposed": False, "calls": trace}
    try:
        for _ in range(8):
            response, latency = request(messages, tools)
            message = response["message"]
            messages.append(message)
            calls = message.get("tool_calls") or []
            if not calls:
                outcome["answer"] = (message.get("content") or "").split("<channel|>")[-1].strip()
                break
            if len(calls) != 1:
                raise ValueError("Gemma must choose one simulation tool per turn")
            call = calls[0]
            name, arguments = call["function"]["name"], json.loads(call["function"]["arguments"])
            if name not in schemas:
                raise ValueError("Unknown tool; no physical backend exists")
            jsonschema.validate(arguments, schemas[name])
            if name == "robot_get_tags":
                result = sim.tagged.call(name, arguments)
                observations.append(result.get("result", {}).get("observations", {}).get("sim", {}).get("gripper_to_paddle_px"))
                sim.observe("Gemma reads simulated tags")
            elif name == "sim_move_pan":
                result = sim.move(arguments["delta_degrees"], 1.5)
            else:
                sim.stopped = True
                result = {"ok": True, "simulation_only": True, "stop_latched": True}
            compact = {k: v for k, v in result.items() if k != "images"}
            trace.append({"name": name, "arguments": arguments, "latency_s": latency, "result": compact})
            print(json.dumps({"gemma_tool": name, "latency_s": latency, "ok": result["ok"]}), flush=True)
            messages.append({"role": "tool", "tool_call_id": call["id"], "content": json.dumps(compact)})
            pictures = [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,"+i["data_base64"]}}
                        for i in result.get("images", [])]
            if pictures:
                messages.append({"role": "user", "content": [{"type": "text", "text": "Rendered simulation observations."}]+pictures})
            if result.get("ok") is not True:
                raise RuntimeError("Simulation tool refused the requested operation")
        names = [row["name"] for row in trace]
        changed = len(observations) >= 2 and all(observations) and math.hypot(
            observations[-1]["dx"]-observations[0]["dx"], observations[-1]["dy"]-observations[0]["dy"]) > 3
        outcome["passed"] = (names == ["robot_get_tags", "sim_move_pan", "robot_get_tags", "sim_stop"]
                             and bool(changed) and sim.stopped)
        outcome["before_after_displacement"] = observations
    except Exception as exc:
        outcome.update(passed=False, error=str(exc))
    finally:
        sim.stopped = True
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--initial-only", action="store_true")
    parser.add_argument("--gemma", action="store_true", help="Also test actual local LM Studio Gemma tool use in simulation")
    parser.add_argument("--metric", action="store_true", help="Also compare camera-relative metric estimates with simulator ground truth")
    parser.add_argument("--resolution", choices=[640, 1280, 1920], type=int, default=640,
                        help="Native render width; height preserves the 4:3 camera field of view")
    args = parser.parse_args()
    sim = CameraSimulation(args.simulation_root.resolve(), args.out.resolve(), metric=args.metric,
                           width=args.resolution, height=args.resolution*3//4)
    try:
        first = sim.observe("All tags visible: initial simulated pose")
        sim.frames[-1].save(sim.out / "initial.png")
        print(json.dumps({k: v for k, v in first.items() if k != "observation"}), flush=True)
        if args.initial_only:
            sim.save(first)
            return
        for delta in (12, -12, 0):
            sim.move(delta, 1.5)
        nominal = list(sim.samples)
        faults = sim.faults()
        sim.stopped = True
        faults["stop_enforced"] = sim.move(2)["ok"] is False
        errors = [v for s in nominal for v in s["center_error_px"].values()]
        pair_errors = [s["relative_error_px"] for s in nominal if s["relative_error_px"] is not None]
        report = {"environment": "MUJOCO_RENDERED_RGB", "mujoco_version": mujoco.__version__,
                  "image_size_px": [sim.width, sim.height],
                  "production_adapter": "farm.perception.gemma_tags.TagRobot", "hardware_connected": False,
                  "motor_writes": 0, "physical_grasp_validated": False,
                  "assumptions": "Existing photo-informed model; tag mounts/camera pose are illustrative, not a calibrated real scene.",
                  "nominal_frames": len(nominal), "all_tags_detected_frames": sum(s["detected_ids"] == [1, 2, 3] for s in nominal),
                  "max_center_error_px": max(errors, default=None), "max_relative_error_px": max(pair_errors, default=None),
                  "measured_pan_travel_degrees": max(s["joint_pan_degrees"] for s in nominal)-min(s["joint_pan_degrees"] for s in nominal),
                  "negative_controls": faults, "samples": sim.samples}
        report["passed"] = (report["all_tags_detected_frames"] == len(nominal) and bool(errors) and max(errors) < 2
                            and bool(pair_errors) and max(pair_errors) < 2 and all(faults.values())
                            and report["measured_pan_travel_degrees"] >= 20)
        if args.metric:
            metric_errors = [v for s in nominal for v in s["metric_center_error_mm"].values()]
            report["metric_validation"] = {"compared_tag_positions": len(metric_errors),
                "expected_tag_positions": len(nominal)*3, "acceptance_target_mm": 5,
                "max_position_error_mm": max(metric_errors, default=None)}
            report["metric_validation"]["passed"] = len(metric_errors) == len(nominal)*3 and max(metric_errors) <= 5
            report["passed"] = report["passed"] and report["metric_validation"]["passed"]
        if args.gemma:
            # A new, deliberately started simulation goal follows the STOP test.
            sim.stopped = False
            report["gemma"] = run_gemma(sim)
            report["passed"] = report["passed"] and report["gemma"]["passed"]
        sim.save(report)
        summary = {k: v for k, v in report.items() if k not in ("samples", "gemma")}
        if "gemma" in report:
            summary["gemma"] = {"passed": report["gemma"]["passed"],
                                "tools_called": [c["name"] for c in report["gemma"]["calls"]],
                                "answer": report["gemma"].get("answer"), "error": report["gemma"].get("error")}
        print(json.dumps(summary, indent=2), flush=True)
        if not report["passed"]:
            raise SystemExit(1)
    finally:
        sim.renderer.close()


if __name__ == "__main__":
    main()
