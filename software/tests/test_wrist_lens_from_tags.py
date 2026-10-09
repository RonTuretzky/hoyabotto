"""Wrist-camera lens calibration from the carton's box tags: geometry, solver, planned capture. No robot, network or
camera access: the capture runs against the deployed owner code over the MuJoCo fold scene (carton.fold_policy_fakes).

The MuJoCo tests need a 220 mm robot-model training trial (FOLD_WRIST_TRIAL, default batch-220-01/trial-020) and are
skipped without it.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import pytest

from carton import box_tag_geometry as G
from carton.servo.tag_kit import marker_image
from tools import calibrate_wrist_from_tags as S
from tools import make_fold_box_tags as M

HACK = Path("/Users/wk/Documents/ChatGPT/Hackatuson/output")
TRIAL = Path(os.environ.get("FOLD_WRIST_TRIAL", HACK / "fold-demos/batch-220-01/trial-020"))
SCENE = TRIAL / "run/scene.xml"
needs_scene = pytest.mark.skipif(not SCENE.exists(), reason="220 mm fold training scene not present")
needs_trial = pytest.mark.skipif(not (TRIAL / "demo.npz").exists(), reason="220 mm fold demonstration not present")


# ------------------------------------------------------------------------------------------------ geometry
def test_detector_corner_order_is_top_right_top_left_bottom_left_bottom_right():
    for tag_id in (10, 13, 24):
        img = marker_image(tag_id, 40)                         # black square spans pixels 40..359
        from carton.servo.features import tags_from_bgr
        corners = np.asarray(tags_from_bgr(img)[tag_id]["corners"]) - S.DETECTOR_OFFSET_PX
        expect = np.array([[359.5, 39.5], [39.5, 39.5], [39.5, 359.5], [359.5, 359.5]])   # OpenCV pixel edges
        assert np.abs(corners - expect).max() < 0.3
    # the geometry lists corners in the same order: +right+up, -right+up, -right-up, +right-up
    r, c = G.tag_frame(10)
    rel = (G.tag_corners(10) - c) @ r
    assert np.allclose(np.sign(rel[:, :2]), G.CORNER_SIGNS)


def test_geometry_matches_the_printed_tag_table():
    assert set(G.TAGS) == {t.tag_id for t in M.TAGS} and len(G.TAGS) == 12
    for t in M.TAGS:
        g = G.TAGS[t.tag_id]
        assert g.face == t.face and abs(g.size * 1000 - t.size) < 1e-9
        assert M.SCENE_BODY[t.tag_id] == g.scene_body
        corners = G.tag_corners(t.tag_id)
        sides = np.linalg.norm(corners - np.roll(corners, 1, 0), axis=1)
        assert np.allclose(sides, g.size)
        centre = corners.mean(0)
        if g.face == "floor":                       # read from above, top edge toward the far wall
            assert np.allclose(g.normal, [0, 0, 1]) and np.allclose(g.up, [0, 1, 0])
        else:                                       # upright, facing out of the carton
            assert np.allclose(g.up, [0, 0, 1]) and np.dot(g.normal, centre * [1, 1, 0]) > 0


@needs_scene
def test_tag_centres_match_the_training_scene_bodies():
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    mujoco.mj_forward(m, d)
    world = G.scene_corners_world(m, d)
    for tag_id, t in G.TAGS.items():
        site = d.site_xpos[m.site(t.scene_body + "_center").id]
        assert np.abs(world[tag_id].mean(0) - site).max() < 1e-6, tag_id
    assert max(M.check_against_scene(SCENE).values()) < 2.0


@needs_trial
@pytest.mark.parametrize("state", ["upright", "demo-start"])
def test_projected_corners_match_detections_in_a_rendered_scene(state):
    """Training-scene cameras: geometry -> pinhole projection vs the detector on the render, within 1.5 px."""
    import mujoco
    from carton.servo.features import tags_from_bgr
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0 if state == "upright" else np.load(TRIAL / "demo.npz")["qpos"][0]
    mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, 480, 640)
    world = G.scene_corners_world(m, d)
    checked = set()
    for cam in ("front", "overhead", "station", "side"):
        renderer.update_scene(d, camera=cam)
        found = tags_from_bgr(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
        k, r, p = G.mujoco_camera(m, d, cam, 640, 480)
        for tag_id, f in found.items():
            if tag_id in G.TAGS and f["hamming"] == 0:
                uv, z = G.project(k, r, p, world[tag_id])
                err = np.abs(uv - (np.asarray(f["corners"]) - S.DETECTOR_OFFSET_PX)).max()
                assert (z > 0).all() and err < 1.5, (cam, tag_id, err)
                checked.add(tag_id)
    renderer.close()
    assert {10, 12, 14, 22, 24, 25, 26, 27} <= checked


# ------------------------------------------------------------------------------------------------ solver
def _free_camera_views(fovy, n=14, seed=0):
    """Renders of the demo-start scene (flaps a few degrees off upright) from free cameras with a known fovy."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    d.qpos[:] = np.load(TRIAL / "demo.npz")["qpos"][0]
    mujoco.mj_forward(m, d)
    m.vis.global_.fovy = fovy
    renderer = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    rng = np.random.default_rng(seed)
    views = []
    while len(views) < n:
        cam.lookat[:] = [rng.uniform(-.12, .12), rng.uniform(-.12, .02), rng.uniform(.04, .12)]
        cam.distance, cam.azimuth, cam.elevation = rng.uniform(.4, .6), rng.uniform(-140, -40), rng.uniform(-60, -15)
        renderer.update_scene(d, camera=cam)
        v = S.detect_view(f"free-{len(views):02d}.png", cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
        if len(v.tags) >= 3:
            views.append(v)
    renderer.close()
    return views


@needs_trial
def test_solver_recovers_a_known_lens_and_the_leaning_flaps():
    views = _free_camera_views(70.0)
    result = S.calibrate(views)
    assert abs(result["vertical_fov_degrees"] - 70.0) < 1.0, result["vertical_fov_degrees"]
    assert abs(result["cx"] - 319.5) < 8 and abs(result["cy"] - 239.5) < 8
    assert result["rms_reprojection_px"] < 0.5 and result["method"] == "box_tags"
    # the demo start leans the short flaps 0.1 rad (5.7 deg); rigid tags hardly move
    adj = result["tag_adjustments"]
    assert all(adj[t]["moved_mm"] < 2 for t in adj if G.TAGS[t].rigid)


@needs_trial
def test_solver_refuses_too_few_views_and_a_bad_fit(tmp_path):
    views = _free_camera_views(80.0, n=9, seed=3)
    with pytest.raises(S.CalibrationRefused, match="need 8"):
        S.calibrate(views[:5])
    noisy = [S.View(v.name, v.size, {t: c + np.random.default_rng(i).normal(0, 2.0, c.shape) for t, c in v.tags.items()})
             for i, v in enumerate(views)]
    with pytest.raises(S.CalibrationRefused, match="RMS reprojection error|frame"):
        S.calibrate(noisy)
    # CLI: a folder with too few frames exits non-zero and writes nothing
    folder = tmp_path / "frames"
    folder.mkdir()
    cv2.imwrite(str(folder / "blank.png"), np.full((480, 640, 3), 255, np.uint8))
    assert S.main(["--images", str(folder), "--out", str(tmp_path / "out.json")]) == 2
    assert not (tmp_path / "out.json").exists()


def test_output_json_has_the_checkerboard_tool_fields(tmp_path):
    """Same layout as tools/calibrate_camera_checkerboard.py plus method=box_tags (synthetic, no MuJoCo)."""
    k = np.array([[300., 0, 320], [0, 300., 240], [0, 0, 1]])
    rng = np.random.default_rng(0)
    views = []
    ids = [10, 26, 27, 22, 12, 14, 24, 25]
    for i in range(10):
        rvec = np.array([rng.uniform(1.1, 1.6), rng.uniform(-.4, .4), rng.uniform(-.3, .3)])
        cam_pos = np.array([rng.uniform(-.2, .2), rng.uniform(-.55, -.4), rng.uniform(.25, .4)])
        r = cv2.Rodrigues(rvec)[0]
        look = np.array([0, 0, .06]) - cam_pos
        z = look / np.linalg.norm(look)
        x = np.cross(z, [0, 0, 1.])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        r = np.array([x, y, z])                       # camera <- carton
        t = -r @ cam_pos
        tags = {}
        for tag_id in ids:
            c = G.tag_corners(tag_id)
            if np.dot(G.TAGS[tag_id].normal, cam_pos - c.mean(0)) <= 0:
                continue
            uv, _ = cv2.projectPoints(c, cv2.Rodrigues(r)[0], t, k, np.array([-.05, .01, 0, 0, 0]))
            uv = uv.reshape(4, 2)
            if (uv > 5).all() and (uv[:, 0] < 635).all() and (uv[:, 1] < 475).all():
                tags[tag_id] = uv + rng.normal(0, .1, uv.shape)
        views.append(S.View(f"syn-{i}.png", (640, 480), tags))
    result = S.calibrate(views)
    reference = {"fx", "fy", "cx", "cy", "width", "height", "distortion", "rms_reprojection_px", "photos_used",
                 "vertical_fov_degrees", "horizontal_fov_degrees"}
    assert reference <= set(result) and result["method"] == "box_tags" and len(result["distortion"]) == 5
    assert abs(result["fy"] - 300) < 3 and abs(result["distortion"][0] + .05) < .02
    json.dumps(S._clean(result))


# ------------------------------------------------------------------------------------------------ capture
@needs_trial
def test_planned_capture_through_the_owner_recovers_the_simulated_lens(tmp_path):
    """Plan from the live (fake) owner state, run every move through robot_move_joint_targets on the deployed owner
    code over the MuJoCo scene, grab wrist frames through robot_get_cameras, then calibrate (true fovy 80 deg)."""
    from carton import fold_policy_fakes as F
    from carton.fold_policy_runner import CAMERA_KEYS, ApiOwnerTransport
    from tools import capture_wrist_calibration as W
    rig = F.build_sim_rig(TRIAL, tmp_path / "rig", owner=True, api_cameras=True, height=480, width=640,
                          camera_keys=CAMERA_KEYS)
    model = rig.plant.m
    model.cam_fovy[model.camera("right_wrist").id] = 80.0
    rig.enable_all()
    snap = ApiOwnerTransport(rig.owner, clock=rig.clock).snapshot()
    planner = W.ViewPlanner(SCENE, "right", rig.arm_maps, snap.ticks, owner_ranges=snap.ranges)
    plan = planner.plan(12, seed=0)
    assert len(plan.poses) >= 10
    for p in plan.poses:
        for n, t in p.targets.items():
            lo, hi = snap.ranges[n]
            assert lo + 40 <= t <= hi - 40
    log = W.Capture(rig.owner, plan, tmp_path / "capture", clock=rig.clock, sleep=rig.sleep, log=lambda *_: None).run()
    assert log["aborted"] is None, log["aborted"]
    assert "robot_stop" not in log["tools_called"] and rig.owner.faults == []
    assert rig.owner.owner.state.get("stop_count") == snap.stop_count
    moves = rig.owner.moves()
    assert len(moves) == 2 * len(plan.poses) and all(m["arm"] == "right" and m.get("wait") for m in moves)
    assert all(not n.endswith("gripper") for m in moves for n in m["positions"])
    end = ApiOwnerTransport(rig.owner, clock=rig.clock).snapshot()
    assert all(abs(end.ticks[n] - snap.ticks[n]) <= W.SETTLED_TICKS for n in planner.names)
    result = S.run(tmp_path / "capture/frames", tmp_path / "right_wrist.json")
    assert abs(result["vertical_fov_degrees"] - 80.0) < 1.5, result["vertical_fov_degrees"]
    assert json.loads((tmp_path / "right_wrist.json").read_text())["method"] == "box_tags"


@needs_trial
def test_capture_aborts_on_stop_without_calling_robot_stop(tmp_path):
    from carton import fold_policy_fakes as F
    from carton.fold_policy_runner import CAMERA_KEYS, ApiOwnerTransport
    from tools import capture_wrist_calibration as W
    rig = F.build_sim_rig(TRIAL, tmp_path / "rig", owner=True, api_cameras=True, height=480, width=640,
                          camera_keys=CAMERA_KEYS)
    rig.enable_all()
    snap = ApiOwnerTransport(rig.owner, clock=rig.clock).snapshot()
    plan = W.ViewPlanner(SCENE, "left", rig.arm_maps, snap.ticks, owner_ranges=snap.ranges).plan(3, samples=400)
    stops = {"n": 0}

    def stop_hook():
        stops["n"] += 1
        return stops["n"] > 6                         # the operator presses Ctrl-C during the second pose
    log = W.Capture(rig.owner, plan, tmp_path / "capture", clock=rig.clock, sleep=rig.sleep, stop_requested=stop_hook,
                    log=lambda *_: None).run()
    assert log["aborted"] and "stop requested" in log["aborted"]
    assert "robot_stop" not in log["tools_called"]
    # a released arm (owner STOP) is refused before anything moves
    rig2 = F.build_sim_rig(TRIAL, tmp_path / "rig2", owner=True, api_cameras=True, height=480, width=640,
                           camera_keys=CAMERA_KEYS)
    log2 = W.Capture(rig2.owner, plan, tmp_path / "capture2", clock=rig2.clock, sleep=rig2.sleep,
                     log=lambda *_: None).run()
    assert "not enabled" in log2["aborted"] and rig2.owner.moves() == []


def test_dry_run_plans_offline_without_any_robot(tmp_path, capsys):
    if not SCENE.exists():
        pytest.skip("220 mm fold training scene not present")
    from tools import capture_wrist_calibration as W
    assert W.main(["--arm", "left", "--out", str(tmp_path / "plan"), "--scene", str(SCENE), "--poses", "8",
                   "--samples", "800"]) == 0
    plan = json.loads((tmp_path / "plan/plan.json").read_text())
    assert len(plan["poses"]) == 8 and "offline" in plan["settings"]["start_source"]
    assert "dry-run: nothing sent" in capsys.readouterr().out
    with pytest.raises(SystemExit, match="operator"):
        W.main(["--arm", "left", "--out", str(tmp_path / "x"), "--execute"])
