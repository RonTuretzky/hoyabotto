"""Automated fold-station check (carton/station_check.py, tools/check_station.py). No robot, camera or motor.

Synthetic head-camera frames come from the MuJoCo fold training scene: the carton's freejoint places it with a
known error, the table geom moves with it to simulate a wrong table height, and the scene's head camera
('front', the model-derived OAK) renders the frame.
"""
import base64
import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from carton import station_check as sc  # noqa: E402

SOFTWARE = Path(__file__).resolve().parents[1]
SCENE = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/fold-demos/batch-220-01/trial-020/run/scene.xml')
ARM_BASE_WORLD = np.array([0., -.3015, .12])     # midway between left/right_base_link in the scene
OAK_169_FY = 504.97                               # real OAK 640x360 stream (tests/test_folding_station_measured.py)
SCENE_TAG_BODY = {26: 'box_tag_near_left', 10: 'box_tag', 27: 'box_tag_near_right', 21: 'box_tag_left',
                  28: 'box_tag_left_near', 22: 'box_tag_right', 25: 'box_tag_floor_center', 24: 'box_tag_floor',
                  11: 'short_left_tag', 12: 'short_right_tag', 13: 'long_far_tag', 14: 'long_near_tag'}

needs_scene = pytest.mark.skipif(not SCENE.exists(), reason='fold training scene absent')


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, SOFTWARE / 'tools' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module      # dataclasses look their module up while the class is created
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------- tag model (no scene needed)

def test_tag_model_matches_printed_layout():
    tags = _load_tool('make_fold_box_tags')
    printed = {t.tag_id: t for t in tags.TAGS}
    assert set(printed) == set(sc.RIGID_TAGS) | set(sc.FLAP_TAGS)
    for tag_id in sc.RIGID_TAGS:
        assert printed[tag_id].size == pytest.approx(sc.tag_size(tag_id) * 1000)
        corners = sc.tag_corners(tag_id)
        sides = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
        assert sides == pytest.approx([sc.WALL_TAG_M] * 4)
        face, centre, right, up = sc.RIGID_TAGS[tag_id]
        # Outward normal = right x up points away from the carton centre (or up for the floor).
        normal = np.cross(right, up)
        assert np.dot(normal, np.asarray(centre) - [0, 0, sc.H / 2]) > 0 or face == 'floor'
    # Face diagram (mm from the left / bottom edge, seen from outside) -> carton frame
    assert sc.RIGID_TAGS[26][1][0] * 1000 == pytest.approx(printed[26].h - sc.L * 500)
    assert sc.RIGID_TAGS[28][1][1] * 1000 == pytest.approx(sc.W * 500 - printed[28].h)
    assert sc.RIGID_TAGS[22][1][1] * 1000 == pytest.approx(printed[22].h - sc.W * 500)
    assert sc.RIGID_TAGS[24][1][0] * 1000 == pytest.approx(printed[24].h - sc.L * 500)


@needs_scene
def test_tag_model_matches_training_scene():
    mujoco = pytest.importorskip('mujoco')
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    a = m.jnt_qposadr[m.joint('carton_free').id]
    d.qpos[a:a + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    for tag_id, body in SCENE_TAG_BODY.items():
        b = d.body(body)
        surface = b.xpos + b.xmat.reshape(3, 3)[:, 2] * .0003     # printed cells sit 0.3 mm above the body
        model = sc.tag_corners(tag_id).mean(axis=0)
        assert np.abs(surface - model).max() < 2e-4, (tag_id, surface, model)
        assert np.allclose(b.xmat.reshape(3, 3)[:, 0], (sc.RIGID_TAGS | sc.FLAP_TAGS)[tag_id][2], atol=1e-6)


# ---------------------------------------------------------------- inputs

def test_load_camera_schema_variants(tmp_path):
    rot = np.eye(3).tolist()
    flat = {'schema': sc.CAMERA_SCHEMA, 'position_m': [0, .05, .42], 'rotation_cv': rot,
            'fx': 505, 'fy': 505, 'cx': 320, 'cy': 180, 'width': 640, 'height': 360, 'dist': [.1, 0, 0, 0, 0]}
    cam = sc.load_camera(flat)
    assert cam['measured'] and cam['intrinsics']['fx'] == 505 and cam['intrinsics']['dist'][0] == .1
    nested = {'position_m': [0, .05, .42], 'rotation_cv': rot,
              'intrinsics': {'fx': 505, 'fy': 505, 'cx': 320, 'cy': 180, 'width': 640, 'height': 360}}
    assert sc.load_camera(nested)['intrinsics']['cy'] == 180
    path = tmp_path / 'station.json'
    path.write_text(json.dumps({'schema': 'xlerobot-fold-station-measurement/1', 'cameras': {'front': nested}}))
    assert sc.load_camera(path)['measured']
    model = sc.model_camera()
    assert not model['measured'] and model['intrinsics'] is None and model['fovy_deg'] == 54
    with pytest.raises(ValueError):
        sc.load_camera({**flat, 'schema': 'something-else'})
    with pytest.raises(ValueError):
        sc.load_camera({**flat, 'rotation_cv': [[1, 0, 0], [0, 1, 0], [0, 0, -1]]})


def test_camera_matrix_scaling_and_aspect():
    intr = {'fx': 505., 'fy': 505., 'cx': 319.5, 'cy': 179.5, 'width': 640, 'height': 360, 'dist': []}
    k, dist, note = sc.camera_matrix(intr, (720, 1280))
    assert k[0, 0] == pytest.approx(1010) and k[0, 2] == pytest.approx(639.5) and 'scaled' in note
    with pytest.raises(ValueError, match='aspect'):
        sc.camera_matrix(intr, (480, 640))
    k, _, note = sc.camera_matrix(None, (480, 640), 54)
    assert k[1, 1] == pytest.approx(240 / math.tan(math.radians(27)))
    with pytest.raises(ValueError, match='intrinsics'):
        sc.camera_matrix(None, (360, 640), 54)    # the 54 deg model field of view is the 4:3 stream only


def test_frame_from_payload_reads_oak_rgb_and_depth():
    tool = _load_tool('check_station')
    rgb = cv2.imencode('.jpg', np.full((36, 64, 3), 128, np.uint8))[1].tobytes()
    depth = cv2.imencode('.png', np.full((36, 64), 500, np.uint16))[1].tobytes()
    rec = lambda raw, mime, cid: {'camera_id': cid, 'mime_type': mime, 'sha256': hashlib.sha256(raw).hexdigest(),
                                  'data_base64': base64.b64encode(raw).decode()}
    payload = {'ok': True, 'result': {'manifest': {'width': 64}},
               'images': [rec(rgb, 'image/jpeg', 'oak-x'), rec(depth, 'image/png', 'oak-x:depth')]}
    bgr, depth_mm, manifest, record = tool.frame_from_payload(payload, 'robot_get_depth')
    assert bgr.shape == (36, 64, 3) and depth_mm.dtype == np.uint16 and depth_mm[0, 0] == 500
    assert manifest == {'width': 64} and record['camera_id'] == 'oak-x'
    with pytest.raises(RuntimeError):
        tool.frame_from_payload({'ok': True, 'result': {'cameras': {}}, 'images': []}, 'robot_get_cameras')
    assert tool.READ_ONLY_TOOLS == ('robot_get_cameras', 'robot_get_depth')


# ---------------------------------------------------------------- rendered frames

class Station:
    """The training scene with a movable carton/table and the head camera; renders BGR frames."""

    def __init__(self):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(SCENE))
        self.d = mujoco.MjData(self.m)
        self.adr = self.m.jnt_qposadr[self.m.joint('carton_free').id]
        self.table = self.m.geom('table').id
        self.table_z = float(self.m.geom_pos[self.table][2])
        self.cam = self.m.camera('front').id
        self.bodies = {t: self.m.body(n).id for t, n in SCENE_TAG_BODY.items()}
        self.body_pos = {t: self.m.body_pos[b].copy() for t, b in self.bodies.items()}

    def render(self, dx_mm=0., yaw_deg=0., table_mm=0., near_mm=0., tilt_x_deg=0., size=(640, 480), fy=None,
               tag_moves=None, flap=None, depth=False):
        """Carton dx_mm right of target, yaw_deg CCW from above, near wall near_mm farther than 160 mm, carton and
        table table_mm higher than the training table. Returns (bgr, camera, K[, depth_mm])."""
        m, d, mj = self.m, self.d, self.mj
        w, h = size
        yaw = math.radians(yaw_deg)
        y = .160 + near_mm / 1000 + sc.L / 2 * abs(math.sin(yaw)) + sc.W / 2 * abs(math.cos(yaw))
        d.qpos[self.adr:self.adr + 3] = ARM_BASE_WORLD + [-.010 + dx_mm / 1000, y, -.120 + table_mm / 1000]
        q = np.zeros(4)
        mj.mju_euler2Quat(q, np.array([math.radians(tilt_x_deg), 0., yaw]), 'xyz')
        d.qpos[self.adr + 3:self.adr + 7] = q
        m.geom_pos[self.table][2] = self.table_z + table_mm / 1000
        for t, b in self.bodies.items():
            m.body_pos[b] = self.body_pos[t] + np.asarray((tag_moves or {}).get(t, (0, 0, 0)))
        hinge = m.jnt_qposadr[m.joint('long_near_hinge').id]
        d.qpos[hinge] = flap or 0.
        m.cam_fovy[self.cam] = math.degrees(2 * math.atan(h / 2 / fy)) if fy else 54.
        mj.mj_forward(m, d)
        renderer = mj.Renderer(m, h, w)
        try:
            renderer.update_scene(d, 'front')
            rgb = renderer.render()
            if depth:
                renderer.enable_depth_rendering()
                renderer.update_scene(d, 'front')
                depth_m = renderer.render()
        finally:
            renderer.close()
        f = h / 2 / math.tan(math.radians(m.cam_fovy[self.cam]) / 2)
        k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.]])
        mat = d.cam_xmat[self.cam].reshape(3, 3)
        camera = {'position_m': d.cam_xpos[self.cam] - ARM_BASE_WORLD,
                  'rotation_cv': np.column_stack((mat[:, 0], -mat[:, 1], -mat[:, 2])),
                  'intrinsics': None, 'fovy_deg': None, 'measured': True, 'source': 'rendered training scene'}
        out = (cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), camera, k)
        if depth:
            out += (np.where(depth_m < 5, depth_m * 1000, 0).astype(np.uint16),)
        return out


@pytest.fixture(scope='module')
def station():
    if not SCENE.exists():
        pytest.skip('fold training scene absent')
    pytest.importorskip('mujoco')
    pytest.importorskip('pupil_apriltags')
    return Station()


def _check(station, **kw):
    bgr, camera, k = station.render(**kw)
    return sc.check_frame(bgr, camera, k, np.zeros(5))


def _amount(instructions, pattern):
    hits = [re.search(pattern, s) for s in instructions]
    hits = [h for h in hits if h]
    assert len(hits) == 1, (pattern, instructions)
    return float(hits[0].group(1))


def test_training_placement_is_ok(station):
    report = _check(station)
    assert report['status'] == 'OK' and report['ok'] and report['instructions'] == ['OK']
    m = report['measured']
    assert m['table_gap_mm'] == pytest.approx(120, abs=1)
    assert m['carton_x_mm'] == pytest.approx(-10, abs=1)
    assert m['near_wall_y_mm'] == pytest.approx(160, abs=1)
    assert abs(m['yaw_deg']) < .3 and m['tilt_deg'] < .5
    assert report['geometry']['consistent'] and len(report['geometry']['faces']) == 2
    assert all(c['ok'] for c in report['checks'])


def test_recovers_carton_right_yaw_and_high_table(station):
    report = _check(station, dx_mm=12, yaw_deg=3, table_mm=7)
    m = report['measured']
    assert m['carton_x_mm'] == pytest.approx(-10 + 12, abs=2)
    assert m['yaw_deg'] == pytest.approx(3, abs=.5)
    assert m['table_gap_mm'] == pytest.approx(113, abs=2)
    assert m['near_wall_y_mm'] == pytest.approx(160, abs=2)
    assert report['status'] == 'ADJUST' and not report['ok']
    ins = report['instructions']
    assert _amount(ins, r'Lower the table (\d+) mm') == pytest.approx(7, abs=2)
    assert _amount(ins, r'Turn the carton ([\d.]+) degrees clockwise \(seen from above\)') == pytest.approx(3, abs=.5)
    assert _amount(ins, r"Move the carton (\d+) mm to the robot's left") == pytest.approx(12, abs=2)
    assert not any('toward the robot' in s or 'away from the robot' in s for s in ins)
    assert ins.index(next(s for s in ins if 'table' in s)) == 0      # table first, then the carton


def test_recovers_low_table_left_carton_far_and_ccw_on_16_9_stream(station):
    report = _check(station, dx_mm=-9, yaw_deg=-2.5, table_mm=-6, near_mm=8, size=(640, 360), fy=OAK_169_FY)
    m = report['measured']
    assert report['image_size'] == [640, 360]
    assert m['carton_x_mm'] == pytest.approx(-19, abs=2)
    assert m['yaw_deg'] == pytest.approx(-2.5, abs=.5)
    assert m['table_gap_mm'] == pytest.approx(126, abs=2)
    assert m['near_wall_y_mm'] == pytest.approx(168, abs=2)
    ins = report['instructions']
    assert _amount(ins, r'Raise the table (\d+) mm') == pytest.approx(6, abs=2)
    assert _amount(ins, r'Turn the carton ([\d.]+) degrees counter-clockwise') == pytest.approx(2.5, abs=.5)
    assert _amount(ins, r"Move the carton (\d+) mm to the robot's right") == pytest.approx(9, abs=2)
    assert _amount(ins, r'Move the carton (\d+) mm toward the robot') == pytest.approx(8, abs=2)
    # Within the policy's training range (x -25..+5 mm, yaw -4..4 deg): no out-of-range warning.
    assert not any('Outside the range' in w for w in report['warnings'])


def test_small_errors_within_tolerance_are_ok(station):
    report = _check(station, dx_mm=1.5, yaw_deg=1, table_mm=1.5)
    assert report['status'] == 'OK', report['instructions']


def test_outside_training_range_is_flagged(station):
    report = _check(station, dx_mm=20, yaw_deg=-5)
    assert report['status'] == 'ADJUST'
    assert any('Outside the range' in w and 'carton rotation' in w and 'carton left/right' in w
               for w in report['warnings'])


def test_table_not_level(station):
    # Near side up 2 deg: the far side is lower. (Tilting the other way turns the near wall away from the
    # steep head camera until its tags are too oblique to decode.)
    report = _check(station, tilt_x_deg=-2)
    level = next(c for c in report['checks'] if c['name'] == 'table level')
    assert not level['ok'] and level['value'] == pytest.approx(2, abs=.5)
    assert level['far_minus_near_mm'] == pytest.approx(-sc.W * 1000 * math.tan(math.radians(2)), abs=3)
    assert 'far side' in level['instruction'] and 'shim the far side up' in level['instruction']
    assert report['status'] == 'ADJUST' and level['instruction'] in report['instructions']


def test_misplaced_tag_reports_box_mismatch(station):
    report = _check(station, tag_moves={24: (.020, 0, 0)})     # floor tag 24 stuck 20 mm too far right
    assert report['status'] == 'MISMATCH' and not report['ok']
    assert not report['geometry']['consistent']
    assert '379 x 283 x 108 mm training carton' in report['instructions'][0]


def test_wrong_box_geometry_reports_mismatch(station):
    # A different box: the floor tags sit 20 mm nearer the near wall than on the training carton.
    report = _check(station, tag_moves={24: (0, -.020, 0), 25: (0, -.020, 0)})
    assert report['status'] == 'MISMATCH'


def test_near_flap_leaning_in(station):
    report = _check(station, flap=.8)      # near long flap turned 46 deg: hides the floor tags
    assert report['status'] == 'ADJUST'
    assert report['flaps'][14]['upright'] is False
    assert any('Stand the near long flap straight up' in s for s in report['instructions'])
    assert any('floor tags 25/24' in s for s in report['instructions'])
    # The near-wall tags alone still place the carton.
    assert report['measured']['table_gap_mm'] == pytest.approx(120, abs=2)


def test_floor_tags_only_still_measure_but_never_ok(station):
    # The 16:9 stream at the model head pose cuts the near wall off the bottom of the image at the training
    # placement: only the two floor tags remain.
    report = _check(station, size=(640, 360), fy=OAK_169_FY)
    assert report['tags']['rigid_used'] == [24, 25]
    m = report['measured']
    assert m['table_gap_mm'] == pytest.approx(120, abs=2) and m['carton_x_mm'] == pytest.approx(-10, abs=2)
    assert m['near_wall_y_mm'] == pytest.approx(160, abs=2) and abs(m['yaw_deg']) < .5
    level = next(c for c in report['checks'] if c['name'] == 'table level')
    assert level['checked'] is False and level['ok']
    assert report['status'] == 'ADJUST' and not report['ok']
    assert report['instructions'] == [next(s for s in report['instructions'] if 'near-wall tags 26/10/27' in s)]
    assert any('cannot be cross-checked' in w for w in report['warnings'])


def test_depth_plane_second_estimate(station):
    bgr, camera, k, depth_mm = station.render(table_mm=5, depth=True)
    report = sc.check_frame(bgr, camera, k, np.zeros(5), depth_mm=depth_mm)
    d = report['depth_table']
    if not d['ok']:
        pytest.skip(f'no table plane in this view: {d["reason"]}')
    # The RANSAC plane (10 mm inliers) merges the table with the carton's floor (3 mm up), which fills the view.
    assert d['gap_mm'] == pytest.approx(115, abs=5)
    assert not any('Depth plane' in w for w in report['warnings'])


# ---------------------------------------------------------------- CLI

def test_cli_image_with_camera_json(station, tmp_path, capsys):
    tool = _load_tool('check_station')
    bgr, camera, k = station.render(dx_mm=12, yaw_deg=3, table_mm=7)
    image = tmp_path / 'front.png'
    cv2.imwrite(str(image), bgr)
    cam_json = tmp_path / 'head-pose.json'
    cam_json.write_text(json.dumps({
        'schema': sc.CAMERA_SCHEMA, 'frame': 'arm_base', 'position_m': camera['position_m'].tolist(),
        'rotation_cv': camera['rotation_cv'].tolist(), 'fx': k[0, 0], 'fy': k[1, 1], 'cx': k[0, 2], 'cy': k[1, 2],
        'width': 640, 'height': 480, 'dist': []}))
    out_json = tmp_path / 'report.json'
    code = tool.main(['--image', str(image), '--camera-json', str(cam_json), '--json', str(out_json)])
    text = capsys.readouterr().out
    assert code == 1
    assert 'Lower the table 7 mm' in text and 'clockwise (seen from above)' in text and "robot's left" in text
    report = json.loads(out_json.read_text())
    assert report['schema'] == sc.SCHEMA and report['motor_writes'] == 0 and report['camera']['measured']


def test_cli_model_camera_fallback_is_flagged(station, tmp_path, capsys):
    tool = _load_tool('check_station')
    bgr, _, _ = station.render()
    image = tmp_path / 'front.png'
    cv2.imwrite(str(image), bgr)
    code = tool.main(['--image', str(image)])
    text = capsys.readouterr().out
    # The scene's head camera is the profile's model camera (54 deg VFOV, 4:3): the nominal carton is OK.
    assert code == 0 and 'Station check: OK' in text
    assert 'UNMEASURED FALLBACK' in text and 'not measured' in text
