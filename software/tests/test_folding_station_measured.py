"""Measured fold-policy cameras/station (carton/folding_station_measured.py). Simulation only."""
import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from carton import folding_station_measured as fsm  # noqa: E402
from carton.folding_station import FoldingStation  # noqa: E402

SOFTWARE = Path(__file__).resolve().parents[1]
SIM_ROOT = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot')
OAK_K = dict(fx=504.89410400390625, fy=504.9738464355469, cx=314.7537841796875, cy=192.5926513671875,
             width=640, height=360)


def mini_scene(base_height=.12, setback=.15, spacing=.30):
    """A scene shaped like carton.folding_sim.build_scene's (bases, table, cameras, a marker, an arm mesh)."""
    edge = -.1515
    base_y = edge - setback
    words = fsm.words
    return f'''<mujoco model="mini">
  <visual><global offwidth="640" offheight="480"/></visual>
  <worldbody>
    <body name="left_base_link" pos="{words([-spacing/2, base_y, base_height])}" euler="0 0 1.570796327">
      <geom type="box" size=".02 .02 .02" contype="0" conaffinity="0" group="1" rgba=".22 .48 .75 1" name="lv"/>
      <geom type="box" size=".02 .02 .02" group="3" rgba="1 .8 .1 1" name="lc"/>
    </body>
    <body name="right_base_link" pos="{words([spacing/2, base_y, base_height])}" euler="0 0 1.570796327"/>
    <geom name="table" type="box" pos="{words([0, edge + .55, -.016])}" size=".55 .55 .016" rgba=".7 .66 .58 1"/>
    <body name="table_tag" pos="-.45 .65 .001">
      <geom name="table_tag_paper" type="box" size=".03 .03 .0001" contype="0" conaffinity="0" mass="0" rgba="1 1 1 1"/>
      <geom name="table_tag_0_0" type="box" size=".005 .005 .0001" contype="0" conaffinity="0" mass="0" rgba="0 0 0 1"/>
    </body>
    <geom name="target" type="sphere" size=".02" pos="0 .3 .05" rgba="1 0 0 1" contype="0" conaffinity="0"/>
    <camera name="overhead" pos="0 0 .85" xyaxes="1 0 0 0 1 0" fovy="48"/>
    <camera name="front" pos="0 -.3815 .57" xyaxes="1 0 0 0 .7885836665 .6149274761" fovy="48"/>
    <camera name="station" pos="-.4 -.45 .85" xyaxes="1 0 0 0 1 0" fovy="48"/>
  </worldbody>
</mujoco>'''


def write(tmp_path, text, name='scene.xml'):
    p = tmp_path / name
    p.write_text(text)
    return p


def test_nominal_measurement_reproduces_build_scene_cameras(tmp_path):
    root = ET.fromstring(mini_scene())
    before = {c.get('name'): (c.get('pos'), c.get('xyaxes')) for c in root.iter('camera')}
    fsm.restage_root(root, fsm.nominal_measurement())
    for c in root.iter('camera'):
        p0, x0 = before[c.get('name')]
        assert np.allclose(np.array(c.get('pos').split(), float), np.array(p0.split(), float), atol=1e-9)
        assert np.allclose(np.array(c.get('xyaxes').split(), float), np.array(x0.split(), float), atol=1e-9)


def test_profiles_validate_and_nominal_is_unmeasured():
    nominal = fsm.load_measurement(SOFTWARE / 'profiles/fold-station-nominal.json')
    example = fsm.load_measurement(SOFTWARE / 'profiles/fold-station-measurement.example.json')
    assert nominal['measured'] is False and example['measured'] is False
    assert nominal['station'] == fsm.nominal_measurement()['station']


def test_look_at_point_renders_at_image_centre(tmp_path):
    m = fsm.nominal_measurement()
    # Arm-base origin is (0, -.3015, .12); the red sphere sits at world (0, .3, .05).
    target = [0., .3 + .3015, .05 - .12]
    m['cameras'] = {'front': {'position_m': [.12, -.05, .38], 'look_at_m': target, 'fovy_deg': 30.},
                    'top': {'position_m': [0., .6015, .6], 'look_at_m': target, 'fovy_deg': 30.}}
    out = tmp_path / 'restaged.xml'
    fsm.restage_scene_xml(write(tmp_path, mini_scene()), out, m)
    model = mujoco.MjModel.from_xml_path(str(out))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 120, 160) as r:
        for cam in ('front', 'overhead'):
            r.update_scene(data, camera=cam)
            px = r.render()[60, 80]
            assert px[0] > 150 and px[1] < 80 and px[2] < 80, (cam, px)
            assert model.cam_fovy[model.camera(cam).id] == pytest.approx(30.)


def test_rotation_cv_and_look_at_agree():
    right, up = fsm.look_at_axes([0., -.08, .45], [0., .2865, -.02])
    rot = fsm.rotation_cv_from_axes(right, up)
    r2, u2 = fsm.axes_from_rotation_cv(rot)
    assert np.allclose(r2, right) and np.allclose(u2, up)
    # Optical axis points down toward the table at about 52 degrees.
    assert math.degrees(math.asin(-rot[2, 2])) == pytest.approx(52.06, abs=.05)
    with pytest.raises(ValueError):
        fsm.axes_from_rotation_cv(np.diag([1., 1., -1.]))


def test_roll_rotates_image_axes():
    r0, u0 = fsm.look_at_axes([0, 0, 1], [0, 1, 1])
    r1, u1 = fsm.look_at_axes([0, 0, 1], [0, 1, 1], roll_deg=90)
    assert np.allclose(r1, u0) and np.allclose(u1, -r0)


def test_policy_crop_from_real_oak_intrinsics():
    crop = fsm.policy_crop(OAK_K)
    # The OAK's 640x360 stream only supports a 36.7 degree vertical field at 4:3: narrower than the sim's 48.
    assert crop['fovy_deg'] == pytest.approx(36.68, abs=.01) and crop['fits']
    assert crop['crop_xyxy'][3] == pytest.approx(360.)
    assert not fsm.policy_crop(OAK_K, 48.)['fits']
    four_three = dict(fx=505, fy=505, cx=320, cy=240, width=640, height=480)
    exact = fsm.policy_crop(four_three, 48.)
    assert exact['fits'] and exact['fovy_deg'] == pytest.approx(48.)
    x0, y0, x1, y1 = exact['crop_xyxy']
    assert (x1 - x0) / (y1 - y0) == pytest.approx(4 / 3)


def test_intrinsics_set_fovy(tmp_path):
    m = fsm.nominal_measurement()
    m['cameras']['front'] = {'position_m': [0., -.01, .40], 'look_at_m': [0., .51, -.12], 'intrinsics': OAK_K}
    out = tmp_path / 's.xml'
    report = fsm.restage_scene_xml(write(tmp_path, mini_scene()), out, m)
    assert report['cameras']['front']['fovy_deg'] == pytest.approx(36.68, abs=.01)
    cam = next(c for c in ET.parse(out).getroot().iter('camera') if c.get('name') == 'front')
    assert float(cam.get('fovy')) == pytest.approx(36.68, abs=.01)
    station = next(c for c in ET.parse(out).getroot().iter('camera') if c.get('name') == 'station')
    assert station.get('pos') == '-.4 -.45 .85'  # the controller's camera is never touched


def test_station_mismatch_refuses_restage_and_reports_values(tmp_path):
    m = fsm.nominal_measurement()
    m['station']['base_height_above_table_m'] = .20
    with pytest.raises(ValueError, match='record new demonstrations'):
        fsm.restage_scene_xml(write(tmp_path, mini_scene()), tmp_path / 'o.xml', m)
    report = fsm.restage_scene_xml(write(tmp_path, mini_scene()), tmp_path / 'o.xml', m, allow_station_change=True)
    assert set(report['station_mismatch']) == {'base_height_above_table_m'}
    s = fsm.scene_station(ET.fromstring(mini_scene(.2, .1, .22)))
    assert s['base_height_above_table_m'] == pytest.approx(.2)
    assert s['base_line_to_table_edge_m'] == pytest.approx(.1)
    assert s['base_spacing_m'] == pytest.approx(.22)


@pytest.mark.parametrize('mutate, message', [
    (lambda m: m.update(schema='other'), 'schema'),
    (lambda m: m['station'].update(base_spacing_m=None), 'base_spacing_m'),
    (lambda m: m['station'].update(carton_near_wall_to_table_edge_m=.03), '10 mm'),
    (lambda m: m['cameras'].update(wrist={}), 'Unknown'),
    (lambda m: m['cameras']['top'].update(rotation_cv=np.eye(3).tolist()), 'exactly one'),
    (lambda m: m['cameras']['front'].pop('fovy_deg'), 'fovy_deg or intrinsics'),
    (lambda m: m['cameras']['front'].update(position_m=[0, 0]), 'position_m'),
    (lambda m: m.update(appearance={'table_size_m': [.2, .2]}), 'carton footprint'),
])
def test_invalid_measurements_are_refused(mutate, message):
    m = copy.deepcopy(fsm.nominal_measurement())
    mutate(m)
    with pytest.raises(ValueError, match=message):
        fsm.load_measurement(m)


def test_appearance_recolours_arms_resizes_table_and_hides_markers(tmp_path):
    m = fsm.nominal_measurement()
    m['appearance'] = {'arm_rgba': [0, 0, 0, 1], 'table_rgba': [.4, .3, .2, 1], 'table_size_m': [.6, .5],
                       'hide_markers': True}
    out = tmp_path / 'a.xml'
    report = fsm.restage_scene_xml(write(tmp_path, mini_scene()), out, m)
    geoms = {g.get('name'): g for g in ET.parse(out).getroot().iter('geom')}
    assert geoms['lv'].get('rgba') == '0 0 0 1'
    assert geoms['lc'].get('rgba') == '1 .8 .1 1'  # collision geometry untouched
    assert np.allclose(np.array(geoms['table'].get('size').split(), float), [.3, .25, .016])
    # Near table edge (and so the carton placement) unchanged.
    pos = np.array(geoms['table'].get('pos').split(), float)
    assert pos[1] - .25 == pytest.approx(-.1515)
    assert geoms['table_tag_paper'].get('rgba').endswith(' 0')
    assert report['appearance']['markers_hidden'] == ['table_tag']


def test_measured_station_class_enforces_cli_values_and_sets_spacing():
    cls = fsm.measured_station_class(FoldingStation, {'base_height_above_table_m': .2,
                                                       'base_line_to_table_edge_m': .12, 'base_spacing_m': .22})
    s = cls(.2, .12, .01, table_marker_xy=(-.5, .55))
    assert s.base_spacing == pytest.approx(.22) and isinstance(s, FoldingStation)
    assert s.base_y == pytest.approx(-.1515 - .12)
    with pytest.raises(ValueError, match='--base-height'):
        cls(.12, .15, .01)
    assert cls.historical_reference().reference_layout


def test_install_patches_and_uninstall_restores(tmp_path):
    import carton.folding_sim as folding_sim
    import carton.folding_station as folding_station
    original_station, original_build = folding_station.FoldingStation, folding_sim.build_scene
    m = fsm.nominal_measurement()
    m['station']['base_spacing_m'] = .26
    path = tmp_path / 'm.json'
    path.write_text(json.dumps(m))
    original_init = folding_sim.FoldingSimulation.__init__
    try:
        fsm.install(path)
        assert folding_station.FoldingStation is not original_station
        assert folding_sim.build_scene is not original_build
        assert folding_sim.FoldingSimulation.__init__ is not original_init
        assert folding_station.FoldingStation(.12, .15, .01).base_spacing == pytest.approx(.26)
    finally:
        fsm.uninstall()
    assert folding_station.FoldingStation is original_station and folding_sim.build_scene is original_build
    assert folding_sim.FoldingSimulation.__init__ is original_init


@pytest.mark.skipif(not (SIM_ROOT / 'scene-assets/arm-import.xml').exists(), reason='simulation assets absent')
def test_installed_build_scene_uses_measured_station_and_cameras(tmp_path):
    import carton.folding_sim as folding_sim
    import carton.folding_station as folding_station
    m = fsm.load_measurement(SOFTWARE / 'profiles/fold-station-measurement.example.json')
    m['station'].update(base_height_above_table_m=.16, base_spacing_m=.26)
    path = tmp_path / 'm.json'
    path.write_text(json.dumps(m))
    try:
        fsm.install(path)
        station = folding_station.FoldingStation(.16, .15, .01, table_marker_xy=(-.5, .55),
                                                 backup_table_marker_xy=(.45, .70))
        model = folding_sim.build_scene(SIM_ROOT, tmp_path / 'run', station=station)
    finally:
        fsm.uninstall()
    root = ET.parse(tmp_path / 'run/scene.xml').getroot()
    s = fsm.scene_station(root)
    assert s['base_height_above_table_m'] == pytest.approx(.16)
    assert s['base_spacing_m'] == pytest.approx(.26)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rep = fsm.camera_report(model, data, 'front', np.asarray(s['arm_base_origin_world_m']), 240, 320)
    assert rep['position_arm_base_m'] == pytest.approx([0., -.01, .40], abs=1e-6)
    assert rep['pitch_below_horizontal_deg'] == pytest.approx(45., abs=.01)
    assert json.loads((tmp_path / 'run/measured-station.json').read_text())['cameras']['front']['fovy_deg'] < 40
    # Appearance is left for restaging: the controller needs its table/carton markers while recording.
    paper = next(g for g in root.iter('geom') if g.get('name') == 'table_tag_paper')
    assert paper.get('rgba') == '1 1 1 1'
    assert np.allclose(np.array(next(g for g in root.iter('geom') if g.get('name') == 'table').get('size').split(),
                                float)[:2], [.55, .55])


# World positions of probe tags (arm_base origin is world (0, -.3015, .12)); the tags lie on the table.
PROBE_TAGS = {1: (0., .15), 20: (-.15, .45), 26: (.15, .45), 27: (.15, .15)}


@pytest.mark.parametrize('position, look_at, tag_ids', [
    ([.05, -.06, .42], [0., .45, -.12], (1,)),            # head-like oblique view, one tag
    ([.2, .3, .5], [0., .45, -.12], (1,)),                 # yawed oblique view, one tag
    ([0., .55, .70], [0., .60, -.12], (1, 20, 26, 27)),    # overhead view needs several tags
])
def test_camera_pose_from_tag_recovers_rendered_camera(tmp_path, position, look_at, tag_ids):
    pytest.importorskip('pupil_apriltags')
    import sys
    sys.path.insert(0, str(SOFTWARE / 'tools'))
    from camera_pose_from_tag import camera_in_base
    from carton.folding_sim import marker
    root = ET.fromstring(mini_scene())
    world = root.find('worldbody')
    for tag in tag_ids:
        marker(world, f'probe_tag_{tag}', tag, .10, [*PROBE_TAGS[tag], .001])
    ET.SubElement(world, 'light', pos='0 0 1.5', dir='0 0 -1', directional='true')
    m = fsm.nominal_measurement()
    m['cameras'] = {'front': {'position_m': position, 'look_at_m': look_at, 'fovy_deg': 50.}}
    fsm.restage_root(root, m)
    path = tmp_path / 'tag.xml'
    path.write_text(ET.tostring(root, encoding='unicode'))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 480, 640) as r:
        r.update_scene(data, camera='front')
        rgb = r.render()
    fy = 240 / math.tan(math.radians(25))
    k = dict(fx=fy, fy=fy, cx=320, cy=240, width=640, height=480)
    tags = {t: [PROBE_TAGS[t][0], PROBE_TAGS[t][1] + .3015, .0012 - .12] for t in tag_ids}
    pose = camera_in_base(rgb, k, tags, .10)
    assert sorted(pose['tags_used']) == sorted(tag_ids)
    assert np.allclose(pose['position_m'], position, atol=.01), pose
    right, up = fsm.look_at_axes(position, look_at)
    expected = fsm.rotation_cv_from_axes(right, up)
    angle = math.degrees(math.acos(min(1., (np.trace(np.asarray(pose['rotation_cv']).T @ expected) - 1) / 2)))
    assert angle < 1.
    assert pose['max_reprojection_px'] < 1.


def gripper_scene():
    """mini_scene plus a gripper body under each base, as in build_scene (SO101 gripper_link)."""
    root = ET.fromstring(mini_scene())
    for side in ('left', 'right'):
        base = next(b for b in root.iter('body') if b.get('name') == side + '_base_link')
        ET.SubElement(base, 'body', name=side + '_gripper_link', pos='0 .2 .1')
    return root


def test_wrist_cameras_are_created_inside_the_gripper_bodies():
    from carton.xlerobot_cameras import wrist_camera_spec
    root = gripper_scene()
    m = fsm.nominal_measurement()
    m['cameras'] = {'left_wrist': wrist_camera_spec('left'), 'right_wrist': wrist_camera_spec('right')}
    report = fsm.restage_root(root, m)
    for side in ('left', 'right'):
        body = next(b for b in root.iter('body') if b.get('name') == side + '_gripper_link')
        cams = [c for c in body.findall('camera') if c.get('name') == side + '_wrist']
        assert len(cams) == 1
        assert np.allclose(np.array(cams[0].get('pos').split(), float), [.0035, .068, -.0138])
        assert report['cameras'][side + '_wrist']['frame'] == side + '_gripper_link'
    # Restaging again moves rather than duplicates the camera.
    fsm.restage_root(root, m)
    assert sum(c.get('name') == 'left_wrist' for c in root.iter('camera')) == 1
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # The wrist camera looks along the gripper's -z (toward the jaw tips).
    cid = model.camera('left_wrist').id
    gid = model.body('left_gripper_link').id
    forward_world = -data.cam_xmat[cid].reshape(3, 3)[:, 2]
    jaw_minus_z = -data.xmat[gid].reshape(3, 3)[:, 2]
    assert forward_world @ jaw_minus_z > .999


def test_unknown_camera_frame_is_refused():
    m = fsm.nominal_measurement()
    m['cameras']['front']['frame'] = 'head_link'
    with pytest.raises(ValueError, match='frame must be one of'):
        fsm.load_measurement(m)


def test_xlerobot_head_camera_from_model_constants():
    from carton import xlerobot_cameras as xc
    spec = xc.head_camera_spec(0.)
    # Tilt 0: the model's head_camera_link (-0.127, 0.002, 1.1811) -> 37 mm ahead of the base origins and
    # 0.452 m above the SO101 mounting plane, looking horizontally forward.
    assert spec['position_m'] == pytest.approx([.002, .037, 1.1811 - .7291], abs=1e-4)
    pos, right, up, fovy = fsm.camera_pose(xc.head_camera_spec(58.), 'front')
    forward = np.cross(up, right)
    assert math.degrees(math.asin(-forward[2])) == pytest.approx(58., abs=1e-6)
    assert fovy == xc.OAK_D_LITE_FOVY_DEG
    with pytest.raises(ValueError, match='outside the model range'):
        xc.head_camera_spec(90.)
    m = xc.station_measurement(58.)
    fsm.load_measurement(m)
    assert m['station']['base_spacing_m'] == pytest.approx(.22)
    assert 'top' not in m['cameras']


@pytest.mark.skipif(not Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/'
                             'xlerobot/xlerobot.xml').exists(), reason='XLeRobot model absent')
@pytest.mark.parametrize('tilt, pan', [(0., 0.), (.9, .2), (-.5, -1.)])
def test_xlerobot_constants_match_the_model_file(tilt, pan):
    from carton.xlerobot_cameras import model_check
    check = model_check(tilt_rad=tilt, pan_rad=pan)
    assert max(check.values()) < 1e-6, check


@pytest.mark.skipif(not (SIM_ROOT / 'scene-assets/arm-import.xml').exists(), reason='simulation assets absent')
def test_installed_simulation_moves_parked_targets_with_the_bases(tmp_path):
    import carton.folding_sim as folding_sim
    import carton.folding_station as folding_station
    from carton.xlerobot_cameras import station_measurement
    path = tmp_path / 'm.json'
    path.write_text(json.dumps(station_measurement(58.)))
    try:
        fsm.install(path)
        station = folding_station.FoldingStation(.12, .15, .01, table_marker_xy=(-.5, .55),
                                                 backup_table_marker_xy=(.45, .70))
        sim = folding_sim.FoldingSimulation(SIM_ROOT, tmp_path / 'run', station=station,
                                            initial_arm_targets={'left': [-.20, -.18, .30], 'right': [.20, -.18, .30]})
    finally:
        fsm.uninstall()
    try:
        report = json.loads((tmp_path / 'run/measured-station.json').read_text())
        assert report['initial_arm_targets_world_m']['left'] == pytest.approx([-.16, -.18, .30])
        assert report['initial_arm_targets_world_m']['right'] == pytest.approx([.16, -.18, .30])
        tip = sim.data.site('left_tip').xpos
        assert tip[0] == pytest.approx(-.16, abs=.01)
        names = {sim.model.camera(i).name for i in range(sim.model.ncam)}
        assert {'front', 'left_wrist', 'right_wrist', 'station'} <= names
    finally:
        if sim.renderer:
            sim.renderer.close()
