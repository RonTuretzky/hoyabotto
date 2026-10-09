"""Automatic head-camera pose (carton/head_pose.py, tools/auto_head_pose.py). Simulation and fakes only: no robot,
network or camera. The closed loop runs through the deployed robot-API code and hardware owner (--head scope) on a
fake bus (carton/fold_policy_fakes.build_head_rig) whose head has a deliberately wrong tick<->angle mapping."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

from carton import fold_policy_fakes as F
from carton import head_pose as hp
from carton.folding_station_measured import load_measurement
from tools import auto_head_pose as A

HACK = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output')
TRIAL = Path(os.environ.get('FOLD_POLICY_220_TRIAL', HACK / 'fold-demos/batch-220-01/trial-000'))
SCENE = TRIAL / 'run/scene.xml'
needs_scene = pytest.mark.skipif(not SCENE.exists(), reason='recorded 220 mm fold training scene not present')
NOMINAL = 360 / 4096


def maps_and_ticks(tmp_path):
    rig = F.build_head_rig(tmp_path / 'maps-rig', lambda m, mp: F.HeadOakSim(m, mp), F.HeadMapping(1900, 2074),
                           {'head_motor_1': 2074, 'head_motor_2': 2300})
    return rig.arm_maps, {n: rig.plant.ticks(n) for n in rig.plant.q}


def measure(sim, ticks, **config):
    rgb = sim.render(ticks)
    return hp.measure_head_pose(rgb, sim.intrinsics(), sim.maps, ticks, hp.HeadPoseConfig(**config))


def truth(sim, ticks):
    position, rotation = sim.camera_pose(ticks)
    tilt, pan = sim.mapping.angles(ticks)
    return position, tilt, pan


# ------------------------------------------------------------------------------------------------- kinematics
def test_tag_corner_order_and_measurement_pose(tmp_path):
    maps, ticks = maps_and_ticks(tmp_path)
    q = hp.arm_joint_radians(maps, ticks)
    pts = hp.gripper_tag_points(q, .22)
    for tag_id, corners in pts.items():
        assert np.isclose(np.linalg.norm(corners[1] - corners[0]), hp.GRIPPER_TAG_SIZE_M)
        normal = -np.cross(corners[1] - corners[0], corners[0] - corners[3])
        to_head = np.array([0, .05, .42]) - corners.mean(axis=0)
        assert normal @ to_head / np.linalg.norm(normal) / np.linalg.norm(to_head) > .95   # tags face the head
    centres = {i: c.mean(axis=0) for i, c in pts.items()}
    assert np.allclose(centres[4], [-.10, .28, .20], atol=.012) and np.allclose(centres[2], [.10, .28, .20], atol=.012)


@needs_scene
def test_gripper_tag_kinematics_match_training_scene():
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = {}
        for side in ('left', 'right'):
            q[side] = []
            for j in hp.ARM_JOINTS:
                jid = m.joint(f'{side}_{j}').id
                v = rng.uniform(*m.jnt_range[jid]) * .8
                d.qpos[m.jnt_qposadr[jid]] = v
                q[side].append(v)
        mujoco.mj_forward(m, d)
        origin = (d.body('left_base_link').xpos + d.body('right_base_link').xpos) / 2
        spacing = float(np.linalg.norm(d.body('right_base_link').xpos - d.body('left_base_link').xpos))
        for tag_id, mount in hp.gripper_tag_mounts(q, spacing).items():
            name = 'left_tag' if tag_id == 4 else 'right_tag'
            assert np.linalg.norm(mount[0] - (d.site(name + '_center').xpos - origin)) < .0005
            assert np.allclose(np.cross(mount[1], mount[2]), d.body(name).xmat.reshape(3, 3)[:, 2], atol=1e-6)
    assert np.allclose(hp.box_tag_points({'base_line_to_table_edge_m': .15, 'base_height_above_table_m': .12})[10].mean(0),
                       [0, .3015 - .1433, .055 - .12 + .0003 * 0], atol=.001)


def test_head_angles_round_trip():
    from carton.xlerobot_cameras import head_camera_spec
    for tilt, pan in ((58, 0), (49, 4), (30, -7), (10, 20)):
        t, p, r = hp.head_angles(head_camera_spec(tilt, pan)['rotation_cv'])
        assert abs(t - tilt) < 1e-6 and abs(p - pan) < 1e-6 and abs(r) < 1e-6
    training = json.loads(hp.STATION_PROFILE.read_text())['cameras']['front']
    t, p, _ = hp.head_angles(training['rotation_cv'])
    assert abs(t - 58) < 1e-4 and abs(p) < 1e-4


# ------------------------------------------------------------------------------------------------- solver
def _check(result, sim, ticks, deg=.7, mm=5.):
    position, tilt, pan = truth(sim, ticks)
    assert abs(result['tilt_deg'] - tilt) <= deg, (result['tilt_deg'], tilt)
    assert abs(result['pan_deg'] - pan) <= deg, (result['pan_deg'], pan)
    assert np.linalg.norm(np.asarray(result['position_m']) - position) * 1000 <= mm
    assert abs(result['roll_deg']) <= deg
    assert result['reprojection_rms_px'] < 1.


@pytest.mark.parametrize('tilt,pan,size,fovy', [(49, 4, (640, 480), 54.), (58, 0, (640, 360), 39.2),
                                                 (36, -3, (640, 360), 39.2)])
def test_solver_recovers_known_pose_synthetic(tmp_path, tilt, pan, size, fovy):
    maps, ticks = maps_and_ticks(tmp_path)
    mapping = F.HeadMapping(1900, 2074, 1.05 * NOMINAL, .93 * NOMINAL)
    sim = F.HeadOakSim(maps, mapping, width=size[0], height=size[1], fovy_deg=fovy, optical_offset=(.003, 0, .0104))
    ticks = {**ticks, **mapping.ticks_for(tilt, pan)}
    result = measure(sim, ticks)
    _check(result, sim, ticks)
    assert result['tags_used'] == [2, 4] and result['stream']['aspect_matches_training'] == (size == (640, 480))


@needs_scene
@pytest.mark.parametrize('tilt,pan,size,fovy', [(49, 4, (640, 480), 54.), (58, 0, (640, 480), 54.),
                                                 (36, -3, (640, 360), 39.2)])
def test_solver_recovers_known_pose_mujoco(tmp_path, tilt, pan, size, fovy):
    maps, ticks = maps_and_ticks(tmp_path)
    mapping = F.HeadMapping(1900, 2074)
    sim = F.HeadOakSim(maps, mapping, width=size[0], height=size[1], fovy_deg=fovy, scene_xml=SCENE)
    ticks = {**ticks, **mapping.ticks_for(tilt, pan)}
    try:
        _check(measure(sim, ticks), sim, ticks)
    finally:
        sim.close()


@needs_scene
def test_box_tags_with_carton_in_its_spot(tmp_path):
    """Training start pose (grippers parked low, their tags out of view) + carton in its nominal spot: only the box tags
    give the pose, and only with use_box_tags."""
    station = json.loads(hp.STATION_PROFILE.read_text())['station']
    demo = np.load(TRIAL / 'demo.npz')
    maps, _ = maps_and_ticks(tmp_path)
    q0 = demo['qpos'][0]
    ticks = {}
    for i, side in enumerate(('left', 'right')):
        ticks.update(zip((f'{side}_arm_{s}' for s in A.ARM_SUFFIXES),
                         (int(round(t)) for t in maps[side].rad_to_ticks(q0[6 * i:6 * i + 6]))))
    mapping = F.HeadMapping(1900, 2074)
    ticks.update(mapping.ticks_for(58, 0))
    sim = F.HeadOakSim(maps, mapping, width=640, height=480, fovy_deg=54., scene_xml=SCENE, carton='nominal',
                       station=station)
    try:
        rgb = sim.render(ticks)
        with pytest.raises(hp.HeadPoseRefused, match='usable tag'):
            hp.measure_head_pose(rgb, sim.intrinsics(), maps, ticks, hp.HeadPoseConfig())
        result = hp.measure_head_pose(rgb, sim.intrinsics(), maps, ticks,
                                      hp.HeadPoseConfig(use_box_tags=True, station=station))
    finally:
        sim.close()
    assert len(result['box_tags_used']) >= 4
    _check(result, sim, ticks)


def test_solver_refuses_one_tag_and_bad_fit(tmp_path):
    maps, ticks = maps_and_ticks(tmp_path)
    sim = F.HeadOakSim(maps, F.HeadMapping(1900, 2074))
    ticks = {**ticks, **sim.mapping.ticks_for(50, 0)}
    rgb = sim.render(ticks)
    found = hp.detect(rgb)
    k, dist, _, _ = hp.intrinsics_matrix(sim.intrinsics())
    q = hp.arm_joint_radians(maps, ticks)
    with pytest.raises(hp.HeadPoseRefused, match='need at least 2'):
        hp.estimate_head_pose({4: found[4]}, k, dist, q)
    bent = {i: dict(v) for i, v in found.items()}
    bent[2]['corners'] = (np.asarray(found[2]['corners']) + [[6, 0], [0, 0], [-6, 0], [0, 0]]).tolist()
    with pytest.raises(hp.HeadPoseRefused, match='RMS'):
        hp.estimate_head_pose(bent, k, dist, q)
    # One arm's shoulder pan off by 3 deg puts its tag where the other tag's geometry disagrees: refused.
    wrong = dict(ticks, right_arm_shoulder_pan=ticks['right_arm_shoulder_pan'] + 34)
    with pytest.raises(hp.HeadPoseRefused, match='RMS'):
        hp.estimate_head_pose(found, k, dist, hp.arm_joint_radians(maps, wrong))


def test_box_tags_cross_check_a_common_joint_zero_error(tmp_path):
    """A zero error shared by both arms (shoulder_lift +2 deg) moves both gripper tags consistently, so the gripper
    solution tilts by about as much with a good fit; the box tags (relative to the table) expose it."""
    station = json.loads(hp.STATION_PROFILE.read_text())['station']
    maps, ticks = maps_and_ticks(tmp_path)
    sim = F.HeadOakSim(maps, F.HeadMapping(1900, 2074), width=640, height=480, fovy_deg=54., carton='nominal',
                       station=station)
    ticks = {**ticks, **sim.mapping.ticks_for(55, 0)}
    rgb = sim.render(ticks)
    believed = {**ticks, 'left_arm_shoulder_lift': ticks['left_arm_shoulder_lift'] + 23,
                'right_arm_shoulder_lift': ticks['right_arm_shoulder_lift'] + 23}
    alone = hp.measure_head_pose(rgb, sim.intrinsics(), maps, believed, hp.HeadPoseConfig())
    assert 1. < alone['tilt_deg'] - 55 < 2.5 and alone['reprojection_rms_px'] < .5     # silently biased
    both = hp.measure_head_pose(rgb, sim.intrinsics(), maps, believed,
                                hp.HeadPoseConfig(use_box_tags=True, station=station, max_rms_px=5.))
    diff = both['subset_solutions']['gripper_minus_box']
    assert 1. < diff['tilt_deg'] < 2.5
    assert abs(both['subset_solutions']['box']['tilt_deg'] - 55) < .7


# ------------------------------------------------------------------------------------------------- closed loop
# The fake robot's head: tilt zero below the range, 5 % more degrees per tick than nominal; pan zero offset and its
# direction OPPOSITE to the twin's convention. Start: tilt 49, pan 4.
WRONG = F.HeadMapping(tilt_zero_tick=1900, pan_zero_tick=2120, tilt_deg_per_tick=1.05 * NOMINAL,
                      pan_deg_per_tick=-.93 * NOMINAL)


def head_rig(tmp_path, mapping=WRONG, start=(49, 4), scene=None, head=True, **sim):
    return F.build_head_rig(tmp_path / 'rig', lambda m, mp: F.HeadOakSim(m, mp, scene_xml=scene, **sim), mapping,
                            mapping.ticks_for(*start), head=head)


def run_loop(tmp_path, rig, execute=True, **config):
    tool = A.AutoHeadPose(rig.owner, rig.arm_maps, tmp_path / 'out', config=A.LoopConfig(**config),
                          clock=rig.clock, sleep=rig.sleep, operator='test')
    return tool.run(execute=execute, enable_head=True)


def calls(rig, name):
    return [a for n, a, _ in rig.owner.calls if n == name]


def _converged(tmp_path, rig, summary):
    assert summary['aborted'] is None, summary['aborted']
    assert summary['converged'] and summary['stop_reason'] == 'within_tolerance'
    tilt, pan = rig.sim.mapping.angles({n: rig.plant.ticks(n) for n in A.HEAD})
    assert abs(tilt - 58) <= 1.5 and abs(pan) <= 1.5, (tilt, pan)
    assert abs(summary['final']['tilt_deg'] - 58) <= 1 and abs(summary['final']['pan_deg']) <= 1
    moves = calls(rig, 'robot_move_head')
    assert 1 <= len(moves) <= 6 and not calls(rig, 'robot_stop')
    lo = {n: rig.calibration[n]['range_min'] + 40 for n in A.HEAD}
    hi = {n: rig.calibration[n]['range_max'] - 40 for n in A.HEAD}
    for c in summary['commands'][1:]:
        for n, t in c['args']['positions'].items():
            assert abs(t - c['from'][n]) <= 60 and lo[n] <= t <= hi[n]
    assert any('opposite way' in w for w in summary['warnings'])        # pan direction learned from the first move
    out = json.loads((tmp_path / 'out/auto-head-pose.json').read_text())
    assert out['camera_entry']['head_ticks'] == {n: rig.plant.ticks(n) for n in A.HEAD}
    station = load_measurement(tmp_path / 'out/station-measurement.json')
    assert station['cameras']['front']['intrinsics']['width'] == 640
    assert all((tmp_path / 'out' / m['frame']).exists() for m in out['measurements'])


def test_closed_loop_converges_despite_wrong_head_mapping(tmp_path):
    rig = head_rig(tmp_path)
    summary = run_loop(tmp_path, rig)
    _converged(tmp_path, rig, summary)
    assert any('not 4:3' in w for w in summary['warnings'])


@needs_scene
def test_closed_loop_converges_mujoco(tmp_path):
    rig = head_rig(tmp_path, scene=SCENE)
    try:
        _converged(tmp_path, rig, run_loop(tmp_path, rig))
    finally:
        rig.sim.close()


def test_dry_run_reads_only(tmp_path):
    rig = head_rig(tmp_path, width=640, height=480, fovy_deg=54.)
    before = {n: rig.plant.ticks(n) for n in rig.plant.q}
    summary = run_loop(tmp_path, rig, execute=False)
    assert summary['stop_reason'] == 'dry_run' and summary['aborted'] is None and summary['moves'] == 0
    assert {n for n, _, _ in rig.owner.calls} <= {'robot_get_execution', 'robot_get_cameras', 'robot_get_state'}
    assert {n: rig.plant.ticks(n) for n in rig.plant.q} == before
    m = summary['measurements'][0]
    assert abs(m['error_deg']['tilt'] - 9) < .7 and abs(m['error_deg']['pan'] + 4) < .7
    assert not any('4:3' in w for w in summary['warnings'])
    assert abs(m['vs_training']['tilt_error_deg'] + 9) < .7


def test_stops_at_the_head_range(tmp_path):
    """Level at 2211 ticks (the 9 Oct released reading) with the twin's direction: 58 deg would need ~2871, beyond the
    commandable 2625; the loop drives to the limit, reports it and moves no further."""
    mapping = F.HeadMapping(tilt_zero_tick=2211, pan_zero_tick=2074)
    rig = head_rig(tmp_path, mapping=mapping, start=(30, 2))
    summary = run_loop(tmp_path, rig)
    assert summary['aborted'] is None and not summary['converged']
    assert summary['stop_reason'] == 'head_range_limit'
    assert rig.plant.ticks('head_motor_2') == 2665 - 40
    assert summary['range_limit']['head_motor_2']['ticks_to_target_estimate'] > 2800
    assert abs(summary['final']['tilt_deg'] - (2625 - 2211) * NOMINAL) < 1.
    assert (tmp_path / 'out/station-measurement.json').exists()


def test_refuses_without_head_scope(tmp_path):
    rig = head_rig(tmp_path, head=False)
    summary = run_loop(tmp_path, rig)
    assert 'does not support head moves' in summary['aborted']
    assert not calls(rig, 'robot_move_head') and not calls(rig, 'robot_set_motor_enable') and not calls(rig, 'robot_stop')


def test_aborts_on_refused_move(tmp_path):
    rig = head_rig(tmp_path)
    real = rig.owner.call

    def call(name, args, request_id=None):
        if name == 'robot_move_head' and len(calls(rig, 'robot_move_head')) >= 1:
            rig.owner.calls.append((name, args, rig.clock()))
            return {'ok': False, 'result': {'error': 'Owner rejected: test'}}
        return real(name, args, request_id)
    rig.owner.call = call
    summary = run_loop(tmp_path, rig)
    assert 'robot_move_head refused' in summary['aborted'] and summary['moves'] == 2
    names = [n for n, _, _ in rig.owner.calls]
    assert names[-1] == 'robot_move_head' and 'robot_stop' not in names     # nothing after the refusal
    assert 'camera_entry' not in summary and not (tmp_path / 'out/station-measurement.json').exists()


def test_aborts_when_an_arm_moves_during_the_frame(tmp_path):
    rig = head_rig(tmp_path)
    publish = rig.owner.camera_publisher

    def nudging(owner, names):
        publish(owner, names)
        rig.plant.q['left_arm_elbow_flex'] += 6
    rig.owner.camera_publisher = nudging
    summary = run_loop(tmp_path, rig)
    assert 'moved during the frame' in summary['aborted'] and not calls(rig, 'robot_move_head')


def test_print_arm_pose_needs_no_robot(capsys):
    assert A.main(['--print-arm-pose']) == 0
    out = json.loads(capsys.readouterr().out)
    assert out['ticks']['right_arm_shoulder_lift'] == int(round(2047 + 34 * 4096 / 360))
