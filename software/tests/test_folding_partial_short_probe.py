"""Bounds and evidence failures must stop this partial dynamics experiment."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from carton.folding_partial_short_probe import (
    _PanelStepAudit, _ShortStroke, _contact_reading, _bounded_contact_goals,
    _validated_jaw_vertex, _subdivide_reversed_joint_paths, _exact_waypoint_details,
    _entry_route_endpoint_drifts,
    probe_shorts_against_passive_majors,
)
from carton.folding_far_contact import mesh_contact_vertices
from carton.folding_sim import JOINTS


@pytest.mark.parametrize('target', [-1., 10.1, float('nan'), float('inf')])
def test_target_cannot_expand_the_bounded_probe(target):
    with pytest.raises(ValueError, match='0 to 10'):
        probe_shorts_against_passive_majors(None, None, target_degrees=target)


def test_prior_passive_stage_is_required_before_plant_access():
    with pytest.raises(ValueError, match='Verified both-hands'):
        probe_shorts_against_passive_majors(None, SimpleNamespace())


def test_unknown_contact_policy_refuses_before_plant_access():
    with pytest.raises(ValueError, match='contact policy'):
        probe_shorts_against_passive_majors(None, None, contact_policy='unbounded')


def test_unknown_approach_policy_refuses_before_plant_access():
    with pytest.raises(ValueError, match='approach policy'):
        probe_shorts_against_passive_majors(None, None, approach_policy='unchecked')


def test_missing_short_observation_stops_before_planning_and_cannot_be_retried():
    pose = np.eye(4).tolist()
    c = SimpleNamespace(
        partial_major_release=dict(fault=None,
            stage='both partial major angles passively retained for five seconds',
            both_hands_parked=True, both_majors_passively_retained=True,
            expected_near_degrees=40., expected_far_degrees=35., checks=[{'seq': 4}]),
        sense=lambda _: dict(seq=5, world_from_box=pose, angles={
            'long_near': {'degrees': 40.}, 'long_far': {'degrees': 35.},
            'short_right': {'degrees': -15.}}))
    with pytest.raises(ValueError, match='Fresh finite'):
        probe_shorts_against_passive_majors(None, c)
    assert c.partial_short_probe['fault']
    assert not c.partial_short_probe['full_task_complete']
    with pytest.raises(ValueError, match='already attempted'):
        probe_shorts_against_passive_majors(None, c)


def test_both_short_commands_follow_their_own_measured_progress():
    stroke = _ShortStroke({'left': -15., 'right': -12.}, 10., step_degrees=1.)
    assert stroke.next_angles() == {'left': -14., 'right': -11.}
    stroke.observe({'left': -14.8, 'right': -11.2})
    assert stroke.next_angles() == {'left': -13.8, 'right': -10.2}


def test_one_stalled_short_stops_even_while_other_short_progresses():
    stroke = _ShortStroke({'left': -15., 'right': -15.}, 10., step_degrees=1.)
    for i in range(11):
        stroke.next_angles()
        stroke.observe({'left': -15., 'right': -14.+i})
    stroke.next_angles()
    with pytest.raises(ValueError, match='left short stalled'):
        stroke.observe({'left': -15., 'right': -3.})


def test_bounded_component_stops_near_target_without_full_fold():
    stroke = _ShortStroke({'left': 8.5, 'right': 9.5}, 10., step_degrees=1.)
    assert stroke.next_angles() == {'left': 9.5, 'right': 10.}
    stroke.observe({'left': 9.2, 'right': 10.2})
    assert stroke.next_angles() is None


def test_quarter_degree_policy_scales_ineffective_command_window():
    stroke = _ShortStroke({'left': -15., 'right': -15.}, 10.)
    assert stroke.next_angles() == {'left': -14.75, 'right': -14.75}
    for _ in range(47):
        stroke.observe({'left': -15., 'right': -15.})
        stroke.next_angles()
    with pytest.raises(ValueError, match='48 bounded commands'):
        stroke.observe({'left': -15., 'right': -15.})


def test_quarter_degree_policy_accepts_progress_and_remains_finitely_bounded():
    stroke = _ShortStroke({'left': -15., 'right': -15.}, 10.)
    for index in range(48):
        stroke.next_angles()
        stroke.observe({'left': -15.+.1*(index+1), 'right': -15.+.1*(index+1)})
    for _ in range(320-48):
        stroke.next_angles()
    with pytest.raises(ValueError, match='320 bounded commands'):
        stroke.next_angles()


def source_reading():
    front = np.eye(4)
    front[0, 3] = .002
    row = dict(degrees=-15., source_camera='additional_camera', observed_seq=5)
    return dict(seq=5, world_from_box=np.eye(4).tolist(), angles={'short_left': row},
                additional_view=dict(status='accepted', seq=5, synchronized_simulation_time_s=2.,
                    primary=dict(camera='station', seq=5),
                    additional=dict(camera='additional_camera', seq=5,
                        world_from_box=front.tolist(), angles={'short_left': {'degrees': -15.}},
                        rgb_timestamp_s=2., depth_timestamp_s=2.)))


def test_contact_geometry_uses_angle_source_pose_without_modifying_primary():
    original = source_reading()
    coherent, provenance = _contact_reading(original, 'left')
    assert coherent['world_from_box'][0][3] == .002
    assert original['world_from_box'][0][3] == 0.
    assert provenance['camera'] == 'additional_camera'
    assert provenance['seq'] == original['seq']


@pytest.mark.parametrize('mutate', [
    lambda row: row['additional_view'].update(status='refused'),
    lambda row: row['additional_view']['additional'].update(seq=4),
    lambda row: row['additional_view']['additional'].update(depth_timestamp_s=1.9),
    lambda row: row['additional_view']['additional']['angles']['short_left'].update(degrees=-14.),
    lambda row: row['angles']['short_left'].update(source_camera='unavailable'),
    lambda row: row['angles']['short_left'].update(observed_seq=4),
])
def test_source_camera_mismatch_or_stale_geometry_cannot_supply_contact_target(mutate):
    reading = source_reading()
    mutate(reading)
    with pytest.raises(ValueError):
        _contact_reading(reading, 'left')


class LinearIK:
    def __init__(self, error=0.):
        self.error = error

    def point(self, q):
        return np.asarray(q[:3])

    def solve(self, target, seed):
        return np.r_[np.asarray(target)+[self.error, 0., 0.], [0., 0.]], self.error


def test_sensor_target_jump_is_substepped_from_actual_encoder_fk():
    q = np.zeros(10)
    sim = SimpleNamespace(data=SimpleNamespace(qpos=q),
                          arm_indices={'left': list(range(5)), 'right': list(range(5, 10))})
    targets = {'left': [.003, 0., 0.], 'right': [0., -.004, 0.]}
    goals, points, details = _bounded_contact_goals(sim, {a: LinearIK() for a in targets}, targets)
    assert np.linalg.norm(goals['left'][:3]) == pytest.approx(.0005)
    assert np.linalg.norm(goals['right'][:3]) == pytest.approx(.0005)
    assert details['left']['requested_distance_m'] == .003
    assert np.array_equal(q, np.zeros(10))


def test_ik_error_cannot_turn_a_bounded_contact_step_into_a_larger_move():
    sim = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(10)),
                          arm_indices={'left': list(range(5)), 'right': list(range(5, 10))})
    with pytest.raises(ValueError, match='CAD substep bound'):
        _bounded_contact_goals(sim, {a: LinearIK(.001) for a in ('left', 'right')},
                               {'left': [.003, 0., 0.], 'right': [.003, 0., 0.]})


def feedback_sim():
    names = [side+'_'+joint for side in ('left', 'right') for joint in JOINTS[:5]]
    model = SimpleNamespace(
        actuator=lambda name: SimpleNamespace(id=names.index(name)),
        joint=lambda _: SimpleNamespace(range=np.array([-1., 1.])),
        actuator_ctrlrange=np.tile([-1., 1.], (10, 1)))
    return SimpleNamespace(model=model,
        data=SimpleNamespace(qpos=np.zeros(10), ctrl=np.zeros(10)),
        arm_indices={'left': list(range(5)), 'right': list(range(5, 10))})


def test_setpoint_feedback_does_not_accumulate_constant_gravity_following_offset():
    # Synthetic static following error tests the controller arithmetic only.
    # It is neither a modified simulation actuator nor physical fold evidence.
    sim = feedback_sim()
    sim.data.qpos[[2, 7]] = -.001
    ik = {a: LinearIK() for a in ('left', 'right')}
    targets = {a: [0., 0., -.010] for a in ik}
    previous = sim.data.qpos.copy()
    for _ in range(25):
        goals, _, details = _bounded_contact_goals(sim, ik, targets,
                                                  contact_policy='setpoint_feedback_v3')
        for a in ik:
            ix = sim.arm_indices[a]
            assert np.linalg.norm(goals[a][:3]-sim.data.ctrl[ix][:3]) <= .000500001
            assert details[a]['actual_minus_setpoint_world'][2] == pytest.approx(-.001)
            sim.data.ctrl[ix] = goals[a]
            sim.data.qpos[ix] = goals[a] + [0., 0., -.001, 0., 0.]
            assert np.linalg.norm(sim.data.qpos[ix][:3]-previous[ix][:3]) <= .000500001
        previous = sim.data.qpos.copy()
    assert sim.data.qpos[[2, 7]] == pytest.approx([-.010, -.010])
    assert sim.data.ctrl[[2, 7]] == pytest.approx([-.009, -.009])


def test_feedback_hold_preserves_setpoint_instead_of_resetting_to_sagged_joints():
    sim = feedback_sim()
    sim.data.qpos[[2, 7]] = -.001
    goals, _, details = _bounded_contact_goals(sim,
        {a: LinearIK() for a in ('left', 'right')},
        {a: [0., 0., -.001] for a in ('left', 'right')}, contact_policy='setpoint_feedback_v3')
    for a in goals:
        assert goals[a] == pytest.approx(np.zeros(5))
        assert details[a]['commanded_fk_increment_m'] == 0.
        assert details[a]['actual_fk_step_m'] == pytest.approx(.001)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1.001, 1.001])
def test_invalid_current_setpoint_is_refused_without_clipping_or_plant_write(bad):
    sim = feedback_sim()
    sim.data.ctrl[0] = bad
    before = sim.data.qpos.copy()
    with pytest.raises(ValueError, match='original joint and control ranges'):
        _bounded_contact_goals(sim, {a: LinearIK() for a in ('left', 'right')},
                               {a: [0., 0., -.001] for a in ('left', 'right')},
                               contact_policy='setpoint_feedback_v3')
    assert np.array_equal(sim.data.qpos, before)
    assert sim.data.ctrl[0] == bad or np.isnan(sim.data.ctrl[0])


def test_control_range_remains_required_when_joint_range_is_wider():
    sim = feedback_sim()
    sim.model.actuator_ctrlrange[0] = [-.5, .5]
    sim.data.ctrl[0] = .6
    with pytest.raises(ValueError, match='original joint and control ranges'):
        _bounded_contact_goals(sim, {a: LinearIK() for a in ('left', 'right')},
                               {a: [0., 0., -.001] for a in ('left', 'right')},
                               contact_policy='setpoint_feedback_v3')


def test_feedback_still_refuses_ik_that_expands_the_command_increment():
    sim = feedback_sim()
    with pytest.raises(ValueError, match='CAD substep bound'):
        _bounded_contact_goals(sim, {a: LinearIK(.001) for a in ('left', 'right')},
                               {a: [.003, 0., 0.] for a in ('left', 'right')},
                               contact_policy='setpoint_feedback_v3')


def jaw_mesh_sim():
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <asset><mesh name="tetra" vertex="0 0 0 .01 0 0 0 .01 0 0 0 .01"/></asset>
      <worldbody><body name="left_gripper_link">
        <geom name="left_wrist_roll_follower_tip" type="mesh" mesh="tetra" pos="0 0 -.1"/>
        <geom name="left_wrist_roll_follower_proximal" type="mesh" mesh="tetra"/>
        <geom name="left_wrist_link_housing" type="mesh" mesh="tetra" pos=".1 0 0"/>
        <body name="left_moving_jaw_link" pos=".03 0 0"><joint type="hinge"/>
          <geom name="left_moving_jaw_finger" type="mesh" mesh="tetra"/>
        </body>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    return SimpleNamespace(model=model, data=data)


def test_proximal_jaw_validation_preserves_honest_distal_only_selector():
    sim = jaw_mesh_sim()
    declared = dict(body='left_gripper_link', local=[0., 0., 0.],
                    geometry='left_wrist_roll_follower_proximal')
    point = _validated_jaw_vertex(sim, 'left', declared)
    assert point['source_vertex_error_m'] < 1e-6
    assert point['geometry'] == declared['geometry']
    distal = mesh_contact_vertices(sim.model, sim.data, 'left')
    assert all(np.linalg.norm(vertex['local']) > .08 for vertex in distal)


@pytest.mark.parametrize('declared', [
    dict(body='left_gripper_link', local=[.1, 0., 0.], geometry='left_wrist_link_housing'),
    dict(body='left_gripper_link', local=[.2, 0., 0.]),
    dict(body='right_gripper_link', local=[0., 0., 0.]),
    dict(body='left_gripper_link', local=[float('nan'), 0., 0.]),
])
def test_nonjaw_nonvertex_wrong_side_or_nonfinite_contact_refuses(declared):
    with pytest.raises(ValueError):
        _validated_jaw_vertex(jaw_mesh_sim(), 'left', declared)


def test_moving_jaw_vertex_cannot_be_frozen_to_same_side_gripper_body():
    sim = jaw_mesh_sim()
    with pytest.raises(ValueError, match='absent from original permitted'):
        _validated_jaw_vertex(sim, 'left', dict(body='left_gripper_link',
            local=[.03, 0., 0.], geometry='left_moving_jaw_finger'))
    valid = _validated_jaw_vertex(sim, 'left', dict(body='left_moving_jaw_link',
        local=[0., 0., 0.], geometry='left_moving_jaw_finger'))
    assert valid['body'] == 'left_moving_jaw_link'


def test_exact_reverse_subdivision_preserves_joint_edges_without_inward_ik():
    class NoSolveIK(LinearIK):
        def solve(self, target, seed):
            raise AssertionError('Exact inward path must not re-solve point IK')
    paths = {'left': [np.zeros(5), np.array([.003, -.001, 0., .2, 0.])],
             'right': [np.zeros(5), np.array([0., .002, 0., 0., -.1])]}
    ik = {a: NoSolveIK() for a in paths}
    waypoints = _subdivide_reversed_joint_paths(paths, ik)
    for a in paths:
        assert np.array_equal(waypoints[0][a], paths[a][0])
        assert np.array_equal(waypoints[-1][a], paths[a][-1])
    for start, end in zip(waypoints, waypoints[1:]):
        for a in paths:
            assert np.linalg.norm(ik[a].point(end[a])-ik[a].point(start[a])) <= .000501
            # All joint coordinates remain on the original checked edge.
            mask = paths[a][-1] != 0
            fractions = end[a][mask]/paths[a][-1][mask]
            assert np.allclose(fractions, fractions[0])


def test_unequal_reverse_paths_cannot_silently_drop_one_arms_waypoint():
    with pytest.raises(ValueError, match='Equal finite paired'):
        _subdivide_reversed_joint_paths({'left': [np.zeros(5), np.ones(5)], 'right': [np.zeros(5)]}, {})


def test_exact_waypoint_rejects_jump_from_real_current_setpoint_without_resolving():
    sim = feedback_sim()
    ik = {a: LinearIK() for a in ('left', 'right')}
    goals = {a: np.array([.001, 0., 0., 0., 0.]) for a in ik}
    with pytest.raises(ValueError, match='Exact normal waypoint exceeds'):
        _exact_waypoint_details(sim, ik, goals, {a: [.001, 0., 0.] for a in ik})
    assert np.array_equal(sim.data.ctrl, np.zeros(10))


def test_exact_waypoint_bound_is_setpoint_increment_not_claimed_physical_motion():
    sim = feedback_sim()
    sim.data.qpos[[2, 7]] = -.001
    ik = {a: LinearIK() for a in ('left', 'right')}
    goals = {a: np.array([0., 0., .0005, 0., 0.]) for a in ik}
    _, details = _exact_waypoint_details(sim, ik, goals, {a: [0., 0., .005] for a in ik})
    for a in ik:
        assert details[a]['commanded_fk_increment_m'] == pytest.approx(.0005)
        assert details[a]['actual_fk_step_m'] == pytest.approx(.0015)


@pytest.mark.parametrize('target', [[.025, .025, 0.], [float('nan'), 0., 0.]])
def test_fresh_endpoint_drift_or_missing_geometry_invalidates_frozen_route(target):
    entry = {a: [0., 0., 0.] for a in ('left', 'right')}
    with pytest.raises(ValueError):
        _entry_route_endpoint_drifts(dict(left=target, right=[0., 0., 0.]), entry)


def test_current_geometry_monitor_does_not_modify_entry_route():
    entry = {a: [0., 0., 0.] for a in ('left', 'right')}
    assert _entry_route_endpoint_drifts(dict(left=[.001, 0., 0.], right=[0., 0., 0.]), entry)['left'] == .001
    assert entry['left'] == [0., 0., 0.]


def contact_sim(tmp_path, *, overlap, prior_fault=None):
    # Two physical panels with one free body exercise the actual solver contact
    # arrays. No pose replay is used to infer forces or an executed fold.
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="0.002"/>
      <worldbody><geom name="long_near_cardboard" type="box" size=".1 .1 .01"/>
      <body pos="0 0 {0.02-overlap}"><freejoint/>
        <geom name="short_left_cardboard" type="box" size=".1 .1 .01" mass=".02"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    original = lambda: prior_fault
    return SimpleNamespace(model=model, data=data, out=tmp_path, events=[],
                           step_diagnostic=original), original


def test_actual_panel_overlap_refuses_and_preserves_audit(tmp_path):
    sim, original = contact_sim(tmp_path, overlap=.0015)
    report = {}
    with _PanelStepAudit(sim, report) as audit:
        mujoco.mj_step(sim.model, sim.data)
        assert 'Panel/panel penetration' in sim.step_diagnostic()
        assert audit.coverage_complete()
    assert sim.step_diagnostic is original
    assert report['panel_panel_audit']['max_panel_panel_penetration_mm'] > 1.
    assert report['panel_panel_audit']['complete_coverage']


def test_added_panel_audit_never_masks_an_existing_fault(tmp_path):
    sim, original = contact_sim(tmp_path, overlap=0., prior_fault='original fault')
    with pytest.raises(RuntimeError, match='stop'):
        with _PanelStepAudit(sim, {}) as audit:
            mujoco.mj_step(sim.model, sim.data)
            assert sim.step_diagnostic() == 'original fault'
            raise RuntimeError('stop')
    assert sim.step_diagnostic is original


def test_missing_actual_step_coverage_cannot_be_called_complete(tmp_path):
    sim, _ = contact_sim(tmp_path, overlap=0.)
    report = {}
    with _PanelStepAudit(sim, report) as audit:
        mujoco.mj_step(sim.model, sim.data)
        mujoco.mj_step(sim.model, sim.data)
        assert 'Incomplete' in sim.step_diagnostic()
        assert not audit.coverage_complete()
    assert not report['panel_panel_audit']['complete_coverage']
