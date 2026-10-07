"""Bounds and evidence failures must stop this partial dynamics experiment."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from carton.folding_partial_short_probe import (
    _PanelStepAudit, _ShortStroke, _contact_reading, _bounded_contact_goals,
    probe_shorts_against_passive_majors,
)


@pytest.mark.parametrize('target', [-1., 10.1, float('nan'), float('inf')])
def test_target_cannot_expand_the_bounded_probe(target):
    with pytest.raises(ValueError, match='0 to 10'):
        probe_shorts_against_passive_majors(None, None, target_degrees=target)


def test_prior_passive_stage_is_required_before_plant_access():
    with pytest.raises(ValueError, match='Verified both-hands'):
        probe_shorts_against_passive_majors(None, SimpleNamespace())


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
