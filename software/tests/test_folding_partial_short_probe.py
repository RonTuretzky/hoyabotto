"""Bounds and evidence failures must stop this partial dynamics experiment."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from carton.folding_partial_short_probe import (
    _PanelStepAudit, _ShortStroke, probe_shorts_against_passive_majors,
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
    stroke = _ShortStroke({'left': -15., 'right': -12.}, 10.)
    assert stroke.next_angles() == {'left': -14., 'right': -11.}
    stroke.observe({'left': -14.8, 'right': -11.2})
    assert stroke.next_angles() == {'left': -13.8, 'right': -10.2}


def test_one_stalled_short_stops_even_while_other_short_progresses():
    stroke = _ShortStroke({'left': -15., 'right': -15.}, 10.)
    for i in range(11):
        stroke.next_angles()
        stroke.observe({'left': -15., 'right': -14.+i})
    stroke.next_angles()
    with pytest.raises(ValueError, match='left short stalled'):
        stroke.observe({'left': -15., 'right': -3.})


def test_bounded_component_stops_near_target_without_full_fold():
    stroke = _ShortStroke({'left': 8.5, 'right': 9.5}, 10.)
    assert stroke.next_angles() == {'left': 9.5, 'right': 10.}
    stroke.observe({'left': 9.2, 'right': 10.2})
    assert stroke.next_angles() is None


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
