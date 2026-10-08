"""Coupon evidence/geometry tests; these cannot certify carton retention."""
from types import SimpleNamespace
import gzip
import hashlib
import json

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from tools.diagnose_extensible_tape_bond import ReturnedContactTimeline, contact_evidence, load_at, make_model, run_case


def test_coupon_starts_unbonded_with_only_one_fixed_40_5mm_overlap():
    model, _, spec = make_model(.000025)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == model.nu == model.neq == 0
    assert model.joint('tape_free').type == mujoco.mjtJoint.mjJNT_FREE
    substrate = model.geom('coupon_substrate').id
    assert model.geom_bodyid[substrate] == 0  # Explicit laboratory fixture.
    rim = model.geom_pos[substrate, 0]+model.geom_size[substrate, 0]
    root = data.body('tape_0').xpos[0]
    assert rim-root == pytest.approx(.0405)
    assert spec.tape.length_m == .180 and spec.tape.segments == 36
    assert np.allclose(model.opt.gravity, [0, 0, -9.81])
    assert model.body_subtreemass[model.body('tape_0').id] == pytest.approx(.0003888)
    for i in range(36):
        gid = model.geom(f'tape_adhesive_{i}').id
        assert model.geom_adhesion[gid] == .020
        assert model.geom_friction[gid, 0] == .5
        assert model.geom_gap[gid] == .000020


def test_negative_control_removes_adhesion_without_changing_backing_or_compliance():
    normal, _, _ = make_model(.000025)
    control, _, _ = make_model(.000025, adhesion=False)
    assert np.array_equal(normal.body_mass, control.body_mass)
    assert np.array_equal(normal.jnt_stiffness, control.jnt_stiffness)
    assert np.array_equal(normal.geom_friction, control.geom_friction)
    assert np.count_nonzero(control.geom_adhesion) == 0


def test_short_local_coupon_keeps_cells_and_material_but_does_not_claim_full_span():
    model, _, spec = make_model(.000025, strip_length=.050)
    assert spec.tape.segments == 10 and spec.tape.segment_length_m == .005
    assert spec.tape.backing_young_pa == 200e6
    assert spec.tape.width_m == .024 and spec.tape.adhesion_per_contact_N == .020
    ids = [model.joint(f'tape_axial_{i}').id for i in range(1, 10)]
    assert np.sum(1/model.jnt_stiffness[ids]) == pytest.approx(.050/(200e6*.024*.0001))
    assert model.nu == model.neq == 0


def test_loading_requires_200ms_contact_formation_and_has_declared_mixed_direction():
    assert np.array_equal(load_at(.199), [0., 0., 0.])
    assert np.allclose(load_at(.25), [.3, 0., .03])
    assert np.allclose(load_at(.3), [.6, 0., .06])
    assert np.allclose(load_at(.4), [.6, 0., .06])


def test_short_coupon_bonds_from_free_settle_then_separates_without_bypassing_contact_gate(tmp_path):
    report = run_case(tmp_path/'local', timestep=.000025, strip_length=.050)
    assert report['initial_contact_count'] == 0
    assert report['returned_contact_timeline']['first_contact_observation_step_s'][0] > 0.
    assert report['settled_sample']['adhesive_contacts'] > 0
    assert report['outcome'] == 'no_returned_contacts_over_at_least_50ms_observation_span'
    assert report['error'] is None
    assert report['max_penetration_m'] < .000050
    assert report['max_wrong_face_tension_N'] == 0.
    assert report['free_box_retention_validated'] is False
    assert report['material_strength_validated'] is False
    assert report['energy_passivity_validated'] is False
    assert report['discrete_contact_sampling_semantics_verified'] is False
    path = tmp_path/'local'/report['actual_contact_step_evidence']['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == report['actual_contact_step_evidence']['sha256']
    with gzip.open(path, 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    assert len(rows) == report['actual_steps']
    assert rows[0]['step_started_s'] == 0.
    assert rows[-1]['step_ended_s'] == report['duration_s']
    assert rows[-1]['step'] == report['final_sample']['step']
    assert all(a['step_ended_s'] == b['step_started_s'] for a, b in zip(rows, rows[1:]))
    assert all('contact_solution_time_s' not in row for row in rows)
    assert not rows[-1]['contacts']


def test_timeline_preserves_two_step_transition_bracket_and_conservative_observation_span():
    timeline = ReturnedContactTimeline()
    timeline.update(0., .01, 1, .001)
    timeline.update(.01, .02, 0, 0.)
    timeline.update(.02, .03, 0, 0.)
    timeline.update(.03, .04, 0, 0.)
    timeline.update(.04, .05, 0, 0.)
    timeline.update(.05, .06, 0, 0.)
    assert not timeline.has_empty_observation_span(.05)
    timeline.update(.06, .07, 0, 0.)
    assert not timeline.has_empty_observation_span(.05)
    timeline.update(.07, .08, 0, 0.)
    assert timeline.has_empty_observation_span(.05)
    report = timeline.report()
    assert report['first_persistent_empty_observation_transition_bracket_s'] == [0., .02]
    assert report['first_contact_observation_step_s'] == [0., .01]
    timeline.update(.08, .09, 1, .001)
    assert not timeline.has_empty_observation_span(.05)
    assert timeline.report()['recontact_observation_count'] == 1


def test_timeline_refuses_replay_or_missing_step_coverage():
    timeline = ReturnedContactTimeline()
    timeline.update(0., .01, 1, .001)
    with pytest.raises(ValueError):
        timeline.update(0., .01, 1, .001)
    with pytest.raises(ValueError):
        timeline.update(.02, .03, 0, 0.)


@pytest.mark.parametrize('reverse', [False, True])
def test_native_contact_force_is_reported_on_tape_with_correct_geom_order(monkeypatch, reverse):
    model, _, _ = make_model(.000025)
    substrate = model.geom('coupon_substrate').id
    sticky = model.geom('tape_adhesive_0').id
    normal = np.array([0., 0., -1. if reverse else 1.])
    contact = SimpleNamespace(geom1=sticky if reverse else substrate,
                              geom2=substrate if reverse else sticky,
                              frame=np.r_[normal, [1., 0., 0.], [0., 1., 0.]],
                              pos=np.array([-.08, 0., .006]), dist=.000001)
    data = SimpleNamespace(contact=[contact], geom_xmat=np.tile(np.eye(3).reshape(9), (model.ngeom, 1)))
    monkeypatch.setattr(mujoco, 'mj_contactForce', lambda m, d, i, out: out.__setitem__(slice(None), [-.02, 0., 0., 0., 0., 0.]))
    evidence = contact_evidence(model, data, substrate, {sticky})
    assert evidence['interface_force_on_tape_N'] == pytest.approx([0., 0., -.02])
    assert evidence['tensile_contact_sum_N'] == pytest.approx(.02)
    assert evidence['wrong_face_tension_N'] == 0.
    assert evidence['adhesive_contacts'] == 1


@pytest.mark.parametrize('kwargs', [{'timestep': .0001}, {'timestep': True}, {'timestep': .000025, 'shear': float('nan')},
                                  {'timestep': .000025, 'opening': True}, {'timestep': .000025, 'adhesion': 1}])
def test_invalid_diagnostic_inputs_fail_before_creating_artifacts(tmp_path, kwargs):
    out = tmp_path/'invalid'
    with pytest.raises(ValueError):
        run_case(out, **kwargs)
    assert not out.exists()
