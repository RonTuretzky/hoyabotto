"""Real tiny MuJoCo solves; no model downloads, renderers or hardware access."""
import copy

import mujoco
import numpy as np
import pytest

from carton.folding_contact_audit import FORCE_ZERO_N, contact_classes, sample_applied_contacts, score_applied_contacts
from carton.folding_sim import FoldingSimulation


def forbidden(a, b):
    return FoldingSimulation.forbidden_contact(None, a, b)


def scene(*, robot='left_upper_arm_housing', obstacle='table', velocity=0., applied=-3., z=.009999):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep=".002" gravity="0 0 0" integrator="implicitfast"/>
      <worldbody>
        <geom name="{obstacle}" type="plane" size="1 1 .1"/>
        <body pos="0 0 {z}"><joint name="probe" type="slide" axis="0 0 1"/>
          <geom name="{robot}" type="sphere" size=".01" mass=".1" solref=".004 1"/>
        </body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[:] = velocity
    data.qfrc_applied[:] = applied
    return model, data


def step(model, data):
    started = float(data.time)
    mujoco.mj_step(model, data)
    return sample_applied_contacts(model, data, step_started_at=started, forbidden_contact=forbidden)


def score(samples):
    return score_applied_contacts(samples, expected_start_time=samples[0]['step_started_at'],
        expected_end_time=samples[-1]['step_ended_at'], forbidden_contact=forbidden)


def test_tiny_forbidden_penetration_with_real_solver_load_is_rejected():
    model, data = scene()
    sample = step(model, data)
    contact = sample['contacts'][0]
    assert -contact['distance_m']*1000 == pytest.approx(.001)  # 1000x below the old 1 mm gate.
    assert contact['wrench_contact_N_Nm'][0] > 2.5
    assert sample['refusal_reason'].startswith('Loaded forbidden')
    result = score([sample])
    assert not result['passed'] and result['coverage_complete']
    assert result['loaded_pairs'][0]['max_penetration_mm'] < 1.
    assert result['loaded_pairs'][0]['integrated_resultant_load_Ns'] > .005


def test_same_tiny_overlap_without_load_is_not_misreported_as_loaded_contact():
    model, data = scene(velocity=10., applied=0.)
    sample = step(model, data)
    contact = sample['contacts'][0]
    assert contact['distance_m'] < 0 and -contact['distance_m']*1000 < 1.
    assert contact['wrench_contact_N_Nm'] == [0.]*6
    assert sample['loaded_unintended_contacts'] == [] and sample['refusal_reason'] is None
    result = score([sample])
    assert result['passed'] and result['existing_penetration_and_task_gates_still_required']


def test_loaded_non_jaw_flap_contact_is_independent_of_existing_forbidden_policy():
    model, data = scene(obstacle='long_near_cardboard')
    assert not forbidden('long_near_cardboard', 'left_upper_arm_housing')
    sample = step(model, data)
    result = score([sample])
    assert sample['refusal_reason'].startswith('Loaded non_jaw_flap')
    assert not result['passed'] and result['loaded_pairs'][0]['category'] == 'non_jaw_flap'


@pytest.mark.parametrize('robot', ['left_moving_jaw_part_15', 'right_wrist_roll_follower_part_3'])
def test_named_jaw_flap_load_is_recorded_but_not_reclassified_as_unintended(robot):
    model, data = scene(obstacle='short_left_cardboard', robot=robot)
    sample = step(model, data)
    assert sample['contacts'][0]['classes'] == ['jaw_flap']
    assert sample['contacts'][0]['wrench_contact_N_Nm'][0] > 2.5
    assert score([sample])['passed']


def test_nonzero_force_at_zero_penetration_is_still_loaded():
    model, data = scene(z=.01)
    # A positive contact margin keeps this exact-touch constraint active.
    model.geom_margin[:] = .00001
    sample = step(model, data)
    assert sample['contacts'][0]['distance_m'] == pytest.approx(0.)
    assert sample['contacts'][0]['wrench_contact_N_Nm'][0] > 2.5
    assert not score([sample])['passed']


def test_sampler_reads_applied_solve_without_forward_or_any_world_mutation(monkeypatch):
    model, data = scene()
    start = float(data.time)
    mujoco.mj_step(model, data)
    before = {name: getattr(data, name).copy() for name in
              ('qpos', 'qvel', 'ctrl', 'qfrc_applied', 'xfrc_applied', 'qacc_warmstart', 'efc_force')}
    time = data.time
    def forbidden_refresh(*args, **kwargs):
        raise AssertionError('Never recompute contact forces for this applied-step audit')
    monkeypatch.setattr(mujoco, 'mj_forward', forbidden_refresh)
    monkeypatch.setattr(mujoco, 'mj_fwdPosition', forbidden_refresh)
    sample = sample_applied_contacts(model, data, step_started_at=start, forbidden_contact=forbidden)
    assert sample['applied_inputs']['qfrc_applied_nonzero'] == [[0, -3.]]
    assert data.time == time
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(data, name), value)
    # The force belongs to the solve that advanced the interval, not final qpos.
    assert data.qpos[0] < -1e-5
    assert sample['contacts'][0]['distance_m'] == pytest.approx(-1e-6)


def test_score_recomputes_classes_and_load_instead_of_trusting_cached_pass_flags():
    model, data = scene(obstacle='long_near_cardboard')
    sample = step(model, data)
    sample.update(refusal_reason=None, loaded_unintended_contacts=[], passed=True)
    sample['contacts'][0]['classes'] = ['jaw_flap']
    result = score([sample])
    assert not result['passed'] and result['loaded_pairs'][0]['category'] == 'non_jaw_flap'


def test_contact_force_changes_with_applied_load_despite_identical_start_positions():
    weak_model, weak = scene(applied=0.)
    strong_model, strong = scene(applied=-3.)
    np.testing.assert_array_equal(weak.qpos, strong.qpos)
    low, high = step(weak_model, weak), step(strong_model, strong)
    assert low['contacts'][0]['distance_m'] == high['contacts'][0]['distance_m']
    assert high['contacts'][0]['wrench_contact_N_Nm'][0] > 1000*low['contacts'][0]['wrench_contact_N_Nm'][0]
    assert low['applied_inputs'] != high['applied_inputs']


@pytest.mark.parametrize('change', ['gap', 'replay', 'missing_wrench', 'legacy_qpos', 'missing_inputs'])
def test_missing_or_reconstructed_evidence_never_counts_as_complete_contact_pass(change):
    model, data = scene(velocity=10, applied=0)
    samples = [step(model, data), step(model, data), step(model, data)]
    end = samples[-1]['step_ended_at']
    if change == 'gap': samples.pop(1)
    elif change == 'replay': samples[1] = copy.deepcopy(samples[0])
    elif change == 'missing_wrench': samples[0]['contacts'][0].pop('wrench_contact_N_Nm')
    elif change == 'legacy_qpos': samples[0] = {'time': .002, 'label': 'legacy replay', 'qpos': [0.]}
    elif change == 'missing_inputs': samples[0].pop('applied_inputs')
    result = score_applied_contacts(samples, expected_start_time=0., expected_end_time=end, forbidden_contact=forbidden)
    assert not result['passed'] and not result['coverage_complete']
    assert result['status'] == 'CONTACT_AUDIT_INCOMPLETE' and result['errors']


def test_no_samples_cannot_be_a_pass():
    result = score_applied_contacts([], expected_start_time=0., expected_end_time=1., forbidden_contact=forbidden)
    assert not result['passed'] and not result['coverage_complete']


def test_wrong_step_interval_and_rk4_are_refused():
    model, data = scene()
    with pytest.raises(ValueError, match='exactly one mj_step'):
        sample_applied_contacts(model, data, step_started_at=0, forbidden_contact=forbidden)
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
    mujoco.mj_step(model, data)
    with pytest.raises(ValueError, match='one-pass integrator'):
        sample_applied_contacts(model, data, step_started_at=0, forbidden_contact=forbidden)


def test_numerical_zero_is_not_a_contact_permission_threshold():
    model, data = scene()
    sample = step(model, data)
    sample['contacts'][0]['wrench_contact_N_Nm'] = [FORCE_ZERO_N/2, 0., 0., 0., 0., 0.]
    assert score([sample])['passed']
    sample['contacts'][0]['wrench_contact_N_Nm'][0] = FORCE_ZERO_N*2
    assert not score([sample])['passed']
    assert score([sample])['force_numerical_zero_N'] == FORCE_ZERO_N


def test_geometry_convention_is_symmetric_and_does_not_misclassify_flap_names():
    assert contact_classes('short_left_cardboard', 'table', forbidden) == []
    for a, b in [('left_wrist_housing', 'short_left_cardboard'), ('short_left_cardboard', 'left_wrist_housing')]:
        assert contact_classes(a, b, forbidden) == ['non_jaw_flap']


@pytest.mark.parametrize('excluded', [True, False])
def test_explicit_same_arm_exclusions_follow_actual_collision_model(excluded):
    # Adjacent links in the production model are explicitly excluded. Disable
    # default parent filtering only in this fixture to isolate that exclusion.
    contact = '<contact><exclude body1="upper" body2="forearm"/></contact>' if excluded else ''
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep=".002" integrator="implicitfast" gravity="0 0 0"><flag filterparent="disable"/></option>
      <worldbody><body name="upper"><freejoint/>
        <geom name="left_upper_link_geom" type="sphere" size=".02" mass=".1"/>
        <body name="forearm" pos=".039999 0 0"><joint axis="0 0 1"/>
          <geom name="left_forearm_link_geom" type="sphere" size=".02" mass=".1"/>
        </body>
      </body></worldbody>{contact}</mujoco>''')
    data = mujoco.MjData(model)
    sample = step(model, data)
    if excluded:
        assert data.ncon == 0 and sample['contacts'] == [] and score([sample])['passed']
    else:
        assert data.ncon > 0 and sample['contacts'][0]['classes'] == ['forbidden']


def test_nonfinite_contact_wrench_cannot_pass_independent_score():
    model, data = scene()
    sample = step(model, data)
    sample['contacts'][0]['wrench_contact_N_Nm'][0] = float('nan')
    result = score([sample])
    assert not result['passed'] and not result['coverage_complete']


def test_incomplete_applied_force_values_cannot_certify_a_contact_solve():
    model, data = scene(velocity=10, applied=0)
    sample = step(model, data)
    sample['applied_inputs']['qfrc_applied_nonzero'] = [[0, float('nan')]]
    result = score([sample])
    assert not result['passed'] and not result['coverage_complete']


def test_simulation_move_stops_on_loaded_non_jaw_contact_and_preserves_step_log(tmp_path):
    import gzip
    import json
    from types import SimpleNamespace
    from carton.folding_sim import FLAPS
    model, data = scene(obstacle='long_near_cardboard')
    sim = SimpleNamespace(model=model, data=data, arm_indices={}, events=[],
        stats={'max_bad_penetration_mm': 0.},
        applied_contact_path=tmp_path/'applied-contact-steps.jsonl.gz',
        forbidden_contact=forbidden,
        measure_carton_motion=lambda: {}, truth_angles=lambda: dict.fromkeys(FLAPS, 0.),
        step_diagnostic=lambda: None)
    event = FoldingSimulation.move(sim, {}, .1, capture=False)
    assert event['stopped_early'] and event['duration_s'] == pytest.approx(.002)
    assert event['max_robot_flap_penetration_mm'] == pytest.approx(.001)
    assert event['step_error'].startswith('Loaded non_jaw_flap')
    assert event['contact_audit']['coverage_complete'] and not event['contact_audit']['passed']
    with gzip.open(sim.applied_contact_path, 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    assert len(rows) == 1
    assert score(rows)['loaded_pairs'][0]['max_resultant_force_N'] > 2.5


@pytest.mark.parametrize('field,value', [
    pytest.param('ctrl', None, id='ctrl-null'),
    pytest.param('ctrl', {}, id='ctrl-object'),
    pytest.param('ctrl', (), id='ctrl-not-list'),
    pytest.param('ctrl', [True], id='ctrl-bool'),
    pytest.param('ctrl', [None], id='ctrl-null-value'),
    pytest.param('ctrl', ['0'], id='ctrl-string-value'),
    pytest.param('ctrl', [[1.]], id='ctrl-nested-value'),
    pytest.param('ctrl', [float('nan')], id='ctrl-nan'),
    pytest.param('ctrl', [float('inf')], id='ctrl-infinity'),
    pytest.param('qfrc_applied_nonzero', None, id='qfrc-null'),
    pytest.param('qfrc_applied_nonzero', True, id='qfrc-bool'),
    pytest.param('qfrc_applied_nonzero', [None], id='qfrc-null-row'),
    pytest.param('qfrc_applied_nonzero', [[0]], id='qfrc-missing-value'),
    pytest.param('qfrc_applied_nonzero', [[0, 1., 2.]], id='qfrc-extra-column'),
    pytest.param('qfrc_applied_nonzero', [[False, 1.]], id='qfrc-bool-index'),
    pytest.param('qfrc_applied_nonzero', [[0., 1.]], id='qfrc-float-index'),
    pytest.param('qfrc_applied_nonzero', [[-1, 1.]], id='qfrc-negative-index'),
    pytest.param('qfrc_applied_nonzero', [[0, True]], id='qfrc-bool-value'),
    pytest.param('qfrc_applied_nonzero', [[0, None]], id='qfrc-null-value'),
    pytest.param('qfrc_applied_nonzero', [[0, float('-inf')]], id='qfrc-infinity'),
    pytest.param('qfrc_applied_nonzero', [[0, 1.], [0, 2.]], id='qfrc-duplicate-index'),
    pytest.param('xfrc_applied_nonzero', None, id='xfrc-null'),
    pytest.param('xfrc_applied_nonzero', [[True, [0.]*6]], id='xfrc-bool-index'),
    pytest.param('xfrc_applied_nonzero', [[-1, [0.]*6]], id='xfrc-negative-index'),
    pytest.param('xfrc_applied_nonzero', [[.5, [0.]*6]], id='xfrc-float-index'),
    pytest.param('xfrc_applied_nonzero', [[0, [0.]*5]], id='xfrc-short-wrench'),
    pytest.param('xfrc_applied_nonzero', [[0, [0.]*7]], id='xfrc-long-wrench'),
    pytest.param('xfrc_applied_nonzero', [[0, (0.,)*6]], id='xfrc-wrench-not-list'),
    pytest.param('xfrc_applied_nonzero', [[0, [True, 0., 0., 0., 0., 0.]]], id='xfrc-bool-component'),
    pytest.param('xfrc_applied_nonzero', [[0, [0., 0., 0., 0., 0., float('nan')]]], id='xfrc-nan-component'),
    pytest.param('xfrc_applied_nonzero', [[0, [0.]*6], [0, [1.]*6]], id='xfrc-duplicate-index'),
    pytest.param('xfrc_applied_nonzero', [[0, None]], id='xfrc-null-wrench'),
    pytest.param('xfrc_applied_nonzero', [[0, 1.]], id='xfrc-scalar-wrench'),
])
def test_invalid_applied_input_contents_cannot_certify_unloaded_contact(field, value):
    model, data = scene(velocity=10., applied=0.)
    record = step(model, data)
    assert score([record])['passed']  # This one-step baseline is otherwise complete and clear.
    record['applied_inputs'][field] = value
    result = score([record])
    assert result['status'] == 'CONTACT_AUDIT_INCOMPLETE'
    assert not result['passed'] and not result['coverage_complete']
    assert result['observed_steps'] == 0 and result['errors']


def test_finite_controls_and_unique_signed_applied_forces_remain_valid():
    model, data = scene(velocity=10., applied=0.)
    record = step(model, data)
    record['applied_inputs'] = {
        'ctrl': [0, -.17, 1.5],
        'qfrc_applied_nonzero': [[0, -3.], [2, .25]],
        'xfrc_applied_nonzero': [[0, [0., -2., 3., -.5, 0., 1.]], [4, [1, 0, 0, 0, 0, -1]]],
    }
    result = score([record])
    assert result['passed'] and result['coverage_complete']
