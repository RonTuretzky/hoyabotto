"""Adversarial evidence contract tests; synthetic fixtures are not task runs."""
import copy
import base64
import zlib

import mujoco
import numpy as np
import pytest

from planter.g4_assembly_score import (AssemblyThresholds, STAGE_ORDER, audit_model,
    audit_source_visuals, capture_contacts, capture_assembly_state,
    full_assembly_contract, score_assembly_episode, score_guide_transfer, validate_trace)


def free_model(extra='', actuator=''):
    return mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep=".002" gravity="0 0 0"/>
      <worldbody><body name="guide"><freejoint name="guide_free"/>
        <geom name="guide_collision" type="box" size=".01 .01 .01" mass=".1"/></body>
        <site name="guide_seat_target"/>{extra}</worldbody>{actuator}</mujoco>''')


def physics_trace(model, count=8):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rows = [capture_assembly_state(model, data, 'initial', 0)]
    for i in range(1, count):
        mujoco.mj_step(model, data)
        applied = capture_contacts(model, data)
        mujoco.mj_forward(model, data)
        rows.append(capture_assembly_state(model, data, 'guide_approach_above', i, step_contacts=applied))
    return rows


def test_coherent_native_trace_replays_every_step():
    model = free_model()
    result = validate_trace(model, physics_trace(model))
    assert result['passed']
    assert result['metrics']['replayed_transitions'] == 7


def test_scoring_accepts_a_single_pass_stream():
    model = free_model()
    rows = physics_trace(model)
    result = score_assembly_episode(iter(rows), model=model)
    assert result['trace_audit']['passed']
    assert result['trace_audit']['metrics']['samples'] == 8
    assert result['observed_stage_order'] == ['guide']


def test_contact_codec_preserves_every_float64_bit_and_replays_packed_rows():
    from planter.g4_contact_codec import pack_contact_lists, decode_contacts
    contact = dict(geom1=7,geom2=12,position_m=[-0.,np.nextafter(0.,1.),np.nextafter(1.,2.)],
                   frame=np.eye(3).tolist(),distance_m=-np.nextafter(.0001,1.),
                   wrench=[1.,-0.,np.nextafter(1.,0.),1e-250,-1e200,0.])
    raw = {'contacts':[contact], 'step_contacts':[copy.deepcopy(contact)]}
    decoded = decode_contacts(pack_contact_lists(raw))
    for key in raw:
        a,b = raw[key][0],decoded[key][0]
        assert a['geom1']==b['geom1'] and a['geom2']==b['geom2']
        for field in ('position_m','frame','distance_m','wrench'):
            assert np.asarray(a[field],dtype='<f8').tobytes()==np.asarray(b[field],dtype='<f8').tobytes()
    model = free_model()
    result = validate_trace(model, (pack_contact_lists(r) for r in physics_trace(model)))
    assert result['passed']


@pytest.mark.parametrize('corruption',['truncated','trailing','shape','nan','fractional_id','invalid_zlib','codec'])
def test_corrupt_contact_codec_is_an_explicit_invalid_trace(corruption):
    from planter.g4_contact_codec import pack_contact_lists
    model = free_model(); rows = [pack_contact_lists(r) for r in physics_trace(model)]
    payload = rows[2]['contacts']
    if corruption in ('nan','fractional_id'):
        values = np.zeros((1,21),dtype='<f8'); values[0,0] = np.nan if corruption=='nan' else .5
        payload['shape']=[1,21]
        payload['zlib_base64']=base64.b64encode(zlib.compress(values.tobytes())).decode()
    elif corruption=='shape':
        payload['shape']=[1,22]
    elif corruption=='codec':
        rows[2]['contact_codec']='fake'
    elif corruption=='invalid_zlib':
        payload['zlib_base64']=base64.b64encode(b'not zlib').decode()
    else:
        value=base64.b64decode(payload['zlib_base64'])
        value=value[:-1] if corruption=='truncated' else value+b'trailing'
        payload['zlib_base64']=base64.b64encode(value).decode()
    result=validate_trace(model, iter(rows))
    assert not result['passed']
    assert any(reason.startswith('invalid_sample_2') for reason in result['failure_reasons'])


def test_compact_guide_states_discard_parked_contacts_after_full_validation():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".002"/>
      <worldbody><geom name="floor" type="plane" size="1 1 .1"/>
      <body name="guide" pos="0 0 1"><freejoint/><geom type="box" size=".01 .01 .01"/></body>
      <body name="parked" pos=".1 0 .01"><freejoint/><geom type="box" size=".01 .01 .01" solref=".004 1"/></body>
      </worldbody></mujoco>''')
    rows = physics_trace(model, 15)
    assert any(r['contacts'] for r in rows)
    result = validate_trace(model, iter(rows))
    assert result['passed'], result['failure_reasons']
    assert result['metrics']['peak_contact_force_n'] > 0
    assert all(not r['contacts'] and not r['step_contacts'] for r in result['states'])
    index = next(i for i, r in enumerate(rows) if r['contacts'])
    rows[index]['contacts'][0]['wrench'][0] = 20.
    result = validate_trace(model, iter(rows))
    assert not result['passed']
    assert 'contact_force_limit' in result['failure_reasons']


def test_teleport_with_consistent_recorded_state_and_contacts_fails_replay():
    model = free_model()
    rows = physics_trace(model)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, np.array(rows[4]['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
    data.qpos[0] += .02
    mujoco.mj_forward(model, data)
    rows[4] = capture_assembly_state(model, data, rows[4]['phase'], 4)
    result = validate_trace(model, rows)
    assert not result['passed']
    assert 'physics_replay_mismatch_at_4' in result['failure_reasons']


@pytest.mark.parametrize('kind', ['dropped', 'nan', 'mutation', 'force', 'lying_qpos'])
def test_incomplete_nonfinite_or_modified_evidence_cannot_pass(kind):
    model = free_model(); rows = physics_trace(model)
    if kind == 'dropped':
        del rows[3]
    elif kind == 'nan':
        rows[3]['integration_state'][0] = float('nan')
    elif kind == 'mutation':
        rows[3]['mutation_events'] = ['guide reset']
    elif kind == 'force':
        data = mujoco.MjData(model)
        mujoco.mj_setState(model, data, np.array(rows[3]['integration_state']), mujoco.mjtState.mjSTATE_INTEGRATION)
        data.xfrc_applied[1, 2] = .001
        mujoco.mj_forward(model, data)
        rows[3] = capture_assembly_state(model, data, rows[3]['phase'], 3)
    else:
        rows[3]['qpos'][0] = .1
    assert not validate_trace(model, rows)['passed']


def test_actual_applied_step_contacts_cannot_be_omitted_or_fabricated():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".002" gravity="0 0 -9.81"/>
      <worldbody><geom type="plane" size="1 1 .1"/><body name="guide" pos="0 0 .01"><freejoint/>
      <geom type="box" size=".01 .01 .01" mass=".1" solref=".004 1"/></body></worldbody></mujoco>''')
    rows = physics_trace(model, 20)
    index = next(i for i, row in enumerate(rows) if row['step_contacts'])
    rows[index]['step_contacts'] = []
    result = validate_trace(model, rows)
    assert f'applied_contact_replay_mismatch_at_{index}' in result['failure_reasons']


def test_initial_overlap_and_distributed_contact_loads_are_explicit_failures():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom type="plane" size="1 1 .1"/><body name="guide" pos="0 0 .00999"><freejoint/>
      <geom type="box" size=".01 .01 .01" mass=".1"/></body></worldbody></mujoco>''')
    rows = physics_trace(model, 2)
    assert 'initial_intersection' in validate_trace(model, rows)['failure_reasons']
    assert len(rows[0]['contacts']) == 4
    for c in rows[0]['contacts']:
        c['wrench'] = [3., 0., 0., 0., 0., 0.]
    result = validate_trace(model, rows)
    assert result['metrics']['peak_contact_force_n'] < 8.
    assert result['metrics']['peak_body_pair_force_n'] == 12.
    assert 'body_pair_contact_force_limit' in result['failure_reasons']


def test_object_motor_and_weld_are_detected_from_compiled_model():
    model = free_model(actuator='<actuator><motor joint="guide_free" gear="1 0 0 0 0 0"/></actuator>')
    assert 'non_robot_or_unlimited_actuator' in audit_model(model)['failure_reasons']
    model = free_model(actuator='<equality><weld body1="guide"/></equality>')
    assert 'unreviewed_equality_constraint' in audit_model(model)['failure_reasons']


def guide_fixture():
    # Direct measurement-unit fixture. These poses are deliberately not claimed
    # to come from physics; full scorer would require transition replay.
    extra = '<body name="holder" pos="0 0 -.1"><freejoint/><geom name="holder_collision" type="box" size=".1 .1 .01"/></body>'
    for side in ('left', 'right'):
        extra += f'''<body name="{side}_gripper_link" pos="0 0 .2">
          <geom name="{side}_wrist_roll_follower_part" type="sphere" size=".002"/>
          <geom name="{side}_moving_jaw_part" type="sphere" size=".002"/></body>'''
    model = free_model(extra)
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    phases = [('approach_above', 5), ('approach', 5), ('acquire', 60), ('transfer', 60),
              ('hold', 60), ('place', 80), ('release', 20), ('withdraw', 20), ('released_hold', 260)]
    bid = model.body('guide').id
    def contact(other, normal, load=.1):
        n = np.asarray(normal, dtype=float); tangent = np.cross(n, [0, 1, 0]); tangent /= np.linalg.norm(tangent)
        frame = [n.tolist(), tangent.tolist(), np.cross(n, tangent).tolist()]
        return dict(geom1=model.geom(other).id, geom2=model.geom('guide_collision').id,
                    position_m=[0., 0., 0.], frame=frame, distance_m=0., wrench=[load, 0., 0., 0., 0., 0.])
    rows = []
    for phase, count in phases:
        for k in range(count):
            s = dict(time_s=len(rows)*.002, phase='guide_'+phase, xpos=data.xpos.copy(),
                     xmat=data.xmat.copy().reshape(-1, 3, 3), site_xpos=data.site_xpos.copy(),
                     site_xmat=data.site_xmat.copy().reshape(-1, 3, 3), qpos=data.qpos.copy(),
                     qvel=data.qvel.copy(), contacts=[], step_contacts=[])
            z = .03*(k+1)/count if phase == 'transfer' else .03 if phase == 'hold' else .03*max(0., 1-2*(k+1)/count) if phase == 'place' else 0.
            x = -.1*(1-(k+1)/count) if phase == 'transfer' else -.1 if phase in ('approach_above', 'approach', 'acquire') else 0.
            s['xpos'][bid] = [x, 0, z]
            for side, sign in [('left', -1), ('right', 1)]:
                s['xpos'][model.body(side+'_gripper_link').id] = [x+sign*.07, 0, z+.04]
            supported = phase in ('release', 'withdraw', 'released_hold') or (phase == 'place' and k >= count//2)
            gripping = phase in ('acquire', 'transfer', 'hold') or (phase == 'place' and not supported)
            if gripping:
                for side in ('left', 'right'):
                    s['contacts'] += [contact(side+'_wrist_roll_follower_part', [1, 0, 0]), contact(side+'_moving_jaw_part', [-1, 0, 0])]
            if supported:
                s['contacts'].append(contact('holder_collision', [0, 0, 1]))
            rows.append(s)
    return model, rows, contact


def test_independent_measured_guide_transfer_contract_has_a_positive_fixture():
    model, rows, _ = guide_fixture()
    result = score_guide_transfer(model, rows)
    assert result['passed'], result
    assert result['maximum_rigid_drift_m'] < 1e-12


def test_initialized_target_does_not_earn_robot_placement_credit():
    model, rows, _ = guide_fixture()
    rows[0]['xpos'][model.body('guide').id] = rows[0]['site_xpos'][model.site('guide_seat_target').id]
    assert 'guide_initialized_at_target_not_robot_placed' in score_guide_transfer(model, rows)['failure_reasons']


def test_late_acquisition_and_cross_phase_grip_loss_reject():
    model, rows, _ = guide_fixture()
    for row in rows:
        if row['phase'] == 'guide_acquire':
            row['contacts'] = []
    assert 'bimanual_opposed_grasp_not_established_before_transfer' in score_guide_transfer(model, rows)['failure_reasons']
    model, rows, _ = guide_fixture()
    transition = next(i for i, s in enumerate(rows) if s['phase'] == 'guide_transfer')
    for row in rows[transition-3:transition+3]:
        row['contacts'] = []
    assert 'opposed_grasp_lost_before_setdown' in score_guide_transfer(model, rows)['failure_reasons']


def test_side_only_rest_contact_does_not_terminate_retention():
    model, rows, contact = guide_fixture()
    for row in rows:
        if row['phase'] == 'guide_place' and row['contacts'] and row['contacts'][0]['geom1'] == model.geom('holder_collision').id:
            row['contacts'] = [contact('holder_collision', [1, 0, 0])]
    assert 'supported_setdown_not_observed_before_release' in score_guide_transfer(model, rows)['failure_reasons']


def test_rigid_rotation_slip_and_one_finger_release_support_reject():
    model, rows, contact = guide_fixture()
    row = next(s for s in rows if s['phase'] == 'guide_hold')
    angle = .2
    row['xmat'][model.body('guide').id] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    assert 'whole_guide_grasp_drift' in score_guide_transfer(model, rows)['failure_reasons']
    model, rows, contact = guide_fixture()
    row = next(s for s in rows if s['phase'] == 'guide_released_hold')
    row['contacts'].append(contact('left_moving_jaw_part', [0, 0, 1]))
    assert 'guide_not_released' in score_guide_transfer(model, rows)['failure_reasons']


def test_full_completion_cannot_be_asserted_with_stage_flags_or_phase_names():
    model = free_model(); rows = physics_trace(model, len(STAGE_ORDER))
    for row, phase in zip(rows, STAGE_ORDER):
        row['phase'] = phase
    result = score_assembly_episode(rows, model=model, spec={'full_success': True, 'stage_success': dict.fromkeys(STAGE_ORDER, True)})
    assert not result['full_assembly_success']
    assert not result['policy_export_eligible']
    assert 'unsupported_caller_scoring_override' in result['failure_reasons']
    assert 'deformable_paper_feed_fold_sheet_evaluators_not_implemented' in result['incomplete_reasons']
    assert 'strip0' in result['model_audit']['missing_bodies']


def test_human_prepared_guide_station_is_not_complete_prefix():
    model = free_model(); result = score_assembly_episode(physics_trace(model), model=model)
    assert 'missing_stage:trough' in result['incomplete_reasons']
    assert 'missing_stage:holder' in result['incomplete_reasons']
    assert not result['primitive_results']['guide_transfer']['source_visual_verified']


def test_source_path_is_checked_instead_of_trusting_a_hash_claim(tmp_path):
    path = tmp_path/'fake.stl'; path.write_text('a source claim is not geometry')
    result = audit_source_visuals(free_model(), {'guide': path})
    assert not result['guide']['passed']
    assert 'not the audited G4 source' in result['guide']['reason']


def test_contract_preserves_four_strips_rightmost_first_and_physical_boundary():
    contract = full_assembly_contract()
    assert contract['stage_order'][-5:] == ['fold3', 'fold2', 'fold1', 'fold0', 'sheet']
    assert [f'strip{i}' for i in range(4)] == [s for s in contract['stage_order'] if s.startswith('strip')]
    assert '>=28 mm' in contract['required_measurements']['guide_remove']
    assert not contract['physical_success']
