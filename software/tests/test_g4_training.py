"""Independent task scoring and actual rigid collision representation tests."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
import xml.etree.ElementTree as E

from tools.train_g4_pusher import score_episode,qualification_reasons,FAULTS,VALIDATION_OFFSETS
from tools.audit_g4_assets import audit
from planter.g4_sim import G4Simulation,JOINTS


def completed_trace():
    rows=[];time=0.
    phases=['approach_above','lower_outside','approach_handle','close','lift','hold','lower','release','withdraw','released_hold']
    for phase in phases:
        duration=2.1 if phase=='hold' else 1.1
        for _ in range(round(duration/.02)):
            time+=.02;held=phase=='hold'
            contacts=([dict(geoms=['moving_jaw_part','pusher_handle'],normal_force_n=.1,
                            penetration_mm=0,normal_world_geom1_to_geom2=[0,0,1],position_world_m=[.38,0,.02]),
                       dict(geoms=['wrist_roll_follower_part','pusher_handle'],normal_force_n=.1,
                            penetration_mm=0,normal_world_geom1_to_geom2=[0,0,-1],position_world_m=[.38,0,.02])]
                      if held else [dict(geoms=['staging_rest','pusher_handle'],normal_force_n=.12,
                                         penetration_mm=0,normal_world_geom1_to_geom2=[0,0,1],position_world_m=[.38,0,-.04])])
            rows.append(dict(phase=phase,time_s=time,clearance_mm=80 if held else 20,
                             rest_clearance_mm=60 if held else 0,table_edge_margin_mm=100,
                             jaw_contact_both=held,grip_world_m=[.38,0,.02 if held else -.04],
                             grasp_world_m=[.38,0,.02 if held else .04],contacts=contacts,
                             gripper_world_rotation=np.eye(3).tolist(),object_pose=[.38,0,.02,1,0,0,0],
                             ctrl=[0]*6,encoder_radians=[0]*6,
                             object_penetration_mm=0,forbidden_penetration_mm=0,arm_object_normal_force_n=1 if held else 0))
    return rows


def score(rows,completed=True,**kwargs):
    return score_episode(rows,completed,timestep_s=.02,**kwargs)


def test_complete_contact_trace_is_required():
    rows=completed_trace()
    assert score(rows)['success']
    assert not score(rows,False)['success']
    assert not score([])['success']
    assert not score([r for r in rows if r['phase']!='close'])['success']
    assert not score([r for r in rows if r['phase']!='released_hold'])['success']


@pytest.mark.parametrize('field,value',[('jaw_contact_both',False),('rest_clearance_mm',0),
                                         ('object_penetration_mm',2),('rest_clearance_mm',float('nan'))])
def test_one_bad_physics_sample_invalidates_hold(field,value):
    rows=completed_trace();next(r for r in rows if r['phase']=='hold')[field]=value
    assert not score(rows)['success']


def test_a_commanded_lift_with_object_on_rest_fails():
    rows=completed_trace()
    for row in rows:
        if row['phase']=='hold':row.update(rest_clearance_mm=0,clearance_mm=20,jaw_contact_both=False)
    result=score(rows)
    assert not result['success']
    assert 'lift_or_two_jaw_retention_failed' in result['failure_reasons']


def test_hold_slip_and_unreleased_jaw_fail():
    rows=completed_trace();hold=[r for r in rows if r['phase']=='hold']
    hold[-1]['grip_world_m'][0]+=.010
    assert 'hold_slip_over_5mm' in score(rows)['failure_reasons']
    rows=completed_trace()
    next(r for r in rows if r['phase']=='released_hold')['contacts'][0]['geoms'][0]='moving_jaw_part'
    assert not score(rows)['success']


def test_repeated_timestamp_cannot_fill_hold_duration():
    rows=completed_trace();rows[9]['time_s']=rows[8]['time_s']
    assert not score(rows)['success']


def test_interrupted_or_reentered_hold_and_sparse_samples_fail():
    rows=completed_trace()
    hold=[r for r in rows if r['phase']=='hold'];hold[len(hold)//2]['phase']='lift'
    assert 'missing_or_out_of_order_phases' in score(rows)['failure_reasons']
    rows=completed_trace();del rows[len(rows)//2]
    assert 'missing_or_irregular_physics_samples' in score(rows)['failure_reasons']
    rows=completed_trace();rows[90]['phase']='settle'
    assert 'missing_or_out_of_order_phases' in score(rows)['failure_reasons']


@pytest.mark.parametrize('field',['arm_object_normal_force_n','table_edge_margin_mm','ctrl','object_pose'])
def test_previously_unchecked_nonfinite_fields_fail(field):
    rows=completed_trace();row=rows[0]
    if isinstance(row[field],list):row[field][0]=float('nan')
    else:row[field]=float('nan')
    assert not score(rows)['success']


def test_loaded_two_jaw_contacts_and_their_opposition_are_independent_evidence():
    rows=completed_trace();held=next(r for r in rows if r['phase']=='hold')
    held['contacts']=[]
    assert 'lift_or_two_jaw_retention_failed' in score(rows)['failure_reasons']
    rows=completed_trace();held=next(r for r in rows if r['phase']=='hold')
    held['contacts'][1]['normal_world_geom1_to_geom2']=[0,0,1]
    assert 'jaw_loads_not_opposing' in score(rows)['failure_reasons']
    # MuJoCo may reverse geom order; force-on-object direction must remain correct.
    held['contacts'][1]['geoms'].reverse()
    assert score(rows)['success']


def test_release_requires_real_support_and_no_other_robot_contact():
    rows=completed_trace();released=next(r for r in rows if r['phase']=='released_hold')
    released['contacts']=[]
    assert 'release_not_free_on_staging_rest' in score(rows)['failure_reasons']
    rows=completed_trace();released=next(r for r in rows if r['phase']=='released_hold')
    extra=copy.deepcopy(released['contacts'][0]);extra['geoms'][0]='elbow_link_part'
    released['contacts'].append(extra)
    assert 'release_not_free_on_staging_rest' in score(rows)['failure_reasons']


def test_raw_force_and_penetration_cannot_hide_behind_summary_fields():
    rows=completed_trace();held=next(r for r in rows if r['phase']=='hold')
    held['contacts'][0]['normal_force_n']=9
    assert 'simulation_force_gate' in score(rows)['failure_reasons']
    rows=completed_trace();rows[0]['contacts'][0]['penetration_mm']=2
    assert 'collision_gate' in score(rows)['failure_reasons']
    rows=completed_trace();rows[0]['contacts'][0]['normal_force_n']=float('nan')
    assert not score(rows)['success']


def test_legacy_evidence_is_limited_and_cannot_qualify():
    rows=completed_trace()
    for row in rows:
        del row['gripper_world_rotation']
        for contact in row['contacts']:
            del contact['normal_world_geom1_to_geom2'];del contact['position_world_m']
    assert 'missing_contact_geometry_evidence' in score(rows)['failure_reasons']
    report=score(rows,require_contact_geometry=False)
    assert report['success'] and not report['qualification_evidence_complete']
    assert report['hold_slip_frame']=='world_relative_legacy'


def test_relative_slip_is_measured_in_the_gripper_frame():
    rows=completed_trace();hold=[r for r in rows if r['phase']=='hold']
    for row in hold:row['grip_world_m'][0]+=.020
    hold[-1]['gripper_world_rotation']=[[-1,0,0],[0,-1,0],[0,0,1]]
    assert 'hold_slip_over_5mm' in score(rows)['failure_reasons']


def test_tool_rotation_cannot_hide_behind_a_stationary_grip_point():
    rows=completed_trace();released=[r for r in rows if r['phase']=='released_hold']
    released[-1]['object_pose'][3:]=[np.cos(.1),0,0,np.sin(.1)]
    assert 'released_object_did_not_settle' in score(rows)['failure_reasons']
    rows=completed_trace();hold=[r for r in rows if r['phase']=='hold']
    hold[-1]['object_pose'][3:]=[np.cos(.1),0,0,np.sin(.1)]
    assert 'hold_rigid_body_slip_over_5mm' in score(rows)['failure_reasons']


def test_policy_promotion_requires_the_full_frozen_evaluation_set():
    candidate=dict(success=True,eligible_for_success_demonstrations=True,qualification_evidence_complete=True,
                   clean_pickup_audit=dict(passed=True,evidence_complete=True),
                   correction_m=[0,-.004,.004],close_angle_rad=-.08,release_height_m=.008,vertical_withdrawal=True)
    validation=[dict(candidate,object_reset_offset_m=list(offset),seed=100+i,fault=None,
                     evaluation_role='fresh_confirmation') for i,offset in enumerate(VALIDATION_OFFSETS)]
    controls=[dict(candidate,success=False,fault=fault,failure_reasons=['expected_control_failure']) for fault in FAULTS]
    assert not qualification_reasons(candidate,validation,controls)
    assert qualification_reasons(candidate,validation[:2],controls)
    assert qualification_reasons(candidate,validation,controls[:1])
    validation[0]['evaluation_role']='development_regression'
    assert 'fresh_confirmation_not_established' in qualification_reasons(candidate,validation,controls)
    controls[-1]['close_angle_rad']=-.09
    assert 'controller_parameters_changed_during_evaluation' in qualification_reasons(candidate,validation,controls)


@pytest.mark.parametrize('field,value',[('grasp_axis_world',[0,1,0]),('withdrawal_distance_m',.025),
                                       ('disengage_drop_m',.008),('cartesian_step_m',.005),
                                       ('require_clear_approach',True)])
def test_path_or_fixture_access_changes_cannot_mix_into_confirmation(field,value):
    candidate=dict(success=True,eligible_for_success_demonstrations=True,qualification_evidence_complete=True,
                   clean_pickup_audit=dict(passed=True,evidence_complete=True),
                   correction_m=[0,-.004,.004],close_angle_rad=-.08,release_height_m=.008,vertical_withdrawal=True)
    validation=[dict(candidate,object_reset_offset_m=list(offset),seed=100+i,fault=None,
                     evaluation_role='fresh_confirmation') for i,offset in enumerate(VALIDATION_OFFSETS)]
    controls=[dict(candidate,success=False,fault=fault,failure_reasons=['expected_control_failure']) for fault in FAULTS]
    validation[0][field]=value
    assert 'controller_parameters_changed_during_evaluation' in qualification_reasons(candidate,validation,controls)


def test_curriculum_retains_g4_function_and_blocks_unvalidated_stages():
    curriculum=json.loads((Path(__file__).parents[1]/'planter/g4_curriculum.json').read_text())
    assert not curriculum['hardware_execution_released']
    assert not curriculum['full_task_success']
    stages={r['id']:r for r in curriculum['stages']}
    assert stages['S3']['status']=='blocked_on_material_and_manual_tests'
    assert any('28mm' in x for x in stages['S4']['done_when'])
    assert any('Paper tails, not plastic' in x for x in stages['P0']['done_when'])


def test_exact_g4_pusher_has_no_filled_void_collision_hull():
    cad=Path('/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4')
    if not cad.exists():pytest.skip('Local G4 source CAD not installed')
    result=audit(cad)
    assert result['passed']
    assert result['missing_volume_mm3']<.005
    assert result['extra_volume_mm3']<.005


def test_simulator_retains_real_joints_and_has_no_object_actuator(tmp_path):
    source=Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot')
    cad=Path('/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4')
    if not source.exists() or not cad.exists():pytest.skip('Local SO101/G4 simulation assets not installed')
    sim=G4Simulation(source,cad,tmp_path)
    root=E.parse(source/'scene-assets/arm-import.xml').getroot()
    declared={j.get('name'):np.fromstring(j.get('range'),sep=' ') for j in root.findall('.//joint')}
    assert sim.model.nu==6 and sim.model.neq==0 and sim.model.nmocap==0
    assert sim.model.joint('pusher_free').type[0]==0
    for name in JOINTS:
        assert np.allclose(sim.model.joint(name).range,declared[name])
    assert not sim.initial_intersections


@pytest.mark.parametrize('audit',[{},dict(passed=False,evidence_complete=True),dict(passed=True,evidence_complete=False)])
def test_policy_promotion_requires_independent_clean_grasp_evidence(audit):
    candidate=dict(success=True,eligible_for_success_demonstrations=True,qualification_evidence_complete=True,
                   clean_pickup_audit=dict(passed=True,evidence_complete=True))
    validation=[dict(candidate,object_reset_offset_m=list(offset),seed=100+i,fault=None,
                     evaluation_role='fresh_confirmation') for i,offset in enumerate(VALIDATION_OFFSETS)]
    controls=[dict(candidate,success=False,fault=fault,failure_reasons=['expected_control_failure']) for fault in FAULTS]
    assert not qualification_reasons(candidate,validation,controls)
    validation[0]['clean_pickup_audit']=audit
    assert 'heldout_evaluation_incomplete_or_failed' in qualification_reasons(candidate,validation,controls)
    candidate['clean_pickup_audit']=audit
    assert 'candidate_not_eligible' in qualification_reasons(candidate,validation,controls)
