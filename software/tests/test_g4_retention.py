"""Concrete late-capture, lost-grasp, setdown and fixture-contact regressions."""
import copy

import numpy as np
import pytest

from planter.g4_retention import audit_retention, OBJECT_VERTICES_M


DT = .002


def contact(other, normal=(0, 0, 1), force=.12):
    return dict(geoms=[other, 'pusher_handle'], normal_force_n=force,
                normal_world_geom1_to_geom2=list(normal))


def trace():
    """Synthetic contract evidence only; deliberately not a full-v3 fixture."""
    rows = []
    time = 0.
    phases = [('approach_above', .02), ('lower_outside', .02), ('approach_handle', .02),
              ('close', .15), ('lift', .12), ('hold', .12), ('lower', .12),
              ('release', .04), ('withdraw', .04), ('released_hold', .04)]
    for phase, duration in phases:
        for i in range(round(duration/DT)):
            time += DT
            progress = (i+1)/round(duration/DT)
            z = .08*progress if phase == 'lift' else .08 if phase == 'hold' else 0.
            if phase == 'lower':
                z = .08*max(0., 1-2*progress)
            supported = phase not in ('lift', 'hold') and not (phase == 'lower' and progress < .5)
            gripped = phase in ('close', 'lift', 'hold') or (phase == 'lower' and not supported)
            contacts = [contact('staging_rest')] if supported else []
            if gripped:
                contacts += [contact('moving_jaw_part'), contact('wrist_roll_follower_part', (0, 0, -1))]
            rows.append(dict(time_s=time, phase=phase, object_pose=[0., 0., z, 1., 0., 0., 0.],
                             rest_clearance_mm=z*1000,
                             grasp_world_m=[.01, 0., z], gripper_world_rotation=np.eye(3).tolist(),
                             contacts=contacts, arm_environment_contacts=[]))
    return rows


def audit(rows):
    return audit_retention(rows, timestep_s=DT)


def remove_jaws(row):
    row['contacts'] = [c for c in row['contacts'] if c['geoms'][0] == 'staging_rest']


def test_clean_grip_and_support_transfer_pass_without_requiring_loaded_jaws_after_setdown():
    result = audit(trace())
    assert result['passed']
    assert result['setdown']['observed_before_release']
    assert result['pre_lift_grasp']['established']
    assert result['source']['sha256']


def test_late_edge_capture_does_not_pass_as_a_pre_lift_grasp():
    rows = trace()
    for row in rows:
        if row['phase'] == 'close':
            remove_jaws(row)
    result = audit(rows)
    assert not result['passed']
    assert 'opposed_loaded_grip_not_established_before_lift' in result['failure_reasons']


def test_short_bounded_contact_chatter_is_explicitly_tolerated():
    rows = trace()
    close = [r for r in rows if r['phase'] == 'close']
    for row in close[-3:-1]:
        remove_jaws(row)
    result = audit(rows)
    assert result['passed']
    assert result['pre_lift_grasp']['longest_unloaded_gap_seconds'] == .004
    assert result['pre_lift_grasp']['loaded_fraction'] == .96


def test_aerial_release_and_regrasp_fails_despite_good_final_hold():
    rows = trace()
    lift = [r for r in rows if r['phase'] == 'lift']
    for row in lift[10:16]:
        remove_jaws(row)
    result = audit(rows)
    assert 'opposed_loaded_grip_lost_before_setdown' in result['failure_reasons']
    assert result['retention_until_setdown']['longest_unloaded_gap_seconds'] == .012


def test_close_to_lift_gap_cannot_hide_at_phase_boundary():
    rows = trace()
    close = [r for r in rows if r['phase'] == 'close']
    lift = [r for r in rows if r['phase'] == 'lift']
    for row in close[-2:] + lift[:4]:
        remove_jaws(row)
    result = audit(rows)
    assert result['pre_lift_grasp']['established']
    assert 'opposed_loaded_grip_lost_before_setdown' in result['failure_reasons']
    assert result['retention_until_setdown']['longest_unloaded_gap_seconds'] == .012


def test_whole_tool_slip_after_established_grasp_is_not_reset_at_hold():
    rows = trace()
    for row in rows:
        if row['phase'] in ('lift', 'hold', 'lower'):
            row['object_pose'][0] += .006
    result = audit(rows)
    assert 'rigid_grasp_drift_exceeds_limit_before_setdown' in result['failure_reasons']
    assert result['retention_until_setdown']['maximum_rigid_relative_drift_mm'] > 5


def test_rigid_rotation_is_counted_even_with_a_stationary_origin():
    rows = trace()
    row = next(r for r in rows if r['phase'] == 'hold')
    row['object_pose'][3:] = [np.cos(.1), 0., 0., np.sin(.1)]
    assert 'rigid_grasp_drift_exceeds_limit_before_setdown' in audit(rows)['failure_reasons']


def test_drop_release_is_distinguished_from_supported_setdown():
    rows = trace()
    for row in rows:
        if row['phase'] == 'lower':
            row['contacts'] = [contact('moving_jaw_part'), contact('wrist_roll_follower_part', (0, 0, -1))]
    assert 'sustained_setdown_not_observed_before_release' in audit(rows)['failure_reasons']


def test_one_ground_contact_sample_does_not_end_grasp_retention():
    rows = trace()
    lower = [r for r in rows if r['phase'] == 'lower']
    lower[0]['contacts'].append(contact('staging_rest'))
    for row in lower[2:9]:
        remove_jaws(row)
    assert 'opposed_loaded_grip_lost_before_setdown' in audit(rows)['failure_reasons']


@pytest.mark.parametrize('normal,clearance', [((1, 0, 0), 0.), ((0, 0, -1), 0.), ((0, 0, 1), 2.)])
def test_rest_brushing_or_non_near_support_does_not_terminate_retention(normal, clearance):
    rows = trace()
    for row in rows:
        if row['phase'] == 'lower':
            row['contacts'] = [contact('staging_rest', normal)]
            row['rest_clearance_mm'] = clearance
    result = audit(rows)
    assert 'sustained_setdown_not_observed_before_release' in result['failure_reasons']
    assert 'opposed_loaded_grip_lost_before_setdown' in result['failure_reasons']


def test_reversed_geom_order_preserves_upward_support_sign():
    rows = trace()
    for row in rows:
        for c in row['contacts']:
            c['geoms'].reverse()
            c['normal_world_geom1_to_geom2'] = [-x for x in c['normal_world_geom1_to_geom2']]
    assert audit(rows)['passed']


def test_loaded_environment_contact_reports_force_duration_and_impulse():
    rows = trace()
    lift = [r for r in rows if r['phase'] == 'lift']
    for row in lift[:10]:
        row['arm_environment_contacts'] = [dict(geoms=['staging_rest', 'wrist_roll_follower_part'], normal_force_n=3.)]
    result = audit(rows)
    assert 'unintended_loaded_arm_environment_contact' in result['failure_reasons']
    evidence = result['arm_environment_contacts']
    assert evidence['maximum_point_normal_force_n'] == 3.
    assert evidence['loaded_duration_seconds'] == .02
    assert evidence['integrated_summed_normal_impulse_n_s'] == .06
    rows = trace()
    rows[0]['arm_environment_contacts'] = [dict(geoms=['staging_rest', 'wrist_roll_follower_part'], normal_force_n=1e-12)]
    assert audit(rows)['passed']


@pytest.mark.parametrize('mutation', ['missing_environment', 'missing_normal', 'nan_pose', 'missing_sample', 'duplicate_phase'])
def test_missing_or_invalid_new_evidence_never_silently_passes(mutation):
    rows = trace()
    if mutation == 'missing_environment':
        del rows[0]['arm_environment_contacts']
    elif mutation == 'missing_normal':
        del rows[0]['contacts'][0]['normal_world_geom1_to_geom2']
    elif mutation == 'nan_pose':
        rows[0]['object_pose'][0] = float('nan')
    elif mutation == 'missing_sample':
        del rows[20]
    elif mutation == 'duplicate_phase':
        next(r for r in rows if r['phase'] == 'hold')['phase'] = 'close'
    assert not audit(rows)['passed']


def test_retention_geometry_matches_the_audited_simulator_blocks():
    from planter.g4_sim import BLOCKS
    expected = np.asarray([np.asarray(p)+np.asarray(s)*[x,y,z] for _,p,s in BLOCKS
                           for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    assert np.array_equal(OBJECT_VERTICES_M, expected)
