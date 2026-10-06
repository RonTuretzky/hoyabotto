import numpy as np
import pytest

from carton.bimanual import JOINTS, UNITS, fold_reach_screen, validate_action_chunk, molmo_checkpoint_contract
from carton.geometry import Box, Stance


def test_bare_grippers_do_not_inherit_paddle_reach():
    r=fold_reach_screen(Box(),Stance(setback=.06,height=0,paddle=.15))
    far=next(p for p in r['phases'] if p['phase']=='long_far')
    assert far['reach_margin_m']<-.02
    assert not r['all_folding_contacts_within_spherical_reach']
    assert not r['uses_paddle'] and not r['motion_ready'] and not r['contact_simulated']


def test_closer_layout_is_only_a_reach_screen_not_a_collision_certificate():
    r=fold_reach_screen(Box(),Stance(setback=.02,height=0))
    assert r['all_folding_contacts_within_spherical_reach']
    assert not r['collision_checked'] and not r['motion_ready']
    assert {p['folding_arm'] for p in r['phases']}=={'left','right'}
    assert all(p['supporting_role'] for p in r['phases'])


@pytest.mark.parametrize('shape',[(30,6),(30,14),(360,),(1,30,12),(0,12)])
def test_wrong_embodiment_cannot_be_reshaped_into_two_arms(shape):
    with pytest.raises(ValueError):validate_action_chunk(np.zeros(shape),joint_order=JOINTS,units=UNITS)


def test_paired_actions_require_explicit_side_order_and_units():
    a=np.zeros((30,12));a[:,5]=1
    r=validate_action_chunk(a,joint_order=JOINTS,units=UNITS)
    assert r['actions'][0][5]==1 and r['actions'][0][11]==0 and not r['motion_ready']
    with pytest.raises(ValueError):validate_action_chunk(a,joint_order=JOINTS[6:]+JOINTS[:6],units=UNITS)
    with pytest.raises(ValueError):validate_action_chunk(a,joint_order=JOINTS,units=['raw_ticks']*12)
    a[0,11]=100
    with pytest.raises(ValueError):validate_action_chunk(a,joint_order=JOINTS,units=UNITS)
    a[0,11]=float('nan')
    with pytest.raises(ValueError):validate_action_chunk(a,joint_order=JOINTS,units=UNITS)


@pytest.mark.parametrize('dim,match',[(6,False),(14,False),(12,True)])
def test_normalizer_dimensions_not_padded_model_size_determine_embodiment(dim,match):
    stats={'metadata_by_tag':{'test':{'state_stats':{'q01':[0]*dim},'action_stats':{'q01':[0]*dim}}}}
    r=molmo_checkpoint_contract(stats,'test')
    assert r['dimension_matches_dual_so101'] is match
    assert not r['direct_deployment_verified']
