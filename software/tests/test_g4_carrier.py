"""Geometry and independent-score contracts; these are not physical tests."""
import pytest
import numpy as np
import trimesh

from planter.g4_carrier import convex_partition, evaluate, solid, union_solids


def trajectory():
    data=[]
    for t in np.arange(0,2.001,.001):
        seated=t>=.2
        contact=dict(geoms=['holder_0000','carrier_0000'],position_m=[-.045,0,0],
                     normal_force_n=.024525,penetration_mm=.01,
                     frame=[0,0,1,1,0,0,0,1,0],wrench_local=[.024525,0,0,0,0,0])
        data.append(dict(time_s=float(t),origin_mm=[-45.,0.,0. if seated else 56*(1-t/.2)],
                         quat=[1.,0.,0.,0.],tilt_deg=0.,linear_speed_mm_s=0.,angular_speed_rad_s=0.,
                         holder_normal_force_n=.024525 if seated else 0.,normal_force_n=.024525 if seated else 0.,
                         penetration_mm=.01 if seated else 0.,contacts=[contact] if seated else []))
    return data


def test_requires_complete_timeline_and_continuous_settle():
    data=trajectory()
    assert evaluate(data)['status']=='pass'
    assert evaluate(data[-300:])['status']=='fail'
    assert evaluate(data[1000:])['status']=='fail'
    data[-200]['linear_speed_mm_s']=20
    assert evaluate(data)['status']=='fail'


def test_commanded_seat_without_holder_contact_is_not_success():
    data=trajectory()
    for r in data:r['holder_normal_force_n']=0
    assert evaluate(data)['status']=='fail'


def test_stop_failure_and_nonfinite_cannot_pass():
    assert evaluate(trajectory(),stop_reason='Contact gate')['status']=='fail'
    data=trajectory();data[-1]['quat'][0]=float('nan')
    assert evaluate(data)['status']=='fail'


def test_source_partition_preserves_a_functional_hole():
    outer=solid(trimesh.creation.box([10,10,2]))
    hole=solid(trimesh.creation.box([8,8,4]))
    source=outer-hole
    pieces,_=convex_partition([source])
    collision=union_solids(pieces)
    assert (collision^hole).volume()<.001
    assert (source-collision).volume()<.001
    assert (collision-source).volume()<.001
    assert len(pieces)>1


def test_malformed_manifold_is_rejected_before_boolean_audit():
    mesh=trimesh.creation.box([10,10,2])
    mesh.update_faces(np.arange(len(mesh.faces)-1))
    with pytest.raises(ValueError,match='Invalid manifold'):
        solid(mesh)


def test_evaluator_independently_checks_force_and_penetration():
    data=trajectory();data[200]['normal_force_n']=3
    assert evaluate(data)['status']=='fail'
    data=trajectory();data[200]['penetration_mm']=.2
    assert evaluate(data)['status']=='fail'


@pytest.mark.parametrize('field,value,index', [
    ('time_s',float('nan'),100),('tilt_deg',float('nan'),100),
    ('penetration_mm',float('nan'),100),('holder_normal_force_n',float('inf'),100),
    ('quat',[0,0,0,0],100)])
def test_nonfinite_or_nonunit_evidence_rejected(field,value,index):
    data=trajectory();data[index][field]=value
    assert evaluate(data)['status']=='fail'


def test_sparse_counterfeit_support_and_already_seated_rejected():
    data=trajectory()
    assert evaluate([data[0],data[1500],data[-1]])['status']=='fail'
    for r in data:r['contacts']=[]
    assert evaluate(data)['status']=='fail'
    data=trajectory();data[0]['origin_mm'][2]=0
    assert evaluate(data)['status']=='fail'


def test_whole_rotated_carrier_must_remain_in_collision_corridor():
    data=trajectory();data[100]['quat']=[2**-.5,0,0,2**-.5]
    assert evaluate(data)['status']=='fail'


def test_zero_normal_tangential_force_cannot_count_as_loaded_support():
    data=trajectory()
    for r in data:
        for c in r['contacts']:
            c['frame']=[1,0,0,0,0,1,0,-1,0]
            c['wrench_local']=[0,.024525,0,0,0,0]
            c['normal_force_n']=0
            r['normal_force_n']=r['holder_normal_force_n']=0
    assert evaluate(data)['status']=='fail'


def test_negative_normal_wrench_is_rejected():
    data=trajectory()
    c=data[-1]['contacts'][0];c['wrench_local'][0]=-1
    c['normal_force_n']=data[-1]['normal_force_n']=data[-1]['holder_normal_force_n']=0
    assert evaluate(data)['status']=='fail'


def test_source_specific_wall_cover_rejects_unrecognized_holder_topology():
    from planter.g4_carrier import holder_wall_cover
    with pytest.raises(ValueError,match='Unsupported holder height topology'):
        holder_wall_cover(solid(trimesh.creation.box([32,83,1.5])))
