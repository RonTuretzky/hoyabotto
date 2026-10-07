"""Physical-structure and asset-integrity checks for the actual G4 roller."""
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np
import pytest

from planter.g4_roller import add_roller, prepare_roller

CAD=Path('/Users/wk/conductor/workspaces/research/minsk/.context/r3-autonomous-planter/original-gravity-v4')


@pytest.fixture(scope='module')
def bundle(tmp_path_factory):
    if not CAD.is_dir():
        pytest.skip('Requires supplied G4 CAD')
    return prepare_roller(CAD,tmp_path_factory.mktemp('roller'))


def model_for(bundle,**kwargs):
    root=E.Element('mujoco')
    E.SubElement(root,'compiler',angle='radian')
    meta=add_roller(root,bundle,**kwargs)
    return mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode')),meta


def test_real_source_bore_and_fork_gaps_preserved(bundle):
    assert all(p['passed'] for p in bundle['geometry_probes'])
    for part in bundle['parts'].values():
        assert part['extra_mm3']<.001
        assert part['missing_mm3']<.001


def test_wheel_is_passive_and_axis_matches_source(bundle):
    model,meta=model_for(bundle)
    assert model.nu==model.nmocap==model.neq==0
    assert model.njnt==2
    hinge=model.joint(meta['wheel_joint'])
    assert model.jnt_type[hinge.id]==mujoco.mjtJoint.mjJNT_HINGE
    assert not model.jnt_limited[hinge.id]
    np.testing.assert_allclose(model.jnt_axis[hinge.id],[0,1,0])
    np.testing.assert_allclose(model.jnt_pos[hinge.id],[0,0,.008])
    assert model.dof_frictionloss[hinge.dofadr[0]]>0
    assert model.dof_damping[hinge.dofadr[0]]>0
    np.testing.assert_allclose(model.body_mass[1:].sum(),.03479)


def test_no_wheel_handle_initial_intersection_constraint(bundle):
    model,meta=model_for(bundle)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert data.ncon==0
    assert model.body_parentid[model.body(meta['wheel_body']).id]==model.body(meta['body']).id


def test_bearing_loss_and_pose_inputs_fail_closed(bundle):
    with pytest.raises(ValueError,match='Bearing losses'):
        model_for(bundle,damping=-1.)
    with pytest.raises(ValueError,match='unit'):
        model_for(bundle,quaternion=(2,0,0,0))


def test_modified_collision_asset_rejected(bundle):
    import copy
    modified=copy.deepcopy(bundle)
    modified['parts']['wheel']['collisions'][0]['sha256']='0'*64
    with pytest.raises(ValueError,match='hash mismatch'):
        model_for(modified)
