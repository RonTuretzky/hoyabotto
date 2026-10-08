"""Assumed-material component tests, not adhesion or robot folding tests."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from carton.folding_extensible_tape import ExtensibleTapeSpec, add_extensible_tape
from carton.folding_tape import TapeSpec, add_tape
from tools.diagnose_extensible_tape import make_model, run_case


def test_finite_series_compliance_preserves_original_material_and_free_root():
    spec=ExtensibleTapeSpec()
    model,_=make_model(spec,.000025)
    assert model.nu == model.neq == 0
    assert model.joint('tape_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert model.body_subtreemass[model.body('tape_0').id] == pytest.approx(.0003888)
    ids=[model.joint(f'tape_axial_{i}').id for i in range(1,36)]
    assert all(model.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE for i in ids)
    assert not np.any(model.jnt_limited[ids])
    assert np.sum(1/model.jnt_stiffness[ids]) == pytest.approx(.180/(200e6*.024*.0001))
    assert all(model.geom_adhesion[model.geom(f'tape_adhesive_{i}').id] == .020 for i in range(36))
    assert all(model.geom_adhesion[model.geom(f'tape_backing_{i}').id] == 0 for i in range(36))


def test_legacy_80mm_coupon_does_not_gain_axial_joints():
    root=ET.fromstring('<mujoco><worldbody/></mujoco>')
    add_tape(root,TapeSpec(),start_m=[0,0,0])
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    assert model.njnt == 1+2*15
    assert all('axial' not in model.joint(i).name for i in range(model.njnt))
    assert model.body_subtreemass[model.body('tape_0').id] == pytest.approx(.0001728)


@pytest.mark.parametrize('tension,expected',[(.3,.0001125),(.6,.000225)])
def test_unpinned_free_strip_extends_under_load_and_recovers_after_unload(tmp_path,tension,expected):
    report=run_case(tmp_path/'axial',timestep=.000025,mode='axial',tension=tension)
    assert report['error'] is None
    assert report['axial_plateau_mean_extension_m'] == pytest.approx(expected,rel=.01)
    assert abs(report['samples'][-1]['contour_extension_m']) < 1e-7
    assert report['peak_net_applied_force_N'] < 1e-10
    assert report['peak_net_applied_torque_Nm'] < 1e-7
    assert report['max_actual_contact_count'] == 0


def test_balanced_transverse_instrument_bends_without_rigid_motion_or_contact(tmp_path):
    report=run_case(tmp_path/'bend',timestep=.000025,mode='three_point')
    assert report['error'] is None
    plateau=[s for s in report['samples'] if .28<s['time_s']<.35]
    # Taut-string limiting estimate: .085 m half-span times .06/.6 slope.
    # Finite EI and the two-center distributed reaction reduce this slightly.
    assert np.mean([s['center_sag_m'] for s in plateau]) == pytest.approx(.0085,rel=.04)
    com=np.array([s['center_of_mass_m'] for s in report['samples']])
    # One micrometre is 5% of the declared 20 um adhesive range. Discrete
    # integration is not exactly momentum conserving; retain/report that drift.
    # This diagnostic tolerance is unrelated to original robot collision gates.
    assert np.max(np.linalg.norm(com-com[0],axis=1)) < 1e-6
    assert report['peak_center_of_mass_drift_m'] < 1e-6
    assert report['energy_passivity_validated'] is False
    assert report['peak_net_applied_force_N'] < 1e-10
    assert report['peak_net_applied_torque_Nm'] < 1e-7
    assert report['max_actual_contact_count'] == 0
    assert report['adhesion_or_peel_strength_test'] is False


@pytest.mark.parametrize('damping',[True,-1.,float('nan'),float('inf')])
def test_invalid_new_axial_material_parameter_is_refused(damping):
    with pytest.raises(ValueError):ExtensibleTapeSpec(axial_damping_Ns_m=damping)


def test_nonfinite_instrument_load_refused_before_evidence_directory(tmp_path):
    out=tmp_path/'bad'
    with pytest.raises(ValueError):run_case(out,timestep=.000025,mode='axial',tension=float('nan'))
    assert not out.exists()
