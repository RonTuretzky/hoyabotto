"""Structural guards for the contact-actuated dual-arm G4 station."""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import pytest

from planter.g4_station import StationConfig,JOINTS,arm_scene,guide_targets

SOURCE=Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot')

@pytest.mark.parametrize('change',[{'base_spacing_m':0},{'timestep_s':float('nan')},{'timestep_s':0},
                                  {'base_y_m':-.20},{'contact_timeconstant_s':-.01}])
def test_invalid_station_configuration_refuses(change):
    with pytest.raises(ValueError):StationConfig(**change).validate()


def test_dual_targets_keep_actual_handle_spacing_and_only_declared_translation():
    origin=np.array([.1,.2,.3]);targets=guide_targets(origin,correction_m=[.002,0,-.001])
    np.testing.assert_allclose(targets['right']-targets['left'],[.138,0,0])
    np.testing.assert_allclose((targets['right']+targets['left'])/2,origin+[.002,0,.042])


@pytest.mark.skipif(not (SOURCE/'scene-assets/arm-import.xml').exists(),reason='Original SO101 imported assets unavailable')
def test_both_imported_arms_keep_source_limits_and_only_joint_actuation():
    original=E.parse(SOURCE/'scene-assets/arm-import.xml').getroot();root=arm_scene(SOURCE,StationConfig())
    source_ranges={j.get('name'):j.get('range') for j in original.findall('.//joint')}
    assert len(root.findall('./actuator/position'))==12
    for side in ('left','right'):
        for name in JOINTS:
            node=root.find(f".//joint[@name='{side}_{name}']")
            assert node.get('range')==source_ranges[name]
            actuator=root.find(f"./actuator/position[@joint='{side}_{name}']")
            assert actuator is not None and actuator.get('ctrlrange')==source_ranges[name]
    assert root.find('equality') is None
    assert len(root.findall(".//site[@name='left_tip']"))==1
    assert len(root.findall(".//site[@name='right_tip']"))==1


def test_contact_codec_is_exact_for_every_field_and_empty_sets():
    from planter.g4_station import pack_contact_lists,unpack_contact_lists
    rng=np.random.default_rng(715)
    contacts=[dict(geom1=i,geom2=4999-i,position_m=rng.normal(size=3).tolist(),
                   frame=rng.normal(size=(3,3)).tolist(),distance_m=float(rng.normal()),
                   wrench=rng.normal(size=6).tolist()) for i in range(40)]
    row=dict(time_s=.123,contacts=contacts,step_contacts=[])
    assert unpack_contact_lists(pack_contact_lists(row))==row
    assert unpack_contact_lists(dict(contacts=[],step_contacts=[]))==dict(contacts=[],step_contacts=[])

def test_contact_codec_rejects_unrecognized_encoding():
    from planter.g4_station import unpack_contact_lists
    with pytest.raises(ValueError):unpack_contact_lists(dict(contact_codec='guess'))


def test_runtime_load_gate_aggregates_subthreshold_points_by_body_pair():
    import mujoco
    from planter.g4_station import StationSimulation
    model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=".1"/><body name="part"><geom size=".1"/><geom size=".1"/></body></worldbody></mujoco>')
    simulation=StationSimulation.__new__(StationSimulation);simulation.model=model
    contacts=[dict(geom1=0,geom2=i,distance_m=-.00001,wrench=[4.5,0,0,0,0,0]) for i in (1,2)]
    force,penetration=simulation._loads(contacts)
    assert force==9.0 and penetration==.00001


def test_selected_manipulated_root_does_not_allow_other_object_contacts():
    import inspect
    import mujoco
    from planter.g4_station import StationSimulation,validate_manipulated_object
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="guide"><freejoint/><geom size=".01"/></body>
      <body name="pusher"><freejoint/><geom size=".01"/></body>
      </worldbody></mujoco>''')
    assert inspect.signature(StationSimulation).parameters['manipulated_object'].default=='guide'
    simulation=StationSimulation.__new__(StationSimulation)
    for selected,other in [('guide','pusher'),('pusher','guide')]:
        simulation.manipulated_object=validate_manipulated_object(model,selected)
        assert not simulation._unintended_arm_contact(['left_jaw',selected])
        assert simulation._unintended_arm_contact(['left_jaw',other])
        assert simulation._unintended_arm_contact(['right_jaw','holder'])
        assert simulation._unintended_arm_contact(['right_jaw','world'])
        assert simulation._unintended_arm_contact(['right_jaw','left_jaw'])
        assert not simulation._unintended_arm_contact(['right_jaw','right_wrist'])
    for invalid in ('holder','unknown',None):
        with pytest.raises(ValueError):validate_manipulated_object(model,invalid)
    fixed=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="guide"><geom size=".01"/></body></worldbody></mujoco>')
    with pytest.raises(ValueError):validate_manipulated_object(fixed,'guide')
    with pytest.raises(ValueError):validate_manipulated_object(fixed,'pusher')


def test_station_rejects_unsupported_flex_before_logging_or_indexing_contacts(tmp_path):
    import mujoco
    from planter.g4_station import StationSimulation
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="guide"><freejoint/><inertial pos="0 0 0" mass=".1" diaginertia=".001 .001 .001"/></body>
      </worldbody><deformable><flex name="source" body="guide" dim="3" radius="0"
      vertex="0 0 0 .01 0 0 0 .01 0 0 0 .01" element="0 1 2 3"/></deformable></mujoco>''')
    with pytest.raises(ValueError,match='flex contact identifiers'):
        StationSimulation(model,tmp_path/'refused',render=False)
    assert not (tmp_path/'refused').exists()


@pytest.mark.skipif(not (SOURCE/'scene-assets/arm-import.xml').exists(),reason='Original SO101 imported assets unavailable')
def test_constructor_joint_override_is_complete_finite_and_source_bounded():
    import mujoco
    from planter.g4_station import validate_initial_joint_positions
    model=mujoco.MjModel.from_xml_string(E.tostring(arm_scene(SOURCE,StationConfig()),encoding='unicode'))
    valid={s:[0,0,0,0,0,.55] for s in ('left','right')}
    assert validate_initial_joint_positions(model,None) is None
    assert set(validate_initial_joint_positions(model,valid))=={'left','right'}
    for bad in [dict(left=valid['left']),{**valid,'right':[0]*5},
                {**valid,'left':[float('nan'),0,0,0,0,.55]},
                {**valid,'right':[100.,0,0,0,0,.55]}]:
        with pytest.raises(ValueError):validate_initial_joint_positions(model,bad)
