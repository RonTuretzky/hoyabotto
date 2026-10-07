"""Offline packet adaptation; synthetic bounds never claim physical validity."""
import copy
from dataclasses import replace
import hashlib
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip('mujoco')

from carton.folding_observed_scene import ENCODER_NAMES, FLAPS, ObservedSceneBuilder
from carton.folding_observed_adapter import (
    OfflineObservationAssumptions, OfflinePixelSceneAdapter, OfflineSensorCapture,
    export_offline_robot_model,
)


def inputs():
    # Reuse only the explicitly synthetic declaration fixture. No simulator
    # object or simulator carton/flap qpos is obtained by the adapter.
    robot, calibration, observation, encoders = runpy.run_path(
        str(Path(__file__).with_name('test_folding_observed_scene.py')))['inputs']()
    robot = replace(robot, model_id='offline:' + robot.model_id, measurement_id='offline:' + robot.measurement_id)
    calibration = replace(calibration, **{field: 'offline:' + getattr(calibration, field) for field in
        ('calibration_id','measurement_id','world_frame_id','clock_id','sensor_id','robot_id','carton_id','inventory_id')})
    reading = dict(seq=1, camera='front', label='Synthetic PixelPort schema fixture', paddle=None,
        world_from_box=observation['world_from_box'], tags=[1,10,11,12,13,14],
        box_registration=dict(visible_ids=[10], selected_id=10,
                              max_translation_disagreement_mm=0.,max_rotation_disagreement_degrees=0.),
        angles={name:{'degrees':row['degrees'],'method':row['method'],'depth_check_degrees':None}
                for name,row in observation['angles'].items()})
    history = dict(seq=1, detected=reading['tags'].copy(), rejected={}, anchor_ids=[1], anchor_fit_rms_mm=1.,
        quality={tag:dict(valid_depth_pixels=100,square_fit_rms_mm=1.,plane_rms_mm=.5) for tag in reading['tags']})
    capture = OfflineSensorCapture(run_id='offline:test-run',clock_id=calibration.clock_id,observation_seq=1,
        rgb_timestamp_s=10.,depth_timestamp_s=10.001,world_from_camera=observation['world_from_camera'],
        encoder_seq=1,encoder_timestamp_s=10.,joint_positions=encoders['joint_positions'])
    assumptions = OfflineObservationAssumptions(run_id='offline:test-run',assumption_id='offline:declared-errors-v1',
        camera_name='front',carton_position_error_m=.001,carton_rotation_error_deg=.1,
        flap_error_bounds_deg=dict.fromkeys(FLAPS,.5),encoder_error_bounds_rad=dict.fromkeys(ENCODER_NAMES,.001),
        uncertainty_horizon_s=.1,allow_paired_short_hinge_registration=False)
    return robot,calibration,assumptions,reading,history,capture


def build(*, edit=None, required_flaps=FLAPS, unknown_flaps='refuse', **kwargs):
    robot,calibration,assumptions,reading,history,capture=inputs()
    if edit:
        result=edit(reading,history,capture)
        if result is not None:capture=result
    adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)
    return adapter.build(reading,history,capture,now_s=10.01,required_flaps=required_flaps,
                         unknown_flaps=unknown_flaps,inventory_complete=True,unmodelled_obstacles=[],**kwargs)


def test_pixelport_rows_and_captured_encoders_build_explicitly_offline_scene():
    result=build();packet=result.packets.observation;scene=result.scene
    assert packet['source']=='calibrated_rgbd'
    assert packet['timestamp_s']==10. and packet['depth_timestamp_s']==10.001
    assert packet['uncertainty_valid_until_s']==10.1
    assert packet['box_registration']['source_seq']==1
    assert set(packet['angles'])==set(FLAPS)
    assert packet['angles']['long_near']['degrees']==-15.
    assert result.packets.audit['omitted_flaps']=={}
    assert scene.metadata['simulation_only'] is True
    assert scene.metadata['physical_calibration_verified'] is False
    assert scene.metadata['scene_sha256'] != scene.metadata['builder_scene_sha256']
    assert scene.metadata['offline_adapter']['uncertainty_source'].startswith('explicit offline assumptions')
    assert not hasattr(scene,'move')
    assert set(result.packets.encoders['joint_positions'])==set(ENCODER_NAMES)


@pytest.mark.parametrize('field', ['calibration_id','measurement_id','world_frame_id','clock_id',
                                 'sensor_id','robot_id','carton_id','inventory_id'])
def test_physical_or_unlabelled_calibration_identity_is_refused(field):
    robot,calibration,assumptions,*_=inputs()
    calibration=replace(calibration,**{field:'physical-looking-id'})
    with pytest.raises(ValueError,match='offline:'):
        OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)


@pytest.mark.parametrize('field', ['model_id','measurement_id'])
def test_robot_model_also_requires_explicit_offline_identity(field):
    robot,calibration,assumptions,*_=inputs();robot=replace(robot,**{field:'not-offline'})
    with pytest.raises(ValueError,match='offline:'):
        OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)


@pytest.mark.parametrize('kind', ['history-sequence','camera-sequence','camera-name','clock','run','future','stale','skew'])
def test_mismatched_or_stale_capture_is_not_relabelled_as_fresh(kind):
    def edit(reading,history,capture):
        if kind=='history-sequence':history['seq']=0
        elif kind=='camera-sequence':return replace(capture,observation_seq=2)
        elif kind=='camera-name':reading['camera']='other'
        elif kind=='clock':return replace(capture,clock_id='offline:other-clock')
        elif kind=='run':return replace(capture,run_id='offline:other-run')
        elif kind=='future':return replace(capture,rgb_timestamp_s=10.1)
        elif kind=='stale':return replace(capture,rgb_timestamp_s=9.)
        else:return replace(capture,encoder_timestamp_s=9.97)
    with pytest.raises(ValueError):build(edit=edit)


@pytest.mark.parametrize('kind', ['history-tags','anchor','missing-quality','weak-depth','bad-fit','rejected','duplicate-quality'])
def test_same_frame_tag_identity_and_quality_are_required(kind):
    def edit(reading,history,capture):
        if kind=='history-tags':history['detected'].remove(10)
        elif kind=='anchor':history['anchor_ids']=[20]
        elif kind=='missing-quality':history['quality'].pop(10)
        elif kind=='weak-depth':history['quality'][10]['valid_depth_pixels']=1
        elif kind=='bad-fit':history['quality'][10]['square_fit_rms_mm']=5.
        elif kind=='rejected':history['rejected'][10]='occluded'
        else:history['quality']['10']=history['quality'][10]
    with pytest.raises(ValueError):build(edit=edit)


def test_json_string_quality_keys_preserve_the_current_frame():
    def edit(reading,history,capture):history['quality']={str(k):v for k,v in history['quality'].items()}
    assert build(edit=edit).scene.metadata['observation_seq']==1


@pytest.mark.parametrize('kind', ['privileged-flag','privileged-method','truth-source','paddle','unknown-obstacle'])
def test_known_privileged_or_unmodelled_input_is_refused(kind):
    def edit(reading,history,capture):
        if kind=='privileged-flag':reading['privileged_mechanics_probe']=True
        elif kind=='privileged-method':reading['angles']['long_near']['method']='PRIVILEGED_SIMULATOR_ANGLE_DIAGNOSTIC_ONLY'
        elif kind=='truth-source':reading['source']='simulator_truth'
        elif kind=='paddle':reading['paddle']={'world_from_paddle':np.eye(4)}
        else:reading['unmodelled_obstacles']=['loose object']
    with pytest.raises(ValueError):build(edit=edit,required_flaps=(),unknown_flaps='swept')


@pytest.mark.parametrize('kind', ['legacy','missing','ambiguous','old-row'])
def test_unobserved_flap_is_refused_or_explicitly_swept_never_filled(kind):
    def edit(reading,history,capture):
        if kind=='legacy':reading['angles']['short_left']['method']='aligned_depth_cardboard_plane'
        elif kind=='missing':reading['angles'].pop('short_left')
        elif kind=='ambiguous':reading['angles']['short_left']['unambiguous']=False
        else:reading['angles']['short_left']['observed_seq']=0
    with pytest.raises(ValueError,match='short_left'):build(edit=edit)
    with pytest.raises(ValueError,match='short_left'):build(edit=edit,required_flaps=(),unknown_flaps='refuse')
    result=build(edit=edit,required_flaps=('long_near',),unknown_flaps='swept')
    assert 'short_left' not in result.packets.observation['angles']
    assert 'short_left' in result.packets.audit['omitted_flaps']
    assert 'short_left' in result.scene.metadata['conservative_sweeps']


def test_hinge_consistent_major_keeps_its_checks_without_a_visible_tag():
    def edit(reading,history,capture):
        reading['tags'].remove(14);history['detected'].remove(14)
        reading['angles']['long_near']={'degrees':-15.,'method':'aligned_depth_hinge_consistent_plane',
            'pixel_support':500,'supported_patches':10,'hinge_axis_error_deg':.2,'hinge_plane_offset_mm':1.}
    assert build(edit=edit).packets.observation['angles']['long_near']['pixel_support']==500
    def weak(reading,history,capture):
        edit(reading,history,capture);reading['angles']['long_near']['supported_patches']=1
    with pytest.raises(ValueError,match='spatial support'):build(edit=weak)


def test_inconsistent_registration_cannot_claim_a_smaller_declared_error():
    def edit(reading,history,capture):reading['box_registration']['max_translation_disagreement_mm']=5.
    packet=build(edit=edit).packets.observation
    assert packet['box_registration']['position_error_m']==.005
    def too_large(reading,history,capture):reading['box_registration']['max_rotation_disagreement_degrees']=4.
    with pytest.raises(ValueError):build(edit=too_large)


def test_small_fit_residual_is_not_used_as_a_transform_accuracy_bound():
    def edit(reading,history,capture):
        history['anchor_fit_rms_mm']=.00001
        for quality in history['quality'].values():
            quality.update(square_fit_rms_mm=.00001,plane_rms_mm=.00001)
    packet=build(edit=edit).packets.observation
    assert packet['box_registration']['position_error_m']==.001
    assert packet['box_registration']['rotation_error_deg']==.1


@pytest.mark.parametrize('field,value', [('visible_ids',[21]),('selected_id',21),('ambiguous',True),
                                      ('identity_verified',False),('source_seq',0),('source','simulator_truth')])
def test_bad_box_registration_is_not_promoted_to_verified(field,value):
    def edit(reading,history,capture):reading['box_registration'][field]=value
    with pytest.raises(ValueError):build(edit=edit)


def test_paired_short_registration_requires_an_explicit_offline_policy():
    robot,calibration,assumptions,reading,history,capture=inputs()
    reading['box_registration']=dict(visible_ids=[11,12],selected_id=None,
        source='paired short-flap hinges from fresh RGB-D tags',hinge_spacing_error_mm=.4,
        hinge_axis_disagreement_degrees=.2,hinge_perpendicular_error_degrees=.1,physical_mounts_measured=False)
    adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)
    with pytest.raises(ValueError,match='registration method'):
        adapter.adapt(reading,history,capture,inventory_complete=True,unmodelled_obstacles=[])
    adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),
                                     replace(assumptions,allow_paired_short_hinge_registration=True))
    packets=adapter.adapt(reading,history,capture,inventory_complete=True,unmodelled_obstacles=[])
    assert packets.audit['registration_method']=='fresh_paired_short_hinge_tags'
    assert packets.observation['physical_calibration_verified'] is False


def test_inventory_assertion_and_flap_policy_have_no_implicit_permissive_defaults():
    robot,calibration,assumptions,reading,history,capture=inputs()
    adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)
    with pytest.raises(TypeError):adapter.build(reading,history,capture,now_s=10.01)
    for complete,unknown in ((False,[]),(True,['unmodeled box']),(True,None)):
        with pytest.raises(ValueError):adapter.adapt(reading,history,capture,inventory_complete=complete,unmodelled_obstacles=unknown)


def test_extra_object_qpos_or_missing_robot_encoder_cannot_enter_encoder_packet():
    def extra(reading,history,capture):capture.joint_positions['carton_free']=0.
    with pytest.raises(ValueError,match='twelve named'):build(edit=extra)
    def missing(reading,history,capture):capture.joint_positions.pop('right_gripper')
    with pytest.raises(ValueError,match='twelve named'):build(edit=missing)


def test_error_assumptions_and_current_pixelport_geometry_must_be_explicit():
    robot,calibration,assumptions,*_=inputs()
    for changed in (replace(assumptions,flap_error_bounds_deg={}),
                    replace(assumptions,encoder_error_bounds_rad={}),
                    replace(assumptions,uncertainty_horizon_s=1.),
                    replace(assumptions,assumption_id='measured-on-hardware')):
        with pytest.raises(ValueError):OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),changed)
    changed=replace(calibration,carton=replace(calibration.carton,length_m=.40))
    with pytest.raises(ValueError,match='task geometry'):
        OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,changed),assumptions)


def test_marker_layout_cannot_change_after_adapter_calibration(monkeypatch):
    from carton.folding_markers import BOX_MARKERS
    robot,calibration,assumptions,reading,history,capture=inputs()
    adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)
    monkeypatch.setitem(BOX_MARKERS,24,('floor-marker',[.08,0,.0038],[1,0,0,0,1,0]))
    with pytest.raises(ValueError,match='declarations changed'):
        adapter.adapt(reading,history,capture,inventory_complete=True,unmodelled_obstacles=[])


def test_irrelevant_hidden_diagnostic_values_cannot_change_packets_scene_or_path():
    first=build()
    def edit(reading,history,capture):
        reading['truth_angles']={name:170. for name in FLAPS}
        reading['hidden_carton_qpos']=[9.,8.,7.,1.,0.,0.,0.]
        reading['grasp_success']=True
        history['independent_evaluator']={'all_closed':True}
    second=build(edit=edit)
    assert first.packets==second.packets
    assert first.scene.metadata['scene_sha256']==second.scene.metadata['scene_sha256']
    np.testing.assert_array_equal(first.scene.data.qpos,second.scene.data.qpos)
    paths=[]
    for result in (first,second):
        scene=result.scene
        p=scene.planner('left',now_s=10.01,clock_id='offline:test-monotonic',calibration_id='offline:fixture-cal-v1')
        paths.append(p.plan(np.array([.2,.01,.01,.01,.01])))
    np.testing.assert_array_equal(*paths)


def test_offline_robot_export_omits_object_geometry_actuation_and_keyframes():
    robot,calibration,assumptions,*_=inputs()
    source=ET.fromstring(robot.xml)
    world=source.find('worldbody')
    body=ET.SubElement(world,'body',name='carton',pos='1 2 3')
    ET.SubElement(body,'freejoint',name='carton_free');ET.SubElement(body,'geom',type='box',size='.1 .1 .1')
    ET.SubElement(source,'actuator');ET.SubElement(ET.SubElement(source,'keyframe'),'key',qpos='1 2 3')
    def export():return export_offline_robot_model(ET.tostring(source,encoding='unicode'),
        root_body_names=robot.root_body_names,asset_root='/tmp',model_id='offline:exported-robot',
        measurement_id='offline:static-source-geometry',geometry_error_m=.0002)
    first=export();body.set('pos','9 8 7');second=export()
    assert first.xml==second.xml and first.sha256==second.sha256
    parsed=ET.fromstring(first.xml)
    assert parsed.find('keyframe') is None and parsed.find('actuator') is None
    assert parsed.find(".//body[@name='carton']") is None
    assert parsed.findall('.//freejoint')==[]
    OfflinePixelSceneAdapter(ObservedSceneBuilder(first,calibration),assumptions)


def test_offline_robot_export_freezes_explicit_asset_identity(tmp_path):
    robot,_,_,*_=inputs();source=ET.fromstring(robot.xml)
    mesh=tmp_path/'part.obj';mesh.write_text('declared asset bytes')
    ET.SubElement(ET.SubElement(source,'asset'),'mesh',name='part',file=mesh.name)
    result=export_offline_robot_model(ET.tostring(source,encoding='unicode'),root_body_names=robot.root_body_names,
        asset_root=str(tmp_path),model_id='offline:asset-fixture',measurement_id='offline:model-declaration',geometry_error_m=.001)
    assert result.asset_sha256[str(mesh)]==hashlib.sha256(mesh.read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='offline:'):
        export_offline_robot_model(robot.xml,root_body_names=robot.root_body_names,asset_root='/tmp',
            model_id='physical-robot',measurement_id='offline:model-declaration',geometry_error_m=.001)
