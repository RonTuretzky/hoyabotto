"""CLI safety boundary tests; tiny synthetic models are not station calibration."""
import copy
import hashlib
import json
from pathlib import Path
import runpy
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco=pytest.importorskip('mujoco')
from tools import diagnose_folding_observed_scene as cli
from carton.folding_observed_scene import ENCODER_NAMES,FLAPS
from carton.folding_observed_adapter import export_offline_robot_model


def source():
    root=ET.Element('mujoco');ET.SubElement(root,'compiler',angle='radian')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'geom',name='table',type='box',pos='0 0 -.05',size='.7 .7 .05')
    ET.SubElement(world,'camera',name='station',pos='-.4 -.45 .85',fovy='48')
    for side in ('left','right'):
        body=ET.SubElement(world,'body',name=side+'_base_link',pos='0 0 .3')
        for name in ENCODER_NAMES:
            if name.startswith(side+'_'):ET.SubElement(body,'joint',name=name,type='hinge',axis='0 0 1',range='-2 2')
        ET.SubElement(body,'geom',name=side+'_sphere',type='sphere',pos='.1 0 0',size='.02')
        tag=ET.SubElement(body,'body',name=side+'_tag',pos='.1 0 .02')
        ET.SubElement(tag,'site',name=side+'_tag_center',pos='0 0 .0003')
    for name in ('table_tag','table_tag_backup'):ET.SubElement(world,'body',name=name)
    carton=ET.SubElement(world,'body',name='carton',pos='0 .3 .1')
    ET.SubElement(carton,'freejoint',name='carton_free')
    ET.SubElement(carton,'geom',name='bottom',type='box',size='.1 .1 .0015')
    for name in FLAPS:
        body=ET.SubElement(carton,'body',name=name,pos='0 0 .1')
        ET.SubElement(body,'joint',name=name+'_hinge',axis='1 0 0',range='-1.7 3.05')
        ET.SubElement(body,'geom',name=name+'_cardboard',type='box',pos='0 0 .07',size='.1 .0015 .07')
    return root


def xml(root):return ET.tostring(root,encoding='unicode')


@pytest.mark.parametrize('values,expected', [(['0,527,529'],[0,527,529]),(['0','5,10'],[0,5,10])])
def test_frame_list_has_explicit_ascending_semantics(values,expected):
    assert cli.parse_frames(values)==expected


@pytest.mark.parametrize('values', [[],['-1'],['2,1'],['1,1'],['1.5'],['1,'],['truth']])
def test_bad_frame_list_is_refused(values):
    with pytest.raises(ValueError):cli.parse_frames(values)


def test_setup_fingerprint_excludes_only_free_carton_world_pose_and_material():
    root=source();before=cli.digest(cli.setup_declaration(xml(root)))
    carton=root.find("./worldbody/body[@name='carton']")
    carton.set('pos','8 -9 7');carton.set('quat','0 1 0 0')
    carton.find('.//joint').set('stiffness','999')
    assert cli.digest(cli.setup_declaration(xml(root)))==before


@pytest.mark.parametrize('kind', ['base','camera','station','carton-size','hinge-axis','marker'])
def test_changed_source_geometry_cannot_silently_borrow_supported_setup(kind):
    root=source();before=cli.digest(cli.setup_declaration(xml(root)))
    if kind=='base':root.find("./worldbody/body[@name='left_base_link']").set('pos','1 0 .3')
    elif kind=='camera':root.find('./worldbody/camera').set('fovy','60')
    elif kind=='station':root.find('./worldbody/geom').set('size','1 1 .05')
    elif kind=='carton-size':root.find(".//geom[@name='bottom']").set('size','.2 .2 .0015')
    elif kind=='hinge-axis':root.find(".//joint[@name='long_near_hinge']").set('axis','0 1 0')
    else:root.find(".//body[@name='left_tag']").set('pos','.2 0 .02')
    assert cli.digest(cli.setup_declaration(xml(root)))!=before


@pytest.mark.parametrize('kind', ['extra-object','missing-anchor','include','extension','equality','deformable'])
def test_unsupported_scene_setup_is_refused_before_rendering(kind):
    root=source()
    if kind=='extra-object':ET.SubElement(root.find('worldbody'),'body',name='paddle')
    elif kind=='missing-anchor':
        world=root.find('worldbody');world.remove(world.find("body[@name='table_tag_backup']"))
    else:ET.SubElement(root,kind)
    with pytest.raises(ValueError):cli.setup_declaration(xml(root))


def test_existing_output_is_preserved(tmp_path):
    output=tmp_path/'out';output.mkdir();sentinel=output/'keep';sentinel.write_text('prior evidence')
    args=SimpleNamespace(run=str(tmp_path/'absent'),out=str(output),frames=['0'],uncertainty_profile=cli.PROFILE_NAME,seed=0)
    with pytest.raises(FileExistsError):cli.run(args)
    assert sentinel.read_text()=='prior evidence' and list(output.iterdir())==[sentinel]


def test_unsupported_setup_has_a_refusal_report_and_input_source_hashes(tmp_path):
    run=tmp_path/'run';run.mkdir();scene=xml(source())
    (run/'scene.xml').write_text(scene);(run/'folding-frames.json').write_text('[{"time":0,"qpos":[]}]')
    args=SimpleNamespace(run=str(run),out=str(tmp_path/'out'),frames=['0'],uncertainty_profile=cli.PROFILE_NAME,seed=0)
    report=cli.run(args)
    assert report['status']=='setup_refused' and report['cases']==[]
    assert report['input_files_sha256']['scene.xml']==hashlib.sha256(scene.encode()).hexdigest()
    assert report['executed_source_manifest_sha256']==cli.digest(report['executed_sources_sha256'])
    assert report['uncertainty_profile_sha256']==cli.digest(cli.PROFILE)
    assert report['asset_manifest_sha256'] is None
    assert report['physical_calibration_verified'] is report['fold_success_assessed'] is False
    assert json.loads((tmp_path/'out'/'result.json').read_text())['status']=='setup_refused'


def test_asset_bytes_are_frozen_with_source_identities(tmp_path):
    root=source();asset=ET.SubElement(root,'asset')
    mesh=tmp_path/'part.obj';mesh.write_text('source asset bytes')
    ET.SubElement(asset,'mesh',name='part',file=mesh.name)
    frozen,blobs,manifest=cli.frozen_assets(xml(root),tmp_path)
    sha=hashlib.sha256(mesh.read_bytes()).hexdigest()
    assert manifest=={str(mesh):sha}
    name=ET.fromstring(frozen).find('./asset/mesh').get('file')
    assert blobs[name]==b'source asset bytes'
    mesh.write_text('changed later')
    assert hashlib.sha256(blobs[name]).hexdigest()==sha


def test_marker_registry_is_restored_after_success_or_failure():
    original=copy.deepcopy(cli.BOX_MARKERS);sizes=copy.deepcopy(cli.SIZES)
    with pytest.raises(RuntimeError):
        with cli.declared_marker_registry():
            assert cli.BOX_MARKERS[24]==cli.FLOOR_MARKERS[24]
            assert cli.SIZES[25]==.045
            raise RuntimeError('stop')
    assert cli.BOX_MARKERS==original and cli.SIZES==sizes


def test_sensor_facade_exposes_encoders_and_pixels_but_no_render_object_state():
    scene_xml=xml(source());model=mujoco.MjModel.from_xml_string(scene_xml)
    robot=export_offline_robot_model(scene_xml,root_body_names=cli.ROOTS,asset_root='/tmp',
        model_id='offline:fixture',measurement_id='offline:fixture',geometry_error_m=.0002)
    encoder_model=mujoco.MjModel.from_xml_string(robot.xml)
    sensor=cli.SensorReplay(model,encoder_model,None,cli.PROFILE)
    state=model.qpos0.copy()
    for name in ENCODER_NAMES:state[model.joint(name).qposadr[0]]=.01
    sensor.load({'time':0.,'qpos':state.tolist()})
    encoders=sensor.named_encoders();fk=sensor.arm_tag_fk('left')
    assert set(encoders)==set(ENCODER_NAMES)
    assert not hasattr(sensor.data,'qpos') and not hasattr(sensor.model,'body')
    state[model.joint('carton_free').qposadr[0]:][:3]=[9.,8.,7.]
    for name in FLAPS:state[model.joint(name+'_hinge').qposadr[0]]=1.5
    sensor.load({'time':1.,'qpos':state.tolist()})
    assert sensor.named_encoders()==encoders
    np.testing.assert_array_equal(sensor.arm_tag_fk('left'),fk)
    encoders['left_gripper']=100.
    assert sensor.named_encoders()['left_gripper']==.01
    assert not hasattr(sensor,'move') and not hasattr(sensor,'truth_angles')


def test_invalid_frame_cannot_reuse_previous_sensor_images():
    model=mujoco.MjModel.from_xml_string(xml(source()))
    sensor=cli.SensorReplay(model,model,None,cli.PROFILE)
    sensor.images['rgb']=np.zeros((2,2,3))
    with pytest.raises(ValueError):sensor.load({'time':0.,'qpos':[]})
    assert sensor.images=={}


def test_reading_evaluation_labels_clear_or_refused_configuration_without_fold_success():
    fixture=runpy.run_path(str(Path(__file__).with_name('test_folding_observed_adapter.py')))
    robot,calibration,assumptions,reading,history,capture=fixture['inputs']()
    result=cli.evaluate_reading(reading,history,capture,robot,calibration,assumptions)
    assert result['all_required']['status']=='configuration_clear'
    assert set(result['all_required']['configuration_checks'])=={'left','right'}
    assert 'success' not in result['all_required']
    reading['angles'].pop('short_right')
    result=cli.evaluate_reading(reading,history,capture,robot,calibration,assumptions)
    assert result['all_required']['status']=='scene_refused'
    assert result['majors_required_optional_sweeps']['status']=='configuration_clear'
    assert 'short_right' in result['majors_required_optional_sweeps']['scene_metadata']['conservative_sweeps']


def test_cli_requires_a_named_synthetic_profile():
    with pytest.raises(SystemExit) as exc:cli.main(['--run','unused','--frames','0','--out','unused'])
    assert exc.value.code==2
