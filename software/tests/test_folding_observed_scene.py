"""Declared synthetic fixtures exercise an observation-only planning boundary."""
import copy
from dataclasses import replace
import hashlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from carton.folding_observed_scene import (
    ENCODER_NAMES, FLAPS, FRAME_CONVENTION, JOINT_NAMES, MeasuredCarton,
    ObservedSceneBuilder, RobotCollisionModel, SceneCalibration, StationBox,
)


def pose(x=0., y=0., z=0.):
    result = np.eye(4); result[:3, 3] = [x, y, z]
    return result


def inputs():
    bodies = []
    for side in ('left', 'right'):
        joints = ''.join(f'<joint name="{side}_{name}" axis="0 0 1" limited="true" range="-2 2" armature=".01"/>'
                         for name in JOINT_NAMES)
        bodies.append(f'<body name="{side}_base">{joints}<geom name="{side}_moving_jaw_fixture" type="sphere" size=".02" pos=".10 0 0" mass=".1"/></body>')
    xml = '<mujoco><compiler angle="radian"/><worldbody>' + ''.join(bodies) + '</worldbody></mujoco>'
    robot = RobotCollisionModel(xml=xml, sha256=hashlib.sha256(xml.encode()).hexdigest(), model_id='fixture-arms-v1',
                                measurement_id='synthetic-fixture-measurements',
                                root_body_names={'left':'left_base', 'right':'right_base'}, asset_root='/tmp',
                                asset_sha256={}, geometry_error_m=.0002)
    carton = MeasuredCarton(length_m=.379, width_m=.283, height_m=.108, flap_m=.14,
                            thickness_m=.003, major_hinge_offset_m=.0035, dimension_error_m=.0005,
                            hinge_limits_deg={name:(-100., 175.) for name in FLAPS},
                            flap_tag_ids=dict(zip(FLAPS, (11, 12, 13, 14))), frame_convention=FRAME_CONVENTION,
                            contents_height_m=None)
    calibration = SceneCalibration(calibration_id='fixture-cal-v1', measurement_id='synthetic-survey',
        measurements_verified=True, world_frame_id='table-world', clock_id='test-monotonic', sensor_id='fixture-rgbd',
        robot_id='fixture-robot', carton_id='carton-A', inventory_id='table-and-carton',
        world_from_robot_bases={'left':pose(-.45,-.4,.35), 'right':pose(.35,-.4,.35)},
        base_position_error_m=.0005, base_rotation_error_deg=.1, anchor_ids=(1,20),
        station_boxes=(StationBox('table',pose(0,0,-.025),(.7,.7,.025),.0005),), carton=carton)
    observation = {'seq':1, 'timestamp_s':10., 'depth_timestamp_s':10.001, 'uncertainty_valid_until_s':10.10,
        'clock_id':'test-monotonic', 'calibration_id':'fixture-cal-v1', 'source':'calibrated_rgbd',
        'length_unit':'m', 'angle_unit':'deg', 'world_frame_id':'table-world', 'sensor_id':'fixture-rgbd',
        'carton_id':'carton-A', 'inventory_id':'table-and-carton', 'unmodelled_obstacles':[],
        'world_from_camera':pose(0,-.5,.8), 'world_from_box':pose(.03,.04,.002),
        'box_registration':{'source_seq':1, 'identity_verified':True, 'ambiguous':False,
                            'position_error_m':.001, 'rotation_error_deg':.1},
        'anchor_ids':[1], 'anchor_fit_rms_mm':1., 'tags':[11,12,13,14],
        'angles':{name:{'degrees':angle, 'observed_seq':1, 'identity':name, 'unambiguous':True,
                        'method':'apriltag_aligned_depth_plane', 'error_bound_deg':.5}
                  for name,angle in zip(FLAPS, [10.,20.,-5.,-15.])}}
    encoders = {'seq':1, 'timestamp_s':10., 'uncertainty_valid_until_s':10.11,
        'clock_id':'test-monotonic', 'calibration_id':'fixture-cal-v1', 'robot_id':'fixture-robot',
        'model_id':'fixture-arms-v1', 'angle_unit':'rad', 'position_frame':'model_joint_coordinates',
        'joint_positions':dict.fromkeys(ENCODER_NAMES,.01), 'error_bounds_rad':dict.fromkeys(ENCODER_NAMES,.001)}
    return robot, calibration, observation, encoders


def build(*, change=None, **kwargs):
    robot, calibration, observation, encoders = inputs()
    if change:
        change(observation, encoders)
    return ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01,**kwargs)


def planner(scene, **kwargs):
    return scene.planner('left',now_s=10.01,clock_id='test-monotonic',calibration_id='fixture-cal-v1',**kwargs)


def test_every_scene_dof_is_observed_and_no_execution_capability_exists():
    scene = build()
    assert scene.model.nq == 12 + 7 + 4
    assert scene.model.nu == scene.model.neq == 0
    assert not hasattr(scene,'move') and not hasattr(scene,'truth_angles')
    np.testing.assert_allclose(scene.data.body('carton').xpos,[.03,.04,.002])
    for name, angle in zip(FLAPS,[10,20,-5,-15]):
        assert scene.data.qpos[scene.model.joint(name+'_hinge').qposadr[0]] == pytest.approx(np.radians(angle))
    assert scene.metadata['observed_flaps'] == sorted(FLAPS)
    assert scene.metadata['conservative_sweeps'] == {}


def test_hidden_simulator_object_state_cannot_change_snapshot_or_path():
    robot,calibration,observation,encoders = inputs()
    first = ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    hidden = mujoco.MjData(first.model)
    hidden.qpos[:] = first.data.qpos
    hidden.qpos[first.model.joint('carton_free').qposadr[0]:][:3] = [1.,-2.,3.]
    for name in FLAPS:
        hidden.qpos[first.model.joint(name+'_hinge').qposadr[0]] = 2.5
    mujoco.mj_forward(first.model,hidden)
    second = ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    np.testing.assert_array_equal(first.data.qpos,second.data.qpos)
    assert first.metadata['scene_sha256'] == second.metadata['scene_sha256']
    goal = np.array([.2,.01,.01,.01,.01])
    path_a = planner(first).plan(goal,max_iterations=20,max_seconds=.1)
    path_b = planner(second).plan(goal,max_iterations=20,max_seconds=.1)
    np.testing.assert_array_equal(path_a,path_b)
    assert hidden.body('carton').xpos[0] != first.data.body('carton').xpos[0]


@pytest.mark.parametrize('field,value', [('source','simulator_truth'),('length_unit','mm'),('angle_unit','rad'),
    ('world_frame_id','other-world'),('sensor_id','other-camera'),('carton_id','other-carton'),
    ('calibration_id','old-calibration'),('clock_id','wall-clock'),('inventory_id','unknown-inventory'),
    ('unmodelled_obstacles',[{'name':'unseen-object'}]),('unmodelled_obstacles',None)])
def test_identity_units_and_unmodelled_inventory_are_mandatory(field,value):
    with pytest.raises(ValueError):
        build(change=lambda observation,encoders:observation.update({field:value}))


@pytest.mark.parametrize('field,value', [('timestamp_s',9.),('timestamp_s',10.02),
    ('depth_timestamp_s',9.9),('depth_timestamp_s',10.02),('uncertainty_valid_until_s',10.),
    ('timestamp_s',float('nan')),('seq',True)])
def test_stale_future_unsynchronized_and_invalid_observations_are_refused(field,value):
    with pytest.raises(ValueError):
        build(change=lambda observation,encoders:observation.update({field:value}))


def test_sequences_only_advance_after_a_successful_build():
    robot,calibration,observation,encoders=inputs();builder=ObservedSceneBuilder(robot,calibration)
    bad=copy.deepcopy(observation);bad['angles'].pop('long_near')
    with pytest.raises(ValueError,match='long_near'):
        builder.build(bad,encoders,now_s=10.01)
    builder.build(observation,encoders,now_s=10.01)
    with pytest.raises(ValueError,match='Stale observation sequence'):
        builder.build(observation,encoders,now_s=10.01)


@pytest.mark.parametrize('field,value', [('observed_seq',0),('identity','long_far'),('unambiguous',False),
    ('method','aligned_depth_cardboard_plane'),('degrees',float('nan')),('degrees',180.),
    ('error_bound_deg',None),('error_bound_deg',6.)])
def test_required_flap_refuses_missing_identity_or_uncertainty(field,value):
    def change(observation,encoders):observation['angles']['long_near'][field]=value
    with pytest.raises(ValueError,match='long_near'):
        build(change=change)


def test_wrong_or_missing_fresh_tag_cannot_supply_flap_identity():
    with pytest.raises(ValueError,match='flap tag missing'):
        build(change=lambda observation,encoders:observation.update(tags=[11,12,13]))


def test_optional_missing_flap_requires_explicit_conservative_policy():
    remove=lambda observation,encoders:observation['angles'].pop('long_near')
    with pytest.raises(ValueError,match='long_near'):
        build(change=remove,required_flaps=('short_left',))
    scene=build(change=remove,required_flaps=('short_left',),unknown_flaps='swept')
    assert scene.model.nq==12+7+3
    assert 'long_near' in scene.metadata['conservative_sweeps']
    assert mujoco.mj_name2id(scene.model,mujoco.mjtObj.mjOBJ_JOINT,'long_near_hinge') == -1
    assert scene.model.geom('long_near_unobserved_sweep').size[1] > .14
    with pytest.raises(ValueError,match='freshly observed'):
        planner(scene,allowed_flaps=('long_near_unobserved_sweep',))
    with pytest.raises(ValueError,match='long_near'):
        build(change=remove,required_flaps=('long_near',),unknown_flaps='swept')


def test_swept_box_contains_missing_flap_at_every_angle_including_outward():
    scene=build(change=lambda observation,encoders:observation['angles'].pop('long_near'),
                required_flaps=(),unknown_flaps='swept')
    size=scene.model.geom('long_near_unobserved_sweep').size
    for angle in np.linspace(-180,180,721):
        radians=np.radians(angle)
        for along in (-.379/2,.379/2):
            for radial in (0.,.14):
                for face in (-.0015,.0015):
                    point=np.array([along,radial*np.sin(radians)+face*np.cos(radians),
                                    radial*np.cos(radians)-face*np.sin(radians)])
                    assert np.all(np.abs(point)<size)


def test_ambiguity_in_optional_flap_becomes_sweep_never_a_default_angle():
    def change(observation,encoders):observation['angles']['long_near']['unambiguous']=False
    scene=build(change=change,required_flaps=('short_left',),unknown_flaps='swept')
    assert 'long_near' not in scene.metadata['observed_flaps']
    assert 'ambiguous' in scene.metadata['conservative_sweeps']['long_near']


def test_conservative_sweep_rejects_space_that_a_guessed_open_flap_would_leave_free():
    robot,calibration,observation,encoders=inputs()
    calibration.world_from_robot_bases['left']=pose(-.1,-.23,.18)
    observed=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    assert planner(observed).valid(np.full(5,.01))
    observation['angles'].pop('long_near')
    uncertain=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01,
                                                          required_flaps=(),unknown_flaps='swept')
    p=planner(uncertain)
    assert not p.valid(np.full(5,.01))
    assert 'long_near_unobserved_sweep' in p.last_collision[:2]


def test_both_arm_errors_are_reserved_in_arm_arm_clearance():
    robot,calibration,observation,encoders=inputs()
    calibration.world_from_robot_bases['left']=pose(-.05,-.4,.35)
    calibration.world_from_robot_bases['right']=pose(-.008,-.4,.35)
    scene=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    # Spheres have 2mm nominal separation, larger than either arm's own
    # error, but smaller than their combined possible relative displacement.
    assert scene.required_robot_clearance_m > .002
    p=planner(scene)
    assert not p.valid(np.full(5,.01))
    assert all(name.endswith('_moving_jaw_fixture') for name in p.last_collision[:2])


def test_major_depth_plane_requires_its_spatial_support_and_hinge_checks():
    def change(observation,encoders):
        row=observation['angles']['long_near']
        row.update(method='aligned_depth_hinge_consistent_plane',pixel_support=120,
                   supported_patches=8,hinge_axis_error_deg=.2,hinge_plane_offset_mm=1.)
        observation['tags'].remove(14)
    assert 'long_near' in build(change=change).metadata['observed_flaps']
    def bad(observation,encoders):
        change(observation,encoders);observation['angles']['long_near']['supported_patches']=1
    with pytest.raises(ValueError,match='spatial support'):build(change=bad)


@pytest.mark.parametrize('field,value', [('position_error_m',.020),('rotation_error_deg',10.),
                                       ('source_seq',0),('identity_verified',False),('ambiguous',True)])
def test_carton_registration_bounds_and_identity_are_not_optional(field,value):
    def change(observation,encoders):observation['box_registration'][field]=value
    with pytest.raises(ValueError):build(change=change)


@pytest.mark.parametrize('field', ['world_from_camera', 'world_from_box'])
@pytest.mark.parametrize('kind', ['missing', 'reflection', 'non-rigid', 'nonfinite'])
def test_observed_transforms_must_be_finite_proper_rigid_poses(field,kind):
    def change(observation,encoders):
        if kind=='missing':observation.pop(field)
        elif kind=='reflection':observation[field][0,0]=-1.
        elif kind=='non-rigid':observation[field][0,0]=2.
        else:observation[field][0,3]=float('nan')
    with pytest.raises(ValueError):build(change=change)


@pytest.mark.parametrize('field,value', [('angle_unit','deg'),('position_frame','normalized_percent'),
    ('model_id','unknown'),('timestamp_s',9.9),('uncertainty_valid_until_s',10.)])
def test_encoder_provenance_and_freshness_are_required(field,value):
    with pytest.raises(ValueError):
        build(change=lambda observation,encoders:encoders.update({field:value}))


def test_missing_out_of_range_or_uncertain_encoders_are_refused():
    for edit in (lambda e:e['joint_positions'].pop('left_gripper'),
                 lambda e:e['error_bounds_rad'].pop('right_gripper'),
                 lambda e:e['joint_positions'].update(left_elbow_flex=2.01),
                 lambda e:e['error_bounds_rad'].update(right_gripper=.1)):
        with pytest.raises(ValueError):build(change=lambda observation,encoders:edit(encoders))


def test_error_bounds_inflate_obstacles_and_mandatory_robot_clearance():
    scene=build();near=scene.model.geom('long_near_cardboard')
    assert near.size[1] > .0015 + scene.metadata['carton_pose_padding_m']
    assert scene.required_robot_clearance_m > .0007
    p=planner(scene)
    assert p.clearance==scene.required_robot_clearance_m
    assert np.all(p.model.geom_margin >= scene.required_robot_clearance_m)
    # Existing allowed-contact 1mm penetration gate is reused; unknown
    # obstacle names are never admitted into that exception list.
    with pytest.raises(ValueError,match='capacity'):planner(scene,extra_clearance_m=.02)


def test_scene_expiry_does_not_get_extended_by_planning():
    scene=build()
    assert scene.metadata['expires_at_s']==10.1
    for now,clock,cal in ((10.11,'test-monotonic','fixture-cal-v1'),
                           (10.01,'wrong-clock','fixture-cal-v1'),
                           (10.01,'test-monotonic','wrong-calibration')):
        with pytest.raises(ValueError):scene.planner('left',now_s=now,clock_id=clock,calibration_id=cal)


def test_declaration_is_copied_and_cannot_change_under_cached_robot_geometry():
    robot,calibration,observation,encoders=inputs()
    builder=ObservedSceneBuilder(robot,calibration)
    calibration.world_from_robot_bases['left'][0,3]=3.
    robot.root_body_names['left']='hidden_changed_root'
    scene=builder.build(observation,encoders,now_s=10.01)
    assert scene.data.body('left_base').xpos[0] == pytest.approx(-.45)


def test_verified_mesh_bytes_are_frozen_and_later_disk_changes_cannot_change_scene(tmp_path):
    robot,calibration,observation,encoders=inputs()
    mesh=tmp_path/'tetra.obj'
    mesh.write_text('v 0 0 0\nv .01 0 0\nv 0 .01 0\nv 0 0 .01\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    root=ET.fromstring(robot.xml)
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'mesh',name='verified_mesh',file=mesh.name)
    geom=root.find(".//geom[@name='left_moving_jaw_fixture']")
    geom.set('type','mesh');geom.set('mesh','verified_mesh');geom.attrib.pop('size')
    xml=ET.tostring(root,encoding='unicode')
    robot=replace(robot,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest(),asset_root=str(tmp_path),
                  asset_sha256={str(mesh):hashlib.sha256(mesh.read_bytes()).hexdigest()})
    builder=ObservedSceneBuilder(robot,calibration)
    mesh.write_text('changed external asset')
    assert builder.build(observation,encoders,now_s=10.01).model.nmesh==1
    with pytest.raises(ValueError,match='Unverified robot collision asset'):
        ObservedSceneBuilder(robot,calibration)


def test_world_frame_change_transforms_every_obstacle_and_preserves_path_checks():
    robot,calibration,observation,encoders=inputs()
    first=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    transform=pose(.2,-.15,.10)
    angle=.3;transform[:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
    calibration=replace(calibration,
        world_from_robot_bases={side:transform@value for side,value in calibration.world_from_robot_bases.items()},
        station_boxes=tuple(replace(box,world_from_box=transform@box.world_from_box) for box in calibration.station_boxes))
    observation['world_from_box']=transform@observation['world_from_box']
    observation['world_from_camera']=transform@observation['world_from_camera']
    second=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    expected=np.einsum('ij,kj->ik',first.data.geom_xpos,transform[:3,:3])+transform[:3,3]
    np.testing.assert_allclose(second.data.geom_xpos,expected,atol=1e-12)
    goal=np.array([.2,.01,.01,.01,.01])
    np.testing.assert_array_equal(planner(first).plan(goal),planner(second).plan(goal))


def test_contents_must_be_explicitly_declared_and_are_conservatively_filled():
    robot,calibration,observation,encoders=inputs()
    calibration=replace(calibration,carton=replace(calibration.carton,contents_height_m=.102))
    scene=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    assert scene.model.geom('contents_conservative').size[2] > .051


@pytest.mark.parametrize('kind', ['unverified','bad-transform','wrong-frame','source-hash','hidden-object'])
def test_declared_geometry_and_calibration_refusal(kind):
    robot,calibration,_,_=inputs()
    if kind=='unverified':calibration=replace(calibration,measurements_verified=False)
    elif kind=='bad-transform':calibration.world_from_robot_bases['left'][0,0]=2.
    elif kind=='wrong-frame':calibration=replace(calibration,carton=replace(calibration.carton,frame_convention='rim-origin'))
    elif kind=='source-hash':robot=replace(robot,sha256='wrong')
    else:
        xml=robot.xml.replace('</worldbody>','<body name="hidden_object"><freejoint/><geom type="sphere" size=".1"/></body></worldbody>')
        robot=replace(robot,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest())
    with pytest.raises(ValueError):ObservedSceneBuilder(robot,calibration)


def test_raw_pixelport_reading_is_not_silently_given_missing_metadata():
    robot,calibration,observation,encoders=inputs()
    raw={key:observation[key] for key in ('seq','world_from_box','angles','tags','box_registration')}
    with pytest.raises(ValueError):ObservedSceneBuilder(robot,calibration).build(raw,encoders,now_s=10.01)


@pytest.mark.parametrize('kind', ['disabled-contact', 'no-collision-geometry', 'incompatible-masks',
                               'environment-exclusion', 'cross-arm-exclusion', 'explicit-pair'])
def test_robot_template_cannot_suppress_required_collision_checks(kind):
    robot,calibration,observation,encoders=inputs();root=ET.fromstring(robot.xml)
    if kind=='disabled-contact':
        ET.SubElement(ET.SubElement(root,'option'),'flag',contact='disable')
    elif kind=='no-collision-geometry':
        root.find(".//geom[@name='left_moving_jaw_fixture']").attrib.update(contype='0',conaffinity='0')
    elif kind=='incompatible-masks':
        root.find(".//geom[@name='right_moving_jaw_fixture']").attrib.update(contype='8',conaffinity='8')
    elif kind=='environment-exclusion':
        ET.SubElement(ET.SubElement(root,'contact'),'exclude',body1='left_base',body2='carton')
    elif kind=='cross-arm-exclusion':
        ET.SubElement(ET.SubElement(root,'contact'),'exclude',body1='left_base',body2='right_base')
    else:
        ET.SubElement(ET.SubElement(root,'contact'),'pair',geom1='left_moving_jaw_fixture',geom2='table')
    xml=ET.tostring(root,encoding='unicode')
    robot=replace(robot,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest())
    with pytest.raises(ValueError):ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)


def test_visual_only_default_and_alternate_robot_mask_cannot_hide_new_obstacles():
    robot,calibration,observation,encoders=inputs();root=ET.fromstring(robot.xml)
    ET.SubElement(ET.SubElement(root,'default'),'geom',contype='0',conaffinity='0')
    for geom in root.findall('.//worldbody//geom'):
        geom.attrib.update(contype='8',conaffinity='8')
    xml=ET.tostring(root,encoding='unicode')
    robot=replace(robot,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest())
    calibration.world_from_robot_bases['left']=pose(-.1,-.23,.18)
    observation['angles'].pop('long_near')
    scene=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01,
                                                       required_flaps=(),unknown_flaps='swept')
    assert scene.model.geom('long_near_unobserved_sweep').contype[0] != 0
    p=planner(scene)
    assert not p.valid(np.full(5,.01))
    assert 'long_near_unobserved_sweep' in p.last_collision[:2]


def test_robot_default_geometry_orientation_and_joint_reference_do_not_change_carton():
    first=build()
    robot,calibration,observation,encoders=inputs();root=ET.fromstring(robot.xml)
    defaults=ET.SubElement(root,'default')
    ET.SubElement(defaults,'geom',quat='0.7071067811865476 0 0 0.7071067811865476')
    ET.SubElement(defaults,'joint',ref='.12',pos='.001 .002 .003')
    xml=ET.tostring(root,encoding='unicode')
    robot=replace(robot,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest())
    second=ObservedSceneBuilder(robot,calibration).build(observation,encoders,now_s=10.01)
    for name in ('table','bottom','wall_left','wall_near',*(flap+'_cardboard' for flap in FLAPS)):
        np.testing.assert_allclose(first.data.geom(name).xpos,second.data.geom(name).xpos,atol=1e-12)
        np.testing.assert_allclose(first.data.geom(name).xmat,second.data.geom(name).xmat,atol=1e-12)
