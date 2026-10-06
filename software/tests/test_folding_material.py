import math
import pytest

from carton.folding_material import CartonMaterial
from carton.folding_station import FoldingStation


@pytest.mark.parametrize('kwargs',[
    {'contents_mass_kg':-1}, {'cardboard_mass_kg':0}, {'table_friction':math.nan},
    {'hinge_stiffness':-1}, {'flap_stiffness':(.01,.02)},
    {'flap_stiffness':(.01,.02,math.inf,.03)}, {'hinge_rest_degrees':90},
    {'contents_top_m':.20}, {'hinge_damping':-1},
])
def test_invalid_physical_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):CartonMaterial(**kwargs)


@pytest.fixture
def scene_factory(tmp_path):
    mujoco=pytest.importorskip('mujoco')
    from carton.folding_sim import build_scene
    source=tmp_path/'source';assets=source/'scene-assets';assets.mkdir(parents=True)
    (assets/'jaw-collision').mkdir();(assets/'jaw-collision/manifest.json').write_text('{}')
    # Tiny arm geometry keeps tests portable; the same scene builder creates
    # the real carton, hinges, contact pairs and free body used in production.
    joints=''.join(f'<joint name="{n}" axis="0 1 0" range="-2 2"/>' for n in
                   ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'))
    (assets/'arm-import.xml').write_text(f'<mujoco><compiler angle="radian"/><asset/><worldbody>'
        f'<body name="base_link"><geom type="sphere" size=".01"/>'
        f'<body name="gripper_link" pos="0 0 .1">{joints}<geom type="sphere" size=".01"/></body>'
        '</body></worldbody></mujoco>')
    def make(material,**kwargs):
        return build_scene(source,tmp_path/'scene',station=FoldingStation(.06,.15,.05),material=material,**kwargs)
    return mujoco,make


def test_empty_carton_has_no_contents_and_is_not_welded(scene_factory):
    mj,make=scene_factory;m=make(CartonMaterial())
    assert mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'contents')==-1
    assert m.joint('carton_free').type==mj.mjtJoint.mjJNT_FREE
    assert m.neq==0
    assert m.body_subtreemass[m.body('carton').id]==pytest.approx(.272)
    assert m.nu==12


def test_loaded_mass_and_independent_crease_torques_reach_physics(scene_factory):
    mj,make=scene_factory
    material=CartonMaterial(contents_mass_kg=.96,flap_stiffness=(.01,.02,.03,.04))
    m=make(material);d=mj.MjData(m)
    assert m.body_subtreemass[m.body('carton').id]==pytest.approx(1.232)
    for name,k in zip(('short_left','short_right','long_far','long_near'),material.stiffnesses):
        j=m.joint(name+'_hinge');d.qpos[j.qposadr[0]]=math.pi/2
    mj.mj_forward(m,d)
    for name,k in zip(('short_left','short_right','long_far','long_near'),material.stiffnesses):
        j=m.joint(name+'_hinge')
        assert d.qfrc_passive[j.dofadr[0]]==pytest.approx(-k*math.pi/2)


@pytest.mark.parametrize('friction_solver',[False,True])
def test_lower_table_friction_really_allows_unbolted_carton_to_slide(scene_factory,friction_solver):
    from carton.folding_solver import FoldingSolver
    mj,make=scene_factory
    distances=[]
    for mu in (.05,.8):
        m=make(CartonMaterial(table_friction=mu),solver=FoldingSolver.friction() if friction_solver else FoldingSolver());d=mj.MjData(m);body=m.body('carton').id
        for _ in range(200):mj.mj_step(m,d)
        start=d.xpos[body].copy();d.xfrc_applied[body,0]=2.
        for _ in range(150):mj.mj_step(m,d)
        distances.append(abs(d.xpos[body,0]-start[0]))
        contacts=[c for c in d.contact if {m.geom(c.geom1).name,m.geom(c.geom2).name}=={'table','bottom'}]
        assert contacts and all(c.friction[0]==pytest.approx(mu) for c in contacts)
    assert distances[0]>.01
    assert distances[1]<.002


@pytest.mark.parametrize('friction_solver',[False,True])
def test_resistant_flaps_spring_open_without_a_hold(scene_factory,friction_solver):
    from carton.folding_solver import FoldingSolver
    from carton.folding_sim import initialize_flaps,FLAPS
    mj,make=scene_factory;m=make(CartonMaterial(),solver=FoldingSolver.friction() if friction_solver else FoldingSolver())
    d=mj.MjData(m)
    initialize_flaps(m,d,dict.fromkeys(FLAPS,math.pi/2))
    for _ in range(1000):mj.mj_step(m,d)
    assert math.degrees(d.qpos[m.joint('long_far_hinge').qposadr[0]])<70


def test_solver_experiment_does_not_change_physical_constraints(scene_factory):
    from carton.folding_solver import FoldingSolver,solver_report
    import numpy as np
    mj,make=scene_factory
    baseline=make(CartonMaterial());experiment=make(CartonMaterial(),solver=FoldingSolver.friction())
    for name in ('body_mass','jnt_range','jnt_stiffness','dof_frictionloss',
                 'geom_friction','pair_friction','actuator_forcerange'):
        np.testing.assert_array_equal(getattr(baseline,name),getattr(experiment,name))
    assert baseline.neq==experiment.neq==0
    assert baseline.nu==experiment.nu==12
    assert solver_report(experiment)['noslip_iterations']==3


def test_initial_flaps_are_separated_without_changing_spring_rest(scene_factory):
    from carton.folding_sim import initialize_flaps
    mj,make=scene_factory;m=make(CartonMaterial());d=mj.MjData(m)
    angles=initialize_flaps(m,d)
    assert angles['short_left']==pytest.approx(math.degrees(.1))
    assert angles['long_near']==pytest.approx(math.degrees(-.1))
    for name in angles:
        joint=m.joint(name+'_hinge')
        assert m.qpos_spring[joint.qposadr[0]]==0
        assert m.jnt_stiffness[joint.id]==pytest.approx(.018)
    assert not any(c.dist<-.0001 and m.geom(c.geom1).name.endswith('_cardboard')
                   and m.geom(c.geom2).name.endswith('_cardboard') for c in d.contact)


def test_old_interpenetrating_initial_pose_is_rejected_and_restored(scene_factory):
    from carton.folding_sim import initialize_flaps,FLAPS
    import numpy as np
    mj,make=scene_factory;m=make(CartonMaterial());d=mj.MjData(m)
    before=d.qpos.copy()
    with pytest.raises(ValueError,match='panels intersect'):
        initialize_flaps(m,d,dict.fromkeys(FLAPS,.1))
    np.testing.assert_array_equal(d.qpos,before)
    with pytest.raises(ValueError,match='all four'):
        initialize_flaps(m,d,{})


def test_hinge_tag_registration_matches_compiled_marker_mounts(scene_factory):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from carton.folding_hinge_tags import carton_pose_from_short_flaps
    mj,make=scene_factory;m=make(CartonMaterial());d=mj.MjData(m)
    adr=m.joint('carton_free').qposadr[0]
    d.qpos[adr:adr+3]=[.08,-.02,.03]
    rotation=Rotation.from_euler('xyz',[.12,-.18,.6]).as_matrix()
    mj.mju_mat2Quat(d.qpos[adr+3:adr+7],rotation.ravel())
    d.qpos[m.joint('short_left_hinge').qposadr[0]]=-.4
    d.qpos[m.joint('short_right_hinge').qposadr[0]]=1.2
    mj.mj_kinematics(m,d)
    tags={}
    for tag,name in ((11,'short_left_tag'),(12,'short_right_tag')):
        pose=np.eye(4)
        pose[:3,:3]=d.body(name).xmat.reshape(3,3)@np.diag([-1,1,-1])
        pose[:3,3]=d.site(name+'_center').xpos
        tags[tag]=pose
    pose,_=carton_pose_from_short_flaps(tags)
    np.testing.assert_allclose(pose[:3,3],d.body('carton').xpos,atol=1e-12)
    np.testing.assert_allclose(pose[:3,:3],d.body('carton').xmat.reshape(3,3),atol=1e-12)


def test_carton_motion_distinguishes_sliding_from_vertical_settling(scene_factory):
    import numpy as np
    from carton.folding_sim import FoldingSimulation
    mj,make=scene_factory;m=make(CartonMaterial());d=mj.MjData(m)
    mj.mj_kinematics(m,d)
    sim=object.__new__(FoldingSimulation);sim.model=m;sim.data=d
    sim.station=FoldingStation(.06,.15,.05)
    sim.box_origin=d.body('carton').xpos.copy()
    sim.box_rotation=d.body('carton').xmat.reshape(3,3).copy()
    sim.motion_stats={'max_translation_mm':0.,'max_rotation_degrees':0.,
                      'minimum_bottom_corner_table_clearance_mm':float('inf')}
    adr=m.joint('carton_free').qposadr[0]
    d.qpos[adr:adr+3]+=np.array([.003,.004,-.012])
    mj.mj_kinematics(m,d)
    measured=sim.measure_carton_motion()
    assert measured['translation_mm']==pytest.approx(13)
    assert measured['horizontal_translation_mm']==pytest.approx(5)
    assert measured['vertical_translation_mm']==pytest.approx(-12)
    assert sim.motion_stats['max_horizontal_translation_mm']==pytest.approx(5)
