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
    def make(material):
        return build_scene(source,tmp_path/'scene',station=FoldingStation(.06,.15,.05),material=material)
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


def test_lower_table_friction_really_allows_unbolted_carton_to_slide(scene_factory):
    mj,make=scene_factory
    distances=[]
    for mu in (.05,.8):
        m=make(CartonMaterial(table_friction=mu));d=mj.MjData(m);body=m.body('carton').id
        for _ in range(200):mj.mj_step(m,d)
        start=d.xpos[body].copy();d.xfrc_applied[body,0]=2.
        for _ in range(150):mj.mj_step(m,d)
        distances.append(abs(d.xpos[body,0]-start[0]))
        contacts=[c for c in d.contact if {m.geom(c.geom1).name,m.geom(c.geom2).name}=={'table','bottom'}]
        assert contacts and all(c.friction[0]==pytest.approx(mu) for c in contacts)
    assert distances[0]>.01
    assert distances[1]<.002


def test_resistant_flaps_spring_open_without_a_hold(scene_factory):
    mj,make=scene_factory;m=make(CartonMaterial(contents_mass_kg=.96,hinge_stiffness=.04))
    d=mj.MjData(m)
    for name in ('short_left','short_right','long_far','long_near'):
        d.qpos[m.joint(name+'_hinge').qposadr[0]]=math.pi/2
    for _ in range(1000):mj.mj_step(m,d)
    assert math.degrees(d.qpos[m.joint('long_far_hinge').qposadr[0]])<70
