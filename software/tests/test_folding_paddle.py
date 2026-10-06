"""Physical invariants for the offline tool comparison, not hardware tests."""
import math
import numpy as np
import pytest
from test_folding_material import scene_factory
from carton.folding_material import CartonMaterial
from carton.folding_sim import FoldingSimulation,JOINTS,FLAPS
from carton.folding_paddle import (PaddleSpec,PaddleFoldingSimulation,
    PaddleFoldingController,GRIP_ROTATION)


def test_paddle_is_passive_and_falls_without_jaw_support(scene_factory):
    mj,make=scene_factory;m=make(CartonMaterial(),paddle=PaddleSpec())
    assert m.joint('paddle_free').type==mj.mjtJoint.mjJNT_FREE
    assert m.body_mass[m.body('paddle').id]==pytest.approx(.03)
    assert m.nu==12 and m.neq==0
    d=mj.MjData(m);mj.mj_forward(m,d);z=d.body('paddle').xpos[2]
    for _ in range(100):mj.mj_step(m,d)
    assert z-d.body('paddle').xpos[2]>.15


def test_ideal_rigid_diagnostic_is_explicit_and_has_no_free_joint(scene_factory):
    mj,make=scene_factory;spec=PaddleSpec(attachment='rigid-diagnostic')
    m=make(CartonMaterial(),paddle=spec)
    assert mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'paddle_free')==-1
    assert m.body_parentid[m.body('paddle').id]==m.body('right_gripper_link').id
    assert 'DIAGNOSTIC' in spec.report()['mounting']


def test_only_intended_tool_contacts_are_allowed():
    sim=object.__new__(PaddleFoldingSimulation)
    assert not sim.forbidden_contact('right_paddle_contact_0','right_moving_jaw_part_0')
    assert not sim.forbidden_contact('short_right_cardboard','right_paddle_contact_5')
    for obstacle in ('table','bottom','contents','wall_left','cart_tray','left_moving_jaw_part_0','right_upper_arm_link_geom_1'):
        assert sim.forbidden_contact('right_paddle_contact_5',obstacle)
        assert sim.forbidden_contact(obstacle,'right_paddle_contact_5')


def test_blade_face_rotates_with_passive_panel_normal():
    for theta in (0,math.pi/4,math.pi/2):
        normal=np.array([math.cos(theta),0,math.sin(theta)])
        radial=np.array([-math.sin(theta),0,math.cos(theta)])
        o=PaddleFoldingController.blade_orientation(normal,radial)
        np.testing.assert_allclose(o['direction'],normal,atol=1e-12)
        np.testing.assert_allclose(o['local_axis'],GRIP_ROTATION[:,2],atol=1e-12)
        assert 'tangent' not in o  # Do not overconstrain the five-axis arm.


def test_observed_tcp_does_not_move_slip_reference_or_change_physics(scene_factory):
    mj,make=scene_factory;m=make(CartonMaterial(),paddle=PaddleSpec(grasp_x_m=.09))
    original=m.site('right_paddle_target').pos.copy()
    m.site('right_paddle_observed_target').pos[:]+=[.01,0,0]
    np.testing.assert_array_equal(m.site('right_paddle_target').pos,original)
    assert m.nu==12 and m.neq==0
    assert m.joint('paddle_free').type==mj.mjtJoint.mjJNT_FREE
    assert m.body_subtreemass[m.body('paddle').id]==pytest.approx(.03)
    # Marker paper and pixels must not add collision support or ballast.
    for i in range(m.ngeom):
        if m.geom(i).name.startswith(('paddle_tag_front','paddle_tag_back')):
            assert m.geom_contype[i]==m.geom_conaffinity[i]==0


@pytest.mark.parametrize('grasp',[np.nan,.009,.191])
def test_out_of_paddle_grasp_rejected(grasp):
    with pytest.raises(ValueError,match='grasp'):PaddleSpec(grasp_x_m=grasp)


def test_motion_report_uses_tool_tangent_axis_instead_of_gripper_x(scene_factory):
    mj,make=scene_factory;m=make(CartonMaterial())
    sim=object.__new__(FoldingSimulation);sim.model=m;sim.data=mj.MjData(m)
    mj.mj_forward(m,sim.data)
    sim.arm_indices={'right':[m.joint('right_'+j).qposadr[0] for j in JOINTS]}
    sim.control_sites={'right':'right_tip'};sim.events=[];sim.stats={'max_bad_penetration_mm':0}
    # This test isolates telemetry; the physical motion/IK guards are tested separately.
    sim.ik=lambda *_:(np.zeros(5),0.)
    sim.forbidden_contact=lambda *_:False
    sim.measure_carton_motion=lambda:{}
    sim.truth_angles=lambda:dict.fromkeys(FLAPS,0.)
    point=sim.data.site('right_tip').xpos.copy()
    tangent=sim.data.body('right_gripper_link').xmat.reshape(3,3)[:,1].copy()
    event=sim.move({'right':point},seconds=.002,capture=False,
                   orientation={'direction':[0,0,1],'tangent':tangent,'local_tangent':[0,1,0]})
    assert event['orientation_error_degrees']['right']['tangent']==pytest.approx(0.,abs=1e-5)


@pytest.mark.parametrize('kw',[dict(mass=0),dict(mass=math.nan),dict(friction=-1),dict(attachment='weld-secretly')])
def test_bad_paddle_parameters_rejected(kw):
    with pytest.raises(ValueError):PaddleSpec(**kw)
