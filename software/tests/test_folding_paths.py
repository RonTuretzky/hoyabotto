"""Offline path guards: reject obstacle sweeps without moving live sim state."""
from types import SimpleNamespace
import numpy as np
import pytest
from carton.folding_sim import JOINTS, FoldingSimulation
from carton.folding_paths import JointPathPlanner
from carton.folding_paddle import PaddleFoldingSimulation


@pytest.fixture
def tool_scene():
    mj=pytest.importorskip('mujoco')
    joints=''.join(f'<joint name="right_{j}" axis="0 0 1" range="-3.14 3.14"/>' for j in JOINTS[:5])
    model=mj.MjModel.from_xml_string(f'''<mujoco><compiler angle="radian"/>
      <worldbody><geom name="table" type="sphere" size=".025" pos="0 .30 .40"/>
      <body name="right_base" pos="0 0 .40">{joints}<geom type="sphere" size=".01"/>
        <body name="right_gripper_link" pos=".20 0 0"><geom name="right_moving_jaw_part_0" type="sphere" size=".01"/></body>
      </body><body name="paddle" pos=".30 0 .40"><freejoint name="paddle_free"/>
        <geom name="right_paddle_contact_5" type="sphere" size=".02"/>
      </body></worldbody></mujoco>''')
    data=mj.MjData(model);mj.mj_forward(model,data)
    sim=SimpleNamespace(model=model,data=data,arm_indices={'right':list(range(5))},
        forbidden_contact=PaddleFoldingSimulation.forbidden_contact.__get__(object.__new__(PaddleFoldingSimulation)))
    return mj,sim


def test_held_tool_sweep_collides_and_never_changes_actual_state(tool_scene):
    mj,sim=tool_scene;before=sim.data.qpos.copy();planner=JointPathPlanner(sim,'right')
    assert planner.valid(np.zeros(5))
    q=np.array([np.pi/2,0,0,0,0])
    assert not planner.valid(q)
    assert set(planner.last_collision[:2])=={'table','right_paddle_contact_5'}
    np.testing.assert_allclose(planner.data.body('paddle').xpos,[0,.30,.40],atol=1e-12)
    np.testing.assert_array_equal(sim.data.qpos,before)
    with pytest.raises(ValueError,match='goal collides'):planner.plan(q)


def test_path_edge_checks_intermediate_tool_collision(tool_scene):
    _,sim=tool_scene;planner=JointPathPlanner(sim,'right')
    end=np.array([3.,0,0,0,0])
    assert planner.valid(np.zeros(5)) and planner.valid(end)
    assert not planner.edge(np.zeros(5),end)


def test_geometric_planner_contacts_match_full_position_pipeline(tool_scene):
    mj,sim=tool_scene
    planner=JointPathPlanner(sim,'right',clearance=.006)
    before=sim.data.qpos.copy()
    for angle in np.linspace(0,np.pi,13):
        planner.valid(np.array([angle,0,0,0,0]))
        full=mj.MjData(planner.model)
        full.qpos[:]=planner.data.qpos
        mj.mj_fwdPosition(planner.model,full)
        def contacts(data):
            return np.array([[c.geom1,c.geom2,c.dist,*c.pos,*c.frame]
                             for c in data.contact]).reshape(-1,15)
        np.testing.assert_allclose(contacts(planner.data),contacts(full),rtol=0,atol=1e-12)
    np.testing.assert_array_equal(sim.data.qpos,before)


def test_planning_clearance_rejects_near_miss_without_inflating_physics(tool_scene):
    mj,sim=tool_scene
    # Put the obstacle 4 mm beyond the held tool's contact surface.
    sim.model.geom_pos[sim.model.geom('table').id]=[.30,.049,.40]
    mj.mj_forward(sim.model,sim.data)
    before=sim.model.geom_margin.copy();positions=sim.data.qpos.copy()
    assert JointPathPlanner(sim,'right').valid(np.zeros(5))
    cautious=JointPathPlanner(sim,'right',clearance=.006)
    assert not cautious.valid(np.zeros(5))
    assert set(cautious.last_collision[:2])=={'table','right_paddle_contact_5'}
    np.testing.assert_array_equal(sim.model.geom_margin,before)
    np.testing.assert_array_equal(sim.data.qpos,positions)


@pytest.mark.parametrize('clearance',[-.001,.021,np.nan,np.inf])
def test_invalid_clearance_rejected(tool_scene,clearance):
    _,sim=tool_scene
    with pytest.raises(ValueError,match='clearance'):
        JointPathPlanner(sim,'right',clearance=clearance)


@pytest.mark.parametrize('q',[[np.nan,0,0,0,0],[4,0,0,0,0],[0,0]])
def test_invalid_joint_candidates_rejected(tool_scene,q):
    _,sim=tool_scene;assert not JointPathPlanner(sim,'right').valid(q)


def test_joint_executor_rejects_out_of_range_before_actuation():
    mj=pytest.importorskip('mujoco')
    joints=''.join(f'<joint name="left_{j}" axis="0 0 1" range="-1 1"/>' for j in JOINTS[:5])
    model=mj.MjModel.from_xml_string(f'<mujoco><compiler angle="radian"/><worldbody><body>{joints}<geom type="sphere" size=".01"/></body></worldbody></mujoco>')
    sim=object.__new__(FoldingSimulation);sim.model=model;sim.data=mj.MjData(model)
    before=sim.data.qpos.copy()
    with pytest.raises(ValueError,match='original model limits'):
        sim.move({},joint_targets={'left':[1.1,0,0,0,0]})
    np.testing.assert_array_equal(sim.data.qpos,before)
    with pytest.raises(ValueError,match='joint or Cartesian'):
        sim.move({'left':[0,0,0]},joint_targets={'left':[0,0,0,0,0]})


def _contact_probe(z):
    """Real signed-distance contacts without depending on downloaded arm meshes."""
    import mujoco
    from types import SimpleNamespace
    from carton.folding_sim import JOINTS,FoldingSimulation
    joints=''.join(f'<joint name="left_{name}" axis="0 1 0" range="-2 2" armature=".01"/>' for name in JOINTS)
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><compiler angle="radian"/>
      <worldbody><geom name="short_left_cardboard" type="box" size=".1 .1 .0015"/>
      <body name="left_gripper_link" pos="0 0 {z}">{joints}
        <geom name="left_moving_jaw_probe" type="sphere" size=".01" mass=".1"/>
      </body></worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    sim=SimpleNamespace(model=model,data=data,arm_indices={'left':[model.joint('left_'+j).qposadr[0] for j in JOINTS]})
    sim.forbidden_contact=lambda a,b:FoldingSimulation.forbidden_contact(sim,a,b)
    return sim


def test_allowed_flap_contact_cannot_hide_deep_penetration():
    from carton.folding_paths import JointPathPlanner
    sim=_contact_probe(.008)
    planner=JointPathPlanner(sim,'left',allowed_flaps=('short_left_cardboard',),clearance=.006)
    assert not planner.valid(np.zeros(5))
    assert planner.last_collision[2]<-.003


def test_allowed_shallow_contact_retains_original_penetration_bound():
    from carton.folding_paths import JointPathPlanner
    sim=_contact_probe(.011)
    planner=JointPathPlanner(sim,'left',allowed_flaps=('short_left_cardboard',),clearance=.006)
    assert planner.valid(np.zeros(5))
    assert not JointPathPlanner(sim,'left',clearance=.006).valid(np.zeros(5))


def test_initialization_rejects_contact_that_is_permitted_later():
    from carton.folding_sim import FoldingSimulation
    sim=_contact_probe(.011)
    # Later fingertip/flap contact is intentional; an initial overlap is not.
    assert not sim.forbidden_contact('left_moving_jaw_probe','short_left_cardboard')
    with pytest.raises(ValueError,match='Initial robot pose'):
        FoldingSimulation.validate_initial_robot_clearance(sim)
    sim=_contact_probe(.013)
    FoldingSimulation.validate_initial_robot_clearance(sim)


def test_dynamic_contact_stops_at_first_excessive_flap_penetration(tmp_path):
    from carton.folding_sim import FoldingSimulation,FLAPS
    sim=_contact_probe(.003)
    sim.applied_contact_path=tmp_path/'applied-contact-steps.jsonl.gz'
    sim.events=[];sim.stats={'max_bad_penetration_mm':0.}
    sim.truth_angles=lambda:dict.fromkeys(FLAPS,0.)
    sim.measure_carton_motion=lambda:{}
    sim.step_diagnostic=lambda:None
    sim.actual_control_position=lambda side:sim.data.geom('left_moving_jaw_probe').xpos
    event=FoldingSimulation.move(sim,{},.1,capture=False)
    assert event['stopped_early']
    assert event['duration_s']==pytest.approx(sim.model.opt.timestep)
    assert event['max_robot_flap_penetration_mm']>8
    assert event['bad_penetration_mm']==0  # This was an intended contact, still bounded.
    assert event['step_error']=='Robot/flap penetration exceeded 1 mm'
