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
