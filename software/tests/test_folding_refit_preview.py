import numpy as np
import pytest
from types import SimpleNamespace
mujoco=pytest.importorskip('mujoco')
from carton.folding_refit_preview import oak_render


def test_rectified_oak_render_projects_off_axis_point_to_measured_K():
    model=mujoco.MjModel.from_xml_string('''<mujoco>
    <visual><global offwidth="800" offheight="600"/></visual>
    <worldbody><camera name="front"/>
    <geom type="sphere" pos=".07 .025 -.5" size=".004" rgba="1 0 0 1"/>
    </worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    K=np.array([[504.894104,0,314.753784],[0,504.973846,192.592651],[0,0,1.]])
    sim=SimpleNamespace(model=model,data=data,option=mujoco.MjvOption())
    im=oak_render(sim,K)
    y,x=np.where((im[:,:,0]>50)&(im[:,:,0]>im[:,:,1]*2)&(im[:,:,0]>im[:,:,2]*2))
    expected=K@np.array([.07,-.025,.5]);expected=expected[:2]/expected[2]
    assert [x.mean(),y.mean()]==pytest.approx(expected,abs=1.)
