import numpy as np
import pytest
import mujoco
from tools.front_camera_from_registration import registered_world_pose


def scene(x, y, z, quat='1 0 0 0'):
    m = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body name="right_base_link" pos="{x} {y} {z}" quat="{quat}"/>
      </worldbody></mujoco>''')
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    return m, d


def test_registered_camera_follows_spacing_height_and_setback_change():
    relative = np.eye(4); relative[:3, 3] = [.1, -.03, .38]
    a = registered_world_pose(relative, *scene(.11, -.33, .0291))
    b = registered_world_pose(relative, *scene(.1552, -.36, .01))
    np.testing.assert_allclose(b[0] - a[0], [.0452, -.03, -.0191])
    np.testing.assert_allclose(b[1], a[1]); np.testing.assert_allclose(b[2], a[2])
    np.testing.assert_allclose(a[1], [1, 0, 0]); np.testing.assert_allclose(a[2], [0, -1, 0])


def test_base_rotation_is_applied_before_optical_axis_conversion():
    relative = np.eye(4); relative[:3, 3] = [1, 0, .2]
    pos, x, y = registered_world_pose(relative, *scene(.15, -.3, .02, '.70710678 0 0 .70710678'))
    np.testing.assert_allclose(pos, [.15, .7, .22], atol=1e-7)
    np.testing.assert_allclose(x, [0, 1, 0], atol=1e-7)
    np.testing.assert_allclose(y, [1, 0, 0], atol=1e-7)


@pytest.mark.parametrize('bad', [np.zeros((4,4)), np.full((4,4), np.nan), np.diag([-1,1,1,1]), np.eye(3)])
def test_camera_registration_rejects_invalid_transform(bad):
    with pytest.raises(ValueError, match='rigid'):
        registered_world_pose(bad, *scene(0,0,0))
