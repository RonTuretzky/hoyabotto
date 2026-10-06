import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from carton.geometry import Box
from carton.folding_hinge_tags import short_flap_tag_mount, carton_pose_from_short_flaps
from carton.folding_markers import carton_pose_from_tags


def observed_tags(box_pose, angles):
    box = Box()
    tags = {}
    for tag, side, sign, angle in zip((11, 12), ('left', 'right'), (-1, 1), angles):
        hinge = np.eye(4)
        hinge[:3, 3] = [sign * box.length / 2, 0, box.height]
        hinge[:3, :3] = Rotation.from_rotvec([0, -sign * angle, 0]).as_matrix()
        tags[tag] = box_pose @ hinge @ short_flap_tag_mount(side)
    return tags


def test_recovers_moving_tilted_box_with_independent_flap_angles():
    rng = np.random.default_rng(4)
    for _ in range(50):
        actual = np.eye(4)
        actual[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        actual[:3, 3] = rng.uniform(-.5, .5, 3)
        tags = observed_tags(actual, rng.uniform(-1.5, 2.5, 2))
        recovered, evidence = carton_pose_from_tags(tags)
        np.testing.assert_allclose(recovered, actual, atol=1e-12)
        assert evidence['visible_ids'] == [11, 12]
        assert not evidence['physical_mounts_measured']


def test_missing_second_tag_cannot_be_replaced_with_old_pose():
    tags = observed_tags(np.eye(4), [.4, 1.3])
    with pytest.raises(ValueError, match='Both fresh'):
        carton_pose_from_short_flaps({11: tags[11]})
    with pytest.raises(ValueError, match='Fresh carton marker'):
        carton_pose_from_tags({11: tags[11]})


def test_wrong_hinge_spacing_is_rejected():
    tags = observed_tags(np.eye(4), [.4, 1.3])
    tags[12][0, 3] += .025
    with pytest.raises(ValueError, match='spacing'):
        carton_pose_from_short_flaps(tags)


def test_inconsistent_hinge_axes_are_rejected():
    tags = observed_tags(np.eye(4), [.4, 1.3])
    tags[12][:3, :3] = Rotation.from_euler('x', 20, degrees=True).as_matrix() @ tags[12][:3, :3]
    with pytest.raises(ValueError):
        carton_pose_from_short_flaps(tags)


def test_nonrigid_observation_is_rejected():
    tags = observed_tags(np.eye(4), [.4, 1.3])
    tags[11][0, 0] += .1
    with pytest.raises(ValueError, match='rigid'):
        carton_pose_from_short_flaps(tags)
