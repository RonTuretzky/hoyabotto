"""Mathematical consistency only; physical joint-zero registration is separate."""
import math

import pytest

from farm.vendor.so101_kinematics import SO101Kinematics


@pytest.mark.parametrize("target", [(0.1629, 0.1131), (0.2, 0), (0.15, 0.1), (0.1, 0.2), (0.2, -0.03)])
def test_reachable_targets_roundtrip(target):
    kin = SO101Kinematics()
    assert kin.forward_kinematics(*kin.inverse_kinematics(*target)) == pytest.approx(target, abs=1e-9)


def test_far_target_is_rejected_without_projection():
    kin = SO101Kinematics()
    with pytest.raises(ValueError, match="outside the two-link workspace"):
        kin.inverse_kinematics(0.4, 0.1)
