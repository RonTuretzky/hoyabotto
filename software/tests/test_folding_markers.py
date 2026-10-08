"""Fresh observed rigid-body markers survive occlusion, not stale cached poses."""
import numpy as np
import pytest
from carton.folding_markers import box_marker_poses,carton_pose_from_tags


def test_either_side_marker_recovers_a_moving_box_when_front_is_hidden():
    box=np.eye(4);theta=.37
    box[:3,:3]=[[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]
    box[:3,3]=[.06,-.02,.003]
    mounts=box_marker_poses()
    for tag_id in (10,21,22):
        pose,info=carton_pose_from_tags({tag_id:box@mounts[tag_id]})
        np.testing.assert_allclose(pose,box,atol=1e-12)
        assert info['selected_id']==tag_id
    with pytest.raises(ValueError,match='Fresh carton'):
        carton_pose_from_tags({2:np.eye(4)})


def test_multiple_markers_must_agree_and_best_depth_support_selects_pose():
    mounts=box_marker_poses();poses={tag_id:p.copy() for tag_id,p in mounts.items()}
    pose,info=carton_pose_from_tags(poses,{10:{'valid_depth_pixels':60},21:{'valid_depth_pixels':200}})
    np.testing.assert_allclose(pose,np.eye(4),atol=1e-12)
    assert info['selected_id']==21
    poses[21][0,3]+=.025
    with pytest.raises(ValueError,match='disagree'):
        carton_pose_from_tags(poses)
