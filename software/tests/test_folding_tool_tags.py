"""Fresh paddle observations must not silently become a perfect rigid grip."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from carton.folding_tool_tags import tool_tag_mounts,paddle_pose_from_tags


def test_either_tool_face_recovers_pose_without_cached_fallback():
    pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[.2,-.4,.7]).as_matrix()
    pose[:3,3]=[.1,-.2,.3]
    for tag_id,mount in tool_tag_mounts().items():
        reading=paddle_pose_from_tags({tag_id:pose@mount})
        np.testing.assert_allclose(reading['world_from_paddle'],pose,atol=1e-12)
        assert not reading['physical_mount_verified']
    assert paddle_pose_from_tags({2:pose}) is None


def test_disagreeing_tool_faces_rejected_instead_of_hiding_slip():
    tags=tool_tag_mounts()
    reading=paddle_pose_from_tags(tags,{23:{'valid_depth_pixels':200}})
    assert reading['tag_id']==23
    tags[3][0,3]+=.02
    with pytest.raises(ValueError,match='disagree'):
        paddle_pose_from_tags(tags)
    tags=tool_tag_mounts();tags[3][:3,:3]=Rotation.from_euler('x',12,degrees=True).as_matrix()@tags[3][:3,:3]
    with pytest.raises(ValueError,match='disagree'):
        paddle_pose_from_tags(tags)


@pytest.mark.parametrize('bad',[np.full((4,4),np.nan),np.eye(3),np.diag([-1,1,1,1])])
def test_bad_tool_transform_rejected(bad):
    with pytest.raises(ValueError):paddle_pose_from_tags({3:bad})
