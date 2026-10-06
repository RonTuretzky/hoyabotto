"""Declared simulated paddle tags; physical sizes and mounts need calibration."""
import numpy as np
from farm.kinematics.lerobot import transform,pose_error

PADDLE_TAG_SIZE=.040
PADDLE_TAGS={
    3: ('paddle_tag_front',[.135,0,.0065],[1,0,0,0,1,0]),
    23: ('paddle_tag_back',[.135,0,-.0005],[1,0,0,0,-1,0]),
}


def tool_tag_mounts():
    mounts={}
    for tag_id,(_,p,axes) in PADDLE_TAGS.items():
        u,v=np.array(axes[:3]),np.array(axes[3:]);normal=np.cross(u,v)
        pose=np.eye(4);pose[:3,:3]=np.column_stack((-u,v,-normal))
        pose[:3,3]=np.array(p)+.0003*normal;mounts[tag_id]=pose
    return mounts


def paddle_pose_from_tags(tags,quality=None):
    visible=sorted(set(tags)&set(PADDLE_TAGS))
    if not visible:return None
    mounts=tool_tag_mounts()
    selected=max(visible,key=lambda t:(quality or {}).get(t,{}).get('valid_depth_pixels',0))
    estimates={tag_id:transform(tags[tag_id])@np.linalg.inv(mounts[tag_id]) for tag_id in visible}
    pose=estimates[selected]
    disagreement=[pose_error(candidate,pose) for candidate in estimates.values()]
    translation=max(v[0] for v in disagreement);angle=max(v[1] for v in disagreement)
    if translation>.012 or angle>8:
        raise ValueError('Fresh paddle markers disagree with the declared rigid tool geometry')
    return {'world_from_paddle':pose.tolist(),'tag_id':selected,'visible_ids':visible,
            'max_translation_disagreement_mm':translation*1000,'max_rotation_disagreement_degrees':angle,
            'source':'fresh_tag_and_aligned_depth','physical_mount_verified':False}
