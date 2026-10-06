"""Declared multi-face carton markers for the offline folding station.

Physical use requires these markers to be printed, mounted and measured.
This module contains geometry, never simulator state or an inferred pose.
"""
import numpy as np
from carton.geometry import Box

_box=Box()
BOX_MARKERS={
    10: ('box_tag', [0,-_box.width/2-.0018,_box.height/2], [1,0,0,0,0,1]),
    21: ('box_tag_left', [-_box.length/2-.0018,.04,_box.height/2], [0,-1,0,0,0,1]),
    22: ('box_tag_right', [_box.length/2+.0018,.04,_box.height/2], [0,1,0,0,0,1]),
}
BOX_TAG_SIZE=.045


def box_marker_poses():
    poses={}
    for tag_id,(_,position,axes) in BOX_MARKERS.items():
        u,v=np.asarray(axes[:3]),np.asarray(axes[3:]);normal=np.cross(u,v)
        pose=np.eye(4)
        # Same detector-frame convention used by the original near-wall ID 10.
        pose[:3,:3]=np.column_stack((-u,v,-normal))
        pose[:3,3]=np.asarray(position)+.0003*normal
        poses[tag_id]=pose
    return poses


def carton_pose_from_tags(tags,quality=None):
    """Use a fresh declared marker; reject disagreement instead of averaging it away."""
    mounts=box_marker_poses();visible=sorted(set(tags)&set(mounts))
    if not visible:raise ValueError('Fresh carton marker and aligned depth required')
    estimates={tag_id:tags[tag_id]@np.linalg.inv(mounts[tag_id]) for tag_id in visible}
    selected=max(visible,key=lambda tag_id:(quality or {}).get(tag_id,{}).get('valid_depth_pixels',0))
    pose=estimates[selected];translation=0.;angle=0.
    for candidate in estimates.values():
        translation=max(translation,float(np.linalg.norm(candidate[:3,3]-pose[:3,3])))
        rotation=pose[:3,:3].T@candidate[:3,:3]
        angle=max(angle,float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1)))))
    if translation>.012 or angle>8:
        raise ValueError('Fresh carton markers disagree with the declared rigid carton geometry')
    return pose,{'visible_ids':visible,'selected_id':selected,
                 'max_translation_disagreement_mm':translation*1000,'max_rotation_disagreement_degrees':angle}
