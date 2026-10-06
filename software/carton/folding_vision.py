"""Pixel-only AprilTag plus aligned depth observations for the folding simulator.

No simulator import or access to object state. Table registration uses a declared
surveyed marker transform; depth is metric optical-axis Z, like oak_camera.py.
"""
from __future__ import annotations
import cv2
import numpy as np
from farm.perception.tags import detect_tags
from farm.perception.tag_geometry import square_points
from farm.status import Reading, Status
from carton.geometry import Box
from carton.folding_markers import BOX_MARKERS,BOX_TAG_SIZE

SIZES={1:.060,2:.040,4:.040,11:.035,12:.035,13:.035,14:.035,20:.060,
       **{tag_id:BOX_TAG_SIZE for tag_id in BOX_MARKERS}}


def depth_tag_pose(corners,depth,k,size):
    corners=np.asarray(corners,dtype=float)
    k=np.asarray(k,dtype=float)
    if corners.shape!=(4,2) or not np.isfinite(corners).all() or k.shape!=(3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:
        raise ValueError('Finite tag corners and matching pinhole intrinsics required')
    if not np.isfinite(size) or size<=0 or depth.ndim!=2:raise ValueError('Positive measured tag size and metric depth image required')
    center=corners.mean(axis=0)
    polygon=np.round(center+.8*(corners-center)).astype(np.int32)
    mask=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(mask,polygon,1)
    yy,xx=np.nonzero(mask & np.isfinite(depth) & (depth>.10) & (depth<2.))
    if len(xx)<30:raise ValueError('Insufficient valid aligned depth inside tag')
    z=depth[yy,xx]
    # Robust plane fit; isolated occluders must not determine the marker plane.
    xyz=np.column_stack([(xx-k[0,2])*z/k[0,0],(yy-k[1,2])*z/k[1,1],z])
    for _ in range(3):
        origin=np.mean(xyz,axis=0);_,_,vt=np.linalg.svd(xyz-origin,full_matrices=False)
        normal=vt[-1];err=np.abs(np.sum((xyz-origin)*normal,axis=1))
        keep=err<max(.0025,3*np.median(err))
        xyz=xyz[keep]
        if len(xyz)<30:raise ValueError('Depth plane is not supported')
    rays=np.column_stack([(corners[:,0]-k[0,2])/k[0,0],(corners[:,1]-k[1,2])/k[1,1],np.ones(4)])
    denom=rays@normal
    if np.min(np.abs(denom))<.1:raise ValueError('Marker is edge-on')
    points=rays*((origin@normal)/denom)[:,None]
    obj=square_points(size);target=points.mean(axis=0)
    u,_,vt=np.linalg.svd(obj.T@(points-target))
    rotation=vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
    residual=np.sqrt(np.mean(np.sum((obj@rotation.T+target-points)**2,axis=1)))
    if residual>.004:raise ValueError('Depth/corners disagree with declared tag size')
    pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=target
    return pose,{'valid_depth_pixels':len(xyz),'square_fit_rms_mm':float(residual*1000),'plane_rms_mm':float(np.std(np.sum((xyz-origin)*normal,axis=1))*1000)}


class RGBDTagObserver:
    def __init__(self,world_from_anchor,*,additional_anchors=None,stationary_camera=False):
        self.world_from_anchor=np.asarray(world_from_anchor)
        self.anchors={1:self.world_from_anchor,**(additional_anchors or {})}
        for tag_id,pose in self.anchors.items():
            pose=np.asarray(pose,dtype=float)
            if tag_id not in SIZES or pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('Calibrated finite anchor poses and known sizes required')
            if not np.allclose(pose[3],[0,0,0,1]) or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(pose[:3,:3])<.999:raise ValueError('Anchor pose must be a rigid proper transform')
            self.anchors[tag_id]=pose
        self.stationary_camera=stationary_camera
        self.world_from_camera=None
        self.sequence=-1
        self.history=[]

    def observe(self,rgb,depth,k,*,seq,timestamp,depth_timestamp):
        if seq<=self.sequence:raise ValueError('Stale RGB/depth sequence')
        if abs(timestamp-depth_timestamp)>.02:raise ValueError('Unsynchronized RGB/depth')
        if depth.shape!=rgb.shape[:2]:raise ValueError('Depth is not aligned to RGB')
        found=detect_tags(Reading(rgb,Status.OK,timestamp))
        if found.status is not Status.OK:raise ValueError(found.note)
        poses={};quality={};rejected={}
        for i,t in found.value.items():
            if i not in SIZES:continue
            try:poses[i],quality[i]=depth_tag_pose(t['corners'],depth,k,SIZES[i])
            except ValueError as exc:rejected[i]=str(exc)
        visible_anchors=sorted(set(poses)&set(self.anchors))
        if not visible_anchors:raise ValueError('Fresh table tag and aligned depth registration not observed')
        if self.world_from_camera is None and self.stationary_camera and set(visible_anchors)!=set(self.anchors):
            raise ValueError('All declared table anchors required for initial stationary-camera calibration')
        cam_points=[];world_points=[]
        for tag_id in visible_anchors:
            obj=square_points(SIZES[tag_id])
            cam_points.extend(obj@poses[tag_id][:3,:3].T+poses[tag_id][:3,3])
            world_points.extend(obj@self.anchors[tag_id][:3,:3].T+self.anchors[tag_id][:3,3])
        cam_points=np.asarray(cam_points);world_points=np.asarray(world_points)
        if not self.stationary_camera or self.world_from_camera is None:
            c,w=cam_points.mean(axis=0),world_points.mean(axis=0)
            u,_,vt=np.linalg.svd((cam_points-c).T@(world_points-w))
            rotation=vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
            candidate=np.eye(4);candidate[:3,:3]=rotation;candidate[:3,3]=w-rotation@c
        else:candidate=self.world_from_camera
        residual=float(np.sqrt(np.mean(np.sum((cam_points@candidate[:3,:3].T+candidate[:3,3]-world_points)**2,axis=1))))
        if residual>.006:raise ValueError('Fresh anchors disagree with stationary camera or surveyed geometry by over 6 mm')
        self.world_from_camera=candidate
        self.sequence=seq
        tags={i:self.world_from_camera@p for i,p in poses.items()}
        self.history.append({'seq':seq,'detected':sorted(tags),'rejected':rejected,'quality':quality,
                             'anchor_ids':visible_anchors,'anchor_fit_rms_mm':residual*1000,'stationary_camera':self.stationary_camera})
        return tags


def depth_flap_angles(rgb,depth,k,world_from_camera,world_from_box,priors=None):
    """Estimate visible cardboard planes around known hinges from RGB-D pixels.

    The CAD dimensions and marker-to-carton mount are supplied task geometry.
    The points, flap angles and carton pose are measured, not simulator states.
    Invisible/occluded surfaces are omitted rather than counted as closed.
    """
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    good=(hsv[:,:,0]>=8)&(hsv[:,:,0]<=32)&(hsv[:,:,1]>=45)&(hsv[:,:,1]<=185)&(depth>.1)&(depth<2)&np.isfinite(depth)
    yy,xx=np.nonzero(good)
    z=depth[yy,xx]
    cam=np.column_stack([(xx-k[0,2])*z/k[0,0],(yy-k[1,2])*z/k[1,1],z])
    box_from_camera=np.linalg.inv(world_from_box)@world_from_camera
    pts=np.einsum("ij,kj->ik",cam,box_from_camera[:3,:3])+box_from_camera[:3,3]
    b=Box()
    specs={'short_left':(0,-b.length/2,1,1,.07,b.height),'short_right':(0,b.length/2,-1,1,.07,b.height),
           # The centre strip lies between the folded short flaps. Sampling
           # above a short flap can confuse its surface with an occluded long
           # panel and can put the only depth patch beneath a working hand.
           'long_far':(1,b.width/2,-1,0,0.,b.height+.0035),'long_near':(1,-b.width/2,1,0,0.,b.height+.0035)}
    out={}
    for name,(axis,hinge,inward,along,center,height) in specs.items():
        horizontal=inward*(pts[:,axis]-hinge);vertical=pts[:,2]-height
        radius=np.hypot(horizontal,vertical)
        keep=(np.abs(pts[:,along]-center)<.028)&(radius>.045)&(radius<.138)&(vertical>-.012)&(horizontal>-.075)
        angles=np.degrees(np.arctan2(horizontal[keep],vertical[keep]))
        if priors is not None:angles=angles[np.abs(angles-priors.get(name,0.))<35]
        if len(angles)<35:continue
        hist,edges=np.histogram(angles,bins=np.arange(-45,102,3))
        peak=edges[np.argmax(hist)]+1.5
        inliers=angles[np.abs(angles-peak)<6]
        if len(inliers)<35 or len(inliers)<.45*len(angles):continue
        out[name]={'degrees':float(np.median(inliers)),'pixel_support':len(inliers),'median_deviation_deg':float(np.median(np.abs(inliers-np.median(inliers)))),'method':'aligned_depth_cardboard_plane'}
    return out
