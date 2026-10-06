"""Metric sensing and refusal behavior; independent of simulator assets."""
import cv2
import numpy as np
import pytest

from carton.folding_vision import RGBDTagObserver,depth_tag_pose
from carton.servo.tag_kit import marker_grid
from farm.perception.tag_geometry import square_points


def test_tilted_tag_with_depth_noise_and_missing_pixels():
    k=np.array([[600.,0,320],[0,600.,240],[0,0,1]])
    rotation,_=cv2.Rodrigues(np.array([.3,-.2,.1]))
    center=np.array([.015,-.025,.65])
    points=square_points(.060)@rotation.T+center
    corners=(points[:,:2]/points[:,2,None])*600+[320,240]
    yy,xx=np.indices((480,640))
    rays=np.stack([(xx-320)/600,(yy-240)/600,np.ones_like(xx)],axis=-1)
    normal=rotation[:,2]
    depth=(normal@center)/np.sum(rays*normal,axis=-1)
    rng=np.random.default_rng(24)
    depth+=rng.normal(0,.001,depth.shape)
    depth[rng.random(depth.shape)<.4]=0
    pose,quality=depth_tag_pose(corners,depth,k,.060)
    assert np.linalg.norm(pose[:3,3]-center)<.0015
    assert np.degrees(np.arccos(np.clip((np.trace(pose[:3,:3].T@rotation)-1)/2,-1,1)))<1.5
    assert quality['valid_depth_pixels']>100
    with pytest.raises(ValueError,match='tag size'):
        depth_tag_pose(corners,depth,k,.080)


def anchor_frame():
    rgb=np.full((480,640,3),255,np.uint8)
    for row,cells in enumerate(marker_grid(1)):
        for col,value in enumerate(cells):
            if value==0:rgb[208+8*row:216+8*row,288+8*col:296+8*col]=0
    return rgb,np.full((480,640),.6),np.array([[640.,0,320],[0,640.,240],[0,0,1]])


def test_registration_requires_fresh_anchor_and_synchronized_depth():
    rgb,depth,k=anchor_frame()
    observer=RGBDTagObserver(np.eye(4))
    assert 1 in observer.observe(rgb,depth,k,seq=1,timestamp=1.,depth_timestamp=1.)
    with pytest.raises(ValueError,match='Stale'):
        observer.observe(rgb,depth,k,seq=1,timestamp=2.,depth_timestamp=2.)
    with pytest.raises(ValueError,match='Unsynchronized'):
        observer.observe(rgb,depth,k,seq=2,timestamp=2.,depth_timestamp=2.1)
    with pytest.raises(ValueError,match='Fresh table tag'):
        observer.observe(np.zeros_like(rgb),depth,k,seq=2,timestamp=2.,depth_timestamp=2.)
    with pytest.raises(ValueError,match='Fresh table tag'):
        observer.observe(rgb,np.zeros_like(depth),k,seq=2,timestamp=2.,depth_timestamp=2.)


@pytest.mark.parametrize('bad_k',[np.eye(2),np.full((3,3),np.nan),np.zeros((3,3))])
def test_invalid_intrinsics_are_not_metric_measurements(bad_k):
    with pytest.raises(ValueError,match='intrinsics'):
        depth_tag_pose(np.array([[1,1],[20,1],[20,20],[1,20]]),np.ones((40,40)),bad_k,.06)


def redundant_anchor_frame(visible=(1,20),pixel_shift=0):
    rgb=np.full((480,640,3),255,np.uint8)
    for tag_id,cx in ((1,200),(20,440)):
        if tag_id not in visible:continue
        for row,cells in enumerate(marker_grid(tag_id)):
            for col,value in enumerate(cells):
                if value==0:
                    x=cx-32+8*col+pixel_shift
                    rgb[208+8*row:216+8*row,x:x+8]=0
    return rgb,np.full((480,640),.6),np.array([[640.,0,320],[0,640.,240],[0,0,1]])


def redundant_observer():
    # Known front-facing marker poses in the optical camera frame. The
    # synthetic camera/world transform is identity; corners decode at <1px.
    anchor=np.eye(4);anchor[:3,:3]=np.diag([-1.,-1.,1.]);anchor[:3,3]=[-.1125,0,.6]
    backup=anchor.copy();backup[0,3]=.1125
    return RGBDTagObserver(anchor,additional_anchors={20:backup},stationary_camera=True)


def test_stationary_camera_needs_both_initial_anchors_then_survives_occlusion():
    observer=redundant_observer()
    rgb,depth,k=redundant_anchor_frame((1,))
    with pytest.raises(ValueError,match='All declared table anchors'):
        observer.observe(rgb,depth,k,seq=1,timestamp=1.,depth_timestamp=1.)
    rgb,depth,k=redundant_anchor_frame()
    observer.observe(rgb,depth,k,seq=1,timestamp=1.,depth_timestamp=1.)
    assert np.linalg.norm(observer.world_from_camera[:3,3])<.0003
    frozen=observer.world_from_camera.copy()
    for seq,ids in enumerate(((20,),(1,)),start=2):
        rgb,depth,k=redundant_anchor_frame(ids)
        observer.observe(rgb,depth,k,seq=seq,timestamp=float(seq),depth_timestamp=float(seq))
        assert observer.history[-1]['anchor_ids']==list(ids)
        np.testing.assert_array_equal(observer.world_from_camera,frozen)
    rgb,depth,k=redundant_anchor_frame(())
    with pytest.raises(ValueError,match='Fresh table tag'):
        observer.observe(rgb,depth,k,seq=4,timestamp=4.,depth_timestamp=4.)


def test_stationary_camera_detects_movement_and_unknown_marker_is_not_an_anchor():
    observer=redundant_observer()
    rgb,depth,k=redundant_anchor_frame()
    observer.observe(rgb,depth,k,seq=1,timestamp=1.,depth_timestamp=1.)
    # A 20-pixel lateral movement corresponds to 18.75 mm at this depth.
    rgb,depth,k=redundant_anchor_frame((20,),pixel_shift=20)
    with pytest.raises(ValueError,match='over 6 mm'):
        observer.observe(rgb,depth,k,seq=2,timestamp=2.,depth_timestamp=2.)
    rgb,depth,k=redundant_anchor_frame((20,))
    with pytest.raises(ValueError,match='Fresh table tag'):
        RGBDTagObserver(np.eye(4)).observe(rgb,depth,k,seq=1,timestamp=1.,depth_timestamp=1.)


def top_down_cardboard(short_flaps=True,near_flap=False):
    from carton.geometry import Box
    b=Box();rgb=np.full((480,640,3),255,np.uint8);depth=np.zeros((480,640))
    k=np.array([[600.,0,320],[0,600.,240],[0,0,1]])
    yy,xx=np.indices(depth.shape)
    def panel(z,mask_fn):
        zcam=.8-z;x=(xx-320)*zcam/600;y=-(yy-240)*zcam/600
        mask=mask_fn(x,y);depth[mask]=zcam;rgb[mask]=[150,100,60]
    if short_flaps:
        panel(b.height,lambda x,y:(abs(y)<b.width/2-.004)&(abs(x)>b.length/2-b.flap)&(abs(x)<b.length/2))
    if near_flap:
        panel(b.height+.0035,lambda x,y:(abs(x)<b.length/2-.004)&(y>-b.width/2)&(y<-b.width/2+b.flap))
    world_from_camera=np.eye(4);world_from_camera[:3,:3]=np.diag([1.,-1.,-1.]);world_from_camera[:3,3]=[0,0,.8]
    return rgb,depth,k,world_from_camera,np.eye(4)


def test_depth_does_not_mistake_underlying_short_flaps_for_long_flap_closure():
    from carton.folding_vision import depth_flap_angles
    # Even an old near-flat prior cannot turn the wrong visible panel into
    # fresh long-flap evidence when the long flaps themselves are invisible.
    priors={'short_left':90.,'short_right':90.,'long_near':80.,'long_far':80.}
    observed=depth_flap_angles(*top_down_cardboard(),priors)
    assert {'short_left','short_right'}<=set(observed)
    assert 'long_near' not in observed and 'long_far' not in observed
    observed=depth_flap_angles(*top_down_cardboard(near_flap=True),priors)
    assert observed['long_near']['degrees']==pytest.approx(90.,abs=.1)
    assert 'long_far' not in observed
