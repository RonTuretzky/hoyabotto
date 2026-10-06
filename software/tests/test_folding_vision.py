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
