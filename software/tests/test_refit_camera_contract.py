"""Actual pixel projection and inference boundary tests; no hardware access."""
import base64
import copy
import hashlib
import json
from types import SimpleNamespace

import cv2
import mujoco
import numpy as np
import pytest

from carton.refit_camera_contract import load_contract, render_policy_camera, save_contract
from carton.fold_policy_runner import ApiCameras, Refused, FoldPolicyRunner, RunnerConfig, Abort
from carton.oak_policy_camera import preprocess
from tools.fold_demos_to_lerobot import render_trial, set_render_pose


def point_scene():
    c = load_contract()
    k = np.array(c['policy']['intrinsics'])
    pixels = np.array([(160,120),(43.2,32.7),(290.3,204.1),(75.6,191.4)])
    geoms = ''
    for u, v in pixels:
        x, y, _ = np.linalg.inv(k) @ [u, v, 1]
        geoms += f'<geom type="sphere" pos="{x} {-y} -1" size=".009" rgba="1 0 0 1"/>'
    xml = f'''<mujoco><visual><headlight ambient="1 1 1" diffuse="0 0 0" specular="0 0 0"/></visual>
      <worldbody><camera name="front" fovy="45"/>{geoms}
      <body name="moving" pos="0 0 -1"><joint type="slide" axis="1 0 0"/>
      <geom type="box" size=".04 .03 .01" pos="0 -.15 0" rgba="0 1 0 1"/></body>
      <geom type="sphere" pos="0 0 -.5" size=".1" rgba="0 0 1 1" group="4"/>
      </worldbody></mujoco>'''
    return c, pixels, xml


def test_actual_renderer_integer_pixel_centers():
    c, pixels, xml = point_scene()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    with mujoco.Renderer(m, 240, 320) as renderer:
        rgb = render_policy_camera(renderer, d, 'front', contract=c)
        observed = []
        for u,v in pixels:
            yy,xx = np.mgrid[int(v)-8:int(v)+9, int(u)-8:int(u)+9]
            weights = rgb[yy,xx,0].astype(float)
            assert weights.sum() > 0
            observed.append([(xx*weights).sum()/weights.sum(), (yy*weights).sum()/weights.sum()])
        # Rasterized spheres have small perspective/tessellation bias. This bound
        # detects the native half-pixel error and erroneous factor-two frustum.
        np.testing.assert_allclose(observed, pixels, atol=.16, rtol=0)
    with mujoco.Renderer(m, 120, 160) as renderer:
        with pytest.raises(ValueError, match='size'):
            render_policy_camera(renderer, d, 'front', contract=c)


def test_conversion_evaluation_fake_and_preview_share_pixels(tmp_path, monkeypatch):
    from tools import eval_fold_policy as base
    from tools import eval_refit_fold_policy as audited
    from carton.fold_policy_fakes import MujocoFoldPlant
    from carton.folding_refit_preview import oak_policy_render
    c, _, xml = point_scene()
    run = tmp_path/'run'; run.mkdir()
    (run/'scene.xml').write_text(xml)
    save_contract(run,c)
    poses = np.array([[0.], [.15], [-.1]])
    np.savez(tmp_path/'demo.npz', qpos=poses)
    rendered = render_trial((str(tmp_path),2,240,320,{'front':'front'}))['front']
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
    monkeypatch.setattr(base,'CAMERAS',{'front':'front'})
    option = mujoco.MjvOption();option.geomgroup[3]=option.geomgroup[4]=0
    with mujoco.Renderer(m,240,320) as renderer:
        ep = SimpleNamespace(renderer=renderer,data=d,camera_contract=c,option=option)
        plant = SimpleNamespace(ep=ep,d=d)
        for i,q in enumerate(poses):
            set_render_pose(m,d,q)
            expected = render_policy_camera(renderer,d,'front',contract=c)
            for image in (rendered[i],oak_policy_render(SimpleNamespace(model=m,data=d),c),base.Episode.images(ep)['front'],
                          audited.AuditedEpisode.images(ep)['front'],MujocoFoldPlant.render(plant,'front')):
                np.testing.assert_array_equal(image,expected)


def payload():
    c = load_contract()
    yy,xx = np.mgrid[:780,:1040]
    rgb = np.stack([xx%256,yy%256,(xx+yy)%256],axis=-1).astype(np.uint8)
    ok,encoded = cv2.imencode('.png',rgb)
    assert ok
    raw=encoded.tobytes()
    meta=copy.deepcopy(c['source'])
    meta.update(width=1040,height=780,rgb_lens_position=79,stream_id='immutable-stream',seq=123,
                captured_at=10.,sha256=hashlib.sha256(raw).hexdigest())
    image={k:meta[k] for k in ('camera_id','stream_id','seq','captured_at','sha256')}
    image.update(data_base64=base64.b64encode(raw).decode(),projection='camera_pinhole_with_factory_distortion')
    return c,{'ok':True,'result':{'cameras':{'oak':meta},'camera_errors':{}},'images':[image]},rgb


def adapter(c,p):
    return ApiCameras(SimpleNamespace(call=lambda *args:p),{'front':'oak'},camera_contract=c)


def test_inference_adapter_and_tensor_preserve_pixels_and_freshness(monkeypatch):
    from farm.learning.infer import PolicyRunner
    import torch
    c,p,raw=payload()
    frame=adapter(c,p).frames()['front']
    np.testing.assert_array_equal(frame.rgb,preprocess(raw,p['result']['cameras']['oak'],c,input_color_order='BGR'))
    assert (frame.seq,frame.captured_at)==(123,10.)
    def no_resize(*args,**kwargs):
        raise AssertionError('Policy geometry must not be resized again')
    monkeypatch.setattr(torch.nn.functional,'interpolate',no_resize)
    policy=SimpleNamespace(_torch=torch)
    tensor=PolicyRunner._image_to_tensor(policy,'observation.images.front',frame.rgb,(240,320))
    np.testing.assert_allclose(tensor.permute(1,2,0).numpy(),frame.rgb.astype(np.float32)/255,atol=0,rtol=0)
    runner=SimpleNamespace(config=RunnerConfig(),camera_keys=('front',),_last_frames={},_stale_frame_ticks={})
    with pytest.raises(Abort,match='watchdog'):
        FoldPolicyRunner.check_frames(runner,{'front':frame},SimpleNamespace(clock_offset_s=0),now=12.)


@pytest.mark.parametrize('target,key,value',[
    ('image','seq',124),('image','sha256','bad'),('image','captured_at',11.),
    ('image','stream_id','different'),('image','camera_id','different'),
    ('image','projection','already_rectified'),('meta','config_sha256','old'),
    ('meta','rgb_lens_position',80),('meta','intrinsics',np.eye(3).tolist()),
    ('meta','distortion_coefficients',[0.]*14),('meta','width',640)])
def test_inference_rejects_mismatched_source(target,key,value):
    c,p,_=payload()
    obj=p['images'][0] if target=='image' else p['result']['cameras']['oak']
    obj[key]=value
    with pytest.raises(Refused):
        adapter(c,p).frames()


def test_checkpoint_sidecars_saved_before_upload(tmp_path,monkeypatch):
    from tools import train_refit_ddp as train
    def original(*args,**kwargs):
        dest=kwargs['checkpoint_dir']/'pretrained_model'
        dest.mkdir(parents=True)
        (dest/'config.json').write_text('{}')
    monkeypatch.setattr(train.lerobot_train,'save_checkpoint',original)
    train.install_camera_checkpoint_hook()
    train.lerobot_train.save_checkpoint(checkpoint_dir=tmp_path/'checkpoint')
    dest=tmp_path/'checkpoint/pretrained_model'
    assert load_contract(dest/'oak-policy-camera.json')==load_contract()
    meta=json.loads((dest/'camera-contract-metadata.json').read_text())
    assert meta['contract_sha256']==load_contract()['contract_sha256']
    assert meta['physical_registration_verified'] is False
