"""Production-detector coverage of saved G4 station states, without actuation.

This is camera/marker visibility evidence, not hand-eye registration, target
accuracy or a deployable visual controller. Depth is aligned synthetic optical Z.
"""
from __future__ import annotations
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from PIL import Image

from farm.perception.gemma_tags import TagObserver


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def envelope(rgb,camera,*,stamp=1000.,seq=1):
    stream='saved-station-coverage-'+camera
    buf=io.BytesIO();Image.fromarray(rgb).save(buf,format='PNG');raw=buf.getvalue()
    meta=dict(camera_id=camera,stream_id=stream,seq=seq,sha256=hashlib.sha256(raw).hexdigest(),
              width=rgb.shape[1],height=rgb.shape[0],rgb_captured_at=stamp,live=True)
    frame=dict(**meta,mime_type='image/png',data_base64=base64.b64encode(raw).decode(),
               view=camera,captured_at=stamp,received_at=stamp,fresh=True)
    return dict(ok=True,result=dict(cameras={camera:meta}),images=[frame])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--state',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,default=719)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    source=dict(scene=str(a.scene),scene_sha256=sha(a.scene),state=str(a.state),state_sha256=sha(a.state),
                executed_source_sha256=sha(__file__),
                perception_sources={str(p):sha(p) for p in [Path(__file__).resolve().parents[1]/('farm/perception/'+name) for name in ['gemma_tags.py','tags.py','tag_geometry.py']]})
    xml=E.parse(a.scene).getroot()
    source['mesh_assets']={m.get('file'):sha(m.get('file')) for m in xml.findall('./asset/mesh') if m.get('file')}
    model=mujoco.MjModel.from_xml_path(str(a.scene));data=mujoco.MjData(model)
    state=np.load(a.state);data.qpos[:]=state['qpos'];data.qvel[:]=state['qvel'];data.ctrl[:]=state['ctrl']
    mujoco.mj_forward(model,data)
    width=int(model.vis.global_.offwidth);height=int(model.vis.global_.offheight)
    renderer=mujoco.Renderer(model,height=height,width=width);option=mujoco.MjvOption();option.geomgroup[3]=0
    rng=np.random.default_rng(a.seed);views=[]
    for ci in range(model.ncam):
        name=model.camera(ci).name;renderer.update_scene(data,camera=name,scene_option=option)
        rgb=renderer.render().copy();Image.fromarray(rgb).save(a.out/(name+'-rgb.png'))
        renderer.enable_depth_rendering();renderer.update_scene(data,camera=name,scene_option=option)
        truth=renderer.render().copy();renderer.disable_depth_rendering()
        depth=truth+rng.normal(0,.0008,truth.shape);depth[rng.random(depth.shape)<.25]=np.nan
        np.savez_compressed(a.out/(name+'-depth.npz'),aligned_depth_m=depth,noise_sigma_m=.0008,dropout_probability=.25,seed=a.seed)
        observer=TagObserver(clock=lambda:1000.)
        observed=observer.observe(envelope(rgb,name),[name],ids=[1,2,4,41],include_images=True)
        for im in observed.pop('images',[]):
            (a.out/(name+'-annotated.png')).write_bytes(base64.b64decode(im['data_base64']))
        stale=TagObserver(clock=lambda:1000.).observe(envelope(rgb,name,stamp=995.),[name],ids=[1,2,4,41],include_images=False)
        missing=TagObserver(clock=lambda:1000.).observe(envelope(np.zeros_like(rgb),name),[name],ids=[1,2,4,41],include_images=False)
        f=height/(2*np.tan(np.radians(model.cam_fovy[ci])/2))
        views.append(dict(camera=name,intrinsics=[[f,0,width/2],[0,f,height/2],[0,0,1]],
                          observed=observed,stale_control=stale,blank_missing_control=missing,
                          valid_depth_fraction=float(np.isfinite(depth).mean())))
        print(json.dumps(dict(camera=name,tags=observed['result']['observations'][name].get('tags',[]))),flush=True)
    renderer.close()
    assert source['scene_sha256']==sha(a.scene) and source['state_sha256']==sha(a.state),'Inputs changed during audit'
    assert all(sha(p)==h for p,h in source['mesh_assets'].items()),'Mesh assets changed during audit'
    report=dict(scope='rendered_visibility_only',source=source,views=views,simulated_state_only=True,
                hand_eye_registration=False,controller_targets_created=False,hardware_commands=0,
                limitations=['One saved station pose, no motion coverage',
                    'No 8/3 hand-eye calibration in this audit',
                    'Synthetic rectified camera and noise/dropout hypotheses, not measured camera distribution',
                    'Blank image tests missing detection; it does not prove arbitrary partial-occlusion handling'],
                artifact_hashes={x.name:sha(x) for x in a.out.iterdir() if x.is_file()})
    (a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
