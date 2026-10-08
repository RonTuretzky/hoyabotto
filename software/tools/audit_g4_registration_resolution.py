#!/usr/bin/env python3
"""Sensor-resolution sensitivity on previously executed registration states.

States and encoder brackets come from an independently replayed actuator run.
Re-rendering those states is a privileged sensor diagnostic, not new robot
motion or controller evidence. Eight train and three validation poses stay fixed.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as E

import mujoco
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def load(path):
    return json.loads(path.read_text())


def pixel_refine(fit, dataset, captures, camera):
    """Separate diagnostic; same fixed algorithm for every resolution."""
    import cv2
    from scipy.optimize import least_squares
    from farm.perception.tag_geometry import square_points
    rows = []
    for entry, sample in zip(captures, dataset['samples']):
        observed = entry['observation']['result']['observations'][camera]
        tag = next(t for t in observed['tags'] if t['tag_id']==4 and t['status']=='DETECTED')
        rows.append(dict(split=sample['split'], bg=np.asarray(sample['base_from_gripper']),
            corners=np.asarray(tag['corners_px']),
            k=np.asarray(observed['pose_3d']['camera_calibration']['intrinsics'])))
    def pack(t):
        return np.r_[cv2.Rodrigues(t[:3,:3])[0].ravel(),t[:3,3]]
    def unpack(x):
        t=np.eye(4);t[:3,:3]=cv2.Rodrigues(x[:3])[0];t[:3,3]=x[3:];return t
    points=square_points(.04)
    def residual(x,selected):
        cb=np.linalg.inv(unpack(x[:6]));gt=unpack(x[6:]);result=[]
        for row in selected:
            ct=cb@row['bg']@gt
            if np.min((points@ct[:3,:3].T+ct[:3,3])[:,2])<=.02:
                raise ValueError('Fitted corner goes behind camera')
            projected,_=cv2.projectPoints(points,cv2.Rodrigues(ct[:3,:3])[0],ct[:3,3],row['k'],np.zeros(5))
            result.extend((projected[:,0]-row['corners']).ravel())
        return np.asarray(result)
    start=np.r_[pack(np.asarray(fit['base_from_camera'])),pack(np.asarray(fit['gripper_from_tag']))]
    train=[r for r in rows if r['split']=='train'];validation=[r for r in rows if r['split']=='validation']
    sol=least_squares(lambda x:residual(x,train),start,max_nfev=500,ftol=1e-12,xtol=1e-12,gtol=1e-12,x_scale='jac')
    metrics={}
    for label,selected in [('train',train),('validation',validation)]:
        metrics[label]=dict(count=len(selected),
            initial_pixel_rms=float(np.sqrt(np.mean(np.sum(residual(start,selected).reshape(-1,2)**2,axis=1)))),
            refined_pixel_rms=float(np.sqrt(np.mean(np.sum(residual(sol.x,selected).reshape(-1,2)**2,axis=1)))))
    return dict(production_fitter=False,truth_used_for_optimization=False,
        optimizer_success=bool(sol.success),optimizer_message=sol.message,evaluations=sol.nfev,
        base_from_camera=unpack(sol.x[:6]).tolist(),gripper_from_tag=unpack(sol.x[6:]).tolist(),residuals=metrics)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--registration',type=Path,required=True)
    p.add_argument('--brackets',type=Path,required=True)
    p.add_argument('--physics-audit',type=Path,required=True)
    p.add_argument('--capture-audit',type=Path,required=True)
    p.add_argument('--widths',default='1280,1920,2560')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    widths=[int(w) for w in a.widths.split(',')]
    if len(widths)!=len(set(widths)) or any(w<640 or w>2560 or w%4 for w in widths):
        raise ValueError('Unique widths640..2560 divisible by4 required')
    manifest=load(a.registration/'launch-manifest.json');source=Path(manifest['executed_source_root'])
    if not all(sha(source/name)==digest for name,digest in manifest['hashes'].items()):
        raise ValueError('Frozen calibration source changed')
    sys.path.insert(0,str(source))
    from farm.perception.gemma_tags import TagObserver
    from farm.perception.tag_sampling import stationary_sample
    from farm.kinematics.tag_registration import assemble_dataset,fit_registration
    from farm.kinematics.lerobot import pose_error
    from planter.g4_station_vision import capture,geometry_for_cameras
    from tools.diagnose_g4_registration import camera_truth_in_base
    batch=load(a.registration/'batch.json');job=batch['results'][0]
    command=job['command'];model_directory=Path(command[command.index('--model-directory')+1])
    run=Path(job['out']) if 'out' in job else Path(command[command.index('--out')+1])
    invocation=load(run/'invocation.json');plan=load(run/'plan.json')
    if invocation['arm']!='left':raise ValueError('Predeclared refinement expects left fixed tag4')
    if not load(a.physics_audit)['physics_replay_verified'] or not load(a.capture_audit)['all_capture_checks_passed']:
        raise ValueError('Original physics and captures must already be independently verified')
    if not all(sha(path)==digest for path,digest in invocation['source_hashes'].items()):
        raise ValueError('Original calibration assets changed')
    brackets=load(a.brackets)
    inputs=[a.registration/'launch-manifest.json',a.registration/'batch.json',
        a.physics_audit,a.capture_audit,a.brackets,run/'invocation.json',run/'plan.json',
        run/'scene.xml',run/'fit.json',Path(__file__),*sorted(run.glob('pose-*/*'))]
    hashes={str(path.resolve()):sha(path) for path in inputs}
    saved_captures=[load(d/'capture.json') for d in sorted(run.glob('pose-*'))]
    bindings=[load(d/'depth-binding.json') for d in sorted(run.glob('pose-*'))]
    save(a.out/'predeclared.json',dict(scope=__doc__,argv=sys.argv,widths=widths,
        source_hashes=hashes,frozen_source_root=str(source),frozen_source_hashes=manifest['hashes'],
        change='Only render resolution and matching intrinsics change. Camera pose, FOV, source executed states, encoder brackets,8/3 split, marker sizes and fitting algorithms stay fixed.',
        depth_noise_sigma_m=.0008,depth_dropout_probability=.25,
        truth_used_for_optimization=False,hardware_commands=0,registration_installed=False,
        new_robot_trajectories=0,full_planter_success=False))
    results=[]
    for width in widths:
        out=a.out/f'width-{width}';out.mkdir()
        tree=E.parse(run/'scene.xml');v=tree.find('./visual/global')
        v.set('offwidth',str(width));v.set('offheight',str(width*3//4))
        tree.write(out/'scene.xml',encoding='unicode')
        model=mujoco.MjModel.from_xml_path(str(out/'scene.xml'));data=mujoco.MjData(model)
        renderer=mujoco.Renderer(model,height=width*3//4,width=width)
        option=mujoco.MjvOption();option.geomgroup[3]=0
        rng=np.random.default_rng(invocation['seed']);now=[0.]
        observer=TagObserver(clock=lambda:now[0],geometry=geometry_for_cameras([invocation['camera']]))
        captures=[];row_result=dict(width=width,height=width*3//4,status='INCOMPLETE',fit=None)
        try:
            for i,(old,binding) in enumerate(zip(saved_captures,bindings)):
                index=round(binding['physics_time_s']/model.opt.timestep);state=brackets[str(index)]
                mujoco.mj_setState(model,data,np.asarray(state['integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
                mujoco.mj_forward(model,data);now[0]=binding['captured_at']
                payload,depth,new_binding=capture(renderer,model,data,invocation['camera'],option=option,
                    stream_id=f'g4-registration18-resolution-{width}',seq=i+1,timestamp=now[0],rng=rng)
                observed=observer.observe(payload,[invocation['camera']],ids=[1,2,4,41],include_images=False)
                directory=out/f'pose-{i:02d}';directory.mkdir()
                (directory/'rgb.png').write_bytes(base64.b64decode(payload['images'][0]['data_base64']))
                np.savez_compressed(directory/'depth.npz',aligned_depth_m=depth);save(directory/'depth-binding.json',new_binding)
                entry=dict(before=old['before'],after=old['after'],arm_geometry_status=old['arm_geometry_status'],
                    observation=observed,sample=None,original_physics_step=index)
                try:
                    entry['sample']=stationary_sample(old['before'],old['after'],observed['result']['observations'][invocation['camera']],invocation['arm'])
                    entry['sample']['split']=plan['poses'][i]['split']
                finally:save(directory/'capture.json',entry)
                captures.append(entry)
            dataset=assemble_dataset(captures,model_directory);save(out/'dataset.json',dataset)
            fit=fit_registration(dataset);save(out/'fit.json',fit);row_result.update(status=fit['status'],fit=fit)
            if fit['status']=='REGISTRATION_VALIDATED':
                refined=pixel_refine(fit,dataset,captures,invocation['camera'])
                # No true camera/marker/object geometry enters either optimizer.
                truth=camera_truth_in_base(model,data,invocation['arm'],invocation['camera'])
                e=pose_error(np.asarray(fit['base_from_camera']),truth)
                row_result['production_absolute_error']=dict(translation_mm=e[0]*1000,rotation_deg=e[1])
                e=pose_error(np.asarray(refined['base_from_camera']),truth)
                refined['absolute_error']=dict(translation_mm=e[0]*1000,rotation_deg=e[1])
                row_result['pixel_refinement']=refined;save(out/'pixel-refinement.json',refined)
            if width==1280:
                baseline=load(run/'fit.json')
                # The diagnostic deliberately creates a new stream identity.
                # Compare all numerical results and every other binding field;
                # its dataset hash must change with the stream metadata.
                numerical=lambda value:{k:v for k,v in value.items() if k not in ('binding','dataset_sha256')}
                bound=lambda value:{k:v for k,v in value['binding'].items() if k!='stream_id'}
                row_result['baseline_production_numerics_exact']=numerical(fit)==numerical(baseline)
                row_result['baseline_binding_except_stream_exact']=bound(fit)==bound(baseline)
                row_result['baseline_identity_difference']='New stream_id and its dataset_sha256 are expected.'
                row_result['baseline_all_rgb_exact']=all(sha(out/f'pose-{i:02d}/rgb.png')==sha(run/f'pose-{i:02d}/rgb.png') for i in range(11))
                if not (row_result['baseline_production_numerics_exact'] and row_result['baseline_binding_except_stream_exact'] and row_result['baseline_all_rgb_exact']):
                    raise ValueError('Baseline rerender/fitting did not reproduce original evidence')
        except Exception as error:
            row_result.update(status='REJECTED',error=str(error))
        finally:renderer.close()
        row_result.update(accepted_samples=len(captures),new_robot_trajectories=0,registration_installed=False,full_planter_success=False)
        save(out/'result.json',row_result);results.append(row_result)
        print(json.dumps({k:v for k,v in row_result.items() if k not in ('fit','pixel_refinement')},allow_nan=False),flush=True)
    changed=[path for path,digest in hashes.items() if sha(path)!=digest]
    report=dict(scope=__doc__,results=results,source_inputs_unchanged=not changed,changed_inputs=changed,
        frozen_source_unchanged=all(sha(source/name)==digest for name,digest in manifest['hashes'].items()),
        current_visual_controller_qualified=False,registration_installed=False,new_robot_trajectories=0,
        full_planter_success=False,physical_success=False,hardware_commands=0)
    save(a.out/'result.json',report);shutil.copy2(__file__,a.out/Path(__file__).name)
    if changed:raise RuntimeError('Source changed during diagnostic')


if __name__=='__main__':main()
