"""Archived commissioning prototype; see ../README.md. Not a production controller."""
import os as _archive_os
if _archive_os.environ.get("XLEROBOT_RUN_ARCHIVED_PROTOTYPE") != "1":
    raise SystemExit("Archived prototype: inspect source and README before explicit opt-in")
import json,struct,sys,cv2,numpy as np
from pathlib import Path
from pupil_apriltags import Detector
sys.path.insert(0,'/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software')
from farm.perception.tag_geometry import estimate_square
m=json.loads(Path('outputs/OAK-Jaw-Depth-Metadata.json').read_text());im=cv2.imread('outputs/OAK-Jaw-Depth.jpg');d=cv2.imread('outputs/OAK-Jaw-Depth.png',-1);tag=next(t for t in Detector(families='tag36h11',quad_decimate=1).detect(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)) if t.tag_id==2);tf=np.array(estimate_square(tag.corners,40,np.array(m['intrinsics']),np.array(m['distortion_coefficients']))['camera_from_tag'])
mask=np.zeros(d.shape,np.uint8);cv2.fillConvexPoly(mask,np.array([[555,182],[521,241],[536,242],[557,197]],np.int32),1);ys,xs=np.where((mask>0)&(d>0));z=d[ys,xs].astype(float);pixels=np.stack((xs,ys),axis=1).astype(float);rays=cv2.undistortPoints(pixels[:,None,:],np.array(m['intrinsics']),np.array(m['distortion_coefficients'])).reshape(-1,2);xyz=np.column_stack((rays*z[:,None],z));obs=(xyz/1000-tf[:3,3])@tf[:3,:3]*1000;obs=obs[::max(1,len(obs)//400)]
b=Path('work/so101-registration-model/assets/wrist_roll_follower_so101_v1.stl').read_bytes();n=struct.unpack('<I',b[80:84])[0];dt=np.dtype([('normal','<f4',(3,)),('verts','<f4',(3,3)),('attr','<u2')]);model=np.unique(np.frombuffer(b,dt,count=n,offset=84)['verts'].reshape(-1,3),axis=0)*1000;model=model[model[:,2]>35];model=model[::max(1,len(model)//1200)]
# Fixed-jaw mesh coordinates: grasp-frame origin is first converted from gripper_link using the visual origin/rotation.
tip=model[model[:,2]>model[:,2].max()-.3].mean(axis=0);measured_tip=np.array(json.loads(Path('outputs/Jaw-Depth-Feature-Check.json').read_text())['features'][0]['point_in_tag2_frame_mm'])
grasp=np.array([-7.9,0.000093,-.0+99.077106]);base=np.array([[0,0,-1],[0,1,0],[1,0,0]],float);results=[]
for dx in (-15,0,15):
 for dz in (-10,0,10):
  R=base.copy();t=np.array([30+dx,0,24+dz],float)
  for iteration in range(18):
   cloud=model@R.T+t;dist=((obs[:,None,:]-cloud[None,:,:])**2).sum(axis=2);ids=dist.argmin(axis=1);errs=np.sqrt(dist[np.arange(len(obs)),ids]);keep=errs<=np.percentile(errs,80);A=model[ids[keep]];B=obs[keep];ac=A.mean(axis=0);bc=B.mean(axis=0);u,s,vh=np.linalg.svd((A-ac).T@(B-bc));rot=vh.T@u.T
   if np.linalg.det(rot)<0:vh[-1]*=-1;rot=vh.T@u.T
   R=rot;t=bc-R@ac
  cloud=model@R.T+t;err=np.sqrt(((obs[:,None,:]-cloud[None,:,:])**2).sum(axis=2).min(axis=1));results.append({'initial_dx_dz_mm':[dx,dz],'held_out_tip_error_mm':float(np.linalg.norm(R@tip+t-measured_tip)),'trimmed_rms_mm':float(np.sqrt(np.mean(np.sort(err)[:int(.8*len(err))]**2))),'tag2_from_cad_grasp_mm':(R@grasp+t).tolist()})
pts=np.array([r['tag2_from_cad_grasp_mm'] for r in results]);out={'status':'EXPLORATORY_PARTIAL_SURFACE_ICP','observed_points':len(obs),'fits':results,'grasp_point_spread_mm':float(np.sqrt(((pts[:,None,:]-pts[None,:,:])**2).sum(axis=2)).max()),'validated':False,'tip_model_mm':tip.tolist(),'measured_tip_tag2_mm':measured_tip.tolist(),'motor_writes':0,'limitations':['Manually selected narrow fixed-jaw surface','Partial surface nearest-neighbour fit can match incorrect mesh regions','No held-out physical grasp-point reference','No arm model zero/sign binding','Depth alignment remains uncommissioned']};Path('outputs/Jaw-CAD-Fit-Ambiguity.json').write_text(json.dumps(out,indent=2));print({'observed_points':len(obs),'best_rms_mm':min(r['trimmed_rms_mm'] for r in results),'grasp_point_spread_mm':out['grasp_point_spread_mm'],'validated':False})
