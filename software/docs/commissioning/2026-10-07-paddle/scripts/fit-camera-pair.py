"""Archived commissioning prototype; see ../README.md. Not a production controller."""
import os as _archive_os
if _archive_os.environ.get("XLEROBOT_RUN_ARCHIVED_PROTOTYPE") != "1":
    raise SystemExit("Archived prototype: inspect source and README before explicit opt-in")
import sys,json,time,hashlib,cv2,numpy as np
from pathlib import Path
from pupil_apriltags import Detector
sys.path.insert(0,'/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software')
from farm.perception.tag_geometry import estimate_square,square_points
roots={'OAK':Path('work/oak-live-stream'),'Phone':Path('/Users/teachera/Documents/Codex/2026-10-05/m/work/phone_camera')};views={};det=Detector(families='tag36h11',quad_decimate=1)
for name,root in roots.items():
 m=json.loads((root/('oak.json' if name=='OAK' else 'latest.json')).read_text());stamp=m.get('rgb_captured_at',m.get('received_at'));assert time.time()-stamp<3
 raw=(root/(m['image'] if name=='OAK' else 'latest.jpg')).read_bytes()
 if name=='OAK':assert hashlib.sha256(raw).hexdigest()==m['sha256']
 im=cv2.imdecode(np.frombuffer(raw,np.uint8),1);ts=det.detect(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY));views[name]=(m,im,{int(t.tag_id):t for t in ts});Path('outputs/'+name+'-Pair-Fit.jpg').write_bytes(raw)
a,im,ot=views['OAK'];b,pim,pt=views['Phone'];assert all(i in ot and i in pt for i in (1,2,3)),'Need all three tags in both views'
objects={};sizes={1:60,2:40,3:40}
for i in (1,2,3):
 pose=estimate_square(ot[i].corners,sizes[i],np.array(a['intrinsics']),np.array(a['distortion_coefficients']));tf=np.array(pose['camera_from_tag']);objects[i]=(square_points(sizes[i]/1000)@tf[:3,:3].T+tf[:3,3]).astype(np.float64)
obj=np.concatenate([objects[i] for i in (1,2)]);pix=np.concatenate([pt[i].corners for i in (1,2)]).astype(np.float64);h,w=pim.shape[:2];best=None
for f in np.linspace(300,1800,151):
 k=np.array([[f,0,w/2],[0,f,h/2],[0,0,1]],float)
 try:
  ok,r,t=cv2.solvePnP(obj,pix,k,np.zeros(5),flags=cv2.SOLVEPNP_ITERATIVE)
  if not ok:continue
  pred=cv2.projectPoints(obj,r,t,k,np.zeros(5))[0].reshape(-1,2);err=float(np.sqrt(np.mean(np.sum((pred-pix)**2,axis=1))))
  if best is None or err<best[0]:best=(err,k,r,t)
 except cv2.error:continue
err,k,r,t=best
flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_ASPECT_RATIO|cv2.CALIB_ZERO_TANGENT_DIST|cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3|cv2.CALIB_FIX_K4|cv2.CALIB_FIX_K5|cv2.CALIB_FIX_K6
err,k,d,rs,ts=cv2.calibrateCamera([obj.astype(np.float32)],[pix.astype(np.float32)],(w,h),k,np.zeros(5),flags=flags);r,t=rs[0],ts[0]
hold=cv2.projectPoints(objects[3],r,t,k,np.zeros(5))[0].reshape(-1,2);he=float(np.sqrt(np.mean(np.sum((hold-pt[3].corners)**2,axis=1))))
out={'status':'EXPLORATORY_CAMERA_PAIR_FIT','train_tags':[1,2],'held_out_tag':3,'train_rms_px':err,'held_out_rms_px':he,'phone_assumed_intrinsics':k.tolist(),'phone_from_oak_rotation_vector':r.tolist(),'phone_from_oak_translation_metres':t.tolist(),'limitations':['Phone focal length and principal point fitted; square pixels and zero distortion assumed','OAK tag poses used as estimates, not measured ground truth','Single scene fit does not independently calibrate the phone lens','No calibrated jaw/contact offsets or arm model binding'],'motion_ready':False,'motor_writes':0}
Path('outputs/Camera-Pair-Exploratory-Fit.json').write_text(json.dumps(out,indent=2));print(out)
