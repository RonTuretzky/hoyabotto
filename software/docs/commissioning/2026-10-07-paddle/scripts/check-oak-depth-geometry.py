"""Archived commissioning prototype; see ../README.md. Not a production controller."""
import os as _archive_os
if _archive_os.environ.get("XLEROBOT_RUN_ARCHIVED_PROTOTYPE") != "1":
    raise SystemExit("Archived prototype: inspect source and README before explicit opt-in")
import sys,json,time,hashlib,cv2,numpy as np
from pathlib import Path
from pupil_apriltags import Detector
sys.path.insert(0,'/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/xlerobot-farm/software')
from farm.perception.tag_geometry import estimate_square
p=Path('work/oak-live-stream');m=json.loads((p/'oak.json').read_text());assert time.time()-m['captured_at']<3
for key,h in [('image','sha256'),('depth_image','depth_sha256')]:assert hashlib.sha256((p/m[key]).read_bytes()).hexdigest()==m[h]
im=cv2.imread(str(p/m['image']));dep=cv2.imread(str(p/m['depth_image']),cv2.IMREAD_UNCHANGED);tags=Detector(families='tag36h11',quad_decimate=1).detect(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY));rows=[]
for t in tags:
 if t.tag_id not in (1,2,3):continue
 pose=estimate_square(t.corners,60 if t.tag_id==1 else 40,np.array(m['intrinsics']),np.array(m['distortion_coefficients']));x,y=np.round(t.center).astype(int);patch=dep[max(0,y-5):y+6,max(0,x-5):x+6];v=patch[patch>0];z=pose['center_camera_mm'][2];med=float(np.median(v)) if len(v) else None
 rows.append({'tag_id':int(t.tag_id),'tag_pose_z_mm':z,'depth_patch_median_mm':med,'depth_valid_fraction':float(len(v)/patch.size),'disagreement_mm':med-z if med is not None else None})
r={'status':'DEPTH_TAG_CONSISTENCY_CHECK','rows':rows,'provider_alignment_verified':m['rgb_depth_pixel_registration_verified'],'method':'tag-centre 11x11 depth patch compared with RGB tag pose','limitations':['Tag depth estimates depend on user-confirmed printed size','Single scene consistency does not establish physical depth accuracy','No contact offsets established'],'motor_writes':0};Path('outputs/OAK-Depth-Tag-Consistency.json').write_text(json.dumps(r,indent=2));print(r)
