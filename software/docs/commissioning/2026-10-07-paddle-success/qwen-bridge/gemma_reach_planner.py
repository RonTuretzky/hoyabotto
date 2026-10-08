"""Read-only repository planner adapter; no transports or serial access."""
import hashlib,json,math,time
from pathlib import Path
import numpy as np
import farm
# The runtime selects live farm first; the audited shared kinematics lives in utility farm.
UTILITY_FARM=Path('/Users/teachera/Documents/Codex/2026-10-02/set-this-up-x20/work/carton-visual-controller/software/farm')
if str(UTILITY_FARM) not in farm.__path__: farm.__path__.append(str(UTILITY_FARM))
import farm.kinematics
if str(UTILITY_FARM/'kinematics') not in farm.kinematics.__path__: farm.kinematics.__path__.append(str(UTILITY_FARM/'kinematics'))
from carton.servo.kinematics import load_arm
from farm.kinematics.assets import verified_model
from farm.kinematics.lerobot import transform
ROOT=Path(__file__).resolve().parents[1]
BRIDGE=Path(__file__).resolve().parent
# left: measured on the robot Mac (outputs/ is not written by a deploy).
# right: shipped next to this file; redeploy_robot_server.py installs both into work/ and fetches its model assets.
CONFIGS={'left':ROOT/'outputs/Standard-Reach-Candidate.json','right':BRIDGE/'right-arm-kinematics.json'}
JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')
POSE_SCHEMA={'type':'array','minItems':1,'maxItems':10,'items':{'type':'array','minItems':4,'maxItems':4,'items':{'type':'array','minItems':4,'maxItems':4,'items':{'type':'number'}}}}
def validate_poses(poses):
 if not isinstance(poses,list) or not 1<=len(poses)<=10:raise ValueError('Supply 1–10 rigid 4x4 tool poses')
 for p in poses:
  if not isinstance(p,list) or len(p)!=4 or any(not isinstance(row,list) or len(row)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in row) for row in p):raise ValueError('Tool poses require finite numeric 4x4 matrices')
  t=transform(p)
  if np.max(np.abs(t[:3,3]))>1:raise ValueError('Tool pose translation exceeds one metre')
def load_config(path):
 """Relative calibration_file/model_directory resolve against the config's folder (as carton.servo.kinematics.load_arm does).
 With calibration_sha256_from_file, the digest is the current hash of calibration_file, never a stored literal:
 a recalibration changes it, which invalidates anything (e.g. a tag registration) bound to the old digest."""
 config=json.loads(path.read_text())
 for k in ('calibration_file','model_directory'):
  if isinstance(config.get(k),str):config[k]=str((path.parent/config[k]).resolve())
 # feetech_degrees_v1 derives every angle from calibration_file itself, so a stored digest can only go stale after a
 # recalibration (the left config's did on 2026-10-07); such configs always take the file's current hash.
 if config.get('calibration_sha256_from_file') is True or config.get('mapping')=='feetech_degrees_v1':
  try:config['calibration_sha256']=hashlib.sha256(Path(config['calibration_file']).read_bytes()).hexdigest()
  except (OSError,KeyError,TypeError):config['calibration_sha256']=None
 return config
def configuration(arm,calibration):
 path=CONFIGS.get(arm);missing=[];config=None
 if path is None or not path.exists():missing.append('measured kinematics configuration for '+arm)
 else:
  config=load_config(path)
  if (config.get('calibration_sha256_from_file') is True or config.get('mapping')=='feetech_degrees_v1') and config.get('calibration_sha256') is None:missing.append('readable calibration_file '+str(config.get('calibration_file')))
  if config.get('arm')!=arm:missing.append('configuration arm mismatch')
  if config.get('mapping')=='feetech_degrees_v1':
   if config.get('mapping_validated') is not True or not config.get('mapping_evidence'):missing.append('physical validation evidence for feetech_degrees_v1 mapping')
  else:
   for n in JOINTS:
    for k in ('model_zero_tick','model_sign'):
     if config.get('joints',{}).get(n,{}).get(k) is None:missing.append(n+'.'+k)
  for k in ('gripper_from_tool','workspace_bounds_m'):
   if config.get(k) is None:missing.append(k)
  actual=hashlib.sha256(calibration.read_bytes()).hexdigest()
  if config.get('calibration_sha256')!=actual:missing.append('matching saved motor calibration digest')
  try:_,model=verified_model(config['model_directory']);assets={'verified':True,'revision':model['revision'],'joint_names':model['joint_names']}
  except Exception as e:assets={'verified':False,'error':str(e)};missing.append('verified model assets')
  return {'available':True,'ready':not missing,'missing':missing,'config':config,'configuration_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'model_assets':assets},path
 return {'available':False,'ready':False,'missing':missing,'config':None},path

def inspect_or_plan(args,status,cal,calibration,camera_status,plan=True):
 arm=args['arm'];info,path=configuration(arm,calibration)
 if plan and args['frame']=='station' and (info['config'] or {}).get('base_from_station') is None:info['missing'].append('base_from_station');info['ready']=False
 result={'status':'NEEDS_GEOMETRIC_CONFIGURATION','configuration':info,'motor_writes':0,'collision_checked':False,'physical_calibration_verified':False,'captured_at':status.get('time'),'owner_status_age_s':time.time()-status.get('time',0),'camera_status':camera_status,'execution_supported':False}
 if not info['ready']:return result
 if not status.get('active') or not 0<=result['owner_status_age_s']<=1:raise ValueError('Fresh sole-owner encoder snapshot required')
 names=[arm+'_arm_'+n for n in JOINTS];ticks={n:status['rows'][n]['Present_Position'] for n in names}
 if any(type(q) not in (int,float) or not math.isfinite(q) for q in ticks.values()):raise ValueError('Finite live encoder readings required')
 experiment={'arm':arm,'calibration_file':str(calibration),'calibration_sha256':hashlib.sha256(calibration.read_bytes()).hexdigest(),'ranges':{n:[cal[n]['range_min'],cal[n]['range_max']] for n in names}}
 model,fingerprint=load_arm(path,experiment)
 result.update(kinematics_fingerprint=fingerprint,starting_ticks=ticks,tool_pose_estimate=model.forward(ticks).tolist(),pose_frame='arm_base',pose_units='metres',source=model.solver.provenance(),status='FK_ESTIMATE_ONLY')
 if plan:
  proposal=model.plan(ticks,{k:args[k] for k in ('frame','units','orientation','tool_poses')})
  for w in proposal['waypoints']:
   for n,q in w['joint_targets_ticks'].items():
    if not cal[n]['range_min']+4<=q<=cal[n]['range_max']-4:raise ValueError('Solver proposal exceeds commandable margin: '+n)
  result.update(proposal)
 return result
