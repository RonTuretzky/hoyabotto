"""Right-arm reach-planner config: no hardware, no network. Run from qwen-bridge/: python3 test_reach_planner_right.py
Optional: CARTON_MODEL_DIR=<verified SO-101 model dir> also checks real hash verification and the offline seeding."""
import ast,hashlib,json,os,shutil,sys,tempfile
from pathlib import Path
BRIDGE=Path(__file__).resolve().parent;SOFTWARE=BRIDGE.parents[3]
sys.path[:0]=[str(BRIDGE),str(SOFTWARE)]  # test this checkout's farm/carton, not an installed copy
TMP=Path(tempfile.mkdtemp());os.environ['XLEROBOT_WORK_ROOT']=str(TMP)  # redeploy module paths point into the temp folder
(TMP/'work').mkdir()
import gemma_reach_planner as P
import redeploy_robot_server as R
MANIFEST=json.loads((SOFTWARE/'farm/kinematics/so101-assets.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def accepted_by_registration(reply,arm,manifest=MANIFEST):
 """The checks farm.kinematics.tag_registration.assemble_dataset applies to robot_get_arm_pose before any motor is enabled."""
 status=reply.get('result',{});cfg=status.get('configuration',{}).get('config') or {}
 assets=status.get('configuration',{}).get('model_assets',{})
 return cfg.get('mapping')=='feetech_degrees_v1' and cfg.get('arm')==arm and assets.get('verified') is True and assets.get('revision')==manifest['revision']

# 1. The shipped config: the fields registration needs, nothing measured is claimed.
shipped=BRIDGE/'right-arm-kinematics.json';c=json.loads(shipped.read_text())
assert P.CONFIGS['right']==shipped and P.CONFIGS['left']==P.ROOT/'outputs/Standard-Reach-Candidate.json'
assert {k:c[k] for k in ('schema','arm','mapping','mapping_validated','gripper_from_tool','workspace_bounds_m')}=={'schema':1,'arm':'right','mapping':'feetech_degrees_v1','mapping_validated':False,'gripper_from_tool':None,'workspace_bounds_m':None}
assert c['calibration_file']=='/Users/teachera/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/farm_xlerobot.json'
assert c['calibration_sha256'] is None and c['calibration_sha256_from_file'] is True  # computed at load, never a stale literal
assert c['model_directory']=='so101-model' and not Path(c['model_directory']).is_absolute()

# 2. A normal deploy installs it (API-side only) and runs this test.
tree=ast.parse((BRIDGE/'redeploy_robot_server.py').read_text())
lists={t.id:ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) for t in n.targets if isinstance(t,ast.Name) and t.id in ('INSTALL','API_ONLY','TESTS')}
for name in ('gemma_reach_planner.py','right-arm-kinematics.json'):assert name in lists['INSTALL'] and name in lists['API_ONLY'] and (BRIDGE/name).exists()
assert 'test_reach_planner_right.py' in lists['TESTS']
assert 'gemma_reach_planner' not in (BRIDGE/'gemma_hardware_owner.py').read_text()  # API-only is accurate
assert R.model_directory()==(TMP/'work/so101-model').resolve()

# 3. Installed layout: work/right-arm-kinematics.json, temp calibration, empty model folder.
work=TMP/'work';cal=TMP/'calibration.json';cal.write_text(json.dumps({'right_arm_shoulder_pan':{'range_min':900,'range_max':3100}}))
cfg=dict(c,calibration_file=str(cal));(work/'right-arm-kinematics.json').write_text(json.dumps(cfg))
P.CONFIGS['right']=work/'right-arm-kinematics.json'
info,_=P.configuration('right',cal)
assert info['available'] and not info['ready'] and info['config']['calibration_sha256']==sha(cal)
assert 'matching saved motor calibration digest' not in info['missing'] and not any('readable calibration_file' in m for m in info['missing'])
assert info['config']['model_directory']==str((work/'so101-model').resolve())  # relative to the config, as load_arm resolves it
for m in ('physical validation evidence for feetech_degrees_v1 mapping','gripper_from_tool','workspace_bounds_m','verified model assets'):assert m in info['missing'],m
assert info['model_assets']['verified'] is False and 'Missing/changed upstream model asset' in info['model_assets']['error']

# 4. Recalibration: the digest follows the file without editing the config.
before=sha(cal);cal.write_text(json.dumps({'right_arm_shoulder_pan':{'range_min':901,'range_max':3100}}))
info,_=P.configuration('right',cal);assert info['config']['calibration_sha256']==sha(cal)!=before
other=TMP/'other-calibration.json';other.write_text('{}')  # API's live file differs from the config's file
info,_=P.configuration('right',other);assert 'matching saved motor calibration digest' in info['missing']
(work/'right-arm-kinematics.json').write_text(json.dumps(dict(cfg,calibration_file=str(TMP/'missing.json'))))
info,_=P.configuration('right',cal);assert info['config']['calibration_sha256'] is None and any('readable calibration_file' in m for m in info['missing'])
(work/'right-arm-kinematics.json').write_text(json.dumps(cfg))

# 5. robot_get_arm_pose(right): unverified assets -> registration refuses; verified -> accepted, still NEEDS_GEOMETRIC_CONFIGURATION, no motion.
status={'time':0,'active':False,'rows':{}}
pose=lambda:P.inspect_or_plan({'arm':'right'},status,{},cal,{'oak':'fresh'},plan=False)
r=pose();assert r['status']=='NEEDS_GEOMETRIC_CONFIGURATION' and r['motor_writes']==0 and r['execution_supported'] is False
assert not accepted_by_registration({'ok':True,'result':r},'right')
real=P.verified_model
P.verified_model=lambda folder:(Path(folder)/MANIFEST['urdf'],MANIFEST)  # stands in for files that pass the hash check
try:
 r=pose();assert r['configuration']['model_assets']=={'verified':True,'revision':MANIFEST['revision'],'joint_names':MANIFEST['joint_names']}
 assert accepted_by_registration({'ok':True,'result':r},'right') and not accepted_by_registration({'ok':True,'result':r},'left')
 assert not accepted_by_registration({'ok':True,'result':r},'right',dict(MANIFEST,revision='other'))  # revision mismatch refuses
 assert r['status']=='NEEDS_GEOMETRIC_CONFIGURATION' and 'verified model assets' not in r['configuration']['missing']
finally:P.verified_model=real
# A left config without the new flag keeps its stored digest (no runtime substitution).
left=TMP/'left.json';left.write_text(json.dumps({'arm':'left','calibration_sha256':'stored','model_directory':'m'}))
assert P.load_config(left)['calibration_sha256']=='stored'

# 6. Deploy-side model check: verification is the real hash check, never assumed.
empty=R.model_assets(TMP/'no-model');assert empty['verified'] is False and 'Missing/changed upstream model asset' in empty['error'],empty
dry=R.ensure_model(dry_run=True);assert dry['verified'] is False and not (TMP/'work/so101-model').exists()
model=os.environ.get('CARTON_MODEL_DIR')
if model and Path(model).is_dir():
 assert R.model_assets(Path(model))=={'verified':True,'revision':MANIFEST['revision']}
 (TMP/'outputs').mkdir();(TMP/'outputs/Standard-Reach-Candidate.json').write_text(json.dumps({'arm':'left','model_directory':str(Path(model).resolve())}))
 out=R.ensure_model(dry_run=False)  # seeds every file offline, so model-fetch downloads nothing
 assert out['verified'] and out['revision']==MANIFEST['revision'] and out['seeded_from']==str(Path(model).resolve()),out
 P.CONFIGS['right']=work/'right-arm-kinematics.json'
 info,_=P.configuration('right',cal);assert info['model_assets']['verified'] is True and info['model_assets']['revision']==MANIFEST['revision']
 assert R.ensure_model(dry_run=False)['verified']  # idempotent
 print('real SO-101 assets checked')
shutil.rmtree(TMP)
print('right-arm reach planner config: all checks passed')
