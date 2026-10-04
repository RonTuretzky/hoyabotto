"""Offline R3.3 acceptance cases; no hardware or optimizer calls."""
import json
import time
from pathlib import Path
import numpy as np
import pytest
from farm.assembly.r32 import *
from farm.assembly.r32_data import *
from farm.learning.r32_train import prepare,validate_payload,materialize_training_dataset
from farm.learning.r32_evaluate import verify_selection

@pytest.mark.parametrize('ring',[False,True])
def test_success_is_rehearsal_only(ring):
 r=rehearsal(ring);assert r['result']=='SIMULATED_COMPLETE'
 assert r['training_eligible'] is False and r['physical_success'] is False
 stages=[e['stage'] for e in r['events'] if e['event']=='STAGE_VERIFIED']
 assert ('PLACE_RETAINER' in stages)==ring

@pytest.mark.parametrize('fault',FAULTS[1:])
def test_faults_stop_without_release_or_retries(fault):
 r=rehearsal(True,fault);assert r['result']=='SIMULATED_STOPPED'
 assert r['events'][-1]['event']=='STOP_HOLD'
 assert sum(e['event']=='STOP_HOLD' for e in r['events'])==1
 if fault in ('no_pick','seat_unknown','stop','deadline'):
  assert not any(e.get('action')=='release' for e in r['events'])

def test_sequence_cannot_skip_or_release_unknown():
 m=Supervisor();e={n:Reading(True,Status.OK,t=0) for n in CHECKS['PREFLIGHT']};m.advance(e,0)
 with pytest.raises(Blocked):m.action('release',{},0)
 with pytest.raises(Blocked):m.advance({},0)

@pytest.mark.parametrize('missing',['supported_before_release','ring_seated'])
def test_ring_must_be_seated_before_jaws_open(missing):
 m=Supervisor(with_retainer=True);m.index=m.order.index('PLACE_RETAINER')
 m.phase='transferred';m.held='HELD'
 e={n:Reading(True,Status.OK,t=0) for n in ('held_after_transfer','supported_before_release','ring_seated','release_method_validated','release_zone_verified')}
 e[missing].status=Status.UNKNOWN
 with pytest.raises(Blocked):m.action('release',e,0)
 assert m.held=='HELD' and m.events[-1]['event']=='STOP_HOLD'
 assert not any(v.get('action')=='release' for v in m.events)

def test_profile_and_mesh_identity(tmp_path):
 p=load_profile();assert not part_problems(p)
 p['execution_enabled']=True;q=tmp_path/'p.json';q.write_text(json.dumps(p))
 with pytest.raises(ValueError):load_profile(q)
 p['execution_enabled']=False;p['parts_dir']=str(tmp_path)
 assert len(part_problems(p))==3

def tick_inputs(schema,t=100.):
 joints=Reading({j:0. for j in schema.joints},Status.OK,t=t)
 frames={cam:Reading(np.zeros((*schema.frame_hw,3),dtype=np.uint8),Status.OK,t=t) for cam in schema.cameras.values()}
 return joints,dict(joints.value),frames,t,t

def test_strict_recording_timing_and_actions():
 s=schema_for(load_profile());check=PhysicalTick(s,step_max=5,watchdog_s=.5)
 args=tick_inputs(s);v=check.build(*args);assert v.ok
 assert np.isfinite(v.frame['extra']['observation.timing']).all()
 assert not check.build(*args).ok # duplicate clock
 args=list(tick_inputs(s,101));args[1]=None;assert not check.build(*args).ok
 args=list(tick_inputs(s,102));args[0].t=103;assert not check.build(*args).ok
 args=list(tick_inputs(s,103));args[2]['head'].t=100;assert not check.build(*args).ok
 args=list(tick_inputs(s,104));args[1][s.joints[0]]=99;assert not check.build(*args).ok
 args=list(tick_inputs(s,105));args[2]['head'].value=np.zeros((2,2,3),dtype=np.uint8);assert not check.build(*args).ok

@pytest.fixture
def dataset(tmp_path):
 # Metadata fixture for adversarial selection tests. Not presented as real robot data.
 p=load_profile();s=schema_for(p);root=tmp_path/'dataset';(root/'meta/r32_episodes').mkdir(parents=True)
 info={'features':s.features(),'fps':s.fps,'total_episodes':4}
 (root/'meta/info.json').write_text(json.dumps(info));(root/'meta/r32_schema.json').write_text(json.dumps(s.to_dict()))
 sessions=tmp_path/'sessions.json';sessions.write_text(json.dumps({'schema':'r32-sessions-1','sessions':{k:{'split':k,'declared_at':1} for k in ['train','val','test']}}))
 for i,split in enumerate(['train','val','test','train']):
  meta={'session_id':split,'split':split,'stage':'PLACE_RESERVOIR','revision':REVISION,'source':'physical_robot','collection_method':'robot_driven_visual_teaching','mesh_sha256':p['part_hashes'],'started_at':2,'episode_index':i,'frames':3,'outcome':'SUCCESS' if i<3 else 'FAILED','interventions':[],'safety_events':[],'rejected_ticks':0,'verified_checks':{'reservoir_stable_in_dock':True}}
  for k in ['calibration_id','robot_id','printed_instance','tool_geometry_id','station_measurement_id','camera_identity_id','material_profile','code_commit']:meta[k]='test_fixture_not_physical_evidence'
  (root/'meta/r32_episodes'/f'{i:06}.json').write_text(json.dumps(meta))
 return p,root,sessions

def mutate(root,i,**changes):
 path=root/'meta/r32_episodes'/f'{i:06}.json';m=json.loads(path.read_text());m.update(changes);path.write_text(json.dumps(m))

def test_launcher_excludes_failed_and_eval_data(dataset,tmp_path):
 p,r,s=dataset;plan=prepare(p,r,s,'PLACE_RESERVOIR',tmp_path/'run')
 assert plan['selection']['episodes']=={'train':[0],'val':[1],'test':[2]}
 assert '--dataset.episodes=[0]' in plan['command']
 assert '--policy.push_to_hub=false' in plan['command']
 assert plan['status']=='PREPARED_NOT_LAUNCHED'

@pytest.mark.parametrize('changes',[{'source':'synthetic_rehearsal'},{'revision':'R2a'},{'split':'train'},{'started_at':0},{'mesh_sha256':{}},{'episode_index':0}])
def test_bad_provenance_refused(dataset,changes):
 p,r,s=dataset;mutate(r,1,**changes)
 with pytest.raises(ValueError):select_episodes(r,p,s,'PLACE_RESERVOIR')

@pytest.mark.parametrize('changes',[{'interventions':['adjustment']},{'safety_events':['stop']},{'rejected_ticks':1},{'outcome':'FAILED'}])
def test_no_eligible_holdout_is_blocked(dataset,changes):
 p,r,s=dataset;mutate(r,2,**changes)
 with pytest.raises(ValueError,match='held-out'):select_episodes(r,p,s,'PLACE_RESERVOIR')

def test_missing_provenance_and_old_data_refused(dataset,tmp_path):
 p,r,s=dataset;(r/'meta/r32_episodes/000001.json').unlink()
 with pytest.raises(ValueError,match='missing provenance'):select_episodes(r,p,s,'PLACE_RESERVOIR')
 with pytest.raises(ValueError):Recorder(p,r,s,step_max=5,watchdog_s=.5)
 (r/'meta/r32_schema.json').unlink()
 with pytest.raises(ValueError,match='non-R3.3'):Recorder(p,r,s,step_max=5,watchdog_s=.5)

def test_real_lerobot_roundtrip(tmp_path):
 # Uses physical-source metadata ONLY inside an isolated test tmpdir to exercise serialization.
 p=load_profile();p['frame_hw']=[32,32];s=schema_for(p);root=tmp_path/'ds';sessions=tmp_path/'sessions.json'
 sessions.write_text(json.dumps({'schema':'r32-sessions-1','sessions':{k:{'split':k,'declared_at':time.time()-10} for k in ['train','val','test']}}))
 rec=Recorder(p,root,sessions,step_max=5,watchdog_s=.5)
 for value,split in enumerate(['train','val','test']):
  meta={'session_id':split,'split':split,'stage':'PLACE_RESERVOIR','revision':REVISION,'source':'physical_robot','collection_method':'robot_driven_visual_teaching','mesh_sha256':p['part_hashes']}
  for k in ['calibration_id','robot_id','printed_instance','tool_geometry_id','station_measurement_id','camera_identity_id','material_profile','code_commit']:meta[k]='test_fixture_not_physical_evidence'
  rec.start(meta)
  t=time.time()
  for i in range(3):
   args=list(tick_inputs(s,t+i*.1))
   args[0].value={j:float(value*10) for j in s.joints};args[1]=dict(args[0].value)
   assert rec.tick(*args)
  assert rec.end('SUCCESS',verified_checks={'reservoir_stable_in_dock':True},interventions=[],safety_events=[])['frames']==3
 rec.close()
 plan=prepare(p,root,sessions,'PLACE_RESERVOIR',tmp_path/'run')
 ds=validate_payload(p,root,plan['selection']);assert ds.meta.total_episodes==3
 train=materialize_training_dataset(ds,plan['selection'])
 assert train.meta.total_episodes==1
 assert np.allclose(train.meta.stats['action']['mean'],0)
 assert np.allclose(ds.meta.stats['action']['mean'],10)
 # Verify evaluation refuses a checkpoint trained on all episodes, then accepts exact train-only selection.
 cp=tmp_path/'checkpoint';cp.mkdir();cfg={'dataset':{'repo_id':plan['selection']['training_repo_id'],'root':plan['selection']['training_root'],'episodes':None}}
 (cp/'train_config.json').write_text(json.dumps(cfg))
 (cp/'config.json').write_text(json.dumps({'input_features':{'observation.state':{'shape':[6]},**{'observation.images.'+k:{'shape':[3,32,32]} for k in s.camera_keys}},'output_features':{'action':{'shape':[6]}}}))
 with pytest.raises(ValueError,match='restricted'):verify_selection(plan,str(cp),p,'test')
 cfg['dataset']['episodes']=[0];(cp/'train_config.json').write_text(json.dumps(cfg))
 assert verify_selection(plan,str(cp),p,'test')[2]==[2]
 # Payload changes invalidate the evaluation claim.
 payload=next(root.rglob('*.parquet'));payload.write_bytes(payload.read_bytes()+b'changed')
 with pytest.raises(ValueError,match='payload changed'):verify_selection(plan,str(cp),p,'test')


def test_session_assignment_cannot_be_rewritten(tmp_path):
 from farm.assembly.r32_data import declare_session
 p=tmp_path/'sessions.json'
 row=declare_session(p,'physical-session-a','train');assert row['split']=='train'
 with pytest.raises(ValueError,match='already declared'):declare_session(p,'physical-session-a','test')
 assert read_sessions(p)['physical-session-a']['split']=='train'


def test_epoch_timestamps_keep_subsecond_precision():
 schema=schema_for(load_profile());c=PhysicalTick(schema,step_max=5,watchdog_s=.5)
 now=1_800_000_000.;assert c.build(*tick_inputs(schema,now)).ok
 v=c.build(*tick_inputs(schema,now+.1));assert v.ok
 assert v.frame['extra']['observation.timing'][0]==pytest.approx(.1,abs=1e-6)
 assert v.frame['extra']['observation.timing'][2]==pytest.approx(.1,abs=1e-6)


def test_training_cli_parses_and_excludes_recording_clock(dataset,tmp_path):
 import draccus
 from lerobot.configs.train import TrainPipelineConfig
 from lerobot.policies.act.configuration_act import ACTConfig
 p,r,s=dataset;plan=prepare(p,r,s,'PLACE_RESERVOIR',tmp_path/'run')
 cfg=draccus.parse(config_class=TrainPipelineConfig,args=plan['command'][1:])
 assert cfg.dataset.episodes==[0]
 assert set(cfg.policy.input_features)=={'observation.state','observation.images.wrist','observation.images.head'}
 assert 'observation.timing' not in cfg.policy.input_features
