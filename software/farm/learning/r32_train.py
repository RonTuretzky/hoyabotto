"""Prepare or run an isolated ACT stage-training job on verified R3.2 recordings."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from farm.assembly.r32 import PROFILE, SOFTWARE, STAGES, load_profile, part_problems
from farm.assembly.r32_data import select_episodes, digest
from farm.learning.train import build_command, run, training_env

def policy_inputs(profile):
 return {'observation.state':{'type':'STATE','shape':[6]},**{'observation.images.'+k:{'type':'VISUAL','shape':[3,*profile['frame_hw']]} for k in profile['cameras']}}

def prepare(profile,root,sessions,stage,output,device=None,steps=None):
 issues=part_problems(profile)
 if issues:raise ValueError('; '.join(issues))
 selection=select_episodes(root,profile,sessions,stage)
 selection['root']=str(Path(root).resolve())
 selection['sessions_file']=str(Path(sessions).resolve())
 selection['payload_sha256']={str(p.relative_to(root)):digest(p) for p in sorted(Path(root).rglob('*.parquet'))}
 if Path(output).exists():raise ValueError('output directory already exists; choose a new run')
 train_root=Path(output).resolve().with_name(Path(output).name+'.datasets')/'train'
 selection['training_root']=str(train_root)
 selection['training_repo_id']=profile['dataset_repo_id']+'_train'
 selection['training_episode_map']={str(i):old for i,old in enumerate(sorted(selection['episodes']['train']))}
 cfg=profile['policy'];n=steps if steps is not None else cfg['steps']
 if type(n) is not int or n<1:raise ValueError('positive steps required')
 command=build_command(selection['training_repo_id'],policy='act',root=train_root,output_dir=Path(output).resolve(),device=device or cfg['device'],steps=n,batch_size=cfg['batch_size'],extra=['--dataset.episodes='+json.dumps(list(range(len(selection['episodes']['train'])))), '--policy.input_features='+json.dumps(policy_inputs(profile))])
 return {'revision':'R3.2','status':'PREPARED_NOT_LAUNCHED','hardware_execution':False,'selection':selection,'command':command,
         'evaluation_note':'Validate on val episodes; reserve test until model selection ends. Offline errors are not physical success.'}

def validate_payload(profile,root,selection):
 # Open the actual local dataset before starting a long optimizer job.
 from lerobot.datasets.lerobot_dataset import LeRobotDataset
 import numpy as np
 from farm.assembly.r32_data import schema_for
 ds=LeRobotDataset(profile['dataset_repo_id'],root=Path(root).resolve())
 for i in sum(selection['episodes'].values(),[]):
  row=ds.meta.episodes[i];start,end=int(row['dataset_from_index']),int(row['dataset_to_index'])
  if end<=start:raise ValueError('empty episode payload')
  meta=next(v for v in selection['provenance'] if v['episode_index']==i)
  expected=json.loads(Path(meta['path']).read_text())['frames']
  if end-start!=expected:raise ValueError('payload/provenance length mismatch')
  for j in range(start,end):
   item=ds[j]
   if int(item['episode_index'].item())!=i:raise ValueError('episode payload index mismatch')
   for key in ['action','observation.state','observation.timing']:
    if not np.isfinite(item[key].numpy()).all():raise ValueError('nonfinite payload '+key)
   for cam in schema_for(profile).camera_keys:
    image=item['observation.images.'+cam]
    if not np.isfinite(image.numpy()).all():raise ValueError('invalid image payload')
 return ds

def materialize_training_dataset(ds,selection):
 from lerobot.datasets.dataset_tools import split_dataset
 train_root=Path(selection['training_root'])
 if train_root.parent.exists():raise ValueError('training dataset staging directory exists; choose a new run')
 result=split_dataset(ds,{'train':selection['episodes']['train']},output_dir=train_root.parent)['train']
 if result.meta.total_episodes!=len(selection['episodes']['train']):raise ValueError('wrong training subset size')
 selection['training_files_sha256']={str(p.relative_to(train_root)):digest(p) for p in sorted(train_root.rglob('*')) if p.is_file()}
 selection['normalization_scope']='materialized training episodes only; LeRobot reaggregates per-episode statistics'
 return result

def main(argv=None):
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--profile',type=Path,default=PROFILE)
 ap.add_argument('--root',type=Path);ap.add_argument('--sessions',type=Path);ap.add_argument('--stage',choices=STAGES[1:],default='PLACE_RESERVOIR')
 ap.add_argument('--output',type=Path,default=SOFTWARE/'data-train/r32-reservoir-v0');ap.add_argument('--steps',type=int);ap.add_argument('--device',choices=['mps','cpu','cuda'])
 ap.add_argument('--launch',action='store_true',help='Run optimizer after local dataset validation; never moves hardware')
 ap.add_argument('--report',type=Path)
 a=ap.parse_args(argv);profile=load_profile(a.profile);root=a.root or SOFTWARE/profile['dataset_root'];sessions=a.sessions or SOFTWARE/profile['sessions_file']
 try:plan=prepare(profile,root,sessions,a.stage,a.output,a.device,a.steps)
 except (ValueError,FileNotFoundError,KeyError) as e:
  print(json.dumps({'status':'WAITING_FOR_R3.2_RECORDINGS','training_started':False,'reason':str(e)},indent=2));return 2
 if a.launch:
  ds=validate_payload(profile,root,plan['selection'])
  materialize_training_dataset(ds,plan['selection'])
  # Freeze the exact eligible indices and provenance alongside, before launch.
  a.output.parent.mkdir(parents=True,exist_ok=True)
  manifest=a.output.with_name(a.output.name+'.selection.json')
  with manifest.open('x') as f:json.dump(plan,f,indent=2)
  rc=run(plan['command'],a.output.with_name(a.output.name+'.train.log'),env=training_env())
  plan['status']='OPTIMIZER_EXITED';plan['exit_code']=rc
 else:rc=0
 if a.report:
  a.report.parent.mkdir(parents=True,exist_ok=True)
  with a.report.open('x') as f:json.dump(plan,f,indent=2)
 print(json.dumps(plan,indent=2));return rc
if __name__=='__main__':raise SystemExit(main())
