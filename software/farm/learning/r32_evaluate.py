"""Evaluate explicit R3.2 validation/test episodes from a frozen training selection."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from farm.assembly.r32 import PROFILE,load_profile
from farm.assembly.r32_data import digest, schema_for, select_episodes
from farm.learning.infer import resolve_pretrained_dir

def verify_selection(plan,checkpoint,profile,split):
 if plan.get('revision')!='R3.2' or split not in ('val','test'):raise ValueError('R3.2 validation or test selection required')
 s=plan['selection'];root=Path(s['root']);sessions=Path(s['sessions_file'])
 # Revalidate session membership and record provenance, not just a claimed held-out flag.
 current=select_episodes(root,profile,sessions,s['stage'])
 for key in ['episodes','provenance','schema_sha256','sessions_sha256','info_sha256']:
  if current[key]!=s[key]:raise ValueError('dataset or split changed since training: '+key)
 if not s.get('payload_sha256'):raise ValueError('no frozen payload hashes')
 for path,h in s['payload_sha256'].items():
  if digest(root/path)!=h:raise ValueError('dataset payload changed: '+path)
 train_root=Path(s['training_root'])
 if not s.get('training_files_sha256'):raise ValueError('training subset was not materialized')
 for path,h in s['training_files_sha256'].items():
  if digest(train_root/path)!=h:raise ValueError('training subset changed')
 cp=Path(resolve_pretrained_dir(checkpoint))
 cfg=json.loads((cp/'train_config.json').read_text());trained=cfg['dataset'].get('episodes')
 if trained is None or sorted(trained)!=list(range(len(s['episodes']['train']))):raise ValueError('checkpoint was not restricted to frozen training episodes')
 if cfg['dataset'].get('repo_id')!=s['training_repo_id'] or Path(cfg['dataset'].get('root','')).resolve()!=train_root.resolve():raise ValueError('checkpoint dataset differs')
 issues=schema_for(profile).problems_with_checkpoint(json.loads((cp/'config.json').read_text()))
 if issues:raise ValueError('; '.join(issues))
 return cp,root,s['episodes'][split]

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--selection',type=Path,required=True);p.add_argument('--checkpoint',required=True)
 p.add_argument('--profile',type=Path,default=PROFILE);p.add_argument('--split',choices=['val','test'],default='val');p.add_argument('--device',default='mps');p.add_argument('--stride',type=int,default=5);p.add_argument('--out',type=Path,required=True)
 a=p.parse_args()
 if a.stride<1:raise ValueError('positive stride required')
 if a.out.exists():raise ValueError('refuse evaluation overwrite')
 profile=load_profile(a.profile);plan=json.loads(a.selection.read_text());cp,root,ids=verify_selection(plan,a.checkpoint,profile,a.split)
 from farm.learning.evaluate import evaluate
 result=evaluate(str(cp),profile['dataset_repo_id'],str(root),stride=a.stride,device=a.device,episode_ids=ids)
 result.update(held_out=True,split=a.split,selection_sha256=digest(a.selection),note='Session-separated offline prediction error; not evidence of physical assembly success.')
 a.out.parent.mkdir(parents=True,exist_ok=True)
 with a.out.open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result,indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
