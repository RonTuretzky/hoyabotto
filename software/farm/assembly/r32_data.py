"""R3.2 physical-recording boundary and session-separated training selection."""
from __future__ import annotations
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np
from farm.assembly.schema import DatasetSchema, StrictTick, TickVerdict
from farm.learning.recorder import EpisodeRecorder
from farm.status import Status
from .r32 import REVISION, STAGES

SCHEMA_VERSION='r32-dataset-1'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
class R32Schema(DatasetSchema):
 @property
 def timing_names(self):
  return ['t_obs_from_origin','t_action_from_origin','dt_prev']+[f'age_{k}' for k in self.camera_keys]

def schema_for(profile):
 return R32Schema(profile['controlled_arm'],profile['state_joints'],profile['cameras'],profile['fps'],tuple(profile['frame_hw']),SCHEMA_VERSION)

def read_sessions(path):
 d=json.loads(Path(path).read_text())
 if d.get('schema')!='r32-sessions-1':raise ValueError('R3.2 session plan required')
 for sid,row in d['sessions'].items():
  if not sid or row.get('split') not in ('train','val','test'):raise ValueError('invalid session split')
  t=row.get('declared_at')
  if not isinstance(t,(int,float)) or isinstance(t,bool) or not math.isfinite(t) or t<=0:raise ValueError('session declaration timestamp required')
 return d['sessions']

def metadata_problems(meta,profile,sessions):
 problems=[]
 for key in ('session_id','stage','calibration_id','robot_id','printed_instance','tool_geometry_id','station_measurement_id','camera_identity_id','material_profile','code_commit'):
  if not isinstance(meta.get(key),str) or not meta[key].strip():problems.append('missing '+key)
 if meta.get('revision')!=REVISION:problems.append('wrong revision')
 if meta.get('source')!='physical_robot':problems.append('synthetic or unspecified source is not training data')
 if meta.get('collection_method')!='robot_driven_visual_teaching':problems.append('wrong collection method')
 if meta.get('mesh_sha256')!=profile['part_hashes']:problems.append('mesh hashes differ')
 if meta.get('stage') not in STAGES[1:]:problems.append('unknown stage')
 row=sessions.get(meta.get('session_id'))
 if not row:problems.append('session not declared before collection')
 else:
  if meta.get('split')!=row['split']:problems.append('session split changed')
  started=meta.get('started_at')
  if not isinstance(started,(float,int)) or not math.isfinite(started) or started<row['declared_at']:problems.append('split declared after recording or missing start time')
 return problems

class PhysicalTick(StrictTick):
 origin=None
 """Add strict clock, image and state checks before shared command/clamp validation."""
 def build(self,joints,action_sent,frames,now,t_action=None):
  clocks=[now,t_action,joints.t]+[r.t for r in frames.values()]
  if any(t is None or not isinstance(t,(int,float)) or not math.isfinite(t) for t in clocks):return self._reject('clock invalid')
  if t_action>now or t_action<now-self.watchdog_s or joints.t>now or any(r.t>now for r in frames.values()):return self._reject('clock stale or future')
  if self.prev_t is not None and now<=self.prev_t:return self._reject('clock not increasing')
  if isinstance(joints.value,dict):
   for j in self.schema.joints:
    v=joints.value.get(j);lo,hi=self.gripper_bounds if j.endswith('gripper') else self.joint_bounds
    if not isinstance(v,(int,float,np.number)) or not math.isfinite(v) or not lo<=v<=hi:return self._reject('state invalid: '+j)
  for cam in self.schema.cameras.values():
   r=frames.get(cam)
   if r is not None and r.value is not None:
    a=np.asarray(r.value)
    if a.shape!=(*self.schema.frame_hw,3) or a.dtype!=np.uint8:return self._reject('image shape/dtype invalid: '+cam)
  prev=self.prev_t
  v=super().build(joints,action_sent,frames,now,t_action)
  if v.ok:
   if self.origin is None:self.origin=now
   v.frame['extra']['observation.timing']=np.asarray([now-self.origin,t_action-self.origin,0. if prev is None else now-prev,*[now-frames[c].t for c in self.schema.cameras.values()]],dtype=np.float32)
  return v

class Recorder:
 """Write real frames to LeRobot plus one provenance file per episode.

 Caller supplies measured limits and actual commands AFTER the hardware clamp.
 This class does not control a robot or generate observations. Failed attempts
 are saved too, but the training selector excludes them.
 """
 def __init__(self,profile,root,sessions_file,*,step_max,watchdog_s):
  if not all(math.isfinite(v) and v>0 for v in [step_max,watchdog_s]):raise ValueError('measured runtime limits required')
  self.profile=profile;self.root=Path(root);self.sessions_file=Path(sessions_file);self.schema=schema_for(profile)
  self.tick_check=PhysicalTick(self.schema,step_max,watchdog_s);self.active=None;self.rejected=0;self.broken=False
  self.rec=None
  info=self.root/'meta/info.json';marker=self.root/'meta/r32_schema.json'
  if info.exists():
   if not marker.exists() or json.loads(marker.read_text())!=self.schema.to_dict():raise ValueError('refuse non-R3.2 or changed dataset')
   issues=self.schema.problems_with_info(json.loads(info.read_text()))
   if issues:raise ValueError('; '.join(issues))
   # A data write without provenance must be reconciled before recording can resume.
   total=json.loads(info.read_text())['total_episodes']
   if len(list((self.root/'meta/r32_episodes').glob('*.json')))!=total:raise ValueError('episode provenance incomplete')
 def start(self,meta):
  if self.active is not None or self.broken:raise ValueError('finish episode or reconcile broken recording first')
  meta=dict(meta);meta['started_at']=time.time()
  issues=metadata_problems(meta,self.profile,read_sessions(self.sessions_file))
  if issues:raise ValueError('; '.join(issues))
  if self.rec is None:
   self.rec=EpisodeRecorder(self.root,self.profile['dataset_repo_id'],self.schema.fps,self.schema.camera_keys,self.schema.frame_hw,self.schema.state_names,extra_features=self.schema.extra_features())
   marker=self.root/'meta/r32_schema.json'
   if not marker.exists():marker.write_text(json.dumps(self.schema.to_dict(),indent=2))
  self.active=meta;self.rejected=0;self.tick_check.prev_t=None;self.tick_check.origin=None
  self.rec.start_episode('R3.2 '+meta['stage'])
 def tick(self,joints,action_sent,frames,now,t_action):
  if self.active is None:raise ValueError('no active episode')
  v=self.tick_check.build(joints,action_sent,frames,now,t_action)
  if not v.ok:self.rejected+=1;return False
  f=v.frame;ok=self.rec.tick(f['joints'],f['action'],f['frames'],extra=f['extra'])
  if not ok:self.rejected+=1
  return ok
 def end(self,outcome,*,verified_checks,interventions,safety_events):
  if self.active is None:raise ValueError('no active episode')
  if outcome not in ('SUCCESS','FAILED','INTERVENED','INCOMPLETE'):raise ValueError('invalid outcome')
  from .r32 import CHECKS
  required=CHECKS[self.active['stage']]
  if outcome=='SUCCESS' and any(verified_checks.get(n) is not True for n in required):raise ValueError('successful outcome needs all stage checks')
  index=self.rec.ds.meta.total_episodes
  dst=self.root/'meta/r32_episodes'/f'{index:06}.json'
  if dst.exists():raise ValueError('refuse provenance overwrite')
  n=self.rec.end_episode(save=True)
  meta={**self.active,'episode_index':index,'frames':n,'outcome':outcome,'verified_checks':verified_checks,
        'interventions':interventions,'safety_events':safety_events,'rejected_ticks':self.rejected,'tick_origin_unix_s':self.tick_check.origin,'ended_at':time.time()}
  self.active=None
  if n==0:self.broken=True;raise ValueError('empty or failed data write; episode not eligible')
  dst.parent.mkdir(parents=True,exist_ok=True)
  try:
   with dst.open('x') as f:json.dump(meta,f,indent=2,allow_nan=False)
  except Exception:
   self.broken=True;raise
  return meta
 def close(self):
  if self.active is not None:raise ValueError('finish episode with explicit outcome before closing')
  if self.rec:self.rec.close()

def select_episodes(root,profile,sessions_file,stage):
 """Files-only selection; launch additionally opens LeRobot payloads and checks frames."""
 root=Path(root);info=json.loads((root/'meta/info.json').read_text());schema=schema_for(profile)
 if json.loads((root/'meta/r32_schema.json').read_text())!=schema.to_dict():raise ValueError('wrong R3.2 schema marker')
 issues=schema.problems_with_info(info)
 if issues:raise ValueError('; '.join(issues))
 sessions=read_sessions(sessions_file);rows=[];seen=set();selected={k:[] for k in ('train','val','test')}
 from .r32 import CHECKS
 if stage not in STAGES[1:]:raise ValueError('unknown stage')
 for path in sorted((root/'meta/r32_episodes').glob('*.json')):
  m=json.loads(path.read_text());issues=metadata_problems(m,profile,sessions)
  if issues:raise ValueError(path.name+': '+'; '.join(issues))
  i=m.get('episode_index')
  if type(i) is not int or i<0 or i>=info['total_episodes'] or i in seen:raise ValueError('duplicate or invalid episode index')
  seen.add(i)
  if type(m.get('frames')) is not int or m['frames']<1:raise ValueError('missing frame count')
  if m['stage']==stage and m.get('outcome')=='SUCCESS' and m.get('interventions')==[] and m.get('safety_events')==[] and m.get('rejected_ticks')==0:
   if not all(m.get('verified_checks',{}).get(k) is True for k in CHECKS[stage]):raise ValueError('missing success evidence')
   selected[m['split']].append(i)
  rows.append({'episode_index':i,'sha256':digest(path),'path':str(path)})
 if seen!=set(range(info['total_episodes'])):raise ValueError('dataset episodes missing provenance')
 if not all(selected.values()):raise ValueError('need eligible train, val and held-out test sessions for this stage')
 return {'stage':stage,'episodes':selected,'provenance':rows,'schema_sha256':digest(root/'meta/r32_schema.json'),'sessions_sha256':digest(sessions_file),'info_sha256':digest(root/'meta/info.json')}

def declare_session(path,session_id,split):
 """Append an immutable session assignment before recording. Never reassign an old session."""
 import fcntl
 if not session_id.strip() or split not in ('train','val','test'):raise ValueError('session id and train/val/test split required')
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 with path.with_suffix(path.suffix+'.lock').open('a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  doc=json.loads(path.read_text()) if path.exists() else {'schema':'r32-sessions-1','sessions':{}}
  if doc.get('schema')!='r32-sessions-1':raise ValueError('wrong session file')
  if session_id in doc['sessions']:raise ValueError('session already declared; cannot reassign it')
  row={'split':split,'declared_at':time.time()};doc['sessions'][session_id]=row
  temp=path.with_suffix(path.suffix+'.new')
  with temp.open('x') as f:json.dump(doc,f,indent=2,allow_nan=False)
  temp.replace(path)
 return row

def main():
 import argparse
 from .r32 import SOFTWARE
 ap=argparse.ArgumentParser(description='Declare an immutable R3.2 collection session before recording')
 ap.add_argument('--sessions',type=Path,default=SOFTWARE/'data/r32/sessions.json');ap.add_argument('--session',required=True);ap.add_argument('--split',required=True,choices=['train','val','test'])
 a=ap.parse_args();print(json.dumps(declare_session(a.sessions,a.session,a.split),indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
