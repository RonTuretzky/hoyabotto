"""R3.2 offline assembly supervisor and rehearsal. Never opens a hardware device."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
from farm.status import Reading, Status

SOFTWARE = Path(__file__).resolve().parents[2]
PROFILE = SOFTWARE / 'profiles/r32-assembly-v0.json'
REVISION = 'R3.2'
STAGES = ('PREFLIGHT', 'PLACE_RESERVOIR', 'PLACE_BASKET_ON_STAND', 'PLACE_CLOTH',
          'PLACE_GROW_PAD', 'GUIDE_TAIL', 'PLACE_RETAINER', 'PLACE_LOADED_BASKET', 'VERIFY_DRY_ASSEMBLY')
CHECKS = {
 'PREFLIGHT': ('calibration_valid', 'cameras_fresh', 'station_measured', 'grips_measured', 'parts_inspected', 'workspace_clear', 'gripper_empty'),
 'PLACE_RESERVOIR': ('reservoir_stable_in_dock',),
 'PLACE_BASKET_ON_STAND': ('basket_stable_on_stand', 'front_accessible'),
 'PLACE_CLOTH': ('one_sheet', 'cloth_flat', 'tail_free'),
 'PLACE_GROW_PAD': ('one_pad', 'pad_covers_target', 'cloth_unmoved'),
 'GUIDE_TAIL': ('tail_unfolded', 'cloth_unmoved', 'gripper_empty'),
 'PLACE_RETAINER': ('release_method_validated', 'ring_seated', 'cloth_unmoved'),
 'PLACE_LOADED_BASKET': ('materials_stable', 'tail_inside', 'basket_seated'),
 'VERIFY_DRY_ASSEMBLY': ('basket_seated', 'pad_flat', 'tail_inside', 'ring_matches_variant', 'gripper_empty'),
}
PLACEMENTS = set(STAGES[1:-1]) - {'GUIDE_TAIL'}
FAULTS = ('none', 'no_pick', 'double_cloth', 'tail_folded', 'ring_misaligned', 'seat_unknown', 'stale', 'stop', 'deadline')

def load_profile(path=PROFILE):
 p = json.loads(Path(path).read_text())
 if p.get('revision') != REVISION or p.get('kind') != 'r32-offline-training-preparation':
  raise ValueError('R3.2 training profile required; old R2a/watering profiles are incompatible')
 if p.get('execution_enabled') is not False:
  raise ValueError('This offline module cannot enable hardware execution')
 if len(p.get('state_joints', [])) != 6 or len(set(p['state_joints'])) != 6:
  raise ValueError('Exactly six distinct arm joints required')
 return p

def part_problems(profile, base=SOFTWARE):
 problems=[]
 for name, expected in profile['part_hashes'].items():
  path=Path(base)/profile['parts_dir']/name
  if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
   problems.append('missing or wrong revision: '+name)
 return problems

class Blocked(RuntimeError): pass

class Supervisor:
 """One attempt per ordered dry stage. Failure ends the episode, with no recovery motion."""
 def __init__(self, with_retainer=False, max_age=1., deadline=60., started_at=0.):
  self.order=[s for s in STAGES if with_retainer or s!='PLACE_RETAINER']
  self.index=0;self.entered=started_at;self.max_age=max_age;self.deadline=deadline
  self.failed=False;self.events=[];self.held='UNKNOWN';self.phase='idle'
 @property
 def stage(self): return 'FAILED' if self.failed else (self.order[self.index] if self.index<len(self.order) else 'DONE')
 def block(self, why):
  self.events.append({'stage':self.stage,'event':'STOP_HOLD','reason':why,'held':self.held})
  self.failed=True
  raise Blocked(why)
 def require(self, evidence, names, now):
  if self.stage in ('FAILED','DONE'):raise Blocked('Episode ended; no retries or resume')
  if not math.isfinite(now) or now<self.entered or now-self.entered>self.deadline:self.block('stage deadline or invalid clock')
  for name in names:
   r=evidence.get(name)
   if (not isinstance(r,Reading) or r.status is not Status.OK or r.value is not True
       or not math.isfinite(r.t) or not 0<=now-r.t<=self.max_age):self.block('missing, false, unknown or stale: '+name)
 def action(self, action, evidence, now):
  # Evidence is supplied by the future observer. No device calls occur here.
  if self.stage not in PLACEMENTS:self.block('placement action outside placement stage')
  transitions={
   'grasp':('idle',('gripper_empty',),'UNKNOWN','grasped'),
   'lift':('grasped',('grasp_confirmed',),'HELD','lifted'),
   'transfer':('lifted',('held_after_lift',)+(('one_sheet',) if self.stage=='PLACE_CLOTH' else ()),'HELD','transferred'),
   'release':('transferred',('held_after_transfer','supported_before_release'),'UNKNOWN','released'),
   'retreat':('released',('jaws_clear','part_stays'),'EMPTY','finished'),
  }
  if action not in transitions:self.block('unknown action')
  phase,names,held,next_phase=transitions[action]
  if self.phase!=phase:self.block('out-of-order action: '+action)
  if action=='release' and self.held!='HELD':self.block('cannot release unknown held object')
  if self.stage=='PLACE_RETAINER' and action=='release':
   names=('held_after_transfer','release_method_validated','release_zone_verified')
  self.require(evidence,names,now)
  self.held=held;self.phase=next_phase
  self.events.append({'stage':self.stage,'event':'INTENT_ONLY','action':action})
 def advance(self, evidence, now):
  self.require(evidence,CHECKS[self.stage] if self.stage in CHECKS else (),now)
  if self.stage in PLACEMENTS and self.phase!='finished':self.block('placement unfinished')
  if self.stage=='PREFLIGHT':self.held='EMPTY'
  self.events.append({'stage':self.stage,'event':'STAGE_VERIFIED'})
  self.index+=1;self.entered=now;self.phase='idle'

def rehearsal(with_retainer=False, fault='none'):
 if fault not in FAULTS:raise ValueError('unknown fault')
 if fault=='ring_misaligned' and not with_retainer:raise ValueError('ring fault needs retainer variant')
 m=Supervisor(with_retainer);now=0.
 def evidence(names):return {n:Reading(True,Status.OK,t=now,source='synthetic_rehearsal') for n in names}
 try:
  while m.stage!='DONE':
   stage=m.stage;now+=.1
   if stage in PLACEMENTS:
    for action in ['grasp','lift','transfer','release','retreat']:
     e=evidence(('gripper_empty','grasp_confirmed','held_after_lift','held_after_transfer','supported_before_release','jaws_clear','part_stays','one_sheet','release_method_validated','release_zone_verified'))
     if stage=='PLACE_RESERVOIR':
      if fault=='no_pick' and action=='lift':e['grasp_confirmed'].value=False
      if fault=='seat_unknown' and action=='release':e['supported_before_release'].status=Status.UNKNOWN
      if fault=='stop' and action=='transfer':m.block('STOP requested')
      if fault=='deadline' and action=='transfer':now+=61
     if fault=='double_cloth' and stage=='PLACE_CLOTH' and action=='transfer':e['one_sheet'].value=False
     m.action(action,e,now)
   e=evidence(CHECKS[stage])
   if fault=='stale' and stage=='PREFLIGHT':e['cameras_fresh'].t=now-2
   if fault=='tail_folded' and stage=='GUIDE_TAIL':e['tail_unfolded'].value=False
   if fault=='ring_misaligned' and stage=='PLACE_RETAINER':e['ring_seated'].value=False
   m.advance(e,now)
 except Blocked as exc:reason=str(exc)
 else:reason=''
 return {'revision':REVISION,'simulated':True,'training_eligible':False,'physical_success':False,
         'result':'SIMULATED_COMPLETE' if m.stage=='DONE' else 'SIMULATED_STOPPED','fault':fault,'reason':reason,'events':m.events}

def main():
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('--profile',type=Path,default=PROFILE)
 a.add_argument('--rehearse',action='store_true');a.add_argument('--with-retainer',action='store_true');a.add_argument('--fault',choices=FAULTS,default='none');a.add_argument('--out',type=Path)
 opt=a.parse_args();p=load_profile(opt.profile);issues=part_problems(p)
 result=rehearsal(opt.with_retainer,opt.fault) if opt.rehearse and not issues else {'revision':REVISION,'part_problems':issues,'hardware_execution':False,'stages':STAGES,'profile':str(opt.profile)}
 if opt.out:
  opt.out.parent.mkdir(parents=True,exist_ok=True)
  with opt.out.open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result,indent=2));return 2 if issues or result.get('result')=='SIMULATED_STOPPED' else 0
if __name__=='__main__':raise SystemExit(main())
