"""Observed pickup profile, run inside the sole owner; never opens a servo port."""
import math,time
class PaddleJointExecutor:
 def __init__(self,joints,ranges,write,clock=time.monotonic,wall=time.time):
  self.joints=joints;self.ranges=ranges;self.write=write;self.clock=clock;self.wall=wall
  self.active=False;self.samples=[];self.diagnostics={}
 def start(self,c,current,session_started):
  p=c.get('positions');duration=c.get('duration_s')
  if type(c.get('id')) is not int or c['id']<=0 or c.get('session_started')!=session_started:raise ValueError('Bound-session command required')
  if not isinstance(p,dict) or len(p)!=1 or set(p)!=set(self.joints):raise ValueError('Pickup profile requires one joint per command')
  self.n=next(iter(p));q=current[self.n];target=p[self.n];lo,hi=self.ranges[self.n]
  if not self.n.startswith('right_arm_') or type(target)is not int:raise ValueError('Integer right-arm target required')
  if type(duration) not in (int,float) or not math.isfinite(duration) or not 0<duration<=25:raise ValueError('Finite duration (0,25] required')
  if not lo+40<=q<=hi-40 or not lo+40<=target<=hi-40 or not 2<abs(target-q)<=341:raise ValueError('Pickup target requires 40-tick margin and 3..341-tick segment')
  self.contact=self.n.endswith('gripper') and target<q
  self.step=10 if self.contact else 40
  self.interval=1.5 if self.contact else max(.4,float(duration)/math.ceil(abs(target-q)/40))
  self.deadline=math.ceil(abs(target-q)/self.step)*self.interval+3
  if self.deadline>(55 if self.contact else 28):raise ValueError('Pickup segment exceeds API completion deadline; shorten segment/duration')
  self.duration=self.deadline-3
  self.target=target;self.goal=q;self.start_q=q;self.started=self.last_tick=self.last_write=self.clock();self.last_q=q;self.stable=0;self.quiet_since=None;self.last_sample=None;self.command_id=c['id'];self.active=True
  self.progress_q=q;self.progress_at=self.started
  return {'accepted':c['id'],'phase':'moving','execution_profile':'paddle-success-v1','direct_start_positions':{self.n:q},'direct_requested_targets':p,'direct_deadline_s':self.deadline,'direct_duration_s':self.deadline-3,'grasp_verified':False}
 def tick(self,current,telemetry_at,rows=None):
  now=self.clock();q=current[self.n];row=(rows or {}).get(self.n,{})
  if not 0<=self.wall()-telemetry_at<=.2 or not 0<=now-self.last_tick<=.2:raise RuntimeError('Pickup telemetry/watchdog expired')
  self.last_tick=now
  if abs(q-self.goal)>96:raise RuntimeError('Pickup following error exceeds96ticks')
  if now-self.started>self.deadline:raise RuntimeError('Pickup segment failed to settle before deadline')
  quiet=row.get('Moving')==0 and abs(row.get('Present_Velocity',999))<3 and abs(q-self.last_q)<=3
  if telemetry_at!=self.last_sample:
   self.quiet_since=(self.quiet_since if self.quiet_since is not None else now) if quiet else None
   self.last_q=q;self.last_sample=telemetry_at
   tolerance=30 if self.n.endswith('gripper') else 57
   self.stable=self.stable+1 if quiet and self.goal==self.target and abs(q-self.target)<=tolerance else 0
  contact_stop=self.contact and self.quiet_since is not None and now-self.quiet_since>=.3 and q-self.goal>=40
  self.diagnostics={'goal_ticks':self.goal,'current_ticks':q,'target_ticks':self.target,'following_error_ticks':q-self.goal,'stable_samples':self.stable,'contact_stop':contact_stop}
  self.samples.append(dict(self.diagnostics));self.samples=self.samples[-128:]
  if contact_stop or (self.stable>=3 and (not self.contact or self.quiet_since is not None and now-self.quiet_since>=.3)):
   self.active=False
   return {'completed':self.command_id,'phase':'holding','direct_actual_positions':current,'grasp_verified':False,'closure_outcome':'stationary_closure_unverified' if contact_stop else 'endpoint_settled','direct_settle_diagnostics':self.diagnostics}
  if abs(q-self.progress_q)>=8:self.progress_q=q;self.progress_at=now
  if self.n.endswith('gripper') and not self.contact and abs(q-self.goal)>30 and now-self.progress_at>1:raise RuntimeError('Pickup gripper no-progress guard')
  advance=self.goal!=self.target and (self.goal==self.start_q or (self.contact and self.quiet_since is not None and now-self.quiet_since>=.3) or (not self.contact and now-self.last_write>=self.interval))
  if self.contact and self.goal!=self.start_q and not advance and now-self.last_write>1.5:raise RuntimeError('Pickup closure did not become stationary')
  if advance:
   delta=self.target-self.goal;self.goal+=max(-self.step,min(self.step,delta));self.write({self.n:self.goal});self.last_write=now;self.quiet_since=None;self.stable=0
  return {'phase':'moving','direct_settle_diagnostics':self.diagnostics}
