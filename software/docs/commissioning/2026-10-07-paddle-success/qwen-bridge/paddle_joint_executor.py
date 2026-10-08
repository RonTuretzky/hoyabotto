"""Observed pickup profile, run inside the sole owner; never opens a servo port.

One command moves any set of right-arm joints together: the 40-tick ramp is scaled per joint so
all of them reach their targets on the same step. A closing gripper (contact mode) is still
commanded alone. A joint that stops short under load gets bounded goal corrections; if it is
still short once those are spent, the segment ends 'settled_short' (holding, endpoint not
reached) instead of faulting. A joint that never comes to rest still fails at the deadline.

A command may carry 'waypoints' (a list of target sets) instead of 'positions'. The ramp passes
through intermediate waypoints without stopping and settles only at the last one, so a long
motion is continuous. halt() stops advancing at any time and holds the last commanded goals;
the owner uses it for robot_halt_motion and for replacing a running motion with a new one.

Contact guard (arm joints; the gripper keeps its close-until-resistance logic): there is no self-collision
model, so a sustained high load on a joint lagging >= 20 ticks behind its command is treated as contact. The joint's goal is
pulled back to where it is (it stops pushing), the others hold, and the motion ends 'contact_halt' without
releasing anything. Settle corrections never push a loaded joint, nor one that did not move after a correction.
"""
import math,time
ENVELOPE=96          # max |present - commanded goal| at any tick
MARGIN=40            # saved-range margin for targets and corrected goals
EDGE=4               # a joint may start (and hold) anywhere this far inside its saved range
SEGMENT=341          # max ticks per joint per leg (command or waypoint-to-waypoint)
MAX_WAYPOINTS=24
STEP,CONTACT_STEP=40,10
CORRECTION_STEP=40   # max goal change per correction write
CORRECTION_ROOM=90   # a correction never commands more than this from the present position
MAX_OVERDRIVE=57     # max |held goal - target| reached through corrections
MAX_CORRECTIONS=3
CORRECTION_BUDGET_S=5
CONTACT_LOAD=600     # settle corrections stop pushing a joint loaded this much (fault/release stays at 800)
CONTACT_HALT_LOAD=350  # a joint loaded this much AND lagging >= CONTACT_PUSH_TICKS is blocked: halt and hold (real contacts on 2026-10-08 peaked at 300-400 and only tripped the 96-tick following-error release)
CONTACT_PUSH_TICKS=50  # lag behind the command that, together with CONTACT_LOAD, means blocked (normal ramps lag 25-40)
def tolerance(n):return 30 if n.endswith('gripper') else 57
class PaddleJointExecutor:
 def __init__(self,joints,ranges,write,clock=time.monotonic,wall=time.time):
  self.joints=list(joints);self.ranges=ranges;self.write=write;self.clock=clock;self.wall=wall
  self.active=False;self.samples=[];self.diagnostics={}
 def start(self,c,current,session_started,held_goals=None):
  path=c.get('waypoints') is not None;legs=c.get('waypoints') if path else [c.get('positions')];duration=c.get('duration_s')
  if type(c.get('id')) is not int or c['id']<=0 or c.get('session_started')!=session_started:raise ValueError('Bound-session command required')
  if not isinstance(legs,list) or not 1<=len(legs)<=MAX_WAYPOINTS:raise ValueError(f'Pickup path needs 1..{MAX_WAYPOINTS} waypoints')
  for p in legs:
   if not isinstance(p,dict) or not p or set(p)!=set(self.joints):raise ValueError('Pickup command joints must match the executor joints')
   if any(not n.startswith(('right_arm_','left_arm_')) or type(t) is not int for n,t in p.items()):raise ValueError('Integer arm-joint target required')
  if type(duration) not in (int,float) or not math.isfinite(duration) or not 0<duration<=(60 if path else 25):raise ValueError('Finite duration (0,25] required (paths: (0,60])')
  goals={}
  for n in self.joints:
   q=current[n];lo,hi=self.ranges[n]
   # Starting near a limit is allowed; every target is 40 ticks inside, so the move can only head back inward.
   if not lo+EDGE<=q<=hi-EDGE:raise ValueError(n+': current position outside saved range')
   g=(held_goals or {}).get(n,q)
   if type(g) is not int or abs(g-q)>ENVELOPE or not lo+EDGE<=g<=hi-EDGE:raise ValueError(n+': pickup previous held goal outside envelope')
   goals[n]=g
  previous=dict(current)
  for i,p in enumerate(legs):
   for n,t in p.items():
    lo,hi=self.ranges[n]
    if not lo+MARGIN<=t<=hi-MARGIN or abs(t-previous[n])>SEGMENT:raise ValueError(f'{n}: waypoint {i+1} must be 40 ticks inside the saved range and at most 341 ticks from the previous point')
   previous=p
  if not path and not all(2<abs(t-current[n]) for n,t in legs[0].items()):raise ValueError('Each joint in a pickup move must travel 3..341 ticks')
  if path and not any(abs(p[n]-current[n])>2 for p in legs for n in self.joints):raise ValueError('Pickup path does not move any joint')
  self.contact=any(n.endswith('gripper') and t<current[n] for n,t in legs[0].items())
  if self.contact and (len(legs[0])!=1 or path):raise ValueError('Gripper closure must be commanded alone, not in a path')
  step=CONTACT_STEP if self.contact else STEP;origin=goals;self.leg_steps=[]
  for p in legs:
   self.leg_steps.append(max(1,max(math.ceil(max(abs(t-origin[n]),abs(t-current[n]) if p is legs[0] else 0)/step) for n,t in p.items())));origin=p
  self.steps=sum(self.leg_steps)
  self.interval=1.5 if self.contact else max(.4,float(duration)/self.steps)
  base=self.steps*self.interval+3
  if base>(55 if self.contact else 80 if path else 28):raise ValueError('Pickup motion exceeds API completion deadline; shorten it or its duration')
  self.duration=base-3;self.deadline=base+(0 if self.contact else CORRECTION_BUDGET_S)
  self.legs=[dict(p) for p in legs];self.leg=0;self.final_target=dict(legs[-1]);self.goal=goals
  self.bias=dict.fromkeys(self.joints,0);self.corrections=dict.fromkeys(self.joints,0)
  self.exhausted={n:n.endswith('gripper') for n in self.joints} # grippers get no corrections
  self.stable=dict.fromkeys(self.joints,0);self.still=dict.fromkeys(self.joints,0);self.last_q={n:current[n] for n in self.joints}
  self.loaded=dict.fromkeys(self.joints,0);self.correction_from={};self.possible_contact=set()
  self.set_leg(0)
  self.first_step=True;self.started=self.last_tick=self.last_write=self.clock();self.quiet_since=None;self.last_sample=None;self.command_id=c['id'];self.active=True
  self.progress_q={n:current[n] for n in self.joints};self.progress_at=dict.fromkeys(self.joints,self.started)
  return {'accepted':c['id'],'phase':'moving','execution_profile':'paddle-success-v1','direct_start_positions':{n:current[n] for n in self.joints},'direct_requested_targets':self.final_target,'motion_waypoints':len(legs),'direct_deadline_s':self.deadline,'direct_duration_s':self.duration,'grasp_verified':False,'closure_outcome':None,'endpoint_reached':None,'settle_residual_ticks':None}
 def set_leg(self,i):
  # The longest ramp of this leg moves full steps (as in the pilot); shorter ramps are spread over the same number of steps.
  self.leg=i;self.targets=dict(self.legs[i]);step=CONTACT_STEP if self.contact else STEP
  longest=max(abs(t-self.goal[n]) for n,t in self.targets.items());count=max(1,math.ceil(longest/step))
  self.step={n:step if abs(t-self.goal[n])==longest else max(1,math.ceil(abs(t-self.goal[n])/count)) for n,t in self.targets.items()}
 def halt(self,current):
  """Stop advancing now and hold the last commanded goals (at most one step ahead of the arm)."""
  self.legs=[dict(self.goal)];self.targets=dict(self.goal);self.bias=dict.fromkeys(self.joints,0)
  return self.finish(current,'halted')
 def aim(self,n):return self.targets[n]+self.bias[n]
 def correct(self,n,q):
  # Overdrive the held goal by the measured residual, inside every envelope; never past the margin.
  t=self.targets[n];lo,hi=self.ranges[n];g=self.goal[n]
  want=t+max(-MAX_OVERDRIVE,min(MAX_OVERDRIVE,self.bias[n]+t-q))
  want=max(q-CORRECTION_ROOM,min(q+CORRECTION_ROOM,want))
  want=max(g-CORRECTION_STEP,min(g+CORRECTION_STEP,want))
  want=max(lo+MARGIN,min(hi-MARGIN,want))
  if want==g or (want-g)*(t-q)<=0:self.exhausted[n]=True;return None
  self.corrections[n]+=1;self.exhausted[n]=self.corrections[n]>=MAX_CORRECTIONS
  self.goal[n]=want;self.bias[n]=want-t;return want
 def finish(self,current,outcome):
  self.active=False
  residual={n:current[n]-self.final_target[n] for n in self.joints}
  return {'completed':self.command_id,'phase':'holding','direct_actual_positions':current,'grasp_verified':False,'closure_outcome':outcome,'endpoint_reached':outcome=='endpoint_settled','settle_residual_ticks':residual,'possible_contact_joints':sorted(self.possible_contact),'contact':None,'direct_settle_diagnostics':self.diagnostics}
 def tick(self,current,telemetry_at,rows=None):
  now=self.clock();rows=rows or {}
  if not 0<=self.wall()-telemetry_at<=1.0 or not 0<=now-self.last_tick<=1.0:raise RuntimeError('Pickup telemetry/watchdog expired')
  self.last_tick=now
  for n in self.joints:
   if abs(current[n]-self.goal[n])>ENVELOPE:raise RuntimeError('Pickup following error exceeds96ticks: '+n)
  if now-self.started>self.deadline:raise RuntimeError('Pickup segment failed to settle before deadline')
  fresh=telemetry_at!=self.last_sample;quiet={}
  for n in self.joints:
   q=current[n];row=rows.get(n,{})
   still=abs(row.get('Present_Velocity',999))<3 and abs(q-self.last_q[n])<=3
   # The firmware Moving flag compares against the commanded goal, so it cannot clear while a correction overdrives it.
   quiet[n]=still and (row.get('Moving')==0 or self.bias[n]!=0)
   if fresh:
    self.last_q[n]=q;self.still[n]=self.still[n]+1 if still else 0
    self.stable[n]=self.stable[n]+1 if quiet[n] and self.goal[n]==self.aim(n) and abs(q-self.targets[n])<=tolerance(n) else 0
  if fresh:
   if self.contact:self.quiet_since=(self.quiet_since if self.quiet_since is not None else now) if quiet[self.joints[0]] else None
   self.last_sample=telemetry_at
  c=self.joints[0]
  contact_stop=self.contact and self.quiet_since is not None and now-self.quiet_since>=.3 and current[c]-self.goal[c]>=40
  self.diagnostics={'leg':self.leg+1,'legs':len(self.legs),'elapsed_s':round(now-self.started,2),'final_targets':self.final_target,'joints':{n:{'goal_ticks':self.goal[n],'current_ticks':current[n],'target_ticks':self.targets[n],'following_error_ticks':current[n]-self.goal[n],'stable_samples':self.stable[n],'still_samples':self.still[n],'overdrive_ticks':self.bias[n],'corrections':self.corrections[n],'load':rows.get(n,{}).get('Present_Load')} for n in self.joints},'contact_stop':contact_stop}
  self.samples.append(dict(self.diagnostics,t=now));self.samples=self.samples[-128:]
  if not self.contact:
   hits={}
   for n in self.joints:
    if n.endswith('gripper'):continue
    load=abs(rows.get(n,{}).get('Present_Load',0))
    if fresh:self.loaded[n]=self.loaded[n]+1 if load>=CONTACT_HALT_LOAD else 0
    # Lagging >=20 ticks behind its command under high load: blocked. (A loaded joint that keeps up is not.)
    if self.loaded[n]>=2 and abs(self.goal[n]-current[n])>=CONTACT_PUSH_TICKS:hits[n]=load
   if hits:
    backoff={}
    for n in hits:
     lo,hi=self.ranges[n];backoff[n]=max(lo+EDGE,min(hi-EDGE,current[n]))
    self.write(backoff);self.goal.update(backoff)
    out=self.finish(current,'contact_halt')
    out['contact']={n:{'load':hits[n],'position_ticks':current[n]} for n in hits}
    out['contact_note']=f'Load >= {CONTACT_HALT_LOAD} on a joint lagging >= {CONTACT_PUSH_TICKS} ticks behind its command: treated as contact with something (there is no self-collision model). That joint stopped pushing and holds where it is; nothing was released.'
    return out
  # Pass through intermediate waypoints without stopping: once the ramp reaches one, aim at the next.
  if self.leg<len(self.legs)-1 and all(self.goal[n]==self.targets[n] for n in self.joints):self.set_leg(self.leg+1)
  final=self.leg==len(self.legs)-1
  settled=final and all(self.stable[n]>=3 for n in self.joints)
  if contact_stop or (settled and (now-self.last_write>=self.interval if not self.contact else self.quiet_since is not None and now-self.quiet_since>=.3)):
   return self.finish(current,'stationary_closure_unverified' if contact_stop else 'endpoint_settled')
  # A closing gripper that is stationary on its final target step but outside tolerance and short of the 40-tick
  # contact_stop band (e.g. 33 ticks short, light load) has stopped: report it as settled_short and keep holding.
  # One that keeps moving still trips 'did not become stationary' below.
  if self.contact and final and self.goal[c]==self.targets[c] and self.quiet_since is not None and now-self.quiet_since>=.3:
   return self.finish(current,'settled_short')
  ramp_done=all(self.goal[n]==self.aim(n) for n in self.joints)
  if not self.contact and final and ramp_done and now-self.last_write>=self.interval and all(self.still[n]>=3 for n in self.joints):
   # Everything is at rest: correct joints that are not settled, or finish if none can be corrected further.
   pending=[n for n in self.joints if self.stable[n]<3]
   for n in pending:
    load=abs(rows.get(n,{}).get('Present_Load',0))
    if not self.exhausted[n] and (load>=CONTACT_LOAD or (n in self.correction_from and abs(current[n]-self.correction_from[n])<3)):
     self.exhausted[n]=True;self.possible_contact.add(n)  # do not push into something
   writes={n:g for n in pending if not self.exhausted[n] and (g:=self.correct(n,current[n])) is not None}
   self.correction_from.update({n:current[n] for n in writes})
   if writes:
    self.write(writes);self.last_write=now
    for n in writes:self.stable[n]=self.still[n]=0
    return {'phase':'moving','direct_settle_diagnostics':self.diagnostics}
   if all(self.exhausted[n] for n in pending):
    outside=any(abs(current[n]-self.targets[n])>tolerance(n) for n in self.joints)
    return self.finish(current,'settled_short' if outside else 'endpoint_settled')
  for n in self.joints:
   q=current[n]
   if abs(q-self.progress_q[n])>=8:self.progress_q[n]=q;self.progress_at[n]=now
   if n.endswith('gripper') and not self.contact and abs(q-self.goal[n])>30 and now-self.progress_at[n]>1:raise RuntimeError('Pickup gripper no-progress guard')
  if self.contact:advance=self.goal[c]!=self.targets[c] and (self.first_step or self.quiet_since is not None and now-self.quiet_since>=.3)
  else:advance=not ramp_done and (self.first_step or now-self.last_write>=self.interval)
  if self.contact and not self.first_step and not advance and now-self.last_write>1.5:raise RuntimeError('Pickup closure did not become stationary')
  if advance:
   writes={}
   for n in self.joints:
    delta=self.aim(n)-self.goal[n]
    if delta:self.goal[n]+=max(-self.step[n],min(self.step[n],delta));writes[n]=self.goal[n];self.stable[n]=self.still[n]=0
   self.write(writes);self.last_write=now;self.first_step=False;self.quiet_since=None
  return {'phase':'moving','direct_settle_diagnostics':self.diagnostics}

 def pause(self,seconds):
  for field in ['started','last_write','quiet_since']:
   value=getattr(self,field,None)
   if value is not None:setattr(self,field,value+seconds)
  self.progress_at={n:v+seconds for n,v in self.progress_at.items()}
  self.last_tick=self.clock();self.stable=dict.fromkeys(self.joints,0);self.still=dict.fromkeys(self.joints,0);self.last_sample=None
