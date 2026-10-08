"""Right-gripper bounded measured-progress waypoints; no bus ownership/activation."""
import math,time
class GripperWaypointExecutor:
 def __init__(self,joints,ranges,write,clock=time.monotonic,wall=time.time):
  if joints!=['right_arm_gripper']:raise ValueError('Measured waypoint controller is right-gripper only')
  self.joints=joints;self.ranges=ranges;self.write=write;self.clock=clock;self.wall=wall;self.active=False;self.samples=[];self.diagnostics={}
 def start(self,c,current,session_started):
  n=self.joints[0];q=current[n];target=c.get('positions',{}).get(n);duration=c.get('duration_s')
  if type(c.get('id')) is not int or c['id']<=0 or c.get('session_started')!=session_started:raise ValueError('Invalid/bound-session command required')
  if set(c.get('positions',{}))!={n} or type(target)is not int or type(q)is not int:raise ValueError('Single integer right-gripper target required')
  if type(duration) not in (int,float) or not math.isfinite(duration) or not 0<duration<=25:raise ValueError('Finite duration (0,25] required')
  lo,hi=self.ranges[n]
  if not lo+4<=q<=hi-4 or not lo+4<=target<=hi-4 or abs(target-q)<=2:raise ValueError('Target/current outside margin or no measurable movement')
  self.requested_duration=float(duration)
  self.n=n;self.start_q=q;self.target=target;self.direction=1 if target>q else -1;self.duration=max(float(duration),abs(target-q)/100);self.deadline=max(self.duration,abs(target-q)/50)+4
  self.command_id=c['id'];self.started=self.last_tick=self.last_write=self.progress_at=self.clock();self.progress_q=q;self.goal=q;self.stable=0;self.last_sample=None;self.active=True
  return {'accepted':self.command_id,'phase':'moving','direct_start_positions':{n:q},'direct_requested_targets':{n:target},'direct_duration_s':self.duration,'direct_deadline_s':self.deadline,'gripper_endpoint_tolerance_ticks':20,'gripper_intermediate_advance_error_ticks':20,'gripper_waypoint_ahead_ticks':48}
 def tick(self,current,telemetry_at):
  now=self.clock();q=current[self.n];elapsed=now-self.started
  self.diagnostics={'elapsed_s':elapsed,'start_ticks':self.start_q,'target_ticks':self.target,'current_ticks':q,'goal_ticks':self.goal,'following_error_ticks':q-self.goal,'endpoint_error_ticks':q-self.target,'endpoint_tolerance_ticks':20,'progress_age_s':now-self.progress_at,'deadline_s':self.deadline};self.samples.append(dict(self.diagnostics));self.samples=self.samples[-128:]
  if not 0<=self.wall()-telemetry_at<=.2 or not 0<=now-self.last_tick<=.2:raise RuntimeError('Gripper telemetry/watchdog expired')
  self.last_tick=now
  if abs(q-self.goal)>48 or (q-self.start_q)*self.direction < -3 or (q-self.target)*self.direction>20:raise RuntimeError('Gripper travel/following envelope exceeded')
  if (q-self.progress_q)*self.direction>=3:self.progress_q=q;self.progress_at=now
  if elapsed>self.deadline:raise RuntimeError('Gripper endpoint deadline; error_ticks='+str(q-self.target))
  required_travel=max(3,abs(self.target-self.start_q)-20)
  directed_travel=(q-self.start_q)*self.direction
  final=abs(q-self.target)<=20 and directed_travel>=required_travel and elapsed>=self.duration
  if telemetry_at!=self.last_sample:self.stable=self.stable+1 if final else 0;self.last_sample=telemetry_at
  if self.stable>=3:
   self.active=False;return {'completed':self.command_id,'phase':'holding','direct_actual_positions':dict(current),'gripper_result':{'requested_ticks':self.target,'actual_ticks':q,'endpoint_error_ticks':q-self.target,'endpoint_tolerance_ticks':20,'stable_fresh_samples':self.stable,'directed_travel_ticks':directed_travel,'required_directed_travel_ticks':required_travel,'requested_duration_s':self.requested_duration,'effective_minimum_duration_s':self.duration,'maximum_velocity_ticks_s':100,'jaw_state_verified':False}}
  if (abs(q-self.target)>20 or directed_travel<required_travel) and now-self.progress_at>=1:raise RuntimeError('Gripper no progress for1second; endpoint_error_ticks='+str(q-self.target))
  if abs(q-self.goal)<=20:
   distance=min(48,abs(self.target-q));next_goal=q+self.direction*distance
   # Goal progression <=100ticks/s, while native firmware limits actual speed/acceleration.
   if next_goal!=self.goal and (self.goal==self.start_q or now-self.last_write>=abs(next_goal-self.goal)/100):
    self.write({self.n:next_goal});self.goal=next_goal;self.last_write=now
  return {'phase':'moving','direct_settle_diagnostics':dict(self.diagnostics),'direct_elapsed_s':elapsed}
