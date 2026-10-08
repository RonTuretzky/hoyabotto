"""Guarded base drive inside the sole owner, ported from the validated drive-pulse.py; never opens a servo port.

One command is one bounded velocity pulse: switch both wheels to velocity mode, drive for at most
MAX_DURATION_S, command zero, wait until they settle, turn torque off, confirm they are not rolling, and
restore the saved Operating_Mode/Acceleration/Torque_Limit/Lock. Wheels never stay powered between commands.
A stale phone feed brakes the pulse early. Any failed check raises; the owner then calls abort(), which zeroes velocity and turns torque off.
"""
import math,time
WHEELS=('base_left_wheel','base_right_wheel')
WHEEL_RADIUS_M=0.0635   # 5-inch walker wheels per upstream docs/hardware/getting_started/assemble_2wheel.md (upstream's 0.05 default is the 4-inch omni wheel)
WHEELBASE_M=0.45        # wheel bodies at y=+-0.225 in upstream xlerobot.xml (vendored farm/sim/assets/xlerobot); not tape-measured on this cart
TICKS_PER_REV=4096
MAX_WHEEL_M_S=0.02                     # 205 ticks/s with the 5-inch wheel; the 261 ticks/s validated on 4 October assumed a 0.05 m radius
MAX_DURATION_S=3.0
BRAKE_SETTLE_S=1.5                     # wheels stay powered at zero velocity until two fresh samples agree they are still
SAVED=('Operating_Mode','Acceleration','Torque_Limit','Lock')
def wheel_ticks_per_s(linear_m_s,angular_rad_s):
 """Differential drive; the left wheel is mounted mirrored, so forward is left negative, right positive."""
 scale=TICKS_PER_REV/(2*math.pi*WHEEL_RADIUS_M)
 left=linear_m_s-angular_rad_s*WHEELBASE_M/2;right=linear_m_s+angular_rad_s*WHEELBASE_M/2
 return {'base_left_wheel':-round(left*scale),'base_right_wheel':round(right*scale)}
def wrap(delta):return ((delta+2048)%4096)-2048
class WheelPulseExecutor:
 joints=WHEELS
 def __init__(self,read,write,camera_fresh,clock=time.monotonic,wall=time.time):
  self.read=read;self.write=write;self.camera_fresh=camera_fresh;self.clock=clock;self.wall=wall
  self.active=False;self.samples=[];self.diagnostics={};self.saved={};self.powered=False
 def start(self,c,rows,session_started):
  if type(c.get('id')) is not int or c['id']<=0 or c.get('session_started')!=session_started:raise ValueError('Bound-session command required')
  self.commands=self.check_request(c);lin,ang,duration=c['linear_m_s'],c.get('angular_rad_s',0),c['duration_s']
  for n in WHEELS:
   r=rows.get(n,{})
   if r.get('Torque_Enable')!=0 or r.get('Status')!=0 or not 100<=r.get('Present_Voltage',0)<=140:raise ValueError(n+': must be released, status 0 and 10..14 V before driving')
  if not self.camera_fresh():raise ValueError('Phone camera stale; base drive refused')
  self.saved={n:{f:self.read(n,f) for f in SAVED} for n in WHEELS}
  self.before={n:rows[n]['Present_Position'] for n in WHEELS}
  for n in WHEELS:
   self.write(n,'Goal_Velocity',0);self.write(n,'Lock',0);self.write(n,'Operating_Mode',1);self.write(n,'Acceleration',10)
  self.powered=True
  for n in WHEELS:self.write(n,'Torque_Enable',1)
  for n,v in self.commands.items():self.write(n,'Goal_Velocity',v)
  self.phase='driving';self.started=self.last_tick=self.clock();self.duration=float(duration);self.deadline=self.duration+BRAKE_SETTLE_S+1.0+2
  self.window=[];self.released=[];self.last_sample=None;self.stopped_early=None;self.command_id=c['id'];self.active=True
  return {'accepted':c['id'],'phase':'moving','base_pulse':{'commands_ticks_per_s':self.commands,'duration_s':self.duration,'linear_m_s':lin,'angular_rad_s':ang},'base_result':None}
 @staticmethod
 def check_request(c):
  """Validate speeds and duration; returns per-wheel raw velocity commands."""
  lin,ang,duration=c.get('linear_m_s'),c.get('angular_rad_s',0),c.get('duration_s')
  if any(type(v) not in (int,float) or not math.isfinite(v) for v in (lin,ang,duration)):raise ValueError('Finite linear_m_s, angular_rad_s and duration_s required')
  if not 0<duration<=MAX_DURATION_S:raise ValueError(f'Base pulse duration must be in (0,{MAX_DURATION_S}] s')
  limit=MAX_WHEEL_M_S*TICKS_PER_REV/(2*math.pi*WHEEL_RADIUS_M)+.5
  commands=wheel_ticks_per_s(lin,ang)
  if any(abs(v)>limit for v in commands.values()):raise ValueError(f'Each wheel is limited to {MAX_WHEEL_M_S} m/s; lower linear_m_s or angular_rad_s')
  if not any(commands.values()):raise ValueError('Base pulse needs a nonzero linear_m_s or angular_rad_s')
  return commands
 def tick(self,current,telemetry_at,rows=None):
  now=self.clock();rows=rows or {}
  if not 0<=now-self.last_tick<=.3 or not 0<=self.wall()-telemetry_at<=.5:raise RuntimeError('Base drive telemetry/watchdog gap')
  self.last_tick=now
  if now-self.started>self.deadline:raise RuntimeError('Base pulse exceeded its deadline')
  for n in WHEELS:
   r=rows[n]
   if r.get('Status')!=0 or abs(r.get('Present_Load',999))>500 or abs(r.get('Present_Velocity',999))>400 or not 100<=r.get('Present_Voltage',0)<=140:raise RuntimeError(f"{n}: base health check failed (status {r.get('Status')}, load {r.get('Present_Load')}, velocity {r.get('Present_Velocity')}, voltage {r.get('Present_Voltage')})")
  fresh=telemetry_at!=self.last_sample;self.last_sample=telemetry_at
  sample={'t':now-self.started,'position':{n:current[n] for n in WHEELS},'velocity':{n:rows[n].get('Present_Velocity') for n in WHEELS},'phase':self.phase}
  self.diagnostics=sample;self.samples.append(sample);self.samples=self.samples[-128:]
  if self.phase=='driving':
   # A stale phone feed ends the pulse early (brake, release) rather than faulting the whole owner.
   if not self.camera_fresh():self.stopped_early='phone camera stale'
   if self.stopped_early or now-self.started>=self.duration:
    for n in WHEELS:self.write(n,'Goal_Velocity',0)
    self.phase='braking';self.stopped_at=now
   return {'phase':'moving','base_drive_phase':self.phase}
  if not fresh:return {'phase':'moving','base_drive_phase':self.phase}
  if self.phase=='braking':
   self.window.append(sample)
   if self.settled(self.window[-2:]):
    for n in WHEELS:self.write(n,'Torque_Enable',0)
    self.powered=False;self.phase='released';self.released_at=now
   elif now-self.stopped_at>BRAKE_SETTLE_S:
    recent=[{n:(x['position'][n],x['velocity'][n]) for n in WHEELS} for x in self.window[-4:]]
    raise RuntimeError(f"Wheels did not settle within {BRAKE_SETTLE_S} s of braking: last (position, velocity) samples {recent}")
   return {'phase':'moving','base_drive_phase':self.phase}
  self.released.append(sample)
  if len(self.released)<5:return {'phase':'moving','base_drive_phase':self.phase}
  if not self.settled(self.released[-5:],spread=5,use_velocity=False):
   moved={n:max(abs(wrap(x['position'][n]-self.released[-5]['position'][n])) for x in self.released[-5:]) for n in WHEELS}
   raise RuntimeError(f"Wheels rolling after release: position change over 5 released samples {moved} ticks (limit 5); velocity readings {[x['velocity'] for x in self.released[-5:]]}")
  self.restore()
  delta={n:wrap(current[n]-self.before[n]) for n in WHEELS}
  self.active=False
  return {'completed':self.command_id,'phase':'idle','base_drive_phase':'done','base_result':{'wheel_delta_ticks':delta,
   'estimated_wheel_travel_cm':{n:abs(v)*2*math.pi*WHEEL_RADIUS_M/TICKS_PER_REV*100 for n,v in delta.items()},'pulse_s':self.stopped_at-self.started,
   'stopped_early':self.stopped_early,'released':True,'settings_restored':True,'odometry_note':'wheel encoder estimate only; slip and floor contact unverified'}}
 @staticmethod
 def settled(samples,spread=3,use_velocity=True):
  # Released Feetech servos report spurious Present_Velocity (e.g. 50) while stationary, so the
  # after-release check judges rolling by encoder position only; braking (still powered) also uses velocity.
  if len(samples)<2:return False
  if use_velocity and any(abs(s['velocity'][n] or 0)>5 for s in samples for n in WHEELS):return False
  return all(abs(wrap(s['position'][n]-samples[0]['position'][n]))<=spread for s in samples for n in WHEELS)
 def restore(self):
  for n in WHEELS:
   if self.read(n,'Torque_Enable')!=0:raise RuntimeError(n+': torque still on before settings restore')
   for f in SAVED:self.write(n,f,self.saved[n][f])
 def pause(self,seconds):raise RuntimeError('Camera pause during base drive; stopping wheels')
 def abort(self):
  """Owner fault/STOP path: zero velocity and torque off on both wheels, then restore settings."""
  errors=[]
  for n in WHEELS:
   for f,v in (('Goal_Velocity',0),('Torque_Enable',0)):
    try:self.write(n,f,v)
    except Exception as e:errors.append(f'{n} {f}: {e}')
  self.active=False;self.powered=bool(errors)
  if not errors and self.saved:
   try:self.restore()
   except Exception as e:errors.append(str(e))
  return errors
