"""Direct, local-only MuJoCo sink for upstream LeRobot position actions.

Deliberately separate from the physical register/owner transport. It has no
credentials, socket client, serial connection, or real robot enable path.
"""
import math
import secrets
import threading
import time

from mujoco_simulator import MujocoBus
from upstream import WindowsMapping, from_model
from wheel_pulse_executor import WHEELS, WHEEL_RADIUS_M, WHEELBASE_M


class UpstreamSimulator:
    preview=True
    simulation=True
    teleop_keys={'forward':'w','backward':'s','rotate_left':'a','rotate_right':'d'}
    speed_levels=[{'linear':.1,'angular':30.}]
    speed_index=0

    def __init__(self,model_path=None,*,render=True,start=True):
        self.bus=MujocoBus(model_path,render=render,forward_arms=True)
        self.bus.render_interval=1/30
        self.lock=threading.RLock();self.stopping=threading.Event()
        self.session=None;self.last_input=0.;self.sequence=0;self.neutral_seen=False
        self.reason='Stopped';self.thread=None;self.started=time.monotonic();self.loop_hz=0.
        # Position control is confined to this simulation plant. The legacy
        # physical owner and its rate limits are never modified or instantiated.
        for n in self.bus.map:self.bus.r[n]['Torque_Enable']=1
        for n in WHEELS:
            self.bus.r[n]['Torque_Enable']=1;self.bus.r[n]['Operating_Mode']=1
        for n,(aid,_,_,_) in self.bus.wheel_map.items():
            self.bus.model.actuator_ctrlrange[aid]=[-4,4]
        if start:
            self.thread=threading.Thread(target=self.run,name='upstream-mujoco',daemon=True);self.thread.start()

    def get_observation(self):
        with self.lock:
            return {n+'.pos':from_model(n,float(self.bus.data.qpos[qadr])) for n,(_,qadr,_,_,_) in self.bus.map.items()}

    def _from_keyboard_to_base_action(self,keys):
        # Implements the robot interface expected by the unchanged base mapper.
        k=self.teleop_keys;s=self.speed_levels[self.speed_index]
        return {'x.vel':s['linear']*(int(k['forward'] in keys)-int(k['backward'] in keys)),
                'theta.vel':s['angular']*(int(k['rotate_left'] in keys)-int(k['rotate_right'] in keys))}

    def make_mapping(self):return WindowsMapping(self)

    def status(self):
        return dict(preview=True,simulation=True,phase='local upstream simulation',status_age_s=0.,
            teleop={'active':self.session is not None,'neutral_seen':self.neutral_seen},
            motors={n:dict(r,Torque_Enable=int(self.session is not None)) for n,r in self.bus.r.items()},
            virtual_pose=dict(self.bus.pose),reason=self.reason,
            simulator=dict(engine=self.bus.engine,model='XLeRobot',physics=True,
                model_sha256=self.bus.model_sha256,frame_sequence=self.bus.frame_sequence,
                frame_age_s=round(time.monotonic()-self.bus.last_render,3) if self.bus.last_render else None,
                render_error=self.bus.render_error,control_hz=50,physics_loop_hz=round(self.loop_hz,1),
                limitations='Synthetic calibration; direct upstream position targets; solid box.'))

    def stop(self,reason):
        self.session=None;self.reason=reason;self.neutral_seen=False
        for n,(_,qadr,_,lo,hi) in self.bus.map.items():
            self.bus.r[n]['Goal_Position']=100+(float(self.bus.data.qpos[qadr])-lo)/(hi-lo)*3900
        for n in WHEELS:self.bus.r[n]['Goal_Velocity']=0

    def call(self,path,body=None,timeout=None):
        with self.lock:
            if self.stopping.is_set():raise ValueError('Simulation closed')
            if self.session and time.monotonic()-self.last_input>.2:self.stop('Control input timed out')
            if path=='status':return self.status()
            b=body or {}
            if path=='claim':
                if self.session:raise ValueError('Practice already active')
                if b.get('scope')!='wholebody':raise ValueError('Whole simulator scope required')
                self.session=dict(token=secrets.token_urlsafe(16),permit=secrets.token_urlsafe(16),owner_started=self.started)
                self.sequence=0;self.neutral_seen=False;self.last_input=time.monotonic();self.reason='Practice active'
                return dict(self.session)
            if path=='release':
                if self.session and b.get('token')!=self.session['token']:raise ValueError('Stale simulation session')
                self.stop('Operator STOP');return {'released':True}
            if path!='input':raise ValueError('Unknown simulation operation')
            if not self.session or any(b.get(k)!=v for k,v in self.session.items()):raise ValueError('Simulation session expired')
            try:
                if type(b.get('sequence')) is not int or b['sequence']<=self.sequence:raise ValueError('Out-of-order simulation command')
                positions=b.get('positions');linear=b.get('linear');angular=b.get('angular')
                if not isinstance(positions,dict) or any(n not in self.bus.map for n in positions):raise ValueError('Unknown simulated joint')
                for n,v in positions.items():
                    lo,hi=self.bus.map[n][3:]
                    if type(v) not in (float,int) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid simulated position')
                for v,limit in ((linear,.1),(angular,math.radians(30))):
                    if type(v) not in (float,int) or not math.isfinite(v) or abs(v)>limit+1e-9:raise ValueError('Invalid simulated wheel speed')
                if not self.neutral_seen:
                    if positions or linear or angular:raise ValueError('First simulation command must be neutral')
                    self.neutral_seen=True
                for n,v in positions.items():
                    lo,hi=self.bus.map[n][3:];self.bus.r[n]['Goal_Position']=100+(v-lo)/(hi-lo)*3900
                left=linear-angular*WHEELBASE_M/2;right=linear+angular*WHEELBASE_M/2
                for n,v,sign in zip(WHEELS,(left,right),(-1,1)):
                    self.bus.r[n]['Goal_Velocity']=sign*v/WHEEL_RADIUS_M*4096/(2*math.pi)
                self.sequence=b['sequence'];self.last_input=time.monotonic()
                self.session['permit']=secrets.token_urlsafe(16)
                return dict(self.session,status=self.status())
            except (TypeError,ValueError):
                self.stop('Invalid simulation command');raise

    def advance(self,dt):
        with self.lock:
            if self.session and time.monotonic()-self.last_input>.2:self.stop('Control input timed out')
            self.bus.step(dt)

    def run(self):
        previous=time.monotonic();count=0;window=previous
        try:
            while not self.stopping.is_set():
                now=time.monotonic();dt=now-previous;previous=now
                try:
                    self.advance(dt)
                    with self.lock:self.bus.render_frame()
                except Exception as e:
                    with self.lock:self.stop(str(e));self.bus.render_error=str(e)
                count+=1
                if now-window>=1:self.loop_hz=count/(now-window);count=0;window=now
                self.stopping.wait(max(.001,.01-(time.monotonic()-now)))
        finally:
            with self.lock:self.stop('Simulation closed');self.bus.close()

    def frame(self):return self.bus.frame()
    def set_view(self,view):self.bus.set_view(view)
    def close(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=3)
            if self.thread.is_alive():raise RuntimeError('Simulator did not exit')
        else:
            with self.lock:self.stop('Simulation closed');self.bus.close()
