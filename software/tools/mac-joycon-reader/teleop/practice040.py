"""Input-only kinematic practice on the SO-101/two-wheel kit model.

No physical transport, credentials or HID reader. Shares the existing bridge's
decoded inputs. Motion is kinematic, not contact/force simulation evidence.
"""
import math
import secrets
import threading
import time
import mujoco
from model040_preview import Model040PreviewBus
from upstream import WindowsMapping, SO101Kinematics


class Practice040:
    preview=True
    simulation=True
    allow_background_input=True  # Virtual-only; never applied to the live transport.
    teleop_keys={'forward':'w','backward':'s','rotate_left':'a','rotate_right':'d'}
    def __init__(self):
        self.bus=Model040PreviewBus();self.lock=threading.RLock()
        self.stopping=threading.Event();self.session=None;self.sequence=0
        self.last_input=0.;self.started=time.monotonic();self.neutral_seen=False
        lift,elbow=SO101Kinematics().inverse_kinematics(.1629,.1131)
        self.positions={n:0. for n in self.bus.map}
        for side in ('left','right'):
            self.positions[side+'_arm_shoulder_lift']=lift
            self.positions[side+'_arm_elbow_flex']=elbow
            self.positions[side+'_arm_gripper']=45.
        self.linear=self.angular=0.;self.pose={'x':0.,'y':0.,'heading':0.}
        self.origin=self.bus.model.body_pos[self.bus.chassis].copy()
        self.reason='Practice stopped'
        self.thread=threading.Thread(target=self.run,name='so101-kinematic-practice',daemon=True)
        self.thread.start()

    def make_mapping(self):return WindowsMapping(self)
    def get_observation(self):
        with self.lock:return {n+'.pos':v for n,v in self.positions.items()}
    def _from_keyboard_to_base_action(self,keys):
        k=self.teleop_keys
        return {'x.vel':.1*(int(k['forward'] in keys)-int(k['backward'] in keys)),
                'theta.vel':30.*(int(k['rotate_left'] in keys)-int(k['rotate_right'] in keys))}
    def bound_upstream_positions(self,positions):
        bounded={};warnings=[]
        for n,v in positions.items():
            lo,hi=(0.,90.) if n.endswith('gripper') else (-100.,100.)
            bounded[n]=max(lo,min(hi,v))
            if bounded[n]!=v:warnings.append(n+' practice limit')
        return bounded,warnings
    def encode_upstream_command(self,command,decoded):return command
    def status(self):
        return dict(preview=True,simulation=True,ok=True,phase='kinematic practice',status_age_s=0.,
                    teleop={'active':self.session is not None},virtual_pose=dict(self.pose),
                    motors={n:dict(Present_Position=round(2048+v*19.5),Torque_Enable=int(self.session is not None),Present_Load=0) for n,v in self.positions.items()},
                    simulator={'engine':self.bus.engine,'model_hardware':'0.4 kit-layout practice · maker SO-101 meshes',
                               'physics':False,'kinematic_practice':True,'frame_sequence':self.bus.frame_sequence,
                               'frame_age_s':None if not self.bus.last_render else time.monotonic()-self.bus.last_render,
                               'render_error':self.bus.render_error,
                               'limitations':'Kinematic practice; synthetic state and unvalidated mounting. No robot motion or contact/force validation.'})
    def stop(self,reason):
        self.session=None;self.linear=self.angular=0.;self.reason=reason
    def call(self,path,body=None,timeout=None):
        with self.lock:
            if path=='status':return self.status()
            b=body or {}
            if path=='claim':
                if self.session or b.get('scope')!='wholebody':raise ValueError('Stop practice; whole-body scope required')
                self.session={k:secrets.token_urlsafe(24) for k in ('token','permit')};self.session['owner_started']=self.started
                self.sequence=0;self.neutral_seen=False;self.last_input=time.monotonic()
                return dict(self.session)
            if path=='release':
                if self.session and b.get('token')!=self.session['token']:raise ValueError('Practice session changed')
                self.stop('Practice stopped');return {'released':True}
            if path!='input' or not self.session or any(b.get(k)!=v for k,v in self.session.items()):raise ValueError('Practice session unavailable')
            if type(b.get('sequence')) is not int or b['sequence']<=self.sequence:raise ValueError('Old practice input')
            positions=b.get('positions');linear=b.get('linear');angular=b.get('angular')
            if not isinstance(positions,dict) or not set(positions)<=set(self.positions):raise ValueError('Invalid practice joints')
            if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>100 for v in positions.values()):raise ValueError('Invalid practice target')
            if not all(type(v) in (int,float) and math.isfinite(v) for v in (linear,angular)) or abs(linear)>.1 or abs(angular)>math.radians(30)+1e-8:raise ValueError('Invalid practice drive')
            if not self.neutral_seen:
                if positions or linear or angular:raise ValueError('First practice frame must be neutral')
                self.neutral_seen=True
            self.positions.update(positions);self.linear=linear;self.angular=angular
            self.sequence=b['sequence'];self.last_input=time.monotonic();self.session['permit']=secrets.token_urlsafe(24)
            return dict(self.session,status=self.status())
    def run(self):
        previous=time.monotonic()
        try:
            while not self.stopping.is_set():
                now=time.monotonic();dt=min(.05,now-previous);previous=now
                with self.lock:
                    if self.session and now-self.last_input>.2:self.stop('Practice input timed out')
                    self.pose['heading']+=self.angular*dt
                    self.pose['x']+=self.linear*math.cos(self.pose['heading'])*dt
                    self.pose['y']+=self.linear*math.sin(self.pose['heading'])*dt
                    for n,v in self.positions.items():
                        _,qadr,_,lo,hi=self.bus.map[n]
                        q=lo+(v/90)*(hi-lo) if n.endswith('gripper') else math.radians(v)
                        self.bus.data.qpos[qadr]=max(lo,min(hi,q))
                    self.bus.model.body_pos[self.bus.chassis]=self.origin+[self.pose['x'],self.pose['y'],0]
                    heading=self.pose['heading'];self.bus.model.body_quat[self.bus.chassis]=[math.cos(heading/2),0,0,math.sin(heading/2)]
                    mujoco.mj_forward(self.bus.model,self.bus.data)
                    self.bus.render_frame()
                self.stopping.wait(.01)
        finally:self.bus.close()
    def frame(self):return self.bus.frame()
    def set_view(self,view):self.bus.set_view(view)
    def close(self):
        self.stopping.set();self.thread.join(timeout=3)
        if self.thread.is_alive():raise RuntimeError('Practice renderer did not exit')
