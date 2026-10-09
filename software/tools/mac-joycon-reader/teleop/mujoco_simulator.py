"""MuJoCo XLeRobot plant for local Joy-Con practice. No robot transport.

Model travel is normalized to synthetic encoder ticks; this is not physical
calibration. Released virtual joints hold their pose to keep practice usable.
Wheel motion and arm feedback come from mj_step, not a drawn pose animation.
"""
import io
import hashlib
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from simulator import VirtualBus, SimulatedRobot
from wheel_pulse_executor import WHEELS

DEFAULT_MODEL = Path('/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/xlerobot/xlerobot.xml')
ARM_NAMES = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
MODEL_NAMES = ('Rotation','Pitch','Elbow','Wrist_Pitch','Wrist_Roll','Jaw')
POSITION_MAP = {f'{side}_arm_{name}':f'{joint}_{suffix}' for side,suffix in (('left','L'),('right','R')) for name,joint in zip(ARM_NAMES,MODEL_NAMES)}
POSITION_MAP.update(head_motor_1='head_pan',head_motor_2='head_tilt')


def scene_xml(path):
    root = ET.parse(path).getroot()
    root.find('compiler').set('meshdir',str(path.parent/'assets'))
    option = root.find('option')
    if option is None: option=ET.SubElement(root,'option')
    option.set('timestep','0.002');option.set('integrator','implicitfast')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='960',offheight='640')
    ET.SubElement(visual,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    asset=root.find('asset');world=root.find('worldbody')
    ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.18 .23 .31',rgb2='.58 .66 .74',width='256',height='1536')
    for side in ('left','right'):
        wheel=world.find(f"./body[@name='chassis']/body[@name='{side}_wheel']")
        wheel.find('geom').set('group','2')
        wheel.find('geom').set('rgba','.08 .09 .11 1')
    for side, body in (('left','Fixed_Jaw'),('right','Fixed_Jaw_2')):
        jaw=world.find(f".//body[@name='{body}']")
        ET.SubElement(jaw,'site',name=side+'_grip_center',pos='0 -.08 0',size='.006',rgba='1 .8 .2 1')
    ET.SubElement(asset,'texture',name='practice_grid',type='2d',builtin='checker',rgb1='.19 .23 .29',rgb2='.28 .33 .39',width='256',height='256')
    ET.SubElement(asset,'material',name='practice_floor',texture='practice_grid',texrepeat='12 12',reflectance='.05')
    ET.SubElement(world,'geom',name='practice_floor',type='plane',size='6 6 .1',material='practice_floor',contype='1',conaffinity='1',condim='3',friction='1 .005 .0001')
    # A reference workbench and free box for spatial practice. No folding policy.
    ET.SubElement(world,'geom',name='practice_table',type='box',size='.32 .50 .035',pos='1.0 0 .665',rgba='.63 .49 .32 1',contype='1',conaffinity='1')
    for x in (.73,1.27):
        for y in (-.43,.43):
            ET.SubElement(world,'geom',type='box',size='.025 .025 .315',pos=f'{x} {y} .315',rgba='.24 .27 .30 1')
    box=ET.SubElement(world,'body',name='practice_carton',pos='1.0 0 .76')
    ET.SubElement(box,'freejoint',name='practice_carton_free')
    ET.SubElement(box,'geom',name='practice_carton_solid',type='box',size='.14 .105 .055',mass='.3',rgba='.76 .55 .29 1',contype='1',conaffinity='1',friction='.8 .005 .0001')
    actuators=root.find('actuator')
    for name in ('forward','turn'):
        actuators.remove(actuators.find(f"motor[@name='{name}']"))
    for side in ('left','right'):
        ET.SubElement(actuators,'velocity',name=side+'_wheel_velocity',joint=side+'_wheel_joint',kv='5',ctrlrange='-.5 .5',forcerange='-2 2')
    return ET.tostring(root,encoding='unicode')


class MujocoBus(VirtualBus):
    engine = 'MuJoCo '+mujoco.__version__
    cartesian_capable = True
    def __init__(self,model_path=None,*,render=True,forward_arms=False):
        super().__init__()
        path=Path(model_path or os.environ.get('XLEROBOT_MUJOCO_MODEL',DEFAULT_MODEL)).expanduser().resolve()
        if not path.is_file():raise FileNotFoundError(f'XLeRobot model missing: {path}. Set XLEROBOT_MUJOCO_MODEL to xlerobot.xml.')
        self.model_path=path
        self.model_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        self.model=mujoco.MjModel.from_xml_string(scene_xml(path))
        self.data=mujoco.MjData(self.model)
        self.map={}
        for n,name in POSITION_MAP.items():
            aid=self.model.actuator(name).id;jid=int(self.model.actuator_trnid[aid,0])
            lo,hi=self.model.actuator_ctrlrange[aid]
            self.map[n]=(aid,int(self.model.jnt_qposadr[jid]),int(self.model.jnt_dofadr[jid]),float(lo),float(hi))
        self.wheel_map={n:(self.model.actuator(side+'_wheel_velocity').id,
                           int(self.model.joint(side+'_wheel_joint').qposadr[0]),
                           int(self.model.joint(side+'_wheel_joint').dofadr[0]),sign)
                        for n,side,sign in ((WHEELS[0],'left',-1),(WHEELS[1],'right',1))}
        x,y,l1,l2=.162,.118,.1159,.135
        offset=math.atan2(.028,.11257)
        t2=math.pi-math.acos(-(x*x+y*y-l1*l1-l2*l2)/(2*l1*l2))
        pitch=math.atan2(y,x)+math.atan2(l2*math.sin(t2),l1+l2*math.cos(t2))+offset
        elbow=t2+math.atan2(.0052,.1349)+offset
        initial={'head_pan':0.,'head_tilt':0.}
        for side,rot in (('L',1.5708),('R',-1.5708)):
            if forward_arms:rot=-rot
            initial.update({f'Rotation_{side}':rot,f'Pitch_{side}':pitch,f'Elbow_{side}':elbow,
                            f'Wrist_Pitch_{side}':pitch-elbow,f'Wrist_Roll_{side}':1.57,f'Jaw_{side}':-.25})
        self.hold={}
        for n,(aid,qadr,_,lo,hi) in self.map.items():
            q=initial[POSITION_MAP[n]];self.data.qpos[qadr]=q;self.data.ctrl[aid]=q;self.hold[n]=q
        mujoco.mj_forward(self.model,self.data)
        for _ in range(500):mujoco.mj_step(self.model,self.data)
        self.sync_registers()
        for n in self.map:self.r[n]['Goal_Position']=self.r[n]['Present_Position']
        self.chassis=self.model.body('chassis').id
        self.accum=0.;self.render_enabled=render;self.renderer=None;self.latest_jpeg=None
        self.frame_lock=threading.Lock();self.render_error=None;self.frame_sequence=0;self.last_render=0.
        self.view='orbit';self.pending_view=None;self.origin=self.data.xpos[self.chassis,:2].copy()
        self.sync_registers()

    def radians(self,n,ticks):
        _,_,_,lo,hi=self.map[n]
        return lo+(ticks-100)/3900*(hi-lo)

    def write(self,field,name,value,**kwargs):
        if field=='Torque_Enable' and value==0 and name in self.map:
            self.hold[name]=float(self.data.qpos[self.map[name][1]])
        super().write(field,name,value,**kwargs)

    def sync_registers(self):
        for n,(aid,qadr,vadr,lo,hi) in self.map.items():
            self.r[n]['Present_Position']=round(100+(float(self.data.qpos[qadr])-lo)/(hi-lo)*3900)
            self.r[n]['Present_Velocity']=round(float(self.data.qvel[vadr])/(hi-lo)*3900)
        for n,(_,qadr,vadr,sign) in self.wheel_map.items():
            self.r[n]['Present_Position']=round(2048+sign*float(self.data.qpos[qadr])*4096/(2*math.pi))%4096
            self.r[n]['Present_Velocity']=round(sign*float(self.data.qvel[vadr])*4096/(2*math.pi))
        if hasattr(self,'chassis'):
            q=self.data.xquat[self.chassis]
            self.pose=dict(x=float(self.data.xpos[self.chassis,0]-self.origin[0]),
                           y=float(self.data.xpos[self.chassis,1]-self.origin[1]),
                           heading=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])))

    def step(self,dt):
        for n,(aid,_,_,_,_) in self.map.items():
            self.data.ctrl[aid]=self.radians(n,self.r[n]['Goal_Position']) if self.r[n]['Torque_Enable'] else self.hold[n]
        for n,(aid,_,_,sign) in self.wheel_map.items():
            r=self.r[n]
            self.data.ctrl[aid]=sign*r['Goal_Velocity']*2*math.pi/4096 if r['Torque_Enable'] and r['Operating_Mode']==1 else 0
        self.accum+=min(max(dt,0),.1)
        while self.accum>=self.model.opt.timestep:
            mujoco.mj_step(self.model,self.data);self.accum-=self.model.opt.timestep
        if not np.isfinite(self.data.qpos).all() or any(int(w.number) for w in self.data.warning):
            raise RuntimeError('MuJoCo invalid state or numerical warning')
        self.sync_registers()

    def render_frame(self,force=False):
        if not self.render_enabled or (not force and time.monotonic()-self.last_render<getattr(self,"render_interval",.08)):return
        if self.renderer is None:
            self.renderer=mujoco.Renderer(self.model,height=640,width=960)
            self.camera=mujoco.MjvCamera();self.option=mujoco.MjvOption();self.option.geomgroup[3]=0
        with self.frame_lock:
            if self.pending_view:self.view=self.pending_view;self.pending_view=None
        if self.view=='front':azimuth,elevation=0,-15
        elif self.view=='side':azimuth,elevation=90,-18
        elif self.view=='top':azimuth,elevation=90,-80
        else:azimuth,elevation=135,-20
        self.camera.lookat[:]=[self.data.xpos[self.chassis,0]+.22,self.data.xpos[self.chassis,1],.62]
        self.camera.distance=2.25;self.camera.azimuth=azimuth;self.camera.elevation=elevation
        self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.option)
        out=io.BytesIO();Image.fromarray(self.renderer.render()).save(out,format='JPEG',quality=80)
        with self.frame_lock:self.latest_jpeg=out.getvalue();self.frame_sequence+=1
        self.last_render=time.monotonic()

    def frame(self):
        with self.frame_lock:return self.latest_jpeg

    def set_view(self,view):
        if view not in ('orbit','front','side','top'):raise ValueError('Unknown simulation view')
        with self.frame_lock:self.pending_view=view

    def close(self):
        if self.renderer:self.renderer.close();self.renderer=None


class MujocoRobot(SimulatedRobot):
    def __init__(self,model_path=None,*,render=True):
        super().__init__(bus=MujocoBus(model_path,render=render))
    def call(self,path,body=None,timeout=None):
        result=super().call(path,body,timeout)
        info=dict(engine=self.bus.engine,model='XLeRobot',model_sha256=self.bus.model_sha256,frame_sequence=self.bus.frame_sequence,frame_age_s=round(time.monotonic()-self.bus.last_render,3) if self.bus.last_render else None,
                  render_error=self.bus.render_error,physics=True,
                  limitations='Synthetic calibration; disarmed joints hold pose; solid practice box, no carton folding validation.')
        if path=='status':result.update(simulator=info)
        elif 'status' in result:result['status']['simulator']=info
        return result
    def frame(self):return self.bus.frame()
    def set_view(self,view):self.bus.set_view(view)
    def make_cartesian_mapping(self):
        from cartesian import CartesianMapping
        return CartesianMapping(self.bus.model, self.bus.map)
