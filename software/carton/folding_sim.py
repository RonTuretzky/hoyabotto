"""Two real SO101 models and a passive hinged carton. Simulation only.

Joint targets are the sole control input. Carton state/contact IDs are only
exposed to the independent evaluator, never to the visual controller.
"""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from scipy.optimize import least_squares
from PIL import Image, ImageDraw

from carton.servo.tag_kit import marker_grid
from carton.geometry import Box

JOINTS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
FLAPS = ('short_left','short_right','long_far','long_near')
TAG_IDS = dict(zip(FLAPS, (11,12,13,14)))
_box=Box()
L,W,H,F = _box.length,_box.width,_box.height,_box.flap

def words(a):
    return ' '.join(f'{float(v):.10g}' for v in a)


def marker(parent,name,tag_id,size,pos,xyaxes=None):
    attrs=dict(name=name,pos=words(pos))
    if xyaxes is not None:attrs['xyaxes']=words(xyaxes)
    b=E.SubElement(parent,'body',**attrs)
    c=size/8
    common=dict(type='box',contype='0',conaffinity='0',mass='0')
    E.SubElement(b,'geom',name=name+'_paper',size=words([5*c,5*c,.0001]),rgba='1 1 1 1',**common)
    for row,vals in enumerate(marker_grid(tag_id)):
        for col,val in enumerate(vals):
            if val==0:E.SubElement(b,'geom',name=f'{name}_{row}_{col}',pos=words([(col-3.5)*c,(3.5-row)*c,.0002]),size=words([c/2,c/2,.0001]),rgba='0 0 0 1',**common)
    E.SubElement(b,'site',name=name+'_center',pos='0 0 .0003',size='.001',rgba='0 0 0 0')
    return b


def build_scene(source:Path,out:Path, *, setback=.04, base_height=.04, stiffness=.018, offset=(0,0), yaw=0.):
    root=E.parse(source/'scene-assets/arm-import.xml').getroot()
    root.set('model','dual_SO101_passive_carton')
    E.SubElement(root,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='80')
    vis=E.SubElement(root,'visual')
    E.SubElement(vis,'global',offwidth='1280',offheight='960')
    E.SubElement(vis,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    asset=root.find('asset');world=root.find('worldbody')
    arm=copy.deepcopy(world.find('body'));world.remove(world.find('body'))
    manifest=json.loads((source/'scene-assets/jaw-collision/manifest.json').read_text())
    for mesh,spec in manifest.items():
        for i,file in enumerate(spec['files']):E.SubElement(asset,'mesh',name=mesh+'_part_'+str(i),file=file)
    contact=E.SubElement(root,'contact');act=E.SubElement(root,'actuator')
    for side,x in [('left',-.15),('right',.15)]:
        b=copy.deepcopy(arm)
        for elem in b.iter():
            if elem.get('name'):elem.set('name',side+'_'+elem.get('name'))
        b.set('pos',words([x,-W/2-setback,base_height]));b.set('euler',words([0,0,math.pi/2]))
        for parent in b.iter('body'):
            for child in parent.findall('body'):E.SubElement(contact,'exclude',body1=parent.get('name'),body2=child.get('name'))
            for i,g in enumerate(list(parent.findall('geom'))):
                g.set('name',parent.get('name')+'_geom_'+str(i))
                if g.get('contype')=='0':
                    if 'sts' not in g.get('mesh',''):g.set('rgba','.22 .48 .75 1' if side=='left' else '.83 .50 .18 1')
                    continue
                g.set('group','3');g.set('friction','.8 .003 .0001');g.set('solref','.004 1');g.set('solimp','.95 .99 .001')
                if g.get('mesh') in manifest:
                    mesh=g.get('mesh');parent.remove(g)
                    for k in range(len(manifest[mesh]['files'])):
                        part=copy.deepcopy(g);part.set('mesh',mesh+'_part_'+str(k));part.set('name',f'{side}_{mesh}_part_{k}');parent.append(part)
        for j in b.iter('joint'):
            j.set('damping','.2');j.set('armature','.01')
            grip=j.get('name').endswith('_gripper')
            E.SubElement(act,'position',name=j.get('name'),joint=j.get('name'),kp='80' if grip else '300',kv='3',ctrlrange=j.get('range'),forcerange='-.5 .5' if grip else '-2.94 2.94')
        grip=b.find(f".//body[@name='{side}_gripper_link']")
        E.SubElement(grip,'site',name=side+'_tip',pos='-.0049 -.0002 -.096',size='.002',rgba='0 0 0 0')
        marker(grip,side+'_tag',4 if side=='left' else 2,.040,[.045,0,.008],[0,1,0,0,0,1])
        world.append(b)
    E.SubElement(world,'light',pos='0 -.2 1.4',dir='0 0 -1',directional='true')
    E.SubElement(world,'geom',name='table',type='box',pos='0 .12 -.016',size='.55 .55 .016',rgba='.70 .66 .58 1',friction='.7 .003 .0001',solref='.004 1')
    marker(world,'table_tag',1,.060,[0,-.35,.001])
    box=E.SubElement(world,'body',name='carton',pos=words([*offset,.001]),euler=words([0,0,yaw]))
    E.SubElement(box,'freejoint',name='carton_free')
    common=dict(type='box',rgba='.68 .45 .24 1',friction='.65 .002 .0001',solref='.004 1',solimp='.95 .99 .001')
    E.SubElement(box,'geom',name='bottom',pos=words([0,0,.0015]),size=words([L/2,W/2,.0015]),mass='.08',**common)
    E.SubElement(box,'geom',name='contents',pos='0 0 .052',size=words([L/2-.012,W/2-.012,.05]),mass='.96',**{**common,'rgba':'.5 .52 .50 1'})
    for name,pos,size in [('wall_left',[-L/2,0,H/2],[.0015,W/2,H/2]),('wall_right',[L/2,0,H/2],[.0015,W/2,H/2]),('wall_far',[0,W/2,H/2],[L/2,.0015,H/2]),('wall_near',[0,-W/2,H/2],[L/2,.0015,H/2])]:
        E.SubElement(box,'geom',name=name,pos=words(pos),size=words(size),mass='.025',**common)
    marker(box,'box_tag',10,.045,[0,-W/2-.0018,H/2],[1,0,0,0,0,1])
    specs=[('short_left',[-L/2,0,H],[0,1,0],[.0015,W/2-.004,F/2],[0,-1,0,0,0,1]),
           ('short_right',[L/2,0,H],[0,-1,0],[.0015,W/2-.004,F/2],[0,1,0,0,0,1]),
           ('long_far',[0,W/2,H+.0035],[1,0,0],[L/2-.004,.0015,F/2],[-1,0,0,0,0,1]),
           ('long_near',[0,-W/2,H+.0035],[-1,0,0],[L/2-.004,.0015,F/2],[1,0,0,0,0,1])]
    for name,pos,axis,size,axes in specs:
        f=E.SubElement(box,'body',name=name,pos=words(pos))
        E.SubElement(f,'joint',name=name+'_hinge',axis=words(axis),range='-1.7 3.05',stiffness=str(stiffness),springref='0',damping='.008',frictionloss='.004')
        E.SubElement(f,'geom',name=name+'_cardboard',pos=words([0,0,F/2]),size=words(size),mass='.023',**common)
        # MuJoCo filters parent/child contacts by default. Contents must explicitly
        # collide with each hinged flap; otherwise a hinge limit could fake support.
        E.SubElement(contact,'pair',geom1='contents',geom2=name+'_cardboard',solref='.004 1',solimp='.95 .99 .001',friction='.65 .65 .002 .0001 .0001')
        # Outside-face markers face upward after folding; offset from hand contacts.
        n=np.cross(axes[:3],axes[3:])
        tag_point=[0,.07,.090] if name.startswith('short') else ([-.08,0,.090] if name=='long_far' else [.08,0,.090])
        marker(f,name+'_tag',TAG_IDS[name],.035,np.array(tag_point)+n*.0018,axes)
    for name,pos,look in [('overhead',[0,.00,.85],[0,0,.06]),('front',[.0,-.55,.85],[0,-.04,.08]),('side',[.65,-.2,.45],[0,0,.1])]:
        back=np.array(pos)-look;back/=np.linalg.norm(back)
        right=np.cross([0,1,0] if name=='overhead' else [0,0,1],back);right/=np.linalg.norm(right)
        up=np.cross(back,right)
        E.SubElement(world,'camera',name=name,pos=words(pos),xyaxes=words(np.r_[right,up]),fovy='48')
    out.mkdir(parents=True,exist_ok=True)
    E.indent(root);E.ElementTree(root).write(out/'scene.xml',encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(out/'scene.xml'))


class FoldingSimulation:
    def __init__(self,source,out,width=960,height=720,**kwargs):
        self.width,self.height=width,height
        self.out=Path(out);self.model=build_scene(Path(source),self.out,**kwargs)
        self.data=mujoco.MjData(self.model);self.kin=mujoco.MjData(self.model)
        self.arm_indices={s:[self.model.jnt_qposadr[self.model.joint(s+'_'+n).id] for n in JOINTS] for s in ('left','right')}
        self.seeds={s:np.radians([0,50,-30,-20,0]) for s in self.arm_indices}
        self.frames=[];self.events=[];self.stats={'max_bad_penetration_mm':0.,'carton_contact_simulated':True}
        self.renderer=None;self.option=mujoco.MjvOption();self.option.geomgroup[3]=0
        for name in FLAPS:self.data.qpos[self.model.jnt_qposadr[self.model.joint(name+'_hinge').id]]=.10
        for s,x in [('left',-.23),('right',.23)]:
            q,err=self.ik(s,[x,-.08,.30],orientation=None)
            ix=self.arm_indices[s];self.data.qpos[ix[:5]]=q;self.data.qpos[ix[5]]=-.17
            self.data.ctrl[[self.model.actuator(s+'_'+n).id for n in JOINTS]]=np.r_[q,-.17]
        mujoco.mj_forward(self.model,self.data)

    def ik(self,side,target,orientation=None):
        ix=self.arm_indices[side][:5];site=self.model.site(side+'_tip').id
        ranges=self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
        for arm_indices in self.arm_indices.values():
            self.kin.qpos[arm_indices]=self.data.qpos[arm_indices]
        def fun(q):
            self.kin.qpos[ix]=q;mujoco.mj_forward(self.model,self.kin)
            e=self.kin.site_xpos[site]-target
            if orientation is not None:
                axis_index=2 if isinstance(orientation,(str,dict)) else 0
                desired=orientation['direction'] if isinstance(orientation,dict) else ([0,0,1] if axis_index==2 else orientation)
                axis=self.kin.body(side+'_gripper_link').xmat.reshape(3,3)[:,axis_index]
                e=np.r_[e,(axis-np.asarray(desired))*.04,q[4]*.005]
                if isinstance(orientation,dict) and 'tangent' in orientation:
                    xaxis=self.kin.body(side+'_gripper_link').xmat.reshape(3,3)[:,0]
                    e=np.r_[e,(xaxis-np.asarray(orientation['tangent']))*.03]
            return e
        def solve(seed):
            return least_squares(fun,np.clip(seed,ranges[:,0]+1e-6,ranges[:,1]-1e-6),bounds=(ranges[:,0],ranges[:,1]),max_nfev=100,ftol=1e-8,gtol=1e-8,xtol=1e-8)
        sol=solve(self.seeds[side])
        if np.linalg.norm(fun(sol.x)[:3])>.004:
            candidates=[sol]
            for shoulder in (-1.,0.,1.):
                for elbow in (-1.,1.):
                    candidates.append(solve([self.seeds[side][0],shoulder,elbow,0.,0.]))
            sol=min(candidates,key=lambda candidate:np.linalg.norm(fun(candidate.x)))
        self.seeds[side]=sol.x.copy()
        return sol.x,float(np.linalg.norm(fun(sol.x)[:3]))

    def move(self,targets,seconds=.5,label='',orientation=None,capture=True):
        ctrl=self.data.ctrl.copy();errors={}
        for side,point in targets.items():
            q,e=self.ik(side,np.asarray(point),orientation);errors[side]=e
            if e>.008:raise ValueError(f'IK {side} target {point} misses by {e*1000:.1f} mm')
            ctrl[[self.model.actuator(side+'_'+j).id for j in JOINTS[:5]]]=q
        start=self.data.ctrl.copy();n=max(1,round(seconds/self.model.opt.timestep))
        contact_names=set();bad=0.
        extrema={f:[float('inf'),float('-inf')] for f in FLAPS}
        for i in range(n):
            t=min(1,(i+1)/(n*.8));self.data.ctrl[:]=start+(ctrl-start)*(t*t*(3-2*t))
            mujoco.mj_step(self.model,self.data)
            for c in self.data.contact:
                a,b=self.model.geom(c.geom1).name,self.model.geom(c.geom2).name
                if 'cardboard' in a or 'cardboard' in b:
                    if a.startswith(('left_','right_')) or b.startswith(('left_','right_')):contact_names.add((a,b))
                arms=(a.startswith(('left_','right_')),b.startswith(('left_','right_')))
                # Arm/table, arm/rigid carton, and opposite-arm penetration.
                forbidden=(all(arms)) or (any(arms) and ('table' in (a,b) or any(v.startswith('wall_') or v=='contents' for v in (a,b)))) or (a.startswith('left_') and b.startswith('right_')) or (a.startswith('right_') and b.startswith('left_'))
                if forbidden:bad=max(bad,-c.dist*1000)
            if i%10==0 or i==n-1:
                for flap,value in self.truth_angles().items():
                    extrema[flap][0]=min(extrema[flap][0],value);extrema[flap][1]=max(extrema[flap][1],value)
            if capture and i%100==0:self.capture(label)
        self.stats['max_bad_penetration_mm']=max(self.stats['max_bad_penetration_mm'],bad)
        event={'label':label,'duration_s':seconds,'flap_angle_extrema_degrees':extrema,'time':float(self.data.time),'ik_error_m':errors,'flap_degrees':self.truth_angles(),'contact_pairs':sorted(contact_names),'bad_penetration_mm':bad,'max_target_tracking_error_m':max([0.]+[float(np.linalg.norm(self.data.site(a+'_tip').xpos-np.asarray(p))) for a,p in targets.items()]),'tip_m':{s:self.data.site(s+'_tip').xpos.tolist() for s in self.arm_indices}}
        self.events.append(event)
        return event

    def arm_tag_fk(self,side):
        # Uses robot encoders, fixed base registration and declared CAD mount.
        # No carton qpos or scene-object truth enters this kinematic prediction.
        for indices in self.arm_indices.values():self.kin.qpos[indices]=self.data.qpos[indices]
        mujoco.mj_forward(self.model,self.kin)
        pose=np.eye(4)
        pose[:3,:3]=self.kin.body(side+'_tag').xmat.reshape(3,3)@np.diag([-1,1,-1])
        pose[:3,3]=self.kin.site(side+'_tag_center').xpos
        return pose

    def truth_angles(self):
        return {f:math.degrees(self.data.qpos[self.model.jnt_qposadr[self.model.joint(f+'_hinge').id]]) for f in FLAPS}

    def render(self,camera='front',depth=False):
        if self.renderer is None:self.renderer=mujoco.Renderer(self.model,height=self.height,width=self.width)
        self.renderer.update_scene(self.data,camera=camera,scene_option=self.option)
        if depth:self.renderer.enable_depth_rendering()
        else:self.renderer.disable_depth_rendering()
        return self.renderer.render().copy()

    def capture(self,label):
        im=Image.fromarray(self.render());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,self.width,44),fill='white');draw.text((10,6),'SIMULATION | Two SO101 arms | Passive carton hinges | No paddle',fill='black');draw.text((10,25),label,fill='black')
        draw.rectangle((8,49,82,66),fill='white');draw.text((12,51),'LEFT ARM',fill='black')
        draw.rectangle((self.width-90,49,self.width-8,66),fill='white');draw.text((self.width-86,51),'RIGHT ARM',fill='black')
        self.frames.append(im)

    def save(self,name='trial'):
        report={'simulation_only':True,'hardware_commands':0,'actuated_carton_joints':0,'engine':mujoco.__version__,'events':self.events,'stats':self.stats,'final_angles':self.truth_angles()}
        (self.out/(name+'.json')).write_text(json.dumps(report,indent=2))
        if self.frames:
            self.frames[0].save(self.out/(name+'-before.png'));self.frames[-1].save(self.out/(name+'-after.png'))
            self.frames[0].save(self.out/(name+'.gif'),save_all=True,append_images=self.frames[1:],duration=100,loop=0)
        if self.renderer:self.renderer.close()
        return report
