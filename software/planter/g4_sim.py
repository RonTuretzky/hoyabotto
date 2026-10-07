"""First G4 curriculum stage: contact pickup of the actual paper pusher.

No object pose is available through the observation/action interface. Ground
truth is isolated in score_state(), for evaluation and abort diagnostics only.
All SI dimensions below are from shared_tools.scad except declared assumptions.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import uuid
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares

from carton.folding_sim import marker, words
from farm.kinematics.assets import verified_model
from farm.kinematics.lerobot import LeRobotSO101
from farm.perception.gemma_tags import TagObserver, TagRobot
# Existing calibration script imports its sibling by executable-script name.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from tools.simulate_gemma_tags import CameraSimulation
from tools.simulate_tag_calibration import RenderedOwner

JOINTS = ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
TABLE_Z = -.06
REST_Z = TABLE_Z + .020
# Print-oriented CAD: x=-z(original), y=y(original), z=x(original)+4mm.
# Each block is exact, including intentional same-body overlap at the crossbar.
BLOCKS = [('blade',[-.0304,0,.004],[.0075,.024,.0039]),
          ('crossbar',[-.035,0,.004],[.001,.047,.004]),
          ('handle',[-.052,0,.004],[.016,.012,.004])]
GRIP_LOCAL = np.array([-.060,0,.004])
TAG_LOCAL = np.array([-.045,0,.0086])
# marker() decoded axes are diag(-1,1,-1) relative to its CAD body.
TAG_FROM_GRIP = np.r_[(GRIP_LOCAL-TAG_LOCAL)*[-1,1,-1],1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_scene(source, cad, out, *, timestep=.002, noslip=3, width=1920,rest_height=.020,
                rest_near_edge=.395,rest_far_edge=.455,rest_half_width=.055):
    if not np.isfinite([rest_height,rest_near_edge,rest_far_edge,rest_half_width]).all():
        raise ValueError('Staging rest dimensions must be finite')
    if rest_height<=0 or rest_half_width<=0 or rest_far_edge<=rest_near_edge:
        raise ValueError('Staging rest dimensions must be positive and ordered')
    out.mkdir(parents=True,exist_ok=True)
    root=E.parse(source/'scene-assets/arm-import.xml').getroot()
    root.set('model','G4_pusher_contact_curriculum')
    # Make all inherited assets explicit; saved scene does not rely on cwd.
    meshdir=Path(root.find('compiler').get('meshdir'))
    for mesh in root.findall('./asset/mesh'):
        mesh.set('file',str((meshdir/mesh.get('file')).resolve()))
    root.find('compiler').attrib.pop('meshdir',None)
    E.SubElement(root,'option',timestep=str(timestep),integrator='implicitfast',
                 cone='elliptic',impratio='10',noslip_iterations=str(noslip),iterations='80',tolerance='1e-10')
    visual=E.SubElement(root,'visual')
    E.SubElement(visual,'global',offwidth=str(width),offheight=str(width*3//4))
    E.SubElement(visual,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    asset,world=root.find('asset'),root.find('worldbody')
    manifest=json.loads((source/'scene-assets/jaw-collision/manifest.json').read_text())
    contact=E.SubElement(root,'contact');act=E.SubElement(root,'actuator')
    for parent in root.findall('.//body'):
        for child in parent.findall('body'):
            E.SubElement(contact,'exclude',body1=parent.get('name'),body2=child.get('name'))
        for i,g in enumerate(list(parent.findall('geom'))):
            g.set('name',parent.get('name')+'_'+str(i))
            if g.get('contype')=='0':
                g.set('rgba','.15 .17 .19 1');continue
            g.set('group','3');g.set('friction','.8 .003 .0001')
            g.set('condim','4');g.set('solref','.004 1');g.set('solimp','.95 .99 .001')
            if g.get('mesh') in manifest:
                name=g.get('mesh');parent.remove(g)
                for k,file in enumerate(manifest[name]['files']):
                    partname=name+'_part_'+str(k)
                    E.SubElement(asset,'mesh',name=partname,file=file)
                    part=copy.deepcopy(g);part.set('mesh',partname);part.set('name',partname);parent.append(part)
    for j in root.findall('.//joint'):
        j.set('damping','.2');j.set('armature','.01')
        grip=j.get('name')=='gripper'
        E.SubElement(act,'position',name=j.get('name'),joint=j.get('name'),
                     kp='80' if grip else '300',kv='3',ctrlrange=j.get('range'),
                     forcerange='-.5 .5' if grip else '-2.94 2.94')
    grip=root.find(".//body[@name='gripper_link']")
    E.SubElement(grip,'site',name='grasp',pos='-.0049 -.0002 -.096',size='.002',rgba='0 0 0 0')
    marker(grip,'tag2',2,.040,[.045,0,.008],[0,1,0,0,0,1])
    E.SubElement(world,'light',pos='.1 -.6 1',dir='0 0 -1',directional='true')
    E.SubElement(world,'geom',name='table',type='box',pos='.79 0 -.076',size='.55 .50 .016',
                 rgba='.73 .69 .60 1',friction='.6 .003 .0001',solref='.004 1')
    # This is a one-arm isolated bench experiment, not the whole XLeRobot cart.
    # Table near edge x=.24m; full cart, G4 assembly and other arm are NOT modeled.
    # Explicit fixture hypothesis. Legacy defaults preserve the original
    # 395--455mm support. Moving its near edge changes support and jaw clearance;
    # it must be evaluated with the same contact and complete-sequence gates.
    E.SubElement(world,'geom',name='staging_rest',type='box',
                 pos=words([(rest_near_edge+rest_far_edge)/2,.025,TABLE_Z+rest_height/2]),
                 size=words([(rest_far_edge-rest_near_edge)/2,rest_half_width,rest_height/2]),
                 rgba='.45 .43 .40 1',friction='.6 .003 .0001',solref='.004 1')
    origin=np.array([.38,.025,TABLE_Z+rest_height+.0003])-GRIP_LOCAL*[1,1,0]
    body=E.SubElement(world,'body',name='pusher',pos=words(origin))
    E.SubElement(body,'freejoint',name='pusher_free')
    # Uniform-density CAD centroid; printed mass/inertia still assumptions.
    E.SubElement(body,'inertial',pos='-.041408215 0 .004',mass='.012',diaginertia='0.000003 0.000002 0.000004')
    meshpath=cad/'feeder_print_PROTOTYPE.stl'
    E.SubElement(asset,'mesh',name='g4_pusher_visual',file=str(meshpath),scale='.001 .001 .001')
    E.SubElement(body,'geom',name='pusher_visual',type='mesh',mesh='g4_pusher_visual',
                 rgba='.27 .39 .62 1',contype='0',conaffinity='0',mass='0')
    for name,pos,size in BLOCKS:
        E.SubElement(body,'geom',name='pusher_'+name,type='box',pos=words(pos),size=words(size),group='3',
                     friction='.8 .003 .0001',condim='4',solref='.004 1',solimp='.95 .99 .001',mass='0')
    E.SubElement(body,'site',name='pusher_grip',pos=words(GRIP_LOCAL),size='.001',rgba='0 0 0 0')
    # 18mm black square, 22.5mm backing fits the 24mm-wide grip. Hypothetical
    # marker mount on the pusher, not a marker attached to the growing paper.
    tag=marker(body,'tag31',31,.018,TAG_LOCAL-[0,0,.0003])
    E.SubElement(tag,'geom',name='tag31_occluder',type='box',pos='0 0 .0009',size='.013 .013 .0001',
                 rgba='.4 .4 .4 0',contype='0',conaffinity='0',mass='0')
    marker(world,'tag1',1,.060,[.60,.03,TABLE_Z+.001])
    back=np.array([.54,-.42,.46])-np.array([.35,.005,-.015]);back/=np.linalg.norm(back)
    right=np.cross([0,0,1],back);right/=np.linalg.norm(right);up=np.cross(back,right)
    E.SubElement(world,'camera',name='tag_camera',pos='.54 -.42 .46',xyaxes=words(np.r_[right,up]),fovy='40')
    E.indent(root);E.ElementTree(root).write(out/'scene.xml',encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(out/'scene.xml'))


class G4Simulation:
    def __init__(self,source,cad,out,**kwargs):
        self.out=out;self.model=build_scene(source,cad,out,**kwargs)
        self.rest_z=TABLE_Z+kwargs.get('rest_height',.020)
        self.data=mujoco.MjData(self.model);self.kin=mujoco.MjData(self.model)
        self.ranges=self.model.jnt_range[:5].copy();self.qseed=np.radians([0,65,-35,-30,0.])
        self.site=self.model.site('grasp').id;self.gripper=self.model.body('gripper_link').id
        self.pusher=self.model.body('pusher').id;self.table=self.model.geom('table').id
        self.object_geoms={self.model.geom('pusher_'+n).id for n,_,_ in BLOCKS}
        self.arm_geoms={i for i in range(self.model.ngeom) if self.model.geom_bodyid[i]!=0
                        and self.model.geom_bodyid[i]!=self.pusher and self.model.geom_contype[i]!=0}
        self.frames=[];self.records=[];self.commands=[];self.record=False;self.frame_callback=None
        self.disabled_motion=False;self.stop_reason=None
        # Reset/initialization may set state. Actions subsequently change ctrl only.
        q=self.ik([.35,.025,.024]);self.data.qpos[:6]=np.r_[q,.55];self.data.ctrl[:]=np.r_[q,.55]
        mujoco.mj_forward(self.model,self.data)
        self.initial_intersections=self.intersections(.1)
        if self.initial_intersections:raise ValueError(f'Initial intersections: {self.initial_intersections}')
        self.advance(.3,'initial_settle')
        self.record=True

    def ik(self,target,*,grasp_axis=(0,0,1)):
        axis=np.asarray(grasp_axis,dtype=float)
        if axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>1e-6:
            raise ValueError('Grasp axis must be a finite unit world vector')
        def fun(q):
            self.kin.qpos[:5]=q;mujoco.mj_kinematics(self.model,self.kin)
            x=self.kin.xmat[self.gripper].reshape(3,3)[:,0]
            return np.r_[self.kin.site_xpos[self.site]-target,(x-axis)*.08]
        result=least_squares(fun,np.clip(self.qseed,self.ranges[:,0]+1e-5,self.ranges[:,1]-1e-5),
                             bounds=(self.ranges[:,0],self.ranges[:,1]),max_nfev=150,
                             ftol=1e-10,xtol=1e-10,gtol=1e-10)
        error=fun(result.x)
        if np.linalg.norm(error[:3])>.0015 or np.linalg.norm(error[3:])>.004:
            raise ValueError(f'IK failed: position {np.linalg.norm(error[:3])*1000:.2f}mm')
        self.qseed=result.x.copy();return result.x

    def intersections(self,limit_mm):
        return [(self.model.geom(c.geom1).name,self.model.geom(c.geom2).name,float(-c.dist*1000))
                for c in self.data.contact if c.dist < -limit_mm/1000]

    def score_state(self):
        """Privileged evaluator. Never passed to target calculation or IK."""
        m,d=self.model,self.data
        rot=d.xmat[self.pusher].reshape(3,3);origin=d.xpos[self.pusher]
        vertices=np.array([np.array(pos)+np.array(size)*[x,y,z] for _,pos,size in BLOCKS
                           for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        world=vertices@rot.T+origin
        contacts=[];environment_contacts=[];bad=0.;penetration=0.;normal_force=0.;jaw_names=set()
        for i,c in enumerate(d.contact):
            pair={int(c.geom1),int(c.geom2)}
            names=[m.geom(c.geom1).name,m.geom(c.geom2).name]
            if pair & self.object_geoms:
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                # MuJoCo mjContact.frame[:3] is a world-frame normal directed
                # from geom1 to geom2 (also declared in the installed mjdata.h).
                contacts.append(dict(geoms=names,normal_force_n=float(force[0]),
                                     normal_world_geom1_to_geom2=c.frame[:3].tolist(),
                                     position_world_m=c.pos.tolist(),
                                     penetration_mm=max(0.,float(-c.dist*1000))))
                penetration=max(penetration,-c.dist*1000)
                if pair & self.arm_geoms:
                    normal_force+=max(0,float(force[0]))
                    if force[0]>.005:jaw_names.update(names)
            if pair & self.arm_geoms and not pair & self.object_geoms:
                bad=max(bad,-c.dist*1000)
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                environment_contacts.append(dict(geoms=names,normal_force_n=float(force[0]),
                    normal_world_geom1_to_geom2=c.frame[:3].tolist(),position_world_m=c.pos.tolist(),
                    penetration_mm=max(0.,float(-c.dist*1000))))
        return dict(time_s=float(d.time),grip_world_m=d.site('pusher_grip').xpos.tolist(),
                    grasp_world_m=d.site('grasp').xpos.tolist(),object_pose=np.r_[origin,d.xquat[self.pusher]].tolist(),
                    gripper_world_rotation=d.xmat[self.gripper].reshape(3,3).tolist(),
                    clearance_mm=float((world[:,2].min()-TABLE_Z)*1000),
                    rest_clearance_mm=float((world[:,2].min()-self.rest_z)*1000),
                    table_edge_margin_mm=float(min(world[:,0].min()-.24,1.34-world[:,0].max(),.5-abs(world[:,1]).max())*1000),
                    jaw_contact_both=any('moving_jaw' in n for n in jaw_names) and any('wrist_roll_follower' in n for n in jaw_names),
                    arm_object_normal_force_n=normal_force,object_penetration_mm=max(0,penetration),
                    forbidden_penetration_mm=max(0,bad),contacts=contacts,
                    arm_environment_contacts=environment_contacts,
                    encoder_radians=d.qpos[:6].tolist(),ctrl=d.ctrl.tolist())

    def advance(self,seconds,label,target=None):
        start=self.data.ctrl.copy();count=round(seconds/self.model.opt.timestep)
        if target is not None:
            if np.asarray(target).shape!=(6,) or not np.isfinite(target).all():raise ValueError('Invalid action')
            if np.any(target<self.model.actuator_ctrlrange[:,0]) or np.any(target>self.model.actuator_ctrlrange[:,1]):
                raise ValueError('Action outside model limits')
            if self.record:self.commands.append(dict(time_s=float(self.data.time),phase=label,target=np.asarray(target).tolist(),duration_s=seconds))
        stats=dict(label=label,max_arm_table_penetration_mm=0.,clearance_samples_mm=[],samples=[])
        for i in range(count):
            if target is not None and not self.disabled_motion:
                t=min(1.,(i+1)/(count*.8));t=t*t*(3-2*t);self.data.ctrl[:]=start+(target-start)*t
            mujoco.mj_step(self.model,self.data)
            if not np.isfinite(self.data.qpos).all():raise ValueError('Nonfinite dynamics')
            # All physics steps are scored: no end-frame-only grasp success.
            state=self.score_state()
            if self.record:self.records.append(dict(phase=label,**state))
            stats['max_arm_table_penetration_mm']=max(stats['max_arm_table_penetration_mm'],state['forbidden_penetration_mm'])
            if i%max(1,round(.02/self.model.opt.timestep))==0 or i==count-1:
                row=dict(phase=label,**state);stats['samples'].append(row)
                stats['clearance_samples_mm'].append(state['clearance_mm'])
            if self.frame_callback and (i%max(1,round(.1/self.model.opt.timestep))==0 or i==count-1):
                self.frame_callback(label)
            # These are simulator-truth aborts, NOT deployable force sensing.
            if state['forbidden_penetration_mm']>1:
                raise ValueError('Forbidden robot/environment penetration exceeds 1mm')
            if state['object_penetration_mm']>1:
                raise ValueError('Pusher contact penetration exceeds 1mm')
            if state['arm_object_normal_force_n']>8:
                raise ValueError('Simulation-only 8N contact gate; no validated physical threshold')
        stats['state']=state
        return stats


class G4Camera(CameraSimulation):
    def __init__(self,source,cad,out,width=1920,**kwargs):
        self.out=out;self.width,self.height=width,width*3//4
        self.sim=G4Simulation(source,cad,out,width=width,**kwargs)
        self.model,self.data=self.sim.model,self.sim.data
        self.camera=self.model.camera('tag_camera').id
        self.renderer=mujoco.Renderer(self.model,height=self.height,width=self.width)
        self.option=mujoco.MjvOption();self.option.geomgroup[3]=0
        self.seq=0;self.calls=[];self.stream_id='g4-'+uuid.uuid4().hex
        self.geometry={'schema':1,'family':'tag36h11','camera_ids':['sim'],
                       'tags':{str(i):{'black_square_mm':s,'source':'Declared G4 simulated marker mount; physical mount unmeasured'}
                               for i,s in ((1,60),(2,40),(31,18))}}
        self.geometry['tags']['2']['mount']={'arm':'right','body':'fixed_gripper_housing','source':'Rigid SO101 gripper_link in MJCF'}
        self.tagged=TagRobot(self,observer=TagObserver(geometry=self.geometry));self.tagged.catalog()
        self.capture_depth=False;self.rng=np.random.default_rng(1)

    def call(self,*args,**kwargs):
        payload=super().call(*args,**kwargs)
        self.last_payload=payload
        if self.capture_depth:
            self.last_depth=self.depth(self.rng)
            self.last_depth_sim_time=float(self.data.time)
        return payload

    def depth(self,rng,noise_m=.0008,dropout=.25):
        self.renderer.enable_depth_rendering();self.renderer.update_scene(self.data,camera='tag_camera',scene_option=self.option)
        z=self.renderer.render().copy();self.renderer.disable_depth_rendering()
        z+=rng.normal(0,noise_m,z.shape);z[rng.random(z.shape)<dropout]=np.nan
        return z


class G4Owner(RenderedOwner):
    """Reuse production calibration transport with only this simulated arm."""
    def __init__(self,source,cad,model_directory,out,**kwargs):
        self.camera=G4Camera(source,cad,out/'scene',**kwargs)
        self.sim,self.model,self.data=self.camera.sim,self.camera.model,self.camera.data
        self.fk=LeRobotSO101(model_directory);self.urdf,self.manifest=verified_model(model_directory)
        self.model_directory=str(model_directory);self.joints=self.fk.names+['gripper']
        self.names=[f'{arm}_arm_{n}' for arm in ('left','right') for n in self.joints]+['head_motor_1','head_motor_2','base_left_wheel','base_right_wheel']
        self.raw_ranges={n:dict(min_ticks=0,max_ticks=4096) for n in self.names}
        for n in self.fk.names:
            radius=math.floor(min(abs(self.fk.ranges[n]))*4096/360)
            self.raw_ranges['right_arm_'+n]=dict(min_ticks=2048-radius,max_ticks=2048+radius)
        self.ranges={n:dict(min_ticks=v['min_ticks']+4,max_ticks=v['max_ticks']-4,margin_ticks=4) for n,v in self.raw_ranges.items()}
        self.now,self.started=1000.,999.;self.enabled=set();self.stopped=False
        self.command_id=self.motor_writes=0;self.commands=[];self.max_arm_table_penetration_mm=0
        self.tagged=TagRobot(self,observer=TagObserver(clock=self.clock,geometry=self.camera.geometry));self.tagged.catalog()
        self.out=out


def asset_report(source,cad,model_directory):
    """Hash every source asset, including unmodeled later-curriculum assets."""
    urdf,manifest=verified_model(model_directory)
    paths=[cad/'shared_tools.scad',cad/'README.txt',source/'scene-assets/arm-import.xml',
           source/'scene-assets/jaw-collision/manifest.json',source/'real-scene/measurement.json',urdf,
           *sorted(cad.glob('*_PROTOTYPE.stl'))]
    for p in (Path('/Users/wk/Downloads/xlerobot-farm-parts')/('cressmaster-'+n+'.stl') for n in ('holder','trough')):
        paths.append(p)
    paths+=list((Path(model_directory)/'assets').glob('*.stl'))
    manifest_jaw=json.loads((source/'scene-assets/jaw-collision/manifest.json').read_text())
    paths += [Path(p) for row in manifest_jaw.values() for p in row['files']]
    return dict(model_revision=manifest['revision'],files=[dict(path=str(p),sha256=sha(p),bytes=p.stat().st_size) for p in paths],
                modeled_task='G4 paper pusher rigid pickup/hold/release only',physical_success=False,hardware_control=False)
