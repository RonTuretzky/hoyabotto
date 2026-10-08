"""Dual-SO101 rigid G4 assembly station; all station measurements are hypotheses.

This module never issues hardware calls or edits source CAD. Object free joints
are initialized once; subsequent manipulation changes joint actuator controls.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
from pathlib import Path
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from scipy.optimize import least_squares

from carton.folding_sim import marker, words
from planter.g4_sim import BLOCKS as PUSHER_BLOCKS
from planter.g4_contact_codec import pack_contact_lists,unpack_contact_lists

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
SIDES=('left','right')

@dataclass(frozen=True)
class StationConfig:
    base_spacing_m: float=.280
    base_y_m: float=-.300
    base_z_m: float=.060
    table_edge_y_m: float=-.180
    guide_pickup_m: tuple=(0.,-.110,.004)
    assembly_origin_m: tuple=(0.,0.,.0235)
    timestep_s: float=.002
    contact_timeconstant_s: float=.004
    noslip_iterations: int=3
    width: int=1280
    roller_handle_support: bool=False

    def validate(self):
        values=[self.base_spacing_m,self.base_y_m,self.base_z_m,self.table_edge_y_m,
                *self.guide_pickup_m,*self.assembly_origin_m,self.timestep_s,self.contact_timeconstant_s]
        if not np.isfinite(values).all() or self.base_spacing_m<=0 or self.timestep_s<=0 or self.contact_timeconstant_s<=0:
            raise ValueError('Finite, positive station dimensions and timestep required')
        if self.base_y_m>=self.table_edge_y_m-.060:
            raise ValueError('Arm bases must remain clear behind tabletop')
        if self.width<320 or self.noslip_iterations<0:raise ValueError('Invalid render/solver configuration')
        return self


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arm_scene(source,config):
    """Import original SO101 bodies, inertias and joint ranges for both arms."""
    config.validate();root=E.parse(Path(source)/'scene-assets/arm-import.xml').getroot()
    root.set('model','G4_dual_SO101_rigid_station')
    compiler=root.find('compiler');meshdir=Path(compiler.get('meshdir'))
    for mesh in root.findall('./asset/mesh'):mesh.set('file',str((meshdir/mesh.get('file')).resolve()))
    compiler.attrib.pop('meshdir',None)
    E.SubElement(root,'size',memory='512M')
    E.SubElement(root,'option',timestep=str(config.timestep_s),integrator='implicitfast',cone='elliptic',
                 impratio='10',iterations='100',tolerance='1e-10',noslip_iterations=str(config.noslip_iterations))
    vis=E.SubElement(root,'visual');E.SubElement(vis,'global',offwidth=str(config.width),offheight=str(config.width*3//4))
    E.SubElement(vis,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    world,assets=root.find('worldbody'),root.find('asset');arm=copy.deepcopy(world.find('body'));world.remove(world.find('body'))
    contact=E.SubElement(root,'contact');actuator=E.SubElement(root,'actuator')
    manifest=json.loads((Path(source)/'scene-assets/jaw-collision/manifest.json').read_text())
    for name,spec in manifest.items():
        for i,path in enumerate(spec['files']):E.SubElement(assets,'mesh',name=f'{name}_part_{i}',file=path)
    for side,sign in [('left',-1),('right',1)]:
        body=copy.deepcopy(arm)
        for element in body.iter():
            if element.get('name'):element.set('name',side+'_'+element.get('name'))
        body.set('pos',words([sign*config.base_spacing_m/2,config.base_y_m,config.base_z_m]))
        body.set('euler',words([0,0,math.pi/2]))
        for parent in body.iter('body'):
            for child in parent.findall('body'):E.SubElement(contact,'exclude',body1=parent.get('name'),body2=child.get('name'))
            for i,geom in enumerate(list(parent.findall('geom'))):
                geom.set('name',f'{parent.get("name")}_geom_{i}')
                if geom.get('contype')=='0':
                    if 'sts' not in geom.get('mesh',''):geom.set('rgba','.22 .46 .76 1' if side=='left' else '.83 .5 .16 1')
                    continue
                geom.set('group','3');geom.set('friction','.8 .003 .0001');geom.set('condim','4')
                geom.set('solref',f'{config.contact_timeconstant_s} 1');geom.set('solimp','.95 .99 .001')
                name=geom.get('mesh')
                if name in manifest:
                    parent.remove(geom)
                    for k in range(len(manifest[name]['files'])):
                        part=copy.deepcopy(geom);part.set('mesh',f'{name}_part_{k}');part.set('name',f'{side}_{name}_part_{k}');parent.append(part)
        for joint in body.iter('joint'):
            joint.set('damping','.2');joint.set('armature','.01');grip=joint.get('name').endswith('_gripper')
            E.SubElement(actuator,'position',name=joint.get('name'),joint=joint.get('name'),kp='80' if grip else '300',
                         kv='3',ctrlrange=joint.get('range'),forcerange='-.5 .5' if grip else '-2.94 2.94')
        grip=body.find(f".//body[@name='{side}_gripper_link']")
        E.SubElement(grip,'site',name=side+'_tip',pos='-.0049 -.0002 -.096',size='.002',rgba='0 0 0 0')
        marker(grip,side+'_tag',4 if side=='left' else 2,.04,[.045,0,.008],[0,1,0,0,0,1])
        world.append(body)
    E.SubElement(world,'geom',name='table',type='box',pos=words([0,config.table_edge_y_m+.55,-.016]),
                 size='.55 .55 .016',rgba='.71 .68 .61 1',friction='.6 .003 .0001',solref=f'{config.contact_timeconstant_s} 1')
    marker(world,'table_tag',1,.04,[.20,.22,.001])
    E.SubElement(world,'light',pos='0 -.35 1.2',dir='0 0 -1',directional='true')
    cameras=[('station',[-.42,-.60,.70],[0,-.01,.04]),('overhead',[0,.04,.80],[0,.04,0]),
             ('front',[0,-.70,.27],[0,-.015,.06]),('overview',[.65,-.8,.62],[0,-.1,.035])]
    for name,pos,look in cameras:
        back=np.array(pos)-look;back/=np.linalg.norm(back);right=np.cross([0,1,0] if name=='overhead' else [0,0,1],back);right/=np.linalg.norm(right);up=np.cross(back,right)
        E.SubElement(world,'camera',name=name,pos=words(pos),xyaxes=words(np.r_[right,up]),fovy='48')
    return root


def write_model(root,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);E.indent(root);path=out/'scene.xml';E.ElementTree(root).write(path,encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(path))


class StationKinematics:
    """IK is based on imported arm geometry, never on observed object truth."""
    def __init__(self,model):
        self.model=model;self.data=mujoco.MjData(model)
        self.indices={s:[model.joint(s+'_'+j).qposadr[0] for j in JOINTS] for s in SIDES}
        self.seeds={s:np.radians([0,50,-30,-20,0]) for s in SIDES}
        self.ranges={s:np.asarray([model.joint(s+'_'+j).range for j in JOINTS[:5]]) for s in SIDES}

    def ik(self,side,target,axis=None,*,strict=True):
        target=np.asarray(target,dtype=float);axis=None if axis is None else np.asarray(axis,dtype=float)
        if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Finite3D target required')
        if axis is not None and (axis.shape!=(3,) or abs(np.linalg.norm(axis)-1)>1e-6):raise ValueError('Unit grasp axis required')
        ix=self.indices[side];ranges=self.ranges[side]
        def error(q):
            self.data.qpos[ix[:5]]=q;mujoco.mj_kinematics(self.model,self.data)
            p=self.data.site(side+'_tip').xpos-target
            return p if axis is None else np.r_[p,(self.data.body(side+'_gripper_link').xmat.reshape(3,3)[:,0]-axis)*.08]
        def solve(seed):return least_squares(error,np.clip(seed,ranges[:,0]+1e-6,ranges[:,1]-1e-6),bounds=(ranges[:,0],ranges[:,1]),max_nfev=200,ftol=1e-11,xtol=1e-11,gtol=1e-11)
        result=solve(self.seeds[side])
        if np.linalg.norm(error(result.x)[:3])>.0015:
            candidates=[result]+[solve([0,a,b,0,0]) for a in (-1.,.5,1.) for b in (-1.,1.)]
            result=min(candidates,key=lambda r:np.linalg.norm(error(r.x)))
        residual=error(result.x);pos=float(np.linalg.norm(residual[:3]));ori=float(np.linalg.norm(residual[3:]))
        if strict and (pos>.0015 or ori>.004):raise ValueError(f'{side} IK failed: {pos*1000:.3f}mm / axis residual{ori:.5f}')
        self.seeds[side]=result.x.copy();return result.x,dict(position_error_mm=pos*1000,axis_error_scaled=ori,rotation_world=self.data.body(side+'_gripper_link').xmat.reshape(3,3).tolist())


def add_source_parts(root,bundle,config):
    """All full-source part colliders remain active; no decorative-only corridors."""
    from planter.g4_collision_assets import load_bundle
    manifest=load_bundle(bundle) if isinstance(bundle,(str,Path)) else bundle
    asset,world=root.find('asset'),root.find('worldbody');metadata={}
    colors={'trough':'.42 .53 .61 1','holder':'.82 .79 .65 1','guide':'.69 .42 .22 1','carrier':'.80 .69 .42 1'}
    for part,spec in manifest['parts'].items():
        if not np.allclose(spec['source_to_assembly'],np.eye(4)):raise ValueError('This station expects unchanged assembly CAD coordinates')
        E.SubElement(asset,'mesh',name=f'g4_{part}_visual',file=str(spec['visual_stl']),scale='.001 .001 .001')
        for i,path in enumerate(spec['collision_meshes']):E.SubElement(asset,'mesh',name=f'g4_{part}_{i}',file=str(path),scale='.001 .001 .001')
    positions={'trough':('trough',config.assembly_origin_m),'holder':('holder',config.assembly_origin_m),
               'guide':('guide',config.guide_pickup_m)}
    for i,x in enumerate([-.30,-.255,.255,.30]):positions[f'carrier{i}']=('carrier',[x,.12,.0206])
    for name,(part,pos) in positions.items():
        spec=manifest['parts'][part];body=E.SubElement(world,'body',name=name,pos=words(pos));E.SubElement(body,'freejoint',name=name+'_free')
        inertia=np.asarray(spec['inertia_kg_m2']);E.SubElement(body,'inertial',mass=str(spec['mass_kg']),pos=words(spec['center_of_mass_m']),
                    fullinertia=words([*np.diag(inertia),inertia[0,1],inertia[0,2],inertia[1,2]]))
        E.SubElement(body,'geom',name=name+'_visual',type='mesh',mesh=f'g4_{part}_visual',contype='0',conaffinity='0',mass='0',rgba=colors[part])
        geoms=[]
        for i in range(len(spec['collision_meshes'])):
            geom=f'{name}_collision_{i}';geoms.append(geom)
            E.SubElement(body,'geom',name=geom,type='mesh',mesh=f'g4_{part}_{i}',group='3',mass='0',condim='4',
                         friction='.6 .003 .0001',solref=f'{config.contact_timeconstant_s} 1',solimp='.95 .99 .001')
        metadata[name]=dict(part=part,body=name,joint=name+'_free',geoms=geoms,mass_kg=spec['mass_kg'])
        if name=='guide':
            for side,sg in [('left',-1),('right',1)]:E.SubElement(body,'site',name='guide_grip_'+side,pos=words([sg*.069,0,.043]),size='.001',rgba='0 0 0 0')
            marker(body,'guide_tag',41,.020,[-.0724,0,.020],[0,-1,0,0,0,1])
        if name=='holder':
            for i,x in enumerate([-.045,-.015,.015,.045]):E.SubElement(body,'site',name=f'holder_slot{i}',pos=words([x,0,0]),size='.001',rgba='0 0 0 0')
    # Three small physical supports avoid a redundant full-face staging contact.
    # This is a hypothetical repeatable station fixture, not modified guide CAD.
    gx,gy,gz=config.guide_pickup_m
    for i,(px,py) in enumerate([(-.066,-.025),(.066,-.025),(0.,.035)]):
        top=gz
        E.SubElement(world,'geom',name=f'guide_staging_pad{i}',type='box',pos=words([gx+px,gy+py,top/2]),size=words([.002,.002,top/2]),friction='.6 .003 .0001',rgba='.32 .33 .35 1')
    for name in ['assembly_origin','guide_seat_target']:
        E.SubElement(world,'site',name=name,pos=words(config.assembly_origin_m),size='.001',rgba='0 0 0 0')
    # Low outside locators are a station-fixture hypothesis, not welded objects.
    ax,ay,_=config.assembly_origin_m
    for name,pos,size in [('left',[ax-.104,ay,.003],[.001,.044,.003]),('right',[ax+.079,ay,.003],[.001,.044,.003]),
                          ('near',[ax-.0125,ay-.044,.003],[.091,.001,.003]),('far',[ax-.0125,ay+.044,.003],[.091,.001,.003])]:
        E.SubElement(world,'geom',name='trough_fixture_'+name,type='box',pos=words(pos),size=words(size),friction='.6 .003 .0001',rgba='.32 .33 .35 1')
    return metadata


def add_pusher(root,cad,position=(.275,-.080,0.)):
    import trimesh
    asset,world=root.find('asset'),root.find('worldbody');path=Path(cad)/'feeder_print_PROTOTYPE.stl'
    mesh=trimesh.load(path,force='mesh');mesh.apply_scale(.001);mesh.density=.012/mesh.volume;I=mesh.moment_inertia
    body=E.SubElement(world,'body',name='pusher',pos=words(position));E.SubElement(body,'freejoint',name='pusher_free')
    E.SubElement(body,'inertial',pos=words(mesh.center_mass),mass='.012',fullinertia=words([*np.diag(I),I[0,1],I[0,2],I[1,2]]))
    E.SubElement(asset,'mesh',name='pusher_visual_mesh',file=str(path),scale='.001 .001 .001')
    E.SubElement(body,'geom',name='pusher_visual',type='mesh',mesh='pusher_visual_mesh',mass='0',contype='0',conaffinity='0',rgba='.28 .38 .59 1')
    for n,p,s in PUSHER_BLOCKS:E.SubElement(body,'geom',name='pusher_'+n,type='box',pos=words(p),size=words(s),group='3',mass='0',friction='.8 .003 .0001',condim='4',solref='.004 1',solimp='.95 .99 .001')
    return dict(body='pusher',joint='pusher_free',mass_kg=.012,source=str(path),source_sha256=sha(path))


def build_station(source,cad,bundle,roller_manifest,out,config=StationConfig()):
    from planter.g4_roller import add_roller
    root=arm_scene(source,config);parts=add_source_parts(root,bundle,config);parts['pusher']=add_pusher(root,cad)
    roller=json.loads(Path(roller_manifest).read_text()) if isinstance(roller_manifest,(str,Path)) else roller_manifest
    # Stage roller on its broad side: its native CAD Z maps to world-X and its
    # native X maps to world-Z. Human axle assembly is explicit preparation.
    parts['roller']=add_roller(root,roller,position_m=(-.275,-.065,.008),quaternion=(math.sqrt(.5),0,math.sqrt(.5),0))
    roller_support=None
    if config.roller_handle_support:
        # Source handle X=+5mm at Z=48..52mm maps to world Z=3mm.
        # The wheel radius is8mm, so a plain floor otherwise leaves this end
        # unsupported initially. This pad is a declared station fixture.
        roller_support=dict(center_m=[-.225,-.065,.0015],size_m=[.004,.004,.003],source_surface='handle X=+5mm, Y=-2..2mm, Z=48..52mm')
        E.SubElement(root.find('worldbody'),'geom',name='roller_handle_staging_pad',type='box',
                     pos=words(roller_support['center_m']),size='.002 .002 .0015',friction='.6 .003 .0001',
                     solref=f'{config.contact_timeconstant_s} 1',rgba='.32 .33 .35 1')
    model=write_model(root,out)
    report=dict(config=asdict(config),parts=parts,physics='All parts free; both arms retain original limits/inertias; roller wheel has passive hinge',
                initial_preparation='Human-prepared holder in trough; loose guide, carriers and tools staged. No robot credit for initialized placements.',
                station_dimensions_status='Explicit layout hypotheses selected by collision/IK diagnostics; not measured physical registration',
                simulated_materials_status='Unmeasured rigid plastic mass/friction/compliance hypotheses',
                actuator_hypotheses=dict(arm_kp=300.,jaw_kp=80.,kv=3.,joint_damping=.2,joint_armature=.01,
                                         arm_force_limit_nm=2.94,jaw_force_limit_nm=.5,
                                         status='Unmeasured position-actuator dynamics; original source joint ranges and link inertias retained'),
                roller_handle_support_pad=roller_support,
                physical_success=False,full_planter_success=False,source_arm_xml_sha256=sha(Path(source)/'scene-assets/arm-import.xml'))
    Path(out,'station.json').write_text(json.dumps(report,indent=2)+'\n');return model,report




def validate_manipulated_object(model,name):
    """Only an explicitly selected, existing free rigid root may touch an arm."""
    if name not in ('guide','pusher'):
        raise ValueError('Manipulated object must be guide or pusher')
    try:body=model.body(name)
    except KeyError as exc:raise ValueError('Manipulated object is absent from station') from exc
    bid=body.id;adr=int(model.body_jntadr[bid])
    if (model.body_parentid[bid]!=0 or model.body_jntnum[bid]!=1 or adr<0 or
            model.jnt_type[adr]!=mujoco.mjtJoint.mjJNT_FREE or model.body_mocapid[bid]>=0):
        raise ValueError('Manipulated object must be an unactuated free rigid root')
    if any(int(model.actuator_trnid[i,0])==adr and
           model.actuator_trntype[i] in (mujoco.mjtTrn.mjTRN_JOINT,mujoco.mjtTrn.mjTRN_JOINTINPARENT)
           for i in range(model.nu)):
        raise ValueError('Manipulated object free joint cannot be actuated')
    return name


class StationSimulation:
    """Joint-actuated station with lossless streamed, independently replayable logs."""
    def __init__(self,model,out,*,config=StationConfig(),render=True,initial_joint_positions=None,manipulated_object='guide'):
        import gzip
        from planter.g4_assembly_score import capture_assembly_state
        if model.nflex:
            raise ValueError('Station contact recorder does not yet support flex contact identifiers')
        self.manipulated_object=validate_manipulated_object(model,manipulated_object)
        self.model=model;self.data=mujoco.MjData(model);self.kin=StationKinematics(model)
        initial_joint_positions=validate_initial_joint_positions(model,initial_joint_positions)
        self.config=config;self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.stream=gzip.open(self.out/'physics.jsonl.gz','wb',compresslevel=1)
        self.frames=[];self.commands=[];self.step_index=0;self.render_enabled=render;self.renderer=None
        self.max_force=0.;self.max_penetration=0.;self.max_contacts=0;self.phase='initial';self.last_points={};self.closed=False
        self.refused=False
        self.control_indices={s:[model.actuator(s+'_'+j).id for j in JOINTS] for s in SIDES}
        for side,sg in [('left',-1),('right',1)]:
            if initial_joint_positions is None:
                target=[sg*.32,-.18,.20];q,_=self.kin.ik(side,target);positions=np.r_[q,.55]
            else:positions=initial_joint_positions[side]
            self.data.qpos[self.kin.indices[side]]=positions
            self.data.ctrl[self.control_indices[side]]=positions
        mujoco.mj_forward(model,self.data)
        bad=[(model.geom(c.geom1).name,model.geom(c.geom2).name,float(-c.dist)) for c in self.data.contact if c.dist<-1e-6]
        if bad:raise ValueError(f'Initial intersection exceeds1micrometre: {bad[:12]}')
        np.savez_compressed(self.out/'initial.npz',qpos=self.data.qpos,qvel=self.data.qvel,ctrl=self.data.ctrl)
        self.write(capture_assembly_state(model,self.data,'initial',0))
        self.capture('initial: full rigid station, human-staged parts')

    def write(self,row):
        row=pack_contact_lists(row)
        try:
            import orjson
            self.stream.write(orjson.dumps(row)+b'\n')
        except ImportError:self.stream.write((json.dumps(row,separators=(',',':'),allow_nan=False)+'\n').encode())

    def capture(self,label):
        if not self.render_enabled:return
        from PIL import Image,ImageDraw
        if self.renderer is None:
            self.renderer=mujoco.Renderer(self.model,height=self.config.width*3//4,width=self.config.width)
            self.option=mujoco.MjvOption();self.option.geomgroup[3]=0
        self.renderer.update_scene(self.data,camera='station',scene_option=self.option)
        image=Image.fromarray(self.renderer.render().copy()).resize((768,576));draw=ImageDraw.Draw(image)
        draw.rectangle((0,0,768,48),fill='white');draw.text((8,6),'G4 RIGID STATION | simulated joint/contact diagnostic | no hardware',fill='black')
        draw.text((8,25),f'{label} | t={self.data.time:.3f}s',fill='black');self.frames.append(image)

    def _loads(self,contacts):
        forces=[float(np.linalg.norm(c['wrench'][:3])) for c in contacts];pairs={}
        for contact,force in zip(contacts,forces):
            pair=tuple(sorted(int(self.model.geom_bodyid[contact[key]]) for key in ('geom1','geom2')))
            pairs[pair]=pairs.get(pair,0.)+force
        return max([0.,*forces,*pairs.values()]),max((-c['distance_m'] for c in contacts),default=0.)

    def _unintended_arm_contact(self,bodies):
        sides=[next((s for s in SIDES if n.startswith(s+'_')),None) for n in bodies]
        return (any(sides) and self.manipulated_object not in bodies and
                not (sides[0] and sides[0]==sides[1]))

    def step(self,phase):
        from planter.g4_assembly_score import capture_contacts,capture_assembly_state
        mujoco.mj_step(self.model,self.data)
        applied=capture_contacts(self.model,self.data)
        warning=[dict(index=i,number=int(w.number),lastinfo=int(w.lastinfo)) for i,w in enumerate(self.data.warning) if w.number]
        mujoco.mj_forward(self.model,self.data)
        self.step_index+=1;self.phase=phase
        row=capture_assembly_state(self.model,self.data,phase,self.step_index,step_contacts=applied)
        self.write(row);self.max_contacts=max(self.max_contacts,len(row['contacts']),len(applied))
        step_peak,step_penetration=self._loads(applied);post_peak,post_penetration=self._loads(row['contacts']);peak=max(step_peak,post_peak);penetration=max(step_penetration,post_penetration);self.max_force=max(self.max_force,peak);self.max_penetration=max(self.max_penetration,penetration)
        if self.step_index%max(1,round(.10/self.model.opt.timestep))==0:self.capture(phase)
        if warning:raise ValueError('MuJoCo runtime warning invalidates simulation: '+str(warning))
        if peak>8.:raise ValueError(f'Provisional8N simulation contact gate: {peak:.4f}N')
        if penetration>.0002:raise ValueError(f'Provisional0.2mm simulation penetration gate: {penetration*1000:.4f}mm')
        if not np.isfinite(self.data.qpos).all():raise ValueError('Nonfinite simulation state')
        # Independent contact abort: not a physical force sensor or visual target.
        for contact in applied+row['contacts']:
            if np.linalg.norm(contact['wrench'][:3])<=.005:continue
            bodies=[self.model.body(self.model.geom_bodyid[contact[key]]).name for key in ('geom1','geom2')]
            if self._unintended_arm_contact(bodies):raise ValueError('Unintended loaded robot/environment contact: '+str(bodies))
        return row

    def move(self,phase,targets=None,*,jaw=None,seconds=1.,cartesian_step=.005):
        print(json.dumps(dict(phase_started=phase,time_s=float(self.data.time))),flush=True)
        targets={} if targets is None else targets;axes={'left':[-1,0,0],'right':[1,0,0]}
        command=self.data.ctrl.copy()
        if jaw is not None:
            for side in SIDES:command[self.control_indices[side][5]]=jaw
        count=max([1]+[int(np.ceil(np.linalg.norm(np.asarray(point)-self.last_points[side])/cartesian_step))
                      for side,point in targets.items() if side in self.last_points and cartesian_step>0])
        starts={s:self.last_points.get(s,np.asarray(p)).copy() for s,p in targets.items()}
        for segment in range(1,count+1):
            ik=[]
            for side,point in targets.items():
                waypoint=starts[side]+(np.asarray(point)-starts[side])*segment/count
                q,error=self.kin.ik(side,waypoint,axes[side]);command[self.control_indices[side][:5]]=q;ik.append(dict(side=side,target_m=waypoint.tolist(),**error))
            if np.any(command<self.model.actuator_ctrlrange[:,0]) or np.any(command>self.model.actuator_ctrlrange[:,1]):raise ValueError('Command violates original arm joint limits')
            self.commands.append(dict(time_s=float(self.data.time),phase=phase,ctrl=command.tolist(),duration_s=seconds/count,ik=ik))
            start=self.data.ctrl.copy();steps=max(1,round(seconds/count/self.model.opt.timestep))
            for i in range(steps):
                frac=min(1.,(i+1)/(steps*.8));frac=frac*frac*(3-2*frac);self.data.ctrl[:]=start+(command-start)*frac;self.step(phase)
        for side,point in targets.items():self.last_points[side]=np.asarray(point).copy()
        self.capture(phase)
        print(json.dumps(dict(phase_completed=phase,time_s=float(self.data.time),step_index=self.step_index)),flush=True)

    def save(self,*,final_label='end: independent score required'):
        if self.closed:return
        self.closed=True;self.stream.close();self.capture(final_label)
        np.savez_compressed(self.out/'final.npz',qpos=self.data.qpos,qvel=self.data.qvel,ctrl=self.data.ctrl)
        (self.out/'commands.json').write_text(json.dumps(self.commands,indent=2)+'\n')
        if self.frames:
            self.frames[0].save(self.out/'before.png');self.frames[-1].save(self.out/'after.png')
            self.frames[0].save(self.out/'timeline.gif',save_all=True,append_images=self.frames[1:],duration=100,loop=0)
        if self.renderer is not None:self.renderer.close()


def guide_targets(origin,*,height_m=.043,correction_m=(0,0,0),inset_m=0.):
    origin=np.asarray(origin);correction=np.asarray(correction_m)
    if not np.isfinite(inset_m) or not 0<=inset_m<.010:raise ValueError('Guide grasp inset must be finite and below10mm')
    return {s:origin+[sg*(.069-inset_m),0,height_m]+correction for s,sg in [('left',-1),('right',1)]}


def validate_initial_joint_positions(model,positions):
    """Constructor-only explicit reset; incomplete or out-of-range arms refuse."""
    if positions is None:return None
    if not isinstance(positions,dict) or set(positions)!=set(SIDES):raise ValueError('Explicit initial positions require both arms and no extra keys')
    result={}
    for side in SIDES:
        q=np.asarray(positions[side],dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Each initial arm requires six finite joint angles')
        limits=np.asarray([model.joint(side+'_'+name).range for name in JOINTS])
        if np.any(q<limits[:,0]) or np.any(q>limits[:,1]):raise ValueError('Initial joint angles violate original arm limits')
        result[side]=q.copy()
    return result
