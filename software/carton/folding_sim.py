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
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_solver import FoldingSolver,solver_report
from carton.folding_markers import BOX_MARKERS,BOX_TAG_SIZE
from carton.folding_cart import cart_boxes,table_overlap,report as cart_report

JOINTS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
FLAPS = ('short_left','short_right','long_far','long_near')
TAG_IDS = dict(zip(FLAPS, (11,12,13,14)))
_box=Box()
L,W,H,F = _box.length,_box.width,_box.height,_box.flap


def initialize_flaps(model,data,angles=None):
    """Start from separated panels, never overlapping rigid cardboard.

    With all four panels leaning inward, the old 0.1 rad initialization
    interpenetrated adjacent panels by 11 mm. Short panels lean inward and
    long panels outward here; these are initial poses, not new spring rests.
    """
    angles=(dict(short_left=.1,short_right=.1,long_far=-.1,long_near=-.1)
            if angles is None else angles)
    if set(angles)!=set(FLAPS):raise ValueError('Declare all four initial flap angles')
    for flap,value in angles.items():
        joint=model.joint(flap+'_hinge')
        if not np.isfinite(value) or not joint.range[0]<=value<=joint.range[1]:
            raise ValueError('Initial flap angle outside physical model range')
    before=data.qpos.copy()
    for flap,value in angles.items():data.qpos[model.joint(flap+'_hinge').qposadr[0]]=value
    mujoco.mj_fwdPosition(model,data)
    bad=[(model.geom(c.geom1).name,model.geom(c.geom2).name,-c.dist*1000)
         for c in data.contact if c.dist<-.0001
         and model.geom(c.geom1).name.endswith('_cardboard')
         and model.geom(c.geom2).name.endswith('_cardboard')]
    if bad:
        data.qpos[:]=before;mujoco.mj_fwdPosition(model,data)
        raise ValueError(f'Initial flap panels intersect: {bad}')
    return {name:math.degrees(value) for name,value in angles.items()}

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


def build_scene(source:Path,out:Path, *, station:FoldingStation, stiffness=.018, material=None, offset=(0,0), yaw=0., paddle=None, solver=None):
    material=material or CartonMaterial(hinge_stiffness=stiffness)
    if not station.carton_footprint(offset,yaw)['fully_on_table']:
        raise ValueError('Initial carton bottom extends beyond the tabletop')
    if not station.reference_layout and table_overlap(station):
        raise ValueError('Fixed cart intersects the tabletop; base-to-edge distance is not cart-front clearance')
    root=E.parse(source/'scene-assets/arm-import.xml').getroot()
    root.set('model','dual_SO101_passive_carton')
    E.SubElement(root,'option',**(solver or FoldingSolver()).xml_attributes())
    vis=E.SubElement(root,'visual')
    E.SubElement(vis,'global',offwidth='1280',offheight='960')
    E.SubElement(vis,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    asset=root.find('asset');world=root.find('worldbody')
    arm=copy.deepcopy(world.find('body'));world.remove(world.find('body'))
    manifest=json.loads((source/'scene-assets/jaw-collision/manifest.json').read_text())
    for mesh,spec in manifest.items():
        for i,file in enumerate(spec['files']):E.SubElement(asset,'mesh',name=mesh+'_part_'+str(i),file=file)
    contact=E.SubElement(root,'contact');act=E.SubElement(root,'actuator')
    if not station.reference_layout:
        for part in cart_boxes(station):
            E.SubElement(world,'geom',name=part.name,type='box',pos=words(part.center),size=words(part.half_size),
                         rgba='.075 .08 .085 1',friction='.6 .003 .0001',solref='.004 1')
        # The fixed cart includes collision geometry; these wheels and the
        # neck are also explicit obstacles, not moving robot actuators.
        for x in (-.23,.23):
            for y in (-.225,.095):
                E.SubElement(world,'geom',name=f'cart_wheel_{x}_{y}',type='sphere',
                             pos=words([x,station.base_y+y,station.base_height-.76]),size='.04',rgba='.04 .04 .04 1')
        E.SubElement(world,'geom',name='cart_neck',type='box',
                     pos=words([0,station.base_y-.10,station.base_height+.18]),size='.025 .025 .22',rgba='.10 .10 .11 1')
        E.SubElement(world,'geom',name='cart_camera_head',type='box',
                     pos=words([0,station.base_y-.10,station.base_height+.43]),size='.045 .035 .025',rgba='.08 .08 .08 1')
    for side,x in [('left',-station.base_spacing/2),('right',station.base_spacing/2)]:
        b=copy.deepcopy(arm)
        for elem in b.iter():
            if elem.get('name'):elem.set('name',side+'_'+elem.get('name'))
        b.set('pos',words([x,station.base_y,station.base_height]));b.set('euler',words([0,0,math.pi/2]))
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
    E.SubElement(world,'geom',name='table',type='box',pos=words([0,station.table_edge_y+.55,-.016]),size='.55 .55 .016',rgba='.70 .66 .58 1',friction='.7 .003 .0001',solref='.004 1')
    marker(world,'table_tag',1,.060,np.asarray(station.table_tag_position)-[0,0,.0003])
    if station.backup_table_marker_xy is not None:
        marker(world,'table_tag_backup',20,.060,[*station.backup_table_marker_xy,.001])
    box=E.SubElement(world,'body',name='carton',pos=words([*offset,.001]),euler=words([0,0,yaw]))
    E.SubElement(box,'freejoint',name='carton_free')
    common=dict(type='box',rgba='.68 .45 .24 1',friction='.65 .002 .0001',solref='.004 1',solimp='.95 .99 .001')
    scale=material.cardboard_mass_kg/.272
    E.SubElement(box,'geom',name='bottom',pos=words([0,0,.0015]),size=words([L/2,W/2,.0015]),mass=str(.08*scale),**common)
    if material.contents_mass_kg>0:
        half=(material.contents_top_m-.002)/2
        E.SubElement(box,'geom',name='contents',pos=words([0,0,.002+half]),size=words([L/2-.012,W/2-.012,half]),mass=str(material.contents_mass_kg),**{**common,'rgba':'.5 .52 .50 1'})
    for name,pos,size in [('wall_left',[-L/2,0,H/2],[.0015,W/2,H/2]),('wall_right',[L/2,0,H/2],[.0015,W/2,H/2]),('wall_far',[0,W/2,H/2],[L/2,.0015,H/2]),('wall_near',[0,-W/2,H/2],[L/2,.0015,H/2])]:
        E.SubElement(box,'geom',name=name,pos=words(pos),size=words(size),mass=str(.025*scale),**common)
    for tag_id,(name,position,axes) in BOX_MARKERS.items():
        marker(box,name,tag_id,BOX_TAG_SIZE,position,axes)
    specs=[('short_left',[-L/2,0,H],[0,1,0],[.0015,W/2-.004,F/2],[0,-1,0,0,0,1]),
           ('short_right',[L/2,0,H],[0,-1,0],[.0015,W/2-.004,F/2],[0,1,0,0,0,1]),
           ('long_far',[0,W/2,H+.0035],[1,0,0],[L/2-.004,.0015,F/2],[-1,0,0,0,0,1]),
           ('long_near',[0,-W/2,H+.0035],[-1,0,0],[L/2-.004,.0015,F/2],[1,0,0,0,0,1])]
    for index,(name,pos,axis,size,axes) in enumerate(specs):
        f=E.SubElement(box,'body',name=name,pos=words(pos))
        E.SubElement(f,'joint',name=name+'_hinge',axis=words(axis),range='-1.7 3.05',stiffness=str(material.stiffnesses[index]),springref=str(math.radians(material.hinge_rest_degrees)),damping=str(material.hinge_damping),frictionloss=str(material.hinge_friction))
        E.SubElement(f,'geom',name=name+'_cardboard',pos=words([0,0,F/2]),size=words(size),mass=str(.023*scale),**common)
        # MuJoCo filters parent/child contacts by default. Contents must explicitly
        # collide with each hinged flap; otherwise a hinge limit could fake support.
        if material.contents_mass_kg>0:
            E.SubElement(contact,'pair',geom1='contents',geom2=name+'_cardboard',solref='.004 1',solimp='.95 .99 .001',friction='.65 .65 .002 .0001 .0001')
        # Outside-face markers face upward after folding; offset from hand contacts.
        n=np.cross(axes[:3],axes[3:])
        tag_point=[0,.07,.090] if name.startswith('short') else ([-.08,0,.090] if name=='long_far' else [.08,0,.090])
        marker(f,name+'_tag',TAG_IDS[name],.035,np.array(tag_point)+n*.0018,axes)
    # Explicit pairs avoid MuJoCo's max(geom friction) mixing silently keeping
    # the old high friction when only the table coefficient is lowered.
    for name in ('bottom','wall_left','wall_right','wall_far','wall_near',*[f+'_cardboard' for f in FLAPS]):
        # Preserve the previous table/cardboard mixed normal-contact response;
        # only the explicitly selected sliding friction changes here.
        E.SubElement(contact,'pair',geom1='table',geom2=name,condim='3',solref='.004 1',solimp='.925 .97 .001',
                     friction=words([material.table_friction,material.table_friction,.003,.0001,.0001]))
    front=([.0,-.55,.85],[0,-.04,.08]) if station.reference_layout else ([0,station.base_y-.08,station.base_height+.45],[0,-.015,.10])
    # Optional, explicitly selected external RGB-D station camera. It does
    # not change the robot/cart placement and is not a physical calibration.
    for name,pos,look in [('overhead',[0,.00,.85],[0,0,.06]),('front',*front),('station',[-.4,-.45,.85],[0,.075,.13]),('side',[.85,-.3,.5],[0,station.base_y/2,.08]),('overview',[1.1,station.base_y-.95,.6],[0,station.base_y,-.14])]:
        back=np.array(pos)-look;back/=np.linalg.norm(back)
        right=np.cross([0,1,0] if name=='overhead' else [0,0,1],back);right/=np.linalg.norm(right)
        up=np.cross(back,right)
        E.SubElement(world,'camera',name=name,pos=words(pos),xyaxes=words(np.r_[right,up]),fovy='48')
    if paddle is not None:
        from carton.folding_paddle import add_paddle
        add_paddle(root, paddle)
    out.mkdir(parents=True,exist_ok=True)
    E.indent(root);E.ElementTree(root).write(out/'scene.xml',encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(out/'scene.xml'))


class FoldingSimulation:
    def __init__(self,source,out,width=960,height=720,initial_right_roll=None,initial_flaps=None,initial_arm_targets=None,**kwargs):
        self.width,self.height=width,height
        self.station=kwargs['station']
        self.material=kwargs.get('material') or CartonMaterial(hinge_stiffness=kwargs.get('stiffness',.018))
        self.out=Path(out);self.model=build_scene(Path(source),self.out,**kwargs)
        self.data=mujoco.MjData(self.model);self.kin=mujoco.MjData(self.model)
        self.arm_indices={s:[self.model.jnt_qposadr[self.model.joint(s+'_'+n).id] for n in JOINTS] for s in ('left','right')}
        self.control_sites={s:s+'_tip' for s in self.arm_indices}
        self.seeds={s:np.radians([0,50,-30,-20,0]) for s in self.arm_indices}
        self.frames=[];self.frame_states=[];self.events=[];self.stats={'max_bad_penetration_mm':0.,'carton_contact_simulated':True,
            'cart':None if self.station.reference_layout else cart_report(self.station)}
        self.renderer=None;self.option=mujoco.MjvOption();self.option.geomgroup[3]=0
        self.initial_flaps_degrees=initialize_flaps(self.model,self.data,initial_flaps)
        for s,sign in [('left',-1),('right',1)]:
            initial=([sign*(self.station.base_spacing/2+.08),self.station.base_y+.1015,self.station.base_height+.04]
                     if self.station.reference_layout else
                     [sign*(self.station.base_spacing/2+.05),self.station.base_y+.22,self.station.base_height+.28])
            if initial_arm_targets and s in initial_arm_targets:
                initial=np.asarray(initial_arm_targets[s],dtype=float)
                if initial.shape!=(3,) or not np.isfinite(initial).all():
                    raise ValueError('Finite three-dimensional initial arm target required')
            q,err=self.ik(s,initial,orientation=None)
            if err>.008:raise ValueError('Initial arm-relative pose is unreachable')
            if s=='right' and initial_right_roll is not None:
                lo,hi=self.model.joint('right_wrist_roll').range
                if not np.isfinite(initial_right_roll) or not lo<=initial_right_roll<=hi:
                    raise ValueError('Initial wrist roll must remain inside model limits')
                q[4]=initial_right_roll;self.seeds[s]=q.copy()
            ix=self.arm_indices[s];self.data.qpos[ix[:5]]=q;self.data.qpos[ix[5]]=-.17
            self.data.ctrl[[self.model.actuator(s+'_'+n).id for n in JOINTS]]=np.r_[q,-.17]
        mujoco.mj_forward(self.model,self.data)
        self.box_origin=self.data.body('carton').xpos.copy()
        self.box_rotation=self.data.body('carton').xmat.reshape(3,3).copy()
        self.motion_stats={'max_translation_mm':0.,'max_rotation_degrees':0.,'minimum_bottom_corner_table_clearance_mm':float('inf')}

    def measure_carton_motion(self):
        """Independent evaluation only; never supplied to the visual controller."""
        pose=self.data.body('carton');r=pose.xmat.reshape(3,3)
        distance=float(np.linalg.norm(pose.xpos-self.box_origin)*1000)
        angle=math.degrees(math.acos(float(np.clip((np.trace(self.box_rotation.T@r)-1)/2,-1,1))))
        corners=np.array([[x,y,0.] for x in (-L/2,L/2) for y in (-W/2,W/2)])@r.T+pose.xpos
        clearance=float(min(np.min(corners[:,0]+.55),np.min(.55-corners[:,0]),
                            np.min(corners[:,1]-self.station.table_edge_y),np.min(self.station.table_edge_y+1.1-corners[:,1]))*1000)
        self.motion_stats['max_translation_mm']=max(self.motion_stats['max_translation_mm'],distance)
        self.motion_stats['max_rotation_degrees']=max(self.motion_stats['max_rotation_degrees'],angle)
        self.motion_stats['minimum_bottom_corner_table_clearance_mm']=min(self.motion_stats['minimum_bottom_corner_table_clearance_mm'],clearance)
        return {'translation_mm':distance,'rotation_degrees':angle,'minimum_bottom_corner_table_clearance_mm':clearance}

    def ik(self,side,target,orientation=None):
        ix=self.arm_indices[side][:5];site=self.model.site(self.control_sites[side]).id
        ranges=self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
        for arm_indices in self.arm_indices.values():
            self.kin.qpos[arm_indices]=self.data.qpos[arm_indices]
        def fun(q):
            # IK needs rigid transforms only. Contact dynamics are evaluated
            # by mj_step on the separate simulation data during each motion.
            self.kin.qpos[ix]=q;mujoco.mj_kinematics(self.model,self.kin)
            e=self.kin.site_xpos[site]-target
            if orientation is not None:
                axis_index=2 if isinstance(orientation,(str,dict)) else 0
                desired=orientation['direction'] if isinstance(orientation,dict) else ([0,0,1] if axis_index==2 else orientation)
                rotation=self.kin.body(side+'_gripper_link').xmat.reshape(3,3)
                axis=(rotation@np.asarray(orientation['local_axis']) if isinstance(orientation,dict) and 'local_axis' in orientation else rotation[:,axis_index])
                e=np.r_[e,(axis-np.asarray(desired))*.04,q[4]*.005]
                if isinstance(orientation,dict) and 'tangent' in orientation:
                    xaxis=(rotation@np.asarray(orientation['local_tangent'])
                           if 'local_tangent' in orientation else rotation[:,0])
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

    def move(self,targets,seconds=.5,label='',orientation=None,capture=True,grippers=None,joint_targets=None):
        ctrl=self.data.ctrl.copy();errors={}
        for side,values in (joint_targets or {}).items():
            if side in targets:raise ValueError('Choose joint or Cartesian targets for each arm')
            values=np.asarray(values,dtype=float)
            limits=self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
            if values.shape!=(5,) or not np.isfinite(values).all() or np.any(values<limits[:,0]) or np.any(values>limits[:,1]):
                raise ValueError('Joint path target outside original model limits')
            ctrl[[self.model.actuator(side+'_'+j).id for j in JOINTS[:5]]]=values
            self.seeds[side]=values.copy()
        for side,opening in (grippers or {}).items():
            actuator=self.model.actuator(side+'_gripper').id
            lo,hi=self.model.actuator_ctrlrange[actuator]
            if not np.isfinite(opening) or not lo<=opening<=hi:raise ValueError('Gripper target outside model limits')
            ctrl[actuator]=opening
        for side,point in targets.items():
            arm_orientation=orientation.get(side) if isinstance(orientation,dict) and side in orientation else orientation
            q,e=self.ik(side,np.asarray(point),arm_orientation);errors[side]=e
            if e>.008:raise ValueError(f'IK {side} target {point} misses by {e*1000:.1f} mm')
            ctrl[[self.model.actuator(side+'_'+j).id for j in JOINTS[:5]]]=q
        start=self.data.ctrl.copy();n=max(1,round(seconds/self.model.opt.timestep))
        contact_names=set();bad_pairs=set();bad=0.
        extrema={f:[float('inf'),float('-inf')] for f in FLAPS}
        for i in range(n):
            t=min(1,(i+1)/(n*.8));self.data.ctrl[:]=start+(ctrl-start)*(t*t*(3-2*t))
            mujoco.mj_step(self.model,self.data)
            motion=self.measure_carton_motion()
            for c in self.data.contact:
                a,b=self.model.geom(c.geom1).name,self.model.geom(c.geom2).name
                if 'cardboard' in a or 'cardboard' in b:
                    if a.startswith(('left_','right_')) or b.startswith(('left_','right_')):contact_names.add((a,b))
                # Arm/table, arm/cart, arm/rigid carton, and arm/arm penetration.
                forbidden=self.forbidden_contact(a,b)
                if forbidden:
                    bad=max(bad,-c.dist*1000)
                    if c.dist<-.001:bad_pairs.add((a,b))
            for flap,value in self.truth_angles().items():
                extrema[flap][0]=min(extrema[flap][0],value);extrema[flap][1]=max(extrema[flap][1],value)
            step_error=self.step_diagnostic()
            if capture and i%100==0:self.capture(label)
            if bad>1. or step_error:
                # End this offline motion at the first forbidden contact,
                # rather than driving through the remainder of the segment.
                if capture:self.capture('STOP: '+(step_error or 'forbidden robot contact'))
                break
        self.stats['max_bad_penetration_mm']=max(self.stats['max_bad_penetration_mm'],bad)
        event={'label':label,'duration_s':(i+1)*self.model.opt.timestep,'requested_duration_s':seconds,'stopped_early':i<n-1,'flap_angle_extrema_degrees':extrema,'time':float(self.data.time),'ik_error_m':errors,'flap_degrees':self.truth_angles(),'contact_pairs':sorted(contact_names),'bad_penetration_mm':bad,'max_target_tracking_error_m':max([0.]+[float(np.linalg.norm(self.data.site(self.control_sites[a]).xpos-np.asarray(p))) for a,p in targets.items()]),'tip_m':{s:self.data.site(self.control_sites[s]).xpos.tolist() for s in self.arm_indices},'step_error':step_error}
        self.events.append(event)
        event['forbidden_contact_pairs']=sorted(bad_pairs)
        event['carton_motion']=motion
        if joint_targets:
            event['joint_targets_radians']={s:np.asarray(v).tolist() for s,v in joint_targets.items()}
            event['max_joint_tracking_error_radians']=max(float(np.max(np.abs(self.data.qpos[self.arm_indices[s][:5]]-v))) for s,v in joint_targets.items())
        event['targets_m']={s:np.asarray(p).tolist() for s,p in targets.items()}
        event['orientation_error_degrees']={}
        if orientation is not None:
            for side in targets:
                ori=orientation.get(side) if isinstance(orientation,dict) and side in orientation else orientation
                if ori is None:continue
                axes=({'direction':[0,0,1]} if isinstance(ori,str) else ori if isinstance(ori,dict) else {'tangent':ori})
                rotation=self.data.body(side+'_gripper_link').xmat.reshape(3,3)
                event['orientation_error_degrees'][side]={key:math.degrees(math.acos(float(np.clip(
                    (rotation@np.asarray(axes['local_axis']) if key=='direction' and 'local_axis' in axes
                     else rotation@np.asarray(axes['local_tangent']) if key=='tangent' and 'local_tangent' in axes
                     else rotation[:,2 if key=='direction' else 0])@np.asarray(vector)/np.linalg.norm(vector),-1,1))))
                    for key,vector in axes.items() if key in ('direction','tangent')}
        return event

    def forbidden_contact(self,a,b):
        arms=(a.startswith(('left_','right_')),b.startswith(('left_','right_')))
        return all(arms) or (any(arms) and ('table' in (a,b) or any(v.startswith(('wall_','cart_')) or v=='contents' for v in (a,b))))

    def step_diagnostic(self):
        return None

    def arm_tag_fk(self,side):
        # Uses robot encoders, fixed base registration and declared CAD mount.
        # No carton qpos or scene-object truth enters this kinematic prediction.
        for indices in self.arm_indices.values():self.kin.qpos[indices]=self.data.qpos[indices]
        mujoco.mj_kinematics(self.model,self.kin)
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
        self.frame_states.append({'time':float(self.data.time),'label':label,'qpos':self.data.qpos.tolist()})
        im=Image.fromarray(self.render());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,self.width,44),fill='white');draw.text((10,6),'SIMULATION | ASSUMED STATION | NO PHYSICAL REGISTRATION',fill='black');draw.text((10,25),label,fill='black')
        draw.rectangle((8,49,82,66),fill='white');draw.text((12,51),'LEFT ARM',fill='black')
        draw.rectangle((self.width-90,49,self.width-8,66),fill='white');draw.text((self.width-86,51),'RIGHT ARM',fill='black')
        self.frames.append(im)

    def save(self,name='trial'):
        report={'simulation_only':True,'hardware_commands':0,'actuated_carton_joints':0,'engine':mujoco.__version__,'solver':solver_report(self.model),'initial_flaps_degrees':self.initial_flaps_degrees,'events':self.events,'stats':self.stats,'carton_motion':self.motion_stats,'material':self.material.report(),'final_angles':self.truth_angles()}
        (self.out/(name+'.json')).write_text(json.dumps(report,indent=2))
        (self.out/(name+'-frames.json')).write_text(json.dumps(self.frame_states))
        if self.frames:
            self.frames[0].save(self.out/(name+'-before.png'));self.frames[-1].save(self.out/(name+'-after.png'))
            self.frames[0].save(self.out/(name+'.gif'),save_all=True,append_images=self.frames[1:],duration=100,loop=0)
        if self.renderer:self.renderer.close()
        return report
