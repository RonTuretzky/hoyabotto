"""Offline free-body paddle and a geometry-aware contact strategy.

The tool starts placed between the jaws. Pickup is outside this experiment.
Its only support is real jaw contact: no weld, hidden actuator or kinematic
reset after initialization. Dimensions reuse parts/make_parts.py and the
earlier paddle grasp experiment; mass and friction remain assumptions.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from carton.folding_sim import FoldingSimulation, words,marker
from carton.folding_tool_tags import PADDLE_TAGS,PADDLE_TAG_SIZE
from carton.folding_diagonal import DiagonalFoldingController

HANDLE = np.array([.030, 0., .003])
BLADE = np.array([.135, 0., .003])
TIP = np.array([-.0049, -.0002, -.096])
# Replayed the existing paddle_sim.py close/lift/hold test, with both jaw
# contacts and 64.37 mm table clearance. This is its measured *simulated*
# settled grip transform, not a physical calibration or an assumed weld.
GRIP_ROTATION = np.array([
    [-.07778209439541345, .03747661935596237, .9962657520928391],
    [.9434231050288606, .32585740493515675, .06139866893205429],
    [-.3223395580582157, .9446758463359656, -.060702180028897525]])
GRIP_ORIGIN = np.array([-.0051046313316657965,-.02879271694312159,-.0865115664670298])
TOOL_POINT = GRIP_ORIGIN + GRIP_ROTATION @ BLADE
PADDLE_CAD = Path(__file__).resolve().parents[1]/'parts/carton/paddle_flap.stl'


@dataclass(frozen=True)
class PaddleSpec:
    mass: float = .030
    friction: float = .8
    attachment: str = 'friction'
    grasp_x_m: float = .030
    grasp_yaw_degrees: float = 0.

    def __post_init__(self):
        if not math.isfinite(self.mass) or self.mass <= 0:
            raise ValueError('Positive finite paddle mass required')
        if not math.isfinite(self.friction) or self.friction < 0:
            raise ValueError('Finite nonnegative paddle friction required')
        if self.attachment not in ('friction','rigid-diagnostic'):
            raise ValueError('Unknown paddle attachment')
        if not math.isfinite(self.grasp_x_m) or not .010<=self.grasp_x_m<=.190:
            raise ValueError('Paddle grasp must lie within the declared CAD')
        if not math.isfinite(self.grasp_yaw_degrees):
            raise ValueError('Finite paddle grasp yaw required')

    @property
    def grip_rotation(self):
        angle=math.radians(self.grasp_yaw_degrees)
        c,s=math.cos(angle),math.sin(angle)
        return GRIP_ROTATION@np.array([[c,-s,0],[s,c,0],[0,0,1]])

    @property
    def grip_origin(self):
        if self.grasp_yaw_degrees==0:
            return GRIP_ORIGIN+GRIP_ROTATION@np.array([.030-self.grasp_x_m,0,0])
        # Rotate the initial placement in the jaw plane about the grip point.
        # This is not an actuator or a tool-pose reset during a run.
        center=GRIP_ORIGIN+GRIP_ROTATION@HANDLE
        return center-self.grip_rotation@np.array([self.grasp_x_m,0,.003])

    @property
    def tool_point(self):
        return self.grip_origin+self.grip_rotation@BLADE

    def report(self):
        return dict(mass_kg=self.mass, sliding_friction=self.friction,
                    parameters_measured=False, dimensions_mm=[210,40,6],
                    mesh_sha256=hashlib.sha256(PADDLE_CAD.read_bytes()).hexdigest(),
                    mounting=('Free body initially placed in right jaws; no weld; pickup not tested' if self.attachment=='friction'
                              else 'IDEAL RIGID ATTACHMENT DIAGNOSTIC: no slip possible, not a validated grip'),
                    grip_from_paddle_rotation=self.grip_rotation.tolist(),
                    grasp_from_handle_base_mm=self.grasp_x_m*1000,
                    initial_grasp_yaw_degrees=self.grasp_yaw_degrees,
                    grip_from_paddle_origin_m=self.grip_origin.tolist())


def add_paddle(root, spec):
    asset=root.find('asset');world=root.find('worldbody')
    E.SubElement(asset,'mesh',name='paddle_visual',file=str(PADDLE_CAD),scale='.001 .001 .001')
    grip=root.find(".//body[@name='right_gripper_link']")
    if spec.attachment=='friction':
        body=E.SubElement(world,'body',name='paddle',pos='0 0 2')
        E.SubElement(body,'freejoint',name='paddle_free')
    else:
        body=E.SubElement(grip,'body',name='paddle',pos=words(spec.grip_origin),
                          xyaxes=words(np.r_[spec.grip_rotation[:,0],spec.grip_rotation[:,1]]))
    E.SubElement(body,'geom',name='right_paddle_visual',type='mesh',mesh='paddle_visual',
                 rgba='.94 .94 .88 1',contype='0',conaffinity='0',group='2',mass='0')
    # Exact rectangular CAD regions including the two thin grip grooves.
    parts=[(.012,.012,.009,.003),(.025,.001,.009,.002),(.035,.009,.009,.003),
           (.045,.001,.009,.002),(.053,.007,.009,.003),(.135,.075,.020,.003)]
    volumes=np.array([8*x*y*z for _,x,y,z in parts]);volumes/=volumes.sum()
    for i,((cx,x,y,z),fraction) in enumerate(zip(parts,volumes)):
        E.SubElement(body,'geom',name=f'right_paddle_contact_{i}',type='box',
                     pos=words([cx,0,.003]),size=words([x,y,z]),mass=str(spec.mass*fraction),
                     friction=words([spec.friction,.005,.0001]),condim='4',group='3',
                     solref='.004 1',solimp='.95 .99 .001')
    E.SubElement(body,'site',name='paddle_blade_actual',pos=words(BLADE),size='.002',rgba='0 0 0 0')
    E.SubElement(body,'site',name='paddle_control_actual',pos=words(BLADE),size='.002',rgba='0 0 0 0')
    E.SubElement(grip,'site',name='right_paddle_target',pos=words(spec.tool_point),size='.002',rgba='0 0 0 0')
    # Separate calibrated TCP; the original slip-reference site stays fixed.
    E.SubElement(grip,'site',name='right_paddle_observed_target',pos=words(spec.tool_point),size='.002',rgba='0 0 0 0')
    for tag_id,(name,position,axes) in PADDLE_TAGS.items():
        marker(body,name,tag_id,PADDLE_TAG_SIZE,position,axes)
    # Explicit friction on tool/jaw contacts makes the zero-friction negative
    # control meaningful despite MuJoCo's default max-coefficient mixing.
    for jaw in list(grip.iter('geom')) if spec.attachment=='friction' else []:
        name=jaw.get('name','')
        if not any(n in name for n in ('moving_jaw','wrist_roll_follower')) or jaw.get('contype')=='0':continue
        for i in range(len(parts)):
            E.SubElement(root.find('contact'),'pair',geom1=name,geom2=f'right_paddle_contact_{i}',
                         condim='4',friction=words([spec.friction,spec.friction,.005 if spec.friction else 0,.0001,.0001]),
                         solref='.004 1',solimp='.95 .99 .001')


class PaddleFoldingSimulation(FoldingSimulation):
    def __init__(self,*args,paddle=None,**kwargs):
        self.paddle_spec=paddle or PaddleSpec()
        self.tool_samples=[];self.monitor_tool=False
        super().__init__(*args,paddle=self.paddle_spec,**kwargs)
        if self.paddle_spec.attachment=='friction':
            grip=self.data.body('right_gripper_link');r=grip.xmat.reshape(3,3)
            adr=self.model.joint('paddle_free').qposadr[0]
            self.data.qpos[adr:adr+3]=grip.xpos+r@self.paddle_spec.grip_origin
            quaternion=np.zeros(4);mujoco.mju_mat2Quat(quaternion,(r@self.paddle_spec.grip_rotation).ravel())
            self.data.qpos[adr+3:adr+7]=quaternion
        # Reuse the settled jaw position from the preceding independent grasp
        # test. Grip torque remains limited by the existing 0.5 Nm actuator.
        self.data.qpos[self.arm_indices['right'][5]]=-.12127145799576845
        mujoco.mj_forward(self.model,self.data)
        self.control_sites['right']='right_paddle_target'
        self.monitor_tool=True

    def register_observed_tcp(self,observation,point,*,observation_sequence,current_sequence):
        """Register a CAD point from a fresh RGB-D tool pose and encoder FK.

        Only measurement sites change. The passive tool, joint state and
        original slip reference remain untouched. Sequence identity prevents
        reusing a previously captured pose after another camera observation.
        """
        from farm.kinematics.lerobot import transform
        if observation is None or observation_sequence!=current_sequence:
            raise ValueError('Fresh paddle marker observation required for TCP registration')
        tool=transform(observation['world_from_paddle'])
        point=np.asarray(point,dtype=float)
        if (point.shape!=(3,) or not np.isfinite(point).all() or
                not 0<=point[0]<=.210 or abs(point[1])>.020 or not 0<=point[2]<=.006):
            raise ValueError('TCP must lie within the declared paddle CAD')
        grip=self.data.body('right_gripper_link');rotation=grip.xmat.reshape(3,3)
        relative_rotation=rotation.T@tool[:3,:3]
        origin=rotation.T@(tool[:3,3]-grip.xpos)
        self.model.site('right_paddle_observed_target').pos[:]=origin+relative_rotation@point
        self.model.site('paddle_control_actual').pos[:]=point
        self.control_sites['right']='right_paddle_observed_target'
        mujoco.mj_forward(self.model,self.data)
        return dict(origin=origin.tolist(),rotation=relative_rotation.tolist(),
                    cad_point_m=point.tolist(),observation_sequence=observation_sequence,
                    source=observation)

    def actual_control_position(self,side):
        if side=='right':return self.data.site('paddle_control_actual').xpos
        return super().actual_control_position(side)

    def forbidden_contact(self,a,b):
        if a.startswith('right_paddle_') or b.startswith('right_paddle_'):
            other=b if a.startswith('right_paddle_') else a
            if other.startswith('right_') and any(s in other for s in ('moving_jaw','wrist_roll_follower')):
                return False  # Intentional grip contact only.
            if other.endswith('_cardboard'):return False
            return other=='table' or other=='bottom' or other=='contents' or other.startswith(('left_','right_','wall_','cart_'))
        return super().forbidden_contact(a,b)

    def step_diagnostic(self):
        if not self.monitor_tool:return None
        grip=self.data.body('right_gripper_link');r=grip.xmat.reshape(3,3)
        actual=self.data.body('paddle');rotation=(r@self.paddle_spec.grip_rotation).T@actual.xmat.reshape(3,3)
        angle=math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)/2,-1,1))))
        error=float(np.linalg.norm(self.data.site('paddle_blade_actual').xpos-self.data.site('right_paddle_target').xpos)*1000)
        self.tool_samples.append(dict(time=float(self.data.time),blade_offset_mm=error,rotation_degrees=angle))
        if error>15 or angle>20:return f'Paddle grip slipped: blade offset {error:.1f} mm, rotation {angle:.1f} deg'
        return None

    def move(self,*args,**kwargs):
        event=super().move(*args,**kwargs)
        if event.get('step_error'):raise ValueError(event['step_error'])
        return event

    def save(self,*args,**kwargs):
        report=super().save(*args,**kwargs)
        report['paddle']=dict(assumptions=self.paddle_spec.report(),
            max_blade_offset_mm=max((s['blade_offset_mm'] for s in self.tool_samples),default=0),
            max_rotation_degrees=max((s['rotation_degrees'] for s in self.tool_samples),default=0),
            final=self.tool_samples[-1] if self.tool_samples else None)
        (self.out/'paddle-grip.json').write_text(__import__('json').dumps(report['paddle'],indent=2))
        return report


class PaddleFoldingController(DiagonalFoldingController):
    """Same flap order/contact locations; right-hand TCP is blade centre.

    The right jaw stays closed. The blade's flat face targets panel normal;
    blade yaw within that plane is free, so five joints solve three position
    and two face-normal constraints. This is a first
    tool-aware strategy, not an optimized planner or physical calibration.
    """
    @staticmethod
    def blade_orientation(normal,radial):
        return {'direction':normal.tolist(),'local_axis':GRIP_ROTATION[:,2].tolist()}

    def contact(self,side,theta,axis,sign,along,radius,tilt,clearance):
        # Same intended panel point, with the appropriate TCP-to-contact-face
        # offset: 3 mm paddle half-thickness + 1.5 mm cardboard half-thickness.
        point,orientation=super().contact(side,theta,axis,sign,along,radius,tilt,
                                         .0045 if side=='right' else clearance)
        if side=='right':
            normal=np.zeros(3);normal[axis]=sign*math.cos(theta);normal[2]=math.sin(theta)
            radial=np.zeros(3);radial[axis]=-sign*math.sin(theta);radial[2]=math.cos(theta)
            orientation=self.blade_orientation(normal,radial)
        return point,orientation
