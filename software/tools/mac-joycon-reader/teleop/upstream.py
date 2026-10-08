"""Boundary adapters for pinned Windows Joy-Con + XLeRobot control definitions.

The controller emits original logical joint values. Destination adapters own
unit conversion, limits, transport, and physical hold-to-run gating.
The movement/filter/IK implementations live unchanged in vendor/.
"""
import math
from types import SimpleNamespace

from mapping import Mapping
from vendor.windows_controller import JoyConController
from vendor.xlerobot_control import SimpleTeleopArm, SimpleHeadControl, get_joycon_base_action
from vendor.so101_kinematics import SO101Kinematics

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')


def to_model(name, degrees):
    """LeRobot logical degrees -> the checked-in MuJoCo model's radians."""
    value=math.radians(degrees)
    if name.endswith('shoulder_pan'):
        return -value+(-math.pi/2 if name.startswith('left') else math.pi/2)
    if name.endswith('shoulder_lift'):return math.pi/2-value
    if name.endswith('elbow_flex'):return value+math.pi/2
    if name.endswith('wrist_roll'):return value+math.pi/2
    if name.endswith('gripper'):return -.25+value
    return value


def from_model(name, radians):
    if name.endswith('shoulder_pan'):radians=(-math.pi/2 if name.startswith('left') else math.pi/2)-radians
    elif name.endswith('shoulder_lift'):radians=math.pi/2-radians
    elif name.endswith('elbow_flex'):radians-=math.pi/2
    elif name.endswith('wrist_roll'):radians-=math.pi/2
    elif name.endswith('gripper'):radians+=.25
    return math.degrees(radians)


def position_for_upstream_ik(kin, shoulder, elbow):
    """Invert the upstream IK's angle convention for simulator readback.

    Its separate forward_kinematics method uses theta1+theta2-pi, which
    does not invert its IK. Do not use that method to seed/rewrite a target:
    it changes a stationary elbow by ~32 degrees at this model's home pose.
    The IK's theta1=beta+gamma requires the second link at theta1-theta2.
    """
    offset=math.atan2(.028,.11257)
    theta1=math.radians(90-shoulder)-offset
    theta2=math.radians(elbow+90)-math.atan2(.0052,.1349)-offset
    return (kin.l1*math.cos(theta1)+kin.l2*math.cos(theta1-theta2),
            kin.l1*math.sin(theta1)+kin.l2*math.sin(theta1-theta2))


class ButtonAdapter:
    """Small interface expected by XLeRobot's head and base functions."""
    def __init__(self,d):self.d=d;self.joycon=self
    def get_button_up(self):return self.d['dpad']['y']>0
    def get_button_down(self):return self.d['dpad']['y']<0
    def get_button_left(self):return self.d['dpad']['x']<0
    def get_button_right(self):return self.d['dpad']['x']>0
    def get_button_x(self):return self.d['buttons']['Button X']
    def get_button_b(self):return self.d['buttons']['Button B']
    def get_button_y(self):return self.d['buttons']['Button Y']
    def get_button_a(self):return self.d['buttons']['Button A']


class WindowsMapping(Mapping):
    mode='upstream'
    interval=.02  # Original controller moves 3 mm per call: 50 Hz = 15 cm/s.
    reader_hz=100
    def __init__(self,robot):
        super().__init__()
        self.robot=robot;self.gyro_enabled=True;self.info={};self.reset()

    def reset(self):
        self.controllers={};self.arms={};self.readers={};self.origins={};self.head=None
        self.info={'source':'Windows JoyConController + XLeRobot direct teleop','warnings':[]}

    def decode(self,f,now=None):
        if f.get('backend')!='Nintendo HID':raise ValueError('Original controls require the independent Mac HID reader')
        d=super().decode(f,now);c=f['controllers'][0]
        b={name:bool(value['pressed']) for name,value in c['buttons'].items()}
        pads=c['pads'];motion=c.get('independent_motion',{})
        def usable(m):
            a=m.get('windows_attitude')
            return (m.get('ready') is True and isinstance(m.get('session'),str)
                    and isinstance(m.get('age_s'),(int,float)) and 0<=m['age_s']<=.12
                    and isinstance(a,dict) and all(type(a.get(k)) in (int,float) and math.isfinite(a[k]) for k in ('roll','pitch','yaw')))
        available=all(usable(motion.get(side,{})) for side in ('left','right'))
        d.update(buttons=b,motion=motion,gyro_available=available,
                 dpad={axis:pads['Direction Pad'][axis]['filtered'] for axis in ('x','y')},
                 deadman={side:b[side.title()+' Shoulder'] for side in ('left','right')})
        d['neutral']=not any(b.values()) and not any(d['axes'].values()) and not any(d['dpad'].values())
        d['ready']=available
        d['identity']=(*d['identity'],*(motion.get(side,{}).get('session') for side in ('left','right')))
        # Original Windows applies its own 0.1 deadzone; do not apply ours twice.
        d['axes']={side+axis:pads[side.title()+' Thumbstick'][axis]['raw'] for side in ('left','right') for axis in ('x','y')}
        if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in d['axes'].values()):raise ValueError('Invalid raw stick axis')
        d['neutral']=not any(b.values()) and not any(abs(v)>=.1 for v in d['axes'].values()) and not any(d['dpad'].values())
        return d

    def initialize(self,d):
        obs=self.robot.get_observation()
        for side in ('left','right'):self.initialize_side(d,side,obs)
        self.head=SimpleHeadControl(obs)

    def initialize_side(self,d,side,obs):
        kin=SO101Kinematics();joints={j:f'{side}_arm_{j}' for j in JOINTS}
        arm=SimpleTeleopArm(joints,obs,kin,prefix=side)
        current={j:obs[n+'.pos'] for j,n in joints.items()}
        x,z=position_for_upstream_ik(kin,current['shoulder_lift'],current['elbow_flex'])
        recovered=kin.inverse_kinematics(x,z)
        if max(abs(recovered[i]-current[j]) for i,j in enumerate(('shoulder_lift','elbow_flex')))>.5:
            raise ValueError(side+': current pose does not fit the original IK branch; no automatic homing')
        reader=SimpleNamespace(state=None);reader.get_state=lambda reader=reader:reader.state
        controller=JoyConController(reader,init_gpos=[x-.1629,current['shoulder_pan']/250,z-.1131,0,0,0])
        controller.gripper_state=controller.gripper_open if current['gripper']>45 else controller.gripper_close
        self.readers[side]=reader;self.controllers[side]=controller;self.arms[side]=arm
        a=d['motion'][side]['windows_attitude']
        self.origins[side]=dict(attitude=dict(a),roll=current['wrist_roll']/45,
            pitch=(10-current['wrist_flex']-current['shoulder_lift']-current['elbow_flex'])/60,
            gripper=current['gripper'],last_gripper=controller.gripper_state)
        arm.target_positions=dict(current)

    def command(self,d,scope):
        return self.robot.encode_upstream_command(self.logical_command(d,scope),d)

    def logical_command(self,d,scope):
        if scope!='wholebody':raise ValueError('Original controls require whole-body scope')
        if not d['gyro_available']:raise ValueError('Independent motion data unavailable; set both Joy-Cons down')
        if not self.controllers:
            self.initialize(d)
            return dict(positions={},linear=0.,angular=0.)
        b=d['buttons'];driving=b['Button Menu'];actions={};warnings=[]
        for side in ('left','right'):
            a=d['motion'][side]['windows_attitude'];origin=self.origins[side];reader=self.readers[side]
            # Wrist zero is captured at Start; original attitude still determines
            # stick directions. Left hardware button names mirror the right demo.
            reader.state=dict(roll=a['roll'],pitch=a['pitch'],yaw=a['yaw']-origin['attitude']['yaw'],
                stick_x=d['axes'][side+'x'],stick_y=d['axes'][side+'y'],buttons={
                'R':b[side.title()+' Shoulder'],'ZR':b[side.title()+' Trigger'],
                'STICK':b[side.title()+' Thumbstick Button'],
                'HOME':b['Button Home' if side=='right' else 'Button Capture'],
                'X':side=='right' and not driving and b['Button X'],
                'B':side=='right' and not driving and b['Button B']})
            controller=self.controllers[side];pose,grip,_=controller.get_control()
            pose[3]=origin['roll']+(pose[3]-(origin['attitude']['roll']-math.pi/2) if self.gyro_enabled else 0)
            pose[4]=origin['pitch']+(pose[4]+origin['attitude']['pitch']*controller.pitch_gain if self.gyro_enabled else 0)
            arm=self.arms[side];arm.handle_joycon_input(pose,grip)
            if grip!=origin['last_gripper']:
                origin['gripper']=90. if grip==controller.gripper_open else 0.
                origin['last_gripper']=grip
            arm.target_positions['gripper']=origin['gripper']
            actions.update(arm.p_control_action(self.robot))
        joycon=ButtonAdapter(d)
        self.head.handle_joycon_input(joycon);actions.update(self.head.p_control_action(self.robot))
        # The original X/B hand shortcuts remain intact. Plus explicitly selects
        # the existing XLeRobot base mapping, so X/B cannot drive and reach at once.
        base=get_joycon_base_action(joycon,self.robot) if driving else {}
        positions,warnings=self.robot.bound_upstream_positions({key.removesuffix('.pos'):value for key,value in actions.items()})
        for name,value in positions.items():
            if name.startswith('head_'):self.head.target_positions[name]=value
        for side in ('left','right'):
            arm=self.arms[side];kin=arm.kinematics
            lift=positions[side+'_arm_shoulder_lift'];elbow=positions[side+'_arm_elbow_flex']
            x,z=position_for_upstream_ik(kin,lift,elbow)
            pan=positions[side+'_arm_shoulder_pan']
            self.controllers[side].set_position([x-.1629,pan/250,z-.1131])
        self.info['warnings']=warnings
        self.info['driving']=driving
        return dict(positions=positions,linear=base.get('x.vel',0.),angular=math.radians(base.get('theta.vel',0.)))
