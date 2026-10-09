"""Local MuJoCo hand-space teleoperation. Never supplies physical calibration.

Five arm DOFs solve XYZ + pitch/roll with a damped Jacobian. The underactuated
SO101 wrist cannot independently satisfy arbitrary XYZ + three-angle poses.
"""
import math
import time
import mujoco
import numpy as np
from mapping import Mapping


def button(controller, name):
    value=controller['buttons'].get(name,{}).get('pressed',False)
    if type(value) is not bool:raise ValueError('Invalid button '+name)
    return value


def axis(controller,pad,component):
    value=controller['pads'].get(pad,{}).get(component,{}).get('filtered',0.)
    if type(value) not in (float,int) or not math.isfinite(value) or abs(value)>1:
        raise ValueError('Invalid directional input')
    return value


class CartesianMapping(Mapping):
    mode='cartesian'
    def __init__(self,model,joint_map):
        super().__init__()
        self.model=model;self.data=mujoco.MjData(model);self.joint_map=joint_map
        self.checked_bumpers=set();self.last_identity=None;self.last_time=None
        self.targets={};self.smoothed={};self.grip_direction={'left':1.,'right':1.}
        self.last_grip={'left':False,'right':False};self.last_dead={'left':False,'right':False}
        self.gyro_refs={};self.gyro_enabled=False;self.gyro_generation=None
        self.home=False;self.feedback=None;self.info={};self.start_pose=None;self.returning=False;self.reference_frame='robot'
        self.sites={s:model.site(s+'_grip_center').id for s in ('left','right')}
        self.names={s:[s+'_arm_'+j for j in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')] for s in ('left','right')}

    def decode(self,f,now=None):
        d=super().decode(f,now);c=f['controllers'][0]
        if d['identity']!=self.last_identity:
            self.checked_bumpers.clear();self.reset();self.last_identity=d['identity']
        buttons={n:button(c,n) for n in ('Left Shoulder','Right Shoulder','Left Thumbstick Button','Right Thumbstick Button','Left Trigger','Right Trigger','Button A','Button B','Button X','Button Y','Button Menu','Button Home')}
        d['deadman']={s:buttons[s.title()+' Shoulder'] for s in ('left','right')}
        if c.get('input_event_count',0)>0:
            self.checked_bumpers.update(s for s,v in d['deadman'].items() if v)
        d['buttons']=buttons;d['dpad']={a:axis(c,'Direction Pad',a) for a in ('x','y')}
        # No held movement control may survive a fresh practice claim.
        d['neutral']=not any(d['deadman'].values()) and not any(d['axes'].values()) and not any(d['dpad'].values()) and not any(buttons.values())
        d['ready']=self.checked_bumpers=={'left','right'}
        d['motion']=c.get('independent_motion',{})
        d['gyro_available']=all(d['motion'].get(s,{}).get('ready') and 0 <= d['motion'][s].get('age_s',1) <= .12 for s in ('left','right'))
        d['mode']='cartesian';d['input_backend']=f.get('backend','Apple GameController')
        return d

    def reset(self):
        self.targets.clear();self.smoothed.clear();self.gyro_refs.clear();self.last_time=None
        self.last_grip={'left':False,'right':False};self.last_dead={'left':False,'right':False}
        self.home=False;self.info={};self.start_pose=None;self.returning=False

    def command(self,d,scope):
        if scope!='wholebody':raise ValueError('Hand-space mode requires local whole-body practice')
        if self.feedback is None:raise ValueError('Simulator feedback unavailable')
        state=self.feedback();motors=state['motors']
        for n,(_,qadr,_,lo,hi) in self.joint_map.items():
            ticks=motors[n]['Present_Position']
            self.data.qpos[qadr]=lo+(ticks-100)/3900*(hi-lo)
        mujoco.mj_forward(self.model,self.data)
        now=time.monotonic();dt=min(.1,max(.001,now-self.last_time)) if self.last_time else .06
        self.last_time=now
        b=d['buttons'];dead=d['deadman'];rates={};warnings=[]
        if self.start_pose is None:self.start_pose={n:motors[n]['Present_Position'] for n in self.joint_map}
        if self.gyro_enabled and not d['gyro_available']:
            raise ValueError('Independent gyro stream missing or stale; enable practice again')
        user_motion=any(d['axes'].values()) or any(d['dpad'].values()) or any(b[k] for k in ('Left Trigger','Right Trigger','Button A','Button B','Button X','Button Y'))
        if b['Button Home'] and not self.home and all(dead.values()) and not user_motion:self.returning=True
        self.home=b['Button Home']
        if self.returning:
            if not all(dead.values()) or user_motion:self.returning=False
            else:
                rates={n:float(np.clip((target-motors[n]['Present_Position'])*4,-60 if n.startswith('head_') else -80,60 if n.startswith('head_') else 80)) for n,target in self.start_pose.items()}
                self.returning=any(abs(target-motors[n]['Present_Position'])>3 for n,target in self.start_pose.items())
                self.targets.clear();self.smoothed.clear();self.gyro_refs.clear()
                self.info={'warnings':['Returning to this practice session’s start pose — release L/R to cancel'] if self.returning else [],'gyro_enabled':self.gyro_enabled}
                return dict(rates=rates,deadman=dead,linear=0.,angular=0.)
        for side in ('left','right'):
            sid=self.sites[side];pos=self.data.site_xpos[sid].copy();rot=self.data.site_xmat[sid].reshape(3,3).copy()
            names=self.names[side];ids=[self.joint_map[n][2] for n in names]
            if side not in self.targets or not dead[side] or not self.last_dead[side]:
                self.targets[side]=(pos.copy(),rot.copy())
                self.smoothed[side]=np.zeros(5)
                self.gyro_refs.pop(side,None)
            target,target_rot=self.targets[side]
            if dead[side]:
                xy=np.array([d['axes'][side+'y'],-d['axes'][side+'x'],0.])
                if self.reference_frame=='hand':xy=-rot[:,1]*d['axes'][side+'y']+rot[:,0]*d['axes'][side+'x']
                if b[side.title()+' Thumbstick Button']:xy=np.array([0.,0.,d['axes'][side+'y']])
                wrist=b['Button Menu']
                if wrist:xy[:]=0
                target+=xy*.025*dt
                # Bounded target lead avoids unreachable-target accumulation.
                delta=target-pos;length=np.linalg.norm(delta)
                if length>.015:target[:]=pos+delta*.015/length;warnings.append(side+': reach/rate limit')
                if self.gyro_enabled:
                    motion=d['motion'][side];q=np.array(motion['quaternion'],dtype=float)
                    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.05:raise ValueError('Invalid gyro quaternion')
                    if side not in self.gyro_refs:self.gyro_refs[side]=(q.copy(),target_rot.copy(),motion['session'])
                    ref,initial,session=self.gyro_refs[side]
                    if motion['session']!=session:raise ValueError('Gyro reconnected; restart practice')
                    # Relative controller roll/pitch, deliberately no unattainable independent yaw.
                    rel=np.zeros(4);conj=ref.copy();conj[1:]*=-1;mujoco.mju_mulQuat(rel,conj,q)
                    roll=math.atan2(2*(rel[0]*rel[1]+rel[2]*rel[3]),1-2*(rel[1]**2+rel[2]**2))
                    pitch=math.asin(np.clip(2*(rel[0]*rel[2]-rel[3]*rel[1]),-1,1))
                    target_rot[:]=rotation(np.array([0.,1.,0.]),np.clip(pitch,-1.2,1.2))@initial
                    target_rot[:]=rotation(initial[:,1],np.clip(roll,-1.2,1.2))@target_rot
                elif wrist:
                    target_rot[:]=rotation(np.array([0.,1.,0.]),d['axes'][side+'y']*.4*dt)@target_rot
                    target_rot[:]=rotation(rot[:,1],d['axes'][side+'x']*.4*dt)@target_rot
                jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
                mujoco.mj_jacSite(self.model,self.data,jp,jr,sid)
                orientation_error=.5*sum((np.cross(rot[:,i],target_rot[:,i]) for i in range(3)),np.zeros(3))
                pitch_axis=np.array([0.,1.,0.]);roll_axis=rot[:,1]
                jac=np.vstack([jp[:,ids],.15*pitch_axis@jr[:,ids],.15*roll_axis@jr[:,ids]])
                error=np.r_[4*(target-pos),.15*3*pitch_axis@orientation_error,.15*3*roll_axis@orientation_error]
                qdot=jac.T@np.linalg.solve(jac@jac.T+np.eye(5)*.002**2,error)
                tick_rates=np.array([v*3900/(self.joint_map[n][4]-self.joint_map[n][3]) for n,v in zip(names,qdot)])
                tick_rates/=max(1.,max(abs(tick_rates))/80)
                # Rate ramps only while held. Release goes immediately to zero.
                previous=self.smoothed[side]
                tick_rates=previous+np.clip(tick_rates-previous,-240*dt,240*dt)
                for i,n in enumerate(names):
                    ticks=motors[n]['Present_Position']
                    if ticks<110 and tick_rates[i]<0 or ticks>3990 and tick_rates[i]>0:tick_rates[i]=0;warnings.append(side+': joint limit')
                self.smoothed[side]=tick_rates
                rates.update(zip(names,map(float,tick_rates)))
            else:rates.update(dict.fromkeys(names,0.))
            grip=b[side.title()+' Trigger']
            if grip and not self.last_grip[side] and dead[side]:self.grip_direction[side]*=-1
            rates[side+'_arm_gripper']=self.grip_direction[side]*80 if grip and dead[side] else 0.
            self.last_grip[side]=grip;self.last_dead[side]=dead[side]
        rates['head_motor_1']=d['dpad']['x']*60 if dead['left'] else 0.
        rates['head_motor_2']=d['dpad']['y']*60 if dead['left'] else 0.
        forward=float(b['Button Y'])-float(b['Button A']);turn=float(b['Button X'])-float(b['Button B'])
        scale=max(1,abs(forward)+abs(turn));linear=forward*.02/scale;angular=turn*.16/scale
        previous=self.smoothed.get('base',np.zeros(2))
        target=np.array([linear,angular]) if all(dead.values()) else np.zeros(2)
        smooth=previous+np.clip(target-previous,-np.array([.04,.32])*dt,np.array([.04,.32])*dt) if all(dead.values()) else target
        # Preserve the shared per-wheel bound while ramping mixed commands.
        smooth/=max(1,(abs(smooth[0])+.125*abs(smooth[1]))/.02)
        self.smoothed['base']=smooth
        self.info={'warnings':sorted(set(warnings)),'hands':{s:list(map(float,self.data.site_xpos[self.sites[s]])) for s in self.sites},'gyro_enabled':self.gyro_enabled}
        return dict(rates=rates,deadman=dead,linear=float(smooth[0]),angular=float(smooth[1]))


def rotation(axis_,angle):
    axis_=np.asarray(axis_)/np.linalg.norm(axis_)
    x,y,z=axis_;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
