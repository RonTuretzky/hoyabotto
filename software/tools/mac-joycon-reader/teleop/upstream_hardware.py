"""Pinned mTLS adapter for original logical actions; no serial ownership here.

Construction never connects. The caller injects the existing authenticated
transport, and the measured reference must match the sole owner's reference.
The existing velocity/ownership protocol and its limits are retained.
"""
import copy
import math
import sys
import time
from pathlib import Path

OWNER_MODULES=Path(__file__).resolve().parents[3]/'docs/commissioning/2026-10-07-paddle-success/qwen-bridge'
if str(OWNER_MODULES) not in sys.path:sys.path.insert(0,str(OWNER_MODULES))
from joycon_reference import PhysicalReference,POSITION_NAMES
from upstream import WindowsMapping


class UpstreamHardware:
    preview=False
    simulation=False
    supports_upstream=True
    teleop_keys={'forward':'w','backward':'s','rotate_left':'a','rotate_right':'d'}
    def __init__(self,transport,reference,clock=time.monotonic,rail_mode='hold'):
        if not isinstance(reference,PhysicalReference):raise ValueError('Exact upstream calibration reference required')
        if rail_mode not in ('hold','toggle'):raise ValueError('Unknown rail mode')
        self.rail_mode=rail_mode
        self.transport=transport;self.reference=reference;self.clock=clock
        self.state=None;self.received=None;self.dead={'left':False,'right':False}

    def call(self,path,body=None,timeout=None):
        if path=='claim':
            self.refresh()
            body=dict(body,control_mode='upstream',reference_id=self.reference.reference_id)
        result=self.transport.call(path,body,timeout=.4 if timeout is None else timeout)
        state=result if path=='status' else result.get('status')
        if state is not None:self.state=state;self.received=self.clock()
        return result

    def refresh(self):
        self.call('status',timeout=2);self.check_feedback()

    def check_feedback(self):
        s=self.state
        if not s or self.received is None or not 0<=self.clock()-self.received+s.get('status_age_s',float('inf'))<=.25:
            raise ValueError('Physical joint feedback stale')
        if s.get('ok') is not True:raise ValueError('Robot owner is not healthy')
        if not isinstance(s.get('goals'),dict):raise ValueError('Motor owner goal feedback unavailable; update the teleop API')
        if s.get('calibration_mismatches'):raise ValueError('Robot calibration does not match hardware')
        if s.get('teleop',{}).get('upstream_reference_id')!=self.reference.reference_id:
            raise ValueError('Robot owner has no matching upstream calibration reference')
        if not set(POSITION_NAMES)<=set(s.get('motors',{})):raise ValueError('All arms, grippers and head feedback required')
        for key in ('wheelbase_m','wheel_limit_m_s'):
            value=s.get('teleop',{}).get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError('Robot owner must report its wheel geometry and limit')
        return s

    def get_observation(self):
        s=self.check_feedback()
        return {n+'.pos':self.reference.from_ticks(n,s['motors'][n]['Present_Position']) for n in POSITION_NAMES}

    def bound_upstream_positions(self,positions):
        state=self.check_feedback();bounded={};warnings=[]
        for n,value in positions.items():
            side='right' if n.startswith('right_') else 'left'
            q=state['motors'][n]['Present_Position'];lo,hi=self.reference.limits(n)
            # Captured neutral positions may be outside command margins (or
            # the original gripper's 0..90 command span). Holding a rail must
            # not pull them to a limit without a changed target.
            current=self.reference.from_ticks(n,q)
            if not self.dead[side] or math.isclose(value,current,abs_tol=1e-9):target=q
            else:
                limited=max(lo,min(hi,value));target=self.reference.to_ticks(n,limited)
                target=max(q-40,min(q+40,target))
            if abs(target-q)<=getattr(self.reference,'quantization_ticks',0):target=q
            limited=self.reference.from_ticks(n,target);bounded[n]=limited
            if abs(limited-value)>1e-6:warnings.append(n.replace('_',' ')+' bounded')
        return bounded,warnings

    def encode_upstream_command(self,command,decoded):
        s=self.check_feedback();rates={}
        for n,value in command['positions'].items():
            side='right' if n.startswith('right_') else 'left'
            if not self.dead[side]:rates[n]=0.;continue
            q=s['motors'][n]['Present_Position']
            current=self.reference.from_ticks(n,q)
            target=q if math.isclose(value,current,abs_tol=1e-9) else self.reference.to_ticks(n,value)
            # Original LeRobot conversion truncates to integer ticks. A
            # round-trip rounding tick must not become a slow idle drift.
            if abs(target-q)<=getattr(self.reference,'quantization_ticks',0):target=q
            # The owner integrates rates into its existing goal. Using measured
            # position here integrates servo lag a second time and can run that
            # goal away from the physical joint, especially at demo speed.
            goal=s.get('goals',{}).get(n)
            if type(goal) not in (int,float) or not math.isfinite(goal):
                raise ValueError('Motor owner goal feedback unavailable; update the teleop API')
            limit=60. if n.startswith('head_') else 80.
            if s['teleop'].get('speed_profile')=='demo' and '_arm_' in n and not n.endswith('gripper'):
                limit=min(300.,s['teleop'].get('position_rate_limits_ticks_s',{}).get(n,80.))
            # Track the upstream target within the existing physical velocity
            # contract. Do not inherit the simulator's unrestricted positions.
            rates[n]=max(-limit,min(limit,(target-goal)*max(5.,limit/40.)))
        linear,angular=command['linear'],command['angular']
        if not all(self.dead.values()):linear=angular=0.
        # Keep the commissioned 2 cm/s *per wheel* cap and 0.16 rad/s cap.
        half_track=s['teleop']['wheelbase_m']/2
        wheel_limit=min(.02,s['teleop']['wheel_limit_m_s'])
        scale=max(1.,abs(linear-angular*half_track)/wheel_limit,abs(linear+angular*half_track)/wheel_limit,abs(angular)/.16)
        return dict(rates=rates,deadman=dict(self.dead),linear=linear/scale,angular=angular/scale)

    def _from_keyboard_to_base_action(self,keys):
        k=self.teleop_keys
        return {'x.vel':.1*(int(k['forward'] in keys)-int(k['backward'] in keys)),
                'theta.vel':30.*(int(k['rotate_left'] in keys)-int(k['rotate_right'] in keys))}

    def make_mapping(self):return HardwareMapping(self)


class HardwareMapping(WindowsMapping):
    def __init__(self,robot):
        self.rail_checks=set();self.rail_identity=None
        super().__init__(robot)
    def reset(self):
        super().reset()
        self.session_enabled=False
        self.latched={'left':False,'right':False}
        self.rail_previous={'left':False,'right':False}
        self.recenter=set()
        self.info['rail_mode']=self.robot.rail_mode
        self.info['physical_hold_to_run']='rail toggle' if self.robot.rail_mode=='toggle' else 'rail buttons'
    def begin_session(self):
        self.session_enabled=True
    def end_session(self):
        self.session_enabled=False
        self.latched={'left':False,'right':False}
        self.robot.dead={'left':False,'right':False}
        self.recenter.clear()
    def decode(self,f,now=None):
        d=super().decode(f,now)
        if d['identity']!=self.rail_identity:
            self.end_session();self.rail_checks.clear();self.rail_identity=d['identity']
        rails={s:any(d['buttons'].get(s.title()+' Rail '+b,False) for b in ('SL','SR')) for s in ('left','right')}
        for s,held in rails.items():
            if held:self.rail_checks.add(s)
        if self.robot.rail_mode=='toggle':
            movement_neutral=(not any(v for n,v in d['buttons'].items() if ' Rail ' not in n)
                              and not any(abs(v)>=.1 for v in d['axes'].values())
                              and not any(d['dpad'].values()))
            for side,held in rails.items():
                if self.session_enabled and held and not self.rail_previous[side]:
                    if self.latched[side] or movement_neutral:
                        self.latched[side]=not self.latched[side]
                        if self.latched[side]:self.recenter.add(side)
            self.rail_previous=rails
            d['deadman']=dict(self.latched)
        else:d['deadman']=rails
        d['rails']=rails
        d['ready']=d['ready'] and self.rail_checks=={'left','right'}
        return d
    def prepare_start(self,d):
        self.robot.refresh();self.initialize(d)
        self.reset()  # Claim's neutral frame takes a fresh post-claim pose.
    def command(self,d,scope):
        self.robot.dead=dict(d['deadman']);gated=copy.deepcopy(d)
        if self.controllers:
            obs=self.robot.get_observation()
            for side,held in d['deadman'].items():
                if held and side not in self.recenter:continue
                home=list(self.controllers[side].init_position)
                self.initialize_side(d,side,obs)
                self.controllers[side].init_position=home
                self.recenter.discard(side)
                for axis in ('x','y'):gated['axes'][side+axis]=0.
                for name in (side.title()+' Shoulder',side.title()+' Trigger',side.title()+' Thumbstick Button','Button Home' if side=='right' else 'Button Capture'):
                    gated['buttons'][name]=False
                if side=='right':
                    for key in ('Button X','Button B'):gated['buttons'][key]=False
            if not d['deadman']['left']:
                gated['dpad']={'x':0.,'y':0.}
                self.head.target_positions={n:obs[n+'.pos'] for n in ('head_motor_1','head_motor_2')}
        return super().command(gated,scope)
