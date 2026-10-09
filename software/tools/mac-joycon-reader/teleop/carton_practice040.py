"""Joy-Con carton practice: geometric pad capture + benchmark crease relaxation.

Uses the existing virtual-only input transport. The maker robot is kinematic;
pinches/slips are geometry proxies, not MuJoCo contact or measured servo forces.
"""
import math
import time
import mujoco
import numpy as np
from model040_preview import Model040PreviewBus
from practice040 import Practice040
from carton_practice_scene import CENTER,FLAPS
from farm.sim.box_scene import PLASTIC,REAL_FLAP


class FoldingPractice:
    def __init__(self,target='right'):
        if target not in FLAPS:raise ValueError('Unknown practice flap')
        self.target=target
        # Latest real benchmark presentation; other flaps remain manipulable.
        angles={'right':-11.,'left':-60.,'near':90.,'far':17.}
        angles[target]=-11. if target in ('left','right') else 0.
        self.flaps={n:dict(angle=a,rest=a,held_by=None,radius=.12,along=0.,yield_s=0.,hold_s=0.) for n,a in angles.items()}
        self.previous={'left':45.,'right':45.};self.note='Open the jaw and aim at the white edge bead.'
        self.elapsed=0.;self.released_at=None;self.verified=False

    def hinge(self,name):return np.asarray(CENTER)+FLAPS[name][0]
    def point(self,name,*,angle=None,along=0.,radius=.12):
        a=math.radians(self.flaps[name]['angle'] if angle is None else angle)
        return self.hinge(name)+np.asarray(FLAPS[name][1])*radius*math.sin(a)+np.asarray(FLAPS[name][2])*along+np.array([0.,0.,radius*math.cos(a)])
    def distance(self,name,tip):
        line=np.asarray(FLAPS[name][2]);along=float(np.dot(tip-self.hinge(name),line))
        along=float(np.clip(along,-FLAPS[name][3]/2+.02,FLAPS[name][3]/2-.02))
        return float(np.linalg.norm(tip-self.point(name,along=along)))
    def release(self,name,reason):
        f=self.flaps[name];f['held_by']=None;f['yield_s']=0.;self.note=reason
        if name==self.target:self.released_at=self.elapsed;self.verified=False
    def update(self,dt,tips,grips,rolls):
        dt=max(0.,min(.05,dt));self.elapsed+=dt
        for side in ('left','right'):
            grip=grips[side]
            if self.previous[side]>15 and grip<=15:
                candidates=[n for n,f in self.flaps.items() if not f['held_by']]
                if not any(f['held_by']==side for f in self.flaps.values()) and candidates:
                    n=min(candidates,key=lambda n:self.distance(n,tips[side]));f=self.flaps[n]
                    # Side flaps need unrolled pads; majors need a quarter-turn.
                    r=abs((rolls[side]+90)%180-90)
                    aligned=r<35 if n in ('left','right') else r>55
                    if self.distance(n,tips[side])<.022 and aligned:
                        f.update(held_by=side,along=float(np.dot(tips[side]-self.hinge(n),FLAPS[n][2])),yield_s=0.,hold_s=0.)
                        self.note=f'{n.title()} flap captured by {side} pads in practice. Follow the dotted arc.'
                        if n==self.target:self.released_at=None;self.verified=False
                    elif not aligned and self.distance(n,tips[side])<.04:self.note='Rotate the wrist so the jaws straddle the flap edge; reopen before retrying.'
                    else:self.note='Closed on air. Reopen, approach the white bead, and try again.'
            self.previous[side]=grip
        for n,f in self.flaps.items():
            side=f['held_by']
            if side:
                v=tips[side]-self.hinge(n);inward=np.asarray(FLAPS[n][1]);line=np.asarray(FLAPS[n][2])
                radial=math.hypot(float(np.dot(v,inward)),float(v[2]))
                along=float(np.dot(v,line))
                if grips[side]>=55:self.release(n,'Jaw opened. Watch whether the flap springs back.')
                elif abs(radial-f['radius'])>.035 or abs(along-f['along'])>.045:
                    self.release(n,'Flap slipped: keep the hand on the hinge arc, rather than lifting the carton.')
                else:
                    f['angle']=float(np.clip(math.degrees(math.atan2(float(np.dot(v,inward)),float(v[2]))),-60,125))
                    f['hold_s']=f['hold_s']+dt if f['angle']>=100 else 0.
                    f['yield_s']=f['yield_s']+dt if abs(f['angle']-f['rest'])>PLASTIC['yield_deg'] else 0.
                    if f['yield_s']>PLASTIC['delay_s']:
                        fraction=float(np.clip((f['angle']-PLASTIC['start_set_deg'])/(PLASTIC['full_set_deg']-PLASTIC['start_set_deg']),0,1))
                        target=f['angle']-PLASTIC['springback_deg']*(1-fraction)
                        if (target-f['rest'])*(f['angle']-f['rest'])>0:
                            f['rest']+=(target-f['rest'])*(1-math.exp(-PLASTIC['rate_per_s']*dt))
            if not f['held_by']:
                # Quasi-static spring/damping relaxation, not rigid-body contact physics.
                f['angle']+=(f['rest']-f['angle'])*(1-math.exp(-min(REAL_FLAP['stiffness']/REAL_FLAP['damping'],6)*dt))
        target=self.flaps[self.target]
        if self.released_at is not None and self.elapsed-self.released_at>=2:
            self.verified=not target['held_by'] and target['angle']>=75
    def status(self,tips):
        f=self.flaps[self.target];side=f['held_by'] or ('left' if self.target=='left' else 'right')
        return dict(target=self.target,label=FLAPS[self.target][4],hand=side,distance_cm=round(self.distance(self.target,tips[side])*100,1),
                    angle_deg=round(f['angle'],1),hold_s=round(f['hold_s'],1),held_by=f['held_by'],verified=self.verified,
                    flaps={n:dict(angle_deg=round(v['angle'],1),rest_deg=round(v['rest'],1),held_by=v['held_by']) for n,v in self.flaps.items()},note=self.note)


class CartonCourse:
    names=['Wrists','Approach','Pinch','Fold arc','Hold','Release','Check']
    def __init__(self,practice):self.practice=practice;self.active=False;self.complete=False;self.step=0;self.seen=set();self.note='Take the short tutorial, then begin.'
    def begin(self,positions,pickup_height):
        self.active=True;self.complete=False;self.step=0;self.seen=set();self.note='Check both wrists, then approach the selected flap.'
        self.baseline={(s,j):positions.get(s+'_arm_'+j,0.) for s in ('left','right') for j in ('wrist_roll','wrist_flex')}
    def update(self,positions,*unused):
        if not self.active or self.complete:return
        state=self.practice.fold_status()
        if self.step==0:
            for side in ('left','right'):
                if any(abs(positions[side+'_arm_'+j]-self.baseline[(side,j)])>=12 for j in ('wrist_roll','wrist_flex')):self.seen.add(side)
            if len(self.seen)==2 and all(abs(positions[s+'_arm_'+j]-self.baseline[(s,j)])<8 for s in ('left','right') for j in ('wrist_roll','wrist_flex')):self.step=1
        elif self.step==1 and state['distance_cm']<2.2 and positions[state['hand']+'_arm_gripper']>=55:self.step=2
        elif self.step==2 and state['held_by']:self.step=3
        elif self.step==3 and state['held_by'] and state['angle_deg']>=100:self.step=4
        elif self.step==4 and state['held_by'] and state['hold_s']>=10:self.step=5
        elif self.step==5 and not state['held_by']:self.step=6
        elif self.step==6 and state['verified']:self.complete=True
        if 3<=self.step<=4 and not state['held_by']:
            self.step=1;self.note='The flap released early. Reopen and recapture its edge.'
        elif self.step==6 and self.practice.folding.released_at is not None and self.practice.folding.elapsed-self.practice.folding.released_at>=2 and not state['verified']:
            self.step=1;self.note='It sprang back. Recapture and hold farther past flat before releasing.'
    def status(self,*unused):
        s=self.practice.fold_status();label=s['label'];hand=s['hand'];jaw='ZL' if hand=='left' else 'ZR';raise_button='L' if hand=='left' else 'R'
        rows=[
            ('Check both wrists','Tilt each Joy-Con, then bring both wrists back to their starting pose.','Tilt bends and rolls its matching wrist.'),
            ('Approach '+label,f'Open the {hand} jaw and bring its tip to the white bead, 2 cm below the flap edge.',f'{hand.title()} stick moves the hand; {raise_button} raises, stick-click lowers. {jaw} toggles the jaw.'),
            ('Pinch the flap','Close with the edge inside the pads. A missed close does not attach anything.',f'Tap {jaw}. Keep side-flap wrists unrolled; roll 90° for near/far flaps.'),
            ('Carry the fold arc','Move inward and down around the hinge, following the dotted arc to 100–110°.','Keep the jaw closed. Stay about 12 cm from the hinge; pulling away causes a slip.'),
            ('Hold past flat','Keep the captured flap past 100° for ten continuous seconds.','Stay still. The fresh crease relaxes slowly; a brief fold springs back.'),
            ('Release the edge',f'Open the {hand} jaw, then move the hand clear.',f'Tap {jaw}. Do not lift the carton.'),
            ('Check springback','Watch for two seconds after release. The flap must remain at least 75° inward.','If it reopens, recapture and try a longer, deeper hold.'),
        ]
        title,instruction,controls=rows[self.step]
        if self.complete:title,instruction,controls='Fold practice complete','The simulated flap stayed folded after release. Try another flap or repeat.','This is practice feedback, not proof of a physical fold.'
        return dict(active=self.active,step=7 if self.complete else self.step,total=7,complete=self.complete,title=title,instruction=instruction,controls=controls,
                    names=self.names,note=self.note+' '+s['note'],distance_cm=s['distance_cm'],box_held=bool(s['held_by']),wrists_checked=sorted(self.seen))


class CartonPractice040(Practice040):
    def __init__(self,*,render=True):
        self.target='right';self.last_fold_update=time.monotonic()
        super().__init__(bus_factory=lambda:Model040PreviewBus(workshop=True,carton=True,render=render),course_factory=lambda:CartonCourse(self))
    def _configure_box(self):
        self.folding=FoldingPractice(self.target);self.box_position=np.asarray(CENTER).copy();self.held=self.placed=False
        self._apply_positions(self.positions);self._sync_flaps()
    def _tip(self,side='right'):return self.bus.data.site_xpos[self.bus.model.site(side+'_tool_tip').id].copy()
    def _tips(self):return {side:self._tip(side) for side in ('left','right')}
    def _grip_distance(self):
        side='left' if self.target=='left' else 'right'
        return self.folding.distance(self.target,self._tip(side))
    def fold_status(self):return self.folding.status(self._tips())
    def _sync_flaps(self):
        for n,f in self.folding.flaps.items():
            self.bus.data.qpos[self.bus.model.joint('fold_hinge_'+n).qposadr[0]]=math.radians(f['angle'])
            self.bus.model.geom_rgba[self.bus.model.geom('fold_target_'+n).id,3]=1 if n==self.target else 0
            for i in range(12):self.bus.model.geom_rgba[self.bus.model.geom(f'fold_arc_{n}_{i}').id,3]=.65 if n==self.target else 0
        mujoco.mj_forward(self.bus.model,self.bus.data)
    def _update_box(self):
        now=time.monotonic();dt=now-self.last_fold_update;self.last_fold_update=now
        self.folding.update(dt,self._tips(),{s:self.positions[s+'_arm_gripper'] for s in ('left','right')},{s:self.positions[s+'_arm_wrist_roll'] for s in ('left','right')})
        self.held=bool(self.folding.flaps[self.target]['held_by']);self.placed=self.folding.verified;self._sync_flaps()
    def stop(self,reason):
        super().stop(reason)
        if hasattr(self,'folding'):
            for n,f in self.folding.flaps.items():
                if f['held_by']:self.folding.release(n,'Practice stopped; virtual jaw released.')
    def lesson_action(self,action):
        if action.startswith('target:'):
            target=action.split(':',1)[1]
            if target not in FLAPS:raise ValueError('Unknown practice flap')
            with self.lock:self.target=target;super().lesson_action('repeat')
        else:
            super().lesson_action(action)
            if action=='reset_box':self.course.note='Carton reset. Open the jaw and approach the selected edge bead.'
    def status(self):
        s=super().status();s['folding']=self.fold_status()
        s['simulator'].update(carton_practice=True,view=self.bus.view,
            limitations='Geometry-based pinch/slip; benchmark crease springback; no contact/servo-force validation. Station placement and joint mapping are unmeasured.',
            station={'carton_mm':[379,283,108],'flap_mm':140,'table_mm':[500,480,700],'arm_spacing_mm':220,'placement_measured':False})
        return s
