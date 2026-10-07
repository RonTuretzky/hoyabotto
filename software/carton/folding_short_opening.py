"""Joint-driven outward short preparation for offline major-first experiments.

Entry follows the existing verified left-minor pinch and physical near-flap
opening. Targets use rendered RGB-D and robot FK. No carton state is assigned.
"""
from __future__ import annotations
import copy
import math

import numpy as np

from carton.folding_diagonal import contact_point
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_progress import ContactProgressGuard


def _opening_reading(reading, flap):
    """Keep visual pose intact while expressing outward angle as progress."""
    value=copy.deepcopy(reading)
    row=value.get('angles',{}).get(flap)
    if row is not None:
        row['degrees']=-row['degrees']
    return value


def _park(sim, controller, side, *, capture):
    actual=sim.actual_control_position(side).copy()
    axis=sim.data.body(side+'_gripper_link').xmat.reshape(3,3)[:,2].copy()
    controller.port.move_arms({side:actual+[0,0,.10]},.8,
        'Lift '+side+' claw clear of outward short',{'direction':axis.tolist()})
    target=np.array([-.20 if side=='left' else .20,-.18,.30])
    q,error=sim.ik(side,target,None)
    if error>.008:
        raise ValueError(side+' short-opening park IK exceeds 8 mm')
    path=JointPathPlanner(sim,side,clearance=.006).plan(q)
    execute_path(sim,side,path,'Park '+side+' claw after outward short opening',capture=capture)


def open_shorts_before_majors(sim,controller,*,capture=False,target_degrees=-15):
    """Release the initial pinch, then push each short from its inside face.

    The failed orientation-preserving pinch rotation is not retried. The
    release/escape follows the established second-short transfer geometry;
    every new free-space transit retains the original six-millimetre gate.
    """
    if not math.isfinite(target_degrees) or not -35<=target_degrees<=-10:
        raise ValueError('Outward short preparation target must be -35 to -10 degrees')
    c=controller
    report=dict(preparation_only=True,full_task_complete=False,hardware_commands=0,
                target_degrees=target_degrees,method='released inside-face pushes',
                progress_guards_use_negated_visual_opening_angles=True,
                stage='release original left-minor pinch',flaps={})
    c.short_opening=report
    reading=c.sense('Register carton before releasing left pinch for outward preparation')
    left=reading['angles'].get('short_left')
    if left is None or not -35<=left['degrees']<=25:
        raise ValueError('Fresh left-short opening entry angle required')
    release_guard=ContactProgressGuard('short_left',_opening_reading(reading,'short_left'))
    report['release_progress_checks']=release_guard.checks
    c.port.set_grippers({'left':.6},.5,'Release left pinch before inside-face opening')
    actual=sim.actual_control_position('left').copy()
    direction=sim.data.body('left_gripper_link').xmat.reshape(3,3)[:,2].copy()
    c.port.move_arms({'left':actual+c.box[:3,:3]@np.array([-.04,0,.04])},.8,
        'Withdraw left fingers clear of released short',{'direction':direction.tolist()})
    c.port.set_grippers({'left':-.17},.5,'Close left claw clear of released short')
    actual=sim.actual_control_position('left').copy()
    direction=sim.data.body('left_gripper_link').xmat.reshape(3,3)[:,2].copy()
    c.port.move_arms({'left':actual+c.box[:3,:3]@np.array([-.025,0,.015])},.5,
        'Clear left short before inside-face free transit',{'direction':direction.tolist()})
    reading=c.sense('Verify carton stayed in place during left-pinch release')
    release_guard.check(_opening_reading(reading,'short_left'),-left['degrees'])

    for side,sign in (('left',-1),('right',1)):
        flap='short_'+side
        report['stage']='push '+flap+' outward from inside face'
        reading=c.sense('Register '+flap+' before inside-face outward push')
        observed=reading['angles'].get(flap)
        if observed is None or not -35<=observed['degrees']<=25:
            raise ValueError('Fresh '+flap+' opening angle required')
        start=math.radians(observed['degrees'])
        guard=ContactProgressGuard(flap,_opening_reading(reading,flap))
        report['flaps'][flap]=dict(progress_checks=guard.checks)
        c.port.set_grippers({side:-.17},.4,'Close '+side+' claw before inside-face push')
        reached=False
        for index,theta in enumerate(np.linspace(start,math.radians(target_degrees),31)):
            point,_=contact_point(theta,0,sign,-.10,.115,0,-.012)
            if index==0:
                pre=point+np.array([-sign*.035,0,.060])
                q,error=sim.ik(side,c.box[:3,:3]@pre+c.box[:3,3],'down')
                if error>.008:
                    raise ValueError(f'Inside {flap} approach IK exceeds 8 mm: {error*1000:.2f}')
                path=JointPathPlanner(sim,side,clearance=.006).plan(q)
                execute_path(sim,side,path,'Reach above inside '+flap+' face',capture=capture)
                reading=c.sense('Verify carton before inside '+flap+' approach')
                guard.check(_opening_reading(reading,flap),-math.degrees(theta))
                for u in np.linspace(.1,1,10):
                    c.move({side:(1-u)*pre+u*point},.15,'Approach inside '+flap,'down')
                    reading=c.sense('Observe '+flap+' during inside-face approach')
                    guard.check(_opening_reading(reading,flap),-math.degrees(theta))
                    if reading['angles'][flap]['degrees']<=target_degrees+.5:
                        reached=True
                        break
            if reached:break
            c.move({side:point},.20,f'Physically push {flap} outward {math.degrees(theta):.1f}','down')
            reading=c.sense('Observe physical '+flap+' outward progress')
            guard.check(_opening_reading(reading,flap),-math.degrees(theta))
            if reading['angles'][flap]['degrees']<=target_degrees+.5:
                break
        _park(sim,c,side,capture=capture)
        c.port.move_arms({},.7,'Observe released '+flap+' opening',None)
        reading=c.sense('Verify '+flap+' remains outward after release')
        observed=reading['angles'].get(flap)
        if observed is None or abs(observed['degrees']-target_degrees)>5:
            raise ValueError(f'{flap} outward preparation not retained after release')
        report['flaps'][flap]['released_visual']=observed
    c.port.move_arms({},1.,'Observe both outward shorts with hands clear',None)
    reading=c.sense('Verify both short flaps remain outward before major-first test')
    for flap in ('short_left','short_right'):
        observed=reading['angles'].get(flap)
        if observed is None or abs(observed['degrees']-target_degrees)>5:
            raise ValueError(f'{flap} outward preparation not retained with hands clear')
    report.update(stage='both shorts outward and released',visual_angles=reading['angles'],
                  independent_final_angles=sim.truth_angles(),motion=dict(sim.motion_stats),
                  physically_opened_and_released=True)
    return report
