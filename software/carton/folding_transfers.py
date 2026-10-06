"""Offline hand transfers for the resistant, freely moving carton.

These are partial experiments. Completing a commanded transfer is not proof
that its previous flap stays folded. Observe again after every release.
"""
import math
import numpy as np
from carton.folding_diagonal import contact_point
from carton.folding_paths import JointPathPlanner,execute_path


def fold_second_short(sim,controller,*,capture=False,clearance=.006,
                      retreat_box=(0,-.04,.025),brace_label='near flap'):
    """Release the declared left brace, keeping the right minor held.

    Joint-path obstacle snapshots are simulator diagnostics. Contact-point
    commands use fresh rendered RGB-D registration; there is no hardware port.
    """
    port=controller.port;c=controller
    retreat=np.asarray(retreat_box,dtype=float)
    if retreat.shape!=(3,) or not np.isfinite(retreat).all() or np.linalg.norm(retreat)>.15:
        raise ValueError('Declare a finite retreat within 150 mm')
    c.sense('Before releasing '+brace_label)
    port.set_grippers({'left':.6},.5,'Release left '+brace_label+' pinch')
    actual=sim.data.site('left_tip').xpos.copy()
    direction=sim.data.body('left_gripper_link').xmat.reshape(3,3)[:,2].copy()
    port.move_arms({'left':actual+c.box[:3,:3]@retreat},.8,
                   'Withdraw left fingers clear of '+brace_label,{'direction':direction.tolist()})
    reading=c.sense('Locate left short flap while right holds the other short flap')
    measured=reading['angles'].get('short_left')
    if measured is None:raise ValueError('Fresh left short-flap observation required')
    start=math.radians(measured['degrees'])
    for i,theta in enumerate(np.linspace(start,math.pi/2,41)):
        point,orientation=contact_point(theta,0,-1,-.10,.095,0,.010)
        if i==0:
            port.set_grippers({'left':-.17},.5,'Close left claw clear of carton')
            outside=point+[-.04,0,.02]
            q,error=sim.ik('left',c.box[:3,:3]@outside+c.box[:3,3],
                           {'direction':(c.box[:3,:3]@orientation['direction']).tolist()})
            if error>.008:raise ValueError(f'Left short approach IK misses {error*1000:.2f} mm')
            # This is the intended pressing surface, including the initial
            # withdrawal from it. Retain the runtime 1 mm penetration bound.
            planner=JointPathPlanner(sim,'left',clearance=clearance,
                                     allowed_flaps=('short_left_cardboard',))
            path=planner.plan(q)
            execute_path(sim,'left',path,'Reach outside left short flap',capture=capture)
            for u in np.linspace(.1,1,10):
                c.move({'left':(1-u)*outside+u*point},.15,
                       'Approach left short while right holds',orientation)
        c.move({'left':point},.3,f'Fold left short flap {math.degrees(theta):.1f}',orientation)
        if i%5==0:c.sense('Observe both short flaps')
    port.move_arms({},2.,'Hold both short flaps',None)
    reading=c.sense('Verify both short flaps held')
    c.require_folded(reading,['short_left','short_right'])
    return {'angles':sim.truth_angles(),'motion':dict(sim.motion_stats),
            'visual_angles':reading['angles'],'held_only':True,'full_task_complete':False}


def release_left_minor(sim,controller,*,seconds=2.):
    """Negative control: withdraw one hand, then actually observe spring-back."""
    port=controller.port
    if not math.isfinite(seconds) or seconds<=0:
        raise ValueError('Positive finite passive observation duration required')
    tip=sim.data.site('left_tip').xpos.copy()
    port.move_arms({'left':tip+[0,0,.065]},.5,'Release left minor to test spring-back','down')
    port.move_arms({},seconds,f'Observe released flap for {seconds:g} seconds',None)
    reading=controller.sense('Measure released minor; do not assume retention')
    return {'angles':sim.truth_angles(),'motion':dict(sim.motion_stats),
            'visual_angles':reading['angles'],'passive_observation_seconds':seconds,
            'full_task_complete':False}
