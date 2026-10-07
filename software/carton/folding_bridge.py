"""Offline bridge approach using observed carton/tool frames and encoder FK."""
import math
import numpy as np
import mujoco
from scipy.optimize import least_squares
from carton.folding_sim import JOINTS
from carton.folding_paths import JointPathPlanner,execute_path
from carton.folding_grasp import panel_grasp_evidence

def fold_right_partial(sim,c,*,capture=False,brace_flap='long_near',contact_x=.060,axis_yaw_degrees=0.,axis_mode='rise',normal_offset=False,along_travel=.085,blend_yaw=False,end_degrees=75.):
    if brace_flap not in ('long_near','short_left'):
        raise ValueError('Declare a supported bracing flap')
    reading=c.sense('Register paddle for raised minor support')
    if contact_x not in (.060,.208):
        raise ValueError('Declare handle or blade-end contact')
    if not math.isfinite(axis_yaw_degrees) or not -60<=axis_yaw_degrees<=60:
        raise ValueError('Declared bridge axis yaw must be within 60 degrees')
    if axis_mode not in ('rise','flat','descend'):
        raise ValueError('Declare rising, horizontal or descending blade approach')
    if not math.isfinite(along_travel) or not 0<=along_travel<=.10:
        raise ValueError('Declare along-flap travel between 0 and 100 mm')
    if not math.isfinite(end_degrees) or not 50<=end_degrees<=90:
        raise ValueError('Declare a partial fold endpoint between 50 and 90 degrees')
    registration=sim.register_observed_tcp(reading.get('paddle'),[contact_x,0,.003],
        observation_sequence=reading['seq'],current_sequence=c.port.readings[-1]['seq'])
    relative=np.asarray(registration['rotation']);ix=sim.arm_indices['right'][:5]
    ranges=sim.model.jnt_range[[sim.model.joint('right_'+j).id for j in JOINTS[:5]]]
    kin=mujoco.MjData(sim.model)
    def check_brace():
        evidence=panel_grasp_evidence(sim.model,sim.data,'left',brace_flap)
        if not evidence['opposing_faces']:raise ValueError('Lost opposing-face '+brace_flap+' brace during bridge approach')
        return evidence
    def solve(theta,raise_by=0.):
        kin.qpos[:]=sim.data.qpos
        frame=c.box.copy();R=frame[:3,:3];O=frame[:3,3]
        # Start with the blade rising above the raised edge; progressively
        # rotate it into the horizontal crossbar as the flap descends.
        angle=math.pi/2 if axis_mode=='flat' else min(math.pi/2,theta+math.radians(15))
        yaw=math.radians(axis_yaw_degrees)
        if blend_yaw:
            yaw*=float(np.clip((math.degrees(theta)-50)/25,0,1))
        z_sign=-1 if axis_mode=='descend' else 1
        desired_axis=R@np.array([-math.sin(angle)*math.cos(yaw),math.sin(angle)*math.sin(yaw),z_sign*math.cos(angle)])
        def details(q):
            kin.qpos[ix]=q;mujoco.mj_kinematics(sim.model,kin)
            tool=kin.body('right_gripper_link').xmat.reshape(3,3)@relative
            extent=.020*abs(tool[:,1]@R[:,2])+.003*abs(tool[:,2]@R[:,2])
            along=-.10+along_travel*float(np.clip(math.sin(theta)**2/math.sin(math.radians(75))**2,0,1))
            clearance=.0015+(raise_by/.015)*.010
            point=np.array([.379/2-.140*math.sin(theta)+clearance*math.cos(theta),along,
                .108+.140*math.cos(theta)+clearance*math.sin(theta)+extent-.0007+raise_by])
            if normal_offset:
                if contact_x!=.208:
                    raise ValueError('Surface-normal offset is defined for blade-end contact')
                normal=R@np.array([math.cos(theta),0,math.sin(theta)])
                # Project the blade-end cross-section onto the actual panel
                # normal. A vertical width offset is wrong on a tilted flap.
                extent=.020*abs(tool[:,1]@normal)+.003*abs(tool[:,2]@normal)
                extent+=max(0.,-.002*float(tool[:,0]@normal))
                stand_off=clearance+extent-.0007
                point=np.array([.379/2-.140*math.sin(theta)+stand_off*math.cos(theta),along,
                    .108+.140*math.cos(theta)+stand_off*math.sin(theta)+raise_by])
            return kin.site(sim.control_sites['right']).xpos.copy(),O+R@point,tool[:,0]
        def objective(q):
            actual,target,axis=details(q)
            return np.r_[actual-target,.15*(axis-desired_axis)]
        seeds=[sim.data.qpos[ix],np.array([-.42,.71,-1.01,1.17,.07])]
        solutions=[least_squares(objective,np.clip(seed,ranges[:,0]+1e-6,ranges[:,1]-1e-6),bounds=(ranges[:,0],ranges[:,1]),max_nfev=300,ftol=1e-11,xtol=1e-11,gtol=1e-11) for seed in seeds]
        sol=min(solutions,key=lambda v:np.linalg.norm(objective(v.x)))
        actual,target,axis=details(sol.x)
        error=np.linalg.norm(actual-target);angle=math.degrees(math.acos(np.clip(axis@desired_axis,-1,1)))
        if error>.008 or angle>5:raise ValueError(f'Bridge IK failed: {error*1000:.2f} mm, axis {angle:.2f} deg')
        return sol.x,target,{'position_error_mm':error*1000,'axis_error_degrees':angle}
    def move(theta,height,seconds,label):
        q,target,check=solve(theta,height)
        event=sim.move({},seconds,label,capture=capture,joint_targets={'right':q})
        if event.get('step_error'):raise ValueError(event['step_error'])
        if event['bad_penetration_mm']>1:raise ValueError('Forbidden contact during bridge motion')
        if event['max_joint_tracking_error_radians']>.08:raise ValueError('Bridge joint tracking exceeds 0.08 rad')
        check['actual_tcp_error_mm']=float(np.linalg.norm(sim.actual_control_position('right')-target)*1000)
        if check['actual_tcp_error_mm']>35:raise ValueError('Bridge TCP tracking exceeds 35 mm')
        check['brace']=check_brace()
        return check
    observed=reading['angles'].get('short_right')
    if observed is None:raise ValueError('Fresh right minor required')
    start=math.radians(observed['degrees']);q,_,_=solve(start,.015)
    planner=JointPathPlanner(sim,'right',clearance=.006)
    path=planner.plan(q);execute_path(sim,'right',path,'Approach above right minor with crossbar',capture=capture)
    check_brace()
    checks=[]
    for h in np.linspace(.0135,0,10):checks.append(move(start,h,.2,'Lower crossbar onto right minor'))
    for i,t in enumerate(np.linspace(start,math.radians(end_degrees),41)):
        checks.append(move(t,0,.3,f'Fold right minor toward raised bridge {math.degrees(t):.1f}'))
        if i%5==0:c.sense('Observe raised right minor fold')
    c.port.move_arms({},1.,'Hold right minor with crossbar',None)
    reading=c.sense('Verify partial right minor')
    return {'registration':registration,'checks':checks,'visual_angles':reading['angles'],
            'angles':sim.truth_angles(),'motion':dict(sim.motion_stats),'full_task_complete':False}
