"""Partial braced-fold diagnostic for an empty, resistant, freely moving carton.

This stops on missing fresh registration, loss of opposing jaw contact, slip,
IK/tracking failure or collision. It always reports whole-carton success false.
It uses only the simulated robot. Planner obstacle snapshots and contact-based
pinch verification are independent simulator diagnostics, not hardware sensing.
The external camera and side markers are proposed, unmeasured station changes.
"""
import argparse,json,math
from pathlib import Path
import numpy as np,mujoco
from carton.folding_sim import FoldingSimulation,JOINTS
from carton.folding_solver import FoldingSolver
from carton.folding_paddle import PaddleFoldingSimulation,GRIP_ROTATION
from scipy.optimize import least_squares
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_paths import JointPathPlanner,execute_path
from carton.folding_diagonal import DiagonalFoldingController,contact_point
from tools.simulate_bimanual_folding import PixelPort
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--out',required=True)
p.add_argument('--simulation-root',required=True)
p.add_argument('--video',action='store_true')
p.add_argument('--tool',choices=['claws','paddle'],default='claws')
p.add_argument('--solver',choices=['legacy','friction'],default='friction')
p.add_argument('--fold-second-short',action='store_true',help='Test both minor flaps instead of the failed near-flap pivot')
p.add_argument('--release-left-minor',action='store_true',help='After the two-minor hold, measure spring-back on release')
p.set_defaults(along=-.08,radius=.125,axis_sign=1,pre_height=.035,end_angle=-30,
               clearance=-.002,camera='station',continue_short=True,
               freeze_hold=True,close_near=True,close_mode='pivot')
a=p.parse_args()
if a.release_left_minor and not a.fold_second_short:p.error('--release-left-minor requires --fold-second-short')
out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
class GraspIKMixin:
 """Prioritize preserving the pinched face normal without relaxing IK limits.

 A weak orientation objective allowed the box to turn instead of the wrist.
 This diagnostic deliberately stops when that grasp-preserving pose cannot
 be reached by the five-axis arm. Motor forces and travel stay unchanged.
 """
 def ik(self,side,target,orientation=None):
  if isinstance(orientation,dict) and 'local_axis' in orientation and side=='left':
   from scipy.optimize import least_squares
   ix=self.arm_indices[side][:5];ranges=self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
   for indices in self.arm_indices.values():self.kin.qpos[indices]=self.data.qpos[indices]
   def normal_objective(q):
    self.kin.qpos[ix]=q;mujoco.mj_kinematics(self.model,self.kin);rot=self.kin.body(side+'_gripper_link').xmat.reshape(3,3)
    return np.r_[self.kin.site(self.control_sites[side]).xpos-target,(rot@orientation['local_axis']-orientation['direction'])*.2]
   sol=least_squares(normal_objective,np.clip(self.seeds[side],ranges[:,0]+1e-6,ranges[:,1]-1e-6),bounds=(ranges[:,0],ranges[:,1]),max_nfev=200,ftol=1e-10,xtol=1e-10,gtol=1e-10)
   self.seeds[side]=sol.x.copy();return sol.x,float(np.linalg.norm(normal_objective(sol.x)[:3]))
  return super().ik(side,target,orientation)

class PinchSimulation(GraspIKMixin,FoldingSimulation):
 pass

class PinchPaddleSimulation(GraspIKMixin,PaddleFoldingSimulation):
 pass

s=(PinchPaddleSimulation if a.tool=='paddle' else PinchSimulation)(Path(a.simulation_root),out,station=FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),backup_table_marker_xy=(.45,.70)),material=CartonMaterial(),width=960,height=540,offset=(0,.0757925946355),yaw=math.pi/6,initial_right_roll=1.5,solver=FoldingSolver.friction() if a.solver=='friction' else FoldingSolver())
port=PixelPort(s,record=a.video,camera=a.camera);c=DiagonalFoldingController(port,rear_cart=True);r={'simulation_only':True,'stage_only':'Pinch and fold near flap','task_complete':False,'camera':a.camera,'grasp_checks':[]}
def pose(theta):
 point,_=contact_point(theta,1,-1,a.along,a.radius,0,a.clearance)
 normal=np.array([0,-math.cos(theta),math.sin(theta)])
 radial=np.array([0,math.sin(theta),math.cos(theta)])
 return point,{'direction':radial.tolist(),'tangent':(a.axis_sign*normal).tolist()}
def grasp_contact():
 from carton.folding_grasp import panel_grasp_evidence
 return panel_grasp_evidence(s.model,s.data,'left','long_near')

try:
 s.capture('Initial free resistant carton');s.move({},.4,'Settle',capture=False);rd=c.sense('Register carton');port.set_grippers({'left':.6},.4,'Open gripper')
 start=math.radians(rd['angles']['long_near']['degrees']);point,ori=pose(start);pre=point+[0,0,a.pre_height]
 world=c.box[:3,:3]@pre+c.box[:3,3];o={k:(c.box[:3,:3]@v).tolist() for k,v in ori.items()};q,err=s.ik('left',world,o)
 if err>.008:raise ValueError(f'Pregrasp IK misses {err*1000:.2f} mm')
 planner=JointPathPlanner(s,'left');path=planner.plan(q);r['path_checks']=planner.checks;execute_path(s,'left',path,'Approach over near flap',capture=a.video)
 for u in np.linspace(.05,1,20):c.move({'left':(1-u)*pre+u*point},.15,'Insert open gripper from above',ori)
 port.set_grippers({'left':-.17},.6,'Pinch near flap');port.move_arms({},.5,'Verify pinch hold',None)
 check=grasp_contact();r['grasp_checks'].append(check)
 if not check['opposing_faces']:raise ValueError('Both jaws do not contact the intended near flap')
 grasp_frame=c.box.copy()
 for i,theta in enumerate(np.linspace(start,math.radians(a.end_angle),41)):
  point,ori=pose(theta)
  port.move_arms({'left':grasp_frame[:3,:3]@point+grasp_frame[:3,3]},.3,f'Pinch fold near flap {math.degrees(theta):.1f}',{k:(grasp_frame[:3,:3]@v).tolist() for k,v in ori.items()})
  check=grasp_contact();r['grasp_checks'].append(check)
  if not check['opposing_faces']:raise ValueError('Lost two-sided near-flap grasp')
  if i%5==0:c.sense('Observe pinched fold')
 port.move_arms({},2.,'Hold near flap in pinch',None)
 r['opening_completed']={'angles':s.truth_angles(),'motion':dict(s.motion_stats)}
 if a.continue_short:
  c.sense('Locate right short flap while left braces box')
  hold_point,hold_ori=pose(math.radians(a.end_angle))
  held_world=grasp_frame[:3,:3]@hold_point+grasp_frame[:3,3];held_ori_world={k:grasp_frame[:3,:3]@v for k,v in hold_ori.items()}
  for i,theta in enumerate(np.linspace(0,math.pi/2,41)):
   if a.freeze_hold:
    hold_point=c.box[:3,:3].T@(held_world-c.box[:3,3]);hold_ori={k:(c.box[:3,:3].T@v).tolist() for k,v in held_ori_world.items()}
   radius=.14-.04*max(0,(theta-math.pi/4)/(math.pi/4))
   point,ori=contact_point(theta,0,1,-.11,radius,1.5,.012)
   if a.tool=='paddle':
    point,_=contact_point(theta,0,1,-.11,radius,1.5,.0045)
    ori={'direction':[math.cos(theta),0,math.sin(theta)],'local_axis':GRIP_ROTATION[:,2].tolist()}
   if i==0:
    outside=point+[.045,0,0];q,err=s.ik('right',c.box[:3,:3]@outside+c.box[:3,3],{**ori,'direction':(c.box[:3,:3]@ori['direction']).tolist()})
    if err>.008:raise ValueError(f'Right short approach IK misses {err*1000:.2f} mm')
    planner=JointPathPlanner(s,'right');path=planner.plan(q);execute_path(s,'right',path,'Reach outside right short flap',capture=a.video)
    for u in np.linspace(.1,1,10):c.move({'left':hold_point,'right':(1-u)*outside+u*point},.15,'Approach right short flap while left braces',{'left':hold_ori,'right':ori})
   c.move({'left':hold_point,'right':point},.3,f'Fold right short flap {math.degrees(theta):.1f}',{'left':hold_ori,'right':ori})
   check=grasp_contact();r['grasp_checks'].append(check)
   if not check['opposing_faces']:raise ValueError('Lost opposing-face brace while folding right short flap')
   if i%5==0:c.sense('Observe right short folding and box position')
  port.move_arms({},2.,'Hold right short flap and open near flap',None)
  r['short_completed']={'angles':s.truth_angles(),'motion':dict(s.motion_stats)}
  if a.fold_second_short:
   from carton.folding_transfers import fold_second_short,release_left_minor
   r['stage_only']='Two short flaps and optional release test'
   r['two_shorts_completed']=fold_second_short(s,c,capture=a.video)
   if a.release_left_minor:r['released_left_minor']=release_left_minor(s,c)
  if a.close_near and not a.fold_second_short:
   close_reading=c.sense('Register current near-flap hinge for grasp-preserving close')
   near=close_reading['angles'].get('long_near')
   if near is None:raise ValueError('Fresh near-flap observation required before closing transition')
   close_start=math.radians(near['degrees'])
   r['closing_angle_source']={'seq':close_reading['seq'],'degrees':near['degrees'],'cached_visual_estimate':False}
   hinge=c.box[:3,:3]@np.array([0,-.283/2,.1115])+c.box[:3,3]
   axis=c.box[:3,:3]@np.array([-1,0,0])
   initial_tip=s.data.site('left_tip').xpos.copy();initial_axis=s.data.body('left_gripper_link').xmat.reshape(3,3)[:,0].copy()
   for i,theta in enumerate(np.linspace(close_start,math.pi/2,61)):
    from scipy.spatial.transform import Rotation
    rotation=Rotation.from_rotvec(axis*(theta-close_start)).as_matrix()
    world=hinge+rotation@(initial_tip-hinge)
    port.move_arms({'left':world},.3,f'Pivot close near flap {math.degrees(theta):.1f}',{'direction':(rotation@initial_axis).tolist(),'local_axis':[1,0,0]})
    check=grasp_contact();r['grasp_checks'].append(check)
    if not check['opposing_faces']:raise ValueError('Lost opposing-face pinch while closing near flap')
    if i%5==0:c.sense('Observe near flap closing over right short')
   port.move_arms({},2.,'Hold near flap closed over right short',None)
   r['near_completed']={'angles':s.truth_angles(),'motion':dict(s.motion_stats)}


except Exception as exc:r['error']=str(exc)
s.capture('End of pinch diagnostic');r['angles']=s.truth_angles();r['motion']=s.motion_stats;r['time']=float(s.data.time);r['readings']=port.readings;r['physics']=s.save('folding');r['success']=False;r['controller']={'error':r.get('error','Partial sequence; full closure and release untested')};r['assumptions']={'material':s.material.report(),'short_initial_angle_degrees':math.degrees(.1),'station_dimensions_measured':False,'camera':'Proposed external camera; physical mount unverified','solver':a.solver};r['configuration']=vars(a);r['source_sha256']={str(p.relative_to(Path(__file__).resolve().parents[1])):__import__('hashlib').sha256(p.read_bytes()).hexdigest() for p in [Path(__file__).resolve(),*sorted((Path(__file__).resolve().parents[1]/'carton').glob('folding*.py')),Path(__file__).resolve().with_name('simulate_bimanual_folding.py')]};(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k not in ('physics','grasp_checks','readings')},indent=2))
