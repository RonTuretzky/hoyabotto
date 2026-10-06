"""Partial near-first folding diagnostic; never claims whole-carton success.

Uses an empty, freely moving carton with resistant hinges and the original
rear-cart station/actuator limits. Short flaps start upright and long flaps
5.73 degrees inward in this declared diagnostic. These poses are checked for
panel intersections before dynamics. No hardware adapter.

Transit planning uses a simulator obstacle snapshot; carton tracking and flap
measurements use rendered tags and aligned depth. The independent evaluator
records true angles separately. Both hands still need a reliable retention
strategy before this can become a four-flap closure controller.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
import mujoco
from carton.folding_sim import FoldingSimulation
from carton.folding_paddle import PaddleFoldingSimulation,GRIP_ROTATION
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_paths import JointPathPlanner,execute_path
from carton.folding_diagonal import DiagonalFoldingController,contact_point
from tools.simulate_bimanual_folding import PixelPort

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--simulation-root',required=True);p.add_argument('--video',action='store_true');p.add_argument('--short-along',type=float,default=-.08);p.add_argument('--short-radius',type=float,default=.10);p.add_argument('--clearance',type=float,default=.012);p.add_argument('--normal',choices=['face','up','tilt'],default='face');p.add_argument('--end-angle',type=float,default=90);p.add_argument('--track-hold',action='store_true');p.add_argument('--tool',choices=['claws','paddle'],default='claws')
    a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    s=(PaddleFoldingSimulation if a.tool=='paddle' else FoldingSimulation)(Path(a.simulation_root),out,station=FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),backup_table_marker_xy=(.45,.70)),material=CartonMaterial(),width=960,height=540,offset=(0,.0757925946355),yaw=math.pi/6,initial_right_roll=1.5,initial_flaps=dict(short_left=0.,short_right=0.,long_far=.1,long_near=.1))
    port=PixelPort(s,record=a.video);c=DiagonalFoldingController(port,rear_cart=True)
    r={'short_initial_angle_degrees':0,'stage_only':'near then short; empty resistant free carton','task_complete':False,'stages':[]}
    held={};held_ori={}
    def move(points,seconds,label,ori):
     if a.track_hold:
      held.update(points);held_ori.update({side:ori for side in points})
      return c.move(held,seconds,label,held_ori)
     return c.move(points,seconds,label,ori)
    def fold(side,flap,axis,sign,along,radius,clearance,normal='face'):
     reading=c.sense('Locate '+flap);start=math.radians(np.clip(reading['angles'][flap]['degrees']-5,-30,0))
     for i,theta in enumerate(np.linspace(start,math.radians(a.end_angle),41)):
      point,ori=contact_point(theta,axis,sign,along,radius,0,clearance)
      n=np.zeros(3);n[axis]=sign*math.cos(theta);n[2]=math.sin(theta)
      if normal=='face':ori={'direction':n.tolist()}
      elif normal=='tilt':ori={'direction':(np.array([-1.5*(1-theta/(math.pi/2)),0,1])/np.linalg.norm([-1.5*(1-theta/(math.pi/2)),0,1])).tolist()}
      if side=='right' and a.tool=='paddle':
       point,_=contact_point(theta,axis,sign,along,radius,0,.0045)
       ori={'direction':n.tolist(),'local_axis':GRIP_ROTATION[:,2].tolist()}
      if i==0:
       outside=point+.045*n;world=c.box[:3,:3]@outside+c.box[:3,3];direction=c.box[:3,:3]@ori['direction']
       world_ori={**ori,'direction':direction.tolist()};q,err=s.ik(side,world,world_ori)
       if err>.008:raise ValueError(f'Approach IK {flap} misses by {err*1000:.1f} mm')
       planner=JointPathPlanner(s,side);path=planner.plan(q);r['stages'].append({'flap':flap,'path_checks':planner.checks})
       execute_path(s,side,path,'Reach outside '+flap,capture=a.video)
       for u in np.linspace(.1,1,10):move({side:(1-u)*outside+u*point},.15,'Approach '+flap,ori)
      move({side:point},.3,f'Fold {flap} {math.degrees(theta):.1f}',ori)
      if i%5==0:c.sense('Observe '+flap)
     port.move_arms({},.6,'Hold '+flap,None)
     r['stages'][-1]['final_angles']=s.truth_angles();s.capture('Held '+flap)
    try:
     s.capture('Initial empty resistant carton; short flaps upright');s.move({},.4,'Settle',capture=False)
     c.sense('Register carton');port.set_grippers({'left':-.17},.3,'Set folding jaw')
     fold('left','long_near',1,-1,0,.11,.008)
     fold('right','short_right',0,1,a.short_along,a.short_radius,a.clearance,a.normal)
     port.move_arms({},2.,'Hold two flaps down',None)
    except Exception as exc:r['error']=str(exc)
    s.capture('End of partial sequence')
    r['angles']=s.truth_angles();r['motion']=s.motion_stats;r['time']=float(s.data.time);r['physics']=s.save('folding');r['success']=False;r['simulation_only']=True;r['controller']={'error':r.get('error','Partial sequence only; four-flap closure and release untested')};r['assumptions']={'material':s.material.report(),'short_initial_angle_degrees':0,'station_dimensions_measured':False,'end_target_degrees':a.end_angle,'track_hold':a.track_hold,'tool':a.tool};r['source_hashes']={str(path.relative_to(Path(__file__).resolve().parents[1])):hashlib.sha256(path.read_bytes()).hexdigest() for path in [Path(__file__).resolve(),*sorted((Path(__file__).resolve().parents[1]/'carton').glob('folding*.py')),Path(__file__).resolve().with_name('simulate_bimanual_folding.py')]};(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='physics'},indent=2))


if __name__=="__main__":main()
