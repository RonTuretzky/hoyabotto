"""Rendered tag/depth -> paired robot joints -> passive carton contact, offline."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import mujoco

from carton.folding_sim import FoldingSimulation,FLAPS,TAG_IDS,W,H
from carton.folding_vision import RGBDTagObserver,depth_flap_angles
from carton.folding_controller import FoldingController

class PixelPort:
    def __init__(self,sim,*,seed=0,noise=.0008,dropout=.25,fault=None,record=True):
        self.sim=sim;self.rng=np.random.default_rng(seed);self.noise=noise;self.dropout=dropout;self.fault=fault
        anchor=np.eye(4);anchor[:3,:3]=np.diag([-1,1,-1]);anchor[:3,3]=[0,-.35,.0013]
        self.observer=RGBDTagObserver(anchor);self.seq=0;self.readings=[];self.record=record
        self.box_from_tag=np.eye(4)
        self.box_from_tag[:3,:3]=[[-1,0,0],[0,0,1],[0,1,0]]
        self.box_from_tag[:3,3]=[0,-W/2-.0021,H/2]
        self.box=None;self.angle_priors={}
        self.arm_tag_checks=[]

    def observe(self,label):
        rgb=self.sim.render('front');depth=self.sim.render('front',True)
        depth+=self.rng.normal(0,self.noise,depth.shape)
        depth[self.rng.random(depth.shape)<self.dropout]=0
        if self.fault=='missing_depth':depth[:]=0
        if self.fault=='missing_tags':rgb[:]=0
        h,w=depth.shape;f=h/(2*math.tan(math.radians(48)/2))
        k=np.array([[f,0,w/2],[0,f,h/2],[0,0,1]])
        self.seq+=1;t=float(self.sim.data.time)
        tags=self.observer.observe(rgb,depth,k,seq=self.seq,timestamp=t,depth_timestamp=t)
        if self.seq==1 and not {2,4}.issubset(tags):raise ValueError('Both housing tags required for initial arm registration check')
        for side,tag_id in [('left',4),('right',2)]:
            if tag_id in tags:
                expected=self.sim.arm_tag_fk(side)[:3,3].copy()
                if self.fault=='bad_gripper_calibration' and side=='right':expected[0]+=.025
                error=float(np.linalg.norm(tags[tag_id][:3,3]-expected))
                self.arm_tag_checks.append({'seq':self.seq,'arm':side,'tag_id':tag_id,'encoder_fk_error_mm':error*1000})
                if error>.012:raise ValueError('Gripper tag disagrees with calibrated encoder FK by over 12 mm')
        if 10 not in tags:raise ValueError('Fresh carton ID 10 and aligned depth required')
        self.box=tags[10]@np.linalg.inv(self.box_from_tag)
        angles=depth_flap_angles(rgb,depth,k,self.observer.world_from_camera,self.box,self.angle_priors)
        outward={'short_left':np.array([-1,0,0]),'short_right':np.array([1,0,0]),'long_far':np.array([0,1,0]),'long_near':np.array([0,-1,0])}
        for flap,tid in TAG_IDS.items():
            if tid in tags:
                normal=-self.box[:3,:3].T@tags[tid][:3,2]
                angle=math.degrees(math.atan2(normal[2],normal@outward[flap]))
                if -40<angle<103:
                    row=angles.get(flap)
                    if row and abs(row['degrees']-angle)>12:row=None # Overlapping cardboard can confuse an unlabelled depth patch; the decoded tag supplies identity.
                    angles[flap]={'degrees':angle,'method':'apriltag_aligned_depth_plane','depth_check_degrees':row['degrees'] if row else None}
        self.angle_priors.update({f:r['degrees'] for f,r in angles.items()})
        reading={'world_from_box':self.box.tolist(),'tags':sorted(tags),'angles':angles,'seq':self.seq,'label':label}
        self.readings.append(reading)
        if self.record:self.sim.capture(label)
        return reading

    def move_arms(self,targets,seconds,label,orientation):
        if self.fault=='right_arm_disabled':targets={k:v for k,v in targets.items() if k!='right'}
        event=self.sim.move(targets,seconds,label,orientation,capture=self.record)
        if event['bad_penetration_mm']>1:raise ValueError('Robot collision exceeded 1 mm')
        # Deliberate contact can cause small compliance; large misses require recovery.
        if event['max_target_tracking_error_m']>.035:raise ValueError('Actual fingertips missed target by over 35 mm')
        return {'joint_positions':{s:self.sim.data.qpos[ix].tolist() for s,ix in self.sim.arm_indices.items()}}


def run(args):
    if (not all(np.isfinite(getattr(args,n)) for n in ('base_height','setback','stiffness','noise','dropout','dx','dy','yaw'))
            or args.noise<0 or args.stiffness<0 or not 0<=args.dropout<=1
            or not 64<=args.width<=1280 or not 64<=args.height<=960):
        raise ValueError('Invalid finite simulation geometry, camera size or sensor noise')
    out=Path(args.out).resolve()
    if out.exists():raise ValueError('Output already exists; preserve previous experiments')
    sim=FoldingSimulation(Path(args.simulation_root),out,base_height=args.base_height,setback=args.setback,stiffness=args.stiffness,width=args.width,height=args.height,offset=(args.dx,args.dy),yaw=math.radians(args.yaw))
    port=PixelPort(sim,seed=args.seed,noise=args.noise,dropout=args.dropout,fault=args.fault,record=not args.no_video)
    if args.fault=='stuck_far_flap':
        j=sim.model.joint('long_far_hinge').id
        sim.model.jnt_stiffness[j]=50.;sim.model.qpos_spring[sim.model.jnt_qposadr[j]]=.10
    if sim.model.nu!=12 or any(not sim.model.joint(int(j)).name.startswith(('left_','right_')) for j in sim.model.actuator_trnid[:,0]):
        raise ValueError('Only the twelve robot joints may be actuated')
    controller=FoldingController(port)
    outcome={'visual_sequence_passed':False}
    try:
        sim.capture('Initial open carton')
        sim.move({},.4,'Settle passive carton',capture=False)
        initial_box=sim.data.body('carton').xpos.copy()
        if args.fault=='no_actions':
            sim.move({},23,'No-actions negative control',capture=False)
        else:outcome=controller.run()
    except Exception as exc:outcome['error']=str(exc)
    # Independent evaluator: this information is never returned to the controller.
    final=sim.truth_angles()
    all_closed=all(85<=v<=95 for v in final.values())
    pairs=[p for event in sim.events for p in event['contact_pairs']]
    finger_contact={side:sorted({a if 'cardboard' in a else b for a,b in pairs if any(side+'_'+part in a or side+'_'+part in b for part in ('moving_jaw','wrist_roll_follower'))}) for side in ('left','right')}
    both_worked=({'short_left_cardboard','long_near_cardboard'}.issubset(finger_contact['left']) and
                 {'short_right_cardboard','long_far_cardboard'}.issubset(finger_contact['right']))
    final_contacts=[{'geoms':[sim.model.geom(c.geom1).name,sim.model.geom(c.geom2).name],'penetration_mm':max(0.,float(-c.dist*1000))} for c in sim.data.contact]
    supported=all(any(set(c['geoms'])=={'contents',f+'_cardboard'} for c in final_contacts) for f in ('short_left','short_right'))
    holds=[e for e in sim.events if e['label']=='Verify two-second closure']
    hold_passed=bool(holds and holds[-1]['duration_s']>=2 and all(85<=lo<=hi<=95 for lo,hi in holds[-1]['flap_angle_extrema_degrees'].values()))
    report={'simulation_only':True,'physical_validation':False,'approach':'geometric_rgbd_tag_contact_controller','learned_policy':False,
            'success':bool(outcome.get('visual_sequence_passed') and all_closed and hold_passed and both_worked and supported and sim.stats['max_bad_penetration_mm']<=1),
            'controller':outcome,'independent_evaluation':{'final_flap_degrees':final,'all_four_closed':all_closed,'continuous_two_second_hold_passed':hold_passed,'short_flaps_supported_by_contents':supported,'final_contacts':final_contacts,
                                      'initial_carton_position_error_mm':float(np.linalg.norm(np.asarray(port.readings[0]['world_from_box'])[:3,3]-initial_box)*1000) if port.readings else None,
                                      'carton_translation_during_run_mm':float(np.linalg.norm(sim.data.body('carton').xpos-initial_box)*1000),
                                      'both_hands_contacted_flaps':both_worked,'finger_contacts':finger_contact},
            'configuration':vars(args),'observations':port.readings,'perception_quality':port.observer.history,'arm_tag_checks':port.arm_tag_checks,
            'assumptions':{'base_height_above_table_m':args.base_height,'base_setback_from_near_rim_m':args.setback,'base_spacing_m':.30,
                           'contents_top_above_table_m':.102,'hinge_range_degrees':[-97.4,174.8],'closure_tolerance_degrees_from_horizontal':5.,
                           'hinge_stiffness_Nm_per_rad':args.stiffness,'hinge_friction_Nm':.004,'flap_mass_kg':.023,'depth_noise_std_m':args.noise,'depth_dropout_fraction':args.dropout,
                           'registration':'Surveyed table tag 1; calibrated robot base locations. Carton pose obtained from detected tag 10 and depth.',
                           'additional_markers':'ID 4 left housing; ID 10 carton wall; IDs 11-14 flap outside faces; exact simulated sizes and rigid mounts.',
                           'cart_geometry':'Fixed arm bases; robot cart body not modeled; tabletop and carton collide.',
                           'cardboard_model':'Rigid panels with passive frictional spring hinges. Material properties assumed, not measured.'},
            'actuated_joint_names':[sim.model.joint(int(j)).name for j in sim.model.actuator_trnid[:,0]],
            'code_sha256':{str(p.relative_to(Path(__file__).resolve().parents[1])):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__).resolve(),*[Path(__file__).resolve().parents[1]/'carton'/n for n in ['folding_sim.py','folding_vision.py','folding_controller.py']]]},
            'source_arm_sha256':hashlib.sha256((Path(args.simulation_root)/'scene-assets/arm-import.xml').read_bytes()).hexdigest()}
    physics=sim.save('folding')
    report['physics']=physics
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({'out':str(out),'success':report['success'],'controller_error':outcome.get('error'),'flap_degrees':final,'hold_passed':hold_passed,'contents_support':supported},indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=360)
    p.add_argument('--base-height',type=float,default=.26);p.add_argument('--setback',type=float,default=.04)
    p.add_argument('--stiffness',type=float,default=.008);p.add_argument('--seed',type=int,default=1)
    p.add_argument('--noise',type=float,default=.0008);p.add_argument('--dropout',type=float,default=.25)
    p.add_argument('--dx',type=float,default=0);p.add_argument('--dy',type=float,default=0);p.add_argument('--yaw',type=float,default=0)
    p.add_argument('--fault',choices=['missing_tags','missing_depth','right_arm_disabled','stuck_far_flap','no_actions','bad_gripper_calibration'])
    p.add_argument('--no-video',action='store_true')
    run(p.parse_args())

if __name__=='__main__':main()
