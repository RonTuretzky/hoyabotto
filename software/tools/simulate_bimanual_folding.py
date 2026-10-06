"""Rendered tag/depth -> paired robot joints -> passive carton contact, offline."""
from __future__ import annotations
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import numpy as np
import mujoco

from carton.folding_sim import FoldingSimulation,FLAPS,TAG_IDS,W,H
from carton.folding_vision import RGBDTagObserver,depth_flap_angles
from carton.folding_controller import FoldingController
from carton.folding_diagonal import DiagonalFoldingController
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_solver import FoldingSolver
from carton.folding_markers import carton_pose_from_tags
from carton.folding_tool_tags import paddle_pose_from_tags


def station_from_args(args):
    dimensions = (args.base_height, args.base_to_table_edge, args.box_from_table_edge)
    marker=(getattr(args,'table_tag_x',None),getattr(args,'table_tag_y',None))
    backup=(getattr(args,'backup_table_tag_x',None),getattr(args,'backup_table_tag_y',None))
    if (marker[0] is None)!=(marker[1] is None):raise ValueError('Provide both table tag coordinates')
    if (backup[0] is None)!=(backup[1] is None):raise ValueError('Provide both backup table tag coordinates')
    if args.reference_layout:
        if any(v is not None for v in dimensions) or args.base_spacing != .30 or marker[0] is not None or backup[0] is not None:
            raise ValueError('--reference-layout cannot be combined with station dimensions')
        return FoldingStation.historical_reference()
    if any(v is None for v in dimensions):
        raise ValueError('Specify --base-height, --base-to-table-edge and --box-from-table-edge in metres, or explicitly choose --reference-layout (not the real station)')
    return FoldingStation(*dimensions, base_spacing=args.base_spacing,table_marker_xy=None if marker[0] is None else marker,backup_table_marker_xy=None if backup[0] is None else backup)

class PixelPort:
    def __init__(self,sim,*,seed=0,noise=.0008,dropout=.25,fault=None,record=True,camera='front'):
        self.sim=sim;self.rng=np.random.default_rng(seed);self.noise=noise;self.dropout=dropout;self.fault=fault
        self.camera=camera
        anchor=np.eye(4);anchor[:3,:3]=np.diag([-1,1,-1]);anchor[:3,3]=sim.station.table_tag_position
        additional={}
        if sim.station.backup_table_marker_xy is not None:
            second=anchor.copy();second[:3,3]=[*sim.station.backup_table_marker_xy,.0013];additional[20]=second
        self.observer=RGBDTagObserver(anchor,additional_anchors=additional,stationary_camera=bool(additional));self.seq=0;self.readings=[];self.record=record
        self.box=None;self.angle_priors={}
        self.arm_tag_checks=[]

    def observe(self,label):
        rgb=self.sim.render(self.camera);depth=self.sim.render(self.camera,True)
        depth+=self.rng.normal(0,self.noise,depth.shape)
        depth[self.rng.random(depth.shape)<self.dropout]=0
        if self.fault=='missing_depth':depth[:]=0
        if self.fault=='missing_tags':rgb[:]=0
        h,w=depth.shape;f=h/(2*math.tan(math.radians(float(self.sim.model.camera(self.camera).fovy[0]))/2))
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
        self.box,box_registration=carton_pose_from_tags(tags,self.observer.history[-1]['quality'])
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
        reading={'world_from_box':self.box.tolist(),'box_registration':box_registration,'camera':self.camera,'tags':sorted(tags),'angles':angles,'seq':self.seq,'label':label}
        reading['paddle']=paddle_pose_from_tags(tags,self.observer.history[-1]['quality'])
        self.readings.append(reading)
        if self.record:self.sim.capture(label)
        return reading

    def move_arms(self,targets,seconds,label,orientation):
        if self.fault=='right_arm_disabled':targets={k:v for k,v in targets.items() if k!='right'}
        event=self.sim.move(targets,seconds,label,orientation,capture=self.record)
        if event.get('step_error'):raise ValueError(event['step_error'])
        if event['bad_penetration_mm']>1:raise ValueError('Robot collision exceeded 1 mm')
        # Deliberate contact can cause small compliance; large misses require recovery.
        if event['max_target_tracking_error_m']>.035:raise ValueError('Actual fingertips missed target by over 35 mm')
        return {'joint_positions':{s:self.sim.data.qpos[ix].tolist() for s,ix in self.sim.arm_indices.items()}}

    def set_grippers(self,openings,seconds,label):
        if hasattr(self.sim,'paddle_spec'):
            # Retain the paddle when releasing carton contact. Tool release
            # would require a separate supported placement, outside this test.
            openings={s:v for s,v in openings.items() if s!='right'}
        if self.fault=='right_arm_disabled':openings={s:v for s,v in openings.items() if s!='right'}
        event=self.sim.move({},seconds,label,None,capture=self.record,grippers=openings)
        if event.get('step_error'):raise ValueError(event['step_error'])
        if event['bad_penetration_mm']>1:raise ValueError('Robot collision exceeded 1 mm during gripper command')
        return {'joint_positions':{s:self.sim.data.qpos[ix].tolist() for s,ix in self.sim.arm_indices.items()}}


def run(args,controller_class=None):
    station=station_from_args(args)
    material=CartonMaterial(cardboard_mass_kg=getattr(args,'cardboard_mass',.272),
        contents_mass_kg=getattr(args,'contents_mass',0.),contents_top_m=getattr(args,'contents_top',.102),
        table_friction=getattr(args,'table_friction',.35),hinge_stiffness=args.stiffness,
        hinge_friction=getattr(args,'hinge_friction',.004),hinge_damping=getattr(args,'hinge_damping',.008),
        hinge_rest_degrees=getattr(args,'hinge_rest',0.),
        flap_stiffness=None if getattr(args,'flap_stiffness',None) is None else tuple(args.flap_stiffness))
    release_seconds=getattr(args,'release_seconds',5.)
    if not np.isfinite(release_seconds) or release_seconds<0:raise ValueError('Nonnegative finite release duration required')
    if controller_class is None:
        strategy=getattr(args,'strategy','original')
        if strategy not in ('original','diagonal'):raise ValueError('Unknown folding strategy')
        if strategy=='diagonal' and station.reference_layout:raise ValueError('Diagonal strategy requires an explicit rear-cart station')
        controller_class=DiagonalFoldingController if strategy=='diagonal' else FoldingController
    tool=getattr(args,'tool','claws')
    sim_class=FoldingSimulation;tool_options={}
    if tool=='paddle':
        from carton.folding_paddle import PaddleSpec,PaddleFoldingSimulation,PaddleFoldingController
        if getattr(args,'strategy',None)!='diagonal':raise ValueError('Paddle experiment uses diagonal strategy')
        sim_class=PaddleFoldingSimulation;controller_class=PaddleFoldingController
        tool_options['paddle']=PaddleSpec(getattr(args,'paddle_mass',.03),getattr(args,'paddle_friction',.8),getattr(args,'paddle_attachment','friction'))
    if (not all(np.isfinite(getattr(args,n)) for n in ('stiffness','noise','dropout','dx','dy','yaw'))
            or args.noise<0 or args.stiffness<0 or not 0<=args.dropout<=1
            or not 64<=args.width<=1280 or not 64<=args.height<=960):
        raise ValueError('Invalid finite simulation geometry, camera size or sensor noise')
    out=Path(args.out).resolve()
    if out.exists():raise ValueError('Output already exists; preserve previous experiments')
    solver=FoldingSolver.friction() if getattr(args,'solver','legacy')=='friction' else FoldingSolver()
    sim=sim_class(Path(args.simulation_root),out,station=station,material=material,width=args.width,height=args.height,offset=(args.dx,args.dy),yaw=math.radians(args.yaw),initial_right_roll=getattr(args,'initial_right_roll',None),solver=solver,**tool_options)
    port=PixelPort(sim,seed=args.seed,noise=args.noise,dropout=args.dropout,fault=args.fault,record=not args.no_video,camera=getattr(args,'camera','front'))
    if args.fault=='stuck_far_flap':
        j=sim.model.joint('long_far_hinge').id
        sim.model.jnt_stiffness[j]=50.;sim.model.qpos_spring[sim.model.jnt_qposadr[j]]=.10
    if sim.model.nu!=12 or any(not sim.model.joint(int(j)).name.startswith(('left_','right_')) for j in sim.model.actuator_trnid[:,0]):
        raise ValueError('Only the twelve robot joints may be actuated')
    controller=controller_class(port,rear_cart=not station.reference_layout)
    assignments=controller.flap_assignments
    if (set(assignments)!= {'left','right'} or any(len(set(v))!=2 for v in assignments.values())
            or set(assignments['left']) & set(assignments['right'])
            or set(assignments['left']) | set(assignments['right']) != set(FLAPS)):
        raise ValueError('Each arm must be assigned two different flaps, covering all four')
    outcome={'visual_sequence_passed':False}
    initial_box=sim.data.body('carton').xpos.copy()
    try:
        sim.capture('Initial open carton')
        settling=sim.move({},.4,'Settle passive carton',capture=False)
        if settling['bad_penetration_mm']>1:raise ValueError('Initial robot pose has a forbidden collision')
        initial_box=sim.data.body('carton').xpos.copy()
        if args.fault=='no_actions':
            sim.move({},23,'No-actions negative control',capture=False)
        else:outcome=controller.run()
    except Exception as exc:outcome['error']=str(exc)
    # Independent evaluator: this information is never returned to the controller.
    final=sim.truth_angles()
    all_closed=all(85<=v<=95 for v in final.values())
    pairs=[p for event in sim.events for p in event['contact_pairs']]
    finger_contact={side:sorted({a if 'cardboard' in a else b for a,b in pairs if any(side+'_'+part in a or side+'_'+part in b for part in ('moving_jaw','wrist_roll_follower','paddle_contact'))}) for side in ('left','right')}
    both_worked=all({flap+'_cardboard' for flap in assignments[side]}.issubset(finger_contact[side]) for side in ('left','right'))
    final_contacts=[{'geoms':[sim.model.geom(c.geom1).name,sim.model.geom(c.geom2).name],'penetration_mm':max(0.,float(-c.dist*1000))} for c in sim.data.contact]
    supported=all(any(set(c['geoms'])=={'contents',f+'_cardboard'} for c in final_contacts) for f in ('short_left','short_right'))
    holds=[e for e in sim.events if e['label']=='Verify two-second closure']
    hold_passed=bool(holds and holds[-1]['duration_s']>=2 and all(85<=lo<=hi<=95 for lo,hi in holds[-1]['flap_angle_extrema_degrees'].values()))
    held_passed=bool(outcome.get('visual_sequence_passed') and all_closed and hold_passed and both_worked and sim.stats['max_bad_penetration_mm']<=1
                     and sim.motion_stats['minimum_bottom_corner_table_clearance_mm']>=0)
    release={'requested_seconds':release_seconds,'performed':False,'passed':False,
             'reason':'No verified held closure to release' if not held_passed else 'Release test disabled'}
    if held_passed and release_seconds>0:
        release['reason']=None
        release_event_start=len(sim.events)
        try:
            port.set_grippers({'left':.35,'right':.35},.4,'Release finger pinch after held closure')
            # Encoder-derived robot tip poses, not hidden carton coordinates.
            raised={side:(sim.data.site(sim.control_sites[side]).xpos+[0,0,.080]).tolist() for side in ('left','right')}
            port.move_arms(raised,.8,'Lift hands off the folded flaps','down')
            port.move_arms({'left':[-.24,-.10,.29],'right':[.24,-.10,.29]},1.,'Withdraw both hands from the carton','down')
            event=sim.move({},release_seconds,'Observe unassisted flap springback',capture=not args.no_video)
            release.update(performed=True,flap_angle_extrema_degrees=event['flap_angle_extrema_degrees'],
                           final_flap_degrees=sim.truth_angles(),robot_flap_contacts=event['contact_pairs'],duration_s=event['duration_s'])
            release['closure_maintained_during_withdrawal']=all(
                85<=lo<=hi<=95 for segment in sim.events[release_event_start:]
                for lo,hi in segment['flap_angle_extrema_degrees'].values())
            # Both physical contact absence and the whole observation window
            # matter: returning to near-horizontal later is not retention.
            release['passed']=bool(release['closure_maintained_during_withdrawal'] and event['duration_s']>=release_seconds and not event['contact_pairs']
                and all(85<=lo<=hi<=95 for lo,hi in event['flap_angle_extrema_degrees'].values())
                and sim.stats['max_bad_penetration_mm']<=1
                and sim.motion_stats['minimum_bottom_corner_table_clearance_mm']>=0)
            if not release['passed']:release['reason']='Flaps did not stay closed without robot contact for the full release interval'
            try:release['visual_observation']=port.observe('Observe flaps after both hands release')
            except ValueError as exc:
                release['visual_error']=str(exc);release['passed']=False
        except ValueError as exc:release['reason']=str(exc)
    sim.capture('After release: retained closure' if release['passed'] else
                ('STOP: closure not retained after release' if release['performed'] else 'STOP: full unassisted closure not demonstrated'))
    report={'simulation_only':True,'physical_validation':False,'approach':'geometric_rgbd_tag_contact_controller','learned_policy':False,
            'success':bool(held_passed and release['passed']),'success_definition':'Four flaps closed under contact, then still closed with no hand contact during the full requested release interval',
            'held_closure_passed':held_passed,'release_test':release,
            'controller':outcome,'independent_evaluation':{'pre_release_flap_degrees':final,'final_flap_degrees':sim.truth_angles(),'all_four_closed_before_release':all_closed,'continuous_two_second_hold_passed':hold_passed,'short_flaps_supported_by_contents':supported if material.contents_mass_kg>0 else None,'pre_release_contacts':final_contacts,
                                      'initial_carton_position_error_mm':float(np.linalg.norm(np.asarray(port.readings[0]['world_from_box'])[:3,3]-initial_box)*1000) if port.readings else None,
                                      'carton_translation_during_run_mm':float(np.linalg.norm(sim.data.body('carton').xpos-initial_box)*1000),
                                      'both_hands_contacted_flaps':both_worked,'finger_contacts':finger_contact,'assigned_flaps':assignments},
            'configuration':vars(args),'station':station.report(),'initial_carton_footprint':station.carton_footprint((args.dx,args.dy),math.radians(args.yaw)),
            'observations':port.readings,'perception_quality':port.observer.history,'arm_tag_checks':port.arm_tag_checks,
            'assumptions':{'base_height_above_table_m':station.base_height,'base_setback_from_near_rim_m':station.setback,'base_spacing_m':station.base_spacing,
                           'material':material.report(),'contents_top_above_table_m':material.contents_top_m if material.contents_mass_kg>0 else None,'hinge_range_degrees':[-97.4,174.8],'closure_tolerance_degrees_from_horizontal':5.,
                           'hinge_stiffness_Nm_per_rad':list(material.stiffnesses),'hinge_friction_Nm':material.hinge_friction,'flap_mass_kg':.023*material.cardboard_mass_kg/.272,'depth_noise_std_m':args.noise,'depth_dropout_fraction':args.dropout,
                           'registration':'Surveyed table tag 1 and optional ID 20; calibrated robot base locations. With two anchors the camera is fixed after initial registration and fresh anchor geometry is checked every observation. Carton pose obtained from fresh detected ID 10, 21 or 22 and aligned depth; inconsistent markers refused.',
                           'additional_markers':'ID 4 left housing; IDs 10, 21, 22 carton walls (45 mm); IDs 11-14 flap outside faces; optional ID 20 second table anchor (60 mm); exact simulated sizes and rigid mounts. Physical mounts are unverified.',
                           'gripper_geometry':'Stock rigid SO101 fingertips'+('; actual paddle CAD in right jaws' if tool=='paddle' else '')+'. White compliant attachments visible in the photos have not been identified or modeled.',
                           'cart_geometry':'Historical fixed arm bases with no cart' if station.reference_layout else 'Fixed three-tray cart approximation; static table overlap refused; arm/cart contacts checked',
                           'cardboard_model':'Rigid panels with passive frictional spring hinges. Material properties assumed, not measured.'},
            'actuated_joint_names':[sim.model.joint(int(j)).name for j in sim.model.actuator_trnid[:,0]],
            'code_sha256':{str(p.relative_to(Path(__file__).resolve().parents[1])):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__).resolve(),*[Path(__file__).resolve().parents[1]/'carton'/n for n in ['folding_sim.py','folding_vision.py','folding_controller.py','folding_diagonal.py','folding_paddle.py','folding_station.py','folding_cart.py','folding_material.py']]]},
            'source_arm_sha256':hashlib.sha256((Path(args.simulation_root)/'scene-assets/arm-import.xml').read_bytes()).hexdigest()}
    controller_source=Path(inspect.getfile(controller_class))
    report['controller_source']={'class':controller_class.__name__,'path':str(controller_source),'sha256':hashlib.sha256(controller_source.read_bytes()).hexdigest()}
    physics=sim.save('folding')
    report['physics']=physics
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({'out':str(out),'success':report['success'],'held_closure_passed':held_passed,'release_passed':release['passed'],'controller_error':outcome.get('error'),'flap_degrees':sim.truth_angles(),'carton_motion':sim.motion_stats},indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=360)
    p.add_argument('--strategy',choices=['original','diagonal'],default='original')
    p.add_argument('--tool',choices=['claws','paddle'],default='claws',help='Paddle is a free body initially placed in the right jaws; no pickup claim')
    p.add_argument('--solver',choices=['legacy','friction'],default='legacy',help='Declared solver sensitivity experiment; physical friction and torque limits are unchanged')
    p.add_argument('--camera',choices=['front','station'],default='front',help='Station is a proposed external camera mount, not physical calibration')
    p.add_argument('--paddle-mass',type=float,default=.03)
    p.add_argument('--paddle-friction',type=float,default=.8)
    p.add_argument('--paddle-attachment',choices=['friction','rigid-diagnostic'],default='friction')
    p.add_argument('--initial-right-roll',type=float,help='Optional starting wrist roll in radians, applied before dynamics within original limits')
    p.add_argument('--reference-layout',action='store_true',help='Explicitly replay the old favorable station, which does not match the photos')
    p.add_argument('--base-height',type=float,help='Arm base_link origin height above tabletop, metres')
    p.add_argument('--base-to-table-edge',type=float,help='Horizontal distance from base_link origin line to near table edge, metres')
    p.add_argument('--box-from-table-edge',type=float,help='Distance from near table edge to carton near wall, metres')
    p.add_argument('--base-spacing',type=float,default=.30)
    p.add_argument('--table-tag-x',type=float,help='Assumed surveyed table anchor X coordinate, metres')
    p.add_argument('--table-tag-y',type=float,help='Assumed surveyed table anchor Y coordinate, metres')
    p.add_argument('--backup-table-tag-x',type=float,help='Optional surveyed ID 20 table anchor X, metres')
    p.add_argument('--backup-table-tag-y',type=float,help='Optional surveyed ID 20 table anchor Y, metres')
    p.add_argument('--stiffness',type=float,default=.018,help='Unmeasured elastic crease stiffness, Nm/rad')
    p.add_argument('--flap-stiffness',type=float,nargs=4,metavar=('LEFT','RIGHT','FAR','NEAR'),help='Optional per-flap stiffnesses, Nm/rad')
    p.add_argument('--hinge-friction',type=float,default=.004);p.add_argument('--hinge-damping',type=float,default=.008)
    p.add_argument('--hinge-rest',type=float,default=0.,help='Passive crease rest angle from upright, degrees')
    p.add_argument('--cardboard-mass',type=float,default=.272,help='Total empty cardboard mass, kg; unmeasured default')
    p.add_argument('--contents-mass',type=float,default=0.,help='Rigid contents mass, kg; zero removes interior support')
    p.add_argument('--contents-top',type=float,default=.102,help='Contents top height within carton, metres')
    p.add_argument('--table-friction',type=float,default=.35,help='Explicit table/cardboard sliding coefficient; unmeasured')
    p.add_argument('--release-seconds',type=float,default=5.,help='Hands-off retention observation; zero leaves retention unverified')
    p.add_argument('--seed',type=int,default=1)
    p.add_argument('--noise',type=float,default=.0008);p.add_argument('--dropout',type=float,default=.25)
    p.add_argument('--dx',type=float,default=0);p.add_argument('--dy',type=float,default=0);p.add_argument('--yaw',type=float,default=0)
    p.add_argument('--fault',choices=['missing_tags','missing_depth','right_arm_disabled','stuck_far_flap','no_actions','bad_gripper_calibration'])
    p.add_argument('--no-video',action='store_true')
    run(p.parse_args())

if __name__=='__main__':main()
