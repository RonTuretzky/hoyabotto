"""Start G4 with audited, rendered-vision pusher pickup experiments.

This trains a small parameterized controller by simulation search, not a
neural policy or full planter assembly. Failed episodes cannot be exported as
successful imitation demonstrations. There is deliberately no hardware mode.
"""
from __future__ import annotations
import argparse
import copy
import json
from itertools import groupby
import shutil
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from planter.g4_sim import BLOCKS,G4Owner,TAG_FROM_GRIP,asset_report,sha
from planter.g4_retention import audit_retention
from carton.servo.common import atomic_json
from farm.perception.gemma_calibration import CalibrationRobot
from farm.kinematics.lerobot import pose_error
from farm.perception.gemma_tags import TagObserver
from carton.folding_vision import depth_tag_pose


OBJECT_GEOMS={'pusher_blade','pusher_crossbar','pusher_handle'}
JAW_PARTS=('moving_jaw','wrist_roll_follower')
FAULTS=('missing_tag','occluded_tag','stale_frame','missing_depth','incorrect_registration',
        'open_jaws','zero_friction','disabled_motion')
VALIDATION_OFFSETS=((.004,.003),(-.004,.006),(.003,-.005))
OBJECT_VERTICES=np.asarray([np.asarray(position)+np.asarray(size)*[x,y,z] for _,position,size in BLOCKS
                           for x in (-1,1) for y in (-1,1) for z in (-1,1)])


def _object_contact(contact):
    """Return the other geom and direction of normal force on the pusher."""
    first,second=contact['geoms']
    if (first in OBJECT_GEOMS)==(second in OBJECT_GEOMS):return None
    return (second,-1.) if first in OBJECT_GEOMS else (first,1.)


def _object_vertices(row,*,gripper_frame=False):
    matrix=np.empty(9);pose=np.asarray(row['object_pose'])
    mujoco.mju_quat2Mat(matrix,pose[3:])
    vertices=OBJECT_VERTICES@matrix.reshape(3,3).T+pose[:3]
    if gripper_frame:vertices=(vertices-row['grasp_world_m'])@np.asarray(row['gripper_world_rotation'])
    return vertices


def score_episode(rows,completed,*,timestep_s=.002,require_contact_geometry=True):
    """Evaluate raw physics evidence, failing closed on missing or sparse data.

    Legacy traces can be audited with require_contact_geometry=False, which
    explicitly withholds qualification for unrecorded normals/gripper frames.
    No completion flag, contact summary boolean, or commanded pose is evidence
    of physical retention or a free, supported release.
    """
    report=dict(success=False,failure_reasons=[],scorer_version=3,hold_slip_mm=None,
                minimum_hold_clearance_mm=None,minimum_hold_rest_clearance_mm=None,
                maximum_object_penetration_mm=None,maximum_contact_force_n=None,
                contact_geometry_verified=False,qualification_evidence_complete=False,
                full_planter_success=False,physical_success=False)
    reasons=report['failure_reasons']
    if not isinstance(completed,bool) or not completed:reasons.append('sequence_incomplete')
    if not np.isfinite(timestep_s) or timestep_s<=0:
        reasons.append('invalid_declared_timestep');return report
    scalar_fields=('time_s','clearance_mm','rest_clearance_mm','table_edge_margin_mm',
                   'forbidden_penetration_mm','object_penetration_mm','arm_object_normal_force_n')
    shapes={'grip_world_m':(3,),'grasp_world_m':(3,),'object_pose':(7,),
            'encoder_radians':(6,),'ctrl':(6,)}
    geometry_present=bool(rows)
    try:
        for row in rows:
            if not isinstance(row['phase'],str) or not isinstance(row['jaw_contact_both'],bool):raise ValueError
            numbers=[row[name] for name in scalar_fields]
            for field,shape in shapes.items():
                array=np.asarray(row[field],dtype=float)
                if array.shape!=shape:raise ValueError
                numbers.extend(array.ravel())
            if not np.isclose(np.linalg.norm(row['object_pose'][3:]),1.,atol=1e-5):raise ValueError
            if not isinstance(row['contacts'],list):raise ValueError
            if 'gripper_world_rotation' in row:
                rotation=np.asarray(row['gripper_world_rotation'],dtype=float)
                if rotation.shape!=(3,3):raise ValueError
                numbers.extend(rotation.ravel())
                if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(rotation),1.,atol=1e-5):raise ValueError
            else:geometry_present=False
            for contact in row['contacts']:
                if len(contact['geoms'])!=2 or not all(isinstance(n,str) for n in contact['geoms']):raise ValueError
                if _object_contact(contact) is None:raise ValueError
                numbers.extend([contact['normal_force_n'],contact['penetration_mm']])
                if contact['normal_force_n'] < -1e-8 or contact['penetration_mm'] < 0:raise ValueError
                if 'normal_world_geom1_to_geom2' in contact and 'position_world_m' in contact:
                    for key in ('normal_world_geom1_to_geom2','position_world_m'):
                        array=np.asarray(contact[key],dtype=float)
                        if array.shape!=(3,):raise ValueError
                        numbers.extend(array)
                    if not np.isclose(np.linalg.norm(contact['normal_world_geom1_to_geom2']),1.,atol=1e-5):raise ValueError
                else:geometry_present=False
            if not np.isfinite(numbers).all():
                reasons.append('nonfinite_evaluation_state');return report
            if any(row[n]<0 for n in ('forbidden_penetration_mm','object_penetration_mm','arm_object_normal_force_n')):raise ValueError
    except (KeyError,TypeError,ValueError,IndexError):
        reasons.append('malformed_evaluation_state');return report
    report['contact_geometry_verified']=geometry_present
    report['qualification_evidence_complete']=geometry_present
    if require_contact_geometry and not geometry_present:reasons.append('missing_contact_geometry_evidence')
    expected=['approach_above','lower_outside','approach_handle','close','lift','hold','lower','release','withdraw','released_hold']
    phases=[phase for phase,_ in groupby(r['phase'] for r in rows)]
    if phases and phases[0]=='settle':phases=phases[1:]
    if phases not in (expected,expected[:8]+['disengage']+expected[8:]):reasons.append('missing_or_out_of_order_phases')
    deltas=np.diff([row['time_s'] for row in rows])
    if np.any(deltas<=0):reasons.append('nonmonotonic_physics_time')
    if np.any(np.abs(deltas-timestep_s)>max(1e-9,timestep_s*1e-5)):reasons.append('missing_or_irregular_physics_samples')
    hold=[r for r in rows if r['phase']=='hold'];released=[r for r in rows if r['phase']=='released_hold']
    if not hold or hold[-1]['time_s']-hold[0]['time_s']<2.:reasons.append('hold_not_observed_for_2s')
    jaw_rows=[];minimum_opposition=None
    for row in hold:
        jaws=[[],[]]
        for contact in row['contacts']:
            other,sign=_object_contact(contact)
            if contact['normal_force_n']>.005:
                for i,name in enumerate(JAW_PARTS):
                    if name in other:jaws[i].append((contact,sign))
        jaw_rows.append(all(jaws))
        if geometry_present and all(jaws):
            directions=[sum((np.asarray(c['normal_world_geom1_to_geom2'])*sign*c['normal_force_n'] for c,sign in jaw),start=np.zeros(3)) for jaw in jaws]
            norms=[np.linalg.norm(vector) for vector in directions]
            cosine=float(np.dot(*directions)/np.prod(norms)) if min(norms)>1e-8 else 1.
            minimum_opposition=max(minimum_opposition if minimum_opposition is not None else -1.,cosine)
    if hold and (min(r['rest_clearance_mm'] for r in hold)<20 or not all(jaw_rows) or not all(r['jaw_contact_both'] for r in hold)):
        reasons.append('lift_or_two_jaw_retention_failed')
    if minimum_opposition is not None and minimum_opposition>=0:reasons.append('jaw_loads_not_opposing')
    if hold:
        vectors=[]
        for row in hold:
            relative=np.asarray(row['grip_world_m'])-row['grasp_world_m']
            vectors.append(np.asarray(row['gripper_world_rotation']).T@relative if geometry_present else relative)
        vectors=np.asarray(vectors);report['hold_slip_mm']=float(np.linalg.norm(vectors-vectors[0],axis=1).max()*1000)
        if report['hold_slip_mm']>5:reasons.append('hold_slip_over_5mm')
        if geometry_present:
            start_vertices=_object_vertices(hold[0],gripper_frame=True)
            report['hold_rigid_body_drift_mm']=float(max(np.linalg.norm(_object_vertices(r,gripper_frame=True)-start_vertices,axis=1).max() for r in hold)*1000)
            if report['hold_rigid_body_drift_mm']>5:reasons.append('hold_rigid_body_slip_over_5mm')
    if not released or released[-1]['time_s']-released[0]['time_s']<1.:reasons.append('release_not_observed_for_1s')
    free_support=True
    for row in released:
        loaded=[c for c in row['contacts'] if c['normal_force_n']>.005]
        if (abs(row['rest_clearance_mm'])>1 or row['table_edge_margin_mm']<0 or
            not any(_object_contact(c)[0]=='staging_rest' for c in loaded) or
            any(_object_contact(c)[0]!='staging_rest' for c in loaded)):
            free_support=False
    if released and not free_support:reasons.append('release_not_free_on_staging_rest')
    if released:
        start_vertices=_object_vertices(released[0])
        report['released_rigid_body_drift_mm']=float(max(np.linalg.norm(_object_vertices(r)-start_vertices,axis=1).max() for r in released)*1000)
        if (report['released_rigid_body_drift_mm']>2 or
            max(np.linalg.norm(np.array(r['grip_world_m'])-released[0]['grip_world_m']) for r in released)>.002):
            reasons.append('released_object_did_not_settle')
    if any(r['forbidden_penetration_mm']>1 or r['object_penetration_mm']>1 or
           any(c['penetration_mm']>1 for c in r['contacts']) for r in rows):reasons.append('collision_gate')
    # Recompute robot/tool force from contact evidence as well as checking the
    # simulator summary, so stale or inconsistent summary fields cannot pass.
    contact_forces=[sum(max(0,c['normal_force_n']) for c in r['contacts']
                        if _object_contact(c)[0] not in ('staging_rest','table')) for r in rows]
    maximum_force=max([r['arm_object_normal_force_n'] for r in rows]+contact_forces,default=0.)
    if maximum_force>8:reasons.append('simulation_force_gate')
    report.update(success=not reasons,hold_slip_frame='gripper' if geometry_present else 'world_relative_legacy',
                  maximum_jaw_normal_cosine=minimum_opposition,
                  minimum_hold_clearance_mm=min((r['clearance_mm'] for r in hold),default=None),
                  minimum_hold_rest_clearance_mm=min((r['rest_clearance_mm'] for r in hold),default=None),
                  maximum_object_penetration_mm=max((r['object_penetration_mm'] for r in rows),default=0),
                  maximum_contact_force_n=maximum_force)
    return report


def qualification_reasons(candidate,validation,controls):
    """A partial/malformed evaluation set cannot promote a controller."""
    reasons=[]
    if (not candidate or not candidate.get('eligible_for_success_demonstrations') or
        not candidate.get('qualification_evidence_complete') or
        not candidate.get('clean_pickup_audit',{}).get('passed') or
        not candidate.get('clean_pickup_audit',{}).get('evidence_complete')):
        reasons.append('candidate_not_eligible')
    if (len(validation)!=len(VALIDATION_OFFSETS) or
        {tuple(row.get('object_reset_offset_m',[])) for row in validation}!=set(VALIDATION_OFFSETS) or
        {row.get('seed') for row in validation}!={100,101,102} or
        not all(row.get('success') and row.get('qualification_evidence_complete') and
                row.get('clean_pickup_audit',{}).get('passed') and row.get('clean_pickup_audit',{}).get('evidence_complete') and
                row.get('fault') is None for row in validation)):
        reasons.append('heldout_evaluation_incomplete_or_failed')
    if not validation or any(row.get('evaluation_role')!='fresh_confirmation' for row in validation):
        reasons.append('fresh_confirmation_not_established')
    if (len(controls)!=len(FAULTS) or {row.get('fault') for row in controls}!=set(FAULTS) or
        any(row.get('success') or not (row.get('failure_reasons') or row.get('stop_reason')) for row in controls)):
        reasons.append('fault_evaluation_incomplete_or_failed')
    parameters=('correction_m','close_angle_rad','release_height_m','vertical_withdrawal',
                'grasp_axis_world','withdrawal_distance_m','disengage_drop_m',
                'cartesian_step_m','require_clear_approach','require_clean_pickup')
    if candidate and any(any(row.get(key)!=candidate.get(key) for key in parameters) for row in validation+controls):
        reasons.append('controller_parameters_changed_during_evaluation')
    return reasons


def run_episode(owner,robot,snapshot,out,*,offset=(0,0),correction=(0,0,0),close_angle=-.174,release_height=0.,vertical_withdrawal=False,fault=None,video=True,seed=1,evaluation_role='development',grasp_axis=(0,0,1),withdrawal_distance=.045,disengage_drop=.012,cartesian_step=0.,require_clear_approach=False,require_clean_pickup=False):
    out.mkdir();sim=owner.sim;m,d=sim.model,sim.data
    for field in ('qpos','qvel','ctrl'):getattr(d,field)[:]=snapshot[field]
    d.time=snapshot['time'];d.qacc_warmstart[:]=0;sim.qseed=snapshot['qseed'].copy()
    address=m.joint('pusher_free').qposadr[0];d.qpos[address:address+2]+=offset
    m.geom_friction[:]=snapshot['friction'];m.geom_rgba[:]=snapshot['rgba']
    sim.disabled_motion=fault=='disabled_motion'
    owner.camera.capture_depth=True;owner.camera.rng=np.random.default_rng(seed)
    if fault=='zero_friction':
        for g in sim.object_geoms|sim.arm_geoms:m.geom_friction[g,:]=0
    tagbody=m.body('tag31').id
    if fault=='missing_tag':m.geom_rgba[m.geom_bodyid==tagbody,3]=0
    if fault=='occluded_tag':m.geom_rgba[m.geom('tag31_occluder').id,3]=1
    original_call=owner.camera.call
    original_registration=robot.registration_path.read_bytes()
    if fault=='incorrect_registration':
        corrupt=json.loads(original_registration);corrupt['base_from_camera'][1][3]+=.03
        atomic_json(robot.registration_path,corrupt)
    mujoco.mj_forward(m,d)
    sim.records=[];sim.commands=[];frames=[];sim.record=True
    np.savez_compressed(out/'replay-reset.npz',qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy(),
                        qacc_warmstart=d.qacc_warmstart.copy(),time_s=float(d.time),qseed=sim.qseed.copy(),
                        friction=m.geom_friction.copy(),rgba=m.geom_rgba.copy())
    owner.now+=3
    def frame(label):
        image=Image.fromarray(owner.camera.render()).resize((768,576))
        draw=ImageDraw.Draw(image);draw.rectangle((0,0,768,46),fill='white')
        draw.text((8,6),'G4 SIMULATION ONLY | rendered tag -> registration -> joint actuators',fill='black')
        draw.text((8,25),f'{out.name} | {label} | t={d.time-snapshot["time"]:.2f}s',fill='black')
        frames.append(image)
    sim.frame_callback=frame if video else None
    report=dict(name=out.name,seed=seed,evaluation_role=evaluation_role,object_reset_offset_m=list(offset),correction_m=list(correction),fault=fault,
                close_angle_rad=close_angle,
                release_height_m=release_height,
                vertical_withdrawal=vertical_withdrawal,
                grasp_axis_world=list(grasp_axis),withdrawal_distance_m=withdrawal_distance,
                disengage_drop_m=disengage_drop,endpoint_diagnostics=[],
                cartesian_step_m=cartesian_step,require_clear_approach=require_clear_approach,
                require_clean_pickup=require_clean_pickup,
                target_source='production tag corners + rendered aligned noisy depth + fitted 8/3 registration + CAD offset',
                controller='parameterized visual target then joint-space IK/actuators',
                abort_source='privileged simulator contact diagnostics, not deployable force sensing',
                timestep_s=float(m.opt.timestep),registration_sha256=sha(robot.registration_path),
                physical_motor_writes=0,completed=False)
    try:
        initial_bad=sim.intersections(.1)
        if initial_bad:raise ValueError(f'Initial intersections after reset: {initial_bad}')
        sim.advance(.25,'settle');owner.now+=.25
        frame('initial')
        if fault=='stale_frame':
            def stale(*args,**kwargs):
                payload=original_call(*args,**kwargs)
                # RenderedOwner normally stamps frames with its own clock.
                payload['images'][0]['fresh']=False;return payload
            owner.camera.call=stale
        result=robot.call('robot_get_registered_tags',{})
        if fault=='stale_frame':owner.camera.call=original_call
        atomic_json(out/'observation.json',{k:v for k,v in result.items() if k!='images'})
        import base64
        raw=owner.camera.last_payload
        (out/'observation-rgb.png').write_bytes(base64.b64decode(raw['images'][0]['data_base64']))
        if fault=='missing_depth':owner.camera.last_depth[:]=np.nan
        np.savez_compressed(out/'observation-depth.npz',depth_m=owner.camera.last_depth)
        if not result.get('ok'):raise ValueError('Registered observation refused: '+str(result.get('result')))
        observed=TagObserver(clock=owner.clock,geometry=owner.camera.geometry).observe(raw,['sim'],ids=[1,2,31],include_images=False)
        row=observed['result']['observations']['sim']
        atomic_json(out/'rgbd-observation.json',observed)
        tag=next((t for t in row['tags'] if t['tag_id']==31 and t['status']=='DETECTED'),None)
        if tag is None:raise ValueError('Fresh G4 pusher marker absent')
        if owner.camera.last_depth_sim_time!=float(d.time):raise ValueError('RGB/depth simulation timestamps differ')
        camera_pose,quality=depth_tag_pose(tag['corners_px'],owner.camera.last_depth,
                                          np.array(raw['result']['cameras']['sim']['intrinsics']),.018)
        report['depth_quality']=quality
        pose=np.array(owner.registration['base_from_camera'])@camera_pose
        target=(pose@TAG_FROM_GRIP)[:3]+correction
        report['target_m']=target.tolist()
        # Evaluation-only error: not used to correct targets or train offset.
        report['independent_target_error_mm']=float(np.linalg.norm(target-d.site('pusher_grip').xpos)*1000)
        report['independent_target_error_reference']='Original GRIP_LOCAL reference; includes declared correction or handle-tip aim offset, not registration error alone'
        report['independent_raw_registered_grip_error_mm']=float(np.linalg.norm((pose@TAG_FROM_GRIP)[:3]-d.site('pusher_grip').xpos)*1000)
        sequence=[('approach_above',target+[-.035,0,.080],.55,1.5),
                  ('lower_outside',target+[-.035,0,0],.55,1.5),
                  ('approach_handle',target,.55,1.5),
                  ('close',None,.55 if fault=='open_jaws' else close_angle,1.5),
                  ('lift',target+[0,0,.080],None,2.),('hold',None,None,2.1),
                  ('lower',target+[0,0,release_height],None,2.),('release',None,.55,1.5),
                  ('withdraw',target+[-withdrawal_distance,0,.035],.55,1.5),('released_hold',None,None,1.1)]
        if vertical_withdrawal:
            sequence[8:9]=[('disengage',target+[0,0,release_height-disengage_drop],.55,1.5),
                           ('withdraw',target+[-withdrawal_distance,0,release_height-disengage_drop],.55,1.5)]
        previous_point=None
        for label,point,jaw,seconds in sequence:
            command=d.ctrl.copy()
            if jaw is not None:command[5]=jaw
            # Cartesian subdivision uses only the last commanded point and
            # visual target. It never looks at simulator object geometry.
            count=(max(1,int(np.ceil(np.linalg.norm(point-previous_point)/cartesian_step)))
                   if point is not None and previous_point is not None and cartesian_step>0 else 1)
            for step in range(1,count+1):
                if point is not None:
                    waypoint=(previous_point+(point-previous_point)*step/count if count>1 else point)
                    command[:5]=sim.ik(waypoint,grasp_axis=grasp_axis)
                sim.advance(seconds/count,label,command)
            owner.now+=seconds
            if point is not None:
                previous_point=np.asarray(point).copy()
                error=float(np.linalg.norm(d.site('grasp').xpos-point))
                state=sim.score_state()
                loaded_environment=[c for c in state['arm_environment_contacts'] if c['normal_force_n']>.005]
                report['endpoint_diagnostics'].append(dict(phase=label,target_m=np.asarray(point).tolist(),
                    actual_grasp_m=d.site('grasp').xpos.tolist(),error_mm=error*1000,
                    loaded_arm_environment_contacts=loaded_environment))
                if error>.008:
                    raise ValueError('Measured simulated gripper missed commanded endpoint by more than 8mm')
                if label=='approach_handle' and require_clear_approach and (loaded_environment or error>.0015):
                    raise ValueError('Pre-grasp approach is obstructed or exceeds 1.5mm endpoint error; simulation-only contact diagnostic')
        report['completed']=True
    except (ValueError,RuntimeError) as exc:
        report['stop_reason']=str(exc)
    finally:
        sim.frame_callback=None;owner.camera.call=original_call
        robot.registration_path.write_bytes(original_registration)
        sim.disabled_motion=False;owner.camera.capture_depth=False
    report.update(score_episode(sim.records,report['completed'],timestep_s=float(m.opt.timestep)))
    report['scorer_v3_success']=report['success']
    report['clean_pickup_audit']=audit_retention(sim.records,timestep_s=float(m.opt.timestep))
    if require_clean_pickup and not report['clean_pickup_audit']['passed']:
        report['success']=False
        report['failure_reasons']+=['clean_pickup:'+reason for reason in report['clean_pickup_audit']['failure_reasons']]
    report['eligible_for_success_demonstrations']=(report['success'] and fault is None and
        report['clean_pickup_audit']['passed'] and report['clean_pickup_audit']['evidence_complete'])
    atomic_json(out/'result.json',report)
    with (out/'physics.jsonl').open('w') as f:
        for row in sim.records:f.write(json.dumps(row,allow_nan=False)+'\n')
    atomic_json(out/'commands.json',sim.commands)
    if video:
        frame('PASS' if report['success'] else 'STOP: '+(report.get('stop_reason') or ','.join(report['failure_reasons'])))
        frames[0].save(out/'before.png');frames[-1].save(out/'after.png')
        frames[0].save(out/'timeline.gif',save_all=True,append_images=frames[1:]+[frames[-1]]*25,duration=100,loop=0)
    print(json.dumps(report),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',type=Path,required=True);p.add_argument('--cad',type=Path,required=True)
    p.add_argument('--model-directory',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--width',type=int,default=1920);p.add_argument('--timestep',type=float,default=.002)
    p.add_argument('--noslip',type=int,default=3);p.add_argument('--probe',action='store_true')
    p.add_argument('--rest-height',type=float,default=.020)
    p.add_argument('--fine-grip',action='store_true')
    p.add_argument('--release-height',type=float,default=0.)
    p.add_argument('--single-candidate',action='store_true',help='Freeze correction=(0,-4,+4)mm and jaw=-0.08rad for solver sensitivity')
    p.add_argument('--vertical-withdrawal',action='store_true')
    args=p.parse_args()
    if args.out.exists():p.error('Use a new output directory; previous evidence is immutable')
    args.out.mkdir(parents=True)
    snapshot_dir=args.out/'source';snapshot_dir.mkdir()
    software=Path(__file__).resolve().parents[1]
    sources={Path(__file__).resolve()}
    for module in list(sys.modules.values()):
        filename=getattr(module,'__file__',None)
        if filename:
            source_file=Path(filename).resolve()
            if source_file.suffix=='.py' and source_file.is_relative_to(software) and '.venv' not in source_file.parts:
                sources.add(source_file)
    for source_file in sorted(sources):
        destination=snapshot_dir/source_file.relative_to(software);destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source_file,destination)
    atomic_json(args.out/'assets.json',asset_report(args.simulation_root,args.cad,args.model_directory))
    atomic_json(args.out/'invocation.json',dict(argv=sys.argv,mujoco=mujoco.__version__,simulation_only=True,
        source_hashes={str(path):sha(path) for path in sorted(sources)},
        assumptions={'table_height_relative_to_arm_base_m':-.06,'table_edge_from_base_m':.24,
                     'camera_position_m':[.54,-.42,.46],'pusher_mass_kg':.012,'friction':.8,
                     'physical_measurements_verified':False,'pusher_tag_black_square_mm':18,
                     'staging_rest_height_mm':args.rest_height*1000,'staging_rest_near_edge_m':.395,
                     'staging_rest_physical_fit_verified':False,
                     'depth_noise_sigma_mm':.8,'depth_dropout':.25,'depth_used_for_control':True},
        solver={'timestep':args.timestep,'noslip_iterations':args.noslip,'impratio':10},
        limits={'paper_modeled':False,'full_cart_modeled':False,'second_arm_modeled':False,
                'paper_feeding_folding_wicking_validated':False,'physical_force_threshold_validated':False}))
    owner=G4Owner(args.simulation_root,args.cad,args.model_directory,args.out,width=args.width,
                  timestep=args.timestep,noslip=args.noslip,rest_height=args.rest_height)
    owner.sim.record=False
    try:
        Image.fromarray(owner.camera.render()).save(args.out/'scene.png')
        initial=owner.tagged.call('robot_get_tags',{'cameras':['sim'],'tag_ids':[1,2,31]})
        atomic_json(args.out/'initial-tags.json',{k:v for k,v in initial.items() if k!='images'})
        cfg=dict(schema=1,arm='right',joints=['shoulder_pan','wrist_flex'],camera='sim',
                 model_directory=str(args.model_directory.resolve()),lock_file=str(args.out/'simulation.lock'))
        config_path=args.out/'.private/tag-calibration.json';atomic_json(config_path,cfg)
        robot=CalibrationRobot(owner.tagged,config_path,clock=owner.clock);robot.catalog()
        response=robot.call('robot_calibrate_tags',{'mode':'registration'})
        atomic_json(args.out/'calibration.json',response)
        if not response.get('ok') or response['result']['status']!='REGISTRATION_VALIDATED':
            atomic_json(args.out/'result.json',dict(status='CALIBRATION_FAILED',policy_exported=False,response=response));return
        registration=response['result'];err=pose_error(np.array(registration['base_from_camera']),owner.camera_ground_truth())
        owner.registration=registration
        atomic_json(args.out/'calibration-diagnostics.json',dict(independent_camera_error_mm=err[0]*1000,
                     independent_camera_angle_deg=err[1],residuals=registration['residuals'],commands=owner.commands))
        sim=owner.sim;d=sim.data
        snapshot={f:getattr(d,f).copy() for f in ('qpos','qvel','ctrl')}
        snapshot.update(time=float(d.time),qseed=sim.qseed.copy(),friction=sim.model.geom_friction.copy(),rgba=sim.model.geom_rgba.copy())
        trials=[]
        parameters=([(correction,angle) for correction in [(0,-.004,.004),(0,-.008,.004)] for angle in [-.080,-.090,-.095]]
                    if args.fine_grip else [(c,-.174) for c in [(0,0,0),(0,0,.004),(0,0,.008),(0,-.004,.004),(0,-.008,.008)]])
        if args.single_candidate:parameters=[((0,-.004,.004),-.080)]
        for index,(correction,angle) in enumerate(parameters):
            trials.append(run_episode(owner,robot,snapshot,args.out/f'train-{index:02d}',correction=correction,close_angle=angle,
                                      release_height=args.release_height,vertical_withdrawal=args.vertical_withdrawal))
            if args.probe:break
        candidates=[t for t in trials if t['eligible_for_success_demonstrations']]
        validation=[];controls=[]
        if candidates:
            best=min(candidates,key=lambda t:t['hold_slip_mm'])
            for i,offset in enumerate(VALIDATION_OFFSETS):
                validation.append(run_episode(owner,robot,snapshot,args.out/f'validation-{i:02d}',
                                  correction=best['correction_m'],close_angle=best['close_angle_rad'],release_height=args.release_height,
                                  vertical_withdrawal=args.vertical_withdrawal,offset=offset,seed=100+i,
                                  evaluation_role='development_regression'))
        if not args.probe:
            selected=(best if candidates else max(trials,key=lambda t:(t['completed'],t.get('minimum_hold_rest_clearance_mm') or -1)))
            for fault in FAULTS:
                controls.append(run_episode(owner,robot,snapshot,args.out/('control-'+fault),fault=fault,video=True,
                                           correction=selected['correction_m'],close_angle=selected['close_angle_rad'],release_height=args.release_height,
                                           vertical_withdrawal=args.vertical_withdrawal,evaluation_role='fault_control'))
        promotion_reasons=qualification_reasons(best if candidates else None,validation,controls)
        qualified=not promotion_reasons
        if qualified:atomic_json(args.out/'policy.json',dict(simulation_only=True,hardware_released=False,
                       solver_sensitivity_pending=True,stage='G4_pusher_pickup',type='parameter_search_visual_pickup',
                       correction_m=best['correction_m'],close_angle_rad=best['close_angle_rad'],release_height_m=args.release_height,
                       vertical_withdrawal=args.vertical_withdrawal))
        atomic_json(args.out/'result.json',dict(status='SIMULATION_STAGE_QUALIFIED' if qualified else 'TRAINING_STAGE_NOT_QUALIFIED',
                     trials=trials,validation=validation,controls=controls,policy_exported=qualified,
                     qualification_failure_reasons=promotion_reasons,
                     successful_demonstrations=sum(t['eligible_for_success_demonstrations'] for t in trials),
                     full_planter_success=False,physical_success=False,physical_motor_writes=0))
    finally:owner.camera.renderer.close()


if __name__=='__main__':main()
