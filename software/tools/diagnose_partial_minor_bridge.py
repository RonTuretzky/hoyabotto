"""Offline prepared-pose support test; this does not execute a folding approach.

An earlier full run supplies the settled carton pose and the initial left-arm
pose. The selected static candidate supplies a right-arm pose and a different
initial paddle grip. Both minor angles are explicitly prepared at 75 degrees.
After preparation only the original twelve robot actuators may hold the pose.
The carton, flaps and paddle retain their ordinary passive dynamics.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from carton.folding_material import CartonMaterial
from carton.folding_paddle import PaddleFoldingSimulation, PaddleSpec
from carton.folding_sim import JOINTS
from carton.folding_solver import FoldingSolver
from carton.folding_station import FoldingStation


def run(args):
    out=Path(args.out).resolve()
    if out.exists():raise ValueError('Preserve previous runs: output must be new')
    prepared=Path(args.prepared_run).resolve()
    candidate_file=Path(args.candidates).resolve()
    candidates=json.loads(candidate_file.read_text())['rows']
    candidate=next(row for row in candidates if not row['bad'])
    frames=json.loads((prepared/'folding-frames.json').read_text())
    held=next(frame for frame in frames if frame['label']=='Verify both short flaps held')
    spec=PaddleSpec(grasp_x_m=candidate['grasp_mm']/1000,
                    grasp_yaw_degrees=candidate['grip_yaw_deg'])
    sim=PaddleFoldingSimulation(Path(args.simulation_root),out,
        station=FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),
                               backup_table_marker_xy=(.45,.70)),
        material=CartonMaterial(),solver=FoldingSolver.friction(),paddle=spec,
        width=960,height=540,offset=(0,.0757925946355),yaw=math.pi/6,
        initial_right_roll=1.5)
    if sim.model.nu!=12 or sim.model.neq:
        raise ValueError('Only the twelve robot actuators, without equality constraints, are allowed')
    # Deliberate test-fixture initialization, not a claimed robot operation.
    # No qpos assignments are made after the preparation below.
    sim.data.qpos[:]=held['qpos']
    sim.data.qpos[sim.arm_indices['left']]=np.asarray(frames[0]['qpos'])[sim.arm_indices['left']]
    sim.data.qpos[sim.arm_indices['right'][:5]]=candidate['q']
    sim.data.qpos[sim.arm_indices['right'][5]]=-.12127145799576845
    for flap in ('short_left','short_right'):
        sim.data.qpos[sim.model.joint(flap+'_hinge').qposadr[0]]=math.radians(75)
    sim.data.qpos[sim.model.joint('long_near_hinge').qposadr[0]]=math.radians(-37.66)
    mujoco.mj_kinematics(sim.model,sim.data)
    grip=sim.data.body('right_gripper_link');rotation=grip.xmat.reshape(3,3)
    adr=sim.model.joint('paddle_free').qposadr[0]
    sim.data.qpos[adr:adr+3]=grip.xpos+rotation@spec.grip_origin
    mujoco.mju_mat2Quat(sim.data.qpos[adr+3:adr+7],(rotation@spec.grip_rotation).ravel())
    for side,indices in sim.arm_indices.items():
        for name,index in zip(JOINTS,indices):
            lo,hi=sim.model.joint(side+'_'+name).range
            if not lo<=sim.data.qpos[index]<=hi:
                raise ValueError('Prepared robot pose exceeds an original joint limit')
            sim.data.ctrl[sim.model.actuator(side+'_'+name).id]=sim.data.qpos[index]
        sim.data.ctrl[sim.model.actuator(side+'_gripper').id]=-.17
    sim.data.qvel[:]=0;sim.data.time=0
    mujoco.mj_forward(sim.model,sim.data)
    sim.validate_initial_robot_clearance()
    sim.initial_flaps_degrees=sim.truth_angles()
    sim.box_origin=sim.data.body('carton').xpos.copy()
    sim.box_rotation=sim.data.body('carton').xmat.reshape(3,3).copy()
    sim.capture('PREPARED POSE: approach and full folding untested')
    error=None;events=[];contacts=[]
    try:
        for step in range(30):
            event=sim.move({},.1,'Prepared partial-minor support, not completed folding')
            events.append(event)
            if event['bad_penetration_mm']>1:raise ValueError('Forbidden contact during support')
            forces={name:0. for name in ('short_left','short_right')}
            for index,contact in enumerate(sim.data.contact):
                a,b=sim.model.geom(contact.geom1).name,sim.model.geom(contact.geom2).name
                for name in forces:
                    if name+'_cardboard' in (a,b) and any(g.startswith('right_paddle_contact_') for g in (a,b)):
                        force=np.zeros(6);mujoco.mj_contactForce(sim.model,sim.data,index,force)
                        forces[name]+=float(force[0])
            contacts.append({'time':float(sim.data.time),'paddle_normal_force_N':forces})
    except ValueError as exc:error=str(exc)
    sim.capture('End of prepared support test - full task remains unverified')
    physics=sim.save('support')
    angle_hold=bool(events) and all(70<=lo<=hi<=80 for event in events
        for name,(lo,hi) in event['flap_angle_extrema_degrees'].items() if name.startswith('short_'))
    both_contacts=bool(contacts) and all(all(force>0 for force in row['paddle_normal_force_N'].values())
        for row in contacts if row['time']>=.3)
    report={'simulation_only':True,'prepared_pose_only':True,'full_task_complete':False,
        'folding_approach_executed':False,'tape_applied':False,'hands_off_retention_tested':False,
        'prepared_support_passed':bool(error is None and sim.data.time>=2.999 and angle_hold and both_contacts),
        'error':error,'duration_s':float(sim.data.time),'candidate':candidate,
        'angles':sim.truth_angles(),'motion':sim.motion_stats,'physics':physics,
        'force_samples':contacts,'material':sim.material.report(),'station':sim.station.report(),
        'assumptions':'Original unmeasured station and material values; stock rigid fingers. No approach, pickup, major fold or physical validation.',
        'initialization':'Carton pose copied from prior run; left arm parked at its prior initial pose; minor angles set to 75 deg, near major to -37.66 deg, right arm and passive tool prepared from static candidate before dynamics.',
        'sources':{str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [
            Path(__file__).resolve(),candidate_file,prepared/'folding-frames.json',
            *sorted((Path(__file__).resolve().parents[1]/'carton').glob('folding*.py'))]}}
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({key:report[key] for key in ('prepared_support_passed','full_task_complete','error','duration_s','angles','motion')},indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-root',required=True)
    parser.add_argument('--prepared-run',required=True)
    parser.add_argument('--candidates',required=True)
    parser.add_argument('--out',required=True)
    run(parser.parse_args())
