"""Rendered AprilTag calibration with the production Gemma adapters and SO101 FK.

Only MuJoCo and local model assets are used. No real Robot client, credentials,
serial connection or physical camera can be instantiated by this script.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from simulate_gemma_tags import CameraSimulation
from carton.servo.common import atomic_json, digest
from farm.kinematics.assets import verified_model
from farm.kinematics.lerobot import LeRobotSO101, pose_error
from farm.perception.gemma_tags import TagObserver, TagRobot
from farm.perception.gemma_calibration import CalibrationRobot

# Decoded tag axes follow square_points() in tag_geometry.py. For the printed
# grid in add_marker(), CAD_from_decoded_tag has R=diag(-1,+1,-1). The physical
# marker plane is at paddle CAD [135,0,7.45] mm, handle at [30,0,3] mm.
# This is a declared SIMULATED mount offset, not a measured physical mounting.
TAG3_FROM_HANDLE_M = np.array([.105, 0, .00445, 1])


class RenderedOwner:
    """API-compatible simulation owner; actual motion comes from MuJoCo dynamics.

    Sixteen named telemetry rows exercise the production transport. Only one
    arm is modeled. Nonmodeled joints and STS load/status bytes are fixtures,
    not a simulation of the physical servo electronics or USB link.
    """
    def __init__(self, simulation_root, model_directory, out, width=1920):
        self.camera = CameraSimulation(simulation_root, out/'scene', metric=True, width=width, height=width*3//4)
        self.sim, self.model, self.data = self.camera.sim, self.camera.model, self.camera.data
        self.fk = LeRobotSO101(model_directory)
        self.urdf, self.manifest = verified_model(model_directory)
        self.model_directory = str(model_directory)
        self.joints = self.fk.names + ['gripper']
        self.names = [f'{arm}_arm_{n}' for arm in ('left', 'right') for n in self.joints]
        self.names += ['head_motor_1', 'head_motor_2', 'base_left_wheel', 'base_right_wheel']
        self.raw_ranges = {n: dict(min_ticks=0, max_ticks=4096) for n in self.names}
        for n in self.fk.names:
            radius = math.floor(min(abs(self.fk.ranges[n])) * 4096/360)
            self.raw_ranges['right_arm_'+n] = dict(min_ticks=2048-radius, max_ticks=2048+radius)
        self.ranges = {n: dict(min_ticks=v['min_ticks']+4, max_ticks=v['max_ticks']-4, margin_ticks=4)
                       for n,v in self.raw_ranges.items()}
        self.now, self.started = 1000., 999.
        self.enabled = set()
        self.stopped = False
        self.command_id = self.motor_writes = 0
        self.commands = []
        self.max_arm_table_penetration_mm = 0
        geometry = copy.deepcopy(self.camera.tagged.observer.geometry.config)
        geometry['tags']['2']['mount'] = {'arm':'right','body':'fixed_gripper_housing','source':'Rigid MJCF marker body attached to gripper_link'}
        self.tagged = TagRobot(self, observer=TagObserver(clock=self.clock, geometry=geometry))
        self.tagged.catalog()
        self.out = out

    def clock(self):
        return self.now

    def positions(self):
        q = dict.fromkeys(self.names, 2048)
        q.update({f'right_arm_{n}':int(round(2048+self.data.qpos[i]*4096/(2*math.pi))) for i,n in enumerate(self.joints)})
        return q

    def catalog(self):
        return self.camera.catalog()

    def call(self, name, args, request_id=None):
        self.now += .01
        if name == 'robot_get_cameras':
            payload = self.camera.call(name, args)
            for frame in payload['images']:
                frame['captured_at'] = self.now
            return payload
        if name == 'robot_get_state':
            r = dict(cached=False, time=self.now, commandable_ranges=self.ranges,
                     raw_calibration_ranges=self.raw_ranges, motors=[dict(name=n, Present_Position=v,
                     Present_Velocity=0, Present_Load=0, Moving=0, Status=0,
                     Torque_Enable=int(n in self.enabled), captured_at=self.now-.001)
                     for n,v in self.positions().items()])
        elif name == 'robot_get_execution':
            r = dict(time=self.now, started=self.started, control_mode='direct_joint', ok=not self.stopped,
                     phase='stopped' if self.stopped else ('holding' if self.enabled else 'idle'),
                     operator_armed=not self.stopped, stop_latched=self.stopped, enabled_motors=sorted(self.enabled),
                     accepted=self.command_id, completed=self.command_id, motor_writes=self.motor_writes,
                     lease_remaining=15.)
        elif name == 'robot_get_capabilities':
            r = dict(motion_units='raw_encoder_ticks', motion_ready=not self.stopped, joint_blockers={}, blockers=[])
        elif name == 'robot_get_arm_pose':
            r = dict(configuration={'config':{'arm':'right','mapping':'feetech_degrees_v1',
                    'calibration_sha256':digest(self.raw_ranges)},
                    'model_assets':{'verified':True,'revision':self.manifest['revision']}})
        elif name == 'robot_stop':
            self.stopped = True
            self.enabled.clear()
            r = dict(release_confirmed=True, simulation_only=True)
        elif name == 'robot_set_motor_enable':
            if self.stopped:
                return {'ok':False,'result':{'error':'Simulation STOP latched'}}
            if any(n not in {'right_arm_'+j for j in self.fk.names} for n in args['names']):
                raise ValueError('Only modeled right positioning joints may be enabled')
            if args['enabled']:
                self.enabled.update(args['names'])
                self.data.ctrl[:5] = self.data.qpos[:5]
            else:
                self.enabled.difference_update(args['names'])
            r = self.ack({n:int(args['enabled']) for n in args['names']})
        elif name == 'robot_move_motor_targets':
            if self.stopped or len(args['positions']) != 1 or not set(args['positions']) <= self.enabled:
                return {'ok':False,'result':{'error':'Simulation owner refuses movement'}}
            n, goal = next(iter(args['positions'].items()))
            index = self.joints.index(n.removeprefix('right_arm_'))
            target = self.data.ctrl.copy()
            target[index] = (goal-2048)*2*math.pi/4096
            if not self.model.jnt_range[index,0] <= target[index] <= self.model.jnt_range[index,1]:
                raise ValueError('Simulation target outside actual URDF joint limits')
            stats = self.sim.advance(args['duration_s']+.6, f'Calibration {n} -> {goal}', target)
            self.now += args['duration_s']+.6
            self.max_arm_table_penetration_mm = max(self.max_arm_table_penetration_mm,stats['max_arm_table_penetration_mm'])
            if stats['max_arm_table_penetration_mm'] >= 1:
                raise ValueError('Calibration produced robot/table contact')
            measured = self.positions()[n]
            self.commands.append(dict(joint=n,target=goal,measured=measured,sim_time_s=float(self.data.time)))
            if abs(measured-goal)>5:
                return {'ok':False,'result':{'error':f'Actual simulated encoder {measured} missed target {goal}'}}
            r = self.ack({n:measured})
        else:
            raise ValueError(f'No hardware backend exists: {name}')
        return {'ok':True,'result':r,'simulation_only':True}

    def ack(self, readbacks):
        self.command_id += 1
        self.motor_writes += 1
        return dict(accepted=True, completed=True, command_id=self.command_id,
                    owner_started=self.started, owner_status_time=self.now, readbacks=readbacks)

    def camera_ground_truth(self):
        # Evaluation only; never supplied to the detector, capture or fitter.
        out = np.eye(4)
        out[:3,:3] = self.data.cam_xmat[self.camera.camera].reshape(3,3) @ np.diag([1,-1,-1])
        out[:3,3] = self.data.cam_xpos[self.camera.camera]
        return out


def use_registration(owner, robot, out):
    """Locate from production registered-tag output; truth is evaluation only.

    The existing simulator's robot-only IK and contact dynamics execute the
    target. No fixed world handle location is passed to IK. Table orientation,
    approach direction and tag-to-handle mount are declared scene assumptions.
    """
    from PIL import Image, ImageDraw
    sim, model, data = owner.sim, owner.model, owner.data
    # Independent trials start at the same calibrated pose in the same stream.
    snapshot = dict(qpos=data.qpos.copy(), qvel=data.qvel.copy(), ctrl=data.ctrl.copy(),
                    time=float(data.time), qseed=sim.qseed.copy())
    frames, trials = [], []

    def capture(label):
        im = Image.fromarray(owner.camera.render()).resize((960,720))
        draw = ImageDraw.Draw(im)
        draw.rectangle((0,0,960,48),fill='white')
        draw.text((10,7),'SIMULATION ONLY | rendered tags -> fitted transform -> handle -> physics',fill='black')
        draw.text((10,27),label,fill='black')
        frames.append(im)

    for name, offset, close in [('nominal',[0,0,0],True),
                                 ('moved_paddle',[.005,.015,0],True),
                                 ('open_jaw_negative',[0,0,0],False)]:
        for field in ('qpos','qvel','ctrl'):
            getattr(data,field)[:] = snapshot[field]
        data.time = snapshot['time']
        data.qacc_warmstart[:] = 0
        sim.qseed = snapshot['qseed'].copy()
        address = model.jnt_qposadr[model.joint('paddle_free').id]
        data.qpos[address:address+3] += offset
        mujoco.mj_forward(model,data)
        sim.advance(.25,'Settle independent simulated trial')
        owner.now += .25
        observed = robot.call('robot_get_registered_tags',{})
        atomic_json(out/f'{name}-observation.json',{k:v for k,v in observed.items() if k!='images'})
        if not observed['ok']:
            raise ValueError(f'Registered tag read refused: {observed}')
        tag = next(t for t in observed['result']['tags'] if t['tag_id']==3)
        if tag['orientation_ambiguous'] or tag['arm_base_from_tag'] is None:
            raise ValueError('Paddle orientation is ambiguous; no grasp target')
        target = (np.array(tag['arm_base_from_tag']) @ TAG3_FROM_HANDLE_M)[:3]
        # Truth enters only this evaluator, never target construction or IK.
        truth = data.site('handle_center').xpos.copy()
        trial = dict(name=name, paddle_displacement_m=offset, close_jaw=close,
                     target_from_registered_tag_m=target.tolist(),
                     independent_true_handle_m=truth.tolist(),
                     handle_estimation_error_mm=float(np.linalg.norm(target-truth)*1000), steps=[])
        capture(f'{name}: target from calibrated AprilTag 3')
        try:
            for label, point, jaw, seconds in [
                ('approach_above',target+[-.03,0,.08],.55,1.5),
                ('lower_outside_edge',target+[-.03,0,0],.55,1.5),
                ('approach_handle',target,.55,1.5),
                ('close_gripper',None,-.174 if close else .55,1.5),
                ('lift',target+[0,0,.08],None,2.),
                ('hold',None,None,2.),
                ('lower_to_table',target,None,2.),
                ('release',None,.55,1.5),
                ('withdraw',target+[-.03,0,0],.55,1.5)]:
                command = data.ctrl.copy()
                if point is not None:
                    command[:5] = sim.ik(point)
                if jaw is not None:
                    command[5] = jaw
                stats = sim.advance(seconds,label,command)
                owner.now += seconds
                stats['label'] = label
                trial['steps'].append(stats)
                capture(f'{name}: {label} | clearance {stats["state"]["paddle_bottom_clearance_mm"]:.1f} mm')
                if stats['max_arm_table_penetration_mm'] >= 1:
                    raise ValueError(f'Arm/table penetration during {label}')
            lift, hold = [s for s in trial['steps'] if s['label'] in ('lift','hold')]
            def held(s):
                c=s['state']['contacts']['paddle_contact_geoms']
                return (s['state']['paddle_bottom_clearance_mm']>20 and 'table' not in c
                        and any('moving_jaw' in n for n in c) and any('wrist_roll_follower' in n for n in c))
            trial['lift_and_hold_verified'] = held(lift) and held(hold) and min(hold['clearance_samples_mm'])>20
            final = trial['steps'][-1]['state']
            trial['release_verified'] = final['contacts']['paddle_contact_geoms']==['table'] and abs(final['paddle_bottom_clearance_mm'])<2
            trial['passed'] = (trial['lift_and_hold_verified'] if close else not trial['lift_and_hold_verified']) and trial['release_verified']
        except (ValueError,RuntimeError) as exc:
            trial.update(passed=False,error=str(exc))
        trials.append(trial)
        atomic_json(out/f'{name}-physics.json',trial)
        print(json.dumps({k:v for k,v in trial.items() if k!='steps'}),flush=True)
    frames[0].save(out/'use-before.png')
    frames[-1].save(out/'use-after.png')
    frames[0].save(out/'calibrated-paddle-use.gif',save_all=True,append_images=frames[1:],duration=400,loop=0)
    return {'trials':trials,'passed':all(t['passed'] for t in trials),
            'target_source':'robot_get_registered_tags + declared simulated tag-to-handle CAD offset',
            'planner':'Existing PaddleSimulation robot-only IK; target is from pixels, not object ground truth',
            'physical_motor_writes':0,'physical_grasp_validated':False,
            'limitations':['Illustrative station/camera geometry, assumed friction 0.8 and paddle mass 30g',
                           'Encoder quantization modeled; USB, servo loads, torque release and watchdog electronics are fixtures',
                           'Ground truth used only to evaluate error and contact, not to choose targets',
                           'Simulated mount offset must not be copied to the physical paddle without measurement']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-root',type=Path,required=True)
    parser.add_argument('--model-directory',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--width',type=int,default=1920)
    parser.add_argument('--initial-only',action='store_true')
    args=parser.parse_args()
    if args.out.exists():parser.error('Preserve previous evidence: choose a new output directory')
    args.out.mkdir(parents=True)
    owner=RenderedOwner(args.simulation_root.resolve(),args.model_directory.resolve(),args.out,args.width)
    try:
        initial=owner.tagged.call('robot_get_tags',{'cameras':['sim']})
        atomic_json(args.out/'initial-observation.json',{k:v for k,v in initial.items() if k!='images'})
        from PIL import Image
        Image.fromarray(owner.camera.render()).save(args.out/'initial.png')
        row=initial.get('result',{}).get('observations',{}).get('sim',{})
        print(json.dumps({'tags':[(t['tag_id'],t['status']) for t in row.get('tags',[])],
                          'poses':[{k:t.get(k) for k in ('tag_id','orientation_ambiguous','reprojection_rms_px','center_camera_mm')} for t in (row.get('pose_3d') or {}).get('tags',[])],
                          'initial_actual_ticks':owner.positions()}),flush=True)
        if args.initial_only:return
        cfg=dict(arm='right',joints=['shoulder_pan','wrist_flex'],camera='sim',
                 model_directory=str(args.model_directory.resolve()),lock_file=str(args.out/'simulation.lock'))
        try:
            cfg['schema']=1
            config_path=args.out/'.private/tag-calibration.json'
            atomic_json(config_path,cfg)
            robot=CalibrationRobot(owner.tagged,config_path,clock=owner.clock)
            robot.catalog()
            response=robot.call('robot_calibrate_tags',{'mode':'registration'})
            if not response['ok']:raise ValueError(response)
            registration=response['result']
            report={'status':registration['status'],'rendered_camera':True,'simulation_only':True,
                    'physical_motor_writes':0,'physical_grasp_validated':False,'commands':owner.commands,
                    'max_arm_table_penetration_mm':owner.max_arm_table_penetration_mm,
                    'registration':registration,
                    'simulation_source_sha256':hashlib.sha256((args.simulation_root/'paddle_sim.py').read_bytes()).hexdigest(),
                    'robot_model':owner.fk.provenance()}
            if registration['status']=='REGISTRATION_VALIDATED':
                error=pose_error(np.array(registration['base_from_camera']),owner.camera_ground_truth())
                report['camera_transform_error']={'position_mm':error[0]*1000,'orientation_degrees':error[1]}
                report['use']=use_registration(owner,robot,args.out)
                report['status']='SIMULATED_CALIBRATION_AND_USE_PASSED' if report['use']['passed'] else 'SIMULATED_USE_FAILED'
            atomic_json(args.out/'result.json',report)
            print(json.dumps({k:v for k,v in report.items() if k not in ('commands','registration','use')},indent=2),flush=True)
        except Exception as exc:
            atomic_json(args.out/'error.json',{'error':str(exc),'simulation_only':True,'commands':owner.commands,'physical_motor_writes':0})
            raise
    finally:
        owner.camera.renderer.close()


if __name__=='__main__':main()
