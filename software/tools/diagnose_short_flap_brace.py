"""Offline minor-flap pinch and paddle-bridge approach; no hardware adapter.

The left hand grasps the left minor without pulling the near major into the
forearm's path. The right hand then attempts a partial fold. No box or flap
state is set after initialization. A partial support pass is not full closure.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from carton.folding_grasp import GraspIKMixin, panel_grasp_evidence
from carton.folding_sim import FoldingSimulation
from carton.folding_paddle import PaddleFoldingSimulation, PaddleSpec
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_solver import FoldingSolver
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_diagonal import DiagonalFoldingController, contact_point


class PinchSimulation(GraspIKMixin, FoldingSimulation):
    pass


class PinchPaddleSimulation(GraspIKMixin, PaddleFoldingSimulation):
    pass


def run(args):
    out = Path(args.out).resolve()
    if out.exists():
        raise ValueError('Preserve prior experiments; use a new output directory')
    out.mkdir(parents=True)
    source_paths = [Path(__file__).resolve(), *sorted((Path(__file__).resolve().parents[1]/'carton').glob('folding*.py')),
                    Path(__file__).resolve().with_name('simulate_bimanual_folding.py')]
    snapshots = {str(p): p.read_bytes() for p in source_paths}
    (out/'sources').mkdir()
    for source, content in snapshots.items():
        (out/'sources'/Path(source).name).write_bytes(content)
    # Explicit proposed marker move; the 122.77 mm grip obscures the old 135 mm
    # tags. Physical mounting is not changed or claimed to be calibrated.
    from carton.folding_tool_tags import PADDLE_TAGS
    if args.tool == 'paddle':
        for tag, (name, pos, axes) in list(PADDLE_TAGS.items()):
            PADDLE_TAGS[tag] = (name, [.180, pos[1], pos[2]], axes)
    if args.floor_marker_x is not None:
        from carton.folding_markers import BOX_MARKERS
        BOX_MARKERS[24]=('box_tag_floor',[args.floor_marker_x,0,.0038],[1,0,0,0,1,0])
    if args.center_floor_marker:
        from carton.folding_markers import BOX_MARKERS
        BOX_MARKERS[25]=('box_tag_floor_center',[0,0,.0038],[1,0,0,0,1,0])
    from tools.simulate_bimanual_folding import PixelPort
    if getattr(args, 'privileged_near_angle', False):
        from carton.folding_mechanics_probe import PrivilegedNearAngleProbe
        PixelPort = PrivilegedNearAngleProbe
    cls = PinchPaddleSimulation if args.tool == 'paddle' else PinchSimulation
    options = {'paddle': PaddleSpec(grasp_x_m=.12276625897181499,
                grasp_yaw_degrees=22.54998861948059)} if args.tool == 'paddle' else {}
    yaw=math.radians(args.carton_yaw_degrees)
    offset_y=-.1515+.01+.379/2*abs(math.sin(yaw))+.283/2*abs(math.cos(yaw))
    if args.park_back:
        options['initial_arm_targets']={'left':[-.20,-.18,.30],'right':[.20,-.18,.30]}
    sim = cls(Path(args.simulation_root), out,
        station=FoldingStation(.06, .15, .01, table_marker_xy=(-.5, .55),
                               backup_table_marker_xy=(.45, .70)),
        material=CartonMaterial(), width=args.width, height=args.height,
        offset=(args.carton_offset_x, offset_y), yaw=yaw, initial_right_roll=1.5,
        initial_flaps={'short_left':.1,'short_right':.1,'long_far':-.1,
                       'long_near':math.radians(args.near_open_degrees)},
        solver=FoldingSolver.friction(), **options)
    sim.capture_images = args.video
    # Always preserve full timestamped states; --video additionally renders images.
    port = PixelPort(sim, record=True, camera='station', seed=args.seed)
    initial_right_joints=sim.data.qpos[sim.arm_indices['right'][:5]].copy()
    controller = DiagonalFoldingController(port, rear_cart=True)
    result = {'simulation_only': True, 'full_task_complete': False, 'success': False,
              'privileged_mechanics_probe':getattr(args, 'privileged_near_angle', False),
              'end_to_end_perception_validated':False,
              'hardware_commands': False, 'configuration': vars(args), 'grasp_checks': [],
              'near_flap_initialization_is_preparation':True,'near_opening_action_executed':False,
              'proposed_tool_tag_x_m': .180 if args.tool == 'paddle' else None,
              'proposed_floor_tag_24_x_m':args.floor_marker_x,
              'proposed_center_floor_tag_25':args.center_floor_marker,
              'stage': 'left minor pinch'}

    def grasp():
        check = panel_grasp_evidence(sim.model, sim.data, 'left', 'short_left')
        result['grasp_checks'].append(check)
        if not check['opposing_faces']:
            raise ValueError('Lost or absent opposing-face left-minor pinch')
        return check

    def pose(theta):
        point, _ = contact_point(theta, 0, -1, args.along, args.radius, 0, args.clearance)
        normal = np.array([-math.cos(theta), 0, math.sin(theta)])
        radial = np.array([math.sin(theta), 0, math.cos(theta)])
        if args.normal_only:
            return point, {'direction': (args.axis_sign*normal).tolist(), 'local_axis': [1, 0, 0]}
        return point, {'direction': radial.tolist(), 'tangent': (args.axis_sign*normal).tolist()}

    try:
        sim.capture('Initial free resistant carton')
        sim.move({}, .4, 'Settle', capture=False)
        reading = controller.sense('Register left minor for bracing grip')
        start = math.radians(reading['angles']['short_left']['degrees'])
        point, orientation = pose(start)
        pre = point + [0, 0, args.pre_height]
        port.set_grippers({'left': .6}, .4, 'Open left claw above minor')
        world = controller.box[:3, :3]@pre + controller.box[:3, 3]
        ori = {k: (controller.box[:3, :3]@v).tolist() if k in ('direction','tangent') else v
               for k, v in orientation.items()}
        q, error = sim.ik('left', world, ori)
        if error > .008:
            raise ValueError(f'Left-minor pregrasp IK misses {error*1000:.2f} mm')
        planner = JointPathPlanner(sim, 'left', clearance=.006,
                                   allowed_flaps=('short_left_cardboard',))
        path = planner.plan(q)
        execute_path(sim, 'left', path, 'Approach over left minor', capture=True)
        for u in np.linspace(.05, 1, 20):
            controller.move({'left': (1-u)*pre+u*point}, .15,
                            'Insert left claw around minor', orientation)
        port.set_grippers({'left': -.17}, .6, 'Pinch left minor')
        port.move_arms({}, .5, 'Verify left minor bracing grip', None)
        grasp()
        result['pinch_verified'] = {'angles': sim.truth_angles(), 'motion': dict(sim.motion_stats)}
        if args.prepare_near_degrees is not None:
            from carton.folding_retention import open_near_for_transfer
            result['stage'] = 'Physically open near flap before minor folds'
            result['near_preparation'] = open_near_for_transfer(sim, controller,
                args.prepare_near_degrees, capture=True)
            result['near_opening_action_executed'] = True
        if getattr(args, 'open_shorts_first', False):
            if args.tool != 'claws' or args.prepare_near_degrees is None or args.fold_right:
                raise ValueError('Major-first opening requires bare claws, prepared near flap and no short-fold prefix')
            from carton.folding_short_opening import open_shorts_before_majors
            from carton.folding_cascade import press_near_over_short
            result['stage'] = 'Physically open short flaps before folding near major'
            result['short_opening'] = open_shorts_before_majors(sim, controller,
                capture=True, target_degrees=args.open_short_angle)
            result['stage'] = 'Fold near major with shorts physically opened outward'
            near_key = 'near_major_transfer' if args.near_hold_degrees == 90 else 'near_major_clearance'
            result[near_key] = press_near_over_short(sim, controller,
                target_degrees=args.near_hold_degrees, capture=True, majors_first=True, along=args.near_press_along,
                pre_out=args.near_pre_out, pre_up=args.near_pre_up)
            if getattr(args, 'far_after_near', False):
                from carton.folding_far_contact import fold_far_from_edge
                if getattr(args, 'additional_far_view', False):
                    from carton.folding_additional_view import AdditionalViewPixelPort, AdditionalViewConfiguration
                    # Explicit hypothetical second calibrated camera, activated
                    # only for the far-contact stage. Original station startup
                    # and physical prefix remain intact.
                    port = AdditionalViewPixelPort(port,
                        configuration=AdditionalViewConfiguration(
                            assumption_id='offline:front-additional-v1', clock_id='offline:simulation',
                            required_flaps=('long_near','long_far')), seed=args.seed)
                    controller.port = port
                result['stage'] = 'Fold far major while left claw retains near major'
                far_key = 'far_major_transfer' if args.far_hold_degrees == 90 else 'far_major_clearance'
                result[far_key] = fold_far_from_edge(sim, controller, capture=True,
                    expected_near_degrees=args.near_hold_degrees, target_degrees=args.far_hold_degrees,
                    slide_left_to=-.08 if args.near_hold_degrees == 90 else None,
                    release_after=args.release_far_after, contact_profile=args.far_contact_profile,
                    central_normal_extra_m=args.far_normal_extra, gripper_opening=args.far_gripper_opening,
                    central_startup_lift_m=args.far_startup_lift)
                if getattr(args, 'release_near_after_far', False):
                    from carton.folding_partial_release import release_near_after_passive_far
                    result['stage'] = 'Release near holder after verified passive far hold'
                    result['partial_major_release'] = release_near_after_passive_far(sim, controller, capture=True)
                if getattr(args, 'probe_shorts_after_release', False):
                    from carton.folding_additional_view import AdditionalViewPixelPort, AdditionalViewConfiguration
                    from carton.folding_partial_short_probe import probe_shorts_against_passive_majors
                    # This new stage explicitly assumes the additional front
                    # camera and independently identified open-short planes.
                    port = AdditionalViewPixelPort(port,
                        configuration=AdditionalViewConfiguration(
                            assumption_id='offline:front-open-short-regrasp-v1', clock_id='offline:simulation',
                            required_flaps=('long_near','long_far','short_left','short_right'),
                            observe_open_shorts=True), seed=args.seed)
                    controller.port = port
                    result['stage'] = 'Probe paired short folds against freely passive majors'
                    result['partial_short_probe'] = probe_shorts_against_passive_majors(
                        sim, controller, capture=True, target_degrees=10.)
        if args.fold_right:
            result['stage'] = 'right minor fold with left-minor brace'
            right_reading=controller.sense('Register the moved carton before right-minor approach')
            if args.tool == 'paddle':
                from carton.folding_bridge import fold_right_partial
                if args.scan_tool and right_reading.get('paddle') is None:
                    base_q=sim.data.qpos[sim.arm_indices['right'][:5]].copy()
                    result['tool_view_attempts']=[]
                    for delta in (-.5,.5,-1.,1.,-1.5):
                        target=base_q.copy();target[4]+=delta
                        planner=JointPathPlanner(sim,'right',clearance=.006)
                        try:
                            path=planner.plan(target)
                        except ValueError as exc:
                            result['tool_view_attempts'].append({'roll_delta':delta,'planning_error':str(exc)})
                            continue
                        execute_path(sim,'right',path,'Turn paddle to observe its markers',capture=True)
                        grasp()
                        reading=controller.sense('Observe paddle after wrist turn')
                        visible=reading.get('paddle') is not None
                        result['tool_view_attempts'].append({'roll_delta':delta,'visible':visible,'seq':reading['seq']})
                        if visible:break
                    else:raise ValueError('No fresh paddle marker after collision-checked wrist views')
                result['right_partial'] = fold_right_partial(sim, controller,
                    capture=True, brace_flap='short_left',
                    contact_x=.208 if args.paddle_contact=='tip' else .060,
                    axis_yaw_degrees=args.paddle_axis_yaw,axis_mode=args.paddle_axis_mode,
                    normal_offset=args.normal_offset,along_travel=args.paddle_along_travel,
                    blend_yaw=args.paddle_blend_yaw,end_degrees=args.paddle_end_degrees)
            else:
                observed=right_reading['angles'].get('short_right')
                if observed is None:
                    raise ValueError('Fresh right-minor observation required')
                right_start=math.radians(observed['degrees'])
                for i, theta in enumerate(np.linspace(right_start, math.pi/2, 41)):
                    radius = .14-.04*max(0, (theta-math.pi/4)/(math.pi/4))
                    p, ori = contact_point(theta, 0, 1, -.11, radius, 1.5, .012)
                    if i == 0:
                        outside = p+[args.right_pre_out, 0, args.right_pre_up]
                        q, error = sim.ik('right', controller.box[:3, :3]@outside+controller.box[:3, 3],
                            {'direction': (controller.box[:3, :3]@ori['direction']).tolist()})
                        if error > .008:
                            raise ValueError(f'Right-minor pregrasp IK misses {error*1000:.2f} mm')
                        path = JointPathPlanner(sim, 'right', clearance=.006,
                            allowed_flaps=('short_right_cardboard',)).plan(q)
                        execute_path(sim, 'right', path, 'Approach right minor', capture=True)
                        grasp()
                        for u in np.linspace(.1, 1, 10):
                            controller.move({'right': (1-u)*outside+u*p}, .15, 'Contact right minor', ori)
                            grasp()
                    controller.move({'right': p}, .3, f'Fold right minor {math.degrees(theta):.1f}', ori)
                    grasp()
                    if i%5 == 0:
                        controller.sense('Observe fold with left-minor brace')
                result['right_partial'] = {'angles': sim.truth_angles(), 'motion': dict(sim.motion_stats)}
            if args.press_left:
                from carton.folding_transfers import fold_second_short
                result['stage']='Release minor pinch and press left short'
                result['both_shorts_held']=fold_second_short(sim,controller,capture=True,
                    retreat_box=(-.04,0,.04),brace_label='left minor',
                    right_min_degrees=args.right_hold_min_degrees)
                if args.open_claw_transfer:
                    from carton.folding_retention import transfer_to_open_claw
                    result['stage'] = 'Transfer both short-flap holds to one open right claw'
                    result['open_claw_transfer'] = transfer_to_open_claw(sim, controller,
                        capture=True, support_height=args.support_height)
                    if args.near_after_open_claw:
                        from carton.folding_cascade import press_near_over_short
                        result['stage'] = 'Transfer open-claw support to near major'
                        result['near_major_transfer'] = press_near_over_short(sim, controller,
                            capture=True, from_open_claw=True,
                            along=args.near_press_along, pre_out=args.near_pre_out,
                            pre_up=args.near_pre_up, release_right_at_degrees=args.near_release_angle)
                if args.release_left_minor:
                    from carton.folding_transfers import release_left_minor
                    result['stage']='Release left short to observe spring-back'
                    result['release_test']=release_left_minor(sim,controller,seconds=5.)
                if args.press_near_after_minors:
                    from carton.folding_cascade import press_near_over_short
                    result['stage']='Transfer left short hold to near major'
                    result['near_major_transfer']=press_near_over_short(sim,controller,
                        capture=True,from_minor=True,along=args.near_press_along,
                        pre_out=args.near_pre_out,pre_up=args.near_pre_up,
                        release_right=args.release_right_before_near,
                        right_park_joints=initial_right_joints if args.tool=='paddle' else None)
            if args.close_left:
                from scipy.spatial.transform import Rotation
                result['stage']='Close pinched left minor while right holds'
                reading=controller.sense('Register current left hinge before pinched fold')
                observed=reading['angles'].get('short_left')
                if observed is None:raise ValueError('Fresh left-minor observation required')
                start=math.radians(observed['degrees'])
                hinge=controller.box[:3,:3]@np.array([-.379/2,0,.108])+controller.box[:3,3]
                axis=controller.box[:3,:3]@np.array([0,1,0])
                tip=sim.data.site('left_tip').xpos.copy()
                normal=sim.data.body('left_gripper_link').xmat.reshape(3,3)[:,0].copy()
                for i,theta in enumerate(np.linspace(start,math.radians(args.left_target_degrees),51)):
                    rotate=Rotation.from_rotvec(axis*(theta-start)).as_matrix()
                    port.move_arms({'left':hinge+rotate@(tip-hinge)},.3,
                        f'Close pinched left minor {math.degrees(theta):.1f}',
                        {'direction':(rotate@normal).tolist(),'local_axis':[1,0,0]})
                    grasp()
                    if i%5==0:controller.sense('Observe pinched left fold and retained right minor')
                port.move_arms({},2.,'Hold both minors after pinched fold',None)
                reading=controller.sense('Verify both minor folds')
                result['left_closed_held']={'angles':sim.truth_angles(),'visual_angles':reading['angles'],
                    'motion':dict(sim.motion_stats),'full_task_complete':False}
    except Exception as exc:
        result['error'] = str(exc)
    sim.capture('End of left-minor brace diagnostic')
    result.update(angles=sim.truth_angles(), motion=dict(sim.motion_stats), time=float(sim.data.time),
                  readings=port.readings, perception_quality=port.observer.history,
                  physics=sim.save('folding'), material=sim.material.report(),
                  contact_progress_checks=getattr(controller, 'contact_progress_checks', []),
                  open_claw_transfer=getattr(controller, 'open_claw_transfer', None),
                  short_opening=getattr(controller, 'short_opening', None),
                  far_edge_attempt=getattr(controller, 'far_edge_attempt', None),
                  partial_major_release=getattr(controller, 'partial_major_release', None),
                  partial_short_probe=getattr(controller, 'partial_short_probe', None),
                  additional_view_history=getattr(port, 'additional_view_history', None),
                  additional_view_declaration=getattr(port, 'declaration', None),
                  additional_view_assumptions_sha256=getattr(port, 'assumptions_sha256', None),
                  source_sha256={p: hashlib.sha256(b).hexdigest() for p, b in snapshots.items()})
    result['controller'] = {'error': result.get('error', 'Partial sequence; full closure untested')}
    (out/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: result.get(k) for k in ('stage', 'error', 'angles', 'motion', 'time')}, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-root', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--tool', choices=['claws', 'paddle'], default='paddle')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--fold-right', action='store_true')
    parser.add_argument('--open-shorts-first', action='store_true')
    parser.add_argument('--far-after-near', action='store_true')
    parser.add_argument('--additional-far-view', action='store_true',
                        help='Explicit hypothetical second calibrated rendered camera during far stage')
    parser.add_argument('--far-hold-degrees', type=float, default=90.)
    parser.add_argument('--far-contact-profile', choices=['edge','central'], default='edge')
    parser.add_argument('--far-normal-extra', type=float, default=0.)
    parser.add_argument('--far-startup-lift', type=float, default=0.)
    parser.add_argument('--far-gripper-opening', type=float, default=-.17)
    parser.add_argument('--release-far-after', action='store_true')
    parser.add_argument('--release-near-after-far', action='store_true')
    parser.add_argument('--probe-shorts-after-release', action='store_true',
                        help='Bounded paired short probe with explicit additional front-camera assumption')
    parser.add_argument('--open-short-angle', type=float, default=-15.)
    parser.add_argument('--along', type=float, default=-.10)
    parser.add_argument('--radius', type=float, default=.125)
    parser.add_argument('--clearance', type=float, default=-.002)
    parser.add_argument('--axis-sign', type=int, choices=[-1, 1], default=1)
    parser.add_argument('--normal-only', action='store_true')
    parser.add_argument('--paddle-contact', choices=['handle','tip'], default='handle')
    parser.add_argument('--paddle-axis-yaw', type=float, default=0.)
    parser.add_argument('--paddle-axis-mode', choices=['rise','flat','descend'], default='rise')
    parser.add_argument('--paddle-along-travel', type=float, default=.085)
    parser.add_argument('--paddle-blend-yaw', action='store_true')
    parser.add_argument('--paddle-end-degrees', type=float, default=75.)
    parser.add_argument('--right-hold-min-degrees', type=float, default=85.)
    parser.add_argument('--near-open-degrees', type=float, default=math.degrees(-.1))
    parser.add_argument('--prepare-near-degrees', type=float)
    parser.add_argument('--open-claw-transfer', action='store_true')
    parser.add_argument('--support-height', type=float, default=.1094)
    parser.add_argument('--near-after-open-claw', action='store_true')
    parser.add_argument('--near-release-angle', type=float, default=15.)
    parser.add_argument('--privileged-near-angle', action='store_true',
                        help='Mechanics diagnostic only: replace near angle with simulator truth after the short-flap prefix')
    parser.add_argument('--right-pre-out', type=float, default=.045)
    parser.add_argument('--right-pre-up', type=float, default=0.)
    parser.add_argument('--normal-offset', action='store_true')
    parser.add_argument('--pre-height', type=float, default=.035)
    parser.add_argument('--carton-yaw-degrees', type=float, default=30.)
    parser.add_argument('--carton-offset-x', type=float, default=0.,
                        help='Explicit proposed lateral carton placement in metres')
    parser.add_argument('--park-back', action='store_true')
    parser.add_argument('--floor-marker-x', type=float)
    parser.add_argument('--center-floor-marker', action='store_true')
    parser.add_argument('--close-left', action='store_true')
    parser.add_argument('--left-target-degrees', type=float, default=75.)
    parser.add_argument('--press-left', action='store_true')
    parser.add_argument('--scan-tool', action='store_true')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=540)
    parser.add_argument('--release-left-minor', action='store_true')
    parser.add_argument('--press-near-after-minors', action='store_true')
    parser.add_argument('--near-press-along', type=float, default=-.13)
    parser.add_argument('--near-pre-out', type=float, default=.055)
    parser.add_argument('--near-pre-up', type=float, default=.040)
    parser.add_argument('--near-hold-degrees', type=float, default=90.)
    parser.add_argument('--release-right-before-near', action='store_true')
    args=parser.parse_args()
    if not (320<=args.width<=1280 and 240<=args.height<=960):
        parser.error('Image dimensions must fit the 1280 by 960 renderer')
    if args.press_left and args.close_left:parser.error('Choose press-left or close-left')
    if (args.press_left or args.close_left or args.scan_tool) and not args.fold_right:
        parser.error('Transfer and view-scan options require --fold-right')
    if args.release_left_minor and not args.press_left:
        parser.error('--release-left-minor requires --press-left')
    if args.press_near_after_minors and (not args.press_left or args.release_left_minor):
        parser.error('Near-major transfer requires --press-left without a prior release test')
    if args.release_right_before_near and not args.press_near_after_minors:
        parser.error('Right-hand withdrawal requires a near-major transfer')
    if args.scan_tool and args.tool!='paddle':parser.error('--scan-tool requires a paddle')
    if args.open_claw_transfer and (args.tool != 'claws' or not args.press_left
            or args.release_left_minor or args.press_near_after_minors):
        parser.error('Open-claw transfer requires claws and --press-left, without another transfer')
    if args.near_after_open_claw and not args.open_claw_transfer:
        parser.error('--near-after-open-claw requires --open-claw-transfer')
    if args.additional_far_view and (not args.far_after_near or args.privileged_near_angle):
        parser.error('Additional view requires the ordinary pixel-based far-after-near experiment')
    if args.release_near_after_far and (not args.far_after_near or not args.release_far_after
            or args.near_hold_degrees != 40. or args.far_hold_degrees != 35.):
        parser.error('Near release requires the verified near40/far35 passive far-release profile')
    if args.probe_shorts_after_release and not args.release_near_after_far:
        parser.error('Paired short probe requires the both-hands-parked partial release')
    if args.center_floor_marker and args.floor_marker_x is not None and abs(args.floor_marker_x)<.057:
        parser.error('Declared floor markers would overlap')
    run(args)
