"""Offline bare-claw retention experiments using physical robot contacts.

No carton joint is commanded or reset. Geometric preparation and support
transfers are partial stages, never four-flap completion evidence.
"""
import math
import copy

import numpy as np
import mujoco
from scipy.optimize import least_squares

from carton.folding_diagonal import contact_point
from carton.folding_grasp import panel_grasp_evidence
from carton.folding_paths import JointPathPlanner, execute_path
from carton.folding_sim import JOINTS


def open_near_for_transfer(sim, controller, degrees=-15., *, capture=False, release_lift=.10):
    """Push the near flap outward while the left claw braces a short flap."""
    if not math.isfinite(degrees) or not -35 <= degrees <= -10:
        raise ValueError('Near-flap opening must be between -35 and -10 degrees')
    if not math.isfinite(release_lift) or not .04 <= release_lift <= .10:
        raise ValueError('Near-flap release lift must be finite and between 40 and 100 mm')
    c = controller
    port = c.port
    reading = c.sense('Locate near flap before physical clearance preparation')
    measured = reading['angles'].get('long_near')
    if measured is None:
        raise ValueError('Fresh near-flap observation required before opening')
    start = math.radians(measured['degrees'])
    checks = []

    def verify_brace():
        check = panel_grasp_evidence(sim.model, sim.data, 'left', 'short_left')
        checks.append(check)
        if not check['opposing_faces']:
            raise ValueError('Opposing-face left-minor brace was not retained')

    verify_brace()
    port.set_grippers({'right': -.17}, .4, 'Close right claw clear of near flap')
    reached = False
    visual_checks = []
    vision_interrupted = False

    def observed_target():
        nonlocal vision_interrupted
        reading = c.sense('Measure near flap to stop outward push at observed target')
        row = reading['angles'].get('long_near')
        if row is None:
            # Stop pushing immediately. Withdraw using robot encoders, then
            # require a new visible angle before this preparation can pass.
            vision_interrupted = True
            visual_checks.append(dict(seq=reading['seq'], degrees=None,
                                      action='Stop contact and withdraw to observe'))
            return True
        visual_checks.append(dict(seq=reading['seq'], degrees=row['degrees']))
        return row['degrees'] <= degrees + .5

    for index, theta in enumerate(np.linspace(start, math.radians(degrees), 25)):
        point, _ = contact_point(theta, 1, -1, 0., .115, 0, -.012)
        if index == 0:
            pre = point + [0, .035, .035]
            q, error = sim.ik('right', c.box[:3, :3] @ pre + c.box[:3, 3], 'down')
            if error > .008:
                raise ValueError(f'Inside near-flap approach IK misses {error * 1000:.2f} mm')
            path = JointPathPlanner(sim, 'right', clearance=.006,
                                   allowed_flaps=('long_near_cardboard',)).plan(q)
            execute_path(sim, 'right', path, 'Reach above inside near-flap face', capture=capture)
            for u in np.linspace(.1, 1, 10):
                c.move({'right': (1-u)*pre + u*point}, .15,
                       'Approach inside near flap while left braces', 'down')
                verify_brace()
                if observed_target():
                    reached = True
                    break
        if reached:
            break
        c.move({'right': point}, .2,
               f'Physically push near flap outward {math.degrees(theta):.1f}', 'down')
        verify_brace()
        if observed_target():
            break
    actual = sim.actual_control_position('right').copy()
    port.move_arms({'right': actual + [0, 0, release_lift]}, .8,
                   'Lift right claw clear of opened near flap', 'down')
    q, error = sim.ik('right', np.array([.20, -.18, .30]), None)
    if error > .008:
        raise ValueError('Right return-to-park IK exceeds 8 mm')
    path = JointPathPlanner(sim, 'right', clearance=.006,
                           allowed_flaps=('long_near_cardboard',)).plan(q)
    execute_path(sim, 'right', path, 'Park right hand after physical near opening', capture=capture)
    port.move_arms({}, 1., 'Observe released near flap while left braces carton', None)
    verify_brace()
    reading = c.sense('Verify opened near flap without a hand holding it')
    observed = reading['angles'].get('long_near')
    if observed is None or abs(observed['degrees']-degrees) > 5:
        raise ValueError('Opened near-flap clearance was not retained after release')
    return dict(requested_degrees=degrees, release_lift_m=release_lift, visual_angle=observed, brace_checks=checks,
                visual_checks=visual_checks,
                vision_interrupted=vision_interrupted,
                physical_opening_executed=True, full_task_complete=False)


def support_vertices(model, data):
    """Select two actual CAD tip vertices in their link-local frames.

    Opening the jaw here affects a separate kinematic copy only. It is a
    geometric contact proposal; loaded support must be measured in dynamics.
    """
    kin = mujoco.MjData(model)
    kin.qpos[:] = data.qpos
    joint = model.joint('right_gripper')
    kin.qpos[joint.qposadr[0]] = joint.range[1]
    mujoco.mj_kinematics(model, kin)
    points = {}
    for kind, body in (('wrist_roll_follower', 'right_gripper_link'),
                       ('moving_jaw', 'right_moving_jaw_so101_v1_link')):
        vertices = []
        for i in range(model.ngeom):
            if (not model.geom(i).name.startswith('right_')
                    or kind not in model.geom(i).name or not model.geom_contype[i]
                    or model.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH):
                continue
            mesh = model.geom_dataid[i]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            local = model.mesh_vert[start:start+count]
            vertices.append(np.einsum('ij,kj->ki', kin.geom_xmat[i].reshape(3, 3), local)
                            + kin.geom_xpos[i])
        if not vertices:
            raise ValueError('Original jaw collision meshes required for support planning')
        world = np.vstack(vertices)
        grip = kin.body('right_gripper_link')
        local = np.einsum('ji,kj->ki', grip.xmat.reshape(3, 3), world-grip.xpos)
        index = np.argmin(local[:, 2]) if kind == 'wrist_roll_follower' else np.argmax(local[:, 0])
        link = kin.body(body)
        points[body] = link.xmat.reshape(3, 3).T @ (world[index]-link.xpos)
    return points


def right_support_evidence(model, data):
    """Independent loaded-contact evidence on both short panels."""
    # mj_step leaves contact/force arrays from before the last integration.
    # Recompute them at the current state in a copy, without advancing time,
    # moving the carton, changing controls or changing solver warm starts.
    data=copy.copy(data)
    mujoco.mj_forward(model,data)
    forces = dict(short_left=0., short_right=0.)
    contacts = []
    for i, contact in enumerate(data.contact):
        a, b = model.geom(contact.geom1).name, model.geom(contact.geom2).name
        for flap in forces:
            panel = flap + '_cardboard'
            if panel not in (a, b):
                continue
            other = b if a == panel else a
            if not other.startswith('right_') or not any(
                    name in other for name in ('moving_jaw', 'wrist_roll_follower')):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            normal = data.body(flap).xmat.reshape(3, 3)[:, 0]
            broad_face = abs(float(normal @ contact.frame[:3])) >= .8
            outward = normal * (-1 if flap == 'short_left' else 1)
            outer_face = float(outward @ (contact.pos-data.geom(panel).xpos)) > 0
            if broad_face and outer_face:
                forces[flap] += float(force[0])
            contacts.append(dict(flap=flap, geom=other, normal_force_N=float(force[0]),
                                 broad_face=broad_face, outer_face=outer_face,
                                 position=contact.pos.tolist()))
    return dict(time=float(data.time), forces_N=forces, contacts=contacts,
                both_loaded=all(value > .02 for value in forces.values()))


def transfer_to_open_claw(sim, controller, *, capture=False, observe_seconds=5.,
                          support_height=.1094, fallback_support_heights=(), support_half_spans=(.052,),
                          support_samples=1):
    """Attempt a two-finger bridge, then withdraw the left supporting hand.

    Joint targets are solved from fresh visual carton registration and original
    CAD vertices. All collision planning happens in a copy. The real simulated
    carton remains free throughout the execution and release observation.
    """
    if not math.isfinite(observe_seconds) or observe_seconds < 5:
        raise ValueError('At least five seconds of support observation required')
    heights = (support_height, *fallback_support_heights)
    if any(not math.isfinite(h) or not .1085 <= h <= .120 for h in heights):
        raise ValueError('Support height must be a declared box-local height of 108.5--120 mm')
    if not support_half_spans or any(not math.isfinite(w) or not .052 <= w <= .065 for w in support_half_spans):
        raise ValueError('Support half-span must be 52--65 mm (short edges at 49.5 mm)')
    c = controller
    reading = c.sense('Register both short flaps before open-claw support transfer')
    c.require_folded(reading, ['short_left', 'short_right'])
    vertices = support_vertices(sim.model, sim.data)
    ix = np.asarray(sim.arm_indices['right'])
    limits = sim.model.jnt_range[[sim.model.joint('right_'+j).id for j in JOINTS]]
    report = dict(full_task_complete=False, support_checks=[], planning=[],
                  support_height_m=support_height,
                  contact_vertices={body: point.tolist() for body, point in vertices.items()})
    c.open_claw_transfer = report

    def verify_physics(event):
        if event.get('step_error') or event['bad_penetration_mm'] > 1:
            raise ValueError(event.get('step_error') or 'Collision during open-claw transfer')
        if event.get('max_joint_tracking_error_radians', 0) > .08:
            raise ValueError('Open-claw transfer joint tracking exceeds 0.08 rad')
        # Independent diagnostic only, never used to generate a target.
        if any(not 80 <= event['flap_degrees'][f] <= 98 for f in ('short_left', 'short_right')):
            raise ValueError('Short-flap support lost during open-claw transfer')

    # Turn the closed claw before opening it. Without this phase, the two
    # fingertip IK path crosses an ambiguous near-closed-jaw orientation.
    q = sim.data.qpos[ix[:5]].copy()
    q[4] = -2.15
    path = JointPathPlanner(sim, 'right', clearance=.006,
                           allowed_flaps=('short_right_cardboard',)).plan(q)
    execute_path(sim, 'right', path, 'Turn closed right claw while retaining right short', capture=capture)
    verify_physics(sim.events[-1])
    reading = c.sense('Re-register short flaps after turning the closed claw')
    c.require_folded(reading, ['short_left', 'short_right'])
    kin = mujoco.MjData(sim.model)
    kin.qpos[:] = sim.data.qpos

    def endpoints(q):
        kin.qpos[ix] = q
        mujoco.mj_kinematics(sim.model, kin)
        return np.array([kin.body(body).xpos + kin.body(body).xmat.reshape(3, 3) @ point
                         for body, point in vertices.items()])

    start_q = sim.data.qpos[ix].copy()
    start_points = endpoints(start_q)
    planner = JointPathPlanner(sim, 'right', clearance=.006,
                               allowed_flaps=('short_left_cardboard', 'short_right_cardboard'))

    def valid(q):
        planner.data.qpos[ix[5]] = q[5]
        return planner.valid(q[:5])

    def plan_sweep(height, half_span):
        # This narrow candidate overlaps each inner edge by only 2.5 mm. It is
        # intentionally a diagnostic: loaded contacts and the release test must
        # establish whether that small margin actually retains the panels.
        local_end = np.array([[-half_span, -.04, height], [half_span, -.04, height]])
        end_points = local_end @ c.box[:3, :3].T + c.box[:3, 3]
        previous = start_q.copy()
        planned = []
        for u in np.linspace(0, 1, 81)[1:]:
            target = (1-u)*start_points + u*end_points
            target[0] += .010*math.sin(math.pi*u)*c.box[:3, 2]
            solution = least_squares(lambda q: (endpoints(q)-target).ravel(),
                np.clip(previous, limits[:, 0]+1e-8, limits[:, 1]-1e-8),
                bounds=(limits[:, 0], limits[:, 1]), max_nfev=180,
                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            q = solution.x
            error = float(np.max(np.linalg.norm(endpoints(q)-target, axis=1)))
            if error > .002:
                raise ValueError(f'Open-claw support-point IK exceeds 2 mm: {error*1000:.2f}')
            steps = max(1, math.ceil(float(np.max(np.abs(q-previous))) / .025))
            clear = all(valid(previous+(q-previous)*v) for v in np.linspace(0, 1, steps+1))
            report['planning'].append(dict(support_height_m=height, half_span_m=half_span, progress=float(u), error_mm=error*1000,
                                           clear=clear, collision=planner.last_collision if not clear else None))
            if not clear:
                raise ValueError(f'Open-claw transfer sweep collides: {planner.last_collision}')
            planned.append(q.copy())
            previous = q.copy()
        return planned

    # The whole sweep is planned before any motion, so explicitly declared
    # alternative spans and heights can be planned when one plan is refused.
    # Wider spans overlap the shorts' free edges by more than the original
    # 2.5 mm, which a few millimetres of carton motion could lose.
    options = [(w, h) for w in support_half_spans for h in heights]
    for index, (half_span, height) in enumerate(options):
        try:
            planned = plan_sweep(height, half_span)
            break
        except ValueError as exc:
            report.setdefault('refused_support_heights', []).append(dict(support_height_m=height,
                                                                         half_span_m=half_span, reason=str(exc)))
            if index == len(options) - 1:
                raise
    report.update(support_height_m=height, support_half_span_m=half_span)
    report['planned_waypoints'] = len(planned)
    for index, q in enumerate(planned):
        duration = max(.18, float(np.max(np.abs(q-sim.data.qpos[ix]))) / .55)
        event = sim.move({}, duration, f'Open right claw across short-flap gap {index+1}/{len(planned)}',
                         capture=capture, joint_targets={'right': q[:5]}, grippers={'right': q[5]})
        verify_physics(event)
        report['support_checks'].append(right_support_evidence(sim.model, sim.data))
        if index % 10 == 0:
            c.sense('Observe carton during open-claw support transfer')
    event = sim.move({}, 1., 'Settle right-claw support on both short flaps', capture=capture)
    verify_physics(event)
    support = right_support_evidence(sim.model, sim.data)
    report['support_checks'].append(support)
    report['progressive_handoff'] = []
    # A millimetre of pose error can leave the new support just above the
    # left-held panel. Unload that hand in 2 mm physical steps, allowing the
    # resisting flap to rise into the waiting right finger. The original
    # loaded-contact check must pass before the full hand withdrawal.
    for step in range(4):
        if support['both_loaded']:
            break
        actual = sim.data.site('left_tip').xpos.copy()
        direction = sim.data.body('left_gripper_link').xmat.reshape(3,3)[:,2].copy()
        port = c.port
        port.move_arms({'left':actual+[0,0,.002]},.25,
                       f'Gradually unload left support {step+1}/4',
                       {'direction':direction.tolist()})
        event=sim.events[-1]
        verify_physics(event)
        if any(not 85 <= low <= high <= 95
               for f in ('short_left','short_right')
               for low,high in [event['flap_angle_extrema_degrees'][f]]):
            raise ValueError('Short flap left closure band during progressive handoff')
        reading=c.sense('Verify both shorts while unloading left support')
        c.require_folded(reading,['short_left','short_right'])
        support=right_support_evidence(sim.model,sim.data)
        report['support_checks'].append(support)
        report['progressive_handoff'].append(dict(time=float(sim.data.time),
            visual_angles=reading['angles'],support=support))
    if not support['both_loaded']:
        raise ValueError('Right claw has not established loaded support on both short flaps')
    from carton.folding_transfers import release_left_minor
    release_start = len(sim.events)
    report['left_release'] = release_left_minor(sim, c, seconds=observe_seconds)
    window = sim.events[release_start:]
    report['release_window'] = [dict(label=e['label'], duration_s=e['duration_s'],
        short_angle_extrema={f:e['flap_angle_extrema_degrees'][f]
                            for f in ('short_left', 'short_right')},
        contact_pairs=e['contact_pairs']) for e in window]
    retained = (window and window[-1]['duration_s'] >= observe_seconds
        and all(85 <= low <= high <= 95 for e in window
                for f in ('short_left', 'short_right')
                for low, high in [e['flap_angle_extrema_degrees'][f]])
        and not any(any(name.startswith('left_') for name in pair)
                    for pair in window[-1]['contact_pairs']))
    if not retained:
        raise ValueError('Both shorts were not retained throughout the left-hand release window')
    reading = c.sense('Verify both shorts after left-hand withdrawal')
    c.require_folded(reading, ['short_left', 'short_right'])
    support = right_support_evidence(sim.model, sim.data)
    report['support_checks'].append(support)
    if not support['both_loaded'] and support_samples > 1:
        # A resting panel's contact chatters at a few hundredths of a millimetre,
        # so one instant can show no load. Sample more instants while the
        # physics runs; each short must be loaded by the right claw in one.
        loaded = {flap for flap, force in support['forces_N'].items() if force > .02}
        for _ in range(support_samples - 1):
            event = sim.move({}, .1, 'Sample right-claw support after left-hand withdrawal', capture=capture)
            verify_physics(event)
            sample = right_support_evidence(sim.model, sim.data)
            report['support_checks'].append(sample)
            loaded |= {flap for flap, force in sample['forces_N'].items() if force > .02}
        support = dict(support, both_loaded=loaded == {'short_left', 'short_right'},
                       sampled_instants=support_samples, loaded_in_any_sample=sorted(loaded))
    if not support['both_loaded']:
        raise ValueError('Right-claw support did not survive left-hand withdrawal')
    report['both_shorts_retained_by_right_claw'] = True
    return report
